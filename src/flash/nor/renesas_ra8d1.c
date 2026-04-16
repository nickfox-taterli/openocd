// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Renesas RA8D1 internal flash driver (Code flash + Data flash)         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

#include <helper/align.h>
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#include "renesas_ra8d1.h"

#define RA8D1_TIMEOUT_PROG_MS        100
#define RA8D1_TIMEOUT_ERASE_CODE_MS  4000
#define RA8D1_TIMEOUT_ERASE_DATA_MS  500

struct ra8d1_flash_bank {
	bool probed;
	bool is_data_flash;
	uint32_t program_unit;
	bool cache_saved;
	uint16_t saved_fcachee;
};

static bool ra8d1_fallback_write_logged;

static int ra8d1_wait_frdy(struct target *target, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint32_t fstatr;
		int retval = target_read_u32(target, RA8D1_REG_FSTATR, &fstatr);
		if (retval != ERROR_OK)
			return retval;

		if (fstatr & RA8D1_FSTATR_FRDY)
			return ERROR_OK;
	}

	return ERROR_TIMEOUT_REACHED;
}

static int ra8d1_wait_dbfull_clear(struct target *target)
{
	uint32_t timeout = 2000;

	while (timeout--) {
		uint32_t fstatr;
		int retval = target_read_u32(target, RA8D1_REG_FSTATR, &fstatr);
		if (retval != ERROR_OK)
			return retval;

		if ((fstatr & RA8D1_FSTATR_DBFULL) == 0)
			return ERROR_OK;
	}

	return ERROR_TIMEOUT_REACHED;
}

static int ra8d1_status_clear(struct target *target)
{
	return target_write_u8(target, RA8D1_FACI_CMD_AREA, RA8D1_FACI_CMD_STATUS_CLEAR);
}

static int ra8d1_check_errors(struct target *target, bool log_error)
{
	uint32_t fstatr;
	uint32_t feaddr;
	uint16_t fcmdr;
	uint8_t fastat;
	int retval = target_read_u32(target, RA8D1_REG_FSTATR, &fstatr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, RA8D1_REG_FEADDR, &feaddr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u16(target, RA8D1_REG_FCMDR, &fcmdr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(target, RA8D1_REG_FASTAT, &fastat);
	if (retval != ERROR_OK)
		return retval;

	if (((fstatr & RA8D1_FSTATR_ERR_MASK) == 0) && ((fastat & RA8D1_FASTAT_CMDLK) == 0))
		return ERROR_OK;

	if (log_error) {
		LOG_ERROR("RA8D1 flash status error: FSTATR=0x%08" PRIx32 ", FASTAT=0x%02" PRIx8
				", FCMDR=0x%04" PRIx16 ", FEADDR=0x%08" PRIx32,
				fstatr, fastat, fcmdr, feaddr);
	}

	retval = ra8d1_status_clear(target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_wait_frdy(target, RA8D1_TIMEOUT_PROG_MS);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_FLASH_OPERATION_FAILED;
}

static int ra8d1_enable_pe(struct flash_bank *bank)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (!info->is_data_flash && !info->cache_saved) {
		retval = target_read_u16(target, RA8D1_REG_FCACHEE, &info->saved_fcachee);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u16(target, RA8D1_REG_FCACHEE, 0);
		if (retval != ERROR_OK)
			return retval;
		info->cache_saved = true;
	}

	if (!info->is_data_flash) {
		retval = target_write_u16(target, RA8D1_REG_FMEPROT, RA8D1_FMEPROT_UNLOCK);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u8(target, RA8D1_REG_FWEPROR, RA8D1_FWEPROR_ENABLE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u16(target, RA8D1_REG_FENTRYR,
			info->is_data_flash ? RA8D1_FENTRYR_DATA_PE : RA8D1_FENTRYR_CODE_PE);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_wait_frdy(target, RA8D1_TIMEOUT_PROG_MS);
	if (retval != ERROR_OK)
		return retval;

	return ra8d1_check_errors(target, true);
}

static int ra8d1_disable_pe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct ra8d1_flash_bank *info = bank->driver_priv;
	int retval;

	retval = target_write_u16(target, RA8D1_REG_FENTRYR, RA8D1_FENTRYR_READ);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA8D1_REG_FWEPROR, RA8D1_FWEPROR_DISABLE);
	if (retval != ERROR_OK)
		return retval;

	if (!info->is_data_flash) {
		retval = target_write_u16(target, RA8D1_REG_FMEPROT, RA8D1_FMEPROT_LOCK);
		if (retval != ERROR_OK)
			return retval;
	}

	if (!info->is_data_flash && info->cache_saved) {
		retval = target_write_u16(target, RA8D1_REG_FCACHEE, info->saved_fcachee);
		if (retval != ERROR_OK)
			return retval;
		info->cache_saved = false;
	}

	return ERROR_OK;
}

static int ra8d1_program_one_unit(struct flash_bank *bank, uint32_t addr, const uint8_t *buffer)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	retval = target_write_u32(target, RA8D1_REG_FSADDR, addr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA8D1_FACI_CMD_AREA, RA8D1_FACI_CMD_PROGRAM);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA8D1_FACI_CMD_AREA, (uint8_t)(info->program_unit / 2));
	if (retval != ERROR_OK)
		return retval;

	for (uint32_t i = 0; i < info->program_unit; i += 2) {
		uint16_t v = (uint16_t)buffer[i] | ((uint16_t)buffer[i + 1] << 8);
		retval = target_write_u16(target, RA8D1_FACI_CMD_AREA, v);
		if (retval != ERROR_OK)
			return retval;

		retval = ra8d1_wait_dbfull_clear(target);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u8(target, RA8D1_FACI_CMD_AREA, RA8D1_FACI_CMD_EXECUTE);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_wait_frdy(target, RA8D1_TIMEOUT_PROG_MS);
	if (retval != ERROR_OK)
		return retval;

	return ra8d1_check_errors(target, true);
}

static int ra8d1_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval;
	uint32_t target_address = bank->base + offset;

	static const uint8_t ra8d1_flash_write_code[] = {
#include "../../../contrib/loaders/flash/renesas/ra8d1.inc"
	};

	if (target_alloc_working_area(target, sizeof(ra8d1_flash_write_code),
			&write_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(ra8d1_flash_write_code), ra8d1_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	const size_t extra_size = sizeof(struct ra8d1_loader_work_area);
	uint32_t buffer_size = target_get_working_area_avail(target) - extra_size;
	buffer_size &= ~(info->program_unit - 1);

	if (buffer_size < 256) {
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (buffer_size > 16384) {
		buffer_size = 16384;
	}

	if (target_alloc_working_area_try(target, buffer_size + extra_size, &source) != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	uint32_t program_unit_buf;
	target_buffer_set_u32(target, (uint8_t *)&program_unit_buf, info->program_unit);
	retval = target_write_buffer(target, source->address +
			offsetof(struct ra8d1_loader_work_area, program_unit),
			sizeof(program_unit_buf), (uint8_t *)&program_unit_buf);
	if (retval != ERROR_OK)
		goto cleanup;

	memset(&armv7m_info, 0, sizeof(armv7m_info));
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, target_address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32,
			source->address + offsetof(struct ra8d1_loader_work_area, stack) + RA8D1_LOADER_STACK_SIZE);

	retval = target_run_flash_async_algorithm(target, buffer, count, info->program_unit,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address + offsetof(struct ra8d1_loader_work_area, fifo),
			source->size - offsetof(struct ra8d1_loader_work_area, fifo),
			write_algorithm->address, 0, &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		uint32_t fstatr = 0;
		uint8_t fastat = 0;
		(void)target_read_u32(target, RA8D1_REG_FSTATR, &fstatr);
		(void)target_read_u8(target, RA8D1_REG_FASTAT, &fastat);
		LOG_ERROR("error executing RA8D1 flash write algorithm");
		LOG_ERROR("RA8D1 async write status: FSTATR=0x%08" PRIx32 ", FASTAT=0x%02" PRIx8,
				fstatr, fastat);
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

cleanup:
	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);
	return retval;
}

static int ra8d1_write_block_sync(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval;
	uint32_t target_address = bank->base + offset;
	uint32_t fifo_ctrl_address;
	uint32_t fifo_data_address;
	uint32_t fifo_data_size;
	uint32_t stack_pointer;

	static const uint8_t ra8d1_flash_write_code[] = {
#include "../../../contrib/loaders/flash/renesas/ra8d1.inc"
	};

	if (target_alloc_working_area(target, sizeof(ra8d1_flash_write_code),
			&write_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(ra8d1_flash_write_code), ra8d1_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	const size_t extra_size = sizeof(struct ra8d1_loader_work_area);
	uint32_t buffer_size = target_get_working_area_avail(target) - extra_size;
	buffer_size &= ~(info->program_unit - 1);

	if (buffer_size < 256) {
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (buffer_size > 16384) {
		buffer_size = 16384;
	}

	if (target_alloc_working_area_try(target, buffer_size + extra_size, &source) != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	uint32_t program_unit_buf;
	target_buffer_set_u32(target, (uint8_t *)&program_unit_buf, info->program_unit);
	retval = target_write_buffer(target, source->address +
			offsetof(struct ra8d1_loader_work_area, program_unit),
			sizeof(program_unit_buf), (uint8_t *)&program_unit_buf);
	if (retval != ERROR_OK)
		goto cleanup;

	fifo_ctrl_address = source->address + offsetof(struct ra8d1_loader_work_area, fifo);
	fifo_data_address = fifo_ctrl_address + 8;
	fifo_data_size = source->size - offsetof(struct ra8d1_loader_work_area, fifo) - 8;
	fifo_data_size &= ~(info->program_unit - 1);
	stack_pointer = source->address + offsetof(struct ra8d1_loader_work_area, stack) + RA8D1_LOADER_STACK_SIZE;

	if (fifo_data_size < info->program_unit) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	memset(&armv7m_info, 0, sizeof(armv7m_info));
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	while (count > 0) {
		uint32_t thisrun_count = fifo_data_size / info->program_unit;
		if (thisrun_count > count)
			thisrun_count = count;

		uint32_t thisrun_bytes = thisrun_count * info->program_unit;
		uint32_t fifo_end = fifo_data_address + thisrun_bytes;
		uint32_t wp = fifo_end;
		uint32_t rp = fifo_data_address;

		retval = target_write_buffer(target, fifo_data_address, thisrun_bytes, buffer);
		if (retval != ERROR_OK)
			break;

		retval = target_write_u32(target, fifo_ctrl_address, wp);
		if (retval != ERROR_OK)
			break;
		retval = target_write_u32(target, fifo_ctrl_address + 4, rp);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, fifo_end);
		buf_set_u32(reg_params[2].value, 0, 32, target_address);
		buf_set_u32(reg_params[3].value, 0, 32, thisrun_count);
		buf_set_u32(reg_params[4].value, 0, 32, stack_pointer);

		retval = target_run_algorithm(target, 0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				write_algorithm->address, 0,
				(thisrun_count * RA8D1_TIMEOUT_PROG_MS) + 200, &armv7m_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("error executing RA8D1 synchronous flash write algorithm");
			break;
		}

		retval = ra8d1_check_errors(target, false);
		if (retval != ERROR_OK)
			break;

		target_address += thisrun_bytes;
		buffer += thisrun_bytes;
		count -= thisrun_count;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

cleanup:
	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);
	return retval;
}

static int ra8d1_write_block_without_loader(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	uint32_t address = bank->base + offset;

	while (count--) {
		int retval = ra8d1_program_one_unit(bank, address, buffer);
		if (retval != ERROR_OK)
			return retval;
		address += info->program_unit;
		buffer += info->program_unit;
	}

	return ERROR_OK;
}

static int ra8d1_erase_one_sector(struct flash_bank *bank, unsigned int sector)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t addr = bank->base + bank->sectors[sector].offset;
	int retval;

	retval = target_write_u32(target, RA8D1_REG_FSADDR, addr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA8D1_FACI_CMD_AREA,
			info->is_data_flash ? RA8D1_FACI_CMD_DATA_ERASE : RA8D1_FACI_CMD_CODE_ERASE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA8D1_FACI_CMD_AREA, RA8D1_FACI_CMD_EXECUTE);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_wait_frdy(target,
			info->is_data_flash ? RA8D1_TIMEOUT_ERASE_DATA_MS : RA8D1_TIMEOUT_ERASE_CODE_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_check_errors(target, true);
	if (retval == ERROR_OK)
		bank->sectors[sector].is_erased = 1;

	return retval;
}

static int ra8d1_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	return target_read_buffer(bank->target, bank->base + offset, count, buffer);
}

static int ra8d1_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (last >= bank->num_sectors || first > last)
		return ERROR_FLASH_SECTOR_INVALID;

	retval = ra8d1_enable_pe(bank);
	if (retval != ERROR_OK)
		goto done;

	for (unsigned int i = first; i <= last; i++) {
		retval = ra8d1_erase_one_sector(bank, i);
		if (retval != ERROR_OK)
			break;
	}

done:
	{
		int retval2 = ra8d1_disable_pe(bank);
		if (retval == ERROR_OK)
			retval = retval2;
	}

	return retval;
}

static int ra8d1_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint8_t *aligned_buffer = NULL;
	uint32_t aligned_count = count;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (count == 0)
		return ERROR_OK;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (!IS_ALIGNED(offset, info->program_unit))
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;

	if (!IS_ALIGNED(count, info->program_unit)) {
		aligned_count = ALIGN_UP(count, info->program_unit);
		if (offset + aligned_count > bank->size)
			return ERROR_FLASH_DST_OUT_OF_BANK;

		aligned_buffer = malloc(aligned_count);
		if (!aligned_buffer)
			return ERROR_FAIL;

		memset(aligned_buffer, 0xFF, aligned_count);
		memcpy(aligned_buffer, buffer, count);
		buffer = aligned_buffer;
	}

	retval = ra8d1_enable_pe(bank);
	if (retval != ERROR_OK)
		goto out;

	{
		uint32_t unit_count = aligned_count / info->program_unit;
		if (info->is_data_flash)
			retval = ra8d1_write_block_async(bank, buffer, offset, unit_count);
		else
			retval = ra8d1_write_block_sync(bank, buffer, offset, unit_count);
		if (retval == ERROR_FLASH_OPERATION_FAILED) {
			uint32_t fstatr = 0;
			uint32_t feaddr = 0;
			uint16_t fcmdr = 0;
			uint8_t fastat = 0;
			(void)target_read_u32(target, RA8D1_REG_FSTATR, &fstatr);
			(void)target_read_u8(target, RA8D1_REG_FASTAT, &fastat);
			(void)target_read_u16(target, RA8D1_REG_FCMDR, &fcmdr);
			(void)target_read_u32(target, RA8D1_REG_FEADDR, &feaddr);
			LOG_ERROR("RA8D1 loader write failed: FSTATR=0x%08" PRIx32 ", FASTAT=0x%02" PRIx8
					", FCMDR=0x%04" PRIx16 ", FEADDR=0x%08" PRIx32,
					fstatr, fastat, fcmdr, feaddr);

			int retval2 = ra8d1_disable_pe(bank);
			if (retval2 == ERROR_OK)
				retval2 = ra8d1_enable_pe(bank);
			if (retval2 != ERROR_OK)
				retval = retval2;
		}
		if ((retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) ||
				(retval == ERROR_FLASH_OPERATION_FAILED)) {
			if (!ra8d1_fallback_write_logged) {
				LOG_WARNING("RA8D1: fallback to slow DAP write path");
				ra8d1_fallback_write_logged = true;
			}
			retval = ra8d1_write_block_without_loader(bank, buffer, offset, unit_count);
		}
	}

	{
		int retval2 = ra8d1_disable_pe(bank);
		if (retval == ERROR_OK)
			retval = retval2;
	}

out:
	free(aligned_buffer);
	return retval;
}

static int ra8d1_build_code_flash_sectors(struct flash_bank *bank)
{
	uint32_t offset = 0;
	unsigned int num_small = 0;
	unsigned int num_large = 0;
	uint32_t small_area = bank->size < RA8D1_CODE_SMALL_AREA_SIZE ? bank->size : RA8D1_CODE_SMALL_AREA_SIZE;

	num_small = small_area / RA8D1_CODE_ERASE_SMALL;
	if (small_area % RA8D1_CODE_ERASE_SMALL)
		return ERROR_FAIL;

	if (bank->size > small_area) {
		uint32_t large_area = bank->size - small_area;
		num_large = large_area / RA8D1_CODE_ERASE_LARGE;
		if (large_area % RA8D1_CODE_ERASE_LARGE)
			return ERROR_FAIL;
	}

	bank->num_sectors = num_small + num_large;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < num_small; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = RA8D1_CODE_ERASE_SMALL;
		offset += RA8D1_CODE_ERASE_SMALL;
	}

	for (unsigned int i = 0; i < num_large; i++) {
		bank->sectors[num_small + i].offset = offset;
		bank->sectors[num_small + i].size = RA8D1_CODE_ERASE_LARGE;
		offset += RA8D1_CODE_ERASE_LARGE;
	}

	return ERROR_OK;
}

static int ra8d1_probe(struct flash_bank *bank)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;

	if (!info)
		return ERROR_FAIL;

	if (bank->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (info->probed)
		return ERROR_OK;

	if (info->is_data_flash) {
		bank->num_sectors = bank->size / RA8D1_DATA_ERASE_BLOCK_SIZE;
		bank->sectors = alloc_block_array(0, RA8D1_DATA_ERASE_BLOCK_SIZE, bank->num_sectors);
		if (!bank->sectors)
			return ERROR_FAIL;
	} else {
		int retval = ra8d1_build_code_flash_sectors(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	info->probed = true;
	return ERROR_OK;
}

static int ra8d1_auto_probe(struct flash_bank *bank)
{
	struct ra8d1_flash_bank *info = bank->driver_priv;
	if (!info)
		return ERROR_FAIL;
	if (info->probed)
		return ERROR_OK;
	return ra8d1_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(ra8d1_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct ra8d1_flash_bank *info = calloc(1, sizeof(*info));
	if (!info)
		return ERROR_FAIL;

	bank->driver_priv = info;
	bank->base = strtoull(CMD_ARGV[1], NULL, 0);
	bank->size = strtoul(CMD_ARGV[2], NULL, 0);
	bank->chip_width = strtoul(CMD_ARGV[3], NULL, 0);
	bank->bus_width = strtoul(CMD_ARGV[4], NULL, 0);
	bank->target = get_target(CMD_ARGV[5]);
	if (!bank->target) {
		free(info);
		bank->driver_priv = NULL;
		return ERROR_FAIL;
	}

	if (bank->base == RA8D1_DATA_BASE) {
		info->is_data_flash = true;
		info->program_unit = RA8D1_DATA_PROGRAM_UNIT;
		if (bank->size == 0)
			bank->size = RA8D1_DATA_SIZE;
	} else {
		info->is_data_flash = false;
		info->program_unit = RA8D1_CODE_PROGRAM_UNIT;
		if (bank->size == 0)
			bank->size = RA8D1_CODE_SIZE;
	}

	return ERROR_OK;
}

static const struct command_registration ra8d1_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ra8d1_flash = {
	.name = "ra8d1",
	.commands = ra8d1_exec_command_handlers,
	.flash_bank_command = ra8d1_flash_bank_command,
	.erase = ra8d1_erase,
	.protect = NULL,
	.write = ra8d1_write,
	.read = ra8d1_read,
	.probe = ra8d1_probe,
	.auto_probe = ra8d1_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
	.info = NULL,
};
