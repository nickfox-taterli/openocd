// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Renesas RA2L1 internal flash driver (Code flash + Data flash)         *
 *                                                                         *
 *   This implementation uses a small target-side loader for program       *
 *   operations and keeps erase / mode control in the host driver.         *
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

#include "renesas_ra2l1.h"

#define RA2L1_FLASH_TIMEOUT_MS 2000

#define RA2L1_SYSC_BASE        0x4001E000u
#define RA2L1_REG_PRCR         (RA2L1_SYSC_BASE + 0x03FEu)
#define RA2L1_PRCR_PRC0        BIT(0)
#define RA2L1_PRCR_PRC1        BIT(1)
#define RA2L1_PRCR_KEY_A5      (0xA5u << 8)

#define RA2L1_DFLCTL_DFLEN     BIT(0)

struct ra2l1_flash_bank {
	bool probed;
	bool is_data_flash;
	uint32_t erase_size;
	uint32_t program_unit;
	uint32_t pe_base;
};

static bool ra2l1_fallback_write_logged;

static int ra2l1_unlock_prcr(struct target *target)
{
	return target_write_u16(target, RA2L1_REG_PRCR,
			RA2L1_PRCR_KEY_A5 | RA2L1_PRCR_PRC0 | RA2L1_PRCR_PRC1);
}

static int ra2l1_lock_prcr(struct target *target)
{
	return target_write_u16(target, RA2L1_REG_PRCR, RA2L1_PRCR_KEY_A5);
}

static int ra2l1_prepare_access(struct target *target)
{
	int retval = ra2l1_unlock_prcr(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA2L1_REG_DFLCTL, RA2L1_DFLCTL_DFLEN);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA2L1_REG_PFBER, 0);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ra2l1_wait_frdy(struct target *target, bool set, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint32_t fstatr1;
		int retval = target_read_u32(target, RA2L1_REG_FSTATR1, &fstatr1);
		if (retval != ERROR_OK)
			return retval;

		if (((fstatr1 & RA2L1_FSTATR1_FRDY) != 0) == set)
			return ERROR_OK;
	}

	return ERROR_TIMEOUT_REACHED;
}

static int ra2l1_clear_errors(struct target *target)
{
	int retval = target_write_u32(target, RA2L1_REG_FRESETR, 1);
	if (retval != ERROR_OK)
		return retval;

	return target_write_u32(target, RA2L1_REG_FRESETR, 0);
}

static int ra2l1_check_errors(struct target *target)
{
	uint32_t fstatr2;
	int retval = target_read_u32(target, RA2L1_REG_FSTATR2, &fstatr2);
	if (retval != ERROR_OK)
		return retval;

	if ((fstatr2 & RA2L1_FSTATR2_ERR_MASK) == 0)
		return ERROR_OK;

	LOG_ERROR("RA2L1 flash status error: FSTATR2=0x%08" PRIx32, fstatr2);
	retval = ra2l1_clear_errors(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_FLASH_OPERATION_FAILED;
}

static int ra2l1_write_fpmcr_seq(struct target *target, uint8_t value)
{
	int retval = target_write_u8(target, RA2L1_REG_FPR, 0xA5);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA2L1_REG_FPMCR, value);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA2L1_REG_FPMCR, (uint8_t)~value);
	if (retval != ERROR_OK)
		return retval;

	return target_write_u8(target, RA2L1_REG_FPMCR, value);
}

static int ra2l1_enter_pe_mode(struct flash_bank *bank)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;

	int retval = target_write_u16(target, RA2L1_REG_FENTRYR,
			info->is_data_flash ? RA2L1_FENTRYR_DATA_PE : RA2L1_FENTRYR_CODE_PE);
	if (retval != ERROR_OK)
		return retval;

	retval = ra2l1_write_fpmcr_seq(target,
			info->is_data_flash ? RA2L1_FPMCR_DATA_PE : RA2L1_FPMCR_CODE_PE);
	if (retval != ERROR_OK)
		return retval;

	return ra2l1_check_errors(target);
}

static int ra2l1_exit_pe_mode(struct target *target)
{
	int retval = ra2l1_write_fpmcr_seq(target, RA2L1_FPMCR_READ);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u16(target, RA2L1_REG_FENTRYR, RA2L1_FENTRYR_READ);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ra2l1_erase_one_block(struct flash_bank *bank, uint32_t pe_addr)
{
	struct target *target = bank->target;
	uint32_t pe_end = pe_addr + bank->sectors[0].size - 1;
	int retval;

	retval = target_write_u16(target, RA2L1_REG_FSARH, (uint16_t)(pe_addr >> 16));
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u16(target, RA2L1_REG_FSARL, (uint16_t)pe_addr);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA2L1_REG_FEARH, (uint16_t)(pe_end >> 16));
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u16(target, RA2L1_REG_FEARL, (uint16_t)pe_end);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA2L1_REG_FCR, RA2L1_FCR_ERASE_PREP);
	if (retval != ERROR_OK)
		return retval;

	retval = ra2l1_wait_frdy(target, true, RA2L1_FLASH_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA2L1_REG_FCR, RA2L1_FCR_ERASE_EXEC);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, RA2L1_REG_FCR, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = ra2l1_wait_frdy(target, false, RA2L1_FLASH_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	return ra2l1_check_errors(target);
}

static int ra2l1_program_one_unit(struct flash_bank *bank, uint32_t pe_addr,
		const uint8_t *buffer)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	retval = target_write_u16(target, RA2L1_REG_FSARH, (uint16_t)(pe_addr >> 16));
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u16(target, RA2L1_REG_FSARL, (uint16_t)pe_addr);
	if (retval != ERROR_OK)
		return retval;

	if (info->program_unit == 4) {
		uint32_t value = target_buffer_get_u32(target, buffer);
		retval = target_write_u32(target, RA2L1_REG_FWBL0, value & 0xFFFF);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, RA2L1_REG_FWBH0, value >> 16);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = target_write_u32(target, RA2L1_REG_FWBL0, buffer[0]);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u8(target, RA2L1_REG_FCR, RA2L1_FCR_PROG_PREP);
	if (retval != ERROR_OK)
		return retval;

	retval = ra2l1_wait_frdy(target, true, RA2L1_FLASH_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA2L1_REG_FCR, RA2L1_FCR_PROG_EXEC);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, RA2L1_REG_FCR, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = ra2l1_wait_frdy(target, false, RA2L1_FLASH_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	return ra2l1_check_errors(target);
}

static int ra2l1_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval;
	uint32_t target_address = info->pe_base + offset;

	static const uint8_t ra2l1_flash_write_code[] = {
#include "../../../contrib/loaders/flash/renesas/ra2l1.inc"
	};

	if (target_alloc_working_area(target, sizeof(ra2l1_flash_write_code),
			&write_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(ra2l1_flash_write_code), ra2l1_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	const size_t extra_size = sizeof(struct ra2l1_loader_work_area);
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
			offsetof(struct ra2l1_loader_work_area, program_unit),
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
			source->address + offsetof(struct ra2l1_loader_work_area, stack) + RA2L1_LOADER_STACK_SIZE);

	retval = target_run_flash_async_algorithm(target, buffer, count, info->program_unit,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address + offsetof(struct ra2l1_loader_work_area, fifo),
			source->size - offsetof(struct ra2l1_loader_work_area, fifo),
			write_algorithm->address, 0, &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("error executing RA2L1 flash write algorithm");

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

static int ra2l1_write_block_without_loader(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	uint32_t address = info->pe_base + offset;

	while (count--) {
		int retval = ra2l1_program_one_unit(bank, address, buffer);
		if (retval != ERROR_OK)
			return retval;
		address += info->program_unit;
		buffer += info->program_unit;
	}

	return ERROR_OK;
}

static int ra2l1_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (last >= bank->num_sectors || first > last)
		return ERROR_FLASH_SECTOR_INVALID;

	retval = ra2l1_prepare_access(target);
	if (retval != ERROR_OK)
		goto done;

	retval = ra2l1_enter_pe_mode(bank);
	if (retval != ERROR_OK)
		goto done;

	for (unsigned int i = first; i <= last; i++) {
		uint32_t pe_addr = info->pe_base + bank->sectors[i].offset;
		retval = ra2l1_erase_one_block(bank, pe_addr);
		if (retval != ERROR_OK)
			break;
		bank->sectors[i].is_erased = 1;
	}

	{
		int retval2 = ra2l1_exit_pe_mode(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}

done:
	{
		int retval2 = ra2l1_lock_prcr(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}

	return retval;
}

static int ra2l1_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;
	uint8_t *aligned_buffer = NULL;
	uint32_t aligned_count = count;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (count == 0)
		return ERROR_OK;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (!info->is_data_flash) {
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
	}

	retval = ra2l1_prepare_access(target);
	if (retval != ERROR_OK)
		goto out;

	retval = ra2l1_enter_pe_mode(bank);
	if (retval != ERROR_OK)
		goto out;

	{
		uint32_t unit_count = aligned_count / info->program_unit;
		retval = ra2l1_write_block(bank, buffer, offset, unit_count);
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			if (!ra2l1_fallback_write_logged) {
				LOG_WARNING("RA2L1: fallback to slow DAP write path");
				ra2l1_fallback_write_logged = true;
			}
			retval = ra2l1_write_block_without_loader(bank, buffer, offset, unit_count);
		}
	}

	{
		int retval2 = ra2l1_exit_pe_mode(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}

out:
	{
		int retval2 = ra2l1_lock_prcr(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	free(aligned_buffer);

	return retval;
}

static int ra2l1_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset,
		uint32_t count)
{
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	return target_read_buffer(bank->target, bank->base + offset, count, buffer);
}

static int ra2l1_probe(struct flash_bank *bank)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info)
		return ERROR_FAIL;

	if (bank->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (info->probed)
		return ERROR_OK;

	retval = ra2l1_prepare_access(bank->target);
	if (retval != ERROR_OK)
		return retval;

	bank->num_sectors = bank->size / info->erase_size;
	bank->sectors = alloc_block_array(0, info->erase_size, bank->num_sectors);
	if (!bank->sectors) {
		ra2l1_lock_prcr(bank->target);
		return ERROR_FAIL;
	}

	retval = ra2l1_lock_prcr(bank->target);
	if (retval != ERROR_OK)
		return retval;

	info->probed = true;
	return ERROR_OK;
}

static int ra2l1_auto_probe(struct flash_bank *bank)
{
	struct ra2l1_flash_bank *info = bank->driver_priv;
	if (!info)
		return ERROR_FAIL;
	if (info->probed)
		return ERROR_OK;
	return ra2l1_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(ra2l1_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct ra2l1_flash_bank *info = calloc(1, sizeof(*info));
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

	if (bank->base == RA2L1_DATA_BASE) {
		info->is_data_flash = true;
		info->erase_size = RA2L1_DATA_ERASE_BLOCK_SIZE;
		info->program_unit = RA2L1_DATA_PROGRAM_UNIT;
		info->pe_base = RA2L1_DATA_PE_BASE;
		if (bank->size == 0)
			bank->size = RA2L1_DATA_SIZE;
	} else {
		info->is_data_flash = false;
		info->erase_size = RA2L1_CODE_ERASE_BLOCK_SIZE;
		info->program_unit = RA2L1_CODE_PROGRAM_UNIT;
		info->pe_base = RA2L1_CODE_BASE;
		if (bank->size == 0)
			bank->size = RA2L1_CODE_SIZE;
	}

	return ERROR_OK;
}

static const struct command_registration ra2l1_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ra2l1_flash = {
	.name = "ra2l1",
	.commands = ra2l1_exec_command_handlers,
	.flash_bank_command = ra2l1_flash_bank_command,
	.erase = ra2l1_erase,
	.protect = NULL,
	.write = ra2l1_write,
	.read = ra2l1_read,
	.probe = ra2l1_probe,
	.auto_probe = ra2l1_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
	.info = NULL,
};
