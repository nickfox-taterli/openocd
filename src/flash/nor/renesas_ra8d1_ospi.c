// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Renesas RA8D1 external OSPI flash driver                              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

#include <helper/align.h>
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#include "renesas_ra8d1_ospi.h"

struct ra8d1_ospi_flash_bank {
	bool probed;
	uint32_t page_size;
	uint32_t sector_size;
	uint8_t cmd_write_enable;
	uint8_t cmd_status;
	uint8_t cmd_read_id;
	uint8_t cmd_read_data;
	uint8_t cmd_page_program;
	uint8_t cmd_sector_erase;
	uint8_t cmd_chip_erase;
	uint8_t busy_bit;
	uint8_t cs_channel;
	uint32_t liocfg;
};

static bool ra8d1_ospi_fallback_write_logged;
static bool ra8d1_ospi_fallback_read_logged;

static int ra8d1_ospi_ensure_halted(struct target *target)
{
	int retval;

	if (target->state == TARGET_HALTED)
		return ERROR_OK;

	retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_wait_state(target, TARGET_HALTED, 1000);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	return ERROR_OK;
}

static int ra8d1_ospi_hw_init(struct flash_bank *bank)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t mstpcrb;
	int retval;

	retval = target_read_u32(target, RA8D1_OSPI_MSTPCRB, &mstpcrb);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_MSTPCRB, mstpcrb & ~BIT(16));
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA8D1_OSPI_PWPRS, 0x00);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, RA8D1_OSPI_PWPRS, 0x40);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_PFS_P100, 0x1C010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA8D1_OSPI_PFS_P101, 0x1C010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA8D1_OSPI_PFS_P103, 0x1C010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA8D1_OSPI_PFS_P803, 0x1C010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA8D1_OSPI_PFS_P808, 0x1C010000);
	if (retval != ERROR_OK)
		return retval;

	if (info->cs_channel == 0) {
		retval = target_write_u32(target, RA8D1_OSPI_PFS_P107, 0x1C010C00);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = target_write_u32(target, RA8D1_OSPI_PFS_P104, 0x1C010000);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Keep external flash reset pin deasserted (P106 high output). */
	retval = target_write_u32(target, RA8D1_OSPI_PFS_P106, 0x00000005);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA8D1_OSPI_PWPRS, 0x00);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, RA8D1_OSPI_PWPRS, 0x80);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_REG_LIOCFGCS0, info->liocfg);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_REG_LIOCFGCS1, info->liocfg);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ra8d1_ospi_wait_trreq_clear(struct target *target, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint32_t cdctl0;
		int retval = target_read_u32(target, RA8D1_OSPI_REG_CDCTL0, &cdctl0);
		if (retval != ERROR_OK)
			return retval;
		if ((cdctl0 & RA8D1_OSPI_CDCTL0_TRREQ) == 0)
			return ERROR_OK;
		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int ra8d1_ospi_direct_transfer(struct flash_bank *bank,
		uint16_t command, uint8_t command_length,
		uint32_t address, uint8_t address_length,
		uint8_t data_length, uint8_t direction,
		uint64_t *data)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t cdt;
	uint32_t cdd0 = 0;
	uint32_t cdd1 = 0;
	uint32_t ints;
	uint32_t cs_setup = ((uint32_t)info->cs_channel << RA8D1_OSPI_CDCTL0_CSSEL_SHIFT);
	int retval;

	if (command_length == 0 || command_length > 2)
		return ERROR_FAIL;
	if (address_length > 4)
		return ERROR_FAIL;
	if (data_length > 8)
		return ERROR_FAIL;

	cdt = (((uint32_t)command_length) << RA8D1_OSPI_CDT_CMDSIZE_SHIFT) |
		(((uint32_t)address_length) << RA8D1_OSPI_CDT_ADDSIZE_SHIFT) |
		(((uint32_t)data_length) << RA8D1_OSPI_CDT_DATASIZE_SHIFT) |
		(((uint32_t)direction) << RA8D1_OSPI_CDT_TRTYPE_SHIFT);

	if (command_length == 1)
		cdt |= ((uint32_t)(command & 0xFFu) << 24);
	else
		cdt |= ((uint32_t)(command & 0xFFFFu) << RA8D1_OSPI_CDT_CMD_SHIFT);

	retval = target_write_u32(target, RA8D1_OSPI_REG_CDCTL0, cs_setup);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_wait_trreq_clear(target, RA8D1_OSPI_TIMEOUT_TRREQ_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_REG_CDBUF0_CDT, cdt);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_REG_CDBUF0_CDA, address);
	if (retval != ERROR_OK)
		return retval;

	if ((direction == RA8D1_OSPI_TRTYPE_WRITE) && (data_length > 0)) {
		if (!data)
			return ERROR_FAIL;
		cdd0 = (uint32_t)(*data & 0xFFFFFFFFu);
		cdd1 = (uint32_t)(*data >> 32);
		retval = target_write_u32(target, RA8D1_OSPI_REG_CDBUF0_CDD0, cdd0);
		if (retval != ERROR_OK)
			return retval;
		if (data_length > 4) {
			retval = target_write_u32(target, RA8D1_OSPI_REG_CDBUF0_CDD1, cdd1);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	retval = target_write_u32(target, RA8D1_OSPI_REG_INTC, 0xFFFFFFFFu);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_REG_CDCTL0, cs_setup | RA8D1_OSPI_CDCTL0_TRREQ);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_wait_trreq_clear(target, RA8D1_OSPI_TIMEOUT_TRREQ_MS);
	if (retval != ERROR_OK)
		return retval;

	if ((direction == RA8D1_OSPI_TRTYPE_READ) && (data_length > 0)) {
		retval = target_read_u32(target, RA8D1_OSPI_REG_CDBUF0_CDD0, &cdd0);
		if (retval != ERROR_OK)
			return retval;
		if (data_length > 4) {
			retval = target_read_u32(target, RA8D1_OSPI_REG_CDBUF0_CDD1, &cdd1);
			if (retval != ERROR_OK)
				return retval;
		}
		if (data)
			*data = ((uint64_t)cdd1 << 32) | cdd0;
	}

	retval = target_read_u32(target, RA8D1_OSPI_REG_INTS, &ints);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA8D1_OSPI_REG_INTC, ints);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ra8d1_ospi_read_status(struct flash_bank *bank, uint8_t *status)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint64_t data = 0;
	int retval;

	retval = ra8d1_ospi_direct_transfer(bank,
			info->cmd_status, 1,
			0, 0,
			1, RA8D1_OSPI_TRTYPE_READ,
			&data);
	if (retval != ERROR_OK)
		return retval;

	*status = (uint8_t)data;
	return ERROR_OK;
}

static int ra8d1_ospi_wait_ready(struct flash_bank *bank, unsigned int timeout_ms)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint8_t status;
		int retval = ra8d1_ospi_read_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & BIT(info->busy_bit)) == 0)
			return ERROR_OK;
		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int ra8d1_ospi_read_jedec_id(struct flash_bank *bank, uint8_t *id)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint64_t data = 0;
	int retval;

	retval = ra8d1_ospi_direct_transfer(bank,
			info->cmd_read_id, 1,
			0, 0,
			3, RA8D1_OSPI_TRTYPE_READ,
			&data);
	if (retval != ERROR_OK)
		return retval;

	id[0] = (uint8_t)data;
	id[1] = (uint8_t)(data >> 8);
	id[2] = (uint8_t)(data >> 16);
	return ERROR_OK;
}

static int ra8d1_ospi_write_enable(struct flash_bank *bank)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;

	return ra8d1_ospi_direct_transfer(bank,
			info->cmd_write_enable, 1,
			0, 0,
			0, RA8D1_OSPI_TRTYPE_WRITE,
			NULL);
}

static int ra8d1_ospi_program_chunk_dap(struct flash_bank *bank,
		uint32_t chip_address, const uint8_t *buffer, uint32_t count)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint64_t data = 0;
	int retval;

	if (count == 0 || count > 8)
		return ERROR_FAIL;

	for (uint32_t i = 0; i < count; i++)
		data |= ((uint64_t)buffer[i] << (8 * i));

	retval = ra8d1_ospi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_direct_transfer(bank,
			info->cmd_page_program, 1,
			chip_address, 3,
			(uint8_t)count, RA8D1_OSPI_TRTYPE_WRITE,
			&data);
	if (retval != ERROR_OK)
		return retval;

	return ra8d1_ospi_wait_ready(bank, RA8D1_OSPI_TIMEOUT_STATUS_MS);
}

static int ra8d1_ospi_erase_sector_dap(struct flash_bank *bank, unsigned int sector)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint32_t chip_address = bank->sectors[sector].offset;
	int retval;

	retval = ra8d1_ospi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_direct_transfer(bank,
			info->cmd_sector_erase, 1,
			chip_address, 3,
			0, RA8D1_OSPI_TRTYPE_WRITE,
			NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_wait_ready(bank, RA8D1_OSPI_TIMEOUT_SECTOR_ERASE_MS);
	if (retval == ERROR_OK)
		bank->sectors[sector].is_erased = 1;

	return retval;
}

static int ra8d1_ospi_chip_erase_dap(struct flash_bank *bank)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	int retval;

	retval = ra8d1_ospi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_direct_transfer(bank,
			info->cmd_chip_erase, 1,
			0, 0,
			0, RA8D1_OSPI_TRTYPE_WRITE,
			NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_wait_ready(bank, RA8D1_OSPI_TIMEOUT_CHIP_ERASE_MS);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	return ERROR_OK;
}

static int ra8d1_ospi_read_data_chunk(struct flash_bank *bank,
		uint32_t chip_address, uint8_t *buffer, uint32_t count)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint64_t data = 0;
	int retval;

	retval = ra8d1_ospi_direct_transfer(bank,
			info->cmd_read_data, 1,
			chip_address, 3,
			(uint8_t)count, RA8D1_OSPI_TRTYPE_READ,
			&data);
	if (retval != ERROR_OK)
		return retval;

	for (uint32_t i = 0; i < count; i++)
		buffer[i] = (uint8_t)(data >> (8 * i));

	return ERROR_OK;
}

static int ra8d1_ospi_write_block_without_loader(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint32_t chip_address = offset;

	while (count) {
		uint32_t page_off = chip_address & (info->page_size - 1);
		uint32_t page_left = info->page_size - page_off;
		uint32_t this_page = (count < page_left) ? count : page_left;

		while (this_page) {
			uint32_t this_chunk = (this_page > 8) ? 8 : this_page;
			int retval = ra8d1_ospi_program_chunk_dap(bank, chip_address, buffer, this_chunk);
			if (retval != ERROR_OK)
				return retval;

			chip_address += this_chunk;
			buffer += this_chunk;
			count -= this_chunk;
			this_page -= this_chunk;
		}
	}

	return ERROR_OK;
}

static int ra8d1_ospi_loader_write_once(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t chip_address, uint32_t count)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *algo = NULL;
	struct working_area *wa = NULL;
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[5];
	struct ra8d1_ospi_loader_cfg cfg;
	uint32_t cfg_addr;
	uint32_t data_addr;
	uint32_t data_max;
	int retval = ERROR_FAIL;

	static const uint8_t ra8d1_ospi_loader_code[] = {
#include "../../../contrib/loaders/flash/renesas/ra8d1_ospi.inc"
	};

	if (target_alloc_working_area(target, sizeof(ra8d1_ospi_loader_code), &algo) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, algo->address,
			sizeof(ra8d1_ospi_loader_code), ra8d1_ospi_loader_code);
	if (retval != ERROR_OK)
		goto cleanup;

	if (target_get_working_area_avail(target) <= sizeof(cfg) + 256) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	if (target_alloc_working_area_try(target,
			target_get_working_area_avail(target), &wa) != ERROR_OK) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	cfg_addr = wa->address;
	data_addr = cfg_addr + sizeof(cfg);
	data_max = wa->size - sizeof(cfg);
	if (data_max == 0) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	cfg.page_size = info->page_size;
	cfg.status_timeout_loops = 800000;
	cfg.cmd_write_enable = info->cmd_write_enable;
	cfg.cmd_page_program = info->cmd_page_program;
	cfg.cmd_status = info->cmd_status;
	cfg.cmd_read_data = info->cmd_read_data;
	cfg.busy_bit_mask = BIT(info->busy_bit);
	cfg.operation = RA8D1_OSPI_LOADER_OP_WRITE;
	cfg.cs_setup = ((uint32_t)info->cs_channel << RA8D1_OSPI_CDCTL0_CSSEL_SHIFT);
	cfg.last_error = 0;
	memset(cfg.stack, 0, sizeof(cfg.stack));

	retval = target_write_buffer(target, cfg_addr, sizeof(cfg), (const uint8_t *)&cfg);
	if (retval != ERROR_OK)
		goto cleanup;

	memset(&armv7m_info, 0, sizeof(armv7m_info));
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	while (count) {
		uint32_t this_size = (count > data_max) ? data_max : count;

		retval = target_write_buffer(target, data_addr, this_size, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, cfg_addr);
		buf_set_u32(reg_params[1].value, 0, 32, data_addr);
		buf_set_u32(reg_params[2].value, 0, 32, chip_address);
		buf_set_u32(reg_params[3].value, 0, 32, this_size);
		buf_set_u32(reg_params[4].value, 0, 32,
				cfg_addr + offsetof(struct ra8d1_ospi_loader_cfg, stack) + RA8D1_OSPI_LOADER_STACK_SIZE);

		retval = target_run_algorithm(target, 0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algo->address, 0,
				(this_size / info->page_size + 1) * RA8D1_OSPI_TIMEOUT_STATUS_MS,
				&armv7m_info);
		if (retval != ERROR_OK)
			break;

		retval = target_read_buffer(target,
				cfg_addr + offsetof(struct ra8d1_ospi_loader_cfg, last_error),
				sizeof(cfg.last_error), (uint8_t *)&cfg.last_error);
		if (retval != ERROR_OK)
			break;
		if (cfg.last_error != 0) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		chip_address += this_size;
		buffer += this_size;
		count -= this_size;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

cleanup:
	if (wa)
		target_free_working_area(target, wa);
	if (algo)
		target_free_working_area(target, algo);

	return retval;
}

static int ra8d1_ospi_loader_read_once(struct flash_bank *bank,
		uint8_t *buffer, uint32_t chip_address, uint32_t count)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *algo = NULL;
	struct working_area *wa = NULL;
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[5];
	struct ra8d1_ospi_loader_cfg cfg;
	uint32_t cfg_addr;
	uint32_t data_addr;
	uint32_t data_max;
	int retval = ERROR_FAIL;

	static const uint8_t ra8d1_ospi_loader_code[] = {
#include "../../../contrib/loaders/flash/renesas/ra8d1_ospi.inc"
	};

	if (target_alloc_working_area(target, sizeof(ra8d1_ospi_loader_code), &algo) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, algo->address,
			sizeof(ra8d1_ospi_loader_code), ra8d1_ospi_loader_code);
	if (retval != ERROR_OK)
		goto cleanup;

	if (target_get_working_area_avail(target) <= sizeof(cfg) + 256) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	if (target_alloc_working_area_try(target,
			target_get_working_area_avail(target), &wa) != ERROR_OK) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	cfg_addr = wa->address;
	data_addr = cfg_addr + sizeof(cfg);
	data_max = wa->size - sizeof(cfg);
	if (data_max == 0) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	cfg.page_size = info->page_size;
	cfg.status_timeout_loops = 800000;
	cfg.cmd_write_enable = info->cmd_write_enable;
	cfg.cmd_page_program = info->cmd_page_program;
	cfg.cmd_status = info->cmd_status;
	cfg.cmd_read_data = info->cmd_read_data;
	cfg.busy_bit_mask = BIT(info->busy_bit);
	cfg.operation = RA8D1_OSPI_LOADER_OP_READ;
	cfg.cs_setup = ((uint32_t)info->cs_channel << RA8D1_OSPI_CDCTL0_CSSEL_SHIFT);
	cfg.last_error = 0;
	memset(cfg.stack, 0, sizeof(cfg.stack));

	retval = target_write_buffer(target, cfg_addr, sizeof(cfg), (const uint8_t *)&cfg);
	if (retval != ERROR_OK)
		goto cleanup;

	memset(&armv7m_info, 0, sizeof(armv7m_info));
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	while (count) {
		uint32_t this_size = (count > data_max) ? data_max : count;
		uint32_t timeout_ms = 1000 + (this_size / 8);

		buf_set_u32(reg_params[0].value, 0, 32, cfg_addr);
		buf_set_u32(reg_params[1].value, 0, 32, data_addr);
		buf_set_u32(reg_params[2].value, 0, 32, chip_address);
		buf_set_u32(reg_params[3].value, 0, 32, this_size);
		buf_set_u32(reg_params[4].value, 0, 32,
				cfg_addr + offsetof(struct ra8d1_ospi_loader_cfg, stack) + RA8D1_OSPI_LOADER_STACK_SIZE);

		retval = target_run_algorithm(target, 0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algo->address, 0,
				timeout_ms,
				&armv7m_info);
		if (retval != ERROR_OK)
			break;

		retval = target_read_buffer(target,
				cfg_addr + offsetof(struct ra8d1_ospi_loader_cfg, last_error),
				sizeof(cfg.last_error), (uint8_t *)&cfg.last_error);
		if (retval != ERROR_OK)
			break;
		if (cfg.last_error != 0) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		retval = target_read_buffer(target, data_addr, this_size, buffer);
		if (retval != ERROR_OK)
			break;

		chip_address += this_size;
		buffer += this_size;
		count -= this_size;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

cleanup:
	if (wa)
		target_free_working_area(target, wa);
	if (algo)
		target_free_working_area(target, algo);

	return retval;
}

static int ra8d1_ospi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint32_t chip_address = offset;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	retval = ra8d1_ospi_loader_read_once(bank, buffer, chip_address, count);
	if ((retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) ||
			(retval == ERROR_FLASH_OPERATION_FAILED)) {
		if (!ra8d1_ospi_fallback_read_logged) {
			LOG_WARNING("RA8D1 OSPI: fallback to slow DAP read path");
			ra8d1_ospi_fallback_read_logged = true;
		}
	} else {
		return retval;
	}

	while (count) {
		uint32_t this_size = (count > 8) ? 8 : count;
		retval = ra8d1_ospi_read_data_chunk(bank, chip_address, buffer, this_size);
		if (retval != ERROR_OK)
			return retval;

		chip_address += this_size;
		buffer += this_size;
		count -= this_size;
	}

	return ERROR_OK;
}

static int ra8d1_ospi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	retval = ra8d1_ospi_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	if (last >= bank->num_sectors || first > last)
		return ERROR_FLASH_SECTOR_INVALID;

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return ra8d1_ospi_chip_erase_dap(bank);

	for (unsigned int i = first; i <= last; i++) {
		retval = ra8d1_ospi_erase_sector_dap(bank, i);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int ra8d1_ospi_write(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	retval = ra8d1_ospi_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	if (count == 0)
		return ERROR_OK;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	retval = ra8d1_ospi_loader_write_once(bank, buffer, offset, count);
	if ((retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) ||
			(retval == ERROR_FLASH_OPERATION_FAILED)) {
		if (!ra8d1_ospi_fallback_write_logged) {
			LOG_WARNING("RA8D1 OSPI: fallback to slow DAP write path");
			ra8d1_ospi_fallback_write_logged = true;
		}
		retval = ra8d1_ospi_write_block_without_loader(bank, buffer, offset, count);
	}

	return retval;
}

static uint32_t ra8d1_ospi_jedec_capacity_to_size(uint8_t capacity_code)
{
	if (capacity_code >= 10 && capacity_code <= 31)
		return (uint32_t)1u << capacity_code;

	return 0;
}

static int ra8d1_ospi_probe(struct flash_bank *bank)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	uint8_t jedec_id[3] = {0};
	uint32_t size_from_id;
	int retval;

	if (!info)
		return ERROR_FAIL;

	retval = ra8d1_ospi_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra8d1_ospi_hw_init(bank);
	if (retval != ERROR_OK)
		return retval;

	if (info->probed)
		return ERROR_OK;

	retval = ra8d1_ospi_read_jedec_id(bank, jedec_id);
	if (retval != ERROR_OK) {
		LOG_ERROR("RA8D1 OSPI JEDEC read failed");
		return retval;
	}

	LOG_INFO("RA8D1 OSPI JEDEC ID: %02" PRIx8 " %02" PRIx8 " %02" PRIx8,
			jedec_id[0], jedec_id[1], jedec_id[2]);

	if (((jedec_id[0] == 0x00) && (jedec_id[1] == 0x00) && (jedec_id[2] == 0x00)) ||
			((jedec_id[0] == 0xFF) && (jedec_id[1] == 0xFF) && (jedec_id[2] == 0xFF))) {
		LOG_ERROR("RA8D1 OSPI JEDEC ID looks invalid. Check OSPI pinmux/CS channel.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	size_from_id = ra8d1_ospi_jedec_capacity_to_size(jedec_id[2]);
	if (bank->size == 0)
		bank->size = size_from_id ? size_from_id : RA8D1_OSPI_DEFAULT_SIZE;

	if (!IS_ALIGNED(bank->size, info->sector_size)) {
		LOG_ERROR("RA8D1 OSPI bank size (0x%08" PRIx32 ") is not aligned to sector size (0x%08" PRIx32 ")",
				bank->size, info->sector_size);
		return ERROR_FAIL;
	}

	bank->num_sectors = bank->size / info->sector_size;
	bank->sectors = alloc_block_array(0, info->sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	info->probed = true;
	return ERROR_OK;
}

static int ra8d1_ospi_auto_probe(struct flash_bank *bank)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;

	if (!info)
		return ERROR_FAIL;
	if (info->probed)
		return ERROR_OK;
	return ra8d1_ospi_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(ra8d1_ospi_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct ra8d1_ospi_flash_bank *info = calloc(1, sizeof(*info));
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

	info->page_size = RA8D1_OSPI_DEFAULT_PAGE_SIZE;
	info->sector_size = RA8D1_OSPI_DEFAULT_SECTOR_SIZE;
	info->cmd_write_enable = RA8D1_OSPI_CMD_WRITE_ENABLE;
	info->cmd_status = RA8D1_OSPI_CMD_READ_STATUS;
	info->cmd_read_id = RA8D1_OSPI_CMD_READ_ID;
	info->cmd_read_data = RA8D1_OSPI_CMD_READ_DATA;
	info->cmd_page_program = RA8D1_OSPI_CMD_PAGE_PROGRAM;
	info->cmd_sector_erase = RA8D1_OSPI_CMD_SECTOR_ERASE;
	info->cmd_chip_erase = RA8D1_OSPI_CMD_CHIP_ERASE;
	info->busy_bit = RA8D1_OSPI_BUSY_BIT;
	info->cs_channel = 1;
	info->liocfg = RA8D1_OSPI_LIOCFG_DEFAULT;

	if (CMD_ARGC > 6)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->sector_size);
	if (CMD_ARGC > 7)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], info->page_size);
	if (CMD_ARGC > 8)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[8], info->cmd_sector_erase);
	if (CMD_ARGC > 9)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[9], info->cmd_page_program);
	if (CMD_ARGC > 10)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[10], info->cmd_status);
	if (CMD_ARGC > 11)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[11], info->cmd_write_enable);
	if (CMD_ARGC > 12)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[12], info->cmd_read_id);
	if (CMD_ARGC > 13)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[13], info->cmd_read_data);
	if (CMD_ARGC > 14)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[14], info->cmd_chip_erase);
	if (CMD_ARGC > 15)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[15], info->cs_channel);
	if (CMD_ARGC > 16)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[16], info->liocfg);

	if (!IS_PWR_OF_2(info->page_size) || !IS_PWR_OF_2(info->sector_size) ||
			(info->page_size > info->sector_size)) {
		LOG_ERROR("invalid RA8D1 OSPI geometry: page=0x%08" PRIx32 ", sector=0x%08" PRIx32,
				info->page_size, info->sector_size);
		return ERROR_FAIL;
	}

	if (info->cs_channel > 1) {
		LOG_ERROR("invalid RA8D1 OSPI cs_channel=%" PRIu8 " (must be 0 or 1)", info->cs_channel);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int ra8d1_ospi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct ra8d1_ospi_flash_bank *info = bank->driver_priv;
	if (!info)
		return ERROR_FAIL;

	command_print_sameline(cmd,
		"RA8D1 OSPI: base=0x%08" TARGET_PRIxADDR " size=0x%08" PRIx32
		" sector=0x%08" PRIx32 " page=0x%08" PRIx32
		" cs_channel=%" PRIu8 " liocfg=0x%08" PRIx32,
		bank->base, (uint32_t)bank->size,
		info->sector_size, info->page_size,
		info->cs_channel, info->liocfg);

	return ERROR_OK;
}

static const struct command_registration ra8d1_ospi_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ra8d1_ospi_flash = {
	.name = "ra8d1_ospi",
	.commands = ra8d1_ospi_exec_command_handlers,
	.flash_bank_command = ra8d1_ospi_flash_bank_command,
	.erase = ra8d1_ospi_erase,
	.protect = NULL,
	.write = ra8d1_ospi_write,
	.read = ra8d1_ospi_read,
	.probe = ra8d1_ospi_probe,
	.auto_probe = ra8d1_ospi_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
	.info = ra8d1_ospi_info,
	.usage = "<base> <size> <chip_width> <bus_width> <target> "
			"[sector_size] [page_size] [sector_erase_cmd] [page_program_cmd] [status_cmd] "
			"[write_enable_cmd] [read_id_cmd] [read_data_cmd] [chip_erase_cmd] [cs_channel] [liocfg]",
};
