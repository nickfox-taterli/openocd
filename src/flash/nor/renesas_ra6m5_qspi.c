// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Renesas RA6M5 external QSPI flash driver                              *
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

#include "renesas_ra6m5_qspi.h"

struct ra6m5_qspi_flash_bank {
	bool probed;
	uint32_t page_size;
	uint32_t sector_size;
	uint32_t address_bytes;
	uint8_t cmd_write_enable;
	uint8_t cmd_status;
	uint8_t cmd_read_id;
	uint8_t cmd_page_program;
	uint8_t cmd_sector_erase;
	uint8_t busy_bit;
};

static bool ra6m5_qspi_fallback_write_logged;

static int ra6m5_qspi_ensure_halted(struct target *target)
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

static int ra6m5_qspi_hw_init(struct target *target)
{
	uint32_t mstpcrb;
	int retval;

	retval = target_write_u16(target, RA6M5_SYSTEM_PRCR, 0xA502);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, RA6M5_MSTPCRB, &mstpcrb);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_MSTPCRB, mstpcrb & ~BIT(6));
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u16(target, RA6M5_SYSTEM_PRCR, 0xA500);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA6M5_PMISC_PWPR, 0x00);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, RA6M5_PMISC_PWPR, 0x40);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA6M5_PFS_P305, 0x11010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_PFS_P306, 0x11010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_PFS_P307, 0x11010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_PFS_P308, 0x11010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_PFS_P309, 0x11010000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_PFS_P310, 0x11010000);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, RA6M5_PMISC_PWPR, 0x80);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMCST, 0x00000000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMSIC, 0x00000000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMPMD, 0x00000000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMCNT1, 0x00000000);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMSPC, 0x00000010);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMSAC, RA6M5_QSPI_ADDR_BYTES_3);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMSKC, 0x00000001);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMSMD, 0x00000045);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, RA6M5_QSPI_REG_SFMSSC, 0x00000034);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ra6m5_qspi_enter_direct(struct target *target)
{
	return target_write_u32(target, RA6M5_QSPI_REG_SFMCMD, 1);
}

static int ra6m5_qspi_close_cycle(struct target *target)
{
	return target_write_u32(target, RA6M5_QSPI_REG_SFMCMD, 1);
}

static int ra6m5_qspi_exit_direct(struct target *target)
{
	return target_write_u32(target, RA6M5_QSPI_REG_SFMCMD, 0);
}

static int ra6m5_qspi_send_byte(struct target *target, uint8_t value)
{
	return target_write_u8(target, RA6M5_QSPI_REG_SFMCOM, value);
}

static int ra6m5_qspi_read_byte(struct target *target, uint8_t *value)
{
	return target_read_u8(target, RA6M5_QSPI_REG_SFMCOM, value);
}

static int ra6m5_qspi_chip_address(struct target *target, uint32_t mapped_addr, uint32_t *chip_addr)
{
	uint32_t sfmcnt1;
	int retval;

	retval = target_read_u32(target, RA6M5_QSPI_REG_SFMCNT1, &sfmcnt1);
	if (retval != ERROR_OK)
		return retval;

	if (mapped_addr < RA6M5_QSPI_MAP_BASE)
		return ERROR_FAIL;

	*chip_addr = (mapped_addr - RA6M5_QSPI_MAP_BASE) + sfmcnt1;
	return ERROR_OK;
}

static int ra6m5_qspi_send_address(struct ra6m5_qspi_flash_bank *info,
		struct target *target, uint32_t chip_addr)
{
	int retval;

	if (info->address_bytes == RA6M5_QSPI_ADDR_BYTES_4) {
		retval = ra6m5_qspi_send_byte(target, (uint8_t)(chip_addr >> 24));
		if (retval != ERROR_OK)
			return retval;
	}

	retval = ra6m5_qspi_send_byte(target, (uint8_t)(chip_addr >> 16));
	if (retval != ERROR_OK)
		return retval;
	retval = ra6m5_qspi_send_byte(target, (uint8_t)(chip_addr >> 8));
	if (retval != ERROR_OK)
		return retval;
	return ra6m5_qspi_send_byte(target, (uint8_t)chip_addr);
}

static int ra6m5_qspi_read_status(struct flash_bank *bank, uint8_t *status)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	retval = ra6m5_qspi_enter_direct(target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_send_byte(target, info->cmd_status);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_read_byte(target, status);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_close_cycle(target);
	if (retval != ERROR_OK)
		goto out;

out:
	{
		int retval2 = ra6m5_qspi_exit_direct(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int ra6m5_qspi_wait_ready(struct flash_bank *bank, unsigned int timeout_ms)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint8_t status = 0;
		int retval = ra6m5_qspi_read_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & BIT(info->busy_bit)) == 0)
			return ERROR_OK;
		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int ra6m5_qspi_read_jedec_id(struct flash_bank *bank, uint8_t *id)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	retval = ra6m5_qspi_enter_direct(target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_send_byte(target, info->cmd_read_id);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_read_byte(target, &id[0]);
	if (retval != ERROR_OK)
		goto out;
	retval = ra6m5_qspi_read_byte(target, &id[1]);
	if (retval != ERROR_OK)
		goto out;
	retval = ra6m5_qspi_read_byte(target, &id[2]);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_close_cycle(target);
	if (retval != ERROR_OK)
		goto out;

out:
	{
		int retval2 = ra6m5_qspi_exit_direct(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int ra6m5_qspi_write_enable(struct flash_bank *bank)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	retval = ra6m5_qspi_enter_direct(target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_send_byte(target, info->cmd_write_enable);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_close_cycle(target);
	if (retval != ERROR_OK)
		goto out;

out:
	{
		int retval2 = ra6m5_qspi_exit_direct(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int ra6m5_qspi_program_page_dap(struct flash_bank *bank,
		uint32_t mapped_addr, const uint8_t *buffer, uint32_t count)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t chip_addr;
	int retval;

	retval = ra6m5_qspi_chip_address(target, mapped_addr, &chip_addr);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_enter_direct(target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_send_byte(target, info->cmd_page_program);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_send_address(info, target, chip_addr);
	if (retval != ERROR_OK)
		goto out;

	for (uint32_t i = 0; i < count; i++) {
		retval = ra6m5_qspi_send_byte(target, buffer[i]);
		if (retval != ERROR_OK)
			goto out;
	}

	retval = ra6m5_qspi_close_cycle(target);
	if (retval != ERROR_OK)
		goto out;

out:
	{
		int retval2 = ra6m5_qspi_exit_direct(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	if (retval != ERROR_OK)
		return retval;

	return ra6m5_qspi_wait_ready(bank, RA6M5_QSPI_TIMEOUT_STATUS_MS);
}

static int ra6m5_qspi_erase_sector_dap(struct flash_bank *bank, unsigned int sector)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t mapped_addr = bank->base + bank->sectors[sector].offset;
	uint32_t chip_addr;
	int retval;

	retval = ra6m5_qspi_chip_address(target, mapped_addr, &chip_addr);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_enter_direct(target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_send_byte(target, info->cmd_sector_erase);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_send_address(info, target, chip_addr);
	if (retval != ERROR_OK)
		goto out;

	retval = ra6m5_qspi_close_cycle(target);
	if (retval != ERROR_OK)
		goto out;

out:
	{
		int retval2 = ra6m5_qspi_exit_direct(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_wait_ready(bank, RA6M5_QSPI_TIMEOUT_ERASE_MS);
	if (retval == ERROR_OK)
		bank->sectors[sector].is_erased = 1;

	return retval;
}

static int ra6m5_qspi_write_block_without_loader(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	uint32_t addr = bank->base + offset;

	while (count) {
		uint32_t page_off = addr & (info->page_size - 1);
		uint32_t this_size = info->page_size - page_off;
		if (this_size > count)
			this_size = count;

		int retval = ra6m5_qspi_program_page_dap(bank, addr, buffer, this_size);
		if (retval != ERROR_OK)
			return retval;

		addr += this_size;
		buffer += this_size;
		count -= this_size;
	}

	return ERROR_OK;
}

static int ra6m5_qspi_loader_write_once(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t mapped_addr, uint32_t count)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *algo;
	struct working_area *wa;
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[5];
	struct ra6m5_qspi_loader_cfg cfg;
	uint32_t cfg_addr;
	uint32_t data_addr;
	uint32_t data_max;
	int retval = ERROR_FAIL;

	static const uint8_t ra6m5_qspi_loader_code[] = {
#include "../../../contrib/loaders/flash/renesas/ra6m5_qspi.inc"
	};

	if (target_alloc_working_area(target, sizeof(ra6m5_qspi_loader_code), &algo) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, algo->address,
			sizeof(ra6m5_qspi_loader_code), ra6m5_qspi_loader_code);
	if (retval != ERROR_OK)
		goto cleanup_algo;

	if (target_get_working_area_avail(target) <= sizeof(cfg) + 256) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup_algo;
	}

	if (target_alloc_working_area_try(target,
			target_get_working_area_avail(target), &wa) != ERROR_OK) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup_algo;
	}

	cfg_addr = wa->address;
	data_addr = cfg_addr + sizeof(cfg);
	data_max = wa->size - sizeof(cfg);
	if (data_max == 0) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup_wa;
	}

	cfg.page_size = info->page_size;
	cfg.address_bytes = info->address_bytes;
	cfg.write_enable_cmd = info->cmd_write_enable;
	cfg.page_program_cmd = info->cmd_page_program;
	cfg.status_cmd = info->cmd_status;
	cfg.busy_bit_mask = BIT(info->busy_bit);
	cfg.status_timeout_loops = 800000;
	cfg.last_error = 0;
	memset(cfg.stack, 0, sizeof(cfg.stack));

	retval = target_write_buffer(target, cfg_addr, sizeof(cfg), (const uint8_t *)&cfg);
	if (retval != ERROR_OK)
		goto cleanup_wa;

	memset(&armv7m_info, 0, sizeof(armv7m_info));
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	while (count) {
		uint32_t this_size = count > data_max ? data_max : count;
		retval = target_write_buffer(target, data_addr, this_size, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, cfg_addr);
		buf_set_u32(reg_params[1].value, 0, 32, data_addr);
		buf_set_u32(reg_params[2].value, 0, 32, mapped_addr);
		buf_set_u32(reg_params[3].value, 0, 32, this_size);
		buf_set_u32(reg_params[4].value, 0, 32,
				cfg_addr + offsetof(struct ra6m5_qspi_loader_cfg, stack) + RA6M5_QSPI_LOADER_STACK_SIZE);

		retval = target_run_algorithm(target, 0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algo->address, 0,
				(this_size / info->page_size + 1) * RA6M5_QSPI_TIMEOUT_STATUS_MS,
				&armv7m_info);
		if (retval != ERROR_OK)
			break;

		retval = target_read_buffer(target,
				cfg_addr + offsetof(struct ra6m5_qspi_loader_cfg, last_error),
				sizeof(cfg.last_error), (uint8_t *)&cfg.last_error);
		if (retval != ERROR_OK)
			break;
		if (cfg.last_error != 0) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		mapped_addr += this_size;
		buffer += this_size;
		count -= this_size;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

cleanup_wa:
	target_free_working_area(target, wa);
cleanup_algo:
	target_free_working_area(target, algo);
	return retval;
}

static int ra6m5_qspi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	return target_read_buffer(bank->target, bank->base + offset, count, buffer);
}

static int ra6m5_qspi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	retval = ra6m5_qspi_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	if (last >= bank->num_sectors || first > last)
		return ERROR_FLASH_SECTOR_INVALID;

	for (unsigned int i = first; i <= last; i++) {
		retval = ra6m5_qspi_erase_sector_dap(bank, i);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int ra6m5_qspi_write(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	retval = ra6m5_qspi_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	if (count == 0)
		return ERROR_OK;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	retval = ra6m5_qspi_loader_write_once(bank, buffer, bank->base + offset, count);
	if ((retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) ||
			(retval == ERROR_FLASH_OPERATION_FAILED)) {
		if (!ra6m5_qspi_fallback_write_logged) {
			LOG_WARNING("RA6M5 QSPI: fallback to slow DAP write path");
			ra6m5_qspi_fallback_write_logged = true;
		}
		retval = ra6m5_qspi_write_block_without_loader(bank, buffer, offset, count);
	}

	return retval;
}

static int ra6m5_qspi_probe(struct flash_bank *bank)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	uint32_t sfmsac;
	uint8_t jedec_id[3] = {0};
	int retval;

	if (!info)
		return ERROR_FAIL;

	retval = ra6m5_qspi_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	retval = ra6m5_qspi_hw_init(bank->target);
	if (retval != ERROR_OK)
		return retval;

	if (info->probed)
		return ERROR_OK;

	if (bank->size == 0)
		bank->size = RA6M5_QSPI_DEFAULT_SIZE;

	retval = target_read_u32(bank->target, RA6M5_QSPI_REG_SFMSAC, &sfmsac);
	if (retval == ERROR_OK) {
		uint32_t mode = sfmsac & RA6M5_QSPI_SFMSAC_SFMAS_MASK;
		if (mode == RA6M5_QSPI_ADDR_BYTES_4)
			info->address_bytes = RA6M5_QSPI_ADDR_BYTES_4;
		else
			info->address_bytes = RA6M5_QSPI_ADDR_BYTES_3;
	}

	retval = ra6m5_qspi_read_jedec_id(bank, jedec_id);
	if (retval != ERROR_OK) {
		LOG_ERROR("RA6M5 QSPI JEDEC read failed");
		return retval;
	}

	LOG_INFO("RA6M5 QSPI JEDEC ID: %02" PRIx8 " %02" PRIx8 " %02" PRIx8,
			jedec_id[0], jedec_id[1], jedec_id[2]);

	if (((jedec_id[0] == 0x00) && (jedec_id[1] == 0x00) && (jedec_id[2] == 0x00)) ||
			((jedec_id[0] == 0xFF) && (jedec_id[1] == 0xFF) && (jedec_id[2] == 0xFF))) {
		LOG_ERROR("RA6M5 QSPI JEDEC ID looks invalid. QSPI pins/clock/controller may be uninitialized.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (!IS_ALIGNED(bank->size, info->sector_size)) {
		LOG_ERROR("RA6M5 QSPI bank size (0x%08" PRIx32 ") is not aligned to sector size (0x%08" PRIx32 ")",
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

static int ra6m5_qspi_auto_probe(struct flash_bank *bank)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;

	if (!info)
		return ERROR_FAIL;
	if (info->probed)
		return ERROR_OK;
	return ra6m5_qspi_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(ra6m5_qspi_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct ra6m5_qspi_flash_bank *info = calloc(1, sizeof(*info));
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

	info->page_size = RA6M5_QSPI_DEFAULT_PAGE_SIZE;
	info->sector_size = RA6M5_QSPI_DEFAULT_SECTOR_SIZE;
	info->address_bytes = RA6M5_QSPI_ADDR_BYTES_3;
	info->cmd_write_enable = RA6M5_QSPI_CMD_WRITE_ENABLE;
	info->cmd_status = RA6M5_QSPI_CMD_READ_STATUS;
	info->cmd_read_id = 0x9f;
	info->cmd_page_program = RA6M5_QSPI_CMD_PAGE_PROGRAM;
	info->cmd_sector_erase = RA6M5_QSPI_CMD_SECTOR_ERASE;
	info->busy_bit = RA6M5_QSPI_BUSY_BIT;

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

	if (!IS_PWR_OF_2(info->page_size) || !IS_PWR_OF_2(info->sector_size) ||
			(info->page_size > info->sector_size)) {
		LOG_ERROR("invalid RA6M5 QSPI geometry: page=0x%08" PRIx32 ", sector=0x%08" PRIx32,
				info->page_size, info->sector_size);
		return ERROR_FAIL;
	}

	if (bank->size == 0)
		bank->size = RA6M5_QSPI_DEFAULT_SIZE;

	return ERROR_OK;
}

static int ra6m5_qspi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct ra6m5_qspi_flash_bank *info = bank->driver_priv;
	if (!info)
		return ERROR_FAIL;

	command_print_sameline(cmd,
		"RA6M5 QSPI: base=0x%08" TARGET_PRIxADDR " size=0x%08" PRIx32
		" sector=0x%08" PRIx32 " page=0x%08" PRIx32 " addr_bytes=%" PRIu32,
		bank->base, (uint32_t)bank->size, info->sector_size, info->page_size, info->address_bytes);
	return ERROR_OK;
}

static const struct command_registration ra6m5_qspi_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ra6m5_qspi_flash = {
	.name = "ra6m5_qspi",
	.commands = ra6m5_qspi_exec_command_handlers,
	.flash_bank_command = ra6m5_qspi_flash_bank_command,
	.erase = ra6m5_qspi_erase,
	.protect = NULL,
	.write = ra6m5_qspi_write,
	.read = ra6m5_qspi_read,
	.probe = ra6m5_qspi_probe,
	.auto_probe = ra6m5_qspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
	.info = ra6m5_qspi_info,
	.usage = "<base> <size> <chip_width> <bus_width> <target> "
		 "[sector_size] [page_size] [sector_erase_cmd] [page_program_cmd] [status_cmd] [write_enable_cmd] [read_id_cmd]",
};
