// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Synwit SWM341 external SFC flash driver                               *
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

#include "swm341_sfc.h"

#define SWM341_SFC_TIMEOUT_CMD_MS       3000
#define SWM341_SFC_TIMEOUT_ERASE_MS     4000

struct swm341_sfc_flash_bank {
	bool probed;
	uint32_t sector_size;
	uint32_t page_size;
	uint8_t cmd_read_jedec;
	uint8_t cmd_read_data;
	uint8_t cmd_read_status;
	uint8_t cmd_write_enable;
	uint8_t cmd_page_program;
	uint8_t cmd_sector_erase;
	uint8_t cmd_chip_erase;
	uint8_t clkdiv;
	uint8_t rdwidth;
	uint8_t ppwidth;
};

static int swm341_sfc_read_status(struct flash_bank *bank, uint8_t *status);
static int swm341_sfc_clear_protection(struct flash_bank *bank);
static int swm341_sfc_write_status(struct flash_bank *bank, uint8_t cmd, uint16_t value);

static int swm341_sfc_ensure_halted(struct target *target)
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

static int swm341_sfc_wait_not_busy(struct target *target, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint32_t sr;
		int retval = target_read_u32(target, SWM341_SFC_SR, &sr);
		if (retval != ERROR_OK)
			return retval;
		if ((sr & SWM341_SFC_SR_BUSY) == 0)
			return ERROR_OK;
		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int swm341_sfc_wait_go_done(struct target *target, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint32_t go;
		int retval = target_read_u32(target, SWM341_SFC_GO, &go);
		if (retval != ERROR_OK)
			return retval;
		if ((go & 1u) == 0)
			return ERROR_OK;
		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int swm341_sfc_hw_init(struct flash_bank *bank)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t clken0;
	uint32_t clken1;
	uint32_t func0;
	uint32_t func1;
	uint32_t inen;
	uint32_t cfg;
	uint32_t cmdahb;
	uint32_t tim;
	int retval;

	retval = target_read_u32(target, SWM341_SYS_CLKEN0, &clken0);
	if (retval != ERROR_OK)
		return retval;
	if ((clken0 & SWM341_SYS_CLKEN0_GPIOD) == 0) {
		retval = target_write_u32(target, SWM341_SYS_CLKEN0, clken0 | SWM341_SYS_CLKEN0_GPIOD);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_read_u32(target, SWM341_SYS_CLKEN1, &clken1);
	if (retval != ERROR_OK)
		return retval;
	if ((clken1 & SWM341_SYS_CLKEN1_SFC) == 0) {
		retval = target_write_u32(target, SWM341_SYS_CLKEN1, clken1 | SWM341_SYS_CLKEN1_SFC);
		if (retval != ERROR_OK)
			return retval;
	}

	/* BSP mapping: PD5=SCLK PD6=SSEL PD8=MOSI PD7=MISO PD3=DATA2 PD4=DATA3 */
	retval = target_read_u32(target, SWM341_PORTD_BASE + SWM341_PORT_FUNC0, &func0);
	if (retval != ERROR_OK)
		return retval;
	func0 &= ~((0xFu << (3 * 4)) | (0xFu << (4 * 4)) | (0xFu << (5 * 4)) |
			(0xFu << (6 * 4)) | (0xFu << (7 * 4)));
	func0 |= (2u << (3 * 4)) | (2u << (4 * 4)) | (1u << (5 * 4)) |
			(1u << (6 * 4)) | (3u << (7 * 4));
	retval = target_write_u32(target, SWM341_PORTD_BASE + SWM341_PORT_FUNC0, func0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, SWM341_PORTD_BASE + SWM341_PORT_FUNC1, &func1);
	if (retval != ERROR_OK)
		return retval;
	func1 &= ~0xFu;
	func1 |= 2u;
	retval = target_write_u32(target, SWM341_PORTD_BASE + SWM341_PORT_FUNC1, func1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, SWM341_PORTD_BASE + SWM341_PORT_INEN, &inen);
	if (retval != ERROR_OK)
		return retval;
	inen &= ~((1u << 5) | (1u << 6));
	inen |= (1u << 3) | (1u << 4) | (1u << 7) | (1u << 8);
	retval = target_write_u32(target, SWM341_PORTD_BASE + SWM341_PORT_INEN, inen);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_UNLOCK, 7);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, SWM341_SFC_CFG, &cfg);
	if (retval != ERROR_OK)
		return retval;

	cfg &= ~(SWM341_SFC_CFG_CLKDIV_MASK |
			SWM341_SFC_CFG_DATA4L_RD_MASK |
			SWM341_SFC_CFG_DATA4L_PP_MASK);
	cfg |= ((uint32_t)(info->clkdiv & 0x3u) << 6) |
			((uint32_t)(info->rdwidth & 0x3u) << 10) |
			((uint32_t)(info->ppwidth & 0x1u) << 9);
	retval = target_write_u32(target, SWM341_SFC_CFG, cfg);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CFG, cfg | SWM341_SFC_CFG_CMDWREN);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, SWM341_SFC_CMDAHB, &cmdahb);
	if (retval != ERROR_OK)
		return retval;
	cmdahb &= ~((0xFFu << 24) | (0xFFu << 16) | (0xFFu << 8) | 0xFFu);
	cmdahb |= ((uint32_t)info->cmd_read_data << 24) |
			((uint32_t)info->cmd_read_status << 16) |
			((uint32_t)info->cmd_page_program << 8) |
			((uint32_t)info->cmd_write_enable << 0);
	retval = target_write_u32(target, SWM341_SFC_CMDAHB, cmdahb);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CFG, cfg & ~SWM341_SFC_CFG_CMDWREN);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, SWM341_SFC_TIM, &tim);
	if (retval != ERROR_OK)
		return retval;
	tim &= ~(SWM341_SFC_TIM_WIP_CHK_ITV_MASK | SWM341_SFC_TIM_WIP_CHK_LMT_MASK);
	tim |= (15u << 0) | (255u << 8);
	retval = target_write_u32(target, SWM341_SFC_TIM, tim);
	if (retval != ERROR_OK)
		return retval;

	{
		uint8_t status1;
		retval = swm341_sfc_read_status(bank, &status1);
		if (retval != ERROR_OK)
			return retval;
		if (status1 & 0x7Cu) {
			retval = swm341_sfc_clear_protection(bank);
			if (retval != ERROR_OK)
				LOG_WARNING("SWM341 SFC clear protection failed");
		}
	}

	return swm341_sfc_wait_not_busy(target, 100);
}

static int swm341_sfc_run_simple_cmd(struct target *target,
		uint8_t cmd, uint8_t cmd_type, uint32_t addr, uint32_t *data)
{
	uint32_t cfg;
	int retval;

	retval = target_read_u32(target, SWM341_SFC_CFG, &cfg);
	if (retval != ERROR_OK)
		return retval;

	cfg &= ~SWM341_SFC_CFG_CMDTYPE_MASK;
	cfg |= SWM341_SFC_CFG_CMDWREN | ((uint32_t)(cmd_type & 0xFu) << 0);
	retval = target_write_u32(target, SWM341_SFC_CFG, cfg);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_ADDR, addr);
	if (retval != ERROR_OK)
		return retval;

	if (data) {
		retval = target_write_u32(target, SWM341_SFC_DATA, *data);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u32(target, SWM341_SFC_CMD, cmd);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_GO, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_wait_go_done(target, SWM341_SFC_TIMEOUT_CMD_MS);
	if (retval != ERROR_OK)
		return retval;

	if (data)
		return target_read_u32(target, SWM341_SFC_DATA, data);

	return ERROR_OK;
}

static int swm341_sfc_read_status(struct flash_bank *bank, uint8_t *status)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	uint32_t value = 0;
	int retval = swm341_sfc_run_simple_cmd(bank->target, info->cmd_read_status, 1, 0, &value);
	if (retval != ERROR_OK)
		return retval;

	*status = (uint8_t)value;
	return ERROR_OK;
}

static int swm341_sfc_wait_flash_ready(struct flash_bank *bank, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint8_t status = 0;
		int retval = swm341_sfc_read_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & BIT(0)) == 0)
			return ERROR_OK;
		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int swm341_sfc_clear_protection(struct flash_bank *bank)
{
	uint8_t sr1 = 0;
	uint8_t sr2 = 0;
	uint16_t wr = 0;
	int retval = swm341_sfc_read_status(bank, &sr1);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_run_simple_cmd(bank->target,
			SWM341_SFC_CMD_READ_STATUS2, 1, 0, (uint32_t *)&sr2);
	if (retval != ERROR_OK)
		return retval;

	wr = ((uint16_t)sr2 << 8) | sr1;
	wr &= ~0xFC7Cu; /* clear SRP/BP/TB/SEC/QE related protection bits */

	retval = swm341_sfc_write_status(bank, SWM341_SFC_CMD_WRITE_STATUS1, wr);
	if (retval != ERROR_OK)
		return retval;

	return swm341_sfc_wait_flash_ready(bank, SWM341_SFC_TIMEOUT_CMD_MS);
}

static int swm341_sfc_write_status(struct flash_bank *bank, uint8_t cmd, uint16_t value)
{
	struct target *target = bank->target;
	uint32_t cfg;
	uint32_t data = value;
	int retval;

	retval = target_read_u32(target, SWM341_SFC_CFG, &cfg);
	if (retval != ERROR_OK)
		return retval;

	cfg &= ~SWM341_SFC_CFG_CMDTYPE_MASK;
	cfg |= SWM341_SFC_CFG_WREN | SWM341_SFC_CFG_CMDWREN | (6u << 0);
	retval = target_write_u32(target, SWM341_SFC_CFG, cfg);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CMD, cmd);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_DATA, data);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_GO, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_wait_go_done(target, SWM341_SFC_TIMEOUT_CMD_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CFG, cfg & ~SWM341_SFC_CFG_WREN);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int swm341_sfc_read_jedec(struct flash_bank *bank, uint32_t *jedec)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	uint32_t value = 0;
	int retval = swm341_sfc_run_simple_cmd(bank->target, info->cmd_read_jedec, 2, 0, &value);
	if (retval != ERROR_OK)
		return retval;

	*jedec = value;
	return ERROR_OK;
}

static int swm341_sfc_erase_one(struct flash_bank *bank, uint32_t addr_off)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t cfg;
	int retval;

	retval = target_read_u32(target, SWM341_SFC_CFG, &cfg);
	if (retval != ERROR_OK)
		return retval;

	cfg &= ~SWM341_SFC_CFG_CMDTYPE_MASK;
	cfg |= SWM341_SFC_CFG_WREN | SWM341_SFC_CFG_CMDWREN | (7u << 0);
	retval = target_write_u32(target, SWM341_SFC_CFG, cfg);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_ADDR, addr_off);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CMD, info->cmd_sector_erase);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_GO, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_wait_go_done(target, SWM341_SFC_TIMEOUT_CMD_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CFG, cfg & ~SWM341_SFC_CFG_WREN);
	if (retval != ERROR_OK)
		return retval;

	return swm341_sfc_wait_flash_ready(bank, SWM341_SFC_TIMEOUT_ERASE_MS);
}

static int swm341_sfc_mass_erase(struct flash_bank *bank)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t cfg;
	int retval;

	retval = target_read_u32(target, SWM341_SFC_CFG, &cfg);
	if (retval != ERROR_OK)
		return retval;

	cfg &= ~SWM341_SFC_CFG_CMDTYPE_MASK;
	cfg |= SWM341_SFC_CFG_WREN | SWM341_SFC_CFG_CMDWREN | (5u << 0);
	retval = target_write_u32(target, SWM341_SFC_CFG, cfg);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_ADDR, 0xFFFFFFFFu);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CMD, info->cmd_chip_erase);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_GO, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_wait_go_done(target, SWM341_SFC_TIMEOUT_CMD_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_SFC_CFG, cfg & ~SWM341_SFC_CFG_WREN);
	if (retval != ERROR_OK)
		return retval;

	return swm341_sfc_wait_flash_ready(bank, 60000);
}

static int swm341_sfc_loader_write(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t mapped_addr, uint32_t count)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *algo = NULL;
	struct working_area *wa = NULL;
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[5];
	struct swm341_sfc_loader_cfg cfg;
	uint32_t cfg_addr;
	uint32_t data_addr;
	uint32_t data_max;
	int retval;

	static const uint8_t swm341_sfc_loader_code[] = {
#include "../../../contrib/loaders/flash/swm341/swm341_sfc.inc"
	};

	retval = target_alloc_working_area(target, sizeof(swm341_sfc_loader_code), &algo);
	if (retval != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, algo->address,
			sizeof(swm341_sfc_loader_code), swm341_sfc_loader_code);
	if (retval != ERROR_OK)
		goto cleanup;

	if (target_get_working_area_avail(target) <= sizeof(cfg) + 64) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	retval = target_alloc_working_area_try(target,
			target_get_working_area_avail(target), &wa);
	if (retval != ERROR_OK) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	cfg_addr = wa->address;
	data_addr = cfg_addr + sizeof(cfg);
	data_max = wa->size - sizeof(cfg);
	if (data_max < 4) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}
	data_max &= ~3u;

	cfg.sfc_cfg_addr = SWM341_SFC_CFG;
	cfg.sfc_sr_addr = SWM341_SFC_SR;
	cfg.map_base = SWM341_SFC_MAP_BASE;
	cfg.page_size = info->page_size;
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
		uint32_t this_size = count > data_max ? data_max : count;
		this_size &= ~3u;
		if (this_size == 0) {
			retval = ERROR_FLASH_DST_BREAKS_ALIGNMENT;
			break;
		}

		retval = target_write_buffer(target, data_addr, this_size, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, cfg_addr);
		buf_set_u32(reg_params[1].value, 0, 32, data_addr);
		buf_set_u32(reg_params[2].value, 0, 32, mapped_addr);
		buf_set_u32(reg_params[3].value, 0, 32, this_size);
		buf_set_u32(reg_params[4].value, 0, 32,
				cfg_addr + offsetof(struct swm341_sfc_loader_cfg, stack) + SWM341_SFC_LOADER_STACK_SIZE);

		retval = target_run_algorithm(target, 0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algo->address, 0,
				SWM341_SFC_TIMEOUT_CMD_MS,
				&armv7m_info);
		if (retval != ERROR_OK)
			break;

		retval = target_read_buffer(target,
				cfg_addr + offsetof(struct swm341_sfc_loader_cfg, last_error),
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

cleanup:
	if (wa)
		target_free_working_area(target, wa);
	if (algo)
		target_free_working_area(target, algo);

	return retval;
}

static int swm341_sfc_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	return target_read_buffer(bank->target, bank->base + offset, count, buffer);
}

static int swm341_sfc_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (last >= bank->num_sectors || first > last)
		return ERROR_FLASH_SECTOR_INVALID;

	retval = swm341_sfc_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_hw_init(bank);
	if (retval != ERROR_OK)
		return retval;

	if (first == 0 && last == bank->num_sectors - 1) {
		retval = swm341_sfc_mass_erase(bank);
		if (retval == ERROR_OK) {
			for (unsigned int i = first; i <= last; i++)
				bank->sectors[i].is_erased = 1;
		}
		return retval;
	}

	for (unsigned int i = first; i <= last; i++) {
		retval = swm341_sfc_erase_one(bank, bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;
		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int swm341_sfc_write(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (count == 0)
		return ERROR_OK;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (!IS_ALIGNED(offset, 4) || !IS_ALIGNED(count, 4)) {
		LOG_ERROR("SWM341 SFC write requires 4-byte alignment");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = swm341_sfc_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_hw_init(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_loader_write(bank, buffer, bank->base + offset, count);
	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		LOG_ERROR("SWM341 SFC loader mode requires a target working area");

	return retval;
}

static int swm341_sfc_probe(struct flash_bank *bank)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;
	uint32_t jedec;
	uint8_t status1 = 0;
	uint8_t capacity_code;
	uint32_t detected_size = 0;
	int retval;

	if (!info)
		return ERROR_FAIL;

	retval = swm341_sfc_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	retval = swm341_sfc_hw_init(bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->size == 0)
		bank->size = SWM341_SFC_DEFAULT_SIZE;

	if (!IS_ALIGNED(bank->size, info->sector_size)) {
		LOG_ERROR("SWM341 SFC size 0x%08" PRIx32 " is not sector aligned", (uint32_t)bank->size);
		return ERROR_FAIL;
	}

	retval = swm341_sfc_read_jedec(bank, &jedec);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("SWM341 SFC JEDEC ID: %02" PRIx8 " %02" PRIx8 " %02" PRIx8,
			(uint8_t)(jedec >> 0), (uint8_t)(jedec >> 8), (uint8_t)(jedec >> 16));
	if (swm341_sfc_read_status(bank, &status1) == ERROR_OK)
		LOG_INFO("SWM341 SFC status1: 0x%02" PRIx8, status1);

	capacity_code = (uint8_t)(jedec >> 16);
	if (capacity_code >= 0x14u && capacity_code <= 0x1Fu)
		detected_size = 1u << capacity_code;

	if (detected_size != 0) {
		if (bank->size == 0 || bank->size == SWM341_SFC_DEFAULT_SIZE || bank->size > detected_size) {
			LOG_INFO("SWM341 SFC detected flash size: 0x%08" PRIx32, detected_size);
			bank->size = detected_size;
		}
	}

	free(bank->sectors);
	bank->num_sectors = bank->size / info->sector_size;
	bank->sectors = alloc_block_array(0, info->sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	bank->write_start_alignment = 4;
	bank->write_end_alignment = 4;

	info->probed = true;
	return ERROR_OK;
}

static int swm341_sfc_auto_probe(struct flash_bank *bank)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;

	if (!info)
		return ERROR_FAIL;
	if (info->probed)
		return ERROR_OK;
	return swm341_sfc_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(swm341_sfc_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct swm341_sfc_flash_bank *info = calloc(1, sizeof(*info));
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

	info->sector_size = SWM341_SFC_DEFAULT_SECTOR_SIZE;
	info->page_size = SWM341_SFC_DEFAULT_PAGE_SIZE;
	info->cmd_read_jedec = SWM341_SFC_CMD_READ_JEDEC;
	info->cmd_read_data = 0x03u;
	info->cmd_read_status = SWM341_SFC_CMD_READ_STATUS1;
	info->cmd_write_enable = SWM341_SFC_CMD_WREN;
	info->cmd_page_program = SWM341_SFC_CMD_PAGE_PROGRAM;
	info->cmd_sector_erase = SWM341_SFC_CMD_SECTOR_ERASE;
	info->cmd_chip_erase = SWM341_SFC_CMD_CHIP_ERASE;
	info->clkdiv = 2;
	info->rdwidth = 0;
	info->ppwidth = 0;

	if (CMD_ARGC > 6)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->sector_size);
	if (CMD_ARGC > 7)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], info->page_size);
	if (CMD_ARGC > 8)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[8], info->cmd_sector_erase);
	if (CMD_ARGC > 9)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[9], info->cmd_page_program);
	if (CMD_ARGC > 10)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[10], info->cmd_read_status);
	if (CMD_ARGC > 11)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[11], info->cmd_write_enable);
	if (CMD_ARGC > 12)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[12], info->cmd_read_jedec);
	if (CMD_ARGC > 13)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[13], info->cmd_chip_erase);

	if (bank->base != SWM341_SFC_MAP_BASE) {
		LOG_WARNING("SWM341 SFC base adjusted to 0x%08x", SWM341_SFC_MAP_BASE);
		bank->base = SWM341_SFC_MAP_BASE;
	}

	if (bank->size == 0)
		bank->size = SWM341_SFC_DEFAULT_SIZE;

	if (!IS_PWR_OF_2(info->page_size) || !IS_PWR_OF_2(info->sector_size) ||
			(info->page_size > info->sector_size)) {
		LOG_ERROR("invalid SWM341 SFC geometry: page=0x%08" PRIx32 ", sector=0x%08" PRIx32,
				info->page_size, info->sector_size);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int swm341_sfc_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct swm341_sfc_flash_bank *info = bank->driver_priv;

	if (!info)
		return ERROR_FAIL;

	command_print_sameline(cmd,
		"SWM341 SFC flash: base=0x%08" TARGET_PRIxADDR " size=0x%08" PRIx32
		" sector=0x%08" PRIx32 " page=0x%08" PRIx32,
		bank->base, (uint32_t)bank->size, info->sector_size, info->page_size);
	return ERROR_OK;
}

const struct flash_driver swm341_sfc_flash = {
	.name = "swm341sfc",
	.flash_bank_command = swm341_sfc_flash_bank_command,
	.erase = swm341_sfc_erase,
	.write = swm341_sfc_write,
	.read = swm341_sfc_read,
	.probe = swm341_sfc_probe,
	.auto_probe = swm341_sfc_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = swm341_sfc_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
