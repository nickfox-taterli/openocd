// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Synwit SWM341 internal flash driver                                   *
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

#include "swm341.h"

#define SWM341_TIMEOUT_PROG_MS_FAST   200
#define SWM341_TIMEOUT_PROG_MS_SLOW   2000
#define SWM341_TIMEOUT_ERASE_MS       300
#define SWM341_SLOW_WRITE_CHUNK       0x100u

struct swm341_flash_bank {
	bool probed;
};

static bool swm341_fallback_write_logged;

static int swm341_ensure_halted(struct target *target)
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

static int swm341_wait_program_idle(struct target *target, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint32_t stat;
		int retval = target_read_u32(target, SWM341_FMC_STAT, &stat);
		if (retval != ERROR_OK)
			return retval;

		if ((stat & SWM341_FMC_STAT_PROGBUSY) == 0 && (stat & SWM341_FMC_STAT_IDLE))
			return ERROR_OK;

		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int swm341_write_dap(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t cache;
	uint32_t written = 0;
	int retval;

	if (!IS_ALIGNED(offset, SWM341_FLASH_WRITE_ALIGN) || !IS_ALIGNED(count, SWM341_FLASH_WRITE_ALIGN)) {
		LOG_ERROR("SWM341 DAP write requires %u-byte alignment", SWM341_FLASH_WRITE_ALIGN);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = target_read_u32(target, SWM341_FMC_CACHE, &cache);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SWM341_FMC_CACHE, cache | SWM341_FMC_CACHE_PROGEN);
	if (retval != ERROR_OK)
		return retval;

	while (written < count) {
		uint32_t word = target_buffer_get_u32(target, buffer + written);

		retval = target_write_u32(target, bank->base + offset + written, word);
		if (retval != ERROR_OK)
			goto out_restore;

		retval = swm341_wait_program_idle(target, SWM341_TIMEOUT_PROG_MS_FAST);
		if (retval != ERROR_OK)
			goto out_restore;

		written += SWM341_FLASH_WRITE_ALIGN;
	}

	retval = target_write_u32(target, SWM341_FMC_CACHE,
			(cache | SWM341_FMC_CACHE_CCLR) | SWM341_FMC_CACHE_PROGEN);
	if (retval != ERROR_OK)
		goto out_restore;

out_restore:
	{
		int retval2 = target_write_u32(target, SWM341_FMC_CACHE, cache);
		if (retval == ERROR_OK)
			retval = retval2;
	}

	return retval;
}

static int swm341_run_loader(struct flash_bank *bank,
		struct swm341_loader_cfg *cfg,
		const uint8_t *buffer,
		uint32_t target_addr,
		uint32_t count,
		unsigned int timeout_ms,
		uint32_t write_chunk_limit)
{
	struct target *target = bank->target;
	struct working_area *algo = NULL;
	struct working_area *wa = NULL;
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[5];
	uint32_t cfg_addr;
	uint32_t data_addr;
	uint32_t data_max;
	int retval;

	static const uint8_t swm341_loader_code[] = {
#include "../../../contrib/loaders/flash/swm341/swm341.inc"
	};

	retval = target_alloc_working_area(target, sizeof(swm341_loader_code), &algo);
	if (retval != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, algo->address,
			sizeof(swm341_loader_code), swm341_loader_code);
	if (retval != ERROR_OK)
		goto cleanup;

	if (target_get_working_area_avail(target) <= sizeof(*cfg) + 64) {
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
	data_addr = cfg_addr + sizeof(*cfg);
	data_max = wa->size - sizeof(*cfg);
	if (data_max < SWM341_FLASH_WRITE_ALIGN) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup;
	}

	data_max &= ~(SWM341_FLASH_WRITE_ALIGN - 1);

	retval = target_write_buffer(target, cfg_addr, sizeof(*cfg), (const uint8_t *)cfg);
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
		uint32_t this_size = 0;

		if (cfg->op == SWM341_LOADER_OP_WRITE) {
			uint32_t chunk_cap = data_max;

			if (write_chunk_limit && write_chunk_limit < chunk_cap)
				chunk_cap = write_chunk_limit;

			this_size = count > chunk_cap ? chunk_cap : count;
			this_size &= ~(SWM341_FLASH_WRITE_ALIGN - 1);
			if (this_size == 0) {
				retval = ERROR_FLASH_DST_BREAKS_ALIGNMENT;
				break;
			}

			retval = target_write_buffer(target, data_addr, this_size, buffer);
			if (retval != ERROR_OK)
				break;
		} else {
			this_size = count;
		}

		buf_set_u32(reg_params[0].value, 0, 32, cfg_addr);
		buf_set_u32(reg_params[1].value, 0, 32, data_addr);
		buf_set_u32(reg_params[2].value, 0, 32, target_addr);
		buf_set_u32(reg_params[3].value, 0, 32, this_size);
		buf_set_u32(reg_params[4].value, 0, 32,
				cfg_addr + offsetof(struct swm341_loader_cfg, stack) + SWM341_LOADER_STACK_SIZE);

		retval = target_run_algorithm(target, 0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algo->address, 0,
				timeout_ms,
				&armv7m_info);
		if (retval != ERROR_OK)
			break;

		retval = target_read_buffer(target,
				cfg_addr + offsetof(struct swm341_loader_cfg, last_error),
				sizeof(cfg->last_error), (uint8_t *)&cfg->last_error);
		if (retval != ERROR_OK)
			break;
		if (cfg->last_error != 0) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (cfg->op != SWM341_LOADER_OP_WRITE)
			break;

		target_addr += this_size;
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

static int swm341_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct swm341_flash_bank *info = bank->driver_priv;
	struct swm341_loader_cfg cfg;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (last >= bank->num_sectors || first > last)
		return ERROR_FLASH_SECTOR_INVALID;

	retval = swm341_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	memset(&cfg, 0, sizeof(cfg));
	cfg.op = SWM341_LOADER_OP_ERASE;
	cfg.magic = SWM341_IAP_MAGIC;
	cfg.iap_cache_reset = SWM341_IAP_CACHE_RESET_ADDR;
	cfg.iap_flash_param = SWM341_IAP_FLASH_PARAM_ADDR;
	cfg.iap_flash_erase = SWM341_IAP_FLASH_ERASE_ADDR;
	cfg.iap_flash_write = SWM341_IAP_FLASH_WRITE_ADDR;
	cfg.flash_param_cfg0 = SWM341_IAP_FLASH_PARAM_CFG0_150MHZ;
	cfg.flash_param_cfg1 = SWM341_IAP_FLASH_PARAM_CFG1_150MHZ;
	cfg.cache_reg_addr = SWM341_FMC_CACHE;
	cfg.cache_cclr_mask = SWM341_FMC_CACHE_CCLR;

	for (unsigned int i = first; i <= last; i++) {
		retval = swm341_run_loader(bank, &cfg, NULL, i, i,
				SWM341_TIMEOUT_ERASE_MS, 0);
		if (retval != ERROR_OK)
			return retval;
		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int swm341_write(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct swm341_flash_bank *info = bank->driver_priv;
	struct swm341_loader_cfg cfg;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (count == 0)
		return ERROR_OK;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (!IS_ALIGNED(offset, SWM341_FLASH_WRITE_ALIGN) || !IS_ALIGNED(count, SWM341_FLASH_WRITE_ALIGN)) {
		LOG_ERROR("SWM341 write requires %u-byte alignment", SWM341_FLASH_WRITE_ALIGN);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = swm341_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	memset(&cfg, 0, sizeof(cfg));
	cfg.op = SWM341_LOADER_OP_WRITE;
	cfg.magic = SWM341_IAP_MAGIC;
	cfg.iap_cache_reset = SWM341_IAP_CACHE_RESET_ADDR;
	cfg.iap_flash_param = SWM341_IAP_FLASH_PARAM_ADDR;
	cfg.iap_flash_erase = SWM341_IAP_FLASH_ERASE_ADDR;
	cfg.iap_flash_write = SWM341_IAP_FLASH_WRITE_ADDR;
	cfg.flash_param_cfg0 = SWM341_IAP_FLASH_PARAM_CFG0_150MHZ;
	cfg.flash_param_cfg1 = SWM341_IAP_FLASH_PARAM_CFG1_150MHZ;
	cfg.cache_reg_addr = SWM341_FMC_CACHE;
	cfg.cache_cclr_mask = SWM341_FMC_CACHE_CCLR;

	retval = swm341_run_loader(bank, &cfg, buffer, bank->base + offset, count,
			SWM341_TIMEOUT_PROG_MS_FAST, 0);
	if (retval != ERROR_OK) {
		if (!swm341_fallback_write_logged) {
			LOG_WARNING("SWM341: fallback to slow loader write path");
			swm341_fallback_write_logged = true;
		}
		retval = swm341_run_loader(bank, &cfg, buffer, bank->base + offset, count,
				SWM341_TIMEOUT_PROG_MS_SLOW, SWM341_SLOW_WRITE_CHUNK);
		if (retval != ERROR_OK)
			retval = swm341_write_dap(bank, buffer, offset, count);
	}

	return retval;
}

static int swm341_probe(struct flash_bank *bank)
{
	struct swm341_flash_bank *info = bank->driver_priv;
	int retval;

	if (!info)
		return ERROR_FAIL;

	retval = swm341_ensure_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	bank->base = SWM341_FLASH_BASE;
	if (bank->size == 0)
		bank->size = SWM341_FLASH_SIZE;

	if (!IS_ALIGNED(bank->size, SWM341_FLASH_SECTOR_SIZE)) {
		LOG_ERROR("SWM341 flash size 0x%08" PRIx32 " is not 4K aligned", (uint32_t)bank->size);
		return ERROR_FAIL;
	}

	bank->num_sectors = bank->size / SWM341_FLASH_SECTOR_SIZE;
	free(bank->sectors);
	bank->sectors = alloc_block_array(0, SWM341_FLASH_SECTOR_SIZE, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	bank->write_start_alignment = SWM341_FLASH_WRITE_ALIGN;
	bank->write_end_alignment = SWM341_FLASH_WRITE_ALIGN;

	info->probed = true;
	return ERROR_OK;
}

static int swm341_auto_probe(struct flash_bank *bank)
{
	struct swm341_flash_bank *info = bank->driver_priv;

	if (!info)
		return ERROR_FAIL;
	if (info->probed)
		return ERROR_OK;
	return swm341_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(swm341_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct swm341_flash_bank *info = calloc(1, sizeof(*info));
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

	if (bank->base != SWM341_FLASH_BASE) {
		LOG_WARNING("SWM341 internal flash base adjusted to 0x%08x", SWM341_FLASH_BASE);
		bank->base = SWM341_FLASH_BASE;
	}

	if (bank->size == 0)
		bank->size = SWM341_FLASH_SIZE;

	return ERROR_OK;
}

static int swm341_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	command_print_sameline(cmd,
		"SWM341 internal flash: base=0x%08" TARGET_PRIxADDR " size=0x%08" PRIx32
		" sector=0x%08x align=%u",
		bank->base, (uint32_t)bank->size, SWM341_FLASH_SECTOR_SIZE, SWM341_FLASH_WRITE_ALIGN);
	return ERROR_OK;
}

const struct flash_driver swm341_flash = {
	.name = "swm341",
	.flash_bank_command = swm341_flash_bank_command,
	.erase = swm341_erase,
	.write = swm341_write,
	.read = default_flash_read,
	.probe = swm341_probe,
	.auto_probe = swm341_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = swm341_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
