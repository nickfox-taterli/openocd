// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Renesas Synergy S7G2 internal code flash driver                        *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

#include <helper/align.h>
#include <helper/time_support.h>

#include "renesas_s7g2.h"

#define S7G2_TIMEOUT_PROG_MS        100
#define S7G2_TIMEOUT_ERASE_SMALL_MS 1000
#define S7G2_TIMEOUT_ERASE_LARGE_MS 4000
#define S7G2_TIMEOUT_CMD_MS         100

struct s7g2_flash_bank {
	bool probed;
	uint32_t fclk_mhz;
	bool cache_saved;
	uint16_t saved_fcachee;
};

static int s7g2_wait_frdy(struct target *target, unsigned int timeout_ms)
{
	int64_t end = timeval_ms() + timeout_ms;

	while (timeval_ms() < end) {
		uint32_t fstatr;
		int retval = target_read_u32(target, S7G2_REG_FSTATR, &fstatr);
		if (retval != ERROR_OK)
			return retval;

		if (fstatr & S7G2_FSTATR_FRDY)
			return ERROR_OK;
	}

	return ERROR_TIMEOUT_REACHED;
}

static int s7g2_wait_dbfull_clear(struct target *target)
{
	int64_t end = timeval_ms() + S7G2_TIMEOUT_CMD_MS;

	while (timeval_ms() < end) {
		uint32_t fstatr;
		int retval = target_read_u32(target, S7G2_REG_FSTATR, &fstatr);
		if (retval != ERROR_OK)
			return retval;

		if ((fstatr & S7G2_FSTATR_DBFULL) == 0)
			return ERROR_OK;
	}

	return ERROR_TIMEOUT_REACHED;
}

static int s7g2_status_clear(struct target *target)
{
	int retval = target_write_u8(target, S7G2_FACI_CMD_AREA, S7G2_FACI_CMD_STATUS_CLEAR);
	if (retval != ERROR_OK)
		return retval;

	return s7g2_wait_frdy(target, S7G2_TIMEOUT_CMD_MS);
}

static int s7g2_forced_stop(struct target *target)
{
	int retval = target_write_u8(target, S7G2_FACI_CMD_AREA, S7G2_FACI_CMD_FORCED_STOP);
	if (retval != ERROR_OK)
		return retval;

	return s7g2_wait_frdy(target, S7G2_TIMEOUT_CMD_MS);
}

static int s7g2_check_errors(struct target *target, const char *op)
{
	uint32_t fstatr;
	uint16_t fcmdr;
	uint16_t fpestat;
	uint8_t fastat;
	int retval = target_read_u32(target, S7G2_REG_FSTATR, &fstatr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(target, S7G2_REG_FASTAT, &fastat);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u16(target, S7G2_REG_FCMDR, &fcmdr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u16(target, S7G2_REG_FPESTAT, &fpestat);
	if (retval != ERROR_OK)
		return retval;

	if (((fstatr & S7G2_FSTATR_ERR_MASK) == 0) && ((fastat & S7G2_FASTAT_CMDLK) == 0))
		return ERROR_OK;

	LOG_ERROR("S7G2 flash %s error: FSTATR=0x%08" PRIx32 ", FASTAT=0x%02" PRIx8
			", FCMDR=0x%04" PRIx16 ", FPESTAT=0x%04" PRIx16,
			op, fstatr, fastat, fcmdr, fpestat);

	(void)s7g2_status_clear(target);
	return ERROR_FLASH_OPERATION_FAILED;
}

static int s7g2_enter_pe(struct flash_bank *bank)
{
	struct s7g2_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	retval = s7g2_wait_frdy(target, S7G2_TIMEOUT_CMD_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u16(target, S7G2_REG_FCACHEE, &info->saved_fcachee);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u16(target, S7G2_REG_FCACHEE, 0);
	if (retval != ERROR_OK)
		return retval;
	info->cache_saved = true;

	retval = target_write_u8(target, S7G2_REG_FWEPROR, S7G2_FWEPROR_ENABLE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u16(target, S7G2_REG_FPCKAR, S7G2_FPCKAR_KEY | info->fclk_mhz);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u16(target, S7G2_REG_FENTRYR, S7G2_FENTRYR_CODE_PE);
	if (retval != ERROR_OK)
		return retval;

	int64_t end = timeval_ms() + S7G2_TIMEOUT_CMD_MS;
	while (timeval_ms() < end) {
		uint16_t fentryr;
		retval = target_read_u16(target, S7G2_REG_FENTRYR, &fentryr);
		if (retval != ERROR_OK)
			return retval;

		if (fentryr == (S7G2_FENTRYR_CODE_PE & 0x00ff))
			return ERROR_OK;
	}

	return ERROR_TIMEOUT_REACHED;
}

static int s7g2_exit_pe(struct flash_bank *bank)
{
	struct s7g2_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval = s7g2_wait_frdy(target, S7G2_TIMEOUT_CMD_MS);
	if (retval != ERROR_OK)
		return retval;

	uint8_t fastat;
	retval = target_read_u8(target, S7G2_REG_FASTAT, &fastat);
	if (retval != ERROR_OK)
		return retval;

	if (fastat & S7G2_FASTAT_CMDLK) {
		retval = s7g2_status_clear(target);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u16(target, S7G2_REG_FENTRYR, S7G2_FENTRYR_READ);
	if (retval != ERROR_OK)
		return retval;

	int64_t end = timeval_ms() + S7G2_TIMEOUT_CMD_MS;
	while (timeval_ms() < end) {
		uint16_t fentryr;
		retval = target_read_u16(target, S7G2_REG_FENTRYR, &fentryr);
		if (retval != ERROR_OK)
			return retval;

		if (fentryr == 0) {
			retval = target_write_u8(target, S7G2_REG_FWEPROR, S7G2_FWEPROR_DISABLE);
			if (retval != ERROR_OK)
				return retval;

			if (info->cache_saved) {
				retval = target_write_u16(target, S7G2_REG_FCACHEE, info->saved_fcachee);
				if (retval != ERROR_OK)
					return retval;
				info->cache_saved = false;
			}
			return ERROR_OK;
		}
	}

	return ERROR_TIMEOUT_REACHED;
}

static int s7g2_program_one_unit(struct flash_bank *bank, uint32_t addr, const uint8_t *buffer)
{
	struct target *target = bank->target;
	int retval = target_write_u32(target, S7G2_REG_FSADDR, addr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, S7G2_FACI_CMD_AREA, S7G2_FACI_CMD_PROGRAM);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, S7G2_FACI_CMD_AREA, S7G2_FACI_CMD_PROGRAM_CF);
	if (retval != ERROR_OK)
		return retval;

	for (uint32_t i = 0; i < S7G2_CODE_PROGRAM_UNIT; i += 2) {
		uint16_t v = (uint16_t)buffer[i] | ((uint16_t)buffer[i + 1] << 8);
		retval = target_write_u16(target, S7G2_FACI_CMD_AREA, v);
		if (retval != ERROR_OK)
			return retval;

		retval = s7g2_wait_dbfull_clear(target);
		if (retval != ERROR_OK) {
			(void)s7g2_forced_stop(target);
			return retval;
		}
	}

	retval = target_write_u8(target, S7G2_FACI_CMD_AREA, S7G2_FACI_CMD_FINAL);
	if (retval != ERROR_OK)
		return retval;

	retval = s7g2_wait_frdy(target, S7G2_TIMEOUT_PROG_MS);
	if (retval != ERROR_OK) {
		(void)s7g2_forced_stop(target);
		return retval;
	}

	return s7g2_check_errors(target, "program");
}

static int s7g2_erase_one_sector(struct flash_bank *bank, unsigned int sector)
{
	struct target *target = bank->target;
	uint32_t addr = bank->base + bank->sectors[sector].offset;
	int retval;

	retval = target_write_u16(target, S7G2_REG_FCPSR, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, S7G2_REG_FSADDR, addr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, S7G2_FACI_CMD_AREA, S7G2_FACI_CMD_BLOCK_ERASE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, S7G2_FACI_CMD_AREA, S7G2_FACI_CMD_FINAL);
	if (retval != ERROR_OK)
		return retval;

	unsigned int timeout = bank->sectors[sector].size > S7G2_CODE_ERASE_SMALL ?
		S7G2_TIMEOUT_ERASE_LARGE_MS : S7G2_TIMEOUT_ERASE_SMALL_MS;

	retval = s7g2_wait_frdy(target, timeout);
	if (retval != ERROR_OK) {
		(void)s7g2_forced_stop(target);
		return retval;
	}

	retval = s7g2_check_errors(target, "erase");
	if (retval == ERROR_OK)
		bank->sectors[sector].is_erased = 1;

	return retval;
}

static int s7g2_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	return target_read_buffer(bank->target, bank->base + offset, count, buffer);
}

static int s7g2_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct s7g2_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (!info || !info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (last >= bank->num_sectors || first > last)
		return ERROR_FLASH_SECTOR_INVALID;

	retval = s7g2_enter_pe(bank);
	if (retval != ERROR_OK)
		goto done;

	for (unsigned int i = first; i <= last; i++) {
		retval = s7g2_erase_one_sector(bank, i);
		if (retval != ERROR_OK)
			break;
	}

done:
	{
		int retval2 = s7g2_exit_pe(bank);
		if (retval == ERROR_OK)
			retval = retval2;
	}

	return retval;
}

static int s7g2_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct s7g2_flash_bank *info = bank->driver_priv;
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

	if (!IS_ALIGNED(offset, S7G2_CODE_PROGRAM_UNIT))
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;

	if (!IS_ALIGNED(count, S7G2_CODE_PROGRAM_UNIT)) {
		aligned_count = ALIGN_UP(count, S7G2_CODE_PROGRAM_UNIT);
		if (offset + aligned_count > bank->size)
			return ERROR_FLASH_DST_OUT_OF_BANK;

		aligned_buffer = malloc(aligned_count);
		if (!aligned_buffer)
			return ERROR_FAIL;

		memset(aligned_buffer, 0xff, aligned_count);
		memcpy(aligned_buffer, buffer, count);
		buffer = aligned_buffer;
	}

	retval = s7g2_enter_pe(bank);
	if (retval != ERROR_OK)
		goto out;

	for (uint32_t done = 0; done < aligned_count; done += S7G2_CODE_PROGRAM_UNIT) {
		retval = s7g2_program_one_unit(bank, bank->base + offset + done, buffer + done);
		if (retval != ERROR_OK)
			break;
	}

	{
		int retval2 = s7g2_exit_pe(bank);
		if (retval == ERROR_OK)
			retval = retval2;
	}

out:
	free(aligned_buffer);
	return retval;
}

static int s7g2_build_sectors(struct flash_bank *bank)
{
	uint32_t offset = 0;
	uint32_t small_area = bank->size < S7G2_CODE_SMALL_AREA_SIZE ?
		bank->size : S7G2_CODE_SMALL_AREA_SIZE;
	unsigned int num_small = small_area / S7G2_CODE_ERASE_SMALL;
	unsigned int num_large = 0;

	if (small_area % S7G2_CODE_ERASE_SMALL)
		return ERROR_FAIL;

	if (bank->size > small_area) {
		uint32_t large_area = bank->size - small_area;
		num_large = large_area / S7G2_CODE_ERASE_LARGE;
		if (large_area % S7G2_CODE_ERASE_LARGE)
			return ERROR_FAIL;
	}

	bank->num_sectors = num_small + num_large;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < num_small; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = S7G2_CODE_ERASE_SMALL;
		offset += S7G2_CODE_ERASE_SMALL;
	}

	for (unsigned int i = 0; i < num_large; i++) {
		bank->sectors[num_small + i].offset = offset;
		bank->sectors[num_small + i].size = S7G2_CODE_ERASE_LARGE;
		offset += S7G2_CODE_ERASE_LARGE;
	}

	return ERROR_OK;
}

static int s7g2_probe(struct flash_bank *bank)
{
	struct s7g2_flash_bank *info = bank->driver_priv;

	if (!info)
		return ERROR_FAIL;

	if (bank->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (info->probed)
		return ERROR_OK;

	int retval = s7g2_build_sectors(bank);
	if (retval != ERROR_OK)
		return retval;

	info->probed = true;
	return ERROR_OK;
}

static int s7g2_auto_probe(struct flash_bank *bank)
{
	struct s7g2_flash_bank *info = bank->driver_priv;
	if (!info)
		return ERROR_FAIL;
	if (info->probed)
		return ERROR_OK;
	return s7g2_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(s7g2_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct s7g2_flash_bank *info = calloc(1, sizeof(*info));
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

	if (bank->base != S7G2_CODE_BASE) {
		LOG_ERROR("S7G2 driver currently supports code flash at 0x%08x only", S7G2_CODE_BASE);
		free(info);
		bank->driver_priv = NULL;
		return ERROR_FAIL;
	}

	if (bank->size == 0)
		bank->size = S7G2_CODE_SIZE_4M;

	if (bank->size != S7G2_CODE_SIZE_4M && bank->size != S7G2_CODE_SIZE_3M) {
		LOG_ERROR("S7G2 code flash size must be 0x%08x or 0x%08x", S7G2_CODE_SIZE_4M, S7G2_CODE_SIZE_3M);
		free(info);
		bank->driver_priv = NULL;
		return ERROR_FAIL;
	}

	info->fclk_mhz = 60;
	if (CMD_ARGC >= 7) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->fclk_mhz);
		if (info->fclk_mhz < 4 || info->fclk_mhz > 60) {
			LOG_ERROR("S7G2 fclk_mhz must be in the range 4..60");
			free(info);
			bank->driver_priv = NULL;
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static const struct command_registration s7g2_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver s7g2_flash = {
	.name = "s7g2",
	.commands = s7g2_exec_command_handlers,
	.flash_bank_command = s7g2_flash_bank_command,
	.erase = s7g2_erase,
	.protect = NULL,
	.write = s7g2_write,
	.read = s7g2_read,
	.probe = s7g2_probe,
	.auto_probe = s7g2_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
	.info = NULL,
};
