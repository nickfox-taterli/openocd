// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"

#include <helper/binarybuffer.h>
#include <target/register.h>

#include "../../../contrib/loaders/flash/imxrt_firert_stub/protocol.h"
#include "imxrt_firert_stub_bin.inc"

#define FIRET_STUB_LOAD_ADDR 0x20202000u
#define FIRET_STUB_STACK_PTR 0x20205ff0u
#define FIRET_STUB_DATA_ADDR 0x20206000u
#define FIRET_STUB_DATA_SIZE 0x00010000u

struct firert_flash_bank {
	struct target *target;
	const struct flash_device *dev;
	uint32_t jedec_raw;
	uint32_t jedec_norm;
	bool probed;
};

static uint32_t firert_jedec_raw_to_device_id(uint32_t raw)
{
	return ((raw & 0x000000ffu) << 16) |
		(raw & 0x0000ff00u) |
		((raw & 0x00ff0000u) >> 16);
}

static int firert_set_reg_u32(struct target *target, const char *name, uint32_t value)
{
	struct reg *reg = register_get_by_name(target->reg_cache, name, true);

	if (!reg)
		return ERROR_FAIL;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = true;
	reg->valid = true;
	return ERROR_OK;
}

static int firert_ensure_halted(struct target *target)
{
	int retval;

	if (target->state == TARGET_HALTED)
		return ERROR_OK;

	retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	return target_wait_state(target, TARGET_HALTED, 1000);
}

static int firert_read_mailbox(struct target *target, struct firert_mailbox *mb)
{
	return target_read_buffer(target, FIRET_MB_ADDR, sizeof(*mb), (uint8_t *)mb);
}

static int firert_resume_wait_halt(struct target *target, int timeout_ms)
{
	int retval;
	struct reg *pc_reg;
	uint32_t pc;
	uint16_t insn = 0;

	retval = firert_ensure_halted(target);
	if (retval != ERROR_OK)
		return retval;

	pc_reg = register_get_by_name(target->reg_cache, "pc", true);
	if (!pc_reg)
		return ERROR_FAIL;

	pc = buf_get_u32(pc_reg->value, 0, 32);
	retval = target_read_u16(target, pc, &insn);
	if (retval == ERROR_OK && insn == 0xbeab) {
		retval = firert_set_reg_u32(target, "pc", pc + 2u);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_resume(target, 1, 0, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	if (retval != ERROR_OK) {
		target_halt(target);
		target_wait_state(target, TARGET_HALTED, 1000);
	}

	return retval;
}

static int firert_stub_load_and_boot(struct flash_bank *bank, struct firert_mailbox *mb)
{
	struct target *target = bank->target;
	int retval;

	retval = firert_ensure_halted(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_buffer(target, FIRET_STUB_LOAD_ADDR,
			imxrt_firert_stub_bin_len, imxrt_firert_stub_bin);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_set_reg_u32(target, "sp", FIRET_STUB_STACK_PTR);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_set_reg_u32(target, "pc", FIRET_STUB_LOAD_ADDR | 1u);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_set_reg_u32(target, "xpsr", 0x01000000u);
	if (retval != ERROR_OK)
		return retval;

	retval = target_resume(target, 1, 0, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_wait_state(target, TARGET_HALTED, 1000);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_read_mailbox(target, mb);
	if (retval != ERROR_OK)
		return retval;

	if (mb->magic != FIRET_MB_MAGIC) {
		LOG_ERROR("i.MXRT stub mailbox magic mismatch: 0x%08" PRIx32, mb->magic);
		return ERROR_FAIL;
	}

	LOG_INFO("imxrt stub boot: status=0x%08" PRIx32 " result=0x%08" PRIx32
		" detail0=0x%08" PRIx32 " detail1=0x%08" PRIx32,
		mb->status, mb->result, mb->detail0, mb->detail1);
	return ERROR_OK;
}

static int firert_stub_go(struct flash_bank *bank, uint32_t cmd, uint32_t addr,
		uint32_t size, uint32_t arg, uint32_t src, int timeout_ms,
		struct firert_mailbox *mb)
{
	struct target *target = bank->target;
	int retval;

	retval = firert_ensure_halted(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FIRET_MB_ADDR + 0x08, addr);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FIRET_MB_ADDR + 0x0c, size);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FIRET_MB_ADDR + 0x10, arg);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FIRET_MB_ADDR + 0x14, src);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FIRET_MB_ADDR + 0x04, cmd);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_resume_wait_halt(target, timeout_ms);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_read_mailbox(target, mb);
	if (retval != ERROR_OK)
		return retval;

	if (mb->status != FIRET_ST_DONE) {
		LOG_ERROR("i.MXRT stub cmd %" PRIu32 " failed: status=0x%08" PRIx32
			" detail0=0x%08" PRIx32 " detail1=0x%08" PRIx32,
			cmd, mb->status, mb->detail0, mb->detail1);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(firert_flash_bank_command)
{
	struct firert_flash_bank *info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	info = calloc(1, sizeof(*info));
	if (!info)
		return ERROR_FAIL;

	info->target = get_target(CMD_ARGV[5]);
	if (!info->target) {
		free(info);
		LOG_ERROR("target '%s' not defined", CMD_ARGV[5]);
		return ERROR_FAIL;
	}

	bank->driver_priv = info;
	return ERROR_OK;
}

static int firert_probe(struct flash_bank *bank)
{
	struct firert_flash_bank *info = bank->driver_priv;
	struct firert_mailbox mb;
	uint32_t fallback_size = 0x2000000u;
	int retval;

	retval = firert_stub_load_and_boot(bank, &mb);
	if (retval != ERROR_OK)
		return retval;

	info->jedec_raw = mb.result;
	info->jedec_norm = firert_jedec_raw_to_device_id(info->jedec_raw);

	info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name; p++) {
		if (p->device_id == info->jedec_raw || p->device_id == info->jedec_norm) {
			info->dev = p;
			break;
		}
	}

	if (info->dev) {
		bank->size = info->dev->size_in_bytes;
		LOG_INFO("Found flash device '%s' raw JEDEC 0x%08" PRIx32
			" normalized 0x%08" PRIx32,
			info->dev->name, info->jedec_raw, info->jedec_norm);
	} else {
		bank->size = fallback_size;
		LOG_WARNING("Unknown flash raw JEDEC 0x%08" PRIx32
			" normalized 0x%08" PRIx32 ", using size 0x%08" PRIx32,
			info->jedec_raw, info->jedec_norm, bank->size);
	}

	free(bank->sectors);
	bank->num_sectors = bank->size / 0x1000u;
	bank->sectors = alloc_block_array(0, 0x1000u, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	info->probed = true;
	return ERROR_OK;
}

static int firert_auto_probe(struct flash_bank *bank)
{
	struct firert_flash_bank *info = bank->driver_priv;
	return info->probed ? ERROR_OK : firert_probe(bank);
}

static int firert_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct firert_mailbox mb;
	uint32_t off;
	uint32_t end;
	int retval;

	retval = firert_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_stub_load_and_boot(bank, &mb);
	if (retval != ERROR_OK)
		return retval;

	if (first > last || last >= bank->num_sectors)
		return ERROR_FLASH_SECTOR_INVALID;

	if (first == 0 && last == bank->num_sectors - 1) {
		retval = firert_stub_go(bank, FIRET_CMD_ERASE, 0, 0,
			FIRET_ERASE_CHIP, 0, 180000, &mb);
		if (retval != ERROR_OK)
			return retval;
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;
		return ERROR_OK;
	}

	off = bank->sectors[first].offset;
	end = bank->sectors[last].offset + bank->sectors[last].size;

	while (off < end) {
		uint32_t kind;
		uint32_t step;

		if ((off % 0x10000u) == 0u && (end - off) >= 0x10000u) {
			kind = FIRET_ERASE_64K;
			step = 0x10000u;
		} else if ((off % 0x8000u) == 0u && (end - off) >= 0x8000u) {
			kind = FIRET_ERASE_32K;
			step = 0x8000u;
		} else {
			kind = FIRET_ERASE_4K;
			step = 0x1000u;
		}

		retval = firert_stub_go(bank, FIRET_CMD_ERASE, off, 0, kind, 0, 30000, &mb);
		if (retval != ERROR_OK)
			return retval;

		for (unsigned int i = first; i <= last; i++) {
			uint32_t so = bank->sectors[i].offset;
			if (so >= off && so < off + step)
				bank->sectors[i].is_erased = 1;
		}
		off += step;
	}

	return ERROR_OK;
}

static int firert_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct firert_mailbox mb;
	uint8_t *verify;
	int retval;

	retval = firert_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_stub_load_and_boot(bank, &mb);
	if (retval != ERROR_OK)
		return retval;

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	verify = malloc(FIRET_STUB_DATA_SIZE);
	if (!verify)
		return ERROR_FAIL;

	while (count) {
		uint32_t this_size = count > FIRET_STUB_DATA_SIZE ? FIRET_STUB_DATA_SIZE : count;

		retval = target_write_buffer(bank->target, FIRET_STUB_DATA_ADDR, this_size, buffer);
		if (retval != ERROR_OK)
			break;

		retval = firert_stub_go(bank, FIRET_CMD_PROGRAM, offset, this_size,
			0, FIRET_STUB_DATA_ADDR, 30000, &mb);
		if (retval != ERROR_OK)
			break;

		retval = target_read_buffer(bank->target, bank->base + offset, this_size, verify);
		if (retval != ERROR_OK)
			break;

		if (memcmp(buffer, verify, this_size) != 0) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			LOG_ERROR("verify mismatch at flash offset 0x%08" PRIx32, offset);
			break;
		}

		buffer += this_size;
		offset += this_size;
		count -= this_size;
	}

	free(verify);
	return retval;
}

static int firert_read(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	return default_flash_read(bank, buffer, offset, count);
}

static int firert_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	return ERROR_OK;
}

static int firert_protect_check(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = 0;
	return ERROR_OK;
}

static int firert_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct firert_flash_bank *info = bank->driver_priv;

	if (!info->probed) {
		command_print_sameline(cmd, "i.MXRT FlexSPI NOR bank not probed");
		return ERROR_OK;
	}

	if (info->dev) {
		command_print_sameline(cmd,
			"i.MXRT FlexSPI NOR: %s, raw JEDEC 0x%08" PRIx32
			", normalized 0x%08" PRIx32 ", size=0x%08" PRIx32
			", erase=0x%08" PRIx32,
			info->dev->name, info->jedec_raw, info->jedec_norm,
			info->dev->size_in_bytes, info->dev->sectorsize);
	} else {
		command_print_sameline(cmd,
			"i.MXRT FlexSPI NOR: unknown raw JEDEC 0x%08" PRIx32
			", normalized 0x%08" PRIx32 ", size=0x%08" PRIx32
			", erase=0x%08x",
			info->jedec_raw, info->jedec_norm, bank->size, 0x1000);
	}

	return ERROR_OK;
}

const struct flash_driver firert_flash = {
	.name = "firert",
	.flash_bank_command = firert_flash_bank_command,
	.erase = firert_erase,
	.protect = firert_protect,
	.write = firert_write,
	.read = firert_read,
	.probe = firert_probe,
	.auto_probe = firert_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = firert_protect_check,
	.info = firert_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
