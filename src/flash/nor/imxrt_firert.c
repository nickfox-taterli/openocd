// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"

#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/register.h>

#include "../../../contrib/loaders/flash/imxrt_firert_stub/protocol.h"
#include "imxrt_firert_stub_bin.inc"

#define FIRET_STUB_LOAD_ADDR 0x20202000u
#define FIRET_STUB_STACK_PTR 0x20205ff0u
#define FIRET_STUB_DATA_ADDR 0x20208000u
#define FIRET_STUB_DATA_SIZE 0x00008000u
#define FIRET_XFER_CHUNK_SIZE 256u
#define FIRET_IO_RETRIES 3

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

static int firert_read_mailbox_retry(struct target *target, struct firert_mailbox *mb)
{
	int retval = ERROR_FAIL;

	for (unsigned int attempt = 1; attempt <= FIRET_IO_RETRIES; attempt++) {
		retval = firert_read_mailbox(target, mb);
		if (retval == ERROR_OK)
			return ERROR_OK;

		LOG_WARNING("i.MXRT mailbox read failed (attempt %u/%u), retval=%d",
			attempt, FIRET_IO_RETRIES, retval);
		alive_sleep(2);
	}

	return retval;
}

static int firert_wait_mailbox_done(struct target *target, int timeout_ms,
		struct firert_mailbox *mb)
{
	int64_t start = timeval_ms();
	int retval = ERROR_FAIL;

	while ((timeval_ms() - start) < timeout_ms) {
		retval = firert_read_mailbox_retry(target, mb);
		if (retval == ERROR_OK) {
			if (mb->status == FIRET_ST_DONE || mb->status == FIRET_ST_ERROR)
				return ERROR_OK;
		}

		keep_alive();
		alive_sleep(1);
	}

	return ERROR_TIMEOUT_REACHED;
}

static int firert_write_buffer_retry(struct target *target, target_addr_t addr,
		const uint8_t *buf, uint32_t len)
{
	int retval = ERROR_FAIL;

	for (unsigned int attempt = 1; attempt <= FIRET_IO_RETRIES; attempt++) {
		uint32_t done = 0;

		retval = firert_ensure_halted(target);
		if (retval != ERROR_OK)
			return retval;

		alive_sleep(1);

		while (done < len) {
			uint32_t chunk = len - done;
			if (chunk > FIRET_XFER_CHUNK_SIZE)
				chunk = FIRET_XFER_CHUNK_SIZE;

			retval = target_write_buffer(target, addr + done, chunk, buf + done);
			if (retval != ERROR_OK)
				break;
			done += chunk;
		}

		if (retval == ERROR_OK)
			return ERROR_OK;

		LOG_WARNING("i.MXRT RAM upload failed (attempt %u/%u), retval=%d",
			attempt, FIRET_IO_RETRIES, retval);
		target_halt(target);
		target_wait_state(target, TARGET_HALTED, 500);
		alive_sleep(2);
	}

	return retval;
}

static int firert_write_buffer_checked(struct target *target, target_addr_t addr,
		const uint8_t *buf, uint32_t len)
{
	uint8_t *verify;
	int retval;

	verify = malloc(len);
	if (!verify)
		return ERROR_FAIL;

	retval = firert_write_buffer_retry(target, addr, buf, len);
	if (retval != ERROR_OK)
		goto out;

	retval = firert_ensure_halted(target);
	if (retval != ERROR_OK)
		goto out;

	retval = target_read_buffer(target, addr, len, verify);
	if (retval != ERROR_OK)
		goto out;

	if (memcmp(buf, verify, len) != 0) {
		LOG_ERROR("i.MXRT RAM staging verify mismatch at 0x%08" TARGET_PRIxADDR,
			addr);
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

out:
	free(verify);
	return retval;
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
		LOG_WARNING("i.MXRT stub wait halt timeout, forcing halt and continuing");
		target_halt(target);
		if (target_wait_state(target, TARGET_HALTED, 1000) == ERROR_OK)
			retval = ERROR_OK;
	}

	return retval;
}

static int firert_stub_load_and_boot(struct flash_bank *bank, struct firert_mailbox *mb)
{
	struct target *target = bank->target;
	int retval = ERROR_FAIL;

	for (unsigned int attempt = 1; attempt <= FIRET_IO_RETRIES; attempt++) {
		retval = firert_ensure_halted(target);
		if (retval != ERROR_OK)
			return retval;

		retval = firert_write_buffer_retry(target, FIRET_STUB_LOAD_ADDR,
				imxrt_firert_stub_bin, imxrt_firert_stub_bin_len);
		if (retval != ERROR_OK)
			continue;

		retval = firert_set_reg_u32(target, "sp", FIRET_STUB_STACK_PTR);
		if (retval != ERROR_OK)
			continue;

		retval = firert_set_reg_u32(target, "pc", FIRET_STUB_LOAD_ADDR | 1u);
		if (retval != ERROR_OK)
			continue;

		retval = firert_set_reg_u32(target, "xpsr", 0x01000000u);
		if (retval != ERROR_OK)
			continue;

		retval = target_resume(target, 1, 0, 0, 0);
		if (retval != ERROR_OK)
			continue;

		retval = target_wait_state(target, TARGET_HALTED, 5000);
		if (retval != ERROR_OK) {
			LOG_WARNING("i.MXRT stub boot halt timeout, forcing halt for mailbox inspection");
			target_halt(target);
			if (target_wait_state(target, TARGET_HALTED, 1000) != ERROR_OK)
				continue;

			retval = firert_read_mailbox_retry(target, mb);
			if (retval != ERROR_OK)
				continue;
		} else {
			retval = firert_read_mailbox_retry(target, mb);
			if (retval != ERROR_OK)
				continue;
		}

		LOG_INFO("imxrt stub boot: status=0x%08" PRIx32 " result=0x%08" PRIx32
			" detail0=0x%08" PRIx32 " detail1=0x%08" PRIx32,
			mb->status, mb->result, mb->detail0, mb->detail1);
		if (mb->magic == FIRET_MB_MAGIC &&
			mb->cmd == FIRET_CMD_NONE &&
			mb->status == FIRET_ST_READY &&
			mb->result != 0u &&
			mb->result != 0xffffffffu)
			return ERROR_OK;

		retval = ERROR_FAIL;
	}

	LOG_ERROR("i.MXRT stub load/boot failed after %u attempts", FIRET_IO_RETRIES);
	return retval;
}

static int firert_stub_go(struct flash_bank *bank, uint32_t cmd, uint32_t addr,
		uint32_t size, uint32_t arg, uint32_t src, int timeout_ms,
		struct firert_mailbox *mb)
{
	struct target *target = bank->target;
	struct firert_mailbox mb_out;
	int retval;

	retval = firert_ensure_halted(target);
	if (retval != ERROR_OK)
		return retval;

	mb_out.magic = FIRET_MB_MAGIC;
	mb_out.cmd = cmd;
	mb_out.addr = addr;
	mb_out.size = size;
	mb_out.arg = arg;
	mb_out.src = src;
	mb_out.status = FIRET_ST_BUSY;
	mb_out.result = 0;
	mb_out.detail0 = 0;
	mb_out.detail1 = 0;

	retval = target_write_buffer(target, FIRET_MB_ADDR, sizeof(mb_out), (uint8_t *)&mb_out);
	if (retval != ERROR_OK)
		return retval;

	retval = target_resume(target, 1, 0, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = firert_resume_wait_halt(target, timeout_ms);
	if (retval != ERROR_OK) {
		LOG_WARNING("i.MXRT stub halt-wait failed, fallback to mailbox polling");
		retval = firert_wait_mailbox_done(target, timeout_ms, mb);
		if (retval != ERROR_OK)
			return retval;

		target_halt(target);
		target_wait_state(target, TARGET_HALTED, 1000);
	}

	retval = firert_read_mailbox_retry(target, mb);
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

		retval = firert_write_buffer_checked(bank->target, FIRET_STUB_DATA_ADDR,
			buffer, this_size);
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
