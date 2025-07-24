/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * OpenOCD flash driver for Renesas RA2L1 (RV40-F flash)
 *
 * Tested with RA2L1 (R7FA2L1AB3CFx) - 256KB Code Flash, 8KB Data Flash
 *
 * Based on RA2L1 User's Manual R01UH0853EJ0150 Rev.1.50 (2024-08-30)
 *
 * Author: TaterLi
 *
 * Notes:
 *  - Code flash erase unit: 2KB blocks; program unit: 32-bit
 *  - Data flash erase unit: 1KB blocks; program unit: 8-bit
 *  - Data flash read window @ 0x40100000, P/E window @ 0xFE000000
 *  - FLCN base: 0x407EC000
 * 
 */

#include "imp.h"
#include "helper/time_support.h"
#include "target/target.h"
#include "target/register.h"
#include "flash/nor/core.h"
#include <stdlib.h>

#define LOG_PREFIX "ra2l1"

/* ------------------------------------------------------------------------- */
/* Register map (offsets from FLCN base)                                      */
#define FLCN_BASE          0x407EC000U
#define REG_DFLCTL         (FLCN_BASE + 0x0090)   /* Data Flash Control */
#define REG_PFBER          (FLCN_BASE + 0x3FC8)   /* Prefetch Buffer Enable */
#define REG_FENTRYR        (FLCN_BASE + 0x3FB0)   /* P/E mode entry */
#define REG_FPMCR          (FLCN_BASE + 0x0100)   /* Mode Control */
#define REG_FPR            (FLCN_BASE + 0x0180)   /* Protection Unlock */
#define REG_FCR            (FLCN_BASE + 0x0114)   /* Command Register */
#define REG_FRESETR        (FLCN_BASE + 0x0124)   /* Error Reset */
#define REG_FISR           (FLCN_BASE + 0x01D8)   /* Frequency */
#define REG_FSARH          (FLCN_BASE + 0x0110)   /* Start Addr High */
#define REG_FSARL          (FLCN_BASE + 0x0108)   /* Start Addr Low  */
#define REG_FEARH          (FLCN_BASE + 0x0120)   /* End Addr High */
#define REG_FEARL          (FLCN_BASE + 0x0118)   /* End Addr Low */
#define REG_FWBH0          (FLCN_BASE + 0x0138)   /* Write Buffer High (code) */
#define REG_FWBL0          (FLCN_BASE + 0x0130)   /* Write Buffer Low  (code & data) */
#define REG_FSTATR1        (FLCN_BASE + 0x012C)   /* Status 1 */
#define REG_FSTATR2        (FLCN_BASE + 0x01F0)   /* Status 2 */
#define REG_FEAML          (FLCN_BASE + 0x01E0)   /* Error Address Low */
#define REG_FEAMH          (FLCN_BASE + 0x01E8)   /* Error Address High */
#define REG_FPSR           (FLCN_BASE + 0x0184)   /* Protection Status */

/* FENTRYR key and values */
#define FENTRY_KEY         0xAA
#define FENTRY_CODE_PE     0xAA01
#define FENTRY_DATA_PE     0xAA80
#define FENTRY_READ        0xAA00

/* FCR commands */
#define FCR_CMD_PROGRAM    0x81
#define FCR_CMD_END1       0x01
#define FCR_CMD_END0       0x00
#define FCR_CMD_ERASE      0x04  /* block erase */
#define FCR_CMD_OPST       0x80  /* operation start bit */
#define FCR_CMD_BLANKCHK   0x83
#define FCR_CMD_CHIPERASE  0x82

/* FPMCR mode bits */
#define FPMCR_CODE_PE      0x02  /* program/erase mode for code */
#define FPMCR_DATA_PE      0x10  /* program/erase mode for data */
#define FPMCR_READ         0x08  /* read mode */

/* Status bits */
#define FSTATR1_FRDY       (1U << 6)   /* Flash ready */
#define FSTATR2_ERERR      (1U << 0)   /* Erase error */
#define FSTATR2_PRGERR     (1U << 1)   /* Program error */
#define FSTATR2_BCERR      (1U << 3)   /* Blank check error */
#define FSTATR2_ILGLERR    (1U << 4)   /* Illegal command/error */

/* Data flash control */
#define DFLCTL_DFLEN       (1U << 0)   /* Data flash enable */

/* Unique ID and Part Number registers */
#define REG_UIDR(n)        (0x01001C00u + ((n) * 4)) /* Unique ID words */
#define REG_PNR(n)         (0x01001C10u + ((n) * 4)) /* Part Number ASCII */
#define REG_MCUVER         0x01001C20u                /* MCU Version */

/* Clock and flash timing */
#define REG_PRCR           0x4001E3FEu   /* Protect register */
#define PRCR_PRC0          (1u << 0)
#define PRCR_PRC1          (1u << 1)
#define PRCR_KEY_A5        (0xA5u << 8)
#define REG_SCKSCR         0x4001E026u   /* Clock source select */
#define CKSEL_MOCO         0x01u         /* MOCO oscillator */
#define REG_SCKDIVCR       0x4001E020u   /* Clock div register */
#define SCKDIVCR_ICK_Pos   24
#define SCKDIVCR_ICK_Msk   (0x7u << SCKDIVCR_ICK_Pos)
#define REG_MEMWAIT        0x4001E031u   /* Memory wait control */
#define FISR_PCKA_8MHZ     0b000111      /* Frequency register setting for 8MHz */

/* Timeouts (ms) */
#define ERASE_TIMEOUT_MS   100  /* per block */
#define PROG_TIMEOUT_MS    10   /* per word */

/* Bank indices */
#define BANK_CODE          0
#define BANK_DATA          1

struct ra2l1_flash_bank {
    uint32_t pe_base;       /* P/E window base address */
    bool     is_data;       /* true for data flash */
};

/*-----------------------------------------------------------------------------*/
/* Helper: unlock/lock protected registers */
static inline int ra2l1_unlock_prcr(struct target *t)
{
    return target_write_u16(t, REG_PRCR, PRCR_KEY_A5 | PRCR_PRC0 | PRCR_PRC1);
}

static inline int ra2l1_lock_prcr(struct target *t)
{
    return target_write_u16(t, REG_PRCR, PRCR_KEY_A5);
}

/* Set internal clock to 8MHz MOCO for flash timing */
int ra2l1_force_iclk_8mhz(struct target *t)
{
    int r = ra2l1_unlock_prcr(t);
    if (r) return r;

    r = target_write_u8(t, REG_SCKSCR, CKSEL_MOCO);
    if (r) goto out;

    uint32_t div;
    r = target_read_u32(t, REG_SCKDIVCR, &div);
    if (r) goto out;
    div = (div & ~SCKDIVCR_ICK_Msk) | (0u << SCKDIVCR_ICK_Pos);
    r = target_write_u32(t, REG_SCKDIVCR, div);
    if (r) goto out;

    r = target_write_u8(t, REG_MEMWAIT, 0);
out:
    ra2l1_lock_prcr(t);
    return r;
}

/* Wait for FRDY flag to match expected level within timeout */
static int ra2l1_wait_frdy(struct target *t, bool ready, int timeout_ms)
{
    int64_t start = timeval_ms();
    uint32_t st;
    while (timeval_ms() - start < timeout_ms) {
        target_read_u32(t, REG_FSTATR1, &st);
        if (!!(st & FSTATR1_FRDY) == ready)
            return ERROR_OK;
    }
    return ERROR_TIMEOUT_REACHED;
}

/* Enable or disable prefetch buffer */
static inline int ra2l1_prefetch_enable(struct target *t, bool en)
{
    return target_write_u8(t, REG_PFBER, en ? 1 : 0);
}

/* Enter or exit program/erase mode */
static int ra2l1_enter_pe(struct target *t, bool data)
{
    int r = target_write_u16(t, REG_FENTRYR, data ? FENTRY_DATA_PE : FENTRY_CODE_PE);
    if (r) return r;
    /* Unlock FPMCR with required mode */
    r = target_write_u8(t, REG_FPR, 0xA5) ||
        target_write_u8(t, REG_FPMCR, data ? FPMCR_DATA_PE : FPMCR_CODE_PE) ||
        target_write_u8(t, REG_FPMCR, (uint8_t)~(data ? FPMCR_DATA_PE : FPMCR_CODE_PE)) ||
        target_write_u8(t, REG_FPMCR, data ? FPMCR_DATA_PE : FPMCR_CODE_PE);
    if (r) return r;
    /* Set frequency for flash operations */
    usleep(50);
    return target_write_u8(t, REG_FISR, FISR_PCKA_8MHZ);
}

static int ra2l1_exit_pe(struct target *t)
{
    int r = target_write_u16(t, REG_FENTRYR, FENTRY_READ);
    if (r) return r;
    /* Return to read mode */
    return target_write_u8(t, REG_FPR, 0xA5) ||
           target_write_u8(t, REG_FPMCR, FPMCR_READ) ||
           target_write_u8(t, REG_FPMCR, (uint8_t)~FPMCR_READ) ||
           target_write_u8(t, REG_FPMCR, FPMCR_READ);
}

/* Check and clear flash errors */
static int ra2l1_check_errors(struct target *t)
{
    uint32_t st2;
    target_read_u32(t, REG_FSTATR2, &st2);
    if (st2 & (FSTATR2_ERERR | FSTATR2_PRGERR | FSTATR2_BCERR | FSTATR2_ILGLERR)) {
        LOG_ERROR(LOG_PREFIX ": flash error=0x%08" PRIx32, st2);
        /* Clear error flags */
        target_write_u32(t, REG_FRESETR, 1);
        target_write_u32(t, REG_FRESETR, 0);
        return ERROR_FAIL;
    }
    return ERROR_OK;
}

static int ra2l1_seq_sync(struct target *t)
{
    uint32_t st1;
    /* OPST 清 0，避免残留 */
    target_write_u8(t, REG_FCR, 0x00);

    target_read_u32(t, REG_FSTATR1, &st1);
    if (st1 & (1u << 6)) {               // FRDY 仍然=1?
        /* 按图中虚线框：FRESETR=1 -> 0 */
        target_write_u32(t, REG_FRESETR, 1);
        target_write_u32(t, REG_FRESETR, 0);
    }
    return ERROR_OK;
}

/* Convert offset to P/E address (data flash mapped to 0xFE00_0000) */
static inline uint32_t ra2l1_to_pe_addr(struct flash_bank *bank, uint32_t off)
{
    struct ra2l1_flash_bank *info = bank->driver_priv;
    return info->pe_base + off;
}

/* Program one word (code) or byte (data) */
static int ra2l1_program(struct flash_bank *bank, uint32_t addr, const void *data, unsigned size)
{
    struct target *t = bank->target;
    int r;
    /* Set address */
    r = target_write_u16(t, REG_FSARL, addr & 0xFFFF) ||
        target_write_u16(t, REG_FSARH, addr >> 16);
    if (r) return r;
    /* Write data buffer */
    if (size == 4) {
        uint32_t w;
        memcpy(&w, data, 4);
        r = target_write_u32(t, REG_FWBL0, w) || target_write_u32(t, REG_FWBH0, w >> 16);
    } else {
        uint8_t b = *(const uint8_t *)data;
        r = target_write_u8(t, REG_FWBL0, b);
    }
    if (r) return r;
    /* Issue program command */
    r = target_write_u8(t, REG_FCR, FCR_CMD_PROGRAM);
    if (r) return r;
    /* Wait ready and finalize */
    r = ra2l1_wait_frdy(t, true, PROG_TIMEOUT_MS) ||
        target_write_u8(t, REG_FCR, FCR_CMD_END1) ||
        target_write_u8(t, REG_FCR, FCR_CMD_END0) ||
        ra2l1_wait_frdy(t, false, PROG_TIMEOUT_MS);
    if (r) return r;
    return ra2l1_check_errors(t);
}

static int ra2l1_block_erase(struct flash_bank *bank, uint32_t addr)
{
    struct target *t = bank->target;
    struct ra2l1_flash_bank *info = bank->driver_priv;
    uint32_t size = info->is_data ? 1024 : 2048;
    uint32_t end = addr + size - 1;
    int r;

    /* Prepare error flags */
    uint32_t st1, st2;
    target_read_u32(t, REG_FSTATR1, &st1);
    target_read_u32(t, REG_FSTATR2, &st2);
    if (st2) {
        /* Reset on existing error */
        target_write_u32(t, REG_FRESETR, 1);
        target_write_u32(t, REG_FRESETR, 0);
    }
    /* Set addresses */
    r = target_write_u16(t, REG_FSARL, addr & 0xFFFF) ||
        target_write_u16(t, REG_FSARH, addr >> 16) ||
        target_write_u16(t, REG_FEARL, end & 0xFFFF) ||
        target_write_u16(t, REG_FEARH, end >> 16);
    if (r) return r;
    /* Issue erase */
    r = target_write_u8(t, REG_FCR, FCR_CMD_OPST | FCR_CMD_ERASE);
    if (r) return r;
    /* Wait for completion or error */
    int64_t start = timeval_ms();
    while (timeval_ms() - start < ERASE_TIMEOUT_MS) {
        target_read_u32(t, REG_FSTATR1, &st1);
        if (st1 & FSTATR1_FRDY) break;
        target_read_u32(t, REG_FSTATR2, &st2);
        if (st2)
            return ERROR_FAIL;
    }
    if (!(st1 & FSTATR1_FRDY))
        return ERROR_TIMEOUT_REACHED;
    return ra2l1_check_errors(t);
}

/* Read unique ID and part info on probe */
static void ra2l1_read_chip_info(struct target *t)
{
    static bool printed = false;
    if (printed)
        return;
    printed = true;

    uint32_t uid[4], pn[4];
    uint8_t ver;

    for (int i = 0; i < 4; i++)
        target_read_u32(t, REG_UIDR(i), &uid[i]);

    for (int i = 0; i < 4; i++)
        target_read_u32(t, REG_PNR(i), &pn[i]);

    target_read_u8(t, REG_MCUVER, &ver);

    char part[17];
    memcpy(part, pn, 16);
    part[16] = '\0';

    LOG_INFO(LOG_PREFIX ": UID=%08" PRIx32 "%08" PRIx32 "%08" PRIx32 "%08" PRIx32,
             uid[0], uid[1], uid[2], uid[3]);
    LOG_INFO(LOG_PREFIX ": Part#='%s', Ver=0x%02x", part, ver);
}

static int ra2l1_flash_probe(struct flash_bank *bank)
{
    struct ra2l1_flash_bank *info = calloc(1, sizeof(*info));
    if (!info) return ERROR_FAIL;
    bank->driver_priv = info;

    /* Distinguish code vs data flash by base address */
    if (bank->base == 0x40100000) {
        info->is_data = true;
        info->pe_base = 0xFE000000;
        bank->size = 8 * 1024;
        bank->num_sectors = 8;
    } else {
        info->is_data = false;
        info->pe_base = 0x00000000;
        bank->size = 256 * 1024;
        bank->num_sectors = bank->size / 2048;
    }
    bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
    if (!bank->sectors) return ERROR_FAIL;
    for (unsigned i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].offset = i * (info->is_data?1024:2048);
        bank->sectors[i].size   = (info->is_data?1024:2048);
    }

    /* Print chip info once for code flash */
    if (!info->is_data)
        ra2l1_read_chip_info(bank->target);

    /* Disable prefetch and enable data flash window */
    ra2l1_prefetch_enable(bank->target, false);
    target_write_u8(bank->target, REG_DFLCTL, DFLCTL_DFLEN);
    return ERROR_OK;
}

static int ra2l1_flash_auto_probe(struct flash_bank *bank)
{
    if (bank->target->state != TARGET_HALTED)
        return ERROR_TARGET_NOT_HALTED;
    return ra2l1_flash_probe(bank);
}

static int ra2l1_flash_erase(
    struct flash_bank *bank, unsigned first, unsigned last)
{
    struct target *t = bank->target;
    struct ra2l1_flash_bank *info = bank->driver_priv;
    if (t->state != TARGET_HALTED)
        return ERROR_TARGET_NOT_HALTED;

    int r = ra2l1_enter_pe(t, info->is_data);
    if (r) return r;
    for (unsigned i = first; i <= last; i++) {
        uint32_t addr = ra2l1_to_pe_addr(bank, bank->sectors[i].offset);
        r = ra2l1_block_erase(bank, addr);
        if (r) break;
    }
    ra2l1_exit_pe(t);
    return r;
}

static int ra2l1_flash_write(struct flash_bank *bank,
                             const uint8_t *buf,
                             uint32_t offset,
                             uint32_t count)
{
    struct target *t = bank->target;
    struct ra2l1_flash_bank *info = bank->driver_priv;
    if (t->state != TARGET_HALTED)
        return ERROR_TARGET_NOT_HALTED;

    const uint32_t block_size = info->is_data ? 1024 : 2048;
    uint32_t addr = ra2l1_to_pe_addr(bank, offset);
    uint32_t end  = addr + count;
    uint32_t cur_addr = addr;

    while (cur_addr < end) {
        // 计算当前 block 起始地址
        uint32_t block_start = cur_addr & ~(block_size - 1);
        uint32_t block_end   = block_start + block_size;
        if (block_end > end)
            block_end = end;

        // 进入 PE 模式
        int r = ra2l1_enter_pe(t, info->is_data);
        if (r != ERROR_OK)
            return r;

        // 擦除当前 block
        r = ra2l1_block_erase(bank, block_start);
        if (r != ERROR_OK) {
            LOG_ERROR("Block erase failed at 0x%08x", block_start);
            ra2l1_exit_pe(t);
            return r;
        }

        ra2l1_seq_sync(t);

        // 写入 block 内的数据
        while (cur_addr < block_end) {
            unsigned chunk = info->is_data ? 1 : 4;
            r = ra2l1_program(bank, cur_addr, buf, chunk);
            if (r != ERROR_OK) {
                LOG_ERROR("Program failed at 0x%08x", cur_addr);
                ra2l1_exit_pe(t);
                return r;
            }
            cur_addr += chunk;
            buf      += chunk;
        }

        ra2l1_exit_pe(t);
    }

    return ERROR_OK;
}


static int ra2l1_flash_read(
    struct flash_bank *bank, uint8_t *buf,
    uint32_t offset, uint32_t count)
{
    return target_read_memory(bank->target, bank->base + offset,
                              1, count, buf);
}

FLASH_BANK_COMMAND_HANDLER(ra2l1_flash_bank_command)
{
    /* Parse: flash bank <name> ra2l1 <base> <size> <chip_width> <bus_width> <target> */
    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;
    bank->base       = strtoull(CMD_ARGV[1], NULL, 0);
    bank->size       = strtoul (CMD_ARGV[2], NULL, 0);
    bank->chip_width = strtol  (CMD_ARGV[3], NULL, 0);
    bank->bus_width  = strtol  (CMD_ARGV[4], NULL, 0);
    bank->target     = get_target(CMD_ARGV[5]);
    if (!bank->target)
        return ERROR_FAIL;
    return ERROR_OK;
}

static const struct command_registration ra2l1_exec_command_handlers[] = {
    COMMAND_REGISTRATION_DONE
};

const struct flash_driver ra2l1_flash = {
    .name               = "ra2l1",
    .commands           = ra2l1_exec_command_handlers,
    .flash_bank_command = ra2l1_flash_bank_command,
    .erase              = ra2l1_flash_erase,
    .protect            = NULL,
    .write              = ra2l1_flash_write,
    .read               = ra2l1_flash_read,
    .probe              = ra2l1_flash_probe,
    .auto_probe         = ra2l1_flash_auto_probe,
    .erase_check        = default_flash_blank_check,
    .info               = NULL,
};
