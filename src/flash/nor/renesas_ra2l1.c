// SPDX-License-Identifier: GPL-2.0-or-later
//
// OpenOCD flash driver for Renesas RA2L1 (RV40-F flash)
//
// Basic verification done on RA2L1 (R7FA2L1AB3CFx): Code Flash 256KB, Data Flash 8KB
// Reference manual: RA2L1 User's Manual R01UH0853EJ0150 Rev.1.50 (2024-08-30)
//
// Notes:
//  - Code Flash erase unit 2KB, programming unit 32bit
//  - Data Flash erase unit 1KB, programming unit 8bit
//  - Data Flash read window 0x40100000, P/E window 0xFE000000
//  - FLCN base address 0x407EC000
//
// Write strategy:
//  1) On entering write(), first do pre-comparison by blocks in "read window", skip if entire block is identical
//  2) Merge consecutive blocks that need update into "segments", enter P/E once per segment, within segment: erase first, then continuous programming per block
//  3) Poll FRDY / FSTATR2, no fixed delays
//
// Author: TaterLi

#include "imp.h"
#include "helper/time_support.h"
#include "target/target.h"
#include "target/register.h"
#include "flash/nor/core.h"
#include <stdlib.h>
#include <string.h>

#define LOG_PREFIX "ra2l1"

// Register map (offsets from FLCN base)
#define FLCN_BASE 0x407EC000U
#define REG_DFLCTL (FLCN_BASE + 0x0090)  // Data Flash Control
#define REG_PFBER (FLCN_BASE + 0x3FC8)   // Prefetch Buffer Enable
#define REG_FENTRYR (FLCN_BASE + 0x3FB0) // P/E mode entry
#define REG_FPMCR (FLCN_BASE + 0x0100)   // Mode Control
#define REG_FPR (FLCN_BASE + 0x0180)     // Protection Unlock
#define REG_FCR (FLCN_BASE + 0x0114)     // Command Register
#define REG_FRESETR (FLCN_BASE + 0x0124) // Error Reset
#define REG_FISR (FLCN_BASE + 0x01D8)    // Frequency/Startup Area
#define REG_FSARH (FLCN_BASE + 0x0110)   // Start Addr High
#define REG_FSARL (FLCN_BASE + 0x0108)   // Start Addr Low
#define REG_FEARH (FLCN_BASE + 0x0120)   // End Addr High
#define REG_FEARL (FLCN_BASE + 0x0118)   // End Addr Low
#define REG_FWBH0 (FLCN_BASE + 0x0138)   // Write Buffer High (code)
#define REG_FWBL0 (FLCN_BASE + 0x0130)   // Write Buffer Low  (code & data)
#define REG_FSTATR1 (FLCN_BASE + 0x012C) // Status 1 (FRDY)
#define REG_FSTATR2 (FLCN_BASE + 0x01F0) // Status 2 (ERR)
#define REG_FEAML (FLCN_BASE + 0x01E0)   // Error Address Low
#define REG_FEAMH (FLCN_BASE + 0x01E8)   // Error Address High
#define REG_FPSR (FLCN_BASE + 0x0184)    // Protection Status

#define REG_FACI_ERRATA 0x407EFFC4U

// FENTRYR key and values
#define FENTRY_KEY 0xAA
#define FENTRY_CODE_PE 0xAA01
#define FENTRY_DATA_PE 0xAA80
#define FENTRY_READ 0xAA00

// FCR commands
#define FCR_CMD_PROGRAM 0x81
#define FCR_CMD_END1 0x01
#define FCR_CMD_END0 0x00
#define FCR_CMD_ERASE_SEQ 0x84 // erase sequence init
#define FCR_CMD_ERASE 0x04     // block erase
#define FCR_CMD_BLANKCHK 0x83

// FPMCR mode bits (need to write FPR=0xA5 to unlock before writing)
#define FPMCR_CODE_PE 0x02 // program/erase mode for code
#define FPMCR_DATA_PE 0x10 // program/erase mode for data
#define FPMCR_READ 0x08    // read mode

// Status bits
#define FSTATR1_FRDY (1U << 6)    // Flash ready
#define FSTATR2_ERERR (1U << 0)   // Erase error
#define FSTATR2_PRGERR (1U << 1)  // Program error
#define FSTATR2_BCERR (1U << 3)   // Blank check error
#define FSTATR2_ILGLERR (1U << 4) // Illegal command/error

// Data flash control
#define DFLCTL_DFLEN (1U << 0) // Data flash enable

// Unique ID and Part Number registers
#define REG_UIDR(n) (0x01001C00u + ((n) * 4)) // Unique ID words
#define REG_PNR(n) (0x01001C10u + ((n) * 4))  // Part Number ASCII
#define REG_MCUVER 0x01001C20u                // MCU Version

// SYSC: Clock / Power registers
#define SYSC_BASE 0x4001E000u
#define REG_PRCR (SYSC_BASE + 0x3FEu) // Protect register
#define PRCR_PRC0 (1u << 0)
#define PRCR_PRC1 (1u << 1)
#define PRCR_KEY_A5 (0xA5u << 8)

#define REG_SCKDIVCR (SYSC_BASE + 0x020u) // Clock div register
#define REG_SCKSCR (SYSC_BASE + 0x026u)   // Clock source select
#define REG_MEMWAIT (SYSC_BASE + 0x031u)  // Memory wait control
#define REG_HOCOCR (SYSC_BASE + 0x036u)   // HOCO control
#define REG_MOCOCR (SYSC_BASE + 0x038u)   // MOCO control
#define REG_OSCSF (SYSC_BASE + 0x03Cu)    // Oscillation flags
#define REG_OPCCR (SYSC_BASE + 0x0A0u)    // Operation power mode

// SCKSCR CKSEL: 0=HOCO, 1=MOCO (only these two are used here)
#define CKSEL_HOCO 0x00u
#define CKSEL_MOCO 0x01u

// SCKDIVCR ICK field
#define SCKDIVCR_ICK_Pos 24
#define SCKDIVCR_ICK_Msk (0x7u << SCKDIVCR_ICK_Pos)

// FISR.PCKA encoding occupies 6 bits, below are placeholder values, final values based on manual
#define FISR_PCKA_8MHZ 0b000111
#define FISR_PCKA_48MHZ 0b100011 // placeholder only

// Timeouts (ms)
#define ERASE_TIMEOUT_MS 100
#define PROG_TIMEOUT_MS 10

// Bank indices
#define BANK_CODE 0
#define BANK_DATA 1

struct ra2l1_flash_bank
{
    uint32_t pe_base; // P/E window base
    bool is_data;     // true: data flash
};

// Utility: unlock/lock write protection register
static inline int ra2l1_unlock_prcr(struct target *t)
{
    return target_write_u16(t, REG_PRCR, PRCR_KEY_A5 | PRCR_PRC0 | PRCR_PRC1);
}

static inline int ra2l1_lock_prcr(struct target *t)
{
    return target_write_u16(t, REG_PRCR, PRCR_KEY_A5);
}

// Force to MOCO function: switch ICLK to MOCO 8MHz, and ensure ICK=/1, MEMWAIT=0
// Here we only do "steady state 8MHz", convenient for using 8MHz PCKA value in FISR later
static int ra2l1_force_moco(struct target *t)
{
    int r;
    uint8_t oscsf = 0, cksel = 0;
    uint32_t div = 0;

    target_read_u8(t, REG_OSCSF, &oscsf);
    target_read_u8(t, REG_SCKSCR, &cksel);
    target_read_u32(t, REG_SCKDIVCR, &div);

    if (cksel == CKSEL_MOCO)
    {
        bool fixed = false;

        if (((div & SCKDIVCR_ICK_Msk) >> SCKDIVCR_ICK_Pos) != 0u)
        {
            r = ra2l1_unlock_prcr(t);
            if (r)
                return r;
            div = (div & ~SCKDIVCR_ICK_Msk) | (0u << SCKDIVCR_ICK_Pos);
            r = target_write_u32(t, REG_SCKDIVCR, div);
            ra2l1_lock_prcr(t);
            if (r)
                return r;
            fixed = true;
        }

        uint8_t mw = 0xFF;
        target_read_u8(t, REG_MEMWAIT, &mw);
        if (mw != 0)
        {
            r = ra2l1_unlock_prcr(t);
            if (r)
                return r;
            r = target_write_u8(t, REG_MEMWAIT, 0);
            ra2l1_lock_prcr(t);
            if (r)
                return r;
            fixed = true;
        }

        target_read_u8(t, REG_OSCSF, &oscsf);
        target_read_u8(t, REG_SCKSCR, &cksel);
        target_read_u32(t, REG_SCKDIVCR, &div);
        // LOG_INFO(LOG_PREFIX ": force MOCO: already MOCO; %sfixed. OSCSF=0x%02x, SCKSCR=0x%02x, ICKdiv=%u, MEMWAIT=%u",
        //          fixed ? "" : "no-", oscsf, cksel,
        //          (unsigned)((div & SCKDIVCR_ICK_Msk) >> SCKDIVCR_ICK_Pos),
        //          (unsigned)mw);
        return ERROR_OK;
    }

    r = ra2l1_unlock_prcr(t);
    if (r)
        return r;

    r = target_write_u8(t, REG_MOCOCR, 0x00);
    if (r)
        goto out_lock;

    r = target_write_u8(t, REG_MEMWAIT, 0);
    if (r)
        goto out_lock;

    // If needed to enable FACI features per chip errata, uncomment here; default keep disabled to avoid issues
    // r = target_write_u8(t, REG_FACI_ERRATA, 1);

    r = target_write_u8(t, REG_SCKSCR, CKSEL_MOCO);
    if (r)
        goto out_lock;

    r = target_read_u32(t, REG_SCKDIVCR, &div);
    if (r)
        goto out_lock;
    div = (div & ~SCKDIVCR_ICK_Msk) | (0u << SCKDIVCR_ICK_Pos);
    r = target_write_u32(t, REG_SCKDIVCR, div);
    if (r)
        goto out_lock;

    target_read_u8(t, REG_OSCSF, &oscsf);
    target_read_u8(t, REG_SCKSCR, &cksel);
    target_read_u32(t, REG_SCKDIVCR, &div);
    {
        uint8_t mw = 0xFF;
        target_read_u8(t, REG_MEMWAIT, &mw);
        LOG_INFO(LOG_PREFIX ": force MOCO: switched. OSCSF=0x%02x, SCKSCR=0x%02x, ICKdiv=%u, MEMWAIT=%u",
                 oscsf, cksel, (unsigned)((div & SCKDIVCR_ICK_Msk) >> SCKDIVCR_ICK_Pos), (unsigned)mw);
    }

out_lock:
    ra2l1_lock_prcr(t);
    return r;
}

// FRDY polling & prefetch control
static int ra2l1_wait_frdy(struct target *t, bool ready, int timeout_ms)
{
    int64_t start = timeval_ms();
    uint32_t st;
    while (timeval_ms() - start < timeout_ms)
    {
        target_read_u32(t, REG_FSTATR1, &st);
        if (!!(st & FSTATR1_FRDY) == ready)
            return ERROR_OK;
    }
    return ERROR_TIMEOUT_REACHED;
}

static inline int ra2l1_prefetch_enable(struct target *t, bool en)
{
    return target_write_u8(t, REG_PFBER, en ? 1 : 0);
}

// Enter/exit P/E, on entering P/E also write FISR.PCKA (for 8MHz)
static int ra2l1_enter_pe(struct target *t, bool data)
{
    int r = target_write_u16(t, REG_FENTRYR, data ? FENTRY_DATA_PE : FENTRY_CODE_PE);
    if (r)
        return r;

    r = target_write_u8(t, REG_FPR, 0xA5) ||
        target_write_u8(t, REG_FPMCR, data ? FPMCR_DATA_PE : FPMCR_CODE_PE) ||
        target_write_u8(t, REG_FPMCR, (uint8_t)~(data ? FPMCR_DATA_PE : FPMCR_CODE_PE)) ||
        target_write_u8(t, REG_FPMCR, data ? FPMCR_DATA_PE : FPMCR_CODE_PE);
    if (r)
        return r;

    r = target_write_u8(t, REG_FISR, FISR_PCKA_8MHZ);
    return r;
}

static int ra2l1_exit_pe(struct target *t)
{
    int r = target_write_u16(t, REG_FENTRYR, FENTRY_READ);
    if (r)
        return r;

    return target_write_u8(t, REG_FPR, 0xA5) ||
           target_write_u8(t, REG_FPMCR, FPMCR_READ) ||
           target_write_u8(t, REG_FPMCR, (uint8_t)~FPMCR_READ) ||
           target_write_u8(t, REG_FPMCR, FPMCR_READ);
}

// Check and clear errors
static int ra2l1_check_errors(struct target *t)
{
    uint32_t st2 = 0;
    target_read_u32(t, REG_FSTATR2, &st2);
    if (st2 & (FSTATR2_ERERR | FSTATR2_PRGERR | FSTATR2_BCERR | FSTATR2_ILGLERR))
    {
        LOG_INFO(LOG_PREFIX ": flash error=0x%08" PRIx32, st2);
        target_write_u32(t, REG_FRESETR, 1);
        target_write_u32(t, REG_FRESETR, 0);
        return ERROR_FAIL;
    }
    return ERROR_OK;
}

// Address conversion & sequence synchronization
static inline uint32_t ra2l1_to_pe_addr(struct flash_bank *bank, uint32_t off)
{
    struct ra2l1_flash_bank *info = bank->driver_priv;
    return info->pe_base + off;
}

static int ra2l1_seq_sync(struct target *t)
{
    uint32_t st1;
    target_write_u8(t, REG_FCR, 0x00);
    target_read_u32(t, REG_FSTATR1, &st1);
    if (st1 & FSTATR1_FRDY)
    {
        target_write_u32(t, REG_FRESETR, 1);
        target_write_u32(t, REG_FRESETR, 0);
    }
    return ERROR_OK;
}

/*
 * Program a unit:
 *  - Code Flash: write 32bit at once, address needs 4-byte alignment
 *  - Data Flash: write 8bit at once
 * Flow reference Figure 37.19: load address → load data → FCR=0x81 → wait FRDY=1 (accepted)
 * → FCR=0x01 → FCR=0x00 → wait FRDY=0 (enter busy) → check early errors → return
 * Do not wait for final completion; use ra2l1_seq_sync() at segment end for unified cleanup
 */
static int ra2l1_program(struct flash_bank *bank, uint32_t addr, const void *data, unsigned size)
{
    struct target *t = bank->target;
    int r;

    if (size == 4 && (addr & 0x3))
    {
        LOG_ERROR(LOG_PREFIX ": program addr not 4-byte aligned: 0x%08x", addr);
        return ERROR_FAIL;
    }

    r = target_write_u16(t, REG_FSARL, (uint16_t)(addr & 0xFFFF)) ||
        target_write_u16(t, REG_FSARH, (uint16_t)(addr >> 16));
    if (r)
        return r;

    if (size == 4)
    {
        uint32_t w;
        memcpy(&w, data, 4);
        r = target_write_u32(t, REG_FWBL0, w) || target_write_u32(t, REG_FWBH0, w >> 16);
    }
    else
    {
        uint8_t b = *(const uint8_t *)data;
        r = target_write_u8(t, REG_FWBL0, b);
        if (r)
            return r;
    }

    r = target_write_u8(t, REG_FCR, FCR_CMD_PROGRAM);
    if (r)
        return r;

    r = ra2l1_wait_frdy(t, true, 5);
    if (r)
    {
        uint32_t st1 = 0, st2 = 0;
        target_read_u32(t, REG_FSTATR1, &st1);
        target_read_u32(t, REG_FSTATR2, &st2);
        LOG_ERROR(LOG_PREFIX ": program: FRDY not 1 after 0x81 @0x%08x (FSTATR1=0x%08x,FSTATR2=0x%08x)",
                  addr, st1, st2);
        target_write_u32(t, REG_FRESETR, 1);
        target_write_u32(t, REG_FRESETR, 0);
        return r;
    }

    r = target_write_u8(t, REG_FCR, FCR_CMD_END1);
    if (r)
        return r;
    r = target_write_u8(t, REG_FCR, FCR_CMD_END0);
    if (r)
        return r;

    {
        uint32_t st1 = 0;
        int64_t t0 = timeval_ms();
        do
        {
            target_read_u32(t, REG_FSTATR1, &st1);
            if ((st1 & FSTATR1_FRDY) == 0)
                break;
        } while (timeval_ms() - t0 < 5);

        if (st1 & FSTATR1_FRDY)
        {
            uint32_t st2 = 0;
            target_read_u32(t, REG_FSTATR2, &st2);
            LOG_ERROR(LOG_PREFIX ": program: did not enter busy @0x%08x (FSTATR2=0x%08x)", addr, st2);
            target_write_u32(t, REG_FRESETR, 1);
            target_write_u32(t, REG_FRESETR, 0);
            return ERROR_FAIL;
        }
    }

    {
        uint32_t st2 = 0;
        target_read_u32(t, REG_FSTATR2, &st2);
        if (st2 & (FSTATR2_ILGLERR | FSTATR2_PRGERR))
        {
            uint32_t feah = 0, feal = 0;
            target_read_u32(t, REG_FEAMH, &feah);
            target_read_u32(t, REG_FEAML, &feal);
            LOG_ERROR(LOG_PREFIX ": program early error @0x%08x, FSTATR2=0x%08x, FEAR=0x%04x%04x",
                      addr, st2, (unsigned)feah, (unsigned)feal);
            target_write_u32(t, REG_FRESETR, 1);
            target_write_u32(t, REG_FRESETR, 0);
            return ERROR_FAIL;
        }
    }

    return ERROR_OK;
}

// Block erase: follow Figure 37.23 flow, first 0x84 accepted, then 0x04 + 0x00 to start
// On entry FRDY may be 0 or 1, on exit as long as FRDY=0 it counts as start success; errors checked after completion
static int ra2l1_block_erase(struct flash_bank *bank, uint32_t addr)
{
    struct target *t = bank->target;
    struct ra2l1_flash_bank *info = bank->driver_priv;
    const uint32_t blk_sz = info->is_data ? 1024u : 2048u;
    const uint32_t start = addr;
    const uint32_t end = addr + blk_sz - 1;

    int r = 0;

    LOG_DEBUG(LOG_PREFIX ": erase @0x%08x..0x%08x", start, end);

    r = target_write_u16(t, REG_FSARL, start & 0xFFFF) ||
        target_write_u16(t, REG_FSARH, start >> 16) ||
        target_write_u16(t, REG_FEARL, end & 0xFFFF) ||
        target_write_u16(t, REG_FEARH, end >> 16);
    if (r)
        return r;

    r = target_write_u8(t, REG_FCR, FCR_CMD_ERASE_SEQ);
    if (r)
        return r;

    r = ra2l1_wait_frdy(t, true, 100);
    if (r)
    {
        LOG_ERROR(LOG_PREFIX ": erase seq 0x84 not accepted @0x%08x..0x%08x", start, end);
        return r;
    }

    r = target_write_u8(t, REG_FCR, FCR_CMD_ERASE);
    if (r)
        return r;
    r = target_write_u8(t, REG_FCR, FCR_CMD_END0);
    if (r)
        return r;

    r = ra2l1_wait_frdy(t, false, 1000);
    if (r)
    {
        LOG_ERROR(LOG_PREFIX ": erase timeout");
        return r;
    }

    r = ra2l1_check_errors(t);
    if (r)
    {
        uint32_t feah = 0, feal = 0;
        target_read_u32(t, REG_FEAMH, &feah);
        target_read_u32(t, REG_FEAML, &feal);
        LOG_ERROR(LOG_PREFIX ": erase error @0x%08x..0x%08x, FEAR=0x%04x%04x",
                  start, end, (unsigned)feah, (unsigned)feal);
        return r;
    }

    return ERROR_OK;
}

// Device information (print only once)
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

// Pre-scan: only compare non-0xFF bytes in image; skip if entire block identical
static int ra2l1_prescan_mark_blocks(struct flash_bank *bank,
                                     const uint8_t *buf,
                                     uint32_t offset,
                                     uint32_t count,
                                     uint8_t *mark)
{
    struct ra2l1_flash_bank *info = bank->driver_priv;
    const uint32_t blk_sz = info->is_data ? 1024u : 2048u;

    const uint32_t first_blk = offset / blk_sz;
    const uint32_t last_blk = (offset + count - 1) / blk_sz;

    uint32_t first_diff_idx = 0;

    uint8_t *cur = (uint8_t *)malloc(blk_sz);
    if (!cur)
        return ERROR_FAIL;

    for (uint32_t b = first_blk; b <= last_blk; ++b)
    {
        const uint32_t b_off = b * blk_sz;
        const uint32_t s = (b_off < offset) ? offset : b_off;
        const uint32_t e = ((b_off + blk_sz) > (offset + count)) ? (offset + count) : (b_off + blk_sz);

        mark[b] = 0;
        if (e <= s)
            continue;

        int r = target_read_memory(bank->target, bank->base + b_off, 1, blk_sz, cur);
        if (r)
        {
            LOG_ERROR(LOG_PREFIX ": prescan: read block %u @ [0x%08x..0x%08x] failed (%d)",
                      (unsigned)b, (unsigned)b_off, (unsigned)(b_off + blk_sz - 1), r);
            free(cur);
            return r;
        }

        const uint8_t *bp = buf + (s - offset);
        const uint8_t *mp = cur + (s - b_off);
        const uint32_t cmp_len = e - s;

        for (uint32_t i = 0; i < cmp_len; ++i)
        {
            uint8_t bi = bp[i];
            if (bi == 0xFF)
                continue;
            if (bi != mp[i])
            {
                first_diff_idx = i;
                mark[b] = 1;
                break;
            }
        }

        if (mark[b])
        {
            uint32_t diff_abs = s + (uint32_t)first_diff_idx;
            uint32_t win = 8;
            uint32_t w_lo = (first_diff_idx > win) ? (first_diff_idx - win) : 0;
            uint32_t w_hi = ((first_diff_idx + win) < cmp_len) ? (first_diff_idx + win) : (cmp_len - 1);

            char img_hex[3 * 17] = {0}, dev_hex[3 * 17] = {0};
            unsigned k = 0;
            for (uint32_t j = w_lo; j <= w_hi && k < 16; ++j, ++k)
            {
                if (bp[j] != 0xFF)
                {
                    snprintf(img_hex + strlen(img_hex), sizeof(img_hex) - strlen(img_hex), "%02X ", bp[j]);
                    snprintf(dev_hex + strlen(dev_hex), sizeof(dev_hex) - strlen(dev_hex), "%02X ", mp[j]);
                }
                else
                {
                    snprintf(img_hex + strlen(img_hex), sizeof(img_hex) - strlen(img_hex), ".. ");
                    snprintf(dev_hex + strlen(dev_hex), sizeof(dev_hex) - strlen(dev_hex), ".. ");
                }
            }

            LOG_DEBUG(LOG_PREFIX
                      ": prescan: block %u need_upd (range=[0x%08x..0x%08x], first_diff=0x%08x, img/dev[%uB]: %s | %s)",
                      (unsigned)b, (unsigned)s, (unsigned)(e - 1),
                      (unsigned)diff_abs, (unsigned)k, img_hex, dev_hex);
        }
    }

    free(cur);
    return ERROR_OK;
}

// Segment processing: enter P/E once, within segment erase per block → continuous programming → exit P/E at segment end
static int ra2l1_process_segment(struct flash_bank *bank,
                                 uint32_t seg_blk_first, uint32_t seg_blk_last,
                                 const uint8_t *buf, uint32_t offset, uint32_t count)
{
    struct target *t = bank->target;
    struct ra2l1_flash_bank *info = bank->driver_priv;
    const uint32_t blk = info->is_data ? 1024u : 2048u;

    int r = ra2l1_enter_pe(t, info->is_data);
    if (r)
        return r;

    for (uint32_t b = seg_blk_first; b <= seg_blk_last; ++b)
    {
        uint32_t b_off = b * blk;
        uint32_t pe_base = ra2l1_to_pe_addr(bank, b_off);

        r = ra2l1_block_erase(bank, pe_base);
        if (r)
        {
            ra2l1_exit_pe(t);
            return r;
        }
        ra2l1_seq_sync(t);

        uint32_t s = (b_off < offset) ? offset : b_off;
        uint32_t e = ((b_off + blk) > (offset + count)) ? (offset + count) : (b_off + blk);
        uint32_t len = (e > s) ? (e - s) : 0;
        if (len == 0)
            continue;

        const uint8_t *bp = buf + (s - offset);

        if (info->is_data)
        {
            for (uint32_t i = 0; i < len; ++i)
            {
                uint8_t v = bp[i];
                if (v == 0xFF)
                    continue;
                uint32_t pe_addr = ra2l1_to_pe_addr(bank, s + i);
                r = ra2l1_program(bank, pe_addr, &v, 1);
                if (r)
                {
                    ra2l1_exit_pe(t);
                    return r;
                }
            }
        }
        else
        {
            uint32_t i = 0;
            while (i < len)
            {
                uint32_t rem = len - i;
                uint8_t tmp[4] = {0xFF, 0xFF, 0xFF, 0xFF};
                uint32_t step = (rem >= 4) ? 4 : rem;
                memcpy(tmp, bp + i, step);

                uint32_t w = ((uint32_t)tmp[0]) |
                             ((uint32_t)tmp[1] << 8) |
                             ((uint32_t)tmp[2] << 16) |
                             ((uint32_t)tmp[3] << 24);

                if (w != 0xFFFFFFFFu)
                {
                    uint32_t pe_addr = ra2l1_to_pe_addr(bank, s + i);

                    if ((pe_addr & 0x3u) != 0)
                    {
                        LOG_ERROR(LOG_PREFIX ": unaligned program addr 0x%08x (s=0x%08x i=0x%08x)",
                                  pe_addr, (unsigned)s, (unsigned)i);
                        ra2l1_exit_pe(t);
                        return ERROR_FAIL;
                    }

                    int rr = ra2l1_program(bank, pe_addr, tmp, 4);
                    if (rr)
                    {
                        ra2l1_exit_pe(t);
                        return rr;
                    }
                }

                i += step;
            }

            r = ra2l1_seq_sync(t);
            if (r)
            {
                ra2l1_exit_pe(t);
                return r;
            }
        }
    }

    return ra2l1_exit_pe(t);
}

// Scan result summary output (for debugging)
static void ra2l1_debug_dump_scan(const struct flash_bank *bank,
                                  const uint8_t *mark,
                                  uint32_t first_blk, uint32_t last_blk,
                                  uint32_t blk_sz, uint32_t write_off, uint32_t write_cnt)
{
    uint32_t total_blks = last_blk - first_blk + 1;
    uint32_t need_upd = 0, same_cnt = 0;
    for (uint32_t b = first_blk; b <= last_blk; ++b)
    {
        if (mark[b])
            need_upd++;
        else
            same_cnt++;
    }

    LOG_INFO(LOG_PREFIX ": prescan: base=0x%08x, write_off=0x%08x, write_cnt=0x%08x, blk_sz=%u, blocks=[%u..%u], total=%u, need_upd=%u, same=%u",
             (unsigned)bank->base, (unsigned)write_off, (unsigned)write_cnt,
             (unsigned)blk_sz, (unsigned)first_blk, (unsigned)last_blk,
             (unsigned)total_blks, (unsigned)need_upd, (unsigned)same_cnt);

    const uint32_t MAX_SEG_PRINT = 32;
    uint32_t seg_printed = 0;

    uint32_t b = first_blk;
    while (b <= last_blk)
    {
        while (b <= last_blk && mark[b] == 0)
            b++;
        if (b > last_blk)
            break;
        uint32_t seg_first = b;
        while (b <= last_blk && mark[b] == 1)
            b++;
        uint32_t seg_last = b - 1;

        if (seg_printed < MAX_SEG_PRINT)
        {
            uint32_t off0 = seg_first * blk_sz;
            uint32_t off1 = (seg_last + 1) * blk_sz - 1;
            uint32_t len = (seg_last - seg_first + 1) * blk_sz;
            LOG_DEBUG(LOG_PREFIX ": prescan seg[%u]: blocks=[%u..%u], off=[0x%08x..0x%08x], len=0x%08x",
                      (unsigned)seg_printed, (unsigned)seg_first, (unsigned)seg_last,
                      (unsigned)off0, (unsigned)off1, (unsigned)len);
        }
        seg_printed++;
    }

    if (seg_printed > MAX_SEG_PRINT)
    {
        LOG_INFO(LOG_PREFIX ": prescan: segments total=%u (printed first %u only)",
                 (unsigned)seg_printed, (unsigned)MAX_SEG_PRINT);
    }
    else
    {
        LOG_INFO(LOG_PREFIX ": prescan: segments total=%u", (unsigned)seg_printed);
    }
}

// Write: pre-scan into segments first, then process per segment; clock forced to MOCO 8MHz first
static int ra2l1_flash_write(struct flash_bank *bank,
                             const uint8_t *buf,
                             uint32_t offset,
                             uint32_t count)
{
    struct target *t = bank->target;
    struct ra2l1_flash_bank *info = bank->driver_priv;
    if (t->state != TARGET_HALTED)
        return ERROR_TARGET_NOT_HALTED;

    // LOG_INFO(LOG_PREFIX ": write enter: base=0x%08x, offset=0x%08x, count=0x%08x, is_data=%d",
    //          (unsigned)bank->base, (unsigned)offset, (unsigned)count, info->is_data ? 1 : 0);

    (void)ra2l1_force_moco(t);

    uint32_t blk = info->is_data ? 1024u : 2048u;
    uint32_t first_blk = offset / blk;
    uint32_t last_blk = (offset + count - 1) / blk;
    uint32_t num_blks = last_blk + 1;

    LOG_DEBUG(LOG_PREFIX ": calc: blk_sz=%u, first_blk=%u, last_blk=%u, mark_slots=%u",
             (unsigned)blk, (unsigned)first_blk, (unsigned)last_blk, (unsigned)num_blks);

    uint8_t *mark = (uint8_t *)calloc(num_blks, 1);
    if (!mark)
    {
        LOG_ERROR(LOG_PREFIX ": calloc(mark, %u) failed", (unsigned)num_blks);
        return ERROR_FAIL;
    }

    int r = ra2l1_prescan_mark_blocks(bank, buf, offset, count, mark);
    if (r)
    {
        LOG_ERROR(LOG_PREFIX ": prescan failed: %d", r);
        free(mark);
        return r;
    }

    ra2l1_debug_dump_scan(bank, mark, first_blk, last_blk, blk, offset, count);

    uint32_t b = first_blk;
    uint32_t seg_idx = 0;
    int overall_ret = ERROR_OK;

    while (b <= last_blk)
    {
        while (b <= last_blk && mark[b] == 0)
            b++;
        if (b > last_blk)
            break;

        uint32_t seg_first = b;
        while (b <= last_blk && mark[b] == 1)
            b++;
        uint32_t seg_last = b - 1;

        uint32_t seg_off = seg_first * blk;
        uint32_t seg_len = (seg_last - seg_first + 1) * blk;
        uint32_t seg_end = seg_off + seg_len - 1;

        LOG_DEBUG(LOG_PREFIX ": SEG[%u] begin: blocks=[%u..%u], off=[0x%08x..0x%08x], len=0x%08x",
                  (unsigned)seg_idx, (unsigned)seg_first, (unsigned)seg_last,
                  (unsigned)seg_off, (unsigned)seg_end, (unsigned)seg_len);

        int64_t t0 = timeval_ms();
        r = ra2l1_process_segment(bank, seg_first, seg_last, buf, offset, count);
        int64_t t1 = timeval_ms();

        if (r != ERROR_OK)
        {
            LOG_ERROR(LOG_PREFIX ": SEG[%u] failed (r=%d): blocks=[%u..%u], off=[0x%08x..0x%08x]",
                      (unsigned)seg_idx, r, (unsigned)seg_first, (unsigned)seg_last,
                      (unsigned)seg_off, (unsigned)seg_end);
            overall_ret = r;
            break;
        }

        LOG_DEBUG(LOG_PREFIX ": SEG[%u] done: elapsed=%lld ms",
                  (unsigned)seg_idx, (long long)(t1 - t0));

        LOG_INFO(LOG_PREFIX ": SEG[%u] blocks=[%u..%u] elapsed=%lld ms",
                 (unsigned)seg_idx, (unsigned)seg_first, (unsigned)seg_last, (long long)(t1 - t0));

        seg_idx++;
    }

    free(mark);

    if (overall_ret == ERROR_OK)
    {
        LOG_DEBUG(LOG_PREFIX ": write exit: OK (segments=%u)", (unsigned)seg_idx);
    }
    else
    {
        LOG_ERROR(LOG_PREFIX ": write exit: ERR=%d (processed_segments=%u)", overall_ret, (unsigned)seg_idx);
    }

    return overall_ret;
}

// Read
static int ra2l1_flash_read(struct flash_bank *bank, uint8_t *readbuf,
                            uint32_t offset, uint32_t count)
{
    return target_read_memory(bank->target, bank->base + offset, 1, count, readbuf);
}

// Erase interface (here delegate to write()'s segment flow, empty implementation is fine)
static int ra2l1_flash_erase(struct flash_bank *bank, unsigned first, unsigned last)
{
    // To reduce P/E entry/exit times, actual erase done in write path
    return ERROR_OK;
}

// Probe
static int ra2l1_flash_probe(struct flash_bank *bank)
{
    struct ra2l1_flash_bank *info = calloc(1, sizeof(*info));
    if (!info)
        return ERROR_FAIL;
    bank->driver_priv = info;

    if (bank->base == 0x40100000)
    {
        info->is_data = true;
        info->pe_base = 0xFE000000;
        bank->size = 8 * 1024;
        bank->num_sectors = 8;
    }
    else
    {
        info->is_data = false;
        info->pe_base = 0x00000000;
        bank->size = 256 * 1024;
        bank->num_sectors = bank->size / 2048;
    }

    bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
    if (!bank->sectors)
        return ERROR_FAIL;

    for (unsigned i = 0; i < bank->num_sectors; i++)
    {
        bank->sectors[i].offset = i * (info->is_data ? 1024 : 2048);
        bank->sectors[i].size = (info->is_data ? 1024 : 2048);
    }

    if (!info->is_data)
        ra2l1_read_chip_info(bank->target);

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

// OpenOCD commands
FLASH_BANK_COMMAND_HANDLER(ra2l1_flash_bank_command)
{
    // flash bank <name> ra2l1 <base> <size> <chip_width> <bus_width> <target>
    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;
    bank->base = strtoull(CMD_ARGV[1], NULL, 0);
    bank->size = strtoul(CMD_ARGV[2], NULL, 0);
    bank->chip_width = strtol(CMD_ARGV[3], NULL, 0);
    bank->bus_width = strtol(CMD_ARGV[4], NULL, 0);
    bank->target = get_target(CMD_ARGV[5]);
    if (!bank->target)
        return ERROR_FAIL;
    return ERROR_OK;
}

static const struct command_registration ra2l1_exec_command_handlers[] = {
    COMMAND_REGISTRATION_DONE};

const struct flash_driver ra2l1_flash = {
    .name = "ra2l1",
    .commands = ra2l1_exec_command_handlers,
    .flash_bank_command = ra2l1_flash_bank_command,
    .erase = ra2l1_flash_erase,
    .protect = NULL,
    .write = ra2l1_flash_write,
    .read = ra2l1_flash_read,
    .probe = ra2l1_flash_probe,
    .auto_probe = ra2l1_flash_auto_probe,
    .erase_check = default_flash_blank_check,
    .info = NULL,
};