/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_RENESAS_RA8D1_H
#define OPENOCD_FLASH_NOR_RENESAS_RA8D1_H

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif

/* RA8D1 (R7FA8D1B*) from BSP/FSP generated headers. */
#define RA8D1_CODE_BASE              0x02000000u
#define RA8D1_CODE_SIZE              0x001F8000u
#define RA8D1_CODE_ERASE_SMALL       0x00002000u
#define RA8D1_CODE_ERASE_LARGE       0x00008000u
#define RA8D1_CODE_SMALL_AREA_SIZE   0x00010000u
#define RA8D1_CODE_PROGRAM_UNIT      128u

#define RA8D1_DATA_BASE              0x27000000u
#define RA8D1_DATA_SIZE              0x00003000u
#define RA8D1_DATA_ERASE_BLOCK_SIZE  64u
#define RA8D1_DATA_PROGRAM_UNIT      4u

#define RA8D1_REG_FWEPROR            0x4001EA54u
#define RA8D1_REG_FCACHEE            0x4001C100u

#define RA8D1_FACI_BASE              0x4011E000u
#define RA8D1_REG_FASTAT             (RA8D1_FACI_BASE + 0x0010u)
#define RA8D1_REG_FSADDR             (RA8D1_FACI_BASE + 0x0030u)
#define RA8D1_REG_FEADDR             (RA8D1_FACI_BASE + 0x0034u)
#define RA8D1_REG_FMEPROT            (RA8D1_FACI_BASE + 0x0044u)
#define RA8D1_REG_FSTATR             (RA8D1_FACI_BASE + 0x0080u)
#define RA8D1_REG_FENTRYR            (RA8D1_FACI_BASE + 0x0084u)
#define RA8D1_REG_FCMDR              (RA8D1_FACI_BASE + 0x00A0u)

#define RA8D1_FACI_CMD_AREA          0x40100000u

#define RA8D1_FENTRYR_CODE_PE        0xAA01u
#define RA8D1_FENTRYR_DATA_PE        0xAA80u
#define RA8D1_FENTRYR_READ           0xAA00u

#define RA8D1_FWEPROR_ENABLE         0x01u
#define RA8D1_FWEPROR_DISABLE        0x02u
#define RA8D1_FMEPROT_UNLOCK         0xD900u
#define RA8D1_FMEPROT_LOCK           0xD901u

#define RA8D1_FASTAT_CMDLK           BIT(4)
#define RA8D1_FSTATR_DBFULL          BIT(10)
#define RA8D1_FSTATR_FRDY            BIT(15)
#define RA8D1_FSTATR_ERR_MASK        0x00F07040u

#define RA8D1_FACI_CMD_PROGRAM       0xE8u
#define RA8D1_FACI_CMD_CODE_ERASE    0x20u
#define RA8D1_FACI_CMD_DATA_ERASE    0x21u
#define RA8D1_FACI_CMD_STATUS_CLEAR  0x50u
#define RA8D1_FACI_CMD_EXECUTE       0xD0u

#endif /* OPENOCD_FLASH_NOR_RENESAS_RA8D1_H */
