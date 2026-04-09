/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_RENESAS_RA6M5_H
#define OPENOCD_FLASH_NOR_RENESAS_RA6M5_H

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif

#define RA6M5_CODE_BASE              0x00000000u
#define RA6M5_CODE_SIZE              0x00200000u
#define RA6M5_CODE_ERASE_SMALL       0x00002000u
#define RA6M5_CODE_ERASE_LARGE       0x00008000u
#define RA6M5_CODE_SMALL_AREA_SIZE   0x00010000u
#define RA6M5_CODE_PROGRAM_UNIT      128u

#define RA6M5_DATA_BASE              0x08000000u
#define RA6M5_DATA_SIZE              0x00002000u
#define RA6M5_DATA_ERASE_BLOCK_SIZE  64u
#define RA6M5_DATA_PROGRAM_UNIT      4u

#define RA6M5_REG_FWEPROR            0x4001E416u
#define RA6M5_REG_CC_ACTL            0x40007000u
#define RA6M5_REG_FCACHEE            0x4001C100u

#define RA6M5_FACI_BASE              0x407FE000u
#define RA6M5_REG_FASTAT             (RA6M5_FACI_BASE + 0x0010u)
#define RA6M5_REG_FSADDR             (RA6M5_FACI_BASE + 0x0030u)
#define RA6M5_REG_FEADDR             (RA6M5_FACI_BASE + 0x0034u)
#define RA6M5_REG_FMEPROT            (RA6M5_FACI_BASE + 0x0044u)
#define RA6M5_REG_FSTATR             (RA6M5_FACI_BASE + 0x0080u)
#define RA6M5_REG_FENTRYR            (RA6M5_FACI_BASE + 0x0084u)
#define RA6M5_REG_FCMDR              (RA6M5_FACI_BASE + 0x00A0u)

#define RA6M5_FACI_CMD_AREA          0x407E0000u

#define RA6M5_FENTRYR_CODE_PE        0xAA01u
#define RA6M5_FENTRYR_DATA_PE        0xAA80u
#define RA6M5_FENTRYR_READ           0xAA00u

#define RA6M5_FWEPROR_ENABLE         0x01u
#define RA6M5_FWEPROR_DISABLE        0x02u
#define RA6M5_FMEPROT_UNLOCK         0xD900u
#define RA6M5_FMEPROT_LOCK           0xD901u

#define RA6M5_FASTAT_CMDLK           BIT(4)
#define RA6M5_FSTATR_DBFULL          BIT(10)
#define RA6M5_FSTATR_FRDY            BIT(15)
#define RA6M5_FSTATR_ERR_MASK        0x00F07000u

#define RA6M5_FACI_CMD_PROGRAM       0xE8u
#define RA6M5_FACI_CMD_CODE_ERASE    0x20u
#define RA6M5_FACI_CMD_DATA_ERASE    0x21u
#define RA6M5_FACI_CMD_STATUS_CLEAR  0x50u
#define RA6M5_FACI_CMD_EXECUTE       0xD0u

#define RA6M5_LOADER_STACK_SIZE      256u

struct ra6m5_loader_work_area {
	uint32_t program_unit;
	uint8_t stack[RA6M5_LOADER_STACK_SIZE];
	struct flash_async_algorithm_circbuf {
#ifdef OPENOCD_CONTRIB_LOADERS_FLASH_RENESAS_RA6M5
		uint8_t *wp;
		uint8_t *rp;
#else
		uint32_t wp;
		uint32_t rp;
#endif
	} fifo;
};

#endif /* OPENOCD_FLASH_NOR_RENESAS_RA6M5_H */
