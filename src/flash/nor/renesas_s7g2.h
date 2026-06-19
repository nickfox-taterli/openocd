/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_RENESAS_S7G2_H
#define OPENOCD_FLASH_NOR_RENESAS_S7G2_H

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif

#define S7G2_CODE_BASE              0x00000000u
#define S7G2_CODE_SIZE_4M           0x00400000u
#define S7G2_CODE_SIZE_3M           0x00300000u
#define S7G2_CODE_ERASE_SMALL       0x00002000u
#define S7G2_CODE_ERASE_LARGE       0x00008000u
#define S7G2_CODE_SMALL_AREA_SIZE   0x00010000u
#define S7G2_CODE_PROGRAM_UNIT      256u

#define S7G2_REG_FWEPROR            0x4001E416u
#define S7G2_REG_FCACHEE            0x4001C100u

#define S7G2_FACI_BASE              0x407FE000u
#define S7G2_REG_FASTAT             (S7G2_FACI_BASE + 0x0010u)
#define S7G2_REG_FSADDR             (S7G2_FACI_BASE + 0x0030u)
#define S7G2_REG_FEADDR             (S7G2_FACI_BASE + 0x0034u)
#define S7G2_REG_FSTATR             (S7G2_FACI_BASE + 0x0080u)
#define S7G2_REG_FENTRYR            (S7G2_FACI_BASE + 0x0084u)
#define S7G2_REG_FCMDR              (S7G2_FACI_BASE + 0x00A0u)
#define S7G2_REG_FPESTAT            (S7G2_FACI_BASE + 0x00C0u)
#define S7G2_REG_FCPSR              (S7G2_FACI_BASE + 0x00D0u)
#define S7G2_REG_FPCKAR             (S7G2_FACI_BASE + 0x00E8u)

#define S7G2_FACI_CMD_AREA          0x407E0000u

#define S7G2_FENTRYR_CODE_PE        0xAA01u
#define S7G2_FENTRYR_READ           0xAA00u

#define S7G2_FWEPROR_ENABLE         0x01u
#define S7G2_FWEPROR_DISABLE        0x02u
#define S7G2_FPCKAR_KEY             0x1E00u

#define S7G2_FASTAT_CMDLK           BIT(4)
#define S7G2_FASTAT_CFAE            BIT(7)
#define S7G2_FSTATR_FHVEERR         BIT(6)
#define S7G2_FSTATR_FCUERR          BIT(7)
#define S7G2_FSTATR_DBFULL          BIT(10)
#define S7G2_FSTATR_PRGERR          BIT(12)
#define S7G2_FSTATR_ERSERR          BIT(13)
#define S7G2_FSTATR_ILGLERR         BIT(14)
#define S7G2_FSTATR_FRDY            BIT(15)
#define S7G2_FSTATR_ERR_MASK        (S7G2_FSTATR_FHVEERR | S7G2_FSTATR_FCUERR | \
		S7G2_FSTATR_PRGERR | S7G2_FSTATR_ERSERR | S7G2_FSTATR_ILGLERR)

#define S7G2_FPESTAT_PGM_ERROR      0x0002u
#define S7G2_FPESTAT_ERASE_ERROR    0x0012u

#define S7G2_FACI_CMD_PROGRAM       0xE8u
#define S7G2_FACI_CMD_PROGRAM_CF    0x80u
#define S7G2_FACI_CMD_BLOCK_ERASE   0x20u
#define S7G2_FACI_CMD_STATUS_CLEAR  0x50u
#define S7G2_FACI_CMD_FORCED_STOP   0xB3u
#define S7G2_FACI_CMD_FINAL         0xD0u

#define S7G2_LOADER_STACK_SIZE      256u

struct s7g2_loader_work_area {
	uint32_t program_unit;
	uint8_t stack[S7G2_LOADER_STACK_SIZE];
	struct flash_async_algorithm_circbuf {
#ifdef OPENOCD_CONTRIB_LOADERS_FLASH_RENESAS_S7G2
		uint8_t *wp;
		uint8_t *rp;
#else
		uint32_t wp;
		uint32_t rp;
#endif
	} fifo;
};

#endif /* OPENOCD_FLASH_NOR_RENESAS_S7G2_H */
