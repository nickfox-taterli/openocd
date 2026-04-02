/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_RENESAS_RA2L1_H
#define OPENOCD_FLASH_NOR_RENESAS_RA2L1_H

/* IMPORTANT: this file is shared by the RA2L1 flash driver and flash loader. */

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif

#define RA2L1_CODE_BASE              0x00000000u
#define RA2L1_CODE_SIZE              0x00040000u
#define RA2L1_CODE_ERASE_BLOCK_SIZE  0x00000800u
#define RA2L1_CODE_PROGRAM_UNIT      4u

#define RA2L1_DATA_BASE              0x40100000u
#define RA2L1_DATA_PE_BASE           0xFE000000u
#define RA2L1_DATA_SIZE              0x00002000u
#define RA2L1_DATA_ERASE_BLOCK_SIZE  0x00000400u
#define RA2L1_DATA_PROGRAM_UNIT      1u

#define RA2L1_FLCN_BASE              0x407EC000u
#define RA2L1_REG_DFLCTL             (RA2L1_FLCN_BASE + 0x0090u)
#define RA2L1_REG_FPMCR              (RA2L1_FLCN_BASE + 0x0100u)
#define RA2L1_REG_FSARL              (RA2L1_FLCN_BASE + 0x0108u)
#define RA2L1_REG_FSARH              (RA2L1_FLCN_BASE + 0x0110u)
#define RA2L1_REG_FCR                (RA2L1_FLCN_BASE + 0x0114u)
#define RA2L1_REG_FEARL              (RA2L1_FLCN_BASE + 0x0118u)
#define RA2L1_REG_FEARH              (RA2L1_FLCN_BASE + 0x0120u)
#define RA2L1_REG_FRESETR            (RA2L1_FLCN_BASE + 0x0124u)
#define RA2L1_REG_FSTATR1            (RA2L1_FLCN_BASE + 0x012Cu)
#define RA2L1_REG_FWBL0              (RA2L1_FLCN_BASE + 0x0130u)
#define RA2L1_REG_FWBH0              (RA2L1_FLCN_BASE + 0x0138u)
#define RA2L1_REG_FPR                (RA2L1_FLCN_BASE + 0x0180u)
#define RA2L1_REG_FSTATR2            (RA2L1_FLCN_BASE + 0x01F0u)
#define RA2L1_REG_FENTRYR            (RA2L1_FLCN_BASE + 0x3FB2u)
#define RA2L1_REG_PFBER              (RA2L1_FLCN_BASE + 0x3FC8u)

#define RA2L1_FSTATR1_FRDY           BIT(6)
#define RA2L1_FSTATR2_ERR_MASK       (BIT(0) | BIT(1) | BIT(2) | BIT(4) | BIT(5))

#define RA2L1_FENTRYR_CODE_PE        0xAA01u
#define RA2L1_FENTRYR_DATA_PE        0xAA80u
#define RA2L1_FENTRYR_READ           0xAA00u

#define RA2L1_FPMCR_CODE_PE          0x02u
#define RA2L1_FPMCR_DATA_PE          0x10u
#define RA2L1_FPMCR_READ             0x08u

#define RA2L1_FCR_ERASE_PREP         0x84u
#define RA2L1_FCR_ERASE_EXEC         0x04u
#define RA2L1_FCR_PROG_PREP          0x81u
#define RA2L1_FCR_PROG_EXEC          0x01u

#define RA2L1_LOADER_STACK_SIZE      128u

struct ra2l1_loader_work_area {
	uint32_t program_unit;
	uint8_t stack[RA2L1_LOADER_STACK_SIZE];
	struct flash_async_algorithm_circbuf {
#ifdef OPENOCD_CONTRIB_LOADERS_FLASH_RENESAS_RA2L1
		uint8_t *wp;
		uint8_t *rp;
#else
		uint32_t wp;
		uint32_t rp;
#endif
	} fifo;
};

#endif /* OPENOCD_FLASH_NOR_RENESAS_RA2L1_H */
