/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_RENESAS_RA6M5_QSPI_H
#define OPENOCD_FLASH_NOR_RENESAS_RA6M5_QSPI_H

#define RA6M5_QSPI_MAP_BASE                 0x60000000u
#define RA6M5_QSPI_DEFAULT_SIZE             0x04000000u

#define RA6M5_QSPI_BASE                     0x64000000u
#define RA6M5_SYSTEM_PRCR                   0x4001E3FEu
#define RA6M5_MSTPCRB                       0x40084004u
#define RA6M5_PMISC_PWPR                    0x40080D03u
#define RA6M5_PFS_P305                      0x400808D4u
#define RA6M5_PFS_P306                      0x400808D8u
#define RA6M5_PFS_P307                      0x400808DCu
#define RA6M5_PFS_P308                      0x400808E0u
#define RA6M5_PFS_P309                      0x400808E4u
#define RA6M5_PFS_P310                      0x400808E8u

#define RA6M5_QSPI_REG_SFMSMD               (RA6M5_QSPI_BASE + 0x0000u)
#define RA6M5_QSPI_REG_SFMSSC               (RA6M5_QSPI_BASE + 0x0004u)
#define RA6M5_QSPI_REG_SFMSKC               (RA6M5_QSPI_BASE + 0x0008u)
#define RA6M5_QSPI_REG_SFMPMD               (RA6M5_QSPI_BASE + 0x0018u)
#define RA6M5_QSPI_REG_SFMSIC               (RA6M5_QSPI_BASE + 0x001Cu)
#define RA6M5_QSPI_REG_SFMCST               (RA6M5_QSPI_BASE + 0x0020u)
#define RA6M5_QSPI_REG_SFMSPC               (RA6M5_QSPI_BASE + 0x0030u)
#define RA6M5_QSPI_REG_SFMCOM               (RA6M5_QSPI_BASE + 0x0010u)
#define RA6M5_QSPI_REG_SFMCMD               (RA6M5_QSPI_BASE + 0x0014u)
#define RA6M5_QSPI_REG_SFMSAC               (RA6M5_QSPI_BASE + 0x0024u)
#define RA6M5_QSPI_REG_SFMCNT1              (RA6M5_QSPI_BASE + 0x0804u)

#define RA6M5_QSPI_SFMSAC_SFMAS_MASK        0x3u
#define RA6M5_QSPI_ADDR_BYTES_3             2u
#define RA6M5_QSPI_ADDR_BYTES_4             3u

#define RA6M5_QSPI_DEFAULT_PAGE_SIZE        256u
#define RA6M5_QSPI_DEFAULT_SECTOR_SIZE      4096u

#define RA6M5_QSPI_CMD_WRITE_ENABLE         0x06u
#define RA6M5_QSPI_CMD_READ_STATUS          0x05u
#define RA6M5_QSPI_CMD_PAGE_PROGRAM         0x02u
#define RA6M5_QSPI_CMD_SECTOR_ERASE         0x20u

#define RA6M5_QSPI_BUSY_BIT                 0

#define RA6M5_QSPI_TIMEOUT_STATUS_MS        3000u
#define RA6M5_QSPI_TIMEOUT_ERASE_MS         4000u
#define RA6M5_QSPI_LOADER_STACK_SIZE        256u

struct ra6m5_qspi_loader_cfg {
	uint32_t page_size;
	uint32_t address_bytes;
	uint32_t write_enable_cmd;
	uint32_t page_program_cmd;
	uint32_t status_cmd;
	uint32_t busy_bit_mask;
	uint32_t status_timeout_loops;
	uint32_t last_error;
	uint8_t stack[RA6M5_QSPI_LOADER_STACK_SIZE];
};

#endif /* OPENOCD_FLASH_NOR_RENESAS_RA6M5_QSPI_H */
