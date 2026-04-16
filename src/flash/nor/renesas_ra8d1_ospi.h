/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_RENESAS_RA8D1_OSPI_H
#define OPENOCD_FLASH_NOR_RENESAS_RA8D1_OSPI_H

#define RA8D1_OSPI_MAP_CS0_BASE                 0x80000000u
#define RA8D1_OSPI_MAP_CS1_BASE                 0x90000000u

#define RA8D1_OSPI_DEFAULT_SIZE                 0x00800000u
#define RA8D1_OSPI_DEFAULT_PAGE_SIZE            256u
#define RA8D1_OSPI_DEFAULT_SECTOR_SIZE          4096u
#define RA8D1_OSPI_LOADER_STACK_SIZE            256u

#define RA8D1_OSPI_MSTPCRB                      0x40203004u
#define RA8D1_OSPI_PWPRS                        0x40400D14u

#define RA8D1_OSPI_PFS_P100                     0x40400840u
#define RA8D1_OSPI_PFS_P101                     0x40400844u
#define RA8D1_OSPI_PFS_P103                     0x4040084Cu
#define RA8D1_OSPI_PFS_P104                     0x40400850u
#define RA8D1_OSPI_PFS_P106                     0x40400858u
#define RA8D1_OSPI_PFS_P107                     0x4040085Cu
#define RA8D1_OSPI_PFS_P803                     0x40400A0Cu
#define RA8D1_OSPI_PFS_P808                     0x40400A20u

#define RA8D1_OSPI_XSPI_BASE                    0x40268000u
#define RA8D1_OSPI_REG_LIOCFGCS0                (RA8D1_OSPI_XSPI_BASE + 0x0050u)
#define RA8D1_OSPI_REG_LIOCFGCS1                (RA8D1_OSPI_XSPI_BASE + 0x0054u)
#define RA8D1_OSPI_REG_CDCTL0                   (RA8D1_OSPI_XSPI_BASE + 0x0070u)
#define RA8D1_OSPI_REG_CDBUF0_CDT               (RA8D1_OSPI_XSPI_BASE + 0x0080u)
#define RA8D1_OSPI_REG_CDBUF0_CDA               (RA8D1_OSPI_XSPI_BASE + 0x0084u)
#define RA8D1_OSPI_REG_CDBUF0_CDD0              (RA8D1_OSPI_XSPI_BASE + 0x0088u)
#define RA8D1_OSPI_REG_CDBUF0_CDD1              (RA8D1_OSPI_XSPI_BASE + 0x008Cu)
#define RA8D1_OSPI_REG_INTS                     (RA8D1_OSPI_XSPI_BASE + 0x0190u)
#define RA8D1_OSPI_REG_INTC                     (RA8D1_OSPI_XSPI_BASE + 0x0194u)

#define RA8D1_OSPI_LIOCFG_DEFAULT               0x00070000u

#define RA8D1_OSPI_CDCTL0_TRREQ                 (1u << 0)
#define RA8D1_OSPI_CDCTL0_CSSEL_SHIFT           3u

#define RA8D1_OSPI_CDT_CMDSIZE_SHIFT            0u
#define RA8D1_OSPI_CDT_ADDSIZE_SHIFT            2u
#define RA8D1_OSPI_CDT_DATASIZE_SHIFT           5u
#define RA8D1_OSPI_CDT_LATE_SHIFT               9u
#define RA8D1_OSPI_CDT_TRTYPE_SHIFT             15u
#define RA8D1_OSPI_CDT_CMD_SHIFT                16u

#define RA8D1_OSPI_TRTYPE_READ                  0u
#define RA8D1_OSPI_TRTYPE_WRITE                 1u

#define RA8D1_OSPI_CMD_WRITE_ENABLE             0x06u
#define RA8D1_OSPI_CMD_READ_STATUS              0x05u
#define RA8D1_OSPI_CMD_READ_ID                  0x9Fu
#define RA8D1_OSPI_CMD_READ_DATA                0x03u
#define RA8D1_OSPI_CMD_PAGE_PROGRAM             0x02u
#define RA8D1_OSPI_CMD_SECTOR_ERASE             0x20u
#define RA8D1_OSPI_CMD_CHIP_ERASE               0xC7u

#define RA8D1_OSPI_BUSY_BIT                     0
#define RA8D1_OSPI_LOADER_OP_WRITE              0u
#define RA8D1_OSPI_LOADER_OP_READ               1u

#define RA8D1_OSPI_TIMEOUT_TRREQ_MS             200u
#define RA8D1_OSPI_TIMEOUT_STATUS_MS            3000u
#define RA8D1_OSPI_TIMEOUT_SECTOR_ERASE_MS      6000u
#define RA8D1_OSPI_TIMEOUT_CHIP_ERASE_MS        180000u

struct ra8d1_ospi_loader_cfg {
	uint32_t page_size;
	uint32_t status_timeout_loops;
	uint32_t cmd_write_enable;
	uint32_t cmd_page_program;
	uint32_t cmd_status;
	uint32_t cmd_read_data;
	uint32_t busy_bit_mask;
	uint32_t operation;
	uint32_t cs_setup;
	uint32_t last_error;
	uint8_t stack[RA8D1_OSPI_LOADER_STACK_SIZE];
};

#endif /* OPENOCD_FLASH_NOR_RENESAS_RA8D1_OSPI_H */
