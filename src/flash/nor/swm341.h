/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_SWM341_H
#define OPENOCD_FLASH_NOR_SWM341_H

#define SWM341_FLASH_BASE                    0x00000000u
#define SWM341_FLASH_SIZE                    0x00080000u
#define SWM341_FLASH_SECTOR_SIZE             0x00001000u
#define SWM341_FLASH_WRITE_ALIGN             16u

#define SWM341_FMC_BASE                      0x4004A000u
#define SWM341_FMC_CACHE                     (SWM341_FMC_BASE + 0x0Cu)
#define SWM341_FMC_STAT                      (SWM341_FMC_BASE + 0x24u)

#define SWM341_FMC_CACHE_PROGEN              (1u << 0)
#define SWM341_FMC_CACHE_CCLR                (1u << 18)

#define SWM341_FMC_STAT_PROGBUSY             (1u << 1)
#define SWM341_FMC_STAT_IDLE                 (1u << 31)

#define SWM341_IAP_MAGIC                     0x0B11FFACu
#define SWM341_IAP_FLASH_PARAM_CFG0_150MHZ   0x00016589u
#define SWM341_IAP_FLASH_PARAM_CFG1_150MHZ   0x00004C74u

#define SWM341_IAP_CACHE_RESET_ADDR          0x11000401u
#define SWM341_IAP_FLASH_PARAM_ADDR          0x11000431u
#define SWM341_IAP_FLASH_ERASE_ADDR          0x11000471u
#define SWM341_IAP_FLASH_WRITE_ADDR          0x110004C1u

#define SWM341_LOADER_STACK_SIZE             256u

#define SWM341_LOADER_OP_WRITE               1u
#define SWM341_LOADER_OP_ERASE               2u

struct swm341_loader_cfg {
	uint32_t op;
	uint32_t magic;
	uint32_t iap_cache_reset;
	uint32_t iap_flash_param;
	uint32_t iap_flash_erase;
	uint32_t iap_flash_write;
	uint32_t flash_param_cfg0;
	uint32_t flash_param_cfg1;
	uint32_t cache_reg_addr;
	uint32_t cache_cclr_mask;
	uint32_t last_error;
	uint8_t stack[SWM341_LOADER_STACK_SIZE];
};

#endif /* OPENOCD_FLASH_NOR_SWM341_H */
