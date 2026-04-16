// SPDX-License-Identifier: GPL-2.0-or-later

#define OPENOCD_CONTRIB_LOADERS_FLASH_SWM341

#include <stdint.h>
#include "../../../../src/flash/nor/swm341.h"

typedef int (*iap_flash_param_t)(uint32_t cfg0, uint32_t cfg1, uint32_t magic);
typedef int (*iap_flash_erase_t)(uint32_t sector, uint32_t magic);
typedef int (*iap_flash_write_t)(uint32_t flash_addr, uint32_t ram_addr, uint32_t count, uint32_t magic);
typedef void (*iap_cache_reset_t)(uint32_t cfg, uint32_t magic);

void entry(struct swm341_loader_cfg *cfg,
		const uint8_t *src,
		uint32_t address,
		uint32_t count)
{
	cfg->last_error = 0;

	__asm("cpsid i");

	iap_cache_reset_t iap_cache_reset = (iap_cache_reset_t)cfg->iap_cache_reset;
	iap_flash_param_t iap_flash_param = (iap_flash_param_t)cfg->iap_flash_param;
	iap_flash_erase_t iap_flash_erase = (iap_flash_erase_t)cfg->iap_flash_erase;
	iap_flash_write_t iap_flash_write = (iap_flash_write_t)cfg->iap_flash_write;

	(void)iap_flash_param(cfg->flash_param_cfg0, cfg->flash_param_cfg1, cfg->magic);

	if (cfg->op == SWM341_LOADER_OP_ERASE) {
		uint32_t first = address;
		uint32_t last = count;
		for (uint32_t s = first; s <= last; s++) {
			if (iap_flash_erase(s, cfg->magic) != 0) {
				cfg->last_error = 1;
				break;
			}
		}
	} else if (cfg->op == SWM341_LOADER_OP_WRITE) {
		if ((address & (SWM341_FLASH_WRITE_ALIGN - 1)) || (count & (SWM341_FLASH_WRITE_ALIGN - 1))) {
			cfg->last_error = 2;
		} else {
			uint32_t count16 = count / SWM341_FLASH_WRITE_ALIGN;
			if (iap_flash_write(address, (uint32_t)src, count16, cfg->magic) != 0)
				cfg->last_error = 3;
		}
	} else {
		cfg->last_error = 4;
	}

	if (cfg->last_error == 0) {
		volatile uint32_t *cache = (volatile uint32_t *)cfg->cache_reg_addr;
		iap_cache_reset(*cache | cfg->cache_cclr_mask, cfg->magic);
	}

	__asm("cpsie i");
	__asm("bkpt 0");
}
