// SPDX-License-Identifier: GPL-2.0-or-later

#define OPENOCD_CONTRIB_LOADERS_FLASH_SWM341_SFC

#include <stdint.h>
#include "../../../../src/flash/nor/swm341_sfc.h"

void entry(struct swm341_sfc_loader_cfg *cfg,
		const uint8_t *src,
		uint32_t address,
		uint32_t count)
{
	volatile uint32_t *sfc_cfg = (volatile uint32_t *)cfg->sfc_cfg_addr;
	volatile uint32_t *sfc_sr = (volatile uint32_t *)cfg->sfc_sr_addr;
	volatile uint32_t *dst = (volatile uint32_t *)address;
	const uint32_t *src32 = (const uint32_t *)src;

	cfg->last_error = 0;

	if ((address & 3u) || (count & 3u) || (cfg->page_size == 0u)) {
		cfg->last_error = 1;
		__asm("bkpt 0");
		return;
	}

	__asm("cpsid i");

	while (count) {
		uint32_t page_off = (address - cfg->map_base) & (cfg->page_size - 1u);
		uint32_t this_size = cfg->page_size - page_off;
		if (this_size > count)
			this_size = count;

		*sfc_cfg |= SWM341_SFC_CFG_WREN;

		for (uint32_t i = 0; i < this_size / 4u; i++)
			dst[i] = src32[i];

		while ((*sfc_sr & SWM341_SFC_SR_BUSY) != 0u)
			;

		*sfc_cfg &= ~SWM341_SFC_CFG_WREN;

		dst += this_size / 4u;
		src32 += this_size / 4u;
		address += this_size;
		count -= this_size;
	}

	__asm("cpsie i");
	__asm("bkpt 0");
}
