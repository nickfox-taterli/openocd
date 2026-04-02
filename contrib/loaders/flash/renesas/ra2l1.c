// SPDX-License-Identifier: GPL-2.0-or-later

#define OPENOCD_CONTRIB_LOADERS_FLASH_RENESAS_RA2L1

#include <stdint.h>
#include "../../../../src/flash/nor/renesas_ra2l1.h"

static inline __attribute__((always_inline))
void write_program_data(uint32_t program_unit, uint32_t target_address, const uint8_t *src)
{
	volatile uint16_t *fsarh = (volatile uint16_t *)RA2L1_REG_FSARH;
	volatile uint16_t *fsarl = (volatile uint16_t *)RA2L1_REG_FSARL;
	volatile uint32_t *fwbl0 = (volatile uint32_t *)RA2L1_REG_FWBL0;
	volatile uint32_t *fwbh0 = (volatile uint32_t *)RA2L1_REG_FWBH0;

	*fsarh = (uint16_t)(target_address >> 16);
	*fsarl = (uint16_t)target_address;

	if (program_unit == 4) {
		uint32_t value = (uint32_t)src[0] |
			((uint32_t)src[1] << 8) |
			((uint32_t)src[2] << 16) |
			((uint32_t)src[3] << 24);
		*fwbl0 = value & 0xFFFF;
		*fwbh0 = value >> 16;
	} else {
		*fwbl0 = src[0];
	}
}

void write(struct ra2l1_loader_work_area *work_area,
		uint8_t *fifo_end,
		uint32_t target_address,
		uint32_t count)
{
	volatile uint32_t *fstatr1 = (volatile uint32_t *)RA2L1_REG_FSTATR1;
	volatile uint32_t *fstatr2 = (volatile uint32_t *)RA2L1_REG_FSTATR2;
	volatile uint8_t *fcr = (volatile uint8_t *)RA2L1_REG_FCR;
	volatile uint32_t *fresetr = (volatile uint32_t *)RA2L1_REG_FRESETR;
	uint32_t program_unit = work_area->program_unit;
	uint8_t *rp_cache = work_area->fifo.rp;
	uint8_t *fifo_start = rp_cache;

	while (count) {
		uint8_t *wp_cache = work_area->fifo.wp;
		if (wp_cache == 0)
			break;

		int32_t fifo_size = wp_cache - rp_cache;
		if (fifo_size < 0)
			fifo_size = fifo_end - rp_cache;

		while (fifo_size >= (int32_t)program_unit) {
			write_program_data(program_unit, target_address, rp_cache);

			*fcr = RA2L1_FCR_PROG_PREP;
			while ((*fstatr1 & RA2L1_FSTATR1_FRDY) == 0)
				;

			*fcr = RA2L1_FCR_PROG_EXEC;
			*fcr = 0;
			while (*fstatr1 & RA2L1_FSTATR1_FRDY)
				;

			if (*fstatr2 & RA2L1_FSTATR2_ERR_MASK) {
				*fresetr = 1;
				*fresetr = 0;
				work_area->fifo.rp = 0;
				goto out;
			}

			target_address += program_unit;
			rp_cache += program_unit;
			if (rp_cache >= fifo_end)
				rp_cache = fifo_start;
			work_area->fifo.rp = rp_cache;

			fifo_size -= program_unit;
			count--;
		}
	}

out:
	__asm("bkpt 0");
}
