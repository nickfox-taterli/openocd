// SPDX-License-Identifier: GPL-2.0-or-later

#define OPENOCD_CONTRIB_LOADERS_FLASH_RENESAS_RA6M5

#include <stdint.h>
#include "../../../../src/flash/nor/renesas_ra6m5.h"

static inline __attribute__((always_inline))
void program_one_unit(uint32_t program_unit, uint32_t target_address, const uint8_t *src)
{
	volatile uint32_t *fsaddr = (volatile uint32_t *)RA6M5_REG_FSADDR;
	volatile uint32_t *fstatr = (volatile uint32_t *)RA6M5_REG_FSTATR;
	volatile uint8_t *faci_cmd = (volatile uint8_t *)RA6M5_FACI_CMD_AREA;
	volatile uint16_t *faci_data = (volatile uint16_t *)RA6M5_FACI_CMD_AREA;

	*fsaddr = target_address;
	*faci_cmd = RA6M5_FACI_CMD_PROGRAM;
	*faci_cmd = (uint8_t)(program_unit / 2);

	for (uint32_t i = 0; i < program_unit; i += 2) {
		uint16_t v = (uint16_t)src[i] | ((uint16_t)src[i + 1] << 8);
		*faci_data = v;
		while ((*fstatr & RA6M5_FSTATR_DBFULL) != 0)
			;
	}

	*faci_cmd = RA6M5_FACI_CMD_EXECUTE;
}

void write(struct ra6m5_loader_work_area *work_area,
		uint8_t *fifo_end,
		uint32_t target_address,
		uint32_t count)
{
	__asm("cpsid i");

	volatile uint32_t *fstatr = (volatile uint32_t *)RA6M5_REG_FSTATR;
	volatile uint8_t *fastat = (volatile uint8_t *)RA6M5_REG_FASTAT;
	volatile uint8_t *faci_cmd = (volatile uint8_t *)RA6M5_FACI_CMD_AREA;
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
			program_one_unit(program_unit, target_address, rp_cache);

			while ((*fstatr & RA6M5_FSTATR_FRDY) == 0)
				;

			if ((*fstatr & RA6M5_FSTATR_ERR_MASK) || (*fastat & RA6M5_FASTAT_CMDLK)) {
				*faci_cmd = RA6M5_FACI_CMD_STATUS_CLEAR;
				while ((*fstatr & RA6M5_FSTATR_FRDY) == 0)
					;
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
	__asm("cpsie i");
	__asm("bkpt 0");
}
