// SPDX-License-Identifier: GPL-2.0-or-later

#define OPENOCD_CONTRIB_LOADERS_FLASH_RENESAS_RA6M5_QSPI

#include <stdint.h>
#include "../../../../src/flash/nor/renesas_ra6m5_qspi.h"

static inline __attribute__((always_inline))
void qspi_send_addr(volatile uint8_t *sfmcom, uint32_t addr, uint32_t address_bytes)
{
	if (address_bytes == RA6M5_QSPI_ADDR_BYTES_4)
		*sfmcom = (uint8_t)(addr >> 24);

	*sfmcom = (uint8_t)(addr >> 16);
	*sfmcom = (uint8_t)(addr >> 8);
	*sfmcom = (uint8_t)addr;
}

static inline __attribute__((always_inline))
int qspi_wait_ready(const struct ra6m5_qspi_loader_cfg *cfg,
		volatile uint32_t *sfmcmd,
		volatile uint8_t *sfmcom)
{
	uint32_t timeout = cfg->status_timeout_loops;

	while (timeout--) {
		*sfmcmd = 1;
		*sfmcom = (uint8_t)cfg->status_cmd;
		uint8_t st = *sfmcom;
		*sfmcmd = 1;
		*sfmcmd = 0;
		if ((st & cfg->busy_bit_mask) == 0)
			return 0;
	}

	return -1;
}

void write(struct ra6m5_qspi_loader_cfg *cfg,
		const uint8_t *src,
		uint32_t target_address,
		uint32_t count)
{
	__asm("cpsid i");

	volatile uint32_t *sfmcmd = (volatile uint32_t *)RA6M5_QSPI_REG_SFMCMD;
	volatile uint8_t *sfmcom = (volatile uint8_t *)RA6M5_QSPI_REG_SFMCOM;

	cfg->last_error = 0;

	while (count) {
		uint32_t page_off = target_address & (cfg->page_size - 1);
		uint32_t this_size = cfg->page_size - page_off;
		if (this_size > count)
			this_size = count;

		*sfmcmd = 1;
		*sfmcom = (uint8_t)cfg->write_enable_cmd;
		*sfmcmd = 1;

		*sfmcom = (uint8_t)cfg->page_program_cmd;
		qspi_send_addr(sfmcom, target_address, cfg->address_bytes);

		for (uint32_t i = 0; i < this_size; i++)
			*sfmcom = src[i];

		*sfmcmd = 1;
		*sfmcmd = 0;

		if (qspi_wait_ready(cfg, sfmcmd, sfmcom) != 0) {
			cfg->last_error = 1;
			break;
		}

		target_address += this_size;
		src += this_size;
		count -= this_size;
	}

	__asm("cpsie i");
	__asm("bkpt 0");
}
