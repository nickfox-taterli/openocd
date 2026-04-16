// SPDX-License-Identifier: GPL-2.0-or-later

#define OPENOCD_CONTRIB_LOADERS_FLASH_RENESAS_RA8D1_OSPI

#include <stdint.h>
#include <string.h>

#include "../../../../src/flash/nor/renesas_ra8d1_ospi.h"

static inline __attribute__((always_inline))
void direct_transfer(const struct ra8d1_ospi_loader_cfg *cfg,
		uint16_t command,
		uint8_t command_length,
		uint32_t address,
		uint8_t address_length,
		uint8_t data_length,
		uint8_t direction,
		uint64_t *data)
{
	volatile uint32_t *cdctl0 = (volatile uint32_t *)RA8D1_OSPI_REG_CDCTL0;
	volatile uint32_t *cdt = (volatile uint32_t *)RA8D1_OSPI_REG_CDBUF0_CDT;
	volatile uint32_t *cda = (volatile uint32_t *)RA8D1_OSPI_REG_CDBUF0_CDA;
	volatile uint32_t *cdd0 = (volatile uint32_t *)RA8D1_OSPI_REG_CDBUF0_CDD0;
	volatile uint32_t *cdd1 = (volatile uint32_t *)RA8D1_OSPI_REG_CDBUF0_CDD1;
	volatile uint32_t *ints = (volatile uint32_t *)RA8D1_OSPI_REG_INTS;
	volatile uint32_t *intc = (volatile uint32_t *)RA8D1_OSPI_REG_INTC;
	uint32_t cdt_value;

	cdt_value = ((uint32_t)command_length << RA8D1_OSPI_CDT_CMDSIZE_SHIFT) |
			((uint32_t)address_length << RA8D1_OSPI_CDT_ADDSIZE_SHIFT) |
			((uint32_t)data_length << RA8D1_OSPI_CDT_DATASIZE_SHIFT) |
			((uint32_t)direction << RA8D1_OSPI_CDT_TRTYPE_SHIFT);

	if (command_length == 1)
		cdt_value |= ((uint32_t)(command & 0xFFu) << 24);
	else
		cdt_value |= ((uint32_t)(command & 0xFFFFu) << RA8D1_OSPI_CDT_CMD_SHIFT);

	*cdctl0 = cfg->cs_setup;
	while ((*cdctl0 & RA8D1_OSPI_CDCTL0_TRREQ) != 0)
		;

	*cdt = cdt_value;
	*cda = address;

	if ((direction == RA8D1_OSPI_TRTYPE_WRITE) && (data_length > 0) && data) {
		*cdd0 = (uint32_t)(*data & 0xFFFFFFFFu);
		if (data_length > 4)
			*cdd1 = (uint32_t)(*data >> 32);
	}

	*intc = 0xFFFFFFFFu;
	*cdctl0 = cfg->cs_setup | RA8D1_OSPI_CDCTL0_TRREQ;
	while ((*cdctl0 & RA8D1_OSPI_CDCTL0_TRREQ) != 0)
		;

	if ((direction == RA8D1_OSPI_TRTYPE_READ) && (data_length > 0) && data) {
		*data = *cdd0;
		if (data_length > 4)
			*data |= ((uint64_t)(*cdd1) << 32);
	}

	*intc = *ints;
}

static inline __attribute__((always_inline))
int wait_ready(const struct ra8d1_ospi_loader_cfg *cfg)
{
	uint32_t timeout = cfg->status_timeout_loops;
	uint64_t data = 0;

	while (timeout--) {
		direct_transfer(cfg,
				(uint16_t)cfg->cmd_status, 1,
				0, 0,
				1, RA8D1_OSPI_TRTYPE_READ,
				&data);
		if ((((uint8_t)data) & cfg->busy_bit_mask) == 0)
			return 0;
	}

	return -1;
}

void run(struct ra8d1_ospi_loader_cfg *cfg,
		uint8_t *buf,
		uint32_t chip_address,
		uint32_t count)
{
	__asm("cpsid i");

	cfg->last_error = 0;

	if (cfg->operation == RA8D1_OSPI_LOADER_OP_READ) {
		while (count) {
			uint64_t data = 0;
			uint32_t this_chunk = (count > 8) ? 8 : count;

			direct_transfer(cfg,
					(uint16_t)cfg->cmd_read_data, 1,
					chip_address, 3,
					(uint8_t)this_chunk, RA8D1_OSPI_TRTYPE_READ,
					&data);

			for (uint32_t i = 0; i < this_chunk; i++)
				buf[i] = (uint8_t)(data >> (8 * i));

			buf += this_chunk;
			chip_address += this_chunk;
			count -= this_chunk;
		}
	} else {
		while (count) {
			uint32_t page_off = chip_address & (cfg->page_size - 1);
			uint32_t this_page = cfg->page_size - page_off;
			if (this_page > count)
				this_page = count;

			while (this_page) {
				uint64_t data = 0;
				uint32_t this_chunk = (this_page > 8) ? 8 : this_page;

				for (uint32_t i = 0; i < this_chunk; i++)
					data |= ((uint64_t)buf[i] << (8 * i));

				direct_transfer(cfg,
						(uint16_t)cfg->cmd_write_enable, 1,
						0, 0,
						0, RA8D1_OSPI_TRTYPE_WRITE,
						NULL);

				direct_transfer(cfg,
						(uint16_t)cfg->cmd_page_program, 1,
						chip_address, 3,
						(uint8_t)this_chunk, RA8D1_OSPI_TRTYPE_WRITE,
						&data);

				if (wait_ready(cfg) != 0) {
					cfg->last_error = 1;
					goto out;
				}

				buf += this_chunk;
				chip_address += this_chunk;
				count -= this_chunk;
				this_page -= this_chunk;
			}
		}
	}

out:
	__asm("cpsie i");
	__asm("bkpt 0");
}
