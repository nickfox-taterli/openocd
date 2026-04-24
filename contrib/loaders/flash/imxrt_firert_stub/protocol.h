#ifndef IMXRT_FIRET_STUB_PROTOCOL_H
#define IMXRT_FIRET_STUB_PROTOCOL_H

#include <stdint.h>

#define FIRET_MB_ADDR 0x20206000u
#define FIRET_MB_MAGIC 0x46525431u

enum firert_mb_cmd {
	FIRET_CMD_NONE = 0,
	FIRET_CMD_JEDEC = 1,
	FIRET_CMD_ERASE = 2,
	FIRET_CMD_PROGRAM = 3,
};

enum firert_mb_erase_kind {
	FIRET_ERASE_4K = 1,
	FIRET_ERASE_32K = 2,
	FIRET_ERASE_64K = 3,
	FIRET_ERASE_CHIP = 4,
};

enum firert_mb_status {
	FIRET_ST_BOOTING = 0x10,
	FIRET_ST_READY = 0x11,
	FIRET_ST_BUSY = 0x12,
	FIRET_ST_DONE = 0x13,
	FIRET_ST_ERROR = 0x1f,
};

struct firert_mailbox {
	volatile uint32_t magic;
	volatile uint32_t cmd;
	volatile uint32_t addr;
	volatile uint32_t size;
	volatile uint32_t arg;
	volatile uint32_t src;
	volatile uint32_t status;
	volatile uint32_t result;
	volatile uint32_t detail0;
	volatile uint32_t detail1;
};

#endif
