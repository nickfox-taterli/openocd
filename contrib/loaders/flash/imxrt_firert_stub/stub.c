#include "protocol.h"

#include "MIMXRT1052.h"
#include "fsl_flexspi.h"

#define MB ((volatile struct firert_mailbox *)FIRET_MB_ADDR)

#define FIRET_FLASH_BASE 0x60000000u
#define FIRET_FLASH_SIZE_KB 0x8000u
#define FIRET_PAGE_SIZE 256u

#define LUT_SEQ_READ 0
#define LUT_SEQ_READSTATUS 1
#define LUT_SEQ_WRITEENABLE 2
#define LUT_SEQ_ERASE4K 3
#define LUT_SEQ_PAGEPROGRAM 4
#define LUT_SEQ_READID 5
#define LUT_SEQ_ERASE32K 6
#define LUT_SEQ_ERASE64K 7
#define LUT_SEQ_CHIPERASE 8

static const uint32_t firert_lut[4 * 16] = {
	[4 * LUT_SEQ_READ] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0b,
			kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
	[4 * LUT_SEQ_READ + 1] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08,
			kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

	[4 * LUT_SEQ_READSTATUS] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05,
			kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x01),

	[4 * LUT_SEQ_WRITEENABLE] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06,
			kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	[4 * LUT_SEQ_ERASE4K] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x20,
			kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

	[4 * LUT_SEQ_PAGEPROGRAM] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02,
			kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
	[4 * LUT_SEQ_PAGEPROGRAM + 1] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04,
			kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	[4 * LUT_SEQ_READID] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9f,
			kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

	[4 * LUT_SEQ_ERASE32K] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x52,
			kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

	[4 * LUT_SEQ_ERASE64K] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xd8,
			kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

	[4 * LUT_SEQ_CHIPERASE] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xc7,
			kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),
};

static void firert_memcpy(void *dst, const void *src, uint32_t len)
{
	uint8_t *d = (uint8_t *)dst;
	const uint8_t *s = (const uint8_t *)src;
	while (len--)
		*d++ = *s++;
}

void *memcpy(void *dst, const void *src, size_t len)
{
	firert_memcpy(dst, src, (uint32_t)len);
	return dst;
}

void *memset(void *dst, int value, size_t len)
{
	uint8_t *d = (uint8_t *)dst;
	while (len--)
		*d++ = (uint8_t)value;
	return dst;
}

static int firert_memcmp(const void *a, const void *b, uint32_t len)
{
	const uint8_t *pa = (const uint8_t *)a;
	const uint8_t *pb = (const uint8_t *)b;
	while (len--) {
		if (*pa != *pb)
			return (int)*pa - (int)*pb;
		pa++;
		pb++;
	}
	return 0;
}

static void firert_bkpt(void)
{
	__DSB();
	__ISB();
	__asm volatile ("bkpt 0xab");
}

static void firert_disable_watchdogs(void)
{
	RTWDOG->CNT = RTWDOG_UPDATE_KEY;
	while ((RTWDOG->CS & RTWDOG_CS_ULK_MASK) == 0u)
		;
	RTWDOG->CS &= ~RTWDOG_CS_EN_MASK;

	WDOG1->WCR &= ~WDOG_WCR_WDE_MASK;
	WDOG2->WCR &= ~WDOG_WCR_WDE_MASK;
}

static void firert_config_mpu_cache(void)
{
	if ((SCB->CCR & SCB_CCR_IC_Msk) != 0u)
		SCB_DisableICache();
	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
		SCB_DisableDCache();
	ARM_MPU_Disable();
}

static void firert_enable_redundant_clocks(void)
{
	CCM->CCGR0 = 0xffffffffu;
	CCM->CCGR1 = 0xffffffffu;
	CCM->CCGR2 = 0xffffffffu;
	CCM->CCGR3 = 0xffffffffu;
	CCM->CCGR4 = 0xffffffffu;
	CCM->CCGR5 = 0xffffffffu;
	CCM->CCGR6 = 0xffffffffu;
	CCM->CCR = (CCM->CCR & ~CCM_CCR_OSCNT_MASK) | CCM_CCR_OSCNT(127);
}

static void firert_pinmux_one(uint32_t mux, uint32_t daisy, uint32_t sel, uint32_t pad)
{
	*(volatile uint32_t *)mux = sel;
	if (daisy != 0u)
		*(volatile uint32_t *)daisy = 0u;
	*(volatile uint32_t *)pad = 0x10f1u;
}

static void firert_config_pins(void)
{
	XTALOSC24M->OSC_CONFIG2 |= XTALOSC24M_OSC_CONFIG2_ENABLE_1M_MASK;
	firert_pinmux_one(0x401f81e8u, 0x401f84a4u, 1u, 0x401f83d8u);
	firert_pinmux_one(0x401f81ecu, 0x00000000u, 1u, 0x401f83dcu);
	firert_pinmux_one(0x401f81f0u, 0x401f84c8u, 1u, 0x401f83e0u);
	firert_pinmux_one(0x401f81f4u, 0x401f84a8u, 1u, 0x401f83e4u);
	firert_pinmux_one(0x401f81f8u, 0x401f84acu, 1u, 0x401f83e8u);
	firert_pinmux_one(0x401f81fcu, 0x401f84b0u, 1u, 0x401f83ecu);
	firert_pinmux_one(0x401f8200u, 0x401f84b4u, 1u, 0x401f83f0u);
}

static void firert_init_flexspi(void)
{
	flexspi_config_t cfg;
	flexspi_device_config_t devcfg = {
		.flexspiRootClk = 24000000u,
		.flashSize = FIRET_FLASH_SIZE_KB,
		.CSIntervalUnit = kFLEXSPI_CsIntervalUnit1SckCycle,
		.CSInterval = 2u,
		.CSHoldTime = 3u,
		.CSSetupTime = 3u,
		.dataValidTime = 0u,
		.columnspace = 0u,
		.enableWordAddress = 0u,
		.AWRSeqIndex = 0u,
		.AWRSeqNumber = 0u,
		.ARDSeqIndex = LUT_SEQ_READ,
		.ARDSeqNumber = 1u,
		.AHBWriteWaitUnit = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
		.AHBWriteWaitInterval = 0u,
	};

	FLEXSPI_GetDefaultConfig(&cfg);
	cfg.ahbConfig.enableAHBPrefetch = false;
	cfg.ahbConfig.enableAHBBufferable = false;
	cfg.ahbConfig.enableReadAddressOpt = false;
	cfg.ahbConfig.enableAHBCachable = false;
	cfg.rxSampleClock = kFLEXSPI_ReadSampleClkLoopbackInternally;

	FLEXSPI_Init(FLEXSPI, &cfg);
	FLEXSPI_SetFlashConfig(FLEXSPI, &devcfg, kFLEXSPI_PortA1);
	FLEXSPI_UpdateLUT(FLEXSPI, 0, firert_lut, sizeof(firert_lut) / sizeof(firert_lut[0]));
	FLEXSPI_SoftwareReset(FLEXSPI);
}

static status_t firert_wren(void)
{
	flexspi_transfer_t xfer = {0};
	xfer.deviceAddress = 0u;
	xfer.port = kFLEXSPI_PortA1;
	xfer.cmdType = kFLEXSPI_Command;
	xfer.SeqNumber = 1u;
	xfer.seqIndex = LUT_SEQ_WRITEENABLE;
	return FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
}

static status_t firert_wait_ready(void)
{
	uint32_t sr = 0u;
	flexspi_transfer_t xfer = {0};
	xfer.deviceAddress = 0u;
	xfer.port = kFLEXSPI_PortA1;
	xfer.cmdType = kFLEXSPI_Read;
	xfer.SeqNumber = 1u;
	xfer.seqIndex = LUT_SEQ_READSTATUS;
	xfer.data = &sr;
	xfer.dataSize = 1u;

	do {
		status_t st = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
		if (st != kStatus_Success)
			return st;
	} while ((sr & 0x01u) != 0u);

	return kStatus_Success;
}

static uint32_t firert_read_jedec(void)
{
	uint32_t jedec = 0u;
	flexspi_transfer_t xfer = {0};

	xfer.deviceAddress = 0u;
	xfer.port = kFLEXSPI_PortA1;
	xfer.cmdType = kFLEXSPI_Read;
	xfer.SeqNumber = 1u;
	xfer.seqIndex = LUT_SEQ_READID;
	xfer.data = &jedec;
	xfer.dataSize = 3u;

	if (FLEXSPI_TransferBlocking(FLEXSPI, &xfer) != kStatus_Success)
		return 0u;

	FLEXSPI_SoftwareReset(FLEXSPI);
	return jedec & 0x00ffffffu;
}

static status_t firert_erase_once(uint32_t addr, uint32_t kind)
{
	flexspi_transfer_t xfer = {0};
	status_t st;

	st = firert_wait_ready();
	if (st != kStatus_Success)
		return st;

	st = firert_wren();
	if (st != kStatus_Success)
		return st;

	xfer.deviceAddress = addr;
	xfer.port = kFLEXSPI_PortA1;
	xfer.cmdType = kFLEXSPI_Command;
	xfer.SeqNumber = 1u;
	xfer.seqIndex = (kind == FIRET_ERASE_4K) ? LUT_SEQ_ERASE4K :
		(kind == FIRET_ERASE_32K) ? LUT_SEQ_ERASE32K :
		(kind == FIRET_ERASE_64K) ? LUT_SEQ_ERASE64K :
		LUT_SEQ_CHIPERASE;

	st = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
	if (st != kStatus_Success)
		return st;

	st = firert_wait_ready();
	FLEXSPI_SoftwareReset(FLEXSPI);
	return st;
}

static status_t firert_program(uint32_t flash_off, const uint8_t *src, uint32_t size)
{
	uint8_t page[FIRET_PAGE_SIZE];

	while (size != 0u) {
		uint32_t page_base = flash_off & ~(FIRET_PAGE_SIZE - 1u);
		uint32_t page_off = flash_off & (FIRET_PAGE_SIZE - 1u);
		uint32_t chunk = FIRET_PAGE_SIZE - page_off;
		flexspi_transfer_t xfer = {0};
		status_t st;

		if (chunk > size)
			chunk = size;

		firert_memcpy(page, (const void *)(FIRET_FLASH_BASE + page_base), FIRET_PAGE_SIZE);
		firert_memcpy(&page[page_off], src, chunk);

		st = firert_wait_ready();
		if (st != kStatus_Success)
			return st;

		st = firert_wren();
		if (st != kStatus_Success)
			return st;

		xfer.deviceAddress = page_base;
		xfer.port = kFLEXSPI_PortA1;
		xfer.cmdType = kFLEXSPI_Write;
		xfer.SeqNumber = 1u;
		xfer.seqIndex = LUT_SEQ_PAGEPROGRAM;
		xfer.data = (uint32_t *)(void *)page;
		xfer.dataSize = FIRET_PAGE_SIZE;

		st = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
		if (st != kStatus_Success)
			return st;

		st = firert_wait_ready();
		if (st != kStatus_Success)
			return st;

		FLEXSPI_SoftwareReset(FLEXSPI);
		if (firert_memcmp((const void *)(FIRET_FLASH_BASE + page_base), page, FIRET_PAGE_SIZE) != 0)
			return kStatus_Fail;

		src += chunk;
		flash_off += chunk;
		size -= chunk;
	}

	return kStatus_Success;
}

static void firert_handle_command(void)
{
	status_t st = kStatus_Success;

	MB->status = FIRET_ST_BUSY;
	MB->detail0 = 0u;
	MB->detail1 = 0u;

	switch (MB->cmd) {
	case FIRET_CMD_JEDEC:
		MB->result = firert_read_jedec();
		break;
	case FIRET_CMD_ERASE:
		st = firert_erase_once(MB->addr, MB->arg);
		break;
	case FIRET_CMD_PROGRAM:
		st = firert_program(MB->addr, (const uint8_t *)(uintptr_t)MB->src, MB->size);
		break;
	default:
		st = kStatus_InvalidArgument;
		break;
	}

	MB->detail0 = (uint32_t)st;
	MB->status = (st == kStatus_Success) ? FIRET_ST_DONE : FIRET_ST_ERROR;
	MB->cmd = FIRET_CMD_NONE;
}

__attribute__((section(".text.stub_entry")))
void stub_entry(void)
{
	firert_config_mpu_cache();
	firert_disable_watchdogs();
	firert_enable_redundant_clocks();
	firert_config_pins();
	firert_init_flexspi();

	MB->magic = FIRET_MB_MAGIC;
	MB->cmd = FIRET_CMD_NONE;
	MB->addr = 0u;
	MB->size = 0u;
	MB->arg = 0u;
	MB->src = 0u;
	MB->result = firert_read_jedec();
	MB->detail0 = 0u;
	MB->detail1 = 0u;
	MB->status = FIRET_ST_READY;

	firert_bkpt();

	for (;;) {
		if (MB->cmd != FIRET_CMD_NONE) {
			firert_handle_command();
			firert_bkpt();
		}
	}
}
