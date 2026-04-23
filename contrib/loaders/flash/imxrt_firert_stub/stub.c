#include "protocol.h"

#include <stddef.h>
#include "MIMXRT1052.h"

typedef int32_t status_t;

#define STATUS_SUCCESS  0
#define STATUS_FAIL     1
#define STATUS_INVAL    4

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
		[4 * LUT_SEQ_READ]           = 0x0818040bu,
		[4 * LUT_SEQ_READ + 1]       = 0x24043008u,
		[4 * LUT_SEQ_READSTATUS]     = 0x24010405u,
		[4 * LUT_SEQ_WRITEENABLE]    = 0x00000406u,
		[4 * LUT_SEQ_ERASE4K]        = 0x08180420u,
		[4 * LUT_SEQ_PAGEPROGRAM]    = 0x08180402u,
		[4 * LUT_SEQ_PAGEPROGRAM + 1] = 0x00002004u,
		[4 * LUT_SEQ_READID]         = 0x2404049fu,
		[4 * LUT_SEQ_ERASE32K]       = 0x08180452u,
		[4 * LUT_SEQ_ERASE64K]       = 0x081804d8u,
		[4 * LUT_SEQ_CHIPERASE]      = 0x000004c7u,
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

/* Minimal FlexSPI helpers replacing SDK functions */
static void firert_sw_reset(void)
{
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	FLEXSPI->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
	while (FLEXSPI->MCR0 & FLEXSPI_MCR0_SWRESET_MASK)
		;
}

static void firert_update_lut(uint32_t index, const uint32_t *cmd, uint32_t count)
{
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	FLEXSPI->LUTKEY = 0x5AF05AF0u;
	FLEXSPI->LUTCR  = 0x02u;
	for (uint32_t i = 0; i < count; i++)
		FLEXSPI->LUT[index + i] = cmd[i];
	FLEXSPI->LUTKEY = 0x5AF05AF0u;
	FLEXSPI->LUTCR  = 0x01u;
}

static status_t firert_ip_cmd(uint32_t addr, uint8_t seq_idx)
{
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	FLEXSPI->FLSHCR2[0] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;
	FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE_MASK | FLEXSPI_INTR_IPCMDERR_MASK |
			FLEXSPI_INTR_IPCMDGE_MASK;
	FLEXSPI->IPCR0 = addr;
	FLEXSPI->IPTXFCR |= FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
	FLEXSPI->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
	FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(seq_idx);
	FLEXSPI->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
	while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPCMDDONE_MASK))
		;
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	if (FLEXSPI->INTR & (FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_IPCMDGE_MASK))
		return STATUS_FAIL;
	return STATUS_SUCCESS;
}

static status_t firert_ip_read(uint32_t addr, uint8_t seq_idx,
			       uint32_t *data, uint32_t size)
{
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	FLEXSPI->FLSHCR2[0] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;
	FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE_MASK | FLEXSPI_INTR_IPCMDERR_MASK |
			FLEXSPI_INTR_IPCMDGE_MASK;
	FLEXSPI->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
	FLEXSPI->IPCR0 = addr;
	FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(seq_idx) | FLEXSPI_IPCR1_IDATSZ(size);
	FLEXSPI->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;

	/* Wait for RX FIFO data */
	while (((FLEXSPI->IPRXFSTS & FLEXSPI_IPRXFSTS_FILL_MASK) >>
		FLEXSPI_IPRXFSTS_FILL_SHIFT) == 0u)
		;
	*data = FLEXSPI->RFDR[0];
	FLEXSPI->INTR = FLEXSPI_INTR_IPRXWA_MASK;

	while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPCMDDONE_MASK))
		;
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	if (FLEXSPI->INTR & (FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_IPCMDGE_MASK))
		return STATUS_FAIL;
	return STATUS_SUCCESS;
}

static status_t firert_ip_write(uint32_t addr, const uint32_t *data, uint32_t size)
{
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	FLEXSPI->FLSHCR2[0] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;
	FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE_MASK | FLEXSPI_INTR_IPCMDERR_MASK |
			FLEXSPI_INTR_IPCMDGE_MASK | FLEXSPI_INTR_IPTXWE_MASK;
	FLEXSPI->IPTXFCR |= FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
	FLEXSPI->IPCR0 = addr;
	FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(LUT_SEQ_PAGEPROGRAM) |
			  FLEXSPI_IPCR1_IDATSZ(size);
	FLEXSPI->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;

	/* Push data to TX FIFO in 8-byte (2-word) chunks */
	const uint32_t *src = data;
	uint32_t remaining = size / 4u;
	while (remaining >= 2u) {
		while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPTXWE_MASK))
			;
		FLEXSPI->TFDR[0] = *src++;
		FLEXSPI->TFDR[1] = *src++;
		FLEXSPI->INTR = FLEXSPI_INTR_IPTXWE_MASK;
		remaining -= 2u;
	}
	if (remaining != 0u) {
		while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPTXWE_MASK))
			;
		FLEXSPI->TFDR[0] = *src;
		FLEXSPI->INTR = FLEXSPI_INTR_IPTXWE_MASK;
	}

	while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPCMDDONE_MASK))
		;
	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
	if (FLEXSPI->INTR & (FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_IPCMDGE_MASK))
		return STATUS_FAIL;
	return STATUS_SUCCESS;
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
	__DSB();
	__ISB();
	SCB->CCR &= ~(SCB_CCR_IC_Msk | SCB_CCR_DC_Msk);
	SCB->ICIALLU = 0UL;
	__DSB();
	__ISB();
	MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
	__DSB();
	__ISB();
}

static void firert_enable_redundant_clocks(void)
{
	CCM->CCGR0 = 0xffffffffu;
	CCM->CCGR1 = 0xffffffffu;
	CCM->CCGR2 = 0xffffffffu;
	CCM->CCGR3 = 0xffffffffu;
	CCM->CCGR4 = 0xffffffffu;
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

static void firert_set_flash_config(void)
{
	uint32_t val;

	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;

	FLEXSPI->FLSHCR0[0] = FIRET_FLASH_SIZE_KB;

	FLEXSPI->FLSHCR1[0] = FLEXSPI_FLSHCR1_CSINTERVAL(2u)
	                     | FLEXSPI_FLSHCR1_CSINTERVALUNIT(0u)
	                     | FLEXSPI_FLSHCR1_TCSH(3u)
	                     | FLEXSPI_FLSHCR1_TCSS(3u);

	val  = FLEXSPI->FLSHCR2[0];
	val &= ~(FLEXSPI_FLSHCR2_AWRWAITUNIT_MASK | FLEXSPI_FLSHCR2_AWRWAIT_MASK
	       | FLEXSPI_FLSHCR2_AWRSEQNUM_MASK | FLEXSPI_FLSHCR2_AWRSEQID_MASK
	       | FLEXSPI_FLSHCR2_ARDSEQNUM_MASK | FLEXSPI_FLSHCR2_ARDSEQID_MASK);
	val |= FLEXSPI_FLSHCR2_ARDSEQID(LUT_SEQ_READ)
	     | FLEXSPI_FLSHCR2_ARDSEQNUM(0u);
	FLEXSPI->FLSHCR2[0] = val;

	FLEXSPI->DLLCR[0] = 0x100u;
	FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

	FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
	FLEXSPI->FLSHCR4 |= FLEXSPI_FLSHCR4_WMOPT1_MASK;
	FLEXSPI->FLSHCR4 &= ~FLEXSPI_FLSHCR4_WMENA_MASK;
	FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

	while (!(FLEXSPI->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)))
		;
}

static void firert_init_flexspi(void)
{
	uint32_t val;

	FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
	firert_sw_reset();

	FLEXSPI->MCR0 = FLEXSPI_MCR0_RXCLKSRC(0u)
	              | FLEXSPI_MCR0_DOZEEN(1u)
	              | FLEXSPI_MCR0_IPGRANTWAIT(0xFFu)
	              | FLEXSPI_MCR0_AHBGRANTWAIT(0xFFu)
	              | FLEXSPI_MCR0_MDIS_MASK;

	FLEXSPI->MCR1 = FLEXSPI_MCR1_SEQWAIT(0xFFFFu)
	              | FLEXSPI_MCR1_AHBBUSWAIT(0xFFFFu);

	val = FLEXSPI->MCR2;
	val &= ~(FLEXSPI_MCR2_RESUMEWAIT_MASK | FLEXSPI_MCR2_SCKBDIFFOPT_MASK
	       | FLEXSPI_MCR2_SAMEDEVICEEN_MASK | FLEXSPI_MCR2_CLRAHBBUFOPT_MASK);
	val |= FLEXSPI_MCR2_RESUMEWAIT(0x20u);
	FLEXSPI->MCR2 = val;

	val = FLEXSPI->AHBCR;
	val &= ~(FLEXSPI_AHBCR_READADDROPT_MASK | FLEXSPI_AHBCR_PREFETCHEN_MASK
	       | FLEXSPI_AHBCR_BUFFERABLEEN_MASK | FLEXSPI_AHBCR_CACHABLEEN_MASK);
	FLEXSPI->AHBCR = val;

	FLEXSPI->AHBRXBUFCR0[2] = FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1u)
	                         | FLEXSPI_AHBRXBUFCR0_BUFSZ(256u / 8u);
	FLEXSPI->AHBRXBUFCR0[3] = FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1u)
	                         | FLEXSPI_AHBRXBUFCR0_BUFSZ(256u / 8u);

	FLEXSPI->IPRXFCR &= ~FLEXSPI_IPRXFCR_RXWMRK_MASK;
	FLEXSPI->IPTXFCR &= ~FLEXSPI_IPTXFCR_TXWMRK_MASK;

	FLEXSPI->FLSHCR0[0] = 0;
	FLEXSPI->FLSHCR0[1] = 0;
	FLEXSPI->FLSHCR0[2] = 0;
	FLEXSPI->FLSHCR0[3] = 0;

	firert_set_flash_config();
	firert_update_lut(0, firert_lut, sizeof(firert_lut) / sizeof(firert_lut[0]));
	firert_sw_reset();
}

static status_t firert_wren(void)
{
	return firert_ip_cmd(0, LUT_SEQ_WRITEENABLE);
}

static status_t firert_wait_ready(void)
{
	uint32_t sr = 0u;
	do {
		status_t st = firert_ip_read(0, LUT_SEQ_READSTATUS, &sr, 1);
		if (st != STATUS_SUCCESS)
			return st;
	} while ((sr & 0x01u) != 0u);
	return STATUS_SUCCESS;
}

static uint32_t firert_read_jedec(void)
{
	uint32_t jedec = 0u;
	if (firert_ip_read(0, LUT_SEQ_READID, &jedec, 3) != STATUS_SUCCESS)
		return 0u;
	firert_sw_reset();
	return jedec & 0x00ffffffu;
}

static status_t firert_erase_once(uint32_t addr, uint32_t kind)
{
	status_t st;
	uint8_t seq;

	st = firert_wait_ready();
	if (st != STATUS_SUCCESS)
		return st;

	st = firert_wren();
	if (st != STATUS_SUCCESS)
		return st;

	seq = (kind == FIRET_ERASE_4K) ? LUT_SEQ_ERASE4K :
	      (kind == FIRET_ERASE_32K) ? LUT_SEQ_ERASE32K :
	      (kind == FIRET_ERASE_64K) ? LUT_SEQ_ERASE64K :
	      LUT_SEQ_CHIPERASE;

	st = firert_ip_cmd(addr, seq);
	if (st != STATUS_SUCCESS)
		return st;

	st = firert_wait_ready();
	firert_sw_reset();
	return st;
}

static status_t firert_program(uint32_t flash_off, const uint8_t *src, uint32_t size)
{
	uint8_t page[FIRET_PAGE_SIZE];

	while (size != 0u) {
		uint32_t page_base = flash_off & ~(FIRET_PAGE_SIZE - 1u);
		uint32_t page_off = flash_off & (FIRET_PAGE_SIZE - 1u);
		uint32_t chunk = FIRET_PAGE_SIZE - page_off;
		status_t st;

		if (chunk > size)
			chunk = size;

		firert_memcpy(page, (const void *)(FIRET_FLASH_BASE + page_base), FIRET_PAGE_SIZE);
		firert_memcpy(&page[page_off], src, chunk);

		st = firert_wait_ready();
		if (st != STATUS_SUCCESS)
			return st;

		st = firert_wren();
		if (st != STATUS_SUCCESS)
			return st;

		st = firert_ip_write(page_base, (const uint32_t *)(const void *)page, FIRET_PAGE_SIZE);
		if (st != STATUS_SUCCESS)
			return st;

		st = firert_wait_ready();
		if (st != STATUS_SUCCESS)
			return st;

		firert_sw_reset();
		if (firert_memcmp((const void *)(FIRET_FLASH_BASE + page_base), page, FIRET_PAGE_SIZE) != 0)
			return STATUS_FAIL;

		src += chunk;
		flash_off += chunk;
		size -= chunk;
	}

	return STATUS_SUCCESS;
}

static void firert_handle_command(void)
{
	status_t st = STATUS_SUCCESS;

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
		st = STATUS_INVAL;
		break;
	}

	MB->detail0 = (uint32_t)st;
	MB->status = (st == STATUS_SUCCESS) ? FIRET_ST_DONE : FIRET_ST_ERROR;
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
