/**
 * \file
 *
 * \brief Proxy PLC Controller interface layer implementation.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "asf.h"

#include "string.h"
#include "board.h"
#include "pplc_if.h"
#include "sysclk.h"
#include "spi.h"
#include "pio.h"
#include "pio_handler.h"
#include "conf_project.h"
#include "conf_pplc_if.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \weakgroup plc_group
 * @{
 */

/** OPCODES */
/*  Normal operation (R/W). */
 #define OPCODE_NONE        0x00
/*  AND operation. */
 #define OPCODE_AND         0x20
/*  OR operation. */
 #define OPCODE_OR          0x40
/*  XOR operation. */
 #define OPCODE_XOR         0x80

/*  Code for No repetition */
 #define NO_REP             0x00

/* PPLC Command header length */
 #define PPLC_HEADER_LEN    3

/* ! PPLC clock setting (Hz). */
static uint32_t gs_ul_pplc_clock = PPLC_CLOCK_24M;
static bool gs_b_disable_full_interrup = false;
/* ! PDC buffer us_size. */
#define PDC_PPLC_BUFFER_SIZE       1024 + PPLC_HEADER_LEN

/* PPLC peripheral chip select config value */
#define PPLC_PCS    spi_get_pcs(PPLC_CS)

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
/* ! PDC RX data packet */
pdc_packet_t g_pplc_rx_packet;
/* ! PDC TX data packet */
pdc_packet_t g_pplc_tx_packet;
/* ! Pointer to PDC register base */
Pdc *g_pplc_pdc;
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
/** XDMA channel used for PPLC_IF */
#define XDMA_CH    0 /* Used for DEBUG */
#define PPLC_XDMAC_CH_TX 0
#define PPLC_XDMAC_CH_RX 1
/* XDMAC channel status  */
#define PPLC_XDMAC_TX_STATUS XDMAC_GS_ST0
#define PPLC_XDMAC_RX_STATUS XDMAC_GS_ST1
/* XDMAC Peripheral IDs */
#define PPLC_XDMAC_SPI0_TX_PERID 1
#define PPLC_XDMAC_SPI0_RX_PERID 2

/** XDMA channel configuration. */
static xdmac_channel_config_t xdmac_tx_channel_cfg;
static xdmac_channel_config_t xdmac_rx_channel_cfg;

/* DMA transfer done flag. */
volatile uint32_t g_xfer_done = 0;
#endif

/* ! PDC Receive buffer */
COMPILER_ALIGNED(32)
static uint8_t gs_pplc_rx_buffer[PDC_PPLC_BUFFER_SIZE];
/* ! PDC Transmission buffer */
COMPILER_ALIGNED(32)
static uint8_t gs_pplc_tx_buffer[PDC_PPLC_BUFFER_SIZE];

/* Static variables to use as ptrs */
static uint8_t resBuf32[4];
static uint8_t resBuf16[2];
static uint8_t res8;
static uint8_t dataBuf32[4];
static uint8_t dataBuf16[2];
static uint8_t data8;

/* LOG SPI */
#if (LOG_SPI == 1)
#define LOG_SPI_BUFFER_SIZE    24000 /* 16384 // 16 kB */
static uint8_t auc_buf_log_spi[LOG_SPI_BUFFER_SIZE];
static uint16_t us_buf_log_spi_idx;
static uint16_t aus_buf_log_lengths[LOG_SPI_BUFFER_SIZE / 4];
static uint16_t us_buf_log_lengths_idx;
#endif

/**
 * Describes a PPLC interrupt handler
 */
static void (*pplc_handler)(void);

static void pplc_if_int_handler(uint32_t ul_id, uint32_t ul_mask)
{
	UNUSED(ul_id);
	UNUSED(ul_mask);
	if (pplc_handler != NULL) {
		pplc_handler();
	}
}

#if (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)

/**
 * \brief Disable XDMAC for spi and forbidden transmit and receive by XDMAC.
 *
 */
static inline void _xdmac_disable(void)
{
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_RX);
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_TX);
}

#endif

/**
 * \brief Reset and default configuration of SPI peripheral.
 *
 */
static void _spi_reset_and_config(void)
{
	uint32_t cpuhz;
	uint8_t divider;

	cpuhz = sysclk_get_peripheral_hz();
	divider = cpuhz / gs_ul_pplc_clock;

	if (cpuhz % gs_ul_pplc_clock) {
		divider++;
	}

	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_MASTER_BASE);
	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, PPLC_PCS);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL, divider);
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS, SPI_DLYBCT);
	spi_enable(SPI_MASTER_BASE);
}

/**
 * \internal
 * \brief Initialize Proxy PLC controller.
 *
 * This function will change the system clock prescaler configuration to
 * match the parameters.
 *
 * \note The parameters to this function are device-specific.
 *
 */
static void _pplc_if_config(void)
{
	/* Reset and configure SPI */
	_spi_reset_and_config();

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	/* Get board SPI PDC base address and enable receiver and transmitter. */
	g_pplc_pdc = spi_get_pdc_base(SPI_MASTER_BASE);
	pdc_enable_transfer(g_pplc_pdc, PERIPH_PTCR_TXTEN);
	pdc_disable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTDIS);

	/* Configure receiving PDC */
	g_pplc_rx_packet.ul_addr = (uint32_t)gs_pplc_rx_buffer;
	g_pplc_rx_packet.ul_size = PDC_PPLC_BUFFER_SIZE;

	/* Configure and stop transmit PDC */
	g_pplc_tx_packet.ul_addr = (uint32_t)gs_pplc_tx_buffer;
	g_pplc_tx_packet.ul_size = 0;
	pdc_tx_init(g_pplc_pdc, &g_pplc_tx_packet, NULL);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
	/* Initialize and enable DMA controller */
	pmc_enable_periph_clk(ID_XDMAC);

	/* Configure TX and RX XDMAC channels */
	xdmac_tx_channel_cfg.mbr_sa = (uint32_t)gs_pplc_tx_buffer;
	xdmac_tx_channel_cfg.mbr_da = (uint32_t)spi_get_tx_access(SPI_MASTER_BASE);
	xdmac_tx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(PPLC_XDMAC_SPI0_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
	xdmac_tx_channel_cfg.mbr_bc = 0;
	xdmac_tx_channel_cfg.mbr_ds = 0;
	xdmac_tx_channel_cfg.mbr_sus = 0;
	xdmac_tx_channel_cfg.mbr_dus = 0;

	xdmac_channel_set_descriptor_control(XDMAC, PPLC_XDMAC_CH_TX, 0);

	xdmac_rx_channel_cfg.mbr_sa = (uint32_t)spi_get_rx_access(SPI_MASTER_BASE);
	xdmac_rx_channel_cfg.mbr_da = (uint32_t)gs_pplc_rx_buffer;
	xdmac_rx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(PPLC_XDMAC_SPI0_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
	xdmac_rx_channel_cfg.mbr_bc = 0;
	xdmac_rx_channel_cfg.mbr_ds =  0;
	xdmac_rx_channel_cfg.mbr_sus = 0;
	xdmac_rx_channel_cfg.mbr_dus = 0;

	xdmac_channel_set_descriptor_control(XDMAC, PPLC_XDMAC_CH_RX, 0);
#endif
}

/**
 * \brief Transmit SPI command to PLC module
 *
 *  \param wr                   Write/Read flag
 *  \param addr                 Address to Read/Write
 *  \param len                  Number of bytes in operation
 *  \param buf                  Pointer to data buffer
 *  \param opcode               Opcode to perform (only used in AND/OR/XOR and Read/WriteJump command)
 *  \param bytesRepeat          Number of repetitions (only use in Read/WriteRep command)
 *  \param block                Blocking/Non-Blocking transfer flag
 *
 *  \return true if there is no error, otherwise returns false.
 */
static int8_t _pplc_cmd_op(uint8_t wr, uint16_t addr, uint16_t len, uint8_t *buf, uint8_t opcode, uint8_t bytesRepeat, bool block)
{
	uint8_t *ptr2data_buf;
	uint16_t numTxBytes;
	uint16_t dataLen;
	uint16_t address;
	uint32_t spi_busy_cnt;

	ptr2data_buf = buf;
	dataLen = len;
	address = addr;

	/* protection to data length */
	if (dataLen > (PDC_PPLC_BUFFER_SIZE - PPLC_HEADER_LEN)) {
		return false;
	}

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	/* Waiting transfer done*/
	spi_busy_cnt = 0;
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RXBUFF) == 0) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			return false;
		}
	}
#endif

#if (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
	/* Wait TX and RX transfer finish */
	spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_TX_STATUS) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			return false;
		}
	}
	spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			return false;
		}
	}
#endif

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
#endif

	numTxBytes = dataLen + PPLC_HEADER_LEN;

	/* Configure PPLC Tx buffer */
	gs_pplc_tx_buffer[0] = (uint8_t)(address >> 8);
	gs_pplc_tx_buffer[1] = (uint8_t)(address & 0xFF);
	gs_pplc_tx_buffer[2] = opcode | bytesRepeat;

	/* Fill data */
	if (wr) {
		memcpy(&gs_pplc_tx_buffer[PPLC_HEADER_LEN], ptr2data_buf, dataLen);
	} else {
		memset(&gs_pplc_tx_buffer[PPLC_HEADER_LEN], 0, dataLen);
	}

	/* Log Spi */
	#if (LOG_SPI == 1)
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	aus_buf_log_lengths[us_buf_log_lengths_idx] = dataLen + PPLC_HEADER_LEN + 4;
	memcpy(&auc_buf_log_spi[us_buf_log_spi_idx], &gs_pplc_tx_buffer[0], dataLen + PPLC_HEADER_LEN);
	us_buf_log_spi_idx += (dataLen + PPLC_HEADER_LEN);
	us_buf_log_lengths_idx++;
	#endif

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	/* Configure DMA channels */
	g_pplc_rx_packet.ul_size = numTxBytes;
	pdc_rx_init(g_pplc_pdc, &g_pplc_rx_packet, NULL);

	g_pplc_tx_packet.ul_size = numTxBytes;
	pdc_tx_init(g_pplc_pdc, &g_pplc_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	if (block) {
		/* Waiting transfer done*/
		spi_busy_cnt = 0;
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RXBUFF) == 0) {
			spi_busy_cnt++;
			if (spi_busy_cnt > 50000) {
				if (!gs_b_disable_full_interrup) {
					Enable_global_interrupt();
				}

				return false;
			}
		}

		/* copy rcv data */
		if (!wr) {
			memcpy(ptr2data_buf, &gs_pplc_rx_buffer[PPLC_HEADER_LEN], dataLen);
			/* Log Spi */
		#if (LOG_SPI == 1)
			memcpy(&auc_buf_log_spi[us_buf_log_spi_idx - dataLen], &gs_pplc_rx_buffer[PPLC_HEADER_LEN], dataLen);
		#endif
		}

		/* Disable the RX and TX PDC transfer requests */
		pdc_disable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	}

#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
#ifdef CONF_BOARD_ENABLE_CACHE
	SCB_InvalidateDCache_by_Addr((uint32_t *)gs_pplc_tx_buffer, 4 + dataLen);
	SCB_InvalidateDCache_by_Addr((uint32_t *)gs_pplc_rx_buffer, 4 + dataLen);
#endif
	/* Configure TX and RX XDMAC channels */
	xdmac_rx_channel_cfg.mbr_ubc = numTxBytes;
	xdmac_tx_channel_cfg.mbr_ubc = numTxBytes;
	xdmac_configure_transfer(XDMAC, PPLC_XDMAC_CH_RX, &xdmac_rx_channel_cfg);
	xdmac_configure_transfer(XDMAC, PPLC_XDMAC_CH_TX, &xdmac_tx_channel_cfg);

	/* XDMAC RX channel enable */
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << PPLC_XDMAC_CH_RX);
	/* XDMAC TX channel enable */
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << PPLC_XDMAC_CH_TX);

	if (block) {
		/* Wait TX transfer finish */
		spi_busy_cnt = 0;
		while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_TX_STATUS) {
			spi_busy_cnt++;
			if (spi_busy_cnt > 50000) {
				return false;
			}
		}
		spi_busy_cnt = 0;
		while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
			spi_busy_cnt++;
			if (spi_busy_cnt > 50000) {
				return false;
			}
		}

		/* copy rcv data */
		if (!wr) {
			memcpy(ptr2data_buf, &gs_pplc_rx_buffer[PPLC_HEADER_LEN], dataLen);
			/* Log Spi */
		#if (LOG_SPI == 1)
			memcpy(&auc_buf_log_spi[us_buf_log_spi_idx - dataLen], &gs_pplc_rx_buffer[PPLC_HEADER_LEN], dataLen);
		#endif
		}

		_xdmac_disable();
	}
#endif

	return true;
}

#if (LOG_SPI == 1)
void dumpLogSpi(void)
{
	uint16_t us_i, us_j, us_k;

	us_j = 0;
	us_k = 0;
	while (us_j < us_buf_log_spi_idx) {
		for (us_i = 0; us_i < aus_buf_log_lengths[us_k]; us_i++) {
			printf("%02X", auc_buf_log_spi[us_j + us_i]);
		}
		printf("\r\n");
		us_j += aus_buf_log_lengths[us_k];
		us_k++;
	}

	us_buf_log_spi_idx = 0;
	us_buf_log_lengths_idx = 0;
}

void clearLogSpi(void)
{
	us_buf_log_spi_idx = 0;
	us_buf_log_lengths_idx = 0;
}

void addIntMarkToLogSpi(void)
{
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0xFF;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0xFF;
	auc_buf_log_spi[us_buf_log_spi_idx++] = 0xFF;
	aus_buf_log_lengths[us_buf_log_lengths_idx] = 7;
	us_buf_log_lengths_idx++;
}

#endif

/**
 * \brief Read 8bits from PLC module
 *
 * \param addrs		Address to read
 *
 * \return Value read
 */
uint8_t pplc_if_read8(uint16_t addrs)
{
	/* Enter critical section */
	Disable_global_interrupt();
	_pplc_cmd_op(0, addrs, 1, &res8, OPCODE_NONE, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return res8;
}

/**
 * \brief Write 8 bits to PLC module
 *
 * \param addrs		Address to destination
 * \param dat		Data to write
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_write8(uint16_t addrs, uint8_t dat)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	data8 = dat;
	uc_res = _pplc_cmd_op(1, addrs | 0x8000, 1, &data8, OPCODE_NONE, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Read 16bits from PLC module
 *
 * \param addrs		Address to read
 *
 * \return Value read
 */
uint16_t pplc_if_read16(uint16_t addrs)
{
	uint16_t res;

	/* Enter critical section */
	Disable_global_interrupt();
	_pplc_cmd_op(0, addrs, 2, resBuf16, OPCODE_NONE, NO_REP, true);
	res = ((uint16_t)resBuf16[0]) << 8;
	res |= (uint16_t)resBuf16[1];
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return res;
}

/**
 * \brief Write 16 bits to PLC module
 *
 * \param addrs		Address to destination
 * \param dat		Data to write
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_write16(uint16_t addrs, uint16_t dat)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	dataBuf16[0] = ((uint8_t)(dat >> 8));
	dataBuf16[1] = ((uint8_t)(dat & 0xFF));
	uc_res = _pplc_cmd_op(1, addrs | 0x8000, 2, dataBuf16, OPCODE_NONE, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Read 32bits from PLC module
 *
 * \param addrs		Address to read
 *
 * \return Value read
 */

uint32_t pplc_if_read32(uint16_t addrs)
{
	uint32_t res;

	/* Enter critical section */
	Disable_global_interrupt();
	_pplc_cmd_op(0, addrs, 4, resBuf32, OPCODE_NONE, NO_REP, true);
	res = ((uint32_t)resBuf32[0]) << 24;
	res |= ((uint32_t)resBuf32[1]) << 16;
	res |= ((uint32_t)resBuf32[2]) << 8;
	res |= (uint32_t)resBuf32[3];

	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return res;
}

/**
 * \brief Write 32 bits to PLC module
 *
 * \param addrs		Address to destination
 * \param dat		Data to write
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_write32(uint16_t addrs, uint32_t dat)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	dataBuf32[0] = ((uint8_t)(dat >> 24));
	dataBuf32[1] = ((uint8_t)(dat >> 16));
	dataBuf32[2] = ((uint8_t)(dat >> 8));
	dataBuf32[3] = ((uint8_t)(dat & 0xFF));
	uc_res = _pplc_cmd_op(1, addrs | 0x8000, 4, dataBuf32, OPCODE_NONE, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Read bytes from PLC module
 *
 * \param addrs		Initial address to read
 * \param buf		Destination of read data
 * \param len		Number of bytes to read
 * \param block		Blocking (true) /Non-Blocking (false) transfer flag
 *					If block = false, read data will be copied in buf when pplc_if_do_read() is called
 *					(If another SPI transfer is made before calling pplc_if_do_read, data will be lost)
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_read_buf(uint16_t addrs, uint8_t *buf, uint16_t len, bool block)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	uc_res = _pplc_cmd_op(0, addrs, len, buf, OPCODE_NONE, NO_REP, block);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Read from PLC module repetitive at the same address
 *
 * \param addrs			Read address
 * \param bytesRepeat	Number of bytes to repeat
 * \param buf			Pointer to data buffer
 * \param len			Length of buffer to read
 * \param block			Blocking (true) /Non-Blocking (false) transfer flag
 *						If block = false, read data will be copied in buf when pplc_if_do_read() is called
 *	                                        (If another SPI transfer is made before calling pplc_if_do_read, data will be lost)
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_read_rep(uint16_t addrs, uint8_t bytesRepeat, uint8_t *buf, uint16_t len, bool block)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	uc_res = _pplc_cmd_op(0, addrs, len, buf, OPCODE_NONE, bytesRepeat, block);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Write to PLC module repetitive at the same address
 *
 * \param addrs			Write address
 * \param bytesRepeat	Number of bytes to repeat
 * \param buf			Pointer to data buffer
 * \param len			Length of buffer to send
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_write_rep(uint16_t addrs, uint8_t bytesRepeat, uint8_t *buf, uint16_t len)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	uc_res = _pplc_cmd_op(1, addrs | 0x8000, len, buf, OPCODE_NONE, bytesRepeat, false);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Write the content of buffer to PLC module
 *
 * \param addrs		Write address
 * \param buf		pointer to data buffer
 * \param len		Length of buffer to send
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_write_buf(uint16_t addrs, uint8_t *buf, uint16_t len)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	uc_res = _pplc_cmd_op(1, addrs | 0x8000, len, buf, OPCODE_NONE, NO_REP, false);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Read from PLC module on non-consecutive addresses
 *
 * \param addrs		Read address
 * \param buf		Pointer to data buffer
 * \param len		Length of buffer to read
 * \param opcode	Code which defines the jump
 * \param block		Blocking (true) /Non-Blocking (false) transfer flag
 *					If block = false, read data will be copied in buf when pplc_if_do_read() is called
 *					(If another SPI transfer is made before calling pplc_if_do_read, data will be lost)
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_read_jump(uint16_t addrs, uint8_t *buf, uint16_t len, uint8_t jump, bool block)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	uc_res = _pplc_cmd_op(0, addrs, len, buf, jump, NO_REP, block);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Write to PLC module on non-consecutive addresses
 *
 * \param addrs		Write address
 * \param buf		Pointer to data buffer
 * \param len		Length of buffer to write
 * \param opcode	Code which defines the jump
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_write_jump(uint16_t addrs, uint8_t *buf, uint16_t len, uint8_t jump)
{
	uint8_t uc_res;

	/* Enter critical section */
	Disable_global_interrupt();
	uc_res = _pplc_cmd_op(1, addrs | 0x8000, len, buf, jump, NO_REP, false);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}

	return uc_res;
}

/**
 * \brief Blocks until last SPI transaction is finished and copies rx data to buffer passed as argument in last read
 *
 * \return true if there is no error, otherwise returns false.
 */
uint8_t pplc_if_do_read(uint8_t *buf, uint16_t len)
{
	uint32_t spi_busy_cnt = 0;
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	/* Waiting transfer done*/
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RXBUFF) == 0) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			return false;
		}
	}
#endif
#if (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
	/* Wait TX and RX transfer finish */
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_TX_STATUS) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			return false;
		}
	}
	spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			return false;
		}
	}
#endif
	if (buf) {
		/* copy rcv data */
		if (len > (PDC_PPLC_BUFFER_SIZE - PPLC_HEADER_LEN)) {
			/* Protection against data corruption */
			return false;
		}

		memcpy(buf, &gs_pplc_rx_buffer[PPLC_HEADER_LEN], len);
		#if (LOG_SPI == 1)
		memcpy(&auc_buf_log_spi[us_buf_log_spi_idx - len], &gs_pplc_rx_buffer[PPLC_HEADER_LEN], len);
		#endif
		return true;
	}

	return false;
}

/**
 * \brief Apply AND mask to 8 bits register on PLC module
 *
 * \param addrs		Addres of register to apply mask
 * \param mask		Mask
 */
void pplc_if_and8(uint16_t addrs, uint8_t mask)
{
	/* Enter critical section */
	Disable_global_interrupt();
	data8 = mask;
	_pplc_cmd_op(1, addrs | 0x8000, 1, &data8, OPCODE_AND, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}
}

/**
 * \brief Apply OR mask to 8 bits register on PLC module
 *
 * \param addrs		Addres of register to apply mask
 * \param mask		Mask
 */
void pplc_if_or8(uint16_t addrs, uint8_t mask)
{
	/* Enter critical section */
	Disable_global_interrupt();
	data8 = mask;
	_pplc_cmd_op(1, addrs | 0x8000, 1, &data8, OPCODE_OR, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}
}

/**
 * \brief Apply XOR mask to 8 bits register on PLC module
 *
 * \param addrs		Addres of register to apply mask
 * \param mask		Mask
 */
void pplc_if_xor8(uint16_t addrs, uint8_t mask)
{
	/* Enter critical section */
	Disable_global_interrupt();
	data8 = mask;
	_pplc_cmd_op(1, addrs | 0x8000, 1, &data8, OPCODE_XOR, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}
}

/**
 * \brief Apply AND mask to 32 bits register on PLC module
 *
 * \param addrs		Addres of register to apply mask
 * \param ul_mask	Mask
 */
void pplc_if_and32(uint16_t addrs, uint32_t ul_mask)
{
	/* Enter critical section */
	Disable_global_interrupt();
	dataBuf32[0] = ((uint8_t)((ul_mask >> 24) & 0xFF));
	dataBuf32[1] = ((uint8_t)((ul_mask >> 16) & 0xFF));
	dataBuf32[2] = ((uint8_t)((ul_mask >> 8) & 0xFF));
	dataBuf32[3] = ((uint8_t)(ul_mask & 0xFF));

	_pplc_cmd_op(1, addrs | 0x8000, 4, dataBuf32, OPCODE_AND, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}
}

/**
 * \brief Apply OR mask to 32 bits register on PLC module
 *
 * \param addrs		Addres of register to apply mask
 * \param ul_mask	Mask
 */
void pplc_if_or32(uint16_t addrs, uint32_t ul_mask)
{
	/* Enter critical section */
	Disable_global_interrupt();
	dataBuf32[0] = ((uint8_t)((ul_mask >> 24) & 0xFF));
	dataBuf32[1] = ((uint8_t)((ul_mask >> 16) & 0xFF));
	dataBuf32[2] = ((uint8_t)((ul_mask >> 8) & 0xFF));
	dataBuf32[3] = ((uint8_t)(ul_mask & 0xFF));

	_pplc_cmd_op(1, addrs | 0x8000, 4, dataBuf32, OPCODE_OR, NO_REP, true);
	/* Exit critical section */
	if (!gs_b_disable_full_interrup) {
		Enable_global_interrupt();
	}
}

/**
 * \brief Configure ATPL250 Reset pin
 *
 */
void pplc_if_config_atpl250_reset(void)
{
	pio_set_output(PIN_RST_ATPL250_PIO, PIN_RST_ATPL250_MASK, LOW, DISABLE, ENABLE);
}

/**
 * \brief Push ATPL250 Reset pin
 *
 */
void pplc_if_push_atpl250_reset(void)
{
	pio_clear(PIN_RST_ATPL250_PIO, PIN_RST_ATPL250_MASK);
}

/**
 * \brief Release ATPL250 Reset pin
 *
 */
void pplc_if_release_atpl250_reset(void)
{
	pio_set(PIN_RST_ATPL250_PIO, PIN_RST_ATPL250_MASK);
}

/**
 * \brief Initialize PPLC interface
 *
 */
void pplc_if_init(void)
{
	/* Initialize SPI as master */
	_pplc_if_config();

	/* Reset ATPL250 */
	pplc_if_push_atpl250_reset();

	/* Configure SPI Interruption */
	pmc_enable_periph_clk(PIN_INT_SPI_ID);
	pio_handler_set(PIN_INT_SPI_PIO, PIN_INT_SPI_ID, PIN_INT_SPI_MASK, PIN_INT_SPI_ATTR, pplc_if_int_handler); /* Interrupt on falling edge */
	NVIC_SetPriority((IRQn_Type)PIN_INT_SPI_ID, SPI_PRIO);
	NVIC_EnableIRQ((IRQn_Type)PIN_INT_SPI_ID);
	pio_enable_interrupt(PIN_INT_SPI_PIO, PIN_INT_SPI_MASK);

#if (LOG_SPI == 1)
	/* Init variables */
	us_buf_log_spi_idx = 0;
	us_buf_log_lengths_idx = 0;
#endif

	/* Release ATPL250 reset */
	pplc_if_release_atpl250_reset();
}

/**
 * \brief Set an interrupt handler for the specified interrput source.
 */
void pplc_set_handler(void (*p_handler)(void))
{
	pplc_handler = p_handler;
}

/**
 * \brief Set SPI clock frequency
 *
 * \param freq		Clock frequency to assign to SPI
 */
void pplc_if_set_speed(uint32_t freq)
{
	gs_ul_pplc_clock = freq;
	uint32_t spi_busy_cnt;
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	/* Waiting transfer done*/
	spi_busy_cnt = 0;
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RXBUFF) == 0) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			break;
		}
	}
#endif
#if (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
	/* Wait TX and RX transfer finish */
	spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_TX_STATUS) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			break;
		}
	}
	spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
		spi_busy_cnt++;
		if (spi_busy_cnt > 50000) {
			break;
		}
	}
#endif
	_pplc_if_config();
}

void enable_pplc_interrupt(void)
{
	NVIC_EnableIRQ((IRQn_Type)PIN_INT_SPI_ID);
}

void disable_pplc_interrupt(void)
{
	NVIC_DisableIRQ((IRQn_Type)PIN_INT_SPI_ID);
}

void pplc_enable_all_interrupts(void)
{
	gs_b_disable_full_interrup = false;
	Enable_global_interrupt();
}

void pplc_disable_all_interrupts(void)
{
	gs_b_disable_full_interrup = true;
	Disable_global_interrupt();
}

/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
