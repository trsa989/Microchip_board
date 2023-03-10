/**
 * \file
 *
 * \brief PLC USART Buffered Interface
 *
 * Copyright (c) 2020 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
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
#include "stdarg.h"
#include "stdio.h"
#include "sysclk.h"
#include "tc.h"
#include "usart.h"
#include "pmc.h"
#include "busart_if.h"
#include "conf_busart_if.h"
#include "conf_board.h"

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

/**
 * \weakgroup busart_plc_group
 * @{
 */
#if (SAMV71 || SAMV70 || SAME70)
/* XDMAC channel status  */
#define BUSART0_XDMAC_TX_STATUS (1 << BUSART0_XDMAC_CH_TX)
#define BUSART0_XDMAC_RX_STATUS (1 << BUSART0_XDMAC_CH_RX)
#define BUSART1_XDMAC_TX_STATUS (1 << BUSART1_XDMAC_CH_TX)
#define BUSART1_XDMAC_RX_STATUS (1 << BUSART1_XDMAC_CH_RX)
#endif

#ifdef BUSART0
/* Reception Buffer 0 */
COMPILER_ALIGNED(32)
static uint8_t rx_usart_buf0[RX_BUSART0_SIZE];
/* Transmission Buffer 0 */
COMPILER_ALIGNED(32)
static uint8_t tx_usart_buf0[TX_BUSART0_SIZE];
/* Pointers to Reception Buffer 0 */
uint8_t *const ptr_rx_usart_buf0 = &rx_usart_buf0[0];
/* Pointers to Transmission Buffer 0 */
uint8_t *const ptr_tx_usart_buf0 = &tx_usart_buf0[0];
#if (SAMV71 || SAMV70 || SAME70)
/** XDMA channel configuration. */
static xdmac_channel_config_t busart0_xdmac_tx_channel_cfg;
static xdmac_channel_config_t busart0_xdmac_rx_channel_cfg;
#endif
#endif

#ifdef BUSART1
/* Reception Buffer 1 */
COMPILER_ALIGNED(32)
static uint8_t rx_usart_buf1[RX_BUSART1_SIZE];
/* Transmission Buffer 1 */
COMPILER_ALIGNED(32)
static uint8_t tx_usart_buf1[TX_BUSART1_SIZE];
/* Pointers to Reception Buffer 1 */
uint8_t *const ptr_rx_usart_buf1 = &rx_usart_buf1[0];
/* Pointers to Transmission Buffer 1 */
uint8_t *const ptr_tx_usart_buf1 = &tx_usart_buf1[0];
#if (SAMV71 || SAMV70 || SAME70)
/** XDMA channel configuration. */
static xdmac_channel_config_t busart1_xdmac_tx_channel_cfg;
static xdmac_channel_config_t busart1_xdmac_rx_channel_cfg;
#endif
#endif

/*! \brief Communications Queue Info */
typedef struct {
	/** Pointer to transmission queue. Buffer */
	uint8_t *puc_tq_buf;
	/** Pointer to reception queue. Buffer */
	uint8_t *puc_rq_buf;
	/** Reception queue. Read index */
	uint16_t us_rq_idx;
	/** Reception queue. Write index */
	uint16_t us_wq_idx;
	/** Reception queue. Occupation count */
	uint16_t us_rq_count;
} busart_comm_data_t;

/* \name Size of the receive buffer used by the PDC, in bytes */
/* @{ */
#define USART_BUFFER_SIZE                       1024
/* @} */

#ifdef BUSART0
/* Data struct to use with USART0 */
COMPILER_ALIGNED(32)
static busart_comm_data_t busart_comm_data_0;
/* Receive buffer use 2 fast buffers */
COMPILER_ALIGNED(32)
static uint8_t gs_puc_usart_buf0[USART_BUFFER_SIZE];
/* Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
/* PDC RX data packet. */
pdc_packet_t g_st_usart_rx_packet0;
/* PDC TX data packet. */
pdc_packet_t g_st_usart_tx_packet0;
/* Pointer to PDC register base. */
Pdc *g_p_usart_pdc0;
#endif
/* Number of bytes received in USART0 */
static uint16_t num_bytes_rx_usart0;
#endif

#ifdef BUSART1
/* Data struct to use with USART0 */
COMPILER_ALIGNED(32)
static busart_comm_data_t busart_comm_data_1;
/* Receive buffer use 2 fast buffers */
COMPILER_ALIGNED(32)
static uint8_t gs_puc_usart_buf1[USART_BUFFER_SIZE];
/* Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
/* PDC RX data packet. */
pdc_packet_t g_st_usart_rx_packet1;
/* PDC TX data packet. */
pdc_packet_t g_st_usart_tx_packet1;
/* Pointer to PDC register base. */
Pdc *g_p_usart_pdc1;
#endif
/* Number of bytes received in USART1 */
static uint16_t num_bytes_rx_usart1;
#endif

/* Uart channel open / closed */
static uint8_t busart_chn_open[2] = {
	false,
	false
};

#if defined(BUSART0) || defined(BUSART1)

/**
 * \brief Configure Timer Counter to generate an interrupt every 10ms.
 * This interrupt will be used to flush USART input and echo back.
 */
static void _configure_TC_usart(void)
{
	uint32_t ul_div = 1;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk;
	uint32_t ul_frec_hz = (uint32_t)FREQ_TIMER_POLL_USART;

	/* Get system clock. */
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	ul_sysclk = sysclk_get_cpu_hz();
#elif (PIC32CX)
	ul_sysclk = sysclk_get_peripheral_bus_hz(TC_USART);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
	ul_sysclk = sysclk_get_peripheral_hz();
#endif

	/* Configure PMC. */
	pmc_enable_periph_clk(ID_TC_USART);

	/* Configure TC for a TC_FREQ frequency and trigger on RC compare. */
	tc_find_mck_divisor(ul_frec_hz, ul_sysclk, &ul_div, &ul_tcclks,
			ul_sysclk);
#if (!PIC32CX)
	tc_init(TC_USART, TC_USART_CHN, ul_tcclks | TC_CMR_CPCTRG);
#else
	tc_init(TC_USART, TC_USART_CHN, ul_tcclks | TC_CMR_CPCTRG, 0);
#endif
	tc_write_rc(TC_USART, TC_USART_CHN, (ul_sysclk / ul_div) / ul_frec_hz);

	/* Configure and enable interrupt on RC compare. */
	NVIC_SetPriority((IRQn_Type)ID_TC_USART, TIMER_USART_PRIO);
	NVIC_EnableIRQ((IRQn_Type)ID_TC_USART);
	tc_enable_interrupt(TC_USART, TC_USART_CHN, TC_IER_CPCS);
}

#ifdef BUSART0

/** @brief	Interrupt handler for USART0
 *
 */
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
void BUSART0_Handler(void)
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
static void _BUSART_RX0_buffer_proc(void)
#endif
{
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
	uint32_t ul_status;
	/* Read USART Status. */
	ul_status = usart_get_status(BUSART0);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_ENDRX) {
#endif
	/* manage data */
	us_wr_idx = busart_comm_data_0.us_wq_idx;
	us_data_count = busart_comm_data_0.us_rq_count;
	us_free_size = RX_BUSART0_SIZE - us_data_count;
	if (gs_ul_size_usart_buf0 <= us_free_size) {
		/* there is enough space to write all data */
		us_end_size = RX_BUSART0_SIZE - us_wr_idx;
		if (us_end_size >= gs_ul_size_usart_buf0) {
#ifdef CONF_BOARD_ENABLE_CACHE
			SCB_InvalidateDCache_by_Addr((uint32_t *)gs_puc_usart_buf0, gs_ul_size_usart_buf0);
#endif
			/* there is no overflow of us_wq_idx */
			memcpy(&busart_comm_data_0.puc_rq_buf[us_wr_idx],
					gs_puc_usart_buf0, gs_ul_size_usart_buf0);
			/* update counters */
			busart_comm_data_0.us_rq_count += gs_ul_size_usart_buf0;
			busart_comm_data_0.us_wq_idx += gs_ul_size_usart_buf0;
		} else {
			/* there is overflow of us_wq_idx -> write in 2
			 * steps	*/
			memcpy(&busart_comm_data_0.puc_rq_buf[us_wr_idx],
					gs_puc_usart_buf0, us_end_size);
			us_part_size = gs_ul_size_usart_buf0 - us_end_size;
			memcpy(&busart_comm_data_0.puc_rq_buf[0],
					&gs_puc_usart_buf0[us_end_size], us_part_size);
			/* update counters */
			busart_comm_data_0.us_rq_count += gs_ul_size_usart_buf0;
			busart_comm_data_0.us_wq_idx = us_part_size;
		}
	} else {
		/* there is not enough space to write all data */
		tc_start(TC_USART, TC_USART_CHN);
	}

	/* change RX buffer */
	gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;

	/* Restart read on buffer. */
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
	g_st_usart_rx_packet0.ul_addr = (uint32_t)gs_puc_usart_buf0;
	g_st_usart_rx_packet0.ul_size = USART_BUFFER_SIZE;
	pdc_rx_init(g_p_usart_pdc0, &g_st_usart_rx_packet0, NULL);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
	busart0_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
	xdmac_configure_transfer(XDMAC, BUSART0_XDMAC_CH_RX, &busart0_xdmac_rx_channel_cfg);
	xdmac_channel_set_descriptor_control(XDMAC, BUSART0_XDMAC_CH_RX, 0);
	xdmac_channel_enable(XDMAC, BUSART0_XDMAC_CH_RX);
#endif

	/* Restart timer. */
	tc_start(TC_USART, TC_USART_CHN);
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
}

#endif
}

#endif /* #ifdef BUSART0 */

#ifdef BUSART1

/** @brief	Interrupt handler for USART1
 *
 */
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
void BUSART1_Handler(void)
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
static void _BUSART_RX1_buffer_proc(void)
#endif
{
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
	uint32_t ul_status;
	/* Read USART Status. */
	ul_status = usart_get_status(BUSART1);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_ENDRX) {
#endif
	/* manage data */
	us_wr_idx = busart_comm_data_1.us_wq_idx;
	us_data_count = busart_comm_data_1.us_rq_count;
	us_free_size = RX_BUSART1_SIZE - us_data_count;
	if (gs_ul_size_usart_buf1 <= us_free_size) {
		/* there is enough space to write all data */
		us_end_size = RX_BUSART1_SIZE - us_wr_idx;
		if (us_end_size >= gs_ul_size_usart_buf1) {
#ifdef CONF_BOARD_ENABLE_CACHE
			SCB_InvalidateDCache_by_Addr((uint32_t *)gs_puc_usart_buf1, gs_ul_size_usart_buf1);
#endif
			/* there is no overflow of us_wq_idx */
			memcpy(&busart_comm_data_1.puc_rq_buf[us_wr_idx],
					gs_puc_usart_buf1, gs_ul_size_usart_buf1);
			/* update counters */
			busart_comm_data_1.us_rq_count += gs_ul_size_usart_buf1;
			busart_comm_data_1.us_wq_idx += gs_ul_size_usart_buf1;
		} else {
			/* there is overflow of us_wq_idx -> write in 2
			 * steps	*/
			memcpy(&busart_comm_data_1.puc_rq_buf[us_wr_idx],
					gs_puc_usart_buf1, us_end_size);
			us_part_size = gs_ul_size_usart_buf1 - us_end_size;
			memcpy(&busart_comm_data_1.puc_rq_buf[0],
					&gs_puc_usart_buf1[us_end_size], us_part_size);
			/* update counters */
			busart_comm_data_1.us_rq_count += gs_ul_size_usart_buf1;
			busart_comm_data_1.us_wq_idx = us_part_size;
		}
	} else {
		/* there is not enough space to write all data */
		tc_start(TC_USART, TC_USART_CHN);
	}

	/* change RX buffer */
	gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;

	/* Restart read on buffer. */
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
	g_st_usart_rx_packet1.ul_addr = (uint32_t)gs_puc_usart_buf1;
	g_st_usart_rx_packet1.ul_size = USART_BUFFER_SIZE;
	pdc_rx_init(g_p_usart_pdc1, &g_st_usart_rx_packet1, NULL);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
	busart1_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
	xdmac_configure_transfer(XDMAC, BUSART1_XDMAC_CH_RX, &busart1_xdmac_rx_channel_cfg);
	xdmac_channel_set_descriptor_control(XDMAC, BUSART1_XDMAC_CH_RX, 0);
	xdmac_channel_enable(XDMAC, BUSART1_XDMAC_CH_RX);
#endif
	/* Restart timer. */
	tc_start(TC_USART, TC_USART_CHN);
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
}

#endif
}
#endif /* #ifdef BUSART1 */
#endif

#if defined(BUSART0) || defined(BUSART1)

/**
 * \brief Interrupt handler. Record the number of bytes received,
 * and then restart a read transfer on the USART if the transfer was stopped.
 */
void TC_USART_Handler(void)
{
	uint32_t ul_status;
#if defined(BUSART0) || defined(BUSART1)
	uint32_t ul_byte_total = 0;
#endif

	/* Read TC_USART Status. */
	ul_status = tc_get_status(TC_USART, TC_USART_CHN);

	/* RC compare. */
	if ((ul_status & TC_SR_CPCS) == TC_SR_CPCS) {
#ifdef BUSART0
		if (busart_chn_open[0]) {
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
			/* Flush PDC buffer. */
			ul_byte_total = USART_BUFFER_SIZE - pdc_read_rx_counter(g_p_usart_pdc0);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
			/* Read received bytes from uB ctrl register */
			ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART0_XDMAC_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
#endif
			if (ul_byte_total > 0) {
#if (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
				/* Flush the received bytes buffer */
				xdmac_channel_software_flush_request(XDMAC, BUSART0_XDMAC_CH_RX);
				/* Read received bytes from uB ctrl register */
				ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART0_XDMAC_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
#endif
				if (ul_byte_total == num_bytes_rx_usart0) {
					/* Disable timer. */
					tc_stop(TC_USART, TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf0 = ul_byte_total;

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
					/* Stop DMA USART_RX -> force Uart Handler*/
					g_st_usart_rx_packet0.ul_size = 0;
					pdc_rx_init(g_p_usart_pdc0, &g_st_usart_rx_packet0, NULL);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
					/* Process RX buffer */
					_BUSART_RX0_buffer_proc();
#endif
				} else {
					num_bytes_rx_usart0 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart0 = 0;
			}
		}
#endif

#ifdef BUSART1
		if (busart_chn_open[1]) {
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
			/* Flush PDC buffer. */
			ul_byte_total = USART_BUFFER_SIZE - pdc_read_rx_counter(g_p_usart_pdc1);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
			/* Read received bytes from uB ctrl register */
			ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART1_XDMAC_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
#endif
			if (ul_byte_total > 0) {
#if (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
				/* Flush the received bytes buffer */
				xdmac_channel_software_flush_request(XDMAC, BUSART1_XDMAC_CH_RX);
				/* Read received bytes from uB ctrl register */
				ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART1_XDMAC_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
#endif
				if (ul_byte_total == num_bytes_rx_usart1) {
					/* Disable timer. */
					tc_stop(TC_USART, TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf1 = ul_byte_total;

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
					/* Stop DMA USART_RX -> force Uart Handler*/
					g_st_usart_rx_packet1.ul_size = 0;
					pdc_rx_init(g_p_usart_pdc1, &g_st_usart_rx_packet1, NULL);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
					/* Process RX buffer */
					_BUSART_RX1_buffer_proc();
#endif
				} else {
					num_bytes_rx_usart1 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart1 = 0;
			}
		}
#endif
	}
}

#endif

/**
 * \brief This function opens an USART
 *
 * \note Opening of the specified USART implies initializing local variables and
 * opening required hardware with the following configuration:
 * - bauds as specified
 * - 8 bits, no parity, 1 stop bit
 * - enable interrupts
 *
 * \param chn			Communication channel [0, 1]
 * \param bauds			Communication speed in bauds
 *
 * \retval true on success.
 * \retval false on failure.
 */
int8_t busart_if_open(uint8_t chn, uint32_t bauds)
{
#if defined(BUSART0) || defined(BUSART1)
	sam_usart_opt_t usart_console_settings;

	/* Expected baud rate. */
	usart_console_settings.baudrate = bauds;

	/* Configure channel mode (Normal, Automatic, Local_loopback or
	 * Remote_loopback) */
	usart_console_settings.channel_mode = US_MR_CHMODE_NORMAL;
	/* Initialize value for USART mode register */
	usart_console_settings.parity_type = US_MR_PAR_NO;
	usart_console_settings.char_length = US_MR_CHRL_8_BIT;
	usart_console_settings.stop_bits = US_MR_NBSTOP_1_BIT;
#else
	UNUSED(bauds);
#endif
	/* check usart and it is close */
	if (chn >= 2) {
		return false;
	}

	if (busart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef BUSART0
	case 0:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_BUSART0);
		/* Configure USART. */
		usart_init_rs232(BUSART0, &usart_console_settings,
				sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_0.puc_tq_buf = ptr_tx_usart_buf0;
		busart_comm_data_0.puc_rq_buf = ptr_rx_usart_buf0;
		busart_comm_data_0.us_rq_count = 0;
		busart_comm_data_0.us_rq_idx = 0;
		busart_comm_data_0.us_wq_idx = 0;

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)

		/* Get board BUSART0 PDC base address and enable receiver and
		 * transmitter. */
		g_p_usart_pdc0 = usart_get_pdc_base(BUSART0);
		pdc_enable_transfer(g_p_usart_pdc0,
				PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

		/* Start receiving data and start timer. */
		g_st_usart_rx_packet0.ul_addr = (uint32_t)gs_puc_usart_buf0;
		g_st_usart_rx_packet0.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc0, &g_st_usart_rx_packet0, NULL);

		/* Stop transmitting data */
		g_st_usart_tx_packet0.ul_addr = (uint32_t)busart_comm_data_0.puc_tq_buf;
		g_st_usart_tx_packet0.ul_size = 0;
		pdc_tx_init(g_p_usart_pdc0, &g_st_usart_tx_packet0, NULL);

		gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;

		/* Transfer to PDC communication mode, disable RXRDY interrupt
		 * and enable RXBUFF interrupt. */
		usart_disable_interrupt(BUSART0, US_IDR_RXRDY);
		usart_enable_interrupt(BUSART0, US_IER_RXBUFF);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		/* Initialize and enable DMA controller */
		pmc_enable_periph_clk(ID_XDMAC);
		/* Configure TX and RX XDMAC channels */
		busart0_xdmac_tx_channel_cfg.mbr_ubc = 0;
		busart0_xdmac_tx_channel_cfg.mbr_sa = (uint32_t)busart_comm_data_0.puc_tq_buf;
		busart0_xdmac_tx_channel_cfg.mbr_da = (uint32_t)(&(BUSART0->US_THR));
		busart0_xdmac_tx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
				XDMAC_CC_DSYNC_MEM2PER |
				XDMAC_CC_PERID(BUSART0_XDMAC_TX_PERID) |
				XDMAC_CC_CSIZE_CHK_1 |
				XDMAC_CC_MEMSET_NORMAL_MODE |
				XDMAC_CC_MBSIZE_SINGLE |
				XDMAC_CC_DWIDTH_BYTE |
				XDMAC_CC_SIF_AHB_IF0 |
				XDMAC_CC_DIF_AHB_IF1 |
				XDMAC_CC_SAM_INCREMENTED_AM |
				XDMAC_CC_DAM_FIXED_AM;
		busart0_xdmac_tx_channel_cfg.mbr_bc = 0;
		busart0_xdmac_tx_channel_cfg.mbr_ds = 0;
		busart0_xdmac_tx_channel_cfg.mbr_sus = 0;
		busart0_xdmac_tx_channel_cfg.mbr_dus = 0;

		busart0_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
		busart0_xdmac_rx_channel_cfg.mbr_sa = (uint32_t)(&(BUSART0->US_RHR));
		busart0_xdmac_rx_channel_cfg.mbr_da = (uint32_t)gs_puc_usart_buf0;
		busart0_xdmac_rx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
				XDMAC_CC_DSYNC_PER2MEM |
				XDMAC_CC_PERID(BUSART0_XDMAC_RX_PERID) |
				XDMAC_CC_CSIZE_CHK_1 |
				XDMAC_CC_MEMSET_NORMAL_MODE |
				XDMAC_CC_MBSIZE_SINGLE |
				XDMAC_CC_DWIDTH_BYTE |
				XDMAC_CC_SIF_AHB_IF1 |
				XDMAC_CC_DIF_AHB_IF0 |
				XDMAC_CC_SAM_FIXED_AM |
				XDMAC_CC_DAM_INCREMENTED_AM;
		busart0_xdmac_rx_channel_cfg.mbr_bc = 0;
		busart0_xdmac_rx_channel_cfg.mbr_ds =  0;
		busart0_xdmac_rx_channel_cfg.mbr_sus = 0;
		busart0_xdmac_rx_channel_cfg.mbr_dus = 0;

		xdmac_configure_transfer(XDMAC, BUSART0_XDMAC_CH_TX, &busart0_xdmac_tx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART0_XDMAC_CH_TX, 0);
		xdmac_configure_transfer(XDMAC, BUSART0_XDMAC_CH_RX, &busart0_xdmac_rx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART0_XDMAC_CH_RX, 0);

		/* Transfer to XDMAC communication mode, disable RXRDY interrupt. */
		usart_disable_interrupt(BUSART0, US_IDR_RXRDY);

		xdmac_channel_enable(XDMAC, BUSART0_XDMAC_CH_TX);
		xdmac_channel_enable(XDMAC, BUSART0_XDMAC_CH_RX);
#endif
		/* Enable the receiver and transmitter. */
		usart_enable_tx(BUSART0);
		usart_enable_rx(BUSART0);

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)BUSART0_IRQn, USART0_PRIO);
		NVIC_EnableIRQ(BUSART0_IRQn);
#endif

		busart_chn_open[chn] = true;
		num_bytes_rx_usart0 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(TC_USART, TC_USART_CHN);

		return true;
	}
	break;
#endif
#ifdef BUSART1
	case 1:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_BUSART1);
		/* Configure USART. */
		usart_init_rs232(BUSART1, &usart_console_settings,
				sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_1.puc_tq_buf = ptr_tx_usart_buf1;
		busart_comm_data_1.puc_rq_buf = ptr_rx_usart_buf1;
		busart_comm_data_1.us_rq_count = 0;
		busart_comm_data_1.us_rq_idx = 0;
		busart_comm_data_1.us_wq_idx = 0;

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)

		/* Get board BUSART1 PDC base address and enable receiver and
		 * transmitter. */
		g_p_usart_pdc1 = usart_get_pdc_base(BUSART1);
		pdc_enable_transfer(g_p_usart_pdc1,
				PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

		/* Start receiving data and start timer. */
		g_st_usart_rx_packet1.ul_addr = (uint32_t)gs_puc_usart_buf1;
		g_st_usart_rx_packet1.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc1, &g_st_usart_rx_packet1, NULL);

		/* Stop transmitting data */
		g_st_usart_tx_packet1.ul_addr = (uint32_t)busart_comm_data_1.puc_tq_buf;
		g_st_usart_tx_packet1.ul_size = 0;
		pdc_tx_init(g_p_usart_pdc1, &g_st_usart_tx_packet1, NULL);

		gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;

		/* Transfer to PDC communication mode, disable RXRDY interrupt
		 * and enable RXBUFF interrupt. */
		usart_disable_interrupt(BUSART1, US_IDR_RXRDY);
		usart_enable_interrupt(BUSART1, US_IER_RXBUFF);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		/* Configure TX and RX XDMAC channels */
		busart1_xdmac_tx_channel_cfg.mbr_ubc = 0;
		busart1_xdmac_tx_channel_cfg.mbr_sa = (uint32_t)busart_comm_data_1.puc_tq_buf;
		busart1_xdmac_tx_channel_cfg.mbr_da = (uint32_t)(&(BUSART1->US_THR));
		busart1_xdmac_tx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
				XDMAC_CC_DSYNC_MEM2PER |
				XDMAC_CC_PERID(BUSART1_XDMAC_TX_PERID) |
				XDMAC_CC_CSIZE_CHK_1 |
				XDMAC_CC_MEMSET_NORMAL_MODE |
				XDMAC_CC_MBSIZE_SINGLE |
				XDMAC_CC_DWIDTH_BYTE |
				XDMAC_CC_SIF_AHB_IF0 |
				XDMAC_CC_DIF_AHB_IF1 |
				XDMAC_CC_SAM_INCREMENTED_AM |
				XDMAC_CC_DAM_FIXED_AM;
		busart1_xdmac_tx_channel_cfg.mbr_bc = 0;
		busart1_xdmac_tx_channel_cfg.mbr_ds = 0;
		busart1_xdmac_tx_channel_cfg.mbr_sus = 0;
		busart1_xdmac_tx_channel_cfg.mbr_dus = 0;

		busart1_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
		busart1_xdmac_rx_channel_cfg.mbr_sa = (uint32_t)(&(BUSART1->US_RHR));
		busart1_xdmac_rx_channel_cfg.mbr_da = (uint32_t)gs_puc_usart_buf1;
		busart1_xdmac_rx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
				XDMAC_CC_DSYNC_PER2MEM |
				XDMAC_CC_PERID(BUSART1_XDMAC_RX_PERID) |
				XDMAC_CC_CSIZE_CHK_1 |
				XDMAC_CC_MEMSET_NORMAL_MODE |
				XDMAC_CC_MBSIZE_SINGLE |
				XDMAC_CC_DWIDTH_BYTE |
				XDMAC_CC_SIF_AHB_IF1 |
				XDMAC_CC_DIF_AHB_IF0 |
				XDMAC_CC_SAM_FIXED_AM |
				XDMAC_CC_DAM_INCREMENTED_AM;
		busart1_xdmac_rx_channel_cfg.mbr_bc = 0;
		busart1_xdmac_rx_channel_cfg.mbr_ds =  0;
		busart1_xdmac_rx_channel_cfg.mbr_sus = 0;
		busart1_xdmac_rx_channel_cfg.mbr_dus = 0;

		xdmac_configure_transfer(XDMAC, BUSART1_XDMAC_CH_TX, &busart1_xdmac_tx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART1_XDMAC_CH_TX, 0);
		xdmac_configure_transfer(XDMAC, BUSART1_XDMAC_CH_RX, &busart1_xdmac_rx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART1_XDMAC_CH_RX, 0);

		/* Transfer to XDMAC communication mode, disable RXRDY interrupt. */
		usart_disable_interrupt(BUSART1, US_IDR_RXRDY);

		xdmac_channel_enable(XDMAC, BUSART1_XDMAC_CH_TX);
		xdmac_channel_enable(XDMAC, BUSART1_XDMAC_CH_RX);
#endif

		/* Enable the receiver and transmitter. */
		usart_enable_tx(BUSART1);
		usart_enable_rx(BUSART1);

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)BUSART1_IRQn, USART1_PRIO);
		NVIC_EnableIRQ(BUSART1_IRQn);
#endif

		busart_chn_open[chn] = true;
		num_bytes_rx_usart1 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(TC_USART, TC_USART_CHN);

		return true;
	}
	break;
#endif

	default:
		return false;
	}
}

/**
 * \brief This function closes and disables communication in the specified
 * USART.
 *
 * \param chn  Communication channel [0, 1]
 *
 * \retval true on success.
 * \retval false on failure.
 */
int8_t busart_if_close(uint8_t chn)
{
	if (chn >= 2) {
		return false;
	}

	if (!busart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef BUSART0
	case 0:
	{
		usart_disable_tx(BUSART0);
		usart_disable_rx(BUSART0);
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		usart_disable_interrupt(BUSART0, US_IDR_RXRDY);
		usart_disable_interrupt(BUSART0, US_IER_ENDRX);
#endif
		/* Stop TC */
		if (!busart_chn_open[1]) {
			tc_stop(TC_USART, TC_USART_CHN);
		}

		busart_chn_open[chn] = false;
		return true;
	}
	break;
#endif

#ifdef BUSART1
	case 1:
	{
		usart_disable_tx(BUSART1);
		usart_disable_rx(BUSART1);

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		usart_disable_interrupt(BUSART1, US_IDR_RXRDY);
		usart_disable_interrupt(BUSART1, US_IER_ENDRX);
#endif

		/* Stop TC */
		if (!busart_chn_open[0]) {
			tc_stop(TC_USART, TC_USART_CHN);
		}

		busart_chn_open[chn] = false;
		return true;
	}
	break;
#endif

	default:
		return false;
	}
}

/**
 * \brief This function receives a message.
 *
 * \note This function receives a given number of characters from the specified
 * USART. If so configured, the function waits until all characters are
 * received. In this case, the watchdog timer must be reloaded to avoid a
 * program reset.
 *
 * \param  chn     Communication channel [0, 1]
 * \param  buffer  Pointer to buffer for information
 * \param  len     Number of characters to receive
 *
 * \retval Number of received characters
 */
uint16_t busart_if_read(uint8_t chn, void *buffer, uint16_t len)
{
#if defined(BUSART0) || defined(BUSART1)
	uint16_t us_rd_idx = 0;
	uint16_t us_num_bytes_read, us_num_bytes_to_end, us_num_bytes_to_start;
	uint16_t us_total_pos;
	uint16_t us_buf_size;

	uint8_t *msg = (uint8_t *)buffer;
#else
	UNUSED(buffer);
	UNUSED(len);
#endif
	if (chn >= 2) {
		return 0;
	}

	/* check usart is open */
	if (!busart_chn_open[chn]) {
		return 0;
	}

	switch (chn) {
#ifdef BUSART0
	case 0:
		us_buf_size = RX_BUSART0_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_0.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_0.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_0.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_0.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
#ifdef CONF_BOARD_ENABLE_CACHE
		SCB_InvalidateDCache_by_Addr((uint32_t *)busart_comm_data_0.puc_rq_buf + us_rd_idx, us_num_bytes_read);
#endif
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[us_rd_idx],
					us_num_bytes_read);
			/* update counters */
			busart_comm_data_0.us_rq_count -= us_num_bytes_read;
			busart_comm_data_0.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[us_rd_idx],
					us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[0],
					us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_0.us_rq_count -= us_num_bytes_read;
			busart_comm_data_0.us_rq_idx = us_num_bytes_to_start;
		}

		return us_num_bytes_read;
#endif

#ifdef BUSART1
	case 1:
		us_buf_size = RX_BUSART1_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_1.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_1.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_1.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_1.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
#ifdef CONF_BOARD_ENABLE_CACHE
		SCB_InvalidateDCache_by_Addr((uint32_t *)busart_comm_data_1.puc_rq_buf + us_rd_idx, us_num_bytes_read);
#endif
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[us_rd_idx],
					us_num_bytes_read);
			/* update counters */
			busart_comm_data_1.us_rq_count -= us_num_bytes_read;
			busart_comm_data_1.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[us_rd_idx],
					us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[0],
					us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_1.us_rq_count -= us_num_bytes_read;
			busart_comm_data_1.us_rq_idx = us_num_bytes_to_start;
		}
		return us_num_bytes_read;
#endif

	default:
		return 0;
	}
}

/**
 * \brief This function transmits a message.
 *
 * \note This function transmits characters via the specified USART.
 * If so configured, the function waits until all characters are inserted
 * in the transmission queue. In this case, the watchdog timer must be
 * reloaded to avoid a program reset.
 *
 * \param  chn     Communication channel [0, 1]
 * \param  buffer  Pointer to information to transmit
 * \param  len     Number of characters to transmit
 *
 * \retval Number of characters sent
 */
uint16_t busart_if_write(uint8_t chn, const void *buffer, uint16_t len)
{
#if !defined(BUSART0) && !defined(BUSART1)
	UNUSED(buffer);
	UNUSED(len);
#endif
	if (chn >= 2) {
		return 0;
	}

	/* check usart is open */
	if (!busart_chn_open[chn]) {
		return 0;
	}

	switch (chn) {
#ifdef BUSART0
	case 0:
		if (len > TX_BUSART0_SIZE) {
			return 0;
		}

		/* Wait previous TX transfer finish */
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		while (pdc_read_tx_counter(g_p_usart_pdc0) > 0) {
		}
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		while (xdmac_channel_get_status(XDMAC) & BUSART0_XDMAC_TX_STATUS) {
		}
#endif
		memcpy(&busart_comm_data_0.puc_tq_buf[0], buffer, len);
#ifdef CONF_BOARD_ENABLE_CACHE
		SCB_InvalidateDCache_by_Addr((uint32_t *)busart_comm_data_0.puc_tq_buf, len);
#endif
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		g_st_usart_tx_packet0.ul_addr = (uint32_t)&busart_comm_data_0.puc_tq_buf[0];
		g_st_usart_tx_packet0.ul_size = len;
		pdc_tx_init(g_p_usart_pdc0, &g_st_usart_tx_packet0, NULL);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		busart0_xdmac_tx_channel_cfg.mbr_ubc = len;
		xdmac_configure_transfer(XDMAC, BUSART0_XDMAC_CH_TX, &busart0_xdmac_tx_channel_cfg);
		xdmac_channel_enable(XDMAC, BUSART0_XDMAC_CH_TX);
#endif
		return len;
#endif

#ifdef BUSART1
	case 1:
		if (len > TX_BUSART1_SIZE) {
			return 0;
		}

		/* Wait previous TX transfer finish */
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		while (pdc_read_tx_counter(g_p_usart_pdc1) > 0) {
		}
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		while (xdmac_channel_get_status(XDMAC) & BUSART1_XDMAC_TX_STATUS) {
		}
#endif
		memcpy(&busart_comm_data_1.puc_tq_buf[0], buffer, len);
#ifdef CONF_BOARD_ENABLE_CACHE
		SCB_InvalidateDCache_by_Addr((uint32_t *)busart_comm_data_1.puc_tq_buf, len);
#endif
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		g_st_usart_tx_packet1.ul_addr = (uint32_t)&busart_comm_data_1.puc_tq_buf[0];
		g_st_usart_tx_packet1.ul_size = len;
		pdc_tx_init(g_p_usart_pdc1, &g_st_usart_tx_packet1, NULL);
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		busart1_xdmac_tx_channel_cfg.mbr_ubc = len;
		xdmac_configure_transfer(XDMAC, BUSART1_XDMAC_CH_TX, &busart1_xdmac_tx_channel_cfg);
		xdmac_channel_enable(XDMAC, BUSART1_XDMAC_CH_TX);
#endif
		return len;
#endif

	default:
		return 0;
	}
}

/**
 * \brief Get byte from USART.
 *
 * \param  chn  Communication channel [0, 1]
 *
 * \retval Byte received
 * \retval -1 in case of no reception
 */
int busart_if_rx_char(uint8_t chn)
{
	uint8_t buf[4] = {0, 0, 0, 0};

	if (busart_if_read(chn, buf, 1) <= 0) {
		return (-1);
	}

	return buf[0];
}

/**
 * \brief Sent byte to USART.
 *
 * \param  chn   Communication channel [0, 1]
 * \param  data  Data to sent
 *
 * \retval Number of characters sent
 */
uint16_t busart_if_tx_char(uint8_t chn, char data)
{
	return (busart_if_write(chn, &data, 1));
}

/* @} */

/**
 * \brief Check USART PDC transmission in course.
 *
 * \param  chn   Communication channel [0, 1]
 *
 * \retval true is USART is free to tx, false in otherwise
 */
bool busart_if_is_free(uint8_t chn)
{
	if (chn >= 2) {
		return false;
	}

	/* check uart is open */
	if (!busart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef BUSART0
	case 0:
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		if (pdc_read_tx_counter(g_p_usart_pdc0)) {
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		if (USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART0_XDMAC_CH_RX].XDMAC_CUBC & 0x00FFFFFF) {
#endif
			return false;
		} else {
			return true;
		}
#endif

#ifdef BUSART1
	case 1:
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM || PIC32CX)
		if (pdc_read_tx_counter(g_p_usart_pdc1)) {
#elif (SAM3U  || SAM3XA || SAMV71 || SAMV70 || SAME70 || SAMS70)
		if (USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART1_XDMAC_CH_RX].XDMAC_CUBC & 0x00FFFFFF) {
#endif
			return false;
		} else {
			return true;
		}
#endif

	default:
		return false;
	}
}

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
