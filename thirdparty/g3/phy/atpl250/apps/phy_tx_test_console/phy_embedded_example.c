/**
 * \file
 *
 * \brief ATMEL PLC PHY TX Test Console Application
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

/* System includes */
#include <stdint.h>
#include <stdio.h>
#include "string.h"

/* ASF includes */
#include "asf.h"

/* App includes */
#include "conf_project.h"
#include "phy_embedded_example.h"
/* Phy includes */
#include "atpl250.h"

static uint8_t uc_data_buf[800];    /*  Receive working buffer */

static uint8_t uc_tx_enabled = 0;

/* TX configuration parameters */
txPhyEmbeddedConfig_t xAppPhyCfgTx;

/* Tx data buffer */
extern uint8_t ucv_tx_data_buffer[512];

/**
 * \internal
 * \brief Callback to capture end of transmission and manage the result of transmission.
 *
 * \param px_tx_result Pointer to Transmission result struct
 */
static void tx_test_console_cb_phy_data_confirm(xPhyMsgTxResult_t *x_write_result)
{
	xAppPhyCfgTx.uc_tx_result_flag = x_write_result->e_tx_result;
	xAppPhyCfgTx.ul_tx_end_time = x_write_result->m_ul_end_tx_time;

	if ((xAppPhyCfgTx.uc_tx_result_flag != PHY_TX_RESULT_PROCESS) && (uc_tx_enabled)) {
		run_tx_task();
		if (xAppPhyCfgTx.uc_is_random) {
			fill_msg_random();
		}
	}
}

/**
 * \internal
 * \brief Callback to capture frame reception and manage the serialization through USI.
 *
 * \param px_msg Pointer to Reception struct
 */
static void tx_test_console_cb_phy_data_indication(xPhyMsgRx_t *x_read_msg)
{
	uint32_t ul_wait_counter;
	/* set pointer to reception data buffer */
	x_read_msg->m_puc_data_buf = uc_data_buf;

	/* build response */
	if (x_read_msg->m_us_data_len) {
		/* blink Reception LED */
	#if (BOARD == SAM4CMP_DB || BOARD == SAM4CMS_DB)
		/* Do Nothing */
	#else
		LED_On(LED1);
	#endif
		ul_wait_counter = 0xFFFF;
		while (ul_wait_counter--) {
		}
	#if (BOARD == SAM4CMP_DB || BOARD == SAM4CMS_DB)
		/* Do Nothing */
	#else
		LED_Off(LED1);
	#endif
	}
}

static struct TPhyCallbacks g_serial_if_phy_callbacks = {
	tx_test_console_cb_phy_data_confirm,
	tx_test_console_cb_phy_data_indication
};

void  app_init(void)
{
	/* Set phy callbacks */
	phy_set_callbacks(&g_serial_if_phy_callbacks);

	xAppPhyCfgTx.ul_tx_end_time = 0;
	/*      phy_set_cfg_param(PHY_ID_CFG_COUPLING_BOARD, &xTxPhyCfg.uc_coupling, 1); */
	/*      phy_set_cfg_param(PHY_ID_CFG_TXRX_CHANNEL, &xTxPhyCfg.uc_channel, 1); */
	/*      if (xTxPhyCfg.uc_autodetect) { */
	/*              phy_set_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &xTxPhyCfg.uc_autodetect, 1); */
	/*              xTxPhyCfg.uc_impedance = HI_STATE; */
	/*              phy_set_cfg_param(PHY_ID_CFG_IMPEDANCE, &xTxPhyCfg.uc_impedance, 1); */
	/*      } else { */
	/*              phy_set_cfg_param(PHY_ID_CFG_IMPEDANCE, &xTxPhyCfg.uc_impedance, 1); */
	/*              xTxPhyCfg.uc_autodetect = false; */
	/*              phy_set_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &xTxPhyCfg.uc_autodetect, 1); */
	/*      } */

	/* Init RESET check validation value */
	/*      ul_check_reset_value = APP_CHECK_RESET_VALUE; */
	/*      phy_set_cfg_param(0xFFAC, &ul_check_reset_value, sizeof(ul_check_reset_value)); */
}

void run_tx_task()
{
	uint32_t ul_curr_time;
	uint32_t ul_prog_time;
	uint32_t ul_end_time = xAppPhyCfgTx.ul_tx_end_time;

	if (xAppPhyCfgTx.xPhyMsg.m_uc_tx_mode & TX_MODE_DELAYED_TX) {
		ul_curr_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
		ul_prog_time = ul_end_time + xAppPhyCfgTx.ul_tx_period;

		if (ul_curr_time > ul_prog_time) {
			xAppPhyCfgTx.xPhyMsg.m_ul_tx_time =     ul_curr_time + xAppPhyCfgTx.ul_tx_period;
		} else {
			xAppPhyCfgTx.xPhyMsg.m_ul_tx_time =     ul_prog_time;
		}
	}

	xAppPhyCfgTx.uc_tx_result_flag = phy_tx_frame(&xAppPhyCfgTx.xPhyMsg);
}

void enable_tx(void)
{
	uc_tx_enabled = 1;
}

void disable_tx(void)
{
	uc_tx_enabled = 0;
}

uint8_t enabled_tx(void)
{
	return uc_tx_enabled;
}

/**
 * Fill data message in random mode.
 */
void fill_msg_random(void)
{
	uint8_t *p_data_buf;
	uint16_t us_len;
	uint32_t ul_random_num;

	/* Asgin pointer to tx data buffer */
	p_data_buf = ucv_tx_data_buffer;

	/* init vars */
	us_len = xAppPhyCfgTx.xPhyMsg.m_us_data_len;

#if (SAM4C)
	/* Configure PMC */
	pmc_enable_periph_clk(ID_TRNG);
	/* Enable TRNG */
	trng_enable(TRNG);
#else
	srand(pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32));
#endif
	/* fill message */
	while (us_len) {
	#if (SAM4C)
		while ((trng_get_interrupt_status(TRNG) & TRNG_ISR_DATRDY) != TRNG_ISR_DATRDY) {
		}
		ul_random_num = trng_read_output_data(TRNG);
	#else
		ul_random_num = rand();
	#endif

		*p_data_buf++ = (uint8_t)ul_random_num;
		if (!us_len--) {
			break;
		}

		*p_data_buf++ = (uint8_t)(ul_random_num >> 8);
		if (!us_len--) {
			break;
		}

		*p_data_buf++ = (uint8_t)(ul_random_num >> 16);
		if (!us_len--) {
			break;
		}

		*p_data_buf++ = (uint8_t)(ul_random_num >> 24);
		if (!us_len--) {
			break;
		}
	}
}
