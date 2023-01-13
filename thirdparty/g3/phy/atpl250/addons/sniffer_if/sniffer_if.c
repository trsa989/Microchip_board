/**
 * \file
 *
 * \brief ATPL230 Sniffer Interface for Physical layer
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "string.h"

/* Phy includes */
#include "atpl250.h"

/* Sniffer interface */
#include "conf_usi.h"
#include "sniffer_if.h"
#include "usi.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \weakgroup sniffer_plc_group
 * @{
 */

#ifdef ENABLE_SNIFFER

/* ! \name Data structure to communicate with USI layer */
/* @{ */
static x_usi_serial_cmd_params_t x_phy_sniffer_msg;
/* @} */

/*! \name Data buffers */
/* @{ */
static uint8_t uc_sniffer_rsp_buf[808];     /* !<  Response working buffer */
/* @} */

/* First and last carriers depending on working band */
static uint8_t uc_first_carrier;
static uint8_t uc_last_carrier;

/* Extern variables needed */
extern uint8_t uc_working_band;

/**
 * \internal
 * \brief Callback to capture end of transmission and manage the serialization of the result of transmission through USI.
 *
 * \param px_tx_result Pointer to Transmission result struct
 */

static void sniffer_if_cb_phy_data_indication(xPhyMsgRx_t *px_msg)
{
	uint16_t us_sniffer_response_len;
	uint32_t ul_timeIni, ul_timeEnd;
	struct s_rx_ber_payload_data_t s_ber_data;
	uint16_t us_num_symbols;
	uint16_t us_j;

	/* build response */
	if (px_msg->m_us_data_len) {
		us_sniffer_response_len = 0;

		uc_sniffer_rsp_buf[us_sniffer_response_len++] = SNIFFER_IF_PHY_COMMAND_G3_VERSION;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = SNIFFER_VERSION;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = SNIFFER_ATPL250;

		/* Fill data depending on Data/ACK frame */
		if ((px_msg->e_delimiter_type == DT_SOF_NO_RESP) || (px_msg->e_delimiter_type == DT_SOF_RESP)) {
			/* Data frame */
			/* ModType (high) + ModScheme (low) */
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)((px_msg->e_mod_scheme & 0x0F) + (((px_msg->e_mod_type) << 4) & 0xF0));
			/* Tone Map */
			if ((uc_working_band == WB_CENELEC_A) || (uc_working_band == WB_CENELEC_B)) {
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_msg->m_auc_tone_map[0];
			} else {
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_msg->m_auc_tone_map[2];
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_msg->m_auc_tone_map[1];
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_msg->m_auc_tone_map[0];
			}

			/* Number of symbols (2 bytes) */
			phy_get_cfg_param(PHY_ID_LAST_RX_MSG_PAYLOAD_SYMBOLS, &us_num_symbols, 2);
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_num_symbols >> 8);
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_num_symbols);
			/* SNR */
			phy_get_cfg_param(PHY_ID_BER_DATA, (void *)&s_ber_data, sizeof(s_ber_data));
			us_j = 0;
			while (s_ber_data.uc_valid_data == 0) {
				/* Protection from trap */
				us_j++;
				if (us_j == 10000) {
					break;
				}

				/* keep asking until valid data is obtained */
				phy_get_cfg_param(PHY_ID_BER_DATA, (void *)&s_ber_data, sizeof(s_ber_data));
			}
			if (us_j < 10000) {
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = s_ber_data.uc_lqi;
			} else {
				uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			}
		} else {
			/* ACK */
			/* ModType (high) + ModScheme (low) */
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			/* Tone Map */
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			/* Number of symbols (2 bytes) */
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			/* SNR */
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0xFF;
		}

		/* Delimiter Type */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_msg->e_delimiter_type;
		/* TimeIni */
		ul_timeIni = px_msg->m_ul_rx_time - px_msg->m_ul_frame_duration;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeIni >> 24);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeIni >> 16) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeIni >> 8) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = ul_timeIni & 0xFF;
		/* TimeEnd */
		ul_timeEnd = px_msg->m_ul_rx_time;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeEnd >> 24);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeEnd >> 16) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeEnd >> 8) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = ul_timeEnd & 0xFF;
		/* RSSI */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_msg->m_us_rssi >> 8);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_msg->m_us_rssi);
		/* AGC_Factor */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_msg->m_us_agc_factor >> 8);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_msg->m_us_agc_factor);
		/* Length in bytes */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_msg->m_us_data_len >> 8);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_msg->m_us_data_len);
		/* PDU */
		memcpy(&uc_sniffer_rsp_buf[us_sniffer_response_len], px_msg->m_puc_data_buf, px_msg->m_us_data_len);
		us_sniffer_response_len += px_msg->m_us_data_len;

		/* set usi parameters */
		x_phy_sniffer_msg.ptr_buf = &uc_sniffer_rsp_buf[0];
		x_phy_sniffer_msg.us_len = us_sniffer_response_len;
		usi_send_cmd(&x_phy_sniffer_msg);
	}
}

static void sniffer_if_cb_phy_data_confirm(xPhyMsgTxResult_t *px_tx_result, xPhyMsgTx_t *px_tx_param)
{
	uint16_t us_sniffer_response_len;
	uint32_t ul_timeIni, ul_timeEnd;
	uint16_t us_rssi = 0xFFFF;
	uint16_t us_agc_factor = 0xFFFF;
	uint16_t us_num_symbols;

	/* build response */
	if (px_tx_param->m_us_data_len) {
		us_sniffer_response_len = 0;

		uc_sniffer_rsp_buf[us_sniffer_response_len++] = SNIFFER_IF_PHY_COMMAND_G3_VERSION;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = SNIFFER_VERSION;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = SNIFFER_ATPL250;
		/* ModType (high) + ModScheme (low) */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)((px_tx_param->e_mod_scheme & 0x0F) + (((px_tx_param->e_mod_type) << 4) & 0xF0));
		/* Tone Map */
		if ((uc_working_band == WB_CENELEC_A) || (uc_working_band == WB_CENELEC_B)) {
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0;
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_tx_param->m_auc_tone_map[0];
		} else {
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_tx_param->m_auc_tone_map[2];
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_tx_param->m_auc_tone_map[1];
			uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_tx_param->m_auc_tone_map[0];
		}

		/* Number of symbols (2 bytes) */
		phy_get_cfg_param(PHY_ID_LAST_TX_MSG_PAYLOAD_SYMBOLS, &us_num_symbols, 2);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_num_symbols >> 8);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_num_symbols);
		/* SNR */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = 0xFF;
		/* Delimiter Type */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = px_tx_param->e_delimiter_type;
		/* TimeIni */
		ul_timeIni = px_tx_param->m_ul_tx_time;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeIni >> 24);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeIni >> 16) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeIni >> 8) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = ul_timeIni & 0xFF;
		/* TimeEnd */
		ul_timeEnd = px_tx_result->m_ul_end_tx_time;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeEnd >> 24);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeEnd >> 16) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (ul_timeEnd >> 8) & 0xFF;
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = ul_timeEnd & 0xFF;
		/* RSSI */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_rssi >> 8);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_rssi & 0xFF);
		/* AGC_Factor */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_agc_factor >> 8);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(us_agc_factor & 0xFF);
		/* Length in bytes */
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_tx_param->m_us_data_len >> 8);
		uc_sniffer_rsp_buf[us_sniffer_response_len++] = (uint8_t)(px_tx_param->m_us_data_len & 0xFF);
		/* PDU */
		memcpy(&uc_sniffer_rsp_buf[us_sniffer_response_len], px_tx_param->m_puc_data_buf, px_tx_param->m_us_data_len);
		us_sniffer_response_len += px_tx_param->m_us_data_len;

		/* set usi parameters */
		x_phy_sniffer_msg.ptr_buf = &uc_sniffer_rsp_buf[0];
		x_phy_sniffer_msg.us_len = us_sniffer_response_len;
		usi_send_cmd(&x_phy_sniffer_msg);
	}
}

static struct TPhySnifferCallbacks g_sniffer_if_phy_callbacks = {
	sniffer_if_cb_phy_data_confirm,
	sniffer_if_cb_phy_data_indication
};

static void _tonemask_to_atpl250_hwf(uint8_t *p_auc_active_carrier_list, uint8_t *p_auc_hardware_format)
{
	uint8_t uc_i;

	memset(p_auc_hardware_format, 0x00, 16 * sizeof(uint8_t));

	for (uc_i = uc_first_carrier; uc_i <= uc_last_carrier; uc_i++) {
		if (p_auc_active_carrier_list[uc_i - uc_first_carrier] == 0) {
			p_auc_hardware_format[uc_i / 8] |= (1 << (uc_i % 8));
		}
	}
}

/**
 * \brief Received message
 *
 * \param puc_rx_msg  Pointer to the received data
 * \param us_len      Data length of the received data
 *
 * \retval true if there is no error
 * \retval false if length is invalid or sniffer command is wrong
 */
uint8_t serial_if_sniffer_g3_api_parser(uint8_t *puc_rx_msg, uint16_t us_len)
{
	uint8_t uc_sniffer_if_cmd;
	uint8_t *puc_data;
	uint8_t puc_byte_tonemask[72];
	uint8_t puc_tonemask[16];
	uint8_t puc_old_tonemask[16];
	int i, j;

	/* Protection for invalid length */
	if (!us_len) {
		return 0;
	}

	/* Process received message */
	uc_sniffer_if_cmd  = puc_rx_msg[0];

	switch (uc_sniffer_if_cmd) {
	/* GET command */
	case SNIFFER_IF_PHY_G3_SET_TONE_MASK:
		puc_data = puc_rx_msg + 1;
		/* Convert bitfield to byte field, reversing order */
		for (i = 8; i >= 0; i--) {
			uint8_t uc_bitfield = puc_data[i];
			for (j = 0; j < 8; j++) {
				puc_byte_tonemask[((i) * 8) + j] = (uc_bitfield & 0x01) ? 1 : 0;
				uc_bitfield >>= 1;
			}
		}
		/* convert byte array to pl250 format */
		_tonemask_to_atpl250_hwf(puc_byte_tonemask, puc_tonemask);
		/* send data to PL360 */
		phy_get_cfg_param(PHY_ID_STATIC_NOTCHING, (void *)&puc_old_tonemask, 16);
		if (memcmp(puc_old_tonemask, puc_tonemask, 16) != 0) {
			phy_set_cfg_param(PHY_ID_STATIC_NOTCHING, (void *)&puc_tonemask, 16);
		}

		break;

	default:
		break;
	}

	return true;
}

/**
 * \brief Initialize sniffer interface.
 */
void sniffer_if_init()
{
	/* Set band-dependent variables */
	if (uc_working_band == WB_FCC) {
		uc_first_carrier = FIRST_CARRIER_FCC;
		uc_last_carrier = LAST_CARRIER_FCC;
	} else if (uc_working_band == WB_ARIB) {
		uc_first_carrier = FIRST_CARRIER_ARIB;
		uc_last_carrier = LAST_CARRIER_ARIB;
	} else {
		uc_first_carrier = FIRST_CARRIER_CENELEC_A;
		uc_last_carrier = LAST_CARRIER_CENELEC_A;
	}

	/* Set phy callbacks */
	phy_set_sniffer_callbacks(&g_sniffer_if_phy_callbacks);

	/* Set usi callbacks */
	usi_set_callback(PROTOCOL_SNIF_G3, serial_if_sniffer_g3_api_parser, PHY_SNIFFER_SERIAL_PORT);

	/* Set USI protocol by default in send usi command */
	x_phy_sniffer_msg.uc_protocol_type = PROTOCOL_SNIF_G3;
}

/* ! @} */

#endif /* ENABLE_SNIFFER */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
