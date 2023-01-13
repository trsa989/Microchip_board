/**
 * \file
 *
 * \brief ATPL250 Serial Interface for Physical layer
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
#include "atpl250_common.h"

/* Serial interface */
#include "conf_usi.h"
#include "serial_if.h"
#include "usi.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \weakgroup serial_plc_group
 * @{
 */

/* Default empty USI interface*/
void Dummy_serial_if_api_parser_extension(uint8_t *puc_rx_msg, uint16_t us_len, uint8_t *puc_serial_data_buf);

#ifdef __GNUC__
void serial_if_api_parser_extension( uint8_t *puc_rx_msg, uint16_t us_len,
		uint8_t *puc_serial_data_buf) __attribute__ ((weak, alias("Dummy_serial_if_api_parser_extension")));

#endif

#if defined(__ICCARM__) || defined(__CC_ARM)
extern void serial_if_api_parser_extension(uint8_t *puc_rx_msg, uint16_t us_len, uint8_t *puc_serial_data_buf);

#pragma weak serial_if_api_parser_extension=Dummy_serial_if_api_parser_extension
#endif

/* Max length of command to be received by this layer */
#define MAX_CMD_LEN             30 /* TODO: Update with correct value */

/* Carriers Buffer Len */
#define SERIAL_IF_CARR_BUFFER_LEN  16

/*! \name Data buffers */
/* @{ */
static uint8_t uc_serial_rsp_buf[808];   /* !<  Response working buffer */
static uint8_t uc_serial_data_buf[808];  /* !<  Receive working buffer */
/* @} */

/* ! \name Data structure to communicate with USI layer */
/* @{ */
static x_usi_serial_cmd_params_t x_phy_serial_msg;
/* @} */

/* Shared variables, used in other files */
uint8_t uc_tone_map_size;
uint8_t uc_protocol_carriers;
uint8_t uc_num_subbands;
uint8_t uc_num_carriers_in_subband;
uint8_t uc_first_carrier;
uint8_t uc_last_carrier;
uint8_t uc_tx_fft_shift;

/* Extern variables needed */
extern uint8_t uc_working_band;

/**
 * \internal
 * \brief Memcopy with byte order reversal.
 *
 * Copies puc_buf[] into puc_dst[], re-ordering the bytes to adapt to the serial communication.
 *
 * \param puc_dst    Pointer to buffer where the data will be copied
 * \param puc_buf    Pointer to buffer data
 * \param us_len     Length of data to copy
 */
static void _memcpy_rev(uint8_t *puc_dst, uint8_t *puc_buf, uint16_t us_len)
{
	uint8_t *ptr_uc_mem_dst, *ptr_uc_mem_src;
	uint16_t us_idx;

	if (us_len <= 4) {
		ptr_uc_mem_dst = puc_dst + us_len - 1;
		ptr_uc_mem_src = puc_buf;
		for (us_idx = 0; us_idx < us_len; us_idx++) {
			*ptr_uc_mem_dst-- = (uint8_t)*ptr_uc_mem_src++;
		}
	} else {
		memcpy(puc_dst, puc_buf, us_len);
	}
}

/**
 * \internal
 * \brief Callback to capture end of transmission and manage the serialization of the result of transmission through USI.
 *
 * \param px_tx_result Pointer to Transmission result struct
 */
static void serial_if_cb_phy_data_confirm(xPhyMsgTxResult_t *px_tx_result)
{
	uint8_t us_serial_response_len;

	/* Manage Result */
	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_IF_PHY_COMMAND_SEND_MSG_RSP;
	uc_serial_rsp_buf[us_serial_response_len++] = px_tx_result->m_uc_id_buffer;
	uc_serial_rsp_buf[us_serial_response_len++] = px_tx_result->e_tx_result;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_tx_result->m_ul_rms_calc >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_tx_result->m_ul_rms_calc >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_tx_result->m_ul_rms_calc >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_tx_result->m_ul_rms_calc & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_tx_result->m_ul_end_tx_time >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_tx_result->m_ul_end_tx_time >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_tx_result->m_ul_end_tx_time >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_tx_result->m_ul_end_tx_time & 0xFF);
	/* set usi parameters */
	x_phy_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_phy_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_phy_serial_msg);
}

/**
 * \internal
 * \brief Callback to capture frame reception and manage the serialization through USI.
 *
 * \param px_msg Pointer to Reception struct
 */
static void serial_if_cb_phy_data_indication(xPhyMsgRx_t *px_msg)
{
	uint16_t us_serial_response_len;
	uint8_t uc_i;
	uint16_t us_j;
	struct s_rx_ber_payload_data_t s_ber_data;

	/* Copy data to local buffer */
	/* memcpy(uc_serial_data_buf, px_msg->m_puc_data_buf, px_msg->m_us_data_len); */

	/* Discard ACKs (mistakenly detected in noise conditions) */
	if ((px_msg->e_delimiter_type == DT_ACK) || (px_msg->e_delimiter_type == DT_NACK)) {
		return;
	}

	/* build response */
	if (px_msg->m_us_data_len) {
		us_serial_response_len = 0;
		uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_IF_PHY_COMMAND_RECEIVE_MSG;
		uc_serial_rsp_buf[us_serial_response_len++] = px_msg->m_uc_buff_id;
		uc_serial_rsp_buf[us_serial_response_len++] = px_msg->e_mod_type;
		uc_serial_rsp_buf[us_serial_response_len++] = px_msg->e_mod_scheme;

		for (uc_i = 0; uc_i < uc_tone_map_size; uc_i++) {
			uc_serial_rsp_buf[us_serial_response_len++] = px_msg->m_auc_tone_map[uc_tone_map_size - uc_i - 1];
		}

		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_evm_header >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_evm_header & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_evm_payload >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_evm_payload & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_rssi >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_rssi & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_agc_factor >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_agc_factor & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = px_msg->m_uc_zct_diff;
		uc_serial_rsp_buf[us_serial_response_len++] = px_msg->e_delimiter_type;

		/* Get Ber data */
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
		/* Fill in structure with ber data */
		if (us_j < 10000) {
			uc_serial_rsp_buf[us_serial_response_len++] = s_ber_data.uc_lqi;
			memcpy(&uc_serial_rsp_buf[us_serial_response_len], s_ber_data.auc_carrier_snr, uc_protocol_carriers);
			us_serial_response_len += uc_protocol_carriers;

			uc_serial_rsp_buf[us_serial_response_len++] = s_ber_data.uc_payload_snr_worst_carrier;
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(s_ber_data.us_payload_corrupted_carriers >> 8);
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(s_ber_data.us_payload_corrupted_carriers & 0xFF);
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(s_ber_data.us_payload_noised_symbols >> 8);
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(s_ber_data.us_payload_noised_symbols & 0xFF);
			uc_serial_rsp_buf[us_serial_response_len++] = s_ber_data.uc_payload_snr_worst_symbol;
			uc_serial_rsp_buf[us_serial_response_len++] = s_ber_data.uc_payload_snr_impulsive;
			uc_serial_rsp_buf[us_serial_response_len++] = s_ber_data.uc_payload_snr_be;
			uc_serial_rsp_buf[us_serial_response_len++] = s_ber_data.uc_payload_snr_background;
		} else {
			uc_serial_rsp_buf[us_serial_response_len++] = 0;
			memset(&uc_serial_rsp_buf[us_serial_response_len], 0, uc_protocol_carriers);
			us_serial_response_len += uc_protocol_carriers;

			uc_serial_rsp_buf[us_serial_response_len++] = 0;
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(0xFFFF >> 8);
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(0xFFFF);
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(0xFFFF >> 8);
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(0xFFFF);
			uc_serial_rsp_buf[us_serial_response_len++] = 0;
			uc_serial_rsp_buf[us_serial_response_len++] = 0;
			uc_serial_rsp_buf[us_serial_response_len++] = 0;
			uc_serial_rsp_buf[us_serial_response_len++] = 0;
		}

		uc_serial_rsp_buf[us_serial_response_len++] = px_msg->m_uc_rs_corrected_errors;
		uc_serial_rsp_buf[us_serial_response_len++] =  0;

		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_data_len >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_us_data_len & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_msg->m_ul_rx_time >> 24) & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_msg->m_ul_rx_time >> 16) & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_msg->m_ul_rx_time >> 8) & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_ul_rx_time & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_msg->m_ul_frame_duration >> 24) & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_msg->m_ul_frame_duration >> 16) & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((px_msg->m_ul_frame_duration >> 8) & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(px_msg->m_ul_frame_duration & 0xFF);
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], px_msg->m_puc_data_buf, px_msg->m_us_data_len);
		us_serial_response_len += px_msg->m_us_data_len;

		/* set usi parameters */
		x_phy_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
		x_phy_serial_msg.us_len = us_serial_response_len;
		usi_send_cmd(&x_phy_serial_msg);
	}
}

static struct TPhyCallbacks g_serial_if_phy_callbacks = {
	serial_if_cb_phy_data_confirm,
	serial_if_cb_phy_data_indication
};

/**
 * \brief Received message
 *
 * \param puc_rx_msg  Pointer to the data attached to the connection request
 * \param us_len      Data length of the data attached to the request
 *
 * \retval true if there is no error
 * \retval false if length is invalid or serial command is wrong
 */
uint8_t serial_if_api_parser(uint8_t *puc_rx_msg, uint16_t us_len)
{
	uint8_t uc_i;
	uint8_t uc_phy_id_len;
	uint8_t uc_serial_if_cmd;
	uint8_t uc_cmd_op, uc_mask;
	uint8_t *puc_rx;
	uint8_t uc_cfg_value[MAX_CMD_LEN];
	uint16_t us_phy_id;
	uint16_t us_serial_response_len;
	xPhyMsgTx_t x_phy_tx_msg;

	/* Protection for invalid us_length */
	if (!us_len) {
		return false;
	}

	/* Process received message */
	uc_serial_if_cmd = puc_rx_msg[0];
	puc_rx = &puc_rx_msg[1];
	us_serial_response_len = 0;

	switch (uc_serial_if_cmd) {
	/* GET command */
	case SERIAL_IF_PHY_COMMAND_GET_CFG:
		us_phy_id = ((uint16_t)*puc_rx++) << 8;
		us_phy_id += (uint16_t)*puc_rx++;
		uc_phy_id_len = *puc_rx++;
		if (phy_get_cfg_param(us_phy_id, uc_cfg_value, uc_phy_id_len) == PHY_CFG_SUCCESS) {
			/* build response */
			uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_IF_PHY_COMMAND_GET_CFG_RSP;
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(us_phy_id >> 8);
			uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(us_phy_id & 0xFF);
			uc_serial_rsp_buf[us_serial_response_len++] = uc_phy_id_len;
			_memcpy_rev(&uc_serial_rsp_buf[4], uc_cfg_value, uc_phy_id_len);
			us_serial_response_len = uc_phy_id_len + 4;
			/* set usi parameters */
			x_phy_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
			x_phy_serial_msg.us_len = us_serial_response_len;
			usi_send_cmd(&x_phy_serial_msg);
		}

		break;

	/* SET command */
	case SERIAL_IF_PHY_COMMAND_SET_CFG:
		us_phy_id = ((uint16_t)*puc_rx++) << 8;
		us_phy_id += (uint16_t)*puc_rx++;
		uc_phy_id_len = *puc_rx++;
		_memcpy_rev(&uc_cfg_value[0], puc_rx, uc_phy_id_len);
		us_serial_response_len = 0;
		/* build response */
		uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_IF_PHY_COMMAND_SET_CFG_RSP;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(us_phy_id >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(us_phy_id & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = phy_set_cfg_param(us_phy_id, &uc_cfg_value, uc_phy_id_len);
		/* set usi parameters */
		x_phy_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
		x_phy_serial_msg.us_len = us_serial_response_len;
		usi_send_cmd(&x_phy_serial_msg);
		break;

	/* CMD command (operations over bitfields, only in 8 bits) */
	case SERIAL_IF_PHY_COMMAND_CMD_CFG:
		us_phy_id = ((uint16_t)*puc_rx++) << 8;
		us_phy_id += (uint16_t)*puc_rx++;
		uc_cmd_op = *puc_rx++;
		uc_mask = *puc_rx++;
		/* build response */
		uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_IF_PHY_COMMAND_CMD_CFG_RSP;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(us_phy_id >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(us_phy_id & 0xFF);
		uc_serial_rsp_buf[us_serial_response_len++] = uc_cmd_op;
		uc_serial_rsp_buf[us_serial_response_len++] = phy_cmd_cfg_param(us_phy_id, uc_cmd_op, uc_mask);
		/* set usi parameters */
		x_phy_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
		x_phy_serial_msg.us_len = us_serial_response_len;
		usi_send_cmd(&x_phy_serial_msg);
		break;

	/* Write command (send data msg) */
	case SERIAL_IF_PHY_COMMAND_SEND_MSG:
		/* configure write parameters */
		x_phy_tx_msg.m_uc_buff_id = *puc_rx++;
		x_phy_tx_msg.m_uc_tx_mode = *puc_rx++;
		x_phy_tx_msg.m_uc_tx_power = *puc_rx++;
		x_phy_tx_msg.e_mod_type = (enum mod_types)(*puc_rx++);
		x_phy_tx_msg.e_mod_scheme = (enum mod_schemes)(*puc_rx++);
		x_phy_tx_msg.m_uc_pdc = *puc_rx++;

		for (uc_i = 0; uc_i < uc_tone_map_size; uc_i++) {
			x_phy_tx_msg.m_auc_tone_map[uc_tone_map_size - uc_i - 1] = *puc_rx++;
		}

		if (uc_working_band == WB_FCC) {
			x_phy_tx_msg.m_uc_2_rs_blocks = *puc_rx++;
		} else {
			x_phy_tx_msg.m_uc_2_rs_blocks = 0;
		}

		memcpy(x_phy_tx_msg.m_auc_preemphasis, puc_rx, uc_num_subbands);
		puc_rx += uc_num_subbands;
		x_phy_tx_msg.e_delimiter_type = (enum delimiter_types)(*puc_rx++);
		x_phy_tx_msg.m_ul_tx_time = ((uint32_t)*puc_rx++) << 24;
		x_phy_tx_msg.m_ul_tx_time += ((uint32_t)*puc_rx++) << 16;
		x_phy_tx_msg.m_ul_tx_time += ((uint32_t)*puc_rx++) << 8;
		x_phy_tx_msg.m_ul_tx_time += (uint32_t)*puc_rx++;
		x_phy_tx_msg.m_us_data_len = ((uint16_t)*puc_rx++) << 8;
		x_phy_tx_msg.m_us_data_len += (uint16_t)*puc_rx++;
		x_phy_tx_msg.m_puc_data_buf = uc_serial_data_buf;
		/* copy data */
		memcpy(uc_serial_data_buf, puc_rx, x_phy_tx_msg.m_us_data_len);

		/* send to phy layer */
		phy_tx_frame(&x_phy_tx_msg);
		break;

	/* Set noise capture mode */
	case SERIAL_IF_PHY_COMMAND_GET_CFG_LIST:
	{
		uint8_t uc_num_elemts;
		uint16_t us_id;
		uint8_t uc_id_len;
		uint8_t *ptrRspIdx;

		/* reserve dynamic memory */
		ptrRspIdx = &uc_serial_rsp_buf[0];
		*ptrRspIdx++ = SERIAL_IF_PHY_COMMAND_GET_CFG_LIST_RSP;
		/* get parameters of the list */
		uc_num_elemts = *puc_rx++;
		*ptrRspIdx++ = uc_num_elemts;
		while (uc_num_elemts--) {
			*ptrRspIdx++ = *puc_rx;
			us_id = ((uint16_t)*puc_rx++) << 8;
			*ptrRspIdx++ = *puc_rx;
			us_id += (uint16_t)*puc_rx++;
			*ptrRspIdx++ = *puc_rx;
			uc_id_len = *puc_rx++;
			phy_get_cfg_param(us_id, uc_cfg_value, uc_id_len);
			_memcpy_rev(ptrRspIdx, uc_cfg_value, uc_id_len);
			ptrRspIdx += uc_id_len;
		}
		/* set usi parameters */
		x_phy_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
		x_phy_serial_msg.us_len = ptrRspIdx - &uc_serial_rsp_buf[0];
		usi_send_cmd(&x_phy_serial_msg);
	}
	break;

	default:
		serial_if_api_parser_extension(puc_rx_msg, us_len, uc_serial_data_buf);
		break;
	}

	return true;
}

/**
 * \brief Initialize serial interface.
 *
 */
void serial_if_init(void)
{
	/* Set phy callbacks */
	phy_set_callbacks(&g_serial_if_phy_callbacks);

	/* Set usi callbacks */
	usi_set_callback(PROTOCOL_PHY_ATPL2X0, serial_if_api_parser, PHY_IFACE_SERIAL_PORT);

	/* Set USI protocol by default in send usi command */
	x_phy_serial_msg.uc_protocol_type = PROTOCOL_PHY_ATPL2X0;

#if defined(CONF_BAND_CENELEC_A)
	uc_tone_map_size = TONE_MAP_SIZE_CENELEC_A;
	uc_protocol_carriers = NUM_CARRIERS_CENELEC_A;
	uc_num_subbands = NUM_SUBBANDS_CENELEC_A;
	uc_num_carriers_in_subband = CARRIERS_IN_SUBBAND_CENELEC_A;
	uc_first_carrier = FIRST_CARRIER_CENELEC_A;
	uc_last_carrier = LAST_CARRIER_CENELEC_A;
	uc_tx_fft_shift = TX_FFT_SHIFT_CENELEC_A;
#elif defined(CONF_BAND_FCC)
	uc_tone_map_size = TONE_MAP_SIZE_FCC;
	uc_protocol_carriers = NUM_CARRIERS_FCC;
	uc_num_subbands = NUM_SUBBANDS_FCC;
	uc_num_carriers_in_subband = CARRIERS_IN_SUBBAND_FCC;
	uc_first_carrier = FIRST_CARRIER_FCC;
	uc_last_carrier = LAST_CARRIER_FCC;
	uc_tx_fft_shift = TX_FFT_SHIFT_FCC;
#elif defined(CONF_BAND_ARIB)
	uc_tone_map_size = TONE_MAP_SIZE_ARIB;
	uc_protocol_carriers = NUM_CARRIERS_ARIB;
	uc_num_subbands = NUM_SUBBANDS_ARIB;
	uc_num_carriers_in_subband = CARRIERS_IN_SUBBAND_ARIB;
	uc_first_carrier = FIRST_CARRIER_ARIB;
	uc_last_carrier = LAST_CARRIER_ARIB;
	uc_tx_fft_shift = TX_FFT_SHIFT_ARIB;
#endif
}

/**
 * \brief Dummy Phy layer extension handler_start_tx
 *
 */
void Dummy_serial_if_api_parser_extension(uint8_t *puc_rx_msg, uint16_t us_len, uint8_t *puc_serial_data_buf)
{
	UNUSED(puc_serial_data_buf);
	UNUSED(puc_rx_msg);
	UNUSED(us_len);
}

/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
