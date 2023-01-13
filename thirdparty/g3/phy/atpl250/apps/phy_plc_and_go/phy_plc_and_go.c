/**
 * \file
 *
 * \brief PHY_PLC_AND_GO : G3 PLC Phy PLC and Go Application
 *
 * Copyright (c) 2018 Atmel Corporation. All rights reserved.
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
/* Application includes */
#include "asf.h"
#include "conf_project.h"

#define STRING_EOL    "\r"
#define STRING_HEADER "\r\n-- PLC & Go Application (G3-PLC) --" \
	"\r\n-- "BOARD_NAME " --" \
	"\r\n-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/* Manage Transmission */
#define TX_WAIT_TIME_MS 1000 /* 1second */
static xPhyMsgTx_t sx_tx_msg;
static uint8_t spuc_tx_data_buff[128];
static uint32_t sul_tx_counter = 0;
static uint32_t sul_tx_wait_time = TX_WAIT_TIME_MS;
#if CONF_APP_MODE == CONF_APP_TX_MODE
static bool sb_send_plc_msg;
#endif

static const uint16_t g_Crc16[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

static uint16_t _crc16Ccitt(const uint8_t *pu8Data, uint32_t u32Length, uint16_t u16Crc)
{
	/* polynom(16): X16 + X12 + X5 + 1 = 0x1021 */
	while (u32Length--) {
		u16Crc = g_Crc16[(u16Crc >> 8) ^ (*pu8Data++)] ^ ((u16Crc & 0xFF) << 8);
	}
	return u16Crc;
}

#if CONF_APP_MODE == CONF_APP_TX_MODE

/** @brief      Get padding length
 *
 * Note : Suppose to use every carriers to symplify operations. ToneMap and ToneMask don't use.
 *        Suppose to use only one RS block.
 *        Suppose to use Differential scheme.
 */
static uint8_t _get_padding_len(enum mod_types uc_mod_type, uint16_t us_data_len)
{
	uint8_t uc_rep_code;
	uint8_t uc_rs_parity;
	uint8_t uc_bits_carrier;
	uint8_t uc_num_carriers;
	uint8_t uc_fl_band;
	uint32_t ul_interleaver_input_size;
	uint32_t ul_interleaver_max_size;
	uint32_t ul_padding;
	uint8_t uc_padding_bytes;

#if defined(CONF_BAND_CENELEC_A)
	uc_num_carriers = NUM_CARRIERS_CENELEC_A;
	uc_fl_band = 4;
#elif defined(CONF_BAND_FCC)
	uc_num_carriers = NUM_CARRIERS_FCC;
	uc_fl_band = 1;
#elif defined(CONF_BAND_ARIB)
	uc_num_carriers = NUM_CARRIERS_ARIB;
	uc_fl_band = 1;
#else
#error ERROR in PHY band definition
#endif

	if (uc_mod_type == MOD_TYPE_BPSK_ROBO) {
		uc_bits_carrier = 1;
		uc_rep_code = 4;
		uc_rs_parity = 8;
	} else {
		uc_bits_carrier = uc_mod_type + 1;
		uc_rep_code = 1;
		uc_rs_parity = 16;
	}

	/* Get interleaver input size */
	ul_interleaver_input_size = (us_data_len + 2 + uc_rs_parity) << 3; /* data len + CRC + RS parity */
	ul_interleaver_input_size += 6; /* 6 bits of Convolutional Code */
	ul_interleaver_input_size *= (2 * uc_rep_code); /* In case of ROBO mode */

	/* Get interleaver maximum buffer */
	ul_interleaver_max_size = div_ceil(ul_interleaver_input_size, uc_num_carriers * uc_bits_carrier * uc_fl_band * uc_rep_code);
	ul_interleaver_max_size = ul_interleaver_max_size * uc_num_carriers * uc_bits_carrier * uc_fl_band * uc_rep_code;

	/* Get padding in bytes */
	ul_padding = ul_interleaver_max_size - ul_interleaver_input_size;
	uc_padding_bytes = ul_padding / (16 * uc_rep_code);

	return uc_padding_bytes;
}

/** @brief      Adjust padding and include CRC in TX message
 *
 */
static void _tx_insert_padding_crc(void)
{
	uint8_t *puc_end;
	uint16_t us_crc;
	uint8_t uc_pad_len;

	puc_end = &sx_tx_msg.m_puc_data_buf[sx_tx_msg.m_us_data_len];

	/* Get padding length */
	uc_pad_len = _get_padding_len(sx_tx_msg.e_mod_type, sx_tx_msg.m_us_data_len);

	/* Encode the padding */
	memset(puc_end, 0, uc_pad_len);
	puc_end += uc_pad_len;

	/* Calculate and append FCS */
	us_crc = _crc16Ccitt(sx_tx_msg.m_puc_data_buf, sx_tx_msg.m_us_data_len + uc_pad_len, 0);
	*puc_end++ = (uint8_t)us_crc;
	*puc_end = (uint8_t)(us_crc >> 8);

	/* Adjust data length */
	sx_tx_msg.m_us_data_len += (2 + uc_pad_len);
}

/** @brief      Update data content to use in TX message
 *
 */
static void _update_tx_data(void)
{
	uint16_t us_len;

	/* Update TX counter in content of message */
	us_len = sprintf((char *)spuc_tx_data_buff, "G3 PLC Message num:%u\r\n", sul_tx_counter);
	/* Update data len */
	sx_tx_msg.m_us_data_len = us_len;
}

#endif

/** @brief      Check CRC in RX message
 *
 */
static bool _rx_check_crc(xPhyMsgRx_t *px_msg)
{
	uint8_t *puc_end;
	uint16_t us_crc;
	uint16_t us_crc_rcv;

	/* Extract CRC */
	puc_end = &px_msg->m_puc_data_buf[px_msg->m_us_data_len];
	us_crc_rcv = *(puc_end - 2);
	us_crc_rcv += (*(puc_end - 1) << 8);

	/* Calculate FCS */
	us_crc = _crc16Ccitt(px_msg->m_puc_data_buf, px_msg->m_us_data_len - 2, 0);

	/* Check CRC */
	if (us_crc != us_crc_rcv) {
		/* CRC Error */
		return false;
	}

	return true;
}

/** @brief      Setup parameters to use in TX message
 *
 */
static void _setup_tx_parameters(void)
{
	uint16_t us_len;

	us_len = sprintf((char *)spuc_tx_data_buff, "G3 PLC Message num:%u\r\n", sul_tx_counter);

	/* Set data content */
	sx_tx_msg.m_puc_data_buf = spuc_tx_data_buff;
	sx_tx_msg.m_us_data_len = us_len;

	/* Configure Tone Map, it depends on PHY band. */
#if defined(CONF_BAND_CENELEC_A)
	sx_tx_msg.m_auc_tone_map[0] = 0x3F;
#elif defined(CONF_BAND_FCC)
	sx_tx_msg.m_auc_tone_map[0] = 0xFF;
	sx_tx_msg.m_auc_tone_map[1] = 0xFF;
	sx_tx_msg.m_auc_tone_map[2] = 0xFF;
#elif defined(CONF_BAND_ARIB)
	sx_tx_msg.m_auc_tone_map[0] = 0xFF;
	sx_tx_msg.m_auc_tone_map[1] = 0xFF;
	sx_tx_msg.m_auc_tone_map[2] = 0x03;
#else
#error ERROR in PHY band definition
#endif

	/* Set preemphasis filter */
	memset(sx_tx_msg.m_auc_preemphasis, 0, sizeof(sx_tx_msg.m_auc_preemphasis));
	/* Use only 1 RS block */
	sx_tx_msg.m_uc_2_rs_blocks = 0;
	/* ACK management is not needed. See atpl250.h file (enum delimiter_types) */
	sx_tx_msg.e_delimiter_type = DT_SOF_NO_RESP;
	/* Set modulation scheme. See atpl250.h file (enum mod_schemes) */
	sx_tx_msg.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
	/* Set modulation type. See atpl250.h file (enum mod_types) */
	sx_tx_msg.e_mod_type = MOD_TYPE_BPSK;
	/* Set phase detector counter */
	sx_tx_msg.m_uc_pdc = 0;
	/* Set Transmission Mode. See atpl250.h file (TX Mode Bit Mask) */
	sx_tx_msg.m_uc_tx_mode = TX_MODE_FORCED_TX;
	/* TX time is not taken into account as Tx mode is immediate */
	sx_tx_msg.m_ul_tx_time = 0;
	/* Set transmission power. It represents 3dBs of Attenuation signal per Unit. 0 value for maximum signal level. */
	sx_tx_msg.m_uc_tx_power = 0;
}

/**
 * \brief Handler to manage confirmation of the last PLC transmission.
 */
static void _handler_data_cfm(xPhyMsgTxResult_t *px_tx_result)
{
	switch (px_tx_result->e_tx_result) {
	case PHY_TX_RESULT_PROCESS:
		printf("-> Cfm TX msg %u : PHY_STATUS_BUSY\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_SUCCESS:
		printf("-> Cfm TX msg %u : TX_RESULT_SUCCESS\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_INV_LENGTH:
		printf("-> Cfm TX msg %u : TX_RESULT_INV_LENGTH\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_INV_SCHEME:
		printf("-> Cfm TX msg %u : TX_RESULT_INV_SCHEME\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_INV_TONEMAP:
		printf("-> Cfm TX msg %u : TX_RESULT_INV_TONEMAP\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_BUSY_CH:
		printf("-> Cfm TX msg %u : TX_RESULT_BUSY_CH\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_BUSY_TX:
		printf("-> Cfm TX msg %u : TX_RESULT_BUSY_TX\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_BUSY_RX:
		printf("-> Cfm TX msg %u : TX_RESULT_BUSY_RX\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_TIMEOUT:
		printf("-> Cfm TX msg %u : TX_RESULT_TIMEOUT\r\n", sul_tx_counter);
		break;

	case PHY_TX_RESULT_NO_TX:
		printf("-> Cfm TX msg %u : TX_RESULT_NO_TX\r\n", sul_tx_counter);
		break;
	}

#if CONF_APP_MODE == CONF_APP_TX_MODE
	/* Increase TX counter */
	sul_tx_counter++;
	/* Update data content */
	_update_tx_data();
	/* Set flag to start new TX in process function */
	sb_send_plc_msg = true;
	/* Set wait time for next TX */
	sul_tx_wait_time = TX_WAIT_TIME_MS;
#endif
}

/**
 * \brief Handler to manage new received PLC message.
 */
static void _handler_data_ind(xPhyMsgRx_t *px_msg)
{
	/* Shows Received PLC message */
	if (px_msg->m_us_data_len) {
		/* Check CRC and extract padding */
		if (_rx_check_crc(px_msg)) {
			/* CRC Ok */
			printf("<- Received new msg (%u bytes)\r\n", (unsigned int)px_msg->m_us_data_len);
			printf("\tData msg: %s", px_msg->m_puc_data_buf);
		} else {
			/* CRC Error */
			printf("<- Received new msg : CRC error\r\n");
		}
	}
}

static void phy_getting_started_update_timers(void)
{
	/* Update timers */
	if (sul_tx_wait_time) {
		sul_tx_wait_time--;
	}
}

static struct TPhyCallbacks g_serial_if_phy_callbacks = {
	_handler_data_cfm,
	_handler_data_ind
};

static void phy_getting_started_init(void)
{
	uint8_t uc_value;
	
	/* Set phy callbacks */
	phy_set_callbacks(&g_serial_if_phy_callbacks);

	/* Init Phy Layer */
#if defined(CONF_BAND_FCC)
	phy_init(0, WB_FCC);
#elif defined(CONF_BAND_ARIB)
	phy_init(0, WB_ARIB);
#else
	phy_init(0, WB_CENELEC_A);
#endif

	/* Show welcome message */
	puts(STRING_HEADER);
	
	/* Force VLO impedance mode */
	uc_value = 0;
	phy_set_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &uc_value, 1);
	uc_value = 2;
	phy_set_cfg_param(PHY_ID_CFG_IMPEDANCE, &uc_value, 1);

	/* Setup G3 PLC parameters to use in transmission */
	_setup_tx_parameters();

#if CONF_APP_MODE == CONF_APP_TX_MODE
	/* Set flag to start new TX in process function */
	sb_send_plc_msg = true;
#endif
}

static void phy_getting_started_process(void)
{
	/* Call phy layer process */
	phy_process();

#if CONF_APP_MODE == CONF_APP_TX_MODE
	/* Check whether a frame has to be sent */
	if (sb_send_plc_msg && (sul_tx_wait_time == 0)) {
		/* Clear flag, will be set automatically after transmission on _handler_data_cfm */
		sb_send_plc_msg = false;
		/* Show tx info */
		printf("\r\n-> Sending new PLC msg(%u)...size:%u bytes\r\n", sul_tx_counter, (uint32_t)sx_tx_msg.m_us_data_len);
		/* Insert padding and CRC */
		_tx_insert_padding_crc();
		/* Send PLC message */
		phy_tx_frame(&sx_tx_msg);
	}
#endif
}

/**
 * \brief Main code entry point.
 */
int main(void)
{
	oss_task_t x_task = {0};

	/* Initialize OSS */
	oss_init();

	/* Register PHY Task */
	x_task.task_init = phy_getting_started_init;
	x_task.task_process = phy_getting_started_process;
	x_task.task_1ms_timer_cb = phy_getting_started_update_timers;
	oss_register_task(&x_task);

	/* Start OSS */
	oss_start();
}
