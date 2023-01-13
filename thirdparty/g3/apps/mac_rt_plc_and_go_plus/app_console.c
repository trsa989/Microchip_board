/**
 * \file
 *
 * \brief MACRT_PLC_AD_GO_PLUS : G3-PLC MacRt PLC And Go Plus Application. Module to manage console
 *
 * Copyright (c) 2020 Atmel Corporation. All rights reserved.
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

/* Atmel library includes. */
#include "asf.h"
#include "MacRtDefs.h"
#include "conf_project.h"
#include "app_console.h"
#include "app_plc.h"

#define MAX_DATA_LEN        MAC_RT_MAX_PAYLOAD_SIZE

/* Application state machine */
typedef enum app_state {
	APP_STATE_IDLE,
	APP_STATE_TYPING,
	APP_STATE_SHOW_CONFIG,
	APP_STATE_CONFIG_MENU,
	APP_STATE_CONFIG_MOD,
	APP_STATE_CONFIG_SRC_ADDRESS,
	APP_STATE_CONFIG_DST_ADDRESS,
#if defined(CONF_MULTIBAND_FCC_CENA) || defined(CONF_MULTIBAND_FCC_CENB)
	APP_STATE_CONFIG_BAND,
#endif
	APP_STATE_TRANSMITTING,
} app_state_t;

static app_state_t suc_app_state;

/* Transmission data buffer */
static uint8_t spuc_tx_data_buff[MAX_DATA_LEN];

/* Index for transmission data buffer */
static uint16_t sus_tx_data_index;

/**
 * \brief Shows Tx modulation (Modulation Type and Modulation Scheme) as string.
 *
 * \param uc_mod_scheme Modulation Scheme.
 * \param uc_mod_type Modulation Type.
 *
 */
static void _show_modulation(enum ERtModulationScheme uc_mod_scheme, enum ERtModulationType uc_mod_type)
{
	if (uc_mod_scheme == RT_MODULATION_SCHEME_DIFFERENTIAL) {
		switch (uc_mod_type) {
		case RT_MODULATION_DBPSK_BPSK:
			printf("BPSK Differential\r\n");
			break;

		case RT_MODULATION_DQPSK_QPSK:
			printf("QPSK Differential\r\n");
			break;

		case RT_MODULATION_D8PSK_8PSK:
			printf("8PSK Differential\r\n");
			break;

		case RT_MODULATION_ROBUST:
			printf("BPSK Robust Differential\r\n");
			break;

		default:
			printf("Unknown\r\n");
			break;
		}
	} else if (uc_mod_scheme == RT_MODULATION_SCHEME_COHERENT) {
		switch (uc_mod_type) {
		case RT_MODULATION_DBPSK_BPSK:
			printf("BPSK Coherent\r\n");
			break;

		case RT_MODULATION_DQPSK_QPSK:
			printf("QPSK Coherent\r\n");
			break;

		case RT_MODULATION_D8PSK_8PSK:
			printf("8PSK Coherent\r\n");
			break;

		case RT_MODULATION_ROBUST:
			printf("BPSK Robust Coherent\r\n");
			break;

		default:
			printf("Unknown\r\n");
			break;
		}
	} else {
		printf("Unknown\r\n");
	}
}

#if defined(CONF_MULTIBAND_FCC_CENA) || defined(CONF_MULTIBAND_FCC_CENB)

/**
 * \brief Shows Band as string.
 *
 * \param uc_band Band identifier (see "general_defs.h").
 *
 */
static void _show_band(uint8_t uc_band)
{
	switch (uc_band) {
	case ATPL360_WB_CENELEC_A:
		printf("CENELEC-A band (35 - 91 kHz)\r\n");
		break;

	case ATPL360_WB_FCC:
		printf("FCC band (154 - 488 kHz)\r\n");
		break;

	case ATPL360_WB_ARIB:
		printf("ARIB band (154 - 404 kHz)\r\n");
		break;

	case ATPL360_WB_CENELEC_B:
		printf("CENELEC-B band (98 - 122 kHz)\r\n");
		break;

	default:
		printf("Unknown band\r\n");
		break;
	}
}

#endif

/**
 * \brief Shows available options through console depending on app state.
 *
 * \param b_full_menu Show full menu (true). Only used in APP_STATE_IDLE.
 *
 */
static void _show_menu(bool b_full_menu)
{
	enum ERtModulationScheme uc_mod_scheme;
	enum ERtModulationType uc_mod_type;
	uint8_t uc_phy_band;

	switch (suc_app_state) {
	case APP_STATE_IDLE:
		/* Idle state: Show available options */
		if (b_full_menu) {
			printf("\r\nPress 'CTRL+D' to show the current configuration.\r\n");
			printf("Press 'CTRL+S' to enter configuration menu.\r\n");
			printf("Enter text and press 'ENTER' to trigger transmission");
		}

		printf("\r\n>>> ");
		fflush(stdout);
		break;

	case APP_STATE_TYPING:
		/* Typing message to transmit */
		printf("\r\n>>> %.*s", sus_tx_data_index - 2, spuc_tx_data_buff + 2);
		fflush(stdout);
		break;

	case APP_STATE_SHOW_CONFIG:
		/* Show Configuration */
		printf("\r\n--- Configuration Parameters ---\r\n");
		printf("\tG3 MAC Tx Modulation: ");
		_show_modulation(app_plc_get_mod_scheme(), app_plc_get_mod_type());
		printf("\tG3 MAC Source Address: 0x%04X\r\n", app_plc_get_mac_src_address());
		printf("\tG3 MAC Destination Address: 0x%04X\r\n", app_plc_get_mac_dst_address());
#if defined(CONF_MULTIBAND_FCC_CENA) || defined(CONF_MULTIBAND_FCC_CENB)
		printf("\tG3 Tx/Rx Band: ");
		_show_band(app_plc_get_phy_band());
#endif
		printf(">>> ");
		fflush(stdout);
		break;

	case APP_STATE_CONFIG_MENU:
		/* Configuration Menu */
		printf("\r\n--- Configuration Menu ---\r\n");
		printf("Select parameter to configure: \r\n");
		printf("\t0: G3 MAC Tx Modulation\r\n");
		printf("\t1: G3 MAC Source Address\r\n");
		printf("\t2: G3 MAC Destination Address\r\n");
#if defined(CONF_MULTIBAND_FCC_CENA) || defined(CONF_MULTIBAND_FCC_CENB)
		printf("\t3: Tx/Rx Band\r\n");
#endif
		printf(">>> ");
		fflush(stdout);
		break;

	case APP_STATE_CONFIG_MOD:
		/* Modulation Configuration */
		uc_mod_scheme = app_plc_get_mod_scheme();
		uc_mod_type = app_plc_get_mod_type();
		printf("\r\n--- Tx Modulation Configuration Menu ---\r\n");
		printf("Select Modulation:\r\n");

		if ((uc_mod_scheme == RT_MODULATION_SCHEME_DIFFERENTIAL) && (uc_mod_type == RT_MODULATION_ROBUST)) {
			printf("->");
		}

		printf("\t0: ");
		_show_modulation(RT_MODULATION_SCHEME_DIFFERENTIAL, RT_MODULATION_ROBUST);

		if ((uc_mod_scheme == RT_MODULATION_SCHEME_DIFFERENTIAL) && (uc_mod_type == RT_MODULATION_DBPSK_BPSK)) {
			printf("->");
		}

		printf("\t1: ");
		_show_modulation(RT_MODULATION_SCHEME_DIFFERENTIAL, RT_MODULATION_DBPSK_BPSK);

		if ((uc_mod_scheme == RT_MODULATION_SCHEME_DIFFERENTIAL) && (uc_mod_type == RT_MODULATION_DQPSK_QPSK)) {
			printf("->");
		}

		printf("\t2: ");
		_show_modulation(RT_MODULATION_SCHEME_DIFFERENTIAL, RT_MODULATION_DQPSK_QPSK);

		uc_phy_band = app_plc_get_phy_band();

		if (uc_phy_band != ATPL360_WB_ARIB) {
			/* 8PSK modulation type and coherent modulation scheme not allowed in ARIB band */
			if ((uc_mod_scheme == RT_MODULATION_SCHEME_DIFFERENTIAL) && (uc_mod_type == RT_MODULATION_D8PSK_8PSK)) {
				printf("->");
			}

			printf("\t3: ");
			_show_modulation(RT_MODULATION_SCHEME_DIFFERENTIAL, RT_MODULATION_D8PSK_8PSK);

			if ((uc_mod_scheme == RT_MODULATION_SCHEME_COHERENT) && (uc_mod_type == RT_MODULATION_ROBUST)) {
				printf("->");
			}

			printf("\t4: ");
			_show_modulation(RT_MODULATION_SCHEME_COHERENT, RT_MODULATION_ROBUST);

			if ((uc_mod_scheme == RT_MODULATION_SCHEME_COHERENT) && (uc_mod_type == RT_MODULATION_DBPSK_BPSK)) {
				printf("->");
			}

			printf("\t5: ");
			_show_modulation(RT_MODULATION_SCHEME_COHERENT, RT_MODULATION_DBPSK_BPSK);

			if ((uc_mod_scheme == RT_MODULATION_SCHEME_COHERENT) && (uc_mod_type == RT_MODULATION_DQPSK_QPSK)) {
				printf("->");
			}

			printf("\t6: ");
			_show_modulation(RT_MODULATION_SCHEME_COHERENT, RT_MODULATION_DQPSK_QPSK);

			if ((uc_mod_scheme == RT_MODULATION_SCHEME_COHERENT) && (uc_mod_type == RT_MODULATION_D8PSK_8PSK)) {
				printf("->");
			}

			printf("\t7: ");
			_show_modulation(RT_MODULATION_SCHEME_COHERENT, RT_MODULATION_D8PSK_8PSK);
		}

		printf(">>> ");
		fflush(stdout);
		break;

	case APP_STATE_CONFIG_SRC_ADDRESS:
		/* Source Address configuration */
		printf("\r\n--- G3 MAC Source Address Configuration Menu ---\r\n");
		printf("Introduce New Address (0x%04x): 0x", app_plc_get_mac_src_address());
		fflush(stdout);
		break;

	case APP_STATE_CONFIG_DST_ADDRESS:
		/* Destination Address configuration */
		printf("\r\n--- G3 MAC Destination Address Configuration Menu ---\r\n");
		printf("Introduce New Address (0x%04x): 0x", app_plc_get_mac_dst_address());
		fflush(stdout);
		break;

#if defined(CONF_MULTIBAND_FCC_CENA)
	case APP_STATE_CONFIG_BAND:
		/* Band configuration */
		printf("\r\n--- Band Configuration Menu ---\r\n");
		printf("Select Band:\r\n");

		uc_phy_band = app_plc_get_phy_band();

		if (uc_phy_band == ATPL360_WB_CENELEC_A) {
			printf("->");
		}

		printf("\t%u: ", (uint32_t)ATPL360_WB_CENELEC_A);
		_show_band(ATPL360_WB_CENELEC_A);

		if (uc_phy_band == ATPL360_WB_FCC) {
			printf("->");
		}

		printf("\t%u: ", (uint32_t)ATPL360_WB_FCC);
		_show_band(ATPL360_WB_FCC);

		printf(">>> ");
		fflush(stdout);
		break;
#elif defined(CONF_MULTIBAND_FCC_CENB)
	case APP_STATE_CONFIG_BAND:
		/* Band configuration */
		printf("\r\n--- Band Configuration Menu ---\r\n");
		printf("Select Band:\r\n");

		uc_phy_band = app_plc_get_phy_band();

		if (uc_phy_band == ATPL360_WB_CENELEC_B) {
			printf("->");
		}

		printf("\t%u: ", (uint32_t)ATPL360_WB_CENELEC_B);
		_show_band(ATPL360_WB_CENELEC_B);

		if (uc_phy_band == ATPL360_WB_FCC) {
			printf("->");
		}

		printf("\t%u: ", (uint32_t)ATPL360_WB_FCC);
		_show_band(ATPL360_WB_FCC);

		printf(">>> ");
		fflush(stdout);
		break;
#endif
	}
}

/**
 * \brief Convert ASCII string into 16b hex value.
 *
 * \param puc_data Pointer to ASCII string
 *
 */
static uint16_t _get_hex16_value(uint8_t *puc_data)
{
	uint8_t uc_idx;
	uint8_t uc_value;
	uint16_t us_hex_value;

	us_hex_value = 0;

	for (uc_idx = 0; uc_idx < 4; uc_idx++) {
		us_hex_value <<= 4;
		uc_value = *(uint8_t *)puc_data;
		if ((uc_value >= '0') && (uc_value <= '9')) {
			uc_value -= 0x30;
		} else if ((uc_value >= 'A') && (uc_value <= 'F')) {
			uc_value -= 0x37;
		} else if ((uc_value >= 'a') && (uc_value <= 'f')) {
			uc_value -= 0x57;
		} else {
			return false;
		}

		us_hex_value += uc_value;
		puc_data++;
	}

	return us_hex_value;
}

/**
 * \brief Read one char from Serial port.
 *
 * \param c Pointer read char.
 *
 * \retval 0 Success. One char was read
 * \retval 1 There is no char to read.
 */
static uint32_t _read_char(uint8_t *c)
{
#ifdef CONF_BOARD_UDC_CONSOLE
	/* Read char through USB (SAMG55) */
	uint16_t us_res;
	us_res = usb_wrp_udc_read_buf(c, 1) ? 0 : 1;
	return us_res;

#else
#  if SAMG55 || PIC32CX
	/* Read char through USART */
	uint32_t ul_char;
	uint32_t ul_res;

	ul_res = usart_read((Usart *)CONF_UART, (uint32_t *)&ul_char);
	*c = (uint8_t)ul_char;

	return ul_res;

#  else
	/* Read char through UART */
	return uart_read((Uart *)CONF_UART, c);
#  endif
#endif
}

/**
 * \brief Handles received message. Prints message parameters and data.
 *
 * \param puc_data_buf Pointer to buffer containing received data.
 * \param us_data_len Length of received data in bytes. It may contain padding length.
 * \param uc_mod_scheme Modulation Scheme of received frame.
 * \param uc_mod_type Modulation Type of received frame.
 * \param uc_lqi Link Quality Indicator in quarters of dB and 10-dB offset (LQI = 0 means -10 dB).
 *
 */
void app_console_handle_rx_msg(uint8_t *puc_data_buf, uint16_t us_data_len, enum ERtModulationScheme uc_mod_scheme, enum ERtModulationType uc_mod_type, uint8_t uc_lqi)
{
	printf("\rRx (");
	/* Show Modulation of received frame */
	_show_modulation(uc_mod_scheme, uc_mod_type);
	/* Show LQI (Link Quality Indicator). It is in quarters of dB and 10-dB offset: SNR(dB) = (LQI - 40) / 4 */
	printf(", LQI %ddB): ", div_round((int16_t)uc_lqi - 40, 4));
	/* Show received message */
	printf("%.*s", us_data_len, puc_data_buf);

	_show_menu(false);
}

/**
 * \brief Handles reset of PL360 device (called also at initialization).
 *
 * \param uc_mod_scheme Configured Tx Modulation Scheme.
 * \param uc_mod_type Configured Tx Modulation Type.
 *
 */
void app_console_handle_pl360_reset(enum ERtModulationScheme uc_mod_scheme, enum ERtModulationType uc_mod_type)
{
	printf("\r\nTx Modulation: ");
	_show_modulation(uc_mod_scheme, uc_mod_type);

	/* Initialize data buffer index */
	sus_tx_data_index = 0;

	/* Reset Chat App state */
	suc_app_state = APP_STATE_IDLE;

	_show_menu(true);
}

/**
 * \brief Handles confirm of transmitted message.
 *
 */
void app_console_handle_tx_cfm(void)
{
	/* Initialize data buffer index */
	sus_tx_data_index = 0;

	/* Reset Chat App state */
	suc_app_state = APP_STATE_IDLE;

	_show_menu(false);
}

/**
 * \brief Initialization of Chat module.
 *
 */
void app_console_init(void)
{
	/* Initialize Chat App state */
	suc_app_state = APP_STATE_IDLE;
}

/**
 * \brief Chat module process. Process Serial port input.
 */
void app_console_process(void)
{
	/* Process Chat input */
	uint8_t uc_char;
	bool b_send_plc_msg;
	enum ERtModulationScheme uc_mod_scheme;
	enum ERtModulationType uc_mod_type;
	bool b_valid_modulation;
	uint8_t uc_phy_band;
	atpl360_res_t uc_result;
#if defined(CONF_MULTIBAND_FCC_CENA) || defined(CONF_MULTIBAND_FCC_CENB)
	bool b_valid_band;
	uint8_t uc_band;
#endif

	if (suc_app_state == APP_STATE_TRANSMITTING) {
		/* If transmitting, wait to end transmission to process incoming characters */
		return;
	}

	if (!_read_char((uint8_t *)&uc_char)) {
		/* Show received character if it is printable (ASCII) */
		if (((uc_char >= 32) && (uc_char <= 126)) || ((uc_char >= 128) && (uc_char <= 254)) || (uc_char == '\t') || (uc_char == '\r') || (uc_char == '\n')) {
			printf("%c", uc_char);
			fflush(stdout);
		}

		switch (suc_app_state) {
		case APP_STATE_IDLE:
			switch (uc_char) {
			case 0x04:
				/* Special character: 0x04 (CTRL+D). Show configuration parameters */
				suc_app_state = APP_STATE_SHOW_CONFIG;
				_show_menu(true);
				suc_app_state = APP_STATE_IDLE;
				_show_menu(false);
				break;

			case 0x13:
				/* Special character: 0x13 (CTRL+S). Enter configuration menu */
				/* Multiband supported: Go to configuration menu to select Band or Modulation configuration */
				suc_app_state = APP_STATE_CONFIG_MENU;
				_show_menu(true);
				break;

			case '\r':
			case '\n':
				/* Special character: '\r' (Carriage Return) or '\n' (Line Feed). Show menu */
				_show_menu(true);
				break;

			default:
				/* Normal character: Start to type message if it is printable (ASCII) */
				if (((uc_char >= 32) && (uc_char <= 126)) || ((uc_char >= 128) && (uc_char <= 254)) || (uc_char == '\t')) {
					suc_app_state = APP_STATE_TYPING;
				}
			}

			if (suc_app_state != APP_STATE_TYPING) {
				break;
			}

		case APP_STATE_TYPING:
			b_send_plc_msg = false;

			switch (uc_char) {
			case '\r':
				/* Special character: '\r' (carriage return). Send message through PLC */
				b_send_plc_msg = true;
				break;

			case '\b':
			case 0x7F:
				/* Special character: '\b' (backspace) or 0x7F (DEL). Remove character from Tx data buffer */
				if (sus_tx_data_index > 0) {
					sus_tx_data_index--;
					printf("\b \b");
					fflush(stdout);
				} else {
					/* All text has been removed: Go back to idle state */
					suc_app_state = APP_STATE_IDLE;
				}

				break;

			default:
				/* Normal character: Add to Tx data buffer if it is printable (ASCII) */
				if (((uc_char >= 32) && (uc_char <= 126)) || ((uc_char >= 128) && (uc_char <= 254)) || (uc_char == '\t') || (uc_char == '\n')) {
					spuc_tx_data_buff[sus_tx_data_index++] = uc_char;
					if (sus_tx_data_index == sizeof(spuc_tx_data_buff)) {
						/* Maximum data length reached: Send message through PLC */
						printf("\r\nMax data length reached... Message will be sent\r\n");
						b_send_plc_msg = true;
					}
				}
			}

			if (b_send_plc_msg) {
				/* Send PLC message */
				uc_result = app_plc_send_msg(spuc_tx_data_buff, sus_tx_data_index);

				switch (uc_result) {
				case ATPL360_SUCCESS:
					suc_app_state = APP_STATE_TRANSMITTING;
					break;

				case ATPL360_ERROR:
					printf("\r\nError sending PLC message: ATPL360_ERROR\r\n");
					suc_app_state = APP_STATE_IDLE;
					break;

				case ATPL360_HIGH_TEMP_110_ERROR:
					printf("\r\nError sending PLC message: ATPL360_HIGH_TEMP_110_ERROR\r\n");
					suc_app_state = APP_STATE_IDLE;
					break;

				default:
					printf("\r\nError sending PLC message: Unknown Error\r\n");
					suc_app_state = APP_STATE_IDLE;
					break;
				}
			}

			break;

		case APP_STATE_CONFIG_MENU:
			switch (uc_char) {
			case '0':
				/* Modulation configuration */
				suc_app_state = APP_STATE_CONFIG_MOD;
				sus_tx_data_index = 0;
				_show_menu(true);
				break;

			case '1':
				/* Source Address (Short) */
				suc_app_state = APP_STATE_CONFIG_SRC_ADDRESS;
				sus_tx_data_index = 0;
				_show_menu(true);
				break;

			case '2':
				/* Destination Address (Short) */
				suc_app_state = APP_STATE_CONFIG_DST_ADDRESS;
				sus_tx_data_index = 0;
				_show_menu(true);
				break;

#if defined(CONF_MULTIBAND_FCC_CENA) || defined(CONF_MULTIBAND_FCC_CENB)
			case '3':
				/* Band configuration */
				suc_app_state = APP_STATE_CONFIG_BAND;
				sus_tx_data_index = 0;
				_show_menu(true);
				break;
#endif

			default:
				printf("\r\nUnknown command. Skipping configuration\r\n");
				suc_app_state = APP_STATE_IDLE;
				break;
			}

			break;

		case APP_STATE_CONFIG_MOD:
			b_valid_modulation = true;
			uc_phy_band = app_plc_get_phy_band();
			switch (uc_char) {
			case '0':
				/* BPSK_ROBO_D */
				uc_mod_type = RT_MODULATION_ROBUST;
				uc_mod_scheme = RT_MODULATION_SCHEME_DIFFERENTIAL;
				break;

			case '1':
				/* BPSK_D */
				uc_mod_type = RT_MODULATION_DBPSK_BPSK;
				uc_mod_scheme = RT_MODULATION_SCHEME_DIFFERENTIAL;
				break;

			case '2':
				/* QPSK_D */
				uc_mod_type = RT_MODULATION_DQPSK_QPSK;
				uc_mod_scheme = RT_MODULATION_SCHEME_DIFFERENTIAL;
				break;

			case '3':
				/* 8PSK_D */
				uc_mod_type = RT_MODULATION_D8PSK_8PSK;
				uc_mod_scheme = RT_MODULATION_SCHEME_DIFFERENTIAL;
				if (uc_phy_band == ATPL360_WB_ARIB) {
					/* 8PSK Modulation Type not supported in ARIB band */
					printf("\r\n8PSK modulation not supported in ARIB band. Skipping configuration\r\n");
					b_valid_modulation = false;
				}

				break;

			case '4':
				/* BPSK_ROBO_C */
				uc_mod_type = RT_MODULATION_ROBUST;
				uc_mod_scheme = RT_MODULATION_SCHEME_COHERENT;
				if (uc_phy_band == ATPL360_WB_ARIB) {
					/* Coherent Modulation Scheme not supported in ARIB band */
					printf("\r\nCoherent modulation not supported in ARIB band. Skipping configuration\r\n");
					b_valid_modulation = false;
				}

				break;

			case '5':
				/* BPSK_C */
				uc_mod_type = RT_MODULATION_DBPSK_BPSK;
				uc_mod_scheme = RT_MODULATION_SCHEME_COHERENT;
				if (uc_phy_band == ATPL360_WB_ARIB) {
					/* Coherent Modulation Scheme not supported in ARIB band */
					printf("\r\nCoherent modulation not supported in ARIB band. Skipping configuration\r\n");
					b_valid_modulation = false;
				}

				break;

			case '6':
				/* QPSK_C */
				uc_mod_type = RT_MODULATION_DQPSK_QPSK;
				uc_mod_scheme = RT_MODULATION_SCHEME_COHERENT;
				if (uc_phy_band == ATPL360_WB_ARIB) {
					/* Coherent Modulation Scheme not supported in ARIB band */
					printf("\r\nCoherent modulation not supported in ARIB band. Skipping configuration\r\n");
					b_valid_modulation = false;
				}

				break;

			case '7':
				/* 8PSK_C */
				uc_mod_type = RT_MODULATION_D8PSK_8PSK;
				uc_mod_scheme = RT_MODULATION_SCHEME_COHERENT;
				if (uc_phy_band == ATPL360_WB_ARIB) {
					/* Coherent Modulation Scheme not supported in ARIB band */
					printf("\r\nCoherent modulation not supported in ARIB band. Skipping configuration\r\n");
					b_valid_modulation = false;
				}

				break;

			default:
				printf("\r\nUnknown command. Skipping configuration\r\n");
				b_valid_modulation = false;
				break;
			}

			if (b_valid_modulation) {
				/* Set new Modulation Type & Modulation Scheme. Get maximum data length allowed with configured Tx Parameters */
				app_plc_set_modulation(uc_mod_scheme, uc_mod_type);
				printf("\r\nTx Modulation: ");
				_show_modulation(uc_mod_scheme, uc_mod_type);
			}

			suc_app_state = APP_STATE_IDLE;
			_show_menu(false);
			break;

		case APP_STATE_CONFIG_SRC_ADDRESS:
			switch (uc_char) {
			case '\b':
			case 0x7F:
				/* Special character: '\b' (backspace) or 0x7F (DEL). Remove character from Tx data buffer */
				if (sus_tx_data_index > 0) {
					sus_tx_data_index--;
					printf("\b \b");
					fflush(stdout);
				} else {
					/* All text has been removed: Go back to idle state */
					suc_app_state = APP_STATE_IDLE;
				}

				break;

			default:
				/* Normal character: Add to Tx data buffer if it is hexadecimal */
				if (((uc_char >= '0') && (uc_char <= '9')) || ((uc_char >= 'a') && (uc_char <= 'f')) || (uc_char >= 'A') || (uc_char <= 'F')) {
					spuc_tx_data_buff[sus_tx_data_index++] = uc_char;
					if (sus_tx_data_index == 4) {
						uint16_t us_address;

						sus_tx_data_index = 0;

						/* Get Hexadecimal value */
						us_address = _get_hex16_value(spuc_tx_data_buff);
						app_plc_set_mac_src_address(us_address);
						printf("\r\nG3 MAC Source Address: 0x%04X", us_address);
						suc_app_state = APP_STATE_IDLE;
						_show_menu(false);
					}
				}
			}
			break;

		case APP_STATE_CONFIG_DST_ADDRESS:
			switch (uc_char) {
			case '\b':
			case 0x7F:
				/* Special character: '\b' (backspace) or 0x7F (DEL). Remove character from Tx data buffer */
				if (sus_tx_data_index > 0) {
					sus_tx_data_index--;
					printf("\b \b");
					fflush(stdout);
				} else {
					/* All text has been removed: Go back to idle state */
					suc_app_state = APP_STATE_IDLE;
				}

				break;

			default:
				/* Normal character: Add to Tx data buffer if it is hexadecimal */
				if (((uc_char >= '0') && (uc_char <= '9')) || ((uc_char >= 'a') && (uc_char <= 'f')) || (uc_char >= 'A') || (uc_char <= 'F')) {
					spuc_tx_data_buff[sus_tx_data_index++] = uc_char;
					if (sus_tx_data_index == 4) {
						uint16_t us_address;

						sus_tx_data_index = 0;

						/* Get Hexadecimal value */
						us_address = _get_hex16_value(spuc_tx_data_buff);
						app_plc_set_mac_dst_address(us_address);
						printf("\r\nG3 MAC Destination Address: 0x%04X", us_address);
						suc_app_state = APP_STATE_IDLE;
						_show_menu(false);
					}
				}
			}
			break;

#if defined(CONF_MULTIBAND_FCC_CENA)
		case APP_STATE_CONFIG_BAND:
			b_valid_band = true;
			switch (uc_char) {
			case '1':
				/* CENELEC-A */
				uc_band = ATPL360_WB_CENELEC_A;
				break;

			case '2':
				/* FCC */
				uc_band = ATPL360_WB_FCC;
				break;

			default:
				printf("\r\nUnknown command. Skipping configuration\r\n");
				b_valid_band = false;
				break;
			}

			if (b_valid_band) {
				/* Set new Band on PL360 */
				b_valid_band = app_plc_set_band(uc_band);
			}

			suc_app_state = APP_STATE_IDLE;

			if (!b_valid_band) {
				_show_menu(false);
			}
			break;
#elif defined(CONF_MULTIBAND_FCC_CENB)
		case APP_STATE_CONFIG_BAND:
			b_valid_band = true;
			switch (uc_char) {
			case '1':
				/* CENELEC-B */
				uc_band = ATPL360_WB_CENELEC_B;
				break;

			case '2':
				/* FCC */
				uc_band = ATPL360_WB_FCC;
				break;

			default:
				printf("\r\nUnknown command. Skipping configuration\r\n");
				b_valid_band = false;
				break;
			}

			if (b_valid_band) {
				/* Set new Band on PL360 */
				b_valid_band = app_plc_set_band(uc_band);
			}

			suc_app_state = APP_STATE_IDLE;

			if (!b_valid_band) {
				_show_menu(false);
			}
			break;
#endif
		}
	}
}
