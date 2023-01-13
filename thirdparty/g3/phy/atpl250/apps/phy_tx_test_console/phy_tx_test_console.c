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

/**
 *  \mainpage ATMEL PLC PHY TX Test Console Application
 *
 *  \section Purpose
 *
 *  The PHY TX Test Console Application demonstrates how to configure some
 * parameters from the PHY layer on PLC boards.
 *
 *  \section Requirements
 *
 *  This package should be used with any PLC board on which there is PLC
 * hardware dedicated.
 *
 *  \section Description
 *
 *  This application can configure the PHY with a serial interface and test
 * PLC transmission/reception processes.
 *
 *  \section Usage
 *
 *  The tool is ready to configure, transmit and receive.
 */

/* System includes */
#include <stdint.h>
#include <stdio.h>
#include "string.h"

/* Atmel library includes. */
#include "asf.h"
#include "math.h"

#include "atpl250_version.h"
#include "phy_embedded_example.h"
#include "atpl250_carrier_mapping.h"

#include "hal/hal.h"

/* Example configuration */
#include "conf_oss.h"
#include "conf_project.h"

#ifndef SAM4C
/* random number library */
	#include "stdlib.h"
#endif

#define STRING_EOL    "\r"
#define STRING_HEADER "\r\n-- ATMEL PLC Getting Started Application --\r\n" \
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --\r\n" \
	"-- PHY ATPL250 --\r\n"

#define MENU_HEADER "\n\r-- Menu Configuration --------------\n\r" \
	"0: Select TX Level\n\r" \
	"1: Select Modulation/Scheme\n\r" \
	"2: Select time period between messages to transmit(us.)\n\r" \
	"3: Select Data to transmit\n\r" \
	"4: Select TX tone Map\n\r" \
	"5: Select TX preemphasis\n\r" \
	"6: Select Branch Mode\n\r" \
	"7: Set/Clear Force No Output Signal\n\r" \
	"v: View TX configuration values\n\r" \
	"e: Execute transmission application\n\r" \
	"otherwise: Display this main menu\n\n\r"

#define MENU_SCHEME "\n\r-- Modulation Scheme --------------\r\n" \
	"0: DBPSK\n\r" \
	"1: DQPSK\n\r" \
	"2: D8PSK\n\r" \
	"3: Differential Robust\n\r" \
	"\n\r" \
	"4: Coherent BPSK\n\r" \
	"5: Coherent QPSK\n\r" \
	"6: Coherent 8PSK\n\r" \
	"7: Coherent Robust\n\r" \

#define MENU_MODE "\n\r-- Transmission Mode --------------\r\n"	\
	"0: Immediate and Not Forced\n\r" \
	"1: Immediate and Forced\n\r" \
	"2: Delayed and Not Forced\n\r"	\
	"3: Delayed and Forced\n\r"

#define MENU_DATA_MODE "\n\r-- Select Data Mode --------------\r\n" \
	"0: Random Data\n\r" \
	"1: Fixed Data\n\r" \
	"2: Manual Data\n\r" \
	"3: Manual Hex Data\n\r"

#define MENU_BRANCH_MODE "\n\r-- Select Branch Mode --------------\r\n"	\
	"0: Autodetect\n\r" \
	"1: High Impedance\n\r"	\
	"2: Low Impedance\n\r" \
	"3: Very Low Impedance\n\r"

#define MENU_NO_OUTPUT "\n\r-- Force No Output Signal --------------\r\n" \
	"0: Clear\n\r" \
	"1: Set\n\r"

#define MENU_CONSOLE "\n\rPHY-Console>"

#define FL_BAND_CENELEC_A   4
#define FL_BAND_FCC_ARIB    1

#define FL_MAX_CENELEC_A    63
#define FL_MAX_FCC          511
#define FL_MAX_ARIB         255

#define MAX_PSDU_CALC_REMOVE_SYMBOLS_CENELEC_A    4
#define MAX_PSDU_CALC_REMOVE_SYMBOLS_FCC_ARIB     1

#define MAX_RS_BLOCK_SIZE 255

#define CC_RATE_INV 2
#define CC_ZERO_TAIL 6

#define MY_DIV_CEIL(X, Y)  (X / Y + (X % Y != 0))

/* Phy data configuration */
extern txPhyEmbeddedConfig_t xAppPhyCfgTx;

/* Tx data buffer */
uint8_t ucv_tx_data_buffer[512];

/*each bit represent a carrier [0-127] (1 inactive 0 active) only for tone map*/
uint8_t m_auc_inactive_carriers_pos[CARR_BUFFER_LEN];

/*each bit represent a carrier [0-127] (1 inactive 0 active) only for tone map*/
uint8_t m_auc_static_notching[CARR_BUFFER_LEN];

/* Number of subbands */
uint8_t uc_local_num_subbands;

/**
 * \brief Calculates maximun length in bytes of psdu
 *
 * \param uc_num_active_carriers   Number of active carriers taking into account dynamic and static notching
 * \param uc_num_pilots            Number of pilots in each symbol, 0 for differential modulations schemes
 * \param e_mod_type               Modulation type of payload
 * \param uc_2_rs_blocks           Payload divided in 2 RS block, '1' second block present  '0' only one block
 *
 * \return Max PSDU size in bytes
 *
 */
static uint16_t get_max_psdu_len(void)
{
	uint8_t uc_rep_code;
	uint8_t uc_mod_size;
	uint8_t uc_parity_len;

	uint16_t us_max_rs_block_size;
	uint8_t uc_cc_zero_tail;

	uint16_t us_ns;

	uint16_t us_aux_ceil;
	uint16_t us_aux_floor;
	uint16_t us_result;

	uint8_t uc_num_active_carriers;
	uint8_t uc_num_pilots = 0;
	uint8_t uc_2_rs_blocks = 0;

	uint16_t us_fl_max = 0;
	uint8_t uc_fl_band;
	uint8_t uc_max_psdu_calc_remove_symbols;

	uc_2_rs_blocks = xAppPhyCfgTx.xPhyMsg.m_uc_2_rs_blocks;

	#if defined(CONF_BAND_CENELEC_A)
	generate_inactive_carriers_cenelec_a(xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0], m_auc_inactive_carriers_pos);
	us_fl_max = FL_MAX_CENELEC_A;
	uc_fl_band = FL_BAND_CENELEC_A;
	uc_max_psdu_calc_remove_symbols = MAX_PSDU_CALC_REMOVE_SYMBOLS_CENELEC_A;
	#elif defined(CONF_BAND_FCC)
	generate_inactive_carriers_fcc(xAppPhyCfgTx.xPhyMsg.m_auc_tone_map, m_auc_inactive_carriers_pos);
	us_fl_max = FL_MAX_FCC;
	uc_fl_band = FL_BAND_FCC_ARIB;
	uc_max_psdu_calc_remove_symbols = MAX_PSDU_CALC_REMOVE_SYMBOLS_FCC_ARIB;
	#else
	generate_inactive_carriers_arib(xAppPhyCfgTx.xPhyMsg.m_auc_tone_map, m_auc_inactive_carriers_pos);
	us_fl_max = FL_MAX_ARIB;
	uc_fl_band = FL_BAND_FCC_ARIB;
	uc_max_psdu_calc_remove_symbols = MAX_PSDU_CALC_REMOVE_SYMBOLS_FCC_ARIB;
	#endif

	if (uc_2_rs_blocks) {
		us_fl_max = 255;
	}

	/* TODO read static notching and make the or with previous result */
	memset(m_auc_static_notching, 0x00, CARR_BUFFER_LEN);

	uc_num_active_carriers = get_active_carriers(m_auc_inactive_carriers_pos, m_auc_static_notching);

	if (xAppPhyCfgTx.xPhyMsg.e_mod_scheme == MOD_SCHEME_COHERENT) {
		uc_num_pilots = uc_num_active_carriers / PILOT_FREQ_SPA;
	}

	us_max_rs_block_size = MAX_RS_BLOCK_SIZE;
	uc_cc_zero_tail = CC_ZERO_TAIL;

	switch (xAppPhyCfgTx.xPhyMsg.e_mod_type) {
	case MOD_TYPE_BPSK:
		uc_rep_code = 1;
		uc_mod_size = 1;
		uc_parity_len = 16;
		break;

	case MOD_TYPE_QPSK:
		uc_rep_code = 1;
		uc_mod_size = 2;
		uc_parity_len = 16;
		break;

	case MOD_TYPE_8PSK:
		uc_rep_code = 1;
		uc_mod_size = 3;
		uc_parity_len = 16;
		break;

	case MOD_TYPE_BPSK_ROBO:
		uc_rep_code = 4;
		uc_mod_size = 1;
		uc_parity_len = 8;
		break;

	default:
		uc_mod_size = 0;
		uc_rep_code = 1;
		uc_parity_len = 16;
		break;
	}

	us_aux_ceil = MY_DIV_CEIL(
			((us_max_rs_block_size * 8 + uc_cc_zero_tail) * uc_rep_code * CC_RATE_INV)
			,
			((uc_fl_band * (uc_num_active_carriers - uc_num_pilots) * uc_mod_size))
			);

	if (us_aux_ceil > us_fl_max) {
		us_aux_ceil = uc_fl_band * us_fl_max;
	} else {
		us_aux_ceil = us_aux_ceil * uc_fl_band;
	}

	us_ns = 0x01FF & us_aux_ceil;

	us_aux_floor
		= (
		(us_ns * (uc_num_active_carriers - uc_num_pilots) * uc_mod_size)
		-
		(uc_cc_zero_tail * uc_rep_code * CC_RATE_INV)
		)
			/
			(8 * uc_rep_code * CC_RATE_INV);

	if (us_aux_floor > us_max_rs_block_size) {
		us_ns -= uc_max_psdu_calc_remove_symbols;
		us_aux_floor =                          (
			(us_ns * (uc_num_active_carriers - uc_num_pilots) * uc_mod_size)
			-
			(uc_cc_zero_tail * uc_rep_code * CC_RATE_INV)
			)
				/
				(8 * uc_rep_code * CC_RATE_INV);
	}

	us_result = us_aux_floor - uc_parity_len;

	if (us_result & 0x8000) {
		us_result = 0;
	}

	us_result *= (uc_2_rs_blocks + 1);

	return us_result;
}

/* StateMachine Events */
typedef enum {
	MAIN_MENU_OPT0,
	MAIN_MENU_OPT1,
	MAIN_MENU_OPT2,
	MAIN_MENU_OPT3,
	MAIN_MENU_OPT4,
	MAIN_MENU_OPT5,
	MAIN_MENU_OPT6,
	MAIN_MENU_OPT7,
	MAIN_MENU_VIEW_CFG,
	MAIN_MENU_EXECUTE,
	STOP_TEST,
	UNKNOW_CHAR,
	NO_CHAR
} EIndication;

/* StateMachine holds the current state pointer. */
typedef struct StateMachine TStateMachine;

/* StateProc function pointer type that represent each state of our machine. */
typedef void (*StateProc)(TStateMachine *sm, EIndication input);

struct StateMachine {
	StateProc curState;
};

void main_menu_show(TStateMachine *sm, EIndication input);
void main_menu_wait(TStateMachine *sm, EIndication input);
void opt0_menu(TStateMachine *sm, EIndication input);
void opt1_menu(TStateMachine *sm, EIndication input);
void opt2_menu(TStateMachine *sm, EIndication input);
void opt3_menu(TStateMachine *sm, EIndication input);
void opt4_menu(TStateMachine *sm, EIndication input);
void opt5_menu(TStateMachine *sm, EIndication input);
void opt6_menu(TStateMachine *sm, EIndication input);
void opt7_menu(TStateMachine *sm, EIndication input);
void show_cfg(TStateMachine *sm, EIndication input);
void executing(TStateMachine *sm, EIndication input);

EIndication processInput(void);

TStateMachine machine = { main_menu_show };

/**
 * Set configuration parameters in GPBR
 */
static void save_config(uint8_t cmd_start_mode)
{
	uint32_t ul_gpbr_value;
#if (SAM4C || SAM4CP || SAM4CM)
  #if defined(CONF_ARIB)
	uint8_t uc_tmp;
  #endif
#endif

	ul_gpbr_value = cmd_start_mode;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.e_mod_type << 4; /* 3 bits */
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.e_mod_scheme << 7; /* 1 bit */
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_uc_tx_power << 8; /* 5 bits */
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_uc_tx_mode << 13; /* 2 bits */
	ul_gpbr_value += xAppPhyCfgTx.uc_is_random << 15; /* 1 bit */
	ul_gpbr_value += xAppPhyCfgTx.uc_force_no_output << 16; /* 1 bit */

	gpbr_write(GPBR0, ul_gpbr_value);
	#if defined(CONF_BAND_CENELEC_A)
	gpbr_write(GPBR1, (uint32_t)xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0]);
	#else
	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[1] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[2] << 16;
	gpbr_write(GPBR1, ul_gpbr_value);
	#endif
	gpbr_write(GPBR2, xAppPhyCfgTx.ul_tx_period);
	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_us_data_len;
	gpbr_write(GPBR3, ul_gpbr_value);

#if (SAM4C || SAM4CP || SAM4CM)
	#if defined(CONF_BAND_CENELEC_A)
	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[1] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[2] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[3] << 24;
	gpbr_write(GPBR8, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[4];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[5] << 8;
	gpbr_write(GPBR9, ul_gpbr_value);
	#elif defined(CONF_BAND_FCC)
	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[1] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[2] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[3] << 24;
	gpbr_write(GPBR8, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[4];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[5] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[6] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[7] << 24;
	gpbr_write(GPBR9, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[8];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[9] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[10] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[11] << 24;
	gpbr_write(GPBR10, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[12];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[13] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[14] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[15] << 24;
	gpbr_write(GPBR11, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[16];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[17] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[18] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[19] << 24;
	gpbr_write(GPBR12, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[20];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[21] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[22] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[23] << 24;
	gpbr_write(GPBR13, ul_gpbr_value);
	#else
	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[1] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[2] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[3] << 24;
	gpbr_write(GPBR8, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[4];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[5] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[6] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[7] << 24;
	gpbr_write(GPBR9, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[8];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[9] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[10] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[11] << 24;
	gpbr_write(GPBR10, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[12];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[13] << 8;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[14] << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[15] << 24;
	gpbr_write(GPBR11, ul_gpbr_value);

	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[16];
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[17] << 8;

	phy_get_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &uc_tmp, 1);
	ul_gpbr_value += uc_tmp << 16;
	phy_get_cfg_param(PHY_ID_CFG_IMPEDANCE, &uc_tmp, 1);
	ul_gpbr_value += uc_tmp << 24;
	gpbr_write(GPBR12, ul_gpbr_value);
	#endif
#endif
}

/**
 * Get configuration parameters from GPBR
 */
static uint8_t load_config(void)
{
	uint32_t uc_gpbr_value;
	uint8_t uc_start_mode;
#if (SAM4C || SAM4CP || SAM4CM)
  #if defined(CONF_ARIB)
	uint8_t uc_tmp;
  #endif
#endif

	uc_gpbr_value = gpbr_read(GPBR0);
	uc_start_mode = uc_gpbr_value & 0x0F;
	if ((uc_start_mode == PHY_APP_CMD_MENU_START_MODE) || \
			(uc_start_mode == PHY_APP_CMD_TX_START_MODE)) {
		xAppPhyCfgTx.xPhyMsg.e_mod_type = (enum mod_types)((uc_gpbr_value >> 4) & 0x07);
		xAppPhyCfgTx.xPhyMsg.e_mod_scheme = (enum mod_schemes)((uc_gpbr_value >> 7) & 0x01);
		xAppPhyCfgTx.xPhyMsg.m_uc_tx_power = (uc_gpbr_value >> 8) & 0x1F;
		xAppPhyCfgTx.xPhyMsg.m_uc_tx_mode = (uc_gpbr_value >> 13) & 0x03;
		xAppPhyCfgTx.uc_is_random = (uc_gpbr_value >> 15) & 0x01;
		xAppPhyCfgTx.uc_force_no_output = (uc_gpbr_value >> 16) & 0x01;

		if (xAppPhyCfgTx.uc_force_no_output) {
			phy_set_cfg_param(PHY_ID_FORCE_NO_OUTPUT_SIGNAL, &xAppPhyCfgTx.uc_force_no_output, 1);
		}

		uc_gpbr_value = gpbr_read(GPBR1);
		#if defined(CONF_BAND_CENELEC_A)
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0] = uc_gpbr_value & 0xFF;
	#else
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[1] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[2] = (uc_gpbr_value & 0xFF0000) >> 16;
	#endif

	#if (SAM4C || SAM4CP || SAM4CM)
		#if defined(CONF_BAND_CENELEC_A)
		uc_gpbr_value = gpbr_read(GPBR8);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[1] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[2] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[3] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR9);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[4] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[5] = (uc_gpbr_value & 0xFF00) >> 8;
		#elif defined(CONF_BAND_FCC)
		uc_gpbr_value = gpbr_read(GPBR8);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[1] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[2] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[3] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR9);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[4] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[5] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[6] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[7] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR10);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[8] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[9] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[10] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[11] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR11);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[12] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[13] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[14] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[15] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR12);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[16] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[17] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[18] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[19] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR13);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[20] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[21] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[22] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[23] = (uc_gpbr_value & 0xFF000000) >> 24;
		#else
		uc_gpbr_value = gpbr_read(GPBR8);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[1] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[2] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[3] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR9);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[4] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[5] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[6] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[7] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR10);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[8] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[9] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[10] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[11] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR11);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[12] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[13] = (uc_gpbr_value & 0xFF00) >> 8;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[14] = (uc_gpbr_value & 0xFF0000) >> 16;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[15] = (uc_gpbr_value & 0xFF000000) >> 24;
		uc_gpbr_value = gpbr_read(GPBR12);
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[16] = uc_gpbr_value & 0xFF;
		xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[17] = (uc_gpbr_value & 0xFF00) >> 8;
		uc_tmp = (uc_gpbr_value & 0xFF0000) >> 16;
		phy_set_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &uc_tmp, 1);
		uc_tmp = (uc_gpbr_value & 0xFF000000) >> 24;
		phy_set_cfg_param(PHY_ID_CFG_IMPEDANCE, &uc_tmp, 1);
		#endif
	#endif

		xAppPhyCfgTx.ul_tx_period = gpbr_read(GPBR2);
		uc_gpbr_value = gpbr_read(GPBR3);
		xAppPhyCfgTx.xPhyMsg.m_us_data_len = uc_gpbr_value & 0xFFFF;

		/* upload the content of data message from flash memory */
		memcpy(ucv_tx_data_buffer, (uint8_t *)ADDR_APP_PHY_MESSAGE_DATA, \
				xAppPhyCfgTx.xPhyMsg.m_us_data_len );
	} else {
		uc_start_mode = PHY_APP_CMD_DEFAULT_MODE;
	}

	return uc_start_mode;
}

/**
 * Display current information
 */
static void display_config(void)
{
	uint8_t uc_i;
	uint8_t uc_tone_map_len;
	uint8_t *puc_tone_map;
	uint8_t uc_branch_mode;
	uint8_t uc_impedance_mode;

	phy_get_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &uc_branch_mode, 1);
	phy_get_cfg_param(PHY_ID_CFG_IMPEDANCE, &uc_impedance_mode, 1);

	printf("\n\r-- Configuration Info --------------\r\n");
	printf("-I- Tx Level: %lu\n\r", (unsigned long)xAppPhyCfgTx.xPhyMsg.m_uc_tx_power);
	switch (xAppPhyCfgTx.xPhyMsg.e_mod_type) {
	case MOD_TYPE_BPSK:
		if (xAppPhyCfgTx.xPhyMsg.e_mod_scheme) {
			printf("-I- Modulation Scheme: Coherent BPSK \n\r");
		} else {
			printf("-I- Modulation Scheme: Differential BPSK \n\r");
		}

		break;

	case MOD_TYPE_QPSK:
		if (xAppPhyCfgTx.xPhyMsg.e_mod_scheme) {
			printf("-I- Modulation Scheme: Coherent QPSK \n\r");
		} else {
			printf("-I- Modulation Scheme: Differential QPSK \n\r");
		}

		break;

	case MOD_TYPE_8PSK:
		if (xAppPhyCfgTx.xPhyMsg.e_mod_scheme) {
			printf("-I- Modulation Scheme: Coherent 8PSK \n\r");
		} else {
			printf("-I- Modulation Scheme: Differential 8PSK \n\r");
		}

		break;

	case MOD_TYPE_BPSK_ROBO:
		if (xAppPhyCfgTx.xPhyMsg.e_mod_scheme) {
			printf("-I- Modulation Scheme: Coherent Robust\n\r");
		} else {
			printf("-I- Modulation Scheme: Differential Robust\n\r");
		}

		break;

	default:
		printf("-I-  Modulation Scheme: ERROR CFG\n\r");
	}
	#if defined(CONF_BAND_CENELEC_A)
	uc_tone_map_len = 1;
	puc_tone_map = &xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0];
	#else
	uc_tone_map_len = 3;
	puc_tone_map = &xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0];
	#endif
	printf("-I- Tone Map: ");
	for (uc_i = 0; uc_i <  uc_tone_map_len; uc_i++) {
		if (uc_i > 0) {
			printf(":");
		}

		printf("%02X", *(puc_tone_map + uc_i));
	}
	printf("\r\n");

	printf("-I- Preemphasis: ");
	for (uc_i = 0; uc_i <  uc_local_num_subbands; uc_i++) {
		if (uc_i > 0) {
			printf(":");
		}

		printf("%02X", *(&xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0] + uc_i));
	}
	printf("\r\n");

	if (uc_branch_mode) {
		printf("-I- Branch Mode : Autodetect \r\n");
	} else if (uc_impedance_mode == HI_STATE) {
		printf("-I- Branch Mode : Fixed High Impedance \r\n");
	} else if (uc_impedance_mode == LO_STATE) {
		printf("-I- Branch Mode : Fixed Low Impedance \r\n");
	} else if (uc_impedance_mode == VLO_STATE) {
		printf("-I- Branch Mode : Fixed Very Low Impedance \r\n");
	}

	printf("-I- Forced No Output Signal: %u\n\r", (unsigned int)xAppPhyCfgTx.uc_force_no_output);
	printf("-I- Time Period: %lu\n\r", (unsigned long)xAppPhyCfgTx.ul_tx_period);
	printf("-I- Data Len: %lu\n\r", (unsigned long)xAppPhyCfgTx.xPhyMsg.m_us_data_len);

	printf(MENU_CONSOLE);
	fflush(stdout);
}

/**
 * Get ID of transmission level.
 */
static void get_transmission_level(void)
{
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif
	uint16_t us_level;

	printf("Enter transmission level using 2 digits [00..31] : ");
	fflush(stdout);
	while (1) {
#if SAMG55
		while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
		}
		uc_char = (uint8_t)ul_char;
#else
		while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
		}
#endif
		printf("%c", uc_char);
		fflush(stdout);
		us_level = (uc_char - 0x30) * 10;
#if SAMG55
		while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
		}
		uc_char = (uint8_t)ul_char;
#else
		while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
		}
#endif
		printf("%c\r\n", uc_char);
		us_level += (uc_char - 0x30);
		if (us_level < 32) {
			printf("->Attenuation level %lu ok\r\n", (unsigned long)us_level);
			xAppPhyCfgTx.xPhyMsg.m_uc_tx_power = us_level;
			printf(MENU_CONSOLE);
			fflush(stdout);
			return;
		} else {
			printf("ERROR: Attenuation level not permitted [0..31]. Try again.\n\r");
		}
	}
}

/**
 * Get scheme of modulation.
 */
static void get_transmission_scheme(void)
{
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif

	puts(MENU_SCHEME);
	fflush(stdout);
	while (1) {
#if SAMG55
		while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
		}
		uc_char = (uint8_t)ul_char;
#else
		while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
		}
#endif
		switch (uc_char) {
		case '0':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_BPSK;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
			break;

		case '1':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_QPSK;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
			break;

		case '2':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_8PSK;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
			break;

		case '3':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_BPSK_ROBO;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
			break;

		case '4':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_BPSK;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_COHERENT;
			break;

		case '5':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_QPSK;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_COHERENT;
			break;

		case '6':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_8PSK;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_COHERENT;
			break;

		case '7':
			xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_BPSK_ROBO;
			xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_COHERENT;
			break;

		default:
			continue;
		}
		printf("->Scheme %c ok\r\n", uc_char);

		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

/**
 * Get branch Tx mode.
 */
static void get_branch_mode(void)
{
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif

	uint8_t uc_branch_mode;
	uint8_t uc_impedance_mode;

	puts(MENU_BRANCH_MODE);
	fflush(stdout);

	while (1) {
#if SAMG55
		while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
		}
		uc_char = (uint8_t)ul_char;
#else
		while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
		}
#endif

		switch (uc_char) {
		case '0':
			uc_impedance_mode = HI_STATE;
			uc_branch_mode = 0x01;
			printf("-->Branch Mode: Autodetect OK \n\r");
			break;

		case '1':
			uc_impedance_mode = HI_STATE;
			uc_branch_mode = 0x00;
			printf("-->Branch Mode: High Impedance OK \n\r");
			break;

		case '2':
			uc_impedance_mode = LO_STATE;
			uc_branch_mode = 0x00;
			printf("-->Branch Mode: Low Impedance OK \n\r");
			break;

		case '3':
			uc_impedance_mode = VLO_STATE;
			uc_branch_mode = 0x00;
			printf("-->Branch Mode: Very Low Impedance OK \n\r");
			break;

		default:
			continue;
		}

		phy_set_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &uc_branch_mode, 1);
		phy_set_cfg_param(PHY_ID_CFG_IMPEDANCE, &uc_impedance_mode, 1);

		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

/**
 * Get Tx Output Signal state.
 */
static void get_forced_output(void)
{
	uint8_t uc_force_no_output;
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif

	puts(MENU_NO_OUTPUT);
	fflush(stdout);

	while (1) {
#if SAMG55
		while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
		}
		uc_char = (uint8_t)ul_char;
#else
		while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
		}
#endif

		switch (uc_char) {
		case '0':
			uc_force_no_output = 0;
			printf("-->Forced No Output Signal CLEARED OK \n\r");
			break;

		case '1':
			uc_force_no_output = 1;
			printf("-->Forced No Output Signal SET OK \n\r");
			break;

		default:
			printf("\r\n->ERROR Invalid char \r\n");
			printf(MENU_CONSOLE);
			fflush(stdout);
			return;
		}

		xAppPhyCfgTx.uc_force_no_output = uc_force_no_output;

		phy_set_cfg_param(PHY_ID_FORCE_NO_OUTPUT_SIGNAL, &uc_force_no_output, 1);

		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

/**
 * Fill data message for tone map.
 */
static void get_tone_map(void)
{
	uint16_t uc_i;
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif
	uint8_t *p_data_buf;
	uint8_t uc_tone_map_len;

	#if defined(CONF_BAND_CENELEC_A)
	printf("Please enter value for tone map (from 01 to 3F) and press enter: ");

	/* Asgin pointer to tx data buffer */
	p_data_buf = &xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0];
	uc_tone_map_len = 1;
	#elif defined(CONF_BAND_FCC)
	printf("Please enter value for tone map (from 000001 to FFFFFF) and press enter: ");
	uc_tone_map_len = 3;

	/* Asgin pointer to tx data buffer */
	p_data_buf = &xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0];
	#else
	printf("Please enter value for tone map (from 000001 to 03FFFF)and press enter: ");
	uc_tone_map_len = 3;

	/* Asgin pointer to tx data buffer */
	p_data_buf = &xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0];
	#endif
	fflush(stdout);

	while (1) {
		for (uc_i = 0; uc_i < uc_tone_map_len * 2 + 1; uc_i++) {
#if SAMG55
			while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
			}
			uc_char = (uint8_t)ul_char;
#else
			while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
			}
#endif

			if (uc_char == 0x0D) {
				printf("\r\n->Tone-Map ready.\r\n");
				printf(MENU_CONSOLE);
				fflush(stdout);
				return;
			} else {
				printf("%c", uc_char);
				fflush(stdout);
				if (uc_char >= 0x30 && uc_char < 0x40) { /* 0 to 9 */
					if (uc_i % 2) {
						*p_data_buf++ += uc_char - 0x30;
					} else {
						*p_data_buf = (uc_char - 0x30) << 4;
					}
				} else if (uc_char >= 0x41 && uc_char < 0x47) {
					if (uc_i % 2) {
						*p_data_buf++ += uc_char - 0x37;
					} else {
						*p_data_buf = (uc_char - 0x37) << 4;
					}
				} else if (uc_char >= 0x61 && uc_char < 0x67) {
					if (uc_i % 2) {
						*p_data_buf++ += uc_char - 0x57;
					} else {
						*p_data_buf = (uc_char - 0x57) << 4;
					}
				} else {
					printf("\r\n->ERROR Invalid char \r\n");
					return;
				}
			}
		}
		printf("\r\n->End: Maximum Length is %d bytes\r\n", uc_tone_map_len);
		fflush(stdout);
	}
}

/**
 * Fill data message for preemphasis
 */
static void get_preemphasis(void)
{
	uint16_t uc_i;
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif
	uint8_t *p_data_buf;
	uint8_t uc_preemphasis_len;

	#if defined(CONF_BAND_CENELEC_A)
	printf("Please enter value for preemphasis (6 bytes default 000000000000): ");

	/* Asgin pointer to tx data buffer */
	p_data_buf = &xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0];
	uc_preemphasis_len = 6;
	#elif defined(CONF_BAND_FCC)
	printf("Please enter value for preemphasis (24 bytes default 000000000000000000000000000000000000000000000000): ");
	uc_preemphasis_len = 24;

	/* Asgin pointer to tx data buffer */
	p_data_buf = &xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0];
	#else
	printf("Please enter value for preemphasis (18 bytes default 000000000000000000000000000000000000): ");
	uc_preemphasis_len = 18;

	/* Asgin pointer to tx data buffer */
	p_data_buf = &xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis[0];
	#endif
	fflush(stdout);

	while (1) {
		for (uc_i = 0; uc_i < uc_preemphasis_len * 2 + 1; uc_i++) {
#if SAMG55
			while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
			}
			uc_char = (uint8_t)ul_char;
#else
			while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
			}
#endif

			if (uc_char == 0x0D) {
				printf("\r\n->Pre-Emphasis ready.\r\n");
				printf(MENU_CONSOLE);
				fflush(stdout);
				return;
			} else {
				printf("%c", uc_char);
				fflush(stdout);
				if (uc_char >= 0x30 && uc_char < 0x40) { /* 0 to 9 */
					if (uc_i % 2) {
						*p_data_buf++ += uc_char - 0x30;
					} else {
						*p_data_buf = (uc_char - 0x30) << 4;
					}
				} else if (uc_char >= 0x41 && uc_char < 0x47) {
					if (uc_i % 2) {
						*p_data_buf++ += uc_char - 0x37;
					} else {
						*p_data_buf = (uc_char - 0x37) << 4;
					}
				} else if (uc_char >= 0x61 && uc_char < 0x67) {
					if (uc_i % 2) {
						*p_data_buf++ += uc_char - 0x57;
					} else {
						*p_data_buf = (uc_char - 0x57) << 4;
					}
				} else {
					printf("\r\n->ERROR Invalid char \r\n");
					return;
				}
			}
		}
		printf("\r\n->End: Maximum Length is %d bytes\r\n", uc_preemphasis_len);
		fflush(stdout);
	}
}

/**
 * Get Period of transmission.
 */
static void get_transmission_period(void)
{
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif
	uint8_t ucv_period[10];
	uint8_t i, c;
	uint32_t ul_mul, ul_unit;

	printf("Enter transmission period in us. (max. 10 digits and value min 2300 us): ");
	fflush(stdout);
	while (1) {
		for (i = 0; i < 10; i++) {
#if SAMG55
			while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
			}
			uc_char = (uint8_t)ul_char;
#else
			while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
			}
#endif

			if (uc_char == 0x0D) {
				xAppPhyCfgTx.ul_tx_period = 0;
				for (c = i; c > 0; c--) {
					ul_mul = (uint32_t)(pow(10, (i - c)) / 1);
					ul_unit = ucv_period[c - 1];
					xAppPhyCfgTx.ul_tx_period += ul_unit * ul_mul;
				}
				printf("\r\n->Transmission period %lu us\r\n", (unsigned long)xAppPhyCfgTx.ul_tx_period);
				printf(MENU_CONSOLE);
				return;
			} else if ((uc_char >= '0') && (uc_char <= '9')) {
				printf("%c", uc_char);
				fflush(stdout);
				ucv_period[i] = (uc_char - 0x30);
			} else {
				printf("Error. Try again\r\n");
				break;
			}

			fflush(stdout);
		}
	}
}

/**
 * Get Length of data to transmit.
 */
static void get_data_len(void)
{
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif
	uint8_t ucv_len[4];
	uint8_t i, c, uc_unit;
	uint16_t us_mul;
	uint16_t us_max_pdu_len;

	us_max_pdu_len = get_max_psdu_len();

	printf("Enter length of data to transmit in bytes. (max. %d bytes): ", us_max_pdu_len);
	fflush(stdout);

	xAppPhyCfgTx.xPhyMsg.m_us_data_len = 0;
	while (1) {
		for (i = 0; i < 4; i++) {
#if SAMG55
			while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
			}
			uc_char = (uint8_t)ul_char;
#else
			while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
			}
#endif

			if (uc_char == 0x0D) {
				xAppPhyCfgTx.xPhyMsg.m_us_data_len = 0;
				for (c = i; c > 0; c--) {
					us_mul = (uint16_t)(pow(10, (i - c)) / 1);
					uc_unit = ucv_len[c - 1];
					xAppPhyCfgTx.xPhyMsg.m_us_data_len += uc_unit * us_mul;
				}

				if (xAppPhyCfgTx.xPhyMsg.m_us_data_len <= us_max_pdu_len) {
					printf("\r\n->Message Data length %lu bytes\r\n", (unsigned long)xAppPhyCfgTx.xPhyMsg.m_us_data_len);
					fflush(stdout);
					return;
				} else {
					printf("\r\n->Error. Invalid Length, try again:\r\n");
					fflush(stdout);
					break;
				}
			} else if ((uc_char >= '0') && (uc_char <= '9')) {
				printf("%c", uc_char);
				fflush(stdout);
				ucv_len[i] = (uc_char - 0x30);
			} else {
				printf("Error. Try again\r\n");
				break;
			}

			fflush(stdout);
		}
	}
}

/**
 * Fill data message in fixed mode.
 */
static void fill_msg_fixed(void)
{
	uint8_t uc_i;
	uint8_t *p_data_buf;
	uint16_t us_len;

	/* Assign pointer to tx data buffer */
	p_data_buf = ucv_tx_data_buffer;

	/* init vars */
	us_len = xAppPhyCfgTx.xPhyMsg.m_us_data_len;
	uc_i = 0;

	/* fill message */
	while (us_len--) {
		*p_data_buf++ = 0x30 + uc_i++;
		if (uc_i == 10) {
			uc_i = 0;
		}
	}

	/* store the content of message in flash memory */
	flash_unlock((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, \
			(uint32_t)ADDR_APP_PHY_MESSAGE_DATA + xAppPhyCfgTx.xPhyMsg.m_us_data_len, \
			0, 0);
	flash_erase_page((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, 2);
	flash_write((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, ucv_tx_data_buffer, \
			xAppPhyCfgTx.xPhyMsg.m_us_data_len, 0);

	printf("->Fixed message ready\r\n");
	printf(MENU_CONSOLE);
	fflush(stdout);
}

/**
 * Fill data message in manual mode (hex).
 */
static void fill_msg_manual_hex(void)
{
	uint16_t uc_i;
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif
	uint8_t *p_data_buf;
	uint16_t us_max_pdu_len;

	uint8_t uc_byte_content = 0x00;
	uint8_t uc_byte_upper_4_bits = 0x01; /* indicate if we are processing upper 4 bits of the byte */

	us_max_pdu_len = get_max_psdu_len();
	printf("Enter bytes data of message to transmit in hex format, i.e. 000102030405060708090A0B0C0D0E0F10... (max. %d bytes): ", us_max_pdu_len);
	fflush(stdout);

	/* Asgin pointer to tx data buffer */
	p_data_buf = ucv_tx_data_buffer;

	xAppPhyCfgTx.xPhyMsg.m_us_data_len = 0;
	while (1) {
		for (uc_i = 0; uc_i < us_max_pdu_len; uc_i++) {
#if SAMG55
			while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
			}
			uc_char = (uint8_t)ul_char;
#else
			while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
			}
#endif

			if (uc_char == 0x0D) {
				/* store the content of message in flash memory */
				flash_unlock((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, \
						(uint32_t)ADDR_APP_PHY_MESSAGE_DATA + xAppPhyCfgTx.xPhyMsg.m_us_data_len, \
						0, 0);
				flash_erase_page((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, 2);
				flash_write((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, ucv_tx_data_buffer, \
						xAppPhyCfgTx.xPhyMsg.m_us_data_len, 0);

				printf("\r\n->Manual hex message ready.\r\n");
				printf(MENU_CONSOLE);
				fflush(stdout);
				return;
			} else {
				printf("%c", uc_char);
				fflush(stdout);
				switch (uc_char) {
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
					if (uc_byte_upper_4_bits) {
						uc_byte_content += (uc_char - 0x30) * 16;
						uc_byte_upper_4_bits = 0x00;
					} else {
						uc_byte_content += (uc_char - 0x30);
						*p_data_buf++ = uc_byte_content;
						xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
						uc_byte_upper_4_bits = 0x01;
						uc_byte_content = 0x00;
					}

					break;

				case 'a':
				case 'A':
					if (uc_byte_upper_4_bits) {
						uc_byte_content += 10 * 16;
						uc_byte_upper_4_bits = 0x00;
					} else {
						uc_byte_content += 10;
						*p_data_buf++ = uc_byte_content;
						xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
						uc_byte_upper_4_bits = 0x01;
						uc_byte_content = 0x00;
					}

					break;

				case 'b':
				case 'B':
					if (uc_byte_upper_4_bits) {
						uc_byte_content += 11 * 16;
						uc_byte_upper_4_bits = 0x00;
					} else {
						uc_byte_content += 11;
						*p_data_buf++ = uc_byte_content;
						xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
						uc_byte_upper_4_bits = 0x01;
						uc_byte_content = 0x00;
					}

					break;

				case 'c':
				case 'C':
					if (uc_byte_upper_4_bits) {
						uc_byte_content += 12 * 16;
						uc_byte_upper_4_bits = 0x00;
					} else {
						uc_byte_content += 12;
						*p_data_buf++ = uc_byte_content;
						xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
						uc_byte_upper_4_bits = 0x01;
						uc_byte_content = 0x00;
					}

					break;

				case 'd':
				case 'D':
					if (uc_byte_upper_4_bits) {
						uc_byte_content += 13 * 16;
						uc_byte_upper_4_bits = 0x00;
					} else {
						uc_byte_content += 13;
						*p_data_buf++ = uc_byte_content;
						xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
						uc_byte_upper_4_bits = 0x01;
						uc_byte_content = 0x00;
					}

					break;

				case 'e':
				case 'E':
					if (uc_byte_upper_4_bits) {
						uc_byte_content += 14 * 16;
						uc_byte_upper_4_bits = 0x00;
					} else {
						uc_byte_content += 14;
						*p_data_buf++ = uc_byte_content;
						xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
						uc_byte_upper_4_bits = 0x01;
						uc_byte_content = 0x00;
					}

					break;

				case 'f':
				case 'F':
					if (uc_byte_upper_4_bits) {
						uc_byte_content += 15 * 16;
						uc_byte_upper_4_bits = 0x00;
					} else {
						uc_byte_content += 15;
						*p_data_buf++ = uc_byte_content;
						xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
						uc_byte_upper_4_bits = 0x01;
						uc_byte_content = 0x00;
					}

					break;

				default:
					break;
				}
				fflush(stdout);
			}
		}
		printf("\r\n->End: Maximum Length is %d bytes\r\n", us_max_pdu_len);
		fflush(stdout);
	}
}

/**
 * Fill data message in manual mode.
 */
static void fill_msg_manual(void)
{
	uint16_t uc_i;
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif
	uint8_t *p_data_buf;
	uint16_t us_max_pdu_len;

	us_max_pdu_len = get_max_psdu_len();
	printf("Enter data message to transmit (max. %d bytes): ", us_max_pdu_len);
	fflush(stdout);

	/* Asgin pointer to tx data buffer */
	p_data_buf = ucv_tx_data_buffer;

	xAppPhyCfgTx.xPhyMsg.m_us_data_len = 0;
	while (1) {
		for (uc_i = 0; uc_i < us_max_pdu_len; uc_i++) {
#if SAMG55
			while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
			}
			uc_char = (uint8_t)ul_char;
#else
			while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
			}
#endif

			if (uc_char == 0x0D) {
				/* set header type to generic message */
				xAppPhyCfgTx.xPhyMsg.m_puc_data_buf[0] = 0;

				/* store the content of message in flash memory */
				flash_unlock((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, \
						(uint32_t)ADDR_APP_PHY_MESSAGE_DATA + xAppPhyCfgTx.xPhyMsg.m_us_data_len, \
						0, 0);
				flash_erase_page((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, 2);
				flash_write((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, ucv_tx_data_buffer, \
						xAppPhyCfgTx.xPhyMsg.m_us_data_len, 0);

				printf("\r\n->Manual message ready.\r\n");
				printf(MENU_CONSOLE);
				fflush(stdout);
				return;
			} else {
				printf("%c", uc_char);
				fflush(stdout);
				*p_data_buf++ = uc_char;
				xAppPhyCfgTx.xPhyMsg.m_us_data_len++;
				fflush(stdout);
			}
		}
		printf("\r\n->End: Maximum Length is %d bytes\r\n", us_max_pdu_len);
		fflush(stdout);
	}
}

/**
 * Get Transmission Data.
 */
static void get_transmission_data(void)
{
#if SAMG55
	uint32_t ul_char;
	uint8_t uc_char;
#else
	uint8_t uc_char;
#endif

	puts(MENU_DATA_MODE);
	fflush(stdout);

	while (1) {
#if SAMG55
		while (uart_read(CONSOLE_UART, (uint32_t *)&ul_char)) {
		}
		uc_char = (uint8_t)ul_char;
#else
		while (uart_read(CONSOLE_UART, (uint8_t *)&uc_char)) {
		}
#endif
		switch (uc_char) {
		case '0':
			printf("%c\r\n", uc_char);
			get_data_len();
			xAppPhyCfgTx.uc_is_random = 1;
			fill_msg_random();
			break;

		case '1':
			printf("%c\r\n", uc_char);
			xAppPhyCfgTx.uc_is_random = 0;
			get_data_len();
			fill_msg_fixed();
			break;

		case '2':
			printf("%c\r\n", uc_char);
			xAppPhyCfgTx.uc_is_random = 0;
			fill_msg_manual();
			break;

		case '3':
			printf("%c\r\n", uc_char);
			xAppPhyCfgTx.uc_is_random = 0;
			fill_msg_manual_hex();
			break;

		default:
			continue;
		}
		break;
	}
}

static void phy_tx_test_console_init(void)
{
	uint8_t uc_start_mode;

	platform_led_cfg_blink_rate(OSS_LED_BLINK_RATE / 10);

	/* Init Phy Layer */
#if defined (CONF_BAND_CENELEC_A)
	phy_init(false, WB_CENELEC_A);
#elif defined (CONF_BAND_FCC)
	phy_init(false, WB_FCC);
#elif defined (CONF_BAND_ARIB)
	phy_init(false, WB_ARIB);
#else /* Default */
	phy_init(false, WB_CENELEC_A);
#endif
	/* Init application */
	app_init();

	/* Configuration management */
	xAppPhyCfgTx.xPhyMsg.m_puc_data_buf = ucv_tx_data_buffer;

	uc_start_mode = load_config();
	if (uc_start_mode == PHY_APP_CMD_DEFAULT_MODE) {
		xAppPhyCfgTx.xPhyMsg.e_delimiter_type = DT_SOF_NO_RESP;
		xAppPhyCfgTx.xPhyMsg.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
		xAppPhyCfgTx.xPhyMsg.e_mod_type = MOD_TYPE_BPSK_ROBO;
		#if defined(CONF_BAND_CENELEC_A)
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map[0] = 0x3F;
		uc_local_num_subbands = NUM_SUBBANDS_CENELEC_A;
		#elif defined(CONF_BAND_FCC)
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map [2] = 0xff;
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map [1] = 0xff;
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map [0] = 0xff;
		xAppPhyCfgTx.xPhyMsg.m_uc_2_rs_blocks = 1;
		uc_local_num_subbands = NUM_SUBBANDS_FCC;
		#else
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map [0] = 0x03;
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map [1] = 0xff;
		xAppPhyCfgTx.xPhyMsg.m_auc_tone_map [2] = 0xff;
		xAppPhyCfgTx.xPhyMsg.m_uc_2_rs_blocks = 0;
		uc_local_num_subbands = NUM_SUBBANDS_ARIB;
	#endif

		xAppPhyCfgTx.xPhyMsg.m_uc_buff_id = 0;
		xAppPhyCfgTx.xPhyMsg.m_uc_pdc = 0;
		memset(xAppPhyCfgTx.xPhyMsg.m_auc_preemphasis, 0x00, uc_local_num_subbands);
		xAppPhyCfgTx.xPhyMsg.m_uc_tx_mode = TX_MODE_FORCED_TX | TX_MODE_DELAYED_TX;
		xAppPhyCfgTx.xPhyMsg.m_us_data_len = 100;
		xAppPhyCfgTx.xPhyMsg.m_ul_tx_time = 5400;

		xAppPhyCfgTx.ul_tx_period = 5400;
		xAppPhyCfgTx.uc_is_random = 1;

		xAppPhyCfgTx.uc_force_no_output = 0;

		/* Fill Data of message: Fixed by default */
		fill_msg_fixed();
		machine.curState = main_menu_show;
	} else if (uc_start_mode == PHY_APP_CMD_TX_START_MODE) {
		/* execute test */
		printf("Press 'x' to finish transmission...\r\n");
		machine.curState = executing;
	}

	xAppPhyCfgTx.uc_tx_result_flag = PHY_TX_RESULT_SUCCESS;

	/* Set value to an unused register to check periodically that ATPL250 did not reset */
	pplc_if_write32(REG_ATPL250_PLC_TIME_ON4_32, 0xA5A5A5A5);
}

static uint16_t init_counter = 0;

static void phy_tx_test_console_process(void)
{
	EIndication input;
	uint32_t reg_value;

	if (init_counter < 2000) {
		init_counter++;
	} else if (init_counter == 2000) {
		init_counter++;
		platform_led_cfg_blink_rate(OSS_LED_BLINK_RATE);
	} else {
		/* Check register preservation */
		reg_value = pplc_if_read32(REG_ATPL250_PLC_TIME_ON4_32);
		if (reg_value != 0xA5A5A5A5) {
			while (1) {
			}
		}
	}

	input = processInput();
	(machine.curState)(&machine, input);
	/* phy process */
	phy_process();
}

/**
 * \brief Main code entry point.0
 */
int main( void )
{
	oss_task_t x_task = {0};

	/* Initialize OSS */
	oss_init();

	puts(STRING_HEADER);

	/* Register PHY Task */
	x_task.task_init = phy_tx_test_console_init;
	x_task.task_process = phy_tx_test_console_process;
	x_task.task_1ms_timer_cb = NULL;
	oss_register_task(&x_task);

	/* Start OSS */
	oss_start();
}

void main_menu_show(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	/* Console Application menu */
	puts(MENU_HEADER);
	printf(MENU_CONSOLE);
	fflush(stdout);
	sm->curState = main_menu_wait;
}

void main_menu_wait(TStateMachine *sm, EIndication input)
{
	switch (input) {
	case MAIN_MENU_OPT0:
		sm->curState = opt0_menu;
		break;

	case MAIN_MENU_OPT1:
		sm->curState = opt1_menu;
		break;

	case MAIN_MENU_OPT2:
		sm->curState = opt2_menu;
		break;

	case MAIN_MENU_OPT3:
		sm->curState = opt3_menu;
		break;

	case MAIN_MENU_OPT4:
		sm->curState = opt4_menu;
		break;

	case MAIN_MENU_OPT5:
		sm->curState = opt5_menu;
		break;

	case MAIN_MENU_OPT6:
		sm->curState = opt6_menu;
		break;

	case MAIN_MENU_OPT7:
		sm->curState = opt7_menu;
		break;

	case MAIN_MENU_VIEW_CFG:
		sm->curState = show_cfg;
		break;

	case MAIN_MENU_EXECUTE:
		/* save configuration parameters */
		save_config(PHY_APP_CMD_TX_START_MODE);
		printf("Press 'x' to finish transmission...\r\n");
		sm->curState = executing;
		break;

	case UNKNOW_CHAR:
		sm->curState = main_menu_show;
		break;

	default:        /* DO NOTHING */
		break;
	}
}

void opt0_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_transmission_level();
	sm->curState = main_menu_show;
}

void opt1_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_transmission_scheme();
	sm->curState = main_menu_show;
}

void opt2_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_transmission_period();
	sm->curState = main_menu_show;
}

void opt3_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_transmission_data();
	sm->curState = main_menu_show;
}

void opt5_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_preemphasis();
	sm->curState = main_menu_show;
}

void opt6_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_branch_mode();
	sm->curState = main_menu_show;
}

void opt7_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_forced_output();
	sm->curState = main_menu_show;
}

void opt4_menu(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	get_tone_map();
	sm->curState = main_menu_show;
}

void show_cfg(TStateMachine *sm, EIndication input)
{
	UNUSED(input);
	display_config();
	sm->curState = main_menu_show;
}

void executing(TStateMachine *sm, EIndication input)
{
	uint32_t ul_gpbr_value;

	if (!enabled_tx()) {
		run_tx_task();
		if (xAppPhyCfgTx.uc_is_random) {
			fill_msg_random();
		}
	}

	/* fflush(stdout); */
	if (input == STOP_TEST) {
		sm->curState = main_menu_show;
		ul_gpbr_value = gpbr_read(GPBR0);
		ul_gpbr_value &= 0xFFFFFFF0;
		ul_gpbr_value |= PHY_APP_CMD_MENU_START_MODE;
		gpbr_write(GPBR0, ul_gpbr_value);
		xAppPhyCfgTx.uc_tx_result_flag = PHY_TX_RESULT_SUCCESS;
		disable_tx();
	} else {
		enable_tx();
	}

	/* else maintain state */
}

EIndication processInput(void)
{
#if SAMG55
	uint32_t uc_choice;
#else
	uint8_t uc_choice;
#endif
	EIndication e_result = NO_CHAR;

#if SAMG55
	if (!uart_read(CONSOLE_UART, (uint32_t *)&uc_choice)) {
#else
	if (!uart_read(CONSOLE_UART, (uint8_t *)&uc_choice)) {
#endif
		switch (uc_choice) {
		case '0':
			e_result = MAIN_MENU_OPT0;
			break;

		case '1':
			e_result = MAIN_MENU_OPT1;
			break;

		case '2':
			e_result = MAIN_MENU_OPT2;
			break;

		case '3':
			e_result = MAIN_MENU_OPT3;
			break;

		case '4':
			e_result = MAIN_MENU_OPT4;
			break;

		case '5':
			e_result = MAIN_MENU_OPT5;
			break;

		case '6':
			e_result = MAIN_MENU_OPT6;
			break;

		case '7':
			e_result = MAIN_MENU_OPT7;
			break;

		case 'v':
		case 'V':
			e_result = MAIN_MENU_VIEW_CFG;
			break;

		case 'e':
		case 'E':
			e_result = MAIN_MENU_EXECUTE;
			break;

		case 'x':
		case 'X':
			e_result = STOP_TEST;
			break;

		case 0x00:
			e_result = NO_CHAR;
			break;

		default:
			e_result = UNKNOW_CHAR;
			break;
		}
	}

	return e_result;
}
