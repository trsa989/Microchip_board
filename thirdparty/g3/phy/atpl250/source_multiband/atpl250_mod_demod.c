/**
 * \file
 *
 * \brief ATPL250 Modulator / Demodulator Access
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

/* System includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Phy layer includes */
#include "atpl250.h"
#include "atpl250_mod_demod.h"
#include "atpl250_reg.h"
#include "atpl250_jump_ram.h"
#include "atpl250_carrier_mapping.h"
#include "atpl250_common.h"
#include "atpl250_channel_estimation.h"
#include "atpl250_sampling_error_estimation.h"

#ifdef DEBUG_CHN_SFO
extern uint8_t uc_num_block_pilots;
#endif

/* Extern variables needed */
extern struct band_phy_constants s_band_constants;
extern uint8_t uc_working_band;
extern uint8_t uc_used_carriers;
extern uint8_t uc_psymbol_len;
extern uint8_t uc_legacy_mode;
extern uint8_t auc_psymbol[];
extern uint8_t uc_index_inob_abs;
extern uint8_t auc_unmasked_carrier_list[];

/* Maximum size to read from interleaver */
#define MAX_PAY_INTLV_READ_SIZE         (MAX_NUM_SYM_MOD * NUM_CARRIERS_FCC * 4 + 7) >> 3
/* Maximum size of PAYLOAD_MODULATION() output */
#define MAX_PAY_MOD_OUTPUT_SIZE         (MAX_NUM_SYM_MOD * NUM_CARRIERS_FCC + 1) >> 1

/* Array to read data from interleaver to modulate */
uint8_t auc_interleaver_matrix_tx[MAX_PAY_INTLV_READ_SIZE];
/* Array for the output of PAYLOAD_MODULATION() */
uint8_t auc_modulator_out_payload[MAX_PAY_MOD_OUTPUT_SIZE];

/* Static variable to copy function parameter and thus send ptr to pplc_if safely */
static struct sym_cfg s_sym_cfg_aux;

/* Declaration of assembler functions */
extern void PAYLOAD_MODULATION(void);
extern void DEMOD_GET_DATA_CARRIERS(uint8_t *puc_demodulator_output, uint8_t *puc_demodulator_output_data_carriers, uint8_t *puc_state_carrier, uint8_t uc_protocol_carriers);

/* Address to read interleaver */
extern uint32_t ul_interleaver_read_pointer;
/* Address to write deinterleaver */
extern uint32_t ul_deinterleaver_write_pointer;

/* Array where output of REMOVE_PILOTS() is stored
 * It contains output of demodulator corresponding to data carriers */
uint8_t auc_demod_data_carriers[MAX_DEMOD_DATA_LEN];

/* Global variables used by assembler functions */
extern uint8_t auc_state_carrier_asm[];
uint32_t ul_mod_input_pointer_asm;
uint32_t ul_mod_output_pointer_asm;
uint8_t uc_protocol_carriers_asm;
const uint8_t auc_lsfr_asm[127] = {0, 0, 224, 152, 177, 81, 201, 152, 177, 177, 81, 41, 224, 120, 201, 152, 81, 41, 224, 120,
				   41, 224, 120, 41, 0, 0, 0, 0, 224, 120, 41, 0, 224, 120, 41, 224, 152, 81, 41, 0, 224, 120, 201, 152, 177, 81, 201, 120, 201,
				   152,
				   81, 201, 152, 81, 41, 0, 0, 0, 224, 152, 81, 41, 224, 152, 81, 201, 120, 201, 120, 41, 224, 152, 177, 81, 41, 224, 152, 177,
				   177,
				   81, 201, 152, 81, 201, 120, 41, 0, 0, 224, 120, 201, 120, 201, 120, 201, 152, 177, 177, 177, 81, 201, 120, 41, 224, 120, 201,
				   120,
				   41, 0, 224, 152, 81, 201, 152, 177, 81, 41, 0, 224, 152, 177, 177, 177, 177, 177, 81, 41};
uint32_t ul_mod_vector_aux_asm;

/* Variables used to store pilots position in order to write them for debugging */
#ifdef PRINT_PILOTS_DBG
extern uint8_t auc_pilot_pos_tmp[][CARR_BUFFER_LEN];
extern uint8_t auc_pilot_pos_tx_tmp[][NUM_CARRIERS_FCC];
uint8_t auc_pilots_aux[NUM_CARRIERS_FCC];
extern uint16_t us_symbol_counter_tmp;
#endif

/**
 * \brief Write P symbol phases to HW, for coherent demodulation
 *
 * \param puc_psymbol  Pointer to P symbol reference
 *
 */
void set_p_symbol_phases(const uint8_t *puc_psymbol, uint8_t uc_reference_to_load)
{
	uint8_t auc_zeros[P_SYMBOL_LEN_MAX_FCC];

	memset(auc_zeros, 0, P_SYMBOL_LEN_MAX_FCC);

	/* Send P symbol angles to Zone2 for later coherent demodulation */
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_H8, 0x20); /* H_1H_BYPASS='1'; H_1H='0'; DEST_Y='0'; SOURCE_H='0' */
	atpl250_set_mod_bpsk_truepoint();
	pplc_if_write16(REG_ATPL250_INOUTB_CONF3_L16, 0x8000); /* Set alpha = 1 beta = 0 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier); /* Avoid overflow to last carrier */

	switch (uc_reference_to_load) {
	case COH_REF_SYMBOL_BPSK:
		/* Reference for BPSK */
		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_BPSK);         /* Set destination to proper symbol */
		pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xC0);         /* Set MOD_PHASE bit and MOD_OFFSET = 0 to store phase instead of real/imag (BPSK) */
		pplc_if_write_jump((BCODE_ICHANNEL | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_2);
		break;

	case COH_REF_SYMBOL_QPSK:
		/* Reference for QPSK */
		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_QPSK);         /* Set destination to proper symbol */
		pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xCE);         /* Set MOD_PHASE bit and MOD_OFFSET = 14 to store phase instead of real/imag (QPSK) */
		pplc_if_write_jump((BCODE_ICHANNEL | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_2);
		break;

	case COH_REF_SYMBOL_8PSK:
		/* Reference for 8PSK */
		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_8PSK);         /* Set destination to proper symbol */
		pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xCF);         /* Set MOD_PHASE bit and MOD_OFFSET = 15 to store phase instead of real/imag (8PSK) */
		pplc_if_write_jump((BCODE_ICHANNEL | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_2);
		break;

	default:
		/* Reference for BPSK */
		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_BPSK);         /* Set destination to proper symbol */
		pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xC0);         /* Set MOD_PHASE bit and MOD_OFFSET = 0 to store phase instead of real/imag (BPSK) */
		pplc_if_write_jump((BCODE_ICHANNEL | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_2);
		/* Reference for QPSK */
		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_QPSK);         /* Set destination to proper symbol */
		pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xCE);         /* Set MOD_PHASE bit and MOD_OFFSET = 14 to store phase instead of real/imag (QPSK) */
		pplc_if_write_jump((BCODE_ICHANNEL | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_2);
		/* Reference for 8PSK */
		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_8PSK);         /* Set destination to proper symbol */
		pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xCF);         /* Set MOD_PHASE bit and MOD_OFFSET = 15 to store phase instead of real/imag (8PSK) */
		pplc_if_write_jump((BCODE_ICHANNEL | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_2);
	}

	/* Restore values */
	pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0x00); /* Clear MOD_PHASE bit and MOD_OFFSET = 0 */
}

/**
 * \brief Sets pilot tone position for first Tx/Rx payload symbol
 *
 * \param uc_num_pilots               Number of pilots to set
 * \param uc_num_active_carriers      Number of active carriers
 * \param puc_pilot_reg_values        Pointer to array where HW registers config will be stored
 * \param puc_pilot_pos               Pointer to array where position will be stored
 * \param puc_inactive_carriers_pos   Pointer to array containing inactive carriers
 * \param puc_static_notching_pos     Pointer to array containing static notching
 *
 */
void set_pilot_position(uint8_t uc_num_pilots, uint8_t uc_num_active_carriers, uint8_t *puc_pilot_reg_values, uint8_t *puc_pilot_pos,
		uint8_t *puc_inactive_carriers_pos, uint8_t *puc_static_notching_pos)
{
	uint8_t uc_i, uc_j, uc_temp;
	uint8_t auc_pilots_rel_pos[13] = {0}; /* Added by JAC */
	uint8_t auc_pilots_unordered_pos[13] = {0}; /* Added by JAC */
	uint8_t uc_byte_to_start = s_band_constants.uc_first_carrier >> 3;
	uint8_t uc_bit_to_start = s_band_constants.uc_first_carrier & 0x07;
	uint8_t uc_byte_to_stop = s_band_constants.uc_last_carrier >> 3;
	uint8_t uc_bit_to_stop = s_band_constants.uc_last_carrier & 0x07;
	uint8_t uc_pilot_idx = 0;
	uint8_t uc_active_carrier_cnt = 0;
	uint8_t uc_current_carrier;
	uint8_t uc_lower_pilot_idx = 0;

	/* Clear buffers */
	memset(auc_pilots_rel_pos, -1, uc_num_pilots * sizeof(uint8_t));
	memset(puc_pilot_pos, 0, CARR_BUFFER_LEN * sizeof(uint8_t));
	memset(puc_pilot_reg_values, 0, 28 * sizeof(uint8_t));

	for (uc_i = 0; uc_i < uc_num_pilots; uc_i++) {
		auc_pilots_rel_pos[uc_i] = (s_band_constants.uc_pilot_offset + (PILOT_FREQ_SPA * uc_i)) % uc_num_active_carriers;
	}

	while (uc_pilot_idx < uc_num_pilots) { /* there are pilot missed at the beginning */
		uc_active_carrier_cnt = 0;
		uc_current_carrier = s_band_constants.uc_first_carrier; /* its going to be increased at the end of the loop */

		uc_byte_to_start = s_band_constants.uc_first_carrier >> 3;
		uc_bit_to_start = s_band_constants.uc_first_carrier & 0x07;

		for (uc_i = uc_byte_to_start; uc_i <= uc_byte_to_stop; uc_i++) {
			uc_temp = puc_inactive_carriers_pos[uc_i] | puc_static_notching_pos[uc_i];

			if (uc_i == uc_byte_to_stop) {
				uc_bit_to_stop  = (s_band_constants.uc_last_carrier & 0x07) + 1;
			} else {
				uc_bit_to_stop = 8;
			}

			if (uc_i == uc_byte_to_start) {
				uc_bit_to_start = s_band_constants.uc_first_carrier & 0x07;
			} else {
				uc_bit_to_start = 0;
			}

			for (uc_j = uc_bit_to_start; uc_j < uc_bit_to_stop; uc_j++) {
				if (!(uc_temp & (1 << uc_j))) {
					if (uc_active_carrier_cnt == auc_pilots_rel_pos[uc_pilot_idx]) {
						puc_pilot_pos[uc_i] |= (1 << uc_j); /* Mark pilot position */
						auc_pilots_unordered_pos[uc_pilot_idx] = uc_current_carrier;
						if (uc_current_carrier < auc_pilots_unordered_pos[uc_lower_pilot_idx]) {
							uc_lower_pilot_idx = uc_pilot_idx;
						}

						/* tack pilot values */
						uc_pilot_idx++;
						if (uc_pilot_idx >= uc_num_pilots) {
							break;
						}
					}

					uc_active_carrier_cnt++;
				}

				uc_current_carrier++;
			}
			if (uc_pilot_idx >= uc_num_pilots) {
				break;
			}
		}
	}

	uc_j = 0;
	for (uc_i = uc_lower_pilot_idx; uc_i < uc_num_pilots; uc_i++) {
		puc_pilot_reg_values[2 * uc_j + 1] = auc_pilots_unordered_pos[uc_i]; /* Save absolute carrier position. */
		uc_j++;
	}

	if (uc_lower_pilot_idx != 0) {
		for (uc_i = 0; uc_i < uc_lower_pilot_idx; uc_i++) {
			puc_pilot_reg_values[2 * uc_j + 1] = auc_pilots_unordered_pos[uc_i]; /* Save absolute carrier position. */
			uc_j++;
		}
	}

	puc_pilot_reg_values[26] = uc_num_pilots | 0x30;    /* Number of pilots per symbol | CONSECUTIVE_PILOTS = '1' SORT_PILOTS = '1' */
	puc_pilot_reg_values[27] = 0x01;                    /* Write */
}

/**
 * \brief Configures modulator and writes next FCH symbols
 *
 * \param uc_num_symbols      Number of symbols to modulate
 * \param uc_offset_symbols   Symbol to start write to
 * \param uc_change_mod       Flag to indicate if modulation parameters have to be changed
 *
 */
void feed_modulator_fch(uint8_t uc_num_symbols, uint8_t uc_offset_symbols, uint8_t uc_change_mod)
{
	uint16_t us_dat_len;
	uint16_t us_num;

	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(uc_num_symbols);
	/* Avoid overflow */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols + uc_offset_symbols - 1)) + s_band_constants.uc_last_used_carrier);

	/* Change modulation if needed */
	if (uc_change_mod) {
		atpl250_set_mod_bpsk();
	}

	/* Data length = Carriers * Bits per carrier * num symbols / 8 bits in byte */
	us_num = uc_used_carriers * uc_num_symbols; /* Bits per carrier is 1 always for FCH */
	us_dat_len = (us_num >> 3);
	if (us_num & 0x0007) {
		us_dat_len++;
	}

	/* Write symbols */
	pplc_if_write_jump((BCODE_DIFT | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0] + (uc_offset_symbols * CFG_IOB_SAMPLES_PER_SYMBOL))),
			u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_2);
}

/**
 * \brief Configures modulator and writes next Payload symbols
 *
 * \param p_tx_ctl            Pointer to Tx control structure
 * \param uc_num_symbols      Number of symbols to modulate
 * \param uc_change_sym_cfg   Bit coded value indicating which symbols change configuration
 * \param p_sym_cfg           Pointer to structure containing new symbol config
 * \param uc_change_mod       Flag to indicate if modulation parameters have to be changed
 *
 */
void feed_modulator_payload(struct phy_tx_ctl *p_tx_ctl, uint8_t uc_num_symbols, uint8_t uc_change_sym_cfg, struct sym_cfg *p_sym_cfg)
{
	uint16_t us_dat_len;
	uint8_t uc_i;
	uint32_t ul_interleaver_read_len_bits;
	uint32_t ul_interleaver_read_len;
	#ifdef PRINT_PILOTS_DBG
	uint8_t uc_j;
	#endif

	/* Compute length of interleaver output data to read */
	ul_interleaver_read_len_bits = uc_num_symbols * p_tx_ctl->m_uc_payload_carriers * ((((uint8_t)p_tx_ctl->e_mod_type) & 0x03) + 1);
	ul_interleaver_read_len = (ul_interleaver_read_len_bits + 7) >> 3;

	/* Disable HW chain */
	disable_HW_chain();
	/* Read data from interleaver */
	pplc_if_write16(REG_ATPL250_INTERLEAVER_SPI_H16, 0x8000 + ul_interleaver_read_pointer); /* SPI2INT_TXRX='1'; ADDR_SPI2INT=0 */
	pplc_if_read_jump(REG_ATPL250_INTERLEAVER_SPI_VL8, auc_interleaver_matrix_tx, ul_interleaver_read_len, 1, true);
	/* Enable HW chain */
	enable_HW_chain();

	ul_mod_input_pointer_asm = (uint32_t)auc_interleaver_matrix_tx;
	ul_mod_output_pointer_asm = (uint32_t)auc_modulator_out_payload;

	/* Update protocol carriers for assembler functions */
	uc_protocol_carriers_asm = s_band_constants.uc_num_carriers - 1;

	for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
		#ifdef PRINT_PILOTS_DBG
		/* Copy pilot position for debuging */
		for (uc_j = 0; uc_j < s_band_constants.uc_num_carriers; uc_j++) {
			auc_pilots_aux[uc_j] = (auc_state_carrier_asm[s_band_constants.uc_num_carriers - uc_j - 1] >> 5) & 0x01;
		}
		memcpy(&auc_pilot_pos_tx_tmp[us_symbol_counter_tmp][0], auc_pilots_aux, s_band_constants.uc_num_carriers);
		us_symbol_counter_tmp++;
		#endif
		/* Modulate symbol */
		PAYLOAD_MODULATION();
	}

	ul_interleaver_read_pointer +=  ul_interleaver_read_len_bits;

	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(uc_num_symbols);

	/* Check symbol config changes */
	if (uc_change_sym_cfg) {
		/* Copy parameter to static variable */
		memcpy(&s_sym_cfg_aux, p_sym_cfg, sizeof(struct sym_cfg));
		if (uc_change_sym_cfg & 0x01) {
			s_sym_cfg_aux.m_uc_sym_idx = 0;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x02) {
			s_sym_cfg_aux.m_uc_sym_idx = 1;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x04) {
			s_sym_cfg_aux.m_uc_sym_idx = 2;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x08) {
			s_sym_cfg_aux.m_uc_sym_idx = 3;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x10) {
			s_sym_cfg_aux.m_uc_sym_idx = 4;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x20) {
			s_sym_cfg_aux.m_uc_sym_idx = 5;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x40) {
			s_sym_cfg_aux.m_uc_sym_idx = 6;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x80) {
			s_sym_cfg_aux.m_uc_sym_idx = 7;
			s_sym_cfg_aux.m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg_aux), sizeof(struct sym_cfg));
		}
	}

	us_dat_len = (uc_num_symbols * s_band_constants.uc_num_carriers) >> 1;

	disable_HW_chain();
	atpl250_set_mod_bpsk_truepoint();

	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + s_band_constants.uc_last_carrier);
	pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier)), &auc_modulator_out_payload[0], us_dat_len, JUMP_COL_3);

	/* enable_HW_chain(); */
}

/**
 * \brief Reads symbols from demodulator for FCH
 *
 * \param uc_num_symbols   Number of symbols to read
 * \param uc_change_mod   Flag to indicate if modulation parameters have to be changed
 * \param uc_num_symbols_next   Number of symbols to wait for next interrupt
 *
 */
void get_demodulator_fch(uint8_t uc_num_symbols, uint8_t uc_change_mod, uint8_t uc_num_symbols_next)
{
	uint16_t us_dat_len;
	uint16_t us_num;
	uint16_t us_bcode;

	/*LOG_PHY(("%u %u\r\n", uc_num_symbols, uc_num_symbols_next));*/

	/* Change modulation if necessary */
	if (uc_change_mod) {
		atpl250_set_mod_bpsk();
	}

	/* Data length = Carriers * Bits per carrier * num symbols / 2 bits in byte (bits soft are coded in 4-bit) */
	us_num = uc_used_carriers * uc_num_symbols; /* Bits per carrier always 1 for FCH */
	us_dat_len = ((us_num + 1) >> 1);

	/* Avoid overflow to n-th symbol when reading */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[uc_used_carriers - 1]);

	if (uc_working_band == WB_FCC) {
		if (uc_legacy_mode) {
			pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_BPSK); /* Set source to symbol 0 (channel) destination to proper symbol */
			us_bcode = BCODE_COH;
		} else {
			us_bcode = BCODE_DIFT;
		}
	} else {
		us_bcode = BCODE_DIFT;
	}

	/* Demodulate symbols using column 2 of jump ram (static notching). Not needed to read data (HW chain enabled) */
	pplc_if_read_jump((us_bcode | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
			u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_2, false);

	/* Set symbols for next interrupt */
	atpl250_set_num_symbols_cfg(uc_num_symbols_next);
}

static void _demodulate_symbols(struct phy_rx_ctl *p_rx_ctl, uint8_t uc_num_symbols, uint8_t uc_bits_per_carrier, uint16_t us_bcode, uint8_t uc_first_sym_index)
{
	uint16_t us_num, us_dat_len, us_deinterleaver_word_len, us_deinterleaver_byte_len;
	uint8_t uc_sym_indx;

	/* Number of bits = Carriers (including pilots, dynamic notching and static notching) * Bits per carrier (read and write symbol by symbol) */
	us_num = uc_used_carriers * uc_bits_per_carrier;
	/* Data length = Number of bits / 2 bits in byte (bits soft are coded in 4-bit) */
	us_dat_len = (us_num + 1) >> 1;

	/* Avoid overflow to first symbol when reading */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * uc_first_sym_index) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[uc_used_carriers - 1]);

	/* Read (non-blocking) first symbol */
	pplc_if_read_jump((us_bcode | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0] + (CFG_IOB_SAMPLES_PER_SYMBOL * uc_first_sym_index))),
			u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_2, false);

	/* Number of bits (write) = Data carriers (without pilots, dynamic notching and static notching) * Bits per carrier */
	us_deinterleaver_word_len = p_rx_ctl->m_uc_payload_carriers * uc_bits_per_carrier;
	us_deinterleaver_byte_len = (us_deinterleaver_word_len + 1) >> 1;

	/* Get data of first symbol */
	pplc_if_do_read(u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len);

	for (uc_sym_indx = uc_first_sym_index + 1; uc_sym_indx < (uc_first_sym_index + uc_num_symbols); uc_sym_indx++) {
		/* Avoid overflow to n-th symbol when reading */
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * uc_sym_indx) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[uc_used_carriers - 1]);
		/* Read (non-blocking) n-th symbol */
		pplc_if_read_jump((us_bcode | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0] + (CFG_IOB_SAMPLES_PER_SYMBOL * uc_sym_indx))),
				u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_2, false);

		/* Remove bits corresponding to static notching, dynamic notching and pilots in (n-1)-th symbol. This is doing in parallel with SPI transaction */
		DEMOD_GET_DATA_CARRIERS(u_shared_buffers.s_mod_demod.auc_mod_demod_data, auc_demod_data_carriers, auc_state_carrier_asm, s_band_constants.uc_num_carriers);

		/* Get data of n-th symbol */
		pplc_if_do_read(u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len);

		/* Write data without pilots corresponding to (n-1)-th symbol to deinterleaver */
		pplc_if_write16(REG_ATPL250_INTERLEAVER_SPI_H16, 0x0000 + ul_deinterleaver_write_pointer); /* SPI2INT_TXRX='1'; ADDR_SPI2INT=0 */
		pplc_if_write_jump(REG_ATPL250_INTERLEAVER_SPI_VL8, auc_demod_data_carriers, us_deinterleaver_byte_len, 1);
		/* Update deinterleaver address */
		ul_deinterleaver_write_pointer += us_deinterleaver_word_len;
	}

	/* Remove bits corresponding to pilots in last symbol. This is doing in parallel with SPI transaction */
	DEMOD_GET_DATA_CARRIERS(u_shared_buffers.s_mod_demod.auc_mod_demod_data, auc_demod_data_carriers, auc_state_carrier_asm, s_band_constants.uc_num_carriers);

	/* Write data without pilots corresponding to last symbol to deinterleaver */
	pplc_if_write16(REG_ATPL250_INTERLEAVER_SPI_H16, 0x0000 + ul_deinterleaver_write_pointer); /* SPI2INT_TXRX='1'; ADDR_SPI2INT=0 */
	pplc_if_write_jump(REG_ATPL250_INTERLEAVER_SPI_VL8, auc_demod_data_carriers, us_deinterleaver_byte_len, 1);
	/* Update deinterleaver address */
	ul_deinterleaver_write_pointer += us_deinterleaver_word_len;
}

/**
 * \brief Reads symbols from demodulator
 *
 * \param p_rx_ctl   Pointer to Rx control structure
 * \param uc_num_symbols   Number of symbols to read
 * \param uc_change_mod   Flag to indicate if modulation parameters have to be changed
 * \param uc_num_symbols_next   Number of symbols to wait for next interrupt
 * \param uc_read_inactive_carriers   Flag to indicate whether inactive and pilot tones have to be read
 *
 */
void get_demodulator_payload(struct phy_rx_ctl *p_rx_ctl, uint8_t uc_num_symbols, uint8_t uc_change_mod, uint8_t uc_num_symbols_next,
		uint8_t uc_read_inactive_carriers, uint8_t uc_apply_phase_shift)
{
	uint8_t uc_bits_per_carrier;
	uint8_t uc_coh_symbol_ref;
	uint8_t uc_coh_eq_off;
	#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
	uint8_t uc_num_unused_carriers;
	#endif
	uint8_t uc_p_symbol_reloaded = false;

	UNUSED(uc_read_inactive_carriers);

	/* LOG_PHY(("%u\r\n", uc_num_symbols)); */

	if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
		uc_change_mod = true;
	}

	/* Set bits per carrier */
	switch (p_rx_ctl->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
	case MOD_TYPE_BPSK:
		uc_bits_per_carrier = 1;
		if (uc_change_mod) {
			atpl250_set_mod_bpsk();
		}

		uc_coh_symbol_ref = COH_REF_SYMBOL_BPSK;
		uc_coh_eq_off = 0;
		break;

	case MOD_TYPE_QPSK:
		uc_bits_per_carrier = 2;
		if (uc_change_mod) {
			atpl250_set_mod_qpsk();
		}

		uc_coh_symbol_ref = COH_REF_SYMBOL_QPSK;
		uc_coh_eq_off = 0;
		break;

	case MOD_TYPE_8PSK:
		uc_bits_per_carrier = 3;
		if (uc_change_mod) {
			atpl250_set_mod_8psk();
		}

		uc_coh_symbol_ref = COH_REF_SYMBOL_8PSK;
		uc_coh_eq_off = 0;
		break;

	case MOD_TYPE_QAM: /* Not supported */
	default:
		uc_bits_per_carrier = 1;
		if (uc_change_mod) {
			atpl250_set_mod_bpsk();
		}

		uc_coh_symbol_ref = COH_REF_SYMBOL_BPSK;
		uc_coh_eq_off = 0;
		break;
	}

	#ifdef DEMOD_AS_BPSK
	/* In differential scheme, data read is useless, so it is read always as BPSK to save time */
	if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
		uc_bits_per_carrier = 1;
		enable_demod_as_bpsk();
	}
	#endif

	if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
		#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
		if (uc_working_band == WB_CENELEC_A) {
			if ((uc_num_symbols == NUM_SYM_PILOTS_H_EST) &&
					((p_rx_ctl->m_us_rx_payload_symbols - p_rx_ctl->m_us_rx_pending_symbols) >=
					(s_band_constants.uc_pay_sym_first_demod + PAY_SYM_SECOND_DEMOD + NUM_SYM_PILOTS_H_EST))) {
				/* Updates the channel estimate using the pilots */
				chn_estimation_from_pilots(p_rx_ctl->m_auc_static_and_dynamic_notching_pos, p_rx_ctl->m_uc_num_pilots,
						(p_rx_ctl->m_us_rx_payload_symbols - p_rx_ctl->m_us_rx_pending_symbols), p_rx_ctl->m_uc_num_active_carriers,
						&(p_rx_ctl->m_ul_pn_seq_idx));

				/* Updates the SFO using the pilots */
				sampling_error_est_from_pilots(p_rx_ctl->m_us_rx_payload_symbols - p_rx_ctl->m_us_rx_pending_symbols);

				#ifdef DEBUG_CHN_SFO
				uc_num_block_pilots++;
				if (uc_num_block_pilots == 5) {
					printf("STOP\r\n");

					/* Reset num_block_pilots to 1 because if the frame is longer than 8 blocks,
					 * SFO would be updated in next block with index 0, which leads to bad store index */
					uc_num_block_pilots = 1;
				}
				#endif
			} else {
				#ifdef DEBUG_CHN_SFO
				uc_num_block_pilots = 0;
				#endif

				/* Update pointer to LFSR */
				uc_num_unused_carriers = s_band_constants.uc_num_carriers - p_rx_ctl->m_uc_num_active_carriers;
				p_rx_ctl->m_ul_pn_seq_idx = (p_rx_ctl->m_ul_pn_seq_idx + 2 * (p_rx_ctl->m_uc_num_pilots) * uc_num_symbols
						+ uc_bits_per_carrier * uc_num_unused_carriers * uc_num_symbols) % 127;
			}
		}
		#endif

		if (uc_coh_eq_off) {
			pplc_if_or8(REG_ATPL250_INOUTB_CONF2_H8, 0x08);
		} else {
			pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, 0xF7);
		}

		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, uc_coh_symbol_ref); /* Set source to symbol 0 (channel) destination to proper symbol */

		/* Demodulate group of symbols */
		_demodulate_symbols(p_rx_ctl, uc_num_symbols, uc_bits_per_carrier, BCODE_COH, 0);
	} else {
		if (uc_apply_phase_shift) {
			/* Demodulate first symbol with phase shift */
			atpl250_set_phase_correction();
			_demodulate_symbols(p_rx_ctl, 1, uc_bits_per_carrier, BCODE_DIFT, 0);

			/* Demodulate remaining symbols */
			atpl250_clear_phase_correction();
			if (uc_num_symbols > 1) {
				_demodulate_symbols(p_rx_ctl, uc_num_symbols - 1, uc_bits_per_carrier, BCODE_DIFT, 1);
			}

			/* If the place where the phase correction has been stored was occupied by reference values for coherent demod, the overwritten
			 * reference is stored again */
			switch (uc_index_inob_abs) {
			case COH_REF_SYMBOL_BPSK:
				set_p_symbol_phases(auc_psymbol, COH_REF_SYMBOL_BPSK);
				uc_p_symbol_reloaded = true;
				break;

			case COH_REF_SYMBOL_QPSK:
				set_p_symbol_phases(auc_psymbol, COH_REF_SYMBOL_QPSK);
				uc_p_symbol_reloaded = true;
				break;

			case COH_REF_SYMBOL_8PSK:
				set_p_symbol_phases(auc_psymbol, COH_REF_SYMBOL_8PSK);
				uc_p_symbol_reloaded = true;
				break;
			}
			/* Restore demodulation state to previous value (changed in set_p_symbol_phases) */
			if (uc_p_symbol_reloaded) {
				switch (p_rx_ctl->e_mod_type) {
				case MOD_TYPE_BPSK_ROBO:
				case MOD_TYPE_BPSK:
					atpl250_set_mod_bpsk();
					break;

				case MOD_TYPE_QPSK:
					atpl250_set_mod_qpsk();
					break;

				case MOD_TYPE_8PSK:
					atpl250_set_mod_8psk();
					break;

				case MOD_TYPE_QAM: /* Not supported */
				default:
					atpl250_set_mod_bpsk();
					break;
				}
				uc_p_symbol_reloaded = false;
			}
		} else {
			/* Demodulate group of symbols */
			_demodulate_symbols(p_rx_ctl, uc_num_symbols, uc_bits_per_carrier, BCODE_DIFT, 0);
		}
	}

	/* Set number of symbols for next interrupt */
	p_rx_ctl->m_uc_next_demod_symbols = uc_num_symbols_next;
	atpl250_set_num_symbols_cfg(uc_num_symbols_next);

	#ifdef DEMOD_AS_BPSK
	if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
		disable_demod_as_bpsk();
	}
	#endif
}
