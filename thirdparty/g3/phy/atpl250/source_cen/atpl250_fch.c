/**
 * \file
 *
 * \brief ATPL250 G3 FCH Handling
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
#include <string.h>

/* Phy layer includes */
#include "atpl250_fch.h"
#include "atpl250.h"
#include "atpl250_common.h"
#include "atpl250_carrier_mapping.h"

/* Extern variables needed */
extern uint8_t uc_num_symbols_fch;
extern uint8_t uc_notched_carriers;

/**
 * \brief Computes CRC5 for a given bit array encoded as an uint32
 *
 * \param ul_crc_input   Input bits to compute crc. Bits have to be aligned to left
 *
 * \return CRC5 value
 *
 */
static uint8_t _compute_crc5(uint32_t ul_crc_input)
{
	const uint32_t ul_poly5 = (0x05u << 27);
	uint32_t ul_crc5_out  = (0x1Fu << 27);
	uint8_t uc_i;

	for (uc_i = 0; uc_i < 28; uc_i++) {
		if ((ul_crc_input ^ ul_crc5_out) & (0x80000000)) {
			ul_crc5_out <<= 1;
			ul_crc5_out ^= ul_poly5;
		} else {
			ul_crc5_out <<= 1;
		}

		ul_crc_input <<= 1;
	}

	/* Shift back into position */
	ul_crc5_out >>= 27;

	/* Invert output bits: */
	ul_crc5_out ^= 0x1f;

	return (uint8_t)(ul_crc5_out);
}

/**
 * \brief Encode FCH for tx frame in G3 Cenelec A band
 *
 * \param p_tx_ctl     Pointer to Tx Control structure
 * \param puc_fch_buf  Pointer to buffer where FCH will be stored
 *
 */
void phy_fch_encode_g3_cenelec_a(struct phy_tx_ctl *p_tx_ctl, uint8_t *puc_fch_buf)
{
	uint16_t us_num, us_den;
	uint8_t uc_fl;
	uint8_t uc_bits_per_carrier;
	uint8_t uc_coded_modulation;
	uint8_t uc_coded_mod_scheme;
	uint32_t ul_fch_word;
	uint8_t uc_crc5;

	if ((p_tx_ctl->e_delimiter_type == DT_ACK) || (p_tx_ctl->e_delimiter_type == DT_NACK)) {
		/* ACK has no payload, and FCH is already filled at this point with last FCS, set values for the other fields */
		p_tx_ctl->m_us_tx_payload_symbols = 0;
		/* Ensure room for crc is filled with 0s */
		puc_fch_buf[3] &= 0xF0;
		puc_fch_buf[4] = 0;
	} else {
		/* Get payload carriers from tone map and static notching */
		p_tx_ctl->m_uc_payload_carriers = get_payload_carriers(p_tx_ctl->m_auc_static_and_dynamic_notching_pos);
		if (p_tx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
			p_tx_ctl->m_uc_payload_carriers -= p_tx_ctl->m_uc_num_pilots;
		}

		/* Set bits per carrier */
		switch (p_tx_ctl->e_mod_type) {
		case MOD_TYPE_BPSK_ROBO:
			uc_bits_per_carrier = 1;
			uc_coded_modulation = 0x00;
			break;

		case MOD_TYPE_BPSK:
			uc_bits_per_carrier = 1;
			uc_coded_modulation = 0x01 << 6;
			break;

		case MOD_TYPE_QPSK:
			uc_bits_per_carrier = 2;
			uc_coded_modulation = 0x02 << 6;
			break;

		case MOD_TYPE_8PSK:
			uc_bits_per_carrier = 3;
			uc_coded_modulation = 0x03 << 6;
			break;

		case MOD_TYPE_QAM:
			/* Not supported in Cenelec A */
			uc_bits_per_carrier = 1;
			uc_coded_modulation = 0x00;
			break;

		default:
			uc_bits_per_carrier = 1;
			uc_coded_modulation = 0x00;
			break;
		}

		/* Set coded modulation scheme */
		uc_coded_mod_scheme = (p_tx_ctl->e_mod_scheme) << 7;

		/* FL and payload symbols */
		if (p_tx_ctl->e_mod_type == MOD_TYPE_BPSK_ROBO) {
			/* Set RS parity */
			p_tx_ctl->m_uc_rs_parity = 7;
			/* us_num = (bits payload + RS bits (8*8) + conv trail (6)) * 2 (conv) * 4 (repetition) */
			us_num = ((((p_tx_ctl->m_us_payload_len) << 3) + 70) << 3);
			/* us_den = num carriers * bits per carrier (1 for ROBO) * 4 (FL is number of symbols divided by 4) */
			us_den = (p_tx_ctl->m_uc_payload_carriers) << 2;
			uc_fl = us_num / us_den;
			if (us_num % us_den) {
				uc_fl++;
			}
		} else {
			/* Set RS parity */
			p_tx_ctl->m_uc_rs_parity = 15;
			/* us_num = (bits payload + RS bits (16*8) + conv trail (6)) * 2 (conv) */
			us_num = ((((p_tx_ctl->m_us_payload_len) << 3) + 134) << 1);
			/* us_den = num carriers * bits per carrier * 4 (FL is number of symbols divided by 4) */
			us_den = (p_tx_ctl->m_uc_payload_carriers * uc_bits_per_carrier) << 2;
			uc_fl = us_num / us_den;
			if (us_num % us_den) {
				uc_fl++;
			}
		}

		/* Set Number of symbols from FL */
		p_tx_ctl->m_us_tx_payload_symbols = uc_fl << 2;
		p_tx_ctl->m_ul_pn_seq_idx = uc_num_symbols_fch * uc_notched_carriers; /* If no notching, uc_notched_carriers == 0, so index will be 0 */
		p_tx_ctl->m_ul_pilot_idx =  0;

		/* Create FCH array */
		puc_fch_buf[0] = p_tx_ctl->m_uc_pdc;
		puc_fch_buf[1] = uc_coded_modulation | (uc_fl & 0x3F);
		puc_fch_buf[2] = p_tx_ctl->m_auc_tone_map[0] & 0x3F;
		puc_fch_buf[3] = uc_coded_mod_scheme | ((p_tx_ctl->e_delimiter_type & 0x07) << 4);
	}

	/* Compute CRC5 */
	ul_fch_word = (puc_fch_buf[0] << 24) | (puc_fch_buf[1] << 16) | (puc_fch_buf[2] << 8) | puc_fch_buf[3];
	uc_crc5 = _compute_crc5(ul_fch_word);

	/* Complete FCH array */
	puc_fch_buf[3] |= (uc_crc5 & 0x0F);
	puc_fch_buf[4] = (uc_crc5 & 0x10) << 3;
}

/**
 * \brief Decode FCH for rx frame in G3 Cenelec A band
 *
 * \param p_rx_ctl                  Pointer to Rx Control structure
 * \param puc_static_notching_pos   Pointer to array containing static notching
 *
 * \return 1 if FCH is received correctly, otherwise 0
 *
 */
uint8_t phy_fch_decode_g3_cenelec_a(struct phy_rx_ctl *p_rx_ctl, uint8_t *puc_static_notching_pos)
{
	uint8_t uc_i;
	uint32_t ul_fch_word;
	uint8_t uc_rx_crc, uc_computed_crc;
	uint8_t uc_coded_modulation;
	uint8_t uc_fl;
	uint8_t uc_conv_rep_factor; /* convolutional and repetition factor */
	uint8_t uc_bits_per_carrier;
	uint8_t uc_complete_matrix_shift;
	uint16_t us_rx_bits, us_matrix_size;
	uint16_t us_reg_value;

	/* First, check crc. Data is available on rx buffer */
	ul_fch_word = (p_rx_ctl->auc_rx_buf[0] << 24) | (p_rx_ctl->auc_rx_buf[1] << 16) | (p_rx_ctl->auc_rx_buf[2] << 8) | (p_rx_ctl->auc_rx_buf[3] & 0xF0);
	uc_computed_crc = _compute_crc5(ul_fch_word);
	uc_rx_crc = ((p_rx_ctl->auc_rx_buf[4] & 0x80) >> 3) | (p_rx_ctl->auc_rx_buf[3] & 0x0F);

	if (uc_rx_crc == uc_computed_crc) {
		/* Get pdc */
		p_rx_ctl->m_uc_tx_pdc = p_rx_ctl->auc_rx_buf[0];

		/* Get delimiter type (frame type) */
		p_rx_ctl->e_delimiter_type = (enum delimiter_types)((p_rx_ctl->auc_rx_buf[3] & 0x70) >> 4);
		/* Cenelec does not support 2 RS blocks */
		p_rx_ctl->e_rs_blocks = RS_BLOCKS_1_BLOCK;

		if ((p_rx_ctl->e_delimiter_type == DT_SOF_RESP) || (p_rx_ctl->e_delimiter_type == DT_SOF_NO_RESP)) { /* Data Frame */
			/* Get modulation */
			uc_coded_modulation = (p_rx_ctl->auc_rx_buf[1] & 0xC0) >> 6;
			switch (uc_coded_modulation) {
			case 0x00:
				p_rx_ctl->e_mod_type = MOD_TYPE_BPSK_ROBO;
				uc_bits_per_carrier = 1;
				uc_conv_rep_factor = 8;
				p_rx_ctl->m_uc_rs_parity = 7;
				uc_complete_matrix_shift = 6;
				break;

			case 0x01:
				p_rx_ctl->e_mod_type = MOD_TYPE_BPSK;
				uc_bits_per_carrier = 1;
				uc_conv_rep_factor = 2;
				p_rx_ctl->m_uc_rs_parity = 15;
				uc_complete_matrix_shift = 4;
				break;

			case 0x02:
				p_rx_ctl->e_mod_type = MOD_TYPE_QPSK;
				uc_bits_per_carrier = 2;
				uc_conv_rep_factor = 2;
				p_rx_ctl->m_uc_rs_parity = 15;
				uc_complete_matrix_shift = 4;
				break;

			case 0x03:
				p_rx_ctl->e_mod_type = MOD_TYPE_8PSK;
				uc_bits_per_carrier = 3;
				uc_conv_rep_factor = 2;
				p_rx_ctl->m_uc_rs_parity = 15;
				uc_complete_matrix_shift = 4;
				break;

			default:
				p_rx_ctl->e_mod_type = MOD_TYPE_BPSK_ROBO;
				uc_bits_per_carrier = 1;
				uc_conv_rep_factor = 8;
				p_rx_ctl->m_uc_rs_parity = 7;
				uc_complete_matrix_shift = 6;
				break;
			}

			/* Get modulation scheme */
			p_rx_ctl->e_mod_scheme = (enum mod_schemes)((p_rx_ctl->auc_rx_buf[3] & 0x80) >> 7);

			/* Get frame length */
			uc_fl = p_rx_ctl->auc_rx_buf[1] & 0x3F;
			p_rx_ctl->m_us_rx_pending_symbols = uc_fl << 2; /* symbols = fl * 4 */
			p_rx_ctl->m_us_rx_payload_symbols = p_rx_ctl->m_us_rx_pending_symbols;
			/* Get tone map */
			p_rx_ctl->m_auc_tone_map[0] = p_rx_ctl->auc_rx_buf[2] & 0x3F;
			p_rx_ctl->m_auc_inv_tone_map[0] = ((uint8_t)(~(unsigned int)(p_rx_ctl->auc_rx_buf[2]))) & 0x3F;
			generate_inactive_carriers_cenelec_a(p_rx_ctl->m_auc_tone_map[0], p_rx_ctl->m_auc_inactive_carriers_pos);
			generate_inactive_carriers_cenelec_a(p_rx_ctl->m_auc_inv_tone_map[0], p_rx_ctl->m_auc_inv_inactive_carriers_pos);
			/* Generate the combination of inactive and notched carriers */
			for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
				p_rx_ctl->m_auc_static_and_dynamic_notching_pos[uc_i] = puc_static_notching_pos[uc_i] |
						p_rx_ctl->m_auc_inactive_carriers_pos[uc_i];
				p_rx_ctl->m_auc_static_and_inv_dynamic_notching_pos[uc_i] = puc_static_notching_pos[uc_i] |
						p_rx_ctl->m_auc_inv_inactive_carriers_pos[uc_i];
			}

			p_rx_ctl->m_uc_num_active_carriers = get_active_carriers(p_rx_ctl->m_auc_inactive_carriers_pos, puc_static_notching_pos);

			p_rx_ctl->m_uc_num_pilots = p_rx_ctl->m_uc_num_active_carriers / PILOT_FREQ_SPA;
			if (p_rx_ctl->m_uc_num_active_carriers % PILOT_FREQ_SPA) {
				p_rx_ctl->m_uc_num_pilots++;
			}

			/* Reset pilot array */
			memset(p_rx_ctl->m_auc_pilot_pos, 0, CARR_BUFFER_LEN * sizeof(uint8_t));

			/* Configure values for pseudo-random sequence */
			p_rx_ctl->m_ul_pn_seq_idx = uc_num_symbols_fch * uc_notched_carriers; /* If no notching, uc_notched_carriers == 0, so index will be 0 */

			/* Check Tone Map */
			if (p_rx_ctl->m_auc_tone_map[0] == 0x00) {
				/* Invalid tone map, return as wrong crc */
				return 0;
			}

			/* Get payload carriers from tone map and static notching */
			p_rx_ctl->m_uc_payload_carriers = get_payload_carriers(p_rx_ctl->m_auc_static_and_dynamic_notching_pos);
			if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
				p_rx_ctl->m_uc_payload_carriers -= p_rx_ctl->m_uc_num_pilots;
			}

			/* Calculate payload length and interleaver bit padding */
			us_matrix_size = p_rx_ctl->m_us_rx_pending_symbols * (uint16_t)p_rx_ctl->m_uc_payload_carriers * (uint16_t)uc_bits_per_carrier;
			us_rx_bits = us_matrix_size - ((uint16_t)6 * (uint16_t)uc_conv_rep_factor);
			p_rx_ctl->m_us_rx_len = us_rx_bits >> uc_complete_matrix_shift;
			if ((p_rx_ctl->m_us_rx_len > (PHY_MAX_PAYLOAD_WITH_RS_SIZE - p_rx_ctl->m_uc_rs_parity)) ||
					(p_rx_ctl->m_us_rx_len < (PHY_MIN_PAYLOAD_SIZE + p_rx_ctl->m_uc_rs_parity + 1))) {
				/* Length error, return as wrong crc */
				return 0;
			}

			p_rx_ctl->m_uc_bit_padding = us_rx_bits - (p_rx_ctl->m_us_rx_len << uc_complete_matrix_shift);
			/* Before correcting padding, get value to write in interleaver config register */
			us_reg_value = us_matrix_size - p_rx_ctl->m_uc_bit_padding;
			if (p_rx_ctl->e_mod_type == MOD_TYPE_BPSK_ROBO) {
				p_rx_ctl->m_uc_bit_padding >>= 2;
			}

			/* Remove RS parity from Rx length */
			p_rx_ctl->m_us_rx_len = p_rx_ctl->m_us_rx_len - p_rx_ctl->m_uc_rs_parity - 1;
			/* Write reg_value to "useful size" register */
			pplc_if_write16(REG_ATPL250_INTERLEAVER_CFG2_L16, us_reg_value);
			if (p_rx_ctl->e_mod_type == MOD_TYPE_BPSK_ROBO) {
				/* Set Viterbi wait time to 0 */
				pplc_if_write8(REG_ATPL250_INTERLEAVER_CFG2_H8, 0);
			} else {
				/* Set value to Viterbi wait for other modulations */
				pplc_if_write8(REG_ATPL250_INTERLEAVER_CFG2_H8, 16);
			}
		} else if ((p_rx_ctl->e_delimiter_type == DT_ACK) || (p_rx_ctl->e_delimiter_type == DT_NACK)) {
			/* ACK/NACK. Set length to 4 bytes (we will send FCH as payload to upper layer) */
			p_rx_ctl->m_us_rx_len = 4;
		} else {
			/* Invalid DT, return as bad crc */
			return 0;
		}

		/* Read EVM of header */
		p_rx_ctl->m_us_evm_header = atpl250_read_evm_bpsk();
		/* Reset it */
		atpl250_reset_evm();

		/* Return 1 to indicate crc is ok and parameters are decoded */
		return 1;
	} else {
		/* Wrong crc */
		return 0;
	}
}
