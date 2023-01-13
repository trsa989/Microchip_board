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

/* Pointer to static notching array defined in atpl250 module */
extern uint8_t auc_static_notching_pos[];
static uint8_t *puc_static_notching = auc_static_notching_pos;

/* Extern variables needed */
extern uint8_t uc_used_carriers;
extern uint8_t uc_psymbol_len;
extern uint16_t aus_pilot_stored[][PROTOCOL_CARRIERS];
extern uint8_t uc_index_set;
extern uint8_t uc_legacy_mode;
extern uint8_t auc_unmasked_carrier_list[];

static const uint8_t scramblerBitArray[127] =   {0, 0, 0, 0, 1, 1, 1, 0,
						 1, 1, 1, 1, 0, 0, 1, 0,
						 1, 1, 0, 0, 1, 0, 0, 1,
						 0, 0, 0, 0, 0, 0, 1, 0,
						 0, 0, 1, 0, 0, 1, 1, 0,
						 0, 0, 1, 0, 1, 1, 1, 0,
						 1, 0, 1, 1, 0, 1, 1, 0,
						 0, 0, 0, 0, 1, 1, 0, 0,
						 1, 1, 0, 1, 0, 1, 0, 0,
						 1, 1, 1, 0, 0, 1, 1, 1,
						 1, 0, 1, 1, 0, 1, 0, 0,
						 0, 0, 1, 0, 1, 0, 1, 0,
						 1, 1, 1, 1, 1, 0, 1, 0,
						 0, 1, 0, 1, 0, 0, 0, 1,
						 1, 0, 1, 1, 1, 0, 0, 0,
						 1, 1, 1, 1, 1, 1, 1};

#define PILOT_POS_LEN   13

/* Buffer to store pilot position */
static uint16_t auc_pos_pilots[PILOT_POS_LEN];
/* Buffer to store pilots info in the correct order to write to registers */
static uint8_t auc_pilot_reg_values[28];

/**
 * \brief Write P symbol phases to HW, for coherent demodulation
 *
 * \param puc_psymbol  Pointer to P symbol reference
 *
 */
void set_p_symbol_phases(const uint8_t *puc_psymbol)
{
	uint8_t auc_zeros[P_SYMBOL_LEN_MAX];

	memset(auc_zeros, 0, P_SYMBOL_LEN_MAX);

	/* Send P symbol angles to Zone2 for later coherent demodulation */
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_H8, 0x20); /* H_1H_BYPASS='1'; H_1H='0'; DEST_Y='0'; SOURCE_H='0' */
	atpl250_set_mod_bpsk_truepoint();
	pplc_if_write16(REG_ATPL250_INOUTB_CONF3_L16, 0x8000); /* Set alpha = 1 beta = 0 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER); /* Avoid overflow to last carrier */

	/* Reference for BPSK */
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_BPSK); /* Set destination to proper symbol */
	pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xC0); /* Set MOD_PHASE bit and MOD_OFFSET = 0 to store phase instead of real/imag ichannel_coh set (BPSK) */
	pplc_if_write_jump((BCODE_ICHANNEL | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_1);
	/* Reference for QPSK */
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_QPSK); /* Set destination to proper symbol */
	pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xCE); /* Set MOD_PHASE bit and MOD_OFFSET = 14 to store phase instead of real/imag ichannel_coh set (QPSK) */
	pplc_if_write_jump((BCODE_ICHANNEL | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_1);
	/* Reference for 8PSK */
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, COH_REF_SYMBOL_8PSK); /* Set destination to proper symbol */
	pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0xCF); /* Set MOD_PHASE bit and MOD_OFFSET = 15 to store phase instead of real/imag ichannel_coh set (8PSK) */
	pplc_if_write_jump((BCODE_ICHANNEL | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)puc_psymbol, uc_psymbol_len, JUMP_COL_1);

	/* Restore values */
	pplc_if_write8(REG_ATPL250_INOUTB_CTL_L8, 0x00); /* Clear MOD_PHASE bit and MOD_OFFSET = 0 */
}

/**
 * \brief Generates LSFR segment of a given size from a given index
 *
 * \param ul_bit_index  Index to start from the sequence generator
 * \param us_num_bits   Number of bits to include in the sequence
 * \param puc_buf       Pointer to P buffer in which to store the sequence
 *
 */
static void _generate_lsfr_segment(uint32_t ul_bit_index, uint16_t us_num_bits, uint8_t *puc_buf)
{
	uint8_t uc_first_bit, uc_i, uc_j;
	uint16_t uc_last_bit;
	uint16_t uc_byte_idx = 0;
	uint8_t uc_bit_idx = 0;
	uint8_t uc_num_turns = 0;

	uc_first_bit = ul_bit_index % 127; /* point to next index of the last bit to pick up  -->   128 bits % 127 = 1, but last bit to pick up has index 0 */

	uc_last_bit = uc_first_bit + us_num_bits;

	while (uc_last_bit > 127) {
		uc_last_bit -= 127;
		uc_num_turns++;
	}

	puc_buf[uc_byte_idx] = 0x00;
	for (uc_i = 0; uc_i < uc_num_turns; uc_i++) {
		for (uc_j = uc_first_bit; uc_j < 127; uc_j++) {
			if (scramblerBitArray[uc_j]) {
				puc_buf[uc_byte_idx] |= (0x01 << (7 - uc_bit_idx));
			}

			uc_bit_idx++;
			if (uc_bit_idx == 8) {
				uc_bit_idx = 0;
				uc_byte_idx++;
				puc_buf[uc_byte_idx] = 0x00;
			}
		}
		uc_first_bit = 0;
	}

	for (uc_j = uc_first_bit; uc_j < uc_last_bit; uc_j++) {
		if (scramblerBitArray[uc_j]) {
			puc_buf[uc_byte_idx] |= (0x01 << (7 - uc_bit_idx));
		}

		uc_bit_idx++;
		if (uc_bit_idx == 8) {
			uc_bit_idx = 0;
			uc_byte_idx++;
			puc_buf[uc_byte_idx] = 0x00;
		}
	}
}

/* COH function */

/**
 * \brief Reads pilot position from HW registers
 *
 * \param uc_num_pilots  Number of pilots to read its position
 *
 */
static void _coh_read_pos_pilots(uint8_t uc_num_pilots)
{
	uint8_t i;
	uint8_t uc_num_pilots_local = uc_num_pilots;
	uint8_t vector_pilots[28] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	if (uc_num_pilots_local > PILOT_POS_LEN) {
		uc_num_pilots_local = PILOT_POS_LEN;
	}

	pplc_if_read_buf(REG_ATPL250_COH_PILOTS01_32, vector_pilots, (uc_num_pilots_local << 1));
	for (i = 0; i < uc_num_pilots_local; i += 2) {
		auc_pos_pilots[i] = (vector_pilots[i << 1] << 8) | vector_pilots[(i << 1) + 1];
		auc_pos_pilots[i + 1] = (vector_pilots[(i << 1) + 2] << 8) | vector_pilots[(i << 1) + 3];
	}
}

/**
 * \brief Gets current pilot position array, and calculates the position for next symbol
 *
 * \param puc_static_and_dynamic_notching_pos  Pointer to array containing static and dynamic notching
 * \param puc_pilot_pos                        Pointer to array of pilot position
 *
 */
static void _get_next_symbol_pilots(uint8_t *puc_static_and_dynamic_notching_pos, uint8_t *puc_pilot_pos)
{
	uint8_t uc_i;
	uint8_t uc_j;
	uint8_t uc_k;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint8_t uc_i_new;
	uint8_t m_auc_pilot_pos_new[CARR_BUFFER_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
		/* Calculate byte and bit position for index */
		uc_byte_index = uc_i >> 3;
		uc_bit_index = uc_i & 0x07;

		if (puc_pilot_pos[uc_byte_index] & (1 << uc_bit_index)) { /* It is a pilot */
			/* Rotate the pilot 2 positions */
			uc_i_new = uc_i;
			for (uc_j = 0; uc_j <= 1; uc_j++) {
				/* Increase 1 position */
				if (uc_i_new == LAST_CARRIER) {
					uc_i_new = FIRST_CARRIER;
				} else {
					uc_i_new++;
				}

				/* Check that there are no notching */
				for (uc_k = 0; uc_k < 128; uc_k++) {
					uc_byte_index = uc_i_new >> 3;
					uc_bit_index = uc_i_new & 0x07;
					if (puc_static_and_dynamic_notching_pos[uc_byte_index] & (1 << uc_bit_index)) {
						/* Increase 1 position */
						if (uc_i_new == LAST_CARRIER) {
							uc_i_new = FIRST_CARRIER;
						} else {
							uc_i_new++;
						}
					} else {
						break;
					}
				}
			}
			/* Assign the new position in a new vector */
			uc_byte_index = uc_i_new >> 3;
			uc_bit_index = uc_i_new & 0x07;
			m_auc_pilot_pos_new[uc_byte_index] |=  (1 << uc_bit_index);
		}
	}
	memcpy(puc_pilot_pos, m_auc_pilot_pos_new, CARR_BUFFER_LEN);
}

/**
 * \brief Calculates the lsfr sequence that modulates the pilots and inactive carriers. For now only the pilots lsfr sequence is returned (the inactive lsfr
 * squence is computed but not returned).
 * Also sets (in auc_pilot_position_current_set) the indexes of the pilots that have been received in the current set of pilots and computes the
 * pilot map of all the pilots received in the set.
 *
 * \param r_tx_ctl         Pointer to Rx control structure
 * \param uc_num_symbols   Number of symbols to write
 * \param uc_num_pilots	   Number of pilots per symbol
 * \param puc_lsfr_pilots  Pointer to the lfsr squence that modulates the pilots
 *
 * \return the number of bits in the lsfr sequency that have been obtained (including pilots, inactive and notched).
 *
 */
static uint16_t _calculates_pilots_inactive_lsfr_sequence(struct phy_rx_ctl *p_rx_ctl, uint8_t uc_num_symbols, uint8_t uc_num_pilots, uint8_t *puc_pilots_map,
		uint8_t *puc_num_bytes_pilots, uint8_t *puc_num_bits_pilots, uint8_t *puc_pilot_carrier_list, uint8_t uc_index_first_symbol_block)
{
	uint8_t uc_i;
	uint8_t uc_j;
	uint8_t uc_k;
	uint8_t uc_pilots_byte_len;
	uint8_t uc_pilots_num_bits;
	uint8_t uc_bits_per_carrier;
	uint16_t us_unused_num_bits;
	uint16_t us_total_lsfr_num_bits;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint16_t us_curr_lsfr_bit_index;
	uint16_t us_rel_lsfr_byte_index;
	uint8_t uc_rel_lsfr_bit_index;
	uint8_t uc_curr_pilot_bit_index;
	uint8_t uc_rel_pilot_byte_index;
	uint8_t uc_rel_pilot_bit_index;
	uint8_t uc_notched_carriers_local;
	uint8_t uc_num_carrier_pilots = 0;

	/* uint16_t start_us, end_us, diff_us; */

	/* Number of bits for pilots = pilots_per_symbol * num symbols  * 2 bits per carrier (QPSK) */
	uc_pilots_num_bits = uc_num_pilots * uc_num_symbols * 2;
	/* Get length in bytes to write */
	uc_pilots_byte_len = (uc_pilots_num_bits >> 3);
	if (uc_pilots_num_bits & 0x07) {
		uc_pilots_byte_len++;
	}

	/* Set bits per carrier */
	switch (p_rx_ctl->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
	case MOD_TYPE_BPSK:
		uc_bits_per_carrier = 1;
		break;

	case MOD_TYPE_QPSK:
		uc_bits_per_carrier = 2;
		break;

	case MOD_TYPE_8PSK:
		uc_bits_per_carrier = 3;
		break;

	case MOD_TYPE_QAM:
		uc_bits_per_carrier = 4;
		break;

	default:
		uc_bits_per_carrier = 1;
		break;
	}

	/* Copy the pilots map to a temp variable. The pilots map in p_rx_ctl->m_auc_pilot_pos must be the one corresponding to the actual symbol */
	memcpy(puc_pilots_map, p_rx_ctl->m_auc_pilot_pos, CARR_BUFFER_LEN);

	/* Number of bits for unused carriers = unused carriers per symbol * num symbols * bits per carrier depending on modulation */
	us_unused_num_bits = (PROTOCOL_CARRIERS - p_rx_ctl->m_uc_payload_carriers - uc_num_pilots) * uc_num_symbols * uc_bits_per_carrier;

	/* LSFR sequence has to be generated for the sum of pilots and unused tones */
	us_total_lsfr_num_bits = us_unused_num_bits + uc_pilots_num_bits;

	/* Use the same buffer as for data, as data content is useless */
	_generate_lsfr_segment(p_rx_ctl->m_ul_pn_seq_idx, us_total_lsfr_num_bits, u_shared_buffers.s_mod_demod.auc_mod_demod_data);

	/* Clear arrays before constructing them */
	memset(u_shared_buffers.s_mod_demod.auc_pn_seq_inactive, 0, sizeof(u_shared_buffers.s_mod_demod.auc_pn_seq_inactive));
	memset(u_shared_buffers.s_mod_demod.auc_pn_seq_pilots, 0, sizeof(u_shared_buffers.s_mod_demod.auc_pn_seq_pilots));

	/* Separate lsfr bits between pilots and inactive carriers */
	us_curr_lsfr_bit_index = 0;
	uc_curr_pilot_bit_index = 0;
	for (uc_j = 0; uc_j < uc_num_symbols; uc_j++) {
		/* Look for unused, inactive and pilot tones in all band */
		uc_notched_carriers_local = 0;
		for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
			/* Calculate byte and bit position for index */
			uc_byte_index = uc_i >> 3;
			uc_bit_index = uc_i & 0x07;

			if (puc_static_notching[uc_byte_index] & (1 << uc_bit_index)) {
				/* inactive carrier, advance lsfr index but not set bit in any array */
				us_curr_lsfr_bit_index += uc_bits_per_carrier;
				uc_notched_carriers_local++;
			} else if (p_rx_ctl->m_auc_inactive_carriers_pos[uc_byte_index] & (1 << uc_bit_index)) {
				/* unused carrier, advance lsfr index but not set bit in any array */
				us_curr_lsfr_bit_index += uc_bits_per_carrier;
			} else if (p_rx_ctl->m_auc_pilot_pos[uc_byte_index] & (1 << uc_bit_index)) {
				/* Put symbol index in the array aus_pilot_stored */
				aus_pilot_stored[uc_index_set][uc_i - uc_notched_carriers_local - FIRST_CARRIER] = uc_index_first_symbol_block + uc_j + 1;

				/* Put pilot carrier index in the pilot carrier list */
				*(puc_pilot_carrier_list + uc_num_carrier_pilots) = uc_i - FIRST_CARRIER - uc_notched_carriers_local;
				uc_num_carrier_pilots++;

				/* Pilot carrier, set bit value in proper array and advance indices */
				for (uc_k = 0; uc_k < 2; uc_k++) { /* 2 bits for pilots are modulated in QPSK */
					us_rel_lsfr_byte_index = us_curr_lsfr_bit_index >> 3;
					uc_rel_lsfr_bit_index = 7 - (us_curr_lsfr_bit_index & 0x07); /* 7 - index because this sequence is ordered in the
					                                                              * reverse way than the others */

					/* Check next 2 bits, because pilots are modulated in QPSK */
					if (u_shared_buffers.s_mod_demod.auc_mod_demod_data[us_rel_lsfr_byte_index] & (1 << uc_rel_lsfr_bit_index)) {
						uc_rel_pilot_byte_index = uc_curr_pilot_bit_index >> 3;
						uc_rel_pilot_bit_index = 7 - (uc_curr_pilot_bit_index & 0x07);

						u_shared_buffers.s_mod_demod.auc_pn_seq_pilots[uc_rel_pilot_byte_index] |= (1 << uc_rel_pilot_bit_index);
					}

					us_curr_lsfr_bit_index++;
					uc_curr_pilot_bit_index++;
				}
			}
		}

		/* Get pilots position for next symbol */
		_get_next_symbol_pilots(p_rx_ctl->m_auc_static_and_dynamic_notching_pos, p_rx_ctl->m_auc_pilot_pos);

		if (uc_j != (uc_num_symbols - 1)) { /* The last pilots map correspond to the first symbol of the following block */
			/* Obtains the pilots map of the uc_num_pilots that have been received */
			for (uc_k = 0; uc_k < CARR_BUFFER_LEN; uc_k++) {
				*(puc_pilots_map + uc_k) = *(puc_pilots_map + uc_k) | *(p_rx_ctl->m_auc_pilot_pos + uc_k);
			}
		}
	}

	/* Set index of lsft sequence for next call */
	p_rx_ctl->m_ul_pn_seq_idx += us_total_lsfr_num_bits;

	*puc_num_bytes_pilots = uc_pilots_byte_len;
	*puc_num_bits_pilots = uc_pilots_num_bits;

	return us_total_lsfr_num_bits;
}

/**
 * \brief Reads data coming in pilot carriers
 *
 * \param p_rx_ctl        Pointer to reception structure
 * \param uc_num_symbols  Number of symbols to read pilots from
 *
 * \return Position of the first data carrier for further reading
 *
 */
static uint8_t _coh_read_pilots(struct phy_rx_ctl *p_rx_ctl, uint8_t uc_num_symbols)
{
	uint8_t uc_i;
	uint8_t uc_pos_first_data_carrier;
	uint8_t uc_pos_error;
	uint8_t uc_num_bits_in_pilots, uc_num_bytes_in_pilots = 0, uc_len_bytes_pilots_two_bits_byte;
	uint8_t auc_pilot_map[CARR_BUFFER_LEN];
	uint16_t us_num_bits_in_lfsr, us_unused_num_bits;
	uint8_t uc_bits_per_carrier;
	uint8_t auc_num_pilots_per_block;
	uint8_t auc_pilots_per_symbol[NUM_SYM_PILOTS_H_EST * 6];

	uc_pos_first_data_carrier = p_rx_ctl->m_uc_rx_first_carrier;  /* By default */
	/* Pilot position in the first symbol of the block is read from HW */
	_coh_read_pos_pilots(p_rx_ctl->m_uc_num_pilots);

	/* Disable HW chain */
	pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

	/* Channel estimation from pilots is called only when there are 6 symbols and when at least 12 symbols have been received. The latter condition */
	/* is because the first block of payload symbols were received when the SFO was being estimated. Hence, some of them might have been processed */
	/* before the skip/dup block was updated */
	if ((uc_num_symbols == NUM_SYM_PILOTS_H_EST) && ((p_rx_ctl->m_us_rx_payload_symbols - p_rx_ctl->m_us_rx_pending_symbols) >= 2 * NUM_SYM_PILOTS_H_EST)) {
		/* Generates the lsfr sequence used to modulate the pilots and the pilots map */
		us_num_bits_in_lfsr = _calculates_pilots_inactive_lsfr_sequence(p_rx_ctl, uc_num_symbols, p_rx_ctl->m_uc_num_pilots, auc_pilot_map,
				&uc_num_bytes_in_pilots, &uc_num_bits_in_pilots, auc_pilots_per_symbol,
				(p_rx_ctl->m_us_rx_payload_symbols - p_rx_ctl->m_us_rx_pending_symbols - NUM_SYM_PILOTS_H_EST));

		/* Updates the channel estimate using the pilots. Even if channel estimation is not updated, pilots must be read. */
		auc_num_pilots_per_block = (p_rx_ctl->m_uc_num_pilots) * NUM_SYM_PILOTS_H_EST;
		chn_estimation_from_pilots(u_shared_buffers.s_mod_demod.auc_pn_seq_pilots, p_rx_ctl->m_auc_static_and_dynamic_notching_pos,
				auc_pilot_map, auc_pilots_per_symbol, auc_num_pilots_per_block);

		/* Updates the SFO using the pilots */
		#if defined(UPDATE_CHN_SFO_EST_PAYLOAD)
		sampling_error_est_from_pilots((p_rx_ctl->m_us_rx_payload_symbols - p_rx_ctl->m_us_rx_pending_symbols), p_rx_ctl->m_uc_num_pilots,
				auc_pos_pilots[0], p_rx_ctl->m_us_rx_pending_symbols);
		#endif
	} else { /* Channel estimation is not accomplished but pilots must be read */
		uc_index_set = 0;
		/* Updates the pilot position. */
		for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
			_get_next_symbol_pilots(p_rx_ctl->m_auc_static_and_dynamic_notching_pos, p_rx_ctl->m_auc_pilot_pos);
		}

		/* Read pilots for the HW to identify data carriers */
		/* Data length = pilots_per_symbol * num symbols * bits per carrier / 2 bits por byte */
		uc_num_bits_in_pilots = p_rx_ctl->m_uc_num_pilots * uc_num_symbols * 2;
		uc_len_bytes_pilots_two_bits_byte = (uc_num_bits_in_pilots / 2);
		if (uc_num_bits_in_pilots % 2) {
			uc_len_bytes_pilots_two_bits_byte++;
		}

		/* Calculates the number of bits in the lsfr */
		switch (p_rx_ctl->e_mod_type) {
		case MOD_TYPE_BPSK_ROBO:
		case MOD_TYPE_BPSK:
			uc_bits_per_carrier = 1;
			break;

		case MOD_TYPE_QPSK:
			uc_bits_per_carrier = 2;
			break;

		case MOD_TYPE_8PSK:
			uc_bits_per_carrier = 3;
			break;

		case MOD_TYPE_QAM:
			uc_bits_per_carrier = 4;
			break;

		default:
			uc_bits_per_carrier = 1;
			break;
		}

		/* Number of bits for unused carriers = unused carriers per symbol * num symbols * bits per carrier depending on modulation */
		us_unused_num_bits = (PROTOCOL_CARRIERS - p_rx_ctl->m_uc_payload_carriers - p_rx_ctl->m_uc_num_pilots) * uc_num_symbols * uc_bits_per_carrier;

		/* LSFR sequence index has to be updated */
		us_num_bits_in_lfsr = us_unused_num_bits + uc_num_bits_in_pilots;
		p_rx_ctl->m_ul_pn_seq_idx += us_num_bits_in_lfsr; /* When channel estimation is accomplished, this value is updated in
		                                                   * _calculates_pilots_inactive_lsfr_sequence */

		atpl250_set_mod_qpsk();
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER + CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1));
		pplc_if_set_low_speed();
		pplc_if_read_jump((BCODE_DIFT | (auc_pos_pilots[0])), u_shared_buffers.s_mod_demod.auc_pn_seq_pilots, uc_len_bytes_pilots_two_bits_byte,
				JUMP_PILOTS);
		pplc_if_set_high_speed();
	}

	p_rx_ctl->m_ul_pilot_idx += uc_num_bits_in_pilots;

	/* Enable HW chain */
	pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x01);

	/* Calculate first data carrier */
	uc_pos_error = 0;
	for (uc_i = 0; uc_i < p_rx_ctl->m_uc_num_pilots; uc_i++) {
		if (uc_pos_first_data_carrier == auc_pos_pilots[uc_i]) {
			uc_pos_error = 1;
		}
	}

	if (uc_pos_error) {
		/* Hay un piloto en la primera portadora => comprobar si hay en la siguiente */
		uc_pos_first_data_carrier++;
		uc_pos_error = 0;
		for (uc_i = 0; uc_i < p_rx_ctl->m_uc_num_pilots; uc_i++) {
			if (uc_pos_first_data_carrier == auc_pos_pilots[uc_i]) {
				uc_pos_error = 1;
			}
		}

		if (uc_pos_error) {      /* Hay un piloto en la segunda portadora => como solo puede haber 2 pilotos consecutivos, la tercera portadora no
			                  * tendra piloto */
			uc_pos_first_data_carrier++;
		}
	}

	return uc_pos_first_data_carrier;
}

/**
 * \brief Writes bits from lsfr on pilot and inactive tones
 *
 * \param p_tx_ctl         Pointer to Tx control structure
 * \param uc_num_symbols   Number of symbols to write
 *
 * \return First data carrier for further writing
 *
 */
static uint8_t _write_pilots_and_inactive_carriers(struct phy_tx_ctl *p_tx_ctl, uint8_t uc_num_symbols)
{
	uint8_t uc_i;
	uint8_t uc_j;
	uint8_t uc_k;
	uint8_t uc_pos_first_data_carrier = p_tx_ctl->m_uc_tx_first_carrier;  /* by default */
	uint8_t uc_error;
	uint8_t uc_pilots_byte_len;
	uint8_t uc_pilots_num_bits;
	uint8_t uc_bits_per_carrier;
	uint16_t us_inactive_byte_len;
	uint16_t us_inactive_num_bits;
	uint16_t us_unused_num_bits;
	uint16_t us_total_lsfr_num_bits;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint16_t us_curr_lsfr_bit_index;
	uint16_t us_rel_lsfr_byte_index;
	uint8_t uc_rel_lsfr_bit_index;
	uint16_t us_curr_inactive_bit_index;
	uint16_t us_rel_inactive_byte_index;
	uint8_t uc_rel_inactive_bit_index;
	uint8_t uc_curr_pilot_bit_index;
	uint8_t uc_rel_pilot_byte_index;
	uint8_t uc_rel_pilot_bit_index;

	/* Read current pilot position from HW */
	_coh_read_pos_pilots(p_tx_ctl->m_uc_num_pilots);

	/* Disable HW chain */
	pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

	/* Number of bits for pilots = pilots_per_symbol * num symbols  * 2 bits per carrier (QPSK) */
	uc_pilots_num_bits = p_tx_ctl->m_uc_num_pilots * uc_num_symbols * 2;
	/* Get length in bytes to write */
	uc_pilots_byte_len = (uc_pilots_num_bits >> 3);
	if (uc_pilots_num_bits & 0x07) {
		uc_pilots_byte_len++;
	}

	/* Set bits per carrier */
	switch (p_tx_ctl->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
	case MOD_TYPE_BPSK:
		uc_bits_per_carrier = 1;
		break;

	case MOD_TYPE_QPSK:
		uc_bits_per_carrier = 2;
		break;

	case MOD_TYPE_8PSK:
		uc_bits_per_carrier = 3;
		break;

	case MOD_TYPE_QAM:
		uc_bits_per_carrier = 4;
		break;

	default:
		uc_bits_per_carrier = 1;
		break;
	}

	/* Number of bits for unused carriers = unused carriers per symbol * num symbols * bits per carrier depending on modulation */
	us_unused_num_bits = (PROTOCOL_CARRIERS - p_tx_ctl->m_uc_payload_carriers - p_tx_ctl->m_uc_num_pilots) * uc_num_symbols * uc_bits_per_carrier;

	/* LSFR sequence has to be generated for the sum of pilots and unused tones */
	us_total_lsfr_num_bits = us_unused_num_bits + uc_pilots_num_bits;
	/* Use the same buffer as for data, as data content is useless */
	_generate_lsfr_segment(p_tx_ctl->m_ul_pn_seq_idx, us_total_lsfr_num_bits, u_shared_buffers.s_mod_demod.auc_mod_demod_data);

	/* Clear arrays before constructing them */
	memset(u_shared_buffers.s_mod_demod.auc_pn_seq_inactive, 0, sizeof(u_shared_buffers.s_mod_demod.auc_pn_seq_inactive));
	memset(u_shared_buffers.s_mod_demod.auc_pn_seq_pilots, 0, sizeof(u_shared_buffers.s_mod_demod.auc_pn_seq_pilots));

	/* Separate lsfr bits between pilots and inactive carriers */
	us_curr_lsfr_bit_index = 0;
	us_curr_inactive_bit_index = 0;
	uc_curr_pilot_bit_index = 0;
	for (uc_j = 0; uc_j < uc_num_symbols; uc_j++) {
		/* Look for unused, inactive and pilot tones in all band */
		for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
			/* Calculate byte and bit position for index */
			uc_byte_index = uc_i >> 3;
			uc_bit_index = uc_i & 0x07;

			if (puc_static_notching[uc_byte_index] & (1 << uc_bit_index)) {
				/* Unused carrier, advance lsfr index but not set bit in any array */
				us_curr_lsfr_bit_index += uc_bits_per_carrier;
			} else if (p_tx_ctl->m_auc_inactive_carriers_pos[uc_byte_index] & (1 << uc_bit_index)) {
				/* Inactive carrier, set bit value in proper array and advance indices */
				for (uc_k = 0; uc_k < uc_bits_per_carrier; uc_k++) {
					us_rel_lsfr_byte_index = us_curr_lsfr_bit_index >> 3;
					/* 7 - index because this sequence is ordered in the reverse way than the others */
					uc_rel_lsfr_bit_index = 7 - (us_curr_lsfr_bit_index & 0x07);

					if (u_shared_buffers.s_mod_demod.auc_mod_demod_data[us_rel_lsfr_byte_index] & (1 << uc_rel_lsfr_bit_index)) {
						us_rel_inactive_byte_index = us_curr_inactive_bit_index >> 3;
						uc_rel_inactive_bit_index = 7 - (us_curr_inactive_bit_index & 0x07);

						u_shared_buffers.s_mod_demod.auc_pn_seq_inactive[us_rel_inactive_byte_index]
							|= (1 << uc_rel_inactive_bit_index);
					}

					us_curr_lsfr_bit_index++;
					us_curr_inactive_bit_index++;
				}
			} else if (p_tx_ctl->m_auc_pilot_pos[uc_byte_index] & (1 << uc_bit_index)) {
				/* Pilot carrier, set bit value in proper array and advance indices */
				for (uc_k = 0; uc_k < 2; uc_k++) { /* 2 bits for pilots are modulated in QPSK */
					us_rel_lsfr_byte_index = us_curr_lsfr_bit_index >> 3;
					/* 7 - index because this sequence is ordered in the reverse way than the others */
					uc_rel_lsfr_bit_index = 7 - (us_curr_lsfr_bit_index & 0x07);

					/* Check next 2 bits, because pilots are modulated in QPSK */
					if (u_shared_buffers.s_mod_demod.auc_mod_demod_data[us_rel_lsfr_byte_index] & (1 << uc_rel_lsfr_bit_index)) {
						uc_rel_pilot_byte_index = uc_curr_pilot_bit_index >> 3;
						uc_rel_pilot_bit_index = 7 - (uc_curr_pilot_bit_index & 0x07);

						u_shared_buffers.s_mod_demod.auc_pn_seq_pilots[uc_rel_pilot_byte_index] |= (1 << uc_rel_pilot_bit_index);
					}

					us_curr_lsfr_bit_index++;
					uc_curr_pilot_bit_index++;
				}
			}
		}

		/* Get pilots position for next symbol */
		_get_next_symbol_pilots(p_tx_ctl->m_auc_static_and_dynamic_notching_pos, p_tx_ctl->m_auc_pilot_pos);
	}

	/* Number of bits that will be written for inactive carriers = inactive carriers per symbol * num symbols * bits per carrier depending on modulation */
	us_inactive_num_bits = get_payload_carriers(p_tx_ctl->m_auc_static_and_inv_dynamic_notching_pos) * uc_num_symbols * uc_bits_per_carrier;
	/* Get length in bytes to write */
	us_inactive_byte_len = (us_inactive_num_bits >> 3);
	if (us_inactive_num_bits & 0x07) {
		us_inactive_byte_len++;
	}

	/* Set modulation back to proper value to write inactive carriers and later data */
	switch (p_tx_ctl->e_mod_type) {
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

	case MOD_TYPE_QAM:
		atpl250_set_mod_qam();
		break;
	}

	/* Write inactive carriers */
	if (us_inactive_num_bits) {
		write_jumps_inactive();
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + LAST_CARRIER);
		pplc_if_write_jump((BCODE_DIFT | (p_tx_ctl->m_uc_tx_first_carrier_pn_seq)),
				u_shared_buffers.s_mod_demod.auc_pn_seq_inactive, us_inactive_byte_len, JUMP_COL_2);
	}

	/* Write pilots */
	write_jumps_data();
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + LAST_CARRIER);
	atpl250_set_mod_qpsk();
	pplc_if_set_low_speed();
	pplc_if_write_jump((BCODE_DIFT | (auc_pos_pilots[0])), u_shared_buffers.s_mod_demod.auc_pn_seq_pilots, uc_pilots_byte_len, JUMP_PILOTS);
	pplc_if_set_high_speed();

	/* Set modulation back to proper value to write data */
	switch (p_tx_ctl->e_mod_type) {
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

	case MOD_TYPE_QAM:
		atpl250_set_mod_qam();
		break;
	}

	/* Enable HW chain to write data on return of this function */
	enable_HW_chain();

	/* Set index of lsft sequence for next call */
	p_tx_ctl->m_ul_pn_seq_idx += us_total_lsfr_num_bits;

	/* Calculate first data carrier */
	uc_error = 0;
	for (uc_i = 0; uc_i < p_tx_ctl->m_uc_num_pilots; uc_i++) {
		if (uc_pos_first_data_carrier == auc_pos_pilots[uc_i]) {
			uc_error = 1;
		}
	}

	while (uc_error) {
		/* Pilot on first carrier, check next */
		uc_pos_first_data_carrier++;
		uc_error = 0;
		for (uc_i = 0; uc_i < p_tx_ctl->m_uc_num_pilots; uc_i++) {
			if (uc_pos_first_data_carrier == auc_pos_pilots[uc_i]) {
				uc_error = 1;
			}
		}

		if (uc_error) { /* Pilot on current carrier too, check next */
			uc_pos_first_data_carrier++;
		}
	}

	return uc_pos_first_data_carrier;
}

/**
 * \brief Writes bits from lsfr on inactive tones
 *
 * \param p_tx_ctl         Pointer to Tx control structure
 * \param uc_num_symbols   Number of symbols to write
 *
 */
static void _write_inactive_carriers(struct phy_tx_ctl *p_tx_ctl, uint8_t uc_num_symbols)
{
	uint8_t uc_i;
	uint8_t uc_j;
	uint8_t uc_k;
	uint8_t uc_bits_per_carrier;
	uint16_t us_inactive_byte_len;
	uint16_t us_inactive_num_bits;
	uint16_t us_unused_num_bits;
	uint16_t us_total_lsfr_num_bits;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint16_t us_curr_lsfr_bit_index;
	uint16_t us_rel_lsfr_byte_index;
	uint8_t uc_rel_lsfr_bit_index;
	uint16_t us_curr_inactive_bit_index;
	uint16_t us_rel_inactive_byte_index;
	uint8_t uc_rel_inactive_bit_index;

	/* Set bits per carrier */
	switch (p_tx_ctl->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
	case MOD_TYPE_BPSK:
		uc_bits_per_carrier = 1;
		break;

	case MOD_TYPE_QPSK:
		uc_bits_per_carrier = 2;
		break;

	case MOD_TYPE_8PSK:
		uc_bits_per_carrier = 3;
		break;

	case MOD_TYPE_QAM:
		uc_bits_per_carrier = 4;
		break;

	default:
		uc_bits_per_carrier = 1;
		break;
	}

	/* Number of bits that will be written for inactive carriers = inactive carriers per symbol * num symbols * bits per carrier depending on modulation */
	us_inactive_num_bits = get_payload_carriers(p_tx_ctl->m_auc_static_and_inv_dynamic_notching_pos) * uc_num_symbols * uc_bits_per_carrier;
	/* Get length in bytes to write */
	us_inactive_byte_len = (us_inactive_num_bits >> 3);
	if (us_inactive_num_bits & 0x07) {
		us_inactive_byte_len++;
	}

	/* Continue only if there are bits to write on inactive carriers */
	if (us_inactive_num_bits) {
		/* Disable HW chain */
		pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

		/* Number of bits for unused carriers = unused carriers per symbol * num symbols * bits per carrier depending on modulation */
		us_unused_num_bits = (PROTOCOL_CARRIERS - p_tx_ctl->m_uc_payload_carriers) * uc_num_symbols * uc_bits_per_carrier;

		/* LSFR sequence has to be generated for unused tones */
		us_total_lsfr_num_bits = us_unused_num_bits;
		/* Use the same buffer as for data, as data content is useless */
		_generate_lsfr_segment(p_tx_ctl->m_ul_pn_seq_idx, us_total_lsfr_num_bits, u_shared_buffers.s_mod_demod.auc_mod_demod_data);

		/* Clear array before constructing it */
		memset(u_shared_buffers.s_mod_demod.auc_pn_seq_inactive, 0, sizeof(u_shared_buffers.s_mod_demod.auc_pn_seq_inactive));

		/* Separate lsfr bits between unused and inactive carriers */
		us_curr_lsfr_bit_index = 0;
		us_curr_inactive_bit_index = 0;
		for (uc_j = 0; uc_j < uc_num_symbols; uc_j++) {
			/* Look for unused and inactive tones in all band */
			for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
				/* Calculate byte and bit position for index */
				uc_byte_index = uc_i >> 3;
				uc_bit_index = uc_i & 0x07;

				if (puc_static_notching[uc_byte_index] & (1 << uc_bit_index)) {
					/* Unused carrier, advance lsfr index but not set bit in any array */
					us_curr_lsfr_bit_index += uc_bits_per_carrier;
				} else if (p_tx_ctl->m_auc_inactive_carriers_pos[uc_byte_index] & (1 << uc_bit_index)) {
					/* Inactive carrier, set bit value in proper array and advance indices */
					for (uc_k = 0; uc_k < uc_bits_per_carrier; uc_k++) {
						us_rel_lsfr_byte_index = us_curr_lsfr_bit_index >> 3;
						uc_rel_lsfr_bit_index = 7 - (us_curr_lsfr_bit_index & 0x07); /* 7 - index (reverse sequence) */

						if (u_shared_buffers.s_mod_demod.auc_mod_demod_data[us_rel_lsfr_byte_index] & (1 << uc_rel_lsfr_bit_index)) {
							us_rel_inactive_byte_index = us_curr_inactive_bit_index >> 3;
							uc_rel_inactive_bit_index = 7 - (us_curr_inactive_bit_index & 0x07);

							u_shared_buffers.s_mod_demod.auc_pn_seq_inactive[us_rel_inactive_byte_index]
								|= (1 << uc_rel_inactive_bit_index);
						}

						us_curr_lsfr_bit_index++;
						us_curr_inactive_bit_index++;
					}
				}
			}
		}

		/* Write inactive carriers */
		write_jumps_inactive();
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + LAST_CARRIER);
		pplc_if_write_jump((BCODE_DIFT | (p_tx_ctl->m_uc_tx_first_carrier_pn_seq)),
				u_shared_buffers.s_mod_demod.auc_pn_seq_inactive, us_inactive_byte_len, JUMP_COL_2);

		/* Enable HW chain to write data on return of this function */
		enable_HW_chain();

		/* Set index of lsft sequence for next call */
		p_tx_ctl->m_ul_pn_seq_idx += us_total_lsfr_num_bits;
	}

	/* Set jumps for data */
	write_jumps_data();
	/* Avoid overflow to n-th symbol when writing */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + LAST_CARRIER);
}

/**
 * \brief Sets pilot tone position for first Tx/Rx payload symbol
 *
 * \param uc_num_pilots               Number of pilots to set
 * \param uc_num_active_carriers      Number of active carriers
 * \param puc_pilot_pos               Pointer to array where position will be stored
 * \param puc_inactive_carriers_pos   Pointer to array containing inactive carriers
 * \param puc_static_notching_pos     Pointer to array containing static notching
 *
 */
void set_pilot_position(uint8_t uc_num_pilots, uint8_t uc_num_active_carriers, uint8_t *puc_pilot_pos,
		uint8_t *puc_inactive_carriers_pos, uint8_t *puc_static_notching_pos)
{
	uint8_t uc_i, uc_j, uc_temp;
	uint8_t auc_pilots_rel_pos[13];
	uint8_t auc_pilots_unordered_pos[13];
	uint8_t uc_byte_to_start = FIRST_CARRIER >> 3;
	uint8_t uc_bit_to_start = FIRST_CARRIER & 0x07;
	uint8_t uc_byte_to_stop = LAST_CARRIER >> 3;
	uint8_t uc_bit_to_stop = LAST_CARRIER & 0x07;
	uint8_t uc_pilot_idx = 0;
	uint8_t uc_active_carrier_cnt = 0;
	uint8_t uc_current_carrier;
	uint8_t uc_lower_pilot_idx = 0;

	/* Clear buffers */
	memset(auc_pilots_rel_pos, 0xFF, sizeof(auc_pilots_rel_pos));
	memset(puc_pilot_pos, 0, CARR_BUFFER_LEN * sizeof(uint8_t));
	memset(auc_pilot_reg_values, 0, sizeof(auc_pilot_reg_values));

	for (uc_i = 0; uc_i < uc_num_pilots; uc_i++) {
		auc_pilots_rel_pos[uc_i] = (PILOT_OFFSET + (PILOT_FREQ_SPA * uc_i)) % uc_num_active_carriers;
	}

	while (uc_pilot_idx < uc_num_pilots) { /* there are pilot missed at the beginning */
		uc_active_carrier_cnt = 0;
		uc_current_carrier = FIRST_CARRIER; /* its going to be increased at the end of the loop */

		uc_byte_to_start = FIRST_CARRIER >> 3;
		uc_bit_to_start = FIRST_CARRIER & 0x07;

		for (uc_i = uc_byte_to_start; uc_i <= uc_byte_to_stop; uc_i++) {
			uc_temp = puc_inactive_carriers_pos[uc_i] | puc_static_notching_pos[uc_i];
			if (uc_i == uc_byte_to_stop) {
				uc_bit_to_stop  = (LAST_CARRIER & 0x07) + 1;
			} else {
				uc_bit_to_stop = 8;
			}

			if (uc_i == uc_byte_to_start) {
				uc_bit_to_start = FIRST_CARRIER & 0x07;
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
					}

					uc_active_carrier_cnt++;
				}

				uc_current_carrier++;
			}
		}
	}

	uc_j = 0;
	for (uc_i = uc_lower_pilot_idx; uc_i < uc_num_pilots; uc_i++) {
		auc_pilot_reg_values[2 * uc_j + 1] = auc_pilots_unordered_pos[uc_i]; /* Save absolute carrier position. */
		uc_j++;
	}

	if (uc_lower_pilot_idx != 0) {
		for (uc_i = 0; uc_i < uc_lower_pilot_idx; uc_i++) {
			auc_pilot_reg_values[2 * uc_j + 1] = auc_pilots_unordered_pos[uc_i]; /* Save absolute carrier position. */
			uc_j++;
		}
	}

	auc_pilot_reg_values[26] = uc_num_pilots | 0x30;    /* Number of pilots per symbol | CONSECUTIVE_PILOTS = '1' SORT_PILOTS = '1' */
	auc_pilot_reg_values[27] = 0x01;                    /* Write */

	/* Write to HW */
	pplc_if_write_buf(REG_ATPL250_COH_PILOTS01_32, auc_pilot_reg_values, 28);
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
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols + uc_offset_symbols - 1)) + LAST_CARRIER);

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
	pplc_if_write_jump((BCODE_DIFT | (FIRST_CARRIER + auc_unmasked_carrier_list[0] + (uc_offset_symbols * CFG_IOB_SAMPLES_PER_SYMBOL))),
			u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_1);
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
void feed_modulator_payload(struct phy_tx_ctl *p_tx_ctl, uint8_t uc_num_symbols, uint8_t uc_change_sym_cfg, struct sym_cfg *p_sym_cfg, uint8_t uc_change_mod)
{
	uint16_t us_dat_len;
	uint16_t us_num;
	uint8_t uc_bits_per_carrier;
	uint8_t uc_pos_first_data_carrier = FIRST_CARRIER;

	/* TODO improve this for FCC */
	if (p_tx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
		/* Write pilots and inactive carriers */
		uc_pos_first_data_carrier = _write_pilots_and_inactive_carriers(p_tx_ctl, uc_num_symbols);
	} else {
		if (uc_change_mod) {
			switch (p_tx_ctl->e_mod_type) {
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

			case MOD_TYPE_QAM:
				atpl250_set_mod_qam();
				break;
			}
		}

		/* Write inactive carriers */
		_write_inactive_carriers(p_tx_ctl, uc_num_symbols);
	}

	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(uc_num_symbols);

	/* Check symbol config changes */
	if (uc_change_sym_cfg) {
		if (uc_change_sym_cfg & 0x01) {
			p_sym_cfg->m_uc_sym_idx = 0;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x02) {
			p_sym_cfg->m_uc_sym_idx = 1;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x04) {
			p_sym_cfg->m_uc_sym_idx = 2;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x08) {
			p_sym_cfg->m_uc_sym_idx = 3;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x10) {
			p_sym_cfg->m_uc_sym_idx = 4;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x20) {
			p_sym_cfg->m_uc_sym_idx = 5;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x40) {
			p_sym_cfg->m_uc_sym_idx = 6;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}

		if (uc_change_sym_cfg & 0x80) {
			p_sym_cfg->m_uc_sym_idx = 7;
			p_sym_cfg->m_uc_reserved = 1;
			pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(p_sym_cfg), sizeof(struct sym_cfg));
		}
	}

	/* Set bits per carrier */
	switch (p_tx_ctl->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
	case MOD_TYPE_BPSK:
		uc_bits_per_carrier = 1;
		break;

	case MOD_TYPE_QPSK:
		uc_bits_per_carrier = 2;
		break;

	case MOD_TYPE_8PSK:
		uc_bits_per_carrier = 3;
		break;

	case MOD_TYPE_QAM:
		uc_bits_per_carrier = 4;
		break;

	default:
		uc_bits_per_carrier = 1;
		break;
	}

	/* Data length = Carriers * Bits per carrier * num symbols / 8 bits in byte */
	us_num = p_tx_ctl->m_uc_payload_carriers * uc_bits_per_carrier * uc_num_symbols;
	us_dat_len = (us_num >> 3);
	if (us_num & 0x0007) {
		us_dat_len++;
	}

	/* Overflow already set on functions which write pilots and/or inactive carriers */
	/* Write data symbols */
	if (p_tx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
		pplc_if_set_low_speed();
		pplc_if_write_jump((BCODE_DIFT | (uc_pos_first_data_carrier)), u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len,
				JUMP_DATA_NO_PILOTS);
		pplc_if_set_high_speed();
	} else {
		pplc_if_write_jump((BCODE_DIFT | (p_tx_ctl->m_uc_tx_first_carrier)), u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_2);
	}
}

/**
 * \brief Reads symbols from demodulator for FCH
 *
 * \param uc_num_symbols   Number of symbols to read
 * \param uc_change_mod   Flag to indicate if modulation parameters have to be changed
 * \param uc_num_symbols_next   Number of symbols to wait for next interrupt
 *
 */
void get_demodulator_fch(uint8_t uc_num_symbols, uint8_t uc_num_symbols_offset, uint8_t uc_change_mod, uint8_t uc_num_symbols_next)
{
	uint16_t us_dat_len;
	uint16_t us_num;

	/* Change modulation if necessary */
	if (uc_change_mod) {
		atpl250_set_mod_bpsk();
	}

	/* Data length = Carriers * Bits per carrier * num symbols / 2 bits in byte (bits soft are coded in 4-bit) */
	us_num = uc_used_carriers * uc_num_symbols; /* Bits per carrier always 1 for FCH */
	us_dat_len = (us_num >> 1);
	if (us_num & 0x0001) {
		us_dat_len++;
	}

	/* Avoid overflow to n-th symbol when reading */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols + uc_num_symbols_offset - 1)) + LAST_CARRIER);
	/* Read data */
	pplc_if_read_jump((BCODE_DIFT | ((CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols_offset)) + FIRST_CARRIER + auc_unmasked_carrier_list[0])),
			u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_1);

	/* Set symbols for next interrupt */
	atpl250_set_num_symbols_cfg(uc_num_symbols_next);
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
		uint8_t uc_read_inactive_carriers)
{
	uint16_t us_dat_len;
	uint16_t us_num;
	uint8_t uc_bits_per_carrier;
	uint16_t us_num_bits_pn_seq;
	uint16_t us_num_bytes_pn_seq;
	uint8_t uc_pos_first_data_carrier = FIRST_CARRIER;
	uint8_t uc_coh_symbol_ref;
	uint8_t uc_coh_eq_off;

	/* Write jumps to read data */
	write_jumps_data();

	/* Avoid overflow to n-th symbol when reading */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + LAST_CARRIER);

	if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
		/* Read pilots and get fist data carrier */
		uc_pos_first_data_carrier = _coh_read_pilots(p_rx_ctl, uc_num_symbols);
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

	case MOD_TYPE_QAM:
		uc_bits_per_carrier = 4;
		if (uc_change_mod) {
			atpl250_set_mod_qam();
		}

		uc_coh_symbol_ref = COH_REF_SYMBOL_BPSK;
		uc_coh_eq_off = 0;
		break;

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
	uc_bits_per_carrier = 1;
	enable_demod_as_bpsk();
	#endif

	/* Data length = Carriers * Bits per carrier * num symbols / 2 bits in byte (bits soft are coded in 4-bit) */
	us_num = p_rx_ctl->m_uc_payload_carriers * uc_bits_per_carrier * uc_num_symbols;
	us_dat_len = (us_num >> 1);
	if (us_num & 0x0001) {
		us_dat_len++;
	}

	/* Read data */
	if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
		if (uc_coh_eq_off) {
			pplc_if_or8(REG_ATPL250_INOUTB_CONF2_H8, 0x08);
		} else {
			pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, 0xF7);
		}

		pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, uc_coh_symbol_ref); /* Set source to symbol 0 (channel) destination to proper symbol */
		pplc_if_set_low_speed();
		pplc_if_read_jump((BCODE_COH | (uc_pos_first_data_carrier)), u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_DATA_NO_PILOTS);
		pplc_if_set_high_speed();
	} else {
		pplc_if_read_jump((BCODE_DIFT | (p_rx_ctl->m_uc_rx_first_carrier)), u_shared_buffers.s_mod_demod.auc_mod_demod_data, us_dat_len, JUMP_COL_2);
	}

	if (uc_read_inactive_carriers) {
		us_num_bits_pn_seq = (PROTOCOL_CARRIERS - p_rx_ctl->m_uc_payload_carriers) * uc_num_symbols * uc_bits_per_carrier;
		us_num_bytes_pn_seq = us_num_bits_pn_seq >> 1; /* times 4 (soft bits) and divided by 8 (bits per byte) = divided by 2 */
		if (us_num_bits_pn_seq & 0x0001) {
			us_num_bytes_pn_seq++;
		}

		if (us_num_bits_pn_seq) {
			write_jumps_inactive();
			/* Avoid overflow */
			pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (uc_num_symbols - 1)) + LAST_CARRIER);
			disable_HW_chain();
			/* Use the same buffer as for data, as data content is useless */
			pplc_if_read_jump((BCODE_DIFT | (p_rx_ctl->m_uc_rx_first_carrier_pn_seq)), u_shared_buffers.s_mod_demod.auc_mod_demod_data,
					us_num_bytes_pn_seq, JUMP_COL_2);
			enable_HW_chain();
		}
	}

	#ifdef DEMOD_AS_BPSK
	disable_demod_as_bpsk();
	#endif

	/* Set number of symbols for next interrupt */
	p_rx_ctl->m_uc_next_demod_symbols = uc_num_symbols_next;
	atpl250_set_num_symbols_cfg(uc_num_symbols_next);
}
