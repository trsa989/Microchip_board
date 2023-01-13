/**
 * \file
 *
 * \brief ATPL250 Channel Estimation
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
#include "atpl250_channel_estimation.h"
#include "atpl250_mod_demod.h"
#include "atpl250_sampling_error_estimation.h"

/* Pointers to external defined arrays */
extern q15_t *pss_rotation_sfo_first, *pss_rotation_sfo_second;

/* Extern PHY Variables */
extern struct band_phy_constants s_band_constants;
extern uint8_t uc_working_band;
extern uint8_t uc_notched_carriers;
extern uint8_t uc_used_carriers;
extern uint8_t uc_num_symbols_fch;
extern uint16_t us_fch_interleaver_size_with_padding;
extern uint8_t uc_fch_interleaver_byte_size;
extern uint8_t uc_fch_symbol_byte_size;
extern uint8_t auc_unmasked_carrier_list[];
extern uint8_t uc_legacy_mode;
extern uint8_t auc_active_carrier_state[];
extern uint8_t auc_static_notching_pos[];

/*-----------------------For debugging ----------------------------------*/
#ifdef DEBUG_CHN_SFO
static q15_t ass_Ypilots_block[NUM_CARRIERS_TIMES_2_FCC * 8] = {0};
static q15_t ass_H_pilotos[NUM_CARRIERS_TIMES_2_FCC * 8] = {0};
static q15_t ass_aux_block[NUM_CARRIERS_TIMES_2_FCC * 8] = {0};
static q15_t ass_payload_block[NUM_CARRIERS_TIMES_2_FCC * 8] = {0};
uint8_t uc_num_block_pilots = 0;
uint8_t auc_pilot_carriers_list_vector[NUM_CARRIERS_FCC * 6] = {0};
#endif
/*---------------------------------------------------------------------*/

/* List of carriers (preamble and FCH) that should be smoothed when static notching is defined. */
uint8_t auc_smooth_control_preamble_fch[NUM_CARRIERS_FCC - 2] = {0};
uint8_t uc_num_carr_smooth_preamble_fch;
/* Contains 1 if the carrier has unnotched predecessor and sucessor, 0 if it has notched predecessor, 2 if notched sucessor */
uint8_t auc_control_avg_invert_chan[NUM_CARRIERS_FCC];

/* Actual number of SYNCP and FCH symbols used for channel estimation. */
/* Maximum number of SYNCP is NUM_SYM_H_EST_PRE and maximum number of fch symbols is NUM_SYM_H_EST_FCH */
uint8_t uc_num_sym_valid_preamble, uc_num_sym_p_detected;
uint8_t uc_num_sym_valid_fch;
uint8_t auc_index_sym_valid_fch[NUM_SYM_H_EST_FCH_CENELEC_A]; /* Last FCH symbol must be at position NUM_SYM_H_EST_FCH-1 */

/* Pointer to the array with the inverse of the noise energy per symbol and of the weighting coefficients */
q31_t asl_Ni[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH_CENELEC_A + 2];
q31_t sl_Ni_min;
static q63_t asll_Ni[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH_CENELEC_A + 2];
q31_t asl_inv_Ni[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH_CENELEC_A + 2];
q15_t ass_Wi[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH_CENELEC_A + 2];

/* Auxiliary variable to estimate channel from FCH and interleaver bits */
static uint8_t auc_previous_ref_bits_interleaver[MAX_FCH_SYMBOL_BYTE_SIZE];

/* Array for storing the p,m fch and s1s2 symbols used for channel estimation and the payload symbols, from which pilot are extracted */
/* auc_pilots_state_carrier_block is an array of values that contain the value 32 if the corresponding carrier is used as pilot */
union shared_preamble_and_payload_symbols u_shared_preamble_and_payload_symbols = {{0}};

/*Array for storing the conjugate of the complex value that modulate one symbol*/
q15_t ass_modulating_symbols[NUM_CARRIERS_TIMES_2_FCC];

/* Data and pilot carrier lists for the current set of pilots used for channel estimation*/
#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
static uint8_t auc_data_carriers_list[NUM_CARRIERS_FCC] = {0};
static uint8_t auc_pilot_carriers_list[NUM_CARRIERS_FCC] = {0}; /* List of pilot carrier per symbol. Ordered from 0...Mactive-1 */
static uint8_t auc_pilot_carriers_list_absolute[NUM_CARRIERS_FCC] = {0}; /* List of pilot carrier per symbol. Ordered from 0...Munnotched-1 */

/* Used in assembly function compute_pilot_position */
uint8_t uc_num_active_carriers_as;
uint8_t uc_index_symbol_in_block;
uint32_t *pul_index_to_lfsr_as;
uint8_t uc_pointer_byte_lfsr_pilots_as;
uint8_t uc_pointer_bitpair_lfsr_pilots_as;
q15_t *pass_Ypilots_as;

/* Used to obtain the LFSR sequence that modulates the pilots */
const uint8_t auc_lfsr_pilots_as[127] = {0, 0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 1, 0, 2, 1, 2, 3, 1, 0, 2, 1, 0, 2, 1, 0, 0, 0, 0, 0, 2, 1, 0, 0,
					 2, 1, 0, 2, 3, 1, 0, 0, 2, 1, 2, 3, 3, 1, 2, 1, 2, 3, 1, 2, 3, 1, 0, 0, 0, 0, 2, 3, 1, 0, 2, 3, 1, 2, 1, 2, 1, 0, 2, 3,
					 3, 1, 0, 2, 3, 3, 3, 1, 2,
					 3, 1, 2, 1, 0, 0, 0, 2, 1, 2, 1, 2, 1, 2, 3, 3, 3, 3, 1, 2, 1, 0, 2, 1, 2, 1, 0, 0, 2, 3, 1, 2, 3, 3, 1, 0, 0, 2, 3, 3,
					 3, 3, 3, 3, 1};
/* auc_carrier_state_pilot : Bit_0=1 indicates that the corresponding carrier is a pilot in symbol 0. Bit_1=1 is the same for symbol 1. */
uint8_t auc_carrier_state_pilot[NUM_CARRIERS_FCC] = {0};
/* List of carriers that are neither notched nor inactive (data + pilots) */
uint8_t auc_active_carrier_state[NUM_CARRIERS_FCC] = {0};

/* Index (either "0" or "1") of the pilots block */
uint8_t uc_index_set = 0;

/* Extern variables from atpl250_sampling_error_estimation */
/*extern q31_t asl_Ypilots[][NUM_CARRIERS_TIMES_2_FCC];*/
extern q15_t ass_Ypilots[][NUM_CARRIERS_TIMES_2_FCC];
extern uint16_t aus_pilot_stored [][NUM_CARRIERS_FCC];
extern uint8_t auc_state_carrier_asm[];

/*Number of used pilot in each symbol (when a pilot appear twice in a block of NUM_SYM_PILOTS_H_EST symbols, only the last one is taken)*/
static uint8_t auc_num_pilots_in_symbol[NUM_SYM_PILOTS_H_EST] = {0};

#endif

/*Array for storing one symbol*/
q15_t ass_symbol_aux[NUM_CARRIERS_TIMES_2_FCC]; /* Reused in atpl250_sampling_error_estimation*/
q31_t asl_symbol_aux[NUM_CARRIERS_TIMES_2_FCC]; /* Reused in atpl250_sampling_error_estimation*/

/* Array for storing the average of the p,m fch and s1s2 symbols */
q15_t ass_average_symbol[NUM_CARRIERS_TIMES_2_FCC];
q15_t ass_average_symbol_fch_fcc[NUM_CARRIERS_TIMES_2_FCC];

/* Array for storing the estimated channel response and its inverse in Q1.15 */
q15_t ass_H[NUM_CARRIERS_TIMES_2_FCC] = {0};  /* Reused in atpl250_sampling_error_estimation*/
q15_t ass_inv_H[NUM_CARRIERS_TIMES_2_FCC] = {0};
static q15_t ass_H_aux[NUM_CARRIERS_TIMES_2_FCC]; /* Used to store the channel to be inverted (which differs from ass_H for the first payload symbols) */

/* Array for storing scale factors */
/* ass_scale contains 1/2, 1/3, 1/4...1/17 in Q.15 */
const q15_t ass_scale[16] = {16384, 10923, 8192, 6554, 5461, 4681, 4096, 3641, 3277, 2979, 2731, 2521, 2341, 2186, 2048, 1928};

const uint8_t auc_zeros[CARR_BUFFER_LEN] = {0};
const uint8_t auc_ones[CARR_BUFFER_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Complex conjugate of the reference SYNCP in Q1.15. Distortion because of the fact that 1+j*0 is not representable is negligible. */
static const q15_t ass_syncp_conj_cenelec_a[NUM_CARRIERS_TIMES_2_CENELEC_A]
	= {23170, -23170, 30274, -12540, 32767, 0, 30274, 12540, 23170, 23170, 0, 32767, -23170, 23170, -30274, -12540, 12540, -30274, 30274, 12540,
	   -12540, 30274, -23170,
	   -23170, 30274, -12540, -12540, 30274, -12540, -30274, 23170, 23170, -30274, -12540, 30274, 12540, -30274,
	   -12540, 30274, 12540, -23170, -23170, 12540, 30274, 23170,
	   -23170, -32768, 0, 12540, 30274, 23170, -23170, -23170, -23170, -23170, 23170, 12540, 30274, 32767, 0,
	   23170, -23170, 12540, -30274, -12540, -30274, -23170, -23170, -30274,
	   -12540, -30274, -12540};

static const q15_t ass_syncm_conj_cenelec_a[NUM_CARRIERS_TIMES_2_CENELEC_A]
	= {-23170, 23170, -30274, 12540, -32768, 0, -30274, -12540, -23170, -23170, 0, -32768, 23170, -23170, 30274, 12540, -12540, 30274, -30274,
	   -12540, 12540, -30274, 23170, 23170,
	   -30274, 12540, 12540, -30274, 12540, 30274, -23170, -23170, 30274, 12540, -30274, -12540, 30274, 12540,
	   -30274, -12540, 23170, 23170, -12540, -30274, -23170, 23170, 32767, 0,
	   -12540, -30274, -23170, 23170, 23170, 23170, 23170, -23170, -12540, -30274, -32768, 0, -23170, 23170,
	   -12540, 30274, 12540, 30274, 23170, 23170, 30274, 12540, 30274, 12540};

static const q15_t ass_syncp_conj_fcc[NUM_CARRIERS_TIMES_2_FCC]
	= {23170, -23170, 30274, -12540, 30274, -12540, 32767, 0, 32767, 0, 30274, 12540, 23170, 23170, 0, 32767, -12540, 30274, -30274, 12540, -30274,
	   -12540, 0, -32768, 30274, -12540,
	   30274, 12540, 0, 32767, -30274, 12540, -12540, -30274, 30274, -12540, 23170, 23170, -23170, 23170,
	   -12540, -30274, 32767, 0, 0, 32767, -23170, -23170, 30274, -12540, 0, 32767,
	   -23170, -23170, 32767, 0, -23170, 23170, 12540, -30274, 12540, 30274, -23170, -23170, 30274, 12540,
	   -30274, -12540, 32767, 0, -32768, 0, 32767, 0, -32768, 0, 30274, 12540, -23170,
	   -23170, 23170, 23170, 0, -32768, -12540, 30274, 23170, -23170, -32768, 0, 23170, 23170, 12540, -30274,
	   -30274, 12540, 30274, 12540, 12540, -30274, -32768, 0, 12540, 30274, 30274,
	   -12540, -12540, -30274, -30274, 12540, 12540, 30274, 30274, -12540, 0, -32768, -30274, -12540, -23170,
	   23170, 12540, 30274, 30274, 12540, 30274, -12540, 12540, -30274, 0, -32768,
	   -12540, -30274, -30274, -12540, -30274, -12540, -32768, 0, -30274, 12540, -23170, 23170, -23170, 23170};

static const q15_t ass_syncm_conj_fcc[NUM_CARRIERS_TIMES_2_FCC]
	= {-23170, 23170, -30274, 12540, -30274, 12540, -32768, 0, -32768, 0, -30274, -12540, -23170, -23170, 0, -32768, 12540, -30274, 30274, -12540,
	   30274, 12540, 0, 32767, -30274, 12540,
	   -30274, -12540, 0, -32768, 30274, -12540, 12540, 30274, -30274, 12540, -23170, -23170, 23170, -23170,
	   12540, 30274, -32768, 0, 0, -32768, 23170, 23170, -30274, 12540, 0, -32768,
	   23170, 23170, -32768, 0, 23170, -23170, -12540, 30274, -12540, -30274, 23170, 23170, -30274, -12540,
	   30274, 12540, -32768, 0, 32767, 0, -32768, 0, 32767, 0, -30274, -12540, 23170,
	   23170, -23170, -23170, 0, 32767, 12540, -30274, -23170, 23170, 32767, 0, -23170, -23170, -12540, 30274,
	   30274, -12540, -30274, -12540, -12540, 30274, 32767, 0, -12540, -30274,
	   -30274, 12540, 12540, 30274, 30274, -12540, -12540, -30274, -30274, 12540, 0, 32767, 30274, 12540, 23170,
	   -23170, -12540, -30274, -30274, -12540, -30274, 12540, -12540, 30274, 0,
	   32767, 12540, 30274, 30274, 12540, 30274, 12540, 32767, 0, 30274, -12540, 23170, -23170, 23170, -23170};

static const q15_t ass_syncp_conj_arib[NUM_CARRIERS_TIMES_2_ARIB]
	= {23170, -23170, 30274, -12540, 30274, -12540, 32767, 0, 30274,
	   12540, 23170, 23170, 12540, 30274, -12540, 30274, -30274, 12540, -23170, -23170, 12540, -30274,
	   32767, 0, 0, 32767,
	   -30274, 12540, 0, -32768, 32767, 0, 0, 32767, -23170, -23170, 30274, -12540, 0, 32767, -23170,
	   -23170, 30274, 12540,
	   -30274, 12540, 23170, -23170, -12540, 30274, 0, -32768, 0, 32767, 0, -32768, 0, 32767, 12540,
	   -30274, -23170, 23170,
	   30274, -12540, -30274, -12540, 23170, 23170, 12540, -30274, -30274, 12540, 30274, 12540, 12540,
	   -30274, -32768, 0,
	   12540, 30274, 30274, -12540, 0, -32768, -32768, 0, -12540, 30274, 23170, 23170, 30274, -12540,
	   12540, -30274, 0,
	   -32768, -23170, -23170, -30274, -12540, -32768, 0, -30274, 12540, -23170, 23170, -23170, 23170};

static const q15_t ass_syncm_conj_arib[NUM_CARRIERS_TIMES_2_ARIB]
	= {-23170, 23170, -30274, 12540, -30274, 12540, -32768, 0, -30274,
	   -12540, -23170, -23170, -12540, -30274, 12540, -30274, 30274, -12540, 23170, 23170, -12540, 30274,
	   -32768, 0, 0, -32768,
	   30274, -12540, 0, 32767, -32768, 0, 0, -32768, 23170, 23170, -30274, 12540, 0, -32768, 23170,
	   23170, -30274, -12540, 30274,
	   -12540, -23170, 23170, 12540, -30274, 0, 32767, 0, -32768, 0, 32767, 0, -32768, -12540, 30274,
	   23170, -23170, -30274, 12540,
	   30274, 12540, -23170, -23170, -12540, 30274, 30274, -12540, -30274, -12540, -12540, 30274, 32767,
	   0, -12540, -30274, -30274,
	   12540, 0, 32767, 32767, 0, 12540, -30274, -23170, -23170, -30274, 12540, -12540, 30274, 0, 32767,
	   23170, 23170, 30274, 12540,
	   32767, 0, 30274, -12540, 23170, -23170, 23170, -23170};

const q15_t *pss_syncp_conj;
const q15_t *pss_syncm_conj;

/* Declaration of assembler functions */
extern void swap_bytes_asm(int16_t *pss_input_array, uint16_t us_num_elements);
extern void invert_channel_asm(uint8_t uc_length_fraccional, uint8_t uc_used_carriers, q15_t *ass_input_channel);
extern void energy_vector_q31(int32_t *pss_input_array, uint8_t uc_num_complex_values, int64_t *psll_output);
extern void compute_weights_asm(uint8_t uc_num_values);
extern void zero_complex_vector_q_asm(uint8_t *psc_address, uint16_t us_num_bytes_in_vector);
extern void shift_add_shift_asm(int16_t *pss_input_array1, int16_t *pss_input_array2, uint8_t uc_num_complex_elements);
extern void scale_shift_add_shift_asm(int16_t *pss_input_array1, int16_t *pss_input_array2, uint8_t uc_num_complex_elements);
extern void cmplx_mag_squared_q15_result_q31_asm(q15_t *pss_input_symbol, q31_t *psl_output_symbol, uint8_t num_complex_elem);

#if (defined(SMOOTHING) || defined(UPDATE_CHN_SFO_EST_PAYLOAD))
extern void smooth_carriers_asm(uint8_t *puc_data_carriers_list, uint8_t uc_num_data_carriers, q15_t *pss_input_symbol);

#endif
#ifdef UPDATE_CHN_SFO_EST_PAYLOAD
extern void dem_pilots_and_update_chan_as(uint8_t *puc_pilots_per_symbol, q15_t *pss_input_symbol,
		q15_t *pss_modulating_symbols, uint8_t uc_num_pilots_per_block);
extern void compute_pilot_position_cenelec_a_as(uint8_t uc_num_symbols, uint8_t uc_num_pilots, uint8_t uc_index_first_symbol);
extern void compute_pilot_position_fcc_arib_as(uint8_t uc_num_symbols, uint8_t uc_num_pilots, uint8_t uc_index_first_symbol);
extern void compute_lfsr_sequence_pilots_as(uint8_t *auc_state_carrier_asm, uint8_t *auc_pn_seq_pilots, uint8_t uc_num_symbols, uint8_t uc_protocol_carriers);

#endif

/* Defines division of Q(16-uc_q_frac values).uc_q_frac values and Q(32-uc_q_frac values).uc_q_frac */
/* div_real_q(dividend,divisor,quotient), the sum of divisor >>1 is for rounding instead of truncating. */
#define div_real_q(sl_dividend, sl_divisor, uc_q_frac, psl_quotient) do { \
		*psl_quotient = ((((int64_t)sl_dividend) << uc_q_frac) + (((int64_t)sl_divisor) >> 1)) / ((int64_t)sl_divisor);	\
} \
	while (0)

/* Defines fixed point multiplication of real values in Quc_q_int.uc_q_frac using half-up rounding. */
#define mult_real_q(sl_a, sl_b, uc_q_frac)  (int32_t)((((int64_t)sl_a * (int64_t)sl_b) + (1 << (uc_q_frac - 1))) >> uc_q_frac)

/* Inverts the byte order of the int16_t elements of array. */

/*static inline void _invert_byte_order(int16_t *pss_input_array, uint16_t us_num_elements)
 * {
 *      uint16_t us_i;
 *      for (us_i = 0; us_i < us_num_elements; us_i++) {
 *(pss_input_array + us_i) = (*(pss_input_array + us_i)  << 8) | ((*(pss_input_array + us_i) >> 8) & 0x00FF);
 *      }
 * }*/

/*Obtains the conjugate of the complex values transmitted in a given symbol.
 */
void obtain_modulating_conj_complex_values(uint8_t *puc_input_bits, uint8_t uc_num_bits, uint8_t uc_frame_part, uint8_t *puc_carrier_list,
		q15_t *pss_modulating_symbol)
{
	uint8_t uc_i, uc_byte_index, uc_bit_index, uc_bit1, uc_bit2, uc_carrier_index;

	if (uc_working_band == WB_FCC) {
		pss_syncp_conj = ass_syncp_conj_fcc;
		pss_syncm_conj = ass_syncm_conj_fcc;
	} else if (uc_working_band == WB_ARIB) {
		pss_syncp_conj = ass_syncp_conj_arib;
		pss_syncm_conj = ass_syncm_conj_arib;
	} else {
		pss_syncp_conj = ass_syncp_conj_cenelec_a;
		pss_syncm_conj = ass_syncm_conj_cenelec_a;
	}

	if (uc_frame_part == PAYLOAD_PART) { /* for pilots */
		for (uc_i = 0; uc_i < (uc_num_bits / 2); uc_i++) {
			uc_byte_index = 2 * uc_i >> 3;
			uc_bit_index = (2 * uc_i - (uc_byte_index << 3));
			uc_bit1 = *(puc_input_bits + uc_byte_index) & (0x01 << uc_bit_index);
			uc_byte_index = (2 * uc_i + 1) >> 3;
			uc_bit_index = ((2 * uc_i + 1) - (uc_byte_index << 3));
			uc_bit2 = *(puc_input_bits + uc_byte_index) & (0x01 << uc_bit_index);

			/* pilots are indexed relative to Mactive but pss_syncp_conj is indexed from 0 to s_band_constants.uc_num_carriers-1 */
			uc_carrier_index = auc_unmasked_carrier_list[*(puc_carrier_list + uc_i)];
			if (uc_bit1 == 0) {
				if (uc_bit2 == 0) {
					*(pss_modulating_symbol + 2 * uc_i) = pss_syncp_conj[2 * uc_carrier_index];
					*(pss_modulating_symbol + 2 * uc_i + 1) = pss_syncp_conj[2 * uc_carrier_index + 1];
				} else {
					*(pss_modulating_symbol + 2 * uc_i) = pss_syncm_conj[2 * uc_carrier_index + 1];
					*(pss_modulating_symbol + 2 * uc_i + 1) = pss_syncp_conj[2 * uc_carrier_index];
				}
			} else {
				if (uc_bit2 == 0) {
					*(pss_modulating_symbol + 2 * uc_i) = pss_syncp_conj[2 * uc_carrier_index + 1];
					*(pss_modulating_symbol + 2 * uc_i + 1) = pss_syncm_conj[2 * uc_carrier_index];
				} else {
					*(pss_modulating_symbol + 2 * uc_i) = pss_syncm_conj[2 * uc_carrier_index];
					*(pss_modulating_symbol + 2 * uc_i + 1) = pss_syncm_conj[2 * uc_carrier_index + 1];
				}
			}
		}
	} else { /* for preamble, S1S2, and FCH */
		for (uc_i = 0; uc_i < uc_num_bits; uc_i++) {
			uc_byte_index = uc_i >> 3;
			uc_bit_index = 7 - uc_i + (uc_byte_index << 3); /* The msb is the first bit */
			uc_bit1 = *(puc_input_bits + uc_byte_index) & (0x01 << uc_bit_index);

			/* in the preamble, S1S2 and the FCH the function is called with the vector of carrier indexes in auc_unmasked_carrier_list */
			uc_carrier_index = *(puc_carrier_list + uc_i);
			if (uc_bit1 == 0) {
				*(pss_modulating_symbol + 2 * uc_i) = pss_syncp_conj[2 * uc_carrier_index];
				*(pss_modulating_symbol + 2 * uc_i + 1) = pss_syncp_conj[2 * uc_carrier_index + 1];
			} else {
				*(pss_modulating_symbol + 2 * uc_i) = pss_syncm_conj[2 * uc_carrier_index];
				*(pss_modulating_symbol + 2 * uc_i + 1) = pss_syncm_conj[2 * uc_carrier_index + 1];
			}
		}
	}
}

/*Estimates the energy of the noise in the symbols stored in the static array ass_pm_fch_s1s2 wrt its average value (in Q1.31) stored in the static array
 * asl_average_symbol.
 * Estimation is done between modulus, not between complex values. This avoids the effect of the SFO.
 * Inputs:
 * uint8_t uc_num_symbols: number of symbols whose noise energy has to be computed
 * Outputs:
 * The energy of the noise is stored in the static array asl_Ni.
 * The minimum value of the asl_Ni
 */
static inline void _compute_symbol_noise(q15_t *pss_average_symbol, uint8_t uc_num_symbols)
{
	uint8_t uc_i, uc_scaling = 7;
	uint64_t ull_max;
	q63_t ll_energy_max = 0;

	sl_Ni_min = MAX_INT32;
	if (uc_num_symbols > 0) {
		/* Magnitude of the reference vector */
		cmplx_mag_squared_q15_result_q31_asm(pss_average_symbol, u_shared_buffers.s_chn_est.asl_squared_mag_symbol, uc_used_carriers);

		/* Computes the energy of the noise in each symbol */
		ll_energy_max = 0;
		for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
			cmplx_mag_squared_q15_result_q31_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i),
					u_shared_buffers.s_chn_est.asl_squared_mag_symbol_1, uc_used_carriers);

			arm_sub_q31(u_shared_buffers.s_chn_est.asl_squared_mag_symbol_1, u_shared_buffers.s_chn_est.asl_squared_mag_symbol,
					asl_symbol_aux, uc_used_carriers); /* Difference of two Q3.29 positive numbers. Max value is <4 */

			/*arm_power_q31(asl_symbol_aux, uc_used_carriers, (asll_Ni + uc_i)); */
			energy_vector_q31(asl_symbol_aux, uc_used_carriers, (asll_Ni + uc_i)); /* asll_Ni is in Q16.48 but max is 2*uc_used_carriers<2^8 */
			if (*(asll_Ni + uc_i) > ll_energy_max) { /* finds the maximum energy value */
				ll_energy_max = *(asll_Ni + uc_i);
			}
		}

		/* To minimize precision loss, scaling value so that the maximum is >0.5 and <1 */
		ull_max = 0x0080000000000000ULL;
		while (!(ll_energy_max & ull_max) && (uc_scaling < FRAC_PART_Q63)) {
			uc_scaling++;
			ull_max = ull_max >> 1;
		}

		if (uc_scaling > FRAC_PART_Q31) {
			for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
				*(asl_Ni + uc_i) = (q31_t)(*(asll_Ni + uc_i) << (uc_scaling - FRAC_PART_Q31 - 1));

				if (*(asl_Ni + uc_i) < sl_Ni_min) {
					sl_Ni_min = *(asl_Ni + uc_i);
				}
			}
		} else {
			for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
				*(asl_Ni + uc_i) = (q31_t)(*(asll_Ni + uc_i) >> (FRAC_PART_Q31 - uc_scaling + 1));

				if (*(asl_Ni + uc_i) < sl_Ni_min) {
					sl_Ni_min = *(asl_Ni + uc_i);
				}
			}
		}
	}
}

/*Calculates the list of carriers that must be smoothed in a set of symbols.
 * It can be used both in the payload and in the preamble and fch. When used in the preamble and fch,
 * the input puc_static_and_dynamic_map should be fixed to the tone mask and puc_pilot_map to all 0.
 * When used in the payload, puc_static_and_dynamic_map should contain the static and dynamic map and puc_pilot_map
 * must contain a 1 in the carriers used as pilots.
 * The returned data carrier indexes range from 0 to uc_used_carriers-1.
 *      IMPORTANT:
 *              -It has been assumed that s_band_constants.uc_first_carrier>8
 *      Inputs:
 *              -uint8_t *puc_static_and_dynamic_map: array of 128 values. Contains 1 when the carrier is affected by static or dynamic notching and 0 elsewhere
 *              -uint8_t *puc_pilot_map: array of values with the set of carrier indexes used as pilots
 * Output:
 *      -*puc_data_carriers_list: pointer to the list of data carriers
 *      -uc_num_data_carrier: number of data carriers in the list
 *      -*puc_pilot_carriers_list: pointer to the list of pilot carriers
 *      -uc_num_pilot_carriers: number of pilots in the list
 *
 */
void control_smooth_and_order_pilot_list(uint8_t *puc_static_and_dynamic_map, uint8_t *puc_pilot_map, uint8_t *puc_data_carriers_list,
		uint8_t *puc_num_data_carrier, uint8_t *puc_pilot_carriers_list, uint8_t *puc_control_avg_invert_chan,
		uint8_t uc_num_pilots_per_symbol, uint8_t uc_index_first_symbol_block)
{
	uint8_t uc_i, uc_byte_index_i, uc_bit_index_in_byte_i, uc_byte_index_i_1, uc_bit_index_in_byte_i_1, uc_byte_index_i_plus_1,
			uc_bit_index_in_byte_i_plus_1;
	uint8_t uc_value_i_1, uc_value_i, uc_value_i_plus_1, uc_num_inactive_carriers = 0, uc_num_notched_carriers = 0;

	#ifdef UPDATE_CHN_SFO_EST_PAYLOAD
	if (uc_working_band == WB_CENELEC_A) {
		memset(auc_num_pilots_in_symbol, 0, NUM_SYM_PILOTS_H_EST * sizeof(uint8_t));
	}

	#else
	UNUSED(puc_pilot_carriers_list);
	UNUSED(uc_num_pilots_per_symbol);
	UNUSED(uc_index_first_symbol_block);
	#endif

	for (uc_i = s_band_constants.uc_first_carrier; uc_i < (s_band_constants.uc_last_carrier + 1); uc_i++) {
		if (*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) == 0) {
			uc_byte_index_i = uc_i >> 3;
			uc_bit_index_in_byte_i = uc_i & 0x07;
			uc_byte_index_i_1 = (uc_i - 1) >> 3;
			uc_byte_index_i_plus_1 = (uc_i + 1) >> 3;
			uc_bit_index_in_byte_i_1 = (uc_i - 1) & 0x07;
			uc_bit_index_in_byte_i_plus_1 = (uc_i + 1) & 0x07;

			/* Determines the values of the actual carrier "i", the previous one "i-1" and the subsequent "i+1" in puc_static_and_dynamic_map */
			uc_value_i_1 = (*(puc_static_and_dynamic_map  + uc_byte_index_i_1) >> uc_bit_index_in_byte_i_1) &  0x01;
			uc_value_i = (*(puc_static_and_dynamic_map  + uc_byte_index_i) >> uc_bit_index_in_byte_i) &  0x01;
			uc_value_i_plus_1 = (*(puc_static_and_dynamic_map  + uc_byte_index_i_plus_1) >> uc_bit_index_in_byte_i_plus_1) &  0x01;

			/* If first or last carrier, the fact that they are notched is not reflected in puc_static_and_dynamic_map */
			if (uc_i == s_band_constants.uc_first_carrier) {
				uc_value_i_1 = 1;
			} else if (uc_i == s_band_constants.uc_last_carrier) {
				uc_value_i_plus_1 = 1;
			}

			/* Determines whether a carrier has valid predecessor and sucessor */
			if (uc_value_i == 0) {
				if (uc_value_i_1 == 0) {
					if (uc_value_i_plus_1 == 0) {
						*(puc_data_carriers_list + *puc_num_data_carrier) = uc_i - uc_num_inactive_carriers - s_band_constants.uc_first_carrier;
						(*puc_num_data_carrier)++;

						*(puc_control_avg_invert_chan + uc_i - s_band_constants.uc_first_carrier - uc_num_inactive_carriers) = 1;
					} else {
						*(puc_control_avg_invert_chan + uc_i - s_band_constants.uc_first_carrier - uc_num_inactive_carriers) = 2;
					}
				} else {
					*(puc_control_avg_invert_chan + uc_i - s_band_constants.uc_first_carrier - uc_num_inactive_carriers) = 0;
				}
			} else {
				uc_num_inactive_carriers++;
				/* Check for masked carrier */
				uc_value_i = (*(auc_static_notching_pos  + uc_byte_index_i) >> uc_bit_index_in_byte_i) &  0x01;
				if (uc_value_i == 1) {
					uc_num_notched_carriers++;
				}
			}
		}

	#ifdef UPDATE_CHN_SFO_EST_PAYLOAD
		else {
			if (uc_working_band == WB_CENELEC_A) {
				if (*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) == 1) {
					*(puc_pilot_carriers_list + auc_num_pilots_in_symbol[0]) = uc_i - uc_num_inactive_carriers - s_band_constants.uc_first_carrier;
					auc_pilot_carriers_list_absolute[auc_num_pilots_in_symbol[0]] =         uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier;
					aus_pilot_stored[uc_index_set][uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier] = uc_index_first_symbol_block + 1;
					auc_num_pilots_in_symbol[0]++;
				} else if ((*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) > 1) && (*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) < 4)) {
					*(puc_pilot_carriers_list + uc_num_pilots_per_symbol + auc_num_pilots_in_symbol[1]) = uc_i - uc_num_inactive_carriers - s_band_constants.uc_first_carrier;
					auc_pilot_carriers_list_absolute[uc_num_pilots_per_symbol +
					auc_num_pilots_in_symbol[1]] = uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier;
					aus_pilot_stored[uc_index_set][uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier] = uc_index_first_symbol_block + 2;
					auc_num_pilots_in_symbol[1]++;
				} else if ((*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) >= 4) && (*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) < 8)) {
					*(puc_pilot_carriers_list + uc_num_pilots_per_symbol * 2 +
					auc_num_pilots_in_symbol[2]) = uc_i - uc_num_inactive_carriers - s_band_constants.uc_first_carrier;
					auc_pilot_carriers_list_absolute[uc_num_pilots_per_symbol * 2 +
					auc_num_pilots_in_symbol[2]] = uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier;
					aus_pilot_stored[uc_index_set][uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier] = uc_index_first_symbol_block + 3;
					auc_num_pilots_in_symbol[2]++;
				} else if ((*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) >= 8) && (*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) < 16)) {
					*(puc_pilot_carriers_list + uc_num_pilots_per_symbol * 3 +
					auc_num_pilots_in_symbol[3]) = uc_i - uc_num_inactive_carriers - s_band_constants.uc_first_carrier;
					auc_pilot_carriers_list_absolute[uc_num_pilots_per_symbol * 3 +
					auc_num_pilots_in_symbol[3]] = uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier;
					aus_pilot_stored[uc_index_set][uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier] = uc_index_first_symbol_block + 4;
					auc_num_pilots_in_symbol[3]++;
				} else if ((*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) >= 16) && (*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) < 32)) {
					*(puc_pilot_carriers_list + uc_num_pilots_per_symbol * 4 +
					auc_num_pilots_in_symbol[4]) = uc_i - uc_num_inactive_carriers - s_band_constants.uc_first_carrier;
					auc_pilot_carriers_list_absolute[uc_num_pilots_per_symbol * 4 +
					auc_num_pilots_in_symbol[4]] = uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier;
					aus_pilot_stored[uc_index_set][uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier] = uc_index_first_symbol_block + 5;
					auc_num_pilots_in_symbol[4]++;
				} else if (*(puc_pilot_map + uc_i - s_band_constants.uc_first_carrier) >= 32) {
					*(puc_pilot_carriers_list + uc_num_pilots_per_symbol * 5 +
					auc_num_pilots_in_symbol[5]) = uc_i - uc_num_inactive_carriers - s_band_constants.uc_first_carrier;
					auc_pilot_carriers_list_absolute[uc_num_pilots_per_symbol * 5 +
					auc_num_pilots_in_symbol[5]] = uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier;
					aus_pilot_stored[uc_index_set][uc_i - uc_num_notched_carriers - s_band_constants.uc_first_carrier] = uc_index_first_symbol_block + 6;
					auc_num_pilots_in_symbol[5]++;
				}
			}
		}
	#else
		UNUSED(uc_num_notched_carriers);
  #endif
	}

	/* The first carrier does not has valid predecessor */
	*puc_control_avg_invert_chan = 0;
	/* The last carrier does not has valid sucessor */
	*(puc_control_avg_invert_chan + s_band_constants.uc_last_carrier - s_band_constants.uc_first_carrier) = 2;
}

/*
 * #if (defined(SMOOTHING) || defined(UPDATE_CHN_SFO_EST_PAYLOAD))
 *
 * Applies smoothing to a channel response estimate at the specified carriers.
 * The value in each carrier is obtained by averaging the previous estimates in the carrier with the information from the adjacent ones.
 * Input:
 *      -puc_data_carriers_list: pointer to the list of data carriers indexes
 *      -uc_num_data_carriers: number of data carriers
 *      -pss_symbol: pointer to the input symbol
 * Output:
 *      -pss_symbol: pointer to the smoothed symbol
 * /
 * void smooth_carriers(uint8_t *puc_data_carriers_list, uint8_t uc_num_data_carriers, q15_t *pss_symbol, uint8_t uc_scale_control)
 * {
 *      uint8_t uc_i, uc_carrier_index;
 *
 *      / * Copy the array with the input estimate to the array with the final estimate * /
 *      arm_copy_q15(pss_symbol, ass_symbol_aux_1, 2 * uc_used_carriers);
 *
 *      if (uc_scale_control == SCALE_BEFORE_SUM) {
 *              / * Scale the values in ass_H by 1/3. * /
 *              arm_scale_q15(pss_symbol, VALUE_1_3_Q15, 0, ass_symbol_aux, 2 * uc_used_carriers);
 *
 *              / * First and last carriers are always set to the input value (no smoothing) * /
 *              / * Applies smoothing to the remaining carriers * /
 *              for (uc_i = 0; uc_i < uc_num_data_carriers; uc_i++) {
 *                      uc_carrier_index = *(puc_data_carriers_list + uc_i);
 *
 *                      / * Real part * /
 *                      ass_symbol_aux_1[2 *
 *                      uc_carrier_index]
 *                              = ass_symbol_aux [2 * uc_carrier_index -
 *                                      2]  + ass_symbol_aux [2 * uc_carrier_index]  + ass_symbol_aux [2 * uc_carrier_index + 2];
 *
 *                      / * Imag part * /
 *                      ass_symbol_aux_1[2 * uc_carrier_index +
 *                      1] = ass_symbol_aux [2 * uc_carrier_index - 1] + ass_symbol_aux [2 * uc_carrier_index + 1] + ass_symbol_aux [2 * uc_carrier_index + 3];
 *              }
 *      } else {
 *              arm_copy_q15(pss_symbol, ass_symbol_aux, 2 * uc_used_carriers);
 *
 *              / * First and last carriers are always set to the input value (no smoothing) * /
 *              / * Applies smoothing to the remaining carriers * /
 *              for (uc_i = 0; uc_i < uc_num_data_carriers; uc_i++) {
 *                      uc_carrier_index = *(puc_data_carriers_list + uc_i);
 *
 *                      / * Real part * /
 *                      ass_symbol_aux_1[2 *
 *                      uc_carrier_index]
 *                              = mult_real_q((ass_symbol_aux [2 * uc_carrier_index -
 *                                      2]  + ass_symbol_aux [2 * uc_carrier_index]  + ass_symbol_aux [2 * uc_carrier_index + 2]), VALUE_1_3_Q15,
 *                                      FRAC_PART_Q15);
 *
 *                      / * Imag part * /
 *                      ass_symbol_aux_1[2 * uc_carrier_index +
 *                      1]
 *                              = mult_real_q((ass_symbol_aux [2 * uc_carrier_index -
 *                                      1] + ass_symbol_aux [2 * uc_carrier_index + 1] + ass_symbol_aux [2 * uc_carrier_index + 3]), VALUE_1_3_Q15,
 *                                      FRAC_PART_Q15);
 *              }
 *      }
 *
 *      / * Copy the result back to ass_H * /
 *      arm_copy_q15(ass_symbol_aux_1, pss_symbol, 2 * uc_used_carriers);
 * }
 *
 * #endif
 */

/*Computes the inverse of the channel estimate stored in ass_H and stores the inverse in ass_inv_H
 * Inputs:
 * uc_length_fraccional: number of bits of the fractional part of the output
 */

/*
 * static inline void _invert_channel(uint8_t uc_length_fraccional)
 * {
 *      uint8_t uc_i;
 *      q63_t sll_num;
 *      q31_t sl_den;
 *      int64_t sll_overflow_pos = 1LL << (FRAC_PART_Q31 + (FRAC_PART_Q15 - uc_length_fraccional));
 *
 *      arm_q15_to_q31(ass_H, asl_symbol_aux, 2 * uc_used_carriers);
 *      / *invert_channel_asm();* /
 *
 *      for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
 *              / * If Re=0 and Im=0, it is fixed to the average of previous and following carriers* /
 *              if (!((asl_symbol_aux[2 * uc_i]) || (asl_symbol_aux[2 * uc_i + 1]))) {
 *                      if (auc_control_avg_invert_chan[uc_i] == 0) {
 *                              asl_symbol_aux[2 * uc_i] = asl_symbol_aux[2 * (uc_i + 1)];
 *                              asl_symbol_aux[2 * uc_i + 1] = asl_symbol_aux[2 * (uc_i + 1) + 1];
 *                      } else if (auc_control_avg_invert_chan[uc_i] == 2) {
 *                              asl_symbol_aux[2 * uc_i] = asl_symbol_aux[2 * (uc_i - 1)];
 *                              asl_symbol_aux[2 * uc_i + 1] = asl_symbol_aux[2 * (uc_i - 1) + 1];
 *                      } else {
 *                              asl_symbol_aux[2 * uc_i] = (asl_symbol_aux[2 * (uc_i - 1)] >> 1) + (asl_symbol_aux[2 * (uc_i + 1)] >> 1);
 *                              asl_symbol_aux[2 * uc_i + 1] = (asl_symbol_aux[2 * (uc_i - 1) + 1] >> 1) + (asl_symbol_aux[2 * (uc_i + 1) + 1] >> 1);
 *                      }
 *              }
 *
 *              / * If it still zero, its inverse will be fixed the maximum value* /
 *              if ((asl_symbol_aux[2 * uc_i]) || (asl_symbol_aux[2 * uc_i + 1])) {
 *                      sl_den = mult_real_q((asl_symbol_aux[2 * uc_i] >> 1), (asl_symbol_aux[2 * uc_i] >> 1), FRAC_PART_Q31)  +
 *                                      mult_real_q((asl_symbol_aux[2 * uc_i + 1] >> 1), (asl_symbol_aux[2 * uc_i + 1] >> 1), FRAC_PART_Q31);
 *                      / * Q3.29. Scaling must be done before square to avoid problems when (MIN_INT32)^2+(MIN_INT32)^2 * /
 *                      / * Real part* /
 *                      div_real_q(asl_symbol_aux[2 * uc_i], sl_den, FRAC_PART_Q29, &sll_num); / * Q1.31 * /
 *                      if (sll_num >= sll_overflow_pos) {
 *                              ass_inv_H[2 * uc_i] = MAX_INT16;
 *                      } else if (sll_num <= -sll_overflow_pos) {
 *                              ass_inv_H[2 * uc_i] = MIN_INT16;
 *                      } else {
 *                              ass_inv_H[2 * uc_i] = (int16_t)(sll_num >> (FRAC_PART_Q31 - uc_length_fraccional));
 *                      }
 *
 *                      / * Imag part* /
 *                      div_real_q(-asl_symbol_aux[2 * uc_i + 1], sl_den, FRAC_PART_Q29, &sll_num);
 *
 *                      if (sll_num >= sll_overflow_pos) {
 *                              ass_inv_H[2 * uc_i + 1] = MIN_INT16;
 *                      } else if (sll_num <= -sll_overflow_pos) {
 *                              ass_inv_H[2 * uc_i + 1] = MAX_INT16;
 *                      } else {
 *                              ass_inv_H[2 * uc_i + 1] = (int16_t)(sll_num >> (FRAC_PART_Q31 - uc_length_fraccional));
 *                      }
 *              } else {
 *                      / * If real and imag part are still zero, they are assigned the max value* /
 *                      ass_inv_H[2 * uc_i] = MAX_INT16;
 *                      ass_inv_H[2 * uc_i + 1] = MAX_INT16;
 *              }
 *      }
 * }
 */

/* Compensates the phase shift caused by the SFO in the channel estimate obtained from the preamble, FCH and S1S2. */
/* Both the compensated channel and its inverse are moved to the ATPL */
void compensate_sfo_in_chan_est(uint8_t uc_channel_to_be_applied)
{
	if (uc_channel_to_be_applied == CHANNEL_BEFORE_CP_CHANGE) {
		/* Compensates the sfo in the estimated channel that will be applied before the FFT window is adjusted */
		/* Result is Q3.13 scaled by 1/2 (because ass_rotation_sfo is scaled by 1/2) */
		arm_cmplx_mult_cmplx_q15(ass_H, pss_rotation_sfo_first, ass_H_aux, uc_used_carriers);
		/* Result is Q1.15. The +1 is to compensate for the scaling in ass_rotation_sfo */
		arm_shift_q15(ass_H_aux, Q3_13_TO_Q1_15 + 1, ass_H_aux, 2 * uc_used_carriers);

		/* Inverts channel, scales by GAIN_SCALE_INV_H_FEQ and swap bytes */
		invert_channel_asm(NUM_BITS_FRAC_PART_INV_H, uc_used_carriers, ass_H_aux);
	} else if (uc_channel_to_be_applied == CHANNEL_AFTER_CP_CHANGE) {
		/* Compensates the sfo in the estimated channel that will be applied after the FFT window is adjusted */
		/* Result is Q3.13 scaled by 1/2 (because ass_rotation_sfo is scaled by 1/2) */
		arm_cmplx_mult_cmplx_q15(ass_H, pss_rotation_sfo_second, ass_H, uc_used_carriers);
		/* Result is Q1.15. The +1 is to compensate for the scaling in ass_rotation_sfo */
		arm_shift_q15(ass_H, Q3_13_TO_Q1_15 + 1, ass_H, 2 * uc_used_carriers);

		/* Inverts channel, scales by GAIN_SCALE_INV_H_FEQ and swap bytes */
		invert_channel_asm(NUM_BITS_FRAC_PART_INV_H, uc_used_carriers, ass_H);
	} else if ((uc_working_band != WB_CENELEC_A) && (uc_channel_to_be_applied == CHANNEL_FOR_SHORT_PAYLOADS)) {
		/* This is only done in FCC and ARIB when the payload is short */
		/* Inverts channel, scales by GAIN_SCALE_INV_H_FEQ and swap bytes */
		invert_channel_asm(NUM_BITS_FRAC_PART_INV_H, uc_used_carriers, ass_H);
	} else {
		/* This is done only in FCC, where the FCH is coherently modulated */
		if (uc_working_band == WB_FCC) {
			/* Compensates the sfo in the estimated channel that will be applied to the FCH in FCC legacy mode */
			/* Result is Q3.13 scaled by 1/2 (because ass_rotation_sfo is scaled by 1/2) */
			arm_cmplx_mult_cmplx_q15(ass_H, pss_rotation_sfo_first, ass_H, uc_used_carriers);
			/* Result is Q1.15. The +1 is to compensate for the scaling in ass_rotation_sfo */
			arm_shift_q15(ass_H, Q3_13_TO_Q1_15 + 1, ass_H, 2 * uc_used_carriers);

			/* Inverts channel, scales by GAIN_SCALE_INV_H_FEQ and swap bytes */
			invert_channel_asm(NUM_BITS_FRAC_PART_INV_H, uc_used_carriers, ass_H);
		}
	}

	/* Moves the inverse of the channel to symbol 0 of the ATPL */
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, 0x00); /* Set destination to symbol 0 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier);
	pplc_if_write_jump((BCODE_ZONE2 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier)), (uint8_t *)ass_inv_H, 2 * uc_used_carriers * 2, JUMP_COL_2);
}

void chn_estimation_from_preamble(void)
{
	uint8_t uc_i;
	uint8_t uc_num_p_sym;

	/* Disable RSSI calculation for channel estimation */
	atpl250_disable_rssi();
	atpl250_disable_evm();

	/* Read how many P symbols are available */
	uc_num_sym_p_detected = ((pplc_if_read8(REG_ATPL250_SYNCM_CTL_L8) & 0x7C) >> 2) - 1;

	/* If there are more SYNCP symbols than NUM_SYM_H_EST_PRE-1, truncate it to use NUM_SYM_H_EST_PRE-1 */
	uc_num_p_sym = uc_num_sym_p_detected;
	if (uc_num_sym_p_detected > (NUM_SYM_H_EST_PRE - 1)) {
		uc_num_p_sym = NUM_SYM_H_EST_PRE - 1;
	}

	uc_num_sym_valid_preamble = uc_num_p_sym + 1;

	/* Block IOB, so no FCH symbols enter until finished */
	atpl250_clear_rx_mode();
	atpl250_clear_tx_mode();

	/* *********************************** Process SYNCPs ************************************** */
	/* Launch DMA read of first SYNCP (it is assumed that we have received, at least one SYNCP) */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier + ((CFG_SYMBOLS_IN_IOB - uc_num_p_sym) * CFG_IOB_SAMPLES_PER_SYMBOL));
	pplc_if_read_jump((BCODE_ZONE4 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier + ((CFG_SYMBOLS_IN_IOB - uc_num_p_sym) * CFG_IOB_SAMPLES_PER_SYMBOL))),
			(uint8_t *)u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols, 2 * uc_used_carriers * 2, JUMP_COL_2, false);

	/* Reset average symbol from previous frame.
	 * By now, symbols to be averaged are scaled by 1/2^2 (plus the 1/2^2 scale of the complex-by-complex product) to avoid overflow and summed.
	 * The division by the number of symbols is done at the end, once the number of FCH and S1S2 are known. */
	zero_complex_vector_q_asm((uint8_t *)ass_average_symbol, 2 * uc_used_carriers * sizeof(q15_t));

	/* Obtain modulating symbols. This is needed for the cases when tone mask exists. To extract values from conj_syncp. */
	obtain_modulating_conj_complex_values((uint8_t *)auc_zeros, uc_used_carriers, PREAMBLE_FCH_S1S2_PART, auc_unmasked_carrier_list,
			ass_modulating_symbols);

	/* Read first SYNCP */
	pplc_if_do_read((uint8_t *)u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols, 2 * uc_used_carriers * 2);

	/* Loop to read next SYNCP (if exist) and process the previous one*/
	for (uc_i = 1; uc_i < uc_num_p_sym; uc_i++) {
		/* Launch DMA read of the next SYNCP*/
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier + ((CFG_SYMBOLS_IN_IOB - uc_num_p_sym + uc_i) * CFG_IOB_SAMPLES_PER_SYMBOL));
		pplc_if_read_jump((BCODE_ZONE4 | (auc_unmasked_carrier_list[0]
				+ s_band_constants.uc_first_carrier + ((CFG_SYMBOLS_IN_IOB - uc_num_p_sym + uc_i) * CFG_IOB_SAMPLES_PER_SYMBOL))),
				(uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i),
				2 * uc_used_carriers * 2, JUMP_COL_2, false);

		/* Invert byte order of the previous SYNCP*/
		swap_bytes_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_i - 1)), uc_used_carriers);

		/* Multiply previous SYNCP by conjugate of the reference symbol */
		/* Result in Q3.13 */
		arm_cmplx_mult_cmplx_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_i - 1)),
				ass_modulating_symbols,
				(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_i - 1)), uc_used_carriers);

		/* Adds a scaled version of SYNCP to the average symbol and scaled SYNCP. */
		shift_add_shift_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_i - 1)),
				ass_average_symbol, uc_used_carriers);

		/* Read SYNCP */
		pplc_if_do_read((uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i), 2 * uc_used_carriers * 2);
	}

	/* *********************************** Process SYNCM and last SYNCP ******************************* */
	/* Launch DMA read of the SYNCM */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier);
	pplc_if_read_jump((BCODE_ZONE4 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier)),
			(uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym),
			2 * uc_used_carriers * 2, JUMP_COL_2, false);

	/* Invert byte order of the previous SYNCP*/
	swap_bytes_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_p_sym - 1)), uc_used_carriers);

	/* Multiply previous SYNCP by conjugate of the reference symbol */
	/* Result in Q3.13 */
	arm_cmplx_mult_cmplx_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_p_sym - 1)),
			ass_modulating_symbols, (u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_p_sym - 1)),
			uc_used_carriers);

	/* Adds a scaled version of SYNCP to the average symbol and scaled SYNCP */
	shift_add_shift_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_p_sym - 1)),
			ass_average_symbol, uc_used_carriers);

	/* Obtain modulating symbols. This is needed for the cases when tone mask exists. To extract values from conj_syncp */
	obtain_modulating_conj_complex_values((uint8_t *)auc_ones, uc_used_carriers, PREAMBLE_FCH_S1S2_PART, auc_unmasked_carrier_list,
			ass_modulating_symbols);

	/* Read SYNCM */
	pplc_if_do_read((uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), 2 * uc_used_carriers * 2);

	/* Invert byte order of the SYNCM*/
	swap_bytes_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), uc_used_carriers);

	/* Multiply received SYNCM by conjugate of the reference symbol */
	/* Result in Q3.13 */
	arm_cmplx_mult_cmplx_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), ass_modulating_symbols,
			(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), uc_used_carriers);

#ifdef SMOOTHING
	smooth_carriers_asm(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch,
			(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym));
#endif

	/* Adds a scaled version of SYNCM to the average symbol and scaled SYNCM */
	shift_add_shift_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym),
			ass_average_symbol, uc_used_carriers);

	if (uc_working_band == WB_FCC) {
		if (uc_legacy_mode) {
			/* Channel estimate should only be obtained in FCC with FCH coherently modulated*/
			/* ************************************ Estimation of the channel response by weighted averaging */
			/* Scales the average symbol and divides by the number of symbols. */
			/* The result is stored in asl_aux because the scaling for ass_average_symbol will depend on the number of FCH symbols */
			/* that will be averaged and if S1S2 symbols are present */
			arm_scale_q15(ass_average_symbol, ass_scale[uc_num_p_sym - 1], 4, ass_average_symbol_fch_fcc, 2 * uc_used_carriers);

			/* Computes the noise energy in each symbol */
			_compute_symbol_noise(ass_average_symbol_fch_fcc, uc_num_sym_valid_preamble); /* asl_symbol_aux is used inside*/

			/* Estimates the weighting factors. Instead of 1/Ni, to avoid overflow in the division, 2^-15/Ni is computed. */
			/*_compute_weights(uc_num_sym_valid_preamble, 3);*/
			compute_weights_asm(uc_num_sym_valid_preamble);

			/* Estimates the weighted averaging */
			/*memset(ass_H, 0, 2 * uc_used_carriers * sizeof(q15_t));*/
			zero_complex_vector_q_asm((uint8_t *)ass_H, 2 * uc_used_carriers * sizeof(q15_t));
			for (uc_i = 0; uc_i < uc_num_sym_valid_preamble; uc_i++) {
				/* Multiplies the LS estimates by their Wi and sum */
				/* Reuse of ass_symbols_aux */
				arm_scale_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + uc_i * 2 * uc_used_carriers),
						*(ass_Wi + uc_i), 0, ass_symbol_aux, 2 * uc_used_carriers);
				arm_add_q15(ass_symbol_aux, ass_H, ass_H, 2 * uc_used_carriers);
			}

			/* Estimates SFO and, in FCC legacy mode, the phase correction for the channel estimate during FCH */
			sampling_error_est_from_preamble_fch_s1s2(MOD_SCHEME_COHERENT, SFO_FOR_FCH_COH);

			/* Computes the inverse of the estimated channel in Q(16-NUM_BITS_FRAC_PART_INV_H).NUM_BITS_FRAC_PART_INV_H */
			invert_channel_asm(NUM_BITS_FRAC_PART_INV_H, uc_used_carriers, ass_H);

			/* Moves the result to the ATPL */
			pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, 0x00); /* Set destination to symbol 0 */
			pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier);
			pplc_if_write_jump((BCODE_ZONE2 | s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0]), (uint8_t *)ass_inv_H, 2 * uc_used_carriers * 2, JUMP_COL_2);

			/* Set reference to average P symbol */
			pplc_if_or8(REG_ATPL250_INOUTB_CONF2_H8, 0x01); /* SOURCE_H='1' */
			pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, 0x10); /* Set source to symbol 1, keep destination to symbol 0 //demod_diff_with_fixed_ref */
		} else {
			pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, 0xFE);
		}
	} else {
		pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, 0xFE);
	}

	/* Set rx mode again to unblock IOB */
	atpl250_set_rx_mode();

	/* Enable RSSI calculation from channel estimation */
	atpl250_enable_rssi();
	atpl250_enable_evm();
}

/**
 * \brief Estimate channel from symbols received FCH
 *
 */
void chn_estimation_from_fch(uint8_t uc_mod_scheme, uint8_t uc_num_sym_used_fch)
{
	uint8_t uc_j;

	uint8_t uc_i, uc_k;
	/*uint8_t uc_ichannel_idx;*/
	uint16_t us_bit_index;
	uint8_t uc_byte_index, uc_rel_bit_index;
	uint8_t uc_local_bit_index, uc_local_byte_index, uc_rel_local_bit_index, uc_local_buf_index;
	uint8_t auc_use_fch_symbol[SYMBOLS_8] = {1, 1, 1, 1, 1, 1, 1, 1}; /* ToDo: By now all FCH symbols are valid. Use only those with BER<Threshold */
	uint8_t uc_next_fch_computed;
	uint8_t uc_num_sym_h_est_fch;

	/* Launch data transmitted for channel estimation */
	pplc_if_write16(REG_ATPL250_INTERLEAVER_SPI_H16, 0x8000); /* SPI2INT_TXRX='1'; ADDR_SPI2INT=0 */
	pplc_if_read_jump(REG_ATPL250_INTERLEAVER_SPI_VL8, u_shared_buffers.s_fch_int.interleaver_matrix_tx, uc_fch_interleaver_byte_size, 1, false);

	/* Convert into a buffer for each symbol */
	uc_local_buf_index = 0;
	uc_local_bit_index = 0;
	uc_local_byte_index = 0;
	uc_rel_local_bit_index = 0;
	memset(&u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[0][0], 0, sizeof(u_shared_buffers.s_fch_int.auc_tx_bits_interleaver));

	/* Read data transmitted for channel estimation */
	pplc_if_do_read(u_shared_buffers.s_fch_int.interleaver_matrix_tx, uc_fch_interleaver_byte_size);

	for (us_bit_index = 0; us_bit_index < us_fch_interleaver_size_with_padding; us_bit_index++) {
		uc_byte_index = us_bit_index / 8;
		uc_rel_bit_index = us_bit_index % 8;
		if (u_shared_buffers.s_fch_int.interleaver_matrix_tx[uc_byte_index] & (0x80 >> uc_rel_bit_index)) {
			u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_local_buf_index][uc_local_byte_index] |= (0x80 >> uc_rel_local_bit_index);
		}

		if (++uc_local_bit_index == uc_used_carriers) {
			uc_local_bit_index = 0;
			uc_local_buf_index++;
		}

		uc_local_byte_index = uc_local_bit_index / 8;
		uc_rel_local_bit_index = uc_local_bit_index % 8;
	}

	/* Get useful symbols starting from the last FCH symbol */
	uc_num_sym_valid_fch = 0;
	if (uc_working_band == WB_CENELEC_A) {
		uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_CENELEC_A;
	} else {
		uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_FCC_ARIB;
	}

	for (uc_i = 0; uc_i < uc_num_sym_used_fch; uc_i++) {
		if (auc_use_fch_symbol[SYMBOLS_8 - uc_num_sym_h_est_fch + uc_i]) {
			auc_index_sym_valid_fch[uc_num_sym_valid_fch] = SYMBOLS_8 - uc_num_sym_h_est_fch + uc_i;
			uc_num_sym_valid_fch++;
		}
	}

	if (uc_working_band != WB_FCC) {
		memset(&auc_previous_ref_bits_interleaver[0], 0, uc_fch_symbol_byte_size);
		for (uc_i = 0; uc_i < (uc_num_symbols_fch - SYMBOLS_8); uc_i++) {
			for (uc_j = 0; uc_j < uc_fch_symbol_byte_size; uc_j++) {
				auc_previous_ref_bits_interleaver[uc_j]
					= auc_previous_ref_bits_interleaver[uc_j] ^ u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_i][uc_j];
			}
		}
	} else { /* WB_FCC */
		if (!uc_legacy_mode) {
			memset(&auc_previous_ref_bits_interleaver[0], 0, uc_fch_symbol_byte_size);
			for (uc_i = 0; uc_i < (uc_num_symbols_fch - SYMBOLS_8); uc_i++) {
				for (uc_j = 0; uc_j < uc_fch_symbol_byte_size; uc_j++) {
					auc_previous_ref_bits_interleaver[uc_j]
						= auc_previous_ref_bits_interleaver[uc_j] ^ u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_i][uc_j];
				}
			}
		}
	}

	uc_next_fch_computed = 0;
	for (uc_i = 0; uc_i < uc_num_sym_valid_fch; uc_i++) {
		/* Get reference to construct header symbol from P and M symbols */
		if (uc_working_band != WB_FCC) {
			for (uc_k = uc_next_fch_computed; uc_k <= auc_index_sym_valid_fch[uc_i]; uc_k++) {         /* JAC changed. */
				for (uc_j = 0; uc_j < uc_fch_symbol_byte_size; uc_j++) {
					auc_previous_ref_bits_interleaver[uc_j] = auc_previous_ref_bits_interleaver[uc_j] ^
							u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_num_symbols_fch - SYMBOLS_8 + uc_k][uc_j];
				}
			}
			/*uc_ichannel_idx = (auc_index_sym_valid_fch[uc_i] + 7) % 8;*/         /* the last symbols is the first */
		} else { /* WB_FCC */
			if (uc_legacy_mode) {
				memcpy(&auc_previous_ref_bits_interleaver[0],
						&u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_num_symbols_fch - SYMBOLS_8 + auc_index_sym_valid_fch[uc_i]][0],
						uc_fch_symbol_byte_size);
				/*uc_ichannel_idx = auc_index_sym_valid_fch[uc_i];*/
			} else {
				for (uc_k = uc_next_fch_computed; uc_k <= auc_index_sym_valid_fch[uc_i]; uc_k++) {         /* JAC changed. */
					for (uc_j = 0; uc_j < uc_fch_symbol_byte_size; uc_j++) {
						auc_previous_ref_bits_interleaver[uc_j] = auc_previous_ref_bits_interleaver[uc_j] ^
								u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_num_symbols_fch - SYMBOLS_8 + uc_k][uc_j];
					}
				}
				/*uc_ichannel_idx = (auc_index_sym_valid_fch[uc_i] + 7) % 8;*/         /* the last symbols is the first */
			}
		}

		uc_next_fch_computed = auc_index_sym_valid_fch[uc_i] + 1;

		obtain_modulating_conj_complex_values(&auc_previous_ref_bits_interleaver[0], uc_used_carriers, PREAMBLE_FCH_S1S2_PART,
				auc_unmasked_carrier_list, ass_modulating_symbols);
		arm_cmplx_mult_cmplx_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers *
				(uc_num_sym_valid_preamble + uc_i)),
				ass_modulating_symbols,
				(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_i)),
				uc_used_carriers);   /* Result in Q3.13 */

		#ifdef SMOOTHING
		smooth_carriers_asm(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch,
				(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_i)));
		#endif

		/* Averaging. Sum FCH symbol to previous sum. Result is Q1.15 */
		scale_shift_add_shift_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers *
				(uc_num_sym_valid_preamble + uc_i)),
				ass_average_symbol, uc_used_carriers);
	}

	/* If the payload is coherently modulated, final channel estimate will be obtained with S1S2. */
	/* If payload is differentially modulated, the estimate of the symbol noise energy must be obtained for */
	/* SFO estimation. */
	if (uc_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
		/* Scale averaged symbol and convert to Q1.31 */
		arm_scale_q15(ass_average_symbol, ass_scale[uc_num_sym_valid_preamble + uc_num_sym_valid_fch - 2], 4, ass_average_symbol, 2 * uc_used_carriers);

		/* ************************************ Estimation of the channel response by weighted averaging */
		/* Computes the noise energy in each symbol */
		_compute_symbol_noise(ass_average_symbol, uc_num_sym_valid_preamble + uc_num_sym_valid_fch);

		/* Computes the weighting factors */
		compute_weights_asm(uc_num_sym_valid_preamble + uc_num_sym_valid_fch);
	}

	/* Clear SOURCE_H for further demodulations */
	pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, (uint8_t)(~0x01u)); /* SOURCE_H='0' */
}

/**
 * \brief Estimate channel using the preamble, FCH and S1S2
 *
 */
void chn_estimation_from_s1s2(void)
{
	uint8_t uc_i;

	/* Scale the average symbol by the number of symbols used in the averaging and to convert from Q5.10 to Q1.15 */
	arm_scale_q15(ass_average_symbol, ass_scale[uc_num_sym_valid_preamble + uc_num_sym_valid_fch], 4, ass_average_symbol, 2 * uc_used_carriers);

	/********** Estimation of the channel response by weighted averaging **********/
	/* Computes the noise energy in each symbol */
	_compute_symbol_noise(ass_average_symbol, uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 2);

	/* Computes the weighting factors */
	compute_weights_asm((uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 2));

	/* Estimates the weighted averaging */
	zero_complex_vector_q_asm((uint8_t *)ass_H, 2 * uc_used_carriers * sizeof(q15_t));
	for (uc_i = 0; uc_i < (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 2); uc_i++) {
		/* Multiplies the LS estimates by their Wi and sum */
		arm_scale_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + uc_i * 2 * uc_used_carriers), *(ass_Wi + uc_i),
				0, ass_symbol_aux, 2 * uc_used_carriers);
		arm_add_q15(ass_symbol_aux, ass_H, ass_H, 2 * uc_used_carriers);
	}
}

#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)

/**
 * \brief Estimate channel from pilots received in payload
 *
 */

void chn_estimation_from_pilots(uint8_t *puc_static_dynamic_notch_map, uint8_t uc_num_pilots_per_symbol, uint8_t uc_last_symbol_index,
		uint8_t uc_num_active_carriers, uint32_t *pul_pn_seq_index)
{
	uint8_t uc_num_data_carriers = 0, auc_cum_num_pilots_used[NUM_SYM_PILOTS_H_EST + 1] = {0};
	uint8_t uc_i, uc_j;

	/* Index for storing the current pilots block in the corresponding position for future use in SFO estimation */
	if (uc_last_symbol_index < (s_band_constants.uc_pay_sym_first_demod + PAY_SYM_SECOND_DEMOD + NUM_SYM_PILOTS_H_EST * 2)) {
		uc_index_set = 0;
		memset(aus_pilot_stored, 0, NUM_CARRIERS_TIMES_2_FCC * sizeof(uint8_t));
	}

	pass_Ypilots_as = ass_Ypilots[uc_index_set];    /* Pointer to the array where the pilots have to be stored in Q8.24 */
	uc_pointer_byte_lfsr_pilots_as = 0;
	uc_pointer_bitpair_lfsr_pilots_as = 0;
	uc_num_active_carriers_as = uc_num_active_carriers;

	/*Launch DMA transfer of the first 3 payload symbols of the block (the number of symbols that can be read is limited by PDC_PPLC_BUFFER_SIZE) */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16,
			(CFG_IOB_SAMPLES_PER_SYMBOL * 2  + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[uc_used_carriers - 1]));
	pplc_if_read_jump((BCODE_ZONE4 | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
			(uint8_t *)u_shared_preamble_and_payload_symbols.ass_payload_symbols, 2 * 2 * uc_used_carriers * 3, JUMP_COL_2, false);

	/* Compute the position of the pilots in the current set of symbols */
	memset(auc_carrier_state_pilot, 0, sizeof(uint8_t) * s_band_constants.uc_num_carriers);
	if (uc_working_band == WB_CENELEC_A) {
		compute_pilot_position_cenelec_a_as(NUM_SYM_PILOTS_H_EST, uc_num_pilots_per_symbol, (uc_last_symbol_index - NUM_SYM_PILOTS_H_EST));
	} else {
		compute_pilot_position_fcc_arib_as(NUM_SYM_PILOTS_H_EST, uc_num_pilots_per_symbol, (uc_last_symbol_index - NUM_SYM_PILOTS_H_EST));
	}

	/* Calculates the indexes of the data carriers to be smoothed in the current set of symbols */
	control_smooth_and_order_pilot_list(puc_static_dynamic_notch_map, auc_carrier_state_pilot, auc_data_carriers_list,
			&uc_num_data_carriers, auc_pilot_carriers_list, auc_control_avg_invert_chan, uc_num_pilots_per_symbol,
			(uc_last_symbol_index - NUM_SYM_PILOTS_H_EST));

	/* Read the first 3 payload symbols of the block */
	pplc_if_do_read((uint8_t *)u_shared_preamble_and_payload_symbols.ass_payload_symbols, 2 * 2 * uc_used_carriers * 3);

	/* Extract pilots of the first 3 payload symbols */
	for (uc_i = 0; uc_i < 3; uc_i++) {
		/* Extract pilot carriers of the previous symbol and place them in ass_symbol_aux */
		for (uc_j = 0; uc_j < auc_num_pilots_in_symbol[uc_i]; uc_j++) {
			ass_symbol_aux[(auc_cum_num_pilots_used[uc_i] + uc_j) * 2]
				= u_shared_preamble_and_payload_symbols.ass_payload_symbols[2 * uc_used_carriers * uc_i
					+ (*(auc_pilot_carriers_list_absolute + uc_i * uc_num_pilots_per_symbol + uc_j)) * 2];
			ass_symbol_aux[(auc_cum_num_pilots_used[uc_i] + uc_j) * 2 + 1]
				= u_shared_preamble_and_payload_symbols.ass_payload_symbols[2 * uc_used_carriers * uc_i
					+ (*(auc_pilot_carriers_list_absolute + uc_i * uc_num_pilots_per_symbol + uc_j)) * 2 + 1];

			/* Eliminate gaps due to pilots used twice in the same block of NUM_SYM_PILOTS_H_EST symbols */
			auc_pilot_carriers_list_absolute[(auc_cum_num_pilots_used[uc_i] + uc_j)]
				= auc_pilot_carriers_list_absolute[uc_i * uc_num_pilots_per_symbol + uc_j];
		}

		/* Number of carrier indexes used as pilots (carrier indexes used twice in a block of NUM_SYM_PILOTS_H_EST symbols are counted only once)*/
		auc_cum_num_pilots_used[uc_i + 1] = auc_cum_num_pilots_used[uc_i] + auc_num_pilots_in_symbol[uc_i];
	}

	/* Launch DMA transfer of the last 3 payload symbols of the block.
	 * Cannot be done just after the first 3 payload symbols are red because compiler optimization lead to a bad result */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16,
			(CFG_IOB_SAMPLES_PER_SYMBOL * 5 + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[uc_used_carriers - 1]));
	pplc_if_read_jump((BCODE_ZONE4 | (CFG_IOB_SAMPLES_PER_SYMBOL * 3 + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
			(uint8_t *)(u_shared_preamble_and_payload_symbols.ass_payload_symbols + 2 * uc_used_carriers * 3),
			2 * 2 * uc_used_carriers * 3, JUMP_COL_2, false);

	/* This is done here to minimize the required computations after the last 3 symbols have been read */
	for (uc_i = 3; uc_i < 6; uc_i++) {
		/* Extract pilot carriers of the previous symbol and place them in ass_symbol_aux */
		for (uc_j = 0; uc_j < auc_num_pilots_in_symbol[uc_i]; uc_j++) {
			/* Eliminate gaps due to pilots used twice in the same block of NUM_SYM_PILOTS_H_EST symbols */
			auc_pilot_carriers_list_absolute[(auc_cum_num_pilots_used[uc_i] + uc_j)]
				= auc_pilot_carriers_list_absolute[uc_i * uc_num_pilots_per_symbol + uc_j];
		}
		/* Number of carrier indexes used as pilots (carrier indexes used twice in a block of NUM_SYM_PILOTS_H_EST symbols are counted only once)*/
		auc_cum_num_pilots_used[uc_i + 1] = auc_cum_num_pilots_used[uc_i] + auc_num_pilots_in_symbol[uc_i];
	}

	/* Compute LFSR sequence for the block of symbols. MUST be done after gaps in auc_pilot_carriers_list_absolute have been removed */
	memset(u_shared_buffers.s_mod_demod.auc_pn_seq_pilots, 0, MAX_PN_SEQ_LEN_PILOTS / 4);
	pul_index_to_lfsr_as = pul_pn_seq_index;
	uc_index_symbol_in_block = 0;
	compute_lfsr_sequence_pilots_as(auc_state_carrier_asm, &u_shared_buffers.s_mod_demod.auc_pn_seq_pilots[0], NUM_SYM_PILOTS_H_EST, s_band_constants.uc_num_carriers);

	/* Obtains the complex values for demodulating the pilot carriers */
	obtain_modulating_conj_complex_values(u_shared_buffers.s_mod_demod.auc_pn_seq_pilots,
			auc_cum_num_pilots_used[NUM_SYM_PILOTS_H_EST] * 2, PAYLOAD_PART, auc_pilot_carriers_list_absolute, ass_modulating_symbols);

	/* Read the last 3 payload symbols of the block */
	pplc_if_do_read((uint8_t *)(u_shared_preamble_and_payload_symbols.ass_payload_symbols + 2 * uc_used_carriers * 3), 2 * 2 * uc_used_carriers * 3);

	/* Extract pilots of the last 3 payload symbols */
	for (uc_i = 3; uc_i < 6; uc_i++) {
		/* Extract pilot carriers of the previous symbol and place them in ass_symbol_aux */
		for (uc_j = 0; uc_j < auc_num_pilots_in_symbol[uc_i]; uc_j++) {
			ass_symbol_aux[(auc_cum_num_pilots_used[uc_i] + uc_j) * 2]
				= u_shared_preamble_and_payload_symbols.ass_payload_symbols[2 * uc_used_carriers * uc_i
					+ (*(auc_pilot_carriers_list_absolute + uc_i * uc_num_pilots_per_symbol + uc_j)) * 2];
			ass_symbol_aux[(auc_cum_num_pilots_used[uc_i] + uc_j) * 2 + 1]
				= u_shared_preamble_and_payload_symbols.ass_payload_symbols[2 * uc_used_carriers * uc_i
					+ (*(auc_pilot_carriers_list_absolute + uc_i * uc_num_pilots_per_symbol + uc_j)) * 2 + 1];
		}
	}

	/* Demodulate received pilots, place their values in vector ass_Ypilot [0...Munnotched] and update of the channel estimate at the pilot frequencies */
	dem_pilots_and_update_chan_as(auc_pilot_carriers_list_absolute, ass_symbol_aux, ass_modulating_symbols, auc_cum_num_pilots_used[NUM_SYM_PILOTS_H_EST]);

	/* Updates index_set for the next iteration */
	uc_index_set = (uc_index_set + 1) % 2;

	/* Smooth data carriers */
	smooth_carriers_asm(auc_data_carriers_list, uc_num_data_carriers, ass_H);

	#ifdef DEBUG_CHN_SFO
	arm_copy_q15(ass_H, (ass_H_pilotos + uc_num_block_pilots * 2 * uc_used_carriers), 2 * uc_used_carriers);
	arm_copy_q15(pass_Ypilots_as, (ass_Ypilots_block + uc_num_block_pilots * 2 * uc_used_carriers), 2 * uc_used_carriers);
	for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
		auc_pilot_carriers_list_vector[uc_num_block_pilots * uc_num_pilots_per_symbol * NUM_SYM_PILOTS_H_EST  + uc_i] = auc_pilot_carriers_list_absolute[uc_i];
	}
	#endif

	/* Invert channel, scale by GAIN_SCALE_INV_H_FEQ and swap bytes*/
	invert_channel_asm(NUM_BITS_FRAC_PART_INV_H, uc_used_carriers, ass_H);

	/* Moves the inverse of the channel to symbol 0 of the ATPL */
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, 0x00); /* Set destination to symbol 0 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier);
	pplc_if_write_jump((BCODE_ZONE2 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier)), (uint8_t *)ass_inv_H, 2 * uc_used_carriers * 2, JUMP_COL_2);

	/* Configure overflow as it was before entering */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (NUM_SYM_PILOTS_H_EST - 1)) + s_band_constants.uc_last_carrier);
}

#endif
