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
#include <string.h>

/* Phy layer includes */
#include "atpl250.h"
#include "atpl250_channel_estimation.h"
#include "atpl250_mod_demod.h"

/* Pointers to external defined arrays */
extern q15_t ass_rotation_sfo[];

/* Extern PHY Variables */
extern uint8_t uc_notched_carriers;
extern uint8_t uc_used_carriers;
extern uint8_t uc_num_symbols_fch;
extern uint16_t us_fch_interleaver_size_with_padding;
extern uint8_t uc_fch_interleaver_byte_size;
extern uint8_t uc_fch_symbol_byte_size;
extern uint8_t auc_unmasked_carrier_list[];
extern uint8_t uc_legacy_mode;

/* Controls whether smoothing should be applied to a given carrier of the preamble and FCH when static notching is defined. */
uint8_t auc_smooth_control_preamble_fch[PROTOCOL_CARRIERS - 2] = {0}; /* 0=apply smoothing 1=no smoothing */
uint8_t uc_num_carr_smooth_preamble_fch;

/* Actual number of SYNCP and FCH symbols used for channel estimation. */
/* Maximum number of SYNCP is NUM_SYM_H_EST_PRE and maximum number of fch symbols is NUM_SYM_H_EST_FCH */
static uint8_t uc_num_p_sym;
uint8_t uc_num_sym_valid_preamble;
uint8_t uc_num_sym_valid_fch;
uint8_t auc_index_sym_valid_fch[NUM_SYM_H_EST_FCH]; /* Last FCH symbol must be at position NUM_SYM_H_EST_FCH-1 */

/* Pointer to the array with the inverse of the noise energy per symbol and of the weighting coefficients */
static q31_t asl_Ni[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH + 2];
static q63_t asll_Ni[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH + 2];
q31_t asl_inv_Ni[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH + 2];
q15_t ass_Wi[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH + 2];

/* Auxiliary variable to estimate channel from FCH and interleaver bits */
static uint8_t auc_previous_ref_bits_interleaver[MAX_FCH_SYMBOL_BYTE_SIZE];

/* Array for storing the p,m fch and s1s2 symbols used for channel estimation */
/* Reused in atpl250_sampling_error_estimation*/
q15_t ass_pm_fch_s1s2_symbols[PROTOCOL_CARRIERS_BY_2 * (NUM_SYM_H_EST_PRE + 2 + NUM_SYM_H_EST_FCH)] = {0};
q15_t ass_Ypilots[PROTOCOL_CARRIERS_BY_2];

/*Array for storing the conjugate of the complex value that modulate one symbol*/
static q15_t ass_modulating_symbols[PROTOCOL_CARRIERS_BY_2];

/* Data and pilot carrier lists for the current set of pilots used for channel estimation*/
#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
static uint8_t auc_data_carriers_list[PROTOCOL_CARRIERS];
static uint8_t auc_pilot_carriers_list[PROTOCOL_CARRIERS];
#endif

/*Array for storing one symbol*/
q15_t ass_symbol_aux[PROTOCOL_CARRIERS_BY_2]; /* Reused in atpl250_sampling_error_estimation*/
#if (defined(SMOOTHING) || defined(UPDATE_CHN_SFO_EST_PAYLOAD))
static q15_t ass_symbol_aux_1[PROTOCOL_CARRIERS_BY_2];
#endif
q31_t asl_symbol_aux[PROTOCOL_CARRIERS_BY_2]; /* Reused in atpl250_sampling_error_estimation*/

/* Array for storing the average of the p,m fch and s1s2 symbols */
static q15_t ass_average_symbol[PROTOCOL_CARRIERS_BY_2];
q31_t asl_average_symbol[PROTOCOL_CARRIERS_BY_2]; /* Reused in atpl250_sampling_error_estimation*/

/* Array for storing the estimated channel response and its inverse in Q1.15 */
q15_t ass_H[PROTOCOL_CARRIERS_BY_2] = {0};  /* Reused in atpl250_sampling_error_estimation*/
static q15_t ass_inv_H[PROTOCOL_CARRIERS_BY_2] = {0};

/* Array for storing scale factors */
/* ass_scale contains 1/2, 1/3, 1/4...1/17 in Q.15 */
const q15_t ass_scale[16] = {16384, 10923, 8192, 6554, 5461, 4681, 4096, 3641, 3277, 2979, 2731, 2521, 2341, 2186, 2048, 1928};

static const uint8_t auc_zeros[CARR_BUFFER_LEN] = {0};
static const uint8_t auc_ones[CARR_BUFFER_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Complex conjugate of the reference SYNCP in Q1.15. Distortion because of the fact that 1+j*0 is not representable is negligible. */
static const q15_t ass_syncp_conj[PROTOCOL_CARRIERS_BY_2]
	= {23170, -23170, 30274, -12540, 32767, 0, 30274, 12540, 23170, 23170, 0, 32767, -23170, 23170, -30274, -12540, 12540, -30274, 30274, 12540,
	   -12540, 30274, -23170,
	   -23170, 30274, -12540, -12540, 30274, -12540, -30274, 23170, 23170, -30274, -12540, 30274, 12540, -30274,
	   -12540, 30274, 12540, -23170, -23170, 12540, 30274, 23170,
	   -23170, -32768, 0, 12540, 30274, 23170, -23170, -23170, -23170, -23170, 23170, 12540, 30274, 32767, 0,
	   23170, -23170, 12540, -30274, -12540, -30274, -23170, -23170, -30274,
	   -12540, -30274, -12540};

static const q15_t ass_syncm_conj[PROTOCOL_CARRIERS_BY_2]
	= {-23170, 23170, -30274, 12540, -32768, 0, -30274, -12540, -23170, -23170, 0, -32768, 23170, -23170, 30274, 12540, -12540, 30274, -30274,
	   -12540, 12540, -30274, 23170, 23170,
	   -30274, 12540, 12540, -30274, 12540, 30274, -23170, -23170, 30274, 12540, -30274, -12540, 30274, 12540,
	   -30274, -12540, 23170, 23170, -12540, -30274, -23170, 23170, 32767, 0,
	   -12540, -30274, -23170, 23170, 23170, 23170, 23170, -23170, -12540, -30274, -32768, 0, -23170, 23170,
	   -12540, 30274, 12540, 30274, 23170, 23170, 30274, 12540, 30274, 12540};

static const q15_t ass_syncm_shift[PROTOCOL_CARRIERS_BY_2]
	= {-2411, 32679, 32138, -6393, -14733, -29269, -24279, 22006, 27684, 17531, 9512, -31357, -32758, -804, 7962, 31786, 28511, -16151, -23170,
	   -23170, -16151, 28511, 31786, 7962, -804, -32758, -31357, 9512, 17531, 27684, 22006, -24279, -29269, -14733, -6393, 32138, 32679, -2411,
	   -11039, -30853, -26791, 18868, 25330, 20788, 13279, -29957, -32413, -4808, 4011, 32522, 30274, -12540, -20160, -25833, -19520, 26320, 30572,
	   11793, 3212, -32610, -32286, 5602, 14010, 29622, 24812, -21403, -27246, -18205, -10279, 31114, 32729, 1608};

/* Defines division of Q1.15 values */
/* div_real_q15(dividend,divisor,quotient), the sum of divisor >>1 is for rounding instead of truncating. */

/*
 * #define div_real_q15(ss_dividend, ss_divisor, pss_quotient)	do{																\
 * pss_quotient = ((((int32_t) ss_dividend) << 15) + (((int32_t) ss_divisor)>>1)) / ((int32_t) ss_divisor);			\
 * } while(0)
 */

/* Defines division of Q(16-uc_q_frac values).uc_q_frac values and Q(32-uc_q_frac values).uc_q_frac */
/* div_real_q(dividend,divisor,quotient), the sum of divisor >>1 is for rounding instead of truncating. */
#define div_real_q(sl_dividend, sl_divisor, uc_q_frac, psl_quotient) do { \
		*psl_quotient = ((((int64_t)sl_dividend) << uc_q_frac) + (((int64_t)sl_divisor) >> 1)) / ((int64_t)sl_divisor);	\
} \
	while (0)

/* Defines fixed point multiplication of real values in Quc_q_int.uc_q_frac using half-up rounding. */
#define mult_real_q(sl_a, sl_b, uc_q_frac)  (int32_t)((((int64_t)sl_a * (int64_t)sl_b) + (1 << (uc_q_frac - 1))) >> uc_q_frac)

/* Defines fixed point multiplication of real values in Q1.15 with halp-up rounding. */
/* #define mult_real_q15(ss_a, ss_b) ((((int32_t) ss_a * (int32_t) ss_b) + (1 << 14) )>> 15) */

/* Defines fixed point multiplication of real values in Q1.15 using half-even rounding. */

/*static inline int32_t mult_real_q15(int16_t a, int16_t b){
 *      int32_t sl_product, sl_scaling;
 *
 *      sl_product =  (int32_t) a * (int32_t) b;
 *
 *      sl_scaling = ( (sl_product & 0x00008000) >> 15) ^ 0x00000001;
 *
 *      sl_product= (sl_product + 0x00004000 - sl_scaling) >> 15;
 *
 *      return sl_product;
 *
 * }
 */

/* Inverts the byte order of the int16_t elements of array. */
static inline void _invert_byte_order(int16_t *pss_input_array, uint16_t us_num_elements)
{
	uint16_t us_i;
	for (us_i = 0; us_i < us_num_elements; us_i++) {
		*(pss_input_array + us_i) = (*(pss_input_array + us_i)  << 8) | ((*(pss_input_array + us_i) >> 8) & 0x00FF);
	}
}

/*Obtains the conjugate of the complex values transmitted in a given symbol.
 */
static inline void _obtain_modulating_conj_complex_values(uint8_t *puc_input_bits, uint8_t uc_num_bits, uint8_t uc_frame_part, uint8_t *puc_carrier_list,
		q15_t *pss_modulating_symbol)
{
	uint8_t uc_i, uc_byte_index, uc_bit_index, uc_bit1, uc_bit2, uc_carrier_index;

	if (uc_frame_part == PAYLOAD_PART) { /* for pilots */
		for (uc_i = 0; uc_i < (uc_num_bits / 2); uc_i++) {
			uc_byte_index = 2 * uc_i >> 3;
			uc_bit_index = 7 - (2 * uc_i - (uc_byte_index << 3)); /* The msb is the first bit */
			uc_bit1 = *(puc_input_bits + uc_byte_index) & (0x01 << uc_bit_index);
			uc_byte_index = (2 * uc_i + 1) >> 3;
			uc_bit_index = 7 - ((2 * uc_i + 1) - (uc_byte_index << 3));
			uc_bit2 = *(puc_input_bits + uc_byte_index) & (0x01 << uc_bit_index);

			/* pilots are indexes relative to Mactive but ass_syncp_conj is indexed from 0 to PROTOCOL_CARRIERS-1 */
			uc_carrier_index = auc_unmasked_carrier_list[*(puc_carrier_list + uc_i)];
			if (uc_bit1 == 0) {
				if (uc_bit2 == 0) {
					*(pss_modulating_symbol + 2 * uc_i) = ass_syncp_conj[2 * uc_carrier_index];
					*(pss_modulating_symbol + 2 * uc_i + 1) = ass_syncp_conj[2 * uc_carrier_index + 1];
				} else {
					*(pss_modulating_symbol + 2 * uc_i) = ass_syncm_conj[2 * uc_carrier_index + 1];
					*(pss_modulating_symbol + 2 * uc_i + 1) = ass_syncp_conj[2 * uc_carrier_index];
				}
			} else {
				if (uc_bit2 == 0) {
					*(pss_modulating_symbol + 2 * uc_i) = ass_syncp_conj[2 * uc_carrier_index + 1];
					*(pss_modulating_symbol + 2 * uc_i + 1) = ass_syncm_conj[2 * uc_carrier_index];
				} else {
					*(pss_modulating_symbol + 2 * uc_i) = ass_syncm_conj[2 * uc_carrier_index];
					*(pss_modulating_symbol + 2 * uc_i + 1) = ass_syncm_conj[2 * uc_carrier_index + 1];
				}
			}
		}
	} else { /* for preamble, S1S2, and FCH */
		for (uc_i = 0; uc_i < uc_num_bits; uc_i++) {
			uc_byte_index = uc_i >> 3;
			uc_bit_index = 7 - uc_i + (uc_byte_index << 3); /* The msb is the first bit */
			uc_bit1 = *(puc_input_bits + uc_byte_index) & (0x01 << uc_bit_index);

			/* in the preamble, S1S2 and the FCH the function is called with the vector of carrier indexes as in auc_unmasked_carrier_list */
			uc_carrier_index = *(puc_carrier_list + uc_i);
			if (uc_bit1 == 0) {
				*(pss_modulating_symbol + 2 * uc_i) = ass_syncp_conj[2 * uc_carrier_index];
				*(pss_modulating_symbol + 2 * uc_i + 1) = ass_syncp_conj[2 * uc_carrier_index + 1];
			} else {
				*(pss_modulating_symbol + 2 * uc_i) = ass_syncm_conj[2 * uc_carrier_index];
				*(pss_modulating_symbol + 2 * uc_i + 1) = ass_syncm_conj[2 * uc_carrier_index + 1];
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
 */
static inline void _compute_symbol_noise(uint8_t uc_num_symbols)
{
	uint8_t uc_i, uc_scaling = 0;
	uint64_t ull_max;
	q63_t ll_energy_max;

	if (uc_num_symbols > 0) {
		/* Magnitude of the reference vector */
		arm_cmplx_mag_squared_q31(asl_average_symbol, u_shared_buffers.s_chn_est.asl_squared_mag_symbol, uc_used_carriers); /* Result in Q3.29 */

		/* Computes the energy of the noise in each symbol */
		ll_energy_max = 0;
		for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
			/*Convert input vector to Q1.31*/
			arm_q15_to_q31((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i), asl_symbol_aux, 2 * uc_used_carriers);

			/* Difference of the magnitudes */
			arm_cmplx_mag_squared_q31(asl_symbol_aux, u_shared_buffers.s_chn_est.asl_squared_mag_symbol_1, uc_used_carriers);  /* Result in Q3.29 */
			arm_sub_q31(u_shared_buffers.s_chn_est.asl_squared_mag_symbol_1, u_shared_buffers.s_chn_est.asl_squared_mag_symbol,
					asl_symbol_aux, uc_used_carriers); /* Difference of two Q3.29 positive numbers. Max value is <4 */
			arm_power_q31(asl_symbol_aux, uc_used_carriers, (asll_Ni + uc_i)); /* asll_Ni is in Q16.48 but max value is 4^2*uc_used_carriers<2^11 */

			if (*(asll_Ni + uc_i) > ll_energy_max) { /* finds the maximum energy value */
				ll_energy_max = *(asll_Ni + uc_i);
			}
		}

		/* To minimize precision loss, scaling value so that the maximum is >0.5 and <1 */
		ull_max = 0x4000000000000000ULL;
		while (!(ll_energy_max & ull_max) && (uc_scaling < FRAC_PART_Q63)) {
			uc_scaling++;
			ull_max = ull_max >> 1;
		}

		if (uc_scaling > FRAC_PART_Q31) {
			for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
				*(asl_Ni + uc_i) = (q31_t)(*(asll_Ni + uc_i) << (uc_scaling - FRAC_PART_Q31 - 1));
			}
		} else {
			for (uc_i = 0; uc_i < uc_num_symbols; uc_i++) {
				*(asl_Ni + uc_i) = (q31_t)(*(asll_Ni + uc_i) >> (FRAC_PART_Q31 - uc_scaling + 1));
			}
		}
	}
}

/*Computes the weights to be applied to a weighted averaging of noisy estimates of the channel.
 * Takes the values of the energy of the noise in each symbol, stored in the static array asl_Ni, and computes its inverse, which is stored in the static array
 * asl_inv_Ni and
 * 0.5*min(1/Ni)/sum(1/Ni), which is stored in the static array ass_Wi.
 * Inputs:
 * uint8_t uc_num_values: number of estimates
 * uint8_t uc_scale_exp:  scaling applied to avoid overflow in sum(1/Ni) is 2^uc_scale_exp.
 * Outputs:
 * Output is written into the static array ass_Wi.
 */
static inline void _compute_weights(uint8_t uc_num_values, uint8_t uc_scale_exp)
{
	uint8_t uc_i;
	uint32_t ul_min;
	q31_t sl_min, sl_sum = 0;

	/* Finds the minimum Ni */
	arm_min_q31(asl_Ni, uc_num_values, &sl_min, &ul_min);
	sl_min = sl_min >> 1; /* Divides by 2 to avoid overflow in the next step */

	/* Instead of 1/Ni, to avoid overflow in the division, (min(Ni)/2)/Ni is computed. */
	for (uc_i = 0; uc_i < uc_num_values; uc_i++) {
		if ((*(asl_Ni + uc_i)) == 0) {
			*(asl_inv_Ni + uc_i) = MAX_INT32;
		} else {
			div_real_q(sl_min, (*(asl_Ni + uc_i)), FRAC_PART_Q31, (asl_inv_Ni + uc_i)); /* (min(Ni)/2)/Ni */
		}

		sl_sum += (*(asl_inv_Ni + uc_i) >> uc_scale_exp);
	}

	/* Computes (1/Ni)/sum(1/Ni) */
	for (uc_i = 0; uc_i < uc_num_values; uc_i++) {
		/* An implicit cast from Q1.31 to Q1.15 would be accomplished */
		div_real_q((*(asl_inv_Ni + uc_i) >> uc_scale_exp), sl_sum, FRAC_PART_Q15, (ass_Wi + uc_i));
	}
}

/*Calculates the list of carriers that must be smoothed in a set of symbols.
 * It can be used both in the payload and in the preamble and fch. When used in the preamble and fch,
 * the input puc_static_and_dynamic_map should be fixed to the tone mask and puc_pilot_map to all 0.
 * When used in the payload, puc_static_and_dynamic_map should contain the static and dynamic map and puc_pilot_map
 * must contain a 1 in the carriers used as pilots.
 * The returned data carrier indexes range from 0 to uc_used_carriers-1.
 *      IMPORTANT:
 *              -It has been assumed that FIRST_CARRIER>8
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
void control_smooth(uint8_t *puc_static_and_dynamic_map, uint8_t *puc_pilot_map, uint8_t *puc_data_carriers_list, uint8_t *puc_num_data_carrier,
		uint8_t *puc_pilot_carriers_list, uint8_t *puc_num_pilot_carriers)
{
	uint8_t uc_i, uc_byte_index_i, uc_bit_index_in_byte_i, uc_byte_index_i_1, uc_bit_index_in_byte_i_1, uc_byte_index_i_plus_1,
			uc_bit_index_in_byte_i_plus_1;
	uint8_t uc_value_i_1, uc_value_i, uc_value_i_plus_1, uc_num_notched_carriers = 0;

	/* Checks if the first carrier is a pilot or is masked */
	uc_i = FIRST_CARRIER;
	uc_byte_index_i = uc_i >> 3;
	uc_bit_index_in_byte_i = uc_i & 0x07;

	uc_value_i = (*(puc_static_and_dynamic_map  + uc_byte_index_i) >> uc_bit_index_in_byte_i) &  0x01;
	if (uc_value_i == 1) {
		uc_num_notched_carriers++;
	} else {
		if (((*(puc_pilot_map + uc_byte_index_i) >> uc_bit_index_in_byte_i) & 0x01) == 1) {
			*(puc_pilot_carriers_list + *puc_num_pilot_carriers) = uc_i - uc_num_notched_carriers - FIRST_CARRIER;
			(*puc_num_pilot_carriers)++;
		}
	}

	for (uc_i = FIRST_CARRIER + 1; uc_i < LAST_CARRIER; uc_i++) {
		/* The first and the last carrier cannot be smoothed because they have no previous and successive carriers */

		uc_byte_index_i = uc_i >> 3;
		uc_bit_index_in_byte_i = uc_i & 0x07;

		/* Checks that current carrier is not a pilot */
		if (((*(puc_pilot_map + uc_byte_index_i) >> uc_bit_index_in_byte_i) & 0x01) == 0) {
			uc_byte_index_i_1 = (uc_i - 1) >> 3;
			uc_byte_index_i_plus_1 = (uc_i + 1) >> 3;
			uc_bit_index_in_byte_i_1 = (uc_i - 1) & 0x07;
			uc_bit_index_in_byte_i_plus_1 = (uc_i + 1) & 0x07;

			/* Determines the values of the actual carrier "i", the previous one "i-1" and the subsequent "i+1" in puc_static_and_dynamic_map */
			uc_value_i_1 = (*(puc_static_and_dynamic_map  + uc_byte_index_i_1) >> uc_bit_index_in_byte_i_1) &  0x01;
			uc_value_i = (*(puc_static_and_dynamic_map  + uc_byte_index_i) >> uc_bit_index_in_byte_i) &  0x01;
			uc_value_i_plus_1 = (*(puc_static_and_dynamic_map  + uc_byte_index_i_plus_1) >> uc_bit_index_in_byte_i_plus_1) &  0x01;

			/* Determines whether a carrier has valid predecessor and sucessor */
			if ((uc_value_i_1 == 0) && (uc_value_i == 0) &&  (uc_value_i_plus_1 == 0)) {
				*(puc_data_carriers_list + *puc_num_data_carrier) = uc_i - uc_num_notched_carriers - FIRST_CARRIER;
				(*puc_num_data_carrier)++;
			} else if (uc_value_i == 1) {
				uc_num_notched_carriers++;
			}
		} else {
			*(puc_pilot_carriers_list + *puc_num_pilot_carriers) = uc_i - uc_num_notched_carriers - FIRST_CARRIER;
			(*puc_num_pilot_carriers)++;
		}
	}

	/* Check if the last carrier is a pilot to include it in the pilot list */
	/* uc_i, uc_byte_index_i and uc_bit_index_in_byte_i with correct values from for loop */
	uc_bit_index_in_byte_i = uc_i & 0x07;
	if (((*(puc_pilot_map + uc_byte_index_i) >> uc_bit_index_in_byte_i) & 0x01) == 1) {
		*(puc_pilot_carriers_list + *puc_num_pilot_carriers) = uc_i - uc_num_notched_carriers - FIRST_CARRIER;
		(*puc_num_pilot_carriers)++;
	}
}

#if (defined(SMOOTHING) || defined(UPDATE_CHN_SFO_EST_PAYLOAD))

/*Applies smoothing to a channel response estimate, ass_H, at the specified carriers.
 * The value in each carrier is obtained by averaging the previous estimates in the carrier with the information from the adjacent ones.
 * Input:
 *      -puc_data_carriers_list: pointer to the list of data carriers indexes
 *      -uc_num_data_carriers: number of data carriers
 *      -pss_symbol: pointer to the input symbol
 * Output:
 *      -pss_symbol: pointer to the smoothed symbol
 */
static inline void _smooth_carriers(uint8_t *puc_data_carriers_list, uint8_t uc_num_data_carriers, q15_t *pss_symbol, uint8_t uc_scale_control)
{
	uint8_t uc_i, uc_carrier_index;

	/* Copy the array with the input estimate to the array with the final estimate */
	arm_copy_q15(pss_symbol, ass_symbol_aux_1, 2 * uc_used_carriers);

	if (uc_scale_control == SCALE_BEFORE_SUM) {
		/* Scale the values in ass_H by 1/3. */
		arm_scale_q15(pss_symbol, VALUE_1_3_Q15, 0, ass_symbol_aux, 2 * uc_used_carriers);

		/* First and last carriers are always set to the input value (no smoothing) */
		/* Applies smoothing to the remaining carriers */
		for (uc_i = 0; uc_i < uc_num_data_carriers; uc_i++) {
			uc_carrier_index = *(puc_data_carriers_list + uc_i);

			/* Real part */
			ass_symbol_aux_1[2 *
			uc_carrier_index]
				= ass_symbol_aux [2 * uc_carrier_index -
					2]  + ass_symbol_aux [2 * uc_carrier_index]  + ass_symbol_aux [2 * uc_carrier_index + 2];

			/* Imag part */
			ass_symbol_aux_1[2 * uc_carrier_index +
			1] = ass_symbol_aux [2 * uc_carrier_index - 1] + ass_symbol_aux [2 * uc_carrier_index + 1] + ass_symbol_aux [2 * uc_carrier_index + 3];
		}
	} else {
		arm_copy_q15(pss_symbol, ass_symbol_aux, 2 * uc_used_carriers);

		/* First and last carriers are always set to the input value (no smoothing) */
		/* Applies smoothing to the remaining carriers */
		for (uc_i = 0; uc_i < uc_num_data_carriers; uc_i++) {
			uc_carrier_index = *(puc_data_carriers_list + uc_i);

			/* Real part */
			ass_symbol_aux_1[2 *
			uc_carrier_index]
				= mult_real_q((ass_symbol_aux [2 * uc_carrier_index -
					2]  + ass_symbol_aux [2 * uc_carrier_index]  + ass_symbol_aux [2 * uc_carrier_index + 2]), VALUE_1_3_Q15,
					FRAC_PART_Q15);

			/* Imag part */
			ass_symbol_aux_1[2 * uc_carrier_index +
			1]
				= mult_real_q((ass_symbol_aux [2 * uc_carrier_index -
					1] + ass_symbol_aux [2 * uc_carrier_index + 1] + ass_symbol_aux [2 * uc_carrier_index + 3]), VALUE_1_3_Q15,
					FRAC_PART_Q15);
		}
	}

	/* Copy the result back to ass_H */
	arm_copy_q15(ass_symbol_aux_1, pss_symbol, 2 * uc_used_carriers);
}

#endif

/*Computes the inverse of the channel estimate stored in ass_H and stores the inverse in ass_inv_H
 * Inputs:
 * uc_length_fraccional: number of bits of the fractional part of the output
 */
static inline void _invert_channel(uint8_t uc_length_fraccional)
{
	uint8_t uc_i;
	q63_t sll_num;
	q31_t sl_den;
	int64_t sll_overflow_pos = 1LL << (FRAC_PART_Q31 + (FRAC_PART_Q15 - uc_length_fraccional));

	arm_q15_to_q31(ass_H, asl_symbol_aux, 2 * uc_used_carriers);
	for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
		/* If Re=0 and Im=0, it is fixed to the average of previous and following carriers*/
		if (!((asl_symbol_aux[2 * uc_i]) || (asl_symbol_aux[2 * uc_i + 1]))) {
			if (uc_i == 0) {
				asl_symbol_aux[2 * uc_i] = asl_symbol_aux[2 * (uc_i + 1)];
				asl_symbol_aux[2 * uc_i + 1] = asl_symbol_aux[2 * (uc_i + 1) + 1];
			} else if (uc_i == (uc_used_carriers - 1)) {
				asl_symbol_aux[2 * uc_i] = asl_symbol_aux[2 * (uc_i - 1)];
				asl_symbol_aux[2 * uc_i + 1] = asl_symbol_aux[2 * (uc_i - 1) + 1];
			} else {
				asl_symbol_aux[2 * uc_i] = (asl_symbol_aux[2 * (uc_i - 1)] >> 1) + (asl_symbol_aux[2 * (uc_i + 1)] >> 1);
				asl_symbol_aux[2 * uc_i + 1] = (asl_symbol_aux[2 * (uc_i - 1) + 1] >> 1) + (asl_symbol_aux[2 * (uc_i + 1) + 1] >> 1);
			}
		}

		/* If it still zero, its inverse will be fixed the maximum value*/
		if ((asl_symbol_aux[2 * uc_i]) || (asl_symbol_aux[2 * uc_i + 1])) {
			sl_den = mult_real_q((asl_symbol_aux[2 * uc_i] >> 1), (asl_symbol_aux[2 * uc_i] >> 1), FRAC_PART_Q31)  +
					mult_real_q((asl_symbol_aux[2 * uc_i + 1] >> 1), (asl_symbol_aux[2 * uc_i + 1] >> 1), FRAC_PART_Q31);
			/* Q3.29. Scaling must be done before square to avoid problems when (MIN_INT32)^2+(MIN_INT32)^2 */
			/* Real part*/
			div_real_q(asl_symbol_aux[2 * uc_i], sl_den, FRAC_PART_Q29, &sll_num); /* Q1.31 */
			if (sll_num >= sll_overflow_pos) {
				ass_inv_H[2 * uc_i] = MAX_INT16;
			} else if (sll_num <= -sll_overflow_pos) {
				ass_inv_H[2 * uc_i] = MIN_INT16;
			} else {
				ass_inv_H[2 * uc_i] = (int16_t)(sll_num >> (FRAC_PART_Q31 - uc_length_fraccional));
			}

			/* Imag part*/
			div_real_q(asl_symbol_aux[2 * uc_i + 1], sl_den, FRAC_PART_Q29, &sll_num);

			if (sll_num >= sll_overflow_pos) {
				ass_inv_H[2 * uc_i + 1] = MIN_INT16;
			} else if (sll_num <= -sll_overflow_pos) {
				ass_inv_H[2 * uc_i + 1] = MAX_INT16;
			} else {
				ass_inv_H[2 * uc_i + 1] = -(int16_t)(sll_num >> (FRAC_PART_Q31 - uc_length_fraccional));
			}
		} else {
			/* If real and imag part are still zero, they are assigned the max value*/
			ass_inv_H[2 * uc_i] = MAX_INT16;
			ass_inv_H[2 * uc_i + 1] = MAX_INT16;
		}
	}
}

/* Compensates the phase shift caused by the SFO in the channel estimate obtained from the preamble, FCH and S1S2. */
/* Both the compensated channel and its inverse are moved to the ATPL */

void compensate_sfo_in_chan_est(void)
{
	if (uc_used_carriers > PROTOCOL_CARRIERS) {
		uc_used_carriers = PROTOCOL_CARRIERS;
	}

	/* Compensates the sfo in the estimated channel */
	arm_cmplx_mult_cmplx_q15(ass_H, ass_rotation_sfo, ass_H, uc_used_carriers);
	arm_shift_q15(ass_H, Q3_13_TO_Q1_15 + 1, ass_H, 2 * uc_used_carriers);

	/* Computes the inverse of the estimated channel in Q(16-NUM_BITS_FRAC_PART_INV_H).NUM_BITS_FRAC_PART_INV_H */
	_invert_channel(NUM_BITS_FRAC_PART_INV_H);
	/*_complex_inversion((int16_t *)ass_H, uc_used_carriers, NUM_BITS_FRAC_PART_INV_H, (int16_t *)ass_inv_H);*/

	/* Scaling to the same precision of the X/Y output */
	arm_scale_q15(ass_inv_H, GAIN_SCALE_INV_H_FEQ, 0, ass_inv_H, 2 * uc_used_carriers);

	/* Moves the inverse of the channel to symbol 0 of the ATPL */
	_invert_byte_order(ass_inv_H, 2 * uc_used_carriers);
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, 0x00); /* Set destination to symbol 0 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER);
	pplc_if_write_jump((BCODE_ZONE2 | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)ass_inv_H, 2 * uc_used_carriers * 2, JUMP_COL_1);
}

void chn_estimation_from_syncp(void)
{
	uint8_t uc_i;

	/* Disable RSSI calculation for channel estimation */
	atpl250_disable_rssi();
	atpl250_disable_evm();

	/* Read how many P symbols are available */
	uc_num_p_sym = ((pplc_if_read8(REG_ATPL250_SYNCM_CTL_L8) & 0x7C) >> 2);

	/* If there are more SYNCP symbols than NUM_SYM_H_EST_PRE-1, truncate it to use NUM_SYM_H_EST_PRE-1 */
	if (uc_num_p_sym > (NUM_SYM_H_EST_PRE - 1)) {
		uc_num_p_sym = NUM_SYM_H_EST_PRE - 1;
	}

	uc_num_sym_valid_preamble = uc_num_p_sym + 1;

	/* Block IOB, so no FCH symbols enter until finished */
	atpl250_clear_rx_mode();
	atpl250_clear_tx_mode();

	/* ************************************* Read symbols from ATPL and multiply by conjugate of reference symbols */
	/* Obtain modulating symbols. This is needed for the cases when tone mask exists. To extract values from conj_syncp. */
	_obtain_modulating_conj_complex_values((uint8_t *)auc_zeros, uc_used_carriers, PREAMBLE_FCH_S1S2_PART, auc_unmasked_carrier_list,
			ass_modulating_symbols);

	/* ********* Processes the received SYNCPs */
	for (uc_i = 0; uc_i < uc_num_p_sym; uc_i++) {
		/* Moves the SYNCP to the SAM4 */
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER + ((CFG_SYMBOLS_IN_IOB - uc_num_p_sym + uc_i) * CFG_IOB_SAMPLES_PER_SYMBOL));
		pplc_if_read_jump((BCODE_ZONE4 | (FIRST_CARRIER + auc_unmasked_carrier_list[0]
				+ ((CFG_SYMBOLS_IN_IOB - uc_num_p_sym + uc_i) * CFG_IOB_SAMPLES_PER_SYMBOL))),
				(uint8_t *)(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i), 2 * uc_used_carriers * 2, JUMP_COL_1);
		/* Invert byte order */
		_invert_byte_order((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i), 2 * uc_used_carriers);

		/* Multiply by conjugate of the reference symbol */
		/* Result in Q3.13 */
		arm_cmplx_mult_cmplx_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i), ass_modulating_symbols,
				(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i), uc_used_carriers);
	}

	/* Averaging. By now, symbols are scaled by 1/2^2 to avoid overflow and summed. */
	/* The division by the number of symbols is done at the end, once the number of FCH and S1S2 are known. */
	memset(ass_average_symbol, 0, 2 * uc_used_carriers * sizeof(q15_t)); /* Clean values from previous frame */
	/* Result in Q5.11. This avoid overflow as long as less than 16 symbols are used (preamble+fch+s1s2) */
	for (uc_i = 0; uc_i < uc_num_p_sym; uc_i++) {
		arm_shift_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_i), -2, ass_symbol_aux, 2 * uc_used_carriers);
		arm_add_q15(ass_symbol_aux, ass_average_symbol, ass_average_symbol, 2 * uc_used_carriers); /* Q5.11 */
	}

	/* Scales demodulated P by 2^2 to compensate for the attenuation in complex by complex product */
	arm_shift_q15(ass_pm_fch_s1s2_symbols, 2, ass_pm_fch_s1s2_symbols, 2 * uc_used_carriers * uc_num_p_sym); /* Q1.15 */

	/* Set rx mode again to unblock IOB */
	atpl250_set_rx_mode();
}

void chn_estimation_from_preamble(void)
{
	/* Block IOB, so no FCH symbols enter until finished */
	atpl250_clear_rx_mode();
	atpl250_clear_tx_mode();

	/* ********* Processes the received SYNCM with displace FFT window ************** */
	/* Obtain modulating symbols. This is needed for the cases when tone mask exists. To extract values from conj_syncp */
	_obtain_modulating_conj_complex_values((uint8_t *)auc_ones, uc_used_carriers, PREAMBLE_FCH_S1S2_PART, auc_unmasked_carrier_list,
			ass_modulating_symbols);
	/* Moves the M symbol to the SAM4 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER);
	pplc_if_read_jump((BCODE_ZONE4 | (FIRST_CARRIER + auc_unmasked_carrier_list[0])),
			(uint8_t *)(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), 2 * uc_used_carriers * 2, JUMP_COL_1);
	_invert_byte_order((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), 2 * uc_used_carriers);

	/* Multiply by conjugate of the reference symbol, including phase shift to the displacement of the FFT window in the SYNCM*/
	/* Result in Q3.13 */
	arm_cmplx_mult_cmplx_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), ass_modulating_symbols,
			(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), uc_used_carriers);
	/* Result in Q5.11 */
	arm_cmplx_mult_cmplx_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), (q15_t *)ass_syncm_shift,
			(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), uc_used_carriers);

#ifdef SMOOTHING
	_smooth_carriers(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch, (ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym),
			SCALE_BEFORE_SUM);
#endif

	/* Averaging. By now, symbols are scaled by 1/2^2 to avoid overflow and summed. */
	/* The division by the number of symbols is done at the end, once the number of FCH and S1S2 are known. */
	/* Result in Q5.11. This avoid overflow as long as less than 16 symbols are used (preamble+fch+s1s2) */
	/* arm_shift_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), -2, ass_symbol_aux, 2 * uc_used_carriers); */
	arm_add_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), ass_average_symbol, ass_average_symbol, 2 * uc_used_carriers); /* Q5.11 */

	/* Scales demodulated M by 2^2 to compensate for the attenuation in complex by complex product */
	arm_shift_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym), 4, (ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_num_p_sym),
			2 * uc_used_carriers); /* Q1.15 */

	pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, 0xFE);

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
	uint8_t uc_ichannel_idx;
	uint16_t us_bit_index;
	uint8_t uc_byte_index, uc_rel_bit_index;
	uint8_t uc_local_bit_index, uc_local_byte_index, uc_rel_local_bit_index, uc_local_buf_index;
	uint8_t auc_use_fch_symbol[SYMBOLS_8] = {1, 1, 1, 1, 1, 1, 1, 1}; /* ToDo: By now all FCH symbols are valid. Change to use only those with BER<Threshold
	                                                                  **/
	uint8_t uc_next_fch_computed;

	/* Disable RSSI calculation for channel estimation */
	atpl250_disable_rssi();
	atpl250_disable_evm();

	if (uc_used_carriers > PROTOCOL_CARRIERS) {
		uc_used_carriers = PROTOCOL_CARRIERS;
	}

	/* Disable HW chain */
	pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

	/* Read data transmitted for channel estimation */
	pplc_if_write16(REG_ATPL250_INTERLEAVER_SPI_H16, 0x8000); /* SPI2INT_TXRX='1'; ADDR_SPI2INT=0 */
	pplc_if_read_jump(REG_ATPL250_INTERLEAVER_SPI_VL8, u_shared_buffers.s_fch_int.interleaver_matrix_tx, uc_fch_interleaver_byte_size, 1);

	/* Convert into a buffer for each symbol */
	uc_local_buf_index = 0;
	uc_local_bit_index = 0;
	uc_local_byte_index = 0;
	uc_rel_local_bit_index = 0;
	memset(&u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[0][0], 0, sizeof(u_shared_buffers.s_fch_int.auc_tx_bits_interleaver));

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
	for (uc_i = 0; uc_i < uc_num_sym_used_fch; uc_i++) {
		if (auc_use_fch_symbol[SYMBOLS_8 - NUM_SYM_H_EST_FCH + uc_i]) {
			auc_index_sym_valid_fch[uc_num_sym_valid_fch] = SYMBOLS_8 - NUM_SYM_H_EST_FCH + uc_i;
			uc_num_sym_valid_fch++;
		}
	}

	memset(&auc_previous_ref_bits_interleaver[0], 0, uc_fch_symbol_byte_size);
	for (uc_i = 0; uc_i < (uc_num_symbols_fch - SYMBOLS_8); uc_i++) {
		for (uc_j = 0; uc_j < uc_fch_symbol_byte_size; uc_j++) {
			auc_previous_ref_bits_interleaver[uc_j]
				= auc_previous_ref_bits_interleaver[uc_j] ^ u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_i][uc_j];
		}
	}

	uc_next_fch_computed = 0;
	for (uc_i = 0; uc_i < uc_num_sym_valid_fch; uc_i++) {
		/* Get reference to construct header symbol from P and M symbols */
		for (uc_k = uc_next_fch_computed; uc_k <= auc_index_sym_valid_fch[uc_i]; uc_k++) {
			for (uc_j = 0; uc_j < uc_fch_symbol_byte_size; uc_j++) {
				auc_previous_ref_bits_interleaver[uc_j] = auc_previous_ref_bits_interleaver[uc_j] ^
						u_shared_buffers.s_fch_int.auc_tx_bits_interleaver[uc_num_symbols_fch - SYMBOLS_8 + uc_k][uc_j];
			}
		}

		uc_ichannel_idx = (auc_index_sym_valid_fch[uc_i] + 7) % 8;         /* the last symbols is the first */

		uc_next_fch_computed = auc_index_sym_valid_fch[uc_i] + 1;

		/* Move FCH raw symbol to the SAM4 and demodulate */
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (uc_ichannel_idx * CFG_IOB_SAMPLES_PER_SYMBOL) + LAST_CARRIER);
		pplc_if_read_jump((BCODE_ZONE4 | (FIRST_CARRIER + auc_unmasked_carrier_list[0] + (CFG_IOB_SAMPLES_PER_SYMBOL * uc_ichannel_idx))),
				(uint8_t *)ass_symbol_aux, 2 * uc_used_carriers * 2, JUMP_COL_1);
		_invert_byte_order(ass_symbol_aux, 2 * uc_used_carriers);
		_obtain_modulating_conj_complex_values(&auc_previous_ref_bits_interleaver[0], uc_used_carriers, PREAMBLE_FCH_S1S2_PART,
				auc_unmasked_carrier_list, ass_modulating_symbols);
		arm_cmplx_mult_cmplx_q15(ass_symbol_aux, ass_modulating_symbols, ass_symbol_aux, uc_used_carriers);   /* Result in Q3.13 */
		/* Q1.15 */
		arm_scale_q15(ass_symbol_aux, VALUE_SQRT_2_Q13, 4, (ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_i)),
				2 * uc_used_carriers);

		#ifdef SMOOTHING
		_smooth_carriers(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch,
				(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_i)), SCALE_BEFORE_SUM);
		#endif
	}

	/* Averaging */
	/* Sum FCH symbols to previous sum */
	for (uc_i = 0; uc_i < uc_num_sym_valid_fch; uc_i++) {
		/* Q5.11 */
		arm_shift_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_i)), -4, ass_symbol_aux, 2 * uc_used_carriers);

		arm_add_q15(ass_symbol_aux, ass_average_symbol, ass_average_symbol, 2 * uc_used_carriers);
	}

	/* If the payload is coherently modulated, final channel estimate will be obtained with S1S2. */
	/* If payload is differentially modulated, the estimate of the symbol noise energy must be obtained for */
	/* SFO estimation. */
	if (uc_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
		/* Scale averaged symbol and convert to Q1.31 */
		arm_scale_q15(ass_average_symbol, ass_scale[uc_num_sym_valid_preamble + uc_num_sym_valid_fch - 2], 4, ass_average_symbol, 2 * uc_used_carriers);
		arm_q15_to_q31(ass_average_symbol, asl_average_symbol, 2 * uc_used_carriers); /* Q1.31 */

		/* ************************************ Estimation of the channel response by weighted averaging */
		/* Computes the noise energy in each symbol */
		_compute_symbol_noise(uc_num_sym_valid_preamble + uc_num_sym_valid_fch);

		/* Computes the weighting factors */
		_compute_weights(uc_num_sym_valid_preamble + uc_num_sym_valid_fch, 4);
	}

	/* Clear SOURCE_H for further demodulations */
	pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, (uint8_t)(~0x01u)); /* SOURCE_H='0' */

	/* Modulation will be changed on first payload demodulation, there is no need to set it here */

	/* Enable HW chain */
	pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x01);

	/* Change value to modulator abcd points */
	/* pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_fullgain, ABCD_POINTS_LEN); */

	/* Enable RSSI calculation after channel estimation */
	atpl250_enable_rssi();
	atpl250_enable_evm();
}

/**
 * \brief Estimate channel using the preamble, FCH and S1S2
 *
 */
void chn_estimation_from_s1s2(void)
{
	uint8_t uc_i;

	/* ************************************ Moves S1 and S2 to the SAM4 and obtain LS estimation of the channel from them */
	/* Obtain modulating symbols. This is needed for the cases when tone mask exists. To extract values from conj_syncp */
	_obtain_modulating_conj_complex_values((uint8_t *)auc_ones, uc_used_carriers, PREAMBLE_FCH_S1S2_PART, auc_unmasked_carrier_list,
			ass_modulating_symbols);
	/* Moves the S1 to the SAM4 and multiplies by the conjugate of the reference symbol */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER);
	pplc_if_read_jump((BCODE_ZONE4 | (FIRST_CARRIER + auc_unmasked_carrier_list[0])),
			(uint8_t *)(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)),
			2 * uc_used_carriers * 2, JUMP_COL_1);
	_invert_byte_order((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), 2 * uc_used_carriers);
	/* Multiply by conjugate of the reference symbol */
	arm_cmplx_mult_cmplx_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), ass_modulating_symbols,
			(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), uc_used_carriers);

#ifdef SMOOTHING
	_smooth_carriers(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch,
			(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), SCALE_BEFORE_SUM);
#endif

	/* Obtain modulating symbols. This is needed for the cases when tone mask exists. To extract values from conj_syncp */
	_obtain_modulating_conj_complex_values((uint8_t *)auc_zeros, uc_used_carriers, PREAMBLE_FCH_S1S2_PART, auc_unmasked_carrier_list,
			ass_modulating_symbols);
	/* Moves the S2 to the SAM4 and multiplies by the conjugate of the reference symbol */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, CFG_IOB_SAMPLES_PER_SYMBOL + LAST_CARRIER);
	pplc_if_read_jump((BCODE_ZONE4 | (CFG_IOB_SAMPLES_PER_SYMBOL + FIRST_CARRIER + auc_unmasked_carrier_list[0])),
			(uint8_t *)(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)),
			2 * uc_used_carriers * 2, JUMP_COL_1);
	_invert_byte_order((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), 2 * uc_used_carriers);
	/* Multiply by conjugate of the reference symbol */
	arm_cmplx_mult_cmplx_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)),
			ass_modulating_symbols,
			(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), uc_used_carriers);

#ifdef SMOOTHING
	_smooth_carriers(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch,
			(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), SCALE_BEFORE_SUM);
#endif

	/* Scale S1S2 and sum */
	arm_shift_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), -2, ass_symbol_aux,
			2 * uc_used_carriers);
	arm_add_q15(ass_symbol_aux, ass_average_symbol, ass_average_symbol, 2 * uc_used_carriers);
	arm_shift_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), -2, ass_symbol_aux,
			2 * uc_used_carriers);
	arm_add_q15(ass_symbol_aux, ass_average_symbol, ass_average_symbol, 2 * uc_used_carriers);

	/* Scale the average symbol */
	arm_scale_q15(ass_average_symbol, ass_scale[uc_num_sym_valid_preamble + uc_num_sym_valid_fch], 4, ass_average_symbol, 2 * uc_used_carriers);

	/* Scales the demodulated S1 and S2 by 2^2 */
	arm_shift_q15((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), 2,
			(ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), 2 * uc_used_carriers * 2);

	/* Converts to Q1.31 */
	arm_q15_to_q31(ass_average_symbol, asl_average_symbol, 2 * uc_used_carriers); /* Q1.31 */

	/* ************************************ Estimation of the channel response by weighted averaging */
	/* Computes the noise energy in each symbol */
	_compute_symbol_noise(uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 2);

	/* Computes the weighting factors */
	_compute_weights((uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 2), 4);

	/* Estimates the weighted averaging */
	memset(ass_H, 0, 2 * uc_used_carriers * sizeof(q15_t));
	for (uc_i = 0; uc_i < (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 2); uc_i++) {
		/* Multiplies the LS estimates by their Wi and sum */
		arm_scale_q15((ass_pm_fch_s1s2_symbols + uc_i * 2 * uc_used_carriers), *(ass_Wi + uc_i), 0, ass_symbol_aux, 2 * uc_used_carriers);
		arm_add_q15(ass_symbol_aux, ass_H, ass_H, 2 * uc_used_carriers);
	}
}

/**
 * \brief Estimate channel from pilots received in payload
 *
 */

void chn_estimation_from_pilots(uint8_t *puc_lfsr_pilots_ch_est_pilots, uint8_t *puc_static_dynamic_notch_map, uint8_t *puc_pilots_map,
		uint8_t *puc_pilots_per_symbol, uint8_t uc_num_pilots_per_block)
{
	#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
	uint8_t uc_i;
	uint8_t uc_num_data_carriers = 0, uc_num_pilot_carriers = 0;
	#endif

	#ifndef UPDATE_CHN_SFO_EST_PAYLOAD
	UNUSED(puc_lfsr_pilots_ch_est_pilots);
	UNUSED(puc_static_dynamic_notch_map);
	UNUSED(puc_pilots_map);
	#endif

	/* ***** Configures INOUT Buffer */
	/* Disable RSSI calculation for channel estimation */
	atpl250_disable_rssi();
	atpl250_disable_evm();

	/* ******Estimates channel from pilots */

	if (uc_used_carriers > PROTOCOL_CARRIERS) {
		uc_used_carriers = PROTOCOL_CARRIERS;
	}

	/* Move pilot carriers to SAM4 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER + CFG_IOB_SAMPLES_PER_SYMBOL * (NUM_SYM_PILOTS_H_EST - 1));
	pplc_if_set_low_speed();
	/* Be careful: when there are masked or inactive tones NUM_PILOTS_PER_SYMBOL*NUM_PILOTS_PER_SYMBOL is not equal to uc_num_pilot_carriers */
	if (uc_num_pilots_per_block > PROTOCOL_CARRIERS) {
		uc_num_pilots_per_block = PROTOCOL_CARRIERS;
	}

	pplc_if_read_jump((BCODE_ZONE4 | (*puc_pilots_per_symbol + FIRST_CARRIER)), (uint8_t *)ass_symbol_aux, uc_num_pilots_per_block * 2 * 2, JUMP_PILOTS);
	pplc_if_set_high_speed();

	/* When channel and SFO estimation is not updated, pilots are read to be marked by HW but nothing more is done */
	#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
	_invert_byte_order(ass_symbol_aux, uc_num_pilots_per_block * 2);

	/* Calculates the indexes of the data carriers to be smoothed in the current set of symbols */
	control_smooth(puc_static_dynamic_notch_map, puc_pilots_map, auc_data_carriers_list, &uc_num_data_carriers, auc_pilot_carriers_list,
			&uc_num_pilot_carriers);

	_obtain_modulating_conj_complex_values(puc_lfsr_pilots_ch_est_pilots, uc_num_pilots_per_block * 2, PAYLOAD_PART, puc_pilots_per_symbol,
			ass_modulating_symbols);

	/* Demodulate and place pilots in their corresponding carrier index. Result in Q3.13 */
	for (uc_i = 0; uc_i < uc_num_pilots_per_block; uc_i++) {
		ass_Ypilots[2 *
		puc_pilots_per_symbol[uc_i]]
			= mult_real_q(ass_symbol_aux[2 * uc_i], ass_modulating_symbols[2 * uc_i],
				FRAC_PART_Q17) - mult_real_q(ass_symbol_aux[2 * uc_i + 1], ass_modulating_symbols[2 * uc_i + 1], FRAC_PART_Q17);
		ass_Ypilots[2 * puc_pilots_per_symbol[uc_i] +
		1]
			= mult_real_q(ass_symbol_aux[2 * uc_i + 1], ass_modulating_symbols[2 * uc_i],
				FRAC_PART_Q17) + mult_real_q(ass_symbol_aux[2 * uc_i], ass_modulating_symbols[2 * uc_i + 1], FRAC_PART_Q17);
	}
	arm_scale_q15(ass_Ypilots, VALUE_SQRT_2_Q13, 4, ass_Ypilots, 2 * uc_used_carriers); /* Q1.15 */

	/* Update channel in pilot carriers */
	for (uc_i = 0; uc_i < uc_num_pilot_carriers; uc_i++) {
		ass_H[ 2 *
		auc_pilot_carriers_list[uc_i] ]
			= mult_real_q(ass_H[ 2 * auc_pilot_carriers_list[uc_i] ], COMP_ALPHA_CHANNEL_EST_Q15,
				FRAC_PART_Q15) + mult_real_q(ass_Ypilots[ 2 * auc_pilot_carriers_list[uc_i] ], ALPHA_CHANNEL_EST_Q15, FRAC_PART_Q15);
		ass_H[ 2 * auc_pilot_carriers_list[uc_i] + 1 ] = mult_real_q(ass_H[ 2 * auc_pilot_carriers_list[uc_i] + 1 ], COMP_ALPHA_CHANNEL_EST_Q15,
				FRAC_PART_Q15) + mult_real_q(ass_Ypilots[ 2 * auc_pilot_carriers_list[uc_i] + 1],
				ALPHA_CHANNEL_EST_Q15, FRAC_PART_Q15);
	}

	/* Smooth data carriers */
	_smooth_carriers(auc_data_carriers_list, uc_num_data_carriers, ass_H, SCALE_BEFORE_SUM);

	/* Computes the inverse of the estimated channel in Q(16-NUM_BITS_FRAC_PART_INV_H).NUM_BITS_FRAC_PART_INV_H */
	_invert_channel(NUM_BITS_FRAC_PART_INV_H);

	/* Scaling to the precision of the X/Y block */
	arm_scale_q15(ass_inv_H, GAIN_SCALE_INV_H_FEQ, 0, ass_inv_H, 2 * uc_used_carriers);

	/* Moves the inverse of the channel to symbol 0 of the ATPL */
	_invert_byte_order(ass_inv_H, 2 * uc_used_carriers);
	pplc_if_write8(REG_ATPL250_INOUTB_CONF2_L8, 0x00); /* Set destination to symbol 0 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER);
	pplc_if_write_jump((BCODE_ZONE2 | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)ass_inv_H, 2 * uc_used_carriers * 2, JUMP_COL_1);

	/* Configure overflow as it was before entering */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * (NUM_SYM_PILOTS_H_EST - 1)) + LAST_CARRIER);
	#endif

	/* Enable RSSI calculation from channel estimation */
	atpl250_enable_rssi();
	atpl250_enable_evm();
}
