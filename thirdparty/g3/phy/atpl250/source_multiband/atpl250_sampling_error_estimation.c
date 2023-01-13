/**
 * \file
 *
 * \brief ATPL250 Sampling Frequency Error Estimation
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
#include "atpl250_sampling_error_estimation.h"
#include "atpl250_mod_demod.h"

/* Extern PHY Variables */
extern struct band_phy_constants s_band_constants;
extern uint8_t uc_working_band;
extern uint8_t uc_legacy_mode;
extern uint8_t uc_notched_carriers;
extern uint8_t uc_used_carriers;
extern int32_t asl_freq_index[], asl_freq_index_squared[];
extern int32_t asl_delay_symbols[];
extern uint8_t uc_num_symbols_fch;

extern q31_t asl_inv_Ni[];
extern q15_t ass_Wi[];

/*-----------------------For debugging ----------------------------------*/
#ifdef DEBUG_CHN_SFO
extern uint8_t uc_num_block_pilots;
q31_t asl_delta_t_vector[10] = {0}, asl_delta_t_est_vector[10] = {0};
q31_t asl_SNRk_vector[NUM_CARRIERS_FCC * 8] = {0};
q31_t asl_Nk_vector[NUM_CARRIERS_FCC * 8] = {0};
q31_t asl_modulus_vector[NUM_CARRIERS_FCC * 8] = {0};
q31_t asl_cum_complex_error_vector[NUM_CARRIERS_TIMES_2_FCC * 8] = {0};
q31_t asl_numerator_pil_vector[8] = {0}, asl_denominator_pil_vector[8] = {0};

q31_t sl_SNRk_by_freq, sl_SNRk_by_freq_squared, sl_distance_div_pst_sum;
uint16_t aus_distance_vector[NUM_CARRIERS_FCC * 8] = {0};
uint8_t auc_pilot_used_twice_cum_vector[NUM_CARRIERS_FCC * 8] = {0};
uint16_t aus_pilot_stored_vector[2][NUM_CARRIERS_FCC * 8] = {{0}};

#endif
/*----------------------------------------------------------------------*/

/* Actual number of SYNCP and FCH symbols used for channel estimation. */
/* Maximum number of SYNCP is NUM_SYM_H_EST_PRE and maximum number of fch symbols is NUM_SYM_H_EST_FCH */
extern uint8_t uc_num_sym_valid_preamble, uc_num_sym_p_detected;
extern uint8_t uc_num_sym_valid_fch;
extern uint8_t auc_index_sym_valid_fch[];

/* Array for storing the p,m fch and s1s2 symbols used for channel estimation */
extern union shared_preamble_and_payload_symbols u_shared_preamble_and_payload_symbols;
extern q15_t ass_H[];
extern q31_t asl_symbol_aux[];
extern q15_t ass_average_symbol[];
extern uint8_t auc_unmasked_carrier_list[];
extern q15_t ass_average_symbol_fch_fcc[];

/*Distance from each symbol to the average symbol*/
static q31_t asl_distance[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH_CENELEC_A + 2];

/*Weighting factors used for the estimation of the channel response but stored in Q1.31*/
/*static q31_t asl_Wi[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH_CENELEC_A + 2];*/

/*Phase that compensates the SFO and complex vector that compensates this phase in the estimated H*/
/*static q31_t asl_phase_first[NUM_CARRIERS_FCC];*/

/* Estimate of the sampling error period */
q31_t sl_delta_t;
q31_t sl_sfo_time_offset;
int8_t sc_sfo_time_offset;
#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
static q31_t sl_delta_t_est;
#endif

/* Absolute pointer of the INOB */
uint8_t uc_index_inob_abs;

/* Number of FCH symbols for updating H in FCC legacy mode */
int16_t ss_num_sym_fch_for_H_update;

/* Number of FCH symbols since last update of the channel estimate */
int16_t ss_num_fch_since_last_H_update;

/*  Value of the resampling register*/
uint8_t uc_num_s1s2; /* Need to be global to be used in assembly  function */
static int32_t sl_resamp_reg;
static int32_t sl_step; /* Step with respect to the actual value of sl_resamp_reg */

uint8_t uc_half_est_delay_sfo_est; /* Need to be global to be used in assembly  function */

/* Rotation vectors*/
struct phase_shift_and_rotation {
	q31_t asl_phase_shift[NUM_CARRIERS_TIMES_2_FCC];
	q31_t asl_rotation_sfo[NUM_CARRIERS_TIMES_2_FCC];
};
struct rotation_vectors_coh {
	q15_t ass_rotation_sfo_first[NUM_CARRIERS_TIMES_2_FCC];
	q15_t ass_rotation_sfo_second[NUM_CARRIERS_TIMES_2_FCC];
};
union shared_rotation_vectors {
	struct phase_shift_and_rotation s_phase_shift_and_rotation;
	struct rotation_vectors_coh s_rotation_vectors_coh;
};
union shared_rotation_vectors s_shared_rotation_vectors;

/* To be used in assembly functions and other modules */
q15_t *pss_rotation_sfo_first = s_shared_rotation_vectors.s_rotation_vectors_coh.ass_rotation_sfo_first;
q15_t *pss_rotation_sfo_second = s_shared_rotation_vectors.s_rotation_vectors_coh.ass_rotation_sfo_second;
q15_t *pss_average_symbol;

/* The following variables are used in atpl250_mod_demod.c even if channel estimation based on pilots is not accomplished*/
/* Stores the symbol index at which the pilots stored in ass_Ypilots appeared */
uint16_t aus_pilot_stored[2][NUM_CARRIERS_FCC] = {{0}};

/* Partial values of the numerator and denominator */
q31_t sl_partial_num, sl_partial_den;  /* Need to be global to be used in assembly function */

/* Contains cos n and sin n, for n=0,0.5, 1, 1.5, 2, 2.5, 3, 3.5,...11 in Q2.30 */
const q31_t asl_cos_n[23] = {1073741824, 942297101, 580145183, 75953492, -446834263, -860221407, -1062996349, -1005512712, -701844494,
			     -226340266, 304579952, 760928376, 1030974995, 1048602979, 809496382, 372196838, -156229472,
			     -646405358, -978318669, -1070705450, -900946194, -510603888, 4752057};
const q31_t asl_sin_n[23] = {0, 514779252, 903522590, 1071052086, 976350678, 642604572, 151526455, -376650623, -812610492, -1049614972,
			     -1029637100, -757568156, -300020107, 230983328, 705433989, 1007169806, 1062315328,
			     857369009, 442508854, -80692901, -584138220, -944566130, -1073731308};

#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)

/* Stores the values of the received pilots in successive blocks */
/*q31_t asl_Ypilots [2][NUM_CARRIERS_TIMES_2_FCC] = {{0}};*/
q15_t ass_Ypilots [2][NUM_CARRIERS_TIMES_2_FCC] = {{0}};

/* Index of ass_Ypilot row that contains the current and previous sets of pilots that have been received */
static uint8_t uc_index_current_pilot_set, uc_index_previous_pilot_set;

/* Distance between the received pilots */
static uint16_t aus_distance[NUM_CARRIERS_FCC] = {0};

/* Number of times that a given pilot has been received twice in the current payload */
static uint8_t auc_pilot_used_twice_cum[NUM_CARRIERS_FCC] = {0};

/* Cumulative complex vector with phase error */
static q31_t asl_cum_complex_error[NUM_CARRIERS_TIMES_2_FCC] = {0};

/*q31_t asl_cum_complex_error_aux[NUM_CARRIERS_TIMES_2_FCC] = {0};*/  /* FOR TESTING */

/* per-carrier Noise and SNR */
q31_t asl_Nk[NUM_CARRIERS_FCC] = {0};  /* Need to be global to be used in assembly function */
static q31_t asl_SNRk[NUM_CARRIERS_FCC] = {0};

/*q31_t asl_Nk_aux[NUM_CARRIERS_FCC] = {0};*/
q31_t *psl_Nk;  /* Need to be global to be used in assembly function */

/* Need to be global to be used in assembly function */
q31_t sl_modulus;
q31_t sl_numerator_pil = 0, sl_denominator_pil = 0;
/*q31_t sl_numerator_aux = 0, sl_denominator_aux = 0;*/

uint8_t uc_pilot_used_twice_cum; /* Need to be global to be used in assembly function */

#endif

/* Declaration of assembler functions */
extern void swap_bytes_asm(int16_t *pss_input_array, uint16_t us_num_elements);
extern void zero_complex_vector_q_asm(uint8_t *psc_address, uint16_t us_num_bytes_in_vector);
extern void compute_partial_num_dem_sfo_cen_a_asm(q15_t *pss_input_symbol);
extern void compute_partial_num_dem_sfo_fcc_asm(q15_t *pss_input_symbol);
extern void compute_partial_num_dem_sfo_arib_asm(q15_t *pss_input_symbol);
extern void compute_ass_rotation_sfo_asm(int16_t *pss_Wi, int32_t *psl_distance, uint8_t uc_num_symbols, int32_t sl_delta_t);

#ifdef  UPDATE_CHN_SFO_EST_PAYLOAD
extern void cum_phase_error_and_carrier_SNR_num_dem_asm(int16_t *pss_Ypilot_current_set, int16_t *pss_Ypilot_previous_set,
		int32_t *psl_cum_complex_error, int16_t *pss_H);
extern void compute_partial_num_dem_sfo_pil_asm(int32_t *psl_cum_complex_error, int32_t *psl_SNRk, int32_t *psl_freq_index, uint16_t *pus_distance);

#endif

/* Defines fixed point multiplication of real values in Quc_q_int.uc_q_frac using half-up rounding. */
#define mult_real_q(sl_a, sl_b, uc_q_frac)  (int32_t)((((int64_t)sl_a * (int64_t)sl_b) + (1 << (uc_q_frac - 1))) >> uc_q_frac)

/* Defines fixed point multiplication of real values in Q1.31 using half-even rounding. */

/*
 * // static inline int64_t mult_real_q31(int32_t a, int32_t b){
 * //   int64_t sll_product, sll_scaling;
 * //
 * //   sll_product =  (int64_t) a * (int64_t) b;
 * //
 * //   sll_scaling = ( (sll_product & 0x0000000080000000) >> 31) ^ 0x0000000000000001;
 * //
 * //   sll_product= (sll_product + 0x0000000040000000 - sll_scaling) >> 31;
 * //
 * //   return sll_product;
 * //
 * // }
 */

/* Modulus of a complex value */
#define modulus_complex_q(sl_real, sl_imag, uc_q_frac) (mult_real_q(sl_real, sl_real, uc_q_frac) + mult_real_q(sl_imag, sl_imag, uc_q_frac))

/* Defines division of Q(16-uc_q_frac values).uc_q_frac values and Q(32-uc_q_frac values).uc_q_frac */
/* div_real_q(dividend,divisor,quotient), the sum of divisor >>1 is for rounding instead of truncating. */
#define div_real_q(sl_dividend, sl_divisor, uc_q_frac, psl_quotient) do { \
		*psl_quotient = ((((int64_t)sl_dividend) << uc_q_frac) + (((int64_t)sl_divisor) >> 1)) / ((int64_t)sl_divisor);	\
} \
	while (0)

/*Estimates the sampling timing error using the last valid symbols of the preamble, last 8 FCH symbols and S1S2
 * Input: Uses the following static/extern variables
 *      -uc_num_sym_valid_preamble
 *      -asl_delay_symbols
 *      -asl_freq_index
 *      -u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
 *      -asl_average_symbol
 *      -ass_inv_Ni
 */
void sampling_error_est_from_preamble_fch_s1s2(uint8_t uc_mod_scheme, uint8_t uc_sfo_objective)
{
	uint8_t uc_k;
	q31_t sl_Ni_by_avg;
	q31_t sl_numerator = 0, sl_denominator = 0;
	q63_t sll_delta_t_scaled;
	q31_t sl_average_distance;
	uint8_t uc_i;
	q31_t sl_phase_shift;
	uint8_t uc_phase_turns;
	uint8_t uc_index_inob_rel;
	q31_t sl_fix_theta, sl_aprox_theta1_real, sl_aprox_theta1_imag;
	uint8_t uc_index_vector_n;

	if ((uc_working_band == WB_FCC) && (uc_sfo_objective == SFO_FOR_FCH_COH)) {
		uc_num_s1s2 = 0;
		pss_average_symbol = ass_average_symbol_fch_fcc;
	} else {
		/* If payload is differentially modulated, there are no S1S2 */
		if (uc_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
			uc_num_s1s2 = 0;
		} else {
			uc_num_s1s2 = 2;
		}

		pss_average_symbol = ass_average_symbol;
	}

	/* Obtain distances for the actual case */
	for (uc_k = 0; uc_k < uc_num_sym_valid_preamble; uc_k++) {
		asl_distance[uc_k] = *(asl_delay_symbols + (NUM_FULL_SYMBOLS_PREAMBLE - uc_num_sym_valid_preamble) + uc_k);
	}
	for (uc_k = 0; uc_k < uc_num_sym_valid_fch; uc_k++) {
		asl_distance[uc_num_sym_valid_preamble + uc_k] = *(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE +  auc_index_sym_valid_fch[uc_k]);
	}
	for (uc_k = 0; uc_k < uc_num_s1s2; uc_k++) {
		asl_distance[ uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_k] = *(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + uc_k);
	}

	/* To equalize the received symbols */
	arm_cmplx_conj_q15(pss_average_symbol, pss_average_symbol, uc_used_carriers);

	/* Computes the average distance */
	/* Average distance in Q1.31 scaled by 1/2^SCALING_DELAY_VALUES */
	arm_mean_q31(asl_distance, (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_num_s1s2), &sl_average_distance);

	/* Computes the numerator and denominator using the vector dot product function of the arm library */
	for (uc_k = 0; uc_k < (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_num_s1s2); uc_k++) { /* time dimension sum */
		/* Computes product of 1/Ni by the distance value for the current index */
		/* Q1.31 scaled by 1/2^SCALING_DELAY_VALUES */
		sl_Ni_by_avg = mult_real_q(*(asl_inv_Ni + uc_k), (asl_distance[uc_k] - sl_average_distance), FRAC_PART_Q31);

		/* Compute the partial numerator (sl_partial_num) and denominator (sl_partial_den) of the quotient that yields the sll_delta_t_scaled */

		/* It does the following operations:
		 *   1. Equalizes each symbol with the complex conjugate of the average symbol. Result is in Q3.29
		 *   2. Obtains sl_partial_num as the dot product of the imaginary part of 1) with asl_freq_index.
		 *      Scales 1/2^(SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION) to avoid overflow
		 *      in the sum that yields sl_numerator.
		 *      Result is Q18.46 scaled by 1/2^(SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION) and 1/2^SCALING_FREQ_VALUES.
		 *   3. Obtains sl_partial_den as the dot product of the real part of 1) with asl_freq_index_squared.
		 *      Scales 1/2^(SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION) to avoid overflow
		 *      in the sum that yields sl_denominator.
		 *      Result is Q18.46 scaled by 1/2^(SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION) and  1/2^(2*SCALING_FREQ_VALUES).
		 */
		if (uc_working_band == WB_FCC) {
			compute_partial_num_dem_sfo_fcc_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_k));
		} else if (uc_working_band == WB_ARIB) {
			compute_partial_num_dem_sfo_arib_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_k));
		} else {
			compute_partial_num_dem_sfo_cen_a_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_k));
		}

		sl_numerator += mult_real_q(sl_partial_num, sl_Ni_by_avg, FRAC_PART_Q31);
		sl_denominator += mult_real_q(mult_real_q(sl_partial_den, sl_Ni_by_avg,
				FRAC_PART_Q31), (asl_distance[uc_k] - sl_average_distance), FRAC_PART_Q31);
	}

	if (sl_denominator == 0) {
		sl_denominator = 1;
	}

	div_real_q(sl_numerator, sl_denominator, FRAC_PART_Q27, &sll_delta_t_scaled); /* Result is Q5.27 scaled by x2^(SCALING_DELAY_VALUES +
	                                                                               * SCALING_FREQ_VALUES) */
	/* Checks that no overflow occurred in the division*/
	if (sll_delta_t_scaled > MAX_INT32) { /* Estimated SFO is larger than 2^4/(2*pi*2^(SCALING_DELAY_VALUES + SCALING_FREQ_VALUES)~=77.71 ppm */
		sl_delta_t = MAX_INT32;
	} else if (sll_delta_t_scaled < MIN_INT32) {
		sl_delta_t = MIN_INT32;
	} else {
		sl_delta_t = (int32_t)sll_delta_t_scaled;
	}

	sl_delta_t = mult_real_q(sl_delta_t, VALUE_1_2_PI_Q1_31, FRAC_PART_Q31); /* Result is Q5.27 */

	/* Scale sl_delta_t by (SCALING_DELAY_VALUES + SCALING_FREQ_VALUES - 4 ) to compensate previous scaling and convert to Q1.31 */
	sl_delta_t = sl_delta_t >> (SCALING_DELAY_VALUES + SCALING_FREQ_VALUES - 4);  /* Result is Q1.31 */
	/*printf("%ld \r\n",sl_delta_t);*/

	#ifdef DEBUG_CHN_SFO
	asl_delta_t_vector[uc_num_block_pilots] = sl_delta_t;
	#endif

	/* Compute increment and add to nominal value */
	sl_step = mult_real_q(sl_delta_t, RESAMPLE_STEP_NOM, FRAC_PART_Q31);
	sl_resamp_reg = sl_step + RESAMPLE_STEP;

	/* uc_half_est_delay_sfo_est incremented by 1 to be used as direct replacement for old (HALF_EST_DELAY_SFO_EST + 1) */
	if (uc_working_band == WB_CENELEC_A) {
		uc_half_est_delay_sfo_est = HALF_EST_DELAY_SFO_EST_CENELEC_A + 1;
	} else {
		uc_half_est_delay_sfo_est = HALF_EST_DELAY_SFO_EST_FCC_ARIB + 1;
	}

	/* Compute the displacement of the FFT window wrt to the useful part of the symbol due to the SFO */
	/* Result is Q5.27 */
	sl_sfo_time_offset = mult_real_q(sl_delta_t, (*(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + uc_num_s1s2 -       1)
			+ (SYMBOL_LENGTH_SAMPLES >> (SCALING_DELAY_VALUES - 1)) * (uc_half_est_delay_sfo_est)),
			(FRAC_PART_Q31 - LOG2_FFT_POINTS - SCALING_DELAY_VALUES + 4));

	/* Computes fix(sl_sfo_time_offset) */
	if (sl_delta_t > 0) {
		sc_sfo_time_offset = (int8_t)(sl_sfo_time_offset >> 27);
	} else {
		sc_sfo_time_offset = (int8_t)-((-sl_sfo_time_offset) >> 27);
	}

	if ((uc_working_band == WB_FCC) && (uc_sfo_objective == SFO_FOR_FCH_COH)) { /* If the FCH is very long, the channel estimate used to equalize FCH must be updated periodically */
		/* Compute the phase shift that will be periodically applied to the channel estimation used for equalizing the FCH */
		if (uc_num_symbols_fch > MIN_NUM_SYM_FCH_UPDATE_H_FCC_LEGACY) {
			/* The phase shift between the average H and the beginning of the FCH is neglected */
			/* Phase shift=(2*pi/N)*sl_delta_t*(N+cp-8). Q2.30 */
			sl_phase_shift = mult_real_q(mult_real_q(sl_delta_t, VALUE_2PI, FRAC_PART_Q27), SYMBOL_LENGTH_SAMPLES, FRAC_PART_Q30);
			arm_scale_q31(asl_freq_index, sl_phase_shift, (SCALING_FREQ_VALUES - 1),
					s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_phase_shift, uc_used_carriers);

			if (sl_delta_t > 0) {
				/* H will be updated after ss_num_sym_fch_for_H_update have been received since the last update */
				ss_num_sym_fch_for_H_update
					= (int16_t)(MAX_PHASE_SHIFT_FOR_H_UPDATE_FCH_FCC_LEGACY
						/ s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_phase_shift[uc_used_carriers - 1]);

				for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
					sl_fix_theta
						= ((s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_phase_shift[uc_i] *
							ss_num_sym_fch_for_H_update) & 0xE0000000);
					sl_aprox_theta1_imag
						= (s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_phase_shift[uc_i] *
							ss_num_sym_fch_for_H_update) - sl_fix_theta;
					sl_aprox_theta1_real = (VALUE_1_Q2_30 - (mult_real_q(sl_aprox_theta1_imag, sl_aprox_theta1_imag, FRAC_PART_Q30) >> 1));

					uc_index_vector_n = (uint8_t)(sl_fix_theta >> 29);

					s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[2 * uc_i]
						= mult_real_q(sl_aprox_theta1_real, asl_cos_n[uc_index_vector_n], FRAC_PART_Q30)
							- mult_real_q(sl_aprox_theta1_imag, asl_sin_n[uc_index_vector_n], FRAC_PART_Q30);
					s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[2 * uc_i + 1]
						= -(mult_real_q(sl_aprox_theta1_real, asl_sin_n[uc_index_vector_n], FRAC_PART_Q30)
							+ mult_real_q(sl_aprox_theta1_imag, asl_cos_n[uc_index_vector_n], FRAC_PART_Q30));
				}
			} else {
				ss_num_sym_fch_for_H_update
					= (int16_t)(MAX_PHASE_SHIFT_FOR_H_UPDATE_FCH_FCC_LEGACY
						/ (-s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_phase_shift[uc_used_carriers - 1]));

				for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
					sl_fix_theta
						= ((s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_phase_shift[uc_i] *
							ss_num_sym_fch_for_H_update) & 0xE0000000);
					sl_aprox_theta1_imag
						= (s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_phase_shift[uc_i] *
							ss_num_sym_fch_for_H_update) - sl_fix_theta;
					sl_aprox_theta1_real = (VALUE_1_Q2_30 - (mult_real_q(sl_aprox_theta1_imag, sl_aprox_theta1_imag, FRAC_PART_Q30) >> 1));

					uc_index_vector_n = (uint8_t)(sl_fix_theta >> 29);

					s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[2 * uc_i]
						= mult_real_q(sl_aprox_theta1_real, asl_cos_n[uc_index_vector_n], FRAC_PART_Q30)
							- mult_real_q(sl_aprox_theta1_imag, asl_sin_n[uc_index_vector_n], FRAC_PART_Q30);
					s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[2 * uc_i + 1]
						= mult_real_q(sl_aprox_theta1_real, asl_sin_n[uc_index_vector_n], FRAC_PART_Q30)
							+ mult_real_q(sl_aprox_theta1_imag, asl_cos_n[uc_index_vector_n], FRAC_PART_Q30);
				}
			}

			arm_q31_to_q15(s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo,
					s_shared_rotation_vectors.s_rotation_vectors_coh.ass_rotation_sfo_first, 2 * uc_used_carriers);
		} else {
			ss_num_sym_fch_for_H_update =   255; /* If the FCH is shorter than the minimum required, channel will not be updated during FCH */
		}

		ss_num_fch_since_last_H_update = ss_num_sym_fch_for_H_update;
	} else {
		/* The effect of the displacement caused by the SFO is compensate differently in coherent/differential payloads */
		if (uc_mod_scheme == MOD_SCHEME_COHERENT) {
			/* Assembly function to compute ass_rotation_sfo_first and ass_rotation_sfo_second the rotation vectors */
			/* to compensate the channel estimate to be applied to the first group of payload symbols */
			/* (before the time offset due to the SFO has been corrected by changing the cp of one symbol) and to the remaining ones (after cp change) */
			/* theta=(2*pi/N)*sl_delta_t*((N+cp-ps_length)*(HALF_EST_DELAY_SFO_EST + 1 + distance_last_employed_symbol)-sl_average_distance)) */
			/* ass_rotation_sfo_first = exp(j*theta), ass_rotation_sfo_second = exp(j*(theta-floor(theta)) */
			/* exp(j*x) is computed by means of a Taylor series centered in 0, 0.5, 1, 1.5, ... */
			/* Result are in Q2.14 because if the angle is very small, 1-x^2/2=1, which cannot be represented in Q1.15 */
			compute_ass_rotation_sfo_asm(ass_Wi, asl_distance, (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_num_s1s2), sl_delta_t);
		} else {
			/* Compute phase shift to be applied to the first symbol after the cp change */
			/* MAXIMUM CP CHANGE WITH CURRENT Q IS 6 */
			sl_phase_shift = mult_real_q(sc_sfo_time_offset, VALUE_2PI_DIV_N, 4);         /* Q5.27 */
			arm_scale_q31(asl_freq_index, sl_phase_shift, SCALING_FREQ_VALUES, s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo,
					uc_used_carriers);         /* Q5.27 */
			/* Phase unwrap */
			uc_phase_turns = 0;
			if (sl_sfo_time_offset > 0) {
				for (uc_k = 0; uc_k < uc_used_carriers; uc_k++) {
					s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k]
						= s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k] - (VALUE_2PI * uc_phase_turns);

					while (s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k] > VALUE_PI) {
						uc_phase_turns++;
						s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k]
							= s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k] - VALUE_2PI;
					}
				}
			} else {
				for (uc_k = 0; uc_k < uc_used_carriers; uc_k++) {
					s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k]
						= s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k] + (VALUE_2PI * uc_phase_turns);

					while (s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k] < -VALUE_PI) {
						uc_phase_turns++;
						s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k]
							= s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo[uc_k] + VALUE_2PI;
					}
				}
			}

			/* Convert result to Q3.13 */
			arm_shift_q31(s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo,
					2, s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo, uc_used_carriers);
			arm_q31_to_q15(s_shared_rotation_vectors.s_phase_shift_and_rotation.asl_rotation_sfo, pss_rotation_sfo_first, uc_used_carriers);
			swap_bytes_asm(pss_rotation_sfo_first, (uc_used_carriers + 1) / 2);

			/* Load phase shift */
			pplc_if_write8(REG_ATPL250_INOUTB_CONF2_H8, 0x04);         /* Configure ZONE2_REL=1 */
			atpl250_set_iob_real_mode();         /* Write only real part= values of Ph */
			/* Index where the symbol used as reference for demodulation of the desired one will be stored */
			uc_index_inob_rel = (s_band_constants.uc_pay_sym_first_demod + PAY_SYM_SECOND_DEMOD) % CFG_SYMBOLS_IN_IOB;
			pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * uc_index_inob_rel + s_band_constants.uc_last_used_carrier));
			pplc_if_write_jump((BCODE_ZONE2 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier + CFG_IOB_SAMPLES_PER_SYMBOL * uc_index_inob_rel)),
					(uint8_t *)pss_rotation_sfo_first, 2 * uc_used_carriers, JUMP_COL_2);
			atpl250_clear_iob_real_mode();

			/* Check if the place where the phase correction has been stored was occupied by reference values for coherent demod */
			uc_index_inob_abs = (uc_num_sym_p_detected + uc_num_symbols_fch + s_band_constants.uc_pay_sym_first_demod + PAY_SYM_SECOND_DEMOD - 1) % CFG_SYMBOLS_IN_IOB;
		}

		/* Configure resampling register */
		pplc_if_write32(REG_ATPL250_RESAMP24BITS_1_32, sl_resamp_reg);
		pplc_if_write32(REG_ATPL250_RESAMP24BITS_2_32, RESAMPLING_24BITS_2_VALUE);
	}
}

#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)

/*Estimates the sampling timing error from a block of pilots
 * Input: Uses the following static/extern variables
 *      -uc_first_symbol_index: index of the first symbol (starting at 0) of the current set of NUM_SYM_PILOTS_H_EST pilots
 *
 */
void sampling_error_est_from_pilots(uint8_t uc_last_symbol_index)
{
	uint8_t uc_i;
	q63_t sll_test;

	/* Reset numerator and denominator for SFO estimation in this block of pilots */
	sl_numerator_pil = 0;
	sl_denominator_pil = 0;

	if (uc_last_symbol_index < (s_band_constants.uc_pay_sym_first_demod + PAY_SYM_SECOND_DEMOD + NUM_SYM_PILOTS_H_EST * 2)) {
		/* If this is the first of set of pilots received from the payload-> we still need another one. */
		/* Initialize variables. This condition must be coherent with the one set in atpl250_mod_demod.c */
		uc_index_current_pilot_set = 1;
		uc_index_previous_pilot_set = 0;
		memset(auc_pilot_used_twice_cum, 0, s_band_constants.uc_num_carriers * sizeof(uint8_t));
		arm_fill_q31(0, asl_Nk, s_band_constants.uc_num_carriers);
		memset(aus_distance, 0, s_band_constants.uc_num_carriers * sizeof(uint16_t));
		zero_complex_vector_q_asm((uint8_t *)asl_cum_complex_error, s_band_constants.uc_num_carriers * 2 * sizeof(uint32_t));
	} else {
		/* If we have already received two sets of pilots with NUM_SYM_PILOTS_H_EST symbols in each one, we must obtain an estimate of the SFO */
		for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
			#ifdef DEBUG_CHN_SFO
			aus_pilot_stored_vector[1][(uc_num_block_pilots - 1) * uc_used_carriers + uc_i] = aus_pilot_stored[uc_index_current_pilot_set][uc_i];
			aus_pilot_stored_vector[0][(uc_num_block_pilots - 1) * uc_used_carriers + uc_i] = aus_pilot_stored[uc_index_previous_pilot_set][uc_i];
			#endif

			if ((aus_pilot_stored[0][uc_i]) && (aus_pilot_stored[1][uc_i])) {
				/* Computes the cumulative phase error,
				 * asl_cum_complex_error and the numerator (modulus of ass_H, sl_modulus) and denominator (asl_Nk) of the SNR per carrier,
				 * asl_SNRk.
				 * asl_cum_complex_error += asl_Ypilot[uc_index_current_pilot_set].*conj(asl_Ypilot[uc_index_previous_pilot_set]).
				 * Result is Q8.24 scaled by 1/2^7.
				 * The scaling is needed to avoid overflow in the sum of the numerator and denominator.
				 * It assumes that we will sum a maximum of 128 values.
				 * asl_Nk[uc_i] += abs(Ypilot[uc_index_current_pilot_set]-H).^2 + abs(Ypilot[uc_index_previous_pilot_set]-H).^2.
				 * Result is Q8.24 scaled by 1/2^SCALING_NK.
				 * sl_modulus = abs(H).^2. Result is Q8.24. */
				psl_Nk = (asl_Nk + uc_i);
				cum_phase_error_and_carrier_SNR_num_dem_asm((ass_Ypilots[uc_index_current_pilot_set] + 2 * uc_i),
						(ass_Ypilots[uc_index_previous_pilot_set] + 2 * uc_i), (asl_cum_complex_error + 2 * uc_i), (ass_H + 2 * uc_i));

				#ifdef DEBUG_CHN_SFO
				asl_modulus_vector[uc_i + uc_used_carriers * (uc_num_block_pilots - 1)] = sl_modulus;
				asl_cum_complex_error_vector[2 * uc_i + 2 * uc_used_carriers * (uc_num_block_pilots - 1)] = asl_cum_complex_error[2 * uc_i];
				asl_cum_complex_error_vector[2 * uc_i + 1 + 2 * uc_used_carriers *
				(uc_num_block_pilots - 1)] = asl_cum_complex_error[2 * uc_i + 1];
				#endif

				/* asl_SNRk contains an estimate of the SNR divided by auc_pilot_used_twice_cum[uc_i]. Result is in Q16.16 scaled by
				 * 1/2^SCALING_SNR */
				div_real_q(sl_modulus, asl_Nk[uc_i], (FRAC_PART_Q24 - (SCALING_NK + Q8_24_TO_Q16_16 + SCALING_SNR)), &asl_SNRk[uc_i]);
				if (asl_SNRk[uc_i] < 0) {
					/* There has been overflow in the division. The divisor is too small->fix to the max SNR */
					asl_SNRk[uc_i] = MAX_INT32;
				}

				/* Number of times each pilot has been used twice */
				auc_pilot_used_twice_cum[uc_i]++;

				/* Distance between pilots received twice */
				aus_distance[uc_i]
					+= (aus_pilot_stored[uc_index_current_pilot_set][uc_i] - aus_pilot_stored[uc_index_previous_pilot_set][uc_i]);

				/* Assembly computation of sl_numerator_pil and sl_denominator_pil:
				 * sl_distance_div_pst_sum = aus_distance/auc_pilot_used_twice_cum. Result is in Q8.24.
				 * sl_SNRk_by_freq = mult_real_q(asl_SNRk[uc_i], asl_freq_index[uc_i], FRAC_PART_Q31).
				 * Result is Q16.16 scaled by 1/2^(SCALING_FREQ_VALUES+SCALING_SNR).
				 * sl_SNRk_by_freq_squared = mult_real_q(sl_SNRk_by_freq, asl_freq_index[uc_i], FRAC_PART_Q31);
				 * Result in Q16.16 scaled by 1/2^(2*SCALING_FREQ_VALUES+SCALING_SNR).
				 * sl_numerator_pil += Im[asl_cum_complex_error].*sl_SNRk_by_freq.*sl_distance_div_pst_sum.
				 * Result is Q8.24 scaled by 1/2^(7+SCALING_FREQ_VALUES+SCALING_DELAY_VALUES)
				 * sl_denominator += Re[asl_cum_complex_error].*sl_SNRk_by_freq_squared.*sl_distance_div_pst_sum.*sl_distance_div_pst_sum.
				 * Result is Q8.24 scaled by 1/2^(7+2*SCALING_FREQ_VALUES+SCALING_DELAY_VALUES)
				 */
				uc_pilot_used_twice_cum = auc_pilot_used_twice_cum[uc_i];
				compute_partial_num_dem_sfo_pil_asm((asl_cum_complex_error + 2 * uc_i), (asl_SNRk + uc_i), (asl_freq_index + uc_i),
						(aus_distance + uc_i));
			} else if ((aus_pilot_stored[uc_index_previous_pilot_set][uc_i]) && (!aus_pilot_stored[uc_index_current_pilot_set][uc_i])) {
				/* Moves unused pilots of the previous set to the current set */
				aus_pilot_stored[uc_index_current_pilot_set][uc_i] = aus_pilot_stored[uc_index_previous_pilot_set][uc_i];
				ass_Ypilots[uc_index_current_pilot_set][2 * uc_i] = ass_Ypilots[uc_index_previous_pilot_set][2 * uc_i];
				ass_Ypilots[uc_index_current_pilot_set][2 * uc_i + 1] = ass_Ypilots[uc_index_previous_pilot_set][2 * uc_i + 1];
			}

			/* Clear information regarding pilots received in previous set */
			aus_pilot_stored[uc_index_previous_pilot_set][uc_i] = 0;

			#ifdef DEBUG_CHN_SFO
			aus_distance_vector[(uc_num_block_pilots - 1) * uc_used_carriers + uc_i] = aus_distance[uc_i];
			auc_pilot_used_twice_cum_vector[(uc_num_block_pilots - 1) * uc_used_carriers + uc_i] = auc_pilot_used_twice_cum[uc_i];
			#endif
		}

		/* Updates indexes for the next iteration */
		if (uc_index_current_pilot_set) {
			uc_index_current_pilot_set = 0;
			uc_index_previous_pilot_set = 1;
		} else {
			uc_index_current_pilot_set = 1;
			uc_index_previous_pilot_set = 0;
		}

		/* Computes estimation of delta_t_est in Q8.24 */
		if (sl_denominator_pil == 0) {
			sl_denominator_pil = 1;
		}

		div_real_q(sl_numerator_pil, sl_denominator_pil, FRAC_PART_Q24, &sl_delta_t_est);

		/* Convert from Q8.24 to Q1.31 is compensated by the scaling of asl_freq_index. Hence, only product by 1/2*pi and by N/(N+cp-ps_length) is done
		**/
		/* Result is Q1.31 */
		sl_delta_t_est = mult_real_q(mult_real_q(sl_delta_t_est, VALUE_1_2_PI_Q1_31, FRAC_PART_Q31), VALUE_N_DIV_SYM_LENGTH_Q1_31, FRAC_PART_Q31);
		/*printf("%ld \r\n",sl_delta_t_est);*/

		#ifdef DEBUG_CHN_SFO
		asl_delta_t_est_vector[(uc_num_block_pilots - 1)] = sl_delta_t_est;
		arm_copy_q31(asl_SNRk, (asl_SNRk_vector + (uc_num_block_pilots - 1) * uc_used_carriers), uc_used_carriers);
		arm_copy_q31(asl_Nk, (asl_Nk_vector + (uc_num_block_pilots - 1) * uc_used_carriers), uc_used_carriers);
		asl_numerator_pil_vector [(uc_num_block_pilots - 1)] = sl_numerator_pil;
		asl_denominator_pil_vector [(uc_num_block_pilots - 1)] = sl_denominator_pil;
		#endif

		/* Checks if the estimated value is greater than a given percentage of the previous estimate. */
		sll_test = mult_real_q(sl_delta_t, PERCENTAGE_LIMIT_SFO_EST_PIL, FRAC_PART_Q31);
		if ((sl_delta_t_est > sll_test) || (sl_delta_t_est < -sll_test)) {
			sl_delta_t_est = sll_test;
		}

		/* Compute increment and add (scaled) to nominal value */

		/*sl_delta_t += mult_real_q(sl_delta_t_est, FORGET_FACTOR_SFO, FRAC_PART_Q31);
		 * sl_resamp_reg = RESAMPLE_STEP + mult_real_q(sl_delta_t, RESAMPLE_STEP_NOM, FRAC_PART_Q31);*/
		sl_resamp_reg = sl_resamp_reg + mult_real_q(mult_real_q(sl_delta_t_est, FORGET_FACTOR_SFO, FRAC_PART_Q31), RESAMPLE_STEP_NOM, FRAC_PART_Q31);

		/* Configure skip_dup */
		pplc_if_write32(REG_ATPL250_RESAMP24BITS_1_32, sl_resamp_reg);
		pplc_if_write32(REG_ATPL250_RESAMP24BITS_2_32, RESAMPLING_24BITS_2_VALUE);
	}
}

#endif
