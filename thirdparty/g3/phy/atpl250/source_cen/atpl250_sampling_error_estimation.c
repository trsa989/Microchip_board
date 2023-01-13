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
#include <string.h>

/* Phy layer includes */
#include "atpl250.h"
#include "atpl250_channel_estimation.h"
#include "atpl250_sampling_error_estimation.h"
#include "atpl250_channel_and_sfo_estimation_params.h"
#include "atpl250_mod_demod.h"

/* Extern PHY Variables */
extern uint8_t uc_notched_carriers;
extern uint8_t uc_used_carriers;
extern int32_t asl_freq_index[], asl_freq_index_squared[];
extern int32_t asl_delay_symbols[];

extern q31_t asl_inv_Ni[];
extern q15_t ass_Wi[];

/* Actual number of SYNCP and FCH symbols used for channel estimation. */
/* Maximum number of SYNCP is NUM_SYM_H_EST_PRE and maximum number of fch symbols is NUM_SYM_H_EST_FCH */
extern uint8_t uc_num_sym_valid_preamble;
extern uint8_t uc_num_sym_valid_fch;
extern uint8_t auc_index_sym_valid_fch[];

/* Array for storing the p,m fch and s1s2 symbols used for channel estimation */
extern q15_t ass_pm_fch_s1s2_symbols[];
extern q15_t ass_Ypilots[];
extern q31_t asl_average_symbol[];
extern q15_t ass_H[];
extern q31_t asl_symbol_aux[];

/*Distance from each symbol to the average symbol*/
static q31_t asl_distance[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH + 2];

/*Weighting factors used for the estimation of the channel response but stored in Q1.31*/
static q31_t asl_Wi[NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH + 2];

/*Phase that compensates the SFO and complex vector that compensates this phase in the estimated H*/
static q31_t asl_phase[PROTOCOL_CARRIERS];

/* Estimate of the sampling error period */
static q31_t sl_delta_t;
#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)
static q31_t sl_delta_t_est;
#endif

/*  Value of the resampling register*/
static int32_t sl_resamp_reg;
static int32_t sl_step; /* Step with respect to the actual value of sl_resamp_reg */

/* Complex vector that compensates the SFO rotation of the channel estimate obtained from the preamble/FCH/S1S2 */
q15_t ass_rotation_sfo[PROTOCOL_CARRIERS_BY_2];

/* The following variables are used in atpl250_mod_demod.c even if channel estimation based on pilots is not accomplished*/
/* Stores the symbol index at which the pilots stored in ass_Ypilots appeared */
uint16_t aus_pilot_stored[2][PROTOCOL_CARRIERS] = {{0}};
/* Index (either "0" or "1") of the pilots block */
uint8_t uc_index_set = 0;

#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)

/* Stores the values of the received pilots in successive blocks */
static q31_t asl_Ypilots [2][PROTOCOL_CARRIERS_BY_2] = {{0}};

/* Index of ass_Ypilot row that contains the current and previous sets of pilots that have been received */
static uint8_t uc_index_current_pilot_set, uc_index_previous_pilot_set;

/* Distance between the received pilots */
static uint16_t aus_distance[PROTOCOL_CARRIERS] = {0};

/* Number of times that a given pilot has been received twice in the current payload */
static uint8_t auc_pilot_used_twice_cum[PROTOCOL_CARRIERS] = {0};

/* Cumulative complex vector with phase error */
static q31_t asl_cum_complex_error[PROTOCOL_CARRIERS_BY_2] = {0};

/* per-carrier Noise and SNR */
static q31_t asl_Nk[PROTOCOL_CARRIERS] = {0};
static q31_t asl_SNRk[PROTOCOL_CARRIERS] = {0};

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
 *      -ass_pm_fch_s1s2_symbols
 *      -asl_average_symbol
 *      -ass_inv_Ni
 */
void sampling_error_est_from_preamble_fch_s1s2(uint8_t uc_mod_scheme, uint16_t us_rx_payload_symbols)
{
	uint8_t uc_i, uc_k;
	q31_t sl_numerator = 0, sl_denominator = 0;
	q31_t sl_aux;
	q31_t sl_Ni_by_avg;
	uint8_t uc_num_s1s2 = 2;
	q31_t sl_average_distance;
	q63_t sll_average_distance;
	q63_t sll_delta_t_scaled;
	q63_t sll_partial_num, sll_partial_den;
	q31_t sl_partial_num, sl_partial_den;

	UNUSED(us_rx_payload_symbols);

	/* If payload is differentially modulated, there are no S1S2 */
	if (uc_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
		uc_num_s1s2 = 0;
	}

	/* Obtain distances for the actual case */
	for (uc_i = 0; uc_i < uc_num_sym_valid_preamble; uc_i++) {
		asl_distance[uc_i] = *(asl_delay_symbols + (NUM_FULL_SYMBOLS_PREAMBLE - uc_num_sym_valid_preamble) + uc_i);
	}
	for (uc_i = 0; uc_i < uc_num_sym_valid_fch; uc_i++) {
		asl_distance[uc_num_sym_valid_preamble + uc_i] = *(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE +  auc_index_sym_valid_fch[uc_i]);
	}
	for (uc_i = 0; uc_i < uc_num_s1s2; uc_i++) {
		asl_distance[ uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_i] = *(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + uc_i);
	}

	/* To equalize the received symbols */
	arm_cmplx_conj_q31(asl_average_symbol, asl_average_symbol, uc_used_carriers);

	/* Computes the average distance */
	/* Average distance in Q1.31 scaled by 1/2^SCALING_DELAY_VALUES */
	arm_mean_q31(asl_distance, (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_num_s1s2), &sl_average_distance);

	/* Computes the numerator and denominator using the vector dot product function of the arm library */
	for (uc_k = 0; uc_k < (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_num_s1s2); uc_k++) { /* time dimension sum */
		/*Convert to Q3.29*/
		arm_q15_to_q31((ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers * uc_k), asl_symbol_aux, 2 * uc_used_carriers);

		/* Result is in Q3.13 */
		arm_cmplx_mult_cmplx_q31(asl_symbol_aux, asl_average_symbol, asl_symbol_aux, uc_used_carriers);

		/* Separate real and imaginary parts to use vector dot product */
		for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
			u_shared_buffers.s_sfo_est.asl_Y_imag[uc_i] = asl_symbol_aux[2 * uc_i + 1];
			u_shared_buffers.s_sfo_est.asl_Y_real[uc_i] = asl_symbol_aux[2 * uc_i];
		}

		/* Computes product of 1/Ni by the distance value for the current index */
		/* Q1.31 scaled by 1/2^SCALING_DELAY_VALUES */
		sl_Ni_by_avg = mult_real_q(*(asl_inv_Ni + uc_k), (asl_distance[uc_k] - sl_average_distance), FRAC_PART_Q31);

		/* Result in Q18.46 because the input symbols were in Q3.29 */
		arm_dot_prod_q31(u_shared_buffers.s_sfo_est.asl_Y_imag, asl_freq_index, uc_used_carriers, &sll_partial_num);
		/* Result in Q18.46 because the input symbols were in Q3.29 */
		arm_dot_prod_q31(u_shared_buffers.s_sfo_est.asl_Y_real, asl_freq_index_squared, uc_used_carriers, &sll_partial_den);

		sl_partial_num = (q31_t)(sll_partial_num >> (Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION));
		sl_partial_den = (q31_t)(sll_partial_den >> (Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION));

		sl_numerator += mult_real_q(sl_partial_num, sl_Ni_by_avg, FRAC_PART_Q31);
		sl_denominator += mult_real_q(mult_real_q(sl_partial_den, sl_Ni_by_avg,
				FRAC_PART_Q31), (asl_distance[uc_k] - sl_average_distance), FRAC_PART_Q31);
	}

	if (sl_denominator == 0) {
		sl_denominator = 1;
	}

	div_real_q(sl_numerator, sl_denominator, FRAC_PART_Q27, &sll_delta_t_scaled);

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

	/* Compute increment and add to nominal value */
	sl_step = mult_real_q(sl_delta_t, RESAMPLE_STEP_NOM, FRAC_PART_Q31);
	sl_resamp_reg = sl_step + RESAMPLE_STEP;

	/* ** Obtains the rotation vector to be applied to the channel estimate made in the preamble */
	arm_q15_to_q31(ass_Wi, asl_Wi, (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_num_s1s2));
	/* Result is Q16.48 but scaled by 1/2^SCALING_DELAY_VALUES */
	arm_dot_prod_q31(asl_distance, asl_Wi, (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + uc_num_s1s2), &sll_average_distance);
	sl_average_distance = (q31_t)(sll_average_distance >> Q16_48_TO_Q1_31);  /* convert to Q1.31 */
	sl_aux = mult_real_q(sl_delta_t, VALUE_PI_DIV_2_Q_2_30, FRAC_PART_Q30); /* Result in Q1.31 */
	/* The first estimated channel in the payload has a phase shift wrt to the one estimated in the preamble caused by the SFO, */
	/* which is not compensated until EST_DELAY_SFO_EST symbols after the preamble/FCH end */
	sl_aux
		= mult_real_q(sl_aux,
			(*(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + uc_num_s1s2 -
			1) + (SYMBOL_LENGTH_SAMPLES >> (SCALING_DELAY_VALUES - 1)) * (EST_DELAY_SFO_EST + 1) - sl_average_distance), FRAC_PART_Q31);
	/* Result in Q1.31 scaled by 1/2=Q2.30. The +1 compensates the pi/2. */
	arm_scale_q31(asl_freq_index, sl_aux, SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1, asl_phase, uc_used_carriers);
	for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
		asl_symbol_aux[2 * uc_i + 1] = asl_phase[uc_i];
		asl_symbol_aux[2 * uc_i] = (VALUE_1_Q2_30 - (mult_real_q(asl_phase[uc_i], asl_phase[uc_i], FRAC_PART_Q30) >> 1));
	}
	/* The format is Q2.30 because if the angle is very small, 1-x^2/2=1, which cannot be represented in Q1.31 */
	arm_q31_to_q15(asl_symbol_aux, ass_rotation_sfo, 2 * uc_used_carriers);

	/* Configure resampling register */
	pplc_if_write32(REG_ATPL250_RESAMP24BITS_1_32, sl_resamp_reg);
	pplc_if_write32(REG_ATPL250_RESAMP24BITS_2_32, RESAMPLING_24BITS_2_VALUE);
}

#if defined (UPDATE_CHN_SFO_EST_PAYLOAD)

/*Estimates the sampling timing error from a block of pilots
 * Input: Uses the following static/extern variables
 *      -uc_first_symbol_index: index of the first symbol (starting at 0) of the current set of NUM_SYM_PILOTS_H_EST pilots
 *
 */
void sampling_error_est_from_pilots(uint8_t uc_last_symbol_index, uint8_t uc_num_pilots, uint8_t uc_first_pilot_pos, uint16_t us_rx_pending_symbols)
{
	uint8_t uc_i;
	q31_t sl_modulus, sl_real, sl_imag;
	q31_t sl_numerator = 0, sl_denominator = 0;
	q31_t sl_SNRk_by_freq, sl_SNRk_by_freq_squared, sl_distance_div_pst_sum;
	q63_t sll_test;

	UNUSED(uc_num_pilots);
	UNUSED(uc_first_pilot_pos);
	UNUSED(us_rx_pending_symbols);

	/* If this is the first of set of pilots received from the payload-> we still need another one. */
	/* Initialize variables. This condition must be coherent with the one set in atpl250_mod_demod.c and with the one fixed in the code below */
	if (uc_last_symbol_index < (NUM_SYM_PILOTS_H_EST * 3)) {
		uc_index_current_pilot_set = 1;
		uc_index_previous_pilot_set = 0;
		memset(auc_pilot_used_twice_cum, 0, uc_used_carriers * sizeof(uint8_t));
		memset(asl_Nk, 0, uc_used_carriers * sizeof(q31_t));
		memset(aus_distance, 0, uc_used_carriers * sizeof(uint16_t));
		memset(asl_cum_complex_error, 0, 2 * uc_used_carriers * sizeof(uint32_t));
	}

	/* Converts pilots from Q1.15 to Q1.31 and scales to Q8.24 (to avoid overflow in asl_cum_complex_error). */
	arm_q15_to_q31(ass_Ypilots, asl_Ypilots[uc_index_set], 2 * uc_used_carriers);
	arm_shift_q31(asl_Ypilots[uc_index_set], -7, asl_Ypilots[uc_index_set], 2 * uc_used_carriers);

	/* Updates index_set for the next iteration */
	uc_index_set = (uc_index_set + 1) % 2;

	/* If we have already received two sets of pilots with NUM_SYM_PILOTS_H_EST symbols in each one, we must obtain an estimate of the SFO */

	/* the 3 is to ensure that all employed symbols have been received with the skip/dup value updated.
	 * The first symbols of the payload were received when the SFO was being estimated. Hence, the skip/dup was not updated. */
	if (uc_last_symbol_index >= (NUM_SYM_PILOTS_H_EST * 3)) {
		for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
			if ((aus_pilot_stored[0][uc_i]) && (aus_pilot_stored[1][uc_i])) {
				/* Distance between pilots received twice */
				aus_distance[uc_i]
					+= (aus_pilot_stored[uc_index_current_pilot_set][uc_i] - aus_pilot_stored[uc_index_previous_pilot_set][uc_i]);

				/* Cumulates the complex vector with the phase error. Result is in Q8.24 but scaled by 1/2^7. The scaling is needed to avoid
				 * overflow in the sum of the numerator */
				/* and denominator. It assumes that we will sum a maximum of 128 values. */
				asl_cum_complex_error[2 * uc_i]
					+= (mult_real_q(asl_Ypilots[uc_index_current_pilot_set][2 * uc_i],
						asl_Ypilots[uc_index_previous_pilot_set][2 * uc_i], FRAC_PART_Q31) +
						mult_real_q(asl_Ypilots[uc_index_current_pilot_set][2 * uc_i + 1],
						asl_Ypilots[uc_index_previous_pilot_set][2 * uc_i + 1], FRAC_PART_Q31));
				asl_cum_complex_error[2 * uc_i + 1]
					+= (mult_real_q(asl_Ypilots[uc_index_current_pilot_set][2 * uc_i + 1],
						asl_Ypilots[uc_index_previous_pilot_set][2 * uc_i], FRAC_PART_Q31) -
						mult_real_q(asl_Ypilots[uc_index_current_pilot_set][2 * uc_i],
						asl_Ypilots[uc_index_previous_pilot_set][2 * uc_i + 1], FRAC_PART_Q31));

				/* Number of times each pilot has been used twice */
				auc_pilot_used_twice_cum[uc_i]++;

				/* per carrier-SNR estimation */
				sl_real = ((q31_t)ass_H[2 * uc_i]) << Q1_15_TO_Q8_24;  /* Converts from Q1.15 to Q8.24 */
				sl_imag = ((q31_t)ass_H[2 * uc_i + 1]) << Q1_15_TO_Q8_24;  /* Converts from Q1.15 to Q8.24 */
				sl_modulus = modulus_complex_q(sl_real, sl_imag, FRAC_PART_Q24);
				/* Result in Q8.24 scaled by 1/2 */
				asl_Nk[uc_i]
					+= ((modulus_complex_q((asl_Ypilots[0][2 * uc_i] - sl_real), (asl_Ypilots[0][2 * uc_i + 1] - sl_imag),
						FRAC_PART_Q24) +
						modulus_complex_q((asl_Ypilots[1][2 * uc_i] - sl_real), (asl_Ypilots[1][2 * uc_i + 1] - sl_imag),
						FRAC_PART_Q24)) >> SCALING_NK);

				/* asl_SNRk contains an estimate of the SNR divided by auc_pilot_used_twice_cum[uc_i]. Result is in Q16.16 scaled by 1/2^8 */
				div_real_q((sl_modulus >> (SCALING_NK + Q8_24_TO_Q16_16 + SCALING_SNR)), asl_Nk[uc_i], FRAC_PART_Q24, &asl_SNRk[uc_i]);
				if (asl_SNRk[uc_i] < 0) {
					/* There has been overflow in the division. The divisor is too small->fix to the max SNR */
					asl_SNRk[uc_i] = MAX_INT32;
				}

				/* Numerator and denominator of delta_t_est */
				sl_SNRk_by_freq = mult_real_q(asl_SNRk[uc_i], asl_freq_index[uc_i], FRAC_PART_Q31); /* Result in Q16.16 */
				sl_SNRk_by_freq_squared = mult_real_q(sl_SNRk_by_freq, asl_freq_index[uc_i], FRAC_PART_Q31); /* Result in Q16.16 */
				div_real_q(aus_distance[uc_i], auc_pilot_used_twice_cum[uc_i], FRAC_PART_Q24, &sl_distance_div_pst_sum);

				/* Result is Q8.24 scaled by 1/2^SCALING_FREQ_VALUES */
				sl_numerator += mult_real_q(mult_real_q(asl_cum_complex_error[2 * uc_i + 1], sl_SNRk_by_freq,
						FRAC_PART_Q16), sl_distance_div_pst_sum, FRAC_PART_Q24);
				/* Result is Q8.24 scaled by 1/2^SCALING_FREQ_VALUES */
				sl_denominator
					+= mult_real_q(mult_real_q(mult_real_q(asl_cum_complex_error[2 * uc_i], sl_SNRk_by_freq_squared,
						FRAC_PART_Q16), sl_distance_div_pst_sum, FRAC_PART_Q24), sl_distance_div_pst_sum, FRAC_PART_Q24);
			} else if ((aus_pilot_stored[0][uc_i]) && (!aus_pilot_stored[1][uc_i])) {
				/* Moves unused pilots of the previous set to the current set */
				aus_pilot_stored[uc_index_current_pilot_set][uc_i] = aus_pilot_stored[uc_index_previous_pilot_set][uc_i];
				asl_Ypilots[uc_index_current_pilot_set][2 * uc_i] = asl_Ypilots[uc_index_previous_pilot_set][2 * uc_i];
				asl_Ypilots[uc_index_current_pilot_set][2 * uc_i + 1] = asl_Ypilots[uc_index_previous_pilot_set][2 * uc_i + 1];
			}

			aus_pilot_stored[uc_index_previous_pilot_set][uc_i] = 0;
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
		if (sl_denominator == 0) {
			sl_denominator = 1;
		}

		div_real_q(sl_numerator, sl_denominator, FRAC_PART_Q24, &sl_delta_t_est);

		/* Convert from Q8.24 to Q1.31 is compensated by the scaling of asl_freq_index. Hence, only product by 1/2*pi and by N/(N+cp-ps_length) is done
		**/
		/* Result is Q1.31 */
		sl_delta_t_est = mult_real_q(mult_real_q(sl_delta_t_est, VALUE_1_2_PI_Q1_31, FRAC_PART_Q31), VALUE_N_DIV_SYM_LENGTH_Q1_31, FRAC_PART_Q31);

		/* Checks if the estimated value is greater than a given percentage of the previous estimate */
		div_real_q(sl_delta_t_est, sl_delta_t, FRAC_PART_Q31, &sll_test);
		if ((sll_test > LIMIT_SFO_EST_PIL) || (sll_test < -LIMIT_SFO_EST_PIL)) {
			sl_delta_t_est = mult_real_q(sl_delta_t, LIMIT_SFO_EST_PIL, FRAC_PART_Q31);
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
