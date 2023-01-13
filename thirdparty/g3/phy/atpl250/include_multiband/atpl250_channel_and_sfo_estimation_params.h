/** * \file
 *
 * \brief HEADER. ATPL250 Channel and Sampling Frequency Offset Estimation Parameters
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

#ifndef ATPL250_CHANNEL_AND_SFO_ESTIMATION_PARAMS_H_INCLUDED
#define ATPL250_CHANNEL_AND_SFO_ESTIMATION_PARAMS_H_INCLUDED

/* ****************************** Control parameters ********************************* */

/* ToDo: Smoothing it is now implemented as a configuration option, but should be done as a PHY procedure when the SNR is very low. */
/*#define SMOOTHING*/

/* If defined, channel and SFO estimation will be updated in the payload (when coherently modulated)*/
/* By now, it is never done in FCC and ARIB */
#define UPDATE_CHN_SFO_EST_PAYLOAD

/* For testing: configures receiver to introduce sampling frequency error */
/*#define INTRODUCE_SAMPLING_ERROR*/

/* To help debugging. Stores values of symbols used for channel and SFO estimation at each payload block */
/*#define DEBUG_CHN_SFO*/

/* ****************************** Configuration parameters ********************************* */

/* Nominal resample limit and step*/
#define RESAMPLE_STEP_NOM   5592405L
#define RESAMPLE_LIMIT      16777215L

#if defined(INTRODUCE_SAMPLING_ERROR)
/* For testing: resampling values to introduce delta_t=+80ppm, defined as (T_Rx-T_Tx)/T_Tx */
/*#define RESAMPLE_STEP	5591957L*/
/* For testing: resampling values to introduce delta_t=+50ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592125L */
/* For testing: resampling values to introduce delta_t=40ppm, defined as (T_Rx-T_Tx)/T_Tx */
/*#define RESAMPLE_STEP	5592181L */
/* For testing: resampling values to introduce delta_t=30ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592237L */
/* For testing: resampling values to introduce delta_t=20ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592293L */
/* For testing: resampling values to introduce delta_t=10ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592349L */
/* For testing: resampling values to introduce delta_t=5ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP 5592377L */
/* For testing: resampling values to introduce delta_t=0ppm, defined as (T_Rx-T_Tx)/T_Tx */
/*#define RESAMPLE_STEP   5592405L*/
/* For testing: resampling values to introduce delta_t=-5ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592433L */
/* For testing: resampling values to introduce delta_t=-10ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592461L */
/* For testing: resampling values to introduce delta_t=-20ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592517L */
/* For testing: resampling values to introduce delta_t=-30ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592573L */
/* For testing: resampling values to introduce delta_t=-40ppm, defined as (T_Rx-T_Tx)/T_Tx */
#define RESAMPLE_STEP   5592629L
/* For testing: resampling values to introduce delta_t=-50ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592684L */
/* For testing: resampling values to introduce delta_t=-80ppm, defined as (T_Rx-T_Tx)/T_Tx */
/* #define RESAMPLE_STEP	5592852L */
#else
/* Nominal resample step */
#define RESAMPLE_STEP   RESAMPLE_STEP_NOM
#endif

/*Number of preamble symbols (including SYNCP and SYNCM and excluding S1 and S2) used for channel estimation. MINIMUM VALUE IS 2: one SYNCP and the SYNCM
 * IMPORTANT: it is assumed that the number of preamble symbols in the INOB is greater or equal than this value.
 * IMPORTANT: If fixed to a value greater than 6 overflow
 * might occur in computing the average symbol, unless current scaling parameters are changed */
#define NUM_SYM_H_EST_PRE       5

/* Number of FCH symbols used for channel and SFO estimation. MAXIMUM VALUE IS 8 */
#define NUM_SYM_H_EST_FCH_COH   0  /* number of FCH symbols for SFO estimation when payload is coherently modulated */

/* Number of FCH symbols for first interruption of the INOB after the PEAK2.
 * It is assumed that (NUM_FCH_SYM_AFTER_PEAK2 + NUM_SYM_H_EST_FCH)<=number of FCH symbols */
#define NUM_FCH_SYM_AFTER_PEAK2_CENELEC_A        5
#define NUM_FCH_SYM_AFTER_PEAK2_FCC_ARIB         7 /* Selected taking into account the time to estimate the channel from the preamble */

/* Number of symbols of the FCH used for channel estimation and for SFO estimation. Maximum value is
 * number of FCH symbols - NUM_FCH_SYM_AFTER_PEAK2. Minimum value is 2 (for SFO estimation with differential
 * modulation) */
#define MIN_NUM_SYM_H_EST_FCH_DIF_CENELEC_A      4       /* MINIMUM number of FCH symbols for SFO estimation when payload is differentially modulated.*/
#define MIN_NUM_SYM_H_EST_FCH_DIF_FCC_ARIB       2

/* Maximum of MIN_NUM_SYM_H_EST_FCH_DIF_CENELEC_A and NUM_SYM_H_EST_FCH_COH*/
#if (NUM_SYM_H_EST_FCH_COH >= MIN_NUM_SYM_H_EST_FCH_DIF_CENELEC_A)
	#define NUM_SYM_H_EST_FCH_CENELEC_A       NUM_SYM_H_EST_FCH_COH
#else
	#define NUM_SYM_H_EST_FCH_CENELEC_A       MIN_NUM_SYM_H_EST_FCH_DIF_CENELEC_A
#endif

/* Maximum of MIN_NUM_SYM_H_EST_FCH_DIF_FCC_ARIB and NUM_SYM_H_EST_FCH_COH*/
#if (NUM_SYM_H_EST_FCH_COH >= MIN_NUM_SYM_H_EST_FCH_DIF_FCC_ARIB)
	#define NUM_SYM_H_EST_FCH_FCC_ARIB       NUM_SYM_H_EST_FCH_COH
#else
	#define NUM_SYM_H_EST_FCH_FCC_ARIB       MIN_NUM_SYM_H_EST_FCH_DIF_FCC_ARIB
#endif

/* Number of bits of the fractional part in 1/H. IMPORTANT: If changed,TWO_POW_SCALE_X_Y must be changed */
#define NUM_BITS_FRAC_PART_INV_H 8

/* Number of payload symbols used to collect pilots for channel estimation */
#define NUM_SYM_PILOTS_H_EST    6

/* Forgetting factors of the channel estimation filter in the payload */
#define ALPHA_CHANNEL_EST_Q15   5461  /* (1/6) */
#define COMP_ALPHA_CHANNEL_EST_Q15      27307 /* (5/6) */

/* Gain scale to adjust the inverse of the channel estimate amplitude to the one expected by the FEQ.
 * This equals the amplitude of the cauc_abcd_fullgain[0] (point a)=0x5508=21768 */
#define GAIN_SCALE_INV_H_FEQ    21768

/*Identifies the part of the frame to which the symbol corresponds*/
#define PAYLOAD_PART    1
#define PREAMBLE_FCH_S1S2_PART  0

/* Identifies the channel estimate to be applied in the payload (before the cp change used to adjust the FFT window) */
#define CHANNEL_BEFORE_CP_CHANGE        0
#define CHANNEL_AFTER_CP_CHANGE         1
#define CHANNEL_IN_FCH                  2
#define CHANNEL_FOR_SHORT_PAYLOADS      3

/* Identifies the frame part where the SFO estimation will be used */
#define SFO_FOR_FCH_COH 0
#define SFO_FOR_PAYLOAD 1

/* ********************************* Parameters exclusively used in SFO estimation *************************************** */

/* Value to configure the RESAMPLING_24BITS_2 register */
#define RESAMPLING_24BITS_2_VALUE (RESAMPLE_LIMIT | (0x01 << 24))

/* Forgetting factor of the filter that combines previous and actual estimate of the SFO in Q1.31 */
/*#define FORGET_FACTOR_SFO       8388608L  Corresponds to 1/256 */
/*#define FORGET_FACTOR_SFO       16777216L  Corresponds to 1/128 */
#define FORGET_FACTOR_SFO       33554432L /* Corresponds to 1/64 */
/*#define FORGET_FACTOR_SFO       67108864L  Corresponds to 1/32 */

/* Half (ceil) of the estimated delay in computing the SFO (in symbols): HALF_EST_DELAY_SFO_EST=ceil((PAY_SYM_FIRST_DEMOD + 1)/2) */
#define HALF_EST_DELAY_SFO_EST_CENELEC_A       2
#define HALF_EST_DELAY_SFO_EST_FCC_ARIB        3

/* Limit to discard the value of delta_t estimated from a block of symbols in Q1.31 */
#define PERCENTAGE_LIMIT_SFO_EST_PIL   322122547LL      /* corresponds to 15% */

/* Minimum number of FCH symbols to update channel estimate during FCH in FCC legacy mode*/
#define MIN_NUM_SYM_FCH_UPDATE_H_FCC_LEGACY     20
/*#define MIN_NUM_SYM_FCH_UPDATE_H_FCC_LEGACY	12 */ /* FOR TESTING */

/* Maximum phase shift at the highest carrier */
/* Maximum phase shift during FCH (phase shift due to preamble symbols is neglected) allowed before updating H in FCH. Q2.30 */
#define MAX_PHASE_SHIFT_FOR_H_UPDATE_FCH_FCC_LEGACY      761934973 /* Corresponds to 20 symbols at delta_t=50ppm at k=104 */
/*#define MAX_PHASE_SHIFT_FOR_H_UPDATE_FCH_FCC_LEGACY   152386995 */ /* Corresponds to 5 symbols at delta_t=40ppm at k=104 */

/* Scaling to avoid overflow in the sum of the preamble+fch+s1s2.
 * It should be fixed to 4 (2^4<NUM_SYM_H_EST_PRE + NUM_SYM_H_EST_FCH + 2) but takes into account that we are computing min(Ni)*0.5/Ni
 * (max value of min(Ni)*0.5/Ni is 0.5, but the remaining values range from 1/4 to 1/3 in ideal conditions)
 * and that the average delay distance is overscaled by 2.
 * It also takes into account that the product of psl_input_symbols by the conj(asl_average_symbol) is
 * Q3.29 but it would suffice with Q2.30 */
#define SCALING_TIME_DIMENSION  1

/* Scaling to avoid overflow in the sum of the carriers. Its value depends on the number of carriers that are summed */
#define SCALING_FREQ_DIMENSION_CENELEC_A  4
#define SCALING_FREQ_DIMENSION_FCC        6
#define SCALING_FREQ_DIMENSION_ARIB       5

/* Scaling of the SNR */
#define SCALING_SNR     8

/* Scaling for the computation of the per-carrier noise. Scaling is to avoid overflow.
 * We are summing at most 2*uc_used_carriers/2 modulus values -scaling of 1/2^7- but the values are in Q7.26. Hence, scaling by 1/2 is enough */
#define SCALING_NK      1

/* Control whether the scaling by 1/3 in the smooth is done before or after summing the 3 values. It will depend on the amplitude of the input values */
#define SCALE_AFTER_SUM         0
#define SCALE_BEFORE_SUM        1

/* ********************************* Constants used in C and assembler *************************************** */

#define NUM_CARRIERS_TIMES_2_CENELEC_A  (2 * NUM_CARRIERS_CENELEC_A)
#define NUM_CARRIERS_TIMES_2_FCC        (2 * NUM_CARRIERS_FCC)
#define NUM_CARRIERS_TIMES_2_ARIB       (2 * NUM_CARRIERS_ARIB)

/* Maximum integer in int16_t and int32_t */
#define MAX_INT16       32767
#define MAX_INT32       2147483647LL
/* Minimum integer in int16_t and int32_t */
#define MIN_INT16       -32768
#define MIN_INT32       -2147483648LL

/* Fractional parts of Q formats */
#define FRAC_PART_Q15   15
#define FRAC_PART_Q16   16
#define FRAC_PART_Q17   17
#define FRAC_PART_Q24   24
#define FRAC_PART_Q27   27
#define FRAC_PART_Q28   28
#define FRAC_PART_Q29   29
#define FRAC_PART_Q30   30
#define FRAC_PART_Q31   31
#define FRAC_PART_Q63   63

/* Format conversion */
#define Q3_13_TO_Q1_15  2
#define Q1_31_TO_Q1_15  16
#define Q1_63_TO_Q1_31  32
#define Q1_31_TO_Q7_25  6
#define Q7_9_TO_Q7_25   16
#define Q7_9_TO_Q8_24   15
#define Q8_24_TO_Q1_31  7
#define Q1_31_TO_Q2_30  1
#define Q1_15_TO_Q8_24  9
#define Q8_24_TO_Q16_16 8

/* Value of 1/3 in Q1.15 */
#define VALUE_1_3_Q15   10923
#define VALUE_SQRT_2_Q13        11585

/* Value of 1/(2*pi) in Q1.31 and Q3.29 */
#define VALUE_1_2_PI_Q1_31      341782638L

/* Value of PI/2 in Q2.30 */
#define VALUE_PI_DIV_2_Q_2_30   1686629713L

/* Value of 1 in Q2.30 */
#define VALUE_1_Q2_30   1073741824  /* The L is removed because it is not accepted by the assembler */

/* Value of N/(N+cp-ps_length) in Q1.31 */
#define VALUE_N_DIV_SYM_LENGTH_Q1_31    1977538899L

/* Scaling applied to the values of asl_delay_symbols when converted to Q1.31 is 1/2^SCALING_DELAY_VALUES. 2^SCALING_DELAY_VALUES>longest FCH length. */
#define SCALING_DELAY_VALUES    8
/* Scaling applied to the frequency values when converted to Q1.31 is 1/2^SCALING_FREQ_VALUES */
#define SCALING_FREQ_VALUES     7

/* Log2(N), where N is number of FFT points */
#define LOG2_FFT_POINTS 8

/* Payload and FCH symbol length normalized by N and expressed in Q2.30 */
#define SYMBOL_LENGTH_SAMPLES 1166016512  /* The L is removed because it is not accepted by the assembler */

/* Value of 2*pi/N in Q1.15 */
/*#define VALUE_2PI_DIV_N	804*/
#define VALUE_2PI_DIV_N 52707179 /* in Q1.31 */

/* Value of pi in Q5.27 */
#define VALUE_PI        421657428
#define VALUE_2PI       843314857

/* ********************************* Constants exclusively used in assembly functions *************************************** */
#define VALUE_2_TO_30 1073741824 /* This value is larger than 65535 but can be generated by shifting left an 8 bit value */
#define VALUE_2_TO_15 32768
#define VALUE_2_TO_14 16384
#define VALUE_2_TO_10 1024
#define Q2_62_TO_Q16_48 14
#define SCALING_VECTOR_INPUT_BEFORE_SUM 2 /* 1/2 */
#define SCALING_VECTOR_INPUT_BEFORE_STORE 2 /* 2 */

#define Q4_60_TO_Q16_48 12
#define Q16_48_TO_Q1_31 17
#define Q2_62_TO_Q16_48 14

#define VALUE_PI_DIV_2_Q_2_30_TOP 0x6487
#define VALUE_PI_DIV_2_Q_2_30_LOW 0xED51

#define NUM_SYMBOLS_8 8
#define NUM_FULL_SYMBOLS_PREAMBLE 9

/* Parameters for computing pilots position */
#define PILOT_FREQ_SPA_AS                        12
#define PILOT_OFFSET_CENELEC_A_AS                6
#define PILOT_OFFSET_FCC_ARIB_AS                 36

#endif /* ATPL250_CHANNEL_AND_SFO_ESTIMATION_PARAMS_H_INCLUDED */
