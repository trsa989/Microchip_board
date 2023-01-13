/**
 * \file
 *
 * \brief HEADER. ATPL250 Common symbol definition
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

#ifndef ATPL250_COMMON_H_INCLUDED
#define ATPL250_COMMON_H_INCLUDED

/* ARM DSP LIB CMSIS include */
#include "compiler.h"
#include "arm_math.h"

#include "atpl250.h"

/* Compilation Constants */
#define CALCULATE_TX_PDC
#define DEMOD_AS_BPSK

#define SYMBOLS_1   1
#define SYMBOLS_2   2
#define SYMBOLS_4   4
#define SYMBOLS_8   8
#define SYMBOLS_16  16
#define CFG_SYMBOLS_IN_IOB   SYMBOLS_8  /* Use just one of the values listed above */

#if (CFG_SYMBOLS_IN_IOB == SYMBOLS_1)
	#define CFG_IOB_SAMPLES_PER_SYMBOL  1024
#elif (CFG_SYMBOLS_IN_IOB == SYMBOLS_2)
	#define CFG_IOB_SAMPLES_PER_SYMBOL  512
#elif (CFG_SYMBOLS_IN_IOB == SYMBOLS_4)
	#define CFG_IOB_SAMPLES_PER_SYMBOL  256
#elif (CFG_SYMBOLS_IN_IOB == SYMBOLS_8)
	#define CFG_IOB_SAMPLES_PER_SYMBOL  128
#elif (CFG_SYMBOLS_IN_IOB == SYMBOLS_16)
	#define CFG_IOB_SAMPLES_PER_SYMBOL  64
#endif

#define P_SYMBOL_LEN_MAX_CENELEC_A   18
#define P_SYMBOL_LEN_MAX_FCC         36
#define P_SYMBOL_LEN_MAX_ARIB        27

#define P_SYMBOL_LEN_MAX         P_SYMBOL_LEN_MAX_CENELEC_A

#define FCH_LEN_BITS_CENELEC_A                           33
#define FCH_INTERLEAVER_USEFUL_SIZE_CENELEC_A            468

#define FCH_LEN_BITS_FCC                                 66
#define FCH_INTERLEAVER_USEFUL_SIZE_FCC                  864

#define FCH_LEN_BITS_ARIB                                66
#define FCH_INTERLEAVER_USEFUL_SIZE_ARIB                 864

#define FCH_LEN_BITS                       FCH_LEN_BITS_CENELEC_A
#define FCH_INTERLEAVER_USEFUL_SIZE        FCH_INTERLEAVER_USEFUL_SIZE_CENELEC_A

#define PILOT_FREQ_SPA_CENELEC_A                        12
#define PILOT_OFFSET_CENELEC_A                          6

#define PILOT_FREQ_SPA_FCC                                      12
#define PILOT_OFFSET_FCC                                        36

#define PILOT_FREQ_SPA_ARIB                                     12
#define PILOT_OFFSET_ARIB                                       36

#define PILOT_FREQ_SPA          PILOT_FREQ_SPA_CENELEC_A
#define PILOT_OFFSET            PILOT_OFFSET_CENELEC_A

#define CARRIERS_IN_SUBBAND_CENELEC_A  6
#define CARRIERS_IN_SUBBAND_FCC        3
#define CARRIERS_IN_SUBBAND_ARIB       3

#define NUM_CARRIERS_IN_SUBBAND  CARRIERS_IN_SUBBAND_CENELEC_A

#define TX_FFT_SHIFT_CENELEC_A  1
#define RX_FFT_SHIFT_CENELEC_A  1

#define TX_FFT_SHIFT_FCC        2
#define RX_FFT_SHIFT_FCC        1

#define TX_FFT_SHIFT_ARIB       2
#define RX_FFT_SHIFT_ARIB       1

#define TX_FFT_SHIFT  TX_FFT_SHIFT_CENELEC_A
#define RX_FFT_SHIFT  RX_FFT_SHIFT_CENELEC_A

/* Transmission steps */
enum phy_tx_steps {
	STEP_TX_NO_TX = 0,
	STEP_TX_PREAMBLE_FIRST = 1,
	STEP_TX_PREAMBLE_LAST = 2,
	STEP_TX_HEADER_FIRST = 3,
	STEP_TX_HEADER = 4,
	STEP_TX_COH_S1S2 = 5,
	STEP_TX_PAYLOAD_FIRST = 6,
	STEP_TX_PAYLOAD = 7,
	STEP_TX_PAYLOAD_LAST = 8,
	STEP_TX_END = 9,
	STEP_TX_CANCELLED_TX = 10,
	STEP_TX_USER_DEF_0 = 11,
	STEP_TX_USER_DEF_1 = 12,
	STEP_TX_USER_DEF_2 = 13
};

/* Reception steps */
enum phy_rx_steps {
	STEP_RX_NO_RX = 0,
	STEP_RX_PEAK1 = 1,
	STEP_RX_PEAK2 = 2,
	STEP_RX_HEADER = 3,
	STEP_RX_PAYLOAD_FIRST = 4,
	STEP_RX_PAYLOAD = 5,
	STEP_RX_COH_S1S2 = 6,
	STEP_RX_NOISE_CAPTURE_ADAPT = 7,
	STEP_RX_NOISE_CAPTURE_FIRST_PASS = 8,
	STEP_RX_NOISE_CAPTURE_SECOND_PASS = 9,
	STEP_RX_NOISE_CAPTURE_ABORTED = 10,
	STEP_RX_WAIT_RS = 11,
	STEP_RX_SYNCM = 12
};

/* Demodulation steps */
enum demod_steps {
	STEP_DEMOD_STD = 0,
	STEP_DEMOD_ONE_BEFORE_LAST = 1,
	STEP_DEMOD_LAST = 2
};

/* Modulation modes */
enum mod_modes {
	MOD_MODE_COHERENT = 0,
	MOD_MODE_DIF_TIME = 1,
	MOD_MODE_DIF_FREQ = 2
};

/* RS blocks */
enum rs_blocks {
	RS_BLOCKS_1_BLOCK = 0,
	RS_BLOCKS_2_BLOCKS_RECEIVING_FIRST = 1,
	RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND = 2,
	RS_BLOCKS_2_BLOCKS_SECOND_RECEIVED = 3,
	RS_BLOCKS_2_BLOCKS_TRANSMITTING_FIRST = 4,
	RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND = 5
};

/* Carriers Buffer Len */
#define CARR_BUFFER_LEN  16

/* Impedance states */
#define NUM_IMPEDANCE_STATES 3

#define HI_STATE   0
#define LO_STATE   1
#define VLO_STATE  2

/* Impedance related transmission modes */
#define FIXED_STATE_FIXED_GAIN   0  /* Fixed Impedance state and applied gain */
#define AUTO_STATE_VAR_GAIN      1  /* Impedance state changes and gain is adjusted, both depending on detected impedance */
#define FIXED_STATE_VAR_GAIN     2  /* Fixed Impedance state and gain is adjusted depending on detected impedance */

/* Emit Gain Adaptation parameters */
#define EMIT_GAIN_HI            0x1E /* No equalization Gain = 0x1B */
#define EMIT_GAIN_LO            0x72 /* No equalization Gain = 0x60 */
#define EMIT_GAIN_VLO           0x72 /* No equalization Gain = 0x60 */

struct emit_gain_limits_type {
	uint8_t uc_emit_gain_init;
	uint8_t uc_emit_gain_max;
	uint8_t uc_emit_gain_min;
};

struct impedance_params_type {
	uint32_t emit_np_delay;
	uint32_t emit_on_active;
	uint32_t emit_off_active;
	uint8_t tx_conf_vl8;
	uint8_t emit_conf_l8;
	uint8_t emit_conf_vl8;
	uint8_t bf_vl8;
};

/* ABCD points array length */
#define ABCD_POINTS_LEN   8

/* CLK Resample constants */
#define RESAMPLE_CONST_1      5592405L
#define RESAMPLE_CONST_2  12653631L

/*Constants used for SFO estimation*/
#define NUM_FULL_SYMBOLS_PREAMBLE  9
#define CP_LENGTH       30
#define FFT_POINTS      256
#define LOG2_FFT_POINTS 8

/* Parameters used to compute the first sample of the FFT window of each symbol */
#define OFFSET_M 0
#define WINDOW_MS1S2 0
#define TIME_ADVANCE 11

/* Scaling applied to the values of asl_delay_symbols when converted to Q1.31 is 1/2^SCALING_DELAY_VALUES. 2^SCALING_DELAY_VALUES>longest FCH length */
#define SCALING_DELAY_VALUES    8
/* Scaling applied to the frequency values when converted to Q1.31 is 1/2^SCALING_FREQ_VALUES */
#define SCALING_FREQ_VALUES     7

/*Sample index of the first FCH sample*/
#define FIRST_SAMPLE_FCH        2424L

/*Value of 1/2 in Q1.31*/
#define VALUE_1_2_Q_1_31        1073741824L

/* structure to define control of a transmission */
struct phy_tx_ctl {
	enum tx_result_values e_tx_state;
	enum phy_tx_steps e_tx_step;
	enum mod_types e_mod_type;
	enum mod_schemes e_mod_scheme;
	uint8_t m_uc_tx_mode;
	uint8_t m_uc_pdc;
	uint8_t m_auc_tone_map[TONE_MAP_SIZE];
	uint8_t m_auc_inv_tone_map[TONE_MAP_SIZE];
	enum delimiter_types e_delimiter_type;
	enum rs_blocks e_rs_blocks;
	uint8_t m_uc_rs_parity;
	uint8_t auc_tx_buf[PHY_MAX_PPDU_SIZE];
	uint16_t m_us_payload_len;
	uint8_t m_uc_payload_carriers;
	uint16_t m_us_tx_payload_symbols;
	uint16_t m_us_tx_pending_symbols;
	uint32_t m_ul_pn_seq_idx;
	uint32_t m_ul_pilot_idx;
	uint8_t m_auc_inactive_carriers_pos[CARR_BUFFER_LEN]; /* each bit represent a carrier [0-127] (1 inactive 0 active) only for tone map */
	uint8_t m_auc_inv_inactive_carriers_pos[CARR_BUFFER_LEN]; /* Inverse of previous array */
	uint8_t m_auc_pilot_pos[CARR_BUFFER_LEN]; /* each bit represent a carrier [0-127] (1 pilot 0 no pilot) */
	uint8_t m_auc_static_and_dynamic_notching_pos[CARR_BUFFER_LEN];
	uint8_t m_auc_static_and_inv_dynamic_notching_pos[CARR_BUFFER_LEN];
	uint8_t m_uc_num_active_carriers;
	uint8_t m_uc_tx_first_carrier;
	uint8_t m_uc_tx_first_carrier_pn_seq;
	uint8_t m_uc_num_pilots;
	uint32_t m_ul_start_tx_watch_ms;
};

/* structure to define control of a reception */
struct phy_rx_ctl {
	uint8_t m_uc_rx_state;
	enum phy_rx_steps e_rx_step;
	enum demod_steps e_demod_step;
	enum mod_types e_mod_type;
	enum mod_schemes e_mod_scheme;
	uint8_t m_auc_tone_map[TONE_MAP_SIZE];
	uint8_t m_auc_inv_tone_map[TONE_MAP_SIZE];
	enum delimiter_types e_delimiter_type;
	enum rs_blocks e_rs_blocks;
	uint8_t m_uc_rs_parity;
	uint8_t m_uc_rs_corrected_errors;
	uint8_t m_uc_bit_padding;
	uint8_t auc_rx_buf[PHY_MAX_PPDU_SIZE];
	uint16_t m_us_rx_len;
	uint8_t m_uc_payload_carriers;
	uint8_t m_uc_next_demod_symbols;
	uint16_t m_us_rx_pending_symbols;
	uint16_t m_us_rx_payload_symbols;
	uint16_t m_us_evm_header;
	uint16_t m_us_evm_payload;
	uint16_t m_us_rssi;
	uint16_t m_us_agc_factor;
	uint8_t m_uc_agcs_active;
	uint16_t m_us_agc_fine;
	uint8_t m_uc_tx_pdc;
	uint8_t m_uc_rx_pdc;
	uint8_t m_uc_zct_diff;
	uint32_t m_ul_pn_seq_idx;
	uint8_t m_auc_inactive_carriers_pos[CARR_BUFFER_LEN]; /* each bit represent a carrier [0-127] (1 inactive 0 active) only for tone map */
	uint8_t m_auc_inv_inactive_carriers_pos[CARR_BUFFER_LEN]; /* Inverse of previous array */
	uint8_t m_auc_pilot_pos_first_symbol[CARR_BUFFER_LEN]; /* each bit represent a carrier [0-127] (1 pilot 0 no pilot) */
	uint8_t m_auc_pilot_pos[CARR_BUFFER_LEN]; /* each bit represent a carrier [0-127] (1 pilot 0 no pilot) */
	uint8_t m_auc_static_and_dynamic_notching_pos[CARR_BUFFER_LEN];
	uint8_t m_auc_static_and_inv_dynamic_notching_pos[CARR_BUFFER_LEN];
	uint8_t m_uc_num_active_carriers;
	uint8_t m_uc_rx_first_carrier;
	uint8_t m_uc_rx_first_carrier_pn_seq;
	uint8_t m_uc_num_pilots;
	uint32_t m_ul_pilot_idx;
	uint32_t m_ul_start_rx_watch_ms;
};

/* FCH BER structure */
struct s_rx_ber_fch_data_t {
	uint8_t uc_fch_snr_worst_carrier;
	uint16_t us_fch_corrupted_carriers;
	uint16_t us_fch_noised_symbols;
	uint8_t uc_fch_snr_worst_symbol;
	uint8_t uc_fch_snr_impulsive;
	uint8_t uc_fch_snr_be;
	uint8_t uc_fch_snr_background;
	uint16_t us_fch_acum_sym_minus8;
	uint16_t us_fch_acum_sym_minus7;
	uint16_t us_fch_acum_sym_minus6;
	uint16_t us_fch_acum_sym_minus5;
	uint16_t us_fch_acum_sym_minus4;
	uint16_t us_fch_acum_sym_minus3;
	uint16_t us_fch_acum_sym_minus2;
	uint16_t us_fch_acum_sym_minus1;
};

/* Symbol config structure */
struct sym_cfg {
	uint8_t m_uc_empty1;
	uint8_t m_uc_empty2;
	uint8_t m_uc_empty3;
	uint8_t m_uc_gain;
	uint8_t m_uc_rep1514;
	uint8_t m_uc_rep1312;
	uint8_t m_uc_rep1110;
	uint8_t m_uc_rep98;
	uint8_t m_uc_rep76;
	uint8_t m_uc_rep54;
	uint8_t m_uc_rep32;
	uint8_t m_uc_rep10;
	uint8_t m_uc_overlap;
	uint8_t m_uc_cyclicprefix;
	uint8_t m_uc_repetitions : 4;
	uint8_t m_uc_is_last_symbol : 4; /* NOTE: Nibbles reversed because of endianness */
	uint8_t m_uc_reserved : 4;
	uint8_t m_uc_sym_idx : 4; /* NOTE: Nibbles reversed because of endianness */
};

union sym_cfg_union {
	struct sym_cfg s_sym_cfg;
	uint8_t auc_sym_cfg[16];
};

/* Structures of buffers to be shared by PHY to reduce footprint */
#define MAX_MOD_DATA_LEN         108 /* 36 carriers * 3 bits per carrier * 8 symbols */
#define MAX_DEMOD_DATA_LEN       432 /* 36 carriers * 3 bits per carrier * 4 (bits in soft) * 8 symbols */
#define MAX_PN_SEQ_LEN_INACTIVE  360 /* 30 unused carriers * 3 bits per carrier * 8 symbols * 4 (bits in soft) */

#define MAX_PN_SEQ_LEN_PILOTS        48 /* 6 pilot carriers * 2 bits per carrier * 8 symbols * 4 (bits in soft) */

struct mod_demod {
	uint8_t auc_mod_demod_data[MAX_DEMOD_DATA_LEN];
	uint8_t auc_pn_seq_inactive[MAX_PN_SEQ_LEN_INACTIVE];
	uint8_t auc_pn_seq_pilots[MAX_PN_SEQ_LEN_PILOTS];
};

#define MAX_NUM_SYMBOLS_FCH                    78
#define MAX_FCH_SYMBOL_BYTE_SIZE               5
#define MAX_FCH_INTERLEAVER_MATRIX_BYTE_SIZE   116

#define EXTRA_CARRIERS_TO_READ_NOISE   32

struct fch_int {
	uint8_t interleaver_matrix_tx[MAX_FCH_INTERLEAVER_MATRIX_BYTE_SIZE];
	uint8_t auc_tx_bits_interleaver[MAX_NUM_SYMBOLS_FCH][MAX_FCH_SYMBOL_BYTE_SIZE];
};

struct sfo_est {
	q31_t asl_Y_imag[PROTOCOL_CARRIERS];
	q31_t asl_Y_real[PROTOCOL_CARRIERS];
};

struct chn_est {
	q31_t asl_squared_mag_symbol[PROTOCOL_CARRIERS];
	q31_t asl_squared_mag_symbol_1[PROTOCOL_CARRIERS];
};

struct noise {
	q15_t ass_noise_capture[(PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE) << 1];
	q15_t ass_noise_squared_mag[PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE];
	q15_t ass_noise_avg[PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE];
};

#define NUMBER_SUBBANDS_TONE_MAP        6

struct ber_info {
	uint8_t auc_ber_carrier_buff[CARR_BUFFER_LEN];
	uint8_t auc_ber_carrier_buff_tmp[CARR_BUFFER_LEN];
	q31_t asl_ber_snr_k[PROTOCOL_CARRIERS];
	uint8_t auc_ber_k[PROTOCOL_CARRIERS * 2];
	uint16_t auc_ber_k_16[PROTOCOL_CARRIERS];
	uint8_t auc_ber_tone_map[NUMBER_SUBBANDS_TONE_MAP];
	uint8_t auc_ber_new_tone_map[NUMBER_SUBBANDS_TONE_MAP];
	uint8_t auc_ber_list_corr_carr[PROTOCOL_CARRIERS];
	q31_t asl_ber_snr_good[NUMBER_SUBBANDS_TONE_MAP];
	q31_t asl_ber_snr_be[NUMBER_SUBBANDS_TONE_MAP];
	uint8_t auc_ber_num_corr_carr[NUMBER_SUBBANDS_TONE_MAP];
	uint8_t auc_ber_tone_map_series[NUMBER_SUBBANDS_TONE_MAP - 1][NUMBER_SUBBANDS_TONE_MAP];
	uint8_t auc_ber_num_subbands_tone_map_series[NUMBER_SUBBANDS_TONE_MAP - 1];
	uint8_t asc_ber_list_good_carriers[PROTOCOL_CARRIERS];
};

#define FILTER_LENGTH_H   32

struct rrc_filter {
	uint8_t auc_rrc_write_buffer[512];
	uint16_t aus_h_t[FILTER_LENGTH_H << 1];
};

struct rrc_calc {
	int16_t ass_a[FILTER_LENGTH_H];
	int32_t asl_at[FILTER_LENGTH_H];
	int16_t ass_A_row[FILTER_LENGTH_H];
	int32_t asl_alpha_32[FILTER_LENGTH_H + 4];
};

struct rs_calc {
	uint8_t auc_syndrome_log[16];
	uint8_t auc_lambda_log[12];
	uint8_t auc_omega_log[9];
	uint8_t auc_error_pos[8];
	uint8_t auc_error_val[8];
	uint8_t auc_error_and_lambda[16];
	uint8_t auc_error_out[8];
	uint8_t auc_lambda_out_log[8];
};

/* NOTE: rrc_calc memory space cannot be shared with rrc_filter.aus_h_t field */
/* It is assumed that rrc_cal size is less or equal to rrc_filter.auc_rrc_write_buffer field */
/* so memory overlapping is avoided. Be very careful if these sizes change!! */

union shared_phy_buffers {
	struct mod_demod s_mod_demod;
	struct fch_int s_fch_int;
	struct sfo_est s_sfo_est;
	struct chn_est s_chn_est;
	struct noise s_noise;
	struct ber_info s_ber_info;
	struct rrc_filter s_rrc_filter;
	struct rrc_calc s_rrc_calc;
	struct rs_calc s_rs_calc;
};

extern union shared_phy_buffers u_shared_buffers;

#endif /* ATPL250_COMMON_H_INCLUDED */
