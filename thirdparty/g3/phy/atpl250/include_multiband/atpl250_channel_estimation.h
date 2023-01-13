/**
 * \file
 *
 * \brief HEADER. ATPL250 Channel Estimation
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

#ifndef ATPL250_CHANNEL_ESTIMATION_H_INCLUDED
#define ATPL250_CHANNEL_ESTIMATION_H_INCLUDED

#include "atpl250_channel_and_sfo_estimation_params.h"

/* ARM DSP LIB CMSIS include */
#include "compiler.h"
#include "arm_math.h"

void chn_estimation_from_preamble(void);
void chn_estimation_from_fch(uint8_t uc_mod_scheme, uint8_t uc_num_sym_used_fch);
void chn_estimation_from_s1s2(void);
void chn_estimation_from_pilots(uint8_t *puc_static_dynamic_notch_map, uint8_t uc_num_pilots_per_symbol, uint8_t uc_last_symbol_index,
		uint8_t uc_num_active_carriers, uint32_t *pul_pn_seq_index);
void control_smooth_and_order_pilot_list(uint8_t *puc_static_and_dynamic_map, uint8_t *puc_pilot_map, uint8_t *puc_data_carriers_list,
		uint8_t *puc_num_data_carrier, uint8_t *puc_pilot_carriers_list, uint8_t *puc_control_avg_invert_chan, uint8_t uc_num_pilots_per_symbol,
		uint8_t uc_index_first_symbol_block);
void compensate_sfo_in_chan_est(uint8_t uc_channel_to_be_applied);
void obtain_modulating_conj_complex_values(uint8_t *puc_input_bits, uint8_t uc_num_bits, uint8_t uc_frame_part, uint8_t *puc_carrier_list,
		q15_t *pss_modulating_symbol);

#if (defined(SMOOTHING) || defined(UPDATE_CHN_SFO_EST_PAYLOAD))
void smooth_carriers(uint8_t *puc_data_carriers_list, uint8_t uc_num_data_carriers, q15_t *pss_symbol, uint8_t uc_scale_control);

#endif

/* Array for storing the p,m fch and s1s2 symbols used for channel estimation and the payload symbols, from which pilot are extracted */
/* auc_pilots_state_carrier_block is an array of values that contain the value 32 if the corresponding carrier is used as pilot */
/* Constants set to Cen or FCC value to make room for max values */
union shared_preamble_and_payload_symbols {
	q15_t ass_pm_fch_s1s2_symbols[NUM_CARRIERS_TIMES_2_FCC * (NUM_SYM_H_EST_PRE + 2 + NUM_SYM_H_EST_FCH_CENELEC_A)];
	q15_t ass_payload_symbols[NUM_CARRIERS_TIMES_2_FCC * NUM_SYM_PILOTS_H_EST];
};

#endif /* ATPL250_CHANNEL_ESTIMATION_H_INCLUDED */
