/**
 * \file
 *
 * \brief HEADER. ATPL250 Modulator / Demodulator Access
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

#ifndef ATPL250_MOD_DEMOD_H_INCLUDED
#define ATPL250_MOD_DEMOD_H_INCLUDED

#include "atpl250_common.h"

/* Values used to modulate/demodulate depending on state machine */
#define SET_SYM_NEXT                               1
#define NO_CHANGE_SYM_NEXT                         0
#define DEFAULT_SYM_NEXT                           6

#define NO_SYM_OFFSET                              0
#define OFFSET_1_SYM                               1
#define CHANGE_MODULATION                          1
#define NO_CHANGE_MODULATION                       0
#define READ_INACTIVE_CARRIERS                     1
#define NO_READ_INACTIVE_CARRIERS                  0

/* PAYLOAD MODULATOR */
#define MAX_NUM_SYM_MOD                            8

/* PAYLOAD DEMODULATOR */
#define PAY_SYM_ONE_BEFORE_LAST_DEMOD              2
#define PAY_SYM_LAST_DEMOD                         1

/* Symbols to store references */
#define COH_REF_SYMBOL_BPSK      0x05
#define COH_REF_SYMBOL_QPSK      0x06
#define COH_REF_SYMBOL_8PSK      0x07

void set_p_symbol_phases(const uint8_t *puc_psymbol);
void set_pilot_position(uint8_t uc_num_pilots, uint8_t uc_num_active_carriers, uint8_t *puc_pilot_pos,
		uint8_t *puc_inactive_carriers_pos, uint8_t *puc_static_notching_pos);
void feed_modulator_fch(uint8_t uc_num_symbols, uint8_t uc_offset_symbols, uint8_t uc_change_mod);
void feed_modulator_payload(struct phy_tx_ctl *p_tx_ctl, uint8_t uc_num_symbols, uint8_t uc_change_sym_cfg, struct sym_cfg *p_sym_cfg, uint8_t uc_change_mod);
void get_demodulator_fch(uint8_t uc_num_symbols, uint8_t uc_num_symbols_offset, uint8_t uc_change_mod, uint8_t uc_num_symbols_next);
void get_demodulator_payload(struct phy_rx_ctl *p_rx_ctl, uint8_t uc_num_symbols, uint8_t uc_change_mod, uint8_t uc_num_symbols_next,
		uint8_t uc_read_inactive_carriers);

#endif /* ATPL250_MOD_DEMOD_H_INCLUDED */
