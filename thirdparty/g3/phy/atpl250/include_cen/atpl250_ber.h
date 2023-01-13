/**
 * \file
 *
 * \brief HEADER. ATPL250 BER Configuration
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

#ifndef ATPL250_BER_H_INCLUDED
#define ATPL250_BER_H_INCLUDED

#include "atpl250_common.h"

void select_modulation_tone_map(uint8_t uc_mod_scheme, uint8_t *puc_static_and_dynamic_notching, uint8_t *puc_pilot_pos, uint8_t *puc_inactive_carriers_pos,
		uint8_t *puc_tone_map_fch, uint8_t *puc_mod_type, uint16_t us_payload_symbols, uint8_t uc_payload_carriers, uint8_t uc_rrc_notch_index);
uint8_t get_lqi_and_per_carrier_snr(uint8_t uc_mod_scheme, uint8_t *puc_static_and_dynamic_notching, uint8_t *puc_pilot_pos, uint8_t *puc_snr_per_carrier,
		uint16_t us_payload_symbols);

void ber_init(void);

void ber_config_cenelec_a(struct phy_rx_ctl *p_rx_ctl);
void ber_save_fch_info(struct s_rx_ber_fch_data_t *p_ber_fch_data);
void ber_save_payload_info(struct s_rx_ber_payload_data_t *p_ber_payload_data);

#endif /* ATPL250_BER_H_INCLUDED */
