/**
 * \file
 *
 * \brief ATPL250 Carrier Mapping Functions
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
#include "atpl250_carrier_mapping.h"
#include "atpl250.h"

extern const uint8_t BitsSetTable256[];
extern struct band_phy_constants s_band_constants;

/**
 * \brief Generate inactive carriers array from Tone Map
 *
 * \param uc_tone_map       Tone Map to convert to array
 * \param puc_inactive_carriers_pos   Pointer to array to store the result
 *
 */
void generate_inactive_carriers_cenelec_a(uint8_t uc_tone_map, uint8_t *puc_inactive_carriers_pos)
{
	/* Clear array */
	memset(puc_inactive_carriers_pos, 0, CARR_BUFFER_LEN * sizeof(uint8_t));

	/* Generate inactive carriers */
	if (!(uc_tone_map & 0x01)) {
		puc_inactive_carriers_pos[2] |= 0x80;
		puc_inactive_carriers_pos[3] |= 0x1F;
	}

	if (!(uc_tone_map & 0x02)) {
		puc_inactive_carriers_pos[3] |= 0xE0;
		puc_inactive_carriers_pos[4] |= 0x07;
	}

	if (!(uc_tone_map & 0x04)) {
		puc_inactive_carriers_pos[4] |= 0xF8;
		puc_inactive_carriers_pos[5] |= 0x01;
	}

	if (!(uc_tone_map & 0x08)) {
		puc_inactive_carriers_pos[5] |= 0x7E;
	}

	if (!(uc_tone_map & 0x10)) {
		puc_inactive_carriers_pos[5] |= 0x80;
		puc_inactive_carriers_pos[6] |= 0x1F;
	}

	if (!(uc_tone_map & 0x20)) {
		puc_inactive_carriers_pos[6] |= 0xE0;
		puc_inactive_carriers_pos[7] |= 0x07;
	}
}

/**
 * \brief Generate inactive carriers array from Tone Map
 *
 * \param puc_tone_map                Pointer to Tone Map to convert to array
 * \param puc_inactive_carriers_pos   Pointer to array to store the result
 *
 */
void generate_inactive_carriers_fcc(uint8_t *puc_tone_map, uint8_t *puc_inactive_carriers_pos)
{
	uint8_t uc_tonemap_byte_idx;
	uint8_t uc_byte_to_write;

	/* Clear array */
	memset(puc_inactive_carriers_pos, 0, CARR_BUFFER_LEN * sizeof(uint8_t));

	/* Generate inactive carriers */
	uc_byte_to_write = 4;
	for (uc_tonemap_byte_idx = 0; uc_tonemap_byte_idx < s_band_constants.uc_tonemap_size; uc_tonemap_byte_idx++) {
		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x01)) {
			puc_inactive_carriers_pos[uc_byte_to_write] |= 0x0E;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x02)) {
			puc_inactive_carriers_pos[uc_byte_to_write] |= 0x70;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x04)) {
			puc_inactive_carriers_pos[uc_byte_to_write] |= 0x80;
			puc_inactive_carriers_pos[uc_byte_to_write + 1] |= 0x03;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x08)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 1] |= 0x1C;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x10)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 1] |= 0xE0;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x20)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 2] |= 0x07;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x40)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 2] |= 0x38;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x80)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 2] |= 0xC0;
			puc_inactive_carriers_pos[uc_byte_to_write + 3] |= 0x01;
		}

		uc_byte_to_write += 3;
	}
}

/**
 * \brief Generate inactive carriers array from Tone Map
 *
 * \param puc_tone_map      Pointer to Tone Map to convert to array
 * \param puc_inactive_carriers_pos   Pointer to array to store the result
 *
 */
void generate_inactive_carriers_arib(uint8_t *puc_tone_map, uint8_t *puc_inactive_carriers_pos)
{
	uint8_t uc_tonemap_byte_idx;
	uint8_t uc_byte_to_write;

	/* Clear array */
	memset(puc_inactive_carriers_pos, 0, CARR_BUFFER_LEN * sizeof(uint8_t));

	/* Generate inactive carriers */
	uc_byte_to_write = 4;
	for (uc_tonemap_byte_idx = 0; uc_tonemap_byte_idx < s_band_constants.uc_tonemap_size - 1; uc_tonemap_byte_idx++) {
		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x01)) {
			puc_inactive_carriers_pos[uc_byte_to_write] |= 0x0E;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x02)) {
			puc_inactive_carriers_pos[uc_byte_to_write] |= 0x70;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x04)) {
			puc_inactive_carriers_pos[uc_byte_to_write] |= 0x80;
			puc_inactive_carriers_pos[uc_byte_to_write + 1] |= 0x03;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x08)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 1] |= 0x1C;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x10)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 1] |= 0xE0;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x20)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 2] |= 0x07;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x40)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 2] |= 0x38;
		}

		if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x80)) {
			puc_inactive_carriers_pos[uc_byte_to_write + 2] |= 0xC0;
			puc_inactive_carriers_pos[uc_byte_to_write + 3] |= 0x01;
		}

		uc_byte_to_write += 3;
	}

	/* 2 pending bits (indices are correctly configured when for loop exits) */
	if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x01)) {
		puc_inactive_carriers_pos[uc_byte_to_write] |= 0x0E;
	}

	if (!(puc_tone_map[uc_tonemap_byte_idx] & 0x02)) {
		puc_inactive_carriers_pos[uc_byte_to_write] |= 0x70;
	}
}

/**
 * \brief Get Number of Active Carriers from inactive carrier array and static notching array
 *
 * \param puc_inactive_carriers_pos   Pointer to array containing inactive carriers
 * \param puc_static_notching_pos     Pointer to array containing static notching
 *
 */
uint8_t get_active_carriers(uint8_t *puc_inactive_carriers_pos, uint8_t *puc_static_notching_pos)
{
	uint8_t uc_i, uc_active_carriers;

	uc_active_carriers = s_band_constants.uc_num_carriers;
	for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
		uc_active_carriers -= BitsSetTable256[*puc_inactive_carriers_pos++ | *puc_static_notching_pos++];
	}
	return uc_active_carriers;
}

/**
 * \brief Gets first carrier to use depending on tone map and static notching
 *
 * \param puc_static_and_dynamic_notching_pos   Pointer to array containing static and dynamic notching
 *
 * \return First carrier to use
 *
 */
uint8_t get_first_used_carrier(uint8_t *puc_static_and_dynamic_notching_pos)
{
	uint8_t uc_i;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint8_t uc_first_carrier = s_band_constants.uc_first_carrier;

	for (uc_i = s_band_constants.uc_first_carrier; uc_i <= s_band_constants.uc_last_carrier; uc_i++) {
		/* Calculate byte and bit position for index */
		uc_byte_index = uc_i >> 3;
		uc_bit_index = uc_i & 0x07;
		/* If bit value is 0, carrier is active, otherwise inactive */
		if (!(puc_static_and_dynamic_notching_pos[uc_byte_index] & (1 << uc_bit_index))) {
			break;
		} else {
			uc_first_carrier++;
		}
	}

	return uc_first_carrier;
}

/**
 * \brief Gets last carrier to use depending on tone map and static notching
 *
 * \param puc_static_and_dynamic_notching_pos   Pointer to array containing static and dynamic notching
 *
 * \return Last carrier to use
 *
 */
uint8_t get_last_used_carrier(uint8_t *puc_static_and_dynamic_notching_pos)
{
	uint8_t uc_i;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint8_t uc_last_carrier = s_band_constants.uc_last_carrier;

	for (uc_i = s_band_constants.uc_last_carrier; uc_i >= s_band_constants.uc_first_carrier; uc_i--) {
		/* Calculate byte and bit position for index */
		uc_byte_index = uc_i >> 3;
		uc_bit_index = uc_i & 0x07;
		/* If bit value is 0, carrier is active, otherwise inactive */
		if (!(puc_static_and_dynamic_notching_pos[uc_byte_index] & (1 << uc_bit_index))) {
			break;
		} else {
			uc_last_carrier--;
		}
	}

	return uc_last_carrier;
}

/**
 * \brief Calculates active carriers in payload depending on static notching and tone map
 *
 * \param puc_static_and_dynamic_notching_pos   Pointer to array containing dynamic and static notching
 *
 * \return Number of carriers to use
 *
 */
uint8_t get_payload_carriers(uint8_t *puc_static_and_dynamic_notching_pos)
{
	uint8_t uc_i;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint8_t uc_num_carriers = 0;

	for (uc_i = s_band_constants.uc_first_carrier; uc_i <= s_band_constants.uc_last_carrier; uc_i++) {
		/* Calculate byte and bit position for index */
		uc_byte_index = uc_i >> 3;
		uc_bit_index = uc_i & 0x07;
		/* If bit value is 0, carrier is active, otherwise inactive */
		if (!(puc_static_and_dynamic_notching_pos[uc_byte_index] & (1 << uc_bit_index))) {
			uc_num_carriers++;
		}
	}

	return uc_num_carriers;
}
