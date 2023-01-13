/**
 * \file
 *
 * \brief ATPL250 Jump RAM configuration
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
#include "atpl250_jump_ram.h"
#include "atpl250.h"
#include "atpl250_common.h"

static uint8_t auc_jumps_concat[NUM_CARRIERS_FCC];
static uint8_t auc_jumps_static[NUM_CARRIERS_FCC];
static uint8_t auc_jumps_static_simple[NUM_CARRIERS_FCC];
static uint8_t auc_jumps_empty[NUM_CARRIERS_FCC];
static uint16_t aus_indices_static[8];
static uint16_t aus_indices_static_simple[4];
static uint16_t aus_indices_empty[8];

static uint8_t auc_static_notching_pos_simple[CARR_BUFFER_LEN];
static uint8_t auc_empty_pos[CARR_BUFFER_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t uc_first_carrier_simple;
static uint8_t uc_num_carriers_simple;

/* Extern variables needed */
extern struct band_phy_constants s_band_constants;

typedef struct s_hole {
	uint8_t uc_start;
	uint8_t uc_end;
	uint8_t uc_length; /* used to indicate active/inactive entry */
} s_hole_t;

#define MAX_HOLES NUM_CARRIERS_FCC >> 1
static s_hole_t as_hole_list[MAX_HOLES];
static uint8_t auc_hole_distance[MAX_HOLES];
static uint8_t auc_diff_hole_lens[MAX_HOLES];
static uint8_t auc_temp_tonemas_acl[NUM_CARRIERS_FCC];

static void _init_hole_list(void)
{
	uint8_t uc_i;
	for (uc_i = 0; uc_i < MAX_HOLES; uc_i++) {
		as_hole_list[uc_i].uc_start = 0;
		as_hole_list[uc_i].uc_end = 0;
		as_hole_list[uc_i].uc_length = 0;
	}
}

static uint8_t _search_holes(uint8_t *p_auc_tonemask, uint8_t *puc_first_carrier, uint8_t *puc_last_carrier)
{
	bool b_hole_found = false;
	uint8_t uc_idx;
	uint8_t uc_hole_idx = 0;
	uint8_t uc_num_holes = 0;

	s_hole_t *p_hole = &as_hole_list[uc_hole_idx];

	for (uc_idx = 0; uc_idx < s_band_constants.uc_num_carriers; uc_idx++) {
		if (p_auc_tonemask[uc_idx] == 1) {
			if (b_hole_found) {
				p_hole->uc_end = uc_idx - 1;
				p_hole->uc_length = p_hole->uc_end - p_hole->uc_start + 1;
				uc_hole_idx++;
				uc_num_holes++;
				b_hole_found = false;
			}
		} else {
			if (!b_hole_found) {
				p_hole = &as_hole_list[uc_hole_idx];
				p_hole->uc_start = uc_idx;
				b_hole_found = true;
			}
		}

		if (uc_idx == (s_band_constants.uc_num_carriers - 1) && b_hole_found) {
			p_hole->uc_end = uc_idx;
			p_hole->uc_length = p_hole->uc_end - p_hole->uc_start + 1;
			uc_hole_idx++;
			uc_num_holes++;
			b_hole_found = false;
		}
	}
	*puc_first_carrier = 0;
	*puc_last_carrier = s_band_constants.uc_num_carriers - 1;

	/* remove holes if they are in the extremes */
	if (uc_num_holes != 0 && as_hole_list[0].uc_start == 0) {
		*puc_first_carrier = as_hole_list[0].uc_end + 1;
		as_hole_list[0].uc_length = 0;  /* invalidate entry of first hole */
		uc_num_holes--;
		/* ensure first element contains first hole */
		memmove(&as_hole_list[0], &as_hole_list[1], uc_num_holes * sizeof(s_hole_t));
		as_hole_list[uc_num_holes].uc_length = 0;  /* invalidate entry of last hole */
	}

	if (uc_num_holes != 0 && as_hole_list[uc_num_holes - 1].uc_end == s_band_constants.uc_num_carriers - 1) {
		*puc_last_carrier = as_hole_list[uc_num_holes - 1].uc_start - 1;
		as_hole_list[uc_num_holes - 1].uc_length = 0;  /* invalidate entry of last hole */
		uc_num_holes--;
	}

	return uc_num_holes;
}

static void _join_closest_holes(uint8_t *p_auc_tonemask, uint8_t *puc_first_carrier, uint8_t *puc_last_carrier, uint8_t uc_num_holes)
{
	uint8_t uc_i, uc_min_distance_value, uc_min_distance_idx;
	uint8_t uc_start_to_1st_hole = as_hole_list[0].uc_start - *puc_first_carrier;
	uint8_t uc_last_hole_to_end = *puc_last_carrier - as_hole_list[uc_num_holes - 1].uc_end;

	uint8_t uc_gap_start, uc_gap_end;

	/*	printf("start_to_first_hole %d\r\n", uc_start_to_1st_hole); */
	/*	printf("last_hole_to_end %d\r\n", uc_last_hole_to_end); */

	uc_min_distance_value = 255;
	uc_min_distance_idx = 0;

	memset(auc_hole_distance, 0x00, MAX_HOLES * sizeof(uint8_t));

	for (uc_i = 0; uc_i < uc_num_holes - 1; uc_i++) {
		auc_hole_distance[uc_i] = as_hole_list[uc_i + 1].uc_start - as_hole_list[uc_i].uc_end - 1;
		/*		printf("%d - %d  - 1 = %d\r\n", as_hole_list[uc_i + 1].uc_start, as_hole_list[uc_i].uc_end, as_hole_list[uc_i + 1].uc_start - as_hole_list[uc_i].uc_end - 1); */

		if (auc_hole_distance[uc_i] < uc_min_distance_value) {
			uc_min_distance_value = auc_hole_distance[uc_i];
			uc_min_distance_idx = uc_i;
		}
	}

	if (uc_min_distance_value < uc_start_to_1st_hole) {
		if (uc_min_distance_value < uc_last_hole_to_end) {
			/*			printf("min_distances is the min\r\n"); */
			uc_gap_start = as_hole_list[uc_min_distance_idx].uc_end + 1;
			uc_gap_end = as_hole_list[uc_min_distance_idx + 1].uc_start;
		} else {
			/*			printf("last_hole_to_end is smaller than min_distances and start_to_first_hole\r\n"); */
			uc_gap_start = as_hole_list[uc_num_holes - 1].uc_end + 1;
			uc_gap_end = *puc_last_carrier + 1;
		}
	} else {
		if (uc_start_to_1st_hole < uc_last_hole_to_end) {
			/*			printf("start_to_first_hole is smaller than last_hole_to_end\r\n"); */
			uc_gap_start = *puc_first_carrier;
			uc_gap_end = as_hole_list[0].uc_start;
		} else {
			/*			printf("last_hole_to_end is smaller than start_to_first_hole\r\n"); */
			uc_gap_start = as_hole_list[uc_num_holes - 1].uc_end + 1;
			uc_gap_end = *puc_last_carrier + 1;
		}
	}

	/*	printf("Hole from %d to %d\r\n", uc_gap_start, uc_gap_end); */
	for (uc_i = uc_gap_start; uc_i < uc_gap_end; uc_i++) {
		p_auc_tonemask[uc_i] = 0;
	}
}

static uint8_t _get_different_hole_lens(uint8_t uc_num_holes)
{
	uint8_t uc_num_diff_hole_lens;
	uint8_t uc_i, uc_j;
	bool b_match;

	if (uc_num_holes == 0) {
		return 0;
	}

	uc_num_diff_hole_lens = 1;
	auc_diff_hole_lens[0] = as_hole_list[0].uc_length;

	for (uc_i = 1; uc_i < uc_num_holes; uc_i++) {
		b_match = false;
		for (uc_j = 0; uc_j < uc_num_diff_hole_lens; uc_j++) {
			if (auc_diff_hole_lens[uc_j] == as_hole_list[uc_i].uc_length) {
				b_match = true;
				break;
			}
		}
		if (!b_match) {
			auc_diff_hole_lens[uc_num_diff_hole_lens] = as_hole_list[uc_i].uc_length;
			uc_num_diff_hole_lens++;
		}
	}
	return uc_num_diff_hole_lens;
}

static uint8_t _count_active_carriers(uint8_t *p_auc_tonemask)
{
	uint8_t uc_idx;
	uint8_t uc_num_carriers = 0;

	for (uc_idx = 0; uc_idx < s_band_constants.uc_num_carriers; uc_idx++) {
		if (p_auc_tonemask[uc_idx] == 1) {
			uc_num_carriers++;
		}
	}
	return uc_num_carriers;
}

static void _simplify_tonemask(uint8_t *p_auc_tonemask, uint8_t *puc_first_carrier, uint8_t *puc_last_carrier, uint8_t *puc_num_carriers)
{
	uint8_t uc_num_dif_hole_lens = 0xff;
	uint8_t uc_num_holes;

	do {
		_init_hole_list();
		uc_num_holes = _search_holes(p_auc_tonemask, puc_first_carrier, puc_last_carrier);
		uc_num_dif_hole_lens = _get_different_hole_lens(uc_num_holes);
		if (uc_num_dif_hole_lens > 2) {
			_join_closest_holes(p_auc_tonemask, puc_first_carrier, puc_last_carrier, uc_num_holes);
		}
	} while (uc_num_dif_hole_lens > 2);

	*puc_num_carriers = _count_active_carriers(p_auc_tonemask);
}

static void _from_hwf_to_acl(uint8_t *p_auc_hardware_format, uint8_t *p_auc_active_carrier_list)
{
	uint8_t uc_i;

	memset(p_auc_active_carrier_list, 0x01, s_band_constants.uc_num_carriers * sizeof(uint8_t));

	for (uc_i = s_band_constants.uc_first_carrier; uc_i <= s_band_constants.uc_last_carrier; uc_i++) {
		if (p_auc_hardware_format[uc_i / 8] & (1 << (uc_i % 8))) {
			p_auc_active_carrier_list[uc_i - s_band_constants.uc_first_carrier] = 0;
		}
	}
}

static void _from_acl_to_hwf(uint8_t *p_auc_active_carrier_list, uint8_t *p_auc_hardware_format)
{
	uint8_t uc_i;

	memset(p_auc_hardware_format, 0x00, 16 * sizeof(uint8_t));

	for (uc_i = s_band_constants.uc_first_carrier; uc_i <= s_band_constants.uc_last_carrier; uc_i++) {
		if (p_auc_active_carrier_list[uc_i - s_band_constants.uc_first_carrier] == 0) {
			p_auc_hardware_format[uc_i / 8] |= (1 << (uc_i % 8));
		}
	}
}

static void _simplify_hw_tonemask(uint8_t *p_auc_hardware_format_in, uint8_t *p_auc_hardware_format_out, uint8_t *puc_first_carrier, uint8_t *puc_num_carriers)
{
	uint8_t uc_fc, uc_lc, uc_active_carriers;

	_from_hwf_to_acl(p_auc_hardware_format_in, auc_temp_tonemas_acl);
	_simplify_tonemask(auc_temp_tonemas_acl, &uc_fc, &uc_lc, &uc_active_carriers);
	_from_acl_to_hwf(auc_temp_tonemas_acl, p_auc_hardware_format_out);

	*puc_first_carrier = uc_fc;
	*puc_num_carriers = uc_active_carriers;
}

/**
 * \brief Set memory value
 *
 * \param puc_dst     Pointer to destination buffer
 * \param puc_buf     Pointer to source buffer
 * \param us_len      Number of bytes to copy
 *
 */
static inline void _set_mem_value(uint8_t *puc_dst, uint8_t *puc_buf, uint16_t us_len)
{
	uint8_t *puc_memPtrDst, *puc_memPtrSrc;
	uint16_t us_i;

	if (us_len <= 4) {
		puc_memPtrDst = puc_dst + us_len - 1;
		puc_memPtrSrc = puc_buf;
		for (us_i = 0; us_i < us_len; us_i++) {
			*puc_memPtrDst-- = (uint8_t)*puc_memPtrSrc++;
		}
	} else {
		memcpy(puc_dst, puc_buf, us_len);
	}
}

/**
 * \brief Write calculated jumps to HW registers
 *
 */
static void _jumps_to_reg(void)
{
	uint8_t uc_i;
	uint32_t aul_concat_indices[10];
	uint8_t auc_db_jumps[40];

	for (uc_i = 0; uc_i < s_band_constants.uc_num_carriers; uc_i++) {
		/* Concatenate jumps */
		auc_jumps_concat[uc_i] = (auc_jumps_static_simple[uc_i] << 6) | (auc_jumps_static[uc_i] << 3) | auc_jumps_empty[uc_i];
	}

	/* On jumps_concat we have now what has to be written to jump ram */
	/* Now create values for DB Jump registers */
	aul_concat_indices[0] = ((uint32_t)aus_indices_static_simple[0] << 16) | (uint32_t)aus_indices_static_simple[1];
	aul_concat_indices[1] = ((uint32_t)aus_indices_static_simple[2] << 16) | (uint32_t)aus_indices_static_simple[3];
	aul_concat_indices[2] = ((uint32_t)aus_indices_static[0] << 16) | (uint32_t)aus_indices_static[1];
	aul_concat_indices[3] = ((uint32_t)aus_indices_static[2] << 16) | (uint32_t)aus_indices_static[3];
	aul_concat_indices[4] = ((uint32_t)aus_indices_static[4] << 16) | (uint32_t)aus_indices_static[5];
	aul_concat_indices[5] = ((uint32_t)aus_indices_static[6] << 16) | (uint32_t)aus_indices_static[7];
	aul_concat_indices[6] = ((uint32_t)aus_indices_empty[0] << 16) | (uint32_t)aus_indices_empty[1];
	aul_concat_indices[7] = ((uint32_t)aus_indices_empty[2] << 16) | (uint32_t)aus_indices_empty[3];
	aul_concat_indices[8] = ((uint32_t)aus_indices_empty[4] << 16) | (uint32_t)aus_indices_empty[5];
	aul_concat_indices[9] = ((uint32_t)aus_indices_empty[6] << 16) | (uint32_t)aus_indices_empty[7];

	for (uc_i = 0; uc_i < 10; uc_i++) {
		_set_mem_value((uint8_t *)&auc_db_jumps[4 * uc_i], (uint8_t *)&aul_concat_indices[uc_i], 4);
	}

	/* Set overflow so all jump ram is written */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x03FF);

	/* Write data to registers */
	pplc_if_write_rep(BCODE_JUMPID | s_band_constants.uc_first_carrier, JUMP_MULTIBUFFER, &auc_jumps_concat[0], s_band_constants.uc_num_carriers);
	pplc_if_write_buf(REG_ATPL250_SPI_DB_JUMP_C1R01_32, auc_db_jumps, 40);
}

/**
 * \brief Convert jump values to values and indices
 *
 * \param puc_jumps_array     Pointer to array containing jump values
 * \param pus_indices_array   Pointer to array to set jumps as indices
 *
 */
static void _set_values_and_indices(uint8_t *puc_jumps_array, uint16_t *pus_indices_array)
{
	uint8_t uc_i, uc_j;
	uint8_t uc_current_db_idx;

	/* Go through array looking for values, and changing them for indices */
	uc_current_db_idx = 0; /* position 0 reserved for value 0 (already set) */
	for (uc_i = 0; uc_i < s_band_constants.uc_num_carriers; uc_i++) {
		/* Look for values different to 0 */
		if (puc_jumps_array[uc_i]) {
			/* Check if value is aready on indices array */
			for (uc_j = 1; uc_j < (uc_current_db_idx + 1); uc_j++) {
				/* Start from index 1 because position 0 is reserved for jump 0 */
				if (pus_indices_array[uc_j] == puc_jumps_array[uc_i]) {
					/* value already on indices array */
					break;
				}
			}
			/* Check if value was found or not */
			if (uc_j < (uc_current_db_idx + 1)) {
				/* Found */
				puc_jumps_array[uc_i] = uc_j;
			} else {
				/* Not found */
				uc_current_db_idx++;
				pus_indices_array[uc_current_db_idx] = puc_jumps_array[uc_i];
				puc_jumps_array[uc_i] = uc_current_db_idx;
			}
		}
	}
}

/**
 * \brief Calcaulate jump values and positions
 *
 * \param puc_jump_array          Array to where write jumps to
 * \param puc_inactive_carriers   Array containing dynamic notching
 *
 */
static void _calculate_jump_values(uint8_t *puc_jump_array, uint8_t *puc_inactive_carriers)
{
	uint8_t uc_i;
	uint8_t uc_index_first0;
	uint8_t uc_index_write_jump;
	uint8_t uc_jump_value;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint8_t uc_curr_bit_value, uc_prev_bit_value = 0;

	/* Initialize valriables */
	uc_index_write_jump = 0xFF;
	uc_index_first0 = 0xFF;
	uc_jump_value = 0;

	for (uc_i = s_band_constants.uc_first_carrier; uc_i < CFG_IOB_SAMPLES_PER_SYMBOL; uc_i++) {
		if (uc_i > s_band_constants.uc_last_carrier) {
			/* If carrier is higher than last used carrier, write last jump and break */
			/* Depending on value of last bit, jump value and write index are different */
			if ((uc_prev_bit_value == 0) && uc_i > s_band_constants.uc_first_carrier) {
				/* Last bit was a 0. Write on last carrier index a jump to the next symbol first bit 0 */
				puc_jump_array[uc_i - 1 - s_band_constants.uc_first_carrier] = CFG_IOB_SAMPLES_PER_SYMBOL - s_band_constants.uc_num_carriers + uc_index_first0;
			} else {
				/* Last bit was a 1. Write on previously stored index (if any) a jump to the next symbol first bit 0, adding the jump to reach
				 * the last carrier */
				if (uc_index_write_jump != 0xFF) {
					uc_jump_value++;
					puc_jump_array[uc_index_write_jump] = CFG_IOB_SAMPLES_PER_SYMBOL - s_band_constants.uc_num_carriers + uc_index_first0 + uc_jump_value;
				}
			}

			/* Once last carrier is reached, exit loop */
			break;
		} else {
			/* Calculate byte and bit position for index */
			uc_byte_index = uc_i >> 3;
			uc_bit_index = uc_i & 0x07;
			/* Get current value */
			uc_curr_bit_value = (puc_inactive_carriers[uc_byte_index] & (1 << uc_bit_index)) >> uc_bit_index;
			if (uc_index_first0 == 0xFF) {
				/* Look for first 0 */
				if (uc_curr_bit_value == 0) {
					/* 0 found, index has to be stored as relative value from first carrier */
					uc_index_first0 = uc_i - s_band_constants.uc_first_carrier;
				}
			} else {
				/* Now depending on current and previous value, calculate jump, write jump or do nothing */
				if ((uc_curr_bit_value == 0) && (uc_prev_bit_value == 0)) {
					/* For bit 0 groups, do nothing */
				} else if ((uc_curr_bit_value == 0) && (uc_prev_bit_value == 1)) {
					/* Transition from 1 to 0 */
					/* Increase jump value and write it in the previously stored index */
					uc_jump_value++;
					puc_jump_array[uc_index_write_jump] = uc_jump_value;
					/* Reset jump value for next gap */
					uc_jump_value = 0;
				} else if ((uc_curr_bit_value == 1) && (uc_prev_bit_value == 0)) {
					/* Transition from 0 to 1 */
					/* Save previous index value, to later write there the jump value */
					uc_index_write_jump = uc_i - 1 - s_band_constants.uc_first_carrier;
				} else if ((uc_curr_bit_value == 1) && (uc_prev_bit_value == 1)) {
					/* For each bit 1 found, increase jump value */
					uc_jump_value++;
				}
			}

			/* Set current bit value as previous for the next loop */
			uc_prev_bit_value = uc_curr_bit_value;
		}
	}
}

void get_jump_simple_params(uint8_t *puc_first_carrier_simple, uint8_t *puc_num_carriers_simple)
{
	*puc_first_carrier_simple = uc_first_carrier_simple;
	*puc_num_carriers_simple = uc_num_carriers_simple;
}

/**
 * \brief Initialize jump ram, writing column 1 with static notching
 *
 * \param puc_static_notching_pos   Array containing static notching
 *
 */
void init_jump_ram(uint8_t *puc_static_notching_pos)
{
	/* Initialize all jumps to 0 */
	memset(auc_jumps_static, 0, sizeof(auc_jumps_static));
	memset(auc_jumps_static_simple, 0, sizeof(auc_jumps_static_simple));
	memset(auc_jumps_empty, 0, sizeof(auc_jumps_empty));
	memset(auc_jumps_concat, 0, sizeof(auc_jumps_concat));
	memset(aus_indices_static, 0, sizeof(aus_indices_static));
	memset(aus_indices_static_simple, 0, sizeof(aus_indices_static_simple));
	memset(aus_indices_empty, 0, sizeof(aus_indices_empty));

	/* Simplify tonemask to initialize JUMP_COL_1 for SYNCM detection. */
	_simplify_hw_tonemask(puc_static_notching_pos, auc_static_notching_pos_simple, &uc_first_carrier_simple, &uc_num_carriers_simple);

	/* Generate jumps */
	_calculate_jump_values(auc_jumps_static, puc_static_notching_pos);
	_calculate_jump_values(auc_jumps_static_simple, auc_static_notching_pos_simple);
	_calculate_jump_values(auc_jumps_empty, auc_empty_pos);

	/* Convert to values and indices */
	_set_values_and_indices(auc_jumps_static, aus_indices_static);
	_set_values_and_indices(auc_jumps_static_simple, aus_indices_static_simple);
	_set_values_and_indices(auc_jumps_empty, aus_indices_empty);

	/* Write jumps to registers */
	_jumps_to_reg();
}
