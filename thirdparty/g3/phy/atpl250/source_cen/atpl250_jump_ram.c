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
#include <string.h>

/* Phy layer includes */
#include "atpl250_jump_ram.h"
#include "atpl250.h"
#include "atpl250_common.h"

static uint8_t auc_jumps_data[PROTOCOL_CARRIERS];
static uint8_t auc_jumps_inactive[PROTOCOL_CARRIERS];
static uint8_t auc_jumps_concat[PROTOCOL_CARRIERS];
static uint8_t auc_jumps_static[PROTOCOL_CARRIERS];
static uint16_t aus_indices_static[4];
static uint16_t aus_indices_data[8];
static uint16_t aus_indices_inactive[8];
static uint8_t auc_inv_static_notching[CARR_BUFFER_LEN];

static uint8_t auc_combined_notching_pos[CARR_BUFFER_LEN];

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
 * \param puc_jumps_dynamic   Pointer to dynamic jumps, may be access to data or inactive lsfr
 * \param pus_indices_array   Pointer to array containing indices to jumps
 *
 */
static void _jumps_to_reg(uint8_t *puc_jumps_dynamic, uint16_t *pus_indices_array)
{
	uint8_t uc_i;
	uint32_t aul_concat_indices[10];
	uint8_t auc_db_jumps[40];

	for (uc_i = 0; uc_i < PROTOCOL_CARRIERS; uc_i++) {
		/* Concatenate jumps */
		auc_jumps_concat[uc_i] = (auc_jumps_static[uc_i] << 6) | (puc_jumps_dynamic[uc_i] << 3);
	}

	/* On jumps_concat we have now what has to be written to jump ram */
	/* Now create values for DB Jump registers */
	aul_concat_indices[0] = ((uint32_t)aus_indices_static[0] << 16) | (uint32_t)aus_indices_static[1];
	aul_concat_indices[1] = ((uint32_t)aus_indices_static[2] << 16) | (uint32_t)aus_indices_static[3];
	aul_concat_indices[2] = ((uint32_t)pus_indices_array[0] << 16) | (uint32_t)pus_indices_array[1];
	aul_concat_indices[3] = ((uint32_t)pus_indices_array[2] << 16) | (uint32_t)pus_indices_array[3];
	aul_concat_indices[4] = ((uint32_t)pus_indices_array[4] << 16) | (uint32_t)pus_indices_array[5];
	aul_concat_indices[5] = ((uint32_t)pus_indices_array[6] << 16) | (uint32_t)pus_indices_array[7];
	aul_concat_indices[6] = 0;
	aul_concat_indices[7] = 0;
	aul_concat_indices[8] = 0;
	aul_concat_indices[9] = 0;

	for (uc_i = 0; uc_i < 10; uc_i++) {
		_set_mem_value((uint8_t *)&auc_db_jumps[4 * uc_i], (uint8_t *)&aul_concat_indices[uc_i], 4);
	}

	/* Set overflow so all jump ram is written */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x03FF);

	/* Write data to registers */
	pplc_if_write_rep(BCODE_JUMPID | FIRST_CARRIER, JUMP_MULTIBUFFER, &auc_jumps_concat[0], PROTOCOL_CARRIERS);
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
	for (uc_i = 0; uc_i < PROTOCOL_CARRIERS; uc_i++) {
		/* Look for values different to 0 */
		if (puc_jumps_array[uc_i]) {
			/* Check if value is aready on indices array */
			for (uc_j = 0; uc_j < uc_current_db_idx; uc_j++) {
				if (pus_indices_array[uc_j] == puc_jumps_array[uc_i]) {
					/* value already on indices array */
					break;
				}
			}
			/* Check if value was found or not */
			if (uc_j < uc_current_db_idx) {
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

	for (uc_i = FIRST_CARRIER; uc_i < CFG_IOB_SAMPLES_PER_SYMBOL; uc_i++) {
		if (uc_i > LAST_CARRIER) {
			/* If carrier is higher than last used carrier, write last jump and break */
			/* Depending on value of last bit, jump value and write index are different */
			if ((uc_prev_bit_value == 0) && uc_i > FIRST_CARRIER) {
				/* Last bit was a 0. Write on last carrier index a jump to the next symbol first bit 0 */
				puc_jump_array[uc_i - 1 - FIRST_CARRIER] = CFG_IOB_SAMPLES_PER_SYMBOL - PROTOCOL_CARRIERS + uc_index_first0;
			} else {
				/* Last bit was a 1. Write on previously stored index (if any) a jump to the next symbol first bit 0, adding the jump to reach
				 * the last carrier */
				if (uc_index_write_jump != 0xFF) {
					uc_jump_value++;
					puc_jump_array[uc_index_write_jump] = CFG_IOB_SAMPLES_PER_SYMBOL - PROTOCOL_CARRIERS + uc_index_first0 + uc_jump_value;
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
					uc_index_first0 = uc_i - FIRST_CARRIER;
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
					uc_index_write_jump = uc_i - 1 - FIRST_CARRIER;
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

/**
 * \brief Set jumps to read/write data
 *
 */
void write_jumps_data(void)
{
	/* Write jumps to registers */
	_jumps_to_reg(auc_jumps_data, aus_indices_data);
}

/**
 * \brief Set jumps to read/write inactive carriers
 *
 */
void write_jumps_inactive(void)
{
	/* Write jumps to registers */
	_jumps_to_reg(auc_jumps_inactive, aus_indices_inactive);
}

/**
 * \brief Generate jumps needed depending on notching, to read/write only desired data
 *
 * \param puc_static_notching_pos   Array containing static notching
 * \param puc_inactive_carriers     Array containing dynamic notching
 *
 */
void generate_jumps(uint8_t *puc_static_notching_pos, uint8_t *puc_inactive_carriers)
{
	uint8_t uc_i;

	/* Initialize data and inactive jumps to 0 */
	memset(auc_jumps_data, 0, PROTOCOL_CARRIERS);
	memset(auc_jumps_inactive, 0, PROTOCOL_CARRIERS);

	/* For payload jumps, write jumps to read data and to read lsfr on inactive carriers */
	for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
		auc_combined_notching_pos[uc_i] = puc_static_notching_pos[uc_i] | puc_inactive_carriers[uc_i];
	}
	_calculate_jump_values(auc_jumps_data, auc_combined_notching_pos);
	_set_values_and_indices(auc_jumps_data, aus_indices_data);

	for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
		auc_combined_notching_pos[uc_i] = puc_static_notching_pos[uc_i] | ((uint8_t)(~(unsigned int)(puc_inactive_carriers[uc_i])));
	}
	_calculate_jump_values(auc_jumps_inactive, auc_combined_notching_pos);
	_set_values_and_indices(auc_jumps_inactive, aus_indices_inactive);
}

/**
 * \brief Initialize jump ram, writing column 1 with static notching
 *
 * \param puc_static_notching_pos   Array containing static notching
 *
 */
void init_jump_ram(uint8_t *puc_static_notching_pos)
{
	uint8_t uc_i;

	/* Initialize all jumps to 0 */
	memset(auc_jumps_static, 0, PROTOCOL_CARRIERS);
	memset(auc_jumps_data, 0, PROTOCOL_CARRIERS);
	memset(auc_jumps_inactive, 0, PROTOCOL_CARRIERS);
	memset(auc_jumps_concat, 0, PROTOCOL_CARRIERS);
	memset(aus_indices_static, 0, sizeof(aus_indices_static));
	memset(aus_indices_data, 0, sizeof(aus_indices_data));
	memset(aus_indices_inactive, 0, sizeof(aus_indices_inactive));

	/* Calculate inverse of notching */
	for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
		auc_inv_static_notching[uc_i] = (uint8_t)(~(unsigned int)(puc_static_notching_pos[uc_i]));
	}

	/* Generate jumps for static notching */
	_calculate_jump_values(auc_jumps_static, puc_static_notching_pos);
	_calculate_jump_values(auc_jumps_concat, &auc_inv_static_notching[0]);
	for (uc_i = 0; uc_i < PROTOCOL_CARRIERS; uc_i++) {
		auc_jumps_static[uc_i] |= auc_jumps_concat[uc_i];
	}
	_set_values_and_indices(auc_jumps_static, aus_indices_static);

	/* Write jumps to registers */
	/* The pointers to dynamic data are useless in this init call, data pointers are provided (set to 0s at the beginning of this function) */
	_jumps_to_reg(auc_jumps_data, aus_indices_data);
}
