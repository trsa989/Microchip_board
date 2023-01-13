/**
 * \file
 *
 * \brief ATPL250 Reed-Solomon block control.
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "atpl250.h"
#include "atpl250_common.h"
#include "atpl250_reed_solomon.h"
#include "atpl250_reg.h"
#include "pplc_if.h"

const uint8_t log_table[256] = {0, 1, 2, 26, 3, 51, 27, 199, 4, 224, 52, 239, 28, 105, 200, 76,
				5, 101, 225, 15, 53, 142, 240, 130, 29, 194, 106, 249, 201, 9, 77, 114,
				6, 139, 102, 48, 226, 37, 16, 34, 54, 148, 143, 219, 241, 19, 131, 70,
				30, 182, 195, 126, 107, 40, 250, 186, 202, 155, 10, 121, 78, 229, 115, 167,
				7, 192, 140, 99, 103, 222, 49, 254, 227, 153, 38, 180, 17, 146, 35, 137,
				55, 209, 149, 207, 144, 151, 220, 190, 242, 211, 20, 93, 132, 57, 71, 65,
				31, 67, 183, 164, 196, 73, 127, 111, 108, 59, 41, 85, 251, 134, 187, 62,
				203, 95, 156, 160, 11, 22, 122, 44, 79, 213, 230, 173, 116, 244, 168, 88,
				8, 113, 193, 248, 141, 129, 100, 14, 104, 75, 223, 238, 50, 198, 255, 25,
				228, 166, 154, 120, 39, 185, 181, 125, 18, 69, 147, 218, 36, 33, 138, 47,
				56, 64, 210, 92, 150, 189, 208, 206, 145, 136, 152, 179, 221, 253, 191, 98,
				243, 87, 212, 172, 21, 43, 94, 159, 133, 61, 58, 84, 72, 110, 66, 163,
				32, 46, 68, 217, 184, 124, 165, 119, 197, 24, 74, 237, 128, 13, 112, 247,
				109, 162, 60, 83, 42, 158, 86, 171, 252, 97, 135, 178, 188, 205, 63, 91,
				204, 90, 96, 177, 157, 170, 161, 82, 12, 246, 23, 236, 123, 118, 45, 216,
				80, 175, 214, 234, 231, 232, 174, 233, 117, 215, 245, 235, 169, 81, 89, 176};

const uint8_t exp_table[256] = {0, 1, 2, 4, 8, 16, 32, 64, 128, 29, 58, 116, 232, 205, 135, 19,
				38, 76, 152, 45, 90, 180, 117, 234, 201, 143, 3, 6, 12, 24, 48, 96,
				192, 157, 39, 78, 156, 37, 74, 148, 53, 106, 212, 181, 119, 238, 193, 159,
				35, 70, 140, 5, 10, 20, 40, 80, 160, 93, 186, 105, 210, 185, 111, 222,
				161, 95, 190, 97, 194, 153, 47, 94, 188, 101, 202, 137, 15, 30, 60, 120,
				240, 253, 231, 211, 187, 107, 214, 177, 127, 254, 225, 223, 163, 91, 182, 113,
				226, 217, 175, 67, 134, 17, 34, 68, 136, 13, 26, 52, 104, 208, 189, 103,
				206, 129, 31, 62, 124, 248, 237, 199, 147, 59, 118, 236, 197, 151, 51, 102,
				204, 133, 23, 46, 92, 184, 109, 218, 169, 79, 158, 33, 66, 132, 21, 42,
				84, 168, 77, 154, 41, 82, 164, 85, 170, 73, 146, 57, 114, 228, 213, 183,
				115, 230, 209, 191, 99, 198, 145, 63, 126, 252, 229, 215, 179, 123, 246, 241,
				255, 227, 219, 171, 75, 150, 49, 98, 196, 149, 55, 110, 220, 165, 87, 174,
				65, 130, 25, 50, 100, 200, 141, 7, 14, 28, 56, 112, 224, 221, 167, 83,
				166, 81, 162, 89, 178, 121, 242, 249, 239, 195, 155, 43, 86, 172, 69, 138,
				9, 18, 36, 72, 144, 61, 122, 244, 245, 247, 243, 251, 235, 203, 139, 11,
				22, 44, 88, 176, 125, 250, 233, 207, 131, 27, 54, 108, 216, 173, 71, 142};

static uint8_t uc_t;

/**
 * \brief Multiplier on Galois field
 *
 * \param uc_const1   First operand to multiply
 * \param uc_const2   Second operand to multiply
 *
 */
static uint8_t _mult_const_const_gf(uint8_t uc_const1, uint8_t uc_const2)
{
	uint16_t us_index;

	if ((uc_const1 == 0) || (uc_const2 == 0)) {
		return 0;
	} else {
		us_index = uc_const1 + uc_const2 - 1;
		if (us_index > 255) {
			us_index = us_index - 255;
		}

		return exp_table[us_index];
	}
}

/**
 * \brief Divider on Galois field
 *
 * \param uc_input    Input operand to division
 * \param uc_const1   First operand to division
 * \param uc_const2   Second operand to division
 *
 */
static uint8_t _div_const_add_const_gf(uint8_t uc_input, uint8_t uc_const1, uint8_t uc_const2)
{
	uint8_t uc_input_log;
	uint16_t us_sum;
	int16_t ss_minus;

	if ((uc_input == 0) || (uc_const1 == 0) || (uc_const2 == 0)) {
		return 0;
	} else {
		uc_input_log = log_table[uc_input];
		ss_minus = uc_input_log - uc_const1;
		if (ss_minus < 0) {
			ss_minus = ss_minus + 255;
		}

		us_sum = ss_minus + uc_const2;
		if (us_sum > 255) {
			us_sum = us_sum - 255;
		}

		return exp_table[us_sum];
	}
}

/**
 * \brief Calculate lambda parameters for Reed Solomon
 *
 */
static void _calculate_lambda(void)
{
	uint8_t uc_i, uc_j;
	int8_t sc_k;
	uint8_t auc_b_log[9];
	uint8_t auc_lambda0_log[9];
	uint8_t uc_gamma_log = 1;
	/* uint8_t uc_mul = 0; */
	uint8_t uc_r1, uc_r2, uc_rx;
	uint8_t uc_delta, uc_delta_log, uc_num, uc_lambda;
	uint8_t uc_aux1, uc_aux2;

	sc_k = 0;
	/* Initialize lambda_log and b_log */
	for (uc_i = 0; uc_i < (uc_t + 1); uc_i++) {
		/* u_shared_buffers.s_rs_calc.auc_lambda_log[uc_i] = 0; */
		auc_b_log[uc_i] = 0;
	}
	u_shared_buffers.s_rs_calc.auc_lambda_log[0] = 1;
	auc_b_log[1] = 1;

	for (uc_i = 0; uc_i < (2 * uc_t); uc_i++) {
		uc_r1 = (2 * uc_t) - uc_i; /* + 1; */
		uc_rx = uc_r1 - 1;
		if ((uc_r1 + uc_t) < (2 * uc_t)) {
			uc_r2 = uc_r1 + uc_t;
		} else {
			uc_r2 = 2 * uc_t;
		}

		uc_num = uc_r2 - uc_r1 + 1;
		uc_delta = 0;

		/* Step 1 */
		for (uc_j = 0; uc_j < uc_num; uc_j++) {
			if (u_shared_buffers.s_rs_calc.auc_lambda_log[uc_j]) {
				uc_delta
					^= _mult_const_const_gf(u_shared_buffers.s_rs_calc.auc_lambda_log[uc_j],
						u_shared_buffers.s_rs_calc.auc_syndrome_log[uc_j + uc_rx]);
				/* uc_mul++; */
			}
		}

		uc_delta_log = log_table[uc_delta];

		/* Step 2 */
		memcpy(auc_lambda0_log, u_shared_buffers.s_rs_calc.auc_lambda_log, sizeof(auc_lambda0_log));
		for (uc_j = 0; uc_j < (uc_t + 1); uc_j++) {
			uc_aux1 = 0;
			if (u_shared_buffers.s_rs_calc.auc_lambda_log[uc_j]) {
				/* uc_mul++; */
				uc_aux1 = _mult_const_const_gf(uc_gamma_log, u_shared_buffers.s_rs_calc.auc_lambda_log[uc_j]);
			}

			uc_aux2 = 0;
			if (auc_b_log[uc_j]) {
				/* uc_mul++; */
				uc_aux2 = _mult_const_const_gf(uc_delta_log, auc_b_log[uc_j]);
			}

			uc_lambda = uc_aux1 ^ uc_aux2;
			u_shared_buffers.s_rs_calc.auc_lambda_log[uc_j] = log_table[uc_lambda];
		}

		/* Step 3 */
		if ((uc_delta) && (sc_k >= 0)) {
			memcpy(&auc_b_log[1], &auc_lambda0_log[0], uc_t);
			uc_gamma_log = uc_delta_log;
			sc_k = -sc_k - 1;
		} else {
			memmove(&auc_b_log[1], &auc_b_log[0], uc_t);
			sc_k = sc_k + 1;
		}
	}
}

/**
 * \brief Calculate omega parameters for Reed Solomon
 *
 */
static void _calculate_omega(void)
{
	uint8_t uc_i;
	uint16_t us_j;
	uint16_t us_omega, us_p, us_k;

	/* Set omega to all 0s */
	/*	memset(u_shared_buffers.s_rs_calc.auc_omega_log, 0, uc_t); */

	for (uc_i = 0; uc_i < uc_t; uc_i++) {
		us_k = (2 * uc_t) - uc_i - 1;
		us_p = 0;
		us_omega = 0;

		for (us_j = us_k; us_j < (2 * uc_t); us_j++) {
			us_omega = (us_omega) ^
					(_mult_const_const_gf(u_shared_buffers.s_rs_calc.auc_lambda_log[us_p],
					u_shared_buffers.s_rs_calc.auc_syndrome_log[us_j]));
			us_p++;
		}
		u_shared_buffers.s_rs_calc.auc_omega_log[uc_i] = log_table[us_omega];
	}
}

/**
 * \brief Correct errors using parameters read from HW
 *
 * \param puc_reccode    Pointer to data buffer with errors
 * \param uc_len         Data length (including parity)
 *
 */
static void _correct_errors(uint8_t *puc_reccode, uint8_t uc_len)
{
	uint8_t uc_i, uc_j, uc_pos, uc_root, uc_omega_v, uc_accu_tb1_log, uc_ev, uc_pos_corr;
	int16_t ss_aux;

	for (uc_i = 0; uc_i < uc_t; uc_i++) {
		if (u_shared_buffers.s_rs_calc.auc_error_out[uc_i] == 0) {
			break;
		}

		uc_pos = u_shared_buffers.s_rs_calc.auc_error_out[uc_i];
		if (uc_pos == 255) {
			uc_root = 1;
		} else {
			uc_root = uc_pos + 1;
		}

		uc_omega_v = 0;
		for (uc_j = 0; uc_j < uc_t; uc_j++) {
			if (uc_j == 0) {
				uc_accu_tb1_log = 1;
			} else {
				ss_aux = (uc_pos + (uc_j - 1) * (uc_pos - 255));
				if (ss_aux < 0) {
					uc_accu_tb1_log = ((uint8_t)((ss_aux % 255) - 1) % 255) + 1;
				} else {
					uc_accu_tb1_log = (uint8_t)(ss_aux % 255) + 1;
				}
			}

			uc_omega_v = (uc_omega_v) ^ (_mult_const_const_gf(uc_accu_tb1_log, u_shared_buffers.s_rs_calc.auc_omega_log[uc_j]));
		}

		uc_ev = _div_const_add_const_gf(uc_omega_v, u_shared_buffers.s_rs_calc.auc_lambda_out_log[uc_i], uc_root);
		u_shared_buffers.s_rs_calc.auc_error_val[uc_i] = uc_ev;
		u_shared_buffers.s_rs_calc.auc_error_pos[uc_i] = uc_pos;
		uc_pos_corr = (uint8_t)((uint16_t)uc_pos + (uint16_t)uc_len - 256);
		puc_reccode[uc_pos_corr] = (puc_reccode[uc_pos_corr]) ^ (uc_ev);
	}
}

/**
 * \brief Error correction using Reed Solomon
 *
 * \param puc_data_buf    Pointer to data buffer
 * \param uc_data_len     Data length (not including parity)
 * \param uc_parity       Parity type
 *
 */
uint8_t rs_correct_errors(uint8_t *puc_data_buf, uint8_t uc_data_len, uint8_t uc_parity)
{
	uint8_t uc_syndrome_zero = 1;
	uint8_t uc_i;
	uint8_t uc_num_errors;
	uint16_t us_j;
	volatile uint32_t ul_int_flags;

	/* Read syndrome */
	memset(u_shared_buffers.s_rs_calc.auc_syndrome_log, 0, sizeof(u_shared_buffers.s_rs_calc.auc_syndrome_log));
	for (uc_i = 0; uc_i < uc_parity; uc_i++) {
		u_shared_buffers.s_rs_calc.auc_syndrome_log[uc_parity - 1 - uc_i] = puc_data_buf[uc_data_len + uc_parity + uc_i];
		if (u_shared_buffers.s_rs_calc.auc_syndrome_log[uc_parity - 1 - uc_i]) {
			uc_syndrome_zero = 0;
		}
	}

	/* If syndrome is all 0, there are no errors */
	if (uc_syndrome_zero) {
		LOG_PHY(("S0\r\n"));
		return 0;
	}

	/* There are errors, correct them */
	memset(u_shared_buffers.s_rs_calc.auc_lambda_log, 0, 12);
	memset(u_shared_buffers.s_rs_calc.auc_omega_log, 0, 9);
	memset(u_shared_buffers.s_rs_calc.auc_error_pos, 0, 8);
	memset(u_shared_buffers.s_rs_calc.auc_error_val, 0, 8);
	uc_t = uc_parity >> 1;

	/* Calculate Lambda */
	_calculate_lambda();

	/* Write it to registers */
	pplc_if_write_buf(REG_ATPL250_RS_LAMBDA0123_32, u_shared_buffers.s_rs_calc.auc_lambda_log, 12);
	/* Reset Chien block */
	pplc_if_or8(REG_ATPL250_RS_CFG_VL8, 0x80);

	/* Trigger Chien calculation */
	pplc_if_or8(REG_ATPL250_RS_CFG_VL8, 0x10);

	/* In parallel, calculate omega */
	_calculate_omega();

	ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
	us_j = 0;
	while (!(ul_int_flags & INT_RS_MASK_32)) {
		/* Protection from trap */
		us_j++;
		if (us_j >= 10000) {
			break;
		}

		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
	}
	/* Clear interrupt flag */
	atpl250_clear_rs_int();

	if (us_j >= 10000) {
		/* RS HW error. Exit discarding RS info */
		return 0;
	}

	/* Read number of errors chien has detected */
	uc_num_errors = pplc_if_read8(REG_ATPL250_RS_NUM_ERR_VL8);

	LOG_PHY(("RSErr %u\r\n", uc_num_errors));

	/* If errors detected are greater than t (parity/2), or coded as 0, RS cannot correct the frame */
	if ((uc_num_errors > uc_t) || (uc_num_errors == 0)) {
		return 255;
	}

	/* Errors can be corrected. Read Error Out and Labda Out */
	pplc_if_read_buf(REG_ATPL250_RS_ERR_LAMBDA01_32, u_shared_buffers.s_rs_calc.auc_error_and_lambda, 16);
	/* Build out arrays */
	for (uc_i = 0; uc_i < 16; uc_i += 2) {
		u_shared_buffers.s_rs_calc.auc_error_out[uc_i >> 1] = 255 - (uc_data_len + uc_parity) + u_shared_buffers.s_rs_calc.auc_error_and_lambda[uc_i];
		u_shared_buffers.s_rs_calc.auc_lambda_out_log[uc_i >> 1] = u_shared_buffers.s_rs_calc.auc_error_and_lambda[uc_i + 1];
	}

	/* Finally, correct errors */
	_correct_errors(puc_data_buf, uc_data_len + uc_parity);

	return uc_num_errors;
}
