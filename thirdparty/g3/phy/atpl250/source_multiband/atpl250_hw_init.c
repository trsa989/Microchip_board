/**
 * \file
 *
 * \brief ATPL250 HW Register initialization
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
#include "atpl250.h"
#include "conf_fw.h"
#include "conf_hw.h"
#include "atpl250_common.h"
#include "atpl250_hw_init.h"
#include "atpl250_jump_ram.h"

/* Extern variables needed */
extern struct band_phy_constants s_band_constants;
extern uint8_t uc_working_band;
extern const uint8_t cauc_abcd_fullgain[];
extern uint8_t uc_used_carriers;
extern uint8_t uc_notched_carriers;
extern uint8_t uc_num_symbols_fch;
extern uint8_t uc_legacy_mode;
extern uint8_t auc_masked_carrier_list[];
extern uint8_t auc_unmasked_carrier_list[];
extern uint8_t auc_predistortion[];
extern uint8_t uc_psymbol_len;
extern uint8_t auc_psymbol[];

#define ENABLE_DYNAMIC_CHIRP_CONFIGURATION

/* Constant buffers for PHY configuration */
const uint8_t auc_coefs[1024]
	= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1D, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2B, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2E, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x21,
	   0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x38, 0x21, 0x00, 0x00, 0xF6, 0x3F, 0x00, 0x00, 0x39, 0x21,
	   0x00, 0x00, 0xDA, 0x86, 0x00, 0x00, 0x3A, 0x21, 0x00, 0x00, 0xB0, 0xF9, 0x00, 0x00, 0x3B, 0x21,
	   0xE0, 0x00, 0x80, 0x00, 0x00, 0x00, 0x3C, 0x21, 0xA0, 0x00, 0x4F, 0x07, 0x00, 0x00, 0x3D, 0x21,
	   0x60, 0x00, 0x25, 0x7A, 0x00, 0x00, 0x3E, 0x21, 0x20, 0x00, 0x09, 0xC1, 0x00, 0x00, 0x3F, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4E, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5B, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5C, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6D, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6E, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x74, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x21,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x21,
	   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x78, 0x21, 0x00, 0x00, 0x09, 0xC1, 0x00, 0x00, 0x79, 0x21,
	   0x00, 0x00, 0x25, 0x7A, 0x00, 0x00, 0x7A, 0x21, 0x00, 0x00, 0x4F, 0x07, 0x00, 0x00, 0x7B, 0x21,
	   0x20, 0x00, 0x80, 0x00, 0x00, 0x00, 0x7C, 0x21, 0x60, 0x00, 0xB0, 0xF9, 0x00, 0x00, 0x7D, 0x21,
	   0xA0, 0x00, 0xDA, 0x86, 0x00, 0x00, 0x7E, 0x21, 0xE0, 0x00, 0xF6, 0x3F, 0x00, 0x00, 0x7F, 0x21};

#ifndef ENABLE_DYNAMIC_CHIRP_CONFIGURATION
const uint8_t auc_chirp_cenelec_a[576]
	= {0xA0, 0xD0, 0xB8, 0xB0, 0x10, 0x94, 0x5D, 0xB3, 0x5B, 0x73, 0x2A, 0x82, 0x13, 0x6B, 0x11, 0x0D,
	   0xE8, 0x96, 0xA5, 0x8D, 0x9E, 0x3A, 0xE9, 0xDB, 0x2D, 0xD6, 0x24, 0xA5, 0x07, 0x14, 0x24, 0x0E, 0x00, 0x00, 0x00, 0x11,
	   0x59, 0xB6, 0x74, 0x33, 0x49, 0x35, 0xE6, 0x32, 0x9A, 0x0D, 0x94, 0x50, 0xB7, 0x32, 0xDA, 0x8D,
	   0x06, 0x5F, 0x46, 0x77, 0x72, 0xA0, 0x5B, 0x44, 0x15, 0xCB, 0xE2, 0x47, 0xD0, 0x23, 0xBA, 0xAB, 0x00, 0x00, 0x01, 0x11,
	   0x1E, 0x92, 0xD0, 0x1B, 0x96, 0x8E, 0xA8, 0xB4, 0xEF, 0x95, 0x2D, 0x7F, 0x4B, 0x7A, 0x57, 0x65,
	   0x4B, 0x70, 0x10, 0x54, 0xB9, 0xF7, 0x8A, 0xE2, 0xA5, 0x21, 0xE0, 0xF1, 0x0C, 0x9F, 0x2D, 0x78, 0x00, 0x00, 0x02, 0x11,
	   0xF7, 0x80, 0xBB, 0x49, 0xAC, 0xD0, 0xC4, 0xB4, 0xF4, 0xA0, 0x2F, 0x4C, 0x54, 0xEA, 0x3F, 0xEF,
	   0xF8, 0x2A, 0xBB, 0xE1, 0xBC, 0xC9, 0xE8, 0x91, 0x0D, 0x9B, 0x20, 0x13, 0x33, 0xEA, 0x40, 0x53, 0x00, 0x00, 0x03, 0x11,
	   0xF7, 0xDD, 0xC0, 0x18, 0xA6, 0xBD, 0xC2, 0xFC, 0x12, 0x2E, 0x61, 0x5F, 0x70, 0x1D, 0x31, 0xCC,
	   0xD9, 0x46, 0xA1, 0x4A, 0x9E, 0x03, 0xC6, 0x69, 0x0B, 0x08, 0x50, 0x46, 0x6B, 0x47, 0x44, 0x6A, 0x00, 0x00, 0x04, 0x11,
	   0x4F, 0x3C, 0x03, 0x69, 0xB8, 0x02, 0xA5, 0xF6, 0xCC, 0x45, 0x07, 0xCF, 0x3C, 0x6B, 0x55, 0x60,
	   0x3B, 0xCF, 0xF1, 0x41, 0xA9, 0xB6, 0xA5, 0xBD, 0xEB, 0x24, 0x3B, 0x20, 0x56, 0x3C, 0x34, 0xD1, 0x00, 0x00, 0x05, 0x11,
	   0xFF, 0x5C, 0x57, 0x67, 0x6B, 0x92, 0x21, 0xD0, 0xBC, 0x5C, 0x99, 0xED, 0xD1, 0xB9, 0x23, 0xFC,
	   0x48, 0xDF, 0x32, 0x74, 0xFF, 0x1B, 0xCE, 0xF1, 0xBA, 0xE5, 0xD7, 0xF4, 0x1E, 0xBE, 0x59, 0xB5, 0x00, 0x00, 0x06, 0x11,
	   0xD3, 0x44, 0x9B, 0xB8, 0xB3, 0xBB, 0x0F, 0xC1, 0x64, 0x24, 0x62, 0x2E, 0x08, 0x47, 0xAB, 0x33,
	   0xA1, 0x72, 0xEC, 0x60, 0x40, 0x50, 0x56, 0xC8, 0x27, 0x41, 0xDA, 0xF7, 0xA6, 0xFC, 0xB2, 0x90, 0x00, 0x00, 0x07, 0x11,
	   0x0A, 0x6B, 0x51, 0x22, 0x4F, 0xDF, 0x09, 0x02, 0xB9, 0x07, 0xA5, 0xA4, 0xE2, 0x47, 0x38, 0x4F,
	   0x55, 0x32, 0x1D, 0x41, 0xCA, 0xEC, 0xB1, 0xF4, 0xE9, 0x0E, 0x38, 0x75, 0x56, 0x3A, 0x28, 0x3D, 0x00, 0x00, 0x08, 0x11,
	   0x45, 0x18, 0x18, 0xE8, 0xD2, 0x3D, 0xB8, 0xFF, 0xEC, 0xAF, 0x3E, 0xEA, 0x57, 0xFA, 0x13, 0xEE,
	   0xB5, 0x78, 0xA3, 0xE1, 0xF4, 0x29, 0x4D, 0x24, 0x4D, 0xC3, 0xFA, 0x35, 0xAF, 0xF7, 0xBA, 0x8D, 0x00, 0x00, 0x09, 0x11,
	   0x38, 0x12, 0xCC, 0xCC, 0x97, 0xFD, 0xD8, 0x6A, 0x42, 0xC7, 0x58, 0x2E, 0x00, 0xCA, 0xA9, 0xF0,
	   0xBE, 0x67, 0x24, 0x10, 0x61, 0x3F, 0x32, 0x5C, 0xD2, 0x13, 0xA9, 0x78, 0xDA, 0x26, 0x28, 0x6C, 0x00, 0x00, 0x0A, 0x11,
	   0x29, 0xEB, 0xC1, 0x29, 0xAF, 0x3F, 0x10, 0x9F, 0x68, 0x42, 0x43, 0x71, 0xCC, 0x88, 0x91, 0x6C,
	   0xD2, 0xD4, 0x3D, 0x55, 0x55, 0xB9, 0x08, 0x1A, 0xB5, 0xB7, 0xC0, 0x74, 0x1C, 0xE7, 0x60, 0x11, 0x00, 0x00, 0x0B, 0x11,
	   0x2F, 0x65, 0xE3, 0xAC, 0xB5, 0x7A, 0xDE, 0xC0, 0x2D, 0x2D, 0x42, 0x44, 0x05, 0xEA, 0xC5, 0x98,
	   0xD7, 0x49, 0x29, 0xA5, 0x50, 0xA3, 0x0E, 0x9A, 0xAE, 0xCB, 0xAD, 0x13, 0x10, 0xD5, 0x5A, 0x90, 0x00, 0x00, 0x0C, 0x11,
	   0x48, 0x47, 0x73, 0x96, 0xFB, 0x96, 0x8E, 0xB1, 0xC5, 0xEB, 0x46, 0x88, 0x52, 0xC9, 0xE0, 0x73,
	   0x9A, 0x6E, 0xE1, 0xD8, 0x4D, 0x60, 0x4C, 0xEF, 0xEF, 0x89, 0xBB, 0x6E, 0xEE, 0x2F, 0x36, 0x77, 0x00, 0x00, 0x0D, 0x11,
	   0xA2, 0x8B, 0xBA, 0x1E, 0x3B, 0xE5, 0x64, 0xD1, 0xF8, 0x49, 0xA6, 0x63, 0xF7, 0x4E, 0x6D, 0x94,
	   0x45, 0x62, 0xAB, 0x3C, 0x80, 0x01, 0x01, 0x54, 0x6B, 0x76, 0x26, 0x14, 0xA9, 0x8E, 0xBE, 0xA8, 0x00, 0x00, 0x0E, 0x11,
	   0x50, 0x24, 0x27, 0x43, 0xBF, 0xE4, 0x9F, 0x5A, 0xEC, 0x75, 0x2A, 0xCB, 0xFD, 0xE2, 0xC0, 0xA7,
	   0xF2, 0x35, 0x5C, 0xAF, 0x5D, 0x9E, 0xE9, 0xFD, 0xAA, 0x5C, 0xF4, 0x76, 0x42, 0x66, 0x0A, 0x90, 0x00, 0x00, 0x0F, 0x11};

const uint8_t auc_chirp_fcc[576]
	= {0x45, 0x7B, 0x65, 0x44, 0x16, 0xF9, 0xBC, 0xB0, 0xB9, 0xEF, 0xB2, 0xF8, 0x21, 0xF2, 0x33, 0x53,
	   0x41, 0x4C, 0x3C, 0x0B, 0xBE, 0x64, 0xD4, 0xA3, 0xB8, 0x3D, 0xF3, 0x84, 0x48, 0xA0, 0x26, 0xFE, 0x00, 0x00, 0x00, 0x11,
	   0xB2, 0xE5, 0xD1, 0x30, 0x2F, 0x5D, 0x73, 0xA1, 0x32, 0x22, 0xE7, 0x94, 0x90, 0xE4, 0xA8, 0xC3,
	   0x05, 0xF7, 0x55, 0xD6, 0x78, 0x0E, 0x0F, 0x2A, 0xCD, 0x55, 0x96, 0xEB, 0xAD, 0x84, 0x29, 0x30, 0x00, 0x00, 0x01, 0x11,
	   0xE5, 0x78, 0x65, 0xEC, 0x6F, 0x70, 0xEB, 0x6B, 0xA4, 0x76, 0xB4, 0x26, 0x1B, 0xA7, 0x50, 0xB8,
	   0x2F, 0x78, 0xE9, 0x27, 0xA2, 0xD9, 0xE0, 0x5D, 0x30, 0x44, 0x56, 0x88, 0x25, 0x80, 0xB5, 0x2C, 0x00, 0x00, 0x02, 0x11,
	   0x01, 0xCA, 0x52, 0x96, 0x3E, 0x96, 0xF2, 0xEB, 0xA0, 0x25, 0xEC, 0x1F, 0x38, 0xB9, 0x47, 0x81,
	   0xF6, 0xA0, 0x90, 0x34, 0xE0, 0xDE, 0x3A, 0x79, 0x5D, 0x0F, 0x28, 0x6F, 0xA8, 0x46, 0x99, 0x1F, 0x00, 0x00, 0x03, 0x11,
	   0x9B, 0xC1, 0xBF, 0x5C, 0x3B, 0xE5, 0x5D, 0x8B, 0xFD, 0xCC, 0x9F, 0xFE, 0xE5, 0x28, 0x3D, 0x61,
	   0x38, 0xAF, 0xE1, 0xA4, 0xA4, 0x8D, 0x01, 0xDF, 0x52, 0xF4, 0x48, 0x7B, 0xD2, 0x73, 0x80, 0x01, 0x00, 0x00, 0x04, 0x11,
	   0x2F, 0x8C, 0x60, 0x1D, 0xD5, 0x0F, 0xB2, 0x7F, 0x18, 0x8B, 0x5B, 0x81, 0x00, 0x43, 0x9F, 0xB1,
	   0xD6, 0xF2, 0x48, 0x24, 0x56, 0x2A, 0xCC, 0x10, 0x9B, 0xDF, 0x18, 0xEB, 0x65, 0x24, 0x16, 0x38, 0x00, 0x00, 0x05, 0x11,
	   0x4A, 0x09, 0x08, 0xFB, 0xAA, 0x04, 0xF8, 0x3E, 0x73, 0xF1, 0xFB, 0x18, 0x9B, 0x1B, 0xFF, 0x49,
	   0x52, 0x4B, 0x11, 0x08, 0x9C, 0x42, 0xE8, 0x4D, 0x60, 0x79, 0x37, 0x0F, 0xC1, 0xAE, 0x9D, 0x07, 0x00, 0x00, 0x06, 0x11,
	   0x41, 0x19, 0x2A, 0x12, 0xAA, 0x42, 0xE6, 0x4A, 0x60, 0x5E, 0x1F, 0x06, 0x9A, 0x1E, 0xF4, 0x5D,
	   0x5F, 0xD7, 0xF3, 0x62, 0xA5, 0xA8, 0xFC, 0xD6, 0x67, 0x4D, 0x14, 0xF5, 0x9C, 0x93, 0xF3, 0xF9, 0x00, 0x00, 0x07, 0x11,
	   0xA1, 0xA2, 0xF7, 0xBD, 0x56, 0xAB, 0xCB, 0xA4, 0xBB, 0x2F, 0x5F, 0xEF, 0x32, 0xB8, 0x99, 0x53,
	   0xEC, 0x04, 0x60, 0x75, 0xE5, 0x6E, 0xA4, 0x7B, 0x32, 0x35, 0x5A, 0x82, 0xCD, 0x16, 0xB3, 0x8B, 0x00, 0x00, 0x08, 0x11,
	   0x27, 0x41, 0x4B, 0x1D, 0xA4, 0x94, 0xED, 0x6F, 0x5E, 0xC3, 0xEB, 0xDB, 0xAF, 0xB0, 0x3E, 0x77,
	   0x2D, 0x9C, 0x99, 0x3F, 0x1B, 0x99, 0x59, 0xC9, 0xA8, 0x78, 0xCD, 0x47, 0x62, 0xD1, 0x23, 0x94, 0x00, 0x00, 0x09, 0x11,
	   0x32, 0x11, 0x22, 0xCE, 0xA4, 0x59, 0x22, 0x63, 0x45, 0xC5, 0xAE, 0x17, 0x05, 0x1C, 0x53, 0x63,
	   0xB9, 0x83, 0xDA, 0x57, 0x50, 0xDE, 0xEA, 0xFC, 0xC1, 0x04, 0x4B, 0x3E, 0x1B, 0x5E, 0x97, 0xD0, 0x00, 0x00, 0x0A, 0x11,
	   0xAB, 0x13, 0x6B, 0xD4, 0xD4, 0x15, 0xBA, 0x88, 0x70, 0xC9, 0xE5, 0x36, 0xC0, 0x6F, 0x56, 0x15,
	   0xE4, 0x45, 0xB6, 0x5E, 0x5F, 0xB1, 0xF8, 0xB0, 0x9C, 0x8B, 0x5A, 0xE6, 0x20, 0x69, 0x9F, 0xF6, 0x00, 0x00, 0x0B, 0x11,
	   0x54, 0x0D, 0x94, 0xC2, 0x24, 0x01, 0x33, 0x10, 0x98, 0x89, 0x49, 0x18, 0x1E, 0x7D, 0xAD, 0xFE,
	   0x48, 0x99, 0xFB, 0x3F, 0xA0, 0xAB, 0x52, 0x1D, 0x07, 0x3B, 0xB7, 0x1F, 0x67, 0x0C, 0xE3, 0x36, 0x00, 0x00, 0x0C, 0x11,
	   0xC6, 0xFF, 0xDB, 0x66, 0x42, 0x22, 0xC4, 0x81, 0x0F, 0xB4, 0x27, 0xFE, 0xCB, 0x2C, 0x2A, 0xE6,
	   0x0D, 0x9E, 0xBD, 0x63, 0x3A, 0xE6, 0xE0, 0x12, 0xD1, 0xB7, 0x6F, 0x02, 0xC0, 0x29, 0xF1, 0xC6, 0x00, 0x00, 0x0D, 0x11,
	   0x00, 0xC1, 0x99, 0x6D, 0x5F, 0xC7, 0x8F, 0x79, 0x2B, 0x08, 0x4A, 0xF0, 0x9A, 0x89, 0x7C, 0x0B,
	   0xC2, 0x43, 0xCA, 0xF4, 0x4B, 0x5A, 0x9D, 0xBD, 0x54, 0x6B, 0x08, 0x94, 0xB3, 0x8F, 0x6A, 0xED, 0x00, 0x00, 0x0E, 0x11,
	   0x4F, 0x9A, 0xAF, 0x0D, 0xCC, 0x0A, 0x1A, 0x29, 0xB4, 0x3B, 0x51, 0x12, 0xFD, 0x34, 0x01, 0x60,
	   0x50, 0xB4, 0x9B, 0xFC, 0x22, 0x7C, 0xD0, 0xD7, 0xD6, 0xCD, 0x71, 0x5A, 0xB3, 0x32, 0x55, 0xB0, 0x00, 0x00, 0x0F, 0x11};

const uint8_t auc_chirp_arib[576]
	= {0x55, 0x90, 0x5C, 0x25, 0x06, 0x18, 0xA5, 0xEB, 0xC8, 0x40, 0xDE, 0xAE, 0xE7, 0x28, 0x52, 0x65,
	   0x5E, 0x6B, 0xFB, 0x02, 0xE9, 0xBC, 0xC7, 0x10, 0xA8, 0x71, 0x15, 0x5F, 0x3E, 0x1A, 0x19, 0xC5, 0x00, 0x00, 0x00, 0x11,
	   0xCC, 0x2A, 0x06, 0xCD, 0x61, 0x37, 0x67, 0xFC, 0xFC, 0xF6, 0xBE, 0x65, 0xA8, 0xBC, 0xAF, 0xA4,
	   0x21, 0x39, 0x76, 0x42, 0x4C, 0xE2, 0x11, 0x5E, 0xC0, 0x83, 0x80, 0x00, 0xCB, 0x7D, 0x32, 0xB9, 0x00, 0x00, 0x01, 0x11,
	   0x49, 0xE1, 0x2F, 0xAF, 0xD0, 0xE8, 0xB1, 0x93, 0xF0, 0x56, 0x2D, 0x32, 0x44, 0xB7, 0x17, 0x84,
	   0xC3, 0x26, 0xB8, 0xE7, 0xE9, 0xCA, 0x24, 0xDD, 0x64, 0xAE, 0x3C, 0xB6, 0xBC, 0xDA, 0x9C, 0x21, 0x00, 0x00, 0x02, 0x11,
	   0xEB, 0xF0, 0x9E, 0x63, 0xC5, 0xAD, 0x1C, 0x39, 0x6B, 0x43, 0x4F, 0x63, 0xBA, 0xB1, 0x83, 0xCA,
	   0xE3, 0xB7, 0x36, 0x00, 0x54, 0x80, 0x3A, 0x21, 0xD2, 0xAF, 0xA0, 0x9C, 0xDC, 0xC6, 0x22, 0x84, 0x00, 0x00, 0x03, 0x11,
	   0xE2, 0x6C, 0xA3, 0xF5, 0xE1, 0x6C, 0x4F, 0x91, 0x51, 0x6B, 0xFE, 0x06, 0xB2, 0x97, 0xA7, 0xA0,
	   0x11, 0x21, 0x75, 0xE7, 0x32, 0x13, 0xBA, 0x25, 0xAD, 0xA1, 0xF0, 0xCB, 0x47, 0xDA, 0x55, 0xCF, 0x00, 0x00, 0x04, 0x11,
	   0x34, 0x5E, 0xD5, 0x8A, 0xB0, 0x0E, 0xF4, 0x81, 0x5A, 0x64, 0x46, 0x44, 0xCF, 0x7B, 0xA0, 0xA9,
	   0xE4, 0x4D, 0x45, 0xDB, 0x52, 0x6F, 0xE9, 0x1E, 0xA4, 0xDB, 0xED, 0xD8, 0x46, 0x27, 0x3A, 0x1B, 0x00, 0x00, 0x05, 0x11,
	   0xE4, 0xD0, 0x48, 0x52, 0x3A, 0xDA, 0xCF, 0xDF, 0xB2, 0x10, 0x1F, 0x6A, 0x5C, 0x5E, 0x07, 0xE4,
	   0xAA, 0x6D, 0xBA, 0x74, 0x2D, 0x7A, 0x6F, 0xF2, 0x03, 0x2B, 0x91, 0xE7, 0xDF, 0x9D, 0x51, 0x49, 0x00, 0x00, 0x06, 0x11,
	   0x3D, 0xD5, 0xC8, 0x7C, 0xAA, 0xE6, 0x21, 0xBD, 0x5F, 0xC0, 0x01, 0x5A, 0xAE, 0x31, 0xE0, 0x18,
	   0x42, 0x8C, 0x3A, 0x06, 0xBE, 0x89, 0xA6, 0x0A, 0x38, 0xB2, 0x71, 0xA2, 0xEC, 0x1B, 0x97, 0x61, 0x00, 0x00, 0x07, 0x11,
	   0xDE, 0xFA, 0x9B, 0x99, 0x15, 0x73, 0x56, 0x85, 0xF6, 0x76, 0xB5, 0x4A, 0x02, 0xD4, 0x55, 0xC0,
	   0x0C, 0x26, 0x99, 0x3F, 0xDB, 0xF4, 0x61, 0xBE, 0x32, 0x19, 0xB3, 0x8C, 0xCC, 0x60, 0x3E, 0xFB, 0x00, 0x00, 0x08, 0x11,
	   0xE1, 0x07, 0x9C, 0xF8, 0x2A, 0xFA, 0x69, 0x00, 0xCB, 0x48, 0x9A, 0x76, 0x38, 0x44, 0x60, 0x8E,
	   0xD5, 0x26, 0xA6, 0xCD, 0x16, 0x1F, 0x50, 0x10, 0xEE, 0xA1, 0xAE, 0xC0, 0x1C, 0xD8, 0x5E, 0xDA, 0x00, 0x00, 0x09, 0x11,
	   0x44, 0x1C, 0xD2, 0xEF, 0xBF, 0xF7, 0x3A, 0x26, 0x3B, 0x54, 0xB6, 0x53, 0xCF, 0x7E, 0x5F, 0xE0,
	   0x28, 0x4B, 0x94, 0xEA, 0xDE, 0x15, 0x60, 0x57, 0x11, 0x14, 0xAE, 0x00, 0x09, 0x83, 0x55, 0x2C, 0x00, 0x00, 0x0A, 0x11,
	   0xBB, 0xBB, 0x44, 0xBA, 0x25, 0xA7, 0xA8, 0x82, 0xF9, 0x39, 0x61, 0x26, 0xE7, 0x6C, 0xAB, 0x9A,
	   0x3F, 0x53, 0x39, 0x6E, 0x9F, 0x49, 0xE0, 0xAE, 0x66, 0x75, 0x05, 0xCA, 0xAB, 0xBF, 0x16, 0x7A, 0x00, 0x00, 0x0B, 0x11,
	   0x46, 0x7E, 0xFB, 0x9B, 0xA0, 0xFC, 0x29, 0x09, 0x46, 0xEB, 0xB7, 0x53, 0xFB, 0xA2, 0x62, 0xC6,
	   0xC8, 0x0E, 0xA2, 0xAB, 0x53, 0x2E, 0x2E, 0x14, 0xA6, 0x66, 0x0B, 0xDB, 0x56, 0x5C, 0xCF, 0x68, 0x00, 0x00, 0x0C, 0x11,
	   0x32, 0x67, 0x82, 0x67, 0x02, 0xAC, 0x4C, 0xF8, 0xC3, 0x77, 0xFB, 0xD0, 0x58, 0x84, 0xD9, 0x2B,
	   0xC0, 0xED, 0x3E, 0x30, 0x0D, 0xFD, 0xAD, 0xDB, 0x11, 0xB6, 0x4F, 0xF5, 0xD9, 0xEC, 0xD4, 0xCC, 0x00, 0x00, 0x0D, 0x11,
	   0x57, 0xA0, 0x9A, 0xC1, 0xD0, 0x92, 0x60, 0x79, 0xC5, 0x41, 0xE0, 0x6D, 0x7C, 0x11, 0xD2, 0x4E,
	   0xB4, 0x12, 0x6A, 0x41, 0xE2, 0xAF, 0x93, 0x95, 0x5C, 0x59, 0x1A, 0xA5, 0xA7, 0xA4, 0x53, 0x8F, 0x00, 0x00, 0x0E, 0x11,
	   0x41, 0x38, 0xF0, 0xF4, 0x8E, 0x0C, 0x09, 0xC6, 0x1D, 0x8D, 0xC4, 0x96, 0x43, 0x21, 0x52, 0x62,
	   0xAF, 0x37, 0xFD, 0x03, 0x29, 0x30, 0x9D, 0x06, 0x03, 0x58, 0x5C, 0xAE, 0xCA, 0xC1, 0x0E, 0xDA, 0x00, 0x00, 0x0F, 0x11};

const uint8_t *puc_chirp;
#endif

const uint16_t rrc_coef1_init_cenelec_a[64] = {0xFFC4, 0x0002, 0xFF80, 0xFEE6, 0xFFC0, 0x01D0, 0x023C, 0x0076,
					       0x0022, 0x0242, 0x01EC, 0xFD58, 0xFB18, 0xFE6A, 0xFF90, 0xFA64,
					       0xF934, 0x0260, 0x08FE, 0x0350, 0x0042, 0x0C46, 0x1384, 0x028A,
					       0xF0E4, 0xFB12, 0x0488, 0xE106, 0xB478, 0xD612, 0x422E, 0x7FFF,
					       0x422E, 0xD612, 0xB478, 0xE106, 0x0488, 0xFB12, 0xF0E4, 0x028A,
					       0x1384, 0x0C46, 0x0042, 0x0350, 0x08FE, 0x0260, 0xF934, 0xFA64,
					       0xFF90, 0xFE6A, 0xFB18, 0xFD58, 0x01EC, 0x0242, 0x0022, 0x0076,
					       0x023C, 0x01D0, 0xFFC0, 0xFEE6, 0xFF80, 0x0002, 0xFFC4, 0x0000};

const uint16_t rrc_coef2_init_cenelec_a[64] = {0x00DD, 0xFFC5, 0xFE66, 0xFC25, 0xF986, 0xF795, 0xF7C2, 0xFB7F,
					       0x03C4, 0x109C, 0x20DC, 0x323C, 0x41C7, 0x4C8D, 0x5068, 0x4C8D,
					       0x41C7, 0x323C, 0x20DC, 0x109C, 0x03C4, 0xFB7F, 0xF7C2, 0xF795,
					       0xF986, 0xFC25, 0xFE66, 0xFFC5, 0x00DD, 0x0000, 0x0000, 0x0000,
					       0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					       0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					       0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					       0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_dec_coef1_init_cenelec_a[64] = {0xFFC4, 0x0002, 0xFF80, 0xFEE6, 0xFFC0, 0x01D0, 0x023C, 0x0076,
						   0x0022, 0x0242, 0x01EC, 0xFD58, 0xFB18, 0xFE6A, 0xFF90, 0xFA64,
						   0xF934, 0x0260, 0x08FE, 0x0350, 0x0042, 0x0C46, 0x1384, 0x028A,
						   0xF0E4, 0xFB12, 0x0488, 0xE106, 0xB478, 0xD612, 0x422E, 0x7FFF,
						   0x422E, 0xD612, 0xB478, 0xE106, 0x0488, 0xFB12, 0xF0E4, 0x028A,
						   0x1384, 0x0C46, 0x0042, 0x0350, 0x08FE, 0x0260, 0xF934, 0xFA64,
						   0xFF90, 0xFE6A, 0xFB18, 0xFD58, 0x01EC, 0x0242, 0x0022, 0x0076,
						   0x023C, 0x01D0, 0xFFC0, 0xFEE6, 0xFF80, 0x0002, 0xFFC4, 0x0000};

const uint16_t rrc_dec_coef2_init_cenelec_a[64] = {0xFFC5, 0xFE66, 0xFC25, 0xF986, 0xF795, 0xF7C2, 0xFB7F, 0x03C4,
						   0x109C, 0x20DC, 0x323C, 0x41C7, 0x4C8D, 0x5068, 0x4C8D, 0x41C7,
						   0x323C, 0x20DC, 0x109C, 0x03C4, 0xFB7F, 0xF7C2, 0xF795, 0xF986,
						   0xFC25, 0xFE66, 0xFFC5, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const int16_t ass_h_in_cenelec_a[32]
	= {(int16_t)0x7A03, (int16_t)0xFCAD, (int16_t)0x0248, (int16_t)0x05D3, (int16_t)0x042C, (int16_t)0xFEDD, (int16_t)0xFAA8, (int16_t)0xFB3F,
	   (int16_t)0xFFFE, (int16_t)0x049A, (int16_t)0x0506, (int16_t)0x010C, (int16_t)0xFC55, (int16_t)0xFB09, (int16_t)0xFE1B, (int16_t)0x02A3,
	   (int16_t)0x049A, (int16_t)0x0279, (int16_t)0xFE64, (int16_t)0xFC06, (int16_t)0xFD42, (int16_t)0x00B2, (int16_t)0x032C, (int16_t)0x02B0,
	   (int16_t)0x0005, (int16_t)0xFDB8, (int16_t)0xFDAA, (int16_t)0xFF89, (int16_t)0x016A, (int16_t)0x01BC, (int16_t)0x0099, (int16_t)0xFF55};

const int16_t ss_cos_om_m_cenelec_a = 18205;         /* round(2^15*cos(2*pi*fm/fs)); (fm/fs)=40/256 */
const int16_t ass_cos_om_0_cenelec_a[100]
	= {32767, 32758, 32729, 32679, 32610, 32522, 32413, 32286, 32138, 31972, 31786, 31581, 31357, 31114, 30853, 30572, 30274, 29957, 29622, 29269,
	   28899, 28511, 28106, 27684, 27246, 26791, 26320, 25833, 25330, 24812, 24279, 23732, 23170, 22595, 22006, 21403, 20788, 20160, 19520,
	   18868, 18205, 17531, 16846, 16151, 15447, 14733, 14010, 13279, 12540, 11793, 11039, 10279, 9512, 8740, 7962, 7180, 6393, 5602, 4808,
	   4011, 3212, 2411, 1608, 804, 0, -804, -1608, -2411, -3212, -4011, -4808, -5602, -6393, -7180, -7962, -8740, -9512, -10279, -11039,
	   -11793, -12540, -13279, -14010, -14733, -15447, -16151, -16846, -17531, -18205, -18868, -19520, -20160, -20788, -21403, -22006, -22595,
	   -23170, -23732, -24279, -24812};
/* round(2^15*cos(2*pi*f_0/fs)); (f_0/fs)=carrier_f_0/256 */

const uint16_t rrc_coef0_init_fcc[64] = {0xFFE8, 0x0095, 0x004B, 0x0036, 0x003B, 0xFF3C, 0xFFDB, 0xFEC8,
					 0xFFD2, 0x003F, 0x003D, 0x0276, 0xFFE9, 0x0221, 0xFE6F, 0xFE67,
					 0xFE5C, 0xFB74, 0x0256, 0xFDDE, 0x0709, 0x03C8, 0x033E, 0x05F0,
					 0xF1F4, 0x00E7, 0xDBB3, 0xFA5D, 0x5162, 0xFA5D, 0xDBB3, 0x00E7,
					 0xF1F4, 0x05F0, 0x033E, 0x03C8, 0x0709, 0xFDDE, 0x0256, 0xFB74,
					 0xFE5C, 0xFE67, 0xFE6F, 0x0221, 0xFFE9, 0x0276, 0x003D, 0x003F,
					 0xFFD2, 0xFEC8, 0xFFDB, 0xFF3C, 0x003B, 0x0036, 0x004B, 0x0095,
					 0xFFE8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_coef1_init_fcc[64] = {0x001B, 0x0009, 0xFFF3, 0xFFF7, 0x0017, 0x0005, 0xFFD9, 0x0002,
					 0x0042, 0xFFF1, 0xFF96, 0x0021, 0x00A5, 0xFFC7, 0xFF04, 0x0056,
					 0x0177, 0xFF88, 0xFDD8, 0x009B, 0x0328, 0xFF42, 0xFB51, 0x00DD,
					 0x074B, 0xFF0B, 0xF2F3, 0x0105, 0x2890, 0x3EF5, 0x2890, 0x0105,
					 0xF2F3, 0xFF0B, 0x074B, 0x00DD, 0xFB51, 0xFF42, 0x0328, 0x009B,
					 0xFDD8, 0xFF88, 0x0177, 0x0056, 0xFF04, 0xFFC7, 0x00A5, 0x0021,
					 0xFF96, 0xFFF1, 0x0042, 0x0002, 0xFFD9, 0x0005, 0x0017, 0xFFF7,
					 0xFFF3, 0x0009, 0x001B, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_coef2_init_fcc[64] = {0x000B, 0x001A, 0x0000, 0xFFA1, 0xFF61, 0x0000, 0x0187, 0x0245,
					 0x0000, 0xFB42, 0xF925, 0x0000, 0x1094, 0x22BE, 0x2AAB, 0x22BE,
					 0x1094, 0x0000, 0xF925, 0xFB42, 0x0000, 0x0245, 0x0187, 0x0000,
					 0xFF61, 0xFFA1, 0x0000, 0x001A, 0x000B, 0x0000, 0x0000, 0x0000,
					 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_dec_coef2_init_fcc[64] = {0x0004, 0x0000, 0xFFE3, 0x0000, 0x0083, 0x0000, 0xFE5B, 0x0000,
					     0x045A, 0x0000, 0xF51F, 0x0000, 0x27C3, 0x4000, 0x27C3, 0x0000,
					     0xF51F, 0x0000, 0x045A, 0x0000, 0xFE5B, 0x0000, 0x0083, 0x0000,
					     0xFFE3, 0x0000, 0x0004, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_dec_coef1_init_fcc[64] = {0x0009, 0xFFF3, 0xFFF7, 0x0017, 0x0005, 0xFFD9, 0x0002, 0x0042,
					     0xFFF1, 0xFF96, 0x0021, 0x00A5, 0xFFC7, 0xFF04, 0x0056, 0x0177,
					     0xFF88, 0xFDD8, 0x009B, 0x0328, 0xFF42, 0xFB51, 0x00DD, 0x074B,
					     0xFF0B, 0xF2F3, 0x0105, 0x2890, 0x3EF5, 0x2890, 0x0105, 0xF2F3,
					     0xFF0B, 0x074B, 0x00DD, 0xFB51, 0xFF42, 0x0328, 0x009B, 0xFDD8,
					     0xFF88, 0x0177, 0x0056, 0xFF04, 0xFFC7, 0x00A5, 0x0021, 0xFF96,
					     0xFFF1, 0x0042, 0x0002, 0xFFD9, 0x0005, 0x0017, 0xFFF7, 0xFFF3,
					     0x0009, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_dec_coef0_init_fcc[64] = {0xFF5E, 0x001E, 0x0036, 0x008D, 0x0128, 0xFEFF, 0x0030, 0xFEF2,
					     0xFF12, 0xFFE1, 0xFF78, 0x029F, 0xFFF7, 0x02A7, 0xFFA6, 0xFE5E,
					     0xFF01, 0xFA8C, 0x01B7, 0xFD12, 0x0615, 0x0489, 0x032A, 0x0789,
					     0xF288, 0x0148, 0xDBDA, 0xF8DC, 0x5120, 0xF8DC, 0xDBDA, 0x0148,
					     0xF288, 0x0789, 0x032A, 0x0489, 0x0615, 0xFD12, 0x01B7, 0xFA8C,
					     0xFF01, 0xFE5E, 0xFFA6, 0x02A7, 0xFFF7, 0x029F, 0xFF78, 0xFFE1,
					     0xFF12, 0xFEF2, 0x0030, 0xFEFF, 0x0128, 0x008D, 0x0036, 0x001E,
					     0xFF5E, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_coef0_init_arib[64] = {0x000E, 0xFFE0, 0xFF31, 0x0032, 0x01E6, 0x003F, 0xFEE7, 0x0011,
					  0xFEA7, 0xFE94, 0x015E, 0x0038, 0x00FD, 0x035A, 0xFF17, 0xFEBE,
					  0x0000, 0xFA6A, 0xFE5A, 0x031D, 0xFEA2, 0x075F, 0x08CE, 0xFAA6,
					  0x0290, 0xF800, 0xD905, 0x0739, 0x3CF8, 0x0739, 0xD905, 0xF800,
					  0x0290, 0xFAA6, 0x08CE, 0x075F, 0xFEA2, 0x031D, 0xFE5A, 0xFA6A,
					  0x0000, 0xFEBE, 0xFF17, 0x035A, 0x00FD, 0x0038, 0x015E, 0xFE94,
					  0xFEA7, 0x0011, 0xFEE7, 0x003F, 0x01E6, 0x0032, 0xFF31, 0xFFE0,
					  0x000E, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_coef1_init_arib[64] = {0x001B, 0x0009, 0xFFF3, 0xFFF7, 0x0017, 0x0005, 0xFFD9, 0x0002,
					  0x0042, 0xFFF1, 0xFF96, 0x0021, 0x00A5, 0xFFC7, 0xFF04, 0x0056,
					  0x0177, 0xFF88, 0xFDD8, 0x009B, 0x0328, 0xFF42, 0xFB51, 0x00DD,
					  0x074B, 0xFF0B, 0xF2F3, 0x0105, 0x2890, 0x3EF5, 0x2890, 0x0105,
					  0xF2F3, 0xFF0B, 0x074B, 0x00DD, 0xFB51, 0xFF42, 0x0328, 0x009B,
					  0xFDD8, 0xFF88, 0x0177, 0x0056, 0xFF04, 0xFFC7, 0x00A5, 0x0021,
					  0xFF96, 0xFFF1, 0x0042, 0x0002, 0xFFD9, 0x0005, 0x0017, 0xFFF7,
					  0xFFF3, 0x0009, 0x001B, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_coef2_init_arib[64] = {0x000B, 0x001A, 0x0000, 0xFFA1, 0xFF61, 0x0000, 0x0187, 0x0245,
					  0x0000, 0xFB42, 0xF925, 0x0000, 0x1094, 0x22BE, 0x2AAB, 0x22BE,
					  0x1094, 0x0000, 0xF925, 0xFB42, 0x0000, 0x0245, 0x0187, 0x0000,
					  0xFF61, 0xFFA1, 0x0000, 0x001A, 0x000B, 0x0000, 0x0000, 0x0000,
					  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_dec_coef2_init_arib[64] = {0x0004, 0x0000, 0xFFE3, 0x0000, 0x0083, 0x0000, 0xFE5B, 0x0000,
					      0x045A, 0x0000, 0xF51F, 0x0000, 0x27C3, 0x4000, 0x27C3, 0x0000,
					      0xF51F, 0x0000, 0x045A, 0x0000, 0xFE5B, 0x0000, 0x0083, 0x0000,
					      0xFFE3, 0x0000, 0x0004, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
					      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_dec_coef1_init_arib[64] = {0x0009, 0xFFF3, 0xFFF7, 0x0017, 0x0005, 0xFFD9, 0x0002, 0x0042,
					      0xFFF1, 0xFF96, 0x0021, 0x00A5, 0xFFC7, 0xFF04, 0x0056, 0x0177,
					      0xFF88, 0xFDD8, 0x009B, 0x0328, 0xFF42, 0xFB51, 0x00DD, 0x074B,
					      0xFF0B, 0xF2F3, 0x0105, 0x2890, 0x3EF5, 0x2890, 0x0105, 0xF2F3,
					      0xFF0B, 0x074B, 0x00DD, 0xFB51, 0xFF42, 0x0328, 0x009B, 0xFDD8,
					      0xFF88, 0x0177, 0x0056, 0xFF04, 0xFFC7, 0x00A5, 0x0021, 0xFF96,
					      0xFFF1, 0x0042, 0x0002, 0xFFD9, 0x0005, 0x0017, 0xFFF7, 0xFFF3,
					      0x0009, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t rrc_dec_coef0_init_arib[64] = {0xFF5E, 0x001E, 0x0036, 0x008D, 0x0128, 0xFEFF, 0x0030, 0xFEF2,
					      0xFF12, 0xFFE1, 0xFF78, 0x029F, 0xFFF7, 0x02A7, 0xFFA6, 0xFE5E,
					      0xFF01, 0xFA8C, 0x01B7, 0xFD12, 0x0615, 0x0489, 0x032A, 0x0789,
					      0xF288, 0x0148, 0xDBDA, 0xF8DC, 0x5120, 0xF8DC, 0xDBDA, 0x0148,
					      0xF288, 0x0789, 0x032A, 0x0489, 0x0615, 0xFD12, 0x01B7, 0xFA8C,
					      0xFF01, 0xFE5E, 0xFFA6, 0x02A7, 0xFFF7, 0x029F, 0xFF78, 0xFFE1,
					      0xFF12, 0xFEF2, 0x0030, 0xFEFF, 0x0128, 0x008D, 0x0036, 0x001E,
					      0xFF5E, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

#if (MOSFET_POWER_AMPLIFIER == FDC6420C)
/* FDC6420C configuration */
const struct impedance_params_type impedance_params_cenelec_a[NUM_IMPEDANCE_STATES] = {
	{0x00002120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0x50},
	{0x00002120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0x5F},
	{0x00002120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0x5F}
};
#else
/* NTGD4167C configuration */
const struct impedance_params_type impedance_params_cenelec_a[NUM_IMPEDANCE_STATES] = {
	{0x00002110, 0x00003F3F, 0x00003F0F, 0x0F, 0x03, 0x55, 0x50},
	{0x00002110, 0x00003F3F, 0x00003F0F, 0x0F, 0x03, 0x55, 0x5F},
	{0x00002110, 0x00003F3F, 0x00003F0F, 0x0F, 0x03, 0x55, 0x5F}
};
#endif

#ifdef CONF_ENABLE_C11_CFG
const struct impedance_params_type impedance_params_fcc[NUM_IMPEDANCE_STATES] = {
	{0x10102120, 0x3F3F0000, 0x3F3F0000, 0xF0, 0x03, 0xAA, 0x00},
	{0x10102120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0xFF},
	{0x21202120, 0x1F3F1F3F, 0x1F1F1F1F, 0xF0, 0x03, 0xAA, 0xFF}
};
#else
const struct impedance_params_type impedance_params_fcc[NUM_IMPEDANCE_STATES] = {
	{0x10102120, 0x3F3F0000, 0x3F3F0000, 0xF0, 0x03, 0xAA, 0x00},
	{0x10102120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0xFF},
	{0x10102120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0xFF}
};
#endif

const struct impedance_params_type impedance_params_arib[NUM_IMPEDANCE_STATES] = {
	{0x00002120, 0x3F3F0000, 0x3F3F0000, 0xF0, 0x01, 0xAA, 0x00},
	{0x10102120, 0x00001F3F, 0x00001F1F, 0x0F, 0x01, 0x55, 0xFF},
	{0x10102120, 0x00001F3F, 0x00001F1F, 0x0F, 0x01, 0x55, 0xFF}
};

const struct impedance_params_type *p_impedance_params;

const uint8_t auc_rep[8] = {0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00};

static uint8_t auc_th_cenelec_a[12] = {0x10, 0x00, 0x70, 0x00, 0x00, 0x00, 0x57, 0xFF, 0x10, 0x01, 0x10, 0x20};
static uint8_t auc_th_fcc_arib[12] = {0x10, 0x00, 0x57, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0x10, 0x01, 0x10, 0x20};
static uint8_t *puc_th;

#define L1_LOAD_RRC_INT         0x00
#define L1_LOAD_RRC_DEC         0x00
#define L1_LOAD_RRC_INT_1       0x01
#define L1_LOAD_RRC_DEC_1       0x02
#define L1_LOAD_RRC_INT_2       0x04
#define L1_LOAD_RRC_DEC_2       0x08

#define L0_LOAD_RRC_INT         0x04
#define L0_LOAD_RRC_DEC         0x08
#define L0_LOAD_RRC_INT_1       0x00
#define L0_LOAD_RRC_DEC_1       0x00
#define L0_LOAD_RRC_INT_2       0x00
#define L0_LOAD_RRC_DEC_2       0x00

/* Num symbols for demodulation block */
uint8_t uc_num_sym_block_demod_fch;

/* RRC filter calculation internal variables */
static int32_t sl_lambda1;
static int32_t sl_lambda2;
static int32_t sl_lambdac1;
static int32_t sl_lambdac2;

/* Constant to represent impedance states */
const uint8_t imp_state_to_string[3][3] = {"HI ", "LO ", "VLO"};

#ifdef ENABLE_DYNAMIC_CHIRP_CONFIGURATION
/* Variable to read ATPL250 interrupt register */
static uint32_t ul_flags_isr;

/* Aux array to write chirp to RAM */
#define CHIRP_ROW_SIZE           36
#define FFT_BLOCK_SIZE_PER_ROW   32
#define CHIRP_NUM_ROWS           16
static uint8_t auc_chirp_from_fft[CHIRP_ROW_SIZE];

/* Dynamic correlation threshold (depending on static notching) */
#define CORRELATION_THRESHOLD_PERCENTAGE   0x7300 /* 0.45 in uQ0.16 */
static uint32_t ul_max_correlation_value;
/* const uint8_t auc_th_per_notched_carriers[72][2] */
/*	= {{0x00, 0x00}, {0x4F, 0x59}, {0x4E, 0xB7}, {0x4E, 0x14}, {0x4D, 0x72}, {0x4C, 0xD0}, {0x4C, 0x2E}, {0x4B, 0x8C}, */
/*	   {0x4A, 0xE9}, {0x4A, 0x47}, {0x49, 0xA5}, {0x49, 0x03}, {0x48, 0x61}, {0x47, 0xBF}, {0x47, 0x1C}, {0x46, 0x7A}, */
/*	   {0x45, 0xD8}, {0x45, 0x36}, {0x44, 0x94}, {0x43, 0xF1}, {0x43, 0x4F}, {0x42, 0xAD}, {0x42, 0x0B}, {0x41, 0x69}, */
/*	   {0x40, 0xC6}, {0x40, 0x24}, {0x3F, 0x82}, {0x3E, 0xE0}, {0x3E, 0x3E}, {0x3D, 0x9B}, {0x3C, 0xF9}, {0x3C, 0x57}, */
/*	   {0x3B, 0xB5}, {0x3B, 0x13}, {0x3A, 0x70}, {0x39, 0xCE}, {0x39, 0x2C}, {0x38, 0x8A}, {0x37, 0xE8}, {0x37, 0x45}, */
/*	   {0x36, 0xA3}, {0x36, 0x01}, {0x35, 0x5F}, {0x34, 0xBD}, {0x34, 0x1A}, {0x33, 0x78}, {0x32, 0xD6}, {0x32, 0x34}, */
/*	   {0x31, 0x92}, {0x30, 0xF0}, {0x30, 0x4D}, {0x2F, 0xAB}, {0x2F, 0x09}, {0x2E, 0x67}, {0x2D, 0xC5}, {0x2D, 0x22}, */
/*	   {0x2C, 0x80}, {0x2B, 0xDE}, {0x2B, 0x3C}, {0x2A, 0x9A}, {0x29, 0xF7}, {0x29, 0x55}, {0x28, 0xB3}, {0x28, 0x11}, */
/*	   {0x27, 0x6F}, {0x26, 0xCC}, {0x26, 0x2A}, {0x25, 0x88}, {0x24, 0xE6}, {0x24, 0x44}, {0x23, 0xA1}, {0x22, 0xFF}}; */

/* Assembly function to scale correlation reference. */
uint32_t SYNC_SCALE_XCORR_REF(int16_t *pss_ifft_out, int16_t *pss_xcorr_ref, uint16_t us_num_samples);
#endif

/**
 * \brief Update value of uc_num_sym_block_demod_fch
 *
 * \param uc_legacy_mode   legacy mode indicator
 *
 */
static void  _update_num_sym_block_fch(void)
{
	if (uc_working_band == WB_FCC) {
		if (uc_legacy_mode) {
			uc_num_sym_block_demod_fch = 8;
		}
	} else {
		uc_num_sym_block_demod_fch = 7;
	}
}

/**
 * \brief Fill the buffer of a RRC filters before write it to the hardware
 *
 * \param pus_rrc_coefs pointer to buffer containing the 64 uint16_t coefficient
 * \param uc_l1 value of L1
 * \param uc_l0 value of L0
 *
 */
static void atpl250_fill_rrc_buffer(uint16_t *pus_rrc_coefs, uint8_t uc_l1, uint8_t uc_l0)
{
	uint8_t uc_i;

	for (uc_i = 0; uc_i < 0x40; uc_i++) {
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8] = 0x00;
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8 + 1] = 0x00;
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8 + 2] = (uint8_t)(pus_rrc_coefs[uc_i] >> 8);
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8 + 3] = (uint8_t)(pus_rrc_coefs[uc_i] & 0xFF);
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8 + 4] = 0x00;
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8 + 5] = uc_l1;
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8 + 6] = uc_i;
		u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_i * 8 + 7] = (uc_l0 << 4) | 0x01;
	}
}

static inline void initialize_predistorsion(void)
{
	uint16_t us_i;
	uint8_t uc_j, uc_notched_carrier;

	for (us_i = 0; us_i < s_band_constants.uc_num_carriers * 4; us_i = us_i + 4) {
		auc_predistortion[us_i + 0] = 0x7F;
		auc_predistortion[us_i + 1] = 0xFF;
		auc_predistortion[us_i + 2] = 0x00;
		auc_predistortion[us_i + 3] = 0x00;
	}

	/* Set 0s in notched carriers */
	for (uc_j = 0; uc_j < uc_notched_carriers; uc_j++) {
		uc_notched_carrier = (auc_masked_carrier_list[uc_j] << 2);
		auc_predistortion[uc_notched_carrier + 0] = 0x00;
		auc_predistortion[uc_notched_carrier + 1] = 0x00;
		auc_predistortion[uc_notched_carrier + 2] = 0x00;
		auc_predistortion[uc_notched_carrier + 3] = 0x00;
	}

	/* Write to Zone1 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 0) + s_band_constants.uc_last_carrier); /* Avoid overflow to 1st symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((0 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 1) + s_band_constants.uc_last_carrier); /* Avoid overflow to 2nd symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((1 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_last_carrier); /* Avoid overflow to 3rd symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((2 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_last_carrier); /* Avoid overflow to 4th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((3 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_last_carrier); /* Avoid overflow to 5th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((4 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_last_carrier); /* Avoid overflow to 6th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((5 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_last_carrier); /* Avoid overflow to 7th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((6 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((7 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * 4);
}

/**
 * \brief Configure emition branch depending on impedance detected
 *
 */
void atpl250_update_branch_cfg(uint8_t uc_imp_state, uint8_t uc_new_emit_gain)
{
	LOG_PHY(("ImpState %.3s EmitGain 0x%02X\r\n", &imp_state_to_string[uc_imp_state][0], uc_new_emit_gain));
	pplc_if_write8(REG_ATPL250_TX_CONF_VL8, p_impedance_params[uc_imp_state].tx_conf_vl8);
	pplc_if_write8(REG_ATPL250_EMIT_CONFIG_VL8, p_impedance_params[uc_imp_state].emit_conf_vl8);
	pplc_if_write8(REG_ATPL250_EMIT_CONFIG_L8, p_impedance_params[uc_imp_state].emit_conf_l8);
	pplc_if_write32(REG_ATPL250_EMIT_NP_DELAY_32, p_impedance_params[uc_imp_state].emit_np_delay);
	pplc_if_write32(REG_ATPL250_EMIT_ON_ACTIVE_32, p_impedance_params[uc_imp_state].emit_on_active);
	pplc_if_write32(REG_ATPL250_EMIT_OFF_ACTIVE_32, p_impedance_params[uc_imp_state].emit_off_active);
	pplc_if_write8(REG_ATPL250_EMIT_GAIN_VL8, uc_new_emit_gain);
	pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_VL8, p_impedance_params[uc_imp_state].bf_vl8);
	if (uc_working_band != WB_CENELEC_A) {
		if (uc_imp_state != HI_STATE) {
			pplc_if_write8(REG_ATPL250_FFT_CONFIG_H8, 0x16);
			if (uc_working_band == WB_FCC) {
				pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_L16, 0x0800);
			} else { /* WB_ARIB */
				pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_L16, 0x0A00);
			}
		} else {
			pplc_if_write8(REG_ATPL250_FFT_CONFIG_H8, 0x0B);
			if (uc_working_band == WB_FCC) {
				pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_L16, 0x1000);
			} else { /* WB_ARIB */
				pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_L16, 0x3FFF);
			}
		}
	}
}

#ifdef ENABLE_DYNAMIC_CHIRP_CONFIGURATION

/**
 * \brief Transmits first part of preamble of a frame
 *
 */
static inline void _config_chirp(void)
{
	uint16_t us_loops;
	uint8_t uc_chirp_row;
	uint8_t uc_chirp_index;
	uint8_t *puc_fft_data;

	/* Disable SYNCM DETECTOR */
	pplc_if_and8(REG_ATPL250_SYNCM_CTL_L8, 0x7F);
	/* Reset IOB, Rotator and FFT */
	atpl250_iob_rotator_and_fft_push_rst();
	/* Reset TXRXBUF */
	atpl250_txrxb_push_rst();
	/* FFT PATH="11" */
	pplc_if_or8(REG_ATPL250_FFT_CONFIG_VL8, 0x93); /* IFFT=1; REAL='1'; PATH='11' */
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_rx_fft_shift);
	/* Enable IOB -> FFT path */
	atpl250_set_iobuf_to_fft();
	/* Release IOB, Rotator and FFT reset */
	atpl250_iob_rotator_and_fft_release_rst();
	/* Clear Rx Mode */
	atpl250_clear_rx_mode();
	/* Set tx mode */
	atpl250_set_tx_mode();

	/* Change value to modulator abcd points (full gain) */
	pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_fullgain, ABCD_POINTS_LEN);

	/* Set number of symbols and write P symbol */
	atpl250_set_num_symbols_cfg(1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier); /* Avoid overflow to n-th symbol when writing */
	atpl250_set_mod_bpsk_truepoint();
	pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);

	/* Set sym ready for IOB to pass symbol to FFT */
	atpl250_set_sym_ready();

	/* Wait for IOB to be empty */
	ul_flags_isr = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
	for (us_loops = 0; us_loops < 1000; us_loops++) {
		if (ul_flags_isr & INT_IOB_MASK_32) {
			break;
		} else {
			ul_flags_isr = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		}
	}

	if (us_loops < 1000) {
		/* Clear interrupt flag */
		atpl250_clear_iob_int();
		/* Reset IOB to set Rx mode */
		atpl250_iob_push_rst();
		/* Clear Tx Mode */
		atpl250_clear_tx_mode();
		/* 8 symbols 256 samples per symbol (just for this special functionality) */
		atpl250_set_iob_partition(IOB_8_SYMBOLS_OF_256_SAMPLES);
		atpl250_iob_release_rst();
		/* Set Rx mode */
		atpl250_set_rx_mode();

		/* Wait for IOB to be full */
		ul_flags_isr = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		for (us_loops = 0; us_loops < 1000; us_loops++) {
			if (ul_flags_isr & INT_IOB_MASK_32) {
				break;
			} else {
				ul_flags_isr = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
			}
		}

		if (us_loops < 1000) {
			pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x03FF);
			/* Access only real part of carrier */
			atpl250_set_iob_real_mode();
			/* Use auc_rrc_write_buffer just because its size fits */
			pplc_if_read_jump(BCODE_ZONE4, u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 512, JUMP_NO_JUMP, true);
			/* Access both real and imaginary part of carrier */
			atpl250_clear_iob_real_mode();

			/* Scale reference */
			ul_max_correlation_value = SYNC_SCALE_XCORR_REF((int16_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer,
					(int16_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);

			/* Write chirp to RAM */
			for (uc_chirp_row = 0; uc_chirp_row < CHIRP_NUM_ROWS; uc_chirp_row++) {
				/* Build row data */
				puc_fft_data = &u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[uc_chirp_row * FFT_BLOCK_SIZE_PER_ROW];
				for (uc_chirp_index = 0; uc_chirp_index < FFT_BLOCK_SIZE_PER_ROW; uc_chirp_index += 2) {
					auc_chirp_from_fft[uc_chirp_index] = *(puc_fft_data + FFT_BLOCK_SIZE_PER_ROW - 2 - uc_chirp_index);
					auc_chirp_from_fft[uc_chirp_index + 1] = *(puc_fft_data + FFT_BLOCK_SIZE_PER_ROW - 2 - uc_chirp_index + 1);
				}
				auc_chirp_from_fft[uc_chirp_index++] = 0x00;
				auc_chirp_from_fft[uc_chirp_index++] = 0x00;
				auc_chirp_from_fft[uc_chirp_index++] = uc_chirp_row;
				auc_chirp_from_fft[uc_chirp_index++] = 0x11;

				/* Write row */
				pplc_if_write_rep(REG_ATPL250_LOAD_DATA7_32, 9, auc_chirp_from_fft, CHIRP_ROW_SIZE);
			}
		}
	}

	atpl250_iob_push_rst();
	atpl250_set_iob_partition(IOB_8_SYMBOLS_OF_128_SAMPLES);  /* 128 samples per symbol */
	/* Disable IOB -> FFT path */
	atpl250_clear_iobuf_to_fft();
	atpl250_iob_release_rst();
	/* RX_FULL = 0 */
	atpl250_clear_rx_full();
}

#endif

/**
 * \brief Erase memory of InOut buffer
 *
 */
static inline void atpl250_erase_iob_memory(void)
{
	uint8_t uc_iob_busy;
	uint16_t us_iob_counter;

	/* Set IO Buffer to 1 symbol and erase it */
	atpl250_iob_push_rst();
	atpl250_set_iob_partition(IOB_1_SYMBOL_OF_1024_SAMPLES);
	atpl250_iob_release_rst();
	atpl250_clear_iobuf();

	/* Wait for busy */
	uc_iob_busy = true;
	us_iob_counter = 0;
	while (uc_iob_busy) {
		uc_iob_busy = atpl250_get_iob_busy();
		if (us_iob_counter++ > 50000) {
			break;
		}
	}
}

/**
 * \brief Initializes HW registers at startup
 *
 */
void atpl250_hw_init(uint8_t uc_imp_state)
{
	uint8_t uc_first_carrier_simple;
	uint8_t uc_num_carriers_simple;

	if (uc_working_band == WB_FCC) {
		p_impedance_params = impedance_params_fcc;
		puc_th = auc_th_fcc_arib;
	} else if (uc_working_band == WB_ARIB) {
		p_impedance_params = impedance_params_arib;
		puc_th = auc_th_fcc_arib;
	} else {
		p_impedance_params = impedance_params_cenelec_a;
		puc_th = auc_th_cenelec_a;
	}

#ifndef ENABLE_DYNAMIC_CHIRP_CONFIGURATION
	if (uc_working_band == WB_FCC) {
		puc_chirp = auc_chirp_fcc;
	} else if (uc_working_band == WB_ARIB) {
		puc_chirp = auc_chirp_arib;
	} else {
		puc_chirp = auc_chirp_cenelec_a;
	}
#endif

	/* Erase IOB memory */
	atpl250_erase_iob_memory();

	/* Configure InOut buffer */
	atpl250_iob_push_rst();
	atpl250_set_iob_partition(IOB_8_SYMBOLS_OF_128_SAMPLES);  /* 128 samples per symbol */
	atpl250_clear_iobuf_to_fft();
	atpl250_iob_release_rst();
	atpl250_set_rx_mode();

	/* Set Alpha RSSI to 5 */
	pplc_if_write8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0x05);

	pplc_if_write8(REG_ATPL250_INOUTB_QNTZ1_VH8, 0x02); /* Block IOB after PEAK2 */

	/* Configure TxRx buffer */
	atpl250_txrxb_push_rst();
	atpl250_rotator_push_rst();
	pplc_if_write32(REG_ATPL250_TXRXB_SYM_LENGTHS_32, 0x00800100); /* 256 samples (alternative = 128) */
	pplc_if_write8(REG_ATPL250_TXRXB_CTL_L8, 0x00); /* Data real from one channel */
	pplc_if_write8(REG_ATPL250_TXRXB_CTL_H8, 0x00); /* Samples transmitted from FFT; samples received to FFT; Frames divided in symbols */

	pplc_if_write8(REG_ATPL250_TXRXB_CTL_VL8, 0x16); /* CP_EXCLUSIVO=22 */

	/* NOISE_SYMBOLS configured to higher value to avoid all symbols passed after error */
	pplc_if_write8(REG_ATPL250_TXRXB_STATE_L8, 0x80);
	/* Block AGC on noise capture */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VH8, 0x10);

	pplc_if_write32(REG_ATPL250_TXRXB_SYM_CFG_32, 0x0000081E); /* cyclic_prefix1=0; overlap1=0; cyclic_prefix2=30; overlap2=8 */
	pplc_if_write32(REG_ATPL250_TXRXB_OFFSETS_32, 0x0000008B); /* Discard 0 samples after PEAK1. Discard 139 samples after PEAK2
	                                                            * (0.5*SYNCM+0.5*(cyclic_prefix2-overlap2) */

	/* pplc_if_write32(REG_ATPL250_TXRXB_PRE_ANALYSIS_32, 0x03010303); //4 prev symbols; Division by 4; average 4 symbols, enable average and send previous
	 * symbols */
	pplc_if_write32(REG_ATPL250_TXRXB_PRE_ANALYSIS_32, 0x01100000); /*pplc_if_write32(REG_ATPL250_TXRXB_PRE_ANALYSIS_32, 0x01100002);*/
	atpl250_rotator_release_rst();
	atpl250_txrxb_release_rst();

	/* Configure overlapping */
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)auc_coefs, 256);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&auc_coefs[256]), 256);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&auc_coefs[512]), 256);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&auc_coefs[768]), 256);
	pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0xDF); /* LOAD_COEFS=0 */

	/* Configure preamble in HW module */
	atpl250_pre_release_rst();
#ifdef ENABLE_DYNAMIC_CHIRP_CONFIGURATION
	_config_chirp();
#else
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA7_32, 9, (uint8_t *)puc_chirp, 288);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA7_32, 9, (uint8_t *)(&puc_chirp[288]), 288);
#endif

	pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0xEF); /* LOAD_PREAMBLE=0 */
	atpl250_pre_push_rst();
	pplc_if_write_buf(REG_ATPL250_PRE_REPH_32, (uint8_t *)auc_rep, 8);
	pplc_if_write8(REG_ATPL250_PRE_CFG_H8, 0x09); /* 9 repetitions */
	pplc_if_write16(REG_ATPL250_PRE_CFG_L16, 0x0808); /* Overlap_preamble=8 Cyclic_prefix_preamble=8 */
	/* pplc_if_write16(REG_ATPL250_PRE_CFG_L16, 0x0000); // Overlap_preamble=0 Cyclic_prefix_preamble=0 */
	pplc_if_and8(REG_ATPL250_TX_CONF_H8, 0x0F); /* OVERLAP_FIRST_EN_PR 0 1 2 & 3 = 0 */
	#ifdef TX_PREAMBLE_HW
	pplc_if_or8(REG_ATPL250_TX_CONF_L8, 0x0F);         /* OVERLAP_FIRST_EN 0 1 2 & 3 = 1 */
	#endif
	atpl250_pre_release_rst();

	/* Configure sync */
	atpl250_sync_push_rst();
#ifdef ENABLE_DYNAMIC_CHIRP_CONFIGURATION
	if (uc_notched_carriers > 0) {
		/* Compute correlation threshold (Peak1) dynamically as a percentage of the maximum correlation value (ideal case). */
		uint64_t ull_th_aux = (uint64_t)ul_max_correlation_value * CORRELATION_THRESHOLD_PERCENTAGE;
		uint32_t ul_threshold = (uint32_t)((ull_th_aux + 0x8000) >> 16);
		if (ul_threshold > 0xFFFFF) {
			/* Overflow protection */
			ul_threshold = 0xFFFFF;
		}

		puc_th[1] = (puc_th[1] & 0xF0) | ((uint8_t)(ul_threshold >> 16));
		puc_th[2] = (uint8_t)((ul_threshold >> 8) & 0xFF);
		puc_th[3] = (uint8_t)(ul_threshold & 0xFF);
	}
#endif

	/* CORR_LENGTH=128/256; THRESHOLD=20000; THRESHOLD_HALF=12000; PEAK_DISTANTE=128/256; CONSEC_PEAK=1; CONSEC_WINDOW=16; WINDOW_WIDTH=32 */
	pplc_if_write_buf(REG_ATPL250_SYNC_THRESHOLD_32, (uint8_t *)puc_th, 12); /* CORR_LENGTH=128/256; THRESHOLD=20000; THRESHOLD_HALF=12000;
	                                                                          * PEAK_DISTANTE=128/256; CONSEC_PEAK=1; CONSEC_WINDOW=16; WINDOW_WIDTH=32 */

	atpl250_sync_release_rst();

	/* Get first carrier and number of carriers of simplified static notching (COL_1) for SYNCM detector. */
	get_jump_simple_params(&uc_first_carrier_simple, &uc_num_carriers_simple);

	pplc_if_write16(REG_ATPL250_SYNCM_VALUE_H16, (uc_num_carriers_simple * 15) / 2); /* (carriers) * 15 (soft value for 1) / 2 */
	if (uc_working_band == WB_CENELEC_A) {
		pplc_if_write8(REG_ATPL250_SYNCM_CTL_H8, uc_used_carriers);
	} else {
		/* TIMEOUT=7; NUM_CARRIERS; INIT_ADDRESS=33; ENABLE=1 */
		pplc_if_write32(REG_ATPL250_SYNCM_CTL_32, 0x38008021 | ((uint32_t)uc_num_carriers_simple << 16));
	}

	/* Set first carrier for SyncM detector (depends on notching) */
	pplc_if_write8(REG_ATPL250_SYNCM_CTL_VL8, s_band_constants.uc_first_carrier + uc_first_carrier_simple);

	/* Configure Emit Control */
	atpl250_emit_ctl_push_rst();
	pplc_if_write32(REG_ATPL250_TXRX_TIME_ON1_32, s_band_constants.ul_txrx_time);
	pplc_if_write32(REG_ATPL250_PLC_TIME_ON1_32, s_band_constants.ul_txrx_plc);
	pplc_if_write32(REG_ATPL250_TXRX_TIME_OFF1_32, CFG_TXRX_TIME_OFF1);
	pplc_if_write32(REG_ATPL250_PLC_TIME_OFF1_32, CFG_TXRX_PLC_OFF1);
	atpl250_emit_ctl_release_rst();

	if (uc_working_band == WB_CENELEC_A) {
		/* Configure Interpolator */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x40);
		pplc_if_write8(REG_ATPL250_INT_CONF_H8, 0x01);
		pplc_if_write8(REG_ATPL250_INT_CONF_L8, 0x03);
		pplc_if_write8(REG_ATPL250_INT_CONF_VH8, 0x01);
		pplc_if_write16(REG_ATPL250_INT_RRC_1_H16, 0x003E);
		pplc_if_write32(REG_ATPL250_INT_RRC_BAUD_SHIFT_1_32, 0x000400B3);
		pplc_if_write32(REG_ATPL250_INT_RRC_2_32, 0x051C00B3);
		/* pplc_if_write8(REG_ATPL250_INT_RRC_BAUD_SHIFT_2_H8, 0x0A); */
		pplc_if_write8(REG_ATPL250_INT_RRC_BAUD_SHIFT_2_H8, 0x08);

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef1_init_cenelec_a, L1_LOAD_RRC_INT_1, L0_LOAD_RRC_INT_1 );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFE);
		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef2_init_cenelec_a, L1_LOAD_RRC_INT_2, L0_LOAD_RRC_INT_2 );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFB);

		/*Clipping configuration*/
		pplc_if_and8(REG_ATPL250_FFT_CONFIG_VL8, 0xDF);        /* clipping mask */
		pplc_if_or8(REG_ATPL250_FFT_CONFIG_VL8, 0x20);       /* clipping activation si descomentas esta linea se activa el clipping */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_H8, 0x0B);       /* CLIPPING_GAIN */

		/*Delay pad emission configuration*/
		pplc_if_and8(REG_ATPL250_EMIT_CONFIG_H8, 0xFD);
		/* pplc_if_or8(REG_ATPL250_EMIT_CONFIG_H8, 0x02);//comment to disable delay */
		/*Switching off pad transmission*/
		pplc_if_write16(REG_ATPL250_EMIT_FREC_DELAY_H16, 0x3000);         /* 0x3000 son unos 175us Clk a 70Mhz */
		pplc_if_write32(REG_ATPL250_EMIT_SOFT_TIME_X_32, 0x00004040);        /* P2_X (31:24);N2_X(23:16);P1_X(15:8);N1_X(7:0) */
		pplc_if_write32(REG_ATPL250_EMIT_SOFT_TIME_Y_32, 0x00000033);        /* N2_Y(15:12);P2_Y(11:8);N1_Y(7:4);P1_Y(3:0) */
		/*Emit gain*/
		pplc_if_write8(REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_VL8, 0xFF);        /* emit_gain_limit(7:0). */
		/* pplc_if_write32(REG_ATPL250_EMIT_PREDIST_32, 0x00000006); */
		/*Clock transmission*/
		pplc_if_and8(REG_ATPL250_EMIT_FREC_DELAY_VL8, 0xF0);
		pplc_if_or8(REG_ATPL250_EMIT_FREC_DELAY_VL8, 0x03);             /* 7 clk 36MHz */
		/*Bit Flipping Control*/
		pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_H8, 0x10);         /* PRF_C(5:0)=0x28  max=0x3F */
		pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_L8, 0x20);         /* ALTER_C(6:0)=0x1E max=0x7F */
		pplc_if_and8(REG_ATPL250_EMIT_BIT_FLIPPING_VL8, 0x00);
		switch (uc_imp_state) {
		case HI_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_hi);
			break;

		case LO_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_lo);
			break;

		case VLO_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_vlo);
			break;

		default:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_hi);
			break;
		}
		pplc_if_write16(REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_H16, 0x2C70);         /* CENELEC INI_DELAY_EMIT_REG configuration */
		pplc_if_write16(REG_ATPL250_INT_BAUDRATE_DELAY_H16, 0x2A78);            /*LAST_SAMPLE_DELAY_INT configured*/

		/* Configure Decimator */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x80);
		pplc_if_write8(REG_ATPL250_DEC_CONF_VH8, 0x03);
		pplc_if_write16(REG_ATPL250_DEC_CONF_L16, 0x011A);
		pplc_if_write16(REG_ATPL250_DEC_RRC_2_H16, 0x051A);
		pplc_if_write32(REG_ATPL250_DEC_RRC_1_32, 0x003E00B3);
		pplc_if_write32(REG_ATPL250_DEC_RRC_SHIFT_32, 0x00050500);

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef1_init_cenelec_a, L1_LOAD_RRC_DEC_1, L0_LOAD_RRC_DEC_1);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFD);

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef2_init_cenelec_a, L1_LOAD_RRC_DEC_2, L0_LOAD_RRC_DEC_2);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xF7);

		/* Configure AGC */
		pplc_if_or8(REG_ATPL250_AGC_CTL_VH8, 0x80);
		pplc_if_write32(REG_ATPL250_AGC_CUP01_32, 0x00260010);
		pplc_if_write32(REG_ATPL250_AGC_CUP23_32, 0x00060004);
		pplc_if_write32(REG_ATPL250_AGC_CDOWN12_32, 0x0056001E);
		pplc_if_write32(REG_ATPL250_AGC_CDOWN34_32, 0x00160008);
		pplc_if_write32(REG_ATPL250_AGC_AT01_32, 0x00110029);
		pplc_if_write32(REG_ATPL250_AGC_AT23_32, 0x00620115);
		pplc_if_write16(REG_ATPL250_AGC_AT4_MAXTRIANG_H16, 0x017E);
		pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_H16, 0x6311);
		pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_L16, 0x0400);
		pplc_if_write32(REG_ATPL250_AGC_FINE_COMP_32, 0x00F00028);
		pplc_if_and8(REG_ATPL250_AGC_CTL_VH8, 0x7F);

		/* Configure CD CENELEC-A (400kbps) */         /* Threshold_full (0x06400) y Threshold_half (0xFFFFF) configured previously */
		pplc_if_write32(REG_ATPL250_CD_CFG_32, 0x01200A06);             /*
		                                                                 *(31:16)=length_peak_full;(15:4)=length_peak_half;(3:0)=wait_robus&peak_full
		                                                                 **/
		pplc_if_write32(REG_ATPL250_CD_MOD_32, 0x99790020);             /* (31:24)=step_m_up;(23:16)=step_m_down;(15:0)=min_m_th; */
		pplc_if_write32(REG_ATPL250_CD_RAMP_32, 0x03010510);            /* (31:24)=step_r_up;(23:16)=step_r_down;(15:8)=min_r_th;(7:0)=r_margin; */
		pplc_if_write32(REG_ATPL250_CD_LENGTH_32, 0x03FF0120);          /* (31:16)=length_ramp;(15:0)=length_chirp; */
		pplc_if_write32(REG_ATPL250_CD_UPDATE_32, 0x12001200);          /* (31:16)=time_update_module_th;(15:0)=time_update_ramp_th; */
	} else if (uc_working_band == WB_FCC) {
		/* Configure Interpolator */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x40);           /* SRST_INT=1 */
		pplc_if_write32(REG_ATPL250_INT_CONF_32, 0x01010400);         /* CORDIC_TX_BYPASS='1'; FSK_MODE='0'; INT_DUAL_CHANNEL='0'; INT_RECTPULSE_2C='0';
		                                                               * INT_BYPASS_2C='1'; INT_SHIFT=4; INT_SHIFT_2C=0 (bypassed); */
		pplc_if_write16(REG_ATPL250_INT_CYCLES_L16, 0x010E);         /* INT_CYCLES=1; INT_CYCLES_2C=14; */

		pplc_if_write32(REG_ATPL250_INT_RRC_0_32, 0x0038003B);                       /* INT_RRC_NUM_CANAL  ='0'; INT_RRC_SIMETRIA  ='0'; INT_RRC_IMPAR  ='0';
		                                                                              * INT_RRC_FACTOR  =0; INT_RRC_USED_COEFS=  56; INT_RRC_NUM_CYCLES=  59;
		                                                                              * INT_RRC_COMPLEX='0'; INT_RRC_BYPASS='0'; */
		pplc_if_write32(REG_ATPL250_INT_RRC_1_32, 0x013A003B);                     /* INT_RRC_NUM_CANAL_1='0'; INT_RRC_SIMETRIA_1='0'; INT_RRC_IMPAR_1='0';
		                                                                            * INT_RRC_FACTOR_1=1; INT_RRC_USED_COEFS_1=56; INT_RRC_NUM_CYCLES_1=59; */
		pplc_if_write32(REG_ATPL250_INT_RRC_2_32, 0x021C001D);                     /* INT_RRC_NUM_CANAL_2='0'; INT_RRC_SIMETRIA_2='0'; INT_RRC_IMPAR_2='0';
		                                                                            * INT_RRC_FACTOR_2=2; INT_RRC_USED_COEFS_2=28; INT_RRC_NUM_CYCLES_2=29; */
		pplc_if_write32(REG_ATPL250_INT_RRC_BAUD_SHIFT_0_32, 0x0007003B);            /* INT_RRC_SHIFT  =7;       INT_RRC_BAUDRATE  =59; */
		pplc_if_write32(REG_ATPL250_INT_RRC_BAUD_SHIFT_1_32, 0x0007001D);          /* INT_RRC_SHIFT_1=7;       INT_RRC_BAUDRATE_1=29; */
		pplc_if_write32(REG_ATPL250_INT_RRC_BAUD_SHIFT_2_32, 0x00070009);          /* INT_RRC_SHIFT_2=7;       INT_RRC_BAUDRATE_2= 9; */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);         /* SRST_INT=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef0_init_fcc, L1_LOAD_RRC_INT, L0_LOAD_RRC_INT );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0xBF);         /* LOAD_INT_RRC=0 */
		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);         /* SRST_INT=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef1_init_fcc, L1_LOAD_RRC_INT_1, L0_LOAD_RRC_INT_1 );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFE);         /* LOAD_INT_RRC_1=0 */
		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);         /* SRST_INT=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef2_init_fcc, L1_LOAD_RRC_INT_2, L0_LOAD_RRC_INT_2 );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFB);         /* LOAD_INT_RRC_2=0 */

		/*Clipping configuration*/
		pplc_if_and8(REG_ATPL250_FFT_CONFIG_VL8, 0xDF);        /* clipping mask */
		pplc_if_or8(REG_ATPL250_FFT_CONFIG_VL8, 0x20); /* clipping activation si descomentas esta linea se activa el clipping * / */

		/*Delay pad emission configuration*/
		pplc_if_and8(REG_ATPL250_EMIT_CONFIG_H8, 0xFD);
		/* pplc_if_or8(REG_ATPL250_EMIT_CONFIG_H8, 0x02);//comment to disable delay */
		/*Switching off pad transmission*/
		pplc_if_write16(REG_ATPL250_EMIT_FREC_DELAY_H16, 0x2400);
		pplc_if_write32(REG_ATPL250_EMIT_SOFT_TIME_X_32, 0x10FF10FF);        /* P2_X (31:24);N2_X(23:16);P1_X(15:8);N1_X(7:0) */
		pplc_if_write32(REG_ATPL250_EMIT_SOFT_TIME_Y_32, 0x0000FFFF);        /* N2_Y(15:12);P2_Y(11:8);N1_Y(7:4);P1_Y(3:0) */
		/*Emit gain*/
		pplc_if_write8(REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_VL8, 0xFF);        /* emit_gain_limit(7:0). */
		/* pplc_if_write32(REG_ATPL250_EMIT_PREDIST_32, 0x00000006); */
		/*Bit Flipping Control*/
		pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_H8, 0x01);         /* PRF_C(5:0)=0x28  max=0x3F */
		pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_L8, 0x20);         /* ALTER_C(6:0)=0x1E max=0x7F */
		pplc_if_and8(REG_ATPL250_EMIT_BIT_FLIPPING_VL8, 0x00);
		switch (uc_imp_state) {
		case HI_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_hi);
			break;

		case LO_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_lo);
			break;

		case VLO_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_vlo);
			break;

		default:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_hi);
			break;
		}
		pplc_if_write16(REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_H16, 0x0DBE);     /* Time (in 72MHz cycles) of delay from TX order and emission pads activation*/
		pplc_if_write16(REG_ATPL250_INT_BAUDRATE_DELAY_H16, 0x0E58);            /*Final transmission edge is delayed LAST_SAMPLE_DELAY_INT clock cycles */

		/* Configure Decimator */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x80);          /* SRST_DEC=1 */
		pplc_if_write32(REG_ATPL250_DEC_CONF_32, 0x03010100);           /* CORDIC_RX_SINGLE_BYPASS='1'; CORDIC_RX_BYPASS='1'; DEC_BYPASS_4C='1'; DEC_SHIFT=1;
		                                                                 * DEC_SHIFT_4C=0 (bypassed); */
		pplc_if_write32(REG_ATPL250_DEC_FACTOR_32, 0x00050001);         /* DEC_FACTOR=5; DEC_FACTOR_4C=1; */

		pplc_if_write32(REG_ATPL250_DEC_RRC_0_32, 0x301C003B);                /* DEC_RRC_NUM_CANAL  =0; DEC_RRC_SIMETRIA  ='1'; DEC_RRC_IMPAR  ='1';
		                                                                       * DEC_RRC_FACTOR  =0; DEC_RRC_USED_COEFS  =28; DEC_RRC_NUM_CYCLES  =59;
		                                                                       * DEC_RRC_COMPLEX='0'; DEC_RRC_BYPASS='0'; */
		pplc_if_write32(REG_ATPL250_DEC_RRC_1_32, 0x1138001D);
		pplc_if_write32(REG_ATPL250_DEC_RRC_2_32, 0x111A000E);
		pplc_if_write32(REG_ATPL250_DEC_RRC_SHIFT_32, 0x00060707);          /* DEC_RRC_SHIFT_2=6; DEC_RRC_SHIFT_1=7; DEC_RRC_SHIFT=7; */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);          /* SRST_DEC=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef2_init_fcc, L1_LOAD_RRC_DEC_2, L0_LOAD_RRC_DEC_2);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xF7);          /* LOAD_DEC_RRC_2=0 */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);          /* SRST_DEC=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef1_init_fcc, L1_LOAD_RRC_DEC_1, L0_LOAD_RRC_DEC_1);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFD);          /* LOAD_DEC_RRC_1=0 */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);          /* SRST_DEC=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef0_init_fcc, L1_LOAD_RRC_DEC, L0_LOAD_RRC_DEC);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0x7F);          /* LOAD_DEC_RRC=0 */

		/* Configure AGC */
		pplc_if_or8(REG_ATPL250_AGC_CTL_VH8, 0x80);
		pplc_if_write32(REG_ATPL250_AGC_CUP01_32, 0x00260010);
		pplc_if_write32(REG_ATPL250_AGC_CUP23_32, 0x00060004);
		pplc_if_write32(REG_ATPL250_AGC_CDOWN12_32, 0x0056001E);
		pplc_if_write32(REG_ATPL250_AGC_CDOWN34_32, 0x00160008);
		pplc_if_write32(REG_ATPL250_AGC_AT01_32, 0x00110029);
		pplc_if_write32(REG_ATPL250_AGC_AT23_32, 0x00620115);
		pplc_if_write16(REG_ATPL250_AGC_AT4_MAXTRIANG_H16, 0x017E);
		pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_H16, 0x6311);
		pplc_if_write32(REG_ATPL250_AGC_FINE_COMP_32, 0x00F00028);
		pplc_if_and8(REG_ATPL250_AGC_CTL_VH8, 0x7F);

		/* Configure CD FCC (1.2MHz Envelope CD not tested)*/
		pplc_if_write32(REG_ATPL250_CD_CFG_32, 0x01200A06);             /*(31:16)=length_peak_full;(15:4)=length_peak_half;(3:0)=wait_robus&peak_full */
		pplc_if_write32(REG_ATPL250_CD_MOD_32, 0x99790020);             /* (31:24)=step_m_up;(23:16)=step_m_down;(15:0)=min_m_th; */
		pplc_if_write32(REG_ATPL250_CD_RAMP_32, 0x03010510);            /* (31:24)=step_r_up;(23:16)=step_r_down;(15:8)=min_r_th;(7:0)=r_margin; */
		pplc_if_write32(REG_ATPL250_CD_LENGTH_32, 0x03FF0120);          /* (31:16)=length_ramp;(15:0)=length_chirp; */
		pplc_if_write32(REG_ATPL250_CD_UPDATE_32, 0x12001200);          /* (31:16)=time_update_module_th;(15:0)=time_update_ramp_th; */
	} else { /* WB_ARIB */
		 /* Configure Interpolator */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x40);           /* SRST_INT=1 */
		pplc_if_write32(REG_ATPL250_INT_CONF_32, 0x01010400);         /* CORDIC_TX_BYPASS='1'; FSK_MODE='0'; INT_DUAL_CHANNEL='0'; INT_RECTPULSE_2C='0';
		                                                               * INT_BYPASS_2C='1'; INT_SHIFT=4; INT_SHIFT_2C=0 (bypassed); */
		pplc_if_write16(REG_ATPL250_INT_CYCLES_L16, 0x010E);         /* INT_CYCLES=1; INT_CYCLES_2C=14; */

		pplc_if_write32(REG_ATPL250_INT_RRC_0_32, 0x0038003B);                       /* INT_RRC_NUM_CANAL  ='0'; INT_RRC_SIMETRIA  ='0'; INT_RRC_IMPAR  ='0';
		                                                                              * INT_RRC_FACTOR  =0; INT_RRC_USED_COEFS=  56; INT_RRC_NUM_CYCLES=  59;
		                                                                              * INT_RRC_COMPLEX='0'; INT_RRC_BYPASS='0'; */
		pplc_if_write32(REG_ATPL250_INT_RRC_1_32, 0x013A003B);                     /* INT_RRC_NUM_CANAL_1='0'; INT_RRC_SIMETRIA_1='0'; INT_RRC_IMPAR_1='0';
		                                                                            * INT_RRC_FACTOR_1=1; INT_RRC_USED_COEFS_1=56; INT_RRC_NUM_CYCLES_1=59; */
		pplc_if_write32(REG_ATPL250_INT_RRC_2_32, 0x021C001D);                     /* INT_RRC_NUM_CANAL_2='0'; INT_RRC_SIMETRIA_2='0'; INT_RRC_IMPAR_2='0';
		                                                                            * INT_RRC_FACTOR_2=2; INT_RRC_USED_COEFS_2=28; INT_RRC_NUM_CYCLES_2=29; */
		pplc_if_write32(REG_ATPL250_INT_RRC_BAUD_SHIFT_0_32, 0x0007003B);            /* INT_RRC_SHIFT  =7;       INT_RRC_BAUDRATE  =59; */
		pplc_if_write32(REG_ATPL250_INT_RRC_BAUD_SHIFT_1_32, 0x0007001D);          /* INT_RRC_SHIFT_1=7;       INT_RRC_BAUDRATE_1=29; */
		pplc_if_write32(REG_ATPL250_INT_RRC_BAUD_SHIFT_2_32, 0x00070009);          /* INT_RRC_SHIFT_2=7;       INT_RRC_BAUDRATE_2= 9; */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);         /* SRST_INT=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef0_init_arib, L1_LOAD_RRC_INT, L0_LOAD_RRC_INT );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0xBF);         /* LOAD_INT_RRC=0 */
		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);         /* SRST_INT=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef1_init_arib, L1_LOAD_RRC_INT_1, L0_LOAD_RRC_INT_1 );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFE);         /* LOAD_INT_RRC_1=0 */
		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);         /* SRST_INT=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_coef2_init_arib, L1_LOAD_RRC_INT_2, L0_LOAD_RRC_INT_2 );
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFB);         /* LOAD_INT_RRC_2=0 */

		/*Clipping configuration*/
		pplc_if_and8(REG_ATPL250_FFT_CONFIG_VL8, 0xDF);        /* clipping mask */
		pplc_if_or8(REG_ATPL250_FFT_CONFIG_VL8, 0x20); /* clipping activation si descomentas esta linea se activa el clipping * / */

		/*Delay pad emission configuration*/
		pplc_if_and8(REG_ATPL250_EMIT_CONFIG_H8, 0xFD);
		/* pplc_if_or8(REG_ATPL250_EMIT_CONFIG_H8, 0x02);//comment to disable delay */
		/*Switching off pad transmission*/
		pplc_if_write16(REG_ATPL250_EMIT_FREC_DELAY_H16, 0x2400);
		pplc_if_write32(REG_ATPL250_EMIT_SOFT_TIME_X_32, 0x10FF10FF);        /* P2_X (31:24);N2_X(23:16);P1_X(15:8);N1_X(7:0) */
		pplc_if_write32(REG_ATPL250_EMIT_SOFT_TIME_Y_32, 0x0000FFFF);        /* N2_Y(15:12);P2_Y(11:8);N1_Y(7:4);P1_Y(3:0) */
		/*Emit gain*/
		pplc_if_write8(REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_VL8, 0xFF);        /* emit_gain_limit(7:0). */
		/* pplc_if_write32(REG_ATPL250_EMIT_PREDIST_32, 0x00000006); */
		/*Bit Flipping Control*/
		pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_H8, 0x01);         /* PRF_C(5:0)=0x28  max=0x3F */
		pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_L8, 0x20);         /* ALTER_C(6:0)=0x1E max=0x7F */
		pplc_if_and8(REG_ATPL250_EMIT_BIT_FLIPPING_VL8, 0x00);
		switch (uc_imp_state) {
		case HI_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_hi);
			break;

		case LO_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_lo);
			break;

		case VLO_STATE:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_vlo);
			break;

		default:
			atpl250_update_branch_cfg(uc_imp_state, s_band_constants.uc_emit_gain_hi);
			break;
		}
		pplc_if_write16(REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_H16, 0x14D2); /* Time (in 72MHz cycles) of delay from TX order and emission pads activation*/
		pplc_if_write16(REG_ATPL250_INT_BAUDRATE_DELAY_H16, 0x19D3);            /*Final transmission edge is delayed LAST_SAMPLE_DELAY_INT clock cycles */

		/* Configure Decimator */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x80);          /* SRST_DEC=1 */
		pplc_if_write32(REG_ATPL250_DEC_CONF_32, 0x03010100);           /* CORDIC_RX_SINGLE_BYPASS='1'; CORDIC_RX_BYPASS='1'; DEC_BYPASS_4C='1'; DEC_SHIFT=1;
		                                                                 * DEC_SHIFT_4C=0 (bypassed); */
		pplc_if_write32(REG_ATPL250_DEC_FACTOR_32, 0x00050001);         /* DEC_FACTOR=5; DEC_FACTOR_4C=1; */

		pplc_if_write32(REG_ATPL250_DEC_RRC_0_32, 0x301C003B);                /* DEC_RRC_NUM_CANAL  =0; DEC_RRC_SIMETRIA  ='1'; DEC_RRC_IMPAR  ='1';
		                                                                       * DEC_RRC_FACTOR  =0; DEC_RRC_USED_COEFS  =28; DEC_RRC_NUM_CYCLES  =59;
		                                                                       * DEC_RRC_COMPLEX='0'; DEC_RRC_BYPASS='0'; */
		pplc_if_write32(REG_ATPL250_DEC_RRC_1_32, 0x1138001D);
		pplc_if_write32(REG_ATPL250_DEC_RRC_2_32, 0x111A000E);
		pplc_if_write32(REG_ATPL250_DEC_RRC_SHIFT_32, 0x00060707);          /* DEC_RRC_SHIFT_2=6; DEC_RRC_SHIFT_1=7; DEC_RRC_SHIFT=7; */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);          /* SRST_DEC=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef2_init_arib, L1_LOAD_RRC_DEC_2, L0_LOAD_RRC_DEC_2);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xF7);          /* LOAD_DEC_RRC_2=0 */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);          /* SRST_DEC=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef1_init_arib, L1_LOAD_RRC_DEC_1, L0_LOAD_RRC_DEC_1);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFD);          /* LOAD_DEC_RRC_1=0 */

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);          /* SRST_DEC=0 */
		atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef0_init_arib, L1_LOAD_RRC_DEC, L0_LOAD_RRC_DEC);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0x7F);          /* LOAD_DEC_RRC=0 */

		/* Configure AGC */
		pplc_if_or8(REG_ATPL250_AGC_CTL_VH8, 0x80);
		pplc_if_write32(REG_ATPL250_AGC_CUP01_32, 0x00260010);
		pplc_if_write32(REG_ATPL250_AGC_CUP23_32, 0x00060004);
		pplc_if_write32(REG_ATPL250_AGC_CDOWN12_32, 0x0056001E);
		pplc_if_write32(REG_ATPL250_AGC_CDOWN34_32, 0x00160008);
		pplc_if_write32(REG_ATPL250_AGC_AT01_32, 0x00110029);
		pplc_if_write32(REG_ATPL250_AGC_AT23_32, 0x00620115);
		pplc_if_write16(REG_ATPL250_AGC_AT4_MAXTRIANG_H16, 0x017E);
		pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_H16, 0x6311);
		pplc_if_write32(REG_ATPL250_AGC_FINE_COMP_32, 0x00F00028);
		pplc_if_and8(REG_ATPL250_AGC_CTL_VH8, 0x7F);

		/* Configure CD ARIB (1.2MHz Envelope CD not tested)*/
		pplc_if_write32(REG_ATPL250_CD_CFG_32, 0x01200A06);             /* (31:16)=length_peak_full;(15:4)=length_peak_half;(3:0)=wait_robus&peak_full */
		pplc_if_write32(REG_ATPL250_CD_MOD_32, 0x99790020);             /* (31:24)=step_m_up;(23:16)=step_m_down;(15:0)=min_m_th; */
		pplc_if_write32(REG_ATPL250_CD_RAMP_32, 0x03010510);            /* (31:24)=step_r_up;(23:16)=step_r_down;(15:8)=min_r_th;(7:0)=r_margin; */
		pplc_if_write32(REG_ATPL250_CD_LENGTH_32, 0x03FF0120);          /* (31:16)=length_ramp;(15:0)=length_chirp; */
		pplc_if_write32(REG_ATPL250_CD_UPDATE_32, 0x12001200);          /* (31:16)=time_update_module_th;(15:0)=time_update_ramp_th; */
	}

	/* General configuration */
	/* Configure RMS_CALC time*/
	pplc_if_write16(REG_ATPL250_RMS_CALC_CFG1_L16, 0x0A00); /* T2_WAIT time */
	pplc_if_write16(REG_ATPL250_RMS_CALC_CFG1_H16, 0x003C); /* "000" & CHANNEL_N & T1_WAIT(11:0) time Valid in CENELEC, FCC and ARIB*/

	/*Clock transmission*/
	pplc_if_and8(REG_ATPL250_EMIT_FREC_DELAY_VL8, 0xF0);
	pplc_if_or8(REG_ATPL250_EMIT_FREC_DELAY_VL8, 0x03);             /* Freq=12MHz */
	if ((uc_working_band == WB_FCC) || (uc_working_band == WB_ARIB)) {
		/* Configure Baudrate for FCC and ARIB */
		pplc_if_write16(REG_ATPL250_TXRXB_BAUDRATE_L16, 0x003B);          /* Reception baudrate: 1.2MHz (G3 FCC). Configured to 400kHz by default (G3 CENELEC)*/
		pplc_if_write16(REG_ATPL250_INT_BAUDRATE_DELAY_L16, 0x003B);
		pplc_if_write8(REG_ATPL250_EMIT_FREC_DELAY_VL8, 0x07);          /* Freq=36MHz */
	}

	/* Enable START, SPI, BER, RX_ERROR, TX_SOLAPE, TX_CD, TX_END, TX_START, VTB, NOPEAK2, PEAK2, PEAK1, IOB, NOISE_START, NOISE_ERROR */
	pplc_if_write32(REG_ATPL250_INT_MASK_32, 0xDDFFFF4F);

	/* Configure SYNCM detection */
	pplc_if_or8(REG_ATPL250_SYNCM_CTL_L8, 0x80); /* Enable SYNCM */

	/* Initialization BER */
	if (uc_working_band == WB_CENELEC_A) {
		pplc_if_write16(REG_ATPL250_BER_PERIPH_CFG3_L16, 0x1F13); /* FEM=0x1F; FEM_CR=0x13 */
		pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG4_32, 0x281A1B12); /* FEM_HIGH=0x28; FEM_HIGH_CR=0x1A; FEM_LOW=0x1B; FEM_LOW_CR=0x12; */
	} else {
		pplc_if_write16(REG_ATPL250_BER_PERIPH_CFG3_L16, 0x1713); /* FEM=0x17; FEM_CR=0x13 */
		pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG4_32, 0x1E1A1512); /* FEM_HIGH=0x1E; FEM_HIGH_CR=0x1A; FEM_LOW=0x15; FEM_LOW_CR=0x12; */
	}

	pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG5_32, 0x01430025); /* FRONT_8PSK_MINUS1=0x143;  FRONT_8PSK=0x025 */
	pplc_if_write8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x16); /* INT_BER=0; MODUL_PY_USED=1 (BPSK); MODUL_HD_USED=1 (BPSK); MODE_BER=2 (ONLY PAYLOAD/FCH) */

	pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG10_32, 0xA5D2C3C3); /* coherent values for 1's and 0's */
	pplc_if_write16(REG_ATPL250_BER_PERIPH_CFG11_L16, 0x96C3);

	if (uc_working_band == WB_FCC) {
		if (uc_legacy_mode) {
			atpl250_set_ber_fch_coh();
		} else {
			atpl250_set_ber_fch_diff();
		}
	} else {
		atpl250_set_ber_fch_diff();
	}

	pplc_if_write16(REG_ATPL250_BER_PERIPH_CFG1_L16, (uc_used_carriers - 1) << 8 | (uc_num_symbols_fch - 1));

	/* SET preamble gain */
	pplc_if_write8(REG_ATPL250_PRE_CFG_VH8, 0x74);

	/* Clear all interrupt flags */
	pplc_if_write32(REG_ATPL250_INT_FLAGS_32, 0x00000000);

	/* Force interrupt pin to generate a new edge if there is a pending flag after raising */
	pplc_if_or8(REG_ATPL250_INT_CFG_VL8, 0x01);

	/* Configure rotator */
	pplc_if_write32(REG_ATPL250_ROTATOR_CONFIG3_32, 0x0502007F); /* No bypass, by default rotation is 0 */
	pplc_if_write8(REG_ATPL250_ROTATOR_CONFIG3_H8, 0x01);

	/* Set FFT_SHIFT for reception */
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_rx_fft_shift);

	/* configure Zero-Cross-Detector */
	pplc_if_write8(REG_ATPL250_ZC_CONFIG_VL8, 0x0D);

	/* Set Rx mode */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x033A); /* Avoid overflow to 8th symbol when reading */
	/* Configure FFT and Clipping */
	atpl250_fft_push_rst();
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_VL8, 0x30); /* REVERSE=0; CLIPPING_ENABLE=1; FFT_REAL=1; FFT_PATH=0 (RX) */
	atpl250_fft_release_rst();

	_update_num_sym_block_fch();
	initialize_predistorsion();
}

static void tposun_fu(uint8_t uc_n_arg, int32_t sl_lambda, int32_t sl_lambdac2_param, int8_t sc_s)
{
	int64_t sll_alpha_64;
	int32_t sl_d1, sl_d2_1, sl_d2_2, sl_d3, sl_d4_1, sl_d4_2, sl_d5;
	int64_t sll_a2_1, sll_a2_2, sll_a3, sll_a4_1, sll_a4_2, sll_a5;
	uint8_t uc_m, uc_k, uc_t;

	memset(u_shared_buffers.s_rrc_calc.asl_alpha_32, 0, sizeof(u_shared_buffers.s_rrc_calc.asl_alpha_32));

	if (uc_n_arg == 0) {
		u_shared_buffers.s_rrc_calc.ass_A_row[0] = 32767; /* 32678 */
	} else {
		u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 1 - 1] = (sl_lambda << 16);
		if (uc_n_arg > 1) {
			for (uc_m = 2; uc_m <= uc_n_arg; uc_m++) {
				/* alpha_32[n+1-1]=(alpha_32[n+1-1] >> 15)*lambda; */
				sll_alpha_64 = (int64_t)u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 1 - 1] * (int64_t)sl_lambda;
				u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 1 - 1] = sll_alpha_64 >> 15;
			}
		}

		for (uc_k = 1; uc_k <= uc_n_arg; uc_k++) {
			sl_d1   = (uc_k * (2 * uc_n_arg - uc_k)) << 16;
			sl_d2_1 = (2 * sc_s * ((uc_k - 1) * (2 * uc_n_arg + 1 - uc_k))) << 16;
			sl_d2_2 = -2 * 2 * sc_s * (int32_t)sl_lambdac2_param * (uc_n_arg + 1 - uc_k) * (2 * uc_n_arg + 1 - 2 * uc_k);
			sl_d3   = 2 * 4 * (int32_t)sl_lambdac2_param * (uc_n_arg + 2 - uc_k);
			sl_d4_1 = (-2 * sc_s * ((uc_k - 3) * (2 * uc_n_arg + 3 - uc_k))) << 16;
			sl_d4_2 = 2 * 2 * sc_s * ((int32_t)sl_lambdac2_param * (uc_n_arg + 3 - uc_k) * (2 * uc_n_arg + 7 - 2 * uc_k));
			sl_d5   = ((uc_k - 4) * (2 * uc_n_arg + 4 - uc_k)) << 16;

			sll_a2_1 = (int64_t)u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 2 - uc_k - 1] * (int64_t)sl_d2_1;
			sll_a2_2 = (int64_t)u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 2 - uc_k - 1] * (int64_t)sl_d2_2;
			sll_a3   = (int64_t)u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 3 - uc_k - 1] * (int64_t)sl_d3;
			sll_a4_1 = (int64_t)u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 4 - uc_k - 1] * (int64_t)sl_d4_1;
			sll_a4_2 = (int64_t)u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 4 - uc_k - 1] * (int64_t)sl_d4_2;
			sll_a5   = (int64_t)u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 5 - uc_k - 1] * (int64_t)sl_d5;

			u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_n_arg + 1 - uc_k -
			1] = (sll_a2_1 + sll_a2_2 + sll_a3 + sll_a4_1 + sll_a4_2 + sll_a5) / sl_d1;
		}

		for (uc_t = 0; uc_t < uc_n_arg + 1; uc_t++) {
			if (uc_t == 0) {
				u_shared_buffers.s_rrc_calc.ass_A_row[0] = u_shared_buffers.s_rrc_calc.asl_alpha_32[0] >> 17;
			} else {
				u_shared_buffers.s_rrc_calc.ass_A_row[uc_t] = u_shared_buffers.s_rrc_calc.asl_alpha_32[uc_t] >> 16;
			}
		}
	}
}

static void VectorA_fu(int32_t sl_lambda, uint8_t uc_dir, uint8_t uc_row, int32_t sl_lambdac1_param)
{
	memset(u_shared_buffers.s_rrc_calc.ass_A_row, 0, sizeof(u_shared_buffers.s_rrc_calc.ass_A_row));

	if (uc_dir == 0) {
		tposun_fu(uc_row - 1, sl_lambda, sl_lambdac1_param, -1);
	} else {
		if (uc_dir == 1) {
			tposun_fu(uc_row - 1, sl_lambda, sl_lambdac1_param, 1);
		} else {
			tposun_fu(uc_row - 1, 1, 0, 1);
		}
	}
}

static void tune(int16_t ss_cos_om_0, uint8_t uc_step)
{
	uint8_t uc_row;
	uint8_t uc_i;
	uint8_t uc_j;
	uint8_t uc_k;

	if (uc_step == 0) {
		/* First pass. Calculate lambdas */
		sl_lambda1  = -477200384 / (-32768 + ss_cos_om_0);
		sl_lambda2  = 1670283264 / (32768 + ss_cos_om_0);
		sl_lambdac1 = ((32768 - sl_lambda1) << 15) / sl_lambda1;
		sl_lambdac2 = ((32768 - sl_lambda2) << 15) / sl_lambda2;

		memset(u_shared_buffers.s_rrc_calc.asl_at, 0, sizeof(u_shared_buffers.s_rrc_calc.asl_at));

		for (uc_k = 0; uc_k < (FILTER_LENGTH_H); uc_k++) {
			if (uc_k == 0) {
				u_shared_buffers.s_rrc_calc.ass_a[0] = ass_h_in_cenelec_a[0];
			} else {
				u_shared_buffers.s_rrc_calc.ass_a[uc_k] = (ass_h_in_cenelec_a[uc_k] << 1);
			}
		}
	}

	/* Obtain 4 coeffs, depending on step */
	if (ss_cos_om_m_cenelec_a > ss_cos_om_0) {
		uc_row = (uc_step << 1) + 1;
		VectorA_fu(sl_lambda1, 0, uc_row, sl_lambdac1);
		for (uc_i = 0; uc_i < (FILTER_LENGTH_H); uc_i++) {
			u_shared_buffers.s_rrc_calc.asl_at[uc_i] = u_shared_buffers.s_rrc_calc.asl_at[uc_i] +
					(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_i]
					* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
		}

		uc_row = (uc_step << 1) + 2;
		VectorA_fu(sl_lambda1, 0, uc_row, sl_lambdac1);
		for (uc_i = 0; uc_i < (FILTER_LENGTH_H); uc_i++) {
			u_shared_buffers.s_rrc_calc.asl_at[uc_i] = u_shared_buffers.s_rrc_calc.asl_at[uc_i] +
					(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_i]
					* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
		}

		uc_row = FILTER_LENGTH_H - (uc_step << 1);
		VectorA_fu(sl_lambda1, 0, uc_row, sl_lambdac1);
		for (uc_i = 0; uc_i < (FILTER_LENGTH_H); uc_i++) {
			u_shared_buffers.s_rrc_calc.asl_at[uc_i] = u_shared_buffers.s_rrc_calc.asl_at[uc_i] +
					(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_i]
					* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
		}

		uc_row = FILTER_LENGTH_H - 1 - (uc_step << 1);
		VectorA_fu(sl_lambda1, 0, uc_row, sl_lambdac1);
		for (uc_i = 0; uc_i < (FILTER_LENGTH_H); uc_i++) {
			u_shared_buffers.s_rrc_calc.asl_at[uc_i] = u_shared_buffers.s_rrc_calc.asl_at[uc_i] +
					(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_i]
					* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
		}
	} else {
		if (ss_cos_om_m_cenelec_a == ss_cos_om_0) {
			uc_row = (uc_step << 1) + 1;
			u_shared_buffers.s_rrc_calc.asl_at[uc_row - 1] = u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1];

			uc_row = (uc_step << 1) + 2;
			u_shared_buffers.s_rrc_calc.asl_at[uc_row - 1] = u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1];

			uc_row = FILTER_LENGTH_H - (uc_step << 1);
			u_shared_buffers.s_rrc_calc.asl_at[uc_row - 1] = u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1];

			uc_row = FILTER_LENGTH_H - 1 - (uc_step << 1);
			u_shared_buffers.s_rrc_calc.asl_at[uc_row - 1] = u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1];
		} else {
			uc_row = (uc_step << 1) + 1;
			VectorA_fu(sl_lambda2, 1, uc_row, sl_lambdac2);
			for (uc_j = 0; uc_j < (FILTER_LENGTH_H); uc_j++) {
				u_shared_buffers.s_rrc_calc.asl_at[uc_j] = u_shared_buffers.s_rrc_calc.asl_at[uc_j] +
						(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_j]
						* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
			}

			uc_row = (uc_step << 1) + 2;
			VectorA_fu(sl_lambda2, 1, uc_row, sl_lambdac2);
			for (uc_j = 0; uc_j < (FILTER_LENGTH_H); uc_j++) {
				u_shared_buffers.s_rrc_calc.asl_at[uc_j] = u_shared_buffers.s_rrc_calc.asl_at[uc_j] +
						(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_j]
						* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
			}

			uc_row = FILTER_LENGTH_H - (uc_step << 1);
			VectorA_fu(sl_lambda2, 1, uc_row, sl_lambdac2);
			for (uc_j = 0; uc_j < (FILTER_LENGTH_H); uc_j++) {
				u_shared_buffers.s_rrc_calc.asl_at[uc_j] = u_shared_buffers.s_rrc_calc.asl_at[uc_j] +
						(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_j]
						* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
			}

			uc_row = FILTER_LENGTH_H - 1 - (uc_step << 1);
			VectorA_fu(sl_lambda2, 1, uc_row, sl_lambdac2);
			for (uc_j = 0; uc_j < (FILTER_LENGTH_H); uc_j++) {
				u_shared_buffers.s_rrc_calc.asl_at[uc_j] = u_shared_buffers.s_rrc_calc.asl_at[uc_j] +
						(((int32_t)u_shared_buffers.s_rrc_calc.ass_A_row[uc_j]
						* (int32_t)u_shared_buffers.s_rrc_calc.ass_a[uc_row - 1]) >> 15);
			}
		}
	}

	if (uc_step == (FILTER_CONFIG_NUM_STEPS - 1)) {
		u_shared_buffers.s_rrc_filter.aus_h_t[FILTER_LENGTH_H - 1] = u_shared_buffers.s_rrc_calc.asl_at[0];
		for (uc_k = 1; uc_k < (FILTER_LENGTH_H); uc_k++) {
			u_shared_buffers.s_rrc_filter.aus_h_t[FILTER_LENGTH_H - 1 - uc_k] = (u_shared_buffers.s_rrc_calc.asl_at[uc_k] >> 1);
			u_shared_buffers.s_rrc_filter.aus_h_t[FILTER_LENGTH_H - 1 + uc_k] = (u_shared_buffers.s_rrc_calc.asl_at[uc_k] >> 1);
		}
	}
}

void atpl250_set_rrc_notch_filter(uint8_t uc_filter_idx, uint8_t uc_step)
{
	if (uc_working_band == WB_CENELEC_A) {
		if (uc_step == 0) {
			/* Prepare for filter configuration */
			/* pplc_if_write32(REG_ATPL250_DEC_RRC_0_32, 0x003E0000); */
			pplc_if_write32(REG_ATPL250_DEC_RRC_0_32, 0x007E0000);

			pplc_if_write8(REG_ATPL250_DEC_RRC_SHIFT_VL8, 0x07);

			pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);  /* SRST_DEC=0 */
		}

		if (uc_step < FILTER_CONFIG_NUM_STEPS) {
			tune(ass_cos_om_0_cenelec_a[uc_filter_idx - 1], uc_step);
		} else {
			/* Last pass. Write filter to HW */
			atpl250_fill_rrc_buffer((uint16_t *)u_shared_buffers.s_rrc_filter.aus_h_t, L1_LOAD_RRC_DEC, L0_LOAD_RRC_DEC);
			pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
			pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
			pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0x7F);  /* LOAD_DEC_RRC=0 */
		}
	}
}

void atpl250_enable_rrc_notch_filter(uint8_t uc_value)
{
	if (uc_working_band == WB_CENELEC_A) {
		if (uc_value) {
			pplc_if_and32(REG_ATPL250_DEC_RRC_0_32, (uint32_t)(~0x00400000UL));
		} else {
			pplc_if_or32(REG_ATPL250_DEC_RRC_0_32, 0x00400000);
		}
	}
}
