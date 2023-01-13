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

#include <stdbool.h>
#include <stdint.h>

/* Phy layer includes */
#include "atpl250.h"
#include "conf_fw.h"
#include "conf_hw.h"
#include "atpl250_common.h"
#include "atpl250_hw_init.h"
#include "string.h"

/* Extern variables needed */
extern uint8_t uc_used_carriers;
extern uint8_t uc_notched_carriers;
extern uint8_t uc_num_symbols_fch;
extern uint8_t uc_legacy_mode;
extern uint8_t auc_masked_carrier_list[];
extern uint8_t auc_unmasked_carrier_list[];
extern uint8_t auc_predistortion[];

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

const uint8_t auc_chirp[576]
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

const atpl250_RRC_filter_cfg_t rrc_coef1_init = {0xFFC4, 0x0002, 0xFF80, 0xFEE6, 0xFFC0, 0x01D0, 0x023C, 0x0076,
						 0x0022, 0x0242, 0x01EC, 0xFD58, 0xFB18, 0xFE6A, 0xFF90, 0xFA64,
						 0xF934, 0x0260, 0x08FE, 0x0350, 0x0042, 0x0C46, 0x1384, 0x028A,
						 0xF0E4, 0xFB12, 0x0488, 0xE106, 0xB478, 0xD612, 0x422E, 0x7FFF,
						 0x422E, 0xD612, 0xB478, 0xE106, 0x0488, 0xFB12, 0xF0E4, 0x028A,
						 0x1384, 0x0C46, 0x0042, 0x0350, 0x08FE, 0x0260, 0xF934, 0xFA64,
						 0xFF90, 0xFE6A, 0xFB18, 0xFD58, 0x01EC, 0x0242, 0x0022, 0x0076,
						 0x023C, 0x01D0, 0xFFC0, 0xFEE6, 0xFF80, 0x0002, 0xFFC4, 0x0000};

const atpl250_RRC_filter_cfg_t rrc_coef2_init = {0x00DD, 0xFFC5, 0xFE66, 0xFC25, 0xF986, 0xF795, 0xF7C2, 0xFB7F,
						 0x03C4, 0x109C, 0x20DC, 0x323C, 0x41C7, 0x4C8D, 0x5068, 0x4C8D,
						 0x41C7, 0x323C, 0x20DC, 0x109C, 0x03C4, 0xFB7F, 0xF7C2, 0xF795,
						 0xF986, 0xFC25, 0xFE66, 0xFFC5, 0x00DD, 0x0000, 0x0000, 0x0000,
						 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const atpl250_RRC_filter_cfg_t filter_60_khz = {0x0124, 0x0122, 0xfff8, 0xfe6a, 0xfddf, 0xff1f, 0x016e, 0x02ef,
						0x0228, 0xff69, 0xfcca, 0xfc8c, 0xff2d, 0x02c2, 0x0463, 0x0288,
						0xfe74, 0xfb5b, 0xfbda, 0xffc0, 0x040e, 0x0547, 0x024b, 0xfd5a,
						0xfa61, 0xfbd3, 0x00a5, 0x050b, 0x057f, 0x0196, 0xfc63, 0x7a08,
						0xfc63, 0x0196, 0x057f, 0x050b, 0x00a5, 0xfbd3, 0xfa61, 0xfd5a,
						0x024b, 0x0547, 0x040e, 0xffc0, 0xfbda, 0xfb5b, 0xfe74, 0x0288,
						0x0463, 0x02c2, 0xff2d, 0xfc8c, 0xfcca, 0xff69, 0x0228, 0x02ef,
						0x016e, 0xff1f, 0xfddf, 0xfe6a, 0xfff8, 0x0122, 0x0124, 0x0000};

const atpl250_RRC_filter_cfg_t rrc_dec_coef1_init = {0xFFC4, 0x0002, 0xFF80, 0xFEE6, 0xFFC0, 0x01D0, 0x023C, 0x0076,
						     0x0022, 0x0242, 0x01EC, 0xFD58, 0xFB18, 0xFE6A, 0xFF90, 0xFA64,
						     0xF934, 0x0260, 0x08FE, 0x0350, 0x0042, 0x0C46, 0x1384, 0x028A,
						     0xF0E4, 0xFB12, 0x0488, 0xE106, 0xB478, 0xD612, 0x422E, 0x7FFF,
						     0x422E, 0xD612, 0xB478, 0xE106, 0x0488, 0xFB12, 0xF0E4, 0x028A,
						     0x1384, 0x0C46, 0x0042, 0x0350, 0x08FE, 0x0260, 0xF934, 0xFA64,
						     0xFF90, 0xFE6A, 0xFB18, 0xFD58, 0x01EC, 0x0242, 0x0022, 0x0076,
						     0x023C, 0x01D0, 0xFFC0, 0xFEE6, 0xFF80, 0x0002, 0xFFC4, 0x0000};

const atpl250_RRC_filter_cfg_t rrc_dec_coef2_init = {0xFFC5, 0xFE66, 0xFC25, 0xF986, 0xF795, 0xF7C2, 0xFB7F, 0x03C4,
						     0x109C, 0x20DC, 0x323C, 0x41C7, 0x4C8D, 0x5068, 0x4C8D, 0x41C7,
						     0x323C, 0x20DC, 0x109C, 0x03C4, 0xFB7F, 0xF7C2, 0xF795, 0xF986,
						     0xFC25, 0xFE66, 0xFFC5, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
						     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const int16_t ass_h_in[32]
	= {(int16_t)0x7A03, (int16_t)0xFCAD, (int16_t)0x0248, (int16_t)0x05D3, (int16_t)0x042C, (int16_t)0xFEDD, (int16_t)0xFAA8, (int16_t)0xFB3F,
	   (int16_t)0xFFFE, (int16_t)0x049A, (int16_t)0x0506, (int16_t)0x010C, (int16_t)0xFC55, (int16_t)0xFB09, (int16_t)0xFE1B, (int16_t)0x02A3,
	   (int16_t)0x049A, (int16_t)0x0279, (int16_t)0xFE64, (int16_t)0xFC06, (int16_t)0xFD42, (int16_t)0x00B2, (int16_t)0x032C, (int16_t)0x02B0,
	   (int16_t)0x0005, (int16_t)0xFDB8, (int16_t)0xFDAA, (int16_t)0xFF89, (int16_t)0x016A, (int16_t)0x01BC, (int16_t)0x0099, (int16_t)0xFF55};

const int16_t ss_cos_om_m = 18205;         /* round(2^15*cos(2*pi*fm/fs)); (fm/fs)=40/256 */
const int16_t ass_cos_om_0[100]
	= {32767, 32758, 32729, 32679, 32610, 32522, 32413, 32286, 32138, 31972, 31786, 31581, 31357, 31114, 30853, 30572, 30274, 29957, 29622, 29269,
	   28899, 28511, 28106, 27684, 27246, 26791, 26320, 25833, 25330, 24812, 24279, 23732, 23170, 22595, 22006, 21403, 20788, 20160, 19520,
	   18868, 18205, 17531, 16846, 16151, 15447, 14733, 14010, 13279, 12540, 11793, 11039, 10279, 9512, 8740, 7962, 7180, 6393, 5602, 4808,
	   4011, 3212, 2411, 1608, 804, 0, -804, -1608, -2411, -3212, -4011, -4808, -5602, -6393, -7180, -7962, -8740, -9512, -10279, -11039,
	   -11793, -12540, -13279, -14010, -14733, -15447, -16151, -16846, -17531, -18205, -18868, -19520, -20160, -20788, -21403, -22006, -22595,
	   -23170, -23732, -24279, -24812};
/* round(2^15*cos(2*pi*f_0/fs)); (f_0/fs)=carrier_f_0/256 */

#if (MOSFET_POWER_AMPLIFIER == FDC6420C)
/* FDC6420C configuration */
const struct impedance_params_type impedance_params[NUM_IMPEDANCE_STATES] = {
	{0x00002120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0x50},
	{0x00002120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0x5F},
	{0x00002120, 0x00001F3F, 0x00001F1F, 0x0F, 0x03, 0x55, 0x5F}
};
#else
/* NTGD4167C configuration */
const struct impedance_params_type impedance_params[NUM_IMPEDANCE_STATES] = {
	{0x00002110, 0x00003F3F, 0x00003F0F, 0x0F, 0x03, 0x55, 0x50},
	{0x00002110, 0x00003F3F, 0x00003F0F, 0x0F, 0x03, 0x55, 0x5F},
	{0x00002110, 0x00003F3F, 0x00003F0F, 0x0F, 0x03, 0x55, 0x5F}
};
#endif

const uint8_t auc_rep[8] = {0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00};

const uint8_t auc_th[12] = {0x10, 0x00, 0x70, 0x00, 0x00, 0x00, 0x57, 0xFF, 0x10, 0x01, 0x10, 0x20};

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

/**
 * \brief Update value of uc_num_sym_block_demod_fch
 *
 * \param uc_legacy_mode   legacy mode indicator
 *
 */
static void  _update_num_sym_block_fch(void)
{
	uc_num_sym_block_demod_fch = 7;
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

	for (us_i = 0; us_i < PROTOCOL_CARRIERS * 4; us_i = us_i + 4) {
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
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 0) + LAST_CARRIER); /* Avoid overflow to 1st symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((0 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 1) + LAST_CARRIER); /* Avoid overflow to 2nd symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((1 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + LAST_CARRIER); /* Avoid overflow to 3rd symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((2 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + LAST_CARRIER); /* Avoid overflow to 4th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((3 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + LAST_CARRIER); /* Avoid overflow to 5th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((4 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + LAST_CARRIER); /* Avoid overflow to 6th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((5 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + LAST_CARRIER); /* Avoid overflow to 7th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((6 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((7 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
}

/**
 * \brief Configure emition branch depending on impedance detected
 *
 */
void atpl250_update_branch_cfg(uint8_t uc_imp_state, uint8_t uc_new_emit_gain)
{
	LOG_PHY(("ImpState %.3s EmitGain 0x%02X\r\n", &imp_state_to_string[uc_imp_state][0], uc_new_emit_gain));
	pplc_if_write8(REG_ATPL250_TX_CONF_VL8, impedance_params[uc_imp_state].tx_conf_vl8);
	pplc_if_write8(REG_ATPL250_EMIT_CONFIG_VL8, impedance_params[uc_imp_state].emit_conf_vl8);
	pplc_if_write8(REG_ATPL250_EMIT_CONFIG_L8, impedance_params[uc_imp_state].emit_conf_l8);
	pplc_if_write32(REG_ATPL250_EMIT_NP_DELAY_32, impedance_params[uc_imp_state].emit_np_delay);
	pplc_if_write32(REG_ATPL250_EMIT_ON_ACTIVE_32, impedance_params[uc_imp_state].emit_on_active);
	pplc_if_write32(REG_ATPL250_EMIT_OFF_ACTIVE_32, impedance_params[uc_imp_state].emit_off_active);
	pplc_if_write8(REG_ATPL250_EMIT_GAIN_VL8, uc_new_emit_gain);
	pplc_if_write8(REG_ATPL250_EMIT_BIT_FLIPPING_VL8, impedance_params[uc_imp_state].bf_vl8);
}

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

	/* Changed to take SYNCM 64 ahead */
	pplc_if_write8(REG_ATPL250_TXRXB_CTL_VL8, 0x4B); /* CP_EXCLUSIVO=75, 64 samples from the 1/2 SYNCM + 11 of the cp of the first FCH symbol*/

	/* NOISE_SYMBOLS configured to higher value to avoid all symbols passed after error */
	pplc_if_write8(REG_ATPL250_TXRXB_STATE_L8, 0x80);
	/* Block AGC on noise capture */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VH8, 0x10);

	pplc_if_write32(REG_ATPL250_TXRXB_SYM_CFG_32, 0x0000081E); /* cyclic_prefix1=0; overlap1=0; cyclic_prefix2=30; overlap2=8 */
	pplc_if_write8(REG_ATPL250_TXRXB_CFG_L8, 0x02); /* Go back after reading the SYNCM (from its first sample) */
	pplc_if_write32(REG_ATPL250_TXRXB_OFFSETS_32, 0x000000C0); /* Go back 192 samples from the first sample of the 1/2 SYNCM. Before it was 139 ahead */

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
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA7_32, 9, (uint8_t *)auc_chirp, 288);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA7_32, 9, (uint8_t *)(&auc_chirp[288]), 288);
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
	pplc_if_write_buf(REG_ATPL250_SYNC_THRESHOLD_32, (uint8_t *)auc_th, 12); /* CORR_LENGTH=128/256; THRESHOLD=20000; THRESHOLD_HALF=12000;
	                                                                          * PEAK_DISTANTE=128/256; CONSEC_PEAK=1; CONSEC_WINDOW=16; WINDOW_WIDTH=32 */

	atpl250_sync_release_rst();

	pplc_if_write16(REG_ATPL250_SYNCM_VALUE_H16, (uc_used_carriers * 15) / 2); /* (carriers) * 15 (soft value for 1) / 2 */
	pplc_if_write8(REG_ATPL250_SYNCM_CTL_H8, uc_used_carriers);
	/* Set first carrier for SyncM detector (depends on notching) */
	pplc_if_write8(REG_ATPL250_SYNCM_CTL_VL8, FIRST_CARRIER + auc_unmasked_carrier_list[0]);

	/* Configure Emit Control */
	atpl250_emit_ctl_push_rst();
	pplc_if_write32(REG_ATPL250_TXRX_TIME_ON1_32, CFG_TXRX_TIME1);
	pplc_if_write32(REG_ATPL250_PLC_TIME_ON1_32, CFG_TXRX_PLC1);
	pplc_if_write32(REG_ATPL250_TXRX_TIME_OFF1_32, CFG_TXRX_TIME_OFF1);
	pplc_if_write32(REG_ATPL250_PLC_TIME_OFF1_32, CFG_TXRX_PLC_OFF1);
	atpl250_emit_ctl_release_rst();

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
	atpl250_fill_rrc_buffer((uint16_t *)rrc_coef1_init, L1_LOAD_RRC_INT_1, L0_LOAD_RRC_INT_1 );
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
	pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFE);
	pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);
	atpl250_fill_rrc_buffer((uint16_t *)rrc_coef2_init, L1_LOAD_RRC_INT_2, L0_LOAD_RRC_INT_2 );
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
		atpl250_update_branch_cfg(uc_imp_state, EMIT_GAIN_HI);
		break;

	case LO_STATE:
		atpl250_update_branch_cfg(uc_imp_state, EMIT_GAIN_LO);
		break;

	case VLO_STATE:
		atpl250_update_branch_cfg(uc_imp_state, EMIT_GAIN_VLO);
		break;

	default:
		atpl250_update_branch_cfg(uc_imp_state, EMIT_GAIN_HI);
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
	atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef1_init, L1_LOAD_RRC_DEC_1, L0_LOAD_RRC_DEC_1);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
	pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
	pplc_if_and8(REG_ATPL250_LOAD_CFG_H8, 0xFD);

	pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);
	atpl250_fill_rrc_buffer((uint16_t *)rrc_dec_coef2_init, L1_LOAD_RRC_DEC_2, L0_LOAD_RRC_DEC_2);
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

	/* Configure RMS_CALC time*/
	pplc_if_write16(REG_ATPL250_RMS_CALC_CFG1_L16, 0x0A00); /* T2_WAIT time */
	pplc_if_write16(REG_ATPL250_RMS_CALC_CFG1_H16, 0x003C); /* "000" & CHANNEL_N & T1_WAIT(11:0) time Valid in CENELEC, FCC and ARIB*/

	/* Enable START, SPI, BER, RX_ERROR, TX_SOLAPE, TX_CD, TX_END, TX_START, VTB, NOPEAK2, PEAK2, PEAK1, IOB, NOISE_START, NOISE_ERROR */
	pplc_if_write32(REG_ATPL250_INT_MASK_32, 0xDDFFFF4F);

	/*	pplc_if_write16(REG_ATPL250_EMIT_FREC_DELAY_H16, 0x0006); */
	/* Configure SYNCM detection */
	pplc_if_or8(REG_ATPL250_SYNCM_CTL_L8, 0x80); /* Enable SYNCM */

	/* Initialization BER */
	pplc_if_write16(REG_ATPL250_BER_PERIPH_CFG3_L16, 0x1F13); /* FEM=0x1F; FEM_CR=0x13 */
	pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG4_32, 0x281A1B12); /* FEM_HIGH=0x28; FEM_HIGH_CR=0x1A; FEM_LOW=0x1B; FEM_LOW_CR=0x12; */
	pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG5_32, 0x01430025); /* FRONT_8PSK_MINUS1=0x143;  FRONT_8PSK=0x025 */
	pplc_if_write8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x16); /* INT_BER=0; MODUL_PY_USED=1 (BPSK); MODUL_HD_USED=1 (BPSK); MODE_BER=2 (ONLY PAYLOAD/FCH) */

	pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG10_32, 0xA5D2C3C3); /* coherent values for 1's and 0's */
	pplc_if_write16(REG_ATPL250_BER_PERIPH_CFG11_L16, 0x96C3);

	atpl250_set_ber_fch_diff();

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
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, RX_FFT_SHIFT);

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
				u_shared_buffers.s_rrc_calc.ass_a[0] = ass_h_in[0];
			} else {
				u_shared_buffers.s_rrc_calc.ass_a[uc_k] = (ass_h_in[uc_k] << 1);
			}
		}
	}

	/* Obtain 4 coeffs, depending on step */
	if (ss_cos_om_m > ss_cos_om_0) {
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
		if (ss_cos_om_m == ss_cos_om_0) {
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
	if (uc_step == 0) {
		/* Prepare for filter configuration */
		/* pplc_if_write32(REG_ATPL250_DEC_RRC_0_32, 0x003E0000); */
		pplc_if_write32(REG_ATPL250_DEC_RRC_0_32, 0x007E0000);

		pplc_if_write8(REG_ATPL250_DEC_RRC_SHIFT_VL8, 0x07);

		pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);  /* SRST_DEC=0 */
	}

	if (uc_step < FILTER_CONFIG_NUM_STEPS) {
		tune(ass_cos_om_0[uc_filter_idx - 1], uc_step);
	} else {
		/* Last pass. Write filter to HW */
		atpl250_fill_rrc_buffer((uint16_t *)u_shared_buffers.s_rrc_filter.aus_h_t, L1_LOAD_RRC_DEC, L0_LOAD_RRC_DEC);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer, 256);
		pplc_if_write_rep(REG_ATPL250_LOAD_DATA0_32, 2, (uint8_t *)(&u_shared_buffers.s_rrc_filter.auc_rrc_write_buffer[256]), 256);
		pplc_if_and8(REG_ATPL250_LOAD_CFG_VL8, 0x7F);  /* LOAD_DEC_RRC=0 */
	}
}

void atpl250_enable_rrc_notch_filter(uint8_t uc_value)
{
	if (uc_value) {
		pplc_if_and32(REG_ATPL250_DEC_RRC_0_32, (uint32_t)(~0x00400000UL));
	} else {
		pplc_if_or32(REG_ATPL250_DEC_RRC_0_32, 0x00400000);
	}
}
