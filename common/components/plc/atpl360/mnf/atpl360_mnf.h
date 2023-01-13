/**
 * \file
 *
 * \brief API manufacture driver for ATPL360 PLC transceiver.
 *
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
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

#ifndef ATPL360_MNF_H_INCLUDED
#define ATPL360_MNF_H_INCLUDED

/* System includes */
#include "compiler.h"
#include "atpl360.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/* ! \name ATPL360 manufacture interface */
/* @{ */
void atpl360_mnf_init(atpl360_hal_wrapper_t *px_hal_wrp);
void atpl360_mnf_write_word(uint32_t ul_address, uint32_t ul_data);
void atpl360_mnf_write_buff(uint32_t ul_address, uint8_t *puc_data, uint16_t us_len);
uint32_t atpl360_mnf_read_word(uint32_t ul_address);
void atpl360_mnf_read_buff(uint32_t ul_address, uint8_t *puc_data, uint16_t us_len);
void atpl360_mnf_write_enc_fuses(uint8_t *puc_data);
void atpl360_mnf_write_tag_fuses(uint8_t *puc_data);
void atpl360_mnf_write_ctrl_fuses(uint32_t ul_ctrl_mask);
void atpl360_mnf_read_enc_fuses(uint8_t *puc_data);
void atpl360_mnf_read_tag_fuses(uint8_t *puc_data);
uint32_t atpl360_mnf_read_ctrl_fuses(void);
bool atpl360_mnf_boot_rdy(void);
void atpl360_mnf_boot_window(void);

/* @} */

/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* ATPL360_MNF_H_INCLUDED */
