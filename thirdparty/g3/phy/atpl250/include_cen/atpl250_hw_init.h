/**
 * \file
 *
 * \brief HEADER. ATPL250 HW Register initialization
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

#ifndef ATPL250_HW_INIT_H_INCLUDED
#define ATPL250_HW_INIT_H_INCLUDED

#define FILTER_CONFIG_NUM_STEPS   8

typedef uint16_t atpl250_RRC_filter_cfg_t[64];

void atpl250_hw_init(uint8_t uc_imp_state);

void atpl250_update_branch_cfg(uint8_t uc_imp_state, uint8_t uc_new_emit_gain);

void atpl250_set_rrc_notch_filter(uint8_t uc_filter_idx, uint8_t uc_step);
void atpl250_enable_rrc_notch_filter(uint8_t uc_value);

/********************************* TRANSMISSION TIMES AND DELAYS *********************************/
/* PLC Config Times */
#define CFG_TXRX_TIME1_US             999 /* 999 us */
#define CFG_TXRX_TIME2_US             999 /* 999 us */
#define CFG_TXRX_TIME3_US             999 /* 999 us */
#define CFG_TXRX_TIME4_US             999 /* 999 us */

#define CFG_TXRX_TIME1              ((CFG_TXRX_TIME1_US * 72) - 1)
#define CFG_TXRX_TIME2              ((CFG_TXRX_TIME2_US * 72) - 1)
#define CFG_TXRX_TIME3              ((CFG_TXRX_TIME3_US * 72) - 1)
#define CFG_TXRX_TIME4              ((CFG_TXRX_TIME4_US * 72) - 1)

#define CFG_TXRX_PLC1_US            1000 /* 1000 us */
#define CFG_TXRX_PLC2_US            1000 /* 1000 us */
#define CFG_TXRX_PLC3_US            1000 /* 1000 us */
#define CFG_TXRX_PLC4_US            1000 /* 1000 us */

#define CFG_TXRX_PLC1               ((CFG_TXRX_PLC1_US * 72) - 1)
#define CFG_TXRX_PLC2               ((CFG_TXRX_PLC2_US * 72) - 1)
#define CFG_TXRX_PLC3               ((CFG_TXRX_PLC3_US * 72) - 1)
#define CFG_TXRX_PLC4               ((CFG_TXRX_PLC4_US * 72) - 1)

#define CFG_TXRX_TIME_OFF1          0x00001C20 /* 100 us */
#define CFG_TXRX_TIME_OFF2          0x00001C20 /* 100 us */
#define CFG_TXRX_TIME_OFF3          0x00001C20 /* 100 us */
#define CFG_TXRX_TIME_OFF4          0x00001C20 /* 100 us */

#define CFG_TXRX_PLC_OFF1           0x00001C21 /* 100 us */
#define CFG_TXRX_PLC_OFF2           0x00001C21 /* 100 us */
#define CFG_TXRX_PLC_OFF3           0x00001C21 /* 100 us */
#define CFG_TXRX_PLC_OFF4           0x00001C21 /* 100 us */

#define CFG_TXRX_PLC_OFF1_US        CFG_TXRX_PLC_OFF1 / 72
#define CFG_TXRX_PLC_OFF2_US        CFG_TXRX_PLC_OFF1 / 72
#define CFG_TXRX_PLC_OFF3_US        CFG_TXRX_PLC_OFF1 / 72
#define CFG_TXRX_PLC_OFF4_US        CFG_TXRX_PLC_OFF1 / 72

#define CFG_END_OF_TX_OFFSET_US     80 /* 80 us ms */

#define RRC_DELAY_US   150
/*************************************************************************************************/

#endif /* ATPL250_HW_INIT_H_INCLUDED */
