/**
 * \file
 *
 * \brief ATPL250 Sniffer Interface for Physical layer
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

#ifndef SNIFFER_IF_H_INCLUDED
#define SNIFFER_IF_H_INCLUDED

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \ingroup phy_plc_group
 * \defgroup sniffer_plc_group PLC PHY Sniffer
 *
 * This module provides configuration and utils for the sniffer in the PLC PHY layer.
 *
 * @{
 */

/* ! \name Task priorities and stack definitions */
/* @{ */
/* #define TASK_SNIFFER_IF_GET_RX_PRIO           (tskIDLE_PRIORITY + 1) */
/* #define TASK_SNIFFER_IF_GET_RX_STACK          (configMINIMAL_STACK_SIZE * 1) */
/* @} */

/* ! \name SNIFFER version */
/* @{ */
#define SNIFFER_ATPL250 0x02
#define SNIFFER_VERSION 0x02
/* @} */

#define TIME_IN_TICS(x)                 (x / 10)
/* @} */

/* ! \name Serial interface commands identifiers */
/* @{ */

#define SERIAL_IF_PHY_MESSAGE_G3_SNIFFER      0x23  /* !< Sniffer Frame Command */

/* ! \name Sniffer interface commands identifiers */
/* @{ */

#define SNIFFER_IF_PHY_COMMAND_G3_VERSION                 0x00  /* !< TO DO */
#define SNIFFER_IF_PHY_G3_SET_TONE_MASK                   1     /* SET G3-PHY Tone Mask */
/* @} */

#define TONE_MASK_SIZE                                    9     /* Byte size of the bitfield containing the tone mask */

/* @} */

/* ! \name Message Structure to communicate with USI layer */
/* @{ */
typedef struct {
	uint8_t uc_protocol_type; /* !<  Protocol Type */
	uint8_t *ptr_buf;      /* !<  Pointer to data buffer */
	uint16_t us_len;       /* !<  Length of data */
} x_usi_sniffer_cmd_params;
/* @} */

/* ! \name PHY sniffer interface */
/* @{ */
void sniffer_if_init(void);
uint8_t serial_if_sniffer_g3_api_parser(uint8_t *puc_rx_msg, uint16_t us_len);

/* void sniffer_if_process(void); */
/* @} */

/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* SNIFFER_IF_H_INCLUDED */
