/**
 * \file
 *
 * \brief Example for a IEC 62056 Server stack - implementation of HDLC layer
 *              functionalities
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#ifndef __HDLC_TYPES_H__
#define __HDLC_TYPES_H__


/*** Includes **************************************************************/
#include <stdio.h>
#include <string.h>
	 
#include "serial_buffer.h"

/* #define DUMP_HDLC */

/*** Constants *************************************************************/
#define HDLC_START_END_FLAG 			0x7E
#define HDLC_FRAME_FORMAT_WITHOUT_SEGMENTATION 	0xA0
#define HDLC_ADDR_METER 			0x03
#define HDLC_ADDR_MODEM 			0xFD
#define HDLC_CONTROL 				0x13

#define HDLC_LLC_DESTINATION_LSAP 		0xE6
#define HDLC_LLC_SOURCE_COMMAND_LSAP  		0xE6
#define HDLC_LLC_SOURCE_RESPONSE_LSAP 		0xE7
#define HDLC_LLC_CONTROL 			0x00

#define MAX_LENGTH_HDLC_PDU       		SERIAL_BUF_SIZE

#define HDLC_INTERFRAME_TIMEOUT			200

/* Maximum length of DLMS APDU over serial_if */
#define OVERHEAD_HDLC 			14
#define DLMS_BUF_SIZE   (SERIAL_BUF_SIZE - OVERHEAD_HDLC)

/*** Structs *************************************************************/
struct dlms_msg
{
          uint8_t todo;          /* Flag to indicate that msg has to be served */
          uint8_t buf[DLMS_BUF_SIZE];  /* Serial data buffer */
          uint16_t length;      /* Length of the data buffer */
};

/*** Functions prototypes **************************************************/
void HDLC_iframe_tout_init(uint16_t *ptr_HDLC_iframe_tout);
void hdlc_init(void);
uint8_t hdlc_RxProcess(struct dlms_msg *ptr_rx_dlms_msg, struct serial_msg *ptr_rx_serial);
uint8_t hdlc_TxProcess(struct serial_msg *ptr_tx_serial, struct dlms_msg *ptr_tx_dlms_msg);

#endif


