/**
 * \file
 *
 * \brief ATPL250 Serial Interface for MAC layer
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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
#include <stdio.h>
#include "conf_usi.h"
#include "serial_if_common.h"
#include "serial_if_mac.h"
#include "serial_if_adp.h"
#if !defined (DISABLE_COORD_COMPILATION)
#include "serial_if_coordinator.h"
#endif
#include "usi.h"
#include "conf_project.h"
#include "AdpApi.h"
#include "mac_wrapper.h"
#include "bs_api.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

static enum ESerialMode e_serial_status = SERIAL_MODE_NOT_INITIALIZED;

/**
 * \brief Display SW version in console
 */
static void _show_version( void )
{
	struct TAdpGetConfirm getConfirm;
	struct TAdpMacGetConfirm x_pib_confirm;

#if defined (CONF_BAND_CENELEC_A)
	printf("G3 Band: CENELEC-A\r\n");
#elif defined (CONF_BAND_CENELEC_B)
	printf("G3 Band: CENELEC-B\r\n");
#elif defined (CONF_BAND_FCC)
	printf("G3 Band: FCC\r\n");
#elif defined (CONF_BAND_ARIB)
	printf("G3 Band: ARIB\r\n");
#else
	printf("G3 Band: CENELEC-A\r\n");
#endif

	AdpGetRequestSync(ADP_IB_SOFT_VERSION, 0, &getConfirm);
	if (getConfirm.m_u8AttributeLength == 6) {
		printf("G3 stack version: %hu.%hu.%hu Date: 20%hu-%hu-%hu\r\n",
				getConfirm.m_au8AttributeValue[0],
				getConfirm.m_au8AttributeValue[1],
				getConfirm.m_au8AttributeValue[2],
				getConfirm.m_au8AttributeValue[3],
				getConfirm.m_au8AttributeValue[4],
				getConfirm.m_au8AttributeValue[5]);
	}

	AdpGetRequestSync(ADP_IB_MANUF_ADP_INTERNAL_VERSION, 0, &getConfirm);
	if (getConfirm.m_u8AttributeLength == 6) {
		printf("ADP version: %hu.%hu.%hu Date: 20%hu-%hu-%hu\r\n",
				getConfirm.m_au8AttributeValue[0],
				getConfirm.m_au8AttributeValue[1],
				getConfirm.m_au8AttributeValue[2],
				getConfirm.m_au8AttributeValue[3],
				getConfirm.m_au8AttributeValue[4],
				getConfirm.m_au8AttributeValue[5]);
	}

	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_MAC_INTERNAL_VERSION, 0, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 6) {
		printf("MAC version: %hu.%hu.%hu Date: 20%hu-%hu-%hu\r\n",
				x_pib_confirm.m_au8AttributeValue[0],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[4],
				x_pib_confirm.m_au8AttributeValue[5]);
	}
	
#ifdef G3_HYBRID_PROFILE
	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_MAC_INTERNAL_VERSION_RF, 0, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 6) {
		printf("MAC RF version: %hu.%hu.%hu Date: 20%hu-%hu-%hu\r\n",
				x_pib_confirm.m_au8AttributeValue[0],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[4],
				x_pib_confirm.m_au8AttributeValue[5]);
	}
#endif

	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_MAC_RT_INTERNAL_VERSION, 0, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 6) {
		printf("MAC RT version: %hu.%hu.%hu Date: 20%hu-%hu-%hu\r\n",
				x_pib_confirm.m_au8AttributeValue[0],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[4],
				x_pib_confirm.m_au8AttributeValue[5]);
	}

	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_PHY_PARAM, MAC_WRP_PHY_PARAM_VERSION, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 4) {
		printf("PHY version: %02x.%02x.%02x.%02x\r\n",
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[0]);
	}

	return;
}

void adp_mac_serial_if_init(void)
{
	/* Set usi callbacks */
	usi_set_callback(PROTOCOL_MAC_G3, serial_if_g3mac_api_parser, MAC_SERIAL_PORT);
	usi_set_callback(PROTOCOL_ADP_G3, serial_if_g3adp_api_parser, ADP_SERIAL_PORT);
#if !defined (DISABLE_COORD_COMPILATION)
	usi_set_callback(PROTOCOL_COORD_G3, serial_if_coordinator_api_parser, COORD_SERIAL_PORT);
#endif
}

void adp_mac_serial_if_process(void)
{
	if (e_serial_status == SERIAL_MODE_ADP) {
		AdpEventHandler();
#if !defined (DISABLE_COORD_COMPILATION)
	} else if (e_serial_status == SERIAL_MODE_COORD) {
		AdpEventHandler();
		bs_process();
#endif
	} else if (e_serial_status == SERIAL_MODE_MAC) {
		MacWrapperEventHandler();
#ifdef G3_HYBRID_PROFILE
		MacWrapperEventHandlerRF();
#endif
	}
}

void adp_mac_serial_if_set_state(enum ESerialMode e_state)
{
	e_serial_status = e_state;

	switch (e_state) {
	case SERIAL_MODE_MAC:
		printf("Serial mode initialized as MAC\r\n");
		break;

	case SERIAL_MODE_ADP:
		printf("Serial mode initialized as ADP\r\n");
		break;

	case SERIAL_MODE_COORD:
		printf("Serial mode initialized as COORD\r\n");
		break;
	
	default:
		printf("Serial mode not set properly\r\n");
		break;
	}
	_show_version();
}

enum ESerialMode  adp_mac_serial_if_get_state(void)
{
	return e_serial_status;
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
