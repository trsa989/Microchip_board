/**
 * \file
 *
 * \brief ATPL250 Serial Interface for MAC layer
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

/* System includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Serial interface */
#include "hal/hal.h"
#include "mac_wrapper.h"
#include "AdpApi.h"
#include "serial_if_mac.h"
#include "serial_if_common.h"
#include "serial_if_mib_common.h"
#include "usi.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
	#endif
/**INDENT-ON**/
/* / @endcond */

/* Default empty USI interface*/

/*
 * extern int8_t usi_send_cmd(void *msg);
 * int8_t Dummy_serial_send_cmd(void *msg);
 *
 * #ifdef __GNUC__
 * int8_t usi_send_cmd ( void *msg ) __attribute__ ((weak, alias("Dummy_serial_send_cmd")));
 * #endif
 *
 * #ifdef __ICCARM__
 * #pragma weak usi_send_cmd=Dummy_serial_send_cmd
 * #endif
 */

/* ! \name Data structure to communicate with USI layer */
/* @{ */
static x_usi_serial_cmd_params_t x_mac_serial_msg;
/* @} */

static uint8_t uc_serial_rsp_buf[2048]; /* !<  Response working buffer */
static uint8_t auc_ext_address_mac[8]; /* EUI64 */

static struct TMacWrpSetRequest set_params;
static struct TMacWrpGetRequest get_params;

/* Known message ids */
enum ESerialMessageId {
	/* Generic messages */
	SERIAL_MSG_STATUS = 0,

	/* MAC ACCESS */
	SERIAL_MSG_MAC_REQUEST_MESSAGES_BEGIN = 50,
	SERIAL_MSG_MAC_INITIALIZE = SERIAL_MSG_MAC_REQUEST_MESSAGES_BEGIN,
	SERIAL_MSG_MAC_DATA_REQUEST,
	SERIAL_MSG_MAC_GET_REQUEST,
	SERIAL_MSG_MAC_SET_REQUEST,
	SERIAL_MSG_MAC_RESET_REQUEST,
	SERIAL_MSG_MAC_SCAN_REQUEST,
	SERIAL_MSG_MAC_START_REQUEST,
	SERIAL_MSG_MAC_REQUEST_MESSAGES_END = SERIAL_MSG_MAC_START_REQUEST,

	SERIAL_MSG_MAC_DATA_CONFIRM = 60,
	SERIAL_MSG_MAC_DATA_INDICATION,
	SERIAL_MSG_MAC_GET_CONFIRM,
	SERIAL_MSG_MAC_SET_CONFIRM,
	SERIAL_MSG_MAC_RESET_CONFIRM,
	SERIAL_MSG_MAC_SCAN_CONFIRM,
	SERIAL_MSG_MAC_BEACON_NOTIFY,
	SERIAL_MSG_MAC_START_CONFIRM,
	SERIAL_MSG_MAC_COMM_STATUS_INDICATION,
#ifdef G3_HYBRID_PROFILE
	SERIAL_MSG_MAC_SNIFFER_INDICATION,
	
	/* Following identifiers overlap with ADP ones, but as they have different ProtocolID, there is no issue */
	SERIAL_MSG_MAC_REQUEST_MESSAGES_BEGIN_RF = 30,
	SERIAL_MSG_MAC_INITIALIZE_RF = SERIAL_MSG_MAC_REQUEST_MESSAGES_BEGIN_RF,
	SERIAL_MSG_MAC_DATA_REQUEST_RF,
	SERIAL_MSG_MAC_GET_REQUEST_RF,
	SERIAL_MSG_MAC_SET_REQUEST_RF,
	SERIAL_MSG_MAC_RESET_REQUEST_RF,
	SERIAL_MSG_MAC_SCAN_REQUEST_RF,
	SERIAL_MSG_MAC_START_REQUEST_RF,
	SERIAL_MSG_MAC_REQUEST_MESSAGES_END_RF = SERIAL_MSG_MAC_START_REQUEST_RF,

	SERIAL_MSG_MAC_DATA_CONFIRM_RF = 40,
	SERIAL_MSG_MAC_DATA_INDICATION_RF,
	SERIAL_MSG_MAC_GET_CONFIRM_RF,
	SERIAL_MSG_MAC_SET_CONFIRM_RF,
	SERIAL_MSG_MAC_RESET_CONFIRM_RF,
	SERIAL_MSG_MAC_SCAN_CONFIRM_RF,
	SERIAL_MSG_MAC_BEACON_NOTIFY_RF,
	SERIAL_MSG_MAC_START_CONFIRM_RF,
	SERIAL_MSG_MAC_COMM_STATUS_INDICATION_RF,
	SERIAL_MSG_MAC_SNIFFER_INDICATION_RF
#else
	SERIAL_MSG_MAC_SNIFFER_INDICATION
#endif
};

/* Status codes related to HostInterface processing */
enum ESerialStatus {
	SERIAL_STATUS_SUCCESS = 0,
	SERIAL_STATUS_NOT_ALLOWED,
	SERIAL_STATUS_UNKNOWN_COMMAND,
	SERIAL_STATUS_INVALID_PARAMETER
};
/* typedef enum ESerialStatus (*MsgFunct)(const struct TSerialMessage *pMessage); */

/* Flag to check whether ADP is already disabled (initialized to NULL) if 2 MACs are initialized */
static bool b_adp_already_disabled = false;

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MsgStatus(enum ESerialStatus status, uint8_t uc_serial_if_cmd)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_STATUS;
	uc_serial_rsp_buf[us_serial_response_len++] = status;
	uc_serial_rsp_buf[us_serial_response_len++] = uc_serial_if_cmd;
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_DataConfirm(struct TMacWrpDataConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_DATA_CONFIRM;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_u8MsduHandle;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eStatus;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nTimestamp & 0xFF);
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_DataIndication(struct TMacWrpDataIndication *pParameters)
{
	uint16_t us_serial_response_len;

	uint8_t uc_src_addr_len = 0;
	uint8_t uc_dst_addr_len = 0;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_DATA_INDICATION;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nSrcPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_nSrcPanId & 0xFF;

	if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_src_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_SrcAddr.m_nShortAddress;
	} else if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_src_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_SrcAddr.m_ExtendedAddress.m_au8Address[0], uc_src_addr_len);
		us_serial_response_len += uc_src_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nDstPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_nDstPanId & 0xFF;

	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_dst_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_DstAddr.m_nShortAddress & 0xFF;
	} else if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_dst_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0], uc_dst_addr_len);
		us_serial_response_len += uc_dst_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8MpduLinkQuality;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8Dsn;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nTimestamp & 0xFF);

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eSecurityLevel;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8KeyIndex;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eQualityOfService;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_RecvToneMap.m_au8Tm[0], 3 );
	us_serial_response_len += 3;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_ComputedToneMap.m_au8Tm[0], 3 );
	us_serial_response_len += 3;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength & 0xFF);

	memcpy(&uc_serial_rsp_buf[us_serial_response_len], pParameters->m_pMsdu, pParameters->m_u16MsduLength);
	us_serial_response_len += pParameters->m_u16MsduLength;

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_MacSnifferIndication(struct TMacWrpSnifferIndication *pParameters)
{
	/* TEST: Send sniffer indication as data indication */
	uint16_t us_serial_response_len;

	uint8_t uc_src_addr_len;
	uint8_t uc_dst_addr_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_SNIFFER_INDICATION;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8FrameType;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nSrcPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_nSrcPanId & 0xFF;

	if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_src_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_src_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_SrcAddr.m_ExtendedAddress.m_au8Address[0], uc_src_addr_len);
		us_serial_response_len += uc_src_addr_len;
	} else {
		uc_src_addr_len = 0;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nDstPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nDstPanId & 0xFF);

	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_dst_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_dst_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0], uc_dst_addr_len);
		us_serial_response_len += uc_dst_addr_len;
	} else {
		uc_dst_addr_len = 0;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8MpduLinkQuality;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8Dsn;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nTimestamp & 0xFF);

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eSecurityLevel;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8KeyIndex;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eQualityOfService;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_RecvToneMap.m_au8Tm[0], 3 );
	us_serial_response_len += 3;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_ComputedToneMap.m_au8Tm[0], 3 );
	us_serial_response_len += 3;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength & 0xFF);

	memcpy(&uc_serial_rsp_buf[us_serial_response_len], pParameters->m_pMsdu, pParameters->m_u16MsduLength);
	us_serial_response_len += pParameters->m_u16MsduLength;

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_GetConfirm(struct TMacWrpGetConfirm *pParameters)
{
	uint8_t uc_serial_response_len;

	uc_serial_response_len = 0;
	uc_serial_rsp_buf[uc_serial_response_len++] = SERIAL_MSG_MAC_GET_CONFIRM;

	uc_serial_response_len += process_MIB_get_confirm(&uc_serial_rsp_buf[uc_serial_response_len], pParameters);

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = uc_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_SetConfirm(struct TMacWrpSetConfirm *pParameters)
{
	uint8_t uc_serial_response_len;

	uc_serial_response_len = 0;
	uc_serial_rsp_buf[uc_serial_response_len++] = SERIAL_MSG_MAC_SET_CONFIRM;

	uc_serial_response_len += process_MIB_set_confirm(&uc_serial_rsp_buf[uc_serial_response_len], pParameters);

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = uc_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_ResetConfirm(struct TMacWrpResetConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_RESET_CONFIRM;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_BeaconNotify(struct TMacWrpBeaconNotifyIndication *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_BEACON_NOTIFY;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nPanId & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_u8LinkQuality);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nLbaAddress >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nLbaAddress & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_u16RcCoord >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_u16RcCoord & 0xFF);
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_ScanConfirm(struct TMacWrpScanConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_SCAN_CONFIRM;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_StartConfirm(struct TMacWrpStartConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_START_CONFIRM;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_CommStatusIndication(struct TMacWrpCommStatusIndication *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uint8_t uc_src_addr_len = 0;
	uint8_t uc_dst_addr_len = 0;

	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_COMM_STATUS_INDICATION;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nPanId & 0xFF);
	if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_src_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_src_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_SrcAddr.m_ExtendedAddress.m_au8Address[0], uc_src_addr_len);
		us_serial_response_len += uc_src_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_dst_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_dst_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0], uc_dst_addr_len);
		us_serial_response_len += uc_dst_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eSecurityLevel;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_u8KeyIndex;

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacInitialize(const uint8_t *puc_msg_content)
{
	struct TMacWrpPibValue pibValue;
	enum ESerialStatus status = SERIAL_STATUS_INVALID_PARAMETER;

	uint8_t u8Band = *puc_msg_content;

	/* disable ADP */
	if (!b_adp_already_disabled) {
		AdpInitialize(NULL, (enum TAdpBand)0);
		b_adp_already_disabled = true;
		/* Disabling ADP creates a temporary callback setting that breaks correct configuration
		in case it is called twice when 2 MACs are initilized (e.g PLC and RF) */
	}

	struct TMacWrpNotifications notifications;

	notifications.m_MacWrpDataConfirm = MacNotification_DataConfirm;
	notifications.m_MacWrpDataIndication = MacNotification_DataIndication;
	notifications.m_MacWrpGetConfirm = MacNotification_GetConfirm;
	notifications.m_MacWrpSetConfirm = MacNotification_SetConfirm;
	notifications.m_MacWrpResetConfirm = MacNotification_ResetConfirm;
	notifications.m_MacWrpBeaconNotifyIndication = MacNotification_BeaconNotify;
	notifications.m_MacWrpScanConfirm = MacNotification_ScanConfirm;
	notifications.m_MacWrpStartConfirm = MacNotification_StartConfirm;
	notifications.m_MacWrpCommStatusIndication = MacNotification_CommStatusIndication;
	notifications.m_MacWrpSnifferIndication = MacNotification_MacSnifferIndication;

	MacWrapperInitialize(&notifications, u8Band);
	
	platform_init_eui64(auc_ext_address_mac);
	memcpy(pibValue.m_au8Value, auc_ext_address_mac, sizeof(auc_ext_address_mac));
	pibValue.m_u8Length = sizeof(auc_ext_address_mac);
	MacWrapperMlmeSetRequestSync(MAC_WRP_PIB_MANUF_EXTENDED_ADDRESS, 0, &pibValue);

	adp_mac_serial_if_set_state(SERIAL_MODE_MAC);
	status = SERIAL_STATUS_SUCCESS;

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacDataRequest(const uint8_t *puc_msg_content)
{
	uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpDataRequest parameters;
		uint8_t u8SrcAddrLen;
		uint8_t u8DstAddrLen;

		parameters.m_u8MsduHandle = (uint8_t)*puc_buffer++;
		parameters.m_eSecurityLevel =  (enum EMacWrpSecurityLevel)*puc_buffer++;
		parameters.m_u8KeyIndex = (uint8_t)*puc_buffer++;
		parameters.m_eQualityOfService = (enum EMacWrpQualityOfService)*puc_buffer++;
		parameters.m_u8TxOptions = (uint8_t)*puc_buffer++;
		parameters.m_nDstPanId = ((TMacWrpPanId) * puc_buffer++) << 8;
		parameters.m_nDstPanId +=  (TMacWrpPanId) * puc_buffer++;
		u8SrcAddrLen = (uint8_t)*puc_buffer++;
		if (u8SrcAddrLen == 2) {
			parameters.m_eSrcAddrMode = MAC_WRP_ADDRESS_MODE_SHORT;
		} else if (u8SrcAddrLen == 8) {
			parameters.m_eSrcAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED;
		} else {
			return SERIAL_STATUS_INVALID_PARAMETER;
		}

		u8DstAddrLen = (uint8_t)*puc_buffer++;
		if (u8DstAddrLen == 2) {
			parameters.m_DstAddr.m_eAddrMode = MAC_WRP_ADDRESS_MODE_SHORT;
			parameters.m_DstAddr.m_nShortAddress = ((TMacWrpShortAddress) * puc_buffer++) << 8;
			parameters.m_DstAddr.m_nShortAddress +=  (TMacWrpShortAddress) * puc_buffer++;
		} else if (u8DstAddrLen == 8) {
			parameters.m_DstAddr.m_eAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED;
			memcpy(&parameters.m_DstAddr.m_ExtendedAddress, puc_buffer, u8DstAddrLen);
			puc_buffer +=  u8DstAddrLen;
		} else {
			return SERIAL_STATUS_INVALID_PARAMETER;
		}

		parameters.m_u16MsduLength = ((uint16_t)*puc_buffer++) << 8;
		parameters.m_u16MsduLength +=  (uint16_t)*puc_buffer++;
		parameters.m_pMsdu = puc_buffer;

		MacWrapperMcpsDataRequest(&parameters);

		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacGetRequest(const uint8_t *puc_msg_content)
{
	enum ESerialStatus status = SERIAL_STATUS_INVALID_PARAMETER;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
		process_MIB_get_request(puc_buffer, &get_params);
		MacWrapperMlmeGetRequest(&get_params);
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacSetRequest(const uint8_t *puc_msg_content)
{
	enum ESerialStatus status = SERIAL_STATUS_INVALID_PARAMETER;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
		process_MIB_set_request(puc_buffer, &set_params);
		MacWrapperMlmeSetRequest(&set_params);
		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacResetRequest(const uint8_t *puc_msg_content)
{
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpResetRequest parameters;
		parameters.m_bSetDefaultPib = (bool) * puc_msg_content;
		MacWrapperMlmeResetRequest(&parameters);

		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacScanRequest(const uint8_t *puc_msg_content)
{
	uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpScanRequest parameters;
		parameters.m_u16ScanDuration = ((uint16_t)*puc_buffer++) << 8;
		parameters.m_u16ScanDuration +=  (uint16_t)*puc_buffer;

		MacWrapperMlmeScanRequest(&parameters);
		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacStartRequest(const uint8_t *puc_msg_content)
{
	uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpStartRequest parameters;
		parameters.m_nPanId = ((uint16_t)*puc_buffer++) << 8;
		parameters.m_nPanId +=  (uint16_t)*puc_buffer;

		MacWrapperMlmeStartRequest(&parameters);

		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

#ifdef G3_HYBRID_PROFILE
/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_DataConfirmRF(struct TMacWrpDataConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_DATA_CONFIRM_RF;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_u8MsduHandle;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eStatus;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nTimestamp & 0xFF);
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_DataIndicationRF(struct TMacWrpDataIndication *pParameters)
{
	uint16_t us_serial_response_len;

	uint8_t uc_src_addr_len = 0;
	uint8_t uc_dst_addr_len = 0;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_DATA_INDICATION_RF;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nSrcPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_nSrcPanId & 0xFF;

	if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_src_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_SrcAddr.m_nShortAddress;
	} else if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_src_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_SrcAddr.m_ExtendedAddress.m_au8Address[0], uc_src_addr_len);
		us_serial_response_len += uc_src_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nDstPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_nDstPanId & 0xFF;

	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_dst_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_DstAddr.m_nShortAddress & 0xFF;
	} else if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_dst_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0], uc_dst_addr_len);
		us_serial_response_len += uc_dst_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8MpduLinkQuality;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8Dsn;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nTimestamp & 0xFF);

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eSecurityLevel;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8KeyIndex;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eQualityOfService;

	/* These values have no sense in RF but are returned with fixed values from RF MAC */
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_RecvToneMap.m_au8Tm[0], 3 );
	us_serial_response_len += 3;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_ComputedToneMap.m_au8Tm[0], 3 );
	us_serial_response_len += 3;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength & 0xFF);

	memcpy(&uc_serial_rsp_buf[us_serial_response_len], pParameters->m_pMsdu, pParameters->m_u16MsduLength);
	us_serial_response_len += pParameters->m_u16MsduLength;

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_MacSnifferIndicationRF(struct TMacWrpSnifferIndication *pParameters)
{
	/* TEST: Send sniffer indication as data indication */
	uint16_t us_serial_response_len;

	uint8_t uc_src_addr_len;
	uint8_t uc_dst_addr_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_SNIFFER_INDICATION_RF;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8FrameType;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nSrcPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_nSrcPanId & 0xFF;

	if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_src_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_src_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_SrcAddr.m_ExtendedAddress.m_au8Address[0], uc_src_addr_len);
		us_serial_response_len += uc_src_addr_len;
	} else {
		uc_src_addr_len = 0;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nDstPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nDstPanId & 0xFF);

	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_dst_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_dst_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0], uc_dst_addr_len);
		us_serial_response_len += uc_dst_addr_len;
	} else {
		uc_dst_addr_len = 0;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
	}

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8MpduLinkQuality;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8Dsn;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 24) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 16) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)((pParameters->m_nTimestamp >> 8) & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nTimestamp & 0xFF);

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eSecurityLevel;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8KeyIndex;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_eQualityOfService;

	/* These values have no sense in RF but are returned with fixed values from RF MAC */
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8RecvModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_RecvToneMap.m_au8Tm[0], 3);
	us_serial_response_len += 3;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulation;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)pParameters->m_u8ComputedModulationScheme;
	memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_ComputedToneMap.m_au8Tm[0], 3);
	us_serial_response_len += 3;

	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_u16MsduLength & 0xFF);

	memcpy(&uc_serial_rsp_buf[us_serial_response_len], pParameters->m_pMsdu, pParameters->m_u16MsduLength);
	us_serial_response_len += pParameters->m_u16MsduLength;

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_GetConfirmRF(struct TMacWrpGetConfirm *pParameters)
{
	uint8_t uc_serial_response_len;

	uc_serial_response_len = 0;
	uc_serial_rsp_buf[uc_serial_response_len++] = SERIAL_MSG_MAC_GET_CONFIRM_RF;

	uc_serial_response_len += process_MIB_get_confirm(&uc_serial_rsp_buf[uc_serial_response_len], pParameters);

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = uc_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_SetConfirmRF(struct TMacWrpSetConfirm *pParameters)
{
	uint8_t uc_serial_response_len;

	uc_serial_response_len = 0;
	uc_serial_rsp_buf[uc_serial_response_len++] = SERIAL_MSG_MAC_SET_CONFIRM_RF;

	uc_serial_response_len += process_MIB_set_confirm(&uc_serial_rsp_buf[uc_serial_response_len], pParameters);

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = uc_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_ResetConfirmRF(struct TMacWrpResetConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_RESET_CONFIRM_RF;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_BeaconNotifyRF(struct TMacWrpBeaconNotifyIndication *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_BEACON_NOTIFY_RF;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nPanId & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_u8LinkQuality);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nLbaAddress >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_nLbaAddress & 0xFF);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_u16RcCoord >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_PanDescriptor.m_u16RcCoord & 0xFF);
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_ScanConfirmRF(struct TMacWrpScanConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_SCAN_CONFIRM_RF;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_StartConfirmRF(struct TMacWrpStartConfirm *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_START_CONFIRM_RF;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void MacNotification_CommStatusIndicationRF(struct TMacWrpCommStatusIndication *pParameters)
{
	uint8_t us_serial_response_len;

	us_serial_response_len = 0;
	uint8_t uc_src_addr_len = 0;
	uint8_t uc_dst_addr_len = 0;

	uc_serial_rsp_buf[us_serial_response_len++] = SERIAL_MSG_MAC_COMM_STATUS_INDICATION_RF;
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nPanId >> 8);
	uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_nPanId & 0xFF);
	if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_src_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_SrcAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_SrcAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_src_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_src_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_SrcAddr.m_ExtendedAddress.m_au8Address[0], uc_src_addr_len);
		us_serial_response_len += uc_src_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		uc_dst_addr_len = 2;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress >> 8);
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)(pParameters->m_DstAddr.m_nShortAddress & 0xFF);
	} else if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
		uc_dst_addr_len = 8;
		uc_serial_rsp_buf[us_serial_response_len++] = (uint8_t)uc_dst_addr_len;
		memcpy(&uc_serial_rsp_buf[us_serial_response_len], &pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0], uc_dst_addr_len);
		us_serial_response_len += uc_dst_addr_len;
	} else {
		return; /* This line must never be reached */
	}

	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eStatus;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_eSecurityLevel;
	uc_serial_rsp_buf[us_serial_response_len++] = pParameters->m_u8KeyIndex;

	/* set usi parameters */
	x_mac_serial_msg.uc_protocol_type = PROTOCOL_MAC_G3;
	x_mac_serial_msg.ptr_buf = &uc_serial_rsp_buf[0];
	x_mac_serial_msg.us_len = us_serial_response_len;
	usi_send_cmd(&x_mac_serial_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacInitializeRF(const uint8_t *puc_msg_content)
{
	struct TMacWrpPibValue pibValue;
	enum ESerialStatus status = SERIAL_STATUS_INVALID_PARAMETER;

	(void)puc_msg_content;

	/* disable ADP */
	if (!b_adp_already_disabled) {
		AdpInitialize(NULL, (enum TAdpBand)0);
		b_adp_already_disabled = true;
		/* Disabling ADP creates a temporary callback setting that breaks correct configuration
		in case it is called twice when 2 MACs are initilized (e.g PLC and RF) */
	}

	struct TMacWrpNotifications notificationsRF;

	notificationsRF.m_MacWrpDataConfirm = MacNotification_DataConfirmRF;
	notificationsRF.m_MacWrpDataIndication = MacNotification_DataIndicationRF;
	notificationsRF.m_MacWrpGetConfirm = MacNotification_GetConfirmRF;
	notificationsRF.m_MacWrpSetConfirm = MacNotification_SetConfirmRF;
	notificationsRF.m_MacWrpResetConfirm = MacNotification_ResetConfirmRF;
	notificationsRF.m_MacWrpBeaconNotifyIndication = MacNotification_BeaconNotifyRF;
	notificationsRF.m_MacWrpScanConfirm = MacNotification_ScanConfirmRF;
	notificationsRF.m_MacWrpStartConfirm = MacNotification_StartConfirmRF;
	notificationsRF.m_MacWrpCommStatusIndication = MacNotification_CommStatusIndicationRF;
	notificationsRF.m_MacWrpSnifferIndication = MacNotification_MacSnifferIndicationRF;

	MacWrapperInitializeRF(&notificationsRF);
	
	/* In case PLC MAC was initialized before, the following code will execute twice, but this causes no issue */
	platform_init_eui64(auc_ext_address_mac);
	memcpy(pibValue.m_au8Value, auc_ext_address_mac, sizeof(auc_ext_address_mac));
	pibValue.m_u8Length = sizeof(auc_ext_address_mac);
	MacWrapperMlmeSetRequestSync(MAC_WRP_PIB_MANUF_EXTENDED_ADDRESS, 0, &pibValue);

	adp_mac_serial_if_set_state(SERIAL_MODE_MAC);
	status = SERIAL_STATUS_SUCCESS;

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacDataRequestRF(const uint8_t *puc_msg_content)
{
	uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpDataRequest parameters;
		uint8_t u8SrcAddrLen;
		uint8_t u8DstAddrLen;

		parameters.m_u8MsduHandle = (uint8_t)*puc_buffer++;
		parameters.m_eSecurityLevel =  (enum EMacWrpSecurityLevel)*puc_buffer++;
		parameters.m_u8KeyIndex = (uint8_t)*puc_buffer++;
		parameters.m_eQualityOfService = (enum EMacWrpQualityOfService)*puc_buffer++;
		parameters.m_u8TxOptions = (uint8_t)*puc_buffer++;
		parameters.m_nDstPanId = ((TMacWrpPanId) * puc_buffer++) << 8;
		parameters.m_nDstPanId +=  (TMacWrpPanId) * puc_buffer++;
		u8SrcAddrLen = (uint8_t)*puc_buffer++;
		if (u8SrcAddrLen == 2) {
			parameters.m_eSrcAddrMode = MAC_WRP_ADDRESS_MODE_SHORT;
		} else if (u8SrcAddrLen == 8) {
			parameters.m_eSrcAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED;
		} else {
			return SERIAL_STATUS_INVALID_PARAMETER;
		}

		u8DstAddrLen = (uint8_t)*puc_buffer++;
		if (u8DstAddrLen == 2) {
			parameters.m_DstAddr.m_eAddrMode = MAC_WRP_ADDRESS_MODE_SHORT;
			parameters.m_DstAddr.m_nShortAddress = ((TMacWrpShortAddress) * puc_buffer++) << 8;
			parameters.m_DstAddr.m_nShortAddress +=  (TMacWrpShortAddress) * puc_buffer++;
		} else if (u8DstAddrLen == 8) {
			parameters.m_DstAddr.m_eAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED;
			memcpy(&parameters.m_DstAddr.m_ExtendedAddress, puc_buffer, u8DstAddrLen);
			puc_buffer +=  u8DstAddrLen;
		} else {
			return SERIAL_STATUS_INVALID_PARAMETER;
		}

		parameters.m_u16MsduLength = ((uint16_t)*puc_buffer++) << 8;
		parameters.m_u16MsduLength +=  (uint16_t)*puc_buffer++;
		parameters.m_pMsdu = puc_buffer;

		MacWrapperMcpsDataRequestRF(&parameters);

		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacGetRequestRF(const uint8_t *puc_msg_content)
{
	enum ESerialStatus status = SERIAL_STATUS_INVALID_PARAMETER;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
		process_MIB_get_request(puc_buffer, &get_params);
		MacWrapperMlmeGetRequestRF(&get_params);
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacSetRequestRF(const uint8_t *puc_msg_content)
{
	enum ESerialStatus status = SERIAL_STATUS_INVALID_PARAMETER;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
		process_MIB_set_request(puc_buffer, &set_params);
		MacWrapperMlmeSetRequestRF(&set_params);
		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacResetRequestRF(const uint8_t *puc_msg_content)
{
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpResetRequest parameters;
		parameters.m_bSetDefaultPib = (bool) * puc_msg_content;
		MacWrapperMlmeResetRequestRF(&parameters);

		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacScanRequestRF(const uint8_t *puc_msg_content)
{
	uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpScanRequest parameters;
		parameters.m_u16ScanDuration = ((uint16_t)*puc_buffer++) << 8;
		parameters.m_u16ScanDuration +=  (uint16_t)*puc_buffer;

		MacWrapperMlmeScanRequestRF(&parameters);
		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static enum ESerialStatus _triggerMacStartRequestRF(const uint8_t *puc_msg_content)
{
	uint8_t *puc_buffer = (uint8_t *)puc_msg_content;
	enum ESerialStatus status = SERIAL_STATUS_NOT_ALLOWED;

	if (adp_mac_serial_if_get_state() == SERIAL_MODE_MAC) {
		struct TMacWrpStartRequest parameters;
		parameters.m_nPanId = ((uint16_t)*puc_buffer++) << 8;
		parameters.m_nPanId +=  (uint16_t)*puc_buffer;

		MacWrapperMlmeStartRequestRF(&parameters);

		status = SERIAL_STATUS_SUCCESS;
	}

	return status;
}

#endif

uint8_t serial_if_g3mac_api_parser(uint8_t *puc_rx_msg, uint16_t us_len)
{
	uint8_t uc_serial_if_cmd;
	uint8_t *puc_rx;
	enum ESerialStatus status = SERIAL_STATUS_UNKNOWN_COMMAND;

	/* Protection for invalid us_length */
	if (!us_len) {
		return false;
	}

	/* Process received message */
	uc_serial_if_cmd = puc_rx_msg[0] & 0x3F;
	puc_rx = &puc_rx_msg[1];

	switch (uc_serial_if_cmd) {
	case SERIAL_MSG_MAC_INITIALIZE:
		status = _triggerMacInitialize(puc_rx);
		break;

	case SERIAL_MSG_MAC_DATA_REQUEST:
		status = _triggerMacDataRequest(puc_rx);
		break;

	case SERIAL_MSG_MAC_GET_REQUEST:
		status = _triggerMacGetRequest(puc_rx);
		break;

	case SERIAL_MSG_MAC_SET_REQUEST:
		status = _triggerMacSetRequest(puc_rx);
		break;

	case SERIAL_MSG_MAC_RESET_REQUEST:
		status = _triggerMacResetRequest(puc_rx);
		break;

	case SERIAL_MSG_MAC_SCAN_REQUEST:
		status = _triggerMacScanRequest(puc_rx);
		break;

	case SERIAL_MSG_MAC_START_REQUEST:
		status = _triggerMacStartRequest(puc_rx);
		break;
		
#ifdef G3_HYBRID_PROFILE
	case SERIAL_MSG_MAC_INITIALIZE_RF:
		status = _triggerMacInitializeRF(puc_rx);
		break;

	case SERIAL_MSG_MAC_DATA_REQUEST_RF:
		status = _triggerMacDataRequestRF(puc_rx);
		break;

	case SERIAL_MSG_MAC_GET_REQUEST_RF:
		status = _triggerMacGetRequestRF(puc_rx);
		break;

	case SERIAL_MSG_MAC_SET_REQUEST_RF:
		status = _triggerMacSetRequestRF(puc_rx);
		break;

	case SERIAL_MSG_MAC_RESET_REQUEST_RF:
		status = _triggerMacResetRequestRF(puc_rx);
		break;

	case SERIAL_MSG_MAC_SCAN_REQUEST_RF:
		status = _triggerMacScanRequestRF(puc_rx);
		break;

	case SERIAL_MSG_MAC_START_REQUEST_RF:
		status = _triggerMacStartRequestRF(puc_rx);
		break;
#endif

	default:
		break;
	}

	/* initialize doesn't have Confirm so send Status */
	/* other messages all have send / confirm send status only if there is a processing error */
	if ((status != SERIAL_STATUS_UNKNOWN_COMMAND) && ((status != SERIAL_STATUS_SUCCESS) || 
#ifdef G3_HYBRID_PROFILE
		(uc_serial_if_cmd == SERIAL_MSG_MAC_INITIALIZE) || (uc_serial_if_cmd == SERIAL_MSG_MAC_INITIALIZE_RF))) {
#else
		(uc_serial_if_cmd == SERIAL_MSG_MAC_INITIALIZE))) {
#endif
		MsgStatus(status, uc_serial_if_cmd);
		status = SERIAL_STATUS_SUCCESS;
	}

	return true;
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
