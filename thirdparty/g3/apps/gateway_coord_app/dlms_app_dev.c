/**
 * \file
 *
 * \brief DLMS_APP : DLMS example application for ATMEL G3 Device
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
#include <string.h>
#include <stdio.h>

#include <storage/storage.h>
#include "compiler.h"
#include "AdpApi.h"
#include "drivers/g3/network_adapter_g3.h"
#include "dlms_app_dev.h"

#include "g3_app_config.h"
#include "conf_project.h"
#include "oss_if.h"
#include "dlms_srv_lib.h"
#include "dlms_srv_data.h"
#include "mac_wrapper.h"

/* Invalid short address (0 can only be the coordinator) */
#define LBS_INVALID_SHORT_ADDRESS               0

#ifdef DLMS_DEBUG_CONSOLE
#       define LOG_APP_DEBUG(a)   printf a
#else
#       define LOG_APP_DEBUG(a)   (void)0
#endif

/** Association configuration: in this example, four possible associations: MGMT(1), READ(2), FW(3) and PUBLIC(16) */
static const assoc_conf_t px_assoc_conf[DLMS_MAX_ASSOC] = {
	{0x0001, 0x0001, LLS_FIXED_PWD, "00000002", COSEM_LOW_LEVEL_SEC},    /* MGMT */
	{0x0002, 0x0001, LLS_FIXED_PWD, "00000001", COSEM_LOW_LEVEL_SEC},    /* READ */
	{0x0003, 0x0001, LLS_FIXED_PWD, "00000003", COSEM_LOW_LEVEL_SEC},    /* FW */
	{0x0010, 0x0001, LLS_FIXED_PWD, "--------", COSEM_LOWEST_LEVEL_SEC}  /* PUBLIC */
};

/* Connection status */
static uint16_t sus_num_devices;
static x_dev_addr spx_current_addr_list[DLMS_MAX_DEV_NUM];

/** Meter params */
static meter_params_t sx_meter_params;

/* Socket for the UDP over PLC communication */
Socket *spx_udp_plc_socket;

/* Local IP address */
static IpAddr sx_local_ip_addr;
/* Local Socket PORT */
uint16_t sus_udp_port;

/* Select the first network interface for PLC */
NetInterface *x_plc_interface;

/* IPv6 input and output buffers */
static uint8_t puc_rx_buff[MAX_LENGTH_IPv6_PDU];
static uint8_t puc_tx_buff[MAX_LENGTH_IPv6_PDU];

/**
 * \brief Display SW version in console
 */
static void _show_version( void )
{
	struct TAdpGetConfirm getConfirm;
	struct TAdpMacGetConfirm x_pib_confirm;

#if defined (CONF_BAND_CENELEC_A)
	LOG_APP_DEBUG(("[DLMS_APP] G3 Band: CENELEC-A\r\n"));
#elif defined (CONF_BAND_CENELEC_B)
	LOG_APP_DEBUG(("[DLMS_APP] G3 Band: CENELEC-B\r\n"));
#elif defined (CONF_BAND_FCC)
	LOG_APP_DEBUG(("[DLMS_APP] G3 Band: FCC\r\n"));
#elif defined (CONF_BAND_ARIB)
	LOG_APP_DEBUG(("[DLMS_APP] G3 Band: ARIB\r\n"));
#else
	LOG_APP_DEBUG(("[DLMS_APP] G3 Band: CENELEC-A\r\n"));
#endif

	AdpGetRequestSync(ADP_IB_SOFT_VERSION, 0, &getConfirm);
	if (getConfirm.m_u8AttributeLength == 6) {
		LOG_APP_DEBUG(("[DLMS_APP] G3 stack version: %hhu.%hhu.%hhu Date: 20%hhu-%hhu-%hhu\r\n",
				getConfirm.m_au8AttributeValue[0],
				getConfirm.m_au8AttributeValue[1],
				getConfirm.m_au8AttributeValue[2],
				getConfirm.m_au8AttributeValue[3],
				getConfirm.m_au8AttributeValue[4],
				getConfirm.m_au8AttributeValue[5]));
	}

	AdpGetRequestSync(ADP_IB_MANUF_ADP_INTERNAL_VERSION, 0, &getConfirm);
	if (getConfirm.m_u8AttributeLength == 6) {
		LOG_APP_DEBUG(("[DLMS_APP] ADP version: %hhu.%hhu.%hhu Date: 20%hhu-%hhu-%hhu\r\n",
				getConfirm.m_au8AttributeValue[0],
				getConfirm.m_au8AttributeValue[1],
				getConfirm.m_au8AttributeValue[2],
				getConfirm.m_au8AttributeValue[3],
				getConfirm.m_au8AttributeValue[4],
				getConfirm.m_au8AttributeValue[5]));
	}

	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_MAC_INTERNAL_VERSION, 0, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 6) {
		LOG_APP_DEBUG(("[DLMS_APP] MAC version: %hhu.%hhu.%hhu Date: 20%hhu-%hhu-%hhu\r\n",
				x_pib_confirm.m_au8AttributeValue[0],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[4],
				x_pib_confirm.m_au8AttributeValue[5]));
	}
	
#ifdef G3_HYBRID_PROFILE
	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_MAC_INTERNAL_VERSION_RF, 0, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 6) {
		LOG_APP_DEBUG(("[DLMS_APP] MAC RF version: %hhu.%hhu.%hhu Date: 20%hhu-%hhu-%hhu\r\n",
				x_pib_confirm.m_au8AttributeValue[0],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[4],
				x_pib_confirm.m_au8AttributeValue[5]));
	}
#endif

	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_MAC_RT_INTERNAL_VERSION, 0, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 6) {
		LOG_APP_DEBUG(("[DLMS_APP] MAC_RT version: %hhu.%hhu.%hhu Date: 20%hhu-%hhu-%hhu\r\n",
				x_pib_confirm.m_au8AttributeValue[0],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[4],
				x_pib_confirm.m_au8AttributeValue[5]));
	}

	AdpMacGetRequestSync((uint32_t)MAC_WRP_PIB_MANUF_PHY_PARAM, MAC_WRP_PHY_PARAM_VERSION, &x_pib_confirm);
	if (x_pib_confirm.m_u8AttributeLength == 4) {
		LOG_APP_DEBUG(("[DLMS_APP] PHY version: %02x.%02x.%02x.%02x\r\n",
				x_pib_confirm.m_au8AttributeValue[3],
				x_pib_confirm.m_au8AttributeValue[2],
				x_pib_confirm.m_au8AttributeValue[1],
				x_pib_confirm.m_au8AttributeValue[0]));
	}

	return;
}

/**
 * Convert functions
 **/
static uint8_t _dlms_app_hex_2_string(uint8_t num)
{
	if (num > 9) {
		return (num + 0x37);
	} else {
		return (num + 0x30);
	}
}

static void _dlms_app_generate_serial(uint8_t *puc_serial_board)
{
	uint8_t i;
	uint8_t uc_num, uc_num1;
	uint8_t uc_mac[8];
	struct TAdpMacGetConfirm x_get_confirm;

	/* Get extended MAC address */
	AdpMacGetRequestSync(MAC_WRP_PIB_MANUF_EXTENDED_ADDRESS, 0, &x_get_confirm);
	memcpy(uc_mac, x_get_confirm.m_au8AttributeValue, sizeof(uc_mac));

	*puc_serial_board = 'A';
	*(puc_serial_board + 1) = 'T';
	*(puc_serial_board + 2) = 'M';
	/* convert hex to ascii */
	for (i = 1; i < 6; i++) {
		uc_num  = ((uc_mac[i] & 0xf0) >> 4);
		uc_num1 = (uc_mac[i] & 0x0f);
		puc_serial_board[2 * (i) + 1] = _dlms_app_hex_2_string(uc_num);
		puc_serial_board[2 * (i) + 2] = _dlms_app_hex_2_string(uc_num1);
	}
	*(puc_serial_board + 13) = 0x00;
}

static error_t _dlms_app_initialize_udp_ip(void)
{
	error_t x_error;

	/* Open UDP over PLC socket */
	spx_udp_plc_socket = socketOpen(SOCKET_TYPE_DGRAM, SOCKET_IP_PROTO_UDP);

	/* Failed to open socket? */
	if (!spx_udp_plc_socket) {
		return ERROR_OPEN_FAILED;
	}

#ifdef __G3_GATEWAY__
	/* Bind to UDP Port */
	socketBind(spx_udp_plc_socket, (const IpAddr *)&IP_ADDR_ANY, APP_SOCKET_PORT);
	/* Associate the socket with the relevant interface ETH or PPP */
	x_error = socketBindToInterface(spx_udp_plc_socket, &netInterface[0]);
#else
	/* Associate the socket with the relevant interface */
	x_plc_interface = &netInterface[0];
	x_error = socketBindToInterface(spx_udp_plc_socket, x_plc_interface);
#endif

	if (x_error) {
		return x_error;
	}

	/* Set timeout for blocking operations */
	x_error = socketSetTimeout(spx_udp_plc_socket, APP_DEFAULT_TIMEOUT);

	return x_error;
}

/**
 * \brief Sending data from DLMS Server lib to 4-32 connection
 *
 * \param us_dst_wport     Destination Wrapper Port
 * \param us_src_wport     Source Wrapper Port
 * \param px_buff          Pointer to the data buffer
 * \param us_buff_len      Length of the data
 */
static void _dlms_app_data_request(uint16_t us_dst_wport, uint16_t us_src_wport, uint8_t *puc_buff, uint16_t us_buff_len)
{
	error_t x_error;
	size_t x_transmitted_bytes;
	uint16_t us_tx_buf_len = 0;

	us_tx_buf_len = dlms_srv_add_wrapper_header(puc_tx_buff, us_dst_wport, us_src_wport, puc_buff, us_buff_len);

	x_error = socketSendTo(spx_udp_plc_socket, &sx_local_ip_addr, sus_udp_port, puc_tx_buff, us_tx_buf_len, &x_transmitted_bytes, SOCKET_FLAG_WAIT_ALL);
	if (x_error != NO_ERROR) {
		LOG_APP_DEBUG(("[DLMS_APP] Unsuccessful socketSendTo()!\r\n"));
	}

	dlms_srv_data_cfm(us_dst_wport, us_src_wport, (x_transmitted_bytes == us_tx_buf_len) ? true : false);
}

/**
 * \brief Process of DLMS Application
 *
 */
void dlms_app_process(void)
{
	size_t act_rx_size = 0;

	/* Check data reception */
	socketReceiveFrom(spx_udp_plc_socket, &sx_local_ip_addr, &sus_udp_port, puc_rx_buff, MAX_LENGTH_IPv6_PDU, &act_rx_size, SOCKET_FLAG_WAIT_ALL);

	if (act_rx_size > 0) {
		dlms_srv_wrapper_data_ind(puc_rx_buff, act_rx_size);
	} else {
		dlms_srv_process();
	}
}

/**
 * \brief Update Link Parameters for New connection
 *
 */
void dlms_app_upd_link(uint16_t us_pan_id, uint16_t us_short_addr)
{
	Ipv6Addr ipv6_addr;

	/* Set link-local address, based on the PAN_ID and the short address */
	ipv6StringToAddr(APP_IPV6_GENERIC_LINK_LOCAL_ADDR, &ipv6_addr);
	/* Adapt the IPv6 address to the G3 connection data */
	ipv6_addr.b[8] = (uint8_t)(us_pan_id >> 8);
	ipv6_addr.b[9] = (uint8_t)(us_pan_id & 0xFF);
	ipv6_addr.b[14] = (uint8_t)(us_short_addr >> 8);
	ipv6_addr.b[15] = (uint8_t)(us_short_addr & 0xFF);

	ipv6SetLinkLocalAddr(x_plc_interface, &ipv6_addr);

#ifndef __G3_GATEWAY__
	socketBind(spx_udp_plc_socket, (const IpAddr *)&IP_ADDR_ANY, APP_SOCKET_PORT);

	/* Associate the socket to the G3 PLC network interface */
	socketBindToInterface(spx_udp_plc_socket, x_plc_interface);
#endif
}

/**
 * \brief Data Indication Handler
 *
 */
void dlms_app_data_ind_handler(void *pv_data_ind)
{
	/* Pass Data Indication to IP layer */
	ipv6_receive_packet((struct TAdpDataIndication *)pv_data_ind);
}

/**
 * \brief Join Node Handler
 *
 */
void dlms_app_join_node(uint8_t *puc_extended_address, uint16_t us_short_address)
{
	uint8_t puc_ext_addr_ascii[24];
	uint16_t us_node_idx;
	bool b_already_connected = false;

	sprintf((char *)puc_ext_addr_ascii, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
			puc_extended_address[0], puc_extended_address[1], puc_extended_address[2],
			puc_extended_address[3], puc_extended_address[4], puc_extended_address[5],
			puc_extended_address[6], puc_extended_address[7]);

	LOG_APP_DEBUG(("[DLMS_APP] dlms_app_join_node: short_address = %hu extended addr %s\n", us_short_address, puc_ext_addr_ascii));

	/* Search if node is already in cycle list */
	for (us_node_idx = 0; us_node_idx < DLMS_MAX_DEV_NUM; us_node_idx++) {
		if (!memcmp(spx_current_addr_list[us_node_idx].puc_ext_addr, puc_extended_address, 8)) {
			b_already_connected = true;
			spx_current_addr_list[us_node_idx].us_short_addr = us_short_address;
			break;
		}
	}

	if (!b_already_connected) {
		/* Add node to cycle list */
		for (us_node_idx = 0; us_node_idx < DLMS_MAX_DEV_NUM; us_node_idx++) {
			if (spx_current_addr_list[us_node_idx].us_short_addr == LBS_INVALID_SHORT_ADDRESS) {
				spx_current_addr_list[us_node_idx].us_short_addr = us_short_address;
				memcpy(spx_current_addr_list[us_node_idx].puc_ext_addr, puc_extended_address, 8);
				sus_num_devices++;
				break;
			}
		}
	}
}

/**
 * \brief Leave Node Handler
 *
 */
void dlms_app_leave_node(uint16_t us_short_address)
{
	uint16_t us_node_idx;

	/* Reset node in cycle list */
	for (us_node_idx = 0; us_node_idx < DLMS_MAX_DEV_NUM; us_node_idx++) {
		if (spx_current_addr_list[us_node_idx].us_short_addr == us_short_address) {
			spx_current_addr_list[us_node_idx].us_short_addr = LBS_INVALID_SHORT_ADDRESS;
			memset(spx_current_addr_list[us_node_idx].puc_ext_addr, 0x00, 8);
			sus_num_devices--;
			break;
		}
	}
}

/**
 * \brief Initialize DLMS Application
 *
 */
void dlms_app_init(void)
{
	uint16_t us_node_idx;

	/*                                 OBIS CODE + IC        READ:  MGMT   READ        FW          PUBLIC       WRITE    */
	obis_element_conf_t x_new_obis = {{0, 0, 0, 0, 0, 0}, 0, {0xFFFF0000, 0xFFFF0000, 0xFFFF0000, 0xFFFF0000}, {0, 0, 0, 0}, NULL};

	/* Init IP stack */
	_dlms_app_initialize_udp_ip();

	/* Set port */
	sus_udp_port = APP_SOCKET_PORT;

	/* Current cycle node list init */
	sus_num_devices = 0;
	for (us_node_idx = 0; us_node_idx < DLMS_MAX_DEV_NUM; us_node_idx++) {
		spx_current_addr_list[us_node_idx].us_short_addr = LBS_INVALID_SHORT_ADDRESS;
		memset(spx_current_addr_list[us_node_idx].puc_ext_addr, 0x00, 8);
	}

	/* Generate Serial Number */
	_dlms_app_generate_serial(sx_meter_params.meter_serial);

	/* Set data pointers */
	sx_meter_params.us_max_num_devices = DLMS_MAX_DEV_NUM;
	sx_meter_params.pus_current_num_devices = &sus_num_devices;
	sx_meter_params.px_current_addr_list = spx_current_addr_list;

	/* DLMS Server lib init */
	dlms_srv_init(px_assoc_conf, DLMS_MAX_ASSOC, &sx_meter_params, _dlms_app_data_request);

	/* DLMS Server Common object list */
	dlms_srv_conf_obis(0, 0, 1, 0, 0, 255, 8, obis_0_0_1_0_0_255_cb, &x_new_obis);     /* Clock */
	dlms_srv_conf_obis(0, 0, 40, 0, 0, 255, 15, obis_0_0_40_0_0_255_cb, &x_new_obis);   /* Current association */

	/* DLMS Server G3 Server specific objects */
	dlms_srv_conf_obis(0, 0, 29, 1, 0, 255, 91, obis_0_0_29_1_0_255_cb, &x_new_obis);   /* Neighbor table */
	dlms_srv_conf_obis(0, 0, 29, 2, 0, 255, 92, obis_0_0_29_2_0_255_cb, &x_new_obis);   /* Routing table */
	dlms_srv_conf_obis(0, 0, 29, 2, 1, 128, 1, obis_0_0_29_2_1_128_cb, &x_new_obis);    /* Extended address table */  /* MANUFACTURER OBIS */
	dlms_srv_conf_obis(0, 0, 29, 2, 2, 128, 1, obis_0_0_29_2_2_128_cb, &x_new_obis);    /* Extended address table */  /* MANUFACTURER OBIS */
	dlms_srv_conf_obis(0, 0, 29, 2, 3, 128, 7, obis_0_0_29_2_3_128_cb, &x_new_obis);    /* Extended address table */  /* MANUFACTURER OBIS */

	LOG_APP_DEBUG(("[DLMS_APP] DLMS Application: GATEWAY\r\n"));
	_show_version();
}
