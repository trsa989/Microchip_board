/**
 * \file
 *
 * \brief G3 Example Application : ADP Manager
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <storage/storage.h>
#include "compiler.h"
#include "AdpApi.h"
#include "drivers/g3/network_adapter_g3.h"
#include "app_adp_mng.h"
#include "dlms_app_dev.h"
#include "g3_app_config.h"
#include "conf_project.h"
#include "hal/hal.h"
#include "conf_bs.h"
#include "bs_api.h"

/* MAC include */
#include "mac_wrapper.h"

/* Net Interfaces Includes */
#ifdef  CONF_USE_ETHERNET
#include "drivers/mac/same70_eth_driver.h"
#include "drivers/phy/ksz8081_driver.h"
#endif
#ifdef  CONF_USE_PPP
#include "ppp/ppp.h"
#include "ppp/ppp_hdlc.h"
#include "drivers/plc_buart.h"
#endif
#include "ipv6/ipv6_routing.h"

Ipv6Addr ipv6Addr_eth_Prefix;
Ipv6Addr ipv6Addr_plc_Prefix;

#ifdef APP_DEBUG_CONSOLE
#       define LOG_APP_DEBUG(a)   printf a
#else
#       define LOG_APP_DEBUG(a)   (void)0
#endif

/*
 * Time to wait before start calling null modem process.
 * Needed in case there is some negotiation on PPP port between PC and board
 */
#define TIME_WAIT_TO_START_NULL_MODEM_PROCESS   0 /* ms */
static uint32_t u32TimerWaitNullModem = TIME_WAIT_TO_START_NULL_MODEM_PROCESS;

/* Context information, to be updated when PanId is known */
#define CONTEXT_INFORMATION_0_SIZE   14
static uint8_t au8ConfContextInformationTable0[CONTEXT_INFORMATION_0_SIZE];

/* MIB types and definitions */
enum MibType {
	MIB_MAC = 0,
	MIB_ADP
};

struct MibData {
	uint8_t m_u8Type;
	const char *m_szName;
	uint32_t m_u32Id;
	uint16_t m_u16Index;
	uint8_t m_u8ValueLength;
	const uint8_t *m_pu8Value;
};

static uint8_t auc_ext_address[8]; /* EUI64 */

uint8_t g_u8MibInitIndex = 0;
const uint8_t g_u8MibTableSize = 9;
struct MibData g_MibSettings[] = {
	{ MIB_MAC, "MAC_WRP_PIB_MANUF_EXTENDED_ADDRESS", MAC_WRP_PIB_MANUF_EXTENDED_ADDRESS, 0, 8, auc_ext_address },
	{ MIB_MAC, "MAC_WRP_PIB_SHORT_ADDRESS", MAC_WRP_PIB_SHORT_ADDRESS, 0, 2, CONF_SHORT_ADDRESS },
	{ MIB_ADP, "ADP_IB_CONTEXT_INFORMATION_TABLE", ADP_IB_CONTEXT_INFORMATION_TABLE, 0, 14, CONF_CONTEXT_INFORMATION_TABLE_0 },
	{ MIB_ADP, "ADP_IB_CONTEXT_INFORMATION_TABLE", ADP_IB_CONTEXT_INFORMATION_TABLE, 1, 10, CONF_CONTEXT_INFORMATION_TABLE_1 },
	{ MIB_ADP, "ADP_IB_GROUP_TABLE", ADP_IB_GROUP_TABLE, 0, 2, CONF_GROUP_TABLE_0 },
	{ MIB_ADP, "ADP_IB_ROUTING_TABLE_ENTRY_TTL", ADP_IB_ROUTING_TABLE_ENTRY_TTL, 0, 2, CONF_ROUTING_TABLE_ENTRY_TTL },
	{ MIB_ADP, "ADP_IB_MAX_JOIN_WAIT_TIME", ADP_IB_MAX_JOIN_WAIT_TIME, 0, 2, CONF_MAX_JOIN_WAIT_TIME },
	{ MIB_ADP, "ADP_IB_MAX_HOPS", ADP_IB_MAX_HOPS, 0, 1, CONF_MAX_HOPS },
	{ MIB_ADP, "ADP_IB_MANUF_EAP_PRESHARED_KEY", ADP_IB_MANUF_EAP_PRESHARED_KEY, 0, 16, CONF_PSK_KEY }
};

static void SetConfirm(uint8_t u8Status, uint32_t u32AttributeId, uint16_t u16AttributeIndex)
{
	if (u8Status == G3_SUCCESS) {
		/* initialization phase? */
		if (g_u8MibInitIndex < g_u8MibTableSize) {
			if ((g_MibSettings[g_u8MibInitIndex].m_u32Id == u32AttributeId) && (g_MibSettings[g_u8MibInitIndex].m_u16Index == u16AttributeIndex)) {
				g_u8MibInitIndex++;
				if (g_u8MibInitIndex == g_u8MibTableSize) {
					LOG_APP_DEBUG(("Modem fully initialized.\r\n"));
				}
			} else {
				LOG_APP_DEBUG((
							"ERR[AppAdpSetConfirm] Invalid SetConfirm received during initialization. Expecting 0x%08X/%u but received 0x%08X/%u\r\n",
							g_MibSettings[g_u8MibInitIndex].m_u32Id, g_MibSettings[g_u8MibInitIndex].m_u16Index,
							u32AttributeId, u16AttributeIndex));
			}
		}
	}
	else if (u8Status == MAC_WRP_STATUS_INVALID_PARAMETER) {
		/* In case parameter is invalid, jump to next IB to avoid getting stuck in initialization */
		LOG_APP_DEBUG(("ERR[AppAdpSetConfirm] Invalid parameter for IB index %u", g_u8MibInitIndex));
		g_u8MibInitIndex++;
	}
	else {
		LOG_APP_DEBUG(("ERR[AppAdpSetConfirm] status: %u\r\n", u8Status));
	}
}

static void AppAdpDataConfirm(struct TAdpDataConfirm *pDataConfirm)
{
	UNUSED(pDataConfirm);
#ifdef DLMS_REPORT
	if (pDataConfirm->m_u8Status != G3_SUCCESS) {
		switch (pDataConfirm->m_u8Status) {
		case G3_INVALID_REQUEST:
			printf("AppAdpDataConfirm  Error: G3_INVALID_REQUEST (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_FAILED:
			printf("AppAdpDataConfirm  Error: G3_FAILED (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_INVALID_IPV6_FRAME:
			printf("AppAdpDataConfirm  Error: G3_INVALID_IPV6_FRAME (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_NOT_PERMITED:
			printf("AppAdpDataConfirm  Error: G3_NOT_PERMITED (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_ROUTE_ERROR:
			printf("AppAdpDataConfirm  Error: G3_ROUTE_ERROR (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_TIMEOUT:
			printf("AppAdpDataConfirm  Error: G3_TIMEOUT (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_INVALID_INDEX:
			printf("AppAdpDataConfirm  Error: G3_INVALID_INDEX (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_INVALID_PARAMETER:
			printf("AppAdpDataConfirm  Error: G3_INVALID_PARAMETER (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_NO_BEACON:
			printf("AppAdpDataConfirm  Error: G3_NO_BEACON (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_READ_ONLY:
			printf("AppAdpDataConfirm  Error: G3_READ_ONLY (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_UNSUPPORTED_ATTRIBUTE:
			printf("AppAdpDataConfirm  Error: G3_UNSUPPORTED_ATTRIBUTE (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_INCOMPLETE_PATH:
			printf("AppAdpDataConfirm  Error: G3_INCOMPLETE_PATH (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_BUSY:
			printf("AppAdpDataConfirm  Error: G3_BUSY (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_NO_BUFFERS:
			printf("AppAdpDataConfirm  Error: G3_NO_BUFFERS (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		case G3_ERROR_INTERNAL:
			printf("AppAdpDataConfirm  Error: G3_ERROR_INTERNAL (%d)\r\n", pDataConfirm->m_u8Status);
			break;

		default:
			printf("AppAdpDataConfirm Unknown Error\r\n");
		}
	}
#endif
}

static void AppAdpDataIndication(struct TAdpDataIndication *pDataIndication)
{
	if (pDataIndication->m_u16NsduLength) {
		ipv6_receive_packet((struct TAdpDataIndication *)pDataIndication);
		LOG_APP_DEBUG(("ERR[AppAdpDataIndication] Len: %u LQI: %u\r\n", pDataIndication->m_u16NsduLength, pDataIndication->m_u8LinkQualityIndicator));
	}
}

static void AppAdpDiscoveryConfirm(uint8_t uc_status)
{
	UNUSED(uc_status);
}

static void AppAdpDiscoveryIndication(struct TAdpPanDescriptor *pPanDescriptor)
{
	UNUSED(pPanDescriptor);
}

static void AppAdpNetworkStartConfirm(struct TAdpNetworkStartConfirm *pNetworkStartConfirm)
{
	UNUSED(pNetworkStartConfirm);
}

static void AppAdpNetworkJoinConfirm(struct TAdpNetworkJoinConfirm *pNetworkJoinConfirm)
{
	UNUSED(pNetworkJoinConfirm);
}

static void AppAdpNetworkLeaveIndication(void)
{
}

static void AppAdpNetworkLeaveConfirm(struct TAdpNetworkLeaveConfirm *pLeaveConfirm)
{
	UNUSED(pLeaveConfirm);
}

static void AppAdpResetConfirm(struct TAdpResetConfirm *pResetConfirm)
{
	UNUSED(pResetConfirm);
}

static void AppAdpSetConfirm(struct TAdpSetConfirm *pSetConfirm)
{
	SetConfirm(pSetConfirm->m_u8Status, pSetConfirm->m_u32AttributeId, pSetConfirm->m_u16AttributeIndex);
}

static void AppAdpMacSetConfirm(struct TAdpMacSetConfirm *pSetConfirm)
{
	SetConfirm(pSetConfirm->m_u8Status, pSetConfirm->m_u32AttributeId, pSetConfirm->m_u16AttributeIndex);
}

static void AppAdpGetConfirm(struct TAdpGetConfirm *pGetConfirm)
{
	UNUSED(pGetConfirm);
}

static void AppAdpMacGetConfirm(struct TAdpMacGetConfirm *pGetConfirm)
{
	UNUSED(pGetConfirm);
}

static void AppAdpRouteDiscoveryConfirm(struct TAdpRouteDiscoveryConfirm *pRouteDiscoveryConfirm)
{
	UNUSED(pRouteDiscoveryConfirm);
}

static void AppAdpPathDiscoveryConfirm(struct TAdpPathDiscoveryConfirm *pPathDiscoveryConfirm)
{
	UNUSED(pPathDiscoveryConfirm);
}

static void AppAdpNetworkStatusIndication(struct TAdpNetworkStatusIndication *pNetworkStatusIndication)
{
	UNUSED(pNetworkStatusIndication);
}

static void AppAdpBufferIndication(struct TAdpBufferIndication *pBufferIndication)
{
	UNUSED(pBufferIndication);
}

static void AppBsJoinIndication(uint8_t *puc_extended_address, uint16_t us_short_address)
{
	dlms_app_join_node(puc_extended_address, us_short_address);
}

#ifdef  CONF_USE_ETHERNET
static error_t _ethInterfaceInit(void)
{
	error_t error;
	NetInterface *interface;
	MacAddr macAddr;
#if (IPV6_SUPPORT == ENABLED)
#if (APP_ETH_USE_SLAAC == ENABLED)
	static SlaacSettings slaacSettings;
	static SlaacContext slaacContext0;
#else
	Ipv6Addr ipv6Addr;
#endif
#endif

	/* Configure the first network interface (Ethernet 10/100) */
	interface = &netInterface[ETH_IF0];

	/* Set interface name */
	netSetInterfaceName(interface, "eth0");
	/* Set host name */
	netSetHostname(interface, "GatewayG3");
	/* Select the relevant network adapter */
	netSetDriver(interface, &same70EthDriver);
	netSetPhyDriver(interface, &ksz8081PhyDriver);

	/* Set host MAC address */
	macStringToAddr(IF0_MAC_ADDR, &macAddr);
	netSetMacAddr(interface, &macAddr);

	/* Initialize network interface */
	error = netConfigInterface(interface);
	/* Any error to report? */
	if (error) {
		/* Debug message */
		printf("Failed to configure interface %s!\r\n", interface->name);
		/* Exit immediately */
		return error;
	}

#if (IPV6_SUPPORT == ENABLED)
#if (APP_ETH_USE_SLAAC == ENABLED)
	/* Get default settings */
	slaacGetDefaultSettings(&slaacSettings);
	/* Set the network interface to be configured */
	slaacSettings.interface = interface;

	/* SLAAC initialization */
	error = slaacInit(&slaacContext0, &slaacSettings);
	/* Failed to initialize SLAAC? */
	if (error) {
		/* Debug message */
		printf("Failed to initialize SLAAC!\r\n");
		/* Exit immediately */
		return error;
	}

	/* Start IPv6 address autoconfiguration process */
	error = slaacStart(&slaacContext0);
	/* Failed to start SLAAC process? */
	if (error) {
		/* Debug message */
		printf("Failed to start SLAAC!\r\n");
		/* Exit immediately */
		return error;
	}

#else
	error = ipv6StringToAddr(IF0_IPV6_LINK_LOCAL_ADDR, &ipv6Addr);
	if (error == NO_ERROR) {
		ipv6SetLinkLocalAddr(interface, &ipv6Addr);
	} else {
		printf("BAD LOCAL LINK Address!\r\n");
	}

	error = ipv6StringToAddr(IF0_IPV6_ULA_ADDR, &ipv6Addr);
	if (error == NO_ERROR) {
		ipv6SetGlobalAddr(interface, 0, &ipv6Addr);
	} else {
		printf("BAD ULA Address!\r\n");
	}

	ipv6StringToAddr(IF0_IPV6_NET_PREFIX, &ipv6Addr_eth_Prefix);
	ipv6SetPrefix(interface, 0, &ipv6Addr_eth_Prefix, IF0_IPV6_NET_PREFIX_LEN);
#endif
#endif

#if IPV6_ROUTING_SUPPORT == ENABLED
	ipv6EnableRouting(interface, TRUE);
#endif

#if NDP_ROUTER_ADV_SUPPORT == ENABLED
	static NdpRouterAdvSettings ndpSettings;
	static NdpRouterAdvContext ndpContext;
	static NdpRouterAdvRouteInfo routeInfo[1];

	ndpRouterAdvGetDefaultSettings(&ndpSettings);

	/* Add prefix route to G3 Network */
	error = ipv6StringToAddr(G3_IF1_IPV6_NET_PREFIX, &routeInfo[0].prefix);
	if (error != NO_ERROR) {
		printf("ipv6StringToAddr error in G3_IF1_IPV6_NET_PREFIX\r\n");
	}

	routeInfo[0].prefix.b[6] = G3_COORDINATOR_PAN_ID >> 8;
	routeInfo[0].prefix.b[7] = G3_COORDINATOR_PAN_ID & 0xFF;
	routeInfo[0].length = 64;
	routeInfo[0].preference = NDP_ROUTER_SEL_PREFERENCE_MEDIUM;
	routeInfo[0].routeLifetime = 0;
	ndpSettings.routeList = routeInfo;
	ndpSettings.routeListLength = 1;

	ndpRouterAdvInit(&ndpContext, &ndpSettings);
	ndpRouterAdvStart(interface->ndpRouterAdvContext);
#endif

	/* Successful initialization */
	return NO_ERROR;
}

#endif /* CONF_USE_ETHERNET */

#ifdef  CONF_USE_PPP

PppSettings _pppSettings;
PppContext _pppContex;

static error_t _pppInterfaceInit(void)
{
	error_t error;
	NetInterface *interface;
	MacAddr macAddr;

	/* Configure the first PPP settings */
	interface = &netInterface[PPP_IF0];

	/* Set host name */
	netSetHostname(interface, "GatewayG3");
	/* Select the relevant network adapter */

	pppGetDefaultSettings(&_pppSettings);
	_pppSettings.interface = interface;
	_pppSettings.accm = 0x00000000;
	_pppSettings.authProtocol = PPP_AUTH_PROTOCOL_PAP; /* | PPP_AUTH_PROTOCOL_CHAP_MD5; */

	pppInit(&_pppContex, &_pppSettings);

	/* Set interface name */
	netSetInterfaceName(interface, "ppp0");
	netSetUartDriver(interface, &plcBuartDriver);

	/* Initialize network interface */
	error = netConfigInterface(interface);
	/* Any error to report? */
	if (error) {
		/* Debug message */
		printf("Failed to configure interface %s!\r\n", interface->name);
		/* Exit immediately */
		return error;
	}

	pppSetAuthInfo(interface, "guest", "microchip.0");

	/* Set host MAC address */
	macStringToAddr(IF0_MAC_ADDR, &macAddr);
	netSetMacAddr(interface, &macAddr);

#if IPV6_ROUTING_SUPPORT == ENABLED
	ipv6EnableRouting(interface, TRUE);
#endif

	/* Successful initialization */
	return NO_ERROR;
}

#endif /* CONF_USE_PPP */

static error_t _g3InterfaceInit(void)
{
	error_t error;
	NetInterface *interface;

	Ipv6Addr ipv6Addr;

	/* Configure the first network interface (PLC) */
	interface = &netInterface[G3_IF1];

	/* Set interface name */
	netSetInterfaceName(interface, "plc0");
	/* Select the relevant network adapter */
	netSetDriver(interface, &g3_adapter);
	/* Initialize network interface */
	error = netConfigInterface(interface);

#if IPV6_ROUTING_SUPPORT == ENABLED
	ipv6EnableRouting(interface, TRUE);
#endif

	if (error) {
		return error;
	}

	ipv6StringToAddr(G3_IF1_IPV6_GENERIC_COORDINATOR_ADDR, &ipv6Addr);
	ipv6Addr.b[8] = (uint8_t)(G3_COORDINATOR_PAN_ID >> 8);
	ipv6Addr.b[9] = (uint8_t)(G3_COORDINATOR_PAN_ID & 0x00FF);
	ipv6SetLinkLocalAddr(interface, &ipv6Addr);

	ipv6StringToAddr(G3_IF1_IPV6_ULA_COORDINATOR_ADDR, &ipv6Addr);
	ipv6Addr.b[6] = (uint8_t)(G3_COORDINATOR_PAN_ID >> 8);
	ipv6Addr.b[7] = (uint8_t)(G3_COORDINATOR_PAN_ID & 0x00FF);
	ipv6SetGlobalAddr(interface, 0, &ipv6Addr);

	ipv6StringToAddr(G3_IF1_IPV6_NET_PREFIX, &ipv6Addr_plc_Prefix);
	ipv6Addr_plc_Prefix.b[6] = (uint8_t)(G3_COORDINATOR_PAN_ID >> 8);
	ipv6Addr_plc_Prefix.b[7] = (uint8_t)(G3_COORDINATOR_PAN_ID & 0x00FF);
	ipv6SetPrefix(interface, 0, &ipv6Addr_plc_Prefix, G3_IF1_IPV6_NET_PREFIX_LEN);

	return NO_ERROR;
}

static void network_init(void)
{
#ifdef  CONF_USE_ETHERNET
	/* initialize ethernet interface for SAME70 Gateway coordinators*/
	_ethInterfaceInit();
#endif

#ifdef CONF_USE_PPP
	/* initialize PPP interface for SAM4CP16C  Gateway coordinators*/
	_pppInterfaceInit();
#endif

	/* initialize G3 interface */
	_g3InterfaceInit();

#if IPV6_ROUTING_SUPPORT == ENABLED
	{
		/* Configure default routes */
		Ipv6Addr ipv6Addr;
		NetInterface *interface;

		/* G3 Interface route settings */
		interface = &netInterface[G3_IF1];

		ipv6StringToAddr(G3_IF1_IPV6_ROUTER_ADDR, &ipv6Addr);
		ipv6Addr.b[6] = (uint8_t)(G3_COORDINATOR_PAN_ID >> 8);
		ipv6Addr.b[7] = (uint8_t)(G3_COORDINATOR_PAN_ID & 0x00FF);

		ipv6AddRoute(&ipv6Addr, 64, interface, NULL, 1);

		/* ETH0 Interface route settings */
#if !(APP_ETH_USE_SLAAC == ENABLED)
		interface = &netInterface[ETH_IF0];

		ipv6StringToAddr(IF0_IPV6_DEFAULT_ROUTE_ADDR, &ipv6Addr);
		ipv6AddRoute(&ipv6Addr, 16, interface, NULL, 1);
#endif
	}
#endif
}

#ifdef  CONF_USE_PPP
static void null_modem_process(void)
{
	char pc_at_cmd[128];
	size_t len = 128;
	NetInterface *interface;
	error_t error;

	interface = &netInterface[PPP_IF0];

	if (interface->nicDriver->type == NIC_TYPE_PPP) {
		if (interface->pppContext == NULL) {
			return; /* PPP Not configured yet */
		}

		if (interface->pppContext->pppPhase == PPP_PHASE_DEAD) {
			/* PPP is ready for new connections */
			error = pppHdlcDriverReceiveAtCommand(interface, pc_at_cmd, len);
			if (error == NO_ERROR) {
				Ipv6Addr ipv6Addr;

				/* Send null-modem response */
				pppHdlcDriverSendAtCommand(interface, "CLIENTSERVER");

				/* Set link-local address*/
				/* Load default */
				error = ipv6StringToAddr(IF0_IPV6_LINK_LOCAL_ADDR, &ipv6Addr);
				if (error == NO_ERROR) {
					/* Update Interface Id*/
					ipv6Addr.b[8] =  (uint8_t)auc_ext_address[7];
					ipv6Addr.b[9] =  (uint8_t)auc_ext_address[6];
					ipv6Addr.b[10] =  (uint8_t)auc_ext_address[5];
					ipv6Addr.b[11] =  (uint8_t)auc_ext_address[4];
					ipv6Addr.b[12] =  (uint8_t)auc_ext_address[3];
					ipv6Addr.b[13] =  (uint8_t)auc_ext_address[2];
					ipv6Addr.b[14] =  (uint8_t)auc_ext_address[1];
					ipv6Addr.b[15] =  (uint8_t)auc_ext_address[0];
					ipv6SetLinkLocalAddr(interface, &ipv6Addr);
				} else {
					printf("BAD LOCAL LINK Address!\r\n");
				}

				error = ipv6StringToAddr(IF0_IPV6_ULA_ADDR, &ipv6Addr);
				if (error == NO_ERROR) {
#ifdef UPDATE_IF0_IPV6_ULA_ADDR_WITH_EUI64
					/* Update Interface Id*/
					ipv6Addr.b[8] =  (uint8_t)auc_ext_address[7];
					ipv6Addr.b[9] =  (uint8_t)auc_ext_address[6];
					ipv6Addr.b[10] =  (uint8_t)auc_ext_address[5];
					ipv6Addr.b[11] =  (uint8_t)auc_ext_address[4];
					ipv6Addr.b[12] =  (uint8_t)auc_ext_address[3];
					ipv6Addr.b[13] =  (uint8_t)auc_ext_address[2];
					ipv6Addr.b[14] =  (uint8_t)auc_ext_address[1];
					ipv6Addr.b[15] =  (uint8_t)auc_ext_address[0];
#endif
					ipv6SetGlobalAddr(interface, 0, &ipv6Addr);
				} else {
					printf("BAD ULA Address!\r\n");
				}

				printf("Modem Initialized....\r\n");
				printf("Start PPP Connection.\r\n");

				pppConnect(interface);
			}
		}
	}
}

#endif

static void AppBsLeaveIndication(uint16_t u16SrcAddr, bool bSecurityEnabled, uint8_t u8LinkQualityIndicator, uint8_t *pNsdu, uint16_t u16NsduLength)
{
	UNUSED(bSecurityEnabled);
	UNUSED(u8LinkQualityIndicator);
	UNUSED(pNsdu);
	UNUSED(u16NsduLength);

	dlms_app_leave_node(u16SrcAddr);
}

static void AppAdpNotification_UpdNonVolatileDataIndication(void)
{
#ifdef ENABLE_PIB_RESTORE
	store_persistent_data_GPBR();
#endif
}

static void InitializeStack(void)
{
	struct TAdpNotifications notifications;
	TBootstrapAdpNotifications *ps_bs_notifications;

	/* Set Bootstrap managment */
	ps_bs_notifications = bs_get_not_handlers();

	/* Set ADP managment */
	notifications.fnctAdpDataConfirm = AppAdpDataConfirm;
	notifications.fnctAdpDataIndication = AppAdpDataIndication;
	notifications.fnctAdpDiscoveryConfirm = AppAdpDiscoveryConfirm;
	notifications.fnctAdpDiscoveryIndication = AppAdpDiscoveryIndication;
	notifications.fnctAdpNetworkStartConfirm = AppAdpNetworkStartConfirm;
	notifications.fnctAdpNetworkJoinConfirm = AppAdpNetworkJoinConfirm;
	notifications.fnctAdpNetworkLeaveIndication = AppAdpNetworkLeaveIndication;
	notifications.fnctAdpNetworkLeaveConfirm = AppAdpNetworkLeaveConfirm;
	notifications.fnctAdpResetConfirm = AppAdpResetConfirm;
	notifications.fnctAdpSetConfirm = AppAdpSetConfirm;
	notifications.fnctAdpMacSetConfirm = AppAdpMacSetConfirm;
	notifications.fnctAdpGetConfirm = AppAdpGetConfirm;
	notifications.fnctAdpMacGetConfirm = AppAdpMacGetConfirm;
	notifications.fnctAdpLbpConfirm = ps_bs_notifications->fnctAdpLbpConfirm;
	notifications.fnctAdpLbpIndication = ps_bs_notifications->fnctAdpLbpIndication;
	notifications.fnctAdpRouteDiscoveryConfirm = AppAdpRouteDiscoveryConfirm;
	notifications.fnctAdpPathDiscoveryConfirm = AppAdpPathDiscoveryConfirm;
	notifications.fnctAdpNetworkStatusIndication = AppAdpNetworkStatusIndication;
	notifications.fnctAdpBufferIndication = AppAdpBufferIndication;
	notifications.fnctAdpUpdNonVolatileDataIndication = AppAdpNotification_UpdNonVolatileDataIndication;
	notifications.fnctAdpRouteNotFoundIndication = NULL;

#if defined (CONF_BAND_CENELEC_A)
	AdpInitialize(&notifications, ADP_BAND_CENELEC_A);
#elif defined (CONF_BAND_CENELEC_B)
	AdpInitialize(&notifications, ADP_BAND_CENELEC_B);
#elif defined (CONF_BAND_FCC)
	AdpInitialize(&notifications, ADP_BAND_FCC);
#elif defined (CONF_BAND_ARIB)
	AdpInitialize(&notifications, ADP_BAND_ARIB);
#else
	AdpInitialize(&notifications, ADP_BAND_CENELEC_A);
#endif
}

static void InitializeModemParameters(void)
{
	/* Serialize ADP Prefix data */
	uint8_t prefix_data[27];
	uint32_t ui_infinite = 0x20C000;

	while (g_u8MibInitIndex < g_u8MibTableSize) {
		if (g_u8MibInitIndex == 0) {
			LOG_APP_DEBUG(("Start modem initialization.\r\n"));
		}

		LOG_APP_DEBUG(("Setting command %u: %s / %u\r\n", g_u8MibInitIndex, g_MibSettings[g_u8MibInitIndex].m_szName,
				g_MibSettings[g_u8MibInitIndex].m_u16Index));
		if (g_MibSettings[g_u8MibInitIndex].m_u8Type == MIB_ADP) {
			AdpSetRequest(g_MibSettings[g_u8MibInitIndex].m_u32Id, g_MibSettings[g_u8MibInitIndex].m_u16Index,
					g_MibSettings[g_u8MibInitIndex].m_u8ValueLength, g_MibSettings[g_u8MibInitIndex].m_pu8Value);
		} else {
			AdpMacSetRequest(g_MibSettings[g_u8MibInitIndex].m_u32Id, g_MibSettings[g_u8MibInitIndex].m_u16Index,
					g_MibSettings[g_u8MibInitIndex].m_u8ValueLength, g_MibSettings[g_u8MibInitIndex].m_pu8Value);
		}
	}

	prefix_data[0] = G3_IF1_IPV6_NET_PREFIX_LEN; /* Prefix lenght */
	prefix_data[1] = 1;   /* OnLinkFlag */
	prefix_data[2] = 1;   /* AutonomuosConfiguration flag */
	memcpy(&prefix_data[3], (uint8_t *)&ui_infinite, 4); /* valid lifetime */
	memcpy(&prefix_data[7], (uint8_t *)&ui_infinite, 4); /* preferred lifetime */
	memcpy(&prefix_data[11], &ipv6Addr_plc_Prefix, 16); /* network prefix */

	/* Configure ADP */
	AdpSetRequest(ADP_IB_PREFIX_TABLE, 0, 27, (const uint8_t *)prefix_data);
}

static void StartCoordinator(void)
{
	struct t_bs_lbp_set_param_confirm lbp_set_confirm;
	TBootstrapConfiguration s_bs_conf;
	struct TAdpGetConfirm getConfirm;

	/* Initialize eui64 */
	platform_init_eui64(auc_ext_address);

	/* Set configuration values for user-specified ADP parameters */
	InitializeModemParameters();

#ifdef ENABLE_PIB_RESTORE
	load_persistent_info();
#endif

	/* Init Bootstrap module */
#if defined (CONF_BAND_CENELEC_A)
	s_bs_conf.m_u8BandInfo = ADP_BAND_CENELEC_A;
#elif defined (CONF_BAND_CENELEC_B)
	s_bs_conf.m_u8BandInfo = ADP_BAND_CENELEC_B;
#elif defined (CONF_BAND_FCC)
	s_bs_conf.m_u8BandInfo = ADP_BAND_FCC;
#elif defined (CONF_BAND_ARIB)
	s_bs_conf.m_u8BandInfo = ADP_BAND_ARIB;
#endif
	AdpGetRequestSync(ADP_IB_MAX_HOPS, 0, &getConfirm);
	s_bs_conf.m_u8MaxHop = getConfirm.m_au8AttributeValue[0];

	bs_init(s_bs_conf);
	bs_lbp_join_ind_set_cb(AppBsJoinIndication);
	bs_lbp_leave_ind_set_cb(AppBsLeaveIndication);

	/* Set PSK */
	bs_lbp_set_param(LBP_IB_PSK, 0, 16, CONF_PSK_KEY, &lbp_set_confirm);

	/* Set GMK */
	bs_lbp_set_param(LBP_IB_GMK, 0, 16, CONF_GMK_KEY, &lbp_set_confirm);

	/* Set Context with network PanId before Network Start */
	memcpy(au8ConfContextInformationTable0, CONF_CONTEXT_INFORMATION_TABLE_0, CONTEXT_INFORMATION_0_SIZE);
	au8ConfContextInformationTable0[CONTEXT_INFORMATION_0_SIZE - 2] = (uint8_t)((G3_COORDINATOR_PAN_ID >> 8) & 0xFF);
	au8ConfContextInformationTable0[CONTEXT_INFORMATION_0_SIZE - 1] = (uint8_t)(G3_COORDINATOR_PAN_ID & 0xFF);
	AdpSetRequest(ADP_IB_CONTEXT_INFORMATION_TABLE, 0, CONTEXT_INFORMATION_0_SIZE, au8ConfContextInformationTable0);

	/* Start G3 Network */
	AdpNetworkStartRequest(G3_COORDINATOR_PAN_ID);
}

/**
 * \brief Create main Cycles Application task and create timer to update internal counters.
 *
 */
void app_init(void)
{
	/* Init network inferfaces */
	network_init();

	/* Init modules */
	InitializeStack();

	/* Start node as coordinator */
	StartCoordinator();

	LOG_APP_DEBUG(("[G3 Gateway Coordinator] PANID = %04X \r\n", G3_COORDINATOR_PAN_ID));
}

/**
 * \brief Update internal counters.
 *
 */
void app_timers_update(void)
{
	if (u32TimerWaitNullModem) {
		u32TimerWaitNullModem--;
	}
}

/**
 * \brief Periodic task to process Cycles App. Initialize and start Cycles Application and launch timer
 * to update internal counters.
 *
 */
void app_process(void)
{
	/* Call bootstrap process*/
	bs_process();

#ifdef  CONF_USE_PPP
	if (u32TimerWaitNullModem == 0) {
		null_modem_process();
	}
#endif
}
