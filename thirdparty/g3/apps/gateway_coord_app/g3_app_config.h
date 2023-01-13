#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "conf_project.h"
#include "conf_tables.h"

/* Port number for the application */
#define APP_SOCKET_PORT 0xF0B1
#ifdef CONF_USE_ETHERNET
#define APP_ETH_USE_SLAAC ENABLED /* Enable SLAAC Ip6 address autoconfiguration */
/* #define APP_ETH_USE_SLAAC DISABLED  //Use static configuration */
#else
#define APP_ETH_USE_SLAAC DISABLED /* Disable SLAAC Ip6 address autoconfiguration */
#endif

#define ETH_IF0 0
#define PPP_IF0 0
#define IF0_MAC_ADDR "00-AB-CD-EF-00-01"
#define IF0_IPV6_LINK_LOCAL_ADDR "fe80:0:0:0:781D:00ff:fe01:0000"

/* Primary IPv6 interface static configuration: Ethernet */
#ifdef CONF_USE_ETHERNET
#define IF0_IPV6_ULA_ADDR "FD00:0:0:1:00AB:CDff:feef:0001"
#define IF0_IPV6_NET_PREFIX  "FD00:0:0:1:0:0:0:0"
#define IF0_IPV6_NET_PREFIX_LEN  64
#define IF0_IPV6_DEFAULT_ROUTE_ADDR "FD00:0:0:0:0:0:0:0"
#define IF0_IPV6_DEFAULT_ROUTE_PREFIX_LEN  16
#endif

/*Primary IPv6 interface configuration: PPP */
#ifdef CONF_USE_PPP
#define IF0_IPV6_ULA_ADDR "FD00:0:0:2:00AB:CDff:feef:0001" /* Updated or fixed according to UPDATE_IF0_IPV6_ULA_ADDR_WITH_EUI64 */
/* Define if ULA Address will be updated by EUI64 or will remain fixed */
/* #define UPDATE_IF0_IPV6_ULA_ADDR_WITH_EUI64 */
#define UPDATE_IF0_IPV6_ULA_ADDR_WITH_EUI64
#define IF0_IPV6_NET_PREFIX  "FD00::0002:0:0:0:0"
#define IF0_IPV6_NET_PREFIX_LEN  64
#define IF0_IPV6_DEFAULT_ROUTE_ADDR "FD00:0:0:2:cafe:cafe:cafe:cafe" /* Modify this value with Computer PPP IPv6 Unicast Address configured */
#define IF0_IPV6_DEFAULT_ROUTE_PREFIX_LEN  16
#endif

/* Secondary IPv6 interface, Generic G3 Gateway coordinator address */
#define G3_IF1_MAC_ADDR "00-AB-CD-EF-00-00"
#define G3_IF1_IPV6_GENERIC_COORDINATOR_ADDR  "fe80::2:781D:ff:fe00:0000"
#define G3_IF1_IPV6_ULA_COORDINATOR_ADDR  "FD00:0:2:781D:00AB:CDff:feef:0000" /* ULA Address Fixed - NOT updated by EUI64 */
#define G3_IF1_IPV6_ROUTER_ADDR  "FD00:0:2:781D:0:0:0:0"
#define G3_IF1_IPV6_NET_PREFIX   "FD00:0:2:781D:0:0:0:0"
#define G3_IF1_IPV6_NET_PREFIX_LEN  64
#define G3_IF1 1

/* Generic G3 IPv6 local-link address */
#define APP_IPV6_GENERIC_LINK_LOCAL_ADDR "fe80::781D:ff:fe00:0001"

/* Default timeout for the application */
#define APP_DEFAULT_TIMEOUT 5000

/* Coordinator short address */
#define CONF_SHORT_ADDRESS (const uint8_t *)"\x00\x00"

/* PSK / Network authentication Key (16 bytes) */
#define CONF_PSK_KEY (const uint8_t *)"\xAB\x10\x34\x11\x45\x11\x1B\xC3\xC1\x2D\xE8\xFF\x11\x14\x22\x04"

/* GMK (16 bytes) */
#define CONF_GMK_KEY (const uint8_t *)"\xAF\x4D\x6D\xCC\xF1\x4D\xE7\xC1\xC4\x23\x5E\x6F\xEF\x6C\x15\x1F"

/* Context information table: index 0 (Context 0 with value c_IPv6_PREFIX & x_PAN_ID (length = 80)) */
#define CONF_CONTEXT_INFORMATION_TABLE_0 (const uint8_t *)"\x02\x00" "\x01" "\x50" "\xFE\x80\x00\x00\x00\x00\x00\x00\x78\x1D"

/* Context information table: index 1 (Context 1 with value �112233445566� (length = 48)) */
#define CONF_CONTEXT_INFORMATION_TABLE_1 (const uint8_t *)"\x02\x00" "\x01" "\x30" "\x11\x22\x33\x44\x55\x66"

/* Routing table entry TTL (2 bytes, little endian) (180 minutes) */
#define CONF_ROUTING_TABLE_ENTRY_TTL (const uint8_t *)"\xB4\x00"

/* GroupTable: index 0 (2 bytes, little endian) (0x8567 � note that the IPv6 layer must listen to ff12:30:1122:3344:5566:0:123:4567 in correspondence to this
 * group) */
#define CONF_GROUP_TABLE_0 (const uint8_t *)"\x67\x85"

/* Max Join Time: 90 seconds */
#define CONF_MAX_JOIN_WAIT_TIME (const uint8_t *)"\x5A"

/* Max Hops: 10 */
#define CONF_MAX_HOPS (const uint8_t *)"\x0A"

/* Max Number of DLMS devices (used for G3 GW and DLMS Client) */
#ifndef CONF_DLMS_MAX_DEV_NUM
  #define CONF_DLMS_MAX_DEV_NUM   50
#endif

#define DLMS_MAX_DEV_NUM   CONF_DLMS_MAX_DEV_NUM

#endif
