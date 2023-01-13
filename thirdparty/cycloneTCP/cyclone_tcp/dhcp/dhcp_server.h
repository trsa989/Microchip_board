/**
 * @file dhcp_server.h
 * @brief DHCP server (Dynamic Host Configuration Protocol)
 *
 * @section License
 *
 * Copyright (C) 2010-2019 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneTCP Eval.
 *
 * This software is provided in source form for a short-term evaluation only. The
 * evaluation license expires 90 days after the date you first download the software.
 *
 * If you plan to use this software in a commercial product, you are required to
 * purchase a commercial license from Oryx Embedded SARL.
 *
 * After the 90-day evaluation period, you agree to either purchase a commercial
 * license or delete all copies of this software. If you wish to extend the
 * evaluation period, you must contact sales@oryx-embedded.com.
 *
 * This evaluation software is provided "as is" without warranty of any kind.
 * Technical support is available as an option during the evaluation period.
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 1.9.4
 **/

#ifndef _DHCP_SERVER_H
#define _DHCP_SERVER_H

//Dependencies
#include "core/net.h"

//DHCP server support
#ifndef DHCP_SERVER_SUPPORT
   #define DHCP_SERVER_SUPPORT DISABLED
#elif (DHCP_SERVER_SUPPORT != ENABLED && DHCP_SERVER_SUPPORT != DISABLED)
   #error DHCP_SERVER_SUPPORT parameter is not valid
#endif

//DHCP server tick interval
#ifndef DHCP_SERVER_TICK_INTERVAL
   #define DHCP_SERVER_TICK_INTERVAL 1000
#elif (DHCP_SERVER_TICK_INTERVAL < 10)
   #error DHCP_SERVER_TICK_INTERVAL parameter is not valid
#endif

//Maximum number of clients
#ifndef DHCP_SERVER_MAX_CLIENTS
   #define DHCP_SERVER_MAX_CLIENTS 16
#elif (DHCP_SERVER_MAX_CLIENTS < 1)
   #error DHCP_SERVER_MAX_CLIENTS parameter is not valid
#endif

//Default lease time, in seconds
#ifndef DHCP_SERVER_DEFAULT_LEASE_TIME
   #define DHCP_SERVER_DEFAULT_LEASE_TIME 86400
#elif (DHCP_SERVER_DEFAULT_LEASE_TIME < 1)
   #error DHCP_SERVER_DEFAULT_LEASE_TIME parameter is not valid
#endif

//Maximum number of DNS servers
#ifndef DHCP_SERVER_MAX_DNS_SERVERS
   #define DHCP_SERVER_MAX_DNS_SERVERS 2
#elif (DHCP_SERVER_MAX_DNS_SERVERS < 1)
   #error DHCP_SERVER_MAX_DNS_SERVERS parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
   extern "C" {
#endif


/**
 * @brief DHCP binding
 *
 * A binding is a collection of configuration parameters associated
 * with a DHCP client
 *
 **/

typedef struct
{
   MacAddr macAddr;     ///<Client's MAC address
   Ipv4Addr ipAddr;     ///<Client's IPv4 address
   bool_t validLease;   ///<Valid lease
   systime_t timestamp; ///<Timestamp
} DhcpServerBinding;


/**
 * @brief DHCP server settings
 **/

typedef struct
{
   NetInterface *interface;                         ///<Underlying network interface
   uint_t ipAddrIndex;                              ///<Index of the IP address assigned to the DHCP server
   bool_t rapidCommit;                              ///<Quick configuration using rapid commit
   uint32_t leaseTime;                              ///<Lease time, in seconds, assigned to the DHCP clients
   Ipv4Addr ipAddrRangeMin;                         ///<Lowest IP address in the pool that is available for dynamic address assignment
   Ipv4Addr ipAddrRangeMax;                         ///<Highest IP address in the pool that is available for dynamic address assignment
   Ipv4Addr subnetMask;                             ///<Subnet mask
   Ipv4Addr defaultGateway;                         ///<Default gateway
   Ipv4Addr dnsServer[DHCP_SERVER_MAX_DNS_SERVERS]; ///<DNS servers
} DhcpServerSettings;


/**
 * @brief DHCP server context
 **/

typedef struct
{
   DhcpServerSettings settings;                              ///<DHCP server settings
   bool_t running;                                           ///<This flag tells whether the DHCP server is running or not
   Ipv4Addr nextIpAddr;                                      ///<Next IP address to be assigned
   DhcpServerBinding clientBinding[DHCP_SERVER_MAX_CLIENTS]; ///<List of bindings
} DhcpServerContext;


//Tick counter to handle periodic operations
extern systime_t dhcpServerTickCounter;

//DHCP server related functions
void dhcpServerGetDefaultSettings(DhcpServerSettings *settings);
error_t dhcpServerInit(DhcpServerContext *context, const DhcpServerSettings *settings);
error_t dhcpServerStart(DhcpServerContext *context);
error_t dhcpServerStop(DhcpServerContext *context);

void dhcpServerTick(DhcpServerContext *context);

void dhcpServerProcessMessage(NetInterface *interface,
   const IpPseudoHeader *pseudoHeader, const UdpHeader *udpHeader,
   const NetBuffer *buffer, size_t offset, void *param);

void dhcpServerParseDiscover(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseRequest(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseDecline(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseRelease(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseInform(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

error_t dhcpServerSendReply(DhcpServerContext *context, uint8_t type,
   Ipv4Addr yourIpAddr, const DhcpMessage *request, size_t length);

DhcpServerBinding *dhcpServerCreateBinding(DhcpServerContext *context);

DhcpServerBinding *dhcpServerFindBindingByMacAddr(DhcpServerContext *context,
   const MacAddr *macAddr);

DhcpServerBinding *dhcpServerFindBindingByIpAddr(DhcpServerContext *context,
   Ipv4Addr ipAddr);

error_t dhcpServerGetNextIpAddr(DhcpServerContext *context, Ipv4Addr *ipAddr);

//C++ guard
#ifdef __cplusplus
   }
#endif

#endif
