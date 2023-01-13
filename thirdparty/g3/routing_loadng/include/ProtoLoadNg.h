/**********************************************************************************************************************/
/** \addtogroup LoadNG Protocol
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/** This file contains primitives of the routing protocol LoadNG
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/
#ifndef __PROTO_LOAD_NG_H__
#define __PROTO_LOAD_NG_H__

#include <RoutingTypes.h>

/**********************************************************************************************************************/
/** Flags
 ***********************************************************************************************************************/
#define LOADNG_FLAG_ROUTE_REPAIR 0x08
#define LOADNG_FLAG_UNICAST_RREQ 0x04


/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
void LOADNG_Reset(uint8_t u8Band, uint8_t u8SpecCompliance, struct TRoutingTables *g_RoutingTables);

/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
void LOADNG_DiscoverPath(uint16_t u16DstAddr, uint8_t u8MetricType, LOADNG_DiscoverPath_Callback callback);

/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
#ifdef G3_HYBRID_PROFILE
void LOADNG_ProcessMessage(uint16_t u16MacSrcAddr, uint8_t u8MediaType, enum EAdpMac_Modulation eModulation, uint8_t u8ActiveTones,
  uint8_t u8SubCarriers, uint8_t u8LQI, uint16_t u16MessageLength, uint8_t *pMessageBuffer);
#else
void LOADNG_ProcessMessage(uint16_t u16MacSrcAddr, enum EAdpMac_Modulation eModulation, uint8_t u8ActiveTones,
  uint8_t u8SubCarriers, uint8_t u8LQI, uint16_t u16MessageLength, uint8_t *pMessageBuffer);
#endif

/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
void LOADNG_NotifyRouteError(uint16_t u16DstAddr, uint16_t u16UnreachableAddress);

/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
void LOADNG_DiscoverRoute(uint16_t u16DstAddr, uint8_t u8MaxHops, bool bRepair, void *pUserData,
  LOADNG_DiscoverRoute_Callback fnctDiscoverCallback);


/**********************************************************************************************************************/
/** Refresh the valid time of the route
 **********************************************************************************************************************/
void LOADNG_RefreshRoute(uint16_t u16DstAddr, bool bRemoveBlacklist);

/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
void LOADNG_AddCircularRoute(uint16_t m_u16LastCircularRouteAddress);

/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
void LOADNG_DeleteRoute(uint16_t u16DstAddr, bool bBlacklist);

/**********************************************************************************************************************/
/**
 **********************************************************************************************************************/
void LOADNG_DeleteRoutePosition(uint32_t u32Position, bool bBlacklist);

/**********************************************************************************************************************/
/** returns true if route is known
 **********************************************************************************************************************/
bool LOADNG_RouteExists(uint16_t u16DestinationAddress);

/**********************************************************************************************************************/
/** before calling this function, check if route exists (LOADNG_RouteExists)
 **********************************************************************************************************************/
uint16_t LOADNG_GetRoute(uint16_t u16DestinationAddress);

#ifdef G3_HYBRID_PROFILE
/**********************************************************************************************************************/
/** before calling this function, check if route exists (LOADNG_RouteExists)
 **********************************************************************************************************************/
uint16_t LOADNG_GetRouteAndMediaType(uint16_t u16DestinationAddress, uint8_t *pu8MediaType);
#endif

/**********************************************************************************************************************/
/** Inserts a route in the routing table
 **********************************************************************************************************************/
struct TAdpRoutingTableEntry *LOADNG_AddRouteEntry(struct TAdpRoutingTableEntry *pNewEntry, bool *pbTableFull);

/**********************************************************************************************************************/
/** Add new candidate route
 **********************************************************************************************************************/
#ifdef G3_HYBRID_PROFILE
struct TAdpRoutingTableEntry *LOADNG_AddRoute(uint16_t u16DstAddr, uint16_t u16NextHopAddr, uint8_t u8MediaType, bool *pbTableFull);
#else
struct TAdpRoutingTableEntry *LOADNG_AddRoute(uint16_t u16DstAddr, uint16_t u16NextHopAddr, bool *pbTableFull);
#endif

/**********************************************************************************************************************/
/** Gets a pointer to Route Entry. before calling this function, check if route exists (LOADNG_RouteExists)
 **********************************************************************************************************************/
struct TAdpRoutingTableEntry *LOADNG_GetRouteEntry(uint16_t u16DestinationAddress);

/**********************************************************************************************************************/
/** Gets the route count
 **********************************************************************************************************************/
uint32_t LOADNG_GetRouteCount(void);

/**********************************************************************************************************************/
/** Returns true if the address is in the Destination Address Set (CCTT#183)
 **********************************************************************************************************************/
bool LOADNG_IsInDestinationAddressSet(uint16_t u16Addr);

/**********************************************************************************************************************/
/** Returns LOADNG MIB value
 **********************************************************************************************************************/
void LOADNG_GetMib(uint32_t u32AttributeId, uint16_t u16AttributeIndex, struct TAdpGetConfirm *pGetConfirm);

/**********************************************************************************************************************/
/** Sets LOADNG MIB value
 **********************************************************************************************************************/
void LOADNG_SetMib(uint32_t u32AttributeId, uint16_t u16AttributeIndex,
  uint8_t u8AttributeLength, const uint8_t *pu8AttributeValue, struct TAdpSetConfirm *pSetConfirm);

#ifdef G3_HYBRID_PROFILE
/**********************************************************************************************************************/
/** Adds node to blacklist for a given medium
 **********************************************************************************************************************/
void LOADNG_AddBlacklistOnMedium(uint16_t u16Addr, uint8_t u8MediaType);

/**********************************************************************************************************************/
/** Removes a node from blacklist for a given medium
 **********************************************************************************************************************/
void LOADNG_RemoveBlacklistOnMedium(uint16_t u16Addr, uint8_t u8MediaType);
#endif

#endif


/**********************************************************************************************************************/
/** @}
 **********************************************************************************************************************/

