#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "conf_global.h"
#include "conf_tables.h"
#include "mac_wrapper.h"
#include "mac_wrapper_defs.h"
#include "MacApi.h"
#ifdef G3_HYBRID_PROFILE
#include "MacRfApi.h"
#endif /* G3_HYBRID_PROFILE */

#define LOG_LEVEL LOG_LVL_INFO
#include <Logger.h>

/* Check SPEC_COMPLIANCE definition */
#ifndef SPEC_COMPLIANCE
  #error "SPEC_COMPLIANCE undefined"
#endif

#ifndef CONF_MAC_NEIGHBOUR_TABLE_ENTRIES
  #define CONF_MAC_NEIGHBOUR_TABLE_ENTRIES      35
#endif

#ifndef CONF_MAC_POS_TABLE_ENTRIES
  #define CONF_MAC_POS_TABLE_ENTRIES            100
#endif

#ifndef CONF_MAC_DSN_SHORT_TABLE_ENTRIES
  #define CONF_MAC_DSN_SHORT_TABLE_ENTRIES      128
#endif

#ifndef CONF_MAC_DSN_EXTENDED_TABLE_ENTRIES
  #define CONF_MAC_DSN_EXTENDED_TABLE_ENTRIES   16
#endif

#define MAC_MAX_NEIGHBOUR_TABLE_ENTRIES         CONF_MAC_NEIGHBOUR_TABLE_ENTRIES
#define MAC_MAX_POS_TABLE_ENTRIES               CONF_MAC_POS_TABLE_ENTRIES
#define MAC_MAX_DSN_SHORT_TABLE_ENTRIES         CONF_MAC_DSN_SHORT_TABLE_ENTRIES
#define MAC_MAX_DSN_EXTENDED_TABLE_ENTRIES      CONF_MAC_DSN_EXTENDED_TABLE_ENTRIES

#define MAC_MAX_DEVICE_TABLE_ENTRIES (128)

static struct TMacWrpNotifications sUpperLayerNotifications;

static struct TMacTables sMacTables;
struct TNeighbourEntry macNeighbourTable[MAC_MAX_NEIGHBOUR_TABLE_ENTRIES];
struct TPOSEntry macPOSTable[MAC_MAX_POS_TABLE_ENTRIES];
struct TDeviceTableEntry macDeviceTable[MAC_MAX_DEVICE_TABLE_ENTRIES];
struct TDsnShortTableEntry macDsnShortTable[MAC_MAX_DSN_SHORT_TABLE_ENTRIES];
struct TDsnExtendedTableEntry macDsnExtendedTable[MAC_MAX_DSN_EXTENDED_TABLE_ENTRIES];

/* mac_wrapper callbacks */
static void _Callback_MacWrapperMcpsDataConfirm(struct TMcpsDataConfirm *pParameters)
{
	LOG_INFO(Log("MacWrapperMcpsDataConfirm() Handle: 0x%02X Status: %u", pParameters->m_u8MsduHandle, (uint8_t)pParameters->m_eStatus));

	if (sUpperLayerNotifications.m_MacWrpDataConfirm != NULL) {
		sUpperLayerNotifications.m_MacWrpDataConfirm((struct TMacWrpDataConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMcpsDataIndication(struct TMcpsDataIndication *pParameters)
{
	LOG_INFO(Log("MacWrapperMcpsDataIndication"));

	if (sUpperLayerNotifications.m_MacWrpDataIndication != NULL) {
		sUpperLayerNotifications.m_MacWrpDataIndication((struct TMacWrpDataIndication *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeGetConfirm(struct TMlmeGetConfirm *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeGetConfirm"));

	if (sUpperLayerNotifications.m_MacWrpGetConfirm != NULL) {
		sUpperLayerNotifications.m_MacWrpGetConfirm((struct TMacWrpGetConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeSetConfirm(struct TMlmeSetConfirm *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeSetConfirm"));

	if (sUpperLayerNotifications.m_MacWrpSetConfirm != NULL) {
		sUpperLayerNotifications.m_MacWrpSetConfirm((struct TMacWrpSetConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeResetConfirm(struct TMlmeResetConfirm *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeResetConfirm: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotifications.m_MacWrpResetConfirm != NULL) {
		sUpperLayerNotifications.m_MacWrpResetConfirm((struct TMacWrpResetConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeBeaconNotify(struct TMlmeBeaconNotifyIndication *pParameters)
{
	LOG_INFO(Log("MacWrapperMlmeBeaconNotify: Pan ID: %04X", pParameters->m_PanDescriptor.m_nPanId));

	if (sUpperLayerNotifications.m_MacWrpBeaconNotifyIndication != NULL) {
		sUpperLayerNotifications.m_MacWrpBeaconNotifyIndication((struct TMacWrpBeaconNotifyIndication *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeScanConfirm(struct TMlmeScanConfirm *pParameters)
{
	LOG_INFO(Log("MacWrapperMlmeScanConfirm: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotifications.m_MacWrpScanConfirm != NULL) {
		sUpperLayerNotifications.m_MacWrpScanConfirm((struct TMacWrpScanConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeStartConfirm(struct TMlmeStartConfirm *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeStartConfirm: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotifications.m_MacWrpStartConfirm != NULL) {
		sUpperLayerNotifications.m_MacWrpStartConfirm((struct TMacWrpStartConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeCommStatusIndication(struct TMlmeCommStatusIndication *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeCommStatusIndication: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotifications.m_MacWrpCommStatusIndication != NULL) {
		sUpperLayerNotifications.m_MacWrpCommStatusIndication((struct TMacWrpCommStatusIndication *)pParameters);
	}
}

static void _Callback_MacWrapperMcpsMacSnifferIndication(struct TMcpsMacSnifferIndication *pParameters)
{
	LOG_DBG(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, "MacWrapperMcpsMacSnifferIndication:  MSDU:"));

	if (sUpperLayerNotifications.m_MacWrpSnifferIndication != NULL) {
		sUpperLayerNotifications.m_MacWrpSnifferIndication((struct TMacWrpSnifferIndication *)pParameters);
	}
}

void MacWrapperInitialize(struct TMacWrpNotifications *pNotifications, uint8_t u8Band)
{
	struct TMacNotifications macWrapperNotifications;

	LOG_INFO(Log("MacWrapperInitialize: Initializing MAC..."));

	memcpy(&sUpperLayerNotifications, pNotifications, sizeof(struct TMacWrpNotifications));

	macWrapperNotifications.m_McpsDataConfirm = _Callback_MacWrapperMcpsDataConfirm;
	macWrapperNotifications.m_McpsDataIndication = _Callback_MacWrapperMcpsDataIndication;
	macWrapperNotifications.m_MlmeGetConfirm = _Callback_MacWrapperMlmeGetConfirm;
	macWrapperNotifications.m_MlmeSetConfirm = _Callback_MacWrapperMlmeSetConfirm;
	macWrapperNotifications.m_MlmeResetConfirm = _Callback_MacWrapperMlmeResetConfirm;
	macWrapperNotifications.m_MlmeBeaconNotifyIndication = _Callback_MacWrapperMlmeBeaconNotify;
	macWrapperNotifications.m_MlmeScanConfirm = _Callback_MacWrapperMlmeScanConfirm;
	macWrapperNotifications.m_MlmeStartConfirm = _Callback_MacWrapperMlmeStartConfirm;
	macWrapperNotifications.m_MlmeCommStatusIndication = _Callback_MacWrapperMlmeCommStatusIndication;
	macWrapperNotifications.m_McpsMacSnifferIndication = _Callback_MacWrapperMcpsMacSnifferIndication;

	memset(macNeighbourTable, 0, sizeof(macNeighbourTable));
	memset(macPOSTable, 0, sizeof(macPOSTable));
	memset(macDeviceTable, 0, sizeof(macDeviceTable));
	memset(macDsnShortTable, 0xFF, sizeof(macDsnShortTable));
	memset(macDsnExtendedTable, 0, sizeof(macDsnExtendedTable));

	sMacTables.m_MacDeviceTableSize = MAC_MAX_DEVICE_TABLE_ENTRIES;
	sMacTables.m_MacDsnShortTableSize = MAC_MAX_DSN_SHORT_TABLE_ENTRIES;
	sMacTables.m_MacDsnExtendedTableSize = MAC_MAX_DSN_EXTENDED_TABLE_ENTRIES;
	sMacTables.m_MacNeighbourTableSize = MAC_MAX_NEIGHBOUR_TABLE_ENTRIES;
	sMacTables.m_MacPosTableSize = MAC_MAX_POS_TABLE_ENTRIES;
	sMacTables.m_NeighbourTable = macNeighbourTable;
	sMacTables.m_PosTable = macPOSTable;
	sMacTables.m_DeviceTable = macDeviceTable;
	sMacTables.m_DsnShortTable = macDsnShortTable;
	sMacTables.m_DsnExtendedTable = macDsnExtendedTable;

	MacInitialize(&macWrapperNotifications, u8Band, &sMacTables, (uint8_t)SPEC_COMPLIANCE);
}

void MacWrapperEventHandler(void)
{
	MacEventHandler();
}

void MacWrapperMcpsDataRequest(struct TMacWrpDataRequest *pParameters)
{
#ifdef PRINT_ADP_TEST_VECTORS
	LOG_INFO(Log("---- TEST VECTOR START ----"));
	LOG_INFO(Log("PLC,"));
	LOG_INFO(Log("%02X,", pParameters->m_eSrcAddrMode));
	LOG_INFO(Log("%02X,", pParameters->m_DstAddr.m_eAddrMode));
	LOG_INFO(Log("%02X,", pParameters->m_nDstPanId));
	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		LOG_INFO(Log("%04X,", pParameters->m_DstAddr.m_nShortAddress));
	}
	else {
		LOG_INFO(Log("%02X%02X%02X%02X%02X%02X%02X%02X,", pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[1],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[2],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[3],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[4],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[5],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[6],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[7]));
	}
	LOG_INFO(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, ""));
	LOG_INFO(Log(",%02X,", pParameters->m_u8TxOptions));
	LOG_INFO(Log("%02X,", pParameters->m_eQualityOfService));
	LOG_INFO(Log("%02X,", pParameters->m_eSecurityLevel));
	LOG_INFO(Log("%02X,", pParameters->m_u8KeyIndex));
	LOG_INFO(Log("---- TEST VECTOR END ----"));
#else
	LOG_INFO(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, "MacWrapperMcpsDataRequest (Handle %02X): ", pParameters->m_u8MsduHandle));
#endif

	MacMcpsDataRequest((struct TMcpsDataRequest *)pParameters);
}

void MacWrapperMlmeGetRequest(struct TMacWrpGetRequest *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeGetRequest: Attribute: %08X; Index: %u", pParameters->m_ePibAttribute, pParameters->m_u16PibAttributeIndex));

	MacMlmeGetRequest((struct TMlmeGetRequest *)pParameters);
}

enum EMacWrpStatus MacWrapperMlmeGetRequestSync(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, struct TMacWrpPibValue *pValue)
{
	LOG_DBG(Log("MacWrapperMlmeGetRequestSync: Attribute: %08X; Index: %u", eAttribute, u16Index));

	return (enum EMacWrpStatus)(MacMlmeGetRequestSync((enum EMacPibAttribute)eAttribute, u16Index, (struct TMacPibValue *)pValue));
}

void MacWrapperMlmeSetRequest(struct TMacWrpSetRequest *pParameters)
{
	LOG_DBG(LogBuffer(pParameters->m_PibAttributeValue.m_au8Value, pParameters->m_PibAttributeValue.m_u8Length,
			"MacWrapperMlmeSetRequest: Attribute: %08X; Index: %u; Value: ", pParameters->m_ePibAttribute, pParameters->m_u16PibAttributeIndex));

	MacMlmeSetRequest((struct TMlmeSetRequest *)pParameters);
}

enum EMacWrpStatus MacWrapperMlmeSetRequestSync(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, const struct TMacWrpPibValue *pValue)
{
	LOG_DBG(LogBuffer(pValue->m_au8Value, pValue->m_u8Length, "MacWrapperMlmeSetRequestSync: Attribute: %08X; Index: %u; Value: ", eAttribute, u16Index));
	return (enum EMacWrpStatus)(MacMlmeSetRequestSync((enum EMacPibAttribute)eAttribute, u16Index, (const struct TMacPibValue *)pValue));
}

void MacWrapperMlmeResetRequest(struct TMacWrpResetRequest *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeResetRequest: Set default PIB: %u", pParameters->m_bSetDefaultPib));

	MacMlmeResetRequest((struct TMlmeResetRequest *)pParameters);
}

void MacWrapperMlmeScanRequest(struct TMacWrpScanRequest *pParameters)
{
	LOG_INFO(Log("MacWrapperMlmeScanRequest: Duration: %u", pParameters->m_u16ScanDuration));

	MacMlmeScanRequest((struct TMlmeScanRequest *)pParameters);
}

void MacWrapperMlmeStartRequest(struct TMacWrpStartRequest *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeStartRequest: Pan ID: %u", pParameters->m_nPanId));

	MacMlmeStartRequest((struct TMlmeStartRequest *)pParameters);
}

uint16_t MacWrapperGetNeighbourTableSize(void)
{
	return sMacTables.m_MacNeighbourTableSize;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
#ifdef G3_HYBRID_PROFILE

#ifndef CONF_MAC_POS_TABLE_ENTRIES_RF
  #define CONF_MAC_POS_TABLE_ENTRIES_RF    100
#endif

#ifndef CONF_MAC_DSN_TABLE_ENTRIES_RF
  #define CONF_MAC_DSN_TABLE_ENTRIES_RF    8
#endif

#define MAC_MAX_POS_TABLE_ENTRIES_RF       CONF_MAC_POS_TABLE_ENTRIES_RF
#define MAC_MAX_DSN_TABLE_ENTRIES_RF       CONF_MAC_DSN_TABLE_ENTRIES_RF

#define MAC_MAX_DEVICE_TABLE_ENTRIES_RF    (128)

static struct TMacWrpNotifications sUpperLayerNotificationsRF;

static struct TMacTablesRF sMacTablesRF;
struct TPOSEntryRF macPOSTableRF[MAC_MAX_POS_TABLE_ENTRIES_RF];
struct TDeviceTableEntry macDeviceTableRF[MAC_MAX_DEVICE_TABLE_ENTRIES_RF];
struct TDsnTableEntry macDsnTableRF[MAC_MAX_DSN_TABLE_ENTRIES_RF];

/* RF mac_wrapper callbacks */
static void _Callback_MacWrapperMcpsDataConfirmRF(struct TMcpsDataConfirm *pParameters)
{
	LOG_INFO(Log("MacWrapperMcpsDataConfirmRF() Handle: 0x%02X Status: %u", pParameters->m_u8MsduHandle, (uint8_t)pParameters->m_eStatus));

	if (sUpperLayerNotificationsRF.m_MacWrpDataConfirm != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpDataConfirm((struct TMacWrpDataConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMcpsDataIndicationRF(struct TMcpsDataIndication *pParameters)
{
	LOG_INFO(Log("MacWrapperMcpsDataIndicationRF"));

	if (sUpperLayerNotificationsRF.m_MacWrpDataIndication != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpDataIndication((struct TMacWrpDataIndication *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeGetConfirmRF(struct TMlmeGetConfirmRF *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeGetConfirmRF"));

	if (sUpperLayerNotificationsRF.m_MacWrpGetConfirm != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpGetConfirm((struct TMacWrpGetConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeSetConfirmRF(struct TMlmeSetConfirmRF *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeSetConfirmRF"));

	if (sUpperLayerNotificationsRF.m_MacWrpSetConfirm != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpSetConfirm((struct TMacWrpSetConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeResetConfirmRF(struct TMlmeResetConfirm *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeResetConfirmRF: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotificationsRF.m_MacWrpResetConfirm != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpResetConfirm((struct TMacWrpResetConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeBeaconNotifyRF(struct TMlmeBeaconNotifyIndication *pParameters)
{
	LOG_INFO(Log("MacWrapperMlmeBeaconNotifyRF: Pan ID: %04X", pParameters->m_PanDescriptor.m_nPanId));

	if (sUpperLayerNotificationsRF.m_MacWrpBeaconNotifyIndication != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpBeaconNotifyIndication((struct TMacWrpBeaconNotifyIndication *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeScanConfirmRF(struct TMlmeScanConfirm *pParameters)
{
	LOG_INFO(Log("MacWrapperMlmeScanConfirmRF: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotificationsRF.m_MacWrpScanConfirm != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpScanConfirm((struct TMacWrpScanConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeStartConfirmRF(struct TMlmeStartConfirm *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeStartConfirmRF: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotificationsRF.m_MacWrpStartConfirm != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpStartConfirm((struct TMacWrpStartConfirm *)pParameters);
	}
}

static void _Callback_MacWrapperMlmeCommStatusIndicationRF(struct TMlmeCommStatusIndication *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeCommStatusIndicationRF: Status: %u", pParameters->m_eStatus));

	if (sUpperLayerNotificationsRF.m_MacWrpCommStatusIndication != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpCommStatusIndication((struct TMacWrpCommStatusIndication *)pParameters);
	}
}

static void _Callback_MacWrapperMcpsMacSnifferIndicationRF(struct TMcpsMacSnifferIndication *pParameters)
{
	LOG_DBG(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, "MacWrapperMcpsMacSnifferIndicationRF:  MSDU:"));

	if (sUpperLayerNotificationsRF.m_MacWrpSnifferIndication != NULL) {
		sUpperLayerNotificationsRF.m_MacWrpSnifferIndication((struct TMacWrpSnifferIndication *)pParameters);
	}
}

void MacWrapperInitializeRF(struct TMacWrpNotifications *pNotifications)
{
	struct TMacNotificationsRF macWrapperNotificationsRF;

	LOG_INFO(Log("MacWrapperInitializeRF: Initializing RF MAC..."));

	memcpy(&sUpperLayerNotificationsRF, pNotifications, sizeof(struct TMacWrpNotifications));

	macWrapperNotificationsRF.m_McpsDataConfirm = _Callback_MacWrapperMcpsDataConfirmRF;
	macWrapperNotificationsRF.m_McpsDataIndication = _Callback_MacWrapperMcpsDataIndicationRF;
	macWrapperNotificationsRF.m_MlmeGetConfirm = _Callback_MacWrapperMlmeGetConfirmRF;
	macWrapperNotificationsRF.m_MlmeSetConfirm = _Callback_MacWrapperMlmeSetConfirmRF;
	macWrapperNotificationsRF.m_MlmeResetConfirm = _Callback_MacWrapperMlmeResetConfirmRF;
	macWrapperNotificationsRF.m_MlmeBeaconNotifyIndication = _Callback_MacWrapperMlmeBeaconNotifyRF;
	macWrapperNotificationsRF.m_MlmeScanConfirm = _Callback_MacWrapperMlmeScanConfirmRF;
	macWrapperNotificationsRF.m_MlmeStartConfirm = _Callback_MacWrapperMlmeStartConfirmRF;
	macWrapperNotificationsRF.m_MlmeCommStatusIndication = _Callback_MacWrapperMlmeCommStatusIndicationRF;
	macWrapperNotificationsRF.m_McpsMacSnifferIndication = _Callback_MacWrapperMcpsMacSnifferIndicationRF;

	memset(macPOSTableRF, 0, sizeof(macPOSTableRF));
	memset(macDeviceTableRF, 0, sizeof(macDeviceTableRF));
	memset(macDsnTableRF, 0, sizeof(macDsnTableRF));

	sMacTablesRF.m_MacDeviceTableSizeRF = MAC_MAX_DEVICE_TABLE_ENTRIES_RF;
	sMacTablesRF.m_MacDsnTableSizeRF = MAC_MAX_DSN_TABLE_ENTRIES_RF;
	sMacTablesRF.m_MacPosTableSizeRF = MAC_MAX_POS_TABLE_ENTRIES_RF;
	sMacTablesRF.m_PosTableRF = macPOSTableRF;
	sMacTablesRF.m_DeviceTableRF = macDeviceTableRF;
	sMacTablesRF.m_DsnTableRF = macDsnTableRF;

	MacInitializeRF(&macWrapperNotificationsRF, &sMacTablesRF);
}

void MacWrapperEventHandlerRF(void)
{
	MacEventHandlerRF();
}

void MacWrapperMcpsDataRequestRF(struct TMacWrpDataRequest *pParameters)
{
#ifdef PRINT_ADP_TEST_VECTORS
	LOG_INFO(Log("---- TEST VECTOR START ----"));
	LOG_INFO(Log("RF,"));
	LOG_INFO(Log("%02X,", pParameters->m_eSrcAddrMode));
	LOG_INFO(Log("%02X,", pParameters->m_DstAddr.m_eAddrMode));
	LOG_INFO(Log("%02X,", pParameters->m_nDstPanId));
	if (pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) {
		LOG_INFO(Log("%04X,", pParameters->m_DstAddr.m_nShortAddress));
	}
	else {
		LOG_INFO(Log("%02X%02X%02X%02X%02X%02X%02X%02X,", pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[0],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[1],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[2],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[3],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[4],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[5],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[6],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[7],
				pParameters->m_DstAddr.m_ExtendedAddress.m_au8Address[8]));
	}
	LOG_INFO(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, ""));
	LOG_INFO(Log(",%02X,", pParameters->m_u8TxOptions));
	LOG_INFO(Log("%02X,", pParameters->m_eQualityOfService));
	LOG_INFO(Log("%02X,", pParameters->m_eSecurityLevel));
	LOG_INFO(Log("%02X,", pParameters->m_u8KeyIndex));
	LOG_INFO(Log("---- TEST VECTOR END ----"));
#else
	LOG_INFO(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, "MacWrapperMcpsDataRequestRF (Handle %02X): ", pParameters->m_u8MsduHandle));
#endif

	MacMcpsDataRequestRF((struct TMcpsDataRequest *)pParameters);
}

void MacWrapperMlmeGetRequestRF(struct TMacWrpGetRequest *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeGetRequestRF: Attribute: %08X; Index: %u", pParameters->m_ePibAttribute, pParameters->m_u16PibAttributeIndex));

	MacMlmeGetRequestRF((struct TMlmeGetRequestRF *)pParameters);
}

enum EMacWrpStatus MacWrapperMlmeGetRequestSyncRF(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, struct TMacWrpPibValue *pValue)
{
	LOG_DBG(Log("MacWrapperMlmeGetRequestSyncRF: Attribute: %08X; Index: %u", eAttribute, u16Index));

	return (enum EMacWrpStatus)(MacMlmeGetRequestSyncRF((enum EMacPibAttributeRF)eAttribute, u16Index, (struct TMacPibValue *)pValue));
}

void MacWrapperMlmeSetRequestRF(struct TMacWrpSetRequest *pParameters)
{
	LOG_DBG(LogBuffer(pParameters->m_PibAttributeValue.m_au8Value, pParameters->m_PibAttributeValue.m_u8Length,
			"MacWrapperMlmeSetRequestRF: Attribute: %08X; Index: %u; Value: ", pParameters->m_ePibAttribute, pParameters->m_u16PibAttributeIndex));

	MacMlmeSetRequestRF((struct TMlmeSetRequestRF *)pParameters);
}

enum EMacWrpStatus MacWrapperMlmeSetRequestSyncRF(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, const struct TMacWrpPibValue *pValue)
{
	LOG_DBG(LogBuffer(pValue->m_au8Value, pValue->m_u8Length, "MacWrapperMlmeSetRequestSyncRF: Attribute: %08X; Index: %u; Value: ", eAttribute, u16Index));
	return (enum EMacWrpStatus)(MacMlmeSetRequestSyncRF((enum EMacPibAttributeRF)eAttribute, u16Index, (const struct TMacPibValue *)pValue));
}

void MacWrapperMlmeResetRequestRF(struct TMacWrpResetRequest *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeResetRequestRF: Set default PIB: %u", pParameters->m_bSetDefaultPib));

	MacMlmeResetRequestRF((struct TMlmeResetRequest *)pParameters);
}

void MacWrapperMlmeScanRequestRF(struct TMacWrpScanRequest *pParameters)
{
	LOG_INFO(Log("MacWrapperMlmeScanRequestRF: Duration: %u", pParameters->m_u16ScanDuration));

	MacMlmeScanRequestRF((struct TMlmeScanRequest *)pParameters);
}

void MacWrapperMlmeStartRequestRF(struct TMacWrpStartRequest *pParameters)
{
	LOG_DBG(Log("MacWrapperMlmeStartRequestRF: Pan ID: %u", pParameters->m_nPanId));

	MacMlmeStartRequestRF((struct TMlmeStartRequest *)pParameters);
}

#endif /* G3_HYBRID_PROFILE */
/**********************************************************************************************************************/
/**********************************************************************************************************************/
