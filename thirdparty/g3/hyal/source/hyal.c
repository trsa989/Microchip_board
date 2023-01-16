#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "hyal.h"
#include "mac_wrapper.h"
#include "mac_wrapper_defs.h"

#define LOG_LEVEL LOG_LVL_INFO
#include <Logger.h>

/* Buffer size to store data to be sent as Mac Data Request */
#define HYAL_BACKUP_BUF_SIZE   400

static const uint16_t crc16_tab[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

struct THyALDuplicatesEntry {
	uint16_t m_u16SrcAddr;
	uint16_t m_u16MsduLen;
	uint16_t m_u16Crc;
	uint8_t m_u8MediaType;
};

#define HYAL_DUPLICATES_TABLE_SIZE   3

static struct THyALDuplicatesEntry hyALDuplicatesTable[HYAL_DUPLICATES_TABLE_SIZE] = {{0}};

static struct THyALNotifications sUpperLayerNotifications;

struct THyALDataReq {
  struct TMacWrpDataRequest m_sDataReqParameters;
  enum EHyALMediaTypeRequest m_eDataReqMediaType;
  uint8_t m_au8BackupBuffer[HYAL_BACKUP_BUF_SIZE];
  enum EMacWrpStatus m_eFirstConfirmStatus;
  bool bWaitingSecondConfirm;
  bool bUsed;
};

#define HYAL_DATA_REQ_QUEUE_SIZE   2

struct THyALData {
  // Data Service Control
  struct THyALDataReq m_DataReqQueue[HYAL_DATA_REQ_QUEUE_SIZE];
  enum EMacWrpStatus m_eFirstScanConfirmStatus;
  bool bWaitingSecondScanConfirm;
  enum EMacWrpStatus m_eFirstResetConfirmStatus;
  bool bWaitingSecondResetConfirm;
  enum EMacWrpStatus m_eFirstStartConfirmStatus;
  bool bWaitingSecondStartConfirm;
};

static const struct THyALData g_HyALDefaults = {
  {{{{MAC_WRP_ADDRESS_MODE_NO_ADDRESS}}}}, // m_DataReqQueue
  MAC_WRP_STATUS_SUCCESS, // m_eFirstScanConfirmStatus
  false, // bWaitingSecondScanConfirm
  MAC_WRP_STATUS_SUCCESS, // m_eFirstResetConfirmStatus
  false, // bWaitingSecondResetConfirm
  MAC_WRP_STATUS_SUCCESS, // m_eFirstStartConfirmStatus
  false, // bWaitingSecondStartConfirm
};

static struct THyALData g_HyAL;

static uint16_t _HyALCrc16(const uint8_t *pu8Data, uint32_t u32Length)
{
	uint16_t u16Crc = 0;

	// polynom(16): X16 + X12 + X5 + 1 = 0x1021
	while (u32Length--) {
		u16Crc = crc16_tab[(u16Crc >> 8) ^ (*pu8Data ++)] ^ ((u16Crc & 0xFF) << 8);
	}
	return u16Crc;
}

static bool _checkDuplicates(uint16_t u16SrcAddr, uint8_t *pMsdu, uint16_t u16MsduLen, uint8_t u8MediaType)
{
	bool bDuplicate = false;
	uint8_t u8Index = 0;
	uint16_t u16Crc;

	// Calculate CRC for incoming frame
	u16Crc = _HyALCrc16(pMsdu, u16MsduLen);

	// Look for entry in the Duplicates Table
	struct THyALDuplicatesEntry *pEntry = &hyALDuplicatesTable[0];
	while (u8Index < HYAL_DUPLICATES_TABLE_SIZE) {
		// Look for same fields and different MediaType
		if ((pEntry->m_u16SrcAddr == u16SrcAddr) && (pEntry->m_u16MsduLen == u16MsduLen) && 
				(pEntry->m_u16Crc == u16Crc) && (pEntry->m_u8MediaType != u8MediaType)) {
			bDuplicate = true;
			break;
		}
		u8Index ++;
		pEntry ++;
	}

	if (!bDuplicate) {
		// Entry not found, store it
		memmove(&hyALDuplicatesTable[1], &hyALDuplicatesTable[0],
			(HYAL_DUPLICATES_TABLE_SIZE - 1) * sizeof(struct THyALDuplicatesEntry));
		// Populate the new entry.
		hyALDuplicatesTable[0].m_u16SrcAddr = u16SrcAddr;
		hyALDuplicatesTable[0].m_u16MsduLen = u16MsduLen;
		hyALDuplicatesTable[0].m_u16Crc = u16Crc;
		hyALDuplicatesTable[0].m_u8MediaType = u8MediaType;
	}

	// Return duplicate or not
	return bDuplicate;
}

static struct THyALDataReq *_getFreeDataReqEntry(void)
{
	uint8_t u8Idx;
	struct THyALDataReq *pFound = NULL;

	for (u8Idx = 0; u8Idx < HYAL_DATA_REQ_QUEUE_SIZE; u8Idx++) {
		if (!g_HyAL.m_DataReqQueue[u8Idx].bUsed) {
			pFound = &g_HyAL.m_DataReqQueue[u8Idx];
			g_HyAL.m_DataReqQueue[u8Idx].bUsed = true;
			LOG_INFO(Log("_getFreeDataReqEntry() Found free HyALDataReqQueueEntry on index %u", u8Idx));
			break;
		}
	}

	return pFound;
}

static struct THyALDataReq *_getDataReqEntryByHandler(uint8_t u8Handle)
{
	uint8_t u8Idx;
	struct THyALDataReq *pFound = NULL;

	for (u8Idx = 0; u8Idx < HYAL_DATA_REQ_QUEUE_SIZE; u8Idx++) {
		if (g_HyAL.m_DataReqQueue[u8Idx].bUsed && 
				(g_HyAL.m_DataReqQueue[u8Idx].m_sDataReqParameters.m_u8MsduHandle == u8Handle)) {
			pFound = &g_HyAL.m_DataReqQueue[u8Idx];
			LOG_INFO(Log("_getDataReqEntryByHandler() Found matching HyALDataReqQueueEntry on index %u, Handle: 0x%02X", u8Idx, u8Handle));
			break;
		}
	}

	return pFound;
}

/* ------------------------------------------------ */
/* ---------------- HyAL callbacks ---------------- */
/* ------------------------------------------------ */

static void _Callback_HyALMacWrpDataConfirm(struct TMacWrpDataConfirm *pParameters)
{
	struct THyALDataReq *pMatchingDataReq;
	struct THyALDataConfirm confParameters;
	struct TMacWrpPibValue pibValue;
	enum EMacWrpStatus status;

	LOG_INFO(Log("HyALMacWrpDataConfirm() Handle: 0x%02X Status: %u", pParameters->m_u8MsduHandle, (uint8_t)pParameters->m_eStatus));

	/* Get Data Request entry matching confirm */
	pMatchingDataReq = _getDataReqEntryByHandler(pParameters->m_u8MsduHandle);

	/* Avoid unmached handling */
	if (pMatchingDataReq == NULL) {
		LOG_ERR(Log("HyALMacWrpDataConfirm() Confirm does not match any previous request!!"));
		return;
	}

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	memcpy(&confParameters, pParameters, sizeof(struct TMacWrpDataConfirm));
	
	switch (pMatchingDataReq->m_eDataReqMediaType) {
		case HYAL_MEDIA_TYPE_REQ_PLC_BACKUP_RF:
			if (pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS) {
				/* Send confirm to upper layer */
				confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_PLC;
				/* Release Data Req entry and send confirm */
				pMatchingDataReq->bUsed = false;
				if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
					sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
				}
			}
			else {
				/* Check Dest Address mode and/or RF POS table before attempting data request */
				if (pMatchingDataReq->m_sDataReqParameters.m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
					status = MAC_WRP_STATUS_SUCCESS;
					LOG_INFO(Log("Extended Address Dest allows backup medium"));
				}
				else {
					LOG_INFO(Log("Look for RF POS Table entry for %0004X", pMatchingDataReq->m_sDataReqParameters.m_DstAddr.m_nShortAddress));
					status = MacWrapperMlmeGetRequestSyncRF(MAC_WRP_PIB_MANUF_POS_TABLE_ELEMENT_RF, 
							pMatchingDataReq->m_sDataReqParameters.m_DstAddr.m_nShortAddress, &pibValue);
				}
				if (status == MAC_WRP_STATUS_SUCCESS) {
					/* Try on backup medium */
					LOG_INFO(Log("Try RF as Backup Meduim"));
					/* Set Msdu pointer to backup buffer, as current pointer is no longer valid */
					pMatchingDataReq->m_sDataReqParameters.m_pMsdu = pMatchingDataReq->m_au8BackupBuffer;
					MacWrapperMcpsDataRequestRF(&pMatchingDataReq->m_sDataReqParameters);
				}
				else {
					LOG_INFO(Log("No POS entry found, discard backup medium"));
					/* Send confirm to upper layer */
					confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_PLC;
					/* Release Data Req entry and send confirm */
					pMatchingDataReq->bUsed = false;
					if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
						sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
					}
				}
			}
			break;
		case HYAL_MEDIA_TYPE_REQ_RF_BACKUP_PLC:
			/* PLC was used as backup medium. Send confirm to upper layer */
			confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_PLC_AS_BACKUP;
			/* Release Data Req entry and send confirm */
			pMatchingDataReq->bUsed = false;
			if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
				sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
			}
			break;
		case HYAL_MEDIA_TYPE_REQ_BOTH:
			if (pMatchingDataReq->bWaitingSecondConfirm) {
				/* Second Confirm arrived. Send confirm to upper layer depending on results */
				confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_BOTH;
				if ((pMatchingDataReq->m_eFirstConfirmStatus == MAC_WRP_STATUS_SUCCESS) ||
						(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
					/* At least one SUCCESS, send confirm with SUCCESS */
					confParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
					/* Release Data Req entry and send confirm */
					pMatchingDataReq->bUsed = false;
					if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
						sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
					}
				}
				else {
					/* None SUCCESS. Return result from second confirm */
					confParameters.m_eStatus = pParameters->m_eStatus;
					/* Release Data Req entry and send confirm */
					pMatchingDataReq->bUsed = false;
					if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
						sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
					}
				}
			}
			else {
				/* This is the First Confirm, store status and wait for Second */
				pMatchingDataReq->m_eFirstConfirmStatus = pParameters->m_eStatus;
				pMatchingDataReq->bWaitingSecondConfirm = true;
			}
			break;
		case HYAL_MEDIA_TYPE_REQ_PLC_NO_BACKUP:
			/* Send confirm to upper layer */
			confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_PLC;
			/* Release Data Req entry and send confirm */
			pMatchingDataReq->bUsed = false;
			if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
				sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
			}
			break;
		case HYAL_MEDIA_TYPE_REQ_RF_NO_BACKUP:
			/* PLC confirm not expected on RF_NO_BACKUP request. Ignore it */
			break;
		default: /* PLC only */
			confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_PLC;
			/* Release Data Req entry and send confirm */
			pMatchingDataReq->bUsed = false;
			if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
				sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
			}
			break;
	}
}

static void _Callback_HyALMacWrpDataConfirmRF(struct TMacWrpDataConfirm *pParameters)
{
	struct THyALDataReq *pMatchingDataReq;
	struct THyALDataConfirm confParameters;
	struct TMacWrpPibValue pibValue;
	enum EMacWrpStatus status;

	LOG_INFO(Log("HyALMacWrpDataConfirmRF() Handle: 0x%02X Status: %u", pParameters->m_u8MsduHandle, (uint8_t)pParameters->m_eStatus));

	/* Get Data Request entry matching confirm */
	pMatchingDataReq = _getDataReqEntryByHandler(pParameters->m_u8MsduHandle);

	/* Avoid unmached handling */
	if (pMatchingDataReq == NULL) {
		LOG_ERR(Log("HyALMacWrpDataConfirm() Confirm does not match any previous request!!"));
		return;
	}

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	memcpy(&confParameters, pParameters, sizeof(struct TMacWrpDataConfirm));
	
	switch (pMatchingDataReq->m_eDataReqMediaType) {
		case HYAL_MEDIA_TYPE_REQ_PLC_BACKUP_RF:
			/* RF was used as backup medium. Send confirm to upper layer */
			confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_RF_AS_BACKUP;
			/* Release Data Req entry and send confirm */
			pMatchingDataReq->bUsed = false;
			if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
				sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
			}
			break;
		case HYAL_MEDIA_TYPE_REQ_RF_BACKUP_PLC:
			if (pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS) {
				/* Send confirm to upper layer */
				confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_RF;
				/* Release Data Req entry and send confirm */
				pMatchingDataReq->bUsed = false;
				if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
					sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
				}
			}
			else {
				/* Check Dest Address mode and/or RF POS table before attempting data request */
				if (pMatchingDataReq->m_sDataReqParameters.m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_EXTENDED) {
					status = MAC_WRP_STATUS_SUCCESS;
					LOG_INFO(Log("Extended Address Dest allows backup medium"));
				}
				else {
					LOG_INFO(Log("Look for PLC POS Table entry for %0004X", pMatchingDataReq->m_sDataReqParameters.m_DstAddr.m_nShortAddress));
					status = MacWrapperMlmeGetRequestSync(MAC_WRP_PIB_MANUF_POS_TABLE_ELEMENT, 
							pMatchingDataReq->m_sDataReqParameters.m_DstAddr.m_nShortAddress, &pibValue);
				}
				if (status == MAC_WRP_STATUS_SUCCESS) {
					/* Try on backup medium */
					LOG_INFO(Log("Try PLC as Backup Meduim"));
					/* Set Msdu pointer to backup buffer, as current pointer is no longer valid */
					pMatchingDataReq->m_sDataReqParameters.m_pMsdu = pMatchingDataReq->m_au8BackupBuffer;
					MacWrapperMcpsDataRequest(&pMatchingDataReq->m_sDataReqParameters);
				}
				else {
					LOG_INFO(Log("No POS entry found, discard backup medium"));
					/* Send confirm to upper layer */
					confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_RF;
					/* Release Data Req entry and send confirm */
					pMatchingDataReq->bUsed = false;
					if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
						sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
					}
				}
			}
			break;
		case HYAL_MEDIA_TYPE_REQ_BOTH:
			if (pMatchingDataReq->bWaitingSecondConfirm) {
				/* Second Confirm arrived. Send confirm to upper layer depending on results */
				confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_BOTH;
				if ((pMatchingDataReq->m_eFirstConfirmStatus == MAC_WRP_STATUS_SUCCESS) ||
						(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
					/* At least one SUCCESS, send confirm with SUCCESS */
					confParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
					/* Release Data Req entry and send confirm */
					pMatchingDataReq->bUsed = false;
					if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
						sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
					}
				}
				else {
					/* None SUCCESS. Return result from second confirm */
					confParameters.m_eStatus = pParameters->m_eStatus;
					/* Release Data Req entry and send confirm */
					pMatchingDataReq->bUsed = false;
					if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
						sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
					}
				}
			}
			else {
				/* This is the First Confirm, store status and wait for Second */
				pMatchingDataReq->m_eFirstConfirmStatus = pParameters->m_eStatus;
				pMatchingDataReq->bWaitingSecondConfirm = true;
			}
			break;
		case HYAL_MEDIA_TYPE_REQ_PLC_NO_BACKUP:
			/* RF confirm not expected after a PLC_NO_BACKUP request. Ignore it */
			break;
		case HYAL_MEDIA_TYPE_REQ_RF_NO_BACKUP:
			/* Send confirm to upper layer */
			confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_RF;
			/* Release Data Req entry and send confirm */
			pMatchingDataReq->bUsed = false;
			if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
				sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
			}
			break;
		default: /* RF only */
			confParameters.m_eMediaType = HYAL_MEDIA_TYPE_CONF_RF;
			/* Release Data Req entry and send confirm */
			pMatchingDataReq->bUsed = false;
			if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
				sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
			}
			break;
	}
}

static void _Callback_HyALMacWrpDataIndication(struct TMacWrpDataIndication *pParameters)
{
	struct THyALDataIndication dataIndParameters;

	LOG_INFO(Log("HyALMacWrpDataIndication"));

	/* Check if the same frame has been received on the other medium (duplicate detection), except for broadcast */
	if (MAC_WRP_SHORT_ADDRESS_BROADCAST != pParameters->m_DstAddr.m_nShortAddress) {
		LOG_INFO(Log("HyALMacWrpDataIndication MAC_WRP_SHORT_ADDRESS_BROADCAST != pParameters->m_DstAddr.m_nShortAddress"));
		if (_checkDuplicates(pParameters->m_SrcAddr.m_nShortAddress, pParameters->m_pMsdu, 
			pParameters->m_u16MsduLength, HYAL_MEDIA_TYPE_IND_PLC)) {
			LOG_INFO(Log("HyALMacWrpDataIndication checkDuplicates(pParameters->m_SrcAddr.m_nShortAddress, pParameters->m_pMsdu, pParameters->m_u16MsduLength, HYAL_MEDIA_TYPE_IND_PLC"));
			/* Same frame was received on RF medium. Drop indication */
			LOG_INFO(Log("Same frame was received on RF medium. Drop indication"));
			return;
		}
	}

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	LOG_INFO(Log("HyALMacWrpDataIndication memcpy(&dataIndParameters, pParameters, sizeof(struct TMacWrpDataIndication))"));
	memcpy(&dataIndParameters, pParameters, sizeof(struct TMacWrpDataIndication));

	if (sUpperLayerNotifications.m_HyALDataIndication != NULL) {
		LOG_INFO(Log("HyALMacWrpDataIndication "));
		dataIndParameters.m_eMediaType = HYAL_MEDIA_TYPE_IND_PLC;
		sUpperLayerNotifications.m_HyALDataIndication(&dataIndParameters);
	}
}

static void _Callback_HyALMacWrpDataIndicationRF(struct TMacWrpDataIndication *pParameters)
{
	struct THyALDataIndication dataIndParameters;

	LOG_INFO(Log("HyALMacWrpDataIndicationRF"));

	/* Check if the same frame has been received on the other medium (duplicate detection), except for broadcast */
	if (MAC_WRP_SHORT_ADDRESS_BROADCAST != pParameters->m_DstAddr.m_nShortAddress) {
		if (_checkDuplicates(pParameters->m_SrcAddr.m_nShortAddress, pParameters->m_pMsdu, 
			pParameters->m_u16MsduLength, HYAL_MEDIA_TYPE_IND_RF)) {
			/* Same frame was received on PLC medium. Drop indication */
			LOG_INFO(Log("Same frame was received on PLC medium. Drop indication"));
			return;
		}
	}

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	memcpy(&dataIndParameters, pParameters, sizeof(struct TMacWrpDataIndication));

	if (sUpperLayerNotifications.m_HyALDataIndication != NULL) {
		dataIndParameters.m_eMediaType = HYAL_MEDIA_TYPE_IND_RF;
		sUpperLayerNotifications.m_HyALDataIndication(&dataIndParameters);
	}
}

static void _Callback_HyALMacWrpGetConfirm(struct TMacWrpGetConfirm *pParameters)
{
	LOG_DBG(Log("HyALMacWrpGetConfirm"));

	if (sUpperLayerNotifications.m_HyALGetConfirm != NULL) {
		sUpperLayerNotifications.m_HyALGetConfirm((struct THyALGetConfirm *)pParameters);
	}
}

static void _Callback_HyALMacWrpGetConfirmRF(struct TMacWrpGetConfirm *pParameters)
{
	LOG_DBG(Log("HyALMacWrpGetConfirmRF"));

	if (sUpperLayerNotifications.m_HyALGetConfirm != NULL) {
		sUpperLayerNotifications.m_HyALGetConfirm((struct THyALGetConfirm *)pParameters);
	}
}

static void _Callback_HyALMacWrpSetConfirm(struct TMacWrpSetConfirm *pParameters)
{
	LOG_DBG(Log("HyALMacWrpSetConfirm"));

	if (sUpperLayerNotifications.m_HyALSetConfirm != NULL) {
		sUpperLayerNotifications.m_HyALSetConfirm((struct THyALSetConfirm *)pParameters);
	}
}

static void _Callback_HyALMacWrpSetConfirmRF(struct TMacWrpSetConfirm *pParameters)
{
	LOG_DBG(Log("HyALMacWrpSetConfirmRF"));

	if (sUpperLayerNotifications.m_HyALSetConfirm != NULL) {
		sUpperLayerNotifications.m_HyALSetConfirm((struct THyALSetConfirm *)pParameters);
	}
}

static void _Callback_HyALMacWrpResetConfirm(struct TMacWrpResetConfirm *pParameters)
{
	struct THyALResetConfirm resetConfParameters;

	LOG_DBG(Log("HyALMacWrpResetConfirm: Status: %u", pParameters->m_eStatus));
	
	if (g_HyAL.bWaitingSecondResetConfirm) {
		/* Second Confirm arrived. Send confirm to upper layer depending on results */
		if ((g_HyAL.m_eFirstResetConfirmStatus == MAC_WRP_STATUS_SUCCESS) &&
				(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
			/* Both SUCCESS, send confirm with SUCCESS */
			resetConfParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
			if (sUpperLayerNotifications.m_HyALResetConfirm != NULL) {
				sUpperLayerNotifications.m_HyALResetConfirm(&resetConfParameters);
			}
		}
		else {
			/* Check which reset failed and report its status */
			if (g_HyAL.m_eFirstResetConfirmStatus != MAC_WRP_STATUS_SUCCESS) {
				resetConfParameters.m_eStatus = g_HyAL.m_eFirstResetConfirmStatus;
				if (sUpperLayerNotifications.m_HyALResetConfirm != NULL) {
					sUpperLayerNotifications.m_HyALResetConfirm(&resetConfParameters);
				}
			}
			else {
				resetConfParameters.m_eStatus = pParameters->m_eStatus;
				if (sUpperLayerNotifications.m_HyALResetConfirm != NULL) {
					sUpperLayerNotifications.m_HyALResetConfirm(&resetConfParameters);
				}
			}
		}
	}
	else {
		/* This is the First Confirm, store status and wait for Second */
		g_HyAL.m_eFirstResetConfirmStatus = pParameters->m_eStatus;
		g_HyAL.bWaitingSecondResetConfirm = true;
	}
}

static void _Callback_HyALMacWrpResetConfirmRF(struct TMacWrpResetConfirm *pParameters)
{
	struct THyALResetConfirm resetConfParameters;

	LOG_DBG(Log("HyALMacWrpResetConfirmRF: Status: %u", pParameters->m_eStatus));
	
	if (g_HyAL.bWaitingSecondResetConfirm) {
		/* Second Confirm arrived. Send confirm to upper layer depending on results */
		if ((g_HyAL.m_eFirstResetConfirmStatus == MAC_WRP_STATUS_SUCCESS) &&
				(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
			/* Both SUCCESS, send confirm with SUCCESS */
			resetConfParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
			if (sUpperLayerNotifications.m_HyALResetConfirm != NULL) {
				sUpperLayerNotifications.m_HyALResetConfirm(&resetConfParameters);
			}
		}
		else {
			/* Check which reset failed and report its status */
			if (g_HyAL.m_eFirstResetConfirmStatus != MAC_WRP_STATUS_SUCCESS) {
				resetConfParameters.m_eStatus = g_HyAL.m_eFirstResetConfirmStatus;
				if (sUpperLayerNotifications.m_HyALResetConfirm != NULL) {
					sUpperLayerNotifications.m_HyALResetConfirm(&resetConfParameters);
				}
			}
			else {
				resetConfParameters.m_eStatus = pParameters->m_eStatus;
				if (sUpperLayerNotifications.m_HyALResetConfirm != NULL) {
					sUpperLayerNotifications.m_HyALResetConfirm(&resetConfParameters);
				}
			}
		}
	}
	else {
		/* This is the First Confirm, store status and wait for Second */
		g_HyAL.m_eFirstResetConfirmStatus = pParameters->m_eStatus;
		g_HyAL.bWaitingSecondResetConfirm = true;
	}
}

static void _Callback_HyALMacWrpBeaconNotify(struct TMacWrpBeaconNotifyIndication *pParameters)
{
	struct THyALBeaconNotifyIndication notifyIndication;

	LOG_INFO(Log("HyALMacWrpBeaconNotify: Pan ID: %04X", pParameters->m_PanDescriptor.m_nPanId));

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	memcpy(&notifyIndication, pParameters, sizeof(struct TMacWrpBeaconNotifyIndication));

	if (sUpperLayerNotifications.m_HyALBeaconNotifyIndication != NULL) {
		notifyIndication.m_PanDescriptor.m_eMediaType = HYAL_MEDIA_TYPE_IND_PLC;
		sUpperLayerNotifications.m_HyALBeaconNotifyIndication(&notifyIndication);
	}
}

static void _Callback_HyALMacWrpBeaconNotifyRF(struct TMacWrpBeaconNotifyIndication *pParameters)
{
	struct THyALBeaconNotifyIndication notifyIndication;
	 
	LOG_INFO(Log("HyALMacWrpBeaconNotifyRF: Pan ID: %04X", pParameters->m_PanDescriptor.m_nPanId));

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	memcpy(&notifyIndication, pParameters, sizeof(struct TMacWrpBeaconNotifyIndication));

	if (sUpperLayerNotifications.m_HyALBeaconNotifyIndication != NULL) {
		notifyIndication.m_PanDescriptor.m_eMediaType = HYAL_MEDIA_TYPE_IND_RF;
		sUpperLayerNotifications.m_HyALBeaconNotifyIndication(&notifyIndication);
	}
}

static void _Callback_HyALMacWrpScanConfirm(struct TMacWrpScanConfirm *pParameters)
{
	struct THyALScanConfirm scanConfParameters;

	LOG_INFO(Log("HyALMacWrpScanConfirm: Status: %u", pParameters->m_eStatus));
	
	if (g_HyAL.bWaitingSecondScanConfirm) {
		/* Second Confirm arrived. Send confirm to upper layer depending on results */
		if ((g_HyAL.m_eFirstScanConfirmStatus == MAC_WRP_STATUS_SUCCESS) ||
				(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
			/* One or Both SUCCESS, send confirm with SUCCESS */
			scanConfParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
			if (sUpperLayerNotifications.m_HyALScanConfirm != NULL) {
				sUpperLayerNotifications.m_HyALScanConfirm(&scanConfParameters);
			}
		}
		else {
			/* None of confirms SUCCESS, send confirm with latests status */
			scanConfParameters.m_eStatus = pParameters->m_eStatus;
			if (sUpperLayerNotifications.m_HyALScanConfirm != NULL) {
				sUpperLayerNotifications.m_HyALScanConfirm(&scanConfParameters);
			}
		}
	}
	else {
		/* This is the First Confirm, store status and wait for Second */
		g_HyAL.m_eFirstScanConfirmStatus = pParameters->m_eStatus;
		g_HyAL.bWaitingSecondScanConfirm = true;
	}
}

static void _Callback_HyALMacWrpScanConfirmRF(struct TMacWrpScanConfirm *pParameters)
{
	struct THyALScanConfirm scanConfParameters;

	LOG_INFO(Log("HyALMacWrpScanConfirmRF: Status: %u", pParameters->m_eStatus));
	
	if (g_HyAL.bWaitingSecondScanConfirm) {
		/* Second Confirm arrived. Send confirm to upper layer depending on results */
		if ((g_HyAL.m_eFirstScanConfirmStatus == MAC_WRP_STATUS_SUCCESS) ||
				(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
			/* One or Both SUCCESS, send confirm with SUCCESS */
			scanConfParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
			if (sUpperLayerNotifications.m_HyALScanConfirm != NULL) {
				sUpperLayerNotifications.m_HyALScanConfirm(&scanConfParameters);
			}
		}
		else {
			/* None of confirms SUCCESS, send confirm with latests status */
			scanConfParameters.m_eStatus = pParameters->m_eStatus;
			if (sUpperLayerNotifications.m_HyALScanConfirm != NULL) {
				sUpperLayerNotifications.m_HyALScanConfirm(&scanConfParameters);
			}
		}
	}
	else {
		/* This is the First Confirm, store status and wait for Second */
		g_HyAL.m_eFirstScanConfirmStatus = pParameters->m_eStatus;
		g_HyAL.bWaitingSecondScanConfirm = true;
	}
}

static void _Callback_HyALMacWrpStartConfirm(struct TMacWrpStartConfirm *pParameters)
{
	struct THyALStartConfirm startConfParameters;

	LOG_DBG(Log("HyALMacWrpStartConfirm: Status: %u", pParameters->m_eStatus));
	
	if (g_HyAL.bWaitingSecondStartConfirm) {
		/* Second Confirm arrived. Send confirm to upper layer depending on results */
		if ((g_HyAL.m_eFirstStartConfirmStatus == MAC_WRP_STATUS_SUCCESS) &&
				(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
			/* Both SUCCESS, send confirm with SUCCESS */
			startConfParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
			if (sUpperLayerNotifications.m_HyALStartConfirm != NULL) {
				sUpperLayerNotifications.m_HyALStartConfirm(&startConfParameters);
			}
		}
		else {
			/* Check which start failed and report its status */
			if (g_HyAL.m_eFirstStartConfirmStatus != MAC_WRP_STATUS_SUCCESS) {
				startConfParameters.m_eStatus = g_HyAL.m_eFirstStartConfirmStatus;
				if (sUpperLayerNotifications.m_HyALStartConfirm != NULL) {
					sUpperLayerNotifications.m_HyALStartConfirm(&startConfParameters);
				}
			}
			else {
				startConfParameters.m_eStatus = pParameters->m_eStatus;
				if (sUpperLayerNotifications.m_HyALStartConfirm != NULL) {
					sUpperLayerNotifications.m_HyALStartConfirm(&startConfParameters);
				}
			}
		}
	}
	else {
		/* This is the First Confirm, store status and wait for Second */
		g_HyAL.m_eFirstStartConfirmStatus = pParameters->m_eStatus;
		g_HyAL.bWaitingSecondStartConfirm = true;
	}
}

static void _Callback_HyALMacWrpStartConfirmRF(struct TMacWrpStartConfirm *pParameters)
{
	struct THyALStartConfirm startConfParameters;

	LOG_DBG(Log("HyALMacWrpStartConfirmRF: Status: %u", pParameters->m_eStatus));
	
	if (g_HyAL.bWaitingSecondStartConfirm) {
		/* Second Confirm arrived. Send confirm to upper layer depending on results */
		if ((g_HyAL.m_eFirstStartConfirmStatus == MAC_WRP_STATUS_SUCCESS) &&
				(pParameters->m_eStatus == MAC_WRP_STATUS_SUCCESS)) {
			/* Both SUCCESS, send confirm with SUCCESS */
			startConfParameters.m_eStatus = MAC_WRP_STATUS_SUCCESS;
			if (sUpperLayerNotifications.m_HyALStartConfirm != NULL) {
				sUpperLayerNotifications.m_HyALStartConfirm(&startConfParameters);
			}
		}
		else {
			/* Check which start failed and report its status */
			if (g_HyAL.m_eFirstStartConfirmStatus != MAC_WRP_STATUS_SUCCESS) {
				startConfParameters.m_eStatus = g_HyAL.m_eFirstStartConfirmStatus;
				if (sUpperLayerNotifications.m_HyALStartConfirm != NULL) {
					sUpperLayerNotifications.m_HyALStartConfirm(&startConfParameters);
				}
			}
			else {
				startConfParameters.m_eStatus = pParameters->m_eStatus;
				if (sUpperLayerNotifications.m_HyALStartConfirm != NULL) {
					sUpperLayerNotifications.m_HyALStartConfirm(&startConfParameters);
				}
			}
		}
	}
	else {
		/* This is the First Confirm, store status and wait for Second */
		g_HyAL.m_eFirstStartConfirmStatus = pParameters->m_eStatus;
		g_HyAL.bWaitingSecondStartConfirm = true;
	}
}

static void _Callback_HyALMacWrpCommStatusIndication(struct TMacWrpCommStatusIndication *pParameters)
{
	struct THyALCommStatusIndication commStatusIndication;

	LOG_DBG(Log("HyALMacWrpCommStatusIndication: Status: %u", pParameters->m_eStatus));

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	memcpy(&commStatusIndication, pParameters, sizeof(struct TMacWrpCommStatusIndication));

	if (sUpperLayerNotifications.m_HyALCommStatusIndication != NULL) {
		commStatusIndication.m_eMediaType = HYAL_MEDIA_TYPE_IND_PLC;
		sUpperLayerNotifications.m_HyALCommStatusIndication(&commStatusIndication);
	}
}

static void _Callback_HyALMacWrpCommStatusIndicationRF(struct TMacWrpCommStatusIndication *pParameters)
{
	struct THyALCommStatusIndication commStatusIndication;

	LOG_DBG(Log("HyALMacWrpCommStatusIndicationRF: Status: %u", pParameters->m_eStatus));

	/* Copy parameters from Mac Wrapper. Media Type will be filled later */
	memcpy(&commStatusIndication, pParameters, sizeof(struct TMacWrpCommStatusIndication));

	if (sUpperLayerNotifications.m_HyALCommStatusIndication != NULL) {
		commStatusIndication.m_eMediaType = HYAL_MEDIA_TYPE_IND_RF;
		sUpperLayerNotifications.m_HyALCommStatusIndication(&commStatusIndication);
	}
}

static void _Callback_HyALMacWrpSnifferIndication(struct TMacWrpSnifferIndication *pParameters)
{
	LOG_DBG(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, "HyALMacWrpSnifferIndication:  MSDU:"));

	/* TODO: Implement Sniffer functionality */

	if (sUpperLayerNotifications.m_HyALSnifferIndication != NULL) {
		sUpperLayerNotifications.m_HyALSnifferIndication((struct THyALSnifferIndication *)pParameters);
	}
}

static void _Callback_HyALMacWrpSnifferIndicationRF(struct TMacWrpSnifferIndication *pParameters)
{
	LOG_DBG(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, "HyALMacWrpSnifferIndicationRF:  MSDU:"));

	/* TODO: Implement Sniffer functionality */

	if (sUpperLayerNotifications.m_HyALSnifferIndication != NULL) {
		sUpperLayerNotifications.m_HyALSnifferIndication((struct THyALSnifferIndication *)pParameters);
	}
}

/* ------------------------------------------------ */
/* ------------------- HyAL API ------------------- */
/* ------------------------------------------------ */

void HyALInitialize(struct THyALNotifications *pNotifications, uint8_t u8Band)
{
	struct TMacWrpNotifications hyALNotifications;
	struct TMacWrpNotifications hyALNotificationsRF;

	LOG_INFO(Log("HyALInitialize: Initializing HyAL..."));

	/* Set upper layer notifications */
	memcpy(&sUpperLayerNotifications, pNotifications, sizeof(struct THyALNotifications));
	
	/* Set default module variables */
	g_HyAL = g_HyALDefaults;

	/* Define callbacks coming from Mac Wrapper (PLC) */
	hyALNotifications.m_MacWrpDataConfirm = _Callback_HyALMacWrpDataConfirm;
	hyALNotifications.m_MacWrpDataIndication = _Callback_HyALMacWrpDataIndication;
	hyALNotifications.m_MacWrpGetConfirm = _Callback_HyALMacWrpGetConfirm;
	hyALNotifications.m_MacWrpSetConfirm = _Callback_HyALMacWrpSetConfirm;
	hyALNotifications.m_MacWrpResetConfirm = _Callback_HyALMacWrpResetConfirm;
	hyALNotifications.m_MacWrpBeaconNotifyIndication = _Callback_HyALMacWrpBeaconNotify;
	hyALNotifications.m_MacWrpScanConfirm = _Callback_HyALMacWrpScanConfirm;
	hyALNotifications.m_MacWrpStartConfirm = _Callback_HyALMacWrpStartConfirm;
	hyALNotifications.m_MacWrpCommStatusIndication = _Callback_HyALMacWrpCommStatusIndication;
	hyALNotifications.m_MacWrpSnifferIndication = _Callback_HyALMacWrpSnifferIndication;

	/* Define callbacks coming from Mac Wrapper (RF) */
	hyALNotificationsRF.m_MacWrpDataConfirm = _Callback_HyALMacWrpDataConfirmRF;
	hyALNotificationsRF.m_MacWrpDataIndication = _Callback_HyALMacWrpDataIndicationRF;
	hyALNotificationsRF.m_MacWrpGetConfirm = _Callback_HyALMacWrpGetConfirmRF;
	hyALNotificationsRF.m_MacWrpSetConfirm = _Callback_HyALMacWrpSetConfirmRF;
	hyALNotificationsRF.m_MacWrpResetConfirm = _Callback_HyALMacWrpResetConfirmRF;
	hyALNotificationsRF.m_MacWrpBeaconNotifyIndication = _Callback_HyALMacWrpBeaconNotifyRF;
	hyALNotificationsRF.m_MacWrpScanConfirm = _Callback_HyALMacWrpScanConfirmRF;
	hyALNotificationsRF.m_MacWrpStartConfirm = _Callback_HyALMacWrpStartConfirmRF;
	hyALNotificationsRF.m_MacWrpCommStatusIndication = _Callback_HyALMacWrpCommStatusIndicationRF;
	hyALNotificationsRF.m_MacWrpSnifferIndication = _Callback_HyALMacWrpSnifferIndicationRF;
	
	/* Initialize MAC Wrapper for both PLC and RF */
	MacWrapperInitialize(&hyALNotifications, u8Band);
	MacWrapperInitializeRF(&hyALNotificationsRF);
}

//#define RX_ADP_TEST_VECTOR
//#define GR2_001_JOIN
//#define GR2_001_CHA
//#define GR5_010_PREQ_FROM_SENDER
//#define GR5_010_PREQ_FROM_RELAY
//#define GR5_010_PREP_FROM_RECEIVER
//#define GR5_010_PREP_FROM_RELAY

#ifdef RX_ADP_TEST_VECTOR
static uint32_t timeout = 3000;
#if defined(GR2_001_JOIN)
static uint8_t rxData[12] = {0x40, 0x02, 0x18, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11};
static uint16_t rxLen = 12;
static uint8_t rxData2[74] = {0x40, 0x02, 0x18, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x08, 0x00, 0x00, 0x3E, 0x2F, 0x40, 0x11, 0x84, 0x8D, 0x16, 0xBC, 0x76, 0x76, 0xF6, 0x35, 0x65, 0x90, 0x12, 0x08, 0x2B, 0x3A, 0x97, 0x1A, 0x58, 0xCD, 0xB3, 0x53, 0xE3, 0xE4, 0x0C, 0x5A, 0x2C, 0xEE, 0xB5, 0x9B, 0x01, 0x0A, 0xF6, 0xF5, 0x94, 0x55, 0xE2, 0xA3, 0x25, 0xCD, 0x70, 0xB1, 0x7E, 0x65, 0x7A, 0x2D, 0x85, 0x13, 0x49, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11};
static uint16_t rxLen2 = 74;
static uint8_t rxData3[60] = {0x40, 0x02, 0x18, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x08, 0x01, 0x00, 0x30, 0x2F, 0xC0, 0x11, 0x84, 0x8D, 0x16, 0xBC, 0x76, 0x76, 0xF6, 0x35, 0x65, 0x90, 0x12, 0x08, 0x2B, 0x3A, 0x97, 0x00, 0x00, 0x00, 0x01, 0x37, 0x5E, 0x33, 0xCC, 0x8A, 0x20, 0x8B, 0x3B, 0xEE, 0x99, 0xE5, 0xDF, 0xE0, 0x21, 0x9C, 0x29, 0x45, 0x84, 0xA0, 0x20, 0x1E, 0xC5};
static uint16_t rxLen3 = 60;
static TMacWrpPanId srcPanId = 0x781D;
//static struct TMacWrpAddress srcAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x002A};
static struct TMacWrpAddress srcAddr = {
	.m_eAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED,
	.m_ExtendedAddress = {{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}}};
static TMacWrpPanId dstPanId = 0x781D;
static struct TMacWrpAddress dstAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x0000};
//static struct TMacWrpAddress dstAddr = {
//	.m_eAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED,
//	.m_ExtendedAddress = {{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}}};
static uint8_t linkQuality = 100;
static uint8_t dsn = 0x55;
static TMacWrpTimestamp timestamp = 0;
static enum EMacWrpSecurityLevel securityLevel = MAC_WRP_SECURITY_LEVEL_NONE;
static uint8_t keyIndex = 0;
static enum EMacWrpQualityOfService qualityOfService = MAC_WRP_QUALITY_OF_SERVICE_NORMAL_PRIORITY;
static uint8_t recvModulation = 0;
static uint8_t recvModulationScheme = 0;
static struct TMacWrpToneMap recvToneMap = {{0x3F, 0x00, 0x00}};
static uint8_t computedModulation = 0;
static uint8_t computedModulationScheme = 0;
static struct TMacWrpToneMap computedToneMap = {{0x3F, 0x00, 0x00}};
#elif defined(GR2_001_CHA)
static uint8_t rxData[42] = {0x40, 0x02, 0xA8, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x04, 0x00, 0x00, 0x1E, 0x2F, 0x00, 0x11, 0x84, 0x8D, 0x16, 0xBC, 0x76, 0x76, 0xF6, 0x35, 0x65, 0x90, 0x12, 0x08, 0x2B, 0x3A, 0x97, 0x81, 0x72, 0x63, 0x54, 0x45, 0x36, 0x27, 0x18};
static uint16_t rxLen = 42;
static uint8_t rxData2[98] = {0x40, 0x02, 0xA8, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x04, 0x01, 0x00, 0x56, 0x2F, 0x80, 0x11, 0x84, 0x8D, 0x16, 0xBC, 0x76, 0x76, 0xF6, 0x35, 0x65, 0x90, 0x12, 0x08, 0x2B, 0x3A, 0x97, 0xD7, 0x49, 0xBE, 0x8E, 0x08, 0x35, 0x12, 0xA1, 0x52, 0x5F, 0xB5, 0x92, 0xF2, 0xF3, 0xDD, 0xCE, 0x00, 0x00, 0x00, 0x00, 0x56, 0x21, 0xFE, 0xB3, 0xA6, 0x0C, 0x10, 0xF8, 0x2E, 0xE6, 0xC8, 0xF3, 0xF8, 0x8B, 0x99, 0x1E, 0x14, 0x31, 0xAE, 0x2D, 0xAF, 0xD9, 0xAC, 0x44, 0x2D, 0x0C, 0x7E, 0x55, 0xB2, 0x9B, 0x89, 0x1B, 0xF1, 0x98, 0x45, 0xC5, 0xA8, 0x88, 0xAB, 0x4F, 0x89, 0x8D, 0x6C, 0x56};
static uint16_t rxLen2 = 98;
static uint8_t rxData3[16] = {0x40, 0x02, 0x98, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x0C, 0x02, 0x00, 0x04};
static uint16_t rxLen3 = 16;
static TMacWrpPanId srcPanId = 0x781D;
static struct TMacWrpAddress srcAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x0000};
//static struct TMacWrpAddress srcAddr = {
//	.m_eAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED,
//	.m_ExtendedAddress = {{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}}};
static TMacWrpPanId dstPanId = 0x781D;
//static struct TMacWrpAddress dstAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x0000};
static struct TMacWrpAddress dstAddr = {
	.m_eAddrMode = MAC_WRP_ADDRESS_MODE_EXTENDED,
	.m_ExtendedAddress = {{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}}};
static uint8_t linkQuality = 100;
static uint8_t dsn = 0x55;
static TMacWrpTimestamp timestamp = 0;
static enum EMacWrpSecurityLevel securityLevel = MAC_WRP_SECURITY_LEVEL_NONE;
static uint8_t keyIndex = 0;
static enum EMacWrpQualityOfService qualityOfService = MAC_WRP_QUALITY_OF_SERVICE_NORMAL_PRIORITY;
static uint8_t recvModulation = 0;
static uint8_t recvModulationScheme = 0;
static struct TMacWrpToneMap recvToneMap = {{0x3F, 0x00, 0x00}};
static uint8_t computedModulation = 0;
static uint8_t computedModulationScheme = 0;
static struct TMacWrpToneMap computedToneMap = {{0x3F, 0x00, 0x00}};
#elif defined(GR5_010_PREQ_FROM_SENDER)
static uint8_t rxData[11] = {0x40, 0x01, 0xFC, 0x01, 0x0C, 0x00, 0x2A, 0xF0, 0x00, 0x00, 0x00};
static uint16_t rxLen = 11;
static TMacWrpPanId srcPanId = 0x781D;
static struct TMacWrpAddress srcAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x002A};
static TMacWrpPanId dstPanId = 0x781D;
static struct TMacWrpAddress dstAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x002B};
static uint8_t linkQuality = 100;
static uint8_t dsn = 0x55;
static TMacWrpTimestamp timestamp = 0;
static enum EMacWrpSecurityLevel securityLevel = MAC_WRP_SECURITY_LEVEL_ENC_MIC_32;
static uint8_t keyIndex = 0;
static enum EMacWrpQualityOfService qualityOfService = MAC_WRP_QUALITY_OF_SERVICE_NORMAL_PRIORITY;
static uint8_t recvModulation = 0;
static uint8_t recvModulationScheme = 0;
static struct TMacWrpToneMap recvToneMap = {{0x3F, 0x00, 0x00}};
static uint8_t computedModulation = 0;
static uint8_t computedModulationScheme = 0;
static struct TMacWrpToneMap computedToneMap = {{0x3F, 0x00, 0x00}};
#elif defined(GR5_010_PREQ_FROM_RELAY)
static uint8_t rxData[15] = {0x40, 0x01, 0xFC, 0x01, 0x0C, 0x00, 0x2A, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x2B, 0x08, 0x02};
static uint16_t rxLen = 15;
static TMacWrpPanId srcPanId = 0x781D;
static struct TMacWrpAddress srcAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x002B};
static TMacWrpPanId dstPanId = 0x781D;
static struct TMacWrpAddress dstAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x010C};
static uint8_t linkQuality = 100;
static uint8_t dsn = 0x55;
static TMacWrpTimestamp timestamp = 0;
static enum EMacWrpSecurityLevel securityLevel = MAC_WRP_SECURITY_LEVEL_ENC_MIC_32;
static uint8_t keyIndex = 0;
static enum EMacWrpQualityOfService qualityOfService = MAC_WRP_QUALITY_OF_SERVICE_NORMAL_PRIORITY;
static uint8_t recvModulation = 0;
static uint8_t recvModulationScheme = 0;
static struct TMacWrpToneMap recvToneMap = {{0x3F, 0x00, 0x00}};
static uint8_t computedModulation = 0;
static uint8_t computedModulationScheme = 0;
static struct TMacWrpToneMap computedToneMap = {{0x3F, 0x00, 0x00}};
#elif defined(GR5_010_PREP_FROM_RECEIVER)
static uint8_t rxData[19] = {0x40, 0x01, 0xFD, 0x00, 0x2A, 0x01, 0x0C, 0xF0, 0x00, 0x01, 0x0C, 0x00, 0x2B, 0x08, 0x02, 0x01, 0x0C, 0x00, 0x0A};
static uint16_t rxLen = 19;
static TMacWrpPanId srcPanId = 0x781D;
static struct TMacWrpAddress srcAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x010C};
static TMacWrpPanId dstPanId = 0x781D;
static struct TMacWrpAddress dstAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x002B};
static uint8_t linkQuality = 100;
static uint8_t dsn = 0x55;
static TMacWrpTimestamp timestamp = 0;
static enum EMacWrpSecurityLevel securityLevel = MAC_WRP_SECURITY_LEVEL_ENC_MIC_32;
static uint8_t keyIndex = 0;
static enum EMacWrpQualityOfService qualityOfService = MAC_WRP_QUALITY_OF_SERVICE_NORMAL_PRIORITY;
static uint8_t recvModulation = 0;
static uint8_t recvModulationScheme = 0;
static struct TMacWrpToneMap recvToneMap = {{0x3F, 0x00, 0x00}};
static uint8_t computedModulation = 0;
static uint8_t computedModulationScheme = 0;
static struct TMacWrpToneMap computedToneMap = {{0x3F, 0x00, 0x00}};
#elif defined(GR5_010_PREP_FROM_RELAY)
static uint8_t rxData[23] = {0x40, 0x01, 0xFD, 0x00, 0x2A, 0x01, 0x0C, 0xF0, 0x00, 0x01, 0x0C, 0x00, 0x2B, 0x08, 0x02, 0x01, 0x0C, 0x00, 0x0A, 0x00, 0x2B, 0x04, 0x0A};
static uint16_t rxLen = 23;
static TMacWrpPanId srcPanId = 0x781D;
static struct TMacWrpAddress srcAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x002B};
static TMacWrpPanId dstPanId = 0x781D;
static struct TMacWrpAddress dstAddr = {MAC_WRP_ADDRESS_MODE_SHORT, 0x002A};
static uint8_t linkQuality = 100;
static uint8_t dsn = 0x55;
static TMacWrpTimestamp timestamp = 0;
static enum EMacWrpSecurityLevel securityLevel = MAC_WRP_SECURITY_LEVEL_ENC_MIC_32;
static uint8_t keyIndex = 0;
static enum EMacWrpQualityOfService qualityOfService = MAC_WRP_QUALITY_OF_SERVICE_NORMAL_PRIORITY;
static uint8_t recvModulation = 0;
static uint8_t recvModulationScheme = 0;
static struct TMacWrpToneMap recvToneMap = {{0x3F, 0x00, 0x00}};
static uint8_t computedModulation = 0;
static uint8_t computedModulationScheme = 0;
static struct TMacWrpToneMap computedToneMap = {{0x3F, 0x00, 0x00}};
#endif
#endif

void HyALEventHandler(void)
{
	/* Call Mac Wrapper Event Handlers for both PLC and RF */
	MacWrapperEventHandler();
	MacWrapperEventHandlerRF();

#ifdef RX_ADP_TEST_VECTOR
#if defined(GR2_001_JOIN) || defined(GR2_001_CHA)
	if (--timeout == 2000) {
		struct TMacWrpDataIndication dataIndRF;
		dataIndRF.m_ComputedToneMap = computedToneMap;
		dataIndRF.m_DstAddr = dstAddr;
		dataIndRF.m_eQualityOfService = qualityOfService;
		dataIndRF.m_eSecurityLevel = securityLevel;
		dataIndRF.m_nDstPanId = dstPanId;
		dataIndRF.m_nSrcPanId = srcPanId;
		dataIndRF.m_nTimestamp = timestamp;
		dataIndRF.m_pMsdu = rxData;
		dataIndRF.m_RecvToneMap = recvToneMap;
		dataIndRF.m_SrcAddr = srcAddr;
		dataIndRF.m_u16MsduLength = rxLen;
		dataIndRF.m_u8ComputedModulation = computedModulation;
		dataIndRF.m_u8ComputedModulationScheme = computedModulationScheme;
		dataIndRF.m_u8Dsn = dsn;
		dataIndRF.m_u8KeyIndex = keyIndex;
		dataIndRF.m_u8MpduLinkQuality = linkQuality;
		dataIndRF.m_u8RecvModulation = recvModulation;
		dataIndRF.m_u8RecvModulationScheme = recvModulationScheme;
		_Callback_HyALMacWrpDataIndicationRF(&dataIndRF);
	}
	if (timeout == 1000) {
		struct TMacWrpDataIndication dataIndRF;
		dataIndRF.m_ComputedToneMap = computedToneMap;
		dataIndRF.m_DstAddr = dstAddr;
		dataIndRF.m_eQualityOfService = qualityOfService;
		dataIndRF.m_eSecurityLevel = securityLevel;
		dataIndRF.m_nDstPanId = dstPanId;
		dataIndRF.m_nSrcPanId = srcPanId;
		dataIndRF.m_nTimestamp = timestamp;
		dataIndRF.m_pMsdu = rxData2;
		dataIndRF.m_RecvToneMap = recvToneMap;
		dataIndRF.m_SrcAddr = srcAddr;
		dataIndRF.m_u16MsduLength = rxLen2;
		dataIndRF.m_u8ComputedModulation = computedModulation;
		dataIndRF.m_u8ComputedModulationScheme = computedModulationScheme;
		dataIndRF.m_u8Dsn = dsn;
		dataIndRF.m_u8KeyIndex = keyIndex;
		dataIndRF.m_u8MpduLinkQuality = linkQuality;
		dataIndRF.m_u8RecvModulation = recvModulation;
		dataIndRF.m_u8RecvModulationScheme = recvModulationScheme;
		_Callback_HyALMacWrpDataIndicationRF(&dataIndRF);
	}
	if (timeout == 0) {
		timeout = 0xFFFFFFFF;
		struct TMacWrpDataIndication dataIndRF;
		dataIndRF.m_ComputedToneMap = computedToneMap;
		dataIndRF.m_DstAddr = dstAddr;
		dataIndRF.m_eQualityOfService = qualityOfService;
		dataIndRF.m_eSecurityLevel = securityLevel;
		dataIndRF.m_nDstPanId = dstPanId;
		dataIndRF.m_nSrcPanId = srcPanId;
		dataIndRF.m_nTimestamp = timestamp;
		dataIndRF.m_pMsdu = rxData3;
		dataIndRF.m_RecvToneMap = recvToneMap;
		dataIndRF.m_SrcAddr = srcAddr;
		dataIndRF.m_u16MsduLength = rxLen3;
		dataIndRF.m_u8ComputedModulation = computedModulation;
		dataIndRF.m_u8ComputedModulationScheme = computedModulationScheme;
		dataIndRF.m_u8Dsn = dsn;
		dataIndRF.m_u8KeyIndex = keyIndex;
		dataIndRF.m_u8MpduLinkQuality = linkQuality;
		dataIndRF.m_u8RecvModulation = recvModulation;
		dataIndRF.m_u8RecvModulationScheme = recvModulationScheme;
		_Callback_HyALMacWrpDataIndicationRF(&dataIndRF);
	}
#elif defined(GR5_010_PREQ_FROM_SENDER) || defined(GR5_010_PREP_FROM_RELAY)
	if (--timeout == 0) {
		timeout = 0xFFFFFFFF;
		struct TMacWrpDataIndication dataIndRF;
		dataIndRF.m_ComputedToneMap = computedToneMap;
		dataIndRF.m_DstAddr = dstAddr;
		dataIndRF.m_eQualityOfService = qualityOfService;
		dataIndRF.m_eSecurityLevel = securityLevel;
		dataIndRF.m_nDstPanId = dstPanId;
		dataIndRF.m_nSrcPanId = srcPanId;
		dataIndRF.m_nTimestamp = timestamp;
		dataIndRF.m_pMsdu = rxData;
		dataIndRF.m_RecvToneMap = recvToneMap;
		dataIndRF.m_SrcAddr = srcAddr;
		dataIndRF.m_u16MsduLength = rxLen;
		dataIndRF.m_u8ComputedModulation = computedModulation;
		dataIndRF.m_u8ComputedModulationScheme = computedModulationScheme;
		dataIndRF.m_u8Dsn = dsn;
		dataIndRF.m_u8KeyIndex = keyIndex;
		dataIndRF.m_u8MpduLinkQuality = linkQuality;
		dataIndRF.m_u8RecvModulation = recvModulation;
		dataIndRF.m_u8RecvModulationScheme = recvModulationScheme;
		_Callback_HyALMacWrpDataIndicationRF(&dataIndRF);
	}
#elif defined(GR5_010_PREQ_FROM_RELAY) || defined(GR5_010_PREP_FROM_RECEIVER)
	if (--timeout == 0) {
		timeout = 0xFFFFFFFF;
		struct TMacWrpDataIndication dataInd;
		dataInd.m_ComputedToneMap = computedToneMap;
		dataInd.m_DstAddr = dstAddr;
		dataInd.m_eQualityOfService = qualityOfService;
		dataInd.m_eSecurityLevel = securityLevel;
		dataInd.m_nDstPanId = dstPanId;
		dataInd.m_nSrcPanId = srcPanId;
		dataInd.m_nTimestamp = timestamp;
		dataInd.m_pMsdu = rxData;
		dataInd.m_RecvToneMap = recvToneMap;
		dataInd.m_SrcAddr = srcAddr;
		dataInd.m_u16MsduLength = rxLen;
		dataInd.m_u8ComputedModulation = computedModulation;
		dataInd.m_u8ComputedModulationScheme = computedModulationScheme;
		dataInd.m_u8Dsn = dsn;
		dataInd.m_u8KeyIndex = keyIndex;
		dataInd.m_u8MpduLinkQuality = linkQuality;
		dataInd.m_u8RecvModulation = recvModulation;
		dataInd.m_u8RecvModulationScheme = recvModulationScheme;
		_Callback_HyALMacWrpDataIndication(&dataInd);
	}
#endif
#endif
}

void HyALDataRequest(struct THyALDataRequest *pParameters)
{
	struct THyALDataReq *pDataReq;

	LOG_INFO(LogBuffer(pParameters->m_pMsdu, pParameters->m_u16MsduLength, "HyALDataRequest (Handle: 0x%02X Media Type: %02X): ", pParameters->m_u8MsduHandle, pParameters->m_eMediaType));

	/* Look for free Data Request Entry */
	pDataReq = _getFreeDataReqEntry();
	
	if (pDataReq == NULL) {
		/* Too many data requests */
		struct THyALDataConfirm confParameters;
		/* Send confirm to upper layer */
		if (sUpperLayerNotifications.m_HyALDataConfirm != NULL) {
			confParameters.m_u8MsduHandle = pParameters->m_u8MsduHandle;
			confParameters.m_eStatus = MAC_WRP_STATUS_QUEUE_FULL;
			confParameters.m_nTimestamp = 0;
			confParameters.m_eMediaType = (enum EHyALMediaTypeConfirm)pParameters->m_eMediaType;
			sUpperLayerNotifications.m_HyALDataConfirm(&confParameters);
		}
	}
	else {
		/* Accept request */
		/* Copy parameters from HyAL struct to MacWrp struct */
		/* Media Type is the last parameter and will not be copied */
		memcpy(&pDataReq->m_sDataReqParameters, pParameters, sizeof(struct TMacWrpDataRequest));
		/* Store Media Type on module variable */
		pDataReq->m_eDataReqMediaType = pParameters->m_eMediaType;
		/* Copy data to backup buffer, just in case backup media has to be used, current pointer will not be valid later */
		if (pParameters->m_u16MsduLength <= HYAL_BACKUP_BUF_SIZE) {
			memcpy(pDataReq->m_au8BackupBuffer, pParameters->m_pMsdu, pParameters->m_u16MsduLength);
		}

		/* Different handling for Broadcast and Unicast requests */
		if ((pParameters->m_DstAddr.m_eAddrMode == MAC_WRP_ADDRESS_MODE_SHORT) &&
			(pParameters->m_DstAddr.m_nShortAddress == MAC_WRP_SHORT_ADDRESS_BROADCAST)) {
			/* Broadcast */
			/* Overwrite MediaType to both */
			pDataReq->m_eDataReqMediaType = HYAL_MEDIA_TYPE_REQ_BOTH;
			/* Set control variables */
			pDataReq->bWaitingSecondConfirm = false;
			/* Request on both Media */
			MacWrapperMcpsDataRequest(&pDataReq->m_sDataReqParameters);
			MacWrapperMcpsDataRequestRF(&pDataReq->m_sDataReqParameters);
		}
		else {
			/* Unicast */
			switch (pDataReq->m_eDataReqMediaType) {
				case HYAL_MEDIA_TYPE_REQ_PLC_BACKUP_RF:
					MacWrapperMcpsDataRequest(&pDataReq->m_sDataReqParameters);
					break;
				case HYAL_MEDIA_TYPE_REQ_RF_BACKUP_PLC:
					MacWrapperMcpsDataRequestRF(&pDataReq->m_sDataReqParameters);
					break;
				case HYAL_MEDIA_TYPE_REQ_BOTH:
					MacWrapperMcpsDataRequest(&pDataReq->m_sDataReqParameters);
					MacWrapperMcpsDataRequestRF(&pDataReq->m_sDataReqParameters);
					break;
				case HYAL_MEDIA_TYPE_REQ_PLC_NO_BACKUP:
					MacWrapperMcpsDataRequest(&pDataReq->m_sDataReqParameters);
					break;
				case HYAL_MEDIA_TYPE_REQ_RF_NO_BACKUP:
					MacWrapperMcpsDataRequestRF(&pDataReq->m_sDataReqParameters);
					break;
				default: /* PLC only */
					pDataReq->m_eDataReqMediaType = HYAL_MEDIA_TYPE_REQ_PLC_NO_BACKUP;
					MacWrapperMcpsDataRequest(&pDataReq->m_sDataReqParameters);
					break;
			}
		}
	}
}

void HyALScanRequest(struct THyALScanRequest *pParameters)
{
	LOG_INFO(Log("HyALScanRequest: Duration: %u", pParameters->m_u16ScanDuration));

	/* Set control variable */
	g_HyAL.bWaitingSecondScanConfirm = false;

	/* Call Scan request on both MACs */
	MacWrapperMlmeScanRequest((struct TMacWrpScanRequest *)pParameters);
	MacWrapperMlmeScanRequestRF((struct TMacWrpScanRequest *)pParameters);
}

void HyALResetRequest(struct THyALResetRequest *pParameters)
{
	LOG_DBG(Log("HyALResetRequest: Set default PIB: %u", pParameters->m_bSetDefaultPib));

	/* Set control variable */
	g_HyAL.bWaitingSecondResetConfirm = false;

	/* Call Reset on both MACs */
	MacWrapperMlmeResetRequest((struct TMacWrpResetRequest *)pParameters);
	MacWrapperMlmeResetRequestRF((struct TMacWrpResetRequest *)pParameters);
}

void HyALStartRequest(struct THyALStartRequest *pParameters)
{
	LOG_DBG(Log("HyALStartRequest: Pan ID: %u", pParameters->m_nPanId));

	/* Set control variable */
	g_HyAL.bWaitingSecondStartConfirm = false;

	/* Call Start on both MACs */
	MacWrapperMlmeStartRequest((struct TMacWrpStartRequest *)pParameters);
	MacWrapperMlmeStartRequestRF((struct TMacWrpStartRequest *)pParameters);
}

static bool _IsAttributeInPLCRange(enum EMacWrpPibAttribute eAttribute)
{
	/* Check attribute ID range to distinguish between PLC and RF MAC */
	if (eAttribute < 0x00000200) {
		/* Standard PLC MAC IB */
		return true;
	}
	else if (eAttribute < 0x00000400) {
		/* Standard RF MAC IB */
		return false;
	}
	else if (eAttribute < 0x08000200) {
		/* Manufacturer PLC MAC IB */
		return true;
	}
	else {
		/* Manufacturer RF MAC IB */
		return false;
	}
}

void HyALGetRequest(struct THyALGetRequest *pParameters)
{
	LOG_DBG(Log("HyALGetRequest: Attribute: %08X; Index: %u", pParameters->m_ePibAttribute, pParameters->m_u16PibAttributeIndex));

	/* Check attribute ID range to redirect to PLC or RF MAC */
	if (_IsAttributeInPLCRange(pParameters->m_ePibAttribute)) {
		/* Get from PLC MAC */
		MacWrapperMlmeGetRequest((struct TMacWrpGetRequest *)pParameters);
	}
	else {
		/* Get from RF MAC */
		MacWrapperMlmeGetRequestRF((struct TMacWrpGetRequest *)pParameters);
	}
}

enum EMacWrpStatus HyALGetRequestSync(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, struct TMacWrpPibValue *pValue)
{
	LOG_DBG(Log("HyALGetRequestSync: Attribute: %08X; Index: %u", eAttribute, u16Index));

	/* Check attribute ID range to redirect to PLC or RF MAC */
	if (_IsAttributeInPLCRange(eAttribute)) {
		/* Get from PLC MAC */
		return MacWrapperMlmeGetRequestSync(eAttribute, u16Index, pValue);
	}
	else {
		/* Get from RF MAC */
		return MacWrapperMlmeGetRequestSyncRF(eAttribute, u16Index, pValue);
	}
}

void HyALSetRequest(struct THyALSetRequest *pParameters)
{
	LOG_DBG(Log("HyALSetRequest: Attribute: %08X; Index: %u", pParameters->m_ePibAttribute, pParameters->m_u16PibAttributeIndex));

	/* Check attribute ID range to redirect to PLC or RF MAC */
	if (_IsAttributeInPLCRange(pParameters->m_ePibAttribute)) {
		/* Set to PLC MAC */
		MacWrapperMlmeSetRequest((struct TMacWrpSetRequest *)pParameters);
	}
	else {
		/* Set to RF MAC */
		MacWrapperMlmeSetRequestRF((struct TMacWrpSetRequest *)pParameters);
	}
}

enum EMacWrpStatus HyALSetRequestSync(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, const struct TMacWrpPibValue *pValue)
{
	LOG_DBG(Log("HyALSetRequestSync: Attribute: %08X; Index: %u", eAttribute, u16Index));

	/* Check attribute ID range to redirect to PLC or RF MAC */
	if (_IsAttributeInPLCRange(eAttribute)) {
		/* Set to PLC MAC */
		return MacWrapperMlmeSetRequestSync(eAttribute, u16Index, pValue);
	}
	else {
		/* Set to RF MAC */
		return MacWrapperMlmeSetRequestSyncRF(eAttribute, u16Index, pValue);
	}
}

uint16_t HyALGetNeighbourTableSize(void)
{
	LOG_DBG(Log("HyALGetNeighbourTableSize"));

	return MacWrapperGetNeighbourTableSize();
}

