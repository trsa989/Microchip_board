#include <AdpApi.h>
#include <AdpApiTypes.h>
#include <BootstrapWrapper.h>
#include <ProcessLbp.h>
#include <ProtoLbp.h>

#include <stdbool.h>
#include <stdint.h>
#include <Byte.h>
#include <string.h>

#include <Random.h>
#include <Timer.h>

#define LOG_LEVEL LOG_LEVEL_ADP
#include <Logger.h>

#define UNUSED(v)          (void)(v)

struct TLBPContext g_LbpContext;

/* States of Bootstrap process for EndDevice */
#define STATE_BOOT_NOT_JOINED 0x00
#define STATE_BOOT_SENT_FIRST_JOINING 0x01
#define STATE_BOOT_WAIT_EAPPSK_FIRST_MESSAGE 0x02
#define STATE_BOOT_SENT_EAPPSK_SECOND_JOINING 0x04
#define STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE 0x08
#define STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING 0x10
#define STATE_BOOT_WAIT_ACCEPTED 0x20
#define STATE_BOOT_JOINED 0x40

/* EXT field types */
#define EAP_EXT_TYPE_CONFIGURATION_PARAMETERS 0x02

/**********************************************************************************************************************/

/** Parameters transferred
 *
 ***********************************************************************************************************************
 * Note: the parameters are already encoded on 1 byte (M field and last bit included
 **********************************************************************************************************************/

#define CONF_PARAM_SHORT_ADDR 0x1D
#define CONF_PARAM_GMK 0x27
#define CONF_PARAM_GMK_ACTIVATION 0x2B
#define CONF_PARAM_GMK_REMOVAL 0x2F
#define CONF_PARAM_RESULT 0x31

/* Parameter result values */
#define RESULT_PARAMETER_SUCCESS 0x00
#define RESULT_MISSING_REQUIRED_PARAMETER 0x01
#define RESULT_INVALID_PARAMETER_VALUE 0x02
#define RESULT_UNKNOWN_PARAMETER_ID 0x03

/* Mask to track the presence of mandatory parameters */
#define CONF_PARAM_SHORT_ADDR_MASK 0x01
#define CONF_PARAM_GMK_MASK 0x02
#define CONF_PARAM_GMK_ACTIVATION_MASK 0x04
#define CONF_PARAM_GMK_REMOVAL_MASK 0x08
#define CONF_PARAM_RESULT_MASK 0x10

/**********************************************************************************************************************/

/** Internal functions
 **********************************************************************************************************************/
static void _Dummy_Callback_DataSend(uint8_t u8Status);

static void _SetBootState(uint8_t u8State);
static bool _IsBootState(uint8_t u8StateMask);
static bool _Joined(void);

static uint16_t _GetCoordShortAddress(void);
static uint8_t _GetActiveKeyIndex(void);
static void _SetActiveKeyIndex(uint8_t index);
/* static uint8_t _GetMaxHops(void); */
static void _GetRandP(struct TEapPskRand *p_RandP);
static void _GetIdP(struct TEapPskNetworkAccessIdentifier *p_IdP);
static void _UpdateNonVolatileData(void);

static bool _ProcessParameters(uint16_t u16DataLength, uint8_t *pData, uint8_t *pu8ReceivedParametersMask,
		uint8_t *pu8ParameterResultLength, uint8_t *pParameterResult);
static void _Rekey_TimerExpired_Callback(struct TTimer *pTimer);
static void _Join_TimerExpired_Callback(struct TTimer *pTimer);
static void _Join_Callback_DataSend(uint8_t u8Status);
static void _Join_Confirm(uint8_t u8Status);
static void _Join_Process_Challenge(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData);
static void _Join_Process_Accepted(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData);
static void _Join_Process_Challenge_FirstMessage(const struct TAdpExtendedAddress *pEUI64Address,
		uint8_t u8EAPIdentifier, uint16_t u16EAPDataLength, uint8_t *pEAPData);
static void _Join_Process_Challenge_ThirdMessage(uint16_t u16HeaderLength, uint8_t *pHeader,
		const struct TAdpExtendedAddress *pEUI64Address, uint8_t u8EAPIdentifier, uint16_t u16EAPDataLength,
		uint8_t *pEAPData);
static void _Join_Process_Accepted_EAP(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData);
static void _Join_Process_Accepted_Configuration(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData);

/**********************************************************************************************************************/

/** Dummy callback which does nothing but avoid buffer reuse before it is set free
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static void _Dummy_Callback_DataSend(uint8_t u8Status)
{
	/* Do nothing */
	UNUSED(u8Status);
}

/**********************************************************************************************************************/

/**
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static void _SetBootState(uint8_t u8State)
{
	g_LbpContext.m_u8BootstrapState = u8State;

	LOG_DBG(Log("_SetBootState() Set state to %s Pending Confirms: %d",
			u8State == STATE_BOOT_NOT_JOINED ? "STATE_BOOT_NOT_JOINED" :
			u8State == STATE_BOOT_SENT_FIRST_JOINING ? "STATE_BOOT_SENT_FIRST_JOINING" :
			u8State == STATE_BOOT_WAIT_EAPPSK_FIRST_MESSAGE ? "STATE_BOOT_WAIT_EAPPSK_FIRST_MESSAGE" :
			u8State == STATE_BOOT_SENT_EAPPSK_SECOND_JOINING ? "STATE_BOOT_SENT_EAPPSK_SECOND_JOINING" :
			u8State == STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE ? "STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE" :
			u8State == STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING ? "STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING" :
			u8State == STATE_BOOT_WAIT_ACCEPTED ? "STATE_BOOT_WAIT_ACCEPTED" :
			u8State == STATE_BOOT_JOINED ? "STATE_BOOT_JOINED" : "?????",
			g_LbpContext.m_u8PendingConfirms
			));

	if (u8State == STATE_BOOT_NOT_JOINED) {
		g_LbpContext.m_u8PendingConfirms = 0;
	}
}

/**********************************************************************************************************************/

/**
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static bool _IsBootState(uint8_t u8StateMask)
{
	return ((g_LbpContext.m_u8BootstrapState & u8StateMask) != 0) ||
	       (g_LbpContext.m_u8BootstrapState == u8StateMask); /* special case for NotJoined (== 0) */
}

/**********************************************************************************************************************/

/**
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static bool _Joined(void)
{
	return (g_LbpContext.m_u16ShortAddress != 0xFFFF);
}

/**********************************************************************************************************************/

/**
 ***********************************************************************************************************************
 * Local functions to get/set ADP PIBs
 **********************************************************************************************************************/
static uint16_t _GetCoordShortAddress(void)
{
	struct TAdpGetConfirm getConfirm;
	uint16_t u16CoordShortAddress = 0;
	AdpGetRequestSync(ADP_IB_COORD_SHORT_ADDRESS, 0, &getConfirm);
	memcpy(&u16CoordShortAddress, &getConfirm.m_au8AttributeValue, getConfirm.m_u8AttributeLength);
	return u16CoordShortAddress;
}

static uint8_t _GetActiveKeyIndex(void)
{
	struct TAdpGetConfirm getConfirm;
	AdpGetRequestSync(ADP_IB_ACTIVE_KEY_INDEX, 0, &getConfirm);
	return getConfirm.m_au8AttributeValue[0];
}

static void _SetActiveKeyIndex(uint8_t index)
{
	struct TAdpSetConfirm setConfirm;
	AdpSetRequestSync(ADP_IB_ACTIVE_KEY_INDEX, 0, sizeof(index), (uint8_t *)&index, &setConfirm);
}

static uint16_t _GetMaxJoinWaitTime(void)
{
	struct TAdpGetConfirm getConfirm;
	uint16_t u16MaxJoinWaitTime = 0;
	AdpGetRequestSync(ADP_IB_MAX_JOIN_WAIT_TIME, 0, &getConfirm);
	memcpy(&u16MaxJoinWaitTime, &getConfirm.m_au8AttributeValue, getConfirm.m_u8AttributeLength);
	return u16MaxJoinWaitTime;
}

static void _GetRandP(struct TEapPskRand *p_RandP)
{
	struct TAdpGetConfirm getConfirm;
	AdpGetRequestSync(ADP_IB_MANUF_RANDP, 0, &getConfirm);
	memcpy(p_RandP->m_au8Value, getConfirm.m_au8AttributeValue, getConfirm.m_u8AttributeLength);
}

static void _GetIdP(struct TEapPskNetworkAccessIdentifier *p_IdP)
{
	struct TAdpGetConfirm getConfirm;
	AdpGetRequestSync(ADP_IB_MANUF_EAP_NETWORK_ACCESS_IDENTIFIER, 0, &getConfirm);
	p_IdP->m_u8Length = getConfirm.m_u8AttributeLength;
	memcpy(p_IdP->m_au8Value, getConfirm.m_au8AttributeValue, getConfirm.m_u8AttributeLength);
}

static void _UpdateNonVolatileData(void)
{
	struct TAdpSetConfirm adpSetConfirm;
	bool value = true;
	AdpSetRequestSync(ADP_IB_MANUF_UPDATE_NON_VOLATILE_DATA, 0, sizeof(value), (uint8_t *)&value, &adpSetConfirm);
}

extern bool AdpMac_SetRcCoordSync(uint16_t u16RcCoord);
extern bool AdpMac_SecurityResetSync(void);
extern bool AdpMac_DeleteGroupMasterKeySync(uint8_t u8KeyId);
extern bool AdpMac_SetGroupMasterKeySync(const struct TGroupMasterKey *pMasterKey);
extern bool AdpMac_SetShortAddressSync(uint16_t u16ShortAddress);

/*  */
/* / ********************************************************************************************************************** / */
/* / ** Local functions to get/set MAC PIBs */
/* ********************************************************************************************************************** / */
/* bool LBP_SetRcCoordSync(uint16_t u16RcCoord) */
/* { */
/*  struct TMacWrpPibValue pibValue; */
/*  */
/*  pibValue.m_u8Length = 2; */
/*  pibValue.m_au8Value[0] = Byte_LoByte(u16RcCoord); */
/*  pibValue.m_au8Value[1] = Byte_HiByte(u16RcCoord); */
/*  */
/*  return (MAC_WRP_STATUS_SUCCESS == MacWrapperMlmeSetRequestSync(MAC_WRP_PIB_RC_COORD, 0, &pibValue)); */
/* } */

/**********************************************************************************************************************/

/**
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static bool _ProcessParameters(uint16_t u16DataLength, uint8_t *pData,
		uint8_t *pu8ReceivedParametersMask, uint8_t *pu8ParameterResultLength, uint8_t *pParameterResult)
{
	uint16_t u16Offset = 0;
	uint8_t u8AttrId = 0;
	uint8_t u8AttrLen = 0;
	uint8_t *pAttrValue = 0L;
	uint8_t u8Result = RESULT_PARAMETER_SUCCESS;
	uint8_t u8ParameterMask = 0;

	uint16_t u16ShortAddress = 0;
	struct TGroupMasterKey gmk;
	uint8_t u8ActiveKeyId = 0;
	uint8_t u8DeleteKeyId = 0;

	/* bootstrapping data carries the configuration parameters: short address, gmk, gmk activation */
	/* decode and process configuration parameters */
	while ((u16Offset < u16DataLength) && (u8Result == RESULT_PARAMETER_SUCCESS)) {
		u8AttrId = pData[u16Offset++];
		u8AttrLen = pData[u16Offset++];
		pAttrValue = &pData[u16Offset];

		/* LOG_DBG(LogBuffer(pAttrValue, u8AttrLen, "_ProcessParameters() Parsing parameter 0x%02X (len=%u): ", u8AttrId, u8AttrLen)); */

		if (u16Offset + u8AttrLen <= u16DataLength) {
			/* interpret attribute */
			if (u8AttrId == CONF_PARAM_SHORT_ADDR) { /* Short_Addr */
				if (u8AttrLen == 2) {
					u16ShortAddress = (pAttrValue[0] << 8) | pAttrValue[1];
					u8ParameterMask |= CONF_PARAM_SHORT_ADDR_MASK;
				} else {
					u8Result = RESULT_INVALID_PARAMETER_VALUE;
				}
			} else if (u8AttrId == CONF_PARAM_GMK) { /* Provide a GMK key. On reception, the key is installed in the provided Key Identifier slot. */
				/* Constituted of the following fields: */
				/* id (1 byte): the Key Identifier of the GMK */
				/* gmk (16 bytes): the value of the current GMK */
				if (u8AttrLen == 17) {
					gmk.m_u8KeyId = pAttrValue[0];
					memcpy(gmk.m_au8Key, &pAttrValue[1], 16);
					u8ParameterMask |= CONF_PARAM_GMK_MASK;
				} else {
					u8Result = RESULT_INVALID_PARAMETER_VALUE;
				}
			} else if (u8AttrId == CONF_PARAM_GMK_ACTIVATION) { /* Indicate the GMK to use for outgoing messages */
				/* Constituted of the following field: */
				/* id (1 byte): the Key Identifier of the active GMK */
				if (u8AttrLen == 1) {
					u8ActiveKeyId = pAttrValue[0];
					u8ParameterMask |= CONF_PARAM_GMK_ACTIVATION_MASK;
				} else {
					u8Result = RESULT_INVALID_PARAMETER_VALUE;
				}
			} else if (u8AttrId == CONF_PARAM_GMK_REMOVAL) { /* Indicate a GMK to delete. */
				/* Constituted of the following field: */
				/* id (1 byte): the Key Identifier of the GMK to delete */
				if (u8AttrLen == 1) {
					u8DeleteKeyId = pAttrValue[0];
					u8ParameterMask |= CONF_PARAM_GMK_REMOVAL_MASK;
				} else {
					u8Result = RESULT_INVALID_PARAMETER_VALUE;
				}
			} else {
				LOG_ERR(Log("_ProcessParameters() Unsupported attribute id %u, len %u", u8AttrId, u8AttrLen));
				u8Result = RESULT_UNKNOWN_PARAMETER_ID;
			}
		} else {
			u8Result = RESULT_INVALID_PARAMETER_VALUE;
		}

		u16Offset += u8AttrLen;
	}

	if (u8Result == RESULT_PARAMETER_SUCCESS) {
		/* verify the validity of parameters */
		bool bParamsValid = true;
		u8AttrId = 0;
		if (!_Joined()) {
			/* if device not joined yet, the following parameters are mandatory: CONF_PARAM_SHORT_ADDR_MASK, CONF_PARAM_GMK_MASK, CONF_PARAM_GMK_ACTIVATION_MASK */
			if ((u8ParameterMask & CONF_PARAM_SHORT_ADDR_MASK) != CONF_PARAM_SHORT_ADDR_MASK) {
				u8AttrId = CONF_PARAM_SHORT_ADDR;
			} else if ((u8ParameterMask & CONF_PARAM_GMK_MASK) != CONF_PARAM_GMK_MASK) {
				u8AttrId = CONF_PARAM_GMK;
			} else if ((u8ParameterMask & CONF_PARAM_GMK_ACTIVATION_MASK) != CONF_PARAM_GMK_ACTIVATION_MASK) {
				u8AttrId = CONF_PARAM_GMK_ACTIVATION;
			}
		} else {
			/* if device already joined, the message should contain one of the following parameters: CONF_PARAM_GMK_MASK, CONF_PARAM_GMK_ACTIVATION_MASK, CONF_PARAM_GMK_REMOVAL_MASK */
			if ((u8ParameterMask & (CONF_PARAM_GMK_MASK | CONF_PARAM_GMK_ACTIVATION_MASK | CONF_PARAM_GMK_REMOVAL_MASK)) != 0) {
				/* one of required parameters has been found; nothing to do */
			} else {
				/* no required parameters was received; just send back an error related to GMK-ACTIVATION (as missing parameters) */
				u8AttrId = CONF_PARAM_GMK_ACTIVATION;
			}
		}

		bParamsValid = (u8AttrId == 0);

		if (!bParamsValid) {
			u8Result = RESULT_MISSING_REQUIRED_PARAMETER;
		} else {
			*pu8ReceivedParametersMask = u8ParameterMask;
			if ((u8ParameterMask & CONF_PARAM_SHORT_ADDR_MASK) == CONF_PARAM_SHORT_ADDR_MASK) {
				/* short address will be set only after receiving the EAP-Success message */
				g_LbpContext.m_u16JoiningShortAddress = u16ShortAddress;
				LOG_DBG(
						Log("_ProcessParameters() ShortAddress %04X", g_LbpContext.m_u16JoiningShortAddress));
			}

			if ((u8ParameterMask & CONF_PARAM_GMK_MASK) == CONF_PARAM_GMK_MASK) {
				g_LbpContext.m_GroupMasterKey = gmk;
				LOG_DBG(
						LogBuffer(g_LbpContext.m_GroupMasterKey.m_au8Key, 16, "_ProcessParameters() New GMK (id=%u) ", g_LbpContext.m_GroupMasterKey.m_u8KeyId));
			}

			if ((u8ParameterMask & CONF_PARAM_GMK_ACTIVATION_MASK) == CONF_PARAM_GMK_ACTIVATION_MASK) {
				if (_GetActiveKeyIndex() != u8ActiveKeyId) {
					/*  On reception of the GMK-activation parameter, the peer shall empty its DeviceTable (in order to allow re-use of previously allocated short addresses). */
					LOG_DBG(Log("_ProcessParameters() Key-id changed; Reset device table"));
					AdpMac_SecurityResetSync();
				}

				_SetActiveKeyIndex(u8ActiveKeyId);
				/* Set flag to update non volatile data */
				_UpdateNonVolatileData();
				LOG_DBG(
						Log("_ProcessParameters() Active GMK id: %u", u8ActiveKeyId));
			}

			if ((u8ParameterMask & CONF_PARAM_GMK_REMOVAL_MASK) == CONF_PARAM_GMK_REMOVAL_MASK) {
				struct TAdpGetConfirm getConfirm;
				uint8_t u8ActiveKeyIdPrev;
				AdpGetRequestSync(ADP_IB_ACTIVE_KEY_INDEX, 0, &getConfirm);
				u8ActiveKeyIdPrev = getConfirm.m_au8AttributeValue[0];
				/* deleting the active key is forbidden */
				if (u8ActiveKeyIdPrev != u8DeleteKeyId) {
					if (AdpMac_DeleteGroupMasterKeySync(u8DeleteKeyId)) {
						LOG_ERR(Log("_ProcessParameters() GMK id %u was deleted!", u8DeleteKeyId));
					} else {
						LOG_ERR(Log("_ProcessParameters() Cannot delete GMK id: %u!", u8DeleteKeyId));
					}
				} else {
					LOG_ERR(Log("_ProcessParameters() Cannot delete active GMK id: %u!", u8DeleteKeyId));
				}
			}
		}
	}

	/* prepare p-result */
	/* If one or more parameter were invalid or missing, the peer send a message 4 with R = DONE_FAILURE and */
	/* an embedded configuration field with at least one Parameter-result indicating the error. */
	pParameterResult[(*pu8ParameterResultLength)++] = CONF_PARAM_RESULT;
	pParameterResult[(*pu8ParameterResultLength)++] = 2;
	pParameterResult[(*pu8ParameterResultLength)++] = u8Result;
	pParameterResult[(*pu8ParameterResultLength)++] = (u8Result == RESULT_PARAMETER_SUCCESS) ? 0 : u8AttrId;

	return (u8Result == RESULT_PARAMETER_SUCCESS);
}

/**********************************************************************************************************************/

/** m_u16AdpMaxJoinWaitTime expired
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static void _Rekey_TimerExpired_Callback(struct TTimer *pTimer)
{
	LOG_DBG(Log("_Rekey_TimerExpired_Callback"));
	UNUSED(pTimer);
	LBP_ClearEapContext();
	if (_Joined()) {
		/* set back automate state */
		LBP_ForceJoinStatus(true);
	} else {
		/* never should be here but check anyway */
		LBP_ForceJoinStatus(false);
	}
}

/**********************************************************************************************************************/

/**
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static void _Join_TimerExpired_Callback(struct TTimer *pTimer)
{
	LOG_DBG(Log("NetworkJoin_TimerExpired_Callback"));
	UNUSED(pTimer);
	_Join_Confirm(G3_TIMEOUT);
}

/**********************************************************************************************************************/

/** MCPS_DATA_Confirm callback used for network joining
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
static void _Join_Callback_DataSend(uint8_t u8Status)
{
	g_LbpContext.m_u8PendingConfirms--;
	if (u8Status == G3_SUCCESS) {
		if (g_LbpContext.m_u8PendingConfirms == 0) {
			/* / message successfully sent: Update state and wait for response or timeout */
			if (_IsBootState(STATE_BOOT_SENT_FIRST_JOINING)) {
				_SetBootState(STATE_BOOT_WAIT_EAPPSK_FIRST_MESSAGE);
			} else if (_IsBootState(STATE_BOOT_SENT_EAPPSK_SECOND_JOINING)) {
				_SetBootState(STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE);
			} else if (_IsBootState(STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING)) {
				_SetBootState(STATE_BOOT_WAIT_ACCEPTED);
			}
		}
	} else {
		_Join_Confirm(u8Status);
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void _Join_Confirm(uint8_t u8Status)
{
	LOG_INFO(Log("_Join_Confirm() Status: %u", u8Status));
	Timer_Unregister(&g_LbpContext.m_JoinTimer);
	LBP_ClearEapContext();

	if (u8Status == G3_SUCCESS) {
		_SetBootState(STATE_BOOT_JOINED);
	} else {
		_SetBootState(STATE_BOOT_NOT_JOINED);
	}

	if (g_LbpContext.m_LbpNotifications.fnctJoinConfirm) {
		g_LbpContext.m_LbpNotifications.fnctJoinConfirm(u8Status, g_LbpContext.m_u16ShortAddress, g_LbpContext.m_u16LbaAddress);
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
static void _Kick_Notify(void)
{
	LOG_INFO(Log("_Kick_Notify()"));
	if (g_LbpContext.m_LbpNotifications.fnctKickNotify) {
		g_LbpContext.m_LbpNotifications.fnctKickNotify();
	}
}

/**********************************************************************************************************************/

/** Network joining
 *
 ***********************************************************************************************************************
 *
 * This function is called by the 6LoWPANProtocol when a LBP Challenge message is received
 *
 **********************************************************************************************************************/
static void _Join_Process_Challenge(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData)
{
	uint8_t u8Code = 0;
	uint8_t u8Identifier = 0;
	uint8_t u8TSubfield = 0;
	uint16_t u16EAPDataLength = 0;
	uint8_t *pEAPData = 0L;

	if (EAP_PSK_Decode_Message(u16BootStrappingDataLength, pBootStrappingData, &u8Code, &u8Identifier, &u8TSubfield,
			&u16EAPDataLength, &pEAPData)) {
		/* the Challenge is always a Request coming from the LBS */
		if (u8Code == EAP_REQUEST) {
			/* Also only 2 kind of EAP messages are accepted as request: first and third message */
			if (u8TSubfield == EAP_PSK_T0) {
				/* this message can be received in the following states: */
				/* - STATE_BOOT_WAIT_EAPPSK_FIRST_MESSAGE: */
				/*    as normal bootstrap procedure */
				/* - STATE_BOOT_SENT_FIRST_JOINING or STATE_BOOT_SENT_EAPPSK_SECOND_JOINING or STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING */
				/*    as a repetition related to a non response of the previous processing (maybe the response was lost) */
				/*    If a peer receives a valid duplicate Request for which it has already sent a Response, it MUST resend its */
				/*    original Response without reprocessing the Request. */
				/* - STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE: */
				/*    as a repetition related to a non response of the previous processing (maybe the response was lost) */
				/*    If a peer receives a valid duplicate Request for which it has already sent a Response, it MUST resend its */
				/*    original Response without reprocessing the Request. */
				/* - STATE_BOOT_JOINED: during rekey procedure */
				if (_IsBootState(STATE_BOOT_WAIT_EAPPSK_FIRST_MESSAGE | STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE | STATE_BOOT_JOINED | STATE_BOOT_SENT_FIRST_JOINING | STATE_BOOT_SENT_EAPPSK_SECOND_JOINING |
						STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING)) {
					/* enforce the current state in order to correctly update state machine after sending this message */
					_SetBootState(STATE_BOOT_WAIT_EAPPSK_FIRST_MESSAGE);
					/* the function is able to detect a valid repetition */
					_Join_Process_Challenge_FirstMessage(pEUI64Address, u8Identifier, u16EAPDataLength, pEAPData);
				} else {
					LOG_ERR(Log("_Join_Process_Challenge() Drop unexpected CHALLENGE_1"));
				}
			} else if (u8TSubfield == EAP_PSK_T2) {
				/* this message can be received in the following states: */
				/* - STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE: */
				/*    as normal bootstrap procedure */
				/* - STATE_BOOT_WAIT_ACCEPTED or  STATE_BOOT_SENT_EAPPSK_SECOND_JOINING or STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING: */
				/*    as a repetition related to a non response of the previous processing (maybe the response was lost) */
				/*    If a peer receives a valid duplicate Request for which it has already sent a Response, it MUST resend its */
				/*    original Response without reprocessing the Request. */
				if (_IsBootState(STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE | STATE_BOOT_WAIT_ACCEPTED | STATE_BOOT_SENT_EAPPSK_SECOND_JOINING | STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING)) {
					/* enforce the current state in order to correctly update state machine after sending this message */
					_SetBootState(STATE_BOOT_WAIT_EAPPSK_THIRD_MESSAGE);
					/* hardcoded length of 22 bytes representing: the first 22 bytes of the EAP Request or Response packet used to compute the auth tag */
					_Join_Process_Challenge_ThirdMessage(22, pBootStrappingData, pEUI64Address, u8Identifier, u16EAPDataLength, pEAPData);
				} else {
					LOG_ERR(Log("_Join_Process_Challenge() Drop unexpected CHALLENGE_2"));
				}
			}

			/* else invalid message */
		}

		/* else invalid message */
	}

	/* else decode error */
}

/**********************************************************************************************************************/

/** Network joining
 *
 ***********************************************************************************************************************
 *
 * This function is called internally (from this file) when the first EAP-PSK message is received
 *
 **********************************************************************************************************************/
static void _Join_Process_Challenge_FirstMessage(const struct TAdpExtendedAddress *pEUI64Address,
		uint8_t u8EAPIdentifier, uint16_t u16EAPDataLength, uint8_t *pEAPData)
{
	struct TEapPskRand randS;
	struct TEapPskNetworkAccessIdentifier idS, IdP;
	_GetIdP(&IdP);

	/* In order to detect if is a valid repetition we have to check the 2 elements carried by the first EAP-PSK message: RandS and IdS */
	/* In case of a valid repetition we have to send back the same response message */
	if (EAP_PSK_Decode_Message1(u16EAPDataLength, pEAPData, &randS, &idS)) {
		struct TAdpAddress dstAddr;
		uint8_t au8Buffer[BOOTSTRAP_COMMAND_BUFFER_SIZE];

		/* encode and send the message T1 */
		uint8_t *pMemoryBuffer = au8Buffer;
		uint16_t u16MemoryBufferLength = sizeof(au8Buffer);
		uint16_t u16RequestLength = 0;

		bool bRepetition = (memcmp(randS.m_au8Value,
				g_LbpContext.m_PskContext.m_RandS.m_au8Value,
				sizeof(randS.m_au8Value)) == 0) &&
				(memcmp(idS.m_au8Value,
				g_LbpContext.m_PskContext.m_IdS.m_au8Value,
				g_LbpContext.m_PskContext.m_IdS.m_u8Length) == 0);

		if (!bRepetition) {
			struct TEapPskRand randP;
			_GetRandP(&randP);
			/* save current values; needed to detect repetitions */
			memcpy(g_LbpContext.m_PskContext.m_RandS.m_au8Value, randS.m_au8Value, sizeof(randS.m_au8Value));
			memcpy(g_LbpContext.m_PskContext.m_IdS.m_au8Value, idS.m_au8Value, idS.m_u8Length);
			g_LbpContext.m_PskContext.m_IdS.m_u8Length = idS.m_u8Length;

			/* process RandP */
			/* use the value from MIB if set by user */
			if (memcmp(&randP,
					"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 16) != 0) {
				memcpy(g_LbpContext.m_PskContext.m_RandP.m_au8Value,
						&randP, sizeof(struct TEapPskRand));
			} else {
				/* initialize RandP with random content */
				Random128(g_LbpContext.m_PskContext.m_RandP.m_au8Value);
			}

			EAP_PSK_InitializeTEK(&g_LbpContext.m_PskContext.m_RandP,
					&g_LbpContext.m_PskContext);
		}

		u16RequestLength = EAP_PSK_Encode_Message2(&g_LbpContext.m_PskContext, u8EAPIdentifier,
				&randS, &g_LbpContext.m_PskContext.m_RandP,
				&g_LbpContext.m_PskContext.m_IdS, &IdP,
				u16MemoryBufferLength, pMemoryBuffer);

		/* Encode now the LBP message */
#ifdef G3_HYBRID_PROFILE
		u16RequestLength = LBP_Encode_JoiningRequest(pEUI64Address, g_LbpContext.m_u8MediaType, u16RequestLength, u16MemoryBufferLength, pMemoryBuffer);
#else
		u16RequestLength = LBP_Encode_JoiningRequest(pEUI64Address, u16RequestLength, u16MemoryBufferLength, pMemoryBuffer);
#endif

		dstAddr.m_u8AddrSize = ADP_ADDRESS_16BITS;
		dstAddr.m_u16ShortAddr = g_LbpContext.m_u16LbaAddress;

		/* if this is a rekey procedure (instead of a join) arm here the timer in order to control the rekey; */
		/* messages can be loss and we want to clean the context after a while */
		if (_Joined()) {
			/* start rekey timer */
			g_LbpContext.m_JoinTimer.m_fnctCallback = _Rekey_TimerExpired_Callback;
			Timer_Register(&g_LbpContext.m_JoinTimer, _GetMaxJoinWaitTime());
		}

		g_LbpContext.m_u8PendingConfirms++;
		_SetBootState(STATE_BOOT_SENT_EAPPSK_SECOND_JOINING);
		AdpLbpRequestExt(_Joined() ? ADP_ADDRESS_16BITS : ADP_ADDRESS_64BITS, &dstAddr, /* send back to origin */
				u16RequestLength, pMemoryBuffer,
				false, /* multicast */
				false, /* auto */
				_Join_Callback_DataSend /* callback */);
	} else {
		LOG_ERR(Log("ADPM_Network_Join_Request() Invalid server response: EAP_PSK_Decode_Message1"));

		/* Wait for NetworkJoin timeout */
	}
}

/**********************************************************************************************************************/

/** Network joining
 *
 ***********************************************************************************************************************
 *
 * This function is called internally (from this file) when the third EAP-PSK message is received
 *
 **********************************************************************************************************************/
static void _Join_Process_Challenge_ThirdMessage(uint16_t u16HeaderLength, uint8_t *pHeader,
		const struct TAdpExtendedAddress *pEUI64Address, uint8_t u8EAPIdentifier, uint16_t u16EAPDataLength,
		uint8_t *pEAPData)
{
	struct TEapPskRand randS;
	uint8_t u8PChannelResult = 0;
	uint32_t u32Nonce = 0;
	uint16_t u16PChannelDataLength = 0;
	uint8_t *pPChannelData = 0L;
	UNUSED(pEUI64Address);

	/* If rand_S does not meet with the one that identifies the */
	if (EAP_PSK_Decode_Message3(u16EAPDataLength, pEAPData, &g_LbpContext.m_PskContext,
			u16HeaderLength, pHeader, &randS, &u32Nonce, &u8PChannelResult, &u16PChannelDataLength, &pPChannelData) &&
			(u8PChannelResult == PCHANNEL_RESULT_DONE_SUCCESS)) {
		if (memcmp(randS.m_au8Value, g_LbpContext.m_PskContext.m_RandS.m_au8Value, sizeof(randS.m_au8Value)) == 0) {
			/* encode and send the message T4 */
			struct TAdpAddress dstAddr;
			uint8_t au8Buffer[BOOTSTRAP_COMMAND_BUFFER_SIZE];

			uint8_t *pMemoryBuffer = au8Buffer;
			uint16_t u16MemoryBufferLength = sizeof(au8Buffer);
			uint16_t u16RequestLength = 0;
			uint8_t u8ReceivedParameters = 0;
			uint8_t au8ParameterResult[10]; /* buffer to encode the parameter result */
			uint8_t u8ParameterResultLength = 0;
			u8PChannelResult = PCHANNEL_RESULT_DONE_FAILURE;

			/* ParameterResult carry config parameters */
			au8ParameterResult[0] = EAP_EXT_TYPE_CONFIGURATION_PARAMETERS;
			u8ParameterResultLength++;

			/* If one or more parameter were invalid or missing, the peer send a message 4 with R = DONE_FAILURE and */
			/* an embedded configuration field with at least one Parameter-result indicating the error. */

			/* check if protected data carries EXT field */
			if (pPChannelData[0] == EAP_EXT_TYPE_CONFIGURATION_PARAMETERS) {
				if (_ProcessParameters(u16PChannelDataLength - 1, &pPChannelData[1],
						&u8ReceivedParameters, &u8ParameterResultLength, au8ParameterResult)) {
					u8PChannelResult = PCHANNEL_RESULT_DONE_SUCCESS;
				}
			} else {
				/* build ParameterResult indicating missing GMK key */
				au8ParameterResult[u8ParameterResultLength++] = CONF_PARAM_RESULT;
				au8ParameterResult[u8ParameterResultLength++] = 2;
				au8ParameterResult[u8ParameterResultLength++] = RESULT_MISSING_REQUIRED_PARAMETER;
				au8ParameterResult[u8ParameterResultLength++] = CONF_PARAM_GMK;
			}

			/* after receiving from the server a valid EAP-PSK message with Nonce */
			/* set to N, the peer will answer with an EAP-PSK message with Nonce set to N+1 */
			u16RequestLength = EAP_PSK_Encode_Message4(&g_LbpContext.m_PskContext, u8EAPIdentifier,
					&randS, u32Nonce + 1, u8PChannelResult, u8ParameterResultLength, au8ParameterResult, u16MemoryBufferLength,
					pMemoryBuffer);

			/* Encode now the LBP message */
#ifdef G3_HYBRID_PROFILE
			u16RequestLength = LBP_Encode_JoiningRequest(&g_LbpContext.m_EUI64Address, g_LbpContext.m_u8MediaType,
					u16RequestLength, u16MemoryBufferLength, pMemoryBuffer);
#else
			u16RequestLength = LBP_Encode_JoiningRequest(&g_LbpContext.m_EUI64Address,
					u16RequestLength, u16MemoryBufferLength, pMemoryBuffer);
#endif

			dstAddr.m_u8AddrSize = ADP_ADDRESS_16BITS;
			dstAddr.m_u16ShortAddr = g_LbpContext.m_u16LbaAddress;

			if (u8PChannelResult != PCHANNEL_RESULT_DONE_SUCCESS) {
				LOG_ERR(Log("ADPM_Network_Join_Request() EAP_PSK_Decode_Message3() Invalid parameters"));
				/* wait for timeout */
			}

			g_LbpContext.m_u8PendingConfirms++;
			_SetBootState(STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING);
			AdpLbpRequestExt(_Joined() ? ADP_ADDRESS_16BITS : ADP_ADDRESS_64BITS, &dstAddr, /* send back to origin */
					u16RequestLength, pMemoryBuffer, false, /* multicast */
					false, _Join_Callback_DataSend /* callback */
					);
		} else {
			LOG_ERR(Log("ADPM_Network_Join_Request() EAP_PSK_Decode_Message3()"));
			/* wait for timeout */
		}
	} else {
		LOG_ERR(Log("ADPM_Network_Join_Request() EAP_PSK_Decode_Message3()"));
		/* wait for timeout */
	}
}

/**********************************************************************************************************************/

/** Network joining
 *
 ***********************************************************************************************************************
 *
 * This function is called by the 6LoWPANProtocol when a LBP Accepted message is received
 *
 * Accepted message can embed: EAP-Success or Configuration parameters
 *
 **********************************************************************************************************************/
static void _Join_Process_Accepted(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData)
{
	/* check the first byte of the Bootstrapping data in order to detect */
	/* the type of the embedded message: EAP or Configuration */
	if ((pBootStrappingData[0] & 0x01) == 0x00) {
		/* EAP message */
		/* check state: this message can be received also when JOINED for re-key procedure */
		if (_IsBootState(STATE_BOOT_WAIT_ACCEPTED) || _IsBootState(STATE_BOOT_SENT_EAPPSK_FOURTH_JOINING)) {
			_Join_Process_Accepted_EAP(pEUI64Address, u16BootStrappingDataLength, pBootStrappingData);
		} else {
			LOG_ERR(Log("_Join_Process_Accepted() Drop unexpected Accepted_EAP"));
		}
	} else {
		/* Configuration message */
		if (_IsBootState(STATE_BOOT_JOINED)) {
			_Join_Process_Accepted_Configuration(pEUI64Address, u16BootStrappingDataLength, pBootStrappingData);
		} else {
			LOG_ERR(Log("_Join_Process_Accepted() Drop unexpected Accepted_Configuration"));
		}
	}
}

/**********************************************************************************************************************/

/** Network joining
 *
 ***********************************************************************************************************************
 *
 * This function is called by the 6LoWPANProtocol when a LBP Accepted embedding an EAP message is received
 *
 **********************************************************************************************************************/
static void _Join_Process_Accepted_EAP(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData)
{
	uint8_t u8Code = 0;
	uint8_t u8Identifier = 0;
	uint8_t u8TSubfield = 0;
	uint16_t u16EAPDataLength = 0;
	uint8_t *pEAPData = 0L;
	UNUSED(pEUI64Address);

	if (EAP_PSK_Decode_Message(u16BootStrappingDataLength, pBootStrappingData, &u8Code, &u8Identifier, &u8TSubfield,
			&u16EAPDataLength, &pEAPData)) {
		if (u8Code == EAP_SUCCESS) {
			/* set the encryption key into mac layer */
			LOG_INFO(
					Log("NetworkJoin() Join / rekey process finish: Set the GMK encryption key %u into the MAC layer", g_LbpContext.m_GroupMasterKey.m_u8KeyId));
			if (AdpMac_SetGroupMasterKeySync(&g_LbpContext.m_GroupMasterKey)) {
				LOG_DBG(
						Log("NetworkJoin() Join / rekey process finish: Active key id is %u", _GetActiveKeyIndex()));
				/* If joining the network (not yet joined) we have to do more initialisation */
				if (!_Joined()) {
					/* A device shall initialise RC_COORD to 0x7FFF on association. */
					LOG_DBG(Log("NetworkJoin() Join process finish: Set the RcCoord into the MAC layer"));
					if (AdpMac_SetRcCoordSync(0x7FFF)) {
						/* set the short address in the mac layer */
						LOG_DBG(Log("NetworkJoin() Join process finish: Set the ShortAddress 0x%04X into the MAC layer", g_LbpContext.m_u16JoiningShortAddress));

						if (AdpMac_SetShortAddressSync(g_LbpContext.m_u16JoiningShortAddress)) {
							g_LbpContext.m_u16ShortAddress = g_LbpContext.m_u16JoiningShortAddress;
							_Join_Confirm(G3_SUCCESS);
						} else {
							_Join_Confirm(G3_FAILED);
						}
					} else {
						_Join_Confirm(G3_FAILED);
					}
				} else {
					/* already joined; this was the rekey procedure; set the state to STATE_BOOT_JOINED */
					_SetBootState(STATE_BOOT_JOINED);
					Timer_Unregister(&g_LbpContext.m_JoinTimer);
				}
			}
		} else {
			LOG_ERR(Log("ADPM_Network_Join_Request() u8Code != EAP_SUCCESS"));
			if (_Joined()) {
				_Join_Confirm(G3_NOT_PERMITED);
			}
		}
	} else {
		LOG_ERR(Log("ADPM_Network_Join_Request() EAP_PSK_Decode_Message"));

		/* Wait for NetworkJoin timeout */
	}
}

/**********************************************************************************************************************/

/** Network joining
 *
 ***********************************************************************************************************************
 *
 * This function is called by the 6LoWPANProtocol when a LBP Accepted embedding a Configuration message is received
 *
 **********************************************************************************************************************/
static void _Join_Process_Accepted_Configuration(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint8_t *pBootStrappingData)
{
	UNUSED(pEUI64Address);

	/* this must be rekey or key removal */
	/* TODO: we are now supporting only key activation, handle also other parameters */
	if (_Joined()) {
		struct TAdpAddress dstAddr;
		uint8_t au8Buffer[BOOTSTRAP_COMMAND_BUFFER_SIZE];

		uint8_t *pMemoryBuffer = au8Buffer;
		uint16_t u16MemoryBufferLength = sizeof(au8Buffer);
		uint16_t u16RequestLength = 0;

		uint8_t u8ReceivedParametersMask = 0;
		uint8_t au8ParameterResult[20];
		uint8_t u8ParameterResultLength = 0;

		_ProcessParameters(u16BootStrappingDataLength, pBootStrappingData,
				&u8ReceivedParametersMask, &u8ParameterResultLength, au8ParameterResult);

		memcpy(pMemoryBuffer, au8ParameterResult, u8ParameterResultLength);

		/* Encode now the LBP message */
#ifdef G3_HYBRID_PROFILE
		u16RequestLength = LBP_Encode_JoiningRequest(&g_LbpContext.m_EUI64Address, g_LbpContext.m_u8MediaType,
				u8ParameterResultLength, u16MemoryBufferLength, pMemoryBuffer);
#else
		u16RequestLength = LBP_Encode_JoiningRequest(&g_LbpContext.m_EUI64Address,
				u8ParameterResultLength, u16MemoryBufferLength, pMemoryBuffer);
#endif

		dstAddr.m_u8AddrSize = ADP_ADDRESS_16BITS;
		dstAddr.m_u16ShortAddr = g_LbpContext.m_u16LbaAddress;

		AdpLbpRequestExt(ADP_ADDRESS_16BITS, &dstAddr, /* send back to origin */
				u16RequestLength, pMemoryBuffer, false, /* multicast */
				true, _Dummy_Callback_DataSend /* callback */
				);
	}

	/* if not joined, ignore this message */
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void LBP_Reset()
{
	memset(&g_LbpContext, 0, sizeof(g_LbpContext));
	g_LbpContext.m_u16ShortAddress = 0xFFFF;
}

void LBP_ClearEapContext()
{
	memset(&g_LbpContext.m_PskContext.m_Tek, 0,
			sizeof(g_LbpContext.m_PskContext.m_Tek));

	memset(&g_LbpContext.m_PskContext.m_IdS, 0,
			sizeof(g_LbpContext.m_PskContext.m_IdS));

	memset(&g_LbpContext.m_PskContext.m_RandP, 0,
			sizeof(g_LbpContext.m_PskContext.m_RandP));

	memset(&g_LbpContext.m_PskContext.m_RandS, 0,
			sizeof(g_LbpContext.m_PskContext.m_RandS));
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void LBP_ForceJoinStatus(bool bJoined)
{
	if (bJoined) {
		/* set back automate state */
		_SetBootState(STATE_BOOT_JOINED);
	} else {
		/* never should be here but check anyway */
		_SetBootState(STATE_BOOT_NOT_JOINED);
		g_LbpContext.m_u16ShortAddress = 0xFFFF;
		g_LbpContext.m_u16JoiningShortAddress = 0xFFFF;
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void LBP_ForceJoined(uint16_t u16ShortAddress,
		struct TAdpExtendedAddress *pEUI64Address,
		BootstrapWrapper_JoinConfirm fnctJoinConfirm,
		BootstrapWrapper_KickNotify fnctKickNotify)
{
	g_LbpContext.m_u16ShortAddress = u16ShortAddress;
	memcpy(&g_LbpContext.m_EUI64Address, pEUI64Address, sizeof(struct TAdpExtendedAddress));
	g_LbpContext.m_LbpNotifications.fnctJoinConfirm = fnctJoinConfirm;
	g_LbpContext.m_LbpNotifications.fnctKickNotify = fnctKickNotify;
	_SetBootState(STATE_BOOT_JOINED);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void LBP_InitEapPsk(const struct TEapPskKey *pKey)
{
	EAP_PSK_Initialize(pKey, &g_LbpContext.m_PskContext);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
#ifdef G3_HYBRID_PROFILE
void LBP_JoinRequest(uint16_t u16LbaAddress, uint8_t u8Mediatype, struct TAdpExtendedAddress *pEUI64Address,
		BootstrapWrapper_JoinConfirm fnctJoinConfirm,
		BootstrapWrapper_KickNotify fnctKickNotify)
#else
void LBP_JoinRequest(uint16_t u16LbaAddress, struct TAdpExtendedAddress *pEUI64Address,
		BootstrapWrapper_JoinConfirm fnctJoinConfirm,
		BootstrapWrapper_KickNotify fnctKickNotify)
#endif
{
	struct TAdpAddress dstAddr;
	uint8_t au8Buffer[BOOTSTRAP_COMMAND_BUFFER_SIZE];

	/* we have to send the Joining request */
	uint8_t *pMemoryBuffer = au8Buffer;
	uint16_t u16MemoryBufferLength = sizeof(au8Buffer);
	uint16_t u16RequestLength = 0;

	/* Store params in the context */
	g_LbpContext.m_u16LbaAddress = u16LbaAddress;
	memcpy(&g_LbpContext.m_EUI64Address, pEUI64Address, sizeof(struct TAdpExtendedAddress));
	g_LbpContext.m_LbpNotifications.fnctJoinConfirm = fnctJoinConfirm;
	g_LbpContext.m_LbpNotifications.fnctKickNotify = fnctKickNotify;
#ifdef G3_HYBRID_PROFILE
	g_LbpContext.m_u8MediaType = u8Mediatype;
#endif

	/* Reset Joining short address */
	g_LbpContext.m_u16JoiningShortAddress = 0xFFFF;

	/* prepare and send the JoinRequest; no Bootstrapping data for the first request */
#ifdef G3_HYBRID_PROFILE
	u16RequestLength = LBP_Encode_JoiningRequest(&g_LbpContext.m_EUI64Address, u8Mediatype, 0, /* u16BootStrappingDataLength */
			u16MemoryBufferLength, pMemoryBuffer);
#else
	u16RequestLength = LBP_Encode_JoiningRequest(&g_LbpContext.m_EUI64Address, 0, /* u16BootStrappingDataLength */
			u16MemoryBufferLength, pMemoryBuffer);
#endif

	dstAddr.m_u8AddrSize = ADP_ADDRESS_16BITS;
	dstAddr.m_u16ShortAddr = g_LbpContext.m_u16LbaAddress;

	LOG_INFO(Log("Registering Network-JOIN timer: %u seconds", _GetMaxJoinWaitTime()));

	/* start MaxJoinWait timer */
	g_LbpContext.m_JoinTimer.m_fnctCallback = _Join_TimerExpired_Callback;

	Timer_Register(&g_LbpContext.m_JoinTimer,
			_GetMaxJoinWaitTime() * 10);

	g_LbpContext.m_u8PendingConfirms++;
	_SetBootState(STATE_BOOT_SENT_FIRST_JOINING);
	/* it's important to have this function called the last one as if it fails it will call the "callback" */
	AdpLbpRequestExt(ADP_ADDRESS_64BITS, &dstAddr, u16RequestLength, pMemoryBuffer,
			false, /* multicast */
			false, _Join_Callback_DataSend);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void LBP_ProcessMessage(const struct TAdpAddress *pSrcDeviceAddress, uint16_t u16MessageLength,
		uint8_t *pMessageBuffer, uint8_t u8LinkQualityIndicator, bool bSecurityEnabled)
{
	/* The coordinator will never get here as is handled by the application using LDB messages */
	uint8_t pu8MessageType = 0;
	struct TAdpExtendedAddress eui64Address;
	uint16_t u16BootStrappingDataLength = 0;
	uint8_t *pBootStrappingData = 0L;
	UNUSED(pSrcDeviceAddress);
	UNUSED(u8LinkQualityIndicator);
	UNUSED(bSecurityEnabled);

	if (LBP_Decode_Message(u16MessageLength, pMessageBuffer, &pu8MessageType, &eui64Address, &u16BootStrappingDataLength,
			&pBootStrappingData)) {
		/* if we are not the coordinator and we are in the network and this bootstrap message is not for us */
		if (memcmp(&eui64Address, &g_LbpContext.m_EUI64Address,
				sizeof(struct TAdpExtendedAddress)) != 0) {
			if (_Joined()) {
				/* LBA (agent): forward the message between server and device */
				struct TAdpAddress dstAddr;
				uint8_t au8Buffer[BOOTSTRAP_COMMAND_BUFFER_SIZE];
				uint8_t *pMemoryBuffer = au8Buffer;

				/* message should be relayed to the server or to the device */
				if (pu8MessageType == LBP_JOINING) {
					/* relay to the server */
					dstAddr.m_u8AddrSize = ADP_ADDRESS_16BITS;
					dstAddr.m_u16ShortAddr = _GetCoordShortAddress();
				} else { /* if ((pu8MessageType == LBP_ACCEPTED) || (pu8MessageType == LBP_CHALLENGE)
					 || (pu8MessageType == LBP_DECLINE))*/
					 /* relay to the device */
					dstAddr.m_u8AddrSize = ADP_ADDRESS_64BITS;
					memcpy(&dstAddr.m_ExtendedAddress, &eui64Address, 8);
				}

				memcpy(pMemoryBuffer, pMessageBuffer, u16MessageLength);

				AdpLbpRequestExt(ADP_ADDRESS_16BITS, &dstAddr, u16MessageLength, pMemoryBuffer,
						false, /* multicast */
						false, _Dummy_Callback_DataSend /* callback */
						);
			} else {
				LOG_ERR(Log("Received message for invalid EUI64; Drop it!"));
			}
		} else {
			/* only end-device will be handled here */
			if (_Joined()) {
				if (pu8MessageType == LBP_CHALLENGE) {
					/* reuse function from join */
					_Join_Process_Challenge(&eui64Address, u16BootStrappingDataLength, pBootStrappingData);
				} else if (pu8MessageType == LBP_ACCEPTED) {
					/* reuse function from join */
					_Join_Process_Accepted(&eui64Address, u16BootStrappingDataLength, pBootStrappingData);
				} else if (pu8MessageType == LBP_DECLINE) {
					LOG_ERR(Log("LBP_DECLINE: Not implemented"));
				} else if (pu8MessageType == LBP_KICK_TO_LBD) {
					_Kick_Notify();
				} else if (pu8MessageType == LBP_KICK_FROM_LBD) {
					LOG_ERR(Log("LBP_KICK_FROM_LBD: Never should be here"));
				} else if (pu8MessageType == LBP_JOINING) {
					LOG_ERR(Log("LBP_JOINING: Never should be here"));
				} else {
					LOG_ERR(Log("Unsupported LBP message: %u", pu8MessageType));
				}
			} else {
				if (pu8MessageType == LBP_CHALLENGE) {
					_Join_Process_Challenge(&eui64Address, u16BootStrappingDataLength, pBootStrappingData);
				} else if (pu8MessageType == LBP_ACCEPTED) {
					_Join_Process_Accepted(&eui64Address, u16BootStrappingDataLength, pBootStrappingData);
				} else if (pu8MessageType == LBP_DECLINE) {
					LOG_INFO(Log("LBP_DECLINE"));

					_Join_Confirm(G3_NOT_PERMITED );
				} else if (pu8MessageType == LBP_KICK_TO_LBD) {
					LOG_ERR(Log("LBP_KICK_TO_LBD: Not joined!"));
				} else if (pu8MessageType == LBP_KICK_FROM_LBD) {
					LOG_ERR(Log("LBP_KICK_FROM_LBD: Never should be here"));
				} else if (pu8MessageType == LBP_JOINING) {
					LOG_ERR(Log("LBP_JOINING: Never should be here"));
				} else {
					LOG_ERR(Log("Unsupported LBP message: %u", pu8MessageType));
				}
			}
		}
	}

	/* else: decoding error */
}

void LBP_LeaveRequest(const struct TAdpExtendedAddress *pEUI64Address, ADP_Common_DataSend_Callback callback)
{
	struct TAdpAddress dstAddr;
	uint8_t au8Buffer[BOOTSTRAP_COMMAND_BUFFER_SIZE];

	uint8_t *pMemoryBuffer = au8Buffer;
	uint16_t u16MemoryBufferLength = sizeof(au8Buffer);
	uint16_t u16RequestLength = 0;

	u16RequestLength = LBP_Encode_KickFromLBDRequest(pEUI64Address, u16MemoryBufferLength, pMemoryBuffer);

	dstAddr.m_u8AddrSize = ADP_ADDRESS_16BITS;
	dstAddr.m_u16ShortAddr = _GetCoordShortAddress();

	AdpLbpRequestExt(ADP_ADDRESS_16BITS, &dstAddr, /* send back to origin */
			u16RequestLength, pMemoryBuffer, false, /* multicast */
			false, callback);
}
