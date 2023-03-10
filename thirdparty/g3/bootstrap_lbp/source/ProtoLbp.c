#include <stdbool.h>
#include <stdint.h>
#include <AdpApiTypes.h>
#include <ProtoLbp.h>
#include <string.h>

#ifdef G3_HYBRID_PROFILE
/* Disable backup medium flag to add in LBP Header */
#define DISABLE_BACKUP_FLAG   0x01
#endif

/**********************************************************************************************************************/

/** The LBP_Encode_KickFromLBDRequest primitive is used to encode the LBP KICK type message generated by LBD
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
uint16_t LBP_Encode_KickFromLBDRequest(const struct TAdpExtendedAddress *pEUI64Address, uint16_t u16MessageLength,
		uint8_t *pMessageBuffer)
{
	uint16_t u16EncodedLength = 0;

	/* check first if the message buffer size if enough */
	if (u16MessageLength >= sizeof(struct TAdpExtendedAddress) + 2) {
		/* start message encoding */
#ifdef G3_HYBRID_PROFILE
		/* DISABLE_BACKUP_FLAG not set in Kick frames, MediaType set to 0x0 */
		pMessageBuffer[0] = (LBP_KICK_FROM_LBD << 4);
#else
		pMessageBuffer[0] = (LBP_KICK_FROM_LBD << 4);
#endif
		pMessageBuffer[1] = 0; /* transaction id is reserved */

		memcpy(&pMessageBuffer[2], pEUI64Address, sizeof(struct TAdpExtendedAddress));

		u16EncodedLength = sizeof(struct TAdpExtendedAddress) + 2;
	}

	return u16EncodedLength;
}

#ifdef G3_HYBRID_PROFILE

/**********************************************************************************************************************/

/** LBP_Encode_JoiningRequest
 **********************************************************************************************************************/
uint16_t LBP_Encode_JoiningRequest(const struct TAdpExtendedAddress *pEUI64Address, uint8_t u8MediaType,
		uint16_t u16BootStrappingDataLength, uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	uint16_t u16EncodedLength = 0;

	/* check first if the message buffer size if enough */
	if (u16MessageLength >= u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2) {
		/* as the bootstrapping data is already in the message buffer, move it to its place */
		/* after the A_LDB field */
		memmove(&pMessageBuffer[sizeof(struct TAdpExtendedAddress) + 2], pMessageBuffer, u16BootStrappingDataLength);

		/* start message encoding */
		pMessageBuffer[0] = (LBP_JOINING << 4) | (u8MediaType << 3) | (DISABLE_BACKUP_FLAG << 2);
		pMessageBuffer[1] = 0; /* transaction id is reserved */

		memcpy(&pMessageBuffer[2], pEUI64Address, sizeof(struct TAdpExtendedAddress));

		u16EncodedLength = u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2;
	}

	return u16EncodedLength;
}

/**********************************************************************************************************************/

/** The LBP_Encode_ChallengeRequest primitive is used to encode the LBP CHALLENGE type message.
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
uint16_t LBP_Encode_ChallengeRequest(const struct TAdpExtendedAddress *pEUI64Address, uint8_t u8MediaType,
		uint8_t u8DisableBackupMedium, uint16_t u16BootStrappingDataLength, uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	uint16_t u16EncodedLength = 0;

	/* check first if the message buffer size if enough */
	if (u16MessageLength >= u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2) {
		/* as the bootstrapping data is already in the message buffer, move it to its place */
		/* after the A_LDB field */
		memmove(&pMessageBuffer[sizeof(struct TAdpExtendedAddress) + 2], pMessageBuffer, u16BootStrappingDataLength);

		/* start message encoding */
		pMessageBuffer[0] = (LBP_CHALLENGE << 4) | (u8MediaType << 3) | (u8DisableBackupMedium << 2);
		pMessageBuffer[1] = 0; /* transaction id is reserved */

		memcpy(&pMessageBuffer[2], pEUI64Address, sizeof(struct TAdpExtendedAddress));

		u16EncodedLength = u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2;
	}

	return u16EncodedLength;
}

/**********************************************************************************************************************/

/** The LBP_Encode_AcceptedRequest primitive is used to encode the LBP ACCEPTED type message.
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
uint16_t LBP_Encode_AcceptedRequest(const struct TAdpExtendedAddress *pEUI64Address, uint8_t u8MediaType,
		uint8_t u8DisableBackupMedium, uint16_t u16BootStrappingDataLength, uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	uint16_t u16EncodedLength = 0;

	/* check first if the message buffer size if enough */
	if (u16MessageLength >= u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2) {
		/* as the bootstrapping data is already in the message buffer, move it to its place */
		/* after the A_LDB field */
		memmove(&pMessageBuffer[8 + 2], pMessageBuffer, u16BootStrappingDataLength);

		/* start message encoding */
		pMessageBuffer[0] = (LBP_ACCEPTED << 4) | (u8MediaType << 3) | (u8DisableBackupMedium << 2);
		pMessageBuffer[1] = 0; /* transaction id is reserved */

		memcpy(&pMessageBuffer[2], pEUI64Address, sizeof(struct TAdpExtendedAddress));

		u16EncodedLength = u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2;
	}

	return u16EncodedLength;
}

#else

/**********************************************************************************************************************/

/** LBP_Encode_JoiningRequest
 **********************************************************************************************************************/
uint16_t LBP_Encode_JoiningRequest(const struct TAdpExtendedAddress *pEUI64Address, uint16_t u16BootStrappingDataLength,
		uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	uint16_t u16EncodedLength = 0;

	/* check first if the message buffer size if enough */
	if (u16MessageLength >= u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2) {
		/* as the bootstrapping data is already in the message buffer, move it to its place */
		/* after the A_LDB field */
		memmove(&pMessageBuffer[sizeof(struct TAdpExtendedAddress) + 2], pMessageBuffer, u16BootStrappingDataLength);

		/* start message encoding */
		pMessageBuffer[0] = (LBP_JOINING << 4);
		pMessageBuffer[1] = 0; /* transaction id is reserved */

		memcpy(&pMessageBuffer[2], pEUI64Address, sizeof(struct TAdpExtendedAddress));

		u16EncodedLength = u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2;
	}

	return u16EncodedLength;
}

/**********************************************************************************************************************/

/** The LBP_Encode_ChallengeRequest primitive is used to encode the LBP CHALLENGE type message.
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
uint16_t LBP_Encode_ChallengeRequest(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	uint16_t u16EncodedLength = 0;

	/* check first if the message buffer size if enough */
	if (u16MessageLength >= u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2) {
		/* as the bootstrapping data is already in the message buffer, move it to its place */
		/* after the A_LDB field */
		memmove(&pMessageBuffer[sizeof(struct TAdpExtendedAddress) + 2], pMessageBuffer, u16BootStrappingDataLength);

		/* start message encoding */
		pMessageBuffer[0] = (LBP_CHALLENGE << 4);
		pMessageBuffer[1] = 0; /* transaction id is reserved */

		memcpy(&pMessageBuffer[2], pEUI64Address, sizeof(struct TAdpExtendedAddress));

		u16EncodedLength = u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2;
	}

	return u16EncodedLength;
}

/**********************************************************************************************************************/

/** The LBP_Encode_AcceptedRequest primitive is used to encode the LBP ACCEPTED type message.
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
uint16_t LBP_Encode_AcceptedRequest(const struct TAdpExtendedAddress *pEUI64Address,
		uint16_t u16BootStrappingDataLength, uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	uint16_t u16EncodedLength = 0;

	/* check first if the message buffer size if enough */
	if (u16MessageLength >= u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2) {
		/* as the bootstrapping data is already in the message buffer, move it to its place */
		/* after the A_LDB field */
		memmove(&pMessageBuffer[8 + 2], pMessageBuffer, u16BootStrappingDataLength);

		/* start message encoding */
		pMessageBuffer[0] = (LBP_ACCEPTED << 4);
		pMessageBuffer[1] = 0; /* transaction id is reserved */

		memcpy(&pMessageBuffer[2], pEUI64Address, sizeof(struct TAdpExtendedAddress));

		u16EncodedLength = u16BootStrappingDataLength + sizeof(struct TAdpExtendedAddress) + 2;
	}

	return u16EncodedLength;
}

#endif

/**********************************************************************************************************************/

/** LBP_IsChallengeResponse
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
bool LBP_IsChallengeResponse(uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	return ((u16MessageLength > 1) && ((pMessageBuffer[0] & (LBP_CHALLENGE << 4)) == LBP_CHALLENGE));
}

/**********************************************************************************************************************/

/** LBP_IsAcceptedResponse
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
bool LBP_IsAcceptedResponse(uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	return ((u16MessageLength > 1) && ((pMessageBuffer[0] & (LBP_ACCEPTED << 4)) == LBP_ACCEPTED));
}

/**********************************************************************************************************************/

/** LBP_IsDeclineResponse
 *
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
bool LBP_IsDeclineResponse(uint16_t u16MessageLength, uint8_t *pMessageBuffer)
{
	return ((u16MessageLength > 1) && ((pMessageBuffer[0] & (LBP_DECLINE << 4)) == LBP_DECLINE));
}

/**********************************************************************************************************************/

/** LBP_Decode_Message
 ***********************************************************************************************************************
 *
 **********************************************************************************************************************/
bool LBP_Decode_Message(uint16_t u16MessageLength, uint8_t *pMessageBuffer, uint8_t *pu8MessageType,
		struct TAdpExtendedAddress *pEUI64Address, uint16_t *pu16BootStrappingDataLength, uint8_t **pBootStrappingData)
{
	bool bRet = false;

	if (u16MessageLength >= 2 + sizeof(struct TAdpExtendedAddress)) {
		*pu8MessageType = ((pMessageBuffer[0] & 0xF0) >> 4);

		memcpy(&pEUI64Address->m_au8Value, &pMessageBuffer[2], sizeof(struct TAdpExtendedAddress));

		*pBootStrappingData = &pMessageBuffer[2 + sizeof(struct TAdpExtendedAddress)];
		*pu16BootStrappingDataLength = u16MessageLength - (2 + sizeof(struct TAdpExtendedAddress));

		bRet = true;
	}

	return bRet;
}
