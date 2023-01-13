/**********************************************************************************************************************/

/** \addtogroup AdaptationSublayer
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/

/** This file contains data types and functions specific to LoWPAN BootStrapping protocol (LBP)
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/
#include "Timer.h"
#include "ProtoEapPsk.h"
#include "BootstrapWrapper.h"
#include "AdpSharedTypes.h"

#ifndef __PROCESS_LBP_H__
#define __PROCESS_LBP_H__

struct TLBPContext {
	/*  uint8_t m_u8NsduHandle; */
	/* Timer to control the join or rekey process if no response is received */
	struct TTimer m_JoinTimer;
	/* Timer to control the kick process (see comments in code _Kick_Notify) */
	/*  struct TTimer m_KickTimer; */
	/* information related to joining */
	uint16_t m_u16LbaAddress;
	uint16_t m_u16JoiningShortAddress;
	/*  // keeps authentication context */
	struct TEapPskContext m_PskContext;
#ifdef G3_HYBRID_PROFILE
	/* Media Type to use between LBD and LBA. It is encoded in LBP frames */
  	uint8_t m_u8MediaType;
#endif

	/* State of the bootstrap process */
	uint8_t m_u8BootstrapState;
	/* Number of pending message confirms */
	uint8_t m_u8PendingConfirms;
	/*  // EUI64 address of the device */
	struct TAdpExtendedAddress m_EUI64Address;
	/*  // This parameter specifies the PAN ID: 0xFFFF means not connected to PAN */
	/*  uint16_t m_u16PanId; */
	/*  16-bit address for new device which is unique inside the PAN */
	uint16_t m_u16ShortAddress;
	/*  Holds the GMK key */
	struct TGroupMasterKey m_GroupMasterKey;

	struct TBootstrapNotifications m_LbpNotifications;
};

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/

/** The LBP_Reset primitive restarts the bootstrap protocol
 ***********************************************************************************************************************
 **********************************************************************************************************************/
void LBP_Reset(void);

/** The LBP_ClearEapContext primitive is used to clear the EAP context
 ***********************************************************************************************************************
 **********************************************************************************************************************/
void LBP_ClearEapContext(void);

/** The LBP_InitEapPsk primitive allows the upper layer to be notified of the completion of an
 * BootstrapWrapper_JoinRequest.
 ***********************************************************************************************************************
 * @param m_u8Status The status of the request.
 * @param m_u16NetworkAddress The 16-bit network address that was allocated to the device.
 * @param m_u16PanId The 16-bit address of the PAN of which the device is now a member.
 **********************************************************************************************************************/
void LBP_InitEapPsk(const struct TEapPskKey *pKey);

/** The LBP_ForceJoinStatus primitive allows the upper layer to set the join status
 ***********************************************************************************************************************
 * @param bJoined true to set the status as "joined", false to set it as "not joined"
 **********************************************************************************************************************/
void LBP_ForceJoinStatus(bool bJoined);

/** The LBP_ForceJoined primitive allows the upper layer to set the join status with callbacks:
 *   - fnctJoinConfirm callback will be called once the joining process ends.
 *   - fnctKickNotify callback will be called if the node has been kicked from the network.
 ***********************************************************************************************************************
 * @param m_u16ShortAddress The 16-bit network address to be forced
 * @param pEUI64Address The EUI64 address of the node.
 * @param fnctJoinConfirm Callback to notify the join confirm.
 * @param fnctKickNotify Callback to notify that the node has been kicked from the network.
 **********************************************************************************************************************/
void LBP_ForceJoined(uint16_t u16ShortAddress,
		struct TAdpExtendedAddress *pEUI64Address,
		BootstrapWrapper_JoinConfirm fnctJoinConfirm,
		BootstrapWrapper_KickNotify fnctKickNotify);

#ifdef G3_HYBRID_PROFILE
/** The LBP_JoinRequest primitive allows the upper layer to init the joining process.
 *  After calling BootstrapWrapper_JoinRequest(), the bootstrap module will create the joining message, and send it
 *  to the network using AdpLbpRequestExt() from ADP.
 *   - fnctJoinConfirm callback will be called once the joining process ends.
 *   - fnctKickNotify callback will be called if the node has been kicked from the network.
 ***********************************************************************************************************************
 * @param m_u16LbaAddress The short address of the LBA.
 * @param u8MediaType The Media Type to use when communiating with LBA.
 * @param pEUI64Address The EUI64 address of the node.
 * @param fnctJoinConfirm Callback to notify the join confirm.
 * @param fnctKickNotify Callback to notify that the node has been kicked from the network.
 **********************************************************************************************************************/
void LBP_JoinRequest(uint16_t u16LbaAddress, uint8_t u8MediaType, struct TAdpExtendedAddress *pEUI64Address,
		BootstrapWrapper_JoinConfirm fnctJoinConfirm,
		BootstrapWrapper_KickNotify fnctKickNotify);
#else
/** The LBP_JoinRequest primitive allows the upper layer to init the joining process.
 *  After calling BootstrapWrapper_JoinRequest(), the bootstrap module will create the joining message, and send it
 *  to the network using AdpLbpRequestExt() from ADP.
 *   - fnctJoinConfirm callback will be called once the joining process ends.
 *   - fnctKickNotify callback will be called if the node has been kicked from the network.
 ***********************************************************************************************************************
 * @param m_u16LbaAddress The short address of the LBA.
 * @param pEUI64Address The EUI64 address of the node.
 * @param fnctJoinConfirm Callback to notify the join confirm.
 * @param fnctKickNotify Callback to notify that the node has been kicked from the network.
 **********************************************************************************************************************/
void LBP_JoinRequest(uint16_t u16LbaAddress, struct TAdpExtendedAddress *pEUI64Address,
		BootstrapWrapper_JoinConfirm fnctJoinConfirm,
		BootstrapWrapper_KickNotify fnctKickNotify);
#endif

/** The LBP_ProcessMessage primitive allows the upper layer to pass the bootstrap messages received from
 *  the network, to the bootstrap module.
 ***********************************************************************************************************************
 * @param m_u8Status The status of the request.
 * @param m_u16NetworkAddress The 16-bit network address that was allocated to the device.
 * @param m_u16PanId The 16-bit address of the PAN of which the device is now a member.
 **********************************************************************************************************************/
void LBP_ProcessMessage(const struct TAdpAddress *pSrcDeviceAddress, uint16_t u16MessageLength,
		uint8_t *pMessageBuffer, uint8_t u8LinkQualityIndicator, bool bSecurityEnabled);

/** The LBP_LeaveRequest primitive allows the upper layer to request the leave from the network.
 ***********************************************************************************************************************
 * @param pEUI64Address The EUI64 address of the node.
 * @param callback Callback to notify that the leave has been sent.
 **********************************************************************************************************************/
void LBP_LeaveRequest(const struct TAdpExtendedAddress *pEUI64Address, ADP_Common_DataSend_Callback callback);
#endif

/**********************************************************************************************************************/

/** @}
 **********************************************************************************************************************/
