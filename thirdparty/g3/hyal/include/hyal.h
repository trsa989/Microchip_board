/**********************************************************************************************************************/
/** \addtogroup G3
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/** Defines the Hybrid Abstraction Layer between ADP and both MAC layers (G3-PLC and RF).
 *  This layer is used in G3 Hybrid profile.
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/
#ifndef __HYAL_H__
#define __HYAL_H__

#include "mac_wrapper_defs.h"

struct THyALNotifications;

enum EHyALMediaTypeRequest {
	HYAL_MEDIA_TYPE_REQ_PLC_BACKUP_RF = 0x00,
	HYAL_MEDIA_TYPE_REQ_RF_BACKUP_PLC = 0x01,
	HYAL_MEDIA_TYPE_REQ_BOTH = 0x02,
	HYAL_MEDIA_TYPE_REQ_PLC_NO_BACKUP = 0x03,
	HYAL_MEDIA_TYPE_REQ_RF_NO_BACKUP = 0x04,
};

enum EHyALMediaTypeConfirm {
	HYAL_MEDIA_TYPE_CONF_PLC = 0x00,
	HYAL_MEDIA_TYPE_CONF_RF = 0x01,
	HYAL_MEDIA_TYPE_CONF_BOTH = 0x02,
	HYAL_MEDIA_TYPE_CONF_PLC_AS_BACKUP = 0x03,
	HYAL_MEDIA_TYPE_CONF_RF_AS_BACKUP = 0x04,
};

enum EHyALMediaTypeIndication {
	HYAL_MEDIA_TYPE_IND_PLC = 0x00,
	HYAL_MEDIA_TYPE_IND_RF = 0x01,
};

struct THyALPanDescriptor {
	TMacWrpPanId m_nPanId;
	uint8_t m_u8LinkQuality;
	TMacWrpShortAddress m_nLbaAddress;
	uint16_t m_u16RcCoord;
    enum EHyALMediaTypeIndication m_eMediaType;
};

/**********************************************************************************************************************/
/** Description of struct THyALDataRequest
 ***********************************************************************************************************************
 * @param m_eSrcAddrMode Source address mode 0, 16, 64 bits
 * @param m_nDstPanId The 16-bit PAN identifier of the entity to which the MSDU is being transferred
 * @param m_DstAddr The individual device address of the entity to which the MSDU is being transferred.
 * @param m_u16MsduLength The number of octets contained in the MSDU to be transmitted by the MAC sublayer entity.
 * @param m_pMsdu The set of octets forming the MSDU to be transmitted by the MAC sublayer entity
 * @param m_u8MsduHandle The handle associated with the MSDU to be transmitted by the MAC sublayer entity.
 * @param m_u8TxOptions Indicate the transmission options for this MSDU:
 *    0: unacknowledged transmission, 1: acknowledged transmission
 * @param m_eSecurityLevel The security level to be used: 0x00 unecrypted; 0x05 encrypted
 * @param m_u8KeyIndex The index of the key to be used.
 * @param m_eQualityOfService The QOS (quality of service) parameter of the MSDU to be transmitted by the MAC sublayer
 *      entity
 *        0x00 = normal priority
 *        0x01 = high priority
 * @param m_eMediaType The Media Type to use on Request.
 **********************************************************************************************************************/
struct THyALDataRequest {
	enum EMacWrpAddressMode m_eSrcAddrMode;
	TMacWrpPanId m_nDstPanId;
	struct TMacWrpAddress m_DstAddr;
	uint16_t m_u16MsduLength;
	const uint8_t *m_pMsdu;
	uint8_t m_u8MsduHandle;
	uint8_t m_u8TxOptions;
	enum EMacWrpSecurityLevel m_eSecurityLevel;
	uint8_t m_u8KeyIndex;
	enum EMacWrpQualityOfService m_eQualityOfService;
    enum EHyALMediaTypeRequest m_eMediaType;
};

/**********************************************************************************************************************/
/** Description of struct THyALDataConfirm
 ***********************************************************************************************************************
 * @param m_u8MsduHandle The handle associated with the MSDU being confirmed.
 * @param m_eStatus The status of the last MSDU transmission.
 * @param m_nTimestamp The time, in symbols, at which the data were transmitted.
 * @param m_eMediaType The Media Type used on last MSDU transmission.
 **********************************************************************************************************************/
struct THyALDataConfirm {
	uint8_t m_u8MsduHandle;
	enum EMacWrpStatus m_eStatus;
	TMacWrpTimestamp m_nTimestamp;
    enum EHyALMediaTypeConfirm m_eMediaType;
};

/**********************************************************************************************************************/
/** Description of struct THyALDataIndication
 ***********************************************************************************************************************
 * @param m_nSrcPanId The 16-bit PAN identifier of the device from which the frame was received.
 * @param m_SrcAddr The address of the device which sent the message.
 * @param m_nDstPanId The 16-bit PAN identifier of the entity to which the MSDU is being transferred.
 * @param m_DstAddr The individual device address of the entity to which the MSDU is being transferred.
 * @param m_u16MsduLength The number of octets contained in the MSDU to be indicated to the upper layer.
 * @param m_pMsdu The set of octets forming the MSDU received by the MAC sublayer entity.
 * @param m_u8MpduLinkQuality The (forward) LQI value measured during reception of the message.
 * @param m_u8Dsn The DSN of the received frame
 * @param m_nTimestamp The absolute time in milliseconds at which the frame was received and constructed, decrypted
 * @param m_eSecurityLevel The security level of the received message: 0x00 unecrypted; 0x05 encrypted
 * @param m_u8KeyIndex The index of the key used.
 * @param u8QualityOfService The QOS (quality of service) parameter of the MSDU received by the MAC sublayer entity
 *        0x00 = normal priority
 *        0x01 = high priority
 * @param m_u8RecvModulation Modulation of the received message.
 * @param m_u8RecvModulationScheme Modulation of the received message.
 * @param m_RecvToneMap The ToneMap of the received message.
 * @param m_u8ComputedModulation Computed modulation of the received message.
 * @param m_u8ComputedModulationScheme Computed modulation of the received message.
 * @param m_ComputedToneMap Compute ToneMap of the received message.
 * @param m_eMediaType The Media Type on which Data was received.
 **********************************************************************************************************************/
struct THyALDataIndication {
	TMacWrpPanId m_nSrcPanId;
	struct TMacWrpAddress m_SrcAddr;
	TMacWrpPanId m_nDstPanId;
	struct TMacWrpAddress m_DstAddr;
	uint16_t m_u16MsduLength;
	uint8_t *m_pMsdu;
	uint8_t m_u8MpduLinkQuality;
	uint8_t m_u8Dsn;
	TMacWrpTimestamp m_nTimestamp;
	enum EMacWrpSecurityLevel m_eSecurityLevel;
	uint8_t m_u8KeyIndex;
	enum EMacWrpQualityOfService m_eQualityOfService;
	uint8_t m_u8RecvModulation;
	uint8_t m_u8RecvModulationScheme;
	struct TMacWrpToneMap m_RecvToneMap;
	uint8_t m_u8ComputedModulation;
	uint8_t m_u8ComputedModulationScheme;
	struct TMacWrpToneMap m_ComputedToneMap;
    enum EHyALMediaTypeIndication m_eMediaType;
};

/**********************************************************************************************************************/
/** Description of struct THyALSnifferIndication
 ***********************************************************************************************************************
 * @param m_u8FrameType The frame type.
 * @param m_nSrcPanId The 16-bit PAN identifier of the device from which the frame was received.
 * @param m_nSrcPanId The 16-bit PAN identifier of the device from which the frame was received.
 * @param m_SrcAddr The address of the device which sent the message.
 * @param m_nDstPanId The 16-bit PAN identifier of the entity to which the MSDU is being transferred.
 * @param m_DstAddr The individual device address of the entity to which the MSDU is being transferred.
 * @param m_u16MsduLength The number of octets contained in the MSDU to be indicated to the upper layer.
 * @param m_pMsdu The set of octets forming the MSDU received by the MAC sublayer entity.
 * @param m_u8MpduLinkQuality The (forward) LQI value measured during reception of the message.
 * @param m_u8Dsn The DSN of the received frame
 * @param m_nTimestamp The absolute time in milliseconds at which the frame was received and constructed, decrypted
 * @param m_eSecurityLevel The security level of the received message: 0x00 unecrypted; 0x05 encrypted
 * @param m_u8KeyIndex The index of the key used.
 * @param u8QualityOfService The QOS (quality of service) parameter of the MSDU received by the MAC sublayer entity
 *        0x00 = normal priority
 *        0x01 = high priority
 * @param m_u8RecvModulation Modulation of the received message.
 * @param m_u8RecvModulationScheme Modulation of the received message.
 * @param m_RecvToneMap The ToneMap of the received message.
 * @param m_u8ComputedModulation Computed modulation of the received message.
 * @param m_u8ComputedModulationScheme Computed modulation of the received message.
 * @param m_ComputedToneMap Compute ToneMap of the received message.
 **********************************************************************************************************************/
struct THyALSnifferIndication {
	uint8_t m_u8FrameType;
	TMacWrpPanId m_nSrcPanId;
	struct TMacWrpAddress m_SrcAddr;
	TMacWrpPanId m_nDstPanId;
	struct TMacWrpAddress m_DstAddr;
	uint16_t m_u16MsduLength;
	uint8_t *m_pMsdu;
	uint8_t m_u8MpduLinkQuality;
	uint8_t m_u8Dsn;
	TMacWrpTimestamp m_nTimestamp;
	enum EMacWrpSecurityLevel m_eSecurityLevel;
	uint8_t m_u8KeyIndex;
	enum EMacWrpQualityOfService m_eQualityOfService;
	uint8_t m_u8RecvModulation;
	uint8_t m_u8RecvModulationScheme;
	struct TMacWrpToneMap m_RecvToneMap;
	uint8_t m_u8ComputedModulation;
	uint8_t m_u8ComputedModulationScheme;
	struct TMacWrpToneMap m_ComputedToneMap;
};

/**********************************************************************************************************************/
/** Description of struct THyALBeaconNotifyIndication
 ***********************************************************************************************************************
 * @param m_PanDescriptor The PAN descriptor
 **********************************************************************************************************************/
struct THyALBeaconNotifyIndication {
	struct THyALPanDescriptor m_PanDescriptor;
};

/**********************************************************************************************************************/
/** Description of struct THyALGetRequest
 ***********************************************************************************************************************
 * @param m_ePibAttribute The attribute id.
 * @param m_u16PibAttributeIndex The index of the element in the table.
 **********************************************************************************************************************/
struct THyALGetRequest {
	enum EMacWrpPibAttribute m_ePibAttribute;
	uint16_t m_u16PibAttributeIndex;
};

/**********************************************************************************************************************/
/** Description of struct THyALGetConfirm
 ***********************************************************************************************************************
 * @param m_eStatus The status of the request.
 * @param m_ePibAttribute The attribute id.
 * @param m_u16PibAttributeIndex The index of the element in the table.
 * @param m_PibAttributeValue The value of the attribute.
 **********************************************************************************************************************/
struct THyALGetConfirm {
	enum EMacWrpStatus m_eStatus;
	enum EMacWrpPibAttribute m_ePibAttribute;
	uint16_t m_u16PibAttributeIndex;
	struct TMacWrpPibValue m_PibAttributeValue;
};

/**********************************************************************************************************************/
/** Description of struct THyALResetRequest
 ***********************************************************************************************************************
 * @param m_bSetDefaultPib True to reset the PIB to the default values, false otherwise
 **********************************************************************************************************************/
struct THyALResetRequest {
	bool m_bSetDefaultPib;
};

/**********************************************************************************************************************/
/** Description of struct THyALResetConfirm
 ***********************************************************************************************************************
 * @param m_eStatus The status of the request.
 **********************************************************************************************************************/
struct THyALResetConfirm {
	enum EMacWrpStatus m_eStatus;
};

/**********************************************************************************************************************/
/** Description of struct THyALScanRequest
 ***********************************************************************************************************************
 * @param m_u16ScanDuration Duration of the scan in seconds
 **********************************************************************************************************************/
struct THyALScanRequest {
	uint16_t m_u16ScanDuration;
};

/**********************************************************************************************************************/
/** Description of struct THyALScanConfirm
 ***********************************************************************************************************************
 * @param m_eStatus The status of the request.
 **********************************************************************************************************************/
struct THyALScanConfirm {
	enum EMacWrpStatus m_eStatus;
};

/**********************************************************************************************************************/
/** Description of struct THyALCommStatusIndication
 ***********************************************************************************************************************
 * @param m_nPanId The PAN identifier of the device from which the frame was received or to which the frame was being
 *    sent.
 * @param m_SrcAddr The individual device address of the entity from which the frame causing the error originated.
 * @param m_DstAddr The individual device address of the device for which the frame was intended.
 * @param m_eStatus The communications status.
 * @param m_eSecurityLevel The security level purportedly used by the received frame.
 * @param m_u8KeyIndex The index of the key purportedly used by the originator of the received frame.
 * @param m_eMediaType The Media Type on which indication was received.
 **********************************************************************************************************************/
struct THyALCommStatusIndication {
	TMacWrpPanId m_nPanId;
	struct TMacWrpAddress m_SrcAddr;
	struct TMacWrpAddress m_DstAddr;
	enum EMacWrpStatus m_eStatus;
	enum EMacWrpSecurityLevel m_eSecurityLevel;
	uint8_t m_u8KeyIndex;
    enum EHyALMediaTypeIndication m_eMediaType;
};

/**********************************************************************************************************************/
/** Description of struct THyALGetRequest
 ***********************************************************************************************************************
 * @param m_ePibAttribute The attribute id.
 * @param m_u16PibAttributeIndex The index of the element in the table.
 * @param m_PibAttributeValue The value of the attribute.
 **********************************************************************************************************************/
struct THyALSetRequest {
	enum EMacWrpPibAttribute m_ePibAttribute;
	uint16_t m_u16PibAttributeIndex;
	struct TMacWrpPibValue m_PibAttributeValue;
};

/**********************************************************************************************************************/
/** Description of struct THyALSetConfirm
 ***********************************************************************************************************************
 * @param m_eStatus The status of the request.
 * @param m_ePibAttribute The attribute id.
 * @param m_u16PibAttributeIndex The index of the element in the table.
 **********************************************************************************************************************/
struct THyALSetConfirm {
	enum EMacWrpStatus m_eStatus;
	enum EMacWrpPibAttribute m_ePibAttribute;
	uint16_t m_u16PibAttributeIndex;
};

/**********************************************************************************************************************/
/** Description of struct THyALStartRequest
 ***********************************************************************************************************************
 * @param m_nPanId The pan id.
 **********************************************************************************************************************/
struct THyALStartRequest {
	TMacWrpPanId m_nPanId;
};

/**********************************************************************************************************************/
/** Description of struct THyALStartConfirm
 ***********************************************************************************************************************
 * @param m_eStatus The status of the request.
 **********************************************************************************************************************/
struct THyALStartConfirm {
	enum EMacWrpStatus m_eStatus;
};

/**********************************************************************************************************************/
/** ADP use this function to initialize both PLC and RF MAC layers.
 * The MAC layers should be initialized before doing any other operation.
 * @param pNotifications Structure with callbacks used by the MAC layer to notify the upper layer about specific events
 * @param band Working band (should be inline with the hardware)
 **********************************************************************************************************************/
void HyALInitialize(struct THyALNotifications *pNotifications, uint8_t u8Band);

/**********************************************************************************************************************/
/** This function must be called periodically in order to allow the G3 stack to run and execute its internal tasks.
 **********************************************************************************************************************/
void HyALEventHandler(void);

/**********************************************************************************************************************/
/** The HyALDataRequest primitive requests the transfer of an application PDU to another device or multiple devices.
 * Parameters from struct THyALDataRequest
 ***********************************************************************************************************************
 * @param pParameters Request parameters
 **********************************************************************************************************************/
void HyALDataRequest(struct THyALDataRequest *pParameters);

/**********************************************************************************************************************/
/** The HyALGetRequest primitive allows the upper layer to get the value of an attribute from the MAC information base.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the request
 **********************************************************************************************************************/
void HyALGetRequest(struct THyALGetRequest *pParameters);

/**********************************************************************************************************************/
/** The HyALGetRequestSync primitive allows the upper layer to get the value of an attribute from the MAC information
 * base in synchronous way.
 ***********************************************************************************************************************
 * @param eAttribute IB identifier
 * @param u16Index IB index
 * @param pValue pointer to results
 **********************************************************************************************************************/
enum EMacWrpStatus HyALGetRequestSync(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, struct TMacWrpPibValue *pValue);

/**********************************************************************************************************************/
/** The HyALSetRequest primitive allows the upper layer to set the value of an attribute in the MAC information base.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the request
 **********************************************************************************************************************/
void HyALSetRequest(struct THyALSetRequest *pParameters);

/**********************************************************************************************************************/
/** The HyALSetRequestSync primitive allows the upper layer to set the value of an attribute in the MAC information
 * base in synchronous way.
 ***********************************************************************************************************************
 * @param eAttribute IB identifier
 * @param u16Index IB index
 * @param pValue pointer to IB new values
 **********************************************************************************************************************/
enum EMacWrpStatus HyALSetRequestSync(enum EMacWrpPibAttribute eAttribute, uint16_t u16Index, const struct TMacWrpPibValue *pValue);

/**********************************************************************************************************************/
/** The HyALResetRequest primitive performs a reset of the mac sublayer and allows the resetting of the MIB
 * attributes.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the request
 **********************************************************************************************************************/
void HyALResetRequest(struct THyALResetRequest *pParameters);

/**********************************************************************************************************************/
/** The HyALScanRequest primitive allows the upper layer to scan for networks operating in its POS.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the request.
 **********************************************************************************************************************/
void HyALScanRequest(struct THyALScanRequest *pParameters);

/**********************************************************************************************************************/
/** The HyALStartRequest primitive allows the upper layer to request the starting of a new network.
 ***********************************************************************************************************************
 * @param pParameters The parameters of the request
 **********************************************************************************************************************/
void HyALStartRequest(struct THyALStartRequest *pParameters);

/**********************************************************************************************************************/
/** The HyALGetNeighbourTableSize primitive gets the size of Mac Neighbour Table.
 **********************************************************************************************************************/
uint16_t HyALGetNeighbourTableSize(void);

/**********************************************************************************************************************/
/** The HyALDataConfirm primitive allows the upper layer to be notified of the completion of a HyALDataRequest.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the confirm
 **********************************************************************************************************************/
typedef void (*HyALDataConfirm)(struct THyALDataConfirm *pParameters);

/**********************************************************************************************************************/
/** The HyALDataIndication primitive is used to transfer received data from the mac sublayer to the upper layer.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the data indication.
 **********************************************************************************************************************/
typedef void (*HyALDataIndication)(struct THyALDataIndication *pParameters);

/**********************************************************************************************************************/
/** The HyALSnifferIndication primitive is used to transfer received data from the mac sublayer to the upper layer.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the data indication.
 **********************************************************************************************************************/
typedef void (*HyALSnifferIndication)(struct THyALSnifferIndication *pParameters);

/**********************************************************************************************************************/
/** The HyALGetConfirm primitive allows the upper layer to be notified of the completion of a HyALGetRequest.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the confirm.
 **********************************************************************************************************************/
typedef void (*HyALGetConfirm)(struct THyALGetConfirm *pParameters);

/**********************************************************************************************************************/
/** The HyALSetConfirm primitive allows the upper layer to be notified of the completion of a HyALSetRequest.
 ***********************************************************************************************************************
 * @param pParameters Parameters of the confirm.
 **********************************************************************************************************************/
typedef void (*HyALSetConfirm)(struct THyALSetConfirm *pParameters);

/**********************************************************************************************************************/
/** The HyALResetConfirm primitive allows upper layer to be notified of the completion of a HyALResetRequest.
 ***********************************************************************************************************************
 * @param m_u8Status The status of the request.
 **********************************************************************************************************************/
typedef void (*HyALResetConfirm)(struct THyALResetConfirm *pParameters);

/**********************************************************************************************************************/
/** The HyALBeaconNotify primitive is generated by the MAC layer to notify the application about the discovery
 * of a new PAN coordinator or LBA
 ***********************************************************************************************************************
 * @param pPanDescriptor PAN descriptor contains information about the PAN
 **********************************************************************************************************************/
typedef void (*HyALBeaconNotify)(struct THyALBeaconNotifyIndication *pParameters);

/**********************************************************************************************************************/
/** The HyALScanConfirm primitive allows the upper layer to be notified of the completion of a HyALScanRequest.
 ***********************************************************************************************************************
 * @param pParameters The parameters of the confirm.
 **********************************************************************************************************************/
typedef void (*HyALScanConfirm)(struct THyALScanConfirm *pParameters);

/**********************************************************************************************************************/
/** The HyALStartConfirm primitive allows the upper layer to be notified of the completion of a THyALStartRequest.
 ***********************************************************************************************************************
 * @param m_u8Status The status of the request.
 **********************************************************************************************************************/
typedef void (*HyALStartConfirm)(struct THyALStartConfirm *pParameters);

/**********************************************************************************************************************/
/** The HyALCommStatusIndication primitive allows the MAC layer to notify the next higher layer when a particular
 * event occurs on the PAN.
 ***********************************************************************************************************************
 * @param pParameters The parameters of the indication
 **********************************************************************************************************************/
typedef void (*HyALCommStatusIndication)(struct THyALCommStatusIndication *pParameters);

struct THyALNotifications {
	HyALDataConfirm m_HyALDataConfirm;
	HyALDataIndication m_HyALDataIndication;
	HyALGetConfirm m_HyALGetConfirm;
	HyALSetConfirm m_HyALSetConfirm;
	HyALResetConfirm m_HyALResetConfirm;
	HyALBeaconNotify m_HyALBeaconNotifyIndication;
	HyALScanConfirm m_HyALScanConfirm;
	HyALStartConfirm m_HyALStartConfirm;
	HyALCommStatusIndication m_HyALCommStatusIndication;
	HyALSnifferIndication m_HyALSnifferIndication;
};


#endif

/**********************************************************************************************************************/
/** @}
 **********************************************************************************************************************/
