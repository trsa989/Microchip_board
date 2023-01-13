/**********************************************************************************************************************/
/** \addtogroup MacSublayer
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/** This file contains data types and functions of the MAC Real Time RX/TX API.
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/

#ifndef MAC_RT_H_
#define MAC_RT_H_

#include <MacRtDefs.h>

struct TMacRtTxRequest {
  struct TMacRtAddress m_DstAddr;
  uint16_t m_u16MsduLength;
  uint8_t *m_pMsdu;
  uint8_t m_nTxGain;
  uint8_t m_au8TxCoef[6];
  uint8_t m_nTxRes;
  enum ERtModulationType m_eModulationType;
  enum ERtModulationScheme m_eModulationScheme;
  struct TRtToneMap m_ToneMap;
  bool m_bRequestAck;
  bool m_bHighPriority;
  bool m_bToneMapRequest;
  bool m_bForceRobo;
};

struct TMacRtPlmeGetConfirm {
  uint8_t m_u8PpduLinkQuality;
  uint8_t m_u8PhaseDifferential;
  enum ERtModulationType m_eModulationType;
  enum ERtModulationScheme m_eModulationScheme;
  struct TRtToneMap m_ToneMap;
};

struct TMacRtDataIndication {
  uint16_t m_u16PsduLength;
  uint8_t *m_pPsdu;
};

struct TMacRtNotifications;

void MacRtInitialize(uint8_t u8Band, struct TMacRtNotifications *pNotifications, uint8_t u8SpecCompliance);
void MacRtEventHandler(void);

typedef void (*MacRtProcessFrame)(struct TMacRtFrame *pFrame, struct TMacRtDataIndication *pParameters);
typedef void (*MacRtTxConfirm)(enum EMacRtStatus eStatus, bool bUpdateTimestamp, enum ERtModulationType eModType);
typedef void (*MacRtPlmeGetConfirm)(struct TMacRtPlmeGetConfirm *pParameters);

void MacRtTxRequest(struct TMacRtTxRequest *pTxRequest, struct TMacRtMhr *pMhr);
void MacRtResetRequest(bool bResetMib);
void MacRtGetToneMapResponseData(struct TRtToneMapResponseData *pParameters);
uint32_t MacRtGetPhyTime(void);

struct TMacRtNotifications {
  MacRtProcessFrame m_pProcessFrame;
  MacRtTxConfirm m_pMacRtTxConfirm;
  MacRtPlmeGetConfirm m_pMacRtPlmeGetConfirm;
};

#endif

/**********************************************************************************************************************/
/** @}
 **********************************************************************************************************************/
