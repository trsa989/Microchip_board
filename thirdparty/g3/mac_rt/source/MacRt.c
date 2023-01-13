#include <stdbool.h>
#include <stdint.h>
#include <hal/hal.h>
#include <MacPhyInterface.h>
#include <MacRt.h>
#include <MacRtDefs.h>
#include <MacRtConstants.h>
#include <MacRtMib.h>
#include <string.h>
#include <stdio.h>

// DSY - Disable logging temporarily
#define LOG_DBG(expression)
#define LOG_INFO(expression)

extern struct TMacRtMib g_MacRtMib;
extern uint8_t u8RtMibSpecCompliance;

#define MAC_MINIMAL_FRAME_SIZE (8)
#define MAC_PHY_MAX_PAYLOAD_LENGTH (512)

#define MAC_RT_MINIMAL_TX_DELAY 2500

enum EMacRtState {
  MAC_RT_IDLE,
  MAC_RT_EIFS,
  MAC_RT_CIFS,
  MAC_RT_CIFS_RETRANSMIT,
  MAC_RT_RIFS, // This includes RIFS, ACK and CIFS time.
  MAC_RT_CP
};

struct TMacRtData {
  // MAC reception / transmission state.
  enum EMacRtState m_eState;
  uint32_t m_u32ChangeTime;
  // Transmission request data.
  bool m_bTxRequest; // Transmission requested.
  struct TMacRtTxRequest m_TxRequest; // TX data.
  // Transmission data.
  enum EMacRtTxState m_eTxState;
  struct TMacRtFrame m_TxFrame;
  uint8_t m_u8TxSegment;
  uint16_t m_u16SegmentLength;
  uint16_t m_u16TxOffset;
  uint16_t m_u16Nb;
  uint32_t m_u32Nbf;
  uint32_t m_u32Cw;
  uint16_t m_u16MinCwCount;
  uint16_t m_u16TxFcs;
  struct TPlmeSetRequest m_TxParameters;
  // Transmission segment data.
  uint8_t m_u8TxRetries;
  uint16_t m_u16TxPhyDataLength;
  uint8_t m_au8TxPhyData[MAC_PHY_MAX_PAYLOAD_LENGTH];
  /// Transmission delay, from the beginning of the correct contention period.
  uint32_t m_u32Backoff;
  /// Length of the interval between the start of CP and start of the correct contention period.
  uint32_t m_u32PeriodWait;
  // Update the set parameters?
  bool m_bExecutePhySetRequest;
  bool m_bLastMsgCsmaFail;
};

static const struct TMacRtData g_MacRtDefaults = {
  MAC_RT_EIFS, // m_eState
  0, // m_u32ChangeTime, set it to the current time during initialization
  false, // m_bTxRequest
  {{{MAC_RT_ADDRESS_MODE_NO_ADDRESS}}}, // m_TxRequest
  MAC_RT_TX_START, // m_eTxState
  {{{{0}}}}, // m_TxFrame
  0, // m_u8TxSegment
  0, // m_u16SegmentLength
  0, // m_u16TxOffset
  0, // m_u16Nb
  0, // m_u32Nbf
  0, // m_u32Cw
  0, // m_u16MinCwCount
  0, // m_u16TxFcs
  {{{0}}}, // m_TxParameters
  0, // m_u8TxRetries
  0, // m_u16TxPhyDataLength
  {{0}}, // m_au8TxPhyData
  0, // m_u32Backoff
  0, // m_u32PeriodWait
  false, // m_bExecutePhySetRequest
  false, // m_bLastMsgCsmaFail
};

// MAC RT callbacks
struct TMacRtNotifications g_mac_rt_notifications = {0};

// MAC RT Tx buffer
static uint8_t *pTxData;

static struct TMacRtData g_MacRt;

static const uint16_t g_Crc16[256] = {
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

static uint8_t u8AvailableRSBlocks;
static uint8_t au8Gain[PHY_MAX_TONE_GROUPS];

static uint16_t Crc16Ccitt(const uint8_t *pu8Data, uint32_t u32Length, uint16_t u16Crc)
{
  // polynom(16): X16 + X12 + X5 + 1 = 0x1021
  while (u32Length--) {
    u16Crc = g_Crc16[(u16Crc >> 8) ^ (*pu8Data ++)] ^ ((u16Crc & 0xFF) << 8);
  }
  return u16Crc;
}

/*
 * static bool CheckSegmentControl(struct TMacRtSegmentControl segmentControl)
 * {
 * // Nothing to check here
 * UNUSED(segmentControl);
 * return true;
 * }
 */

static bool CheckFc(struct TMacRtFc fc)
{
  return (fc.m_nFrameType < 0x04) && (fc.m_nFramePending == 0) &&
         (fc.m_nDestAddressingMode != 0x01) && (fc.m_nSrcAddressingMode != 0x01);
}

static bool DecodeFrame(struct TMacRtFrame *pFrame, struct TPdDataIndication *pParameters)
{
  // We have received a frame, decode it.
  uint16_t u16PsduLength = pParameters->m_u16PsduLength;
  uint8_t *pPsdu = pParameters->m_pPsdu;
  bool bFrameOk = u16PsduLength >= MAC_MINIMAL_FRAME_SIZE;
  bool bFcsOk = false;
  uint16_t u16ExpectedFcs = 0;
  uint32_t u32SegmentControl = 0;

  memset(pFrame, 0, sizeof(struct TMacRtFrame));
  if (bFrameOk) {
    // Decode and check FCS.
    pFrame->m_u16Fcs = pPsdu[u16PsduLength - 2] | (pPsdu[u16PsduLength - 1] << 8);
    u16ExpectedFcs = Crc16Ccitt(pPsdu, u16PsduLength - 2, 0);
    bFcsOk = (u16ExpectedFcs == pFrame->m_u16Fcs);
    // Decode SegmentControl field.
    u32SegmentControl = (pPsdu[0] << 16) | (pPsdu[1] << 8) | pPsdu[2];
    pFrame->m_Header.m_SegmentControl.m_nSl = u32SegmentControl & 0x3FF;
    u32SegmentControl >>= 10;
    pFrame->m_Header.m_SegmentControl.m_nSc = u32SegmentControl & 0x3F;
    u32SegmentControl >>= 6;
    pFrame->m_Header.m_SegmentControl.m_nLsf = u32SegmentControl & 0x01;
    u32SegmentControl >>= 1;
    pFrame->m_Header.m_SegmentControl.m_nCap = u32SegmentControl & 0x01;
    u32SegmentControl >>= 1;
    pFrame->m_Header.m_SegmentControl.m_nCc = u32SegmentControl & 0x01;
    u32SegmentControl >>= 1;
    pFrame->m_Header.m_SegmentControl.m_nTmr = u32SegmentControl & 0x01;
    u32SegmentControl >>= 1;
    pFrame->m_Header.m_SegmentControl.m_nRes = u32SegmentControl & 0x0F;
    // Check the SegmentControl field.
    /* bFrameOk = CheckSegmentControl(pFrame->m_Header.m_SegmentControl); */
    LOG_DBG(LogBuffer(pPsdu, 3, "Decode SegmentControl. Ok: %u; Res: %u; Tmr: %u; Cc: %u; Cap: %u; Lsf: %u; Sc: %u; Sl: %u; Encoded: ",
      bFrameOk, pFrame->m_Header.m_SegmentControl.m_nRes, pFrame->m_Header.m_SegmentControl.m_nTmr,
      pFrame->m_Header.m_SegmentControl.m_nCc, pFrame->m_Header.m_SegmentControl.m_nCap,
      pFrame->m_Header.m_SegmentControl.m_nLsf, pFrame->m_Header.m_SegmentControl.m_nSc,
      pFrame->m_Header.m_SegmentControl.m_nSl));
    pPsdu += 3;
  }
  if (bFrameOk) {
    // Decode FC field.
    uint16_t u16Fc = pPsdu[0] | (pPsdu[1] << 8);
    pFrame->m_Header.m_Fc.m_nFrameType = u16Fc & 0x07;
    u16Fc >>= 3;
    pFrame->m_Header.m_Fc.m_nSecurityEnabled = u16Fc & 0x01;
    u16Fc >>= 1;
    pFrame->m_Header.m_Fc.m_nFramePending = u16Fc & 0x01;
    u16Fc >>= 1;
    pFrame->m_Header.m_Fc.m_nAckRequest = u16Fc & 0x01;
    u16Fc >>= 1;
    pFrame->m_Header.m_Fc.m_nPanIdCompression = u16Fc & 0x01;
    u16Fc >>= 1;
    pFrame->m_Header.m_Fc.m_nReserved = u16Fc & 0x07;
    u16Fc >>= 3;
    pFrame->m_Header.m_Fc.m_nDestAddressingMode = u16Fc & 0x03;
    u16Fc >>= 2;
    pFrame->m_Header.m_Fc.m_nFrameVersion = u16Fc & 0x03;
    u16Fc >>= 2;
    pFrame->m_Header.m_Fc.m_nSrcAddressingMode = u16Fc & 0x03;
    // Check the FC field.
    bFrameOk = CheckFc(pFrame->m_Header.m_Fc);
    LOG_DBG(LogBuffer(pPsdu, 2,
      "Decode FC. Ok: %u; FrameType: %u; SecurityEnabled: %u; FramePending: %u; AckRequest: %u; PanIdCompression: %u; "
      "Reserved: %u; DestAddressingMode: %u; FrameVersion: %u; SrcAddressingMode: %u; Encoded: ",
      bFrameOk, pFrame->m_Header.m_Fc.m_nFrameType, pFrame->m_Header.m_Fc.m_nSecurityEnabled,
      pFrame->m_Header.m_Fc.m_nFramePending, pFrame->m_Header.m_Fc.m_nAckRequest,
      pFrame->m_Header.m_Fc.m_nPanIdCompression, pFrame->m_Header.m_Fc.m_nReserved,
      pFrame->m_Header.m_Fc.m_nDestAddressingMode, pFrame->m_Header.m_Fc.m_nFrameVersion,
      pFrame->m_Header.m_Fc.m_nSrcAddressingMode));
    pPsdu += 2;
  }
  if (bFrameOk) {
    // Decode SequenceNumber field.
    pFrame->m_Header.m_u8SequenceNumber = *pPsdu;
    pPsdu ++;
  }
  if (bFrameOk) {
    // Check frame length.
    uint16_t u16FrameLength = 3 + 2 + 1 + 2;
    switch (pFrame->m_Header.m_Fc.m_nDestAddressingMode) {
      case 0x02:
        u16FrameLength += 4;
        break;
      case 0x03:
        u16FrameLength += 10;
        break;
      default:
        break;
    }
    switch (pFrame->m_Header.m_Fc.m_nSrcAddressingMode) {
      case 0x02:
        u16FrameLength += 4;
        break;
      case 0x03:
        u16FrameLength += 10;
        break;
      default:
        break;
    }
    if (pFrame->m_Header.m_Fc.m_nPanIdCompression != 0) {
      if ((pFrame->m_Header.m_Fc.m_nDestAddressingMode != 0) && (pFrame->m_Header.m_Fc.m_nSrcAddressingMode != 0)) {
        u16FrameLength -= 2;
      }
      else {
        bFrameOk = false;
      }
    }
    if ((pFrame->m_Header.m_SegmentControl.m_nSc == 0) && (pFrame->m_Header.m_Fc.m_nSecurityEnabled != 0)) {
      u16FrameLength += 6;
    }
    u16FrameLength += pFrame->m_Header.m_SegmentControl.m_nSl;
    bFrameOk = (u16FrameLength <= u16PsduLength);
    pFrame->m_u8PadLength = u16PsduLength - u16FrameLength;
  }
  if (bFrameOk) {
    // Decode destination PAN and address.
    switch (pFrame->m_Header.m_Fc.m_nDestAddressingMode) {
      case 0x02:
        pFrame->m_Header.m_nDestinationPanIdentifier = pPsdu[0] | (pPsdu[1] << 8);
        pPsdu += 2;
        pFrame->m_Header.m_DestinationAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_SHORT;
        pFrame->m_Header.m_DestinationAddress.m_nShortAddress = pPsdu[0] | (pPsdu[1] << 8);
        pPsdu += 2;
        LOG_DBG(Log("Destination PAN id: %04X; Destination address: %04X;",
        pFrame->m_Header.m_nDestinationPanIdentifier, pFrame->m_Header.m_DestinationAddress.m_nShortAddress));
        break;
      case 0x03:
        pFrame->m_Header.m_nDestinationPanIdentifier = pPsdu[0] | (pPsdu[1] << 8);
        pPsdu += 2;
        pFrame->m_Header.m_DestinationAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_EXTENDED;
        memcpy(pFrame->m_Header.m_DestinationAddress.m_ExtendedAddress.m_au8Address, pPsdu, sizeof(struct TExtAddress));
        pPsdu += sizeof(struct TExtAddress);
        LOG_DBG(LogBuffer(pFrame->m_Header.m_DestinationAddress.m_ExtendedAddress.m_au8Address, sizeof(struct TExtAddress),
        "Destination PAN id: %04X; Destination address:", pFrame->m_Header.m_nDestinationPanIdentifier));
        break;
      default:
        pFrame->m_Header.m_DestinationAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_NO_ADDRESS;
        break;
    }
  }
  if (bFrameOk) {
    // Decode source PAN and address.
    switch (pFrame->m_Header.m_Fc.m_nSrcAddressingMode) {
      case 0x02:
        if (pFrame->m_Header.m_Fc.m_nPanIdCompression == 0) {
          pFrame->m_Header.m_nSourcePanIdentifier = pPsdu[0] | (pPsdu[1] << 8);
          pPsdu += 2;
        }
        else {
          pFrame->m_Header.m_nSourcePanIdentifier = pFrame->m_Header.m_nDestinationPanIdentifier;
        }
        pFrame->m_Header.m_SourceAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_SHORT;
        pFrame->m_Header.m_SourceAddress.m_nShortAddress = pPsdu[0] | (pPsdu[1] << 8);
        pPsdu += 2;
        LOG_DBG(Log("Source PAN id: %04X; Source address: %04X;",
        pFrame->m_Header.m_nSourcePanIdentifier, pFrame->m_Header.m_SourceAddress.m_nShortAddress));
        break;
      case 0x03:
        if (pFrame->m_Header.m_Fc.m_nPanIdCompression == 0) {
          pFrame->m_Header.m_nSourcePanIdentifier = pPsdu[0] | (pPsdu[1] << 8);
          pPsdu += 2;
        }
        else {
          pFrame->m_Header.m_nSourcePanIdentifier = pFrame->m_Header.m_nDestinationPanIdentifier;
        }
        pFrame->m_Header.m_SourceAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_EXTENDED;
        memcpy(pFrame->m_Header.m_SourceAddress.m_ExtendedAddress.m_au8Address, pPsdu, sizeof(struct TExtAddress));
        pPsdu += sizeof(struct TExtAddress);
        LOG_DBG(LogBuffer(pFrame->m_Header.m_SourceAddress.m_ExtendedAddress.m_au8Address, sizeof(struct TExtAddress),
        "Source PAN id: %04X; Source address:", pFrame->m_Header.m_nSourcePanIdentifier));
        break;
      default:
        pFrame->m_Header.m_SourceAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_NO_ADDRESS;
        break;
    }
  }
  if (bFrameOk) {
    // Fill the PAN id for the missing address.
    if (pFrame->m_Header.m_Fc.m_nDestAddressingMode == MAC_RT_ADDRESS_MODE_NO_ADDRESS) {
      if (pFrame->m_Header.m_Fc.m_nSrcAddressingMode == MAC_RT_ADDRESS_MODE_NO_ADDRESS) {
        bFrameOk = false;
      }
      else {
        pFrame->m_Header.m_nDestinationPanIdentifier = pFrame->m_Header.m_nSourcePanIdentifier;
      }
    }
    else if (pFrame->m_Header.m_Fc.m_nSrcAddressingMode == MAC_RT_ADDRESS_MODE_NO_ADDRESS) {
      pFrame->m_Header.m_nSourcePanIdentifier = pFrame->m_Header.m_nDestinationPanIdentifier;
    }
  }
  if (bFrameOk) {
    // Decode auxiliary security header.
    if ((pFrame->m_Header.m_Fc.m_nSecurityEnabled != 0) && (pFrame->m_Header.m_SegmentControl.m_nSc == 0)) {
      // Decode Security Control.
      uint8_t u8SecurityControl = pPsdu[0];
      pFrame->m_Header.m_SecurityHeader.m_nSecurityLevel = u8SecurityControl & 0x07;
      u8SecurityControl >>= 3;
      pFrame->m_Header.m_SecurityHeader.m_nKeyIdentifierMode = u8SecurityControl & 0x03;
      u8SecurityControl >>= 2;
      pFrame->m_Header.m_SecurityHeader.m_nReserved = u8SecurityControl;
      pPsdu ++;
      // Decode the frame counter.
      pFrame->m_Header.m_SecurityHeader.m_u32FrameCounter =
        (((((pPsdu[3] << 8) | pPsdu[2]) << 8) | pPsdu[1]) << 8) | pPsdu[0];
      pPsdu += 4;
      // Decode the key identifier.
      pFrame->m_Header.m_SecurityHeader.m_u8KeyIdentifier = pPsdu[0];
      pPsdu ++;
    }
  }
  if (bFrameOk) {
    // Decode payload.
    pFrame->m_u16PayloadLength = (uint8_t) pFrame->m_Header.m_SegmentControl.m_nSl;
    pFrame->m_pu8Payload = pPsdu;
    LOG_DBG(LogBuffer(pFrame->m_pu8Payload, pFrame->m_u16PayloadLength, "Payload: "));
  }
  if (!bFcsOk) {
    g_MacRtMib.m_u32BadCrcCount ++;
    bFrameOk = false;
  }

  if ((pFrame->m_Header.m_SourceAddress.m_eAddrMode == MAC_RT_ADDRESS_MODE_SHORT) && (pFrame->m_Header.m_SourceAddress.m_nShortAddress == MAC_RT_SHORT_ADDRESS_UNDEFINED)) {
    bFrameOk = false;
  }

  if ((!bFrameOk) && bFcsOk) {
    // Segment decode error. Increment counter
    g_MacRtMib.m_u32RxSegmentDecodeErrorCount ++;
  }

  return bFrameOk;
}

static bool CheckDestinationAck(struct TMacRtFrame *pFrame)
{
  bool bOk = false;
  uint16_t nPanId = (pFrame->m_Header.m_Fc.m_nDestAddressingMode == 0x00) ?
    pFrame->m_Header.m_nSourcePanIdentifier : pFrame->m_Header.m_nDestinationPanIdentifier;
  if (nPanId == g_MacRtMib.m_nPanId) {
    switch (pFrame->m_Header.m_DestinationAddress.m_eAddrMode) {
      case MAC_RT_ADDRESS_MODE_NO_ADDRESS:
        // No destination address specified, message for the coordinator.
        bOk = g_MacRtMib.m_bCoordinator;
        break;
      case MAC_RT_ADDRESS_MODE_SHORT:
        bOk = pFrame->m_Header.m_DestinationAddress.m_nShortAddress == g_MacRtMib.m_nShortAddress;
        break;
      case MAC_RT_ADDRESS_MODE_EXTENDED:
        bOk = memcmp(&pFrame->m_Header.m_DestinationAddress.m_ExtendedAddress, &g_MacRtMib.m_ExtendedAddress,
        sizeof(struct TExtAddress)) == 0;
        break;
      default:
        // Unknown address scheme, message not for this device.
        break;
    }
  }
  return bOk;
}

static uint8_t * EncodeMhr(struct TMacRtMhr *pHeader, uint8_t *pu8Psdu)
{
  uint8_t *pu8End = pu8Psdu;
  uint32_t u32Field;
  uint8_t u8Control = 0;

  // FC
  u32Field = pHeader->m_Fc.m_nSrcAddressingMode;
  u32Field = (u32Field << 2) | pHeader->m_Fc.m_nFrameVersion;
  u32Field = (u32Field << 2) | pHeader->m_Fc.m_nDestAddressingMode;
  u32Field = (u32Field << 3) | pHeader->m_Fc.m_nReserved;
  u32Field = (u32Field << 1) | pHeader->m_Fc.m_nPanIdCompression;
  u32Field = (u32Field << 1) | pHeader->m_Fc.m_nAckRequest;
  u32Field = (u32Field << 1) | pHeader->m_Fc.m_nFramePending;
  u32Field = (u32Field << 1) | pHeader->m_Fc.m_nSecurityEnabled;
  u32Field = (u32Field << 3) | pHeader->m_Fc.m_nFrameType;
  pu8End[0] = u32Field & 0xFF;
  u32Field >>= 8;
  pu8End[1] = u32Field & 0xFF;
  LOG_DBG(LogBuffer(pu8End, 2,
    "Encode FC. FrameType: %u; SecurityEnabled: %u; FramePending: %u; AckRequest: %u; PanIdCompression: %u; "
    "Reserved: %u; DestAddressingMode: %u; FrameVersion: %u; SrcAddressingMode: %u; Encoded: ",
    pHeader->m_Fc.m_nFrameType, pHeader->m_Fc.m_nSecurityEnabled,
    pHeader->m_Fc.m_nFramePending, pHeader->m_Fc.m_nAckRequest,
    pHeader->m_Fc.m_nPanIdCompression, pHeader->m_Fc.m_nReserved,
    pHeader->m_Fc.m_nDestAddressingMode, pHeader->m_Fc.m_nFrameVersion,
    pHeader->m_Fc.m_nSrcAddressingMode));
  pu8End += 2;
  // Sequence number.
  pu8End[0] = pHeader->m_u8SequenceNumber;
  pu8End ++;
  // Destination PAN.
  if (pHeader->m_DestinationAddress.m_eAddrMode != MAC_RT_ADDRESS_MODE_NO_ADDRESS) {
    LOG_DBG(Log("Destination PAN id: %04X", pHeader->m_nDestinationPanIdentifier));
    u32Field = pHeader->m_nDestinationPanIdentifier;
    pu8End[0] = u32Field & 0xFF;
    u32Field >>= 8;
    pu8End[1] = u32Field & 0xFF;
    pu8End += 2;
  }
  // Destination address.
  switch (pHeader->m_DestinationAddress.m_eAddrMode) {
    case MAC_RT_ADDRESS_MODE_NO_ADDRESS:
      // Nothing to do.
      LOG_DBG(Log("No destination address."));
      break;
    case MAC_RT_ADDRESS_MODE_SHORT:
      // Encode short address.
      LOG_DBG(Log("Destination address: %04X", pHeader->m_DestinationAddress.m_nShortAddress));
      u32Field = pHeader->m_DestinationAddress.m_nShortAddress;
      pu8End[0] = u32Field & 0xFF;
      u32Field >>= 8;
      pu8End[1] = u32Field & 0xFF;
      pu8End += 2;
      break;
    case MAC_RT_ADDRESS_MODE_EXTENDED:
      // Encode extended address.
      memcpy(&pu8End[0], pHeader->m_DestinationAddress.m_ExtendedAddress.m_au8Address, sizeof(struct TExtAddress));
      LOG_DBG(LogBuffer(pu8End, sizeof(struct TExtAddress), "Destination address: "));
      pu8End += sizeof(struct TExtAddress);
      break;
    default:
      // No idea what this might be.
      break;
  }
  // Source PAN.
  if ((pHeader->m_SourceAddress.m_eAddrMode != MAC_RT_ADDRESS_MODE_NO_ADDRESS) &&
    ((pHeader->m_DestinationAddress.m_eAddrMode == MAC_RT_ADDRESS_MODE_NO_ADDRESS) ||
    (pHeader->m_Fc.m_nPanIdCompression == 0))) {
    LOG_DBG(Log("Source PAN id: %04X", pHeader->m_nSourcePanIdentifier));
    u32Field = pHeader->m_nSourcePanIdentifier;
    pu8End[0] = u32Field & 0xFF;
    u32Field >>= 8;
    pu8End[1] = u32Field & 0xFF;
    pu8End += 2;
  }
  // Source address.
  switch (pHeader->m_SourceAddress.m_eAddrMode) {
    case MAC_RT_ADDRESS_MODE_NO_ADDRESS:
      // Nothing to do.
      LOG_DBG(Log("No source address."));
      break;
    case MAC_RT_ADDRESS_MODE_SHORT:
      // Encode short address.
      LOG_DBG(Log("Source address: %04X", pHeader->m_SourceAddress.m_nShortAddress));
      u32Field = pHeader->m_SourceAddress.m_nShortAddress;
      pu8End[0] = u32Field & 0xFF;
      u32Field >>= 8;
      pu8End[1] = u32Field & 0xFF;
      pu8End += 2;
      break;
    case MAC_RT_ADDRESS_MODE_EXTENDED:
      // Encode extended address.
      memcpy(&pu8End[0], pHeader->m_SourceAddress.m_ExtendedAddress.m_au8Address, sizeof(struct TExtAddress));
      LOG_DBG(LogBuffer(pu8End, sizeof(struct TExtAddress), "Source address: "));
      pu8End += sizeof(struct TExtAddress);
      break;
    default:
      // No idea what this might be.
      break;
  }
  if ((pHeader->m_Fc.m_nSecurityEnabled) && (pHeader->m_SegmentControl.m_nSc == 0)) {
    // Encode the security header.
    // Security Control field.
    u8Control = pHeader->m_SecurityHeader.m_nReserved;
    u8Control = (u8Control << 2) | pHeader->m_SecurityHeader.m_nKeyIdentifierMode;
    u8Control = (u8Control << 3) | pHeader->m_SecurityHeader.m_nSecurityLevel;
    pu8End[0] = u8Control;
    pu8End ++;
    // Frame counter.
    u32Field = pHeader->m_SecurityHeader.m_u32FrameCounter;
    pu8End[0] = u32Field & 0xFF;
    u32Field >>= 8;
    pu8End[1] = u32Field & 0xFF;
    u32Field >>= 8;
    pu8End[2] = u32Field & 0xFF;
    u32Field >>= 8;
    pu8End[3] = u32Field & 0xFF;
    pu8End += 4;
    // Key identifier.
    pu8End[0] = pHeader->m_SecurityHeader.m_u8KeyIdentifier;
    pu8End ++;
  }
  return pu8End;
}

static void CalculateTxParameters(void)
{
  uint8_t u8Idx = 0;
  uint16_t u16SumToneMapBytes = 0;
  uint8_t u8TxPower = 0;
  int8_t i8BaseGain = 0;
  uint32_t u32ToneMapByteIndex = 0;
  uint8_t u8ToneMapBits = 0;
  uint8_t u8ToneMapByteValue = 0;
  uint32_t u32Index;
  uint8_t u8Attenuation3db;
  uint8_t u8Coef = 0;
  int8_t i8Gain = 0;
  bool bNegative = false;

  // Fill TX parameters based on this neighbour entry.
  {
    // Calculate total gain on each band.
    u8TxPower = 0x7F;

    if (u8RtMibSpecCompliance >= MAC_RT_SPEC_COMPLIANCE_17) {
      u8Attenuation3db = (g_MacRtMib.m_u8TransmitAtten + 2) / 3;
    }
    else {
      u8Attenuation3db = 0;
    }

    i8BaseGain = g_MacRt.m_TxRequest.m_nTxGain & 0x07;
    if (g_MacRt.m_TxRequest.m_nTxGain & 0x08) {
      i8BaseGain = -i8BaseGain;
    }

    for (u32Index = 0; u32Index < g_PhyBandInformation.m_u8Carriers; u32Index ++) {
      if (u8ToneMapBits == 0) {
        u8ToneMapByteValue = g_MacRt.m_TxRequest.m_au8TxCoef[u32ToneMapByteIndex];
        u32ToneMapByteIndex ++;
        u8ToneMapBits = 8;
      }
      u8Coef = u8ToneMapByteValue;
      bNegative = (u8Coef & 0x80) != 0;
      u8Coef >>= (8 - g_PhyBandInformation.m_u8TxCoefBits);
      if (bNegative) {
        u8Coef |= ~((1 << g_PhyBandInformation.m_u8TxCoefBits) - 1);
      }
      u8ToneMapByteValue <<= g_PhyBandInformation.m_u8TxCoefBits;
      u8ToneMapBits -= g_PhyBandInformation.m_u8TxCoefBits;
      i8Gain = u8Coef;
      i8Gain += i8BaseGain;
      if (g_MacRt.m_TxRequest.m_nTxRes == 0) {
        i8Gain *= 2;
      }
      i8Gain = -i8Gain;
      if (i8Gain < 0) {
        i8Gain = 0;
      }
      au8Gain[u32Index] = i8Gain;
      if (i8Gain < u8TxPower) {
        u8TxPower = i8Gain;
      }
    }
    if (u8TxPower > 0x1F) {
      u8TxPower = 0x1F;
    }
    for (u32Index = 0; u32Index < g_PhyBandInformation.m_u8Carriers; u32Index ++) {
      au8Gain[u32Index] -= u8TxPower;
      if (au8Gain[u32Index] > 0x1F) {
        au8Gain[u32Index] = 0x1F;
      }
    }
    if (u8Attenuation3db != 0) {
      // Can only be different to 0 in case u8RtMibSpecCompliance >= 17
      if (u8Attenuation3db > 0x1f) {
        u8TxPower = 0x1f;
      }
      else {
        u8TxPower = u8Attenuation3db;
      }
      for (u32Index = 0; u32Index < g_PhyBandInformation.m_u8Carriers; u32Index ++) {
        au8Gain[u32Index] = 0;
      }
    }
    // Encode.
    g_MacRt.m_TxParameters.m_TxParameters.m_u8TxPower = u8TxPower;
    memset(g_MacRt.m_TxParameters.m_TxParameters.m_PreEmphasis.m_au8PreEmphasis, 0,
      sizeof(g_MacRt.m_TxParameters.m_TxParameters.m_PreEmphasis.m_au8PreEmphasis));
    for (u32Index = 0; u32Index < g_PhyBandInformation.m_u8Carriers; u32Index ++) {
      g_MacRt.m_TxParameters.m_TxParameters.m_PreEmphasis.m_au8PreEmphasis[u32Index] = au8Gain[u32Index];
    }
  }

  switch (g_MacRtMib.m_u8ForcedModType) {
    case 1:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_ROBUST;
      break;
    case 2:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_DBPSK_BPSK;
      break;
    case 3:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_DQPSK_QPSK;
      break;
    case 4:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_D8PSK_8PSK;
      break;
    case 0:
    default:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = (enum EPhyModulationType)g_MacRt.m_TxRequest.m_eModulationType;
      break;
  }

  switch (g_MacRtMib.m_u8ForcedModScheme) {
    case 1:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationScheme = PHY_MODULATION_SCHEME_DIFFERENTIAL;
      break;
    case 2:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationScheme = PHY_MODULATION_SCHEME_COHERENT;
      break;
    case 0:
    default:
      g_MacRt.m_TxParameters.m_TxParameters.m_eModulationScheme = (enum EPhyModulationScheme)g_MacRt.m_TxRequest.m_eModulationScheme;
      break;
  }

  u16SumToneMapBytes = 0;
  for (u8Idx = 0; u8Idx < sizeof(g_MacRtMib.m_ForcedToneMap); u8Idx++) {
    u16SumToneMapBytes += g_MacRtMib.m_ForcedToneMap.m_au8Tm[u8Idx];
  }
  if (u16SumToneMapBytes == 0) {
    // Use tonemap from neighbour table
    memcpy(&g_MacRt.m_TxParameters.m_TxParameters.m_ToneMap, &g_MacRt.m_TxRequest.m_ToneMap, sizeof(g_MacRt.m_TxParameters.m_TxParameters.m_ToneMap));
  }
  else {
    // Use forced tonemap
    memcpy(&g_MacRt.m_TxParameters.m_TxParameters.m_ToneMap, &g_MacRtMib.m_ForcedToneMap, sizeof(g_MacRt.m_TxParameters.m_TxParameters.m_ToneMap));
  }

  memcpy(&g_MacRt.m_TxParameters.m_TxParameters.m_ToneMask, &g_MacRtMib.m_ToneMask, sizeof(g_MacRt.m_TxParameters.m_TxParameters.m_ToneMask));
  g_MacRt.m_TxParameters.m_u8Dt = (g_MacRt.m_TxRequest.m_bRequestAck) ? 1 : 0;
  g_MacRt.m_TxParameters.m_TxParameters.m_u8TwoRSBlocks = 0;
}

struct TPhyParameters {
  uint8_t m_u8SubCarriers;
  uint8_t m_u8BitsSubcarrier;
  uint8_t m_u8RepCode;
  uint8_t m_u8RsParity;
};

static uint8_t CalculateSubCarriers(struct TPhyToneMap *pToneMap, struct TPhyToneMask *pToneMask, enum EPhyModulationScheme eModulationScheme)
{
  // Calculate the number of carriers. During a symbol data is transmitted on all carriers.
  uint8_t u8Result = 0;
  uint8_t u8Carriers = g_PhyBandInformation.m_u8Carriers;
  uint8_t u8SubCarriersInCarrier = g_PhyBandInformation.m_u8TonesInCarrier;
  uint8_t u8PilotFreqSpacing = g_PhyBandInformation.m_u8PilotsFreqSpa;
  uint8_t u8ToneMapNextByte = 0;
  uint8_t u8ToneMapBitsLeft = 0;
  uint8_t u8ToneMapByteValue = 0;
  uint8_t u8ToneMaskNextByte = 0;
  uint8_t u8ToneMaskBitsLeft = 0;
  uint8_t u8ToneMaskByteValue = 0;
  uint8_t u8NumPilots = 0;
  uint8_t u8Carrier;
  uint8_t u8SubCarrier;
  bool bMapEnabled = false;
  bool bMaskEnabled = false;

  for (u8Carrier = 0; u8Carrier < u8Carriers; u8Carrier ++) {
    if (u8ToneMapBitsLeft == 0) {
      u8ToneMapByteValue = pToneMap->m_au8Tm[u8ToneMapNextByte];
      u8ToneMapNextByte ++;
      u8ToneMapBitsLeft = 8;
    }
    bMapEnabled = (u8ToneMapByteValue & 0x01) != 0;
    u8ToneMapByteValue >>= 1;
    u8ToneMapBitsLeft --;
    for (u8SubCarrier = 0; u8SubCarrier < u8SubCarriersInCarrier; u8SubCarrier ++) {
      if (u8ToneMaskBitsLeft == 0) {
        u8ToneMaskByteValue = pToneMask->m_au8ToneMask[u8ToneMaskNextByte];
        u8ToneMaskNextByte ++;
        u8ToneMaskBitsLeft = 8;
      }
      bMaskEnabled = (u8ToneMaskByteValue & 0x01) != 0;
      u8ToneMaskByteValue >>= 1;
      u8ToneMaskBitsLeft --;
      if (bMapEnabled && bMaskEnabled) {
        u8Result ++;
      }
    }
  }

  if (eModulationScheme == PHY_MODULATION_SCHEME_COHERENT) {
    if (u8PilotFreqSpacing == 0) {
      // Protection, set default Cen-A value
      u8PilotFreqSpacing = 12;
    }
    u8NumPilots = (u8Result + u8PilotFreqSpacing - 1) / u8PilotFreqSpacing;
    u8Result -= u8NumPilots;
  }

  return u8Result;
}

static struct TPhyParameters CalculatePhyParameters(struct TPlmeSetRequest *pTxParameters)
{
  struct TPhyParameters result;
  result.m_u8SubCarriers = CalculateSubCarriers(&pTxParameters->m_TxParameters.m_ToneMap,
    &pTxParameters->m_TxParameters.m_ToneMask, pTxParameters->m_TxParameters.m_eModulationScheme);
  // Calculate the multiplier of the modulation.
  result.m_u8BitsSubcarrier = 0;
  switch (pTxParameters->m_TxParameters.m_eModulationType) {
    case PHY_MODULATION_ROBUST:
      result.m_u8BitsSubcarrier = 1;
      break;
    case PHY_MODULATION_DBPSK_BPSK:
      result.m_u8BitsSubcarrier = 1;
      break;
    case PHY_MODULATION_DQPSK_QPSK:
      result.m_u8BitsSubcarrier = 2;
      break;
    case PHY_MODULATION_D8PSK_8PSK:
      result.m_u8BitsSubcarrier = 3;
      break;
    case PHY_MODULATION_16_QAM:
      result.m_u8BitsSubcarrier = 4;
      break;
    default:
      result.m_u8BitsSubcarrier = 1;
      break;
  }
  // In robust mode we have to divide this by 4, because each bit is repeated 4 times.
  result.m_u8RepCode = (pTxParameters->m_TxParameters.m_eModulationType == PHY_MODULATION_ROBUST) ? 4 : 1;
  // Remove the Reed-Solomon parity bytes.
  result.m_u8RsParity = (pTxParameters->m_TxParameters.m_eModulationType == PHY_MODULATION_ROBUST) ? 8 : 16;
  // Return the calculated values.
  return result;
}

static uint16_t CalculateMaxPhyPayload(struct TPhyParameters *pParameters)
{
  // Find out the maximal number of symbols.
  uint32_t u32Ns = ((255 * 8) + 6) * pParameters->m_u8RepCode * 2; // (MaxRsBlockSize * 8 + CcZeroTail) * RepCode / CcRate
  uint32_t u32Div = g_PhyBandInformation.m_u8FlBand * pParameters->m_u8SubCarriers * pParameters->m_u8BitsSubcarrier;
  uint32_t u32RsBlock = 0;

  if (u32Div == 0) {
    // Protection, set Cen-A default values
    u32Div = 4 * 36;
  }
  u32Ns = (u32Ns + u32Div - 1) / u32Div * g_PhyBandInformation.m_u8FlBand;
  if (u32Ns > (uint32_t)(g_PhyBandInformation.m_u16FlMax >> 1)) { // Only applies to FCC beacuse ARIB/CENELEC u8AvailableRSBlocks will always be 1
    u8AvailableRSBlocks = 1;
  }
  if (u32Ns > (uint32_t)(g_PhyBandInformation.m_u16FlMax * g_PhyBandInformation.m_u8FlBand)) {
    u32Ns = g_PhyBandInformation.m_u16FlMax * g_PhyBandInformation.m_u8FlBand;
  }
  // Calculate the maximal RS block size, adjust if necessary.
  u32RsBlock = ((u32Ns * pParameters->m_u8SubCarriers * pParameters->m_u8BitsSubcarrier) - (6 * 2 * pParameters->m_u8RepCode)) /
    (8 * 2 * pParameters->m_u8RepCode);
  if (u32RsBlock > 255) {
    u32Ns -= g_PhyBandInformation.m_u8FlBand;
    u32RsBlock = ((u32Ns * pParameters->m_u8SubCarriers * pParameters->m_u8BitsSubcarrier) - (6 * 2 * pParameters->m_u8RepCode)) /
      (8 * 2 * pParameters->m_u8RepCode);
  }
  // Return the maximal payload, which is the RS size minus the RS parity bytes.
  return u32RsBlock - pParameters->m_u8RsParity;
}

static uint16_t EncodeFrame(struct TMacRtFrame *pFrame, uint8_t *pu8Psdu)
{
  uint8_t *pu8End = pu8Psdu;
  // Encode the header.
  // Segment control.
  uint32_t u32Field = pFrame->m_Header.m_SegmentControl.m_nRes;
  u32Field = (u32Field << 1) | pFrame->m_Header.m_SegmentControl.m_nTmr;
  u32Field = (u32Field << 1) | pFrame->m_Header.m_SegmentControl.m_nCc;
  u32Field = (u32Field << 1) | pFrame->m_Header.m_SegmentControl.m_nCap;
  u32Field = (u32Field << 1) | pFrame->m_Header.m_SegmentControl.m_nLsf;
  u32Field = (u32Field << 6) | pFrame->m_Header.m_SegmentControl.m_nSc;
  u32Field = (u32Field << 10) | pFrame->m_Header.m_SegmentControl.m_nSl;
  pu8End[2] = u32Field & 0xFF;
  u32Field >>= 8;
  pu8End[1] = u32Field & 0xFF;
  u32Field >>= 8;
  pu8End[0] = u32Field & 0xFF;
  LOG_DBG(LogBuffer(pu8End, 3, "Encode SegmentControl. Res: %u; Tmr: %u; Cc: %u; Cap: %u; Lsf: %u; Sc: %u; Sl: %u; Encoded: ",
    pFrame->m_Header.m_SegmentControl.m_nRes, pFrame->m_Header.m_SegmentControl.m_nTmr,
    pFrame->m_Header.m_SegmentControl.m_nCc, pFrame->m_Header.m_SegmentControl.m_nCap,
    pFrame->m_Header.m_SegmentControl.m_nLsf, pFrame->m_Header.m_SegmentControl.m_nSc,
    pFrame->m_Header.m_SegmentControl.m_nSl));
  pu8End += 3;
  // Rest of the header.
  pu8End = EncodeMhr(&pFrame->m_Header, pu8End);
  // Encode the payload.
  memcpy(&pu8End[0], pFrame->m_pu8Payload, pFrame->m_u16PayloadLength);
  LOG_DBG(LogBuffer(pu8End, pFrame->m_u16PayloadLength, "Payload: "));
  pu8End += pFrame->m_u16PayloadLength;
  // Encode the padding.
  memset(&pu8End[0], 0, pFrame->m_u8PadLength);
  LOG_DBG(LogBuffer(pu8End, pFrame->m_u8PadLength, "Padding: "));
  pu8End += pFrame->m_u8PadLength;
  // Calculate and append FCS.
  u32Field = Crc16Ccitt(pu8Psdu, pu8End - pu8Psdu, 0);
  pFrame->m_u16Fcs = u32Field;
  pu8End[0] = u32Field & 0xFF;
  u32Field >>= 8;
  pu8End[1] = u32Field & 0xFF;
  pu8End += 2;
  return pu8End - pu8Psdu;
}

static uint8_t CalculateFrameOverhead(struct TMacRtFrame *pFrame)
{
  uint8_t u8Length = 0;
  // Segment control.
  u8Length += 3;
  // FC
  u8Length += 2;
  // Sequence number.
  u8Length ++;
  // Destination PAN.
  if (pFrame->m_Header.m_DestinationAddress.m_eAddrMode != MAC_RT_ADDRESS_MODE_NO_ADDRESS) {
    u8Length += 2;
  }
  // Destination address.
  switch (pFrame->m_Header.m_DestinationAddress.m_eAddrMode) {
    case MAC_RT_ADDRESS_MODE_NO_ADDRESS:
      // Nothing to do.
      break;
    case MAC_RT_ADDRESS_MODE_SHORT:
      // Add short address.
      u8Length += 2;
      break;
    case MAC_RT_ADDRESS_MODE_EXTENDED:
      // Add extended address.
      u8Length += sizeof(struct TExtAddress);
      break;
    default:
      // No idea what this might be.
      break;
  }
  // Source PAN.
  if ((pFrame->m_Header.m_SourceAddress.m_eAddrMode != MAC_RT_ADDRESS_MODE_NO_ADDRESS) &&
    ((pFrame->m_Header.m_DestinationAddress.m_eAddrMode == MAC_RT_ADDRESS_MODE_NO_ADDRESS) ||
    (pFrame->m_Header.m_Fc.m_nPanIdCompression == 0))) {
    u8Length += 2;
  }
  // Source address.
  switch (pFrame->m_Header.m_SourceAddress.m_eAddrMode) {
    case MAC_RT_ADDRESS_MODE_NO_ADDRESS:
      // Nothing to do.
      break;
    case MAC_RT_ADDRESS_MODE_SHORT:
      // Add short address.
      u8Length += 2;
      break;
    case MAC_RT_ADDRESS_MODE_EXTENDED:
      // Add extended address.
      u8Length += sizeof(struct TExtAddress);
      break;
    default:
      // No idea what this might be.
      break;
  }
  // Security header.
  if ((pFrame->m_Header.m_Fc.m_nSecurityEnabled != 0) && (pFrame->m_Header.m_SegmentControl.m_nSc == 0)) {
    u8Length += 6;
  }
  // Add FCS length.
  u8Length += 2;
  return u8Length;
}

static void CalculateCsmaCaBackoff(void)
{
  if (g_MacRt.m_TxRequest.m_bHighPriority) {
    g_MacRt.m_u32Backoff = (platform_random_32() % g_MacRtMib.m_u8HighPriorityWindowSize) * g_u32MacSlotTime;
    g_MacRt.m_u32PeriodWait = 1 * g_u32MacSlotTime;
  }
  else {
    if (g_MacRt.m_u32Cw == (1u << g_MacRtMib.m_u8MinBe)) {
      g_MacRt.m_u16MinCwCount ++;
      g_MacRt.m_u32Nbf = 0;
    }
    else {
      g_MacRt.m_u16MinCwCount = 0;
    }
    if (g_MacRt.m_u16MinCwCount > g_MacRtMib.m_u8MinCwAttempts) {
      g_MacRt.m_u32Cw = (1 << g_MacRtMib.m_u8MaxBe);
    }
    if (u8RtMibSpecCompliance == MAC_RT_SPEC_COMPLIANCE_15) {
      g_MacRt.m_u32Backoff = (platform_random_32() % g_MacRt.m_u32Cw) * g_u32MacSlotTime;
    }
    else {
      if ((g_MacRt.m_TxRequest.m_DstAddr.m_nShortAddress == MAC_RT_SHORT_ADDRESS_BROADCAST)
        && g_MacRtMib.m_bBroadcastMaxCwEnable) {
        // Maximum CSMA contention window for normal priority broadcast packets (CCTT#188)
        g_MacRt.m_u32Backoff = (platform_random_32() % (1 << g_MacRtMib.m_u8MaxBe)) * g_u32MacSlotTime;
      }
      else {
        g_MacRt.m_u32Backoff = (platform_random_32() % g_MacRt.m_u32Cw) * g_u32MacSlotTime;
      }
    }
    LOG_INFO(Log("CW %u, BO %u", g_MacRt.m_u32Cw, g_MacRt.m_u32Backoff));
    g_MacRt.m_u32PeriodWait = (1 + g_MacRtMib.m_u8HighPriorityWindowSize) * g_u32MacSlotTime;
  }
}

static uint8_t CalculatePadding(struct TPhyParameters *pParameters, uint16_t u16FrameLength, uint8_t u8RsBlocks)
{
  if (u8RsBlocks == 0) {
    // Protection, set Cen-A default value
    u8RsBlocks = 1;
  }
  uint16_t u16BlockLength = (u16FrameLength + u8RsBlocks - 1) / u8RsBlocks;
  uint32_t u32MaxPdu;
  // Find out the number of symbols.
  uint32_t u32Ns = (((u16BlockLength + pParameters->m_u8RsParity) * 8) + 6) * pParameters->m_u8RepCode * 2;
  uint32_t u32Div = g_PhyBandInformation.m_u8FlBand * pParameters->m_u8SubCarriers * pParameters->m_u8BitsSubcarrier;
  if (u32Div == 0) {
    // Protection, set Cen-A default values
    u32Div = 4 * 36;
  }
  u32Ns = (u32Ns + u32Div - 1) / u32Div * g_PhyBandInformation.m_u8FlBand;
  // Find out max PDU size for this NS.
  u32MaxPdu = (((u32Ns * pParameters->m_u8SubCarriers * pParameters->m_u8BitsSubcarrier) - (6 * 2 * pParameters->m_u8RepCode)) /
    (8 * 2 * pParameters->m_u8RepCode)) - pParameters->m_u8RsParity;
  return (u32MaxPdu * u8RsBlocks) - u16FrameLength;
}

static void CreateNextSegment(void)
{
  struct TPhyParameters phyParams = CalculatePhyParameters(&g_MacRt.m_TxParameters);
  uint8_t u8Overhead;
  bool bLast;
  uint16_t u16OneRsLength = 0;
  uint8_t u8RsBlocks = 0;

  // Calculate the payload length.
  u8AvailableRSBlocks = g_PhyBandInformation.m_u8MaxRsBlocks;
  u16OneRsLength = CalculateMaxPhyPayload(&phyParams);
  if (u16OneRsLength == 0) {
    // Protection, set Cen-A default value for ROBO
    u16OneRsLength = 133;
  }
  g_MacRt.m_u16SegmentLength = u16OneRsLength * u8AvailableRSBlocks;
  g_MacRt.m_TxFrame.m_Header.m_SegmentControl.m_nSc = g_MacRt.m_u8TxSegment;
  // Subtract MAC header and FCS from the segment length.
  u8Overhead = CalculateFrameOverhead(&g_MacRt.m_TxFrame);
  // TODO: Segment length can be negative here! We have to handle this case.
  g_MacRt.m_u16SegmentLength -= u8Overhead;
  bLast = g_MacRt.m_u16SegmentLength >= (g_MacRt.m_TxRequest.m_u16MsduLength - g_MacRt.m_u16TxOffset);
  if (bLast) {
    g_MacRt.m_u16SegmentLength = g_MacRt.m_TxRequest.m_u16MsduLength - g_MacRt.m_u16TxOffset;
  }
  // Fill in segment related info.
  g_MacRt.m_TxFrame.m_Header.m_SegmentControl.m_nTmr = (bLast && g_MacRt.m_TxRequest.m_bToneMapRequest) ? 1 : 0;
  g_MacRt.m_TxFrame.m_Header.m_SegmentControl.m_nCc = (bLast) ? 0 : 1;
  g_MacRt.m_TxFrame.m_Header.m_SegmentControl.m_nLsf = (bLast) ? 1 : 0;
  g_MacRt.m_TxFrame.m_Header.m_SegmentControl.m_nSl = g_MacRt.m_u16SegmentLength;
  // Payload.
  g_MacRt.m_TxFrame.m_u16PayloadLength = g_MacRt.m_u16SegmentLength;
  g_MacRt.m_TxFrame.m_pu8Payload = (pTxData + g_MacRt.m_u16TxOffset);
  // Add the padding.
  u8RsBlocks = (g_MacRt.m_u16SegmentLength + u8Overhead + u16OneRsLength - 1) / u16OneRsLength;
  g_MacRt.m_TxParameters.m_TxParameters.m_u8TwoRSBlocks = u8RsBlocks - 1;
  g_MacRt.m_TxFrame.m_u8PadLength = CalculatePadding(&phyParams, g_MacRt.m_u16SegmentLength + u8Overhead, u8RsBlocks);
  // Encode the frame.
  g_MacRt.m_u16TxPhyDataLength = EncodeFrame(&g_MacRt.m_TxFrame, g_MacRt.m_au8TxPhyData);
  g_MacRt.m_u16TxFcs = g_MacRt.m_TxFrame.m_u16Fcs;
}

//#define PRINT_MAC_TEST_VECTORS

static void RequestTransmission(uint32_t u32Time)
{
  struct TPdDataRequest request;

  request.m_u16PsduLength = g_MacRt.m_u16TxPhyDataLength;
  request.m_pPsdu = g_MacRt.m_au8TxPhyData;
  request.m_bDelayed = true;
  request.m_u32Time = u32Time;
  request.m_bPerformCs = true;

  g_MacRt.m_bLastMsgCsmaFail = false;
  g_MacRt.m_eTxState = MAC_RT_TX_WAIT_CONFIRM;
#ifdef PRINT_MAC_TEST_VECTORS
  printf("PdDataRequest: ");
  uint16_t i;
  for (i = 0; i < request.m_u16PsduLength; i++) {
    printf("%02X", request.m_pPsdu[i]);
  }
  printf("\r\n");
#endif
  PhyPlmeSetRequest(&g_MacRt.m_TxParameters);
  PhyPdDataRequest(&request);
}

static void ProcessTxRequest(void)
{
  uint8_t u8RetriesLeft;
  uint32_t u32MinBe = 0;
  uint32_t u32Subtract = 0;
  uint32_t u32MaxBe = 0;
  uint32_t u32TxTime = 0;

  // Check if a tx was requested.
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_START)) {
    // Initialize transmission.
    g_MacRt.m_eTxState = MAC_RT_TX_CSMA_CA;
    g_MacRt.m_u8TxRetries = 0;
    g_MacRt.m_u16Nb = 0;
    g_MacRt.m_u32Nbf = 0;
    CalculateTxParameters();
  }
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_ABORT)) {
    // Unknown error, abort.
    g_MacRt.m_bTxRequest = false;
    if (g_mac_rt_notifications.m_pMacRtTxConfirm != NULL) {
      g_mac_rt_notifications.m_pMacRtTxConfirm(MAC_RT_STATUS_DENIED, false, (enum ERtModulationType)g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType);
    }
  }
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_BIG_FAIL)) {
    if (g_MacRt.m_u8TxRetries >= g_MacRtMib.m_u8MaxFrameRetries) {
      // Too many retries, abort.
      g_MacRt.m_bTxRequest = false;
      if (g_mac_rt_notifications.m_pMacRtTxConfirm != NULL) {
        g_mac_rt_notifications.m_pMacRtTxConfirm(MAC_RT_STATUS_NO_ACK, false, (enum ERtModulationType)g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType);
      }
    }
    else {
      g_MacRt.m_u8TxRetries ++;
      g_MacRt.m_eTxState = MAC_RT_TX_CSMA_CA;
      g_MacRt.m_u16Nb = 0;
      g_MacRt.m_u32Nbf = 0;

      // If the modulation is not forced and there are retries:
      // - force ROBO (MAC_PIB_MANUF_RETRIES_LEFT_TO_FORCE_ROBO)
      // - decrease the modulation (CCTT#182, SPEC17)
      if (g_MacRtMib.m_u8ForcedModType == 0) {
        u8RetriesLeft = g_MacRtMib.m_u8MaxFrameRetries - g_MacRt.m_u8TxRetries;

        if (!g_MacRt.m_TxRequest.m_bForceRobo && (g_MacRt.m_TxRequest.m_DstAddr.m_eAddrMode == MAC_RT_ADDRESS_MODE_SHORT)) {
          if (u8RetriesLeft < g_MacRtMib.m_u8RetriesToForceRobo) {
            // Force ROBO (MAC_PIB_MANUF_RETRIES_LEFT_TO_FORCE_ROBO)
            g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_ROBUST;
            memcpy(&g_MacRt.m_TxParameters.m_TxParameters.m_ToneMap, &g_RtToneMapDefault, sizeof(struct TPhyToneMap));
          }
          else {
            if (u8RtMibSpecCompliance >= MAC_RT_SPEC_COMPLIANCE_17) {
              // If the modulation is not forced and there are retries, decrease the modulation (CCTT#182)
              switch (u8RetriesLeft) {
                case 0:
                case 1:
                  if (g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType > PHY_MODULATION_ROBUST) {
                    g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_ROBUST;
                    memcpy(&g_MacRt.m_TxParameters.m_TxParameters.m_ToneMap, &g_RtToneMapDefault, sizeof(struct TPhyToneMap));
                  }
                  break;
                case 2:
                  if (g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType > PHY_MODULATION_DBPSK_BPSK) {
                    g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_DBPSK_BPSK;
                  }
                  break;
                case 3:
                  if (g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType > PHY_MODULATION_DQPSK_QPSK) {
                    g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_DQPSK_QPSK;
                  }
                  break;
                case 4:
                  if (g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType > PHY_MODULATION_D8PSK_8PSK) {
                    g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType = PHY_MODULATION_D8PSK_8PSK;
                  }
                  break;
              }
            }
          }
        }
      }
    }
  }
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_LITTLE_FAIL)) {
    if (g_MacRt.m_u8TxRetries >= g_MacRtMib.m_u8MaxFrameRetries) {
      // Too many retries, abort.
      g_MacRt.m_bTxRequest = false;
      // Go from CIFS_RETRANSMIT to normal CIFS.
      g_MacRt.m_eState = MAC_RT_CIFS;
      if (g_mac_rt_notifications.m_pMacRtTxConfirm != NULL) {
        g_mac_rt_notifications.m_pMacRtTxConfirm(MAC_RT_STATUS_NO_ACK, false, (enum ERtModulationType)g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType);
      }
    }
    else {
      g_MacRt.m_u8TxRetries ++;
      g_MacRt.m_eTxState = MAC_RT_TX_WAIT_SEND;
      g_MacRt.m_u32Backoff = 0;
      g_MacRt.m_u32PeriodWait = 0;
    }
  }
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_SEND_OK)) {
    g_MacRt.m_eTxState = MAC_RT_TX_WAIT_SEND;
    g_MacRt.m_u16TxOffset += g_MacRt.m_u16SegmentLength;
    if (g_MacRt.m_u16TxOffset < g_MacRt.m_TxRequest.m_u16MsduLength) {
      // Step to the next fragment.
      g_MacRt.m_u8TxSegment ++;
      g_MacRt.m_u32Backoff = 0;
      g_MacRt.m_u32PeriodWait = 0;
      CreateNextSegment();
    }
    else {
      u32MinBe = (1 << g_MacRtMib.m_u8MinBe);
      u32Subtract = g_MacRtMib.m_u8A * u32MinBe;
      if (g_MacRt.m_u32Cw > (u32Subtract + u32MinBe)) {
        g_MacRt.m_u32Cw -= u32Subtract;
      }
      else {
        g_MacRt.m_u32Cw = u32MinBe;
      }
      LOG_DBG(Log("Updated CW %u", g_MacRt.m_u32Cw));
      // Frame transmission terminated.
      g_MacRt.m_bTxRequest = false;
      if (g_mac_rt_notifications.m_pMacRtTxConfirm != NULL) {
        g_mac_rt_notifications.m_pMacRtTxConfirm(MAC_RT_STATUS_SUCCESS, false, (enum ERtModulationType)g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType);
      }
    }
  }
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_FAIL_CSMA_CA)) {
    // Update values and abort if the limit was reached.
    if (g_MacRt.m_TxRequest.m_bHighPriority) {
      g_MacRt.m_u16Nb ++;
    }
    else {
      g_MacRt.m_u16Nb ++;
      g_MacRt.m_u32Nbf ++;
      LOG_DBG(Log("Updated NB %u, NBF %u", g_MacRt.m_u16Nb, g_MacRt.m_u32Nbf));
      if (g_MacRt.m_u32Nbf >= g_MacRtMib.m_u8CsmaFairnessLimit) {
        if ((g_MacRt.m_u32Nbf % g_MacRtMib.m_u8K) == 0) {
          u32MinBe = (1 << g_MacRtMib.m_u8MinBe);
          u32Subtract = g_MacRtMib.m_u8A * u32MinBe;
          if (g_MacRt.m_u32Cw > (u32Subtract + u32MinBe)) {
            g_MacRt.m_u32Cw -= u32Subtract;
          }
          else {
            g_MacRt.m_u32Cw = u32MinBe;
          }
          LOG_DBG(Log("*Updated CW %u", g_MacRt.m_u32Cw));
        }
      }
      else {
        g_MacRt.m_u32Cw *= 2;
        u32MaxBe = (1 << g_MacRtMib.m_u8MaxBe);
        if (g_MacRt.m_u32Cw > u32MaxBe) {
          g_MacRt.m_u32Cw = u32MaxBe;
        }
        LOG_DBG(Log("**Updated CW %u", g_MacRt.m_u32Cw));
      }
    }
    if (g_MacRt.m_u16Nb >= g_MacRtMib.m_u8MaxCsmaBackoffs) {
      LOG_DBG(Log("m_u8MaxCsmaBackoffs limit reached!!!\r\n"));


      // Backoff limit reached, abort.
      g_MacRt.m_bTxRequest = false;
      if (g_mac_rt_notifications.m_pMacRtTxConfirm != NULL) {
        g_mac_rt_notifications.m_pMacRtTxConfirm(MAC_RT_STATUS_CHANNEL_ACCESS_FAILURE, false, (enum ERtModulationType)g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType);
      }
    }
    else {
      g_MacRt.m_eTxState = MAC_RT_TX_CSMA_CA;
    }
  }
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_CSMA_CA)) {
    // Start with CSMA_CA from first segment.
    g_MacRt.m_eTxState = MAC_RT_TX_WAIT_SEND;
    g_MacRt.m_u8TxSegment = 0;
    g_MacRt.m_u16TxOffset = 0;
    CreateNextSegment();
    CalculateCsmaCaBackoff();
  }
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_WAIT_SEND)) {
    switch (g_MacRt.m_eState) {
      case MAC_RT_IDLE:
        // We can start the backoff period after idle spacing.
        RequestTransmission(PhyGetTime() + g_u32MacIdleSpacingTime + g_MacRt.m_u32Backoff);
        break;
      case MAC_RT_EIFS:
        // We could start the backoff period after the timeout, but
        // send only after EIFS ended (either because of a timeout of because of a reception).
        break;
      case MAC_RT_CIFS:
        // We can start the backoff period after CIFS finished + CFA + priority.
        RequestTransmission(g_MacRt.m_u32ChangeTime + g_u32MacCifsTime + g_MacRt.m_u32PeriodWait + g_MacRt.m_u32Backoff);
        break;
      case MAC_RT_CIFS_RETRANSMIT:
        // We can start the backoff period after CIFS finished + CFA + priority.
        RequestTransmission(g_MacRt.m_u32ChangeTime + g_u32MacCifsRetransmitTime + g_MacRt.m_u32PeriodWait + g_MacRt.m_u32Backoff);
        break;
      case MAC_RT_RIFS:
        // We can not start transmission here.
        break;
      case MAC_RT_CP:
        // Check if we have enough time to schedule a transmission.
      {
        u32TxTime = g_MacRt.m_u32ChangeTime + g_MacRt.m_u32PeriodWait + g_MacRt.m_u32Backoff;
        if (((int32_t) (u32TxTime - PhyGetTime())) >= MAC_RT_MINIMAL_TX_DELAY) {
          // There is still time, request the transmission.
          RequestTransmission(u32TxTime);
        }
        else {
          // Better wait for idle or CIFS.
        }
      }
      break;
      default:
        break;
    }
  }
}

void MacRtTxRequest(struct TMacRtTxRequest *pTxRequest, struct TMacRtMhr *pMhr)
{
  if (pTxRequest->m_u16MsduLength <= MAC_RT_MAX_PAYLOAD_SIZE) {
    pTxData = pTxRequest->m_pMsdu;
    memcpy(&g_MacRt.m_TxRequest, pTxRequest, sizeof(struct TMacRtTxRequest));
    memcpy(&g_MacRt.m_TxFrame.m_Header, pMhr, sizeof(struct TMacRtMhr));
    g_MacRt.m_bTxRequest = true;
    g_MacRt.m_eTxState = MAC_RT_TX_START;
  }
}

void MacRtResetRequest(bool bResetMib)
{
  if (bResetMib) {
    MacRtMibReset();
  }
  g_MacRt = g_MacRtDefaults;
  g_MacRt.m_u32Cw = (1 << g_MacRtMib.m_u8MinBe);
  LOG_DBG(Log("Initialization of CW %u", g_MacRt.m_u32Cw));
  PhyPlmeResetRequest();
  g_MacRt.m_u32ChangeTime = PhyGetTime();
}

void MacRtGetToneMapResponseData(struct TRtToneMapResponseData *pParameters)
{
  PhyGetToneMapResponseData((struct TPlmeGetToneMapResponseData *)pParameters);
}

uint32_t MacRtGetPhyTime(void)
{
  return PhyGetTime();
}

static void MacCbPhyPdDataConfirm(struct TPdDataConfirm *pParameters)
{
  LOG_INFO(Log("PdDataConfirm. Status: %u; Time: %u", pParameters->m_eStatus, pParameters->m_u32Time));
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_WAIT_CONFIRM)) {
    switch (pParameters->m_eStatus) {
      case PHY_STATUS_BUSY: // CS reported busy
      case PHY_STATUS_BUSY_RX: // receive in progress
        g_MacRt.m_bLastMsgCsmaFail = true;
        // This means we have failed to send in this try.
        if (g_MacRt.m_u32PeriodWait != 0) {
          // We are in CSMA-CA.
//          if (((int32_t) (pParameters->m_u32Time - (g_MacRt.m_u32ChangeTime + g_MacRt.m_u32PeriodWait))) <= 0) {
//            // Already in high/normal priority slot, retry CSMA-CA.
          g_MacRt.m_eTxState = MAC_RT_TX_FAIL_CSMA_CA;
//          }
//          else {
//            // A message was received before the correct slot, try again later.
//            g_MacRt.m_eTxState = MAC_RT_TX_WAIT_SEND;
//          }
          g_MacRt.m_eState = MAC_RT_EIFS;
          g_MacRt.m_u32ChangeTime = pParameters->m_u32Time;
        }
        else {
          // We are in CFS, so this fail is unexpected. Go to EIFS and start CSMA-CA again.
          // Even if in different condition, do the same as above, otherwise this can be misunderstood as NO_ACK
          g_MacRt.m_eTxState = MAC_RT_TX_FAIL_CSMA_CA;
          g_MacRt.m_eState = MAC_RT_EIFS;
          g_MacRt.m_u32ChangeTime = pParameters->m_u32Time;
        }
        break;
      case PHY_STATUS_BUSY_TX:
        // This should not happen, abort transmission.
        g_MacRt.m_bTxRequest = false;
        if (g_mac_rt_notifications.m_pMacRtTxConfirm != NULL) {
          g_mac_rt_notifications.m_pMacRtTxConfirm(MAC_RT_STATUS_TRANSACTION_OVERFLOW, true, (enum ERtModulationType)g_MacRt.m_TxParameters.m_TxParameters.m_eModulationType);
        }
        break;
      case PHY_STATUS_SUCCESS:
        // Success, now wait for ACK or go to the next step.
        g_MacRt.m_u32ChangeTime = pParameters->m_u32Time;
        if (g_MacRt.m_TxRequest.m_bRequestAck) {
          g_MacRt.m_eState = MAC_RT_RIFS;
          g_MacRt.m_eTxState = MAC_RT_TX_WAIT_ACK;
        }
        else {
          g_MacRt.m_eState = MAC_RT_CIFS;
          g_MacRt.m_eTxState = MAC_RT_TX_SEND_OK;
        }
        break;
      default:
        // Unknown status.
        g_MacRt.m_eState = MAC_RT_EIFS;
        g_MacRt.m_u32ChangeTime = pParameters->m_u32Time;
        g_MacRt.m_eTxState = MAC_RT_TX_ABORT;
        break;
    }
  }
}

static void MacCbPhyPdDataIndication(struct TPdDataIndication *pParameters)
{
  struct TMacRtFrame frame;
  struct TMacRtDataIndication indication;
  bool bOk = DecodeFrame(&frame, pParameters);
  if ((pParameters->m_u8Dt == 0x01) && CheckDestinationAck(&frame)) {
    struct TPdAckRequest request;
    uint8_t u8Attenuation3db;

    request.m_u8TxPower = 0;
    if (u8RtMibSpecCompliance >= MAC_RT_SPEC_COMPLIANCE_17) {
      // macTransmitAtten affects also to ACKs
      u8Attenuation3db = (g_MacRtMib.m_u8TransmitAtten + 2) / 3;
      if (u8Attenuation3db > 0x1f) {
        request.m_u8TxPower = 0x1f;
      }
      else {
        request.m_u8TxPower = u8Attenuation3db;
      }
    }

    request.m_AckFch.m_u16Fcs = frame.m_u16Fcs;
    request.m_AckFch.m_u8Ssca = (bOk && (frame.m_Header.m_SegmentControl.m_nLsf != 0)) ? 0 : 1;
    request.m_AckFch.m_eDelimiterType = (bOk) ? PHY_DELIMITER_ACK : PHY_DELIMITER_NACK;
    request.m_bDelayed = true;
    request.m_u32Time = pParameters->m_u32Time + g_u32MacRifsTime;
    PhyPdAckRequest(&request);
  }
  if (bOk) {

    if ((frame.m_Header.m_SegmentControl.m_nSc > 0) && g_MacRt.m_bLastMsgCsmaFail) {
      g_MacRt.m_u16Nb --;
      g_MacRt.m_u32Nbf --;
      g_MacRt.m_bLastMsgCsmaFail = false;
      LOG_INFO(Log("Decreased NB %u, NBF %u", g_MacRt.m_u16Nb, g_MacRt.m_u32Nbf));
    }

    // Valid frame received.
    // Update Rx state.
    g_MacRt.m_eState = (pParameters->m_u8Dt == 0x01) ? MAC_RT_RIFS : MAC_RT_CIFS;
    g_MacRt.m_u32ChangeTime = pParameters->m_u32Time;
    {
      struct TPlmeGetRequest plmeGetRequest;
      PhyPlmeGetRequest(&plmeGetRequest);
    }

    // Callback to Process Frame, plmeGetRequest parameters already sent to upper layer in Confirm function.
    if (g_mac_rt_notifications.m_pProcessFrame != NULL) {
      // Before calling Process Frame callback, check Tx state and change it if necessary
      if (g_MacRt.m_bTxRequest) {
        if (g_MacRt.m_eTxState == MAC_RT_TX_WAIT_CONFIRM) {
          // Scheduled transmission was interrupted by a data indication. Try again later.
          g_MacRt.m_eTxState = MAC_RT_TX_WAIT_SEND;
        }
        else if (g_MacRt.m_eTxState == MAC_RT_TX_WAIT_ACK) {
          // Data indication while waiting for ACK. This is a collision.
          g_MacRt.m_eTxState = MAC_RT_TX_BIG_FAIL;
        }
        // Other cases do not need to be addressed
      }

      // Build indication and call Callback function
      indication.m_u16PsduLength = pParameters->m_u16PsduLength;
      indication.m_pPsdu = pParameters->m_pPsdu;
      g_mac_rt_notifications.m_pProcessFrame(&frame, &indication);
    }
  }
  else {
    // Invalid frame received, ignore.
  }
}

static void MacCbPhyPdAckConfirm(struct TPdAckConfirm *pParameters)
{
  g_MacRt.m_u32ChangeTime = pParameters->m_u32Time;
  if (pParameters->m_eStatus == PHY_STATUS_SUCCESS) {
    // All right, update the state.
    g_MacRt.m_eState = MAC_RT_CIFS;
  }
  else {
    // Something unexpected happened, do EIFS.
    g_MacRt.m_eState = MAC_RT_EIFS;
  }
}

static void MacCbPhyPdAckIndication(struct TPdAckIndication *pParameters)
{
  // Update layer state.
  g_MacRt.m_u32ChangeTime = pParameters->m_u32Time;
  if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_WAIT_ACK)) {
    if (pParameters->m_AckFch.m_eDelimiterType == PHY_DELIMITER_ACK) {
      if (pParameters->m_AckFch.m_u16Fcs == g_MacRt.m_u16TxFcs) {
        // Positive ACK, FCS correct - enter CIFS and continue sending.
        g_MacRt.m_eState = MAC_RT_CIFS;
        g_MacRt.m_eTxState = MAC_RT_TX_SEND_OK;
      }
      else {
        // Positive ACK, but FCS incorrect - enter EIFS and start over.
        g_MacRt.m_eTxState = MAC_RT_TX_BIG_FAIL;
        g_MacRt.m_eState = MAC_RT_EIFS;
      }
    }
    else {
      if (pParameters->m_AckFch.m_u8Ssca == 1) {
        g_MacRt.m_eTxState = MAC_RT_TX_LITTLE_FAIL;
        g_MacRt.m_eState = MAC_RT_CIFS_RETRANSMIT;
      }
      else {
        g_MacRt.m_eTxState = MAC_RT_TX_BIG_FAIL;
        g_MacRt.m_eState = MAC_RT_EIFS;
      }
    }
  }
  else {
    if (g_MacRt.m_bTxRequest) {
      // Update Tx state.
      switch (g_MacRt.m_eTxState) {
        case MAC_RT_TX_WAIT_CONFIRM:
          // Scheduled transmission was interrupted by an ack indication. Try again on the next ocasion.
          g_MacRt.m_eTxState = MAC_RT_TX_WAIT_SEND;
          break;
        case MAC_RT_TX_WAIT_ACK:
        case MAC_RT_TX_START:
        case MAC_RT_TX_CSMA_CA:
        case MAC_RT_TX_FAIL_CSMA_CA:
        case MAC_RT_TX_WAIT_SEND:
        case MAC_RT_TX_SEND_OK:
        case MAC_RT_TX_BIG_FAIL:
        case MAC_RT_TX_LITTLE_FAIL:
        default:
          // Nothing to do.
          break;
      }
    }
    if (pParameters->m_AckFch.m_u8Ssca == 1) {
      g_MacRt.m_eState = MAC_RT_EIFS;
    }
    else {
      g_MacRt.m_eState = MAC_RT_CIFS;
    }
  }
}

static void MacCbPhyPlmeSetConfirm(struct TPlmeSetConfirm *pParameters)
{
  // Nothing to do, we are assuming that the PHY accepted.
  UNUSED(pParameters);
}

static void MacCbPhyPlmeGetConfirm(struct TPlmeGetConfirm *pParameters)
{
  struct TMacRtPlmeGetConfirm macRtPlmeParams;

  macRtPlmeParams.m_u8PpduLinkQuality = pParameters->m_u8PpduLinkQuality;
  macRtPlmeParams.m_u8PhaseDifferential = pParameters->m_u8PhaseDifferential;
  macRtPlmeParams.m_eModulationType = (enum ERtModulationType)pParameters->m_eModulationType;
  macRtPlmeParams.m_eModulationScheme = (enum ERtModulationScheme)pParameters->m_eModulationScheme;
  memcpy(&macRtPlmeParams.m_ToneMap, &pParameters->m_ToneMap, sizeof(macRtPlmeParams.m_ToneMap));
  if (g_mac_rt_notifications.m_pMacRtPlmeGetConfirm != NULL) {
    g_mac_rt_notifications.m_pMacRtPlmeGetConfirm(&macRtPlmeParams);
  }
}

static void MacCbPhyPlmeSetTrxStateConfirm(struct TPlmeSetTrxStateConfirm *pParameters)
{
  UNUSED(pParameters);
}

static void MacCbPhyPlmeCsConfirm(struct TPlmeCsConfirm *pParameters)
{
  // Nothing to do, CS request / confirm not called directly.
  UNUSED(pParameters);
}

static struct TPhyNotifications g_MacPhyNotifications = {
  MacCbPhyPdDataConfirm,
  MacCbPhyPdDataIndication,
  MacCbPhyPdAckConfirm,
  MacCbPhyPdAckIndication,
  MacCbPhyPlmeSetConfirm,
  MacCbPhyPlmeGetConfirm,
  MacCbPhyPlmeSetTrxStateConfirm,
  MacCbPhyPlmeCsConfirm
};

void MacRtInitialize(uint8_t u8Band, struct TMacRtNotifications *pNotifications, uint8_t u8SpecCompliance)
{
  // Initialize Constants and MIB
  MacRtConstantsInitialize(u8Band, NULL);
  MacRtMibInitialize(u8SpecCompliance);

  g_mac_rt_notifications = *pNotifications;
  g_MacRt = g_MacRtDefaults;
  g_MacRt.m_u32Cw = (1 << g_MacRtMib.m_u8MinBe);
  PhyInitialize(&g_MacPhyNotifications, u8Band);
  g_MacRt.m_u32ChangeTime = PhyGetTime();
  g_MacRt.m_bExecutePhySetRequest = true;
}

void MacRtEventHandler(void)
{
  uint32_t u32CurrentTime = 0;
  uint32_t u32CpTime = 0;

  if (g_MacRt.m_bExecutePhySetRequest) {
    memcpy(&g_MacRt.m_TxParameters.m_TxParameters.m_ToneMask, &g_MacRtMib.m_ToneMask, sizeof(g_MacRt.m_TxParameters.m_TxParameters.m_ToneMask));
    PhyPlmeSetRequest(&g_MacRt.m_TxParameters);
    g_MacRt.m_bExecutePhySetRequest = false;
  }

  PhyEventHandler();
  // Update the RX / TX state.
  u32CurrentTime = PhyGetTime();
  for (;;) {
    enum EMacRtState eOldRxState = g_MacRt.m_eState;
    switch (g_MacRt.m_eState) {
      case MAC_RT_IDLE:
        // Nothing to do, waiting for something.
        break;
      case MAC_RT_EIFS:
        // Go to idle after EIFS time.
        if ((u32CurrentTime - g_MacRt.m_u32ChangeTime) >= g_u32MacEifsTime) {
          g_MacRt.m_eState = MAC_RT_IDLE;
          g_MacRt.m_u32ChangeTime += g_u32MacEifsTime;
        }
        break;
      case MAC_RT_CIFS:
        // Go to CP after CIFS time.
        if ((u32CurrentTime - g_MacRt.m_u32ChangeTime) >= g_u32MacCifsTime) {
          g_MacRt.m_eState = MAC_RT_CP;
          g_MacRt.m_u32ChangeTime += g_u32MacCifsTime;
        }
        break;
      case MAC_RT_CIFS_RETRANSMIT:
        // Go to CP after CIFS time.
        if ((u32CurrentTime - g_MacRt.m_u32ChangeTime) >= g_u32MacCifsRetransmitTime) {
          g_MacRt.m_eState = MAC_RT_CP;
          g_MacRt.m_u32ChangeTime += g_u32MacCifsRetransmitTime;
        }
        break;
      case MAC_RT_RIFS:
        // Go to ACK after RIFS time.
        if ((u32CurrentTime - g_MacRt.m_u32ChangeTime) >= (g_u32MacRifsTime + g_u32MacAckTime + g_u32MacCifsTime)) {
          // Here the ACK was not received, we go to EIFS and restart the transmission.
          g_MacRt.m_eState = MAC_RT_EIFS;
          g_MacRt.m_u32ChangeTime += g_u32MacRifsTime + g_u32MacAckTime + g_u32MacCifsTime;
          if (g_MacRt.m_bTxRequest && (g_MacRt.m_eTxState == MAC_RT_TX_WAIT_ACK)) {
            g_MacRt.m_eTxState = MAC_RT_TX_BIG_FAIL;
            g_MacRtMib.m_u32CsmaNoAckCount ++;
          }
        }
        break;
      case MAC_RT_CP:
        // Go to IDLE after the contention period.
      {
        u32CpTime = (1 + g_MacRtMib.m_u8HighPriorityWindowSize + (1 << g_MacRtMib.m_u8MaxBe)) * g_u32MacSlotTime;
        if ((u32CurrentTime - g_MacRt.m_u32ChangeTime) >= u32CpTime) {
          g_MacRt.m_eState = MAC_RT_IDLE;
          g_MacRt.m_u32ChangeTime += u32CpTime;
        }
      }
      break;
      default:
        // Unknown state.
        g_MacRt.m_eState = MAC_RT_EIFS;
        g_MacRt.m_u32ChangeTime = u32CurrentTime;
        break;
    }
    if (eOldRxState == g_MacRt.m_eState) {
      break;
    }
  }
  // Request transmission of the next segment.
  ProcessTxRequest();
}
