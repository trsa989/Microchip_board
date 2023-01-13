#include <stdbool.h>
#include <stdint.h>
#include <MacPhyInterface.h>
#include <MacRtDefs.h>
#include <MacRtConstants.h>
#include <stddef.h>

uint32_t g_u32MacSlotTime = 0;
uint32_t g_u32MacCifsTime = 0;
uint32_t g_u32MacCifsRetransmitTime = 0;
uint32_t g_u32MacRifsTime = 0;
uint32_t g_u32MacIdleSpacingTime = 0;
uint32_t g_u32MacAckTime = 0;
uint32_t g_u32MacEifsTime = 0;
uint32_t g_u32MacAckWaitDuration = 0;

struct TRtToneMask g_ToneMaskDefault;
struct TRtToneMask g_ToneMaskNotched;

uint8_t g_u8ToneMaskSizeBytes;

static const struct TRtToneMask g_ToneMaskDefaultCenelecA = {{0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00}};
static const struct TRtToneMask g_ToneMaskNotchedCenelecA = {{0xFF, 0xFF, 0x00, 0xF8, 0x0F, 0x00, 0x00, 0x00, 0x00}};
static const struct TRtToneMask g_ToneMaskDefaultCenelecB = {{0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static const struct TRtToneMask g_ToneMaskDefaultFcc = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
static const struct TRtToneMask g_ToneMaskDefaultArib = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00}};

static const uint8_t g_u8ToneMaskSizeBytesCenelecA = 5;
static const uint8_t g_u8ToneMaskSizeBytesCenelecB = 2;
static const uint8_t g_u8ToneMaskSizeBytesFcc = 9;
static const uint8_t g_u8ToneMaskSizeBytesArib = 7;

struct TRtToneMap g_RtToneMapDefault;

static const struct TRtToneMap g_RtToneMapDefaultCenelecA = {{0x3F, 0x00, 0x00}};
static const struct TRtToneMap g_RtToneMapDefaultCenelecB = {{0x0F, 0x00, 0x00}};
static const struct TRtToneMap g_RtToneMapDefaultFcc = {{0xFF, 0xFF, 0xFF}};
static const struct TRtToneMap g_RtToneMapDefaultArib = {{0xFF, 0xFF, 0x03}};

struct TPhyBandInformation g_PhyBandInformation;

static void InitBand(uint8_t u8Band)
{
  g_PhyBandInformation.m_u8Band = u8Band;
  switch (g_PhyBandInformation.m_u8Band) {
    case PHY_BAND_CENELEC_A:
      g_PhyBandInformation.m_u8Tones = 36;
      g_PhyBandInformation.m_u8Carriers = 6;
      g_PhyBandInformation.m_u8TonesInCarrier = 6;
      g_PhyBandInformation.m_u8FlBand = 4;
      g_PhyBandInformation.m_u16FlMax = 63;
      g_PhyBandInformation.m_u8MaxRsBlocks = 1;
      g_PhyBandInformation.m_u8TxCoefBits = 4;
      g_PhyBandInformation.m_u8PilotsFreqSpa = 12;
      break;
    case PHY_BAND_CENELEC_B:
      g_PhyBandInformation.m_u8Tones = 16;
      g_PhyBandInformation.m_u8Carriers = 4;
      g_PhyBandInformation.m_u8TonesInCarrier = 4;
      g_PhyBandInformation.m_u8FlBand = 4;
      g_PhyBandInformation.m_u16FlMax = 63;
      g_PhyBandInformation.m_u8MaxRsBlocks = 1;
      g_PhyBandInformation.m_u8TxCoefBits = 4;
      g_PhyBandInformation.m_u8PilotsFreqSpa = 8;
      break;
    case PHY_BAND_FCC:
      g_PhyBandInformation.m_u8Tones = 72;
      g_PhyBandInformation.m_u8Carriers = 24;
      g_PhyBandInformation.m_u8TonesInCarrier = 3;
      g_PhyBandInformation.m_u8FlBand = 1;
      g_PhyBandInformation.m_u16FlMax = 511;
      g_PhyBandInformation.m_u8MaxRsBlocks = 2;
      g_PhyBandInformation.m_u8TxCoefBits = 2;
      g_PhyBandInformation.m_u8PilotsFreqSpa = 12;
      break;
    case PHY_BAND_ARIB:
      g_PhyBandInformation.m_u8Tones = 54;
      g_PhyBandInformation.m_u8Carriers = 18;
      g_PhyBandInformation.m_u8TonesInCarrier = 3;
      g_PhyBandInformation.m_u8FlBand = 1;
      g_PhyBandInformation.m_u16FlMax = 511;
      g_PhyBandInformation.m_u8MaxRsBlocks = 1;
      g_PhyBandInformation.m_u8TxCoefBits = 2;
      g_PhyBandInformation.m_u8PilotsFreqSpa = 12;
      break;
    default:
      break;
  }
}

void MacRtConstantsInitialize(uint8_t u8Band, struct TRtToneMask *pToneMask)
{
  uint32_t u32MacPreambleSymbolTime = 0;
  uint32_t u32MacSymbolTime = 0;
  uint8_t u8MacNPreTimesTwo = 0;
  uint8_t u8MacFcSize = 0;
  uint8_t u8MacCifs = 0;
  uint8_t u8MacCifsRetransmit = 0;
  uint8_t u8MacRifs = 0;
  uint8_t u8MacIdleSpacing = 0;
  uint8_t u8MacMaxFrameSize = 0;
  switch (u8Band) {
    case PHY_BAND_CENELEC_A:
      u32MacPreambleSymbolTime = 640;
      u32MacSymbolTime = 695;
      u8MacNPreTimesTwo = 19;
      u8MacFcSize = 33;
      u8MacCifs = 8;
      u8MacCifsRetransmit = 8;
      u8MacRifs = 8;
      u8MacIdleSpacing = 8;
      u8MacMaxFrameSize = 252;
      g_RtToneMapDefault = g_RtToneMapDefaultCenelecA;
      g_ToneMaskDefault = g_ToneMaskDefaultCenelecA;
      g_ToneMaskNotched = g_ToneMaskNotchedCenelecA;
      g_u8ToneMaskSizeBytes = g_u8ToneMaskSizeBytesCenelecA;
      break;
    case PHY_BAND_CENELEC_B:
      u32MacPreambleSymbolTime = 640;
      u32MacSymbolTime = 695;
      u8MacNPreTimesTwo = 19;
      u8MacFcSize = 33;
      u8MacCifs = 8;
      u8MacCifsRetransmit = 8;
      u8MacRifs = 8;
      u8MacIdleSpacing = 8;
      u8MacMaxFrameSize = 252;
      g_RtToneMapDefault = g_RtToneMapDefaultCenelecB;
      g_ToneMaskDefault = g_ToneMaskDefaultCenelecB;
      g_ToneMaskNotched = g_ToneMaskDefaultCenelecB;
      g_u8ToneMaskSizeBytes = g_u8ToneMaskSizeBytesCenelecB;
      break;
    case PHY_BAND_FCC:
      u32MacPreambleSymbolTime = 213;
      u32MacSymbolTime = 232;
      u8MacNPreTimesTwo = 19;
      u8MacFcSize = 66;
      u8MacCifs = 10;
      u8MacCifsRetransmit = 10;
      u8MacRifs = 10;
      u8MacIdleSpacing = 8;
      u8MacMaxFrameSize = 252;
      g_RtToneMapDefault = g_RtToneMapDefaultFcc;
      g_ToneMaskDefault = g_ToneMaskDefaultFcc;
      g_ToneMaskNotched = g_ToneMaskDefaultFcc;
      g_u8ToneMaskSizeBytes = g_u8ToneMaskSizeBytesFcc;
      break;
    case PHY_BAND_ARIB:
      u32MacPreambleSymbolTime = 213;
      u32MacSymbolTime = 232;
      u8MacNPreTimesTwo = 19;
      u8MacFcSize = 66;
      u8MacCifs = 108;
      u8MacCifsRetransmit = 10;
      u8MacRifs = 10;
      u8MacIdleSpacing = 8;
      u8MacMaxFrameSize = 252;
      g_RtToneMapDefault = g_RtToneMapDefaultArib;
      g_ToneMaskDefault = g_ToneMaskDefaultArib;
      g_ToneMaskNotched = g_ToneMaskDefaultArib;
      g_u8ToneMaskSizeBytes = g_u8ToneMaskSizeBytesArib;
      break;
    default:
      break;
  }
  if (pToneMask == NULL) {
    pToneMask = &g_ToneMaskDefault;
  }
  uint8_t u8SubCarrier = 0;
  uint8_t u8Index;
  for (u8Index = 0; u8Index < g_u8ToneMaskSizeBytes; u8Index ++) {
    uint8_t u8Byte = pToneMask->m_au8ToneMask[u8Index];
    if ((u8Band == PHY_BAND_CENELEC_A) && (u8Index == (g_u8ToneMaskSizeBytes - 1))) {
      u8Byte &= 0x0F;
    }
    else if ((u8Band == PHY_BAND_ARIB) && (u8Index == (g_u8ToneMaskSizeBytes - 1))) {
      u8Byte &= 0x3F;
    }
    u8Byte = ((u8Byte & 0xAA) >> 1) + (u8Byte & 0x55);
    u8Byte = ((u8Byte & 0xCC) >> 2) + (u8Byte & 0x33);
    u8Byte = ((u8Byte & 0xF0) >> 4) + (u8Byte & 0x0F);
    u8SubCarrier += u8Byte;
  }
  if (u8SubCarrier == 0) {
    // Protection, this would mean an empty ToneMask
    // Set carriers to default value in Cen-A
    u8SubCarrier = 36;
  }
  uint8_t u8MacNFch = (((u8MacFcSize + 6) * (2 * 6)) + u8SubCarrier - 1) / u8SubCarrier;
  uint32_t u32NPreTime = (u32MacPreambleSymbolTime * u8MacNPreTimesTwo) / 2;
  uint32_t u32NFchTime = u32MacSymbolTime * u8MacNFch;
  uint32_t u32CoherentTime = u32MacSymbolTime * 2;
  g_u32MacSlotTime = u32MacSymbolTime * 2;
  g_u32MacCifsTime = u32MacSymbolTime * u8MacCifs;
  g_u32MacCifsRetransmitTime = u8MacCifsRetransmit * u8MacCifs;
  g_u32MacRifsTime = u32MacSymbolTime * u8MacRifs;
  g_u32MacIdleSpacingTime = u32MacSymbolTime * u8MacIdleSpacing;
  uint32_t u32MacMaxFrameSizeTime = u32NFchTime + u32CoherentTime + (u32MacSymbolTime * u8MacMaxFrameSize);
  g_u32MacAckTime = u32NPreTime + u32NFchTime;
  g_u32MacEifsTime = u32MacMaxFrameSizeTime + g_u32MacRifsTime + g_u32MacCifsTime + g_u32MacAckTime;
  g_u32MacAckWaitDuration = g_u32MacRifsTime + g_u32MacAckTime + g_u32MacCifsTime;
  InitBand(u8Band);
}
