#include <stdbool.h>
#include <stdint.h>
#include <MacPhyInterface.h>
#include <MacRtMib.h>
#include <MacRtConstants.h>
#include <MacRtVersion.h>
#include <string.h>

uint8_t u8RtMibSpecCompliance = MAC_RT_SPEC_COMPLIANCE_17;

struct TMacRtMib g_MacRtMib;

static const struct TMacRtMib g_MacRtMibDefaults = {
  0, // m_u32CsmaNoAckCount
  0, // m_u32BadCrcCount
  0, // m_u32RxSegmentDecodeErrorCount
  0xFFFF, // m_u16PanId
  0xFFFF, // m_u16ShortAddress
  {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}}, // m_ToneMask
  {{0}}, // m_ExtendedAddress
  {{0}}, // m_ForcedToneMap
  7, // m_u8HighPriorityWindowSize
  15, // m_u8CsmaFairnessLimit
  8, // m_u8A
  5, // m_u8K
  10, // m_u8MinCwAttempts
  8, // m_u8MaxBe
  50, // m_u8MaxCsmaBackoffs
  5, // m_u8MaxFrameRetries
  3, // m_u8MinBe
  0, // m_u8ForcedModScheme
  0, // m_u8ForcedModType
  0, // m_u8RetriesToForceRobo
  0, // m_u8TransmitAtten
  false, // m_bBroadcastMaxCwEnable
  false, // m_bCoordinator
};

struct TMacRtSoftVersion {
  uint8_t m_u8Major;
  uint8_t m_u8Minor;
  uint8_t m_u8Revision;
  uint8_t m_u8Year; // year since 2000
  uint8_t m_u8Month;
  uint8_t m_u8Day;
};

static void RtMibResetInternal(void)
{
  g_MacRtMib = g_MacRtMibDefaults;
  g_MacRtMib.m_ToneMask = g_ToneMaskDefault;
}

void MacRtMibReset(void)
{
  RtMibResetInternal();
}

void MacRtMibInitialize(uint8_t u8SpecCompliance)
{
  RtMibResetInternal();
  u8RtMibSpecCompliance = u8SpecCompliance;
}

void MacRtSetCoordinator(void)
{
  g_MacRtMib.m_bCoordinator = true;
}

static enum EMacRtStatus MacPibGetAckWaitDuration(struct TMacRtPibValue *pValue)
{
  uint16_t u16AckWaitDuration = g_u32MacAckWaitDuration;
  pValue->m_u8Length = sizeof(u16AckWaitDuration);
  memcpy(pValue->m_au8Value, &u16AckWaitDuration, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibGetMaxBe(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8MaxBe);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8MaxBe, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetMaxBe(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value <= 20) && (u8Value > g_MacRtMib.m_u8MinBe)) {
    g_MacRtMib.m_u8MaxBe = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetMaxCsmaBackoffs(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8MaxCsmaBackoffs);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8MaxCsmaBackoffs, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetMaxCsmaBackoffs(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_u8MaxCsmaBackoffs)) {
    memcpy(&g_MacRtMib.m_u8MaxCsmaBackoffs, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetMaxFrameRetries(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8MaxFrameRetries);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8MaxFrameRetries, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetMaxFrameRetries(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value <= 10)) {
    g_MacRtMib.m_u8MaxFrameRetries = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetMinBe(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8MinBe);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8MinBe, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetMinBe(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value <= 20) && (u8Value < g_MacRtMib.m_u8MaxBe)) {
    g_MacRtMib.m_u8MinBe = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetPanId(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_nPanId);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_nPanId, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetPanId(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_nPanId)) {
    memcpy(&g_MacRtMib.m_nPanId, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetShortAddress(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_nShortAddress);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_nShortAddress, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetShortAddress(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_nShortAddress)) {
    memcpy(&g_MacRtMib.m_nShortAddress, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetHighPriorityWindowSize(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8HighPriorityWindowSize);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8HighPriorityWindowSize, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetHighPriorityWindowSize(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value >= 1) && (u8Value <= 7)) {
    g_MacRtMib.m_u8HighPriorityWindowSize = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetCsmaNoAckCount(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u32CsmaNoAckCount);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u32CsmaNoAckCount, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetCsmaNoAckCount(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_u32CsmaNoAckCount)) {
    memcpy(&g_MacRtMib.m_u32CsmaNoAckCount, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetBadCrcCount(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u32BadCrcCount);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u32BadCrcCount, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetBadCrcCount(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_u32BadCrcCount)) {
    memcpy(&g_MacRtMib.m_u32BadCrcCount, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetCsmaFairnessLimit(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8CsmaFairnessLimit);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8CsmaFairnessLimit, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetCsmaFairnessLimit(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value >= 2 * (g_MacRtMib.m_u8MaxBe - g_MacRtMib.m_u8MinBe))) {
    g_MacRtMib.m_u8CsmaFairnessLimit = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetToneMask(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_ToneMask);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_ToneMask, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetToneMask(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_ToneMask)) {
    memcpy(&g_MacRtMib.m_ToneMask, pValue->m_au8Value, pValue->m_u8Length);
    MacRtConstantsInitialize(g_PhyBandInformation.m_u8Band, &g_MacRtMib.m_ToneMask);
    PhySetToneMask(&g_MacRtMib.m_ToneMask.m_au8ToneMask[0]);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetBandInformation(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_PhyBandInformation);
  memcpy(pValue->m_au8Value, &g_PhyBandInformation, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibGetA(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8A);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8A, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetA(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_u8A)) {
    uint8_t u8Value;
    memcpy(&u8Value, pValue->m_au8Value, pValue->m_u8Length);
    if ((u8Value >= 3) && (u8Value <= 20)) {
      g_MacRtMib.m_u8A = u8Value;
    }
    else {
      eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
    }
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetK(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8K);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8K, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetK(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value >= 1) && (u8Value <= g_MacRtMib.m_u8CsmaFairnessLimit)) {
    g_MacRtMib.m_u8K = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetMinCwAttempts(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8MinCwAttempts);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8MinCwAttempts, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetMinCwAttempts(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_u8MinCwAttempts)) {
    memcpy(&g_MacRtMib.m_u8MinCwAttempts, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetCenelecLegacyMode(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = 1;
  pValue->m_au8Value[0] = PhyGetLegacyMode();
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibGetFccLegacyMode(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = 1;
  pValue->m_au8Value[0] = PhyGetLegacyMode();
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibGetTransmitAtten(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8TransmitAtten);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8TransmitAtten, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetTransmitAtten(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_u8TransmitAtten)) {
    memcpy(&g_MacRtMib.m_u8TransmitAtten, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetExtendedAddress(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_ExtendedAddress);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_ExtendedAddress, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetExtendedAddress(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_ExtendedAddress)) {
    memcpy(&g_MacRtMib.m_ExtendedAddress, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetForcedModScheme(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8ForcedModScheme);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8ForcedModScheme, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetForcedModScheme(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value <= 2)) {
    g_MacRtMib.m_u8ForcedModScheme = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetForcedModType(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u8ForcedModType);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u8ForcedModType, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetForcedModType(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value <= 4)) {
    g_MacRtMib.m_u8ForcedModType = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetForcedToneMap(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_ForcedToneMap);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_ForcedToneMap, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetForcedToneMap(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_ForcedToneMap)) {
    memcpy(&g_MacRtMib.m_ForcedToneMap, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetBroadcastMaxCwEnable(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_bBroadcastMaxCwEnable);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_bBroadcastMaxCwEnable, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetBroadcastMaxCwEnable(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if ((pValue->m_u8Length == sizeof(u8Value)) && (u8Value <= 1)) {
    g_MacRtMib.m_bBroadcastMaxCwEnable = (u8Value == 1);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetRxSegmentDecodeErrorCount(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(g_MacRtMib.m_u32RxSegmentDecodeErrorCount);
  memcpy(pValue->m_au8Value, &g_MacRtMib.m_u32RxSegmentDecodeErrorCount, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetRxSegmentDecodeErrorCount(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(g_MacRtMib.m_u32RxSegmentDecodeErrorCount)) {
    memcpy(&g_MacRtMib.m_u32RxSegmentDecodeErrorCount, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetRetriesToForceRobo(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = 1;
  pValue->m_au8Value[0] = g_MacRtMib.m_u8RetriesToForceRobo;
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetRetriesToForceRobo(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  uint8_t u8Value;
  memcpy(&u8Value, pValue->m_au8Value, sizeof(u8Value));
  if (pValue->m_u8Length == sizeof(u8Value)) {
    g_MacRtMib.m_u8RetriesToForceRobo = u8Value;
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetMacRtInternalVersion(struct TMacRtPibValue *pValue)
{
  struct TMacRtSoftVersion version = {MAC_RT_VERSION_MAJOR, MAC_RT_VERSION_MINOR, MAC_RT_VERSION_REVISION,
                                      MAC_RT_VERSION_YEAR, MAC_RT_VERSION_MONTH, MAC_RT_VERSION_DAY};

  pValue->m_u8Length = sizeof(struct TMacRtSoftVersion);
  memcpy(pValue->m_au8Value, &version, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibGetAllRtMib(struct TMacRtPibValue *pValue)
{
  pValue->m_u8Length = sizeof(struct TMacRtMib);
  memcpy(pValue->m_au8Value, &g_MacRtMib, pValue->m_u8Length);
  return MAC_RT_STATUS_SUCCESS;
}

static enum EMacRtStatus MacPibSetAllRtMib(const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_SUCCESS;
  if (pValue->m_u8Length == sizeof(struct TMacRtMib)) {
    memcpy(&g_MacRtMib, pValue->m_au8Value, pValue->m_u8Length);
  }
  else {
    eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  }
  return eStatus;
}

static enum EMacRtStatus MacPibGetPhyParam(uint16_t u16Index, struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  enum EPhyGetSetResult eResult;

  switch (u16Index) {
    case PHY_PARAM_VERSION:
    case PHY_PARAM_TX_TOTAL:
    case PHY_PARAM_TX_TOTAL_BYTES:
    case PHY_PARAM_TX_TOTAL_ERRORS:
    case PHY_PARAM_BAD_BUSY_TX:
    case PHY_PARAM_TX_BAD_BUSY_CHANNEL:
    case PHY_PARAM_TX_BAD_LEN:
    case PHY_PARAM_TX_BAD_FORMAT:
    case PHY_PARAM_TX_TIMEOUT:
    case PHY_PARAM_RX_TOTAL:
    case PHY_PARAM_RX_TOTAL_BYTES:
    case PHY_PARAM_RX_RS_ERRORS:
    case PHY_PARAM_RX_EXCEPTIONS:
    case PHY_PARAM_RX_BAD_LEN:
    case PHY_PARAM_RX_BAD_CRC_FCH:
    case PHY_PARAM_RX_FALSE_POSITIVE:
    case PHY_PARAM_RX_BAD_FORMAT:
      pValue->m_u8Length = 4;
      break;
    case PHY_PARAM_ENABLE_AUTO_NOISE_CAPTURE:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_TIME_BETWEEN_NOISE_CAPTURES:
      pValue->m_u8Length = 4;
      break;
    case PHY_PARAM_DELAY_NOISE_CAPTURE_AFTER_RX:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_CFG_AUTODETECT_BRANCH:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_CFG_IMPEDANCE:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_RRC_NOTCH_ACTIVE:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_RRC_NOTCH_INDEX:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_PLC_DISABLE:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_NOISE_PEAK_POWER:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_LAST_MSG_LQI:
      pValue->m_u8Length = 1;
      break;
    case PHY_PARAM_LAST_MSG_RSSI:
    case PHY_PARAM_ACK_TX_CFM:
      pValue->m_u8Length = 2;
      break;
    case PHY_PARAM_TONE_MAP_RSP_ENABLED_MODS:
      pValue->m_u8Length = 1;
      break;
    default:
      pValue->m_u8Length = PhyGetPIBLen(u16Index);
      if (pValue->m_u8Length == 0) {
        return eStatus;
      }
      break;
  }

  /* Call Get param function on PAL */
  eResult = PhyGetParam(u16Index, (void *)(pValue->m_au8Value), (uint16_t)pValue->m_u8Length);

  if (eResult == PHY_GETSET_RESULT_OK) {
    eStatus = MAC_RT_STATUS_SUCCESS;
  }

  return eStatus;
}

static enum EMacRtStatus MacPibSetPhyParam(uint16_t u16Index, const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus = MAC_RT_STATUS_INVALID_PARAMETER;
  enum EPhyGetSetResult eResult = PHY_GETSET_RESULT_INVALID_PARAM;
  uint8_t u8LenFromPAL;

  switch (u16Index) {
    case PHY_PARAM_VERSION:
    case PHY_PARAM_TX_TOTAL:
    case PHY_PARAM_TX_TOTAL_BYTES:
    case PHY_PARAM_TX_TOTAL_ERRORS:
    case PHY_PARAM_BAD_BUSY_TX:
    case PHY_PARAM_TX_BAD_BUSY_CHANNEL:
    case PHY_PARAM_TX_BAD_LEN:
    case PHY_PARAM_TX_BAD_FORMAT:
    case PHY_PARAM_TX_TIMEOUT:
    case PHY_PARAM_RX_TOTAL:
    case PHY_PARAM_RX_TOTAL_BYTES:
    case PHY_PARAM_RX_RS_ERRORS:
    case PHY_PARAM_RX_EXCEPTIONS:
    case PHY_PARAM_RX_BAD_LEN:
    case PHY_PARAM_RX_BAD_CRC_FCH:
    case PHY_PARAM_RX_FALSE_POSITIVE:
    case PHY_PARAM_RX_BAD_FORMAT:
      eStatus = MAC_RT_STATUS_READ_ONLY;
      break;
    case PHY_PARAM_ENABLE_AUTO_NOISE_CAPTURE:
    case PHY_PARAM_TIME_BETWEEN_NOISE_CAPTURES:
    case PHY_PARAM_DELAY_NOISE_CAPTURE_AFTER_RX:
    case PHY_PARAM_CFG_AUTODETECT_BRANCH:
    case PHY_PARAM_CFG_IMPEDANCE:
    case PHY_PARAM_RRC_NOTCH_ACTIVE:
    case PHY_PARAM_RRC_NOTCH_INDEX:
    case PHY_PARAM_PLC_DISABLE:
    case PHY_PARAM_NOISE_PEAK_POWER:
    case PHY_PARAM_LAST_MSG_LQI:
    case PHY_PARAM_LAST_MSG_RSSI:
    case PHY_PARAM_ACK_TX_CFM:
    case PHY_PARAM_TONE_MAP_RSP_ENABLED_MODS:
      /* Call Set param function on PAL */
      eResult = PhySetParam(u16Index, (void *)(pValue->m_au8Value), (uint16_t)pValue->m_u8Length);
      break;
    default:
      /* Get length from PAL */
      u8LenFromPAL = PhyGetPIBLen(u16Index);
      if (u8LenFromPAL != 0) {
        /* Call Set param function on PAL */
        eResult = PhySetParam(u16Index, (void *)(pValue->m_au8Value), (uint16_t)u8LenFromPAL);
      }
      break;
  }

  if (eResult == PHY_GETSET_RESULT_OK) {
    eStatus = MAC_RT_STATUS_SUCCESS;
  }
  else if (eResult == PHY_GETSET_RESULT_READ_ONLY) {
    eStatus = MAC_RT_STATUS_READ_ONLY;
  }

  return eStatus;
}

enum EMacRtStatus MacRtGetRequestSync(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus;
  bool bArray = (eAttribute == MAC_RT_PIB_MANUF_PHY_PARAM);
  if (!bArray && (u16Index != 0)) {
    eStatus = MAC_RT_STATUS_INVALID_INDEX;
  }
  else {
    switch (eAttribute) {
      case MAC_RT_PIB_ACK_WAIT_DURATION:
        eStatus = MacPibGetAckWaitDuration(pValue);
        break;
      case MAC_RT_PIB_MAX_BE:
        eStatus = MacPibGetMaxBe(pValue);
        break;
      case MAC_RT_PIB_MAX_CSMA_BACKOFFS:
        eStatus = MacPibGetMaxCsmaBackoffs(pValue);
        break;
      case MAC_RT_PIB_MAX_FRAME_RETRIES:
        eStatus = MacPibGetMaxFrameRetries(pValue);
        break;
      case MAC_RT_PIB_MIN_BE:
        eStatus = MacPibGetMinBe(pValue);
        break;
      case MAC_RT_PIB_PAN_ID:
        eStatus = MacPibGetPanId(pValue);
        break;
      case MAC_RT_PIB_SHORT_ADDRESS:
        eStatus = MacPibGetShortAddress(pValue);
        break;
      case MAC_RT_PIB_HIGH_PRIORITY_WINDOW_SIZE:
        eStatus = MacPibGetHighPriorityWindowSize(pValue);
        break;
      case MAC_RT_PIB_CSMA_NO_ACK_COUNT:
        eStatus = MacPibGetCsmaNoAckCount(pValue);
        break;
      case MAC_RT_PIB_BAD_CRC_COUNT:
        eStatus = MacPibGetBadCrcCount(pValue);
        break;
      case MAC_RT_PIB_CSMA_FAIRNESS_LIMIT:
        eStatus = MacPibGetCsmaFairnessLimit(pValue);
        break;
      case MAC_RT_PIB_TONE_MASK:
        eStatus = MacPibGetToneMask(pValue);
        break;
      case MAC_RT_PIB_A:
        eStatus = MacPibGetA(pValue);
        break;
      case MAC_RT_PIB_K:
        eStatus = MacPibGetK(pValue);
        break;
      case MAC_RT_PIB_MIN_CW_ATTEMPTS:
        eStatus = MacPibGetMinCwAttempts(pValue);
        break;
      case MAC_RT_PIB_CENELEC_LEGACY_MODE:
        eStatus = MacPibGetCenelecLegacyMode(pValue);
        break;
      case MAC_RT_PIB_FCC_LEGACY_MODE:
        eStatus = MacPibGetFccLegacyMode(pValue);
        break;
      case MAC_RT_PIB_TRANSMIT_ATTEN:
        if (u8RtMibSpecCompliance >= MAC_RT_SPEC_COMPLIANCE_17) {
          eStatus = MacPibGetTransmitAtten(pValue);
        }
        else {
          eStatus = MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE;
        }
        break;
      case MAC_RT_PIB_BROADCAST_MAX_CW_ENABLE:
        if (u8RtMibSpecCompliance >= MAC_RT_SPEC_COMPLIANCE_17) {
          eStatus = MacPibGetBroadcastMaxCwEnable(pValue);
        }
        else {
          eStatus = MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE;
        }
        break;
      case MAC_RT_PIB_MANUF_BAND_INFORMATION:
        eStatus = MacPibGetBandInformation(pValue);
        break;
      case MAC_RT_PIB_MANUF_EXTENDED_ADDRESS:
        eStatus = MacPibGetExtendedAddress(pValue);
        break;
      case MAC_RT_PIB_MANUF_FORCED_MOD_SCHEME:
        eStatus = MacPibGetForcedModScheme(pValue);
        break;
      case MAC_RT_PIB_MANUF_FORCED_MOD_TYPE:
        eStatus = MacPibGetForcedModType(pValue);
        break;
      case MAC_RT_PIB_MANUF_FORCED_TONEMAP:
        eStatus = MacPibGetForcedToneMap(pValue);
        break;
      case MAC_RT_PIB_MANUF_RX_SEGMENT_DECODE_ERROR_COUNT:
        eStatus = MacPibGetRxSegmentDecodeErrorCount(pValue);
        break;
      case MAC_RT_PIB_MANUF_RETRIES_LEFT_TO_FORCE_ROBO:
        eStatus = MacPibGetRetriesToForceRobo(pValue);
        break;
      case MAC_RT_PIB_MANUF_MAC_RT_INTERNAL_VERSION:
        eStatus = MacPibGetMacRtInternalVersion(pValue);
        break;
      case MAC_RT_PIB_GET_SET_ALL_MIB:
        eStatus = MacPibGetAllRtMib(pValue);
        break;
      case MAC_RT_PIB_MANUF_PHY_PARAM:
        eStatus = MacPibGetPhyParam(u16Index, pValue);
        break;

      default:
        eStatus = MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE;
        break;
    }
  }

  if (eStatus != MAC_RT_STATUS_SUCCESS) {
    pValue->m_u8Length = 0;
  }
  return eStatus;
}

enum EMacRtStatus MacRtSetRequestSync(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, const struct TMacRtPibValue *pValue)
{
  enum EMacRtStatus eStatus;
  bool bArray = (eAttribute == MAC_RT_PIB_MANUF_PHY_PARAM);
  if (!bArray && (u16Index != 0)) {
    eStatus = MAC_RT_STATUS_INVALID_INDEX;
  }
  else {
    switch (eAttribute) {
      case MAC_RT_PIB_MAX_BE:
        eStatus = MacPibSetMaxBe(pValue);
        break;
      case MAC_RT_PIB_MAX_CSMA_BACKOFFS:
        eStatus = MacPibSetMaxCsmaBackoffs(pValue);
        break;
      case MAC_RT_PIB_MAX_FRAME_RETRIES:
        eStatus = MacPibSetMaxFrameRetries(pValue);
        break;
      case MAC_RT_PIB_MIN_BE:
        eStatus = MacPibSetMinBe(pValue);
        break;
      case MAC_RT_PIB_PAN_ID:
        eStatus = MacPibSetPanId(pValue);
        break;
      case MAC_RT_PIB_SHORT_ADDRESS:
        eStatus = MacPibSetShortAddress(pValue);
        break;
      case MAC_RT_PIB_HIGH_PRIORITY_WINDOW_SIZE:
        eStatus = MacPibSetHighPriorityWindowSize(pValue);
        break;
      case MAC_RT_PIB_CSMA_NO_ACK_COUNT:
        eStatus = MacPibSetCsmaNoAckCount(pValue);
        break;
      case MAC_RT_PIB_BAD_CRC_COUNT:
        eStatus = MacPibSetBadCrcCount(pValue);
        break;
      case MAC_RT_PIB_CSMA_FAIRNESS_LIMIT:
        eStatus = MacPibSetCsmaFairnessLimit(pValue);
        break;
      case MAC_RT_PIB_TONE_MASK:
        eStatus = MacPibSetToneMask(pValue);
        break;
      case MAC_RT_PIB_A:
        eStatus = MacPibSetA(pValue);
        break;
      case MAC_RT_PIB_K:
        eStatus = MacPibSetK(pValue);
        break;
      case MAC_RT_PIB_MIN_CW_ATTEMPTS:
        eStatus = MacPibSetMinCwAttempts(pValue);
        break;
      case MAC_RT_PIB_MANUF_EXTENDED_ADDRESS:
        eStatus = MacPibSetExtendedAddress(pValue);
        break;
      case MAC_RT_PIB_TRANSMIT_ATTEN:
        if (u8RtMibSpecCompliance >= MAC_RT_SPEC_COMPLIANCE_17) {
          eStatus = MacPibSetTransmitAtten(pValue);
        }
        else {
          eStatus = MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE;
        }
        break;
      case MAC_RT_PIB_BROADCAST_MAX_CW_ENABLE:
        if (u8RtMibSpecCompliance >= MAC_RT_SPEC_COMPLIANCE_17) {
          eStatus = MacPibSetBroadcastMaxCwEnable(pValue);
        }
        else {
          eStatus = MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE;
        }
        break;
      case MAC_RT_PIB_MANUF_FORCED_MOD_SCHEME:
        eStatus = MacPibSetForcedModScheme(pValue);
        break;
      case MAC_RT_PIB_MANUF_FORCED_MOD_TYPE:
        eStatus = MacPibSetForcedModType(pValue);
        break;
      case MAC_RT_PIB_MANUF_FORCED_TONEMAP:
        eStatus = MacPibSetForcedToneMap(pValue);
        break;
      case MAC_RT_PIB_MANUF_RX_SEGMENT_DECODE_ERROR_COUNT:
        eStatus = MacPibSetRxSegmentDecodeErrorCount(pValue);
        break;
      case MAC_RT_PIB_MANUF_RETRIES_LEFT_TO_FORCE_ROBO:
        eStatus = MacPibSetRetriesToForceRobo(pValue);
        break;
      case MAC_RT_PIB_GET_SET_ALL_MIB:
        eStatus = MacPibSetAllRtMib(pValue);
        break;

      case MAC_RT_PIB_MANUF_PHY_PARAM:
        eStatus = MacPibSetPhyParam(u16Index, pValue);
        break;

      case MAC_RT_PIB_ACK_WAIT_DURATION:
      case MAC_RT_PIB_CENELEC_LEGACY_MODE:
      case MAC_RT_PIB_FCC_LEGACY_MODE:
      case MAC_RT_PIB_MANUF_BAND_INFORMATION:
      case MAC_RT_PIB_MANUF_MAC_RT_INTERNAL_VERSION:
        eStatus = MAC_RT_STATUS_READ_ONLY;
        break;
      default:
        eStatus = MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE;
        break;
    }
  }
  return eStatus;
}
