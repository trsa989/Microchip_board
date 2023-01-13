/**********************************************************************************************************************/
/** \addtogroup MacSublayer
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/** This file contains the MAC MIB.
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/

#ifndef MAC_MIB_H_
#define MAC_MIB_H_

#include <MacDefs.h>

#define MAC_KEY_TABLE_ENTRIES (2)

// Mac layer Table Pointers and Sizes
struct TMacTables {
  // Pointers
  struct TNeighbourEntry *m_NeighbourTable;
  struct TPOSEntry *m_PosTable;
  struct TDeviceTableEntry *m_DeviceTable;
  struct TDsnShortTableEntry *m_DsnShortTable;
  struct TDsnExtendedTableEntry *m_DsnExtendedTable;
  // Sizes
  uint16_t m_MacNeighbourTableSize;
  uint16_t m_MacPosTableSize;
  uint16_t m_MacDeviceTableSize;
  uint16_t m_MacDsnShortTableSize;
  uint16_t m_MacDsnExtendedTableSize;
};

#pragma pack(push,1)
struct TMacTxCoef {
  uint8_t m_au8TxCoef[6];
};

struct TNeighbourEntry {
  TShortAddress m_nShortAddress;
  struct TToneMap m_ToneMap;
  uint8_t m_nModulationType : 3;
  uint8_t m_nTxGain : 4;
  uint8_t m_nTxRes : 1;
  struct TMacTxCoef m_TxCoef;
  uint8_t m_nModulationScheme : 1;
  uint8_t m_nPhaseDifferential : 3;
  uint8_t m_u8Lqi;
  uint16_t m_u16TmrValidTime;
  uint16_t m_u16NeighbourValidTime;
};

struct TPOSEntry {
  TShortAddress m_nShortAddress;
  uint8_t m_u8Lqi;
  uint16_t m_u16POSValidTime;
};

struct TDeviceTableEntry {
  TShortAddress m_nShortAddress;
  uint32_t m_u32FrameCounter;
};

struct TDsnShortTableEntry {
  TShortAddress m_nShortAddress;
  uint8_t m_u8Dsn;
};

struct TDsnExtendedTableEntry {
  struct TExtendedAddress m_ExtendedAddress;
  uint8_t m_u8Dsn;
};
#pragma pack(pop)

struct TMacMib {
  uint32_t m_u32TxDataPacketCount;
  uint32_t m_u32RxDataPacketCount;
  uint32_t m_u32TxCmdPacketCount;
  uint32_t m_u32RxCmdPacketCount;
  uint32_t m_u32CsmaFailCount;
  uint32_t m_u32RxDataBroadcastCount;
  uint32_t m_u32TxDataBroadcastCount;
  struct TNeighbourEntry *m_aNeighbourTable;
  uint16_t m_u16NeighbourTableSize;
  struct TPOSEntry *m_aPOSTable;
  uint16_t m_u16PosTableSize;
  struct TDeviceTableEntry *m_aDeviceTable;
  uint16_t m_u16DeviceTableSize;
  struct TDsnShortTableEntry *m_aDsnShortTable;
  struct TDsnExtendedTableEntry *m_aDsnExtendedTable;
  uint16_t m_u16DsnShortTableSize;
  uint16_t m_u16DsnExtendedTableSize;
  bool m_bFreqNotching;
  uint8_t m_u8TmrTtl;
  uint8_t m_u8NeighbourPOSTableEntryTtl;
  uint16_t m_u16RcCoord;
  uint8_t m_u8BeaconRandomizationWindowLength;
  uint8_t m_u8Bsn;
  uint8_t m_u8Dsn;
  bool m_bCoordinator;
  TPanId m_nPanId;
  struct TExtendedAddress m_ExtendedAddress;
  TShortAddress m_nShortAddress;
  bool m_bPromiscuousMode;
  struct TMacSecurityKey m_aKeyTable[MAC_KEY_TABLE_ENTRIES];
  uint32_t m_u32FrameCounter;
  TShortAddress m_nCoordShortAddress;
  uint8_t m_u8ForcedModSchemeOnTMResponse;
  uint8_t m_u8ForcedModTypeOnTMResponse;
  struct TToneMap m_ForcedToneMapOnTMResponse;
  enum EModulationScheme m_LastRxModScheme;
  enum EModulationType m_LastRxModType;
  bool m_bLBPFrameReceived;
  bool m_bLNGFrameReceived;
  bool m_bBCNFrameReceived;
  uint32_t m_u32RxOtherDestinationCount;
  uint32_t m_u32RxInvalidFrameLengthCount;
  uint32_t m_u32RxMACRepetitionCount;
  uint32_t m_u32RxWrongAddrModeCount;
  uint32_t m_u32RxUnsupportedSecurityCount;
  uint32_t m_u32RxWrongKeyIdCount;
  uint32_t m_u32RxInvalidKeyCount;
  uint32_t m_u32RxWrongFCCount;
  uint32_t m_u32RxDecryptionErrorCount;
  bool m_bMacSniffer;
  uint8_t m_MacSpecCompliance;
};

enum EMacPibAttribute {
  MAC_PIB_ACK_WAIT_DURATION = 0x00000040,
  MAC_PIB_MAX_BE = 0x00000047,
  MAC_PIB_BSN = 0x00000049,
  MAC_PIB_DSN = 0x0000004C,
  MAC_PIB_MAX_CSMA_BACKOFFS = 0x0000004E,
  MAC_PIB_MIN_BE = 0x0000004F,
  MAC_PIB_PAN_ID = 0x00000050,
  MAC_PIB_PROMISCUOUS_MODE = 0x00000051,
  MAC_PIB_SHORT_ADDRESS = 0x00000053,
  MAC_PIB_MAX_FRAME_RETRIES = 0x00000059,
  MAC_PIB_TIMESTAMP_SUPPORTED = 0x0000005C,
  MAC_PIB_SECURITY_ENABLED = 0x0000005D,
  MAC_PIB_KEY_TABLE = 0x00000071,
  MAC_PIB_FRAME_COUNTER = 0x00000077,
  MAC_PIB_HIGH_PRIORITY_WINDOW_SIZE = 0x00000100,
  MAC_PIB_TX_DATA_PACKET_COUNT = 0x00000101,
  MAC_PIB_RX_DATA_PACKET_COUNT = 0x00000102,
  MAC_PIB_TX_CMD_PACKET_COUNT = 0x00000103,
  MAC_PIB_RX_CMD_PACKET_COUNT = 0x00000104,
  MAC_PIB_CSMA_FAIL_COUNT = 0x00000105,
  MAC_PIB_CSMA_NO_ACK_COUNT = 0x00000106,
  MAC_PIB_RX_DATA_BROADCAST_COUNT = 0x00000107,
  MAC_PIB_TX_DATA_BROADCAST_COUNT = 0x00000108,
  MAC_PIB_BAD_CRC_COUNT = 0x00000109,
  MAC_PIB_NEIGHBOUR_TABLE = 0x0000010A,
  MAC_PIB_FREQ_NOTCHING = 0x0000010B,
  MAC_PIB_CSMA_FAIRNESS_LIMIT = 0x0000010C,
  MAC_PIB_TMR_TTL = 0x0000010D,
  MAC_PIB_NEIGHBOUR_TABLE_ENTRY_TTL = 0x0000010E, // Used in Spec15
  MAC_PIB_POS_TABLE_ENTRY_TTL = 0x0000010E,       // Used in Spec17
  MAC_PIB_RC_COORD = 0x0000010F,
  MAC_PIB_TONE_MASK = 0x00000110,
  MAC_PIB_BEACON_RANDOMIZATION_WINDOW_LENGTH = 0x00000111,
  MAC_PIB_A = 0x00000112,
  MAC_PIB_K = 0x00000113,
  MAC_PIB_MIN_CW_ATTEMPTS = 0x00000114,
  MAC_PIB_CENELEC_LEGACY_MODE = 0x00000115,
  MAC_PIB_FCC_LEGACY_MODE = 0x00000116,
  MAC_PIB_BROADCAST_MAX_CW_ENABLE = 0x0000011E,
  MAC_PIB_TRANSMIT_ATTEN = 0x0000011F,
  MAC_PIB_POS_TABLE = 0x00000120,
  // manufacturer specific
  // provides access to device table
  MAC_PIB_MANUF_DEVICE_TABLE = 0x08000000,
  // Extended address of this node.
  MAC_PIB_MANUF_EXTENDED_ADDRESS = 0x08000001,
  // provides access to neighbour table by short address (transmitted as index)
  MAC_PIB_MANUF_NEIGHBOUR_TABLE_ELEMENT = 0x08000002,
  // returns the maximum number of tones used by the band
  MAC_PIB_MANUF_BAND_INFORMATION = 0x08000003,
  // Short address of the coordinator.
  MAC_PIB_MANUF_COORD_SHORT_ADDRESS = 0x08000004,
  // Maximal payload supported by MAC.
  MAC_PIB_MANUF_MAX_MAC_PAYLOAD_SIZE = 0x08000005,
  // Resets the device table upon a GMK activation.
  MAC_PIB_MANUF_SECURITY_RESET = 0x08000006,
  // Forces Modulation Scheme in every transmitted frame
  // 0 - Not forced, 1 - Force Differential, 2 - Force Coherent
  MAC_PIB_MANUF_FORCED_MOD_SCHEME = 0x08000007,
  // Forces Modulation Type in every transmitted frame
  // 0 - Not forced, 1 - Force BPSK_ROBO, 2 - Force BPSK, 3 - Force QPSK, 4 - Force 8PSK
  MAC_PIB_MANUF_FORCED_MOD_TYPE = 0x08000008,
  // Forces ToneMap in every transmitted frame
  // {0} - Not forced, other value will be used as tonemap
  MAC_PIB_MANUF_FORCED_TONEMAP = 0x08000009,
  // Forces Modulation Scheme bit in Tone Map Response
  // 0 - Not forced, 1 - Force Differential, 2 - Force Coherent
  MAC_PIB_MANUF_FORCED_MOD_SCHEME_ON_TMRESPONSE = 0x0800000A,
  // Forces Modulation Type bits in Tone Map Response
  // 0 - Not forced, 1 - Force BPSK_ROBO, 2 - Force BPSK, 3 - Force QPSK, 4 - Force 8PSK
  MAC_PIB_MANUF_FORCED_MOD_TYPE_ON_TMRESPONSE = 0x0800000B,
  // Forces ToneMap field Tone Map Response
  // {0} - Not forced, other value will be used as tonemap field
  MAC_PIB_MANUF_FORCED_TONEMAP_ON_TMRESPONSE = 0x0800000C,
  // Gets Modulation Scheme of last received frame
  MAC_PIB_MANUF_LAST_RX_MOD_SCHEME = 0x0800000D,
  // Gets Modulation Scheme of last received frame
  MAC_PIB_MANUF_LAST_RX_MOD_TYPE = 0x0800000E,
  // Indicates whether an LBP frame for other destination has been received
  MAC_PIB_MANUF_LBP_FRAME_RECEIVED = 0x0800000F,
  // Indicates whether an LBP frame for other destination has been received
  MAC_PIB_MANUF_LNG_FRAME_RECEIVED = 0x08000010,
  // Indicates whether an Beacon frame from other nodes has been received
  MAC_PIB_MANUF_BCN_FRAME_RECEIVED = 0x08000011,
  // Gets number of valid elements in the Neighbour Table
  MAC_PIB_MANUF_NEIGHBOUR_TABLE_COUNT = 0x08000012,
  // Gets number of discarded packets due to Other Destination
  MAC_PIB_MANUF_RX_OTHER_DESTINATION_COUNT = 0x08000013,
  // Gets number of discarded packets due to Invalid Frame Lenght
  MAC_PIB_MANUF_RX_INVALID_FRAME_LENGTH_COUNT = 0x08000014,
  // Gets number of discarded packets due to MAC Repetition
  MAC_PIB_MANUF_RX_MAC_REPETITION_COUNT = 0x08000015,
  // Gets number of discarded packets due to Wrong Addressing Mode
  MAC_PIB_MANUF_RX_WRONG_ADDR_MODE_COUNT = 0x08000016,
  // Gets number of discarded packets due to Unsupported Security
  MAC_PIB_MANUF_RX_UNSUPPORTED_SECURITY_COUNT = 0x08000017,
  // Gets number of discarded packets due to Wrong Key Id
  MAC_PIB_MANUF_RX_WRONG_KEY_ID_COUNT = 0x08000018,
  // Gets number of discarded packets due to Invalid Key
  MAC_PIB_MANUF_RX_INVALID_KEY_COUNT = 0x08000019,
  // Gets number of discarded packets due to Wrong Frame Counter
  MAC_PIB_MANUF_RX_WRONG_FC_COUNT = 0x0800001A,
  // Gets number of discarded packets due to Decryption Error
  MAC_PIB_MANUF_RX_DECRYPTION_ERROR_COUNT = 0x0800001B,
  // Gets number of discarded packets due to Segment Decode Error
  MAC_PIB_MANUF_RX_SEGMENT_DECODE_ERROR_COUNT = 0x0800001C,
  // Enables MAC Sniffer
  MAC_PIB_MANUF_ENABLE_MAC_SNIFFER = 0x0800001D,
  // Gets number of valid elements in the POS Table. Unused in SPEC-15
  MAC_PIB_MANUF_POS_TABLE_COUNT = 0x0800001E,
  // Gets or Sets number of retires left before forcing ROBO mode
  MAC_PIB_MANUF_RETRIES_LEFT_TO_FORCE_ROBO = 0x0800001F,
  // Gets internal MAC version
  MAC_PIB_MANUF_MAC_INTERNAL_VERSION = 0x08000021,
  // Gets internal MAC RT version
  MAC_PIB_MANUF_MAC_RT_INTERNAL_VERSION = 0x08000022,
  // Resets MAC statistics
  MAC_PIB_MANUF_RESET_MAC_STATS = 0x08000023,
  // Enable/Disable Sleep Mode
  MAC_PIB_MANUF_MAC_RT_SLEEP_MODE = 0x08000024,
  // Set PLC Debug Mode
  MAC_PIB_MANUF_MAC_RT_DEBUG_SET = 0x08000025,
  // Read PL360 Debug Information
  MAC_PIB_MANUF_MAC_RT_DEBUG_READ = 0x08000026,
  // Provides access to POS table by short address (referenced as index)
  MAC_PIB_MANUF_POS_TABLE_ELEMENT = 0x08000027,
  // Gets or sets a parameter in Phy layer. Index will be used to contain PHY parameter ID.
  // Check 'enum EPhyParam' in MacRtMib.h for available Phy parameter IDs
  MAC_PIB_MANUF_PHY_PARAM = 0x08000020
};

#define MAC_PIB_MAX_VALUE_LENGTH (144)

struct TMacPibValue {
  uint8_t m_u8Length;
  uint8_t m_au8Value[MAC_PIB_MAX_VALUE_LENGTH];
};

#endif

/**********************************************************************************************************************/
/** @}
 **********************************************************************************************************************/
