#ifndef __CONFIG_H__
#define __CONFIG_H__

#define NUM_CARRIERS_CENELEC_A                     36
#define NUM_CARRIERS_FCC                           72
#define NUM_CARRIERS_ARIB                          54
#define NUM_CARRIERS_CENELEC_B                     16

enum EMacLowFrameType {
	MAC_LOW_FRAME_TYPE_BEACON = 0x00,
	MAC_LOW_FRAME_TYPE_DATA = 0x01,
	MAC_LOW_FRAME_TYPE_ACKNOWLEDGMENT = 0x02,
	MAC_LOW_FRAME_TYPE_MAC_COMMAND = 0x03,
	MAC_LOW_FRAME_TYPE_RESERVED_4 = 0x04,
	MAC_LOW_FRAME_TYPE_RESERVED_5 = 0x05,
	MAC_LOW_FRAME_TYPE_RESERVED_6 = 0x06,
	MAC_LOW_FRAME_TYPE_RESERVED_7 = 0x07,
};

enum EMacSecurityLevel {
	MAC_SECURITY_LEVEL_NONE = 0x00,
	MAC_SECURITY_LEVEL_ENC_MIC_32 = 0x05,
};

#endif