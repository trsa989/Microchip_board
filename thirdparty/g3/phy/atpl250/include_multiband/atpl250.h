/**
 * \file
 *
 * \brief ATPL250 G3 Physical layer
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef ATPL250_H_INCLUDED
#define ATPL250_H_INCLUDED

#include "conf_fw.h"
#include "atpl250_reg.h"
#include "pplc_if.h"

#define UNUSED(v)          (void)(v)
/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \defgroup plc_group PLC
 *
 * This module provides configuration and utils for Powerline Communications.
 */

/**
 * \ingroup plc_group
 * \defgroup phy_plc_group G3 Physical Layer
 *
 * This module provides configuration and utils for the PLC PHY layer interface.
 *
 * @{
 */

#ifdef ENABLE_PHY_TRACES
	#define LOG_PHY(a)   printf a
#else
	#define LOG_PHY(a)   (void)0
#endif

/* ! Max RS syndrome size */
#define MAX_RS_SYNDROME_SIZE  16

/* ! Maximum physical payload size */
#define PHY_MAX_PAYLOAD_SIZE                    255

/* ! FCH length in bytes for each bandplan */
#define FCH_LEN_CENELEC_A          5
#define FCH_LEN_FCC                9
#define FCH_LEN_ARIB               9

/* ! Maximum physical pdu size */
/* Buffer holds the syndrome as well as the parity, (2*) is for 2 RS blocks */
#define PHY_MAX_PPDU_SIZE             (FCH_LEN_FCC + 2 * (PHY_MAX_PAYLOAD_SIZE + MAX_RS_SYNDROME_SIZE))

/* ! Minimum physical pdu size */
#define PHY_MIN_PAYLOAD_SIZE       4

/* ! Tone Map sizes for each bandplan */
#define TONE_MAP_SIZE_CENELEC_A    1
#define TONE_MAP_SIZE_FCC          3
#define TONE_MAP_SIZE_ARIB         3

/* ! First, Last, and Number of carriers in Cenelec-A bandplan */
#define FIRST_CARRIER_CENELEC_A    23
#define LAST_CARRIER_CENELEC_A     58
#define NUM_CARRIERS_CENELEC_A     36

/* ! First, Last, and Number of carriers in FCC bandplan */
#define FIRST_CARRIER_FCC          33
#define LAST_CARRIER_FCC           104
#define NUM_CARRIERS_FCC           72

/* ! First, Last, and Number of carriers in ARIB bandplan */
#define FIRST_CARRIER_ARIB         33
#define LAST_CARRIER_ARIB          86
#define NUM_CARRIERS_ARIB          54

/* ! Number of subbands for each bandplan */
#define NUM_SUBBANDS_CENELEC_A     6
#define NUM_SUBBANDS_FCC           24
#define NUM_SUBBANDS_ARIB          18

/* ! \name Working Band */
enum working_band {
	WB_CENELEC_A = 0, WB_CENELEC_B = 1, WB_FCC = 2, WB_ARIB = 3
};

/* ! \name Modulation types */
enum mod_types {
	MOD_TYPE_BPSK = 0,
	MOD_TYPE_QPSK = 1,
	MOD_TYPE_8PSK = 2,
	MOD_TYPE_QAM = 3,
	MOD_TYPE_BPSK_ROBO = 4
};

/* ! \name Modulation schemes */
enum mod_schemes {
	MOD_SCHEME_DIFFERENTIAL = 0,
	MOD_SCHEME_COHERENT = 1
};

/* ! \name Frame Delimiter Types */
enum delimiter_types {
	DT_SOF_NO_RESP = 0, /* ! Data frame requiring ACK */
	DT_SOF_RESP = 1, /* ! Data frame Not requiring ACK */
	DT_ACK = 2, /* ! Positive ACK */
	DT_NACK = 3 /* ! Negative ACK */
};

/* ! \name TX Mode flags */
/* @{ */
/* ! TX Mode: Not Forced Not Delayed transmission, Carrier Sense before transmission, transmission stars immediately after request */
#define TX_MODE_NOT_FORCED_NOT_DELAYED_TX   0x00
/* ! TX Mode: Forced transmission, Noo Carrier Sense before transmission, transmission stars immediately after request */
#define TX_MODE_FORCED_TX                   0x01
/* ! TX Mode: Delayed transmission, Carrier Sense before transmission, transmission in a specified instant in time */
#define TX_MODE_DELAYED_TX                  0x02
/* ! TX Mode: Forced and Delayed transmission, No Carrier Sense, transmission in a specified instant in time */
#define TX_MODE_FORCED_AND_DELAYED_TX       0x03
/* @} */

/* ! \name Commands to access configuration parameters */
/* @{ */
/* ! Read operation */
#define PHY_CMD_CFG_READ           0
/* ! Write operation */
#define PHY_CMD_CFG_WRITE          1
/* ! AND operation */
#define PHY_CMD_CFG_AND            2
/* ! OR operation */
#define PHY_CMD_CFG_OR             3
/* ! XOR operation */
#define PHY_CMD_CFG_XOR            4
/* @} */

/* ! \name TX Result values */
enum tx_result_values {
	PHY_TX_RESULT_PROCESS = 0,     /* /< Transmission result: already in process */
	PHY_TX_RESULT_SUCCESS = 1,     /* /< Transmission result: end successfully */
	PHY_TX_RESULT_INV_LENGTH = 2,  /* /< Transmission result: invalid length error */
	PHY_TX_RESULT_BUSY_CH = 3,     /* /< Transmission result: busy channel error */
	PHY_TX_RESULT_BUSY_TX = 4,     /* /< Transmission result: busy in transmission error */
	PHY_TX_RESULT_BUSY_RX = 5,     /* /< Transmission result: busy in reception error */
	PHY_TX_RESULT_INV_SCHEME = 6,  /* /< Transmission result: invalid modulation scheme error */
	PHY_TX_RESULT_TIMEOUT = 7,     /* /< Transmission result: timeout error */
	PHY_TX_RESULT_INV_TONEMAP = 8, /* /< Transmission result: invalid tone map error */
	PHY_TX_RESULT_NO_TX = 255      /* /< Transmission result: No transmission ongoing */
};

/* ! \name Configuration return values */
/* @{ */
/* ! Set configuration result: success */
#define PHY_CFG_SUCCESS                 0
/* ! Set configuration result: invalid input error or read only */
#define PHY_CFG_INVALID_INPUT           1
/* ! Set configuration result: read only */
#define PHY_CFG_READ_ONLY               2
/* @} */

/* ! \name Configuration Parameters */
/* @{ */
enum e_phy_cfg_params {
	/* ! Product identifier */
	PHY_ID_INFO_PRODUCT                 = 0x0100,
	/* ! Model identifier */
	PHY_ID_INFO_MODEL                   = 0x010A,
	/* ! Version identifier */
	PHY_ID_INFO_VERSION                 = 0x010C,
	/* ! Transmitted correctly messages count */
	PHY_ID_TX_TOTAL                     = 0x0110,
	/* ! Transmitted bytes count */
	PHY_ID_TX_TOTAL_BYTES               = 0x0114,
	/* ! Transmission errors count */
	PHY_ID_TX_TOTAL_ERRORS              = 0x0118,
	/* ! Transmission failure due to already in transmission */
	PHY_ID_BAD_BUSY_TX                  = 0x011C,
	/* ! Transmission failure due to busy channel */
	PHY_ID_TX_BAD_BUSY_CHANNEL          = 0x0120,
	/* ! Bad len in message (too short - too long) */
	PHY_ID_TX_BAD_LEN                   = 0x0124,
	/* ! Message to transmit in bad format */
	PHY_ID_TX_BAD_FORMAT                = 0x0128,
	/* ! Timeout error in transmission */
	PHY_ID_TX_TIMEOUT                   = 0x012C,
	/* ! Received correctly messages count */
	PHY_ID_RX_TOTAL                     = 0x0130,
	/* ! Received bytes count */
	PHY_ID_RX_TOTAL_BYTES               = 0x0134,
	/* ! Reception RS errors count */
	PHY_ID_RX_RS_ERRORS                 = 0x0138,
	/* ! Reception Exceptions count */
	PHY_ID_RX_EXCEPTIONS                = 0x013C,
	/* ! Bad len in message (too short - too long) */
	PHY_ID_RX_BAD_LEN                   = 0x0140,
	/* ! Bad CRC in received FCH */
	PHY_ID_RX_BAD_CRC_FCH               = 0x0144,
	/* ! CRC correct but invalid protocol */
	PHY_ID_RX_FALSE_POSITIVE            = 0x0148,
	/* ! Received message in bad format */
	PHY_ID_RX_BAD_FORMAT                = 0x014C,
	/* ! Total time with free channel */
	PHY_ID_TIME_FREELINE                = 0x0150,
	/* ! Total time with busy channel */
	PHY_ID_TIME_BUSYLINE                = 0x0154,
	/* ! Time between noise captures (in ms) */
	PHY_ID_TIME_BETWEEN_NOISE_CAPTURES  = 0x0158,
	/* ! Number of symbols for the last tx msg */
	PHY_ID_LAST_TX_MSG_PAYLOAD_SYMBOLS  = 0x015C,
	/* ! Number of symbols for the last rx msg */
	PHY_ID_LAST_RX_MSG_PAYLOAD_SYMBOLS  = 0x015E,
	/* ! Number of symbols of fch */
	PHY_ID_FCH_SYMBOLS                  = 0x0160,
	/* ! Auto detect impedance */
	PHY_ID_CFG_AUTODETECT_IMPEDANCE     = 0x0161,
	/* ! manual impedance configuration */
	PHY_ID_CFG_IMPEDANCE                = 0x0162,
	/* ! RRC NOTCH active */
	PHY_ID_RRC_NOTCH_ACTIVE             = 0x0163,
	/* ! RRC NOTCH filter index */
	PHY_ID_RRC_NOTCH_INDEX              = 0x0164,
	/* ! RRC NOTCH filter autodetect */
	PHY_ID_RRC_NOTCH_AUTODETECT         = 0x0165,
	/* ! RRC NOTCH filter autodetect */
	PHY_ID_ENABLE_AUTO_NOISE_CAPTURE    = 0x0166,
	/* ! RRC NOTCH filter autodetect */
	PHY_ID_DELAY_NOISE_CAPTURE_AFTER_RX = 0x0167,
	/* ! Value for Legacy Mode Indicator */
	PHY_ID_LEGACY_MODE                  = 0x0168,
	/* ! Value for Static Notching */
	PHY_ID_STATIC_NOTCHING              = 0x0169,
	/* ! Disable PLC Tx and Rx */
	PHY_ID_PLC_DISABLE                  = 0x016A,
	/* ! Indicate noise power in dBuV for the noisier carrier */
	PHY_ID_NOISE_PEAK_POWER             = 0x016B,
	/* ! LQI value of the last received message */
	PHY_ID_LAST_MSG_LQI                 = 0x016C,
	/* ! Trigger signal dump */
	PHY_ID_TRIGGER_SIGNAL_DUMP          = 0x016F,
	/* ! Trigger signal dump */
	PHY_ID_LAST_RMSCALC_CORRECTED       = 0x0170,
	/* ! Inform PHY layer about enabled modulations on TMR */
	PHY_ID_TONE_MAP_RSP_ENABLED_MODS    = 0x0174,
	/* ! Force no Output Signal */
	PHY_ID_FORCE_NO_OUTPUT_SIGNAL       = 0x0175,
	/* ! Reset Phy Statistics */
	PHY_ID_RESET_PHY_STATS              = 0x0176
};

/* Valid PHY PIB range defined */
#define PHY_PIB_LOWEST_VALUE                    0x0100
#define PHY_PIB_HIGHEST_VALUE                   0x0176

/* @} */

/* ! \name Structure defining PHY parameters */
typedef struct _atpl250_t {
	char m_ac_prod_id[10];                        /* /< Product identifier */
	uint16_t m_us_model;                          /* /< Model number */
	uint32_t m_ul_version;                        /* /< Version number */

	uint32_t m_ul_tx_total;                       /* /< Transmitted correctly messages count */
	uint32_t m_ul_tx_total_bytes;                 /* /< Transmitted bytes count */
	uint32_t m_ul_tx_total_errors;                /* /< Transmission errors count */
	uint32_t m_ul_tx_bad_busy_tx;                 /* /< Transmission failure due to already in transmission */
	uint32_t m_ul_tx_bad_busy_channel;            /* /< Transmission failure due to busy channel */
	uint32_t m_ul_tx_bad_len;                     /* /< Bad len in message (too short - too long) */
	uint32_t m_ul_tx_bad_format;                  /* /< Message to transmit in bad format */
	uint32_t m_ul_tx_timeout;                     /* /< Timeout error in transmission */
	uint32_t m_ul_rx_total;                       /* /< Received correctly messages count */
	uint32_t m_ul_rx_total_bytes;                 /* /< Received bytes count */
	uint32_t m_ul_rx_RS_errors;                   /* /< Reception Reed Solomon errors count */
	uint32_t m_ul_rx_exception;                   /* /< Others Reception errors count */
	uint32_t m_ul_rx_bad_len;                     /* /< Bad len in message (too short - too long) */
	uint32_t m_ul_rx_bad_crc_fch;                 /* /< Bad CRC in received FCH */
	uint32_t m_ul_rx_false_positive;              /* /< CRC correct but invalid protocol */
	uint32_t m_ul_rx_bad_format;                  /* /< Received message in bad format */
	uint32_t m_ul_time_freeline;                  /* /< Total time with free channel */
	uint32_t m_ul_time_busyline;                  /* /< Total time with busy channel */
	uint32_t m_ul_time_between_noise_captures;    /* /< Time between automatic noise captures, in ms */

	uint16_t m_us_last_tx_msg_payload_symbols;    /* /< Number of symbols of last message trasnmitted */
	uint16_t m_us_last_rx_msg_payload_symbols;    /* /< Number of symbols of last message received */
	uint8_t m_uc_fch_symbols;                     /* /< Number of symbols of fch */
	uint8_t m_uc_auto_detect_impedance;           /* /< Flag to enable impedance auto detection */
	uint8_t m_uc_impedance_state;                 /* /< When impedance auto detection disabled, indicate impedance to use */
	uint8_t m_uc_rrc_notch_active;                /* /< Indicate if notch is active */
	uint8_t m_uc_rrc_notch_index;                 /* /< Index of the active filter */
	uint8_t m_uc_rrc_notch_autodetect;            /* /< Trigger notch filter autodetection */
	uint8_t m_uc_enable_auto_noise_capture;       /* /< Enable automatic noise captures to adapt Phy layer to noise conditions */
	uint8_t m_uc_delay_noise_capture_after_rx;    /* /< If True, each correct Rx frame delays next noise capture */
	uint8_t m_uc_reserved1;                       /* /< Reserved byte */
	uint8_t m_uc_reserved2;                       /* /< Reserved byte */
	uint8_t m_uc_plc_disable;                     /* /< Disable PLC Tx and Rx */
	uint8_t m_uc_noise_peak_power;                /* /< Indicate noise power in dBuV for the noisier carrier */
	uint8_t m_uc_last_msg_lqi;                    /* /< LQI value of the last received message */
	uint8_t m_uc_padding1[2];                     /* /< Rquired padding */
	uint8_t m_uc_trigger_signal_dump;             /* /< Trigger signal dump */
	uint32_t m_ul_last_rmscalc_corrected;         /* /< Last Rmscalc Corrected */
	uint8_t m_uc_reserved3;                       /* /< Reserved byte */
	uint8_t m_uc_force_no_output_signal;          /* /< Force transmssion without output signal */
	uint8_t m_uc_reserved4;                       /* /< Reserved byte */
} atpl250_t;

/* ! BER Data */
#define PHY_ID_BER_DATA                    0x0200

struct s_rx_ber_payload_data_t {
	uint8_t uc_valid_data;                        /* /< (r) Flag to indicate whether read data is valid */
	uint8_t uc_payload_snr_worst_carrier;         /* /< (r) SNR of the worst carrier */
	uint8_t uc_payload_snr_worst_symbol;          /* /< (r) SNR of the worst symbol */
	uint8_t uc_payload_snr_impulsive;             /* /< (r) SNR on impulsive noise */
	uint8_t uc_payload_snr_be;                    /* /< (r) Narrowband SNR */
	uint8_t uc_payload_snr_background;            /* /< (r) Background SNR */
	uint16_t us_payload_corrupted_carriers;       /* /< (r) Number of corrupted carriers */
	uint16_t us_payload_noised_symbols;           /* /< (r) Number of noised symbols */
	uint8_t uc_lqi;                               /* /< (r)Link quality indicator */
	uint8_t auc_carrier_snr[NUM_CARRIERS_FCC];    /* /< (r) SNR per carrier */
};

/* ! ToneMap Response Data */
#define PHY_ID_TM_RESPONSE_DATA            0x0280

struct s_tone_map_response_data_t {
	enum mod_types e_mod_type;
	enum mod_schemes e_mod_scheme;
	uint8_t m_auc_tone_map[TONE_MAP_SIZE_FCC];
};

/* ! \name Structure defining Tx message */
typedef struct _xPhyMsgTx_t {
	uint8_t m_uc_buff_id;                         /* /< Buffer identifier */
	uint8_t m_uc_tx_mode;                         /* /< Transmission Mode (forced, delayed, ...). Constants above */
	uint8_t m_uc_tx_power;                        /* /< Power to transmit [0 = Full gain, 1 = (Full gain - 3dB), 2 = (Full gain - 6dB) and so on] */
	enum mod_types e_mod_type;                    /* /< Modulation type. Constants above */
	enum mod_schemes e_mod_scheme;                /* /< Modulation scheme. Constants above */
	uint8_t m_uc_pdc;                             /* /< Phase Detector Counter. Calculated and filled internally by PHY layer */
	uint8_t m_auc_tone_map[TONE_MAP_SIZE_FCC];    /* /< Tone Map to use on transmission */
	uint8_t m_uc_2_rs_blocks;                     /* /< Flag to indicate whether 2 RS blocks have to be used (only used in FCC bandplan) */
	uint8_t m_auc_preemphasis[NUM_SUBBANDS_FCC];  /* /< Preemphasis for transmission. Same as m_uc_tx_power but for each subband */
	enum delimiter_types e_delimiter_type;        /* /< DT field to be used in header. Constants above */
	uint8_t *m_puc_data_buf;                      /* /< Pointer to data buffer */
	uint16_t m_us_data_len;                       /* /< Length of the data buffer */
	uint32_t m_ul_tx_time;                        /* /< Instant when transmission has to start referred to 1us PHY counter (Absolute value, not relative).
	                                               * Only used when m_uc_tx_mode is Delayed transmission */
} xPhyMsgTx_t;

/* ! \name Structure defining Rx message */
typedef struct _xPhyMsgRx_t {
	uint8_t m_uc_buff_id;                         /* /< Buffer identifier */
	enum mod_types e_mod_type;                    /* /< Modulation type of the last received frame. Constants above */
	enum mod_schemes e_mod_scheme;                /* /< Modulation scheme of the last received frame. Constants above */
	uint8_t m_auc_tone_map[TONE_MAP_SIZE_FCC];    /* /< Tone Map in received frame */
	uint16_t m_us_evm_header;                     /* /< EVM on header reception */
	uint16_t m_us_evm_payload;                    /* /< EVM on payload reception */
	uint16_t m_us_rssi;                           /* /< Reception RSSI */
	uint16_t m_us_agc_factor;                     /* /< AGC factor */
	uint8_t m_uc_zct_diff;                        /* /< Phase difference with transmitting node */
	enum delimiter_types e_delimiter_type;        /* /< DT field coming in header. Constants above */
	uint8_t m_uc_rs_corrected_errors;             /* /< Errors corrected by RS (errors == 0xff means could not correct) */
	uint8_t *m_puc_data_buf;                      /* /< Pointer to data buffer containing received frame */
	uint16_t m_us_data_len;                       /* /< Length of received frame */
	uint32_t m_ul_rx_time;                        /* /< Instant when frame was received (end of frame) referred to 1us PHY counter */
	uint32_t m_ul_frame_duration;                 /* /< Frame duration in us (Preamble + FCH + Payload) */
} xPhyMsgRx_t;

/* ! \name Structure defining result of a transmission */
typedef struct _xPhyMsgTxResult_t {
	uint8_t m_uc_id_buffer;                  /* /< Buffer to which result refers to */
	enum tx_result_values e_tx_result;       /* /< Tx Result. Constants above */
	uint32_t m_ul_rms_calc;                  /* /< RMS_CALC value after transmission. Allows to estimate tx power injected */
	uint32_t m_ul_end_tx_time;               /* /< Instant when frame transmission ended referred to 1us PHY counter */
} xPhyMsgTxResult_t;

/* Macros to distinguish between configuration parameters */
#define ATPL250_REG_PARAM(val)       (val & 0x0400)
#define ATPL250_PARAM(val)           (val & 0x0F00)
#define ATPL250_EXTENDED_PARAM(val)  (val & 0x0FF0)

#define ATPL250_PARAM_MSK            0x0100u
#define ATPL250_BER_PARAM_MSK        0x0200u
#define ATPL250_TM_RESP_PARAM_MSK    0x0280u

/* ! \name Structure defining constants which depend on working band */
struct band_phy_constants {
	uint32_t ul_txrx_time;
	uint32_t ul_txrx_time_us;
	uint32_t ul_txrx_plc;
	uint32_t ul_txrx_plc_us;
	uint32_t ul_end_tx_offset_us;
	uint32_t ul_rrc_delay_us;
	uint32_t ul_frame_symbol_duration;
	uint32_t ul_half_symbol_duration;
	uint32_t ul_preamble_duration;
	uint16_t us_fch_interleaver_useful_size;
	uint8_t uc_fch_len;
	uint8_t uc_fch_len_bits;
	uint8_t uc_tonemap_size;
	uint8_t uc_first_carrier;
	uint8_t uc_last_carrier;
	uint8_t uc_last_used_carrier;
	uint8_t uc_num_carriers;
	uint8_t uc_num_subbands;
	uint8_t uc_num_carriers_in_subband;
	uint8_t uc_pilot_offset;
	uint8_t uc_tx_fft_shift;
	uint8_t uc_rx_fft_shift;
	uint8_t uc_emit_gain_hi;
	uint8_t uc_emit_gain_lo;
	uint8_t uc_emit_gain_vlo;
	uint8_t uc_pay_sym_first_demod;
	uint8_t uc_min_sym_for_offset_correction_dif;
	uint8_t uc_min_sym_for_offset_correction_coh;
	uint8_t uc_noise_capture_adapt_symbols;
	uint8_t uc_noise_correction_factor_db;
	uint8_t uc_min_noise_notch_db;
};

/* Phy layer Callbacks */

struct TPhyCallbacks;
struct TPhySnifferCallbacks;

void phy_set_callbacks(struct TPhyCallbacks *p_callbacks);
void phy_set_sniffer_callbacks(struct TPhySnifferCallbacks *p_callbacks);
void phy_clear_sniffer_callbacks(void);

typedef void (*phy_pd_data_confirm)(xPhyMsgTxResult_t *px_tx_result);
typedef void (*phy_pd_data_indication)(xPhyMsgRx_t *px_msg);
typedef void (*phy_pd_sniffer_data_confirm)(xPhyMsgTxResult_t *px_tx_result, xPhyMsgTx_t *px_tx_param);

struct TPhyCallbacks {
	phy_pd_data_confirm m_p_data_confirm;
	phy_pd_data_indication m_p_data_indication;
};

struct TPhySnifferCallbacks {
	phy_pd_sniffer_data_confirm m_p_sniffer_data_confirm;
	phy_pd_data_indication m_p_sniffer_data_indication;
};

/* ! \name ATPL250 Physical Layer Interface */
/* @{ */
void phy_init(uint8_t uc_serial_enable, uint8_t uc_band);
uint8_t phy_get_cfg_param(uint16_t us_id, void *p_val, uint16_t us_len);
uint8_t phy_set_cfg_param(uint16_t us_id, void *p_val, uint16_t us_len);
uint8_t phy_cmd_cfg_param(uint16_t us_id, uint8_t uc_cmd, uint8_t uc_mask);
uint8_t phy_tx_frame(xPhyMsgTx_t *px_msg);
void phy_reset_request(void);
uint8_t phy_process(void);

/* @} */

/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* ATPL250_H_INCLUDED */
