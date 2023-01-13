#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pal.h"
#include "atpl250_reg.h"
#include "atpl250.h"

/* Set Request Parameters */
struct TPlmeSetRequest g_plme_set_request_params;
/* Get Request Parameters */
struct TPlmeGetRequest g_plme_get_request_params;
/* Get Confirm parameters */
struct TPlmeGetConfirm g_plme_get_confirm_params;

/* Working band */
static uint8_t uc_band;
static uint8_t uc_tonemap_size;
static uint8_t uc_first_carrier;
static uint8_t uc_last_carrier;
static uint8_t uc_num_carriers;
static uint8_t uc_num_subbands;

/* ACK buffer */
#define ACK_SIZE  FCH_LEN_FCC
static uint8_t auc_ack[ACK_SIZE];
static uint8_t uc_ack_len;

/* PHY Serialization */
#define PHY_SERIALIZATION_ENABLE   1
#define PHY_SERIALIZATION_DISABLE  0

/* Structure for the association tables between PIB and size */
typedef struct {
	uint16_t us_pib;
	uint8_t uc_size;
} pib_size_t;

#define PIB_TABLE_SIZE 41

/* Association table for PIBs */
static const pib_size_t pib_table[PIB_TABLE_SIZE] = {
	{PHY_ID_INFO_PRODUCT, sizeof(uint8_t) * 10},
	{PHY_ID_INFO_MODEL, sizeof(uint16_t)},
	{PHY_ID_INFO_VERSION, sizeof(uint32_t)},
	{PHY_ID_TX_TOTAL, sizeof(uint32_t)},
	{PHY_ID_TX_TOTAL_BYTES, sizeof(uint32_t)},
	{PHY_ID_TX_TOTAL_ERRORS, sizeof(uint32_t)},
	{PHY_ID_BAD_BUSY_TX, sizeof(uint32_t)},
	{PHY_ID_TX_BAD_BUSY_CHANNEL, sizeof(uint32_t)},
	{PHY_ID_TX_BAD_LEN, sizeof(uint32_t)},
	{PHY_ID_TX_BAD_FORMAT, sizeof(uint32_t)},
	{PHY_ID_TX_TIMEOUT, sizeof(uint32_t)},
	{PHY_ID_RX_TOTAL, sizeof(uint32_t)},
	{PHY_ID_RX_TOTAL_BYTES, sizeof(uint32_t)},
	{PHY_ID_RX_RS_ERRORS, sizeof(uint32_t)},
	{PHY_ID_RX_EXCEPTIONS, sizeof(uint32_t)},
	{PHY_ID_RX_BAD_LEN, sizeof(uint32_t)},
	{PHY_ID_RX_BAD_CRC_FCH, sizeof(uint32_t)},
	{PHY_ID_RX_FALSE_POSITIVE, sizeof(uint32_t)},
	{PHY_ID_RX_BAD_FORMAT, sizeof(uint32_t)},
	{PHY_ID_TIME_FREELINE, sizeof(uint32_t)},
	{PHY_ID_TIME_BUSYLINE, sizeof(uint32_t)},
	{PHY_ID_TIME_BETWEEN_NOISE_CAPTURES, sizeof(uint32_t)},
	{PHY_ID_LAST_TX_MSG_PAYLOAD_SYMBOLS, sizeof(uint16_t)},
	{PHY_ID_LAST_RX_MSG_PAYLOAD_SYMBOLS, sizeof(uint16_t)},
	{PHY_ID_FCH_SYMBOLS, sizeof(uint8_t)},
	{PHY_ID_CFG_AUTODETECT_IMPEDANCE, sizeof(uint8_t)},
	{PHY_ID_CFG_IMPEDANCE, sizeof(uint8_t)},
	{PHY_ID_RRC_NOTCH_ACTIVE, sizeof(uint8_t)},
	{PHY_ID_RRC_NOTCH_INDEX, sizeof(uint8_t)},
	{PHY_ID_RRC_NOTCH_AUTODETECT, sizeof(uint8_t)},
	{PHY_ID_ENABLE_AUTO_NOISE_CAPTURE, sizeof(uint8_t)},
	{PHY_ID_DELAY_NOISE_CAPTURE_AFTER_RX, sizeof(uint8_t)},
	{PHY_ID_LEGACY_MODE, sizeof(uint8_t)},
	{PHY_ID_STATIC_NOTCHING, sizeof(uint8_t)},
	{PHY_ID_PLC_DISABLE, sizeof(uint8_t)},
	{PHY_ID_NOISE_PEAK_POWER, sizeof(uint8_t)},
	{PHY_ID_LAST_MSG_LQI, sizeof(uint8_t)},
	{PHY_ID_TRIGGER_SIGNAL_DUMP, sizeof(uint8_t)},
	{PHY_ID_LAST_RMSCALC_CORRECTED, sizeof(uint32_t)},
	{PHY_ID_TONE_MAP_RSP_ENABLED_MODS, sizeof(uint8_t)},
	{PHY_ID_BER_DATA, sizeof(struct s_rx_ber_payload_data_t)},
};

/* PAL callbacks */
struct TPhyNotifications g_phy_notifications = {0};

/* Control of last Tx request */
static bool last_tx_request_is_ack;

static void pal_cb_phy_data_confirm(xPhyMsgTxResult_t *px_tx_result)
{
	struct TPdAckConfirm ack_confirm_params;
	struct TPdDataConfirm data_confirm_params;

	/* Check wheher last Tx request was for ACK or Data */
	if (last_tx_request_is_ack) {
		/* ACK Confirm */
		LOG_PHY(("ACK Confirm (%u)\r\n", px_tx_result->e_tx_result));
		if (g_phy_notifications.m_pPdAckConfirm != NULL) {
			switch (px_tx_result->e_tx_result) {
			case PHY_TX_RESULT_PROCESS:
				ack_confirm_params.m_eStatus = PHY_STATUS_BUSY;
				break;

			case PHY_TX_RESULT_SUCCESS:
				ack_confirm_params.m_eStatus = PHY_STATUS_SUCCESS;
				break;

			case PHY_TX_RESULT_INV_LENGTH:
			case PHY_TX_RESULT_INV_SCHEME:
			case PHY_TX_RESULT_INV_TONEMAP:
				ack_confirm_params.m_eStatus = PHY_STATUS_INVALID_PARAMETER;
				break;

			case PHY_TX_RESULT_BUSY_CH:
				ack_confirm_params.m_eStatus = PHY_STATUS_BUSY;
				break;

			case PHY_TX_RESULT_BUSY_TX:
				ack_confirm_params.m_eStatus = PHY_STATUS_BUSY_TX;
				break;

			case PHY_TX_RESULT_BUSY_RX:
				ack_confirm_params.m_eStatus = PHY_STATUS_BUSY_RX;
				break;

			case PHY_TX_RESULT_TIMEOUT:
				ack_confirm_params.m_eStatus = PHY_STATUS_BUSY_TX;
				break;

			case PHY_TX_RESULT_NO_TX:
				ack_confirm_params.m_eStatus = PHY_STATUS_IDLE;
				break;
			}
			ack_confirm_params.m_u32Time = px_tx_result->m_ul_end_tx_time;
			g_phy_notifications.m_pPdAckConfirm(&ack_confirm_params);
		}
	} else {
		/* Data Confirm */
		LOG_PHY(("Data confirm, %u\r\n", px_tx_result->e_tx_result));
		if (g_phy_notifications.m_pPdDataConfirm != NULL) {
			switch (px_tx_result->e_tx_result) {
			case PHY_TX_RESULT_PROCESS:
				data_confirm_params.m_eStatus = PHY_STATUS_BUSY;
				break;

			case PHY_TX_RESULT_SUCCESS:
				data_confirm_params.m_eStatus = PHY_STATUS_SUCCESS;
				break;

			case PHY_TX_RESULT_INV_LENGTH:
			case PHY_TX_RESULT_INV_SCHEME:
			case PHY_TX_RESULT_INV_TONEMAP:
				data_confirm_params.m_eStatus = PHY_STATUS_INVALID_PARAMETER;
				break;

			case PHY_TX_RESULT_BUSY_CH:
				data_confirm_params.m_eStatus = PHY_STATUS_BUSY;
				break;

			case PHY_TX_RESULT_BUSY_TX:
				data_confirm_params.m_eStatus = PHY_STATUS_BUSY_TX;
				break;

			case PHY_TX_RESULT_BUSY_RX:
				data_confirm_params.m_eStatus = PHY_STATUS_BUSY_RX;
				break;

			case PHY_TX_RESULT_TIMEOUT:
				data_confirm_params.m_eStatus = PHY_STATUS_BUSY_TX;
				break;

			case PHY_TX_RESULT_NO_TX:
				data_confirm_params.m_eStatus = PHY_STATUS_IDLE;
				break;
			}
			data_confirm_params.m_u32Time = px_tx_result->m_ul_end_tx_time;
			g_phy_notifications.m_pPdDataConfirm(&data_confirm_params);
		}
	}
}

static void pal_cb_phy_data_indication(xPhyMsgRx_t *px_msg)
{
	uint8_t uc_i;
	struct TPdAckIndication ack_indication_params;
	struct TPdDataIndication data_indication_params;

	g_plme_get_request_params.m_u8Dummy = 0; /* TODO: Implement parameters if necessary */
	g_plme_get_confirm_params.m_u8PhaseDifferential = px_msg->m_uc_zct_diff;
	g_plme_get_confirm_params.m_u8Dt = px_msg->e_delimiter_type;
	switch (px_msg->e_mod_scheme) {
	case MOD_SCHEME_DIFFERENTIAL:
		g_plme_get_confirm_params.m_eModulationScheme = PHY_MODULATION_SCHEME_DIFFERENTIAL;
		break;

	case MOD_SCHEME_COHERENT:
		g_plme_get_confirm_params.m_eModulationScheme = PHY_MODULATION_SCHEME_COHERENT;
		break;
	}
	switch (px_msg->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
		g_plme_get_confirm_params.m_eModulationType = PHY_MODULATION_ROBUST;
		break;

	case MOD_TYPE_BPSK:
		g_plme_get_confirm_params.m_eModulationType = PHY_MODULATION_DBPSK_BPSK;
		break;

	case MOD_TYPE_QPSK:
		g_plme_get_confirm_params.m_eModulationType = PHY_MODULATION_DQPSK_QPSK;
		break;

	case MOD_TYPE_8PSK:
		g_plme_get_confirm_params.m_eModulationType = PHY_MODULATION_D8PSK_8PSK;
		break;

	case MOD_TYPE_QAM:
		g_plme_get_confirm_params.m_eModulationType = PHY_MODULATION_16_QAM;
		break;
	}
	memset(&g_plme_get_confirm_params.m_ToneMap, 0, sizeof(g_plme_get_confirm_params.m_ToneMap));
	for (uc_i = 0; uc_i < uc_tonemap_size; uc_i++) {
		g_plme_get_confirm_params.m_ToneMap.m_au8Tm[uc_i] = px_msg->m_auc_tone_map[uc_i];
	}

	/* Check whether frame is data or ACK */
	if ((px_msg->e_delimiter_type == DT_ACK) || (px_msg->e_delimiter_type == DT_NACK)) {
		/* ACK frame. Call Notification function */
		if (g_phy_notifications.m_pPdAckIndication != NULL) {
			ack_indication_params.m_AckFch.m_eDelimiterType = (enum EPhyDelimiterType)(px_msg->e_delimiter_type);
			ack_indication_params.m_u32Time = px_msg->m_ul_rx_time;
			if ((uc_band == PHY_BAND_CENELEC_A) || (uc_band == PHY_BAND_CENELEC_B)) {
				ack_indication_params.m_AckFch.m_u16Fcs = ((uint16_t)px_msg->m_puc_data_buf[2] << 8) | px_msg->m_puc_data_buf[0];
				ack_indication_params.m_AckFch.m_u8Ssca = (px_msg->m_puc_data_buf[1] & 0x80) >> 7;
			} else {
				ack_indication_params.m_AckFch.m_u16Fcs = ((uint16_t)px_msg->m_puc_data_buf[3] << 8) | px_msg->m_puc_data_buf[0];
				ack_indication_params.m_AckFch.m_u8Ssca = (px_msg->m_puc_data_buf[1] & 0x80) >> 7;
			}

			g_phy_notifications.m_pPdAckIndication(&ack_indication_params);
		}
	} else {
		/* Data frame. Call Notification function */
		if (g_phy_notifications.m_pPdDataIndication != NULL) {
			data_indication_params.m_pPsdu = px_msg->m_puc_data_buf;
			data_indication_params.m_u32Time = px_msg->m_ul_rx_time;
			data_indication_params.m_u8Dt = (uint8_t)px_msg->e_delimiter_type;
			data_indication_params.m_u16PsduLength = px_msg->m_us_data_len;

			LOG_PHY(("m_pPdDataIndication\r\n"));

			g_phy_notifications.m_pPdDataIndication(&data_indication_params);
		}
	}
}

static struct TPhyCallbacks g_pal_phy_callbacks = {
	pal_cb_phy_data_confirm,
	pal_cb_phy_data_indication
};

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyInitialize(struct TPhyNotifications *pNotifications, uint8_t u8Band)
{
	uc_band = u8Band;

	if (uc_band == PHY_BAND_ARIB) {
		uc_ack_len = FCH_LEN_ARIB;
		uc_tonemap_size = TONE_MAP_SIZE_ARIB;
		uc_first_carrier = FIRST_CARRIER_ARIB;
		uc_last_carrier = LAST_CARRIER_ARIB;
		uc_num_carriers = NUM_CARRIERS_ARIB;
		uc_num_subbands = NUM_SUBBANDS_ARIB;
	} else if (uc_band == PHY_BAND_FCC) {
		uc_ack_len = FCH_LEN_FCC;
		uc_tonemap_size = TONE_MAP_SIZE_FCC;
		uc_first_carrier = FIRST_CARRIER_FCC;
		uc_last_carrier = LAST_CARRIER_FCC;
		uc_num_carriers = NUM_CARRIERS_FCC;
		uc_num_subbands = NUM_SUBBANDS_FCC;
	} else {
		uc_ack_len = FCH_LEN_CENELEC_A;
		uc_tonemap_size = TONE_MAP_SIZE_CENELEC_A;
		uc_first_carrier = FIRST_CARRIER_CENELEC_A;
		uc_last_carrier = LAST_CARRIER_CENELEC_A;
		uc_num_carriers = NUM_CARRIERS_CENELEC_A;
		uc_num_subbands = NUM_SUBBANDS_CENELEC_A;
	}

	g_phy_notifications = *pNotifications;
	/* Set phy callbacks */
	phy_set_callbacks(&g_pal_phy_callbacks);
	/* Init Phy Layer */
	phy_init(PHY_SERIALIZATION_DISABLE, u8Band);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyEventHandler(void)
{
	uint8_t uc_phy_process_result;
	uint16_t us_loop_count = 0;

	/* Call Phy Process to check events */
	uc_phy_process_result = phy_process();
	while (uc_phy_process_result) {
		uc_phy_process_result = phy_process();
		if (us_loop_count++ > 50000) {
			break;
		}
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyPdDataRequest(struct TPdDataRequest *pParameters)
{
	uint8_t uc_i;
	xPhyMsgTx_t x_tx_msg;

	LOG_PHY(("PhyPdDataRequest\r\n"));

	last_tx_request_is_ack = false;

	x_tx_msg.m_uc_pdc = 0;
	x_tx_msg.m_uc_buff_id = 0;
	x_tx_msg.e_delimiter_type = (enum delimiter_types)(g_plme_set_request_params.m_u8Dt);
	switch (g_plme_set_request_params.m_TxParameters.m_eModulationType) {
	case PHY_MODULATION_ROBUST:
		x_tx_msg.e_mod_type = MOD_TYPE_BPSK_ROBO;
		break;

	case PHY_MODULATION_DBPSK_BPSK:
		x_tx_msg.e_mod_type = MOD_TYPE_BPSK;
		break;

	case PHY_MODULATION_DQPSK_QPSK:
		x_tx_msg.e_mod_type = MOD_TYPE_QPSK;
		break;

	case PHY_MODULATION_D8PSK_8PSK:
		x_tx_msg.e_mod_type = MOD_TYPE_8PSK;
		break;

	case PHY_MODULATION_16_QAM:
		x_tx_msg.e_mod_type = MOD_TYPE_QAM;
		break;
	}
	x_tx_msg.e_mod_scheme = (enum mod_schemes)g_plme_set_request_params.m_TxParameters.m_eModulationScheme;
	memcpy(x_tx_msg.m_auc_preemphasis, g_plme_set_request_params.m_TxParameters.m_PreEmphasis.m_au8PreEmphasis, uc_num_subbands);
	if (pParameters->m_bDelayed) {
		x_tx_msg.m_uc_tx_mode = TX_MODE_DELAYED_TX;
	} else {
		x_tx_msg.m_uc_tx_mode = 0x00;
	}

	x_tx_msg.m_uc_tx_power = g_plme_set_request_params.m_TxParameters.m_u8TxPower;
	x_tx_msg.m_ul_tx_time = pParameters->m_u32Time;
	for (uc_i = 0; uc_i < uc_tonemap_size; uc_i++) {
		x_tx_msg.m_auc_tone_map[uc_i] = g_plme_set_request_params.m_TxParameters.m_ToneMap.m_au8Tm[uc_i];
	}
	if (uc_band == PHY_BAND_FCC) {
		x_tx_msg.m_uc_2_rs_blocks = g_plme_set_request_params.m_TxParameters.m_u8TwoRSBlocks;
	} else {
		x_tx_msg.m_uc_2_rs_blocks = 0;
	}

	x_tx_msg.m_us_data_len = pParameters->m_u16PsduLength;
	x_tx_msg.m_puc_data_buf = pParameters->m_pPsdu;

	/* pParameters->m_bPerformCs Not used, implement it */
	/* g_plme_set_request_params.m_TxParameters.m_ToneMask Not used, fixed at compilation */

	phy_tx_frame(&x_tx_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyPdAckRequest(struct TPdAckRequest *pParameters)
{
	xPhyMsgTx_t x_tx_msg;

	LOG_PHY(("PhyPdAckRequest\r\n"));

	last_tx_request_is_ack = true;

	x_tx_msg.m_uc_pdc = 0; /* Not used in ACK */
	x_tx_msg.m_uc_buff_id = 0;
	x_tx_msg.e_delimiter_type = (enum delimiter_types)(pParameters->m_AckFch.m_eDelimiterType);
	x_tx_msg.e_mod_type = MOD_TYPE_BPSK_ROBO; /* Not used in ACK */
	x_tx_msg.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL; /* Not used in ACK */
	memset(x_tx_msg.m_auc_preemphasis, 0x00, uc_num_subbands); /* Not used in ACK */
	if (pParameters->m_bDelayed) {
		x_tx_msg.m_uc_tx_mode = TX_MODE_DELAYED_TX;
	} else {
		x_tx_msg.m_uc_tx_mode = 0x00;
	}

	x_tx_msg.m_uc_tx_power = pParameters->m_u8TxPower;
	x_tx_msg.m_ul_tx_time = pParameters->m_u32Time;
	if (uc_band == PHY_BAND_CENELEC_A) {
		x_tx_msg.m_auc_tone_map[0] = 0x3F; /* Not used in ACK */
	} else if (uc_band == PHY_BAND_CENELEC_B) {
		x_tx_msg.m_auc_tone_map[0] = 0x0F; /* Not used in ACK */
	} else {
		x_tx_msg.m_auc_tone_map[0] = 0xFF; /* Not used in ACK */
		x_tx_msg.m_auc_tone_map[1] = 0xFF; /* Not used in ACK */
		x_tx_msg.m_auc_tone_map[2] = 0xFF; /* Not used in ACK */
	}

	x_tx_msg.m_us_data_len = uc_ack_len;
	memset(auc_ack, 0, ACK_SIZE);
	if ((uc_band == PHY_BAND_CENELEC_A) || (uc_band == PHY_BAND_CENELEC_B)) {
		auc_ack[0] = (uint8_t)(pParameters->m_AckFch.m_u16Fcs);
		auc_ack[1] = (pParameters->m_AckFch.m_u8Ssca << 7);
		auc_ack[2] = (uint8_t)(pParameters->m_AckFch.m_u16Fcs >> 8);
		auc_ack[3] = (pParameters->m_AckFch.m_eDelimiterType << 4);
	} else {
		auc_ack[0] = (uint8_t)(pParameters->m_AckFch.m_u16Fcs);
		auc_ack[1] = (pParameters->m_AckFch.m_u8Ssca << 7) | (pParameters->m_AckFch.m_eDelimiterType << 1);
		auc_ack[3] = (uint8_t)(pParameters->m_AckFch.m_u16Fcs >> 8);
	}

	x_tx_msg.m_uc_2_rs_blocks = 0;
	x_tx_msg.m_puc_data_buf = auc_ack;

	phy_tx_frame(&x_tx_msg);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyPlmeSetRequest(struct TPlmeSetRequest *pParameters)
{
	g_plme_set_request_params = *pParameters;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyPlmeGetRequest(struct TPlmeGetRequest *pParameters)
{
	struct s_rx_ber_payload_data_t s_ber_data;
	uint16_t us_j;

	UNUSED(pParameters);

	/* Get Ber data */
	phy_get_cfg_param(PHY_ID_BER_DATA, (void *)&s_ber_data, sizeof(s_ber_data));
	us_j = 0;
	while (s_ber_data.uc_valid_data == 0) {
		/* Protection from trap */
		us_j++;
		if (us_j == 50000) {
			break;
		}

		/* keep asking until valid data is obtained */
		phy_get_cfg_param(PHY_ID_BER_DATA, (void *)&s_ber_data, sizeof(s_ber_data));
	}
	if (us_j < 50000) {
		/* Fill in structure with ber data */
		g_plme_get_confirm_params.m_u8PpduLinkQuality = s_ber_data.uc_lqi;
		memcpy(g_plme_get_confirm_params.m_u8CarrierSnr, s_ber_data.auc_carrier_snr, uc_num_carriers);
	} else {
		g_plme_get_confirm_params.m_u8PpduLinkQuality = 0;
		memset(g_plme_get_confirm_params.m_u8CarrierSnr, 0, uc_num_carriers);
	}

	if (g_phy_notifications.m_pPlmeGetConfirm != NULL) {
		g_phy_notifications.m_pPlmeGetConfirm(&g_plme_get_confirm_params);
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyPlmeSetTrxStateRequest(struct TPlmeSetTrxStateRequest *pParameters)
{
	UNUSED(pParameters);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyPlmeCsRequest(struct TPlmeCsRequest *pParameters)
{
	UNUSED(pParameters);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyPlmeResetRequest(void)
{
	phy_reset_request();
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
uint32_t PhyGetTime(void)
{
	uint32_t phy_time;

	phy_get_cfg_param(REG_ATPL250_TX_TIMER_REF_32, &phy_time, 4);

	return phy_time;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void PhyGetToneMapResponseData(struct TPlmeGetToneMapResponseData *pParameters)
{
	struct s_tone_map_response_data_t tm_response_data;
	uint8_t uc_i;

	phy_get_cfg_param(PHY_ID_TM_RESPONSE_DATA, &tm_response_data, sizeof(struct s_tone_map_response_data_t));

	switch (tm_response_data.e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
		pParameters->m_eModulationType = PHY_MODULATION_ROBUST;
		break;

	case MOD_TYPE_BPSK:
		pParameters->m_eModulationType = PHY_MODULATION_DBPSK_BPSK;
		break;

	case MOD_TYPE_QPSK:
		pParameters->m_eModulationType = PHY_MODULATION_DQPSK_QPSK;
		break;

	case MOD_TYPE_8PSK:
		pParameters->m_eModulationType = PHY_MODULATION_D8PSK_8PSK;
		break;

	case MOD_TYPE_QAM:
		pParameters->m_eModulationType = PHY_MODULATION_16_QAM;
		break;
	}
	switch (tm_response_data.e_mod_scheme) {
	case MOD_SCHEME_DIFFERENTIAL:
		pParameters->m_eModulationScheme = PHY_MODULATION_SCHEME_DIFFERENTIAL;
		break;

	case MOD_SCHEME_COHERENT:
		pParameters->m_eModulationScheme = PHY_MODULATION_SCHEME_COHERENT;
		break;
	}
	memset(&pParameters->m_ToneMap, 0, sizeof(pParameters->m_ToneMap));
	for (uc_i = 0; uc_i < uc_tonemap_size; uc_i++) {
		pParameters->m_ToneMap.m_au8Tm[uc_i] = tm_response_data.m_auc_tone_map[uc_i];
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
uint8_t PhyGetPIBLen(uint16_t us_id)
{
	uint8_t uc_idx;

	/* Find PIB in table */
	for (uc_idx = 0; uc_idx < PIB_TABLE_SIZE; uc_idx++) {
		if (pib_table[uc_idx].us_pib == us_id) {
			return pib_table[uc_idx].uc_size;
		}
	}

	/* PIB not found. Return invalid length */
	return 0;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
enum EPhyGetSetResult PhyGetParam(uint16_t us_id, void *p_val, uint16_t us_len)
{
	uint8_t uc_result;

	uc_result = phy_get_cfg_param(us_id, p_val, us_len);

	if (uc_result == PHY_CFG_SUCCESS) {
		return PHY_GETSET_RESULT_OK;
	} else {
		return PHY_GETSET_RESULT_INVALID_PARAM;
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
enum EPhyGetSetResult PhySetParam(uint16_t us_id, void *p_val, uint16_t us_len)
{
	uint8_t uc_result;

	uc_result = phy_set_cfg_param(us_id, p_val, us_len);

	switch (uc_result) {
	case PHY_CFG_SUCCESS:
		return PHY_GETSET_RESULT_OK;

	case PHY_CFG_INVALID_INPUT:
		return PHY_GETSET_RESULT_INVALID_PARAM;

	case PHY_CFG_READ_ONLY:
		return PHY_GETSET_RESULT_READ_ONLY;

	default:
		return PHY_GETSET_RESULT_INVALID_PARAM;
	}
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
#define MAC_TONE_MASK_LEN         9
#define PHY_NOTCHED_CARRIERS_LEN  16
void PhySetToneMask(uint8_t *puc_tone_mask)
{
	uint8_t auc_phy_notched_carriers[PHY_NOTCHED_CARRIERS_LEN];
	uint8_t uc_i, uc_j;
	uint8_t uc_byte_idx_phy;
	uint8_t uc_bit_idx_phy;
	uint8_t uc_byte_idx_mac;
	uint8_t uc_bit_idx_mac;

	/* Tone Mask from MAC is 9 byte long, to hold 72 carriers */
	/* It has to be translated to the format that PHY understands */

	memset(auc_phy_notched_carriers, 0, PHY_NOTCHED_CARRIERS_LEN);

	for (uc_i = uc_first_carrier, uc_j = 0; uc_i <= uc_last_carrier; uc_i++, uc_j++) {
		/* Calculate byte and bit position for Phy and Mac arrays */
		uc_byte_idx_phy = uc_i >> 3;
		uc_bit_idx_phy = uc_i & 0x07;
		uc_byte_idx_mac = uc_j >> 3;
		uc_bit_idx_mac = uc_j & 0x07;

		if (!(puc_tone_mask[uc_byte_idx_mac] & (1 << uc_bit_idx_mac))) {
			auc_phy_notched_carriers[uc_byte_idx_phy] |= (1 << uc_bit_idx_phy);
		}
	}

	phy_set_cfg_param(PHY_ID_STATIC_NOTCHING, (void *)auc_phy_notched_carriers, PHY_NOTCHED_CARRIERS_LEN);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
uint8_t PhyGetLegacyMode(void)
{
	uint8_t uc_legacy;

	phy_get_cfg_param(PHY_ID_LEGACY_MODE, (void *)&uc_legacy, 1);

	return uc_legacy;
}
