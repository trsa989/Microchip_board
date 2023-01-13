/**
 * \file
 *
 * \brief APP_PLC : Module to manage PL360 G3 MAC RT Layer.
 *
 * Copyright (c) 2020 Atmel Corporation. All rights reserved.
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

/* Atmel library includes. */
#include "asf.h"
#include "conf_project.h"
#include "app_console.h"
#include "app_plc.h"
#include "g3_mac_defs.h"
#include "coup_pl360_wrp.h"

/* Time in ms that LED is ON after message reception */
#define COUNT_MS_IND_LED        50

/* Personal Address Network Identificator */
#define APP_G3_PAN_ID           0x781D

/* ATPL360 descriptor definition */
static atpl360_descriptor_t sx_atpl360_desc;

/* Exception counters */
static uint8_t suc_err_unexpected;
static uint8_t suc_err_critical;
static uint8_t suc_err_reset;
static uint8_t suc_err_none;
static bool sb_exception_pend;
static bool sb_enabling_pl360;

/* G3 Band identifier */
static uint8_t suc_phy_band;

/* G3 Extended Address */
static uint8_t spuc_extended_address[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

/* G3 PIB management */
static struct TMacRtPibValue sx_g3_pib;

/* G3 Transmission management */
static uint8_t spuc_tx_buffer[ATPL360_MAX_PHY_DATA_LENGTH];
static struct TMacRtTxRequest sx_tx_msg;
static struct TMacRtMhr sx_tx_mac_header;

/* G3 Reception management */
static struct TMacRtPlmeGetConfirm sx_rx_parameters;

#ifdef CONF_TONEMASK_STATIC_NOTCHING
#define TONE_MASK_SIZE   (PROTOCOL_CARRIERS_MAX + 7) / 8
/* Tone Mask (Static Notching) array. Each carrier corresponding to the band can be notched (no energy is sent in those carriers) */
/* Each carrier is represented by one byte (0: carrier used; 1: carrier notched). By default it is all 0's in PL360 device */
/* The same Tone Mask must be set in both transmitter and receiver. Otherwise they don't understand each other */
static const uint8_t spuc_tone_mask[TONE_MASK_SIZE] = TONE_MASK_STATIC_NOTCHING;
#endif

/* LED management variables */
extern uint32_t sul_ind_count_ms;

static void _init_random(void)
{
#ifdef TRNG
	/* Configure PMC */
	pmc_enable_periph_clk(ID_TRNG);

#if (!PIC32CX)
	/* Enable TRNG */
	trng_enable(TRNG);
#else
	/* Get peripheral clock frequency */
	uint32_t ul_cpuhz = sysclk_get_peripheral_hz();

	/* Enable TRNG */
	trng_enable(TRNG, ul_cpuhz);
#endif
#else
	uint32_t ul_random_num;

	ul_random_num = DWT->CYCCNT;

	/* Initialize seed */
	srand(ul_random_num);
#endif
}

static uint32_t _get_random_32(void)
{
#ifdef TRNG
	while ((trng_get_interrupt_status(TRNG) & TRNG_ISR_DATRDY) != TRNG_ISR_DATRDY) {
	}

	return trng_read_output_data(TRNG);

#else
	uint32_t ul_random_num;

	ul_random_num = rand() & 0xFF;
	ul_random_num = (ul_random_num << 8) | (rand() & 0xFF);
	ul_random_num = (ul_random_num << 8) | (rand() & 0xFF);
	ul_random_num = (ul_random_num << 8) | (rand() & 0xFF);

	return ul_random_num;
#endif
}

/**
 * \brief Set PL360 configuration. Called at initialization (once the binary is loaded) to configure required parameters in PL360 device
 */
static void _set_pl360_configuration(void)
{
	uint8_t uc_band_coup;

	/********* The following lines show how to configure different parameters on PL360 device *********/
	/********* The user can customize it depending on the requirements ********************************/

	/* Select band for pl360_g3_coup_tx_config() */
	switch (suc_phy_band) {
	case ATPL360_WB_CENELEC_A:
		uc_band_coup = COUP_BAND_CEN_A;
		break;

	case ATPL360_WB_CENELEC_B:
		uc_band_coup = COUP_BAND_CEN_B;
		break;

	case ATPL360_WB_FCC:
		uc_band_coup = COUP_BAND_FCC;
		break;

	case ATPL360_WB_ARIB:
		uc_band_coup = COUP_BAND_ARIB;
		break;

	default:
		break;
	}

	/* Configure Coupling and TX parameters */
	pl360_g3_coup_tx_config(&sx_atpl360_desc, uc_band_coup);

	/* Force Transmission to VLO mode by default in order to maximize signal level in anycase */
	/* Disable autodetect mode */
	sx_g3_pib.m_au8Value[0] = 0;
	sx_g3_pib.m_u8Length = 1;
	sx_atpl360_desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_CFG_AUTODETECT_IMPEDANCE, &sx_g3_pib);
	/* Set VLO mode */
	sx_g3_pib.m_au8Value[0] = 2;
	sx_g3_pib.m_u8Length = 1;
	sx_atpl360_desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_CFG_IMPEDANCE, &sx_g3_pib);

#ifdef CONF_TONEMASK_STATIC_NOTCHING
	/* Example to configure Tone Mask (Static Notching). Each carrier corresponding to the band can be notched (no energy is sent in those carriers) */
	/* Each carrier is represented by one byte (0: carrier used; 1: carrier notched). By default it is all 0's in PL360 device */
	/* The length is the number of carriers corresponding to the band in use (see "general_defs.h") */
	/* The same Tone Mask must be set in both transmitter and receiver. Otherwise they don't understand each other */
	memcpy(sx_g3_pib.m_au8Value, (uint8_t *)spuc_tone_mask, TONE_MASK_SIZE);
	sx_g3_pib.m_u8Length = TONE_MASK_SIZE;
	sx_atpl360_desc.set_req(MAC_RT_PIB_TONE_MASK, 0, &sx_g3_pib);
#endif
}

/**
 * \brief Setup parameters to use in Tx message
 */
static void _setup_tx_parameters(void)
{
	/* Init tranmission data */
	sx_tx_msg.m_u16MsduLength = 0;
	sx_tx_msg.m_pMsdu = spuc_tx_buffer;

	/* Set transmission address */
	sx_tx_msg.m_DstAddr.m_eAddrMode = sx_tx_mac_header.m_DestinationAddress.m_eAddrMode;
	sx_tx_msg.m_DstAddr.m_nShortAddress = sx_tx_mac_header.m_DestinationAddress.m_nShortAddress;

	/* Set transmission modulation */
	/* Modulation Type. See "enum ERtModulationType" in "MacRtDefs.h" file */
	/* Ordered from higher to lower data rate and from higher to lower required SNR (Signal to Noise Ratio): 8PSK, QPSK, BPSK, Robust BPSK */
	sx_tx_msg.m_eModulationType = RT_MODULATION_DBPSK_BPSK;
	/* Modulation Scheme. Differential or Coherent. See "enum ERtModulationScheme" in "MacRtDefs.h" file */
	/* Coherent Scheme supports worst SNR (about 3 dB) than Differential Scheme */
	/* Differential Scheme provides a bit higher data rate because Coherent Scheme uses some carriers for pilots */
	/* Coherent Scheme requires an accurate crystal oscillator. G3-PLC specifies that the frequency error must be less than 25 PPM */
	sx_tx_msg.m_eModulationScheme = RT_MODULATION_SCHEME_DIFFERENTIAL;

	/* ACK Request. Set to 1 in order to force ACK response from the other side for each received message. It should be the same value as the previous one
	 * used in "sx_tx_mac_header.m_Fc.m_nAckRequest" field. See "conf_project.h" file */
	sx_tx_msg.m_bRequestAck = sx_tx_mac_header.m_Fc.m_nAckRequest;

	/* Gain Resolution. Tx Gain resolution corresponding to one gain step. [0: 6dB, 1: 3dB]  */
	sx_tx_msg.m_nTxRes = 1;

	/* Transmission Gain. Desired transmitter gain specifying how many gain steps are requested. Range value: 0 - 15. */
	sx_tx_msg.m_nTxGain = 0;

	/* MAC High Priority. Set to 1 in order to set high priority for the next message to be transmitted */
	sx_tx_msg.m_bHighPriority = 0;

	/* Tone Map Request. Set to 1 in order to force Tone Map response from the other side */
	sx_tx_msg.m_bToneMapRequest = 0;

	/* Force Robo modo. Set to 1 in order to force robust modulation for the next transmission */
	sx_tx_msg.m_bForceRobo = 0;

	/* Configure Tone Map (Dynamic Notching) to use all carriers */
	/* Each bit corresponds to a subband */
	/* If the bit is '1' the subband is used to carry data. If the bit is '0' the subband does not carry data, but energy is sent in those carriers */
	/* The number of subbands is different in each G3 band (CENELEC-A, CENELEC-B, FCC, ARIB). See "general_defs.h" */
	/* The number of carriers per subband is 6 for CENELEC-A, 4 for CENELEC-B and 3 for FCC and ARIB */
	/* Full Tone Map: 0x3F0000 (CENELEC-A, 6 subbands); 0x0F0000 (CENELEC-B, 4 subbands); 0xFFFFFF (FCC, 24 subbands); 0xFFFF03 (ARIB, 18 subbands) */
	/* The next loop shows how to go across all subbands from lower to higher frequency */
	if (suc_phy_band == ATPL360_WB_CENELEC_A) {
		sx_tx_msg.m_ToneMap.m_au8Tm[0] = 0x3F;
		sx_tx_msg.m_ToneMap.m_au8Tm[1] = 0x00;
		sx_tx_msg.m_ToneMap.m_au8Tm[2] = 0x00;
	} else if (suc_phy_band == ATPL360_WB_FCC) {
		sx_tx_msg.m_ToneMap.m_au8Tm[0] = 0xFF;
		sx_tx_msg.m_ToneMap.m_au8Tm[1] = 0xFF;
		sx_tx_msg.m_ToneMap.m_au8Tm[2] = 0xFF;
	} else if (suc_phy_band == ATPL360_WB_CENELEC_B) {
		sx_tx_msg.m_ToneMap.m_au8Tm[0] = 0x0F;
		sx_tx_msg.m_ToneMap.m_au8Tm[1] = 0x00;
		sx_tx_msg.m_ToneMap.m_au8Tm[2] = 0x00;
	} else if (suc_phy_band == ATPL360_WB_ARIB) {
		sx_tx_msg.m_ToneMap.m_au8Tm[0] = 0x03;
		sx_tx_msg.m_ToneMap.m_au8Tm[1] = 0xFF;
		sx_tx_msg.m_ToneMap.m_au8Tm[2] = 0xFF;
	}

	/* Specifies the number of gain steps requested for the tones represented by Tone Map. Range value: 0 - 15. */
	/* TXCOEF[0] = Specifies the number of gain steps requested for the tones represented by ToneMap[0] & 0x0F */
	/* TXCOEF[1] = Specifies the number of gain steps requested for the tones represented by ToneMap[0] & 0xF0 */
	/* TXCOEF[2] = Specifies the number of gain steps requested for the tones represented by ToneMap[1] & 0x0F */
	/* TXCOEF[3] = Specifies the number of gain steps requested for the tones represented by ToneMap[1] & 0xF0 */
	/* TXCOEF[4] = Specifies the number of gain steps requested for the tones represented by ToneMap[2] & 0x0F */
	/* TXCOEF[5] = Specifies the number of gain steps requested for the tones represented by ToneMap[2] & 0xF0 */
	memset(sx_tx_msg.m_au8TxCoef, 0, sizeof(sx_tx_msg.m_au8TxCoef));
}

/**
 * \brief Setup G3 MAC Real-Time header
 */
static void _setup_mac_rt_header(void)
{
	memset(&sx_tx_mac_header, 0, sizeof(sx_tx_mac_header));

	/* MAC HEADER. Set G3 header info */
	sx_tx_mac_header.m_Fc.m_nAckRequest = CONF_ACK_REQUEST;
	/* MAC HEADER. Set DATA message. This example application only use this kind of frame type. */
	sx_tx_mac_header.m_Fc.m_nFrameType = MAC_LOW_FRAME_TYPE_DATA;
	/* MAC HEADER. Disable Security */
	sx_tx_mac_header.m_Fc.m_nSecurityEnabled = MAC_SECURITY_LEVEL_NONE;
	/* MAC HEADER. Set Sequence number. It should be incremented after each transmission. */
	sx_tx_mac_header.m_u8SequenceNumber = (uint8_t)_get_random_32();

	/* MAC HEADER. Set Network Addressing */
	sx_tx_mac_header.m_nDestinationPanIdentifier = CONF_PAN_ID;
	sx_tx_mac_header.m_nSourcePanIdentifier = CONF_PAN_ID;

	/* MAC HEADER. Set SHORT addressing mode: Destination address */
	sx_tx_mac_header.m_DestinationAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_SHORT;
	sx_tx_mac_header.m_Fc.m_nDestAddressingMode = sx_tx_mac_header.m_DestinationAddress.m_eAddrMode;

	/* MAC HEADER. Set SHORT addressing mode: Source address */
	sx_tx_mac_header.m_SourceAddress.m_eAddrMode = MAC_RT_ADDRESS_MODE_SHORT;
	sx_tx_mac_header.m_Fc.m_nSrcAddressingMode = sx_tx_mac_header.m_SourceAddress.m_eAddrMode;

	/* PL360 Set Addresses: Source and Destination */
	app_plc_set_mac_src_address((uint16_t)_get_random_32());
	app_plc_set_mac_dst_address(MAC_RT_SHORT_ADDRESS_BROADCAST);

	/* PL360 Set Personal Area Network Identification (PAN ID). See "conf_project.h" file. */
	app_plc_set_mac_panid(CONF_PAN_ID);

	printf("\r\nConfiguring G3 MAC RT Header");
	printf("\r\nG3 MAC PAN ID: 0x%04X", sx_tx_mac_header.m_nDestinationPanIdentifier);
	printf("\r\nG3 MAC Source Address: 0x%04X", sx_tx_mac_header.m_SourceAddress.m_nShortAddress);
	printf("\r\nG3 MAC Destination Address: 0x%04X\r\n", (unsigned int)sx_tx_mac_header.m_DestinationAddress.m_nShortAddress);
}

/**
 * \brief Check G3 MAC RT Header
 *
 * \param pMrtsdu Pointer to message data
 *
 * \return True if destination address match with own source address. False in any other cases.
 */
static bool _check_mac_rt_header(struct TMacRtFrame *pFrame)
{
	if (pFrame->m_Header.m_nDestinationPanIdentifier == sx_tx_mac_header.m_nSourcePanIdentifier) {
		if ((pFrame->m_Header.m_DestinationAddress.m_nShortAddress == sx_tx_mac_header.m_SourceAddress.m_nShortAddress) ||
				(pFrame->m_Header.m_DestinationAddress.m_nShortAddress == MAC_RT_SHORT_ADDRESS_BROADCAST)) {
			return true;
		}
	}

	return false;
}

/**
 * \brief Handler to manage confirmation of the last PLC transmission.
 *
 * \param px_msg_cfm Pointer to struct containing Tx Confirm paramters
 *
 */
static void _handler_data_cfm(enum EMacRtStatus eStatus, bool bUpdateTimestamp, enum ERtModulationType eModType)
{
	/* Update Sequence Number for next transmission */
	sx_tx_mac_header.m_u8SequenceNumber++;

	switch (eStatus) {
	case MAC_RT_STATUS_SUCCESS:
		printf("...MAC_RT_STATUS_SUCCESS");
		break;

	case MAC_RT_STATUS_CHANNEL_ACCESS_FAILURE:
		printf("...MAC_RT_STATUS_CHANNEL_ACCESS_FAILURE");
		break;

	case MAC_RT_STATUS_NO_ACK:
		printf("...MAC_RT_STATUS_NO_ACK");
		break;

	case MAC_RT_STATUS_DENIED:
		printf("...MAC_RT_STATUS_DENIED");
		break;

	case MAC_RT_STATUS_INVALID_INDEX:
		printf("...MAC_RT_STATUS_INVALID_INDEX");
		break;

	case MAC_RT_STATUS_INVALID_PARAMETER:
		printf("...MAC_RT_STATUS_INVALID_PARAMETER");
		break;

	case MAC_RT_STATUS_TRANSACTION_OVERFLOW:
		printf("...MAC_RT_STATUS_TRANSACTION_OVERFLOW");
		break;

	case MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE:
		printf("...MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE");
		break;

	default:
		/* Unexpeted result */
		printf(" MAC RT UNEXPECTED STATUS");
		break;
	}

	printf(" UpdateTimestamp: %u, ModType:%u", (unsigned int)bUpdateTimestamp, (unsigned int)eModType);

	app_console_handle_tx_cfm();
}

/**
 * \brief Handler to manage data from a new received message.
 *
 * \param pMrtsdu Pointer to message data
 * \param u16MrtsduLen Length of the message data in bytes
 *
 */
static void _handler_new_data_received(struct TMacRtFrame *pFrame, struct TMacRtDataIndication *pParameters)
{
	uint8_t *puc_data_buf;
	uint16_t us_data_len;

#ifdef LED1_GPIO
	/* Turn on LED1 to indicate the reception of PLC message */
	sul_ind_count_ms = COUNT_MS_IND_LED;
	LED_On(LED1);
#endif
	(void)pParameters;

	/* Look for data. Skip MAC Header */
	if (_check_mac_rt_header(pFrame)) {
		us_data_len = pFrame->m_u16PayloadLength - sizeof(sx_tx_mac_header);
		puc_data_buf = pFrame->m_pu8Payload + sizeof(sx_tx_mac_header);

		/* Send Data to Console Application */
		app_console_handle_rx_msg(puc_data_buf, us_data_len, sx_rx_parameters.m_eModulationScheme, sx_rx_parameters.m_eModulationType, sx_rx_parameters.m_u8PpduLinkQuality);
	}
}

/**
 * \brief Handler to manage parameters from a new received message.
 *
 * \param pParameters Pointer to struct containing message parameters
 *
 */
static void _handler_new_parameters_received(struct TMacRtPlmeGetConfirm *pParameters)
{
	/* Capture parameters of the new received message */
	sx_rx_parameters = *pParameters;
}

/**
 * \brief Handler to manage PL360 Exceptions. This callback is also called after loading binary at initization
 */
static void _handler_exception_event(atpl360_exception_t exception)
{
	printf("\r\n");

	switch (exception) {
	case ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS:
		/* SPI has detected an unexpected status, reset is recommended */
		suc_err_unexpected++;
		printf("ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS\r\n");
		break;

	case ATPL360_EXCEPTION_SPI_CRITICAL_ERROR:
		/* SPI critical error */
		suc_err_critical++;
		printf("ATPL360_EXCEPTION_SPI_CRITICAL_ERROR\r\n");
		break;

	case ATPL360_EXCEPTION_RESET:
		/* Device Reset */
		if (sb_enabling_pl360) {
			/* This callback is also called after loading binary at initization */
			/* This message is shown to indicate that the followitn exception is normal because PL360 binary has just been loaded */
			sb_enabling_pl360 = false;
			printf("PL360 initialization event: ");
		}

		suc_err_reset++;
		printf("ATPL360_EXCEPTION_RESET\r\n");
		break;

	default:
		printf("ATPL360_EXCEPTION_UNKNOWN\r\n");
		suc_err_none++;
	}

	/* Set flag to manage exception in app_plc_process() */
	sb_exception_pend = true;
}

/**
 * \brief Get PL360 binary addressing.
 *
 * \param pul_address   Pointer to store the initial address of PL360 binary data
 *
 * \return Size of PL360 binary file
 */
static uint32_t _get_pl360_bin_addressing(uint32_t *pul_address)
{
	uint32_t ul_bin_addr;
	uint8_t *puc_bin_start;
	uint8_t *puc_bin_end;

#if defined(CONF_MULTIBAND_FCC_CENA)
	/* Multiband: FCC and CENELEC-A binaries are linked */
	if (suc_phy_band == ATPL360_WB_FCC) {
		/* Select FCC binary */
#    if defined (__CC_ARM)
		extern uint8_t atpl_bin_fcc_start[];
		extern uint8_t atpl_bin_fcc_end[];
		ul_bin_addr = (int)(atpl_bin_fcc_start - 1);
		puc_bin_start = atpl_bin_fcc_start - 1;
		puc_bin_end = atpl_bin_fcc_end;
#    elif defined (__GNUC__)
		extern uint8_t atpl_bin_fcc_start;
		extern uint8_t atpl_bin_fcc_end;
		ul_bin_addr = (int)&atpl_bin_fcc_start;
		puc_bin_start = (uint8_t *)&atpl_bin_fcc_start;
		puc_bin_end = (uint8_t *)&atpl_bin_fcc_end;
#    elif defined (__ICCARM__)
#      pragma section = "P_atpl_bin_fcc"
		extern uint8_t atpl_bin_fcc;
		ul_bin_addr = (int)&atpl_bin_fcc;
		puc_bin_start = __section_begin("P_atpl_bin_fcc");
		puc_bin_end = __section_end("P_atpl_bin_fcc");
#    else
#      error This compiler is not supported for now.
#    endif
	} else { /* ATPL360_WB_CENELEC_A */
		 /* Select CENELEC-A binary */
#    if defined (__CC_ARM)
		extern uint8_t atpl_bin_cena_start[];
		extern uint8_t atpl_bin_cena_end[];
		ul_bin_addr = (int)(atpl_bin_cena_start - 1);
		puc_bin_start = atpl_bin_cena_start - 1;
		puc_bin_end = atpl_bin_cena_end;
#    elif defined (__GNUC__)
		extern uint8_t atpl_bin_cena_start;
		extern uint8_t atpl_bin_cena_end;
		ul_bin_addr = (int)&atpl_bin_cena_start;
		puc_bin_start = (int)&atpl_bin_cena_start;
		puc_bin_end = (int)&atpl_bin_cena_end;
#    elif defined (__ICCARM__)
#      pragma section = "P_atpl_bin_cena"
		extern uint8_t atpl_bin_cena;
		ul_bin_addr = (int)&atpl_bin_cena;
		puc_bin_start = __section_begin("P_atpl_bin_cena");
		puc_bin_end = __section_end("P_atpl_bin_cena");
#    else
#      error This compiler is not supported for now.
#    endif
	}

#elif defined(CONF_MULTIBAND_FCC_CENB)
	/* Multiband: FCC and CENELEC-B binaries are linked */
	if (suc_phy_band == ATPL360_WB_FCC) {
		/* Select FCC binary */
#    if defined (__CC_ARM)
		extern uint8_t atpl_bin_fcc_start[];
		extern uint8_t atpl_bin_fcc_end[];
		ul_bin_addr = (int)(atpl_bin_fcc_start - 1);
		puc_bin_start = atpl_bin_fcc_start - 1;
		puc_bin_end = atpl_bin_fcc_end;
#    elif defined (__GNUC__)
		extern uint8_t atpl_bin_fcc_start;
		extern uint8_t atpl_bin_fcc_end;
		ul_bin_addr = (int)&atpl_bin_fcc_start;
		puc_bin_start = (uint8_t *)&atpl_bin_fcc_start;
		puc_bin_end = (uint8_t *)&atpl_bin_fcc_end;
#    elif defined (__ICCARM__)
#      pragma section = "P_atpl_bin_fcc"
		extern uint8_t atpl_bin_fcc;
		ul_bin_addr = (int)&atpl_bin_fcc;
		puc_bin_start = __section_begin("P_atpl_bin_fcc");
		puc_bin_end = __section_end("P_atpl_bin_fcc");
#    else
#      error This compiler is not supported for now.
#    endif
	} else { /* ATPL360_WB_CENELEC_B */
		 /* Select CENELEC-B binary */
#    if defined (__CC_ARM)
		extern uint8_t atpl_bin_cenb_start[];
		extern uint8_t atpl_bin_cenb_end[];
		ul_bin_addr = (int)(atpl_bin_cenb_start - 1);
		puc_bin_start = atpl_bin_cenb_start - 1;
		puc_bin_end = atpl_bin_cenb_end;
#    elif defined (__GNUC__)
		extern uint8_t atpl_bin_cenb_start;
		extern uint8_t atpl_bin_cenb_end;
		ul_bin_addr = (int)&atpl_bin_cenb_start;
		puc_bin_start = (int)&atpl_bin_cenb_start;
		puc_bin_end = (int)&atpl_bin_cenb_end;
#    elif defined (__ICCARM__)
#      pragma section = "P_atpl_bin_cenb"
		extern uint8_t atpl_bin_cenb;
		ul_bin_addr = (int)&atpl_bin_cenb;
		puc_bin_start = __section_begin("P_atpl_bin_cenb");
		puc_bin_end = __section_end("P_atpl_bin_cenb");
#    else
#      error This compiler is not supported for now.
#    endif
	}

#else /* ifdef CONF_MULTIBAND_FCC_CENA */
	/* Only one binary linked */
#  if defined (__CC_ARM)
	extern uint8_t atpl_bin_start[];
	extern uint8_t atpl_bin_end[];
	ul_bin_addr = (int)(atpl_bin_start - 1);
	puc_bin_start = atpl_bin_start - 1;
	puc_bin_end = atpl_bin_end;
#  elif defined (__GNUC__)
	extern uint8_t atpl_bin_start;
	extern uint8_t atpl_bin_end;
	ul_bin_addr = (int)&atpl_bin_start;
	puc_bin_start = (int)&atpl_bin_start;
	puc_bin_end = (int)&atpl_bin_end;
#  elif defined (__ICCARM__)
#    pragma section = "P_atpl_bin"
	extern uint8_t atpl_bin;
	ul_bin_addr = (int)&atpl_bin;
	puc_bin_start = __section_begin("P_atpl_bin");
	puc_bin_end = __section_end("P_atpl_bin");
#  else
#    error This compiler is not supported for now.
#  endif
#endif
	*pul_address = ul_bin_addr;
	/* cppcheck-suppress deadpointer */
	return ((uint32_t)puc_bin_end - (uint32_t)puc_bin_start);
}

/**
 * \brief Enables PL360 device. Load PHY binary.
 *
 */
static void _pl360_enable(void)
{
	uint32_t ul_bin_addr;
	uint32_t ul_bin_size;
	uint8_t uc_ret;

	/* Get PL360 binary address and size */
	ul_bin_size = _get_pl360_bin_addressing(&ul_bin_addr);

	/* Enable PL360: Load binary */
	printf("\r\nEnabling PL360 device: Loading PHY binary\r\n");
	sb_enabling_pl360 = true;
	uc_ret = atpl360_enable(ul_bin_addr, ul_bin_size);
	if (uc_ret == ATPL360_ERROR) {
		printf("\r\nCRITICAL ERROR: PL360 binary load failed (%d)\r\n", uc_ret);
		while (1) {
		}
	}

	printf("\r\nPL360 binary loaded correctly\r\n");

	/* Get PHY version */
	sx_atpl360_desc.get_req(MAC_RT_PIB_MANUF_PHY_PARAM, PHY_PARAM_VERSION, &sx_g3_pib);

	printf("PHY version: 0x%08x (", *(uint32_t *)sx_g3_pib.m_au8Value);

	/* sx_g3_pib.m_au8Value[2] correspons to G3 band [0x01: CEN-A, 0x02: FCC, 0x03: ARIB, 0x04: CEN-B] */
	switch (sx_g3_pib.m_au8Value[2]) {
	case ATPL360_WB_CENELEC_A:
		printf("CENELEC-A band: 35 - 91 kHz)\r\n");
		break;

	case ATPL360_WB_FCC:
		printf("FCC band: 154 - 488 kHz)\r\n");
		break;

	case ATPL360_WB_ARIB:
		printf("ARIB band: 154 - 404 kHz)\r\n");
		break;

	case ATPL360_WB_CENELEC_B:
		printf("CENELEC-B band: 98 - 122 kHz)\r\n");
		break;

	default:
		printf("Unknown band)\r\n");
		break;
	}

	if (sx_g3_pib.m_au8Value[2] != suc_phy_band) {
		printf("ERROR: PHY band does not match with band configured in application\r\n");
	}
}

/**
 * \brief Initialization of PL360 G3 MAC RT.
 *
 */
void app_plc_init(void)
{
	atpl360_dev_callbacks_t x_atpl360_cbs;
	atpl360_hal_wrapper_t x_atpl360_hal_wrp;

	/* Initialize G3 band static variable (ATPL360_WB defined in conf_atpl360.h) */
	suc_phy_band = ATPL360_WB;

	/* Initialize PL360 controller */
	x_atpl360_hal_wrp.plc_init = pplc_if_init;
	x_atpl360_hal_wrp.plc_reset = pplc_if_reset;
	x_atpl360_hal_wrp.plc_set_stby_mode = pplc_if_set_stby_mode;
	x_atpl360_hal_wrp.plc_set_handler = pplc_if_set_handler;
	x_atpl360_hal_wrp.plc_send_boot_cmd = pplc_if_send_boot_cmd;
	x_atpl360_hal_wrp.plc_write_read_cmd = pplc_if_send_wrrd_cmd;
	x_atpl360_hal_wrp.plc_enable_int = pplc_if_enable_interrupt;
	x_atpl360_hal_wrp.plc_delay = pplc_if_delay;
	x_atpl360_hal_wrp.plc_get_thw = pplc_if_get_thermal_warning;
	atpl360_init(&sx_atpl360_desc, &x_atpl360_hal_wrp);

	/* Callback functions configuration. Set NULL as Not used */
	x_atpl360_cbs.tx_confirm = _handler_data_cfm;
	x_atpl360_cbs.process_frame = _handler_new_data_received;
	x_atpl360_cbs.plme_get_cfm = _handler_new_parameters_received;
	x_atpl360_cbs.exception_event = _handler_exception_event;
	x_atpl360_cbs.sleep_mode_cb = NULL;
	x_atpl360_cbs.debug_mode_cb = NULL;
	sx_atpl360_desc.set_callbacks(&x_atpl360_cbs);

	/* Enable PL360 device: Load binary */
	_pl360_enable();

	/* Enable Random Generator */
	_init_random();
}

/**
 * \brief Send PLC message.
 *
 * \param puc_data_buff Pointer to buffer containing the data to send.
 * \param us_data_len Data length in bytes.
 *
 */
atpl360_res_t app_plc_send_msg(uint8_t *puc_data_buff, uint16_t us_data_len)
{
	uint8_t *puc_data;
	atpl360_res_t uc_result;

	/* Build MAC RT DATA Frame */
	puc_data = spuc_tx_buffer;

	/* Insert MAC header information */
	memcpy(puc_data, (uint8_t *)&sx_tx_mac_header, sizeof(sx_tx_mac_header));
	puc_data += sizeof(sx_tx_mac_header);
	/* Insert Data Payload to be transmitted */
	memcpy(puc_data, puc_data_buff, us_data_len);
	puc_data += us_data_len;

	/* Set Msdu Length */
	sx_tx_msg.m_u16MsduLength = puc_data - spuc_tx_buffer;

	/* Send PLC message. send_data returns ATPL360_SUCCESS if transmission was correctly programmed */
	/* The result will be reported in Tx Confirm (_handler_data_cfm) when message is completely sent */
	uc_result = sx_atpl360_desc.tx_request(&sx_tx_msg, &sx_tx_mac_header);
	printf("\r\nTx (%u bytes): ", (unsigned int)sx_tx_msg.m_u16MsduLength);

	return uc_result;
}

/**
 * \brief Set Modulation (Modulation Type and Modulation Scheme).
 * They are stored at application level, not in PL360 G3 MAC RT. They are sent to PL360 G3 MAC RT just before sending data.
 *
 * \param uc_mod_scheme Modulation Scheme to be configured.
 * \param uc_mod_type Modulation Type to be configured.
 *
 */
void app_plc_set_modulation(enum ERtModulationScheme uc_mod_scheme, enum ERtModulationType uc_mod_type)
{
	/* Store Modulation Type and Modulation Scheme in Tx Parameters structure */
	sx_tx_msg.m_eModulationScheme = uc_mod_scheme;
	sx_tx_msg.m_eModulationType = uc_mod_type;
}

#if defined(CONF_MULTIBAND_FCC_CENA)

/**
 * \brief Change G3 band. In this example it is only possible to change between CENELEC-A and FCC.
 * CONF_MULTIBAND_FCC_CENA must be defined to enable Multiband capability.
 * PLCOUP011 should be used for Multiband purposes (Branch 0 for CENELEC-A and Branch 1 for FCC)
 *
 * \param uc_band Band to be configured (see band definitions in "general_defs.h").
 *
 * \retval false. Invalid configuration
 * \retval true. Band reconfigured successfully
 */
bool app_plc_set_band(uint8_t uc_band)
{
	if (uc_band == suc_phy_band) {
		printf("\r\nBand has not changed: Skipping Band Reconfiguration\r\n");
		return false;
	}

	if ((uc_band == ATPL360_WB_CENELEC_A) || (uc_band == ATPL360_WB_FCC)) {
		/* Reset PL360 device to load new binary */
		atpl360_disable();

		/* Set static Band variable */
		suc_phy_band = uc_band;

		/* Enable PL360 device: Load binary to change Band */
		_pl360_enable();

		return true;
	} else {
		printf("Band not supported: Skipping Band Configuration\r\n");
		return false;
	}
}

#elif defined(CONF_MULTIBAND_FCC_CENB)

/**
 * \brief Change G3 band. In this example it is only possible to change between CENELEC-B and FCC.
 * CONF_MULTIBAND_FCC_CENB must be defined to enable Multiband capability.
 *
 * \param uc_band Band to be configured (see band definitions in "general_defs.h").
 *
 * \retval false. Invalid configuration
 * \retval true. Band reconfigured successfully
 */
bool app_plc_set_band(uint8_t uc_band)
{
	if (uc_band == suc_phy_band) {
		printf("\r\nBand has not changed: Skipping Band Reconfiguration\r\n");
		return false;
	}

	if ((uc_band == ATPL360_WB_CENELEC_B) || (uc_band == ATPL360_WB_FCC)) {
		/* Reset PL360 device to load new binary */
		atpl360_disable();

		/* Set static Band variable */
		suc_phy_band = uc_band;

		/* Enable PL360 device: Load binary to change Band */
		_pl360_enable();

		return true;
	} else {
		printf("Band not supported: Skipping Band Configuration\r\n");
		return false;
	}
}

#endif

/**
 * \brief Get current Modulation Scheme configured.
 * It is stored at application level, not in PL360 G3 MAC RT. It is sent to PL360 G3 MAC RT in Tx Request (send_data).
 *
 * \return Current Modulation Scheme configured.
 */
enum ERtModulationScheme app_plc_get_mod_scheme(void)
{
	/* Return Modulation Scheme stored in Tx Parameters structure */
	return sx_tx_msg.m_eModulationScheme;
}

/**
 * \brief Get current Modulation Type configured.
 * It is stored at application level, not in PL360 G3 MAC RT. It is sent to PL360 G3 MAC RT in Tx Request (send_data).
 *
 * \return Current Modulation Type configured.
 */
enum ERtModulationType app_plc_get_mod_type(void)
{
	/* Return Modulation Type stored in Tx Parameters structure */
	return sx_tx_msg.m_eModulationType;
}

/**
 * \brief Get current G3 band configured.
 *
 * \return Current G3 band configured.
 */
uint8_t app_plc_get_phy_band(void)
{
	/* Return G3 band configured */
	return suc_phy_band;
}

/**
 * \brief Get current G3 MAC Source Address.
 *
 * \return Current MAC Source Address.
 */
uint16_t app_plc_get_mac_src_address(void)
{
	/* Return G3 Source Address */
	return sx_tx_mac_header.m_SourceAddress.m_nShortAddress;
}

/**
 * \brief Get current G3 MAC Destination Address.
 *
 * \return Current MAC Destination Address.
 */
uint16_t app_plc_get_mac_dst_address(void)
{
	/* Return G3 Destination Address */
	return (uint16_t)sx_tx_mac_header.m_DestinationAddress.m_nShortAddress;
}

/**
 * \brief Set G3 MAC Source Address.
 *
 * \parameter address MAC Source Address.
 */
void app_plc_set_mac_src_address(uint16_t address)
{
	/* Set Source Address */
	sx_tx_mac_header.m_SourceAddress.m_nShortAddress = address;

	/* Set PIB Short Address */
	memcpy(sx_g3_pib.m_au8Value, (uint8_t *)&sx_tx_mac_header.m_SourceAddress.m_nShortAddress, 2);
	sx_g3_pib.m_u8Length = 2;
	sx_atpl360_desc.set_req(MAC_RT_PIB_SHORT_ADDRESS, 0, &sx_g3_pib);

	/* Update Extended Address */
	spuc_extended_address[6] = (uint8_t)(address >> 8);
	spuc_extended_address[7] = (uint8_t)address;

	/* Set PIB Extended Address */
	memcpy(sx_g3_pib.m_au8Value, (uint8_t *)spuc_extended_address, 8);
	sx_g3_pib.m_u8Length = 8;
	sx_atpl360_desc.set_req(MAC_RT_PIB_MANUF_EXTENDED_ADDRESS, 0, &sx_g3_pib);
}

/**
 * \brief Set G3 MAC Personal Area Network Identifier (PAN ID).
 *
 * \parameter panid Personal Area Network Identifier (PAN ID)
 */
void app_plc_set_mac_panid(uint16_t pan_id)
{
	/* Set Source Address */
	sx_tx_mac_header.m_nSourcePanIdentifier = pan_id;

	/* Set PIB Short Address */
	memcpy(sx_g3_pib.m_au8Value, (uint8_t *)&sx_tx_mac_header.m_nSourcePanIdentifier, 2);
	sx_g3_pib.m_u8Length = 2;
	sx_atpl360_desc.set_req(MAC_RT_PIB_PAN_ID, 0, &sx_g3_pib);
}

/**
 * \brief Set G3 MAC Destination Address.
 *
 * \parameter address MAC Destination Address.
 */
void app_plc_set_mac_dst_address(uint16_t address)
{
	/* Set Destination Address */
	sx_tx_mac_header.m_DestinationAddress.m_nShortAddress = address;
	sx_tx_msg.m_DstAddr.m_nShortAddress = address;
}

/**
 * \brief Phy Controller module process.
 */
void app_plc_process(void)
{
	/* Manage PL360 exceptions. At initialization ATPL360_EXCEPTION_RESET is reported */
	if (sb_exception_pend) {
		/* Clear exception flag */
		sb_exception_pend = false;

		/* Set PL360 specific configuration from application */
		/* Called at initialization and if an exception ocurrs */
		/* If an exception occurs, PL360 is reset and some parameters may have to be reconfigured */
		_set_pl360_configuration();

		/* Setup G3 MAC RT parameters to use in transmission */
		_setup_tx_parameters();

		/* Setup G3 MAC RT header */
		_setup_mac_rt_header();

		/* Update configured Modulation and maximum data length in Chat App */
		app_console_handle_pl360_reset(sx_tx_msg.m_eModulationScheme, sx_tx_msg.m_eModulationType);
	}

	/* Check ATPL360 pending events. It must be called from application periodically to handle G3 MAC RT events */
	atpl360_handle_events();
}
