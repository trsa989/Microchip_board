/**
 * \file
 *
 * \brief ATPL250 Store persistent data
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
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

/* System includes */
#include <stdbool.h>
#include <stdint.h>
#include <hal/hal.h>
#include <AdpApi.h>
#include <conf_hal.h>
#include <gpbr.h>
#include <oss_if.h>
#include <string.h>

/* Storage includes */
#include "storage.h"

#ifdef STORAGE_DEBUG_ENABLE
#define LOG_STORAGE(a)   printf a 
#else
#define LOG_STORAGE(a)   (void)0
#endif

#if defined(_SAMG55J19_)
#define RSTC_SR_RSTTYP_GeneralReset   RSTC_SR_RSTTYP_GENERAL_RST
#endif

static struct TPersistentInfo persistentInfo;

static uint16_t Crc16(const uint8_t *pu8Data, uint32_t u32Length, uint16_t u16Poly, uint16_t u16Crc);
static void _get_persistent_data(struct TPersistentData *data);
static void _set_persistent_data(struct TPersistentData *data);
static void _read_persistent_data_GPBR(struct TPersistentData *data);
static void _write_persistent_data_GPBR(struct TPersistentData *data);
static bool _update_persistent_data_GPBR(struct TPersistentData *data, bool b_upd_info);

/**
 * \brief Stores persistent data.
 *
 * \remarks GPBR registers are not updated, as this function is intended to be
 * triggered on power down.
 */
void store_persistent_info(void)
{
	LOG_STORAGE(("PDD_CB: Persistent data stored.\r\n"));

	_get_persistent_data(&persistentInfo.m_data);

	/* Persistent info Header */
	persistentInfo.m_u16Version = STORAGE_VERSION;
	persistentInfo.m_u16Crc16 = Crc16((const uint8_t *)(&persistentInfo.m_data), sizeof(struct TPersistentData), 0x1021, 0xFFFF);

	/* Write internal data to the persistent storage */
	platform_write_storage(sizeof(struct TPersistentInfo), &persistentInfo);
}

/**
 * \brief Stores persistent data in GPBR.
 *
 * \remarks This function is intended to be triggered periodically (e. g. every 3 secs).
 */
void store_persistent_data_GPBR(void)
{
	_get_persistent_data(&persistentInfo.m_data);
	_write_persistent_data_GPBR(&persistentInfo.m_data);

	LOG_STORAGE(("Persistent data stored in GPBR (fr_cnt: 0x%04x).\r\n", persistentInfo.m_data.m_u32FrameCounter));
}

/**
 * \brief Loads persistent data
 *
 * \remarks sets callbacks to store persistent data periodically and on power off.
 *
 */
void load_persistent_info(void)
{
	bool b_upd_info;
	LOG_STORAGE(("Loading persistent data...\r\n"));

#if defined(PLATFORM_PDD_INTERNAL_SUPPLY_MONITOR) || defined(PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER)
	/* Set callback for power down */
	platform_set_pdd_callback(&store_persistent_info);
	LOG_STORAGE(("PDD Callback to store persistent data set.\r\n"));
#endif

#if defined(PLATFORM_RST_INTERRUPT)
	/* Set callback for power down */
	platform_set_reset_callback(&store_persistent_info);
	LOG_STORAGE(("RST Callback to store persistent data set.\r\n"));
#endif	
	
	b_upd_info = true;

	platform_init_storage();

	/* Read the persistent storage */
	if (platform_read_storage(sizeof(struct TPersistentInfo), &persistentInfo)) {
		/* Check the CRC */
		uint16_t u16Crc16 = Crc16((const uint8_t *)(&persistentInfo.m_data), sizeof(struct TPersistentData), 0x1021, 0xFFFF);

		if (persistentInfo.m_u16Crc16 != u16Crc16) {
			LOG_STORAGE(("load_persistent_info() CRC error. Read: %u, Calc: %u\r\n", persistentInfo.m_u16Crc16, u16Crc16));
			b_upd_info = false;
		} else if (persistentInfo.m_u16Version != STORAGE_VERSION) {
			LOG_STORAGE(("load_persistent_info() storage version error.\r\n"));
			b_upd_info = false;
		}
	} else {
		LOG_STORAGE(("load_persistent_info() unable to read storage.\r\n"));
		b_upd_info = false;
	}
	
	/* Increment startup counter */
	persistentInfo.m_u32StartupCounter++;

	/* Set Values to G3 Stack */
	if (_update_persistent_data_GPBR(&persistentInfo.m_data, b_upd_info)) {
		_set_persistent_data(&persistentInfo.m_data);
		LOG_STORAGE(("Persistent data loaded. m_u32StartupCounter: %d\r\n", persistentInfo.m_u32StartupCounter));
		LOG_STORAGE(("Persistent data loaded. m_u16Version: %d\r\n", persistentInfo.m_u16Version));
		LOG_STORAGE(("Persistent data loaded. m_u16Crc16: %d\r\n", persistentInfo.m_u16Crc16));
		LOG_STORAGE(("Persistent data loaded. m_u32FrameCounter: %d\r\n", persistentInfo.m_data.m_u32FrameCounter));
		LOG_STORAGE(("Persistent data loaded. m_u32FrameCounterRF: %d\r\n", persistentInfo.m_data.m_u32FrameCounterRF));
		LOG_STORAGE(("Persistent data loaded. m_u16DiscoverSeqNumber: %d\r\n", persistentInfo.m_data.m_u16DiscoverSeqNumber));
		LOG_STORAGE(("Persistent data loaded. m_u8BroadcastSeqNumber: %d\r\n", persistentInfo.m_data.m_u8BroadcastSeqNumber));	
	}

	/* Pre-erase flash page for further quick writing on power-down */
	platform_erase_storage(sizeof(struct TPersistentInfo));
}

/**
 * \brief Calculates CRC-16
 *
 */
static uint16_t Crc16(const uint8_t *pu8Data, uint32_t u32Length, uint16_t u16Poly, uint16_t u16Crc)
{
	uint8_t u8Index;
	while (u32Length--) {
		u16Crc ^= (*pu8Data++) << 8;
		for (u8Index = 0; u8Index < 8; u8Index++) {
			u16Crc = (u16Crc << 1) ^ ((u16Crc & 0x8000U) ? u16Poly : 0);
		}
	}
	return u16Crc;
}

/**
 * \brief Gets G3 stack info to be stored
 *
 * \param data     Pointer to the persistent info
 *
 */
static void _get_persistent_data(struct TPersistentData *data)
{
	struct TAdpMacGetConfirm macGetConfirm;
	struct TAdpGetConfirm adpGetConfirm;

	/* Read internal data from the stack */
	AdpMacGetRequestSync(MAC_WRP_PIB_FRAME_COUNTER, 0, &macGetConfirm);
	memcpy(&data->m_u32FrameCounter, macGetConfirm.m_au8AttributeValue, macGetConfirm.m_u8AttributeLength);
#ifdef G3_HYBRID_PROFILE
	AdpMacGetRequestSync(MAC_WRP_PIB_FRAME_COUNTER_RF, 0, &macGetConfirm);
	memcpy(&data->m_u32FrameCounterRF, macGetConfirm.m_au8AttributeValue, macGetConfirm.m_u8AttributeLength);
#endif
	AdpGetRequestSync(ADP_IB_MANUF_DISCOVER_SEQUENCE_NUMBER, 0, &adpGetConfirm);
	memcpy(&data->m_u16DiscoverSeqNumber, adpGetConfirm.m_au8AttributeValue, adpGetConfirm.m_u8AttributeLength);
	AdpGetRequestSync(ADP_IB_MANUF_BROADCAST_SEQUENCE_NUMBER, 0, &adpGetConfirm);
	memcpy(&data->m_u8BroadcastSeqNumber, adpGetConfirm.m_au8AttributeValue, adpGetConfirm.m_u8AttributeLength);
}

/**
 * \brief Sets stored info in the G3 stack
 *
 * \param info     Pointer to the persistent info
 *
 */
static void _set_persistent_data(struct TPersistentData *data)
{
	struct TAdpMacSetConfirm macSetConfirm;
	struct TAdpSetConfirm adpSetConfirm;

	/* Write internal data to the stack */
	AdpMacSetRequestSync(MAC_WRP_PIB_FRAME_COUNTER, 0, sizeof(data->m_u32FrameCounter), 
			(const uint8_t *)(&data->m_u32FrameCounter), &macSetConfirm);
#ifdef G3_HYBRID_PROFILE
	AdpMacSetRequestSync(MAC_WRP_PIB_FRAME_COUNTER_RF, 0, sizeof(data->m_u32FrameCounterRF), 
			(const uint8_t *)(&data->m_u32FrameCounterRF), &macSetConfirm);
#endif
	AdpSetRequestSync(ADP_IB_MANUF_DISCOVER_SEQUENCE_NUMBER, 0, sizeof(data->m_u16DiscoverSeqNumber),
			(const uint8_t *)(&data->m_u16DiscoverSeqNumber), &adpSetConfirm);
	AdpSetRequestSync(ADP_IB_MANUF_BROADCAST_SEQUENCE_NUMBER, 0, sizeof(data->m_u8BroadcastSeqNumber),
			(const uint8_t *)(&data->m_u8BroadcastSeqNumber), &adpSetConfirm);
}

/**
 * \brief Reads the persistent info from the GPBR registers.
 *
 * \param info     Pointer to the persistent info
 *
 */
static void _read_persistent_data_GPBR(struct TPersistentData *data)
{
	data->m_u16DiscoverSeqNumber = gpbr_read(GPBR0);
	data->m_u8BroadcastSeqNumber = (uint8_t)(gpbr_read(GPBR0) >> 16);
	data->m_u32FrameCounter = gpbr_read(GPBR1);
#ifdef G3_HYBRID_PROFILE
	data->m_u32FrameCounterRF = gpbr_read(GPBR2);
#endif
}

/**
 * \brief Writes the persistent info in the GPBR registers.
 *
 * \param info     Pointer to the persistent info
 *
 */
static void _write_persistent_data_GPBR(struct TPersistentData *data)
{
	uint32_t u32aux = 0;
	u32aux = data->m_u16DiscoverSeqNumber;
	u32aux += ((uint32_t)data->m_u8BroadcastSeqNumber) << 16;
	gpbr_write(GPBR0, u32aux);
	u32aux = data->m_u32FrameCounter;
	gpbr_write(GPBR1, u32aux);
#ifdef G3_HYBRID_PROFILE
	u32aux = data->m_u32FrameCounterRF;
	gpbr_write(GPBR2, u32aux);
#endif
}

/**
 * \brief Updates the persistent info from the GPBR registers.
 *
 * \param info     Pointer to the persistent info
 *
 */
static bool _update_persistent_data_GPBR(struct TPersistentData *data, bool b_upd_info)
{
	bool res;

	res = false;

	/* Check last reset type and synchronize status of FC_KEY values between GPBR and FLASH */
#if (SAMG || SAM4E || SAME70 || PIC32CX)
	if ((RSTC_RSTC_SR & RSTC_SR_RSTTYP_Msk) != RSTC_SR_RSTTYP_GENERAL_RST) {
#else
	if ((RSTC_RSTC_SR & RSTC_SR_RSTTYP_Msk) != RSTC_SR_RSTTYP_GeneralReset) {
#endif
		/* Not a power down reset: read from GPBR */
		_read_persistent_data_GPBR(data);
		if (data->m_u32FrameCounter < 0xFFFFFFFF) {
			res = true;
			LOG_STORAGE(("Not a power down reset: read from GPBR.\r\n"));
		} else {
			LOG_STORAGE(("Not a power down reset: GPBR invalid.\r\n"));
		}
	} else {
		/* Power down reset */
		if (b_upd_info) {
			/* Update GPBR */
			_write_persistent_data_GPBR(data);
			res = true;
			LOG_STORAGE(("Power down reset: update GPBR.\r\n"));
		} else {
			/* Do nothing, GPBR's update in each tx */
			LOG_STORAGE(("Power down reset: Do Nothing.\r\n"));
		}
	}

	return res;
}
