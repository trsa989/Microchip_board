/**
 * \file
 *
 * \brief HEADER. ATPL250  Store persistent data
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

#ifndef __STORAGE_H__
#define __STORAGE_H__

/* System includes */
#include <mac_wrapper.h>

#ifdef G3_HYBRID_PROFILE
#define STORAGE_VERSION 2
#else
#define STORAGE_VERSION 1
#endif

/* struct to store persistent data from stack (MAC & ADP) */
struct TPersistentData {
	uint32_t m_u32FrameCounter;
#ifdef G3_HYBRID_PROFILE
	uint32_t m_u32FrameCounterRF;
#endif
	uint16_t m_u16DiscoverSeqNumber;
	uint8_t m_u8BroadcastSeqNumber;
};

/* struct to store persistent info: header + data */
struct TPersistentInfo {
	uint32_t m_u32StartupCounter; /* First 4 bytes. Also changed from platform. */
	uint16_t m_u16Version;
	uint16_t m_u16Crc16;
	struct TPersistentData m_data;
};

void store_persistent_data_GPBR(void);
void store_persistent_info(void);
void load_persistent_info(void);

#endif
