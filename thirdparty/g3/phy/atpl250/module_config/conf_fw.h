/**
 * \file
 *
 * \brief PHY Layer Configuration Header for ATPL250.
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
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

#ifndef CONF_FW_H_INCLUDE
#define CONF_FW_H_INCLUDE

#include "conf_project.h"

/************************************ ZERO CROSS SIGNAL DELAY ************************************/
/* ZC offset between real zero cross and ZC signal provided to modem (in us) */
/* Value to be configured is the positive delay from real ZC to signal provided to modem */
#define CONF_ZC_OFFSET_CORRECTION   (uint32_t)0
/*************************************************************************************************/

/**************************************** PHY LAYER TRACES ***************************************/
/* Activate PHY Traces */
#ifndef ENABLE_SIGNAL_DUMP
/* #define ENABLE_PHY_TRACES */
#endif
/*************************************************************************************************/

/************************************** IMPEDANCE DETECTION **************************************/
/* Disable VLO to HI state jump back in Auto mode (comment the line to enable jump back) */
/* #define DISABLE_VLO_TO_HI_JUMP */
/*************************************************************************************************/

/************************************** PHY PROCESS RECALL ***************************************/
/* Enable phy process recall to speed up processes in FCC/ARIB */
#define ENABLE_PYH_PROCESS_RECALL
/*************************************************************************************************/

#endif  /* CONF_FW_H_INCLUDE */
