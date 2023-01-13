/**
 * \file
 *
 * \brief Example embedded application for ATMEL PLC PHY.
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

/**
 *  \mainpage ATMEL PLC Phy Embedded Example
 *
 *  \section Purpose
 *
 *
 *
 *  \section Requirements
 *
 *
 *
 *  \section Description
 *
 *
 *
 *  \section Usage
 *
 *
 */

/* System includes */
#include <stdint.h>
#include <stdio.h>
#include "string.h"

/* Atmel library includes. */
#include "asf.h"

/* Example configuration */
#include "conf_project.h"

#define SERIAL_IF_ENABLE        1

static void phy_tester_tool_init(void)
{
	uint8_t uc_value;
	
	/* Init Phy Layer */
#if defined (CONF_BAND_CENELEC_A)
	phy_init(SERIAL_IF_ENABLE, WB_CENELEC_A);
#elif defined (CONF_BAND_FCC)
	phy_init(SERIAL_IF_ENABLE, WB_FCC);
#elif defined (CONF_BAND_ARIB)
	phy_init(SERIAL_IF_ENABLE, WB_ARIB);
#else /* Default */
	phy_init(SERIAL_IF_ENABLE, WB_CENELEC_A);
#endif
	
	/* Force VLO impedance mode */
	uc_value = 0;
	phy_set_cfg_param(PHY_ID_CFG_AUTODETECT_IMPEDANCE, &uc_value, 1);
	uc_value = 2;
	phy_set_cfg_param(PHY_ID_CFG_IMPEDANCE, &uc_value, 1);
}

static void phy_tester_tool_process(void)
{
	/* phy process */
	phy_process();
}

/**
 * \brief Main code entry point.
 */
int main(void)
{
	oss_task_t x_task = {0};

	/* Initialize OSS */
	oss_init();

	/* Register PHY Task */
	x_task.task_init = phy_tester_tool_init;
	x_task.task_process = phy_tester_tool_process;
	x_task.task_1ms_timer_cb = NULL;
	oss_register_task(&x_task);

	/* Start OSS */
	oss_start();
}
