/**
 * \file
 *
 * \brief ATMEL PLC Phy Sniffer Example
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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
 *  \mainpage ATMEL PLC Phy Sniffer Example
 *
 *  \section Purpose
 *
 *  The Phy Sniffer example uses the Phy layer to monitor data traffic and then
 * sends it via serial communications to the ATPL Multiprotocol Sniffer Tool.
 *
 *  \section Requirements
 *
 *  This package should be used with any PLC board on which there is PLC hardware dedicated.
 *
 *  \section Description
 *
 *  This application will configure the G3 PHY and its serial interface to communicate with
 * ATMEL ATPL Multiprotocol Sniffer Tool.
 *
 *  \section Usage
 *
 *  The tool is ready to monitor data traffic.
 *
 */

/* System includes */
#include <stdint.h>
#include <stdio.h>
#include "string.h"

/* Atmel library includes. */
#include "asf.h"

/* Example configuration. */
#include "conf_project.h"

#define SERIAL_IF_ENABLE        0

#define STRING_EOL    "\r"
#define STRING_HEADER "-- ATMEL PLC Phy Tester Tool Application --\r\n"	\
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

static void phy_sniffer_tool_init(void)
{
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

	/* Init sniffer IF */
	sniffer_if_init();
}

static void phy_sniffer_tool_process(void)
{
	/* phy process */
	phy_process();
}

/**
 * \brief Main code entry point.
 */
int main( void )
{
	oss_task_t x_task = {0};

	/* Initialize OSS */
	oss_init();

	/* Display application startup printout */
	puts(STRING_HEADER);

	/* Register PHY Task */
	x_task.task_init = phy_sniffer_tool_init;
	x_task.task_process = phy_sniffer_tool_process;
	x_task.task_1ms_timer_cb = NULL;
	oss_register_task(&x_task);

	/* Start OSS */
	oss_start();
}
