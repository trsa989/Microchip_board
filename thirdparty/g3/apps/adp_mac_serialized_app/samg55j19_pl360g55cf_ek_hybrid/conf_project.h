/**
 * \file
 *
 * \brief Example configuration for ATMEL G3 Conformance application
 *
 * Copyright (c) 2018 Atmel Corporation. All rights reserved.
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

#ifndef CONF_PROJECT_H_INCLUDED
#define CONF_PROJECT_H_INCLUDED

/* Define work band */
#define CONF_BAND_CENELEC_A

/* Port mapping for USI and Console */

/* USI. Select One or None of the following options */
#define USI_ON_MIKROBUS_USART
/*#define USI_ON_USB*/

/* Console. Select One or None of the following options */
/*#define CONSOLE_ON_MIKROBUS_USART*/
//#define CONSOLE_ON_USB

#if defined(USI_ON_MIKROBUS_USART) && defined(CONSOLE_ON_MIKROBUS_USART)
#error "USI and Console cannot be both mapped to MikroBUS USART"
#endif
#if defined(USI_ON_USB) && defined(CONSOLE_ON_USB)
#error "USI and Console cannot be both mapped to USB port"
#endif

#ifdef CONSOLE_ON_USB 
	#define CONF_BOARD_UDC_CONSOLE
#endif

/* Enable restoring PIB values from flash */
#define ENABLE_PIB_RESTORE

/* CONF_MULTIBAND_FCC_CENA enables the dynamic selection of binary file to load on PL360 device
 * depending on G3 stack band configuration
 * This has to be inline with a linker configuration that links more than one PL360 binary file */
#define CONF_MULTIBAND_FCC_CENA

/* PL360G55CF_EK: Enable PLCOUP011 configuration */
#define CONF_ENABLE_C11_CFG

#endif /* CONF_PROJECT_H_INCLUDED */
