/**
 * \file
 *
 * \brief USI Layer Configuration
 *
 * Copyright (c) 2019 Atmel Corporation. All rights reserved.
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

#ifndef CONF_USI_H_INCLUDED
#define CONF_USI_H_INCLUDED

#define USI_PORT_0      0

/* Port Communications configuration */
#define NUM_PORTS                               1
/* In case of use USB_TYPE, refer to conf_usb.h in order to set baudrate by default */
#define PORT_0 CONF_PORT(USB_TYPE, 0, 230400, 1024, 1024)
/* In case of USI on Mikro BUS, comment previous line, and uncomment next line and conf_busart_if.h configurations */
/* #define PORT_0 CONF_PORT(USART_TYPE, 0, 230400, 1024, 1024) */

/* Select PORT to serialize phy iface */
#define PHY_IFACE_SERIAL_PORT                  USI_PORT_0
#define PHY_RF_IFACE_SERIAL_PORT               USI_PORT_0

#endif /* CONF_USI_H_INCLUDED */
