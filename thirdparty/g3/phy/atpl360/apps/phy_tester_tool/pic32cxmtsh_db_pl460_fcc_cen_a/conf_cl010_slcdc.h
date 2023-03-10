/**
 * \file
 *
 * \brief CL010 configuration.
 *
 * Copyright (c) 2020 Microchip Corporation. All rights reserved.
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
 * 3. The name of Microchip may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Microchip microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY MICROCHIP "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL MICROCHIP BE LIABLE FOR
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

/** Configuration of the CL010 LCD glass driver */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Microchip Support</a>
 */

#ifndef CONF_CL010_SLCDC_H_INCLUDED
#define CONF_CL010_SLCDC_H_INCLUDED

/** LCD buffer on-time */
#define CONF_CL010_BUF_TIME          SLCDC_BUFTIME_PERCENT_100

/** LCD frame rate value */
#define CONF_CL010_FRAME_RATE        64

/** LCD display mode */
#define CONF_CL010_DISP_MODE         SLCDC_DISPMODE_NORMAL

/** LCD power mode */
#define CONF_CL010_POWER_MODE        SLCDC_POWER_MODE_LCDON_INVR

/** LCD blinking frequency */
#define CONF_CL010_BLINK_FREQ        2

/** LCD Contrast value */
#define CONF_CL010_CONTRAST          8

#endif /* CONF_CL010_SLCDC_H_INCLUDED */
