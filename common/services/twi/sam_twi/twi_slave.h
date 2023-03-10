/**
 * \file
 *
 * \brief TWI Slave driver for SAM.
 *
 * Copyright (c) 2011-2020 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef _TWI_SLAVE_H_
#define _TWI_SLAVE_H_

#include "twi.h"
#include "sysclk.h"

typedef Twi *twi_slave_t;

static inline void twi_slave_setup(twi_slave_t p_twi, uint32_t dw_device_addr)
{
#if SAMG55
	if (p_twi == TWI0) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM0);
	} else if (p_twi == TWI1) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM1);
	} else if (p_twi == TWI2) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM2);
	} else if (p_twi == TWI3) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM3);
	} else if (p_twi == TWI4) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM4);
	} else if (p_twi == TWI5) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM5);
	} else if (p_twi == TWI6) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM6);
#ifdef _SAMG55_FLEXCOM7_INSTANCE_
	} else if (p_twi == TWI7) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM7);
#endif /* _SAMG55_FLEXCOM7_INSTANCE_*/
	} else {
		// Do Nothing
	}
#elif PIC32CX
	if (p_twi == TWI0) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM0);
	} else if (p_twi == TWI1) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM1);
	} else if (p_twi == TWI2) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM2);
	} else if (p_twi == TWI3) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM3);
	} else if (p_twi == TWI4) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM4);
	} else if (p_twi == TWI5) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM5);
	} else if (p_twi == TWI6) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM6);
	} else if (p_twi == TWI7) {
		sysclk_enable_peripheral_clock(ID_FLEXCOM7);
	} else {
		// Do Nothing
	}
#else
#if (!(SAMG51 || SAMG53 || SAMG54))
	if (p_twi == TWI0) {
		sysclk_enable_peripheral_clock(ID_TWI0);
	} else
#endif
	if (p_twi == TWI1) {
		sysclk_enable_peripheral_clock(ID_TWI1);
#if (SAM4N || SAMG)
	} else if (p_twi == TWI2) {
		sysclk_enable_peripheral_clock(ID_TWI2);
#endif
	} else {
		// Do Nothing
	}
#endif

	twi_slave_init(p_twi, dw_device_addr);
}

#define twi_slave_enable(p_twi)  twi_enable_slave_mode(p_twi)

#define twi_slave_disable(p_twi)  twi_disable_slave_mode(p_twi)

#endif // _TWI_SLAVE_H_
