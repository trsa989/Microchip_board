/**
 * \file
 *
 * \brief Monitor functions for SERIAL on SAM0
 *
 * Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef _MONITOR_SERIAL_H_
#define _MONITOR_SERIAL_H_

#define SERIAL_BOOT_VERSION              "2.2"

/* Selects USART as the communication interface of the monitor */
#define SERIAL_INTERFACE_USART      1
/* Selects USB as the communication interface of the monitor */
#define SERIAL_INTERFACE_USBCDC     0

/* Selects USB as the communication interface of the monitor */
#define SIZEBUFMAX                  64

extern bool usart_timeout; /* Notify serial monitor that some kind of timeout occured */

/**
 * \brief Initialize the monitor
 *
 */
void serial_monitor_init(uint8_t com_interface);

/**
 * \brief Main function of the SERIAL Monitor
 *
 */
void serial_monitor_run(void);

/**
 * \brief
 */
void serial_putdata_term(uint8_t *data, uint32_t length);

/**
 * \brief
 */
void call_app(uint32_t address);
void check_start_application_from_console(void);

#endif /* _MONITOR_SERIAL_H_ */
