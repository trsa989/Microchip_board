/**
 * \file
 *
 * \brief Serial USART service configuration.
 *
 * Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
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

#ifndef CONF_USART_SERIAL_H
#define CONF_USART_SERIAL_H

/** UART Interface */
#define CONF_CONSOLE_UART               USART6
/** UART Interface */
#define CONF_CONSOLE_UART_ID            ID_FLEXCOM6
/** Baudrate setting */
#define CONF_CONSOLE_UART_BAUDRATE      (115200UL)
/** Parity setting */
#define CONF_CONSOLE_UART_PARITY        US_MR_PAR_NO
/** Char Length setting */
#define CONF_CONSOLE_UART_CHAR_LENGTH   US_MR_CHRL_8_BIT
/** Stop bits setting */
#define CONF_CONSOLE_UART_STOP_BITS     US_MR_NBSTOP_1_BIT
/** UART IRQn */
#define CONF_CONSOLE_UART_IRQn          CONSOLE_UART_IRQn
/** UART Handler */
#define CONF_CONSOLE_UART_Handler       CONSOLE_UART_Handler

#endif /* CONF_USART_SERIAL_H_INCLUDED */
