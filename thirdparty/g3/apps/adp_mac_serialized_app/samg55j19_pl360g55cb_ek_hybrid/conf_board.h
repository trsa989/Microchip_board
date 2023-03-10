/**
 *
 * \file
 *
 * \brief PL360G55CF-EK board configuration.
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

#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

/* Keep watchdog at board initialization (not disable it) */
#define CONF_BOARD_KEEP_WATCHDOG_AT_INIT

/* Configure console UART (mikroBUS TX and RX pins) */
#define CONF_BOARD_UART_CONSOLE
#define CONSOLE_UART                   MIKRO_BUS_USART
#define CONSOLE_UART_ID                ID_FLEXCOM4

/* Configure MIKROBUS_SPI (connected to ATREB215-XPRO SPI) */
#define CONF_BOARD_SPI_MIKROBUS

/* RF215 Reset pin definition */
#define RF215_RESET_GPIO               PIO_PA23_IDX
#define RF215_RESET_ACTIVE_LEVEL       IOPORT_PIN_LEVEL_LOW
#define RF215_RESET_INACTIVE_LEVEL     IOPORT_PIN_LEVEL_HIGH

/* RF215 interrupt pin definition */
#define RF215_INT_GPIO                 PIO_PA14_IDX
#define RF215_INT_ATTR                 (PIO_DEGLITCH | PIO_IT_HIGH_LEVEL)

/* ATREB215-XPRO board LEDs */
#define RF215_LED1                     PIO_PB3_IDX
#define RF215_LED2                     PIO_PB2_IDX

#endif /* CONF_BOARD_H_INCLUDED */
