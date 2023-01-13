/**
 * \file
 *
 * \brief ATPL250CH Board Definition.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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

#ifndef ATPL250CH_H_INCLUDED
#define ATPL250CH_H_INCLUDED

#include "compiler.h"
#include "conf_board.h"

/**
 * \ingroup group_common_boards
 * \defgroup atpl250ch_group "ATPL250CH"
 * Evaluation Module for atpl250a devices.
 *
 * @{
 */

/**
 * \defgroup atpl250ch_board_info_group "ATPL250CH - Board informations"
 * Definitions related to the board description.
 *
 * @{
 */

/** Name of the board */
#define BOARD_NAME "ATPL250CH"
/** Board definition */
#define atpl250ch
/** Family definition (already defined) */
#define samg55
/** Core definition */
#define cortexm4
/** Board revision definition */
#define BOARD_REV_1     1


#ifndef BOARD_REV
#define BOARD_REV BOARD_REV_1
#endif

/* @} */

/**
 *  \defgroup atpl250amb_opfreq_group "ATPL250AMB - Operating frequencies"
 *  Definitions related to the board operating frequency.
 *
 *  @{
 */

/**
 * \name Board oscillator settings
 * @{
 */
#define BOARD_FREQ_SLCK_XTAL        (32768U)
#define BOARD_FREQ_SLCK_BYPASS      (32768U)
#define BOARD_FREQ_MAINCK_XTAL      0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS    0 /* Not Mounted */
#define BOARD_MCK                   CHIP_FREQ_CPU_MAX
/*TBD startup time needs to be adjusted according to measurements */
#define BOARD_OSC_STARTUP_US        15625U
/* @} */

/**
 * \defgroup atpl250amb_features_group "ATPL250CH - Features"
 * Symbols that describe features and capabilities of the board.
 *
 * @{
 */

 //LEDs definitions
 
 /**
 * \name LED #0 pin definition
 * @{
 */
#define LED0_GPIO            (PIO_PA31_IDX)
#define LED0_ACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW
#define LED0_INACTIVE_LEVEL  IOPORT_PIN_LEVEL_HIGH

/* Wrapper macros to ensure common naming across all boards */
#define LED_0_NAME      "green LED"
#define PIN_LED_0       {PIO_PA31, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_LED_0_MASK   PIO_PA31
#define PIN_LED_0_PIO    PIOA
#define PIN_LED_0_ID     ID_PIOA
#define PIN_LED_0_TYPE   PIO_OUTPUT_1
#define PIN_LED_0_ATTR   PIO_DEFAULT
/* @} */

/**
 * \name LED #1 pin definition
 * @{
 */
#define LED1_GPIO            (PIO_PA17_IDX)
#define LED1_ACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW
#define LED1_INACTIVE_LEVEL  IOPORT_PIN_LEVEL_HIGH

/* Wrapper macros to ensure common naming across all boards */
#define LED_1_NAME      "user green LED"
#define PIN_LED_1       {PIO_PA17, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_LED_1_MASK   PIO_PA17
#define PIN_LED_1_PIO    PIOA
#define PIN_LED_1_ID     ID_PIOA
#define PIN_LED_1_TYPE   PIO_OUTPUT_1
#define PIN_LED_1_ATTR   PIO_DEFAULT
/* @} */

/**
 * \name LED #2 pin definition
 * @{
 */
#define LED2_GPIO            (PIO_PA18_IDX)
#define LED2_ACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW
#define LED2_INACTIVE_LEVEL  IOPORT_PIN_LEVEL_HIGH

/* Wrapper macros to ensure common naming across all boards */
#define LED_2_NAME      "user red LED"
#define PIN_LED_2       {PIO_PA18, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_LED_2_MASK   PIO_PA18
#define PIN_LED_2_PIO    PIOA
#define PIN_LED_2_ID     ID_PIOA
#define PIN_LED_2_TYPE   PIO_OUTPUT_1
#define PIN_LED_2_ATTR   PIO_DEFAULT
/* @} */

/**
 * \name TC pins definition
 * @{
 */
#define PIN_TC_TIOA0        (PIO_PA0_IDX)
#define PIN_TC_TIOA0_MUX    (IOPORT_MODE_MUX_B)
#define PIN_TC_TIOA0_FLAGS  (IOPORT_MODE_MUX_B)

/** USART5 pins (TXD5 and RXD5) definitions, PA13,12. */
#define PINS_USART5       (PIO_PA12A_RXD5| PIO_PA13A_TXD5)
#define PINS_USART5_FLAGS  (IOPORT_MODE_MUX_A)

#define PINS_USART5_PORT   IOPORT_PIOA
#define PINS_USART5_MASK   (PIO_PA12A_RXD5 | PIO_PA13A_TXD5)
#define PINS_USART5_PIO    PIOA
#define PINS_USART5_ID     ID_PIOA
#define PINS_USART5_TYPE   PIO_PERIPH_A
#define PINS_USART5_ATTR   PIO_DEFAULT

/** USART1 pins (TXD1 and RXD1) definitions, PB2,3. */
#define PINS_USART1       (PIO_PB2A_RXD1| PIO_PB3A_TXD1)
#define PINS_USART1_FLAGS  (IOPORT_MODE_MUX_A)

#define PINS_USART1_PORT   IOPORT_PIOB
#define PINS_USART1_MASK   (PIO_PB2A_RXD1 | PIO_PB3A_TXD1)
#define PINS_USART1_PIO    PIOB
#define PINS_USART1_ID     ID_PIOB
#define PINS_USART1_TYPE   PIO_PERIPH_A
#define PINS_USART1_ATTR   PIO_DEFAULT

/** USART0 pins (TXD0 and RXD0) definitions, PA9,10. */
#define PINS_USART0       (PIO_PA9A_RXD0| PIO_PA10A_TXD0)
#define PINS_USART0_FLAGS  (IOPORT_MODE_MUX_A)

#define PINS_USART0_PORT   IOPORT_PIOA
#define PINS_USART0_MASK   (PIO_PA9A_RXD0 | PIO_PA10A_TXD0)
#define PINS_USART0_PIO    PIOA
#define PINS_USART0_ID     ID_PIOA
#define PINS_USART0_TYPE   PIO_PERIPH_A
#define PINS_USART0_ATTR   PIO_DEFAULT

/**
 * \name SPI pin definitions
 * @{
 */
 /** SPI3 MISO pin definition. */
#define SPI3_MISO_GPIO         (PIO_PA4_IDX)
#define SPI3_MISO_FLAGS       (IOPORT_MODE_MUX_A)
/** SPI3 MOSI pin definition. */
#define SPI3_MOSI_GPIO         (PIO_PA3_IDX)
#define SPI3_MOSI_FLAGS       (IOPORT_MODE_MUX_A)
/** SPI3 SPCK pin definition. */
#define SPI3_SPCK_GPIO         (PIO_PB13_IDX)
#define SPI3_SPCK_FLAGS       (IOPORT_MODE_MUX_A)

/** SPI3 chip select 0 pin definition. */
#define SPI3_NPCS0_GPIO        (PIO_PB14_IDX)
#define SPI3_NPCS0_FLAGS      (IOPORT_MODE_MUX_A)


/**
 * \name WKUP INPUT pin definitions
 * @{
 */
#define PIN_WKUP1_GPIO         (PIO_PA1_IDX)
#define PIN_WKUP3_GPIO         (PIO_PA23_IDX)
#define PIN_WKUP7_GPIO         (PIO_PA16_IDX)

/* @} */ /* End of atpl250ch_features_group */

/* @} */ /* End of atpl250ch_group */

#endif  /* ATPL250CH_H_INCLUDED */

