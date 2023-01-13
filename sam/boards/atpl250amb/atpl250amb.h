/**
 * \file
 *
 * \brief ATPL250AMB Board Definition.
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

#ifndef ATPL250AMB_H_INCLUDED
#define ATPL250AMB_H_INCLUDED

#include "compiler.h"
#include "conf_board.h"

/**
 * \ingroup group_common_boards
 * \defgroup atpl250amb_group "ATPL250AMB"
 * Evaluation Board for atpl250a devices.
 *
 * @{
 */

/**
 * \defgroup atpl250amb_board_info_group "ATPL250AMB - Board informations"
 * Definitions related to the board description.
 *
 * @{
 */

/** Name of the board */
#define BOARD_NAME "ATPL250AMB"
/** Board definition */
#define atpl250amb
/** Family definition (already defined) */
#define sam4c
/** Core definition */
#define cortexm4
/** Board revision definition */
//#define BOARD_REV_0     0
#define BOARD_REV_1     1
//#define BOARD_REV_2     2
//#define BOARD_REV_3     3

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
#define BOARD_FREQ_MAINCK_XTAL      (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS    (12000000U)
/* @} */

/** Master clock frequency */
#define BOARD_MCK                   CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US        15625U

/* @} */

/**
 * \defgroup atpl250amb_features_group "ATPL250AMB - Features"
 * Symbols that describe features and capabilities of the board.
 *
 * @{
 */

 /**
	 * \name LED #0 pin definition
	 * @{
	 */
	#define LED0_GPIO            (PIO_PB14_IDX)
	#define LED0_ACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW
	#define LED0_INACTIVE_LEVEL  IOPORT_PIN_LEVEL_HIGH

	/* Wrapper macros to ensure common naming across all boards */
	#define LED_0_NAME      "green LED"
	#define PIN_LED_0       {PIO_PB14, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}
	#define PIN_LED_0_MASK   PIO_PB14
	#define PIN_LED_0_PIO    PIOB
	#define PIN_LED_0_ID     ID_PIOB
	#define PIN_LED_0_TYPE   PIO_OUTPUT_1
	#define PIN_LED_0_ATTR   PIO_DEFAULT
	/* @} */


#if BOARD_REV == BOARD_REV_1

	/**
	 * \name LED #1 pin definition
	 * @{
	 */
	#define LED1_GPIO            (PIO_PB15_IDX)
	#define LED1_ACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW
	#define LED1_INACTIVE_LEVEL  IOPORT_PIN_LEVEL_HIGH

	/* Wrapper macros to ensure common naming across all boards */
	#define LED_1_NAME      "red LED"
	#define PIN_LED_1       {PIO_PB15, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}
	#define PIN_LED_1_MASK   PIO_PB15
	#define PIN_LED_1_PIO    PIOB
	#define PIN_LED_1_ID     ID_PIOB
	#define PIN_LED_1_TYPE   PIO_OUTPUT_1
	#define PIN_LED_1_ATTR   PIO_DEFAULT
	/* @} */
#else
	#error No known ATPL250AMB revision board defined
#endif

/**
 * \name TC pins definition
 * @{
 */
#define PIN_TC0_TIOA0        (PIO_PA13_IDX)
#define PIN_TC0_TIOA0_MUX    (IOPORT_MODE_MUX_B)
#define PIN_TC0_TIOA0_FLAGS  (IOPORT_MODE_MUX_B)

/**
 * \name UART0 pis (UTXD0 and URXD0) definitions
 * @{
 */
#define PINS_UART0        (PIO_PB4A_URXD0 | PIO_PB5A_UTXD0)
#define PINS_UART0_FLAGS  (IOPORT_MODE_MUX_A)

#define PINS_UART0_PORT   IOPORT_PIOB
#define PINS_UART0_MASK   (PIO_PB4A_URXD0 | PIO_PB5A_UTXD0)
#define PINS_UART0_PIO    PIOB
#define PINS_UART0_ID     ID_PIOB
#define PINS_UART0_TYPE   PIO_PERIPH_A
#define PINS_UART0_ATTR   PIO_DEFAULT
/* @} */

/**
 * \name UART1 pis (UTXD1 and URXD1) definitions
 * @{
 */
#define PINS_UART1        (PIO_PC1A_URXD1 | PIO_PC0A_UTXD1)
#define PINS_UART1_FLAGS  (IOPORT_MODE_MUX_A)

#define PINS_UART1_PORT   IOPORT_PIOC
#define PINS_UART1_MASK   (PIO_PC1A_URXD1 | PIO_PC0A_UTXD1)
#define PINS_UART1_PIO    PIOC
#define PINS_UART1_ID     ID_PIOC
#define PINS_UART1_TYPE   PIO_PERIPH_A
#define PINS_UART1_ATTR   PIO_DEFAULT
/* @} */

/**
 * \name SPI pin definitions
 * @{
 */
/** SPI0 MISO pin definition. */
#define SPI0_MISO_GPIO         (PIO_PA6_IDX)
#define SPI0_MISO_FLAGS        (IOPORT_MODE_MUX_A | IOPORT_MODE_PULLDOWN)
/** SPI0 MOSI pin definition. */
#define SPI0_MOSI_GPIO         (PIO_PA7_IDX)
#define SPI0_MOSI_FLAGS        (IOPORT_MODE_MUX_A)
/** SPI0 SPCK pin definition. */
#define SPI0_SPCK_GPIO         (PIO_PA8_IDX)
#define SPI0_SPCK_FLAGS        (IOPORT_MODE_MUX_A)
/** SPI0 chip select 0 pin definition. */
#define SPI0_NPCS0_GPIO        (PIO_PA5_IDX)
#define SPI0_NPCS0_FLAGS       (IOPORT_MODE_MUX_A)

/* @} */


/**
 * \name TWIx pin definitions
 * @{
 */
/*! TWI0 Data pin for EEPROM */
#define TWIO_DATA_GPIO            PIO_PA24_IDX
#define TWIO_DATA_FLAG            IOPORT_MODE_MUX_A
/*! TWI0 Clock pin for EEPROM */
#define TWIO_CLK_GPIO             PIO_PA25_IDX
#define TWIO_CLK_FLAG             IOPORT_MODE_MUX_A
#define BOARD_CLK_TWI_EEPROM      TWIO_CLK_GPIO
#define BOARD_CLK_TWI_MUX_EEPROM  TWIO_CLK_FLAG
/* @} */

/**
 * \name USARTx pin definitions
 * @{
 */
/** USART0 pin RTS */
#define PIN_USART0_RTS_IDX    (PIO_PA19_IDX)
#define PIN_USART0_RTS_FLAGS  (IOPORT_MODE_MUX_A)
/** USART0 pin RX */
#define PIN_USART0_RXD_IDX    (PIO_PB16_IDX)
#define PIN_USART0_RXD_FLAGS  (IOPORT_MODE_MUX_A)
/** USART0 pin SCK */
#define PIN_USART0_SCK_IDX    (PIO_PA20_IDX)
#define PIN_USART0_SCK_FLAGS  (IOPORT_MODE_MUX_A)
/** USART0 pin TX */
#define PIN_USART0_TXD_IDX    (PIO_PB17_IDX)
#define PIN_USART0_TXD_FLAGS  (IOPORT_MODE_MUX_A)

/** USART1 pin RTS */
#define PIN_USART1_RTS_IDX    (PIO_PA17_IDX)
#define PIN_USART1_RTS_FLAGS  (IOPORT_MODE_MUX_A)
/** USART1 pin RX */
#define PIN_USART1_RXD_IDX    (PIO_PA11_IDX)
#define PIN_USART1_RXD_FLAGS  (IOPORT_MODE_MUX_A)
/** USART1 pin SCK */
#define PIN_USART1_SCK_IDX    (PIO_PA18_IDX)
#define PIN_USART1_SCK_FLAGS  (IOPORT_MODE_MUX_A)
/** USART1 pin TX */
#define PIN_USART1_TXD_IDX    (PIO_PA12_IDX)
#define PIN_USART1_TXD_FLAGS  (IOPORT_MODE_MUX_A)

/** USART2 pin RTS */
#define PIN_USART2_RTS_IDX    (PIO_PA14_IDX)
#define PIN_USART2_RTS_FLAGS  (IOPORT_MODE_MUX_A)
/** USART2 pin RX */
#define PIN_USART2_RXD_IDX    (PIO_PA9_IDX)
#define PIN_USART2_RXD_FLAGS  (IOPORT_MODE_MUX_A)
/** USART2 pin SCK */
#define PIN_USART2_SCK_IDX    (PIO_PA13_IDX)
#define PIN_USART2_SCK_FLAGS  (IOPORT_MODE_MUX_A)
/** USART2 pin TX */
#define PIN_USART2_TXD_IDX    (PIO_PA10_IDX)
#define PIN_USART2_TXD_FLAGS  (IOPORT_MODE_MUX_A)
/* @} */


/**
 * \name Voltage Monitor pins definition
 * @{
 */
#define VZ_CROSS_GPIO    PIO_PB11_IDX
#define V5V_SENSE_GPIO   PIO_PB23_IDX
#define VDD_SENSE_GPIO   PIO_PB31_IDX
/* @} */


/* @} */ /* End of atpl250amb_features_group */

/* @} */ /* End of atpl250amb_group */

#endif  /* ATPL250AMB_H_INCLUDED */

