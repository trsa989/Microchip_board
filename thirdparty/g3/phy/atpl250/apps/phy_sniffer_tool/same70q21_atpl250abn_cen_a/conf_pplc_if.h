/**
 * \file
 *
 * \brief PPLC interface Configuration.
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

#ifndef CONF_PPLC_IF_H_INCLUDE
#define CONF_PPLC_IF_H_INCLUDE

#include "board.h"

/* Select the SPI module that PPLC is connected to */
#define SPI_MASTER_BASE     SPI0
/* Spi Hw ID */
#define SPI_ID              ID_SPI0

/* Chip select used by PPLC internal peripheral  */
#define PPLC_CS             2

/* Interruption pin used by PPLC internal peripheral */
#define PPLC_INT_GPIO  (PIO_PD24_IDX)
#define PPLC_INT_FLAGS (IOPORT_MODE_DEBOUNCE)
#define PPLC_INT_SENSE (IOPORT_SENSE_FALLING)

#define PPLC_INT       {PIO_PD24, PIOD, ID_PIOD, PIO_INPUT, PIO_DEBOUNCE | PIO_IT_FALL_EDGE}
#define PIN_INT_SPI_MASK  PIO_PD24
#define PIN_INT_SPI_PIO   PIOD
#define PIN_INT_SPI_PIO_IDX     PIO_PD24_IDX
#define PIN_INT_SPI_ID    ID_PIOD
#define PPLC_INT_TYPE  PIO_INPUT
#define PIN_INT_SPI_ATTR  (PIO_DEBOUNCE | PIO_IT_FALL_EDGE)
#define PIN_INT_SPI_FLAGS       PIO_INPUT | PIO_DEGLITCH | PIO_IT_FALL_EDGE
#define PPLC_INT_IRQn  PIOD_IRQn

#define PIN_RST_ATPL250        {PIO_PA23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_RST_ATPL250_MASK   PIO_PA23
#define PIN_RST_ATPL250_PIO    PIOA
#define PIN_RST_ATPL250_ID     ID_PIOA
#define PIN_RST_ATPL250_TYPE   PIO_OUTPUT_1
#define PIN_RST_ATPL250_ATTR   PIO_DEFAULT
#define RST_ATPL250_GPIO       (PIO_PA23_IDX)
#define RST_ATPL250_FLAGS      (PIO_OUTPUT_1 | PIO_DEFAULT)

/* Asynchronous PPLC Reset pin definition */
/* #define PPLC_ARST_GPIO             (PIO_PB0_IDX) */
/* #define PPLC_ARST_ACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW */
/* #define PPLC_ARST_INACTIVE_LEVEL  IOPORT_PIN_LEVEL_HIGH */

/* Wrapper macros to ensure common naming across all boards */
/* #define PPLC_ARST       {PIO_PB0, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT} */
/* #define PPLC_ARST_MASK   PIO_PB0 */
/* #define PPLC_ARST_PIO    PIOB */
/* #define PPLC_ARST_ID     ID_PIOB */
/* #define PPLC_ARST_TYPE   PIO_OUTPUT_1 */
/* #define PPLC_ARST_ATTR   PIO_DEFAULT */

/* Synchronous PPLC Reset pin definition */
/* #define PPLC_SRST_GPIO             (PIO_PA25_IDX) */
/* #define PPLC_SRST_ACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW */
/* #define PPLC_SRST_INACTIVE_LEVEL  IOPORT_PIN_LEVEL_HIGH */

/* Wrapper macros to ensure common naming across all boards */
/* #define PPLC_SRST       {PIO_PA25, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} */
/* #define PPLC_SRST_MASK   PIO_PA25 */
/* #define PPLC_SRST_PIO    PIOA */
/* #define PPLC_SRST_ID     ID_PIOA */
/* #define PPLC_SRST_TYPE   PIO_OUTPUT_1 */
/* #define PPLC_SRST_ATTR   PIO_DEFAULT */

#endif  /* CONF_PPLC_IF_H_INCLUDE */
