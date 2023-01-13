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
#define SPI_MASTER_BASE     SPI3
/* Spi Hw ID */
#define SPI_ID              ID_FLEXCOM3

/* Chip select used by PPLC internal peripheral  */
#define PPLC_CS             0

/* Interruption pin used by PPLC internal peripheral */
#define PPLC_INT_GPIO   (PIO_PA2_IDX)
#define PPLC_INT_FLAGS (IOPORT_MODE_DEBOUNCE)
#define PPLC_INT_SENSE (IOPORT_SENSE_FALLING)

#define PPLC_INT       {PIO_PA2, PIOA, ID_PIOA, PIO_INPUT, PIO_DEBOUNCE | PIO_IT_FALL_EDGE}
#define PIN_INT_SPI_MASK  PIO_PA2
#define PIN_INT_SPI_PIO   PIOA
#define PIN_INT_SPI_PIO_IDX     PIO_PA2_IDX
#define PIN_INT_SPI_ID    ID_PIOA
#define PPLC_INT_TYPE  PIO_INPUT
#define PIN_INT_SPI_ATTR  (PIO_DEBOUNCE | PIO_IT_FALL_EDGE)
#define PIN_INT_SPI_FLAGS       PIO_INPUT | PIO_DEGLITCH | PIO_IT_FALL_EDGE
#define PPLC_INT_IRQn  PIOA_IRQn

#define PIN_RST_ATPL250        {PIO_PA29, PIOA, ID_PIOA PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_RST_ATPL250_MASK   PIO_PA29
#define PIN_RST_ATPL250_PIO    PIOA
#define PIN_RST_ATPL250_ID     ID_PIOA
#define PIN_RST_ATPL250_TYPE   PIO_OUTPUT_1
#define PIN_RST_ATPL250_ATTR   PIO_DEFAULT
#define RST_ATPL250_GPIO       (PIO_PA29_IDX)
#define RST_ATPL250_FLAGS      (PIO_OUTPUT_1 | PIO_DEFAULT)

#endif  /* CONF_PPLC_IF_H_INCLUDE */
