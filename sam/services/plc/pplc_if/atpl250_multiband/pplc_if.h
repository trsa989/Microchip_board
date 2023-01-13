/**
 * \file
 *
 * \brief Proxy PLC Controller interface layer implementation.
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

#ifndef CHIP_PPLC_IF_H_INCLUDED
#define CHIP_PPLC_IF_H_INCLUDED

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \weakgroup plc_group
 * @{
 */

#include <stdbool.h>

/* LOG SPI */
#define LOG_SPI       0

/* JUMPS */
/*  No Jump */
#define JUMP_NO_JUMP        0x00
/*  Jump depending on Column 1 in RAM */
#define JUMP_COL_1          0x20
/*  Jump depending on Column 2 in RAM */
#define JUMP_COL_2          0x40
/*  Jump depending on Column 3 in RAM */
#define JUMP_COL_3          0x60
/*  R/W G3 coherent pilots */
#define JUMP_PILOTS         0x80
/*  R/W G3 coherent data without pilots */
#define JUMP_DATA_NO_PILOTS 0xA0
/*  Multibuffer writing */
#define JUMP_MULTIBUFFER    0xE0
/*  Frequency average depending on pilots */
#define JUMP_FREQAVG        0xC0

/* SPI clock setting (Hz). */
#define PPLC_CLOCK_30M  30000000  /* 30 MHz */
#define PPLC_CLOCK_24M  24000000  /* 24 MHz */
#define PPLC_CLOCK_15M  15000000  /* 15 MHz */

/* SPI interruption priority */
#define SPI_PRIO      11
/* Chip select. */
#define SPI_CHIP_SEL PPLC_CS
/* Clock polarity. */
#define SPI_CLK_POLARITY 0
/* Clock phase. */
#define SPI_CLK_PHASE 1
/* Delay before SPCK */
#define SPI_DLYBS 10
/* Delay between consecutive transfers */
#define SPI_DLYBCT 0

/* ! \name Proxy PLC Controller interface */
/* @{ */
void pplc_if_init(void);
uint8_t pplc_if_read8(uint16_t us_addr);
uint8_t pplc_if_write8(uint16_t us_addr, uint8_t uc_dat);
uint16_t pplc_if_read16(uint16_t us_addr);
uint8_t pplc_if_write16(uint16_t us_addr, uint16_t us_dat);
uint32_t pplc_if_read32(uint16_t us_addr);
uint8_t pplc_if_write32(uint16_t us_addr, uint32_t ul_dat);
uint8_t pplc_if_read_buf(uint16_t us_addr, uint8_t *buf, uint16_t us_len, bool block);
uint8_t pplc_if_write_buf(uint16_t us_addr, uint8_t *buf, uint16_t us_len);
uint8_t pplc_if_read_rep(uint16_t addrs, uint8_t bytesRepeat, uint8_t *buf, uint16_t len, bool block);
uint8_t pplc_if_write_rep(uint16_t us_addr, uint8_t uc_bytes_rep, uint8_t *ptr_buf, uint16_t us_len);

uint8_t pplc_if_read_jump(uint16_t addrs, uint8_t *buf, uint16_t len, uint8_t jump, bool block);
uint8_t pplc_if_write_jump(uint16_t addrs, uint8_t *buf, uint16_t len, uint8_t jump);

uint8_t pplc_if_do_read(uint8_t *buf, uint16_t len);

void pplc_if_and8(uint16_t us_addr, uint8_t uc_mask);
void pplc_if_or8(uint16_t us_addr, uint8_t uc_mask);
void pplc_if_xor8(uint16_t us_addr, uint8_t uc_mask);
void pplc_if_and32(uint16_t us_addr, uint32_t ul_mask);
void pplc_if_or32(uint16_t us_addr, uint32_t ul_mask);
void pplc_set_handler(void (*p_handler)(void));

void pplc_if_set_speed(uint32_t freq);

void pplc_if_config_atpl250_reset(void);
void pplc_if_push_atpl250_reset(void);
void pplc_if_release_atpl250_reset(void);

void enable_pplc_interrupt(void);
void disable_pplc_interrupt(void);

void pplc_enable_all_interrupts(void);
void pplc_disable_all_interrupts(void);
#if (LOG_SPI == 1)
void dumpLogSpi(void);
void addIntMarkToLogSpi(void);
void clearLogSpi(void);
#endif

/* @} */
/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* CHIP_PPLC_IF_H_INCLUDED */
