/**
 * \file
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
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

#ifndef _ATPL250REG_INCLUDED_
#define _ATPL250REG_INCLUDED_

/* Masks to apply for register access */
#define MASK_VH8    (0x4400)
#define MASK_H8     (0x2400)
#define MASK_L8     (0x1400)
#define MASK_VL8    (0x0C00)
#define MASK_H16    (0x6400)
#define MASK_L16    (0x1C00)
#define MASK_32     (0x7C00)

/* ========== Register definition for ATPL250 peripheral ========== */

/* / Mapped address for INOUT buffer control */
#define REG_ATPL250_CHIP_ID                          (0x0000)
#define REG_ATPL250_CHIP_ID_32                       (REG_ATPL250_CHIP_ID | MASK_32)
/* / Mapped address for INOUT buffer control */
#define REG_ATPL250_INOUTB_CTL                       (0x0001)
#define REG_ATPL250_INOUTB_CTL_32                    (REG_ATPL250_INOUTB_CTL | MASK_32)
#define REG_ATPL250_INOUTB_CTL_H16                   (REG_ATPL250_INOUTB_CTL | MASK_H16)
#define REG_ATPL250_INOUTB_CTL_L16                   (REG_ATPL250_INOUTB_CTL | MASK_L16)
#define REG_ATPL250_INOUTB_CTL_VH8                   (REG_ATPL250_INOUTB_CTL | MASK_VH8)
#define REG_ATPL250_INOUTB_CTL_H8                    (REG_ATPL250_INOUTB_CTL | MASK_H8)
#define REG_ATPL250_INOUTB_CTL_L8                    (REG_ATPL250_INOUTB_CTL | MASK_L8)
#define REG_ATPL250_INOUTB_CTL_VL8                   (REG_ATPL250_INOUTB_CTL | MASK_VL8)
/* / Mapped address for INOUT buffer config 1 */
#define REG_ATPL250_INOUTB_CONF1                     (0x0002)
#define REG_ATPL250_INOUTB_CONF1_32                  (REG_ATPL250_INOUTB_CONF1 | MASK_32)
#define REG_ATPL250_INOUTB_CONF1_H16                 (REG_ATPL250_INOUTB_CONF1 | MASK_H16)
#define REG_ATPL250_INOUTB_CONF1_L16                 (REG_ATPL250_INOUTB_CONF1 | MASK_L16)
/* / Mapped address for INOUT buffer config 2 */
#define REG_ATPL250_INOUTB_CONF2                     (0x0003)
#define REG_ATPL250_INOUTB_CONF2_32                  (REG_ATPL250_INOUTB_CONF2 | MASK_32)
#define REG_ATPL250_INOUTB_CONF2_VH8                 (REG_ATPL250_INOUTB_CONF2 | MASK_VH8)
#define REG_ATPL250_INOUTB_CONF2_H8                  (REG_ATPL250_INOUTB_CONF2 | MASK_H8)
#define REG_ATPL250_INOUTB_CONF2_L8                  (REG_ATPL250_INOUTB_CONF2 | MASK_L8)
#define REG_ATPL250_INOUTB_CONF2_VL8                 (REG_ATPL250_INOUTB_CONF2 | MASK_VL8)
/* / Mapped address for INOUT buffer config 3 */
#define REG_ATPL250_INOUTB_CONF3                     (0x0004)
#define REG_ATPL250_INOUTB_CONF3_L16                 (REG_ATPL250_INOUTB_CONF3 | MASK_L16)
#define REG_ATPL250_INOUTB_CONF3_L8                  (REG_ATPL250_INOUTB_CONF3 | MASK_L8)
#define REG_ATPL250_INOUTB_CONF3_VL8                 (REG_ATPL250_INOUTB_CONF3 | MASK_VL8)
/* / Mapped address for INOUT buffer instant */
#define REG_ATPL250_INOUTB_INST                      (0x0005)
#define REG_ATPL250_INOUTB_INST_32                   (REG_ATPL250_INOUTB_INST | MASK_32)
#define REG_ATPL250_INOUTB_INST_H16                  (REG_ATPL250_INOUTB_INST | MASK_H16)
#define REG_ATPL250_INOUTB_INST_L16                  (REG_ATPL250_INOUTB_INST | MASK_L16)
/* / Mapped address for INOUT buffer symbol time tx */
#define REG_ATPL250_INOUTB_SYMBOL_TIME_TX            (0x0006)
#define REG_ATPL250_INOUTB_SYMBOL_TIME_TX_32         (REG_ATPL250_INOUTB_SYMBOL_TIME_TX | MASK_32)
/* / Mapped address for INOUT buffer symbol time rx */
#define REG_ATPL250_INOUTB_SYMBOL_TIME_RX            (0x0007)
#define REG_ATPL250_INOUTB_SYMBOL_TIME_RX_32         (REG_ATPL250_INOUTB_SYMBOL_TIME_RX | MASK_32)
/* / Mapped address for INOUT buffer PSK values 1 */
#define REG_ATPL250_INOUTB_PSK_VALUES1               (0x0008)
#define REG_ATPL250_INOUTB_PSK_VALUES1_32            (REG_ATPL250_INOUTB_PSK_VALUES1 | MASK_32)
#define REG_ATPL250_INOUTB_PSK_VALUES1_H16           (REG_ATPL250_INOUTB_PSK_VALUES1 | MASK_H16)
#define REG_ATPL250_INOUTB_PSK_VALUES1_L16           (REG_ATPL250_INOUTB_PSK_VALUES1 | MASK_L16)
/* / Mapped address for INOUT buffer PSK values 0 */
#define REG_ATPL250_INOUTB_PSK_VALUES0               (0x0009)
#define REG_ATPL250_INOUTB_PSK_VALUES0_32            (REG_ATPL250_INOUTB_PSK_VALUES0 | MASK_32)
#define REG_ATPL250_INOUTB_PSK_VALUES0_H16           (REG_ATPL250_INOUTB_PSK_VALUES0 | MASK_H16)
#define REG_ATPL250_INOUTB_PSK_VALUES0_L16           (REG_ATPL250_INOUTB_PSK_VALUES0 | MASK_L16)
/* / Mapped address for INOUT buffer QUAM values */
#define REG_ATPL250_INOUTB_QAM_VALUES                (0x000A)
#define REG_ATPL250_INOUTB_QAM_VALUES_32             (REG_ATPL250_INOUTB_QAM_VALUES | MASK_32)
#define REG_ATPL250_INOUTB_QAM_VALUES_H16            (REG_ATPL250_INOUTB_QAM_VALUES | MASK_H16)
#define REG_ATPL250_INOUTB_QAM_VALUES_L16            (REG_ATPL250_INOUTB_QAM_VALUES | MASK_L16)
/* / Mapped address for INOUT buffer overflow */
#define REG_ATPL250_INOUTB_OVERFLOW                  (0x000B)
#define REG_ATPL250_INOUTB_OVERFLOW_32               (REG_ATPL250_INOUTB_OVERFLOW | MASK_32)
#define REG_ATPL250_INOUTB_OVERFLOW_H16              (REG_ATPL250_INOUTB_OVERFLOW | MASK_H16)
#define REG_ATPL250_INOUTB_OVERFLOW_VL8              (REG_ATPL250_INOUTB_OVERFLOW | MASK_VL8)
/* / Mapped address for INOUT buffer QNTZ */
#define REG_ATPL250_INOUTB_QNTZ1                     (0x000C)
#define REG_ATPL250_INOUTB_QNTZ1_32                  (REG_ATPL250_INOUTB_QNTZ1 | MASK_32)
#define REG_ATPL250_INOUTB_QNTZ1_VH8                 (REG_ATPL250_INOUTB_QNTZ1 | MASK_VH8)
#define REG_ATPL250_INOUTB_QNTZ2                     (0x000D)
#define REG_ATPL250_INOUTB_QNTZ2_32                  (REG_ATPL250_INOUTB_QNTZ2 | MASK_32)
#define REG_ATPL250_INOUTB_QNTZ2_VH8                 (REG_ATPL250_INOUTB_QNTZ2 | MASK_VH8)
#define REG_ATPL250_INOUTB_QNTZ3                     (0x000E)
#define REG_ATPL250_INOUTB_QNTZ3_32                  (REG_ATPL250_INOUTB_QNTZ3 | MASK_32)
#define REG_ATPL250_INOUTB_QNTZ3_VH8                 (REG_ATPL250_INOUTB_QNTZ3 | MASK_VH8)
#define REG_ATPL250_INOUTB_QNTZ4                     (0x000F)
#define REG_ATPL250_INOUTB_QNTZ4_32                  (REG_ATPL250_INOUTB_QNTZ4 | MASK_32)
#define REG_ATPL250_INOUTB_QNTZ4_VH8                 (REG_ATPL250_INOUTB_QNTZ4 | MASK_VH8)
#define REG_ATPL250_INOUTB_QNTZ4_VL8                 (REG_ATPL250_INOUTB_QNTZ4 | MASK_VL8)
/* / Mapped address for FFT config */
#define REG_ATPL250_FFT_CONFIG                       (0x0010)
#define REG_ATPL250_FFT_CONFIG_32                    (REG_ATPL250_FFT_CONFIG | MASK_32)
#define REG_ATPL250_FFT_CONFIG_VH8                   (REG_ATPL250_FFT_CONFIG | MASK_VH8)
#define REG_ATPL250_FFT_CONFIG_H8                    (REG_ATPL250_FFT_CONFIG | MASK_H8)
#define REG_ATPL250_FFT_CONFIG_L8                    (REG_ATPL250_FFT_CONFIG | MASK_L8)
#define REG_ATPL250_FFT_CONFIG_VL8                   (REG_ATPL250_FFT_CONFIG | MASK_VL8)
/* / Mapped address for UART32TX */
#define REG_ATPL250_SYNCM_MODULE                     (0x0012)
#define REG_ATPL250_SYNCM_MODULE_32                  (REG_ATPL250_SYNCM_MODULE | MASK_32)
/* / Mapped address for SYNCM value */
#define REG_ATPL250_SYNCM_VALUE                      (0x0013)
#define REG_ATPL250_SYNCM_VALUE_32                   (REG_ATPL250_SYNCM_VALUE | MASK_32)
#define REG_ATPL250_SYNCM_VALUE_H16                  (REG_ATPL250_SYNCM_VALUE | MASK_H16)
#define REG_ATPL250_SYNCM_VALUE_L16                  (REG_ATPL250_SYNCM_VALUE | MASK_L16)
/* / Mapped address for SYNCM control */
#define REG_ATPL250_SYNCM_CTL                        (0x0014)
#define REG_ATPL250_SYNCM_CTL_32                     (REG_ATPL250_SYNCM_CTL | MASK_32)
#define REG_ATPL250_SYNCM_CTL_H16                    (REG_ATPL250_SYNCM_CTL | MASK_H16)
#define REG_ATPL250_SYNCM_CTL_L16                    (REG_ATPL250_SYNCM_CTL | MASK_L16)
#define REG_ATPL250_SYNCM_CTL_VH8                    (REG_ATPL250_SYNCM_CTL | MASK_VH8)
#define REG_ATPL250_SYNCM_CTL_H8                     (REG_ATPL250_SYNCM_CTL | MASK_H8)
#define REG_ATPL250_SYNCM_CTL_L8                     (REG_ATPL250_SYNCM_CTL | MASK_L8)
#define REG_ATPL250_SYNCM_CTL_VL8                    (REG_ATPL250_SYNCM_CTL | MASK_VL8)
/* / Mapped address for SPI */
#define REG_ATPL250_SPI                              (0x0015)
#define REG_ATPL250_SPI_VL8                          (REG_ATPL250_SPI | MASK_VL8)
#define REG_ATPL250_SPI_DB_JUMP_C1R01                (0x0016)
#define REG_ATPL250_SPI_DB_JUMP_C1R01_32             (REG_ATPL250_SPI_DB_JUMP_C1R01 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C1R23                (0x0017)
#define REG_ATPL250_SPI_DB_JUMP_C1R23_32             (REG_ATPL250_SPI_DB_JUMP_C1R23 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C2R01                (0x0018)
#define REG_ATPL250_SPI_DB_JUMP_C2R01_32             (REG_ATPL250_SPI_DB_JUMP_C2R01 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C2R23                (0x0019)
#define REG_ATPL250_SPI_DB_JUMP_C2R23_32             (REG_ATPL250_SPI_DB_JUMP_C2R23 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C2R45                (0x001A)
#define REG_ATPL250_SPI_DB_JUMP_C2R45_32             (REG_ATPL250_SPI_DB_JUMP_C2R45 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C2R67                (0x001B)
#define REG_ATPL250_SPI_DB_JUMP_C2R67_32             (REG_ATPL250_SPI_DB_JUMP_C2R67 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C3R01                (0x001C)
#define REG_ATPL250_SPI_DB_JUMP_C3R01_32             (REG_ATPL250_SPI_DB_JUMP_C3R01 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C3R23                (0x001D)
#define REG_ATPL250_SPI_DB_JUMP_C3R23_32             (REG_ATPL250_SPI_DB_JUMP_C3R23 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C3R45                (0x001E)
#define REG_ATPL250_SPI_DB_JUMP_C3R45_32             (REG_ATPL250_SPI_DB_JUMP_C3R45 | MASK_32)
#define REG_ATPL250_SPI_DB_JUMP_C3R67                (0x001F)
#define REG_ATPL250_SPI_DB_JUMP_C3R67_32             (REG_ATPL250_SPI_DB_JUMP_C3R67 | MASK_32)
/* / Mapped address for Tx Gain */
#define REG_ATPL250_TXRXB_GAIN                       (0x0020)
#define REG_ATPL250_TXRXB_GAIN_32                    (REG_ATPL250_TXRXB_GAIN | MASK_32)
#define REG_ATPL250_TXRXB_GAIN_VL8                   (REG_ATPL250_TXRXB_GAIN | MASK_VL8)
/* / Mapped address for Tx Repetition */
#define REG_ATPL250_TXRXB_REPH                       (0x0021)
#define REG_ATPL250_TXRXB_REPH_32                    (REG_ATPL250_TXRXB_REPH | MASK_32)
#define REG_ATPL250_TXRXB_REPH_VH8                   (REG_ATPL250_TXRXB_REPH | MASK_VH8)
#define REG_ATPL250_TXRXB_REPH_H8                    (REG_ATPL250_TXRXB_REPH | MASK_H8)
#define REG_ATPL250_TXRXB_REPH_L8                    (REG_ATPL250_TXRXB_REPH | MASK_L8)
#define REG_ATPL250_TXRXB_REPH_VL8                   (REG_ATPL250_TXRXB_REPH | MASK_VL8)
#define REG_ATPL250_TXRXB_REPL                       (0x0022)
#define REG_ATPL250_TXRXB_REPL_32                    (REG_ATPL250_TXRXB_REPL | MASK_32)
#define REG_ATPL250_TXRXB_REPL_VH8                   (REG_ATPL250_TXRXB_REPL | MASK_VH8)
#define REG_ATPL250_TXRXB_REPL_H8                    (REG_ATPL250_TXRXB_REPL | MASK_H8)
#define REG_ATPL250_TXRXB_REPL_L8                    (REG_ATPL250_TXRXB_REPL | MASK_L8)
#define REG_ATPL250_TXRXB_REPL_VL8                   (REG_ATPL250_TXRXB_REPL | MASK_VL8)
/* / Mapped address for Tx Load */
#define REG_ATPL250_TXRXB_LOAD                       (0x0023)
#define REG_ATPL250_TXRXB_LOAD_32                    (REG_ATPL250_TXRXB_LOAD | MASK_32)
#define REG_ATPL250_TXRXB_LOAD_VH8                   (REG_ATPL250_TXRXB_LOAD | MASK_VH8)
#define REG_ATPL250_TXRXB_LOAD_H8                    (REG_ATPL250_TXRXB_LOAD | MASK_H8)
#define REG_ATPL250_TXRXB_LOAD_L8                    (REG_ATPL250_TXRXB_LOAD | MASK_L8)
#define REG_ATPL250_TXRXB_LOAD_VL8                   (REG_ATPL250_TXRXB_LOAD | MASK_VL8)
/* / Mapped address for Symbol lengths */
#define REG_ATPL250_TXRXB_SYM_LENGTHS                (0x0024)
#define REG_ATPL250_TXRXB_SYM_LENGTHS_32             (REG_ATPL250_TXRXB_SYM_LENGTHS | MASK_32)
#define REG_ATPL250_TXRXB_SYM_LENGTHS_H16            (REG_ATPL250_TXRXB_SYM_LENGTHS | MASK_H16)
#define REG_ATPL250_TXRXB_SYM_LENGTHS_L16            (REG_ATPL250_TXRXB_SYM_LENGTHS | MASK_L16)
/* / Mapped address for Symbol Config */
#define REG_ATPL250_TXRXB_SYM_CFG                    (0x0025)
#define REG_ATPL250_TXRXB_SYM_CFG_32                 (REG_ATPL250_TXRXB_SYM_CFG | MASK_32)
#define REG_ATPL250_TXRXB_SYM_CFG_VH8                (REG_ATPL250_TXRXB_SYM_CFG | MASK_VH8)
#define REG_ATPL250_TXRXB_SYM_CFG_H8                 (REG_ATPL250_TXRXB_SYM_CFG | MASK_H8)
#define REG_ATPL250_TXRXB_SYM_CFG_L8                 (REG_ATPL250_TXRXB_SYM_CFG | MASK_L8)
#define REG_ATPL250_TXRXB_SYM_CFG_VL8                (REG_ATPL250_TXRXB_SYM_CFG | MASK_VL8)
/* / Mapped address for Preamble Analysis */
#define REG_ATPL250_TXRXB_PRE_ANALYSIS               (0x0026)
#define REG_ATPL250_TXRXB_PRE_ANALYSIS_32            (REG_ATPL250_TXRXB_PRE_ANALYSIS | MASK_32)
#define REG_ATPL250_TXRXB_PRE_ANALYSIS_VH8           (REG_ATPL250_TXRXB_PRE_ANALYSIS | MASK_VH8)
#define REG_ATPL250_TXRXB_PRE_ANALYSIS_H8            (REG_ATPL250_TXRXB_PRE_ANALYSIS | MASK_H8)
#define REG_ATPL250_TXRXB_PRE_ANALYSIS_L8            (REG_ATPL250_TXRXB_PRE_ANALYSIS | MASK_L8)
#define REG_ATPL250_TXRXB_PRE_ANALYSIS_VL8           (REG_ATPL250_TXRXB_PRE_ANALYSIS | MASK_VL8)
/* / Mapped address for TxRx Buffer state */
#define REG_ATPL250_TXRXB_CFG                        (0x0027)
#define REG_ATPL250_TXRXB_CFG_L8                     (REG_ATPL250_TXRXB_CFG | MASK_L8)
#define REG_ATPL250_TXRXB_CFG_VL8                    (REG_ATPL250_TXRXB_CFG | MASK_VL8)
/* / Mapped address for TxRx Buffer state */
#define REG_ATPL250_TXRXB_STATE                      (0x0028)
#define REG_ATPL250_TXRXB_STATE_32                   (REG_ATPL250_TXRXB_STATE | MASK_32)
#define REG_ATPL250_TXRXB_STATE_VH8                  (REG_ATPL250_TXRXB_STATE | MASK_VH8)
#define REG_ATPL250_TXRXB_STATE_H8                   (REG_ATPL250_TXRXB_STATE | MASK_H8)
#define REG_ATPL250_TXRXB_STATE_L8                   (REG_ATPL250_TXRXB_STATE | MASK_L8)
#define REG_ATPL250_TXRXB_STATE_VL8                  (REG_ATPL250_TXRXB_STATE | MASK_VL8)
/* / Mapped address for TxRx Buffer offsets */
#define REG_ATPL250_TXRXB_OFFSETS                    (0x0029)
#define REG_ATPL250_TXRXB_OFFSETS_32                 (REG_ATPL250_TXRXB_OFFSETS | MASK_32)
#define REG_ATPL250_TXRXB_OFFSETS_H16                (REG_ATPL250_TXRXB_OFFSETS | MASK_H16)
#define REG_ATPL250_TXRXB_OFFSETS_L16                (REG_ATPL250_TXRXB_OFFSETS | MASK_L16)
/* / Mapped address for TxRx Buffer control */
#define REG_ATPL250_TXRXB_CTL                        (0x002A)
#define REG_ATPL250_TXRXB_CTL_32                     (REG_ATPL250_TXRXB_CTL | MASK_32)
#define REG_ATPL250_TXRXB_CTL_VH8                    (REG_ATPL250_TXRXB_CTL | MASK_VH8)
#define REG_ATPL250_TXRXB_CTL_H8                     (REG_ATPL250_TXRXB_CTL | MASK_H8)
#define REG_ATPL250_TXRXB_CTL_L8                     (REG_ATPL250_TXRXB_CTL | MASK_L8)
#define REG_ATPL250_TXRXB_CTL_VL8                    (REG_ATPL250_TXRXB_CTL | MASK_VL8)
#define REG_ATPL250_TXRXB_CTL_L16                    (REG_ATPL250_TXRXB_CTL | MASK_L16)
/* / Mapped address for Baudrate */
#define REG_ATPL250_TXRXB_BAUDRATE                   (0x002B)
#define REG_ATPL250_TXRXB_BAUDRATE_32                (REG_ATPL250_TXRXB_BAUDRATE | MASK_32)
#define REG_ATPL250_TXRXB_BAUDRATE_L16               (REG_ATPL250_TXRXB_BAUDRATE | MASK_L16)
/* / Mapped address for AGC */
#define REG_ATPL250_AGC_CTL                          (0x002C)
#define REG_ATPL250_AGC_CTL_32                       (REG_ATPL250_AGC_CTL | MASK_32)
#define REG_ATPL250_AGC_CTL_VH8                      (REG_ATPL250_AGC_CTL | MASK_VH8)
#define REG_ATPL250_AGC_CTL_H8                       (REG_ATPL250_AGC_CTL | MASK_H8)
#define REG_ATPL250_AGC_CTL_L8                       (REG_ATPL250_AGC_CTL | MASK_L8)
#define REG_ATPL250_AGC_CTL_VL8                      (REG_ATPL250_AGC_CTL | MASK_VL8)
#define REG_ATPL250_AGC_FINE_COMP                    (0x002D)
#define REG_ATPL250_AGC_FINE_COMP_32                 (REG_ATPL250_AGC_FINE_COMP | MASK_32)
#define REG_ATPL250_AGC_BETA0                        (0x002E)
#define REG_ATPL250_AGC_BETA0_32                     (REG_ATPL250_AGC_BETA0 | MASK_32)
#define REG_ATPL250_AGC_BETA1                        (0x002F)
#define REG_ATPL250_AGC_BETA1_32                     (REG_ATPL250_AGC_BETA1 | MASK_32)
#define REG_ATPL250_AGC_VREF2                        (0x0030)
#define REG_ATPL250_AGC_VREF2_32                     (REG_ATPL250_AGC_VREF2 | MASK_32)
#define REG_ATPL250_AGC_CUP01                        (0x0031)
#define REG_ATPL250_AGC_CUP01_32                     (REG_ATPL250_AGC_CUP01 | MASK_32)
#define REG_ATPL250_AGC_CUP23                        (0x0032)
#define REG_ATPL250_AGC_CUP23_32                     (REG_ATPL250_AGC_CUP23 | MASK_32)
#define REG_ATPL250_AGC_CDOWN12                      (0x0033)
#define REG_ATPL250_AGC_CDOWN12_32                   (REG_ATPL250_AGC_CDOWN12 | MASK_32)
#define REG_ATPL250_AGC_CDOWN34                      (0x0034)
#define REG_ATPL250_AGC_CDOWN34_32                   (REG_ATPL250_AGC_CDOWN34 | MASK_32)
#define REG_ATPL250_AGC_AT01                         (0x0035)
#define REG_ATPL250_AGC_AT01_32                      (REG_ATPL250_AGC_AT01 | MASK_32)
#define REG_ATPL250_AGC_AT23                         (0x0036)
#define REG_ATPL250_AGC_AT23_32                      (REG_ATPL250_AGC_AT23 | MASK_32)
#define REG_ATPL250_AGC_AT4_MAXTRIANG                (0x0037)
#define REG_ATPL250_AGC_AT4_MAXTRIANG_32             (REG_ATPL250_AGC_AT4_MAXTRIANG | MASK_32)
#define REG_ATPL250_AGC_AT4_MAXTRIANG_H16            (REG_ATPL250_AGC_AT4_MAXTRIANG | MASK_H16)
#define REG_ATPL250_AGC_AT4_MAXTRIANG_VL8            (REG_ATPL250_AGC_AT4_MAXTRIANG | MASK_VL8)
#define REG_ATPL250_AGC_MAXRAMP                      (0x0038)
#define REG_ATPL250_AGC_MAXRAMP_32                   (REG_ATPL250_AGC_MAXRAMP | MASK_32)
#define REG_ATPL250_AGC_MAXRAMP_H16                  (REG_ATPL250_AGC_MAXRAMP | MASK_H16)
#define REG_ATPL250_AGC_INFO                         (0x0039)
#define REG_ATPL250_AGC_INFO_32                      (REG_ATPL250_AGC_INFO | MASK_32)
#define REG_ATPL250_AGC_INFO_VH8                     (REG_ATPL250_AGC_INFO | MASK_VH8)
#define REG_ATPL250_AGC_INFO_H8                      (REG_ATPL250_AGC_INFO | MASK_H8)
#define REG_ATPL250_AGC_INFO_L8                      (REG_ATPL250_AGC_INFO | MASK_L8)
#define REG_ATPL250_AGC_INFO_VL8                     (REG_ATPL250_AGC_INFO | MASK_VL8)
#define REG_ATPL250_AGC_INFO_H16                     (REG_ATPL250_AGC_INFO | MASK_H16)
#define REG_ATPL250_AGC_INFO_L16                     (REG_ATPL250_AGC_INFO | MASK_L16)
#define REG_ATPL250_AGC_DELAYS_CONF                  (0x003A)
#define REG_ATPL250_AGC_DELAYS_CONF_32               (REG_ATPL250_AGC_DELAYS_CONF | MASK_32)
#define REG_ATPL250_AGC_DELAYS_CONF_L8               (REG_ATPL250_AGC_DELAYS_CONF | MASK_L8)
#define REG_ATPL250_AGC_DELAYS_CONF_VL8              (REG_ATPL250_AGC_DELAYS_CONF | MASK_VL8)
#define REG_ATPL250_AGC_FORCE_INT_CONF               (0x003B)
#define REG_ATPL250_AGC_FORCE_INT_CONF_32            (REG_ATPL250_AGC_FORCE_INT_CONF | MASK_32)
#define REG_ATPL250_AGC_FORCE_INT_CONF_H16           (REG_ATPL250_AGC_FORCE_INT_CONF | MASK_H16)
#define REG_ATPL250_AGC_FORCE_INT_CONF_L16           (REG_ATPL250_AGC_FORCE_INT_CONF | MASK_L16)
/* / Mapped address for SPI Pilots */
#define REG_ATPL250_COH_PILOTS01                     (0x003C)
#define REG_ATPL250_COH_PILOTS01_32                  (REG_ATPL250_COH_PILOTS01 | MASK_32)
#define REG_ATPL250_COH_PILOTS01_H16                 (REG_ATPL250_COH_PILOTS01 | MASK_H16)
#define REG_ATPL250_COH_PILOTS01_L16                 (REG_ATPL250_COH_PILOTS01 | MASK_L16)
#define REG_ATPL250_COH_PILOTS23                     (0x003D)
#define REG_ATPL250_COH_PILOTS23_32                  (REG_ATPL250_COH_PILOTS23 | MASK_32)
#define REG_ATPL250_COH_PILOTS23_H16                 (REG_ATPL250_COH_PILOTS23 | MASK_H16)
#define REG_ATPL250_COH_PILOTS23_L16                 (REG_ATPL250_COH_PILOTS23 | MASK_L16)
#define REG_ATPL250_COH_PILOTS45                     (0x003E)
#define REG_ATPL250_COH_PILOTS45_32                  (REG_ATPL250_COH_PILOTS45 | MASK_32)
#define REG_ATPL250_COH_PILOTS45_H16                 (REG_ATPL250_COH_PILOTS45 | MASK_H16)
#define REG_ATPL250_COH_PILOTS45_L16                 (REG_ATPL250_COH_PILOTS45 | MASK_L16)
#define REG_ATPL250_COH_PILOTS67                     (0x003F)
#define REG_ATPL250_COH_PILOTS67_32                  (REG_ATPL250_COH_PILOTS67 | MASK_32)
#define REG_ATPL250_COH_PILOTS67_H16                 (REG_ATPL250_COH_PILOTS67 | MASK_H16)
#define REG_ATPL250_COH_PILOTS67_L16                 (REG_ATPL250_COH_PILOTS67 | MASK_L16)
#define REG_ATPL250_COH_PILOTS89                     (0x0040)
#define REG_ATPL250_COH_PILOTS89_32                  (REG_ATPL250_COH_PILOTS89 | MASK_32)
#define REG_ATPL250_COH_PILOTS89_H16                 (REG_ATPL250_COH_PILOTS89 | MASK_H16)
#define REG_ATPL250_COH_PILOTS89_L16                 (REG_ATPL250_COH_PILOTS89 | MASK_L16)
#define REG_ATPL250_COH_PILOTSAB                     (0x0041)
#define REG_ATPL250_COH_PILOTSAB_32                  (REG_ATPL250_COH_PILOTSAB | MASK_32)
#define REG_ATPL250_COH_PILOTSAB_H16                 (REG_ATPL250_COH_PILOTSAB | MASK_H16)
#define REG_ATPL250_COH_PILOTSAB_L16                 (REG_ATPL250_COH_PILOTSAB | MASK_L16)
#define REG_ATPL250_COH_PILOTCNUM                    (0x0042)
#define REG_ATPL250_COH_PILOTCNUM_32                 (REG_ATPL250_COH_PILOTCNUM | MASK_32)
#define REG_ATPL250_COH_PILOTCNUM_L8                 (REG_ATPL250_COH_PILOTCNUM | MASK_L8)
#define REG_ATPL250_COH_PILOTCNUM_VL8                (REG_ATPL250_COH_PILOTCNUM | MASK_VL8)
#define REG_ATPL250_COH_PILOTCNUM_H16                (REG_ATPL250_COH_PILOTCNUM | MASK_H16)
/* / Mapped address for BER Peripheral */
#define REG_ATPL250_BER_PERIPH_CFG1                  (0x0043)
#define REG_ATPL250_BER_PERIPH_CFG1_32               (REG_ATPL250_BER_PERIPH_CFG1 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG1_L16              (REG_ATPL250_BER_PERIPH_CFG1 | MASK_L16)
#define REG_ATPL250_BER_PERIPH_CFG1_VH8              (REG_ATPL250_BER_PERIPH_CFG1 | MASK_VH8)
#define REG_ATPL250_BER_PERIPH_CFG2                  (0x0044)
#define REG_ATPL250_BER_PERIPH_CFG2_32               (REG_ATPL250_BER_PERIPH_CFG2 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG2_VH8              (REG_ATPL250_BER_PERIPH_CFG2 | MASK_VH8)
#define REG_ATPL250_BER_PERIPH_CFG2_H8               (REG_ATPL250_BER_PERIPH_CFG2 | MASK_H8)
#define REG_ATPL250_BER_PERIPH_CFG2_L8               (REG_ATPL250_BER_PERIPH_CFG2 | MASK_L8)
#define REG_ATPL250_BER_PERIPH_CFG2_VL8              (REG_ATPL250_BER_PERIPH_CFG2 | MASK_VL8)
#define REG_ATPL250_BER_PERIPH_CFG3                  (0x0045)
#define REG_ATPL250_BER_PERIPH_CFG3_32               (REG_ATPL250_BER_PERIPH_CFG3 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG3_VH8              (REG_ATPL250_BER_PERIPH_CFG3 | MASK_VH8)
#define REG_ATPL250_BER_PERIPH_CFG3_H8               (REG_ATPL250_BER_PERIPH_CFG3 | MASK_H8)
#define REG_ATPL250_BER_PERIPH_CFG3_L8               (REG_ATPL250_BER_PERIPH_CFG3 | MASK_L8)
#define REG_ATPL250_BER_PERIPH_CFG3_VL8              (REG_ATPL250_BER_PERIPH_CFG3 | MASK_VL8)
#define REG_ATPL250_BER_PERIPH_CFG3_L16              (REG_ATPL250_BER_PERIPH_CFG3 | MASK_L16)

#define REG_ATPL250_BER_PERIPH_CFG4                  (0x0046)
#define REG_ATPL250_BER_PERIPH_CFG4_32               (REG_ATPL250_BER_PERIPH_CFG4 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG5                  (0x0047)
#define REG_ATPL250_BER_PERIPH_CFG5_32               (REG_ATPL250_BER_PERIPH_CFG5 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG6                  (0x0048)
#define REG_ATPL250_BER_PERIPH_CFG6_32               (REG_ATPL250_BER_PERIPH_CFG6 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG7                  (0x0049)
#define REG_ATPL250_BER_PERIPH_CFG7_32               (REG_ATPL250_BER_PERIPH_CFG7 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG8                  (0x004A)
#define REG_ATPL250_BER_PERIPH_CFG8_32               (REG_ATPL250_BER_PERIPH_CFG8 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG9                  (0x004B)
#define REG_ATPL250_BER_PERIPH_CFG9_32               (REG_ATPL250_BER_PERIPH_CFG9 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG10                 (0x004C)
#define REG_ATPL250_BER_PERIPH_CFG10_32              (REG_ATPL250_BER_PERIPH_CFG10 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG10_VH8             (REG_ATPL250_BER_PERIPH_CFG10 | MASK_VH8)
#define REG_ATPL250_BER_PERIPH_CFG10_H8              (REG_ATPL250_BER_PERIPH_CFG10 | MASK_H8)
#define REG_ATPL250_BER_PERIPH_CFG10_L8              (REG_ATPL250_BER_PERIPH_CFG10 | MASK_L8)
#define REG_ATPL250_BER_PERIPH_CFG10_VL8             (REG_ATPL250_BER_PERIPH_CFG10 | MASK_VL8)
#define REG_ATPL250_BER_PERIPH_CFG11                 (0x004D)
#define REG_ATPL250_BER_PERIPH_CFG11_32              (REG_ATPL250_BER_PERIPH_CFG11 | MASK_32)
#define REG_ATPL250_BER_PERIPH_CFG11_H16             (REG_ATPL250_BER_PERIPH_CFG11 | MASK_H16)
#define REG_ATPL250_BER_PERIPH_CFG11_L16             (REG_ATPL250_BER_PERIPH_CFG11 | MASK_L16)
#define REG_ATPL250_BER_PERIPH_CFG11_VH8             (REG_ATPL250_BER_PERIPH_CFG11 | MASK_VH8)
#define REG_ATPL250_BER_PERIPH_CFG11_H8              (REG_ATPL250_BER_PERIPH_CFG11 | MASK_H8)
#define REG_ATPL250_BER_PERIPH_CFG11_L8              (REG_ATPL250_BER_PERIPH_CFG11 | MASK_L8)
#define REG_ATPL250_BER_PERIPH_CFG11_VL8             (REG_ATPL250_BER_PERIPH_CFG11 | MASK_VL8)
#define REG_ATPL250_BER_PERIPH_OUT1                  (0x004E)
#define REG_ATPL250_BER_PERIPH_OUT1_32               (REG_ATPL250_BER_PERIPH_OUT1 | MASK_32)
#define REG_ATPL250_BER_PERIPH_OUT2                  (0x004F)
#define REG_ATPL250_BER_PERIPH_OUT2_32               (REG_ATPL250_BER_PERIPH_OUT2 | MASK_32)
#define REG_ATPL250_BER_PERIPH_OUT3                  (0x0050)
#define REG_ATPL250_BER_PERIPH_OUT3_32               (REG_ATPL250_BER_PERIPH_OUT3 | MASK_32)
#define REG_ATPL250_BER_PERIPH_OUT4                  (0x0051)
#define REG_ATPL250_BER_PERIPH_OUT4_32               (REG_ATPL250_BER_PERIPH_OUT4 | MASK_32)
#define REG_ATPL250_BER_PERIPH_OUT5                  (0x0052)
#define REG_ATPL250_BER_PERIPH_OUT5_32               (REG_ATPL250_BER_PERIPH_OUT5 | MASK_32)
#define REG_ATPL250_BER_PERIPH_OUT6                  (0x0053)
#define REG_ATPL250_BER_PERIPH_OUT6_32               (REG_ATPL250_BER_PERIPH_OUT6 | MASK_32)
/* / Mapped address for Synchro threshold */
#define REG_ATPL250_SYNC_THRESHOLD                   (0x0056)
#define REG_ATPL250_SYNC_THRESHOLD_32                (REG_ATPL250_SYNC_THRESHOLD | MASK_32)
#define REG_ATPL250_SYNC_THRESHOLD_HALF              (0x0057)
#define REG_ATPL250_SYNC_THRESHOLD_HALF_32           (REG_ATPL250_SYNC_THRESHOLD_HALF | MASK_32)
/* / Mapped address for Synchro Config */
#define REG_ATPL250_SYNC_CFG                         (0x0058)
#define REG_ATPL250_SYNC_CFG_32                      (REG_ATPL250_SYNC_CFG | MASK_32)
#define REG_ATPL250_SYNC_CFG_VH8                     (REG_ATPL250_SYNC_CFG | MASK_VH8)
#define REG_ATPL250_SYNC_CFG_H8                      (REG_ATPL250_SYNC_CFG | MASK_H8)
#define REG_ATPL250_SYNC_CFG_L8                      (REG_ATPL250_SYNC_CFG | MASK_L8)
#define REG_ATPL250_SYNC_CFG_VL8                     (REG_ATPL250_SYNC_CFG | MASK_VL8)
#define REG_ATPL250_SYNC_MAX_HALF_DEL                (0x0059)
#define REG_ATPL250_SYNC_MAX_HALF_DEL_32             (REG_ATPL250_MAX_HALF_DEL | MASK_32)
/* / Mapped address for Global Reset */
#define REG_ATPL250_GLOBAL_SRST                      (0x005A)
#define REG_ATPL250_GLOBAL_SRST_32                   (REG_ATPL250_GLOBAL_SRST | MASK_32)
#define REG_ATPL250_GLOBAL_SRST_H16                  (REG_ATPL250_GLOBAL_SRST | MASK_H16)
#define REG_ATPL250_GLOBAL_SRST_L16                  (REG_ATPL250_GLOBAL_SRST | MASK_L16)
#define REG_ATPL250_GLOBAL_SRST_VH8                  (REG_ATPL250_GLOBAL_SRST | MASK_VH8)
#define REG_ATPL250_GLOBAL_SRST_H8                   (REG_ATPL250_GLOBAL_SRST | MASK_H8)
#define REG_ATPL250_GLOBAL_SRST_L8                   (REG_ATPL250_GLOBAL_SRST | MASK_L8)
#define REG_ATPL250_GLOBAL_SRST_VL8                  (REG_ATPL250_GLOBAL_SRST | MASK_VL8)
/* / Empty Register for internal checking */
#define REG_ATPL250_EMPTY                            (0x005C)
#define REG_ATPL250_EMPTY_32                         (REG_ATPL250_EMPTY | MASK_32)
/* / Mapped address for Preamble Config */
#define REG_ATPL250_PRE_CFG                          (0x005E)
#define REG_ATPL250_PRE_CFG_32                       (REG_ATPL250_PRE_CFG | MASK_32)
#define REG_ATPL250_PRE_CFG_H16                      (REG_ATPL250_PRE_CFG | MASK_H16)
#define REG_ATPL250_PRE_CFG_L16                      (REG_ATPL250_PRE_CFG | MASK_L16)
#define REG_ATPL250_PRE_CFG_VH8                      (REG_ATPL250_PRE_CFG | MASK_VH8)
#define REG_ATPL250_PRE_CFG_H8                       (REG_ATPL250_PRE_CFG | MASK_H8)
#define REG_ATPL250_PRE_CFG_L8                       (REG_ATPL250_PRE_CFG | MASK_L8)
#define REG_ATPL250_PRE_CFG_VL8                      (REG_ATPL250_PRE_CFG | MASK_VL8)
/* / Mapped address for Preamble Compatible Config */
#define REG_ATPL250_PRE_COMP_CFG                     (0x005F)
#define REG_ATPL250_PRE_COMP_CFG_32                  (REG_ATPL250_PRE_COMP_CFG | MASK_32)
#define REG_ATPL250_PRE_COMP_CFG_VH8                 (REG_ATPL250_PRE_COMP_CFG | MASK_VH8)
#define REG_ATPL250_PRE_COMP_CFG_H8                  (REG_ATPL250_PRE_COMP_CFG | MASK_H8)
#define REG_ATPL250_PRE_COMP_CFG_L8                  (REG_ATPL250_PRE_COMP_CFG | MASK_L8)
#define REG_ATPL250_PRE_COMP_CFG_VL8                 (REG_ATPL250_PRE_COMP_CFG | MASK_VL8)
/* / Mapped address for Preamble Compatible Repetition */
#define REG_ATPL250_PRE_REPH_COMP                    (0x0060)
#define REG_ATPL250_PRE_REPH_COMP_32                 (REG_ATPL250_PRE_REPH_COMP | MASK_32)
#define REG_ATPL250_PRE_REPH_COMP_VH8                (REG_ATPL250_PRE_REPH_COMP | MASK_VH8)
#define REG_ATPL250_PRE_REPH_COMP_H8                 (REG_ATPL250_PRE_REPH_COMP | MASK_H8)
#define REG_ATPL250_PRE_REPH_COMP_L8                 (REG_ATPL250_PRE_REPH_COMP | MASK_L8)
#define REG_ATPL250_PRE_REPH_COMP_VL8                (REG_ATPL250_PRE_REPH_COMP | MASK_VL8)
#define REG_ATPL250_PRE_REPL_COMP                    (0x0061)
#define REG_ATPL250_PRE_REPL_COMP_32                 (REG_ATPL250_PRE_REPL_COMP | MASK_32)
#define REG_ATPL250_PRE_REPL_COMP_VH8                (REG_ATPL250_PRE_REPL_COMP | MASK_VH8)
#define REG_ATPL250_PRE_REPL_COMP_H8                 (REG_ATPL250_PRE_REPL_COMP | MASK_H8)
#define REG_ATPL250_PRE_REPL_COMP_L8                 (REG_ATPL250_PRE_REPL_COMP | MASK_L8)
#define REG_ATPL250_PRE_REPL_COMP_VL8                (REG_ATPL250_PRE_REPL_COMP | MASK_VL8)
/* / Mapped address for Preamble Repetition */
#define REG_ATPL250_PRE_REPH                         (0x0062)
#define REG_ATPL250_PRE_REPH_32                      (REG_ATPL250_PRE_REPH | MASK_32)
#define REG_ATPL250_PRE_REPH_VH8                     (REG_ATPL250_PRE_REPH | MASK_VH8)
#define REG_ATPL250_PRE_REPH_H8                      (REG_ATPL250_PRE_REPH | MASK_H8)
#define REG_ATPL250_PRE_REPH_L8                      (REG_ATPL250_PRE_REPH | MASK_L8)
#define REG_ATPL250_PRE_REPH_VL8                     (REG_ATPL250_PRE_REPH | MASK_VL8)
#define REG_ATPL250_PRE_REPL                         (0x0063)
#define REG_ATPL250_PRE_REPL_32                      (REG_ATPL250_PRE_REPL | MASK_32)
#define REG_ATPL250_PRE_REPL_VH8                     (REG_ATPL250_PRE_REPL | MASK_VH8)
#define REG_ATPL250_PRE_REPL_H8                      (REG_ATPL250_PRE_REPL | MASK_H8)
#define REG_ATPL250_PRE_REPL_L8                      (REG_ATPL250_PRE_REPL | MASK_L8)
#define REG_ATPL250_PRE_REPL_VL8                     (REG_ATPL250_PRE_REPL | MASK_VL8)
/* / Mapped address for Load Data */
#define REG_ATPL250_LOAD_DATA7                       (0x0064)
#define REG_ATPL250_LOAD_DATA7_32                    (REG_ATPL250_LOAD_DATA7 | MASK_32)
#define REG_ATPL250_LOAD_DATA7_H16                   (REG_ATPL250_LOAD_DATA7 | MASK_H16)
#define REG_ATPL250_LOAD_DATA7_L16                   (REG_ATPL250_LOAD_DATA7 | MASK_L16)
#define REG_ATPL250_LOAD_DATA6                       (0x0065)
#define REG_ATPL250_LOAD_DATA6_32                    (REG_ATPL250_LOAD_DATA6 | MASK_32)
#define REG_ATPL250_LOAD_DATA6_H16                   (REG_ATPL250_LOAD_DATA6 | MASK_H16)
#define REG_ATPL250_LOAD_DATA6_L16                   (REG_ATPL250_LOAD_DATA6 | MASK_L16)
#define REG_ATPL250_LOAD_DATA5                       (0x0066)
#define REG_ATPL250_LOAD_DATA5_32                    (REG_ATPL250_LOAD_DATA5 | MASK_32)
#define REG_ATPL250_LOAD_DATA5_H16                   (REG_ATPL250_LOAD_DATA5 | MASK_H16)
#define REG_ATPL250_LOAD_DATA5_L16                   (REG_ATPL250_LOAD_DATA5 | MASK_L16)
#define REG_ATPL250_LOAD_DATA4                       (0x0067)
#define REG_ATPL250_LOAD_DATA4_32                    (REG_ATPL250_LOAD_DATA4 | MASK_32)
#define REG_ATPL250_LOAD_DATA4_H16                   (REG_ATPL250_LOAD_DATA4 | MASK_H16)
#define REG_ATPL250_LOAD_DATA4_L16                   (REG_ATPL250_LOAD_DATA4 | MASK_L16)
#define REG_ATPL250_LOAD_DATA3                       (0x0068)
#define REG_ATPL250_LOAD_DATA3_32                    (REG_ATPL250_LOAD_DATA3 | MASK_32)
#define REG_ATPL250_LOAD_DATA3_H16                   (REG_ATPL250_LOAD_DATA3 | MASK_H16)
#define REG_ATPL250_LOAD_DATA3_L16                   (REG_ATPL250_LOAD_DATA3 | MASK_L16)
#define REG_ATPL250_LOAD_DATA2                       (0x0069)
#define REG_ATPL250_LOAD_DATA2_32                    (REG_ATPL250_LOAD_DATA2 | MASK_32)
#define REG_ATPL250_LOAD_DATA2_H16                   (REG_ATPL250_LOAD_DATA2 | MASK_H16)
#define REG_ATPL250_LOAD_DATA2_L16                   (REG_ATPL250_LOAD_DATA2 | MASK_L16)
#define REG_ATPL250_LOAD_DATA1                       (0x006A)
#define REG_ATPL250_LOAD_DATA1_32                    (REG_ATPL250_LOAD_DATA1 | MASK_32)
#define REG_ATPL250_LOAD_DATA1_H16                   (REG_ATPL250_LOAD_DATA1 | MASK_H16)
#define REG_ATPL250_LOAD_DATA1_L16                   (REG_ATPL250_LOAD_DATA1 | MASK_L16)
#define REG_ATPL250_LOAD_DATA0                       (0x006B)
#define REG_ATPL250_LOAD_DATA0_32                    (REG_ATPL250_LOAD_DATA0 | MASK_32)
#define REG_ATPL250_LOAD_DATA0_H16                   (REG_ATPL250_LOAD_DATA0 | MASK_H16)
#define REG_ATPL250_LOAD_DATA0_L16                   (REG_ATPL250_LOAD_DATA0 | MASK_L16)
/* / Mapped address for Load Config */
#define REG_ATPL250_LOAD_CFG                         (0x006C)
#define REG_ATPL250_LOAD_CFG_32                      (REG_ATPL250_LOAD_CFG | MASK_32)
#define REG_ATPL250_LOAD_CFG_H8                      (REG_ATPL250_LOAD_CFG | MASK_H8)
#define REG_ATPL250_LOAD_CFG_L8                      (REG_ATPL250_LOAD_CFG | MASK_L8)
#define REG_ATPL250_LOAD_CFG_VL8                     (REG_ATPL250_LOAD_CFG | MASK_VL8)
/* / Mapped address for Tx Configuration */
#define REG_ATPL250_TX_CONF                          (0x006E)
#define REG_ATPL250_TX_CONF_32                       (REG_ATPL250_TX_CONF | MASK_32)
#define REG_ATPL250_TX_CONF_VH8                      (REG_ATPL250_TX_CONF | MASK_VH8)
#define REG_ATPL250_TX_CONF_H8                       (REG_ATPL250_TX_CONF | MASK_H8)
#define REG_ATPL250_TX_CONF_L8                       (REG_ATPL250_TX_CONF | MASK_L8)
#define REG_ATPL250_TX_CONF_VL8                      (REG_ATPL250_TX_CONF | MASK_VL8)
/* / Mapped address for Timer Reference */
#define REG_ATPL250_TX_TIMER_REF                     (0x006F)
#define REG_ATPL250_TX_TIMER_REF_32                  (REG_ATPL250_TX_TIMER_REF | MASK_32)
/* / Mapped address for Noise Time */
#define REG_ATPL250_TX_NOISE_TIME                    (0x0070)
#define REG_ATPL250_TX_NOISE_TIME_32                 (REG_ATPL250_TX_NOISE_TIME | MASK_32)
/* / Mapped address for TX Preescaler */
#define REG_ATPL250_TX_PREESCALER                    (0x0071)
#define REG_ATPL250_TX_PREESCALER_32                 (REG_ATPL250_TX_PREESCALER | MASK_32)
/* / Mapped address for Tx Time */
#define REG_ATPL250_TX_TIME1                         (0x0072)
#define REG_ATPL250_TX_TIME1_32                      (REG_ATPL250_TX_TIME1 | MASK_32)
#define REG_ATPL250_TX_TIME2                         (0x0073)
#define REG_ATPL250_TX_TIME2_32                      (REG_ATPL250_TX_TIME2 | MASK_32)
#define REG_ATPL250_TX_TIME3                         (0x0074)
#define REG_ATPL250_TX_TIME3_32                      (REG_ATPL250_TX_TIME3 | MASK_32)
#define REG_ATPL250_TX_TIME4                         (0x0075)
#define REG_ATPL250_TX_TIME4_32                      (REG_ATPL250_TX_TIME4 | MASK_32)
/* / Mapped address for Tx PLC Time */
#define REG_ATPL250_PLC_TIME_ON1                     (0x0076)
#define REG_ATPL250_PLC_TIME_ON1_32                  (REG_ATPL250_PLC_TIME_ON1 | MASK_32)
#define REG_ATPL250_PLC_TIME_ON2                     (0x0077)
#define REG_ATPL250_PLC_TIME_ON2_32                  (REG_ATPL250_PLC_TIME_ON2 | MASK_32)
#define REG_ATPL250_PLC_TIME_ON3                     (0x0078)
#define REG_ATPL250_PLC_TIME_ON3_32                  (REG_ATPL250_PLC_TIME_ON3 | MASK_32)
#define REG_ATPL250_PLC_TIME_ON4                     (0x0079)
#define REG_ATPL250_PLC_TIME_ON4_32                  (REG_ATPL250_PLC_TIME_ON4 | MASK_32)
/* / Mapped address for Time PLC Off */
#define REG_ATPL250_PLC_TIME_OFF1                    (0x007A)
#define REG_ATPL250_PLC_TIME_OFF1_32                 (REG_ATPL250_PLC_TIME_OFF1 | MASK_32)
#define REG_ATPL250_PLC_TIME_OFF2                    (0x007B)
#define REG_ATPL250_PLC_TIME_OFF2_32                 (REG_ATPL250_PLC_TIME_OFF2 | MASK_32)
#define REG_ATPL250_PLC_TIME_OFF3                    (0x007C)
#define REG_ATPL250_PLC_TIME_OFF3_32                 (REG_ATPL250_PLC_TIME_OFF3 | MASK_32)
#define REG_ATPL250_PLC_TIME_OFF4                    (0x007D)
#define REG_ATPL250_PLC_TIME_OFF4_32                 (REG_ATPL250_PLC_TIME_OFF4 | MASK_32)
/* / Mapped address for TxRx Config */
#define REG_ATPL250_TXRX_CFG                         (0x007E)
#define REG_ATPL250_TXRX_CFG_32                      (REG_ATPL250_TXRX_CFG | MASK_32)
#define REG_ATPL250_TXRX_CFG_VH8                     (REG_ATPL250_TXRX_CFG | MASK_VH8)
#define REG_ATPL250_TXRX_CFG_H8                      (REG_ATPL250_TXRX_CFG | MASK_H8)
#define REG_ATPL250_TXRX_CFG_L8                      (REG_ATPL250_TXRX_CFG | MASK_L8)
/* / Mapped address for TxRx Time Delay */
#define REG_ATPL250_TXRX_TIME_DELAY                  (0x007F)
#define REG_ATPL250_TXRX_TIME_DELAY_32               (REG_ATPL250_TXRX_TIME_DELAY | MASK_32)
/* / Mapped address for TxRx Time */
#define REG_ATPL250_TXRX_TIME_ON1                    (0x0080)
#define REG_ATPL250_TXRX_TIME_ON1_32                 (REG_ATPL250_TXRX_TIME_ON1 | MASK_32)
#define REG_ATPL250_TXRX_TIME_ON2                    (0x0081)
#define REG_ATPL250_TXRX_TIME_ON2_32                 (REG_ATPL250_TXRX_TIME_ON2 | MASK_32)
#define REG_ATPL250_TXRX_TIME_ON3                    (0x0082)
#define REG_ATPL250_TXRX_TIME_ON3_32                 (REG_ATPL250_TXRX_TIME_ON3 | MASK_32)
#define REG_ATPL250_TXRX_TIME_ON4                    (0x0083)
#define REG_ATPL250_TXRX_TIME_ON4_32                 (REG_ATPL250_TXRX_TIME_ON4 | MASK_32)
/* / Mapped address for TxRx Time Off */
#define REG_ATPL250_TXRX_TIME_OFF1                   (0x0084)
#define REG_ATPL250_TXRX_TIME_OFF1_32                (REG_ATPL250_TXRX_TIME_OFF1 | MASK_32)
#define REG_ATPL250_TXRX_TIME_OFF2                   (0x0085)
#define REG_ATPL250_TXRX_TIME_OFF2_32                (REG_ATPL250_TXRX_TIME_OFF2 | MASK_32)
#define REG_ATPL250_TXRX_TIME_OFF3                   (0x0086)
#define REG_ATPL250_TXRX_TIME_OFF3_32                (REG_ATPL250_TXRX_TIME_OFF3 | MASK_32)
#define REG_ATPL250_TXRX_TIME_OFF4                   (0x0087)
#define REG_ATPL250_TXRX_TIME_OFF4_32                (REG_ATPL250_TXRX_TIME_OFF4 | MASK_32)
/* / Mapped address for Test1 (turn on/off leds) */
#define REG_ATPL250_TEST1                            (0x0096)
#define REG_ATPL250_TEST1_32                         (REG_ATPL250_TEST1 | MASK_32)
#define REG_ATPL250_TEST1_VH8                        (REG_ATPL250_TEST1 | MASK_VH8)
/* / Mapped address for Rotator */
#define REG_ATPL250_ROTATOR_CONFIG0                  (0x0097)
#define REG_ATPL250_ROTATOR_CONFIG0_32               (REG_ATPL250_ROTATOR_CONFIG0 | MASK_32)
#define REG_ATPL250_ROTATOR_CONFIG1                  (0x0098)
#define REG_ATPL250_ROTATOR_CONFIG1_32               (REG_ATPL250_ROTATOR_CONFIG1 | MASK_32)
#define REG_ATPL250_ROTATOR_CONFIG2                  (0x0099)
#define REG_ATPL250_ROTATOR_CONFIG2_32               (REG_ATPL250_ROTATOR_CONFIG2 | MASK_32)
#define REG_ATPL250_ROTATOR_CONFIG3                  (0x009A)
#define REG_ATPL250_ROTATOR_CONFIG3_32               (REG_ATPL250_ROTATOR_CONFIG3 | MASK_32)
#define REG_ATPL250_ROTATOR_CONFIG3_L16              (REG_ATPL250_ROTATOR_CONFIG3 | MASK_L16)
#define REG_ATPL250_ROTATOR_CONFIG3_VH8              (REG_ATPL250_ROTATOR_CONFIG3 | MASK_VH8)
#define REG_ATPL250_ROTATOR_CONFIG3_H8               (REG_ATPL250_ROTATOR_CONFIG3 | MASK_H8)
#define REG_ATPL250_ROTATOR_CONFIG4                  (0x009B)
#define REG_ATPL250_ROTATOR_CONFIG4_32               (REG_ATPL250_ROTATOR_CONFIG4 | MASK_32)
/* / Mapped address for Clock Phase */
#define REG_ATPL250_CLKPHVAR_CONFIG                  (0x009D)
#define REG_ATPL250_CLKPHVAR_CONFIG_32               (REG_ATPL250_CLKPHVAR_CONFIG | MASK_32)
#define REG_ATPL250_CLKPHVAR_CONFIG_VH8              (REG_ATPL250_CLKPHVAR_CONFIG | MASK_VH8)
#define REG_ATPL250_CLKPHVAR_CONFIG_L16              (REG_ATPL250_CLKPHVAR_CONFIG | MASK_L16)
#define REG_ATPL250_CLKPHVAR_ACCUM                   (0x009E)
#define REG_ATPL250_CLKPHVAR_ACCUM_32                (REG_ATPL250_CLKPHVAR_ACCUM | MASK_32)
/* / Mapped address for Zero Cross Config */
#define REG_ATPL250_ZC_CONFIG                        (0x00A0)
#define REG_ATPL250_ZC_CONFIG_32                     (REG_ATPL250_ZC_CONFIG | MASK_32)
#define REG_ATPL250_ZC_CONFIG_VH8                    (REG_ATPL250_ZC_CONFIG | MASK_VH8)
#define REG_ATPL250_ZC_CONFIG_H8                     (REG_ATPL250_ZC_CONFIG | MASK_H8)
#define REG_ATPL250_ZC_CONFIG_L8                     (REG_ATPL250_ZC_CONFIG | MASK_L8)
#define REG_ATPL250_ZC_CONFIG_VL8                    (REG_ATPL250_ZC_CONFIG | MASK_VL8)
/* / Mapped address for Zero Cross Lag Lead */
#define REG_ATPL250_ZC_CLAGLEAD                      (0x00A1)
#define REG_ATPL250_ZC_CLAGLEAD_32                   (REG_ATPL250_ZC_CLAGLEAD | MASK_32)
#define REG_ATPL250_ZC_CLAGLEAD_H16                  (REG_ATPL250_ZC_CLAGLEAD | MASK_H16)
#define REG_ATPL250_ZC_CLAGLEAD_L16                  (REG_ATPL250_ZC_CLAGLEAD | MASK_L16)
/* / Mapped address for Zero Cross K frequency */
#define REG_ATPL250_ZC_KFREQ                         (0x00A2)
#define REG_ATPL250_ZC_KFREQ_32                      (REG_ATPL250_ZC_KFREQ | MASK_32)
/* / Mapped address for Zero Cross Offset */
#define REG_ATPL250_ZC_OFFSET                        (0x00A3)
#define REG_ATPL250_ZC_OFFSET_32                     (REG_ATPL250_ZC_OFFSET | MASK_32)
/* / Mapped address for Zero Cross Time */
#define REG_ATPL250_ZC_TIME                          (0x00A4)
#define REG_ATPL250_ZC_TIME_32                       (REG_ATPL250_ZC_TIME | MASK_32)
#define REG_ATPL250_ZC_TIME_PAST                     (0x00A5)
#define REG_ATPL250_ZC_TIME_PAST_32                  (REG_ATPL250_ZC_TIME_PAST | MASK_32)
/* / Mapped address for Peak Time */
#define REG_ATPL250_PEAK_TIME                        (0x00A6)
#define REG_ATPL250_PEAK_TIME_32                     (REG_ATPL250_PEAK_TIME | MASK_32)
#define REG_ATPL250_PEAK2_TIME                       (0x00A7)
#define REG_ATPL250_PEAK2_TIME_32                    (REG_ATPL250_PEAK2_TIME | MASK_32)
#define REG_ATPL250_PEAK13_TIME                      (0x00A8)
#define REG_ATPL250_PEAK13_TIME_32                   (REG_ATPL250_PEAK13_TIME | MASK_32)
#define REG_ATPL250_PEAK_ZC_TIME                     (0x00A9)
#define REG_ATPL250_PEAK_ZC_TIME_32                  (REG_ATPL250_PEAK_ZC_TIME | MASK_32)
/* / Mapped address for Reed-Solomon Lambda */
#define REG_ATPL250_RS_LAMBDA0123                    (0x00AA)
#define REG_ATPL250_RS_LAMBDA0123_32                 (REG_ATPL250_RS_LAMBDA0123 | MASK_32)
#define REG_ATPL250_RS_LAMBDA0123_VH8                (REG_ATPL250_RS_LAMBDA0123 | MASK_VH8)
#define REG_ATPL250_RS_LAMBDA0123_H8                 (REG_ATPL250_RS_LAMBDA0123 | MASK_H8)
#define REG_ATPL250_RS_LAMBDA0123_L8                 (REG_ATPL250_RS_LAMBDA0123 | MASK_L8)
#define REG_ATPL250_RS_LAMBDA0123_VL8                (REG_ATPL250_RS_LAMBDA0123 | MASK_VL8)
#define REG_ATPL250_RS_LAMBDA4567                    (0x00AB)
#define REG_ATPL250_RS_LAMBDA4567_32                 (REG_ATPL250_RS_LAMBDA4567 | MASK_32)
#define REG_ATPL250_RS_LAMBDA4567_VH8                (REG_ATPL250_RS_LAMBDA4567 | MASK_VH8)
#define REG_ATPL250_RS_LAMBDA4567_H8                 (REG_ATPL250_RS_LAMBDA4567 | MASK_H8)
#define REG_ATPL250_RS_LAMBDA4567_L8                 (REG_ATPL250_RS_LAMBDA4567 | MASK_L8)
#define REG_ATPL250_RS_LAMBDA4567_VL8                (REG_ATPL250_RS_LAMBDA4567 | MASK_VL8)
#define REG_ATPL250_RS_LAMBDA8                       (0x00AC)
#define REG_ATPL250_RS_LAMBDA8_32                    (REG_ATPL250_RS_LAMBDA8 | MASK_32)
#define REG_ATPL250_RS_LAMBDA8_VH8                   (REG_ATPL250_RS_LAMBDA8 | MASK_VH8)
/* / Mapped address for Reed-Solomon Config */
#define REG_ATPL250_RS_CFG                           (0x00AD)
#define REG_ATPL250_RS_CFG_32                        (REG_ATPL250_RS_CFG | MASK_32)
#define REG_ATPL250_RS_CFG_VH8                       (REG_ATPL250_RS_CFG | MASK_VH8)
#define REG_ATPL250_RS_CFG_H8                        (REG_ATPL250_RS_CFG | MASK_H8)
#define REG_ATPL250_RS_CFG_L8                        (REG_ATPL250_RS_CFG | MASK_L8)
#define REG_ATPL250_RS_CFG_VL8                       (REG_ATPL250_RS_CFG | MASK_VL8)
/* / Mapped address for Reed-Solomon Number of Errors */
#define REG_ATPL250_RS_NUM_ERR                       (0x00AE)
#define REG_ATPL250_RS_NUM_ERR_32                    (REG_ATPL250_RS_NUM_ERR | MASK_32)
#define REG_ATPL250_RS_NUM_ERR_L8                    (REG_ATPL250_RS_NUM_ERR | MASK_L8)
#define REG_ATPL250_RS_NUM_ERR_VL8                   (REG_ATPL250_RS_NUM_ERR | MASK_VL8)
/* / Mapped address for Reed-Solomon Calculated Lambda and Error */
#define REG_ATPL250_RS_ERR_LAMBDA01                  (0x00AF)
#define REG_ATPL250_RS_ERR_LAMBDA01_32               (REG_ATPL250_RS_ERR_LAMBDA01 | MASK_32)
#define REG_ATPL250_RS_ERR_LAMBDA01_VH8              (REG_ATPL250_RS_ERR_LAMBDA01 | MASK_VH8)
#define REG_ATPL250_RS_ERR_LAMBDA01_H8               (REG_ATPL250_RS_ERR_LAMBDA01 | MASK_H8)
#define REG_ATPL250_RS_ERR_LAMBDA01_L8               (REG_ATPL250_RS_ERR_LAMBDA01 | MASK_L8)
#define REG_ATPL250_RS_ERR_LAMBDA01_VL8              (REG_ATPL250_RS_ERR_LAMBDA01 | MASK_VL8)
#define REG_ATPL250_RS_ERR_LAMBDA23                  (0x00B0)
#define REG_ATPL250_RS_ERR_LAMBDA23_32               (REG_ATPL250_RS_ERR_LAMBDA23 | MASK_32)
#define REG_ATPL250_RS_ERR_LAMBDA23_VH8              (REG_ATPL250_RS_ERR_LAMBDA23 | MASK_VH8)
#define REG_ATPL250_RS_ERR_LAMBDA23_H8               (REG_ATPL250_RS_ERR_LAMBDA23 | MASK_H8)
#define REG_ATPL250_RS_ERR_LAMBDA23_L8               (REG_ATPL250_RS_ERR_LAMBDA23 | MASK_L8)
#define REG_ATPL250_RS_ERR_LAMBDA23_VL8              (REG_ATPL250_RS_ERR_LAMBDA23 | MASK_VL8)
#define REG_ATPL250_RS_ERR_LAMBDA45                  (0x00B1)
#define REG_ATPL250_RS_ERR_LAMBDA45_32               (REG_ATPL250_RS_ERR_LAMBDA45 | MASK_32)
#define REG_ATPL250_RS_ERR_LAMBDA45_VH8              (REG_ATPL250_RS_ERR_LAMBDA45 | MASK_VH8)
#define REG_ATPL250_RS_ERR_LAMBDA45_H8               (REG_ATPL250_RS_ERR_LAMBDA45 | MASK_H8)
#define REG_ATPL250_RS_ERR_LAMBDA45_L8               (REG_ATPL250_RS_ERR_LAMBDA45 | MASK_L8)
#define REG_ATPL250_RS_ERR_LAMBDA45_VL8              (REG_ATPL250_RS_ERR_LAMBDA45 | MASK_VL8)
#define REG_ATPL250_RS_ERR_LAMBDA67                  (0x00B2)
#define REG_ATPL250_RS_ERR_LAMBDA67_32               (REG_ATPL250_RS_ERR_LAMBDA67 | MASK_32)
#define REG_ATPL250_RS_ERR_LAMBDA67_VH8              (REG_ATPL250_RS_ERR_LAMBDA67 | MASK_VH8)
#define REG_ATPL250_RS_ERR_LAMBDA67_H8               (REG_ATPL250_RS_ERR_LAMBDA67 | MASK_H8)
#define REG_ATPL250_RS_ERR_LAMBDA67_L8               (REG_ATPL250_RS_ERR_LAMBDA67 | MASK_L8)
#define REG_ATPL250_RS_ERR_LAMBDA67_VL8              (REG_ATPL250_RS_ERR_LAMBDA67 | MASK_VL8)
/* / Mapped address for Interleaver */
#define REG_ATPL250_INTERLEAVER_CFG0                 (0x00B4)
#define REG_ATPL250_INTERLEAVER_CFG0_32              (REG_ATPL250_INTERLEAVER_CFG0 | MASK_32)
#define REG_ATPL250_INTERLEAVER_CFG1                 (0x00B5)
#define REG_ATPL250_INTERLEAVER_CFG1_32              (REG_ATPL250_INTERLEAVER_CFG1 | MASK_32)
#define REG_ATPL250_INTERLEAVER_CFG2                 (0x00B6)
#define REG_ATPL250_INTERLEAVER_CFG2_32              (REG_ATPL250_INTERLEAVER_CFG2 | MASK_32)
#define REG_ATPL250_INTERLEAVER_CFG2_H16             (REG_ATPL250_INTERLEAVER_CFG2 | MASK_H16)
#define REG_ATPL250_INTERLEAVER_CFG2_L16             (REG_ATPL250_INTERLEAVER_CFG2 | MASK_L16)
#define REG_ATPL250_INTERLEAVER_CFG2_VH8             (REG_ATPL250_INTERLEAVER_CFG2 | MASK_VH8)
#define REG_ATPL250_INTERLEAVER_CFG2_H8              (REG_ATPL250_INTERLEAVER_CFG2 | MASK_H8)
#define REG_ATPL250_INTERLEAVER_CFG2_L8              (REG_ATPL250_INTERLEAVER_CFG2 | MASK_L8)
#define REG_ATPL250_INTERLEAVER_CFG2_VL8             (REG_ATPL250_INTERLEAVER_CFG2 | MASK_VL8)
#define REG_ATPL250_INTERLEAVER_CFG3                 (0x00B7)
#define REG_ATPL250_INTERLEAVER_CFG3_32              (REG_ATPL250_INTERLEAVER_CFG3 | MASK_32)
#define REG_ATPL250_INTERLEAVER_CFG3_VH8             (REG_ATPL250_INTERLEAVER_CFG3 | MASK_VH8)
#define REG_ATPL250_INTERLEAVER_CFG3_H8              (REG_ATPL250_INTERLEAVER_CFG3 | MASK_H8)
#define REG_ATPL250_INTERLEAVER_CFG3_L8              (REG_ATPL250_INTERLEAVER_CFG3 | MASK_L8)
#define REG_ATPL250_INTERLEAVER_CFG3_VL8             (REG_ATPL250_INTERLEAVER_CFG3 | MASK_VL8)
#define REG_ATPL250_INTERLEAVER_CTL                  (0x00B8)
#define REG_ATPL250_INTERLEAVER_CTL_32               (REG_ATPL250_INTERLEAVER_CTL | MASK_32)
#define REG_ATPL250_INTERLEAVER_CTL_H16              (REG_ATPL250_INTERLEAVER_CTL | MASK_H16)
#define REG_ATPL250_INTERLEAVER_CTL_L16              (REG_ATPL250_INTERLEAVER_CTL | MASK_L16)
#define REG_ATPL250_INTERLEAVER_CTL_H8               (REG_ATPL250_INTERLEAVER_CTL | MASK_H8)
#define REG_ATPL250_INTERLEAVER_CTL_L8               (REG_ATPL250_INTERLEAVER_CTL | MASK_L8)
#define REG_ATPL250_INTERLEAVER_CTL_VL8              (REG_ATPL250_INTERLEAVER_CTL | MASK_VL8)
#define REG_ATPL250_INTERLEAVER_INFO1                (0x00B9)
#define REG_ATPL250_INTERLEAVER_INFO1_32             (REG_ATPL250_INTERLEAVER_INFO1 | MASK_32)
#define REG_ATPL250_INTERLEAVER_INFO1_H16            (REG_ATPL250_INTERLEAVER_INFO1 | MASK_H16)
#define REG_ATPL250_INTERLEAVER_INFO1_L8             (REG_ATPL250_INTERLEAVER_INFO1 | MASK_L8)
#define REG_ATPL250_INTERLEAVER_INFO1_VL8            (REG_ATPL250_INTERLEAVER_INFO1 | MASK_VL8)
#define REG_ATPL250_INTERLEAVER_INFO2                (0x00BA)
#define REG_ATPL250_INTERLEAVER_INFO2_32             (REG_ATPL250_INTERLEAVER_INFO2 | MASK_32)
#define REG_ATPL250_INTERLEAVER_INFO2_H16            (REG_ATPL250_INTERLEAVER_INFO2 | MASK_H16)
#define REG_ATPL250_INTERLEAVER_INFO2_H8             (REG_ATPL250_INTERLEAVER_INFO2 | MASK_H8)
#define REG_ATPL250_INTERLEAVER_INFO2_L16            (REG_ATPL250_INTERLEAVER_INFO2 | MASK_L16)
#define REG_ATPL250_INTERLEAVER_INFO2_L8             (REG_ATPL250_INTERLEAVER_INFO2 | MASK_L8)
#define REG_ATPL250_INTERLEAVER_INFO2_VL8            (REG_ATPL250_INTERLEAVER_INFO2 | MASK_VL8)
#define REG_ATPL250_INTERLEAVER_SPI                  (0x00BB)
#define REG_ATPL250_INTERLEAVER_SPI_32               (REG_ATPL250_INTERLEAVER_SPI | MASK_32)
#define REG_ATPL250_INTERLEAVER_SPI_H16              (REG_ATPL250_INTERLEAVER_SPI | MASK_H16)
#define REG_ATPL250_INTERLEAVER_SPI_VL8              (REG_ATPL250_INTERLEAVER_SPI | MASK_VL8)
#define REG_ATPL250_INTERLEAVER_BPSCR                (0x00BC)
#define REG_ATPL250_INTERLEAVER_BPSCR_32             (REG_ATPL250_INTERLEAVER_BPSCR | MASK_32)
#define REG_ATPL250_INTERLEAVER_BPSCR_H16            (REG_ATPL250_INTERLEAVER_BPSCR | MASK_H16)
#define REG_ATPL250_INTERLEAVER_BPSCR_L16            (REG_ATPL250_INTERLEAVER_BPSCR | MASK_L16)
#define REG_ATPL250_INTERLEAVER_BPSCR_H8             (REG_ATPL250_INTERLEAVER_BPSCR | MASK_H8)
/* / Mapped address for Impulsive Detector */
#define REG_ATPL250_INTRLV_PRIME0                    (0x00BD)
#define REG_ATPL250_INTRLV_PRIME0_32                 (REG_ATPL250_INTRLV_PRIME0 | MASK_32)
#define REG_ATPL250_INTRLV_PRIME1                    (0x00BE)
#define REG_ATPL250_INTRLV_PRIME1_32                 (REG_ATPL250_INTRLV_PRIME1 | MASK_32)
#define REG_ATPL250_INTRLV_INT                       (0x00BF)
#define REG_ATPL250_INTRLV_INT_32                    (REG_ATPL250_INTRLV_INT | MASK_32)
/* / Mapped address for impulsive detector */
#define REG_ATPL250_RAW_DET_CFG                      (0x00C1)
#define REG_ATPL250_RAW_DET_CFG_32                   (REG_ATPL250_RAW_DET_CFG | MASK_32)
#define REG_ATPL250_RAW_DET_TIMERS                   (0x00C2)
#define REG_ATPL250_RAW_DET_TIMERS_32                (REG_ATPL250_RAW_DET_TIMERS | MASK_32)
#define REG_ATPL250_RAW_DET_LIMITS                   (0x00C3)
#define REG_ATPL250_RAW_DET_LIMITS_32                (REG_ATPL250_RAW_DET_LIMITS | MASK_32)
#define REG_ATPL250_RAW_DET_INFO                     (0x00C4)
#define REG_ATPL250_RAW_DET_INFO_32                  (REG_ATPL250_RAW_DET_INFO | MASK_32)
/* / Mapped address for Raw Data */
#define REG_ATPL250_RAW_DATA                         (0x00C5)
#define REG_ATPL250_RAW_DATA_32                      (REG_ATPL250_RAW_DATA | MASK_32)
#define REG_ATPL250_RAW_DATA_H16                     (REG_ATPL250_RAW_DATA | MASK_H16)
#define REG_ATPL250_RAW_DATA_VL8                     (REG_ATPL250_RAW_DATA | MASK_VL8)
/* / Mapped address for Interpolator */
#define REG_ATPL250_INT_PHASE_STEP1                  (0x00C9)
#define REG_ATPL250_INT_PHASE_STEP1_32               (REG_ATPL250_INT_PHASE_STEP1 | MASK_32)
#define REG_ATPL250_INT_PHASE_STEP2                  (0x00CA)
#define REG_ATPL250_INT_PHASE_STEP2_32               (REG_ATPL250_INT_PHASE_STEP2 | MASK_32)
#define REG_ATPL250_INT_BAUDRATE_DELAY               (0x00CB)
#define REG_ATPL250_INT_BAUDRATE_DELAY_32            (REG_ATPL250_INT_BAUDRATE_DELAY | MASK_32)
#define REG_ATPL250_INT_BAUDRATE_DELAY_H16           (REG_ATPL250_INT_BAUDRATE_DELAY | MASK_H16)
#define REG_ATPL250_INT_BAUDRATE_DELAY_L16           (REG_ATPL250_INT_BAUDRATE_DELAY | MASK_L16)
#define REG_ATPL250_INT_CONF                         (0x00CC)
#define REG_ATPL250_INT_CONF_32                      (REG_ATPL250_INT_CONF | MASK_32)
#define REG_ATPL250_INT_CONF_H16                     (REG_ATPL250_INT_CONF | MASK_H16)
#define REG_ATPL250_INT_CONF_L16                     (REG_ATPL250_INT_CONF | MASK_L16)
#define REG_ATPL250_INT_CONF_H8                      (REG_ATPL250_INT_CONF | MASK_H8)
#define REG_ATPL250_INT_CONF_VH8                     (REG_ATPL250_INT_CONF | MASK_VH8)
#define REG_ATPL250_INT_CONF_L8                      (REG_ATPL250_INT_CONF | MASK_L8)
#define REG_ATPL250_INT_CONF_VL8                     (REG_ATPL250_INT_CONF | MASK_VL8)
#define REG_ATPL250_INT_CYCLES                       (0x00CD)
#define REG_ATPL250_INT_CYCLES_32                    (REG_ATPL250_INT_CYCLES | MASK_32)
#define REG_ATPL250_INT_CYCLES_L16                   (REG_ATPL250_INT_CYCLES | MASK_L16)
#define REG_ATPL250_INT_CYCLES_L8                    (REG_ATPL250_INT_CYCLES | MASK_L8)
#define REG_ATPL250_INT_CYCLES_VL8                   (REG_ATPL250_INT_CYCLES | MASK_VL8)
#define REG_ATPL250_INT_RRC_0                        (0x00CE)
#define REG_ATPL250_INT_RRC_0_32                     (REG_ATPL250_INT_RRC_0 | MASK_32)
#define REG_ATPL250_INT_RRC_0_H16                    (REG_ATPL250_INT_RRC_0 | MASK_H16)
#define REG_ATPL250_INT_RRC_0_L16                    (REG_ATPL250_INT_RRC_0 | MASK_L16)
#define REG_ATPL250_INT_RRC_0_VH8                    (REG_ATPL250_INT_RRC_0 | MASK_VH8)
#define REG_ATPL250_INT_RRC_0_H8                     (REG_ATPL250_INT_RRC_0 | MASK_H8)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_0             (0x00CF)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_0_32          (REG_ATPL250_INT_RRC_BAUD_SHIFT_0 | MASK_32)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_0_L16         (REG_ATPL250_INT_RRC_BAUD_SHIFT_0 | MASK_L16)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_0_H8          (REG_ATPL250_INT_RRC_BAUD_SHIFT_0 | MASK_H8)
#define REG_ATPL250_INT_RRC_1                        (0x00D0)
#define REG_ATPL250_INT_RRC_1_32                     (REG_ATPL250_INT_RRC_1 | MASK_32)
#define REG_ATPL250_INT_RRC_1_H16                    (REG_ATPL250_INT_RRC_1 | MASK_H16)
#define REG_ATPL250_INT_RRC_1_L16                    (REG_ATPL250_INT_RRC_1 | MASK_L16)
#define REG_ATPL250_INT_RRC_1_VH8                    (REG_ATPL250_INT_RRC_1 | MASK_VH8)
#define REG_ATPL250_INT_RRC_1_H8                     (REG_ATPL250_INT_RRC_1 | MASK_H8)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_1             (0x00D1)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_1_32          (REG_ATPL250_INT_RRC_BAUD_SHIFT_1 | MASK_32)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_1_L16         (REG_ATPL250_INT_RRC_BAUD_SHIFT_1 | MASK_L16)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_1_H8          (REG_ATPL250_INT_RRC_BAUD_SHIFT_1 | MASK_H8)
#define REG_ATPL250_INT_RRC_2                        (0x00D2)
#define REG_ATPL250_INT_RRC_2_32                     (REG_ATPL250_INT_RRC_2 | MASK_32)
#define REG_ATPL250_INT_RRC_2_H16                    (REG_ATPL250_INT_RRC_2 | MASK_H16)
#define REG_ATPL250_INT_RRC_2_L16                    (REG_ATPL250_INT_RRC_2 | MASK_L16)
#define REG_ATPL250_INT_RRC_2_VH8                    (REG_ATPL250_INT_RRC_2 | MASK_VH8)
#define REG_ATPL250_INT_RRC_2_H8                     (REG_ATPL250_INT_RRC_2 | MASK_H8)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_2             (0x00D3)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_2_32          (REG_ATPL250_INT_RRC_BAUD_SHIFT_2 | MASK_32)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_2_L16         (REG_ATPL250_INT_RRC_BAUD_SHIFT_2 | MASK_L16)
#define REG_ATPL250_INT_RRC_BAUD_SHIFT_2_H8          (REG_ATPL250_INT_RRC_BAUD_SHIFT_2 | MASK_H8)
/* / Mapped address for Decimator */
#define REG_ATPL250_DEC_PHASE_STEP1                  (0x00D4)
#define REG_ATPL250_DEC_PHASE_STEP1_32               (REG_ATPL250_DEC_PHASE_STEP1 | MASK_32)
#define REG_ATPL250_DEC_PHASE_STEP2                  (0x00D5)
#define REG_ATPL250_DEC_PHASE_STEP2_32               (REG_ATPL250_DEC_PHASE_STEP2 | MASK_32)
#define REG_ATPL250_DEC_CONF                         (0x00D6)
#define REG_ATPL250_DEC_CONF_32                      (REG_ATPL250_DEC_CONF | MASK_32)
#define REG_ATPL250_DEC_CONF_H16                     (REG_ATPL250_DEC_CONF | MASK_H16)
#define REG_ATPL250_DEC_CONF_L16                     (REG_ATPL250_DEC_CONF | MASK_L16)
#define REG_ATPL250_DEC_CONF_VH8                     (REG_ATPL250_DEC_CONF | MASK_VH8)
#define REG_ATPL250_DEC_CONF_H8                      (REG_ATPL250_DEC_CONF | MASK_H8)
#define REG_ATPL250_DEC_CONF_L8                      (REG_ATPL250_DEC_CONF | MASK_L8)
#define REG_ATPL250_DEC_CONF_VL8                     (REG_ATPL250_DEC_CONF | MASK_VL8)
#define REG_ATPL250_DEC_FACTOR                       (0x00D7)
#define REG_ATPL250_DEC_FACTOR_32                    (REG_ATPL250_DEC_FACTOR | MASK_32)
#define REG_ATPL250_DEC_FACTOR_L16                   (REG_ATPL250_DEC_FACTOR | MASK_L16)
#define REG_ATPL250_DEC_FACTOR_H8                    (REG_ATPL250_DEC_FACTOR | MASK_H8)
#define REG_ATPL250_DEC_RRC_0                        (0x00D8)
#define REG_ATPL250_DEC_RRC_0_32                     (REG_ATPL250_DEC_RRC_0 | MASK_32)
#define REG_ATPL250_DEC_RRC_0_H16                    (REG_ATPL250_DEC_RRC_0 | MASK_H16)
#define REG_ATPL250_DEC_RRC_0_L16                    (REG_ATPL250_DEC_RRC_0 | MASK_L16)
#define REG_ATPL250_DEC_RRC_0_VH8                    (REG_ATPL250_DEC_RRC_0 | MASK_VH8)
#define REG_ATPL250_DEC_RRC_0_H8                     (REG_ATPL250_DEC_RRC_0 | MASK_H8)
#define REG_ATPL250_DEC_RRC_1                        (0x00D9)
#define REG_ATPL250_DEC_RRC_1_32                     (REG_ATPL250_DEC_RRC_1 | MASK_32)
#define REG_ATPL250_DEC_RRC_1_H16                    (REG_ATPL250_DEC_RRC_1 | MASK_H16)
#define REG_ATPL250_DEC_RRC_1_L16                    (REG_ATPL250_DEC_RRC_1 | MASK_L16)
#define REG_ATPL250_DEC_RRC_1_VH8                    (REG_ATPL250_DEC_RRC_1 | MASK_VH8)
#define REG_ATPL250_DEC_RRC_1_H8                     (REG_ATPL250_DEC_RRC_1 | MASK_H8)
#define REG_ATPL250_DEC_RRC_2                        (0x00DA)
#define REG_ATPL250_DEC_RRC_2_32                     (REG_ATPL250_DEC_RRC_2 | MASK_32)
#define REG_ATPL250_DEC_RRC_2_H16                    (REG_ATPL250_DEC_RRC_2 | MASK_H16)
#define REG_ATPL250_DEC_RRC_2_L16                    (REG_ATPL250_DEC_RRC_2 | MASK_L16)
#define REG_ATPL250_DEC_RRC_2_VH8                    (REG_ATPL250_DEC_RRC_2 | MASK_VH8)
#define REG_ATPL250_DEC_RRC_2_H8                     (REG_ATPL250_DEC_RRC_2 | MASK_H8)
#define REG_ATPL250_DEC_RRC_SHIFT                    (0x00DB)
#define REG_ATPL250_DEC_RRC_SHIFT_32                 (REG_ATPL250_DEC_RRC_SHIFT | MASK_32)
#define REG_ATPL250_DEC_RRC_SHIFT_L16                (REG_ATPL250_DEC_RRC_SHIFT | MASK_L16)
#define REG_ATPL250_DEC_RRC_SHIFT_L8                 (REG_ATPL250_DEC_RRC_SHIFT | MASK_L8)
#define REG_ATPL250_DEC_RRC_SHIFT_VL8                (REG_ATPL250_DEC_RRC_SHIFT | MASK_VL8)
/* / Mapped address for EVM */
#define REG_ATPL250_EVM_BPSK_QPSK                    (0x00DC)
#define REG_ATPL250_EVM_BPSK_QPSK_32                 (REG_ATPL250_EVM_BPSK_QPSK | MASK_32)
#define REG_ATPL250_EVM_BPSK_QPSK_H16                (REG_ATPL250_EVM_BPSK_QPSK | MASK_H16)
#define REG_ATPL250_EVM_BPSK_QPSK_L16                (REG_ATPL250_EVM_BPSK_QPSK | MASK_L16)
#define REG_ATPL250_EVM_8PSK                         (0x00DD)
#define REG_ATPL250_EVM_8PSK_32                      (REG_ATPL250_EVM_8PSK | MASK_32)
#define REG_ATPL250_EVM_8PSK_L16                     (REG_ATPL250_EVM_8PSK | MASK_L16)
/* / Mapped address for RSSI */
#define REG_ATPL250_RSSI                             (0x00DE)
#define REG_ATPL250_RSSI_32                          (REG_ATPL250_RSSI | MASK_32)
#define REG_ATPL250_RSSI_H16                         (REG_ATPL250_RSSI | MASK_H16)
#define REG_ATPL250_RSSI_L16                         (REG_ATPL250_RSSI | MASK_L16)
/* / Mapped address for PEAK_AD */
#define REG_ATPL250_PEAK_AD                          (0x00DF)
#define REG_ATPL250_PEAK_AD_32                       (REG_ATPL250_PEAK_AD | MASK_32)
#define REG_ATPL250_PEAK_AD_H16                      (REG_ATPL250_PEAK_AD | MASK_H16)
#define REG_ATPL250_PEAK_AD_L16                      (REG_ATPL250_PEAK_AD | MASK_L16)
/* / Mapped address for Emisor */
#define REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY        (0x00E1)
#define REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_32     (REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY | MASK_32)
#define REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_H16    (REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY | MASK_H16)
#define REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY_VL8    (REG_ATPL250_EMIT_GAIN_LIMIT_INI_DELAY | MASK_VL8)
#define REG_ATPL250_EMIT_GAIN                        (0x00E2)
#define REG_ATPL250_EMIT_GAIN_32                     (REG_ATPL250_EMIT_GAIN | MASK_32)
#define REG_ATPL250_EMIT_GAIN_VL8                    (REG_ATPL250_EMIT_GAIN | MASK_VL8)
#define REG_ATPL250_EMIT_PREDIST                     (0x00E3)
#define REG_ATPL250_EMIT_PREDIST_32                  (REG_ATPL250_EMIT_PREDIST | MASK_32)
#define REG_ATPL250_EMIT_FREC_DELAY                  (0x00E4)
#define REG_ATPL250_EMIT_FREC_DELAY_32               (REG_ATPL250_EMIT_FREC_DELAY | MASK_32)
#define REG_ATPL250_EMIT_FREC_DELAY_H16              (REG_ATPL250_EMIT_FREC_DELAY | MASK_H16)
#define REG_ATPL250_EMIT_FREC_DELAY_VL8              (REG_ATPL250_EMIT_FREC_DELAY | MASK_VL8)
#define REG_ATPL250_EMIT_FREC_DELAY_H8               (REG_ATPL250_EMIT_FREC_DELAY | MASK_H8)
#define REG_ATPL250_EMIT_FREC_DELAY_L8               (REG_ATPL250_EMIT_FREC_DELAY | MASK_L8)
#define REG_ATPL250_EMIT_BIT_FLIPPING                (0x00E5)
#define REG_ATPL250_EMIT_BIT_FLIPPING_32             (REG_ATPL250_EMIT_BIT_FLIPPING | MASK_32)
#define REG_ATPL250_EMIT_BIT_FLIPPING_H8             (REG_ATPL250_EMIT_BIT_FLIPPING | MASK_H8) /* PRFC */
#define REG_ATPL250_EMIT_BIT_FLIPPING_L8             (REG_ATPL250_EMIT_BIT_FLIPPING | MASK_L8)
#define REG_ATPL250_EMIT_BIT_FLIPPING_VL8            (REG_ATPL250_EMIT_BIT_FLIPPING | MASK_VL8)
#define REG_ATPL250_EMIT_CONFIG                      (0x00E6)
#define REG_ATPL250_EMIT_CONFIG_32                   (REG_ATPL250_EMIT_CONFIG | MASK_32)
#define REG_ATPL250_EMIT_CONFIG_H16                  (REG_ATPL250_EMIT_CONFIG | MASK_H16)
#define REG_ATPL250_EMIT_CONFIG_L16                  (REG_ATPL250_EMIT_CONFIG | MASK_L16)
#define REG_ATPL250_EMIT_CONFIG_VH8                  (REG_ATPL250_EMIT_CONFIG | MASK_VH8)
#define REG_ATPL250_EMIT_CONFIG_H8                   (REG_ATPL250_EMIT_CONFIG | MASK_H8)
#define REG_ATPL250_EMIT_CONFIG_L8                   (REG_ATPL250_EMIT_CONFIG | MASK_L8)
#define REG_ATPL250_EMIT_CONFIG_VL8                  (REG_ATPL250_EMIT_CONFIG | MASK_VL8)
#define REG_ATPL250_EMIT_SOFT_TIME_X                 (0x00E7)
#define REG_ATPL250_EMIT_SOFT_TIME_X_32              (REG_ATPL250_EMIT_SOFT_TIME_X | MASK_32)
#define REG_ATPL250_EMIT_SOFT_TIME_Y                 (0x00E8)
#define REG_ATPL250_EMIT_SOFT_TIME_Y_32              (REG_ATPL250_EMIT_SOFT_TIME_Y | MASK_32)
#define REG_ATPL250_EMIT_NP_DELAY                    (0x00E9)
#define REG_ATPL250_EMIT_NP_DELAY_32                 (REG_ATPL250_EMIT_NP_DELAY | MASK_32)
#define REG_ATPL250_EMIT_NP_DELAY_H16                (REG_ATPL250_EMIT_NP_DELAY | MASK_H16)
#define REG_ATPL250_EMIT_NP_DELAY_L16                (REG_ATPL250_EMIT_NP_DELAY | MASK_L16)
#define REG_ATPL250_EMIT_ON_ACTIVE                   (0x00EA)
#define REG_ATPL250_EMIT_ON_ACTIVE_32                (REG_ATPL250_EMIT_ON_ACTIVE | MASK_32)
#define REG_ATPL250_EMIT_OFF_ACTIVE                  (0x00EB)
#define REG_ATPL250_EMIT_OFF_ACTIVE_32               (REG_ATPL250_EMIT_OFF_ACTIVE | MASK_32)
/* / Mapped address for Resampling */
#define REG_ATPL250_RESAMP16BITS                     (0x00EC)
#define REG_ATPL250_RESAMP16BITS_32                  (REG_ATPL250_RESAMP16BITS | MASK_32)
#define REG_ATPL250_RESAMP24BITS_1                   (0x00ED)
#define REG_ATPL250_RESAMP24BITS_1_32                (REG_ATPL250_RESAMP24BITS_1 | MASK_32)
#define REG_ATPL250_RESAMP24BITS_2                   (0x00EE)
#define REG_ATPL250_RESAMP24BITS_2_32                (REG_ATPL250_RESAMP24BITS_2 | MASK_32)
#define REG_ATPL250_RESAMP24BITS_2_VH8               (REG_ATPL250_RESAMP24BITS_2 | MASK_VH8)
/* / Mapped address for RMS CALC */
#define REG_ATPL250_RMS_CALC_CFG1                    (0x00F0)
#define REG_ATPL250_RMS_CALC_CFG1_32                 (REG_ATPL250_RMS_CALC_CFG1 | MASK_32)
#define REG_ATPL250_RMS_CALC_CFG1_H16                (REG_ATPL250_RMS_CALC_CFG1 | MASK_H16)
#define REG_ATPL250_RMS_CALC_CFG1_L16                (REG_ATPL250_RMS_CALC_CFG1 | MASK_L16)
#define REG_ATPL250_RMS_CALC                         (0x00F1)
#define REG_ATPL250_RMS_CALC_32                      (REG_ATPL250_RMS_CALC | MASK_32)
/* / Mapped address for CD */
#define REG_ATPL250_CD_CFG                           (0x00F3)
#define REG_ATPL250_CD_CFG_32                        (REG_ATPL250_CD_CFG | MASK_32)
#define REG_ATPL250_CD_CFG_H16                       (REG_ATPL250_CD_CFG | MASK_H16)
#define REG_ATPL250_CD_CFG_L16                       (REG_ATPL250_CD_CFG | MASK_L16)
#define REG_ATPL250_CD_CFG_VH8                       (REG_ATPL250_CD_CFG | MASK_VH8)
#define REG_ATPL250_CD_CFG_H8                        (REG_ATPL250_CD_CFG | MASK_H8)
#define REG_ATPL250_CD_CFG_L8                        (REG_ATPL250_CD_CFG | MASK_L8)
#define REG_ATPL250_CD_CFG_VL8                       (REG_ATPL250_CD_CFG | MASK_VL8)
#define REG_ATPL250_CD_MOD                           (0x00F4)
#define REG_ATPL250_CD_MOD_32                        (REG_ATPL250_CD_MOD | MASK_32)
#define REG_ATPL250_CD_MOD_H16                       (REG_ATPL250_CD_MOD | MASK_H16)
#define REG_ATPL250_CD_MOD_L16                       (REG_ATPL250_CD_MOD | MASK_L16)
#define REG_ATPL250_CD_MOD_VH8                       (REG_ATPL250_CD_MOD | MASK_VH8)
#define REG_ATPL250_CD_MOD_H8                        (REG_ATPL250_CD_MOD | MASK_H8)
#define REG_ATPL250_CD_MOD_L8                        (REG_ATPL250_CD_MOD | MASK_L8)
#define REG_ATPL250_CD_MOD_VL8                       (REG_ATPL250_CD_MOD | MASK_VL8)
#define REG_ATPL250_CD_RAMP                          (0x00F5)
#define REG_ATPL250_CD_RAMP_32                       (REG_ATPL250_CD_RAMP | MASK_32)
#define REG_ATPL250_CD_RAMP_H16                      (REG_ATPL250_CD_RAMP | MASK_H16)
#define REG_ATPL250_CD_RAMP_L16                      (REG_ATPL250_CD_RAMP | MASK_L16)
#define REG_ATPL250_CD_RAMP_VH8                      (REG_ATPL250_CD_RAMP | MASK_VH8)
#define REG_ATPL250_CD_RAMP_H8                       (REG_ATPL250_CD_RAMP | MASK_H8)
#define REG_ATPL250_CD_RAMP_L8                       (REG_ATPL250_CD_RAMP | MASK_L8)
#define REG_ATPL250_CD_RAMP_VL8                      (REG_ATPL250_CD_RAMP | MASK_VL8)
#define REG_ATPL250_CD_LENGTH                        (0x00F6)
#define REG_ATPL250_CD_LENGTH_32                     (REG_ATPL250_CD_LENGTH | MASK_32)
#define REG_ATPL250_CD_UPDATE                        (0x00F7)
#define REG_ATPL250_CD_UPDATE_32                     (REG_ATPL250_CD_UPDATE | MASK_32)
/* / Mapped address for RX Block */
#define REG_ATPL250_RX_BLOCK                         (0x00F9)
#define REG_ATPL250_RX_BLOCK_32                      (REG_ATPL250_RX_BLOCK | MASK_32)
/* / Mapped address for Interruptions */
#define REG_ATPL250_INT_CFG                          (0x00FD)
#define REG_ATPL250_INT_CFG_32                       (REG_ATPL250_INT_CFG | MASK_32)
#define REG_ATPL250_INT_CFG_VL8                      (REG_ATPL250_INT_CFG | MASK_VL8)
#define REG_ATPL250_INT_MASK                         (0x00FE)
#define REG_ATPL250_INT_MASK_32                      (REG_ATPL250_INT_MASK | MASK_32)
#define REG_ATPL250_INT_MASK_VH8                     (REG_ATPL250_INT_MASK | MASK_VH8)
#define REG_ATPL250_INT_MASK_H8                      (REG_ATPL250_INT_MASK | MASK_H8)
#define REG_ATPL250_INT_MASK_L8                      (REG_ATPL250_INT_MASK | MASK_L8)
#define REG_ATPL250_INT_MASK_VL8                     (REG_ATPL250_INT_MASK | MASK_VL8)
#define REG_ATPL250_INT_FLAGS                        (0x00FF)
#define REG_ATPL250_INT_FLAGS_32                     (REG_ATPL250_INT_FLAGS | MASK_32)
#define REG_ATPL250_INT_FLAGS_VH8                    (REG_ATPL250_INT_FLAGS | MASK_VH8)
#define REG_ATPL250_INT_FLAGS_H8                     (REG_ATPL250_INT_FLAGS | MASK_H8)
#define REG_ATPL250_INT_FLAGS_L8                     (REG_ATPL250_INT_FLAGS | MASK_L8)
#define REG_ATPL250_INT_FLAGS_VL8                    (REG_ATPL250_INT_FLAGS | MASK_VL8)

/* Codes to access different buffers */
#define BCODE_ZONE0      (0x0000)
#define BCODE_ZONE1      (0x0800)
#define BCODE_ZONE2      (0x1000)
#define BCODE_ZONE3      (0x1800)
#define BCODE_ZONE4      (0x2000)
#define BCODE_ICHANNEL   (0x2800)
#define BCODE_COH        (0x3000)
#define BCODE_DIFT       (0x3800)
#define BCODE_DIFF       (0x4000)
#define BCODE_JUMPID     (0x4800)
#define BCODE_FREQAVG    (0x5000)
#define BCODE_ERASE      (0x7800)

/* Interrupt Masks */
#define PHY_NUM_INTERRUPT_SOURCES         32
#define INT_IOB_MASK_32                   0x00000001u
#define INT_PEAK1_MASK_32                 0x00000002u
#define INT_PEAK2_MASK_32                 0x00000004u
#define INT_NO_PEAK2_MASK_32              0x00000008u
#define INT_PEAK13_MASK_32                0x00000010u
#define INT_ZC_MASK_32                    0x00000020u
#define INT_VTB_MASK_32                   0x00000040u
#define INT_RS_MASK_32                    0x00000080u
#define INT_START_TX_MASK_32              0x00000F00u
#define INT_START_TX1_MASK_32             0x00000100u
#define INT_START_TX2_MASK_32             0x00000200u
#define INT_START_TX3_MASK_32             0x00000400u
#define INT_START_TX4_MASK_32             0x00000800u
#define INT_END_TX_MASK_32                0x0000F000u
#define INT_END_TX1_MASK_32               0x00001000u
#define INT_END_TX2_MASK_32               0x00002000u
#define INT_END_TX3_MASK_32               0x00004000u
#define INT_END_TX4_MASK_32               0x00008000u
#define INT_CD_TX_MASK_32                 0x000F0000u
#define INT_CD_TX1_MASK_32                0x00010000u
#define INT_CD_TX2_MASK_32                0x00020000u
#define INT_CD_TX3_MASK_32                0x00040000u
#define INT_CD_TX4_MASK_32                0x00080000u
#define INT_OVERLAP_TX_MASK_32            0x00F00000u
#define INT_OVERLAP_TX1_MASK_32           0x00100000u
#define INT_OVERLAP_TX2_MASK_32           0x00200000u
#define INT_OVERLAP_TX3_MASK_32           0x00400000u
#define INT_OVERLAP_TX4_MASK_32           0x00800000u
#define INT_RX_ERROR_MASK_32              0x01000000u
#define INT_TX_ERROR_MASK_32              0x02000000u
#define INT_NOISE_ERROR_MASK_32           0x04000000u
#define INT_NOISE_CAPTURE_MASK_32         0x08000000u
#define INT_BER_MASK_32                   0x10000000u
#define INT_CD_MASK_32                    0x20000000u
#define INT_SPI_ERR_MASK_32               0x40000000u
#define INT_START_MASK_32                 0x80000000u
#define INT_IOB_MASK_8                    0x01u
#define INT_PEAK1_MASK_8                  0x02u
#define INT_PEAK2_MASK_8                  0x04u
#define INT_NO_PEAK2_MASK_8               0x08u
#define INT_PEAK13_MASK_8                 0x10u
#define INT_ZC_MASK_8                     0x20u
#define INT_VTB_MASK_8                    0x40u
#define INT_RS_MASK_8                     0x80u
#define INT_START_TX_MASK_8               0x0Fu
#define INT_START_TX1_MASK_8              0x01u
#define INT_START_TX2_MASK_8              0x02u
#define INT_START_TX3_MASK_8              0x04u
#define INT_START_TX4_MASK_8              0x08u
#define INT_END_TX_MASK_8                 0xF0u
#define INT_END_TX1_MASK_8                0x10u
#define INT_END_TX2_MASK_8                0x20u
#define INT_END_TX3_MASK_8                0x40u
#define INT_END_TX4_MASK_8                0x80u
#define INT_CD_TX_MASK_8                  0x0Fu
#define INT_CD_TX1_MASK_8                 0x01u
#define INT_CD_TX2_MASK_8                 0x02u
#define INT_CD_TX3_MASK_8                 0x04u
#define INT_CD_TX4_MASK_8                 0x08u
#define INT_OVERLAP_TX_MASK_8             0xF0u
#define INT_OVERLAP_TX1_MASK_8            0x10u
#define INT_OVERLAP_TX2_MASK_8            0x20u
#define INT_OVERLAP_TX3_MASK_8            0x40u
#define INT_OVERLAP_TX4_MASK_8            0x80u
#define INT_RX_ERROR_MASK_8               0x01u
#define INT_TX_ERROR_MASK_8               0x02u
#define INT_NOISE_ERROR_MASK_8            0x04u
#define INT_NOISE_CAPTURE_MASK_8          0x08u
#define INT_BER_MASK_8                    0x10u
#define INT_CD_MASK_8                     0x20u
#define INT_SPI_ERR_MASK_8                0x40u
#define INT_START_MASK_8                  0x80u

/* Modulation Masks and shifts */
#define MOD_BPSK_MASK                     0x00u
#define MOD_QPSK_MASK                     0x20u
#define MOD_8PSK_MASK                     0x40u
#define MOD_QAM_MASK                      0x60u
#define MOD_TRUEPOINT_MASK                0x80u
#define MOD_BITS_MASK                     0x60u
#define MOD_BITS_AND_TRUEPOINT_MASK       0xE0u
#define MOD_TYPE_SHIFT                    5
#define TRUEPOINT_SHIFT                   7
#define HW_CHAIN_MOD_BITS_MASK            0xC0u
#define HW_CHAIN_MOD_BPSK_MASK            0x00u
#define HW_CHAIN_MOD_QPSK_MASK            0x40u
#define HW_CHAIN_MOD_8PSK_MASK            0x80u
#define HW_CHAIN_MOD_QAM_MASK             0xC0u

/* =========================================== MACROS TO READ AND WRITE REGISTERS =========================================== */

/* ---------------- Macros for REG_ATPL250_INOUTB_CTL ---------------- */

/* Set number of symbols to use of the InOut buffer */
#define atpl250_set_num_symbols_cfg(num_sym)         (pplc_if_write8(REG_ATPL250_INOUTB_CTL_VL8, num_sym - 1))

/* Set constellation points offset */
#define OFFSET_0            0 /* offset 0 */
#define OFFSET_1_PI_16      1 /* offset (1*PI/16) */
#define OFFSET_2_PI_16      2 /* offset (2*PI/16) */
#define OFFSET_3_PI_16      3 /* offset (3*PI/16) */
#define OFFSET_4_PI_16      4 /* offset (4*PI/16) */
#define OFFSET_5_PI_16      5 /* offset (5*PI/16) */
#define OFFSET_6_PI_16      6 /* offset (6*PI/16) */
#define OFFSET_7_PI_16      7 /* offset (7*PI/16) */
#define OFFSET_8_PI_16      8 /* offset (8*PI/16) */
#define OFFSET_9_PI_16      9 /* offset (9*PI/16) */
#define OFFSET_10_PI_16     10 /* offset (10*PI/16) */
#define OFFSET_11_PI_16     11 /* offset (11*PI/16) */
#define OFFSET_12_PI_16     12 /* offset (12*PI/16) */
#define OFFSET_13_PI_16     13 /* offset (13*PI/16) */
#define OFFSET_14_PI_16     14 /* offset (14*PI/16) */
#define OFFSET_15_PI_16     15 /* offset (15*PI/16) */
#define atpl250_set_constallation_offset(offset)     (pplc_if_and8(REG_ATPL250_INOUTB_CTL_L8, 0xF0); pplc_if_or8(REG_ATPL250_INOUTB_CTL_L8, offset))

/* Phase correction */
#define atpl250_set_phase_correction()               (pplc_if_or8(REG_ATPL250_INOUTB_CTL_L8, 0x10))
#define atpl250_clear_phase_correction()             (pplc_if_and8(REG_ATPL250_INOUTB_CTL_L8, 0xEF))

/* DIFT reference */
#define atpl250_dift_reference_prev()                (pplc_if_and8(REG_ATPL250_INOUTB_CTL_L8, 0xDF))
#define atpl250_dift_reference_2before()             (pplc_if_or8(REG_ATPL250_INOUTB_CTL_L8, 0x20))
#define atpl250_dift_reference_zone3()               (pplc_if_or8(REG_ATPL250_INOUTB_CTL_L8, 0x40))
#define atpl250_dift_reference_zone4()               (pplc_if_and8(REG_ATPL250_INOUTB_CTL_L8, 0xBF))
#define atpl250_dift_reference_present()             (pplc_if_or8(REG_ATPL250_INOUTB_CTL_L8, 0x80))
#define atpl250_dift_reference_above()               (pplc_if_and8(REG_ATPL250_INOUTB_CTL_L8, 0x7F))

/* Clear Rx full bit to continue reception */
#define atpl250_clear_rx_full()                      (pplc_if_and8(REG_ATPL250_INOUTB_CTL_H8, 0xFD))

/* Real and imaginary width */
#define atpl250_set_complex_width_8bit               (pplc_if_or8(REG_ATPL250_INOUTB_CTL_H8, 0x04))
#define atpl250_set_complex_width_16bit              (pplc_if_and8(REG_ATPL250_INOUTB_CTL_H8, 0xFB))

/* Real mode */
#define atpl250_set_iob_real_mode()                  (pplc_if_or8(REG_ATPL250_INOUTB_CTL_H8, 0x01))
#define atpl250_clear_iob_real_mode()                (pplc_if_and8(REG_ATPL250_INOUTB_CTL_H8, 0xFE))

/* RX lock after peak2 */
#define atpl250_set_peak2_rx_lock()                  (pplc_if_or8(REG_ATPL250_INOUTB_CTL_H8, 0x08))
#define atpl250_clear_peak2_rx_lock()                (pplc_if_and8(REG_ATPL250_INOUTB_CTL_H8, 0xF7))

/* Get InOut buffer busy */
#define atpl250_get_iob_busy()                       (pplc_if_read8(REG_ATPL250_INOUTB_CTL_H8) & 0x10)

/* Overwrite Msymbol of G3 preamble */
#define atpl250_set_overwrite_m()                    (pplc_if_or8(REG_ATPL250_INOUTB_CTL_H8, 0x20))
#define atpl250_clear_overwrite_m()                  (pplc_if_and8(REG_ATPL250_INOUTB_CTL_H8, 0xDF))

/* Set and clear Rx and Tx mode */
#define atpl250_set_rx_mode()                        (pplc_if_or8(REG_ATPL250_INOUTB_CTL_VH8, 0x02))
#define atpl250_clear_rx_mode()                      (pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~0x02u)))
#define atpl250_set_tx_mode()                        (pplc_if_or8(REG_ATPL250_INOUTB_CTL_VH8, 0x01))
#define atpl250_clear_tx_mode()                      (pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~0x01u)))

/* Set symbol ready to continue transmitting */
#define atpl250_set_sym_ready()                      (pplc_if_or8(REG_ATPL250_INOUTB_CTL_VH8, 0x04))
#define atpl250_read_sym_ready()                     ((pplc_if_read8(REG_ATPL250_INOUTB_CTL_VH8) & 0x04) >> 2)

/* Config channel characterization */
#define atpl250_divide_received_by_reference()       (pplc_if_or8(REG_ATPL250_INOUTB_CTL_VH8, 0x08))
#define atpl250_divide_reference_by_received()       (pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, 0xF7))

/* Connect InOut buffer to FFT */
#define atpl250_set_iobuf_to_fft()                   (pplc_if_or8(REG_ATPL250_INOUTB_CTL_VH8, 0x10))
#define atpl250_clear_iobuf_to_fft()                 (pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, 0xEF))

/* Set and clear True Point */
#define atpl250_set_true_point()                     (pplc_if_or8(REG_ATPL250_INOUTB_CTL_VH8, 0x80))
#define atpl250_clear_true_point()                   (pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~0x80u)))

/* Set modulation on modulator/demodulator */
#define atpl250_set_mod(mod, truepoint)   pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK)); pplc_if_or8( \
		REG_ATPL250_INOUTB_CTL_VH8, (truepoint << TRUEPOINT_SHIFT) | (mod << MOD_TYPE_SHIFT))
#define atpl250_set_mod_bpsk()            pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK))
#define atpl250_set_mod_bpsk_truepoint()  pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK)); pplc_if_or8( \
		REG_ATPL250_INOUTB_CTL_VH8, (1 << TRUEPOINT_SHIFT))
#define atpl250_set_mod_qpsk()            pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK)); pplc_if_or8( \
		REG_ATPL250_INOUTB_CTL_VH8, (0x01 << MOD_TYPE_SHIFT))
#define atpl250_set_mod_qpsk_truepoint()  pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK)); pplc_if_or8( \
		REG_ATPL250_INOUTB_CTL_VH8, (1 << TRUEPOINT_SHIFT) | (0x01 << MOD_TYPE_SHIFT))
#define atpl250_set_mod_8psk()            pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK)); pplc_if_or8( \
		REG_ATPL250_INOUTB_CTL_VH8, (0x02 << MOD_TYPE_SHIFT))
#define atpl250_set_mod_8psk_truepoint()  pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK)); pplc_if_or8( \
		REG_ATPL250_INOUTB_CTL_VH8, (1 << TRUEPOINT_SHIFT) | (0x02 << MOD_TYPE_SHIFT))
#define atpl250_set_mod_qam()             pplc_if_and8(REG_ATPL250_INOUTB_CTL_VH8, (uint8_t)(~MOD_BITS_AND_TRUEPOINT_MASK)); pplc_if_or8( \
		REG_ATPL250_INOUTB_CTL_VH8, (0x03 << MOD_TYPE_SHIFT))
#define atpl250_set_mod_qam_truepoint()   pplc_if_or8(REG_ATPL250_INOUTB_CTL_VH8, (1 << TRUEPOINT_SHIFT) | (0x03 << MOD_TYPE_SHIFT))
/* Set modulation on HW chain */
#define atpl250_set_hw_chain_bpsk()       pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~HW_CHAIN_MOD_BITS_MASK))
#define atpl250_set_hw_chain_qpsk()       pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~HW_CHAIN_MOD_BITS_MASK)); pplc_if_or8( \
		REG_ATPL250_INTERLEAVER_CTL_VL8, HW_CHAIN_MOD_QPSK_MASK)
#define atpl250_set_hw_chain_8psk()       pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~HW_CHAIN_MOD_BITS_MASK)); pplc_if_or8( \
		REG_ATPL250_INTERLEAVER_CTL_VL8, HW_CHAIN_MOD_8PSK_MASK)
#define atpl250_set_hw_chain_qam()        pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~HW_CHAIN_MOD_BITS_MASK)); pplc_if_or8( \
		REG_ATPL250_INTERLEAVER_CTL_VL8, HW_CHAIN_MOD_QAM_MASK)

/* RSSI and EVM macros */
#define atpl250_read_agc_factor()                    pplc_if_read16(REG_ATPL250_RSSI_H16)
#define atpl250_read_agcs_active()                   pplc_if_read8(REG_ATPL250_AGC_INFO_H8)
#define atpl250_read_agc_fine()                      pplc_if_read16(REG_ATPL250_AGC_INFO_L16) & 0x03FF
#define atpl250_read_rssi()                          pplc_if_read16(REG_ATPL250_RSSI_L16)
#define atpl250_read_evm_bpsk()                      pplc_if_read16(REG_ATPL250_EVM_BPSK_QPSK_H16)
#define atpl250_read_evm_qpsk()                      pplc_if_read16(REG_ATPL250_EVM_BPSK_QPSK_L16)
#define atpl250_read_evm_8psk()                      pplc_if_read16(REG_ATPL250_EVM_8PSK_L16)
#define atpl250_reset_rssi()                         pplc_if_or8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0x80)
#define atpl250_reset_evm()                          pplc_if_or8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0x40)
#define atpl250_enable_evm()                         pplc_if_or8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0x10)
#define atpl250_disable_evm()                        pplc_if_and8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0xEF)
#define atpl250_enable_rssi()                        pplc_if_or8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0x20)
#define atpl250_disable_rssi()                       pplc_if_and8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0xDF)
#define atpl250_enable_evm()                         pplc_if_or8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0x10)
#define atpl250_disable_evm()                        pplc_if_and8(REG_ATPL250_INOUTB_OVERFLOW_VL8, 0xEF)

#define atpl250_force_agc_ext(agc_active)            pplc_if_write8(REG_ATPL250_AGC_DELAYS_CONF_L8, 0x08 + agc_active)
#define atpl250_unforce_agc_ext(agc_active)          pplc_if_write8(REG_ATPL250_AGC_DELAYS_CONF_L8, agc_active)

#define atpl250_force_agc_int(agc_fine)              pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_L16, 0x8000 + (agc_fine << 4))
#define atpl250_unforce_agc_int(agc_fine)            pplc_if_write16(REG_ATPL250_AGC_FORCE_INT_CONF_L16, (agc_fine << 4))

#define atpl250_read_agcs_ext()                      (pplc_if_read8(REG_ATPL250_AGC_DELAYS_CONF_L8) & 0xF7)
#define atpl250_read_agc_int()                       ((pplc_if_read16(REG_ATPL250_AGC_FORCE_INT_CONF_L16) & 0x7FFF) >> 4)

#define atpl250_enable_ichannel_coh()                pplc_if_or8(REG_ATPL250_INOUTB_CTL_L8, 0x80); /* ichannel_coh set */
#define atpl250_disable_ichannel_coh()               pplc_if_and8(REG_ATPL250_INOUTB_CTL_L8, 0x7F); /* ichannel_coh unset */

/* Distance to reference carrier (only for differential in freq mod/demod) */
#define atpl250_set_distance_to_reference(distance)  (pplc_if_write16(REG_ATPL250_INOUTB_CONF1_L16, distance))

/* Number of symbols and samples per symbol in InOut buffer */
#define IOB_16_SYMBOLS_OF_64_SAMPLES   0x803F
#define IOB_8_SYMBOLS_OF_128_SAMPLES   0x407F
#define IOB_4_SYMBOLS_OF_256_SAMPLES   0x20FF
#define IOB_2_SYMBOLS_OF_512_SAMPLES   0x11FF
#define IOB_1_SYMBOL_OF_1024_SAMPLES   0x03FF
#define atpl250_set_iob_partition(partition)         (pplc_if_write16(REG_ATPL250_INOUTB_CONF1_H16, partition))

/* Set reference P symbol persistent */
#define atpl250_hold_on_reference(boolean_value)     (boolean_value ? pplc_if_or8(REG_ATPL250_INOUTB_CONF2_VL8, 0x10) :	\
	pplc_if_and8(REG_ATPL250_INOUTB_CONF2_VL8, (uint8_t)(~0x10u)))

/* Set reference distance in symbols */
#define atpl250_set_reference_dist_sym(num_sym)      (pplc_if_and8(REG_ATPL250_INOUTB_CONF2_VL8, \
	0xF0), pplc_if_or8(REG_ATPL250_INOUTB_CONF2_VL8, (num_sym & 0x0F)))

#define atpl250_set_ber_payload_coh()                (pplc_if_or8(REG_ATPL250_BER_PERIPH_CFG3_VH8, 0x02))
#define atpl250_set_ber_payload_diff()               (pplc_if_and8(REG_ATPL250_BER_PERIPH_CFG3_VH8, (uint8_t)(~0x02u)))
#define atpl250_set_ber_fch_coh()                    (pplc_if_or8(REG_ATPL250_BER_PERIPH_CFG3_VH8, 0x01))
#define atpl250_set_ber_fch_diff()                   (pplc_if_and8(REG_ATPL250_BER_PERIPH_CFG3_VH8, (uint8_t)(~0x01u)))

#define atpl250_get_peak1_addr()                     (pplc_if_read16(REG_ATPL250_PEAK_AD_L16))
#define atpl250_get_peak2_addr()                     (pplc_if_read16(REG_ATPL250_PEAK_AD_H16))

/* Interrupt clearing */
#define atpl250_clear_iob_int()                      (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_IOB_MASK_8)))
#define atpl250_clear_peak1_int()                    (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_PEAK1_MASK_8)))
#define atpl250_clear_peak2_int()                    (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_PEAK2_MASK_8)))
#define atpl250_clear_no_peak2_int()                 (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_NO_PEAK2_MASK_8)))
#define atpl250_clear_peak13_int()                   (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_PEAK13_MASK_8)))
#define atpl250_clear_zc_int()                       (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_ZC_MASK_8)))
#define atpl250_clear_vtb_int()                      (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_VTB_MASK_8)))
#define atpl250_clear_rs_int()                       (pplc_if_write8(REG_ATPL250_INT_FLAGS_VL8, (uint8_t)(~INT_RS_MASK_8)))
#define atpl250_clear_start_tx1_int()                (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_START_TX1_MASK_8)))
#define atpl250_clear_start_tx2_int()                (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_START_TX2_MASK_8)))
#define atpl250_clear_start_tx3_int()                (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_START_TX3_MASK_8)))
#define atpl250_clear_start_tx4_int()                (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_START_TX4_MASK_8)))
#define atpl250_clear_end_tx1_int()                  (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_END_TX1_MASK_8)))
#define atpl250_clear_end_tx2_int()                  (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_END_TX2_MASK_8)))
#define atpl250_clear_end_tx3_int()                  (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_END_TX3_MASK_8)))
#define atpl250_clear_end_tx4_int()                  (pplc_if_write8(REG_ATPL250_INT_FLAGS_L8, (uint8_t)(~INT_END_TX4_MASK_8)))
#define atpl250_clear_cd_tx1_int()                   (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_CD_TX1_MASK_8)))
#define atpl250_clear_cd_tx2_int()                   (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_CD_TX2_MASK_8)))
#define atpl250_clear_cd_tx3_int()                   (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_CD_TX3_MASK_8)))
#define atpl250_clear_cd_tx4_int()                   (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_CD_TX4_MASK_8)))
#define atpl250_clear_overlap_tx1_int()              (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_OVERLAP_TX1_MASK_8)))
#define atpl250_clear_overlap_tx2_int()              (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_OVERLAP_TX2_MASK_8)))
#define atpl250_clear_overlap_tx3_int()              (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_OVERLAP_TX3_MASK_8)))
#define atpl250_clear_overlap_tx4_int()              (pplc_if_write8(REG_ATPL250_INT_FLAGS_H8, (uint8_t)(~INT_OVERLAP_TX4_MASK_8)))
#define atpl250_clear_rx_error_int()                 (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_RX_ERROR_MASK_8)))
#define atpl250_clear_tx_error_int()                 (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_TX_ERROR_MASK_8)))
#define atpl250_clear_noise_error_int()              (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_NOISE_ERROR_MASK_8)))
#define atpl250_clear_noise_capture_int()            (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_NOISE_CAPTURE_MASK_8)))
#define atpl250_clear_ber_int()                      (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_BER_MASK_8)))
#define atpl250_clear_cd_int()                       (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_CD_MASK_8)))
#define atpl250_read_cd_int()                        (pplc_if_read8(REG_ATPL250_INT_FLAGS_VH8) & (uint8_t)(INT_CD_MASK_8))
#define atpl250_clear_spi_err_int()                  (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_SPI_ERR_MASK_8)))
#define atpl250_clear_start_int()                    (pplc_if_write8(REG_ATPL250_INT_FLAGS_VH8, (uint8_t)(~INT_START_MASK_8)))
/* FFT Reset */
#define atpl250_fft_push_rst()                       (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x01))
#define atpl250_fft_release_rst()                    (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x01u)))
/* InOut buffer Reset */
#define atpl250_iob_push_rst()                       (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x02))
#define atpl250_iob_release_rst()                    (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x02u)))
/* InOut buffer and FFT Reset */
#define atpl250_iob_and_fft_push_rst()               (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x03))
#define atpl250_iob_and_fft_release_rst()            (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x03u)))
/* TxRx buffer Reset */
#define atpl250_txrxb_push_rst()                     (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x04))
#define atpl250_txrxb_release_rst()                  (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x04u)))
/* Rotator Reset */
#define atpl250_rotator_push_rst()                   (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_L8, 0x02))
#define atpl250_rotator_release_rst()                (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_L8, (uint8_t)(~0x02u)))
/* Rotator and FFT Reset */
#define atpl250_rotator_and_fft_push_rst()           (pplc_if_or32(REG_ATPL250_GLOBAL_SRST_32, 0x01000200))
#define atpl250_rotator_and_fft_release_rst()        (pplc_if_and32(REG_ATPL250_GLOBAL_SRST_32, (uint32_t)(~0x01000200u)))
/* TxRx buffer, Rotator and FFT Reset */
#define atpl250_txrxb_rotator_and_fft_push_rst()     (pplc_if_or32(REG_ATPL250_GLOBAL_SRST_32, 0x05000200))
#define atpl250_txrxb_rotator_and_fft_release_rst()  (pplc_if_and32(REG_ATPL250_GLOBAL_SRST_32, (uint32_t)(~0x05000200u)))
/* InOut buffer, rotator and FFT Reset */
#define atpl250_iob_rotator_and_fft_push_rst()       (pplc_if_or32(REG_ATPL250_GLOBAL_SRST_32, 0x03000200))
#define atpl250_iob_rotator_and_fft_release_rst()    (pplc_if_and32(REG_ATPL250_GLOBAL_SRST_32, (uint32_t)(~0x03000200u)))
/* SyncM detector Reset */
#define atpl250_syncm_detector_push_rst()            (pplc_if_and8(REG_ATPL250_SYNCM_CTL_L8, 0x7F))
#define atpl250_syncm_detector_release_rst()         (pplc_if_or8(REG_ATPL250_SYNCM_CTL_L8, 0x80))
/* Sync Reset */
#define atpl250_sync_push_rst()                      (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x08))
#define atpl250_sync_release_rst()                   (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x08u)))
/* Sync, Preamble and TXRXB Reset */
#define atpl250_sync_pream_txrxb_push_rst()          (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x1C))
#define atpl250_sync_pream_txrxb_release_rst()       (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x1Cu)))
/* BER Reset */
#define atpl250_ber_push_rst()                       (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_L8, 0x01))
#define atpl250_ber_release_rst()                    (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_L8, (uint8_t)(~0x01u)))
/* Preamble Reset */
#define atpl250_pre_push_rst()                       (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x10))
#define atpl250_pre_release_rst()                    (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x10u)))
/* Emit Ctl Reset */
#define atpl250_emit_ctl_push_rst()                  (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x20))
#define atpl250_emit_ctl_release_rst()               (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x20u)))
/* Zero Cross Reset */
#define atpl250_zcross_push_rst()                    (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x40))
#define atpl250_zcross_release_rst()                 (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x40u)))
/* Interleaver Reset */
#define atpl250_intlv_push_rst()                     (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x80))
#define atpl250_intlv_release_rst()                  (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, (uint8_t)(~0x80u)))
/* CLKPHVAR Reset */
#define atpl250_clkphvar_push_rst()                  (pplc_if_or8(REG_ATPL250_GLOBAL_SRST_L8, 0x04))
#define atpl250_clkphvar_release_rst()               (pplc_if_and8(REG_ATPL250_GLOBAL_SRST_L8, (uint8_t)(~0x04u)))

/* Clear InOut buffer */
#define atpl250_clear_iobuf()                        (pplc_if_write32(BCODE_ERASE, 0))
/* Enable / Disable HW chain */
#define enable_HW_chain()                            (pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x01))
#define disable_HW_chain()                           (pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u)))
/* Enable / Disable HW chain auto reset */
#define enable_HW_chain_autorst()                    (pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_H8, (uint8_t)(~0x10u)))
#define disable_HW_chain_autorst()                   (pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_H8, 0x10))
/* Enable/Disable Demodulation as BPSK */
#define enable_demod_as_bpsk()                       (pplc_if_write8(REG_ATPL250_INOUTB_QNTZ4_VL8, 0x04))
#define disable_demod_as_bpsk()                      (pplc_if_write8(REG_ATPL250_INOUTB_QNTZ4_VL8, 0x00))

/*CD Interrupt Enable/Disable*/
#define enable_CD_interrupt()                        (pplc_if_or8(REG_ATPL250_INT_MASK_VH8, 0x20))
#define disable_CD_interrupt()                       (pplc_if_and8(REG_ATPL250_INT_MASK_VH8, (uint8_t)(~0x20u)))

/* CD Peak Full Enable/Disable */
#define atpl250_enable_CD_peakfull()                 (pplc_if_or8(REG_ATPL250_CD_CFG_VL8, 0x04))
#define atpl250_disable_CD_peakfull()                (pplc_if_and8(REG_ATPL250_CD_CFG_VL8, (uint8_t)(~0x04u)))

/* Check fixed value register integrity */
#define atpl250_spi_comm_corrupted()                 (pplc_if_read32(REG_ATPL250_EMPTY_32) != 0xA250A250)

#endif /* _ATPL250REG_INCLUDED_ */
