/**
 * \file
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

#ifndef _PIC32CX_SPI4_INSTANCE_
#define _PIC32CX_SPI4_INSTANCE_

/* ========== Register definition for SPI4 peripheral ========== */
#if (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
  #define REG_SPI4_CR                       (0x40010400U) /**< \brief (SPI4) Control Register */
  #define REG_SPI4_MR                       (0x40010404U) /**< \brief (SPI4) Mode Register */
  #define REG_SPI4_RDR                      (0x40010408U) /**< \brief (SPI4) Receive Data Register */
  #define REG_SPI4_TDR                      (0x4001040CU) /**< \brief (SPI4) Transmit Data Register */
  #define REG_SPI4_SR                       (0x40010410U) /**< \brief (SPI4) Status Register */
  #define REG_SPI4_IER                      (0x40010414U) /**< \brief (SPI4) Interrupt Enable Register */
  #define REG_SPI4_IDR                      (0x40010418U) /**< \brief (SPI4) Interrupt Disable Register */
  #define REG_SPI4_IMR                      (0x4001041CU) /**< \brief (SPI4) Interrupt Mask Register */
  #define REG_SPI4_CSR                      (0x40010430U) /**< \brief (SPI4) Chip Select Register */
  #define REG_SPI4_FMR                      (0x40010440U) /**< \brief (SPI4) FIFO Mode Register */
  #define REG_SPI4_FLR                      (0x40010444U) /**< \brief (SPI4) FIFO Level Register */
  #define REG_SPI4_CMPR                     (0x40010448U) /**< \brief (SPI4) Comparison Register */
  #define REG_SPI4_WPMR                     (0x400104E4U) /**< \brief (SPI4) Write Protection Mode Register */
  #define REG_SPI4_WPSR                     (0x400104E8U) /**< \brief (SPI4) Write Protection Status Register */
  #define REG_SPI4_RPR                      (0x40010500U) /**< \brief (SPI4) Receive Pointer Register */
  #define REG_SPI4_RCR                      (0x40010504U) /**< \brief (SPI4) Receive Counter Register */
  #define REG_SPI4_TPR                      (0x40010508U) /**< \brief (SPI4) Transmit Pointer Register */
  #define REG_SPI4_TCR                      (0x4001050CU) /**< \brief (SPI4) Transmit Counter Register */
  #define REG_SPI4_RNPR                     (0x40010510U) /**< \brief (SPI4) Receive Next Pointer Register */
  #define REG_SPI4_RNCR                     (0x40010514U) /**< \brief (SPI4) Receive Next Counter Register */
  #define REG_SPI4_TNPR                     (0x40010518U) /**< \brief (SPI4) Transmit Next Pointer Register */
  #define REG_SPI4_TNCR                     (0x4001051CU) /**< \brief (SPI4) Transmit Next Counter Register */
  #define REG_SPI4_PTCR                     (0x40010520U) /**< \brief (SPI4) Transfer Control Register */
  #define REG_SPI4_PTSR                     (0x40010524U) /**< \brief (SPI4) Transfer Status Register */
#else
  #define REG_SPI4_CR      (*(__O  uint32_t*)0x40010400U) /**< \brief (SPI4) Control Register */
  #define REG_SPI4_MR      (*(__IO uint32_t*)0x40010404U) /**< \brief (SPI4) Mode Register */
  #define REG_SPI4_RDR     (*(__I  uint32_t*)0x40010408U) /**< \brief (SPI4) Receive Data Register */
  #define REG_SPI4_TDR     (*(__O  uint32_t*)0x4001040CU) /**< \brief (SPI4) Transmit Data Register */
  #define REG_SPI4_SR      (*(__I  uint32_t*)0x40010410U) /**< \brief (SPI4) Status Register */
  #define REG_SPI4_IER     (*(__O  uint32_t*)0x40010414U) /**< \brief (SPI4) Interrupt Enable Register */
  #define REG_SPI4_IDR     (*(__O  uint32_t*)0x40010418U) /**< \brief (SPI4) Interrupt Disable Register */
  #define REG_SPI4_IMR     (*(__I  uint32_t*)0x4001041CU) /**< \brief (SPI4) Interrupt Mask Register */
  #define REG_SPI4_CSR     (*(__IO uint32_t*)0x40010430U) /**< \brief (SPI4) Chip Select Register */
  #define REG_SPI4_FMR     (*(__IO uint32_t*)0x40010440U) /**< \brief (SPI4) FIFO Mode Register */
  #define REG_SPI4_FLR     (*(__I  uint32_t*)0x40010444U) /**< \brief (SPI4) FIFO Level Register */
  #define REG_SPI4_CMPR    (*(__IO uint32_t*)0x40010448U) /**< \brief (SPI4) Comparison Register */
  #define REG_SPI4_WPMR    (*(__IO uint32_t*)0x400104E4U) /**< \brief (SPI4) Write Protection Mode Register */
  #define REG_SPI4_WPSR    (*(__I  uint32_t*)0x400104E8U) /**< \brief (SPI4) Write Protection Status Register */
  #define REG_SPI4_RPR     (*(__IO uint32_t*)0x40010500U) /**< \brief (SPI4) Receive Pointer Register */
  #define REG_SPI4_RCR     (*(__IO uint32_t*)0x40010504U) /**< \brief (SPI4) Receive Counter Register */
  #define REG_SPI4_TPR     (*(__IO uint32_t*)0x40010508U) /**< \brief (SPI4) Transmit Pointer Register */
  #define REG_SPI4_TCR     (*(__IO uint32_t*)0x4001050CU) /**< \brief (SPI4) Transmit Counter Register */
  #define REG_SPI4_RNPR    (*(__IO uint32_t*)0x40010510U) /**< \brief (SPI4) Receive Next Pointer Register */
  #define REG_SPI4_RNCR    (*(__IO uint32_t*)0x40010514U) /**< \brief (SPI4) Receive Next Counter Register */
  #define REG_SPI4_TNPR    (*(__IO uint32_t*)0x40010518U) /**< \brief (SPI4) Transmit Next Pointer Register */
  #define REG_SPI4_TNCR    (*(__IO uint32_t*)0x4001051CU) /**< \brief (SPI4) Transmit Next Counter Register */
  #define REG_SPI4_PTCR    (*(__O  uint32_t*)0x40010520U) /**< \brief (SPI4) Transfer Control Register */
  #define REG_SPI4_PTSR    (*(__I  uint32_t*)0x40010524U) /**< \brief (SPI4) Transfer Status Register */
#endif /* (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#endif /* _PIC32CX_SPI4_INSTANCE_ */
