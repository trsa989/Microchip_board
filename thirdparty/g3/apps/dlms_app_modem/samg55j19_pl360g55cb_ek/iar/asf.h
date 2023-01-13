/**
 * \file
 *
 * \brief Autogenerated API include file for the Atmel Software Framework (ASF)
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
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

#ifndef ASF_H
#define ASF_H

/*
 * This file includes all API header files for the selected drivers from ASF.
 * Note: There might be duplicate includes required by more than one driver.
 *
 * The file is automatically generated and will be re-written when
 * running the ASF driver selector tool. Any changes will be discarded.
 */

// From module: ADC - Analog-to-digital Converter
#include <adc2.h>

// From module: Common SAM/PIC32CX compiler driver
#include <compiler.h>
#include <status_codes.h>

// From module: Coupling TX configuration for PL360 (G3)
#include <coup_tx_config.h>

// From module: Delay routines
#include <delay.h>

// From module: EEFC - Enhanced Embedded Flash Controller
#include <efc.h>

// From module: FLEXCOM - Flexible Serial Communication Controller
#include <flexcom.h>

// From module: Flash - SAM Flash Service API
#include <flash_efc.h>

// From module: FreeRTOS mini Real-Time Kernel
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <StackMacros.h>
#include <croutine.h>
#include <list.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>

// From module: G3 MAC Real Time(RT) PLC Host Driver
#include <atpl360.h>
#include <atpl360_boot.h>
#include <atpl360_comm.h>
#include <atpl360_exception.h>

// From module: G3 Operative System Support (OSS) for Atmel PLC devices
#include <oss_if.h>

// From module: GPBR - General Purpose Backup Register
#include <gpbr.h>

// From module: GPIO - General purpose Input/Output
#include <gpio.h>

// From module: Generic board support
#include <board.h>

// From module: IOPORT - General purpose I/O service
#include <ioport.h>

// From module: Interrupt management - SAM implementation
#include <interrupt.h>

// From module: MAC Wrapper Layer
#include <mac_wrapper.h>
#include <mac_wrapper_defs.h>

// From module: MATRIX - Bus Matrix
#include <matrix.h>

// From module: PDC - Peripheral DMA Controller Example
#include <pdc.h>

// From module: PIO - Parallel Input/Output Controller - SAM implementation
#include <pio.h>

// From module: PL360G55CB_EK LED support enabled
#include <led.h>

// From module: PLC Buffered PDC Uart Service Interface - SAMG
#include <buart_if.h>

// From module: PLC Buffered PDC Usart Service Interface - SAM4 and SAMG
#include <busart_if.h>

// From module: PLC Buffered USB Service Interface - SAMG
#include <usb_wrp.h>

// From module: PLC PRIME CRC calculation service
#include <pcrc.h>

// From module: PLC Universal Serial Interface
#include <usi.h>

// From module: PMC - Power Management Controller - SAM implementation
#include <pmc.h>
#include <sleep.h>

// From module: Part identification macros
#include <parts.h>

// From module: Physical Abstraction Layer (PAL) interface - ATPL360 and G3 MAC RT
#include <pal.h>

// From module: Proxy Power Line Communication (PLC) Controller Interface - SAM4C/SAMG55 implementation
#include <pplc_if.h>

// From module: SAM/PIC32CX FPU driver
#include <fpu.h>

// From module: SPI - Serial Peripheral Interface
#include <spi.h>

// From module: SUPC - Supply Controller
#include <supc.h>

// From module: Sleep manager - SAM implementation
#include <sam/sleepmgr.h>
#include <sleepmgr.h>

// From module: Standard UDC I/O (stdio) - SAM implementation
#include <stdio_udc.h>

// From module: Standard serial I/O (stdio) - SAM implementation
#include <stdio_serial.h>

// From module: System Clock Control - SAMG implementation
#include <sysclk.h>

// From module: TC - Timer Counter
#include <tc.h>

// From module: USART - Serial interface - SAM implementation for devices with only USART
#include <serial.h>

// From module: USART - Univ. Syn Async Rec/Trans
#include <usart.h>

// From module: USB CDC Protocol
#include <usb_protocol_cdc.h>

// From module: USB Device CDC (Single Interface Device)
#include <udi_cdc.h>

// From module: USB Device Stack Core (Common API)
#include <udc.h>
#include <udd.h>

// From module: WDT - Watchdog Timer
#include <wdt.h>

// From module: pio_handler support enabled
#include <pio_handler.h>

#endif // ASF_H
