/**
 * \file
 *
 * \brief ATPL250ABN_v2 board init.
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

#include "compiler.h"
#include "pmc.h"
#include "pio.h"
#include "ioport.h"
#include "board.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "asf.h"

/**
 * \addtogroup atpl250abn_group
 * @{
 */

/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do {\
		ioport_set_port_mode(port, masks, mode);\
		ioport_disable_port(port, masks);\
	} while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)

/**
 * \brief Set input mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 * \param sense Sense for interrupt detection (\ref ioport_sense)
 */
#define ioport_set_pin_input_mode(pin, mode, sense) \
	do {\
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);\
		ioport_set_pin_mode(pin, mode);\
		ioport_set_pin_sense_mode(pin, sense);\
	} while (0)

void board_init(void)
{
#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
#else
	WDT->WDT_MR = (WDT->WDT_MR & 0xFFFFF000) | 0x7FF; /* 8 secs */
#endif

	/* Select the crystal oscillator to be the source of the slow clock,
	 * as it provides a more accurate frequency
	 */
        
#ifdef CONF_BOARD_ENABLE_CACHE
  /* Enabling the Cache */
  SCB_EnableICache(); 
  SCB_EnableDCache();
#endif

#ifdef CONF_BOARD_32K_XTAL
	supc_switch_sclk_to_32kxtal(SUPC, 0);
#endif

	/* GPIO has been deprecated, the old code just keeps it for compatibility.
	 * In new designs IOPORT is used instead.
	 * Here IOPORT must be initialized for others to use before setting up IO.
	 */
	ioport_init();

	/* Configure the pins connected to LEDs as output and set their
	 * default initial state to high (LEDs off).
	 */
	ioport_set_pin_dir(LED0_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
	ioport_set_pin_dir(LED1_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED1_GPIO, LED0_INACTIVE_LEVEL);

	/* Configure UART0 pins */
#ifdef CONF_BOARD_UART0
	ioport_set_port_peripheral_mode(PINS_UART0_PORT, PINS_UART0, PINS_UART0_FLAGS);
#endif

#ifdef CONF_BOARD_UART2
	ioport_set_port_peripheral_mode(PINS_UART2_PORT, PINS_UART2, PINS_UART2_FLAGS);
#endif

#ifdef CONF_BOARD_UART4
	ioport_set_port_peripheral_mode(PINS_UART4_PORT, PINS_UART4, PINS_UART4_FLAGS);
#endif

	/* Configure UART1 pins (CONSOLE) */
#ifdef CONF_BOARD_UART_CONSOLE
	switch (CONSOLE_UART_ID){
	case ID_UART0:
		ioport_set_port_peripheral_mode(PINS_UART0_PORT, PINS_UART0, PINS_UART0_FLAGS);
		break;
	case ID_UART2:
		ioport_set_port_peripheral_mode(PINS_UART2_PORT, PINS_UART2, PINS_UART2_FLAGS);
		break;
	case ID_UART4:
		ioport_set_port_peripheral_mode(PINS_UART4_PORT, PINS_UART4, PINS_UART4_FLAGS);
		break;
	}
#endif

	/* Configure USART0 pins */
#ifdef CONF_BOARD_USART0
	/* Configure USART0 RXD pin */
	ioport_set_pin_peripheral_mode(PIN_USART0_RXD_IDX, PIN_USART0_RXD_FLAGS);
	/* Configure USART0 TXD pin */
	ioport_set_pin_peripheral_mode(PIN_USART0_TXD_IDX, PIN_USART0_TXD_FLAGS);
#endif

	/* Configure USART1 pins */
#ifdef CONF_BOARD_USART1
	/* Configure USART1 RXD pin */
	ioport_set_pin_peripheral_mode(PIN_USART1_RXD_IDX, PIN_USART1_RXD_FLAGS);
	/* Configure USART1 TXD pin */
	ioport_set_pin_peripheral_mode(PIN_USART1_TXD_IDX, PIN_USART1_TXD_FLAGS);
#endif

	/* Configure Xplain PRO SLP pin */
#ifdef CONF_BOARD_XP_SLP
	ioport_set_pin_dir(XP_SLP_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(XP_SLP_GPIO, XP_SLP_INACTIVE_LEVEL);
#endif

	/* Configure USB Detect pin */
#ifdef CONF_BOARD_USB_DETECT
	ioport_set_pin_input_mode(GPIO_USB_DETECT, GPIO_USB_DETECT_FLAGS, GPIO_USB_DETECT_SENSE);
#endif

	/* Configure Shutdown Detect pin */
#ifdef CONF_SHUTDOWN_DETECT
	ioport_set_pin_dir(SHUTDOWN_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SHUTDOWN_GPIO, SHUTDOWN_INACTIVE_LEVEL);
#endif

#if defined (CONF_BOARD_SD_MMC_HSMCI)
	/* Configure HSMCI pins */
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCDA_GPIO, PIN_HSMCI_MCCDA_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCK_GPIO, PIN_HSMCI_MCCK_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA0_GPIO, PIN_HSMCI_MCDA0_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA1_GPIO, PIN_HSMCI_MCDA1_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA2_GPIO, PIN_HSMCI_MCDA2_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA3_GPIO, PIN_HSMCI_MCDA3_FLAGS);
	ioport_set_pin_peripheral_mode(SD_MMC_0_CD_GPIO, SD_MMC_0_CD_FLAGS);
#endif

#ifdef CONF_BOARD_KSZ8051MNL
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_RXC_IDX, PIN_KSZ8051MNL_RXC_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_TXC_IDX, PIN_KSZ8051MNL_TXC_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_TXEN_IDX, PIN_KSZ8051MNL_TXEN_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_TXD3_IDX, PIN_KSZ8051MNL_TXD3_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_TXD2_IDX, PIN_KSZ8051MNL_TXD2_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_TXD1_IDX, PIN_KSZ8051MNL_TXD1_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_TXD0_IDX, PIN_KSZ8051MNL_TXD0_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_RXD3_IDX, PIN_KSZ8051MNL_RXD3_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_RXD2_IDX, PIN_KSZ8051MNL_RXD2_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_RXD1_IDX, PIN_KSZ8051MNL_RXD1_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_RXD0_IDX, PIN_KSZ8051MNL_RXD0_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_RXER_IDX, PIN_KSZ8051MNL_RXER_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_RXDV_IDX, PIN_KSZ8051MNL_RXDV_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_CRS_IDX, PIN_KSZ8051MNL_CRS_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_COL_IDX, PIN_KSZ8051MNL_COL_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_MDC_IDX, PIN_KSZ8051MNL_MDC_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_KSZ8051MNL_MDIO_IDX, PIN_KSZ8051MNL_MDIO_FLAGS);
	ioport_set_pin_dir(PIN_KSZ8051MNL_INTRP_IDX, IOPORT_DIR_INPUT);
#endif

#if (defined(CONF_BOARD_SPI0) || defined(CONF_BOARD_ATPL250))
	ioport_set_pin_peripheral_mode(SPI_MISO_GPIO, SPI_MISO_FLAGS);
	ioport_set_pin_peripheral_mode(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
	ioport_set_pin_peripheral_mode(SPI_SPCK_GPIO, SPI_SPCK_FLAGS);
  ioport_set_pin_peripheral_mode(SPI_NPCS2_GPIO, SPI_NPCS2_FLAGS);
#ifdef CONF_BOARD_SPI_NPCS0
	ioport_set_pin_peripheral_mode(SPI_NPCS0_GPIO, SPI_NPCS0_FLAGS);
#endif
#ifdef CONF_BOARD_SPI_NPCS1
	ioport_set_pin_peripheral_mode(SPI_NPCS1_GPIO, SPI_NPCS1_FLAGS);
#endif
#endif

#if (defined(CONF_BOARD_TWI0) || defined(CONF_BOARD_EDBG_TWI))
	ioport_set_pin_peripheral_mode(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
	ioport_set_pin_peripheral_mode(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
#endif

#ifdef CONF_BOARD_SDRAMC
	pio_configure_pin(SDRAM_BA0_PIO, SDRAM_BA0_FLAGS);
	pio_configure_pin(SDRAM_BA1_PIO, SDRAM_BA1_FLAGS);
	pio_configure_pin(SDRAM_SDCK_PIO, SDRAM_SDCK_FLAGS);
	pio_configure_pin(SDRAM_SDCKE_PIO, SDRAM_SDCKE_FLAGS);
	pio_configure_pin(SDRAM_SDCS_PIO, SDRAM_SDCS_FLAGS);
	pio_configure_pin(SDRAM_RAS_PIO, SDRAM_RAS_FLAGS);
	pio_configure_pin(SDRAM_CAS_PIO, SDRAM_CAS_FLAGS);
	pio_configure_pin(SDRAM_SDWE_PIO, SDRAM_SDWE_FLAGS);
	pio_configure_pin(SDRAM_NBS0_PIO, SDRAM_NBS0_FLAGS);
	pio_configure_pin(SDRAM_NBS1_PIO, SDRAM_NBS1_FLAGS);
	pio_configure_pin(SDRAM_A2_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A3_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A4_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A5_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A6_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A7_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A8_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A9_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A10_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A11_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_SDA10_PIO, SDRAM_SDA10_FLAGS);
	pio_configure_pin(SDRAM_A13_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_D0_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D1_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D2_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D3_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D4_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D5_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D6_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D7_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D8_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D9_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D10_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D11_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D12_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D13_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D14_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D15_PIO, SDRAM_D_FLAGS); 
        
#endif
   

        
	/* Configure VDD sense GPIO to AFEC0 */
	ioport_set_pin_dir(VDD_SENSE_GPIO, IOPORT_DIR_INPUT);


         
}
