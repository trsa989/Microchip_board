/**
 * \file
 *
 * \brief SAM SERIAL Bootloader
 *
 * Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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

/**
 * \mainpage SERIAL Bootloader Application
 *
 * \section Purpose
 *
 * The example will help new users get familiar with Microchip's SAMG55 microcontrollers
 * bootloader for SERIAL.
 *
 * \section Requirements
 *
 * This package can be used with PL360G55CB_EK and PL360G55CF_EK.
 *
 * \section Description
 *
 * The bootloader code will be located at 0x0 and executed before any applicative code.
 * Applications compiled to be executed along with the bootloader will start at 0x4000.
 * Before jumping to the application, the bootloader changes the VTOR register
 * to use the interrupt vectors of the application.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board.
 * -# Start the application.
 * -# Setting Boot Jumper the board will enter SERIAL monitor mode.
 *
 */

/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "asf.h"
#include <stdio.h>
#include <stdlib.h>
#include <io.h>
#include <string.h>
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_bootloader.h"
#include "serial_monitor.h"
#include "usart_serial.h"
#include "gpbr.h"

#define USE_BOOT_LOAD_PIN
#define STAY_IN_BOOT_KEY	777 /* If this number is written in GPBR3 stay in bootloader  */


const unsigned short crc16_lookup_table[256] =
{
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


static void check_start_application(void);
static uint32_t	check_app_consistency();
static bool CheckAppCRC();

static void CRC16_InitHeader();
static void updateCRC(unsigned char c);


#ifdef CONF_USBCDC_INTERFACE_SUPPORT
static volatile bool main_b_cdc_enable = false;
static volatile bool main_b_dtr_enable = false;
#endif


uint8_t CRC16_High = 0x00;
uint8_t	CRC16_Low = 0x00;


/**
 * \brief Execute an application from the specified address without any check
 *
 * \param address Application address
 */
void call_app(uint32_t address)
{
	uint8_t i;

	__disable_irq();
	/* Disable SysTick */
	SysTick->CTRL = 0;
	/* Disable IRQs & clear pending IRQs */
	for (i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	/* Modify vector table location */
	__DSB();
	__ISB();
	/* set the stack pointer also to the start of the firmware app */
	__set_MSP(*(int *)(address));
	/* offset the start of the vector table (first 6 bits must be zero) */
	SCB->VTOR = ((uint32_t)address & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();
	__enable_irq();

	/* jump to the start of the firmware, casting the address as function
	 * pointer to the start of the firmware. We want to jump to the address
	 * of the reset */

	/* handler function, that is the value that is being pointed at position */
	void (*runFirmware)(void) = NULL;
	runFirmware = (void (*)(void))(*(uint32_t *)(address + 4));
	runFirmware();
}

/**
 * \brief Check the application startup condition (called from console, boot pin is always ignored )
 *
 */
void check_start_application_from_console(void)
{
	
	/* Check application consistency (CRC)*/
	volatile uint32_t result = check_app_consistency(); /* Check return value for additional info */	
	if(result != 0) return; 

	call_app(APP_START_ADDRESS);
}

static uint32_t	check_app_consistency()
{
	
	/**
    * Test reset vector of application @APP_START_ADDRESS+4
    * Stay in SERIAL if *(APP_START+0x4) == 0xFFFFFFFF
    * Application erased condition
  */
	volatile uint32_t app_start_address = *(uint32_t *)(APP_START_ADDRESS + 4);
	if (app_start_address == 0xFFFFFFFF) {
    /* Stay in bootloader */
		return 1;
	}
	
	/* Check app CRC */
	volatile bool crc_res = CheckAppCRC();
	
	if(!crc_res) return 2;
	
	return 0; /* All good, application is valid */
}


static bool CheckAppCRC()
{
	uint32_t app_len = *((uint32_t*)APP_ADDITIONAL_INFO_BASE);
	uint32_t end_address = APP_START_ADDRESS + app_len - 2; /* -2 is reserved for CRC */
	CRC16_InitHeader();
	
	uint8_t buff;
	for(uint32_t idx = APP_START_ADDRESS; idx < end_address; idx++)
	{
		buff = *(uint8_t*)idx;
		updateCRC(buff);
	}

	uint16_t crc16 = (CRC16_High << 8) | CRC16_Low;
	uint16_t crc_flash = *(uint16_t*)end_address;
	return (crc16 == crc_flash);
}


/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{
	#ifdef USE_BOOT_LOAD_PIN
  /* Check Boot Jumper */
	ioport_init();
	ioport_set_pin_dir(BOOT_LOAD_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BOOT_LOAD_PIN, IOPORT_MODE_PULLUP);
	
	volatile bool pin_level = ioport_get_pin_level(BOOT_LOAD_PIN);
	if (pin_level == 0) {
		return;
	}
	#endif
	
	/* Check if app has forced bootloader */
	if(gpbr_read(GPBR3) == STAY_IN_BOOT_KEY) return;
	
	/* Check application consistency (Flash not erased and CRC)*/
	volatile uint32_t result = check_app_consistency(); /* Check return value for additional info */
	if(result != 0) return; 

	call_app(APP_START_ADDRESS);
}

/**
 *  \brief SERIAL Main loop.
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	
	/* Jump in application if all conditions are satisfied */
	check_start_application();
	
  /* ASF function to setup clocking. */
	sysclk_init();

  /* Atmel library function to setup for the evaluation kit being used. */
	board_init();
	LED_On(LED0);

#ifdef CONF_USBCDC_INTERFACE_SUPPORT
  /* Start USB stack */
	udc_start();
#endif
  /* UART is enabled in all cases */
	usart_open();
	
	printf("BL: %s  %s\r\n", __DATE__, __TIME__);
	
	/* Wait for a complete enum on usb or a '#' char on serial line */
	while (1) {
#ifdef CONF_USBCDC_INTERFACE_SUPPORT
    /* Check if a USB enumeration has succeeded and com port was opened */
		if (main_b_cdc_enable && main_b_dtr_enable) {
			serial_monitor_init(SERIAL_INTERFACE_USBCDC);
            /* SERIAL on USB loop */
			while (1) {
				serial_monitor_run();
			}
		}
#endif
    /* Check if a '#' has been received */
		if (usart_sharp_received()) {
			serial_monitor_init(SERIAL_INTERFACE_USART);
            /* SERIAL on UART loop */
			while (1) {
				serial_monitor_run();
			}
		}
	}
}

/* Test CRC*/
static void CRC16_InitHeader() 
{
	CRC16_High = 0x00;
	CRC16_Low = 0x00;
}

static void updateCRC(unsigned char c)
{
	uint16_t tmp, short_c;
	unsigned short CRC_LOCAL;
	CRC_LOCAL = CRC16_High * 256 + CRC16_Low;
	short_c = /*0xff00 & */(((uint16_t)c) << 8);
	tmp = CRC_LOCAL ^ short_c;
	CRC_LOCAL = tmp << 8;
	CRC_LOCAL = (CRC_LOCAL) ^ crc16_lookup_table[(tmp >> 8)/* & 0xff*/];
	CRC16_High = (CRC_LOCAL & 0xFF00) >> 8;
	CRC16_Low = CRC_LOCAL & 0x00FF;
}


#ifdef CONF_USBCDC_INTERFACE_SUPPORT
#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
}

void main_remotewakeup_lpm_disable(void)
{
}

void main_remotewakeup_lpm_enable(void)
{
}

#endif

bool main_cdc_enable(uint8_t port)
{
	(void)port;
	main_b_cdc_enable = true;
	return true;
}

void main_cdc_disable(uint8_t port)
{
	(void)port;
	main_b_cdc_enable = false;
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	(void)port;
	main_b_dtr_enable = b_enable;
}

void main_cdc_rx_notify(uint8_t port)
{
	(void)port;
}

void main_cdc_set_coding(uint8_t port, usb_cdc_line_coding_t *cfg)
{
	(void)port;
	(void)cfg;
}

#endif
