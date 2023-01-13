/**
 * \file
 *
 * \brief PHY_MNF_TOOL : ATMEL PLC Phy Fuses Validation Example
 *
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
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

/**
 *  \mainpage ATMEL PLC Phy Fuses Validation Example
 *
 *  \section Purpose
 *
 *  The Phy Fuses Validation Tool example demonstrates how to use the PRIME PHY layer
 * on PLC boards in Fuses Validation mode.
 *
 *  \section Requirements
 *
 *  This package should be used with any PLC board on which there is PLC
 * hardware dedicated.
 *
 *  \section Description
 *
 *  This application will configure the PRIME PHY and its serial interface to
 * communicate with ATMEL PLC Phy Tester Tool and test PLC
 * transmission/reception processes.
 *
 *  \section Usage
 *
 *  The tool is ready for set up the device configuration and perform some
 * communications test.
 *
 */

/* Atmel boards includes. */
#include "board.h"

/* Atmel library includes. */
#include "asf.h"

#include "conf_project.h"
#include "conf_pplc_if.h"

/* ATPL360 component */
static atpl360_descriptor_t sx_atpl360_desc;
/* Hardware abstraction layer */
static atpl360_hal_wrapper_t sx_atpl360_hal_wrp;

/* Define Time to led swapping */
#define COUNT_MS_SWAP_LED                          500
static uint32_t ul_count_ms = COUNT_MS_SWAP_LED;

#define ID_TC_1MS               ID_TC3
#define TC_1MS                  TC1
#define TC_1MS_CHN              0
#define TC_1MS_IRQn             TC3_IRQn
#define TC_1MS_Handler          TC3_Handler

/* Define Presentation Message */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- ATMEL PLC Phy Fuses Tester Tool Application --\r\n" \
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/* Fuses Validation Tests buffers */
static uint8_t spuc_buf_wr[ATPL360_BUFF_LEN];
static uint8_t spuc_buf_rd[ATPL360_BUFF_LEN];

#define MENU_HEADER "\n\r--- BOOT Commands ------------------------\n\r" \
	"1: Reset Device\n\r" \
	"2: Write CONTROL_FUSES\n\r" \
	"3: Read CONTROL_FUSES\n\r" \
	"4: Write ENC_KEY\n\r" \
	"5: Read ENC_KEY\n\r" \
	"6: Write TAG_KEY\n\r" \
	"7: Read TAG_KEY\n\r" \
	"otherwise: Display this main menu\n\n\r"

#define MENU_CONSOLE "\n\rFUSES-Console>"

/* Function declarations */
void __user_low_level_init(void);

/**
 * \brief Low level initialization called from Reset_Handler, before calling
 * to main and before data initialization. This function should not use
 * initialized data.
 */
void __user_low_level_init(void)
{
#if BOARD == PIC32CXMTSH_DB
	/* Enable coprocessor peripherals to allow access to PIOD */
	uint32_t read_reg;

	/* Assert coprocessor reset and reset its peripheral */
	read_reg = RSTC->RSTC_MR;
	read_reg &= ~(RSTC_MR_KEY_Msk | RSTC_MR_CPEREN | RSTC_MR_CPROCEN);
	read_reg |= RSTC_MR_KEY_PASSWD;
	RSTC->RSTC_MR = read_reg;

	/* Enables Coprocessor Bus Master Clock */
	PMC->PMC_SCER = PMC_SCER_CPBMCK | PMC_SCER_CPKEY_PASSWD;

	/* Set coprocessor clock prescaler */
	read_reg = PMC->PMC_CPU_CKR;
	read_reg &= ~PMC_CPU_CKR_CPPRES_Msk;
	PMC->PMC_CPU_CKR = read_reg | PMC_CPU_CKR_CPPRES(1);

	/*  Wait to PMC_SR_CPMCKRDY = 1 */
	while ((PMC->PMC_SR & PMC_SR_CPMCKRDY) == 0) {
	}

	/* Release coprocessor peripheral reset */
	RSTC->RSTC_MR |= (RSTC_MR_KEY_PASSWD | RSTC_MR_CPEREN);

	/* Set coprocessor clock prescaler */
	read_reg = PMC->PMC_CPU_CKR;
	read_reg &= ~PMC_CPU_CKR_CPPRES_Msk;
	PMC->PMC_CPU_CKR = read_reg | PMC_CPU_CKR_CPPRES(0);

	/*  Wait to PMC_SR_CPMCKRDY = 1 */
	while ((PMC->PMC_SR & PMC_SR_CPMCKRDY) == 0) {
	}
#endif

	/* Critical initialization of PLC pins */
	pplc_if_crit_init();
}

static void _reset_device(void)
{
	ioport_set_pin_dir(ATPL360_RESET_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	delay_ms(10);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_INACTIVE_LEVEL);
	delay_ms(10);

	atpl360_mnf_init(&sx_atpl360_hal_wrp);
	atpl360_mnf_boot_window();

	printf("\r\nReset device and set boot command mode... \r\n");
	/* Enable secure mode */
	atpl360_mnf_read_ctrl_fuses();
}

static bool _get32(uint32_t *pul_data)
{
	uint32_t ul_32;
	uint8_t puc_value[8];
	uint8_t uc_char;
	uint8_t uc_idx;

	printf("0x");
	fflush(stdout);

	for (uc_idx = 0; uc_idx < sizeof(puc_value);) {
		while (uart_read(CONF_UART, &uc_char)) {
		}

		if ((uc_char == 'x') || (uc_char == 'X')) {
			/* Restart */
			uc_idx = 0;
			printf("  ----   Cancel and enter again...\r\n");
			printf("0x");
		} else if ((uc_char >= 0x30) && (uc_char <= 0x39)) {
			printf("%c", uc_char);
			puc_value[uc_idx++] = uc_char - 0x30;
		} else if ((uc_char >= 0x41) && (uc_char <= 0x46)) {
			printf("%c", uc_char);
			puc_value[uc_idx++] = uc_char - 55;
		} else if ((uc_char >= 0x61) && (uc_char <= 0x66)) {
			printf("%c", uc_char);
			puc_value[uc_idx++] = uc_char - 87;
		} else if (uc_char == 0x1B) {
			/* ESC character */
			return false;
		}

		fflush(stdout);
	}

	/* Catch \r */
	while (uc_char != '\r') {
		uart_read(CONF_UART, &uc_char);
	}
	printf("\r\n");
	fflush(stdout);

	ul_32 = (uint32_t)puc_value[0] << 28;
	ul_32 += (uint32_t)puc_value[1] << 24;
	ul_32 += (uint32_t)puc_value[2] << 20;
	ul_32 += (uint32_t)puc_value[3] << 16;
	ul_32 += (uint32_t)puc_value[4] << 12;
	ul_32 += (uint32_t)puc_value[5] << 8;
	ul_32 += (uint32_t)puc_value[6] << 4;
	ul_32 += (uint32_t)puc_value[7];

	*pul_data = ul_32;

	return true;
}

static bool _get_data_buff(uint8_t *puc_data_buf, uint16_t us_data_len)
{
	uint16_t us_idx;
	uint8_t uc_char;

	printf("0x");
	fflush(stdout);

	if (us_data_len > ATPL360_BUFF_LEN) {
		us_data_len = ATPL360_BUFF_LEN;
	}

	for (us_idx = 0; us_idx < us_data_len; us_idx++) {
		while (uart_read(CONF_UART, &uc_char)) {
		}
		if ((uc_char >= 0x30) && (uc_char <= 0x39)) {
			printf("%c", uc_char);
			uc_char -= 0x30;
		} else if ((uc_char >= 0x41) && (uc_char <= 0x46)) {
			printf("%c", uc_char);
			uc_char -= 55;
		} else if ((uc_char >= 0x61) && (uc_char <= 0x66)) {
			printf("%c", uc_char);
			uc_char -= 87;
		} else if (uc_char == 0x1B) {
			/* ESC character */
			return false;
		} else {
			us_idx--;
			continue;
		}

		fflush(stdout);
		if (us_idx & 0x01) {
			*puc_data_buf++ += uc_char;
		} else {
			*puc_data_buf = uc_char << 4;
		}
	}

	/* Catch \r */
	while (uc_char != '\r') {
		uart_read(CONF_UART, &uc_char);
	}
	printf("\r\n");
	fflush(stdout);

	return true;
}

static void _show_buffer(uint8_t *puc_buff, uint16_t us_len)
{
	uint8_t *puc_tst_ptr;
	uint16_t us_idx;

	printf("0x");
	puc_tst_ptr = puc_buff;
	for (us_idx = 0; us_idx < us_len; us_idx++) {
		printf("%02x", *puc_tst_ptr++);
	}
	printf("\r\n");
	fflush(stdout);
}

/** @brief	Interrupt handler for Timer 3
 *
 * Handler for Timer 3 */
void TC_1MS_Handler(void)
{
	/* Clear status bit to acknowledge interrupt */
	tc_get_status(TC_1MS, TC_1MS_CHN);

	/* update count ms */
	if (!ul_count_ms--) {
		ul_count_ms = COUNT_MS_SWAP_LED;
		LED_Toggle(LED0);
	}
}

/** @brief	Init Timer interrupt (1ms)
 *
 * Initialize 1mSec timer 3 interrupt */
static void initTimer1ms(void)
{
	uint32_t ul_div, ul_tcclks;
	uint32_t ul_sysclk = 0;

	/* Configure PMC */
	pmc_enable_periph_clk(ID_TC_1MS);

	ul_tcclks = 2;

#if (!PIC32CX)
	ul_sysclk = sysclk_get_peripheral_hz();
	/* MCK = ul_sysclk -> tcclks = 2 : TCLK3 = MCK/32 Hz -> ul_div = (MCK/32)/1000 to get 1ms timer */
	ul_div = ul_sysclk / 32000;
	tc_init(TC_1MS, TC_1MS_CHN, ul_tcclks | TC_CMR_CPCTRG);
#else
	ul_sysclk = sysclk_get_cpu_hz();
	/* MCK = ul_sysclk -> tcclks = 2 : TCLK3 = MCK/32 Hz -> ul_div = (MCK/32)/1000 to get 1ms timer */
	ul_div = ul_sysclk / 32000;
	tc_init(TC_1MS, TC_1MS_CHN, ul_tcclks | TC_CMR_CPCTRG, 0);
#endif

	tc_write_rc(TC_1MS, TC_1MS_CHN, ul_div);

	/* Configure and enable interrupt on RC compare */
	NVIC_SetPriority((IRQn_Type)ID_TC_1MS, 0);
	NVIC_EnableIRQ((IRQn_Type)ID_TC_1MS);
	tc_enable_interrupt(TC_1MS, TC_1MS_CHN, TC_IER_CPCS);

	/** Start the timer. TC1, chanel 0 = TC3 */
	tc_start(TC_1MS, TC_1MS_CHN);
}

/**
 *  Configure UART console.
 */
static void configure_dbg_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONF_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Configure the hardware.
 */
static void prvSetupHardware(void)
{
	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();
}

static void _menu_show(void)
{
	puts(MENU_HEADER);
	fflush(stdout);
}

static void _console_show(void)
{
	printf(MENU_CONSOLE);
	fflush(stdout);
}

static void _fuses_info_show(void)
{
	printf("\r\n--- FUSES bitfield information -----------\r\n");
	printf("Enable secure mode:                    \t0x%08x\r\n", ATPL360_FUSES_ENCRNOTPLAIN);
	printf("Disable AES key fuses reading          \t0x%08x\r\n", ATPL360_FUSES_READ_AES_KEY);
	printf("Disable AES key fuses writing          \t0x%08x\r\n", ATPL360_FUSES_WRITE_AES_KEY);
	printf("Disable fuses control reading          \t0x%08x\r\n", ATPL360_FUSES_READ_CONTROL);
	printf("Disable fuses control writing          \t0x%08x\r\n", ATPL360_FUSES_WRITE_CONTROL);
	printf("Disable ram reading                    \t0x%08x\r\n", ATPL360_FUSES_READ_RAM);
	printf("Disable bootloader in master mode      \t0x%08x\r\n", ATPL360_FUSES_KEY_MCHP_RDY);
	printf("Force IV + NB in Signature             \t0x%08x\r\n", ATPL360_FUSES_SIG_IV_NB);
	printf("Disable Debug access                   \t0x%08x\r\n", ATPL360_FUSES_DISABLE_DBG);
	printf("Debug access depending on MSSC register\t0x%08x\r\n", ATPL360_FUSES_DBG_MSSC);
	printf("----------------------------------------------\r\n");

	fflush(stdout);
}

/**
 * \brief Main code entry point.
 */
int main( void )
{
#ifdef CONF_BOARD_LCD_EN
	status_code_t status;
#endif
	uint8_t *puc_buf_wr;
	uint8_t *puc_buf_rd;
	uint32_t ul_value;
	uint16_t us_data_len;
	uint8_t uc_choice;

	ul_count_ms = COUNT_MS_SWAP_LED;
	puc_buf_wr = spuc_buf_wr;
	puc_buf_rd = spuc_buf_rd;

	/* Prepare the hardware */
	prvSetupHardware();

	/* UART debug */
	configure_dbg_console();
	puts(STRING_HEADER);

#ifdef CONF_BOARD_LCD_EN
#if BOARD == ATPL360MB
	status = (status_code_t)c0216CiZ_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
	c0216CiZ_show((const char *)"ATPL360MB MNF");
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	c0216CiZ_show((const char *)"Phy fuses tool");
#else
#error ERROR in board definition
#endif
#endif

	/* Init process timers */
	initTimer1ms();

	/* Init ATPL360 */
	sx_atpl360_hal_wrp.plc_init = pplc_if_init;
	sx_atpl360_hal_wrp.plc_reset = pplc_if_reset;
	sx_atpl360_hal_wrp.plc_set_stby_mode = pplc_if_set_stby_mode;
	sx_atpl360_hal_wrp.plc_set_handler = pplc_if_set_handler;
	sx_atpl360_hal_wrp.plc_send_boot_cmd = pplc_if_send_boot_cmd;
	sx_atpl360_hal_wrp.plc_write_read_cmd = pplc_if_send_wrrd_cmd;
	sx_atpl360_hal_wrp.plc_enable_int = pplc_if_enable_interrupt;
	sx_atpl360_hal_wrp.plc_delay = pplc_if_delay;
	sx_atpl360_hal_wrp.plc_get_thw = pplc_if_get_thermal_warning;
	atpl360_init(&sx_atpl360_desc, &sx_atpl360_hal_wrp);

	/* Init Fuses Validation Mode */
	atpl360_mnf_init(&sx_atpl360_hal_wrp);

	/* Stop boot window */
	atpl360_mnf_boot_window();

	/* Show menu */
	_menu_show();
	_console_show();

	while (1) {
		/* Check inputs */
		if (!uart_read(CONF_UART, &uc_choice)) {
			switch (uc_choice) {
			case '1':
				_reset_device();
				break;

			case '2':
				/* Write CONTROL_FUSES */
				_fuses_info_show();
				printf("\r\nEnter FUSES mask : ");
				if (_get32(&ul_value)) {
					uint32_t ul_val_chk;

					atpl360_mnf_write_ctrl_fuses(ul_value);
					/* Check result */
					ul_val_chk = atpl360_mnf_read_ctrl_fuses();
					if (ul_value != ul_val_chk) {
						printf("\r\nERROR in writing CONTROL_FUSES\r\n");
					}
				} else {
					printf("\r\n--- [ESC] Cancelled operation ---\r\n");
				}

				break;

			case '3':
				/* Read CONTROL_FUSES */
				ul_value = atpl360_mnf_read_ctrl_fuses();
				printf("\r\nRead FUSES mask : 0x%08x\r\n", (uint16_t)ul_value);
				break;

			case '4':
				/* Write ENC_KEY */
				printf("\r\nEnter ENC_KEY Value (32 Hexadecimal Characters): ");
				us_data_len = 16;
				if (_get_data_buff(puc_buf_wr, us_data_len << 1)) {
					atpl360_mnf_write_enc_fuses(puc_buf_wr);
					/* Check result */
					atpl360_mnf_read_enc_fuses(spuc_buf_rd);
					if (memcmp(puc_buf_wr, spuc_buf_rd, 16)) {
						printf("\r\nERROR in writing ENC_KEY\r\n");
						printf("\r\nWrite value:\t");
						_show_buffer(puc_buf_wr, us_data_len);
						printf("\r\nRead value:\t");
						_show_buffer(puc_buf_rd, us_data_len);
					}
				} else {
					printf("\r\n--- [ESC] Cancelled operation ---\r\n");
				}

				break;

			case '5':
				/* Read ENC_KEY */
				printf("\r\nRead ENC_KEY Value: \r\n");
				us_data_len = 16;
				puc_buf_rd = spuc_buf_rd;
				atpl360_mnf_read_enc_fuses(puc_buf_rd);
				printf("Data read:\t");
				_show_buffer(puc_buf_rd, us_data_len);
				break;

			case '6':
				/* Write TAG_KEY */
				printf("\r\nEnter TAG_KEY Value (32 Hexadecimal Characters): ");
				us_data_len = 16;
				if (_get_data_buff(puc_buf_wr, us_data_len << 1)) {
					atpl360_mnf_write_tag_fuses(puc_buf_wr);
					/* Check result */
					atpl360_mnf_read_tag_fuses(spuc_buf_rd);
					if (memcmp(puc_buf_wr, spuc_buf_rd, us_data_len)) {
						printf("\r\nERROR in writing TAG_KEY\r\n");
						printf("\r\nWrite value:\t");
						_show_buffer(puc_buf_wr, us_data_len);
						printf("\r\nRead value:\t");
						_show_buffer(puc_buf_rd, us_data_len);
					}
				} else {
					printf("\r\n--- [ESC] Cancelled operation ---\r\n");
				}

				break;

			case '7':
				/* Read TAG_KEY */
				printf("\r\nRead TAG_KEY Value: \r\n");
				us_data_len = 16;
				puc_buf_rd = spuc_buf_rd;
				atpl360_mnf_read_tag_fuses(puc_buf_rd);
				printf("Data read:\t");
				_show_buffer(puc_buf_rd, us_data_len);
				break;

			default:
				/* Show menu */
				_menu_show();
				break;
			}

			_console_show();
			fflush(stdout);
		}
	}
}
