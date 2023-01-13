#include <stdbool.h>
#include <stdint.h>
#include <hal/hal.h>
#include <string.h>
#if !(SAM4C || SAM4CP || SAM4CM || SAME70)
#include <stdlib.h>
#endif

/* #define LOG_PLATFORM(a)   printf a */
#define LOG_PLATFORM(a)   (void)0

#if SAM
#include "asf.h"
#endif /* SAM */

#include "conf_board.h"
#include "conf_hal.h"
#include "conf_project.h"

/* Variable definition for microcontroller mode */
uint32_t ul_led_count_ms_cfg = 0;
uint32_t ul_led_count_ms;
uint8_t uc_led_swap = 0;
volatile uint8_t uc_flag_call_process = 0;
volatile uint8_t uc_flag_call_app_process = 0;
uint32_t ul_call_process_count_ms_cfg = 0;
uint32_t ul_call_process_count_ms = 0;
uint32_t ul_call_app_process_count_ms_cfg = 0;
uint32_t ul_call_app_process_count_ms = 0;
void (*pf_platform_ms_timer_callback)(void) = NULL;
void (*pf_pdd_callback)(void) = NULL;
void platform_pdd_handler(void);

/* IPv6 stack includes */
#ifdef OSS_ENABLE_IPv6_STACK_SUPPORT
#include "core/net.h"
#endif

#ifdef PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER
/** ADC sample data */
uint16_t us_adc_value;
/** Size of the power down detector (PDD) power supply ADC measurement buffer */
#define PDD_BUFFER_SIZE     1

/** PDD's Tracking Time*/
#define PDD_TRACKING_TIME         1
/** PDD's Transfer Period */
#define PDD_TRANSFER_PERIOD       1

/** Reference voltage for ADC, in mv. */
#define VOLT_REF        (3300)
#define MAX_DIGITAL     (1023)
#endif /* #ifdef PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER */

#if defined(CONF_STORAGE_INTERNAL_FLASH)
/* Address points to last Lock Region in Flash */
#if BOARD == ATPL360AMB
/* Reserve memory to store ATPL360 binary file */
	#define ADDR_APP_DATA (0x010F0000 - IFLASH_LOCK_REGION_SIZE)
#elif defined(_SAM4C16C_0_) || defined(_SAM4CP16C_0_) || defined(_SAM4CMS16C_0_) || defined(_SAM4CMP16C_0_)
	#define ADDR_APP_DATA (IFLASH_CNC_ADDR + IFLASH_SIZE - IFLASH_LOCK_REGION_SIZE)
#elif defined(_SAM4SD32C_)
	#define ADDR_APP_DATA (IFLASH0_ADDR + IFLASH0_SIZE - IFLASH0_LOCK_REGION_SIZE)
#elif defined(_SAM4E16E_) || defined(_SAMG55J19_) || defined(_SAME70Q21_)
	#define ADDR_APP_DATA (IFLASH_ADDR + IFLASH_SIZE - IFLASH_LOCK_REGION_SIZE)
#endif
#endif

#if defined(CONF_STORAGE_AT45DBX_FLASH)
	#include "at45db32.h"
	#define EXT_ADDR_APP_DATA_PAGE 0x007FFC00
#endif

/* Reserve 256 bytes for permanent data, such as EUI-64 */
#define PERMANENT_DATA_RESERVED_SIZE   256

#if BOARD == ATPL360AMB
/* Reserve memory to store ATPL360 binary file */
	#define ADDR_PERMANENT_DATA (0x010F0000 - PERMANENT_DATA_RESERVED_SIZE)
#elif defined(_SAM4C16C_0_) || defined(_SAM4CP16C_0_) || defined(_SAM4CMS16C_0_) || defined(_SAM4CMP16C_0_)
	#define ADDR_PERMANENT_DATA (IFLASH_CNC_ADDR + IFLASH_SIZE - PERMANENT_DATA_RESERVED_SIZE)
/* 0x01000000 + 0x100000 - 0x100 = 0x010FFF00 for _SAM4C16C_0_ */
#elif defined(_SAM4SD32C_)
	#define ADDR_PERMANENT_DATA (IFLASH0_ADDR + IFLASH0_SIZE - PERMANENT_DATA_RESERVED_SIZE)
#elif defined(_SAM4E16E_) || defined(_SAMG55J19_) || defined(_SAME70Q21_)
	#define ADDR_PERMANENT_DATA (IFLASH_ADDR + IFLASH_SIZE - PERMANENT_DATA_RESERVED_SIZE)
#endif

#define CHIP_ID_SIZE 4  /* 4 uint32 words (128 bits) */

#define MAC_CONFIG_KEY_0      0xAA
#define MAC_CONFIG_KEY_1      0x55
#define MAC_CONFIG_KEY_SIZE      2
#define EUI64_SIZE               8

#if (SAMV71 || SAMV70 || SAME70)
#define PLATFORM_PDD_PRIO          1
#define PLATFORM_PDD_VDD_MAX   0xFFF
#endif

#if (SAME70) && defined(CONF_BOARD_SDRAMC)
/* Configuration for on-board external SDRAM */
const sdramc_memory_dev_t SDRAM_IS42S16400J_7B2LI = {
	20,
	SDRAMC_MR_MODE_NORMAL,
	{
		SDRAMC_CR_NC_COL8      |  /* 8 column bits */
		SDRAMC_CR_NR_ROW12     |  /* 12 row bits (4K) */
		SDRAMC_CR_NB_BANK4     |  /* 4 banks */
		SDRAMC_CR_CAS_LATENCY3 |  /* CAS Latency 3 */
		SDRAMC_CR_DBW          |  /* 16 bit */
		SDRAMC_CR_TWR(4)       |  /* 4 SDCK cycles minimum */
		SDRAMC_CR_TRC_TRFC(10) |  /* Command period (Ref to Ref / ACT to ACT) 63ns minimum. If SDCK=143MHz minimum TRFC=10 */
		SDRAMC_CR_TRP(3)       |  /* Command period (PRE to ACT) 15 ns min. If SDCK=143MHz minimum TRP=3 */
		SDRAMC_CR_TRCD(3)      |  /* Active Command to read/Write Command delay 15ns. If SDCK=143MHz minimum TRCD=3 */
		SDRAMC_CR_TRAS(7)      |  /* Command period (ACT to PRE)  42ns min. If SDCK=143MHz minimum TRCD=7 */
		SDRAMC_CR_TXSR(11)        /* Exit self-refresh to active time  70ns Min. If SDCK=143MHz minimum TRCD=11 */
	},
};
#endif

/* The mem page is 512 bytes. R&W the whole page ensure previous data of the page is not deleted */
union memory_union {
	uint32_t u32StartUpCounter; /* The first 4 bytes store the startup counter */
	uint8_t u8MemPage[IFLASH_PAGE_SIZE];
};
union memory_union mem;

#if defined(CONF_EUI64_FROM_USER_SIGNATURE) || defined(CONF_STORAGE_USER_SIGNATURE)
/* Buufer to handle User Signature */
static uint8_t auc_user_sig_container[IFLASH_PAGE_SIZE];
#endif

#if defined(CONF_EUI64_FROM_USER_SIGNATURE)
/* Intermediate buffer for EUI64 validation on User Signature */
static uint8_t uc_eui64_buf[MAC_CONFIG_KEY_SIZE + EUI64_SIZE];
#endif

/**********************************************************************************************************************/
/************************************************ --------------------- ***********************************************/
/************************************************ RANDOM NUM GENERATION ***********************************************/
/************************************************ --------------------- ***********************************************/
/**********************************************************************************************************************/

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void platform_random_init(void)
{
#if !(SAM4C || SAM4CP || SAM4CM || SAME70)
	uint32_t ul_random_num;
#endif

	platform_init_storage();

	platform_read_storage(sizeof(union memory_union), &mem);
	mem.u32StartUpCounter++;

	LOG_PLATFORM(("Start-up counter: %lu\n", mem.u32StartUpCounter));

#if (SAM4C || SAM4CP || SAM4CM || SAME70)
	/* Configure PMC */
	pmc_enable_periph_clk(ID_TRNG);
	/* Enable TRNG */
	trng_enable(TRNG);
#else
	ul_random_num = DWT->CYCCNT;

	ul_random_num = 36969 * (ul_random_num & 65535) + (ul_random_num >> 16);
	mem.u32StartUpCounter = 18000 * (mem.u32StartUpCounter & 65535) + (mem.u32StartUpCounter >> 16);

	ul_random_num = (ul_random_num << 16) + mem.u32StartUpCounter;

	/* Initialize seed */
	srand(ul_random_num);

	LOG_PLATFORM(("RANDOM SEED: %d\r\n", ul_random_num));
#endif
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
uint16_t platform_random_16(void)
{
	uint16_t us_random_num;

#if (SAM4C || SAM4CP || SAM4CM || SAME70)
	while ((trng_get_interrupt_status(TRNG) & TRNG_ISR_DATRDY) != TRNG_ISR_DATRDY) {
	}
	us_random_num = (uint16_t)(trng_read_output_data(TRNG));
#else
	us_random_num = rand() & 0xFF;
	us_random_num = (us_random_num << 8) | (rand() & 0xFF);
#endif
	return us_random_num;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
uint32_t platform_random_32(void)
{
	uint32_t ul_random_num;

#if (SAM4C || SAM4CP || SAM4CM || SAME70)
	while ((trng_get_interrupt_status(TRNG) & TRNG_ISR_DATRDY) != TRNG_ISR_DATRDY) {
	}
	ul_random_num = trng_read_output_data(TRNG);
#else
	ul_random_num = rand() & 0xFF;
	ul_random_num = (ul_random_num << 8) | (rand() & 0xFF);
	ul_random_num = (ul_random_num << 8) | (rand() & 0xFF);
	ul_random_num = (ul_random_num << 8) | (rand() & 0xFF);
#endif
	return ul_random_num;
}

/**********************************************************************************************************************/
/************************************************ -------------------- ************************************************/
/************************************************   EUI64 AND STORAGE  ************************************************/
/************************************************ -------------------- ************************************************/
/**********************************************************************************************************************/

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void platform_init_storage(void)
{
	#if defined(CONF_STORAGE_AT45DBX_FLASH)
	at45db32_init();
	/* Wait for at45db32 to be ready after initialization */
	at45db32_wait_is_ready();
	#endif
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
bool platform_write_storage(uint32_t u32Length, const void *pData)
{
	bool bRet = false;

	Disable_global_interrupt();

	/* Write to flash */
#ifdef CONF_STORAGE_INTERNAL_FLASH
	/* unlocks the needed space depending on "data_len" */
	flash_unlock((uint32_t)ADDR_APP_DATA, (uint32_t)ADDR_APP_DATA + u32Length, 0, 0);

	/* Writes "data_len" bytes from "data_buffer" to flash */
	bRet = (flash_write((uint32_t)ADDR_APP_DATA, pData, u32Length, 0) == FLASH_RC_OK);
#endif

#ifdef CONF_STORAGE_AT45DBX_FLASH
	/* Wait for at45db32 to be ready */
	at45db32_wait_is_ready();

	/* Write data to mem through buf 1*/
	bRet = at45db32_send_cmd(AT45DBX_PR_PAGE_TH_BUF1, EXT_ADDR_APP_DATA_PAGE, (uint8_t *)pData, (uint16_t)u32Length);
#endif

#ifdef CONF_STORAGE_USER_SIGNATURE
	/* Write to User Signature */
	if (u32Length > IFLASH_PAGE_SIZE - G3CFG_OFFSET_USER_SIGN) {
		u32Length = IFLASH_PAGE_SIZE - G3CFG_OFFSET_USER_SIGN;
	}

	memcpy(&auc_user_sig_container[G3CFG_OFFSET_USER_SIGN], pData, u32Length);
	platform_write_user_signature(&auc_user_sig_container);
	bRet = true;
#endif

#if !defined(CONF_STORAGE_INTERNAL_FLASH) && !defined(CONF_STORAGE_AT45DBX_FLASH) && !defined(CONF_STORAGE_USER_SIGNATURE)
	UNUSED(u32Length);
	UNUSED(pData);
#endif

	Enable_global_interrupt();

	return bRet;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
bool platform_read_storage(uint32_t u32Length, void *pData)
{
	bool bRet = false;
	Disable_global_interrupt();

#ifdef CONF_STORAGE_INTERNAL_FLASH
	/* Read from flash (direct operation): reads "data_len" bytes from flash to "data_buffer" */
	memcpy(pData, (uint8_t *)ADDR_APP_DATA, u32Length);
	bRet = true;
#endif

#ifdef CONF_STORAGE_AT45DBX_FLASH
	/* Wait for at45db32 to be ready */
	at45db32_wait_is_ready();
	/* Read data from memory */
	bRet = at45db32_send_cmd(AT45DBX_RD_PAGE, EXT_ADDR_APP_DATA_PAGE, (uint8_t *)pData, (uint16_t)u32Length);
#endif

#ifdef CONF_STORAGE_USER_SIGNATURE
	/* Read from User Signature */
	platform_read_user_signature(&auc_user_sig_container);
	if (u32Length > IFLASH_PAGE_SIZE - G3CFG_OFFSET_USER_SIGN) {
		u32Length = IFLASH_PAGE_SIZE - G3CFG_OFFSET_USER_SIGN;
	}

	memcpy(pData, &auc_user_sig_container[G3CFG_OFFSET_USER_SIGN], u32Length);
	bRet = true;
#endif

#if !defined(CONF_STORAGE_INTERNAL_FLASH) && !defined(CONF_STORAGE_AT45DBX_FLASH) && !defined(CONF_STORAGE_USER_SIGNATURE)
	UNUSED(u32Length);
	UNUSED(pData);
#endif

	Enable_global_interrupt();

	return bRet;
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void platform_erase_storage(uint32_t u32Length)
{
	Disable_global_interrupt();

#ifdef CONF_STORAGE_INTERNAL_FLASH
	/* unlocks the needed space depending on "data_len" */
	flash_unlock((uint32_t)ADDR_APP_DATA, (uint32_t)ADDR_APP_DATA + u32Length, 0, 0);

	/* Erases the region for the persistent data (the minimum erasable size is 16 pages) */
	flash_erase_page((uint32_t)ADDR_APP_DATA, IFLASH_ERASE_PAGES_16);
#endif

#ifdef CONF_STORAGE_AT45DBX_FLASH
	UNUSED(u32Length);

	/* Wait for at45db32 to be ready */
	at45db32_wait_is_ready();

	/* Erase page */
	at45db32_send_cmd(AT45DBX_ER_PAGE, EXT_ADDR_APP_DATA_PAGE, 0, 0);

	/* Wait for at45db32 to be ready */
	at45db32_wait_is_ready();
#endif

#ifdef CONF_STORAGE_USER_SIGNATURE
	UNUSED(u32Length);

	/* Erase User Signature */
	platform_erase_user_signature();
#endif

#if !defined(CONF_STORAGE_INTERNAL_FLASH) && !defined(CONF_STORAGE_AT45DBX_FLASH) && !defined(CONF_STORAGE_USER_SIGNATURE)
	UNUSED(u32Length);
#endif

	Enable_global_interrupt();
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void platform_init_eui64(uint8_t *eui64)
{
	/* Set default value */
	eui64[0] = 0x88;
	eui64[1] = 0x77;
	eui64[2] = 0x66;
	eui64[3] = 0x55;
	eui64[4] = 0x44;
	eui64[5] = 0x33;
	eui64[6] = 0x22;
	eui64[7] = 0x11;

#ifdef CONF_EUI64_FROM_USER_SIGNATURE
	uint8_t uc_i;

	LOG_PLATFORM(("Look for EUI64 previously configured in User Signature\r\n"));

	/* Read from User Signature */
	platform_read_user_signature(&auc_user_sig_container);
	memcpy(uc_eui64_buf, &auc_user_sig_container[MACCFG_OFFSET_USER_SIGN], sizeof(uc_eui64_buf));
	/* Check eui64 integrity */
	if (((uc_eui64_buf[0] == MAC_CONFIG_KEY_0) && (uc_eui64_buf[1] == MAC_CONFIG_KEY_1)) ||
			((uc_eui64_buf[1] == MAC_CONFIG_KEY_0) && (uc_eui64_buf[0] == MAC_CONFIG_KEY_1))) {
		/* Valid EUI64 address found, overwrite parameter */
		for (uc_i = 0; uc_i < EUI64_SIZE; uc_i++) {
			eui64[EUI64_SIZE - 1 - uc_i] = uc_eui64_buf[MAC_CONFIG_KEY_SIZE + uc_i];
		}
		LOG_PLATFORM(("Overwriting EUI64 with value found in User Signature "));
	} else {
		/* Else, do not overwrite parameter */
		LOG_PLATFORM(("No EUI64 found, using default "));
	}
#endif

#ifdef CONF_EUI64_FROM_CHIP_ID
	uint8_t auc_chip_id_container[16]; /* 128 bits */
	uint32_t res;

	LOG_PLATFORM(("Get EUI64 from Chip ID\r\n"));

	/* Read from Chip ID */
	res = platform_read_chip_id(&auc_chip_id_container);
	if (res == FLASH_RC_OK) {
		eui64[7] = auc_chip_id_container[4];
		eui64[6] = auc_chip_id_container[5];
		eui64[5] = auc_chip_id_container[6];
		eui64[4] = auc_chip_id_container[7];
		eui64[3] = (auc_chip_id_container[8] << 4) | (auc_chip_id_container[9] & 0x0F);
		eui64[2] = (auc_chip_id_container[10] << 4) | (auc_chip_id_container[11] & 0x0F);
		eui64[1] = (auc_chip_id_container[12] << 4) | (auc_chip_id_container[13] & 0x0F);
		eui64[0] = (auc_chip_id_container[14] << 4) | (auc_chip_id_container[15] & 0x0F);
		LOG_PLATFORM(("Overwriting EUI64 with Chip ID "));
	} else {
		/* Else, do not overwrite parameter */
		LOG_PLATFORM(("Could not read Chip ID, using default "));
	}
#endif

	LOG_PLATFORM(("%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
			eui64[7], eui64[6], eui64[5], eui64[4], eui64[3], eui64[2], eui64[1], eui64[0]));
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
uint32_t platform_read_chip_id(void *chip_id_buf)
{
	return flash_read_unique_id((uint32_t *)chip_id_buf, CHIP_ID_SIZE);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void platform_read_user_signature(void *user_sign_buf)
{
	flash_read_user_signature((uint32_t *)user_sign_buf, USER_SIGNATURE_SIZE);
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void platform_erase_user_signature(void)
{
	flash_erase_user_signature();
}

/**********************************************************************************************************************/

/**
 **********************************************************************************************************************/
void platform_write_user_signature(void *user_sign_buf)
{
	flash_write_user_signature((uint32_t *)user_sign_buf, USER_SIGNATURE_SIZE);
}

/**********************************************************************************************************************/
/************************************************ -------------------- ************************************************/
/************************************************ POWER DOWN DETECTION ************************************************/
/************************************************ -------------------- ************************************************/
/**********************************************************************************************************************/

#ifdef PLATFORM_PDD_INTERNAL_SUPPLY_MONITOR

void SUPC_Handler(void)
{
	uint32_t ul_status;

	ul_status = supc_get_status(SUPC);
	UNUSED(ul_status);

	if (pf_pdd_callback != NULL) {
		/* Launch user-defined PDD callback */
		pf_pdd_callback();
	}
}

#endif /* PLATFORM_PDD_INTERNAL_SUPPLY_MONITOR */

#ifdef PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER

#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)

/**
 * \brief Read converted data through PDC channel.
 *
 * \param p_adc The pointer of adc peripheral.
 * \param p_s_buffer The destination buffer.
 * \param ul_size The size of the buffer.
 */
static uint32_t adc_read_buffer(Adc *p_adc, uint16_t *p_s_buffer, uint32_t ul_size)
{
	/* Check if the first PDC bank is free. */
	if ((p_adc->ADC_RCR == 0) && (p_adc->ADC_RNCR == 0)) {
		p_adc->ADC_RPR = (uint32_t)p_s_buffer;
		p_adc->ADC_RCR = ul_size;
		p_adc->ADC_PTCR = ADC_PTCR_RXTEN;

		return 1;
	} else { /* Check if the second PDC bank is free. */
		if (p_adc->ADC_RNCR == 0) {
			p_adc->ADC_RNPR = (uint32_t)p_s_buffer;
			p_adc->ADC_RNCR = ul_size;
			p_adc->ADC_PTCR = ADC_PTCR_RXTEN;

			return 1;
		} else {
			return 0;
		}
	}
}

/**
 * \brief Interrupt handler for the ADC
 *        (Used for power down detection)
 */
#if SAMG55
void ADC_PDD_Handler(void);

void ADC_PDD_Handler(void)
#else
void ADC_Handler(void)
#endif
{
	static bool b_info_saved = true; /* True to avoid saving info in flash when the */
	/* board starts up with USB power supply. */

	/* Read the ADC value with a PDC transfer */
#if SAMG
	if (adc_get_interrupt_status(ADC) & (1 << PLATFORM_PDD_ADC_CHANNEL_POWER_SUPPLY)) {
		us_adc_value = adc_channel_get_value(ADC, PLATFORM_PDD_ADC_CHANNEL_POWER_SUPPLY);
#else
	if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) ==
			ADC_ISR_RXBUFF) {
		adc_read_buffer(ADC, &us_adc_value, PDD_BUFFER_SIZE);
		/* Only keep sample value, and discard channel number. */
		us_adc_value &= ADC_LCDR_LDATA_Msk;
#endif

		LOG_PLATFORM(("%d ", us_adc_value));

		/* Check if Power Supply is under power down threshold */
		if (us_adc_value < PLATFORM_PDD_VDD_CRITICAL_THRESHOLD) {
			if (!b_info_saved) {
				if (pf_pdd_callback != NULL) {
					/* Launch user-defined PDD callback */
					pf_pdd_callback();
				}

				b_info_saved = true;
				LOG_PLATFORM(("Power down detected! Backup in flash done.\n"));
			}
		} else if (b_info_saved) { /* Power Supply level is in normal range */
			/* Configure flag so that info is saved when the next power down is detected */
			b_info_saved = false;
		}
	}
}

/**
 * \brief Configure to trigger ADC by TIOA output of timer.
 */
static void configure_time_trigger(void)
{
	uint32_t ul_div = 1;
	uint32_t ul_tc_clks = 0;
	uint32_t ul_sysclk = sysclk_get_peripheral_hz();
	uint32_t ul_f_obj = PLATFORM_PDD_POWER_SUPPLY_MONITORING_FREQ;

	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_TC0);

	/* TIOA configuration */
#if SAMG
	gpio_configure_pin(PIN_TC_TIOA0, PIN_TC_TIOA0_FLAGS);
#else
	gpio_configure_pin(PIN_TC0_TIOA0, PIN_TC0_TIOA0_FLAGS);
#endif

	/* Configure TC for a ul_f_obj frequency and trigger on RC compare. */
	tc_find_mck_divisor(ul_f_obj, ul_sysclk, &ul_div, &ul_tc_clks, ul_sysclk);
	tc_init(TC0, 0, ul_tc_clks | TC_CMR_CPCTRG | TC_CMR_WAVE |
			TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
	TC0->TC_CHANNEL[0].TC_RA = ((ul_sysclk / ul_div) / ul_f_obj) / 2;
	TC0->TC_CHANNEL[0].TC_RC = ((ul_sysclk / ul_div) / ul_f_obj) / 1;

	/* Start the Timer. */
	tc_start(TC0, 0);

	/* Set TIOA0 trigger. */
#if SAMG
	adc_set_trigger(ADC, ADC_TRIG_TIO_CH_0);
#else
	adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
#endif
}

#elif (SAMV71 || SAMV70 || SAME70)

/**
 * \brief Configure to trigger AFEC by TIOA output of timer.
 */
static void configure_time_trigger(void)
{
	uint32_t ul_div = 1;
	uint32_t ul_tc_clks = 0;
	uint32_t ul_sysclk = sysclk_get_peripheral_hz();
	uint32_t ul_f_obj = PLATFORM_PDD_POWER_SUPPLY_MONITORING_FREQ;

	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_TC0);

	/* TIOA configuration */
	gpio_configure_pin(PIN_TC0_TIOA0, PIN_TC0_TIOA0_FLAGS);

	/* Configure TC for a ul_f_obj frequency and trigger on RC compare. */
	tc_find_mck_divisor(ul_f_obj, ul_sysclk, &ul_div, &ul_tc_clks, ul_sysclk);
	tc_init(TC0, 0, ul_tc_clks | TC_CMR_CPCTRG | TC_CMR_WAVE |
			TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
	TC0->TC_CHANNEL[0].TC_RA = ((ul_sysclk / ul_div) / ul_f_obj) / 2;
	TC0->TC_CHANNEL[0].TC_RC = ((ul_sysclk / ul_div) / ul_f_obj) / 1;

	/* Start the Timer */
	tc_start(TC0, 0);

	/* Set TIOA0 trigger for AFEC */
	afec_set_trigger(AFEC0, AFEC_TRIG_TIO_CH_0);
}

/**
 * \brief Interrupt handler for the ADC
 *        (Used for power down detection)
 */
void platform_pdd_handler(void)
{
	static bool b_info_saved = true; /* True to avoid saving info in flash when the */
	/* board starts up with USB power supply. */

	/* Read the AFEC value */
	AFEC0->AFEC_CSELR = 0;
	us_adc_value = AFEC0->AFEC_CDR;

	LOG_PLATFORM(("%d ", us_adc_value));

	/* Check if Power Supply is under power down threshold */
	if ((us_adc_value < PLATFORM_PDD_VDD_CRITICAL_THRESHOLD)) {
		if (!b_info_saved) {
			if (pf_pdd_callback != NULL) {
				/* Launch user-defined PDD callback */
				pf_pdd_callback();
			}

			b_info_saved = true;
			LOG_PLATFORM(("Power down detected! Backup in flash done.\n"));
		}
	} else if (b_info_saved) { /* Power Supply level is in normal range */
		/* Configure flag so that info is saved when the next power down is detected */
		b_info_saved = false;
	}
}

#endif

#endif /* #ifdef PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER */

#if defined(PLATFORM_PDD_INTERNAL_SUPPLY_MONITOR) || defined(PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER)

/**
 * \brief Initialize HW necessary to detect Power Down.
 */
void platform_init_power_down_det(void)
{
#ifdef PLATFORM_PDD_INTERNAL_SUPPLY_MONITOR
	/* Enable SUPC CLK */
	pmc_enable_periph_clk(ID_SUPC);
	/* Set monitoring rate */
	supc_set_monitor_sampling_period(SUPC, PLATFORM_PDD_MONITORING_RATE);
	/* Set Monitor Threshold */
	supc_set_monitor_threshold(SUPC, PLATFORM_PDD_MONITOR_THRESHOLD);
	/* Configure monitor to raise interrupt */
	supc_enable_monitor_interrupt(SUPC);
	/* Configure and enable SUPC interrupt */
	NVIC_SetPriority((IRQn_Type)ID_SUPC, 0);
	NVIC_EnableIRQ((IRQn_Type)ID_SUPC);
#endif /* PLATFORM_PDD_INTERNAL_SUPPLY_MONITOR */

#ifdef PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER
#if (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM)
	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_ADC);

	/* Initialize ADC. */

	/*
	 * Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
	 * For example, MCK = 64MHZ, PRESCAL = 4, then:
	 * ADCClock = 64 / ((4+1) * 2) = 6.4MHz;
	 */

#if SAMG
	struct adc_config adc_cfg;

	adc_enable();

	/* Get the ADC default configurations: */
	adc_get_config_defaults(&adc_cfg);

	/* Initialize the ADC Module: */
	adc_init(ADC, &adc_cfg);

	/* Set ADC callback for PDD */
	adc_set_callback(ADC, (enum adc_interrupt_source)PLATFORM_PDD_ADC_CHANNEL_POWER_SUPPLY, ADC_PDD_Handler, 10);

	/* Enable Channel: */
	adc_channel_enable(ADC, PLATFORM_PDD_ADC_CHANNEL_POWER_SUPPLY);

#else /* SAMG */

	/* Formula:
	 *     Startup  Time = startup value / ADCClock
	 *     Startup time = 64 / 6.4MHz = 10 us
	 */
	adc_init(ADC, sysclk_get_peripheral_hz(), 6400000, ADC_STARTUP_TIME_4);

	/* Init ADC data value. */
	us_adc_value = 0;

	/* Set ADC timing. */
	adc_configure_timing(ADC, PDD_TRACKING_TIME);

	/* Enable channel number tag. */
	adc_enable_tag(ADC);

	/* Disable sequencer. */
	adc_stop_sequencer(ADC);

	/* Enable channels. */
	adc_enable_channel(ADC, PLATFORM_PDD_ADC_CHANNEL_POWER_SUPPLY);

	/* Disable power save. */
	adc_configure_power_save(ADC, 0);
#endif /* SAMG */

	/* Transfer with PDC. */
	adc_read_buffer(ADC, &us_adc_value, PDD_BUFFER_SIZE);
	/* Enable PDC channel interrupt. */
#if SAMG55
	adc_enable_interrupt(ADC, (enum adc_interrupt_source)PLATFORM_PDD_ADC_CHANNEL_POWER_SUPPLY);
#else
	adc_enable_interrupt(ADC, ADC_IER_RXBUFF);
#endif

	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);

	/* Configure time trigger and start convertion. */
	configure_time_trigger();
#endif /* (SAM4S || SAM4E || SAM4N || SAM4C || SAMG || SAM4CP || SAM4CM) */

#if (SAMV71 || SAMV70 || SAME70)
	pmc_enable_periph_clk(ID_PIOD);

	afec_enable(AFEC0);

	struct afec_config afec_cfg;

	afec_get_config_defaults(&afec_cfg);

	afec_init(AFEC0, &afec_cfg);

	/* Configure AFEC channel to be triggered by the TC */
	configure_time_trigger();

	struct afec_ch_config afec_ch_cfg;

	afec_ch_get_config_defaults(&afec_ch_cfg);

	afec_ch_cfg.gain = AFEC_GAINVALUE_0;

	afec_ch_set_config(AFEC0,
			PLATFORM_PDD_AFEC_CHANNEL_POWER_SUPPLY,
			&afec_ch_cfg);

	afec_channel_set_analog_offset(AFEC0,
			PLATFORM_PDD_AFEC_CHANNEL_POWER_SUPPLY,
			0x200);

	/* Set PDD callback */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0, platform_pdd_handler, PLATFORM_PDD_PRIO);

	/* Enable channel for Vdd */
	afec_channel_enable(AFEC0, PLATFORM_PDD_AFEC_CHANNEL_POWER_SUPPLY);
#endif /* (SAMV71 || SAMV70 || SAME70) */
#endif /* PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER */

	/* Initialize PDD user-defined callback */
	pf_pdd_callback = NULL;
}

/**
 * \brief Set up Power Down Detector callback.
 *
 */
void platform_set_pdd_callback(void (*pf_user_callback)(void))
{
	pf_pdd_callback = pf_user_callback;
}

#endif /* defined(PLATFORM_PDD_INTERNAL_SUPPLY_MONITOR) || defined(PLATFORM_PDD_EXTERNAL_VOLTAGE_DIVIDER) */

/**********************************************************************************************************************/
/************************************************ -------------------- ************************************************/
/************************************************   HW TIMERS CONFIG   ************************************************/
/************************************************ -------------------- ************************************************/
/**********************************************************************************************************************/

/**
 * \internal
 * \brief Timer interrupt handler.
 *
 */
void _platform_tc_1ms_handler(void)
{
	volatile uint32_t ul_dummy;
	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC_1MS, TC_1MS_CHN);
	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Execute user 1ms timer callback, if defined */
	if (pf_platform_ms_timer_callback != NULL) {
		pf_platform_ms_timer_callback();
	}

	if (ul_call_process_count_ms != 0) {
		ul_call_process_count_ms--;
		if (ul_call_process_count_ms == 0) {
			/* Signal call to process */
			uc_flag_call_process = 1;
			ul_call_process_count_ms = ul_call_process_count_ms_cfg;
		}
	}

	if (ul_call_app_process_count_ms != 0) {
		ul_call_app_process_count_ms--;
		if (ul_call_app_process_count_ms == 0) {
			/* Signal call to process */
			uc_flag_call_app_process = 1;
			ul_call_app_process_count_ms = ul_call_app_process_count_ms_cfg;
		}
	}

	if (!ul_led_count_ms--) {
		ul_led_count_ms = ul_led_count_ms_cfg;
		uc_led_swap = 1;
	}
}

/**
 * \brief Set up 1ms timer.
 *
 */
void platform_init_1ms_timer(void)
{
	uint32_t ul_div, ul_tcclks;
	uint32_t ul_sysclk = 0;

	/* Configure PMC */
	pmc_enable_periph_clk(ID_TC_1MS);

	ul_tcclks = 2;
#if (PIC32CX)
	ul_sysclk = sysclk_get_cpu_hz();
#else
	ul_sysclk = sysclk_get_peripheral_hz();
#endif	
	
	/* MCK = ul_sysclk -> tcclks = 2 : TCLK3 = MCK/32 Hz -> ul_div = (MCK/32)/1000 to get 1ms timer */
	ul_div = ul_sysclk / 32000;

	tc_init(TC_1MS, TC_1MS_CHN, ul_tcclks | TC_CMR_CPCTRG);

	tc_write_rc(TC_1MS, TC_1MS_CHN, ul_div);

	/* Configure and enable interrupt on RC compare */
	NVIC_SetPriority((IRQn_Type)ID_TC_1MS, TC_1MS_PRIO);
	NVIC_EnableIRQ((IRQn_Type)ID_TC_1MS);
	tc_enable_interrupt(TC_1MS, TC_1MS_CHN, TC_IER_CPCS);

	/* Start the timer. TC1, chanel 0 = TC3 */
	tc_start(TC_1MS, TC_1MS_CHN);
}

/**
 * \brief Set up LED blinking count (ms).
 *
 */
void platform_set_ms_callback(void (*pf_user_callback)(void))
{
	pf_platform_ms_timer_callback = pf_user_callback;
}

/**
 * \brief Set up LED blinking count (ms).
 *
 */
void platform_led_cfg_blink_rate(uint32_t ul_count_cfg)
{
	ul_led_count_ms_cfg = ul_count_cfg;
}

/**
 * \brief Update LED status according to the ms timer.
 *
 */
void platform_led_update(void)
{
	if (uc_led_swap) {
		uc_led_swap = 0;
#if (BOARD == SAM4CMP_DB || BOARD == SAM4CMS_DB)
		LED_Toggle(LED4);
#else
		LED_Toggle(LED0);
#endif
	}
}

/**
 * \brief Update LED status according to raising interrupt.
 *
 */
void platform_led_int_toggle(void)
{
#if (BOARD == SAM4CMP_DB || BOARD == SAM4CMS_DB)
	/* Do nothing */
#else
	LED_Toggle(LED1);
#endif
}

/**
 * \brief Configure the number of milliseconds to wait before consecutive
 *        calls to the stack process.
 */
void platform_cfg_call_process_rate(uint32_t ul_user_call_process_rate)
{
	ul_call_process_count_ms_cfg = ul_user_call_process_rate;
	ul_call_process_count_ms = ul_user_call_process_rate;
}

/**
 * \brief Configure the number of milliseconds to wait before consecutive
 *        calls to the application process.
 */
void platform_cfg_call_app_process_rate(uint32_t ul_user_call_app_process_rate)
{
	ul_call_app_process_count_ms_cfg = ul_user_call_app_process_rate;
	ul_call_app_process_count_ms = ul_user_call_app_process_rate;
}

/**
 * \brief Update call process flag according to the ms timer.
 *
 */
uint8_t platform_flag_call_process(void)
{
	uint8_t uc_result = 0;

	if (uc_flag_call_process) {
		uc_result = 1;
		uc_flag_call_process = 0;
	}

	return(uc_result);
}

/**
 * \brief Update call process flag according to the ms timer.
 *
 */
uint8_t platform_flag_call_app_process(void)
{
	uint8_t uc_result = 0;

	if (uc_flag_call_app_process) {
		uc_result = 1;
		uc_flag_call_app_process = 0;
	}

	return(uc_result);
}

/**********************************************************************************************************************/
/*********************************************** ---------------------- ***********************************************/
/*********************************************** GENERAL INITIALIZATION ***********************************************/
/*********************************************** ---------------------- ***********************************************/
/**********************************************************************************************************************/

/**
 *  Configure UART console.
 */
/* [main_console_configure] */
static void configure_dbg_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONF_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Configure the hardware.
 */
void platform_init_hw(void)
{
#ifdef PLATFORM_LCD_SIGNALLING_ENABLE
	status_code_t status;
#endif

	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

#if (SAME70) && defined(CONF_BOARD_SDRAMC)
	/* Initialize SDRAM controller */
	sysclk_enable_peripheral_clock(ID_SDRAMC);
	sysclk_enable_peripheral_clock(ID_PIOA);
	sysclk_enable_peripheral_clock(ID_PIOC);
	matrix_set_nandflash_cs(CCFG_SMCNFCS_SDRAMEN);
	sdramc_init((sdramc_memory_dev_t *)&SDRAM_IS42S16400J_7B2LI, sysclk_get_peripheral_hz());
	sdram_enable_unaligned_support();
#endif

#if (BOARD != ATPL360AMB)
#if defined(CONF_BAND_CENELEC_A) || defined(CONF_BAND_CENELEC_B) || defined(CONF_BAND_FCC) || defined(CONF_BAND_ARIB)
	/* Configure pin and reset ATPL250 */
	pplc_if_config_atpl250_reset();
	pplc_if_push_atpl250_reset();
#endif  
#endif

#ifdef PLATFORM_LCD_SIGNALLING_ENABLE
#if (BOARD == ATPL360AMB)
	/* Initialize the vim878 LCD glass component. */
	status = vim878_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	vim878_set_contrast(1);
	vim878_clear_all();
	vim878_show_text((const uint8_t *)PLATFORM_LCD_TEXT);
#else
	/* Initialize the C42364A LCD glass component. */
	status = c42364a_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	c42364a_set_contrast(15);
	c42364a_clear_all();
	c42364a_show_icon(C42364A_ICON_ATMEL);
	c42364a_show_icon(C42364A_ICON_WLESS);
	c42364a_show_text((const uint8_t *)PLATFORM_LCD_TEXT);
#endif
#endif

	/* Configure Console over UART */
	configure_dbg_console();
}
