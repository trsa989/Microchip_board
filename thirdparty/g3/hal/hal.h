/**********************************************************************************************************************/

/** \addtogroup Platform
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/

/** This file contains data types and functions needed to interface with the system.
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/

#ifndef PLATFORM_DISPATCH_H
#define PLATFORM_DISPATCH_H

#include <stdint.h>
#include <stdbool.h>

#include "board.h"

/*#define PLATFORM_DEBUG_ENABLE */

#ifdef _G3_SIM_
	#include <hal/Simulator/hal.h>
#else
#if BOARD == ATPL360AMB || BOARD == ATPL360ASB || BOARD == ATPL360MB || BOARD == PL360G55CB_EK || BOARD == PL360G55CF_EK || BOARD == SAME70_XPLAINED || \
		BOARD == PL360BN || BOARD == PL485_VB || BOARD == PL485_EK || BOARD == PIC32CXMTSH_DB || BOARD == PIC32CXMTG_EK || BOARD == PL460_VB || BOARD == SAMG55_XPLAINED_PRO || BOARD == SAM4CMS_DB || \
		BOARD == PL480_VB || BOARD == PL480_EK
	#include <hal/SAM_atpl360/hal.h>
#else
	#include <hal/SAM/hal.h>
#endif
#endif

void platform_init_hw(void);
void platform_random_init(void);
uint16_t platform_random_16(void);
uint32_t platform_random_32(void);
void platform_init_storage(void);
bool platform_write_storage(uint32_t u32Length, const void *pData);
bool platform_read_storage(uint32_t u32Length, void *pData);
void platform_erase_storage(uint32_t u32Length);
void platform_init_eui64(uint8_t *eui64);
uint32_t platform_read_chip_id(void *chip_id_buf);
void platform_read_user_signature(void *user_sign_buf);
void platform_write_user_signature(void *user_sign_buf);
void platform_erase_user_signature(void);
void platform_init_power_down_det(void);
void platform_set_pdd_callback(void (*pf_user_callback)(void));
void platform_init_1ms_timer(void);
void platform_set_ms_callback(void (*pf_user_callback)(void));
void platform_led_cfg_blink_rate(uint32_t ul_blink_rate_ms);
void platform_led_update(void);
void platform_led_int_toggle(void);

void platform_init_reset_det(void);
void platform_set_reset_callback(void (*pf_user_callback)(void));

#if BOARD == ATPL360AMB || BOARD == ATPL360ASB || BOARD == ATPL360MB || BOARD == PL360G55CB_EK || BOARD == PL360G55CF_EK || BOARD == SAME70_XPLAINED || \
		BOARD == PL360BN || BOARD == PL485_VB || BOARD == PL485_EK || BOARD == PIC32CXMTSH_DB || BOARD == PIC32CXMTG_EK || BOARD == PL460_VB || BOARD == SAMG55_XPLAINED_PRO || BOARD == SAM4CMS_DB || \
		BOARD == PL480_VB || BOARD == PL480_EK
void platform_led_int_on(void);
void platform_led_int_off(void);
#endif

void platform_cfg_call_process_rate(uint32_t ul_user_call_process_rate);
void platform_cfg_call_app_process_rate(uint32_t ul_user_call_app_process_rate);
uint8_t platform_flag_call_process(void);
uint8_t platform_flag_call_app_process(void);

#endif /* #ifndef PLATFORM_DISPATCH_H */

/**********************************************************************************************************************/

/** @}
 **********************************************************************************************************************/
