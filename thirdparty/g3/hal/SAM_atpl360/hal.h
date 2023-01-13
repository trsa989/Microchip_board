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

#ifndef HAL_H
#define HAL_H

#ifndef _G3_SIM_

#if (!PIC32CX)
#define RESET_WATCHDOG                WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
#define RSTC_RSTC_SR                  RSTC->RSTC_SR
#else
#define RESET_WATCHDOG                DWDT->WDT0_CR = WDT0_CR_KEY_PASSWD | WDT0_CR_WDRSTT; DWDT->WDT1_CR = WDT1_CR_KEY_PASSWD | WDT1_CR_WDRSTT;
#define RSTC_RSTC_SR                  RSTC->RSTC_SR
/* Internal flash page size. */
#define IFLASH_PAGE_SIZE             IFLASH0_PAGE_SIZE
/* User Signature page size. */
#define USER_SIG_PAGE_SIZE           (128) /* 128 32-bit words = 512 bytes */
#endif

#else

/* Empty definitions for simulation. */
#define RESET_WATCHDOG

#define RSTC_RSTC_SR                 0
#define RSTC_SR_RSTTYP_Msk           0
#define RSTC_SR_RSTTYP_GeneralReset  0

#define IFLASH_PAGE_SIZE             512

#define Disable_global_interrupt()
#define Enable_global_interrupt()

#define flash_read_user_signature(p_data, ul_size)
#define flash_write_user_signature(p_buffer, ul_size)
#define flash_erase_user_signature()

#endif

#include <compiler.h>

/* \name User Signature configuration parameters */
/* @{ */
#define USER_SIGNATURE_SIZE                (IFLASH_PAGE_SIZE / (sizeof(uint32_t)))
#define MACCFG_OFFSET_USER_SIGN            0
#define PHYCFG_OFFSET_USER_SIGN            16
#define G3CFG_OFFSET_USER_SIGN             32
/* @} */

#endif /* #ifndef HAL_H */

/**********************************************************************************************************************/

/** @}
 **********************************************************************************************************************/
