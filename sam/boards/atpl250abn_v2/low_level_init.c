/**
 * \file
 *
 * \brief ATPL250ABN_v2 board low level init.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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

/* Atmel library includes. */
#include "asf.h"
#include "conf_board.h"

#ifdef CONF_BOARD_CONFIG_MPU_AT_INIT
  #include "mpu.h"
#endif
#ifdef ENABLE_TCM
  #include "efc.h"
#endif   

#if defined(__GNUC__)
/* Not included in IAR, because it is performed via weak link. */
    #include "low_level_init.h"
#endif

#if defined(ENABLE_TCM) && defined(__GNUC__)
	extern char _itcm_lma, _sitcm, _eitcm, _edtcm_stack;
#endif

typedef enum TCM_MemorySize_t {
  TCM_MEMORY_NONE = 0,  /* sram = 384 / 256 */
  TCM_MEMORY_32KB = 1,  /* sram = 320 / 192 */
  TCM_MEMORY_64KB = 2,  /* sram = 256 / 128 */
  TCM_MEMORY_128KB = 3  /* sram = 128 / 0 */    
}TCM_MemorySize;

/** \brief  TCM memory disable

    The function disables TCM memories
 */
__STATIC_INLINE void _TCM_Disable(void) 
{
  __DSB();
  __ISB();
  SCB->ITCMCR &= ~(uint32_t)SCB_ITCMCR_EN_Msk;
  SCB->DTCMCR &= ~(uint32_t)SCB_ITCMCR_EN_Msk;
  __DSB();
  __ISB();
}

/** \brief  TCM memory configuration

    The function configures TCM with the desired size

 * \param memSize The size of the TCM
 */
__STATIC_INLINE void _TCM_Configure(TCM_MemorySize memSize)
{
    switch(memSize) {
    case TCM_MEMORY_NONE:
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(7));
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(8));
      break;
    case TCM_MEMORY_32KB:
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SGPB | EEFC_FCR_FARG(7));
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(8));
      break;    
    case TCM_MEMORY_64KB:
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(7));
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SGPB | EEFC_FCR_FARG(8));
      break;
    case TCM_MEMORY_128KB:
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SGPB | EEFC_FCR_FARG(7));
      EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SGPB | EEFC_FCR_FARG(8));
      break;
    }
}

#ifdef ENABLE_TCM
/** \brief  TCM memory enable

    The function enables TCM memories
 */
__STATIC_INLINE void _TCM_Enable(void) 
{
  __DSB();
  __ISB();

//  SCB->ITCMCR = (SCB_ITCMCR_EN_Msk  | SCB_ITCMCR_RMW_Msk | SCB_ITCMCR_RETEN_Msk);
//  SCB->DTCMCR = ( SCB_DTCMCR_EN_Msk | SCB_DTCMCR_RMW_Msk | SCB_DTCMCR_RETEN_Msk);

  SCB->ITCMCR = SCB_ITCMCR_EN_Msk ;
  SCB->DTCMCR = SCB_DTCMCR_EN_Msk ;
  __DSB();
  __ISB();
}
#endif /* ENABLE_TCM */          
          
#ifdef CONF_BOARD_CONFIG_MPU_AT_INIT
/**
 * \brief Sets up the memory regions in the TCM
 */
static void _setup_memory_region( void )
{

                uint32_t dw_region_base_addr;
                uint32_t dw_region_attr;

                __DMB();

/**
*            ITCM memory region --- Normal
*            START_Addr:-  0x00000000UL
*            END_Addr:-    0x00400000UL
*/
                dw_region_base_addr =
                                ITCM_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_DEFAULT_ITCM_REGION;

                dw_region_attr =
                                MPU_AP_PRIVILEGED_READ_WRITE |
                                INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
                                mpu_cal_mpu_region_size(ITCM_END_ADDRESS - ITCM_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
*            Internal flash memory region --- Normal read-only
*            (update to Strongly ordered in write accesses)
*            START_Addr:-  0x00400000UL
*            END_Addr:-    0x00600000UL
*/

                dw_region_base_addr =
                                IFLASH_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_DEFAULT_IFLASH_REGION;

                dw_region_attr =
                                MPU_AP_PRIVILEGED_READ_WRITE |
                                INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
                                mpu_cal_mpu_region_size(IFLASH_END_ADDRESS - IFLASH_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
*            DTCM memory region --- Normal
*            START_Addr:-  0x20000000L
*            END_Addr:-    0x20400000UL
*/

                /* DTCM memory region */
                dw_region_base_addr =
                                DTCM_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_DEFAULT_DTCM_REGION;

                dw_region_attr =
                                MPU_AP_PRIVILEGED_READ_WRITE |
                                INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
                                mpu_cal_mpu_region_size(DTCM_END_ADDRESS - DTCM_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
*            SRAM Cacheable memory region --- Normal
*            START_Addr:-  0x20400000UL
*            END_Addr:-    0x2043FFFFUL
*/
                /* SRAM memory  region */
                dw_region_base_addr =
                                SRAM_FIRST_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_DEFAULT_SRAM_REGION_1;

                dw_region_attr =
                                MPU_AP_FULL_ACCESS    |
                                INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
                                mpu_cal_mpu_region_size(SRAM_FIRST_END_ADDRESS - SRAM_FIRST_START_ADDRESS)
                                | MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);


/**
*            Internal SRAM second partition memory region --- Normal
*            START_Addr:-  0x20440000UL
*            END_Addr:-    0x2045FFFFUL
*/
                /* SRAM memory region */
                dw_region_base_addr =
                                SRAM_SECOND_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_DEFAULT_SRAM_REGION_2;

                dw_region_attr =
                                MPU_AP_FULL_ACCESS    |
                                INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
                                mpu_cal_mpu_region_size(SRAM_SECOND_END_ADDRESS - SRAM_SECOND_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);

#ifdef MPU_HAS_NOCACHE_REGION
                dw_region_base_addr =
        SRAM_NOCACHE_START_ADDRESS |
        MPU_REGION_VALID |
        MPU_NOCACHE_SRAM_REGION;

    dw_region_attr =
        MPU_AP_FULL_ACCESS    |
        INNER_OUTER_NORMAL_NOCACHE_TYPE( SHAREABLE ) |
        mpu_cal_mpu_region_size(NOCACHE_SRAM_REGION_SIZE) |
        MPU_REGION_ENABLE;

    mpu_set_region( dw_region_base_addr, dw_region_attr);
#endif

/**
*            Peripheral memory region --- DEVICE Shareable
*            START_Addr:-  0x40000000UL
*            END_Addr:-    0x5FFFFFFFUL
*/
                dw_region_base_addr =
                                PERIPHERALS_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_PERIPHERALS_REGION;

                dw_region_attr = MPU_AP_FULL_ACCESS |
                                MPU_REGION_EXECUTE_NEVER |
                                SHAREABLE_DEVICE_TYPE |
                                mpu_cal_mpu_region_size(PERIPHERALS_END_ADDRESS - PERIPHERALS_START_ADDRESS)
                                |MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);


/**
*            External EBI memory  memory region --- Strongly Ordered
*            START_Addr:-  0x60000000UL
*            END_Addr:-    0x6FFFFFFFUL
*/
                dw_region_base_addr =
                                EXT_EBI_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_EXT_EBI_REGION;

                dw_region_attr =
                                MPU_AP_FULL_ACCESS |
                                /* External memory Must be defined with 'Device' or 'Strongly Ordered' attribute for write accesses (AXI) */
                                STRONGLY_ORDERED_SHAREABLE_TYPE |
                                mpu_cal_mpu_region_size(EXT_EBI_END_ADDRESS - EXT_EBI_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
*            SDRAM cacheable memory region --- Normal
*            START_Addr:-  0x70000000UL
*            END_Addr:-    0x7FFFFFFFUL
*/
                dw_region_base_addr =
                                SDRAM_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_DEFAULT_SDRAM_REGION;

                dw_region_attr =
                                MPU_AP_FULL_ACCESS    |
                                INNER_NORMAL_WB_RWA_TYPE( SHAREABLE ) |
                                mpu_cal_mpu_region_size(SDRAM_END_ADDRESS - SDRAM_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
*            QSPI memory region --- Strongly ordered
*            START_Addr:-  0x80000000UL
*            END_Addr:-    0x9FFFFFFFUL
*/
                dw_region_base_addr =
                                QSPI_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_QSPIMEM_REGION;

                dw_region_attr =
                                MPU_AP_FULL_ACCESS |
                                STRONGLY_ORDERED_SHAREABLE_TYPE |
                                mpu_cal_mpu_region_size(QSPI_END_ADDRESS - QSPI_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);


/**
*            USB RAM Memory region --- Device
*            START_Addr:-  0xA0100000UL
*            END_Addr:-    0xA01FFFFFUL
*/
                dw_region_base_addr =
                                USBHSRAM_START_ADDRESS |
                                MPU_REGION_VALID |
                                MPU_USBHSRAM_REGION;

                dw_region_attr =
                                MPU_AP_FULL_ACCESS |
                                MPU_REGION_EXECUTE_NEVER |
                                SHAREABLE_DEVICE_TYPE |
                                mpu_cal_mpu_region_size(USBHSRAM_END_ADDRESS - USBHSRAM_START_ADDRESS) |
                                MPU_REGION_ENABLE;

                mpu_set_region( dw_region_base_addr, dw_region_attr);


                /* Enable the memory management fault , Bus Fault, Usage Fault exception */
                SCB->SHCSR |= (SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk
                                                                                | SCB_SHCSR_USGFAULTENA_Msk);

                /* Enable the MPU region */
                mpu_enable( MPU_ENABLE | MPU_PRIVDEFENA);

                __DSB();
                __ISB();
}
#endif


/**
 * \brief Custom implementation of the low level initialization
 *
 * This is the code that gets called on processor reset. To initialize the
 * device.
 *
 */
int __user_low_level_init(void)
{
        SystemInit();
        
        /* Configure MPU */
        #ifdef CONF_BOARD_CONFIG_MPU_AT_INIT
          _setup_memory_region();
        #endif
  
        /* Configure TCM */
        _TCM_Disable();       

        #ifdef ENABLE_TCM  
          _TCM_Configure(TCM_MEMORY_64KB);
          _TCM_Enable();
		#else
         _TCM_Configure(TCM_MEMORY_NONE);
		#endif      

		#if defined(ENABLE_TCM) && defined(__GNUC__)
		/* In Atmel Studio (GCC), copy code_TCM from flash to ITCM */
		volatile char *dst = &_sitcm;
		volatile char *src = &_itcm_lma;
		while(dst < &_eitcm){
			*dst++ = *src++;
		}
 
		#endif


   
      return 1;
}
