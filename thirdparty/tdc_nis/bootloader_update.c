#include <stdint.h>
#include "bootloader_update.h"
#include "flash_efc.h"
#include "gpbr.h"
#include "conf_busart_if.h"
#include "pl360g55cf_ek.h"


#define BOOTLOADER_START_ADDRESS 0x00400000
#define APP_ADDITIONAL_INFO_BASE	 0x4041E0
#define BOOTLOADER_MAX_SIZE 16384
#define BUFFER_SIZE  1024
#define STAY_IN_BOOT_KEY	777


#ifdef __cplusplus
extern "C" {
#endif

uint8_t data[BUFFER_SIZE] = {0};

static void EraseFlash();
static void ReadFromFlash(uint32_t start_addr, uint8_t *buff, uint32_t len);

void CheckVersion()
{
	printf(__DATE__);
  printf(" ");
	printf(__TIME__);
	printf("\r\n");
}

/* Replace bootloader with bootloader image appended at the end of this app bin file */
void ReplaceBootloader()
{
	uint32_t app_size = *((uint32_t*)APP_ADDITIONAL_INFO_BASE);
	uint32_t bootloader_size = *((uint32_t*)(APP_ADDITIONAL_INFO_BASE + 4));
	
	/* Check for possible errors */
	if(app_size == 0 || app_size == 0xFFFFFFFF)
	{
		printf("App size is not valid, aborting replace bootloader!\r\n");
		return;
	}
	if(bootloader_size == 0 || bootloader_size == 0xFFFFFFFF)
	{
		printf("Bootloader size is not valid, aborting replace bootloader!\r\n");
		return;
	}

	uint32_t bootloader_start_address = 0x404000 + app_size; // new bootloader starting location
	uint32_t bootloader_end_address = bootloader_start_address + bootloader_size;
	
	if( bootloader_end_address - bootloader_start_address >= BOOTLOADER_MAX_SIZE)
	{
		printf("Bootloader larger than 16k, abort!!!\r\n");
		return;
	}
	
	__disable_irq();
	
	EraseFlash();
	printf("Flash erased!!!\r\n");
	
	uint32_t bl_current_index = bootloader_start_address;
	
	uint32_t write_index = BOOTLOADER_START_ADDRESS;
	
	while(bl_current_index + BUFFER_SIZE <= bootloader_end_address)
	{
		// read and write full buffer from flash
		ReadFromFlash(bl_current_index, data, BUFFER_SIZE);
		flash_write(write_index, (const void *)data, BUFFER_SIZE, 0);
		
		bl_current_index += BUFFER_SIZE;
		write_index += BUFFER_SIZE;
	}
	
	if(bootloader_end_address > bl_current_index)
	{
		/* Write remaining bytes*/
		uint32_t remaining_bytes = bootloader_end_address - bl_current_index;
		ReadFromFlash(bl_current_index, data, remaining_bytes);
		flash_write(write_index, (const void *)data, remaining_bytes, 0);
	}
	
	printf("Bootloader replaced!!!\r\n");
	__enable_irq();
}


void JumpToBootloader()
{
	gpbr_write(GPBR3, STAY_IN_BOOT_KEY); /* Write key so that bootloader knows that it needs to stay in bootloader */
	// Reset processor and its peripherials
	Rstc *pHw = RSTC;
  pHw->RSTC_CR = RSTC_CR_PROCRST | RSTC_CR_PERRST | RSTC_CR_EXTRST | RSTC_CR_KEY_PASSWD;
}


static void ReadFromFlash(uint32_t start_addr, uint8_t *buff, uint32_t len)
{
	for(uint32_t idx = start_addr; idx < (start_addr + len); idx++)
	{
		*buff = *(uint8_t*)idx;
		buff++;
	}
}

static void EraseFlash()
{
	uint32_t address = BOOTLOADER_START_ADDRESS + 0x1000; /* Add offset for clearing in 'middle' of sector*/
	uint32_t sector_size = 0x2000; /* This is first 2 small sector size */
	
	for (int idx = 0; idx < 2; idx++) 
	{
		flash_erase_sector(BOOTLOADER_START_ADDRESS + (sector_size * idx));
	}
}

#ifdef __cplusplus
}
#endif