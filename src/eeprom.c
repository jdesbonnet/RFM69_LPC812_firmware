/**
 * EEPROM emulation using flash In-Application-Programming (IAP) library
 */

#include <LPC8xx.h>
#include <stdint.h>
#include <string.h>
#include "eeprom.h"
#include "iap_driver.h"


// Store EEPROM page at offset 0x2000 (8kB into flash space)
#define CONFIG_FLASH_ADDR_OFFSET     0x2000
#define CONFIG_FLASH_PAGE            (CONFIG_FLASH_ADDR_OFFSET >> 6)
#define CONFIG_FLASH_SECTOR_FOR_PAGE (CONFIG_FLASH_PAGE >> 4)
#define CONFIG_FLASH_PAGE_SIZE       1
#define CONFIG_FLASH_SECTOR_SIZE     (CONFIG_FLASH_PAGE_SIZE >> 4)


// iap_copy_ram_to_flash() must have source data in SRAM. To ensure this is the case
// copy to buffer we know for sure is in RAM (For example data could be a const
// string stored in flash: it seems that doesn't work.. flash-to-flash copy
// is not possible
static uint8_t eeprom_ram_buffer[64];

static uint32_t part_id, bootcode_rev, unique_id[4];

int32_t eeprom_write (uint8_t *data) {


	uint32_t iap_status;

	iap_init();

	memcpy (eeprom_ram_buffer, data, 64);


	/* Q1: What is the part id of this MCU ? */
	iap_read_part_id(&part_id);
	if (part_id == 0x00) {
		return -1;
	}

	/* Q2: What is the bootROM revision of this MCU ? */
	iap_read_bootcode_rev(&bootcode_rev);
	if (bootcode_rev == 0x00) {
		/* Invalid bootcode rev */
		return -2;
	}

	/* Q3: What is the unique ID of this MCU ? Is it same with other MCU ? */
	iap_read_unique_id(unique_id);
	if ((unique_id[0] == 0x00) && (unique_id[1] == 0x00)
			&& (unique_id[2] == 0x00) && (unique_id[3] == 0x00)) {
		/* Invalid unique id */
		return -3;
	}



	/* Prepare the page for erase */
	iap_status = (__e_iap_status) iap_prepare_sector(CONFIG_FLASH_SECTOR_FOR_PAGE,
			(CONFIG_FLASH_SECTOR_FOR_PAGE + CONFIG_FLASH_SECTOR_SIZE));
	if (iap_status != CMD_SUCCESS) return -4;

	/* Erase the page */
	iap_status = (__e_iap_status) iap_erase_page(CONFIG_FLASH_PAGE,
			(CONFIG_FLASH_PAGE + CONFIG_FLASH_PAGE_SIZE - 1));
	if (iap_status != CMD_SUCCESS) return -5;

	/* Prepare the page for writing */
	iap_status = (__e_iap_status) iap_prepare_sector(CONFIG_FLASH_SECTOR_FOR_PAGE,
			(CONFIG_FLASH_SECTOR_FOR_PAGE + CONFIG_FLASH_SECTOR_SIZE));
	if (iap_status != CMD_SUCCESS) return -6;

	/* Write data to page */
	//iap_status = (__e_iap_status) iap_copy_ram_to_flash(data,
	//		(void *)CONFIG_FLASH_ADDR_OFFSET, 64);
	iap_status = (__e_iap_status) iap_copy_ram_to_flash(&eeprom_ram_buffer,
			(void *)CONFIG_FLASH_ADDR_OFFSET, 64);
	if (iap_status != CMD_SUCCESS) return -7;

	return 0;
}

