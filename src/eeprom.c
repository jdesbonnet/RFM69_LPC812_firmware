/**
 * EEPROM emulation using flash In-Application-Programming (IAP) library to write/read
 * one 64 byte page from flash memory.
 */

#include <LPC8xx.h>
#include <stdint.h>
#include <string.h>
#include "config.h"
#include "eeprom.h"
#include "params.h"
#include "iap_driver.h"

// Flags is deprecated
#include "flags.h"

// Allocate 64 byte aligned, 64 byte block in flash memory (0x00000000 - 0x00003999) on
// 16kB LPC812. Assignment to constant ( = {0}) seems necessary to force allocation in
// flash area.

//const char eeprom_flashpage[64] __attribute__ ((aligned (64))) = {0};


const params_union_type eeprom_flashpage __attribute__ ((aligned (64))) = {
		.params = {
		.node_addr = 0xff,
		//.operating_mode = MODE_LOW_POWER_POLL,
		.operating_mode = MODE_AWAKE,

		.poll_interval = 5,
		.listen_period_cs = 80,
		.link_loss_timeout_s = 1800
		}
};


/**
 * Write 64 byte page to flash.
 *
 * @param data Pointer to 64 byte block of memory to write to flash. This must be in SRAM
 * area of memory.
 *
 * @return 0 for success, negative value for error.
 */
int32_t eeprom_write (uint8_t *data) {

	uint32_t iap_status;

	iap_init();

	uint32_t flash_page = (uint32_t)&eeprom_flashpage >> 6;
	uint32_t flash_sector = flash_page >> 4;

	// Example code checks MCU part ID, bootcode revision number and serial number. There are some
	// differences in behavior across silicon revisions (in particular to do with ability
	// to erase multiple sectors at the same time). In this case we require to be able to program
	// just one sector, so this does not conern us.

	/* Prepare the page for erase */
	iap_status = (__e_iap_status) iap_prepare_sector(flash_sector, flash_sector);
	if (iap_status != CMD_SUCCESS) return -4;

	/* Erase the page */
	iap_status = (__e_iap_status) iap_erase_page(flash_page, flash_page);
	if (iap_status != CMD_SUCCESS) return -5;

	/* Prepare the page for writing */
	iap_status = (__e_iap_status) iap_prepare_sector(flash_sector, flash_sector);
	if (iap_status != CMD_SUCCESS) return -6;

	/* Write data to page */
	iap_status = (__e_iap_status) iap_copy_ram_to_flash(data,
			(void *)&eeprom_flashpage, 64);
	if (iap_status != CMD_SUCCESS) return -7;

	return 0;
}

/**
 * Copy 64 byte content of EEPROM flash area to supplied buffer.
 */
int32_t eeprom_read (uint8_t *data) {
	memcpy (data, &eeprom_flashpage, 64);
	return 0;
}

void * eeprom_get_addr () {
	return (void *)&eeprom_flashpage;
}

