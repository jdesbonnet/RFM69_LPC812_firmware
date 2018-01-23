#include "config.h"
#include "iap_driver.h"
#include "myuart.h"
#include "params.h"
#include "frame_buffer.h"
#include "mac.h"

// Radio packet frame buffers
extern frame_buffer_type tx_buffer;
extern frame_buffer_type rx_buffer;
extern params_union_type params_union;

/**
 * Experimental bootloader that runs entirely in RAM so that any page of
 * flash memory can be reprogrammed from here. Implements a very bare
 * minimum set of radio commands that allow for flash reprogramming and
 * verification.
 */

RAM_FUNC
void ram_memcpy (uint8_t *dst, uint8_t *src, int len) {
	int i;
	for (i = 0; i < len; i++) {
		dst[i] = src[i];
	}
}

RAM_FUNC
int flash_write_page (void *page_base_addr, void *buf) {

	iap_init();

	uint32_t flash_page = (uint32_t)page_base_addr >> 6;
	uint32_t flash_sector = flash_page >> 4;

	int iap_status;
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
	iap_status = (__e_iap_status) iap_copy_ram_to_flash(buf,
			page_base_addr, 64);
	if (iap_status != CMD_SUCCESS) return -7;

	return 0;
}

RAM_FUNC
static void ota_irq_disable () {
	// Disable all interrupts: can't have entry into ISR while ISR is being rewritten
	NVIC_DisableIRQ(SysTick_IRQn);
	NVIC_DisableIRQ(WDT_IRQn);
	NVIC_DisableIRQ(WKT_IRQn);
	NVIC_DisableIRQ(CMP_IRQn);
	NVIC_DisableIRQ(UART0_IRQn);
	NVIC_DisableIRQ(UART1_IRQn);
	NVIC_DisableIRQ(PININT0_IRQn);
	NVIC_DisableIRQ(PININT1_IRQn);
	NVIC_DisableIRQ(PININT2_IRQn);
	NVIC_DisableIRQ(PININT3_IRQn);
}

RAM_FUNC
static void ota_irq_enable () {
	// Disable all interrupts: can't have entry into ISR while ISR is being rewritten
	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WKT_IRQn);
	NVIC_EnableIRQ(CMP_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);
	NVIC_EnableIRQ(PININT0_IRQn);
	NVIC_EnableIRQ(PININT1_IRQn);
	NVIC_EnableIRQ(PININT2_IRQn);
	NVIC_EnableIRQ(PININT3_IRQn);
}

RAM_FUNC
uint32_t ota_page_crc (uint8_t *addr) {

	// Enable click to CRC block
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<13);

	//uint8_t *addr = page * 64;
	int i;

	// UM10601 §19.7.3, page 264 CRC-32 set-up
	LPC_CRC->MODE = 0x00000036;
	LPC_CRC->SEED = 0xFFFFFFFF;

	// Can do this byte by byte or by DWORD (latter more efficient)
	for (i = 0; i < 64; i++) {
		LPC_CRC->WRDATA8 = addr[i];
	}

	uint32_t crc32 = LPC_CRC->SUM;

	// Disable clock to CRC block.
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<13);

	return crc32;
}

RAM_FUNC
void ota_bootloader (uint8_t myaddr) {

	tfp_printf("RAM resident bootloader\r\n");

	led_on();

	//ota_irq_disable();

	int i = 0;
	do {
		if (IS_PACKET_READY()) {

			led_off();
#ifdef RADIO_RFM9x
			int frame_len = rfm98_frame_rx(rx_buffer.buffer,RXTX_BUFFER_SIZE);
			//rfm_register_write(RFM98_IRQFLAGS,0xff);
#else
			int frame_len = rfm69_frame_rx(rx_buffer.buffer,RXTX_BUFFER_SIZE);
#endif

#ifdef RADIO_RFM9x
			{
			int ii;
			tfp_printf("; FRAME: [ ");
			for (ii = 0; ii < frame_len; ii++) {
				tfp_printf(" %x", rx_buffer.buffer[ii]);
			}
			tfp_printf (" ] %d %d\r\n", rfm98_last_packet_rssi(), rfm98_last_packet_snr() );
			}
#endif

			led_on();


			// 0xff is the broadcast address
			if (rx_buffer.header.to_addr == params_union.params.node_addr) {

				// bit 7: ack request
				if (rx_buffer.header.msg_type & 0x80) {
					tx_buffer.header.msg_type = PKT_ACK;
					tx_buffer.payload[0] = 0;
					rfm69_frame_tx(tx_buffer.buffer, 4);
				}

				MyUARTSendStringZ("packet: ");
				int j;
				for (j = 0; j < frame_len; j++) {
					MyUARTPrintHex8(rx_buffer.buffer[j]);
					MyUARTSendByte(' ');
				}
				MyUARTSendCRLF();

				switch (rx_buffer.header.msg_type & 0x7f) {
				case PKT_OTA_FLASH_WRITE:
				{
					// Flash starts at 0x0 and will never exceed 64k, so 16 bit addr ok.
					// Radio frames are not large enough to accommodate the 64 byte page
					// so need to write in 32 byte chunks. It means each flash page must
					// be written twice. But FW upgrades are not expected to happen very
					// often.
					void *addr = (void *)(rx_buffer.payload[0]<<8 | rx_buffer.payload[1]);
					void *page_base_addr = (void *)(rx_buffer.payload[0]<<8 | (rx_buffer.payload[1]&0xc0));

					uint8_t buf[64];
					ram_memcpy(buf,page_base_addr,64);

					// Upper or lower part of flash page?
					if ((uint32_t)addr & 0x0020) {
						ram_memcpy(buf+32, &rx_buffer.payload[2], 32);
						//MyUARTSendStringZ("Upper half page\r\n");
					} else {
						ram_memcpy(buf, &rx_buffer.payload[2], 32);
						//MyUARTSendStringZ("Lower half page\r\n");
					}

					flash_write_page(page_base_addr,buf);

					MyUARTSendStringZ(" done.\r\n");
					break;
				}
				case PKT_OTA_FLASH_CRC_REQUEST:
				{
					// Flash starts at 0x0 and will never exceed 64k, so 16 bit addr ok
					// Frames are not large enough to accommodate the 64byte page so
					// need to write in 32byte chunks.

					uint8_t *addr = (uint8_t *)(rx_buffer.payload[0]<<8 | (rx_buffer.payload[1]&0xc0));

					tx_buffer.header.msg_type = PKT_OTA_FLASH_CRC_RESPONSE;
					tx_buffer.payload[0] = rx_buffer.payload[0];
					tx_buffer.payload[1] = rx_buffer.payload[1];
					uint32_t *crc32 = (uint32_t *)&tx_buffer.payload[2];
					*crc32 = ota_page_crc(addr);
					MyUARTSendStringZ("CRC ");
					MyUARTPrintHex(*crc32);
					MyUARTSendCRLF();

					*crc32 = __builtin_bswap32(*crc32);

					rfm69_frame_tx(tx_buffer.buffer, 3+2+4);

					break;
				}
				case PKT_OTA_FLASH_REQUEST:
				{
					// Flash starts at 0x0 and will never exceed 64k, so 16 bit addr ok
					// Frames are not large enough to accommodate the 64byte page so
					// need to write in 32byte chunks.
					uint32_t *addr = (uint32_t *)(rx_buffer.payload[0]<<8 | (rx_buffer.payload[1]&0xc0));
					tx_buffer.header.msg_type = PKT_OTA_FLASH_RESPONSE;
					tx_buffer.payload[0] = rx_buffer.payload[0];
					tx_buffer.payload[1] = rx_buffer.payload[1];
					ram_memcpy(&tx_buffer.payload[2], addr, 32);
					//rfm69_frame_tx(tx_buffer.buffer, 3+2+32);
					break;
				}
				case PKT_OTA_EXIT_BOOTLOADER:
				{
					return;
				}
				case PKT_OTA_REBOOT:
				{
					NVIC_SystemReset();
				}

				case PKT_OTA_REPROG_TEST:
				{
					void *addr = (void *)(rx_buffer.payload[0]<<8 | rx_buffer.payload[1]);
					uint8_t buf[64];
					ram_memcpy(buf, addr, 64);

					MyUARTSendStringZ("PROG_TEST ");
					MyUARTPrintHex(addr);
					flash_write_page(addr,buf);
					MyUARTSendStringZ("OK\r\n");
				}
				}
			}

			i++;
		}
	} while (i < 100000);
	MyUARTSendStringZ("; OTA_TO\r\n");
}
