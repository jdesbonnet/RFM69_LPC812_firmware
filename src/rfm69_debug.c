/**
 * RFM69 debugging functions. These have dependencies on MCU specific libraries.
 * Not required for normal use.
 *
 */

//#define RFM69_DEBUG
#ifdef RFM69_DEBUG

#include <stdint.h>
#include "rfm69.h"
#include "uart.h"
#include "print_util.h"


extern const uint8_t RFM69_CONFIG[][2];


void rfm69_config_print () {
	int i;
	for (i = 0; RFM69_CONFIG[i][0] != 255; i++) {
	    print_hex8(RFM69_CONFIG[i][0]);
	    UARTSend(" ",1);
	    print_hex8(RFM69_CONFIG[i][1]);
	    UARTSend("\r\n",2);
	}
}

rfm69_config_verify() {
	int i;
	uint8_t v;
	int bad=0;
	int count = 0;
	for (i = 0; RFM69_CONFIG[i][0] != 255; i++) {
		v = rfm69_register_read(RFM69_CONFIG[i][0]);
		if (v != RFM69_CONFIG[i][1]) {
			UARTSend("ECFG@",5);
			print_hex8(RFM69_CONFIG[i][0]);
			UARTSend(": ",2);
			print_hex8(RFM69_CONFIG[i][1]);
			UARTSend(" ",1);
			print_hex8(v);
			UARTSend("\r\n",2);
			bad++;
		}
		count++;
	}
	print_decimal(bad);
	UARTSend("/",1);
	print_decimal(count);
	UARTSend(" failed\r\n",9);
}

void rfm69_showirqreg () {
    while (1) {
    	print_hex8(rfm69_register_read(RFM69_IRQFLAGS1));
    	print_hex8(rfm69_register_read(RFM69_IRQFLAGS2));
    	UARTSend ("\r\n",2);
    }
}

void rfm69_register_print () {
	int i;

	uint8_t reg_addr,reg_val;
	uint8_t reg_val_a[0x70];
	uint8_t reg_val_b[0x70];

	UARTSend ("\r\n",2);

	// Read register file in single mode
	for (i = 1; i < 0x6f; i++) {
		UARTSend ("reg[",4);
		print_hex8(i);
		UARTSend ("]=",2);
		reg_val = rfm69_register_read(i);
		print_hex8(reg_val);
		UARTSend ("\r\n",2);
		reg_val_a[i] = reg_val;
	}

	// Read register file in BURST mode
	rfm69_nss_assert();
	reg_addr=1;
	spi_send (&reg_addr,1);
	for (i = 1; i < 0x6f; i++) {
		//spi_receive (&reg_val,1);
		reg_val=spi_transfer_byte(0xff);
		UARTSend ("reg[",4);
		print_hex8(i);
		UARTSend ("]=",2);
		print_hex8(reg_val);
		UARTSend ("\r\n",2);
		reg_val_b[i] = reg_val;
	}
	spi_flush();
	rfm69_nss_deassert();

	for (i = 1; i < 0x6f; i++) {
		UARTSend ("reg[",4);
		print_hex8(i);
		UARTSend ("]=",2);
		print_hex8(reg_val_a[i]);
		UARTSend (" ",1);
		print_hex8(reg_val_b[i]);
		UARTSend ("\r\n",2);
	}
}

#endif


