#include "config.h"
#include "mac.h"

/**
 * Fixed MAC address assignments based on MCU ID.
 * TODO: MCU unique ID is actually a 128bit value, but for convenience
 * incorrectly assuming that first 32bits are unique.
 */
int getNodeAddrFromMpuId (void){
    // Read MCU serial number
    uint32_t mcu_unique_id[4];
	iap_read_unique_id(&mcu_unique_id);

	switch (mcu_unique_id[0]) {
	case 0x18044043: return 0x40;
	case 0x1902E033: return 0x41; // hand wired prototype
	case 0x05034039: return 0x42; // tip-bucket sensor (V1B hack required)maps
	case 0x05034043: return 0x43;
	case 0x05046049: return 0x44;
	case 0x05046043: return 0x45;
	case 0x1901402c: return 0x46;
	case 0x1902e004: return 0x47;

	// RFM98 433MHz radios
	case 0x19017037: return 0x51;
	case 0x1902e034: return 0x52;
	case 0x1900c037: return 0x53; // UGSS
	case 0x19016034: return 0x54; // ABPM
	case 0x19008027: return 0x55;

	case 0x1901603a: return 0x56; // UG Onagh
	case 0x0502302d: return 0x57; // Base station Onagh
	case 0x0c012022: return 0x58;

	// RFM95 868MHz radios
	case 0x1901703c:return 0x61; // RFM95 868

	}

	return 0xff;

}
