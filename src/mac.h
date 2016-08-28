#ifndef MAC_H_
#define MAC_H_
typedef enum {
	PKT_NOP = 0,
	PKT_ACK = 1,
	PKT_APP_DATA = 2,
	PKT_RELAY = 0x42,
	PKT_REMOTE_CMD = 0x44,
	PKT_EXEC_MEM = 0x45,
	PKT_START_PINGPONG = 0x50,
	PKT_LOCATION_REPORT = 0x52,
	PKT_STATUS_REPORT = 0x53,
	PKT_STATUS_RESPONSE = 0x73,
	PKT_LED_BLINK = 0x55,
	PKT_RADIO_REG_READ = 0x58,
	PKT_RADIO_REG_WRITE = 0x59,
	PKT_MEM_READ_REQUEST = 0x3c,
	PKT_MEM_READ_RESPONSE = 0x1c,
	PKT_MEM_WRITE = 0x3e,
} packet_type;
#endif
