#ifndef FRAME_BUFFER_H_
#define FRAME_BUFFER_H_

typedef struct {
	//uint8_t flags; // proposed for v0.3
	uint8_t to_addr;
	uint8_t from_addr;
	uint8_t msg_type;
} frame_header_type;

typedef union {
	struct {
		frame_header_type header;
		uint8_t payload[66 - sizeof(frame_header_type)];
	};
	uint8_t buffer[66];
} frame_buffer_type;

#endif
