#ifndef CMD_H_
#define CMD_H_

#define E_OK (0)
#define E_INVALID_ARG (-2)
#define E_WRONG_ARGC (-3)
#define E_PKT_TOO_LONG (-4)

int cmd_set_node_addr(int argc, uint8_t **argv);
int cmd_packet_transmit(int argc, uint8_t **argv);

#endif
