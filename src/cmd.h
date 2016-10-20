#ifndef CMD_H_
#define CMD_H_

int cmd_set_node_addr(int argc, uint8_t **argv);
int cmd_packet_transmit(int argc, uint8_t **argv);
int cmd_flags(int argc, uint8_t **argv);
int cmd_version(int argc, uint8_t **argv);
int cmd_set_uart_speed (int argc, uint8_t **argv);
int cmd_remote_cmd (int argc, uint8_t **argv);
int cmd_node_query (int argc, uint8_t **argv);
int cmd_gps (int argc, uint8_t **argv);
int cmd_param (int argc, uint8_t **argv);
int cmd_wake_node (int argc, uint8_t **argv);
#endif
