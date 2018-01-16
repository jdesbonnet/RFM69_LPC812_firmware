#ifndef DELAY_H_
#define DELAY_H_
void delay_init(void);
void delay_deinit(void);
void delay_nop_loop(uint32_t);
void delay_milliseconds(uint32_t);
void delay_microseconds(uint32_t);
void delay_wkt(uint32_t);
#endif
