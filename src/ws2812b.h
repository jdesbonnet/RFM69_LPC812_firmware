#ifndef WS2812B_H_
#define WS2812B_H_
//void ws2812b_init(int port, int pin);
void ws2812b_init();
void ws2812b_bitbang (uint32_t color);
void ws2812b_reset ();
#endif
