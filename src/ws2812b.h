#ifndef WS2812B_H_
#define WS2812B_H_
void ws2812b_init(int port, int pin);
void ws2812b_bigbang (uint32_t rgb);
void ws2812b_reset ();
#endif
