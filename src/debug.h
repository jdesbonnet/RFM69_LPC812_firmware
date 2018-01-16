#ifndef DEBUG_H_
#define DEBUG_H_
//#define debug(x...) tfp_printf(x)
#define debug(fmt, ...) tfp_printf("; DEBUG: " fmt "\r\n", ##__VA_ARGS__)

void debug_show_registers();

#endif
