#ifndef __PARSE_UTIL_H__
#define __PARSE_UTIL_H__
uint32_t parse_hex (uint8_t *buf);
uint32_t parse_dec_or_hex (uint8_t *buf);
int32_t parse_dec(uint8_t *buf, uint8_t **end);

#endif
