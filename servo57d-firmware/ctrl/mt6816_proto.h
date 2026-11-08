#ifndef MT6816_PROTO_H
#define MT6816_PROTO_H
#include <stdint.h>

/* Decode the 4-byte frame clocked out after the command byte 0x83
 * (read reg 0x03, then reg 0x04): rx[1] = angle<13:6>, rx[2] = angle<5:0>,
 * No_Mag, parity (even over rx[1] and rx[2] bits 7..1).
 * Returns SV_ENCF_* flags (0 = good); *angle gets the 14 bit angle. */
uint8_t mt6816_decode(const uint8_t rx[4], uint16_t *angle);
#endif
