#include "mt6816_proto.h"
#include "servo.h"

uint8_t mt6816_decode(const uint8_t rx[4], uint16_t *angle)
{
    uint8_t flags = 0;
    uint16_t w = ((uint16_t)rx[1] << 8) | rx[2];
    *angle = w >> 2;
    uint16_t bits = (w >> 1);                 /* 0x03[7:0] and 0x04[7:1] */
    bits ^= bits >> 8; bits ^= bits >> 4; bits ^= bits >> 2; bits ^= bits >> 1;
    if ((bits & 1) != (rx[2] & 1)) flags |= SV_ENCF_PARITY;
    if (rx[2] & 0x02) flags |= SV_ENCF_NOMAG;
    return flags;
}
