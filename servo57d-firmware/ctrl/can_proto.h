/* CAN protocol of the open firmware: commands in, force / state telemetry out.
 * Standard 11 bit IDs, little endian. n = node id (1..127).
 *   host -> drive  0x100+n  command, byte0 = command code
 *   drive -> host  0x180+n  telemetry A: pos i32 | pos_err i16 | state u8 | fault u8
 *   drive -> host  0x280+n  telemetry B (force): iq i16 mA | torque i16 mNm |
 *                                                 vel i16 0.1 rpm | vbus u16 10 mV
 *   drive -> host  0x080+n  reply to a command that has one (byte0 = code, byte1 = 0 ok)
 *
 * Commands (byte0):
 *   0x01 enable        u8 0/1
 *   0x02 mode          u8 0 torque, 1 velocity, 2 position
 *   0x10 torque        i16 mA              (torque mode)
 *   0x11 velocity      i32 0.1 rpm         (velocity mode)
 *   0x12 position      i32 counts          (position mode, 16384 per rev)
 *   0x20 current limit u16 mA
 *   0x21 stream        u16 ms, 0 = off     (telemetry A and B every period)
 *   0x30 gain          u8 which (0 pos_kp, 1 vel_kp, 2 vel_ki), f32
 *   0x40 read once     (sends telemetry A and B)
 *   0x7F clear fault
 * Telemetry A/B are only sent on 0x21 or 0x40, never unasked. */
#ifndef CAN_PROTO_H
#define CAN_PROTO_H
#include <stdint.h>
#include "servo.h"

typedef struct { uint16_t id; uint8_t len; uint8_t data[8]; } can_frame_t;

typedef struct {
    uint8_t  node;
    uint16_t stream_ms;
    uint16_t countdown_ms;
} can_proto_t;

void can_proto_init(can_proto_t *c, uint8_t node);

/* Handle one received frame. Frames to send go to out[] (max 4); returns the count. */
int can_proto_rx(can_proto_t *c, servo_t *s, const can_frame_t *in, can_frame_t *out);

/* Call every millisecond; returns the frames to send (0 or 2). */
int can_proto_tick_ms(can_proto_t *c, const servo_t *s, can_frame_t *out);

/* Encoders, public for tests and for the host side */
void can_proto_telemetry(const can_proto_t *c, const servo_t *s, can_frame_t *a, can_frame_t *b);

#endif
