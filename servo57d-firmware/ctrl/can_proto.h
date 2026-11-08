/* CAN protocol of the open firmware: commands in, force / state telemetry out, every
 * parameter readable and writable. Standard 11 bit IDs, little endian, node id n (1..127).
 *   host -> drive  0x100+n  command, byte0 = command code
 *   drive -> host  0x080+n  reply to a command (byte0 = code, byte1 = status)
 *   drive -> host  0x180+n  telemetry A: pos i32 | pos_err i16 | state u8 | fault u8
 *   drive -> host  0x280+n  telemetry B (force): iq i16 mA | torque i16 mNm |
 *                                                 vel i16 0.1 rpm | vbus u16 10 mV
 *   drive -> host  0x380+n  parameter reply: code u8 | status u8 | id u16 | value 4 bytes
 *
 * Commands (byte0):
 *   0x01 enable        u8 0/1
 *   0x02 mode          u8 0 torque, 1 velocity, 2 position
 *   0x10 torque        i16 mA              (torque mode)
 *   0x11 velocity      i32 0.1 rpm         (velocity mode)
 *   0x12 position      i32 counts          (position mode, 16384 per rev)
 *   0x20 current limit u16 mA
 *   0x21 stream        u16 ms, 0 = off     (telemetry A and B every period)
 *   0x30 set parameter id u16, value 4 bytes   -> 0x380+n reply with the value read back
 *   0x31 get parameter id u16                  -> 0x380+n reply
 *   0x32 parameter info id u16, sel u8         -> sel 0: type, flags; 1: min f32; 2: max f32
 *                                                 (reply byte1: status | sel << 4)
 *   0x33 parameter at  index u16 (0xFFFF: count) -> reply value = id (or the count)
 *   0x35 save          store the saved set (PF_PERSIST) through the flash callback
 *   0x36 defaults      restore the configuration defaults (drive off)
 *   0x40 read once     (sends telemetry A and B)
 *   0x7F clear fault
 * Telemetry A/B are only sent on 0x21 or 0x40, never unasked.
 * Values: PT_F32 is the IEEE float, PT_I32 an int32, PT_U16 / PT_U8 are zero-extended.
 * Parameter ids, types, ranges, units and flags: params.c, docs/can_params.md.
 * Status: 0 ok, 1 bad argument, 2 unknown, 3 read-only, 4 out of range,
 *         5 not while running, 6 no flash store. */
#ifndef CAN_PROTO_H
#define CAN_PROTO_H
#include <stdint.h>
#include "servo.h"

typedef struct { uint16_t id; uint8_t len; uint8_t data[8]; } can_frame_t;

typedef struct can_proto {
    uint8_t  node;
    uint16_t stream_ms;
    uint16_t countdown_ms;
    /* called by the save command with the blob to store; returns 0 on success.
     * NULL: the drive answers status 6. Set by the MCU glue (flash) or the simulator. */
    int (*save_cb)(const uint8_t *blob, uint16_t len);
} can_proto_t;

void can_proto_init(can_proto_t *c, uint8_t node);

/* Handle one received frame. Frames to send go to out[] (max 4); returns the count. */
int can_proto_rx(can_proto_t *c, servo_t *s, const can_frame_t *in, can_frame_t *out);

/* Call every millisecond; returns the frames to send (0 or 2). */
int can_proto_tick_ms(can_proto_t *c, const servo_t *s, can_frame_t *out);

/* Encoders, public for tests and for the host side */
void can_proto_telemetry(const can_proto_t *c, const servo_t *s, can_frame_t *a, can_frame_t *b);

#endif
