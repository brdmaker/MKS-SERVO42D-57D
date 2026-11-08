#include "can_proto.h"
#include <string.h>
#include <math.h>

#define RPM_PER_RADS (60.0f / 6.2831853f)

static int16_t sat16(float x)
{
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lrintf(x);
}
static void put16(uint8_t *d, uint16_t v) { d[0] = v & 0xFF; d[1] = v >> 8; }
static void put32(uint8_t *d, uint32_t v) { for (int i = 0; i < 4; i++) d[i] = (v >> (8 * i)) & 0xFF; }
static int16_t  get16(const uint8_t *d) { return (int16_t)(d[0] | (d[1] << 8)); }
static int32_t  get32(const uint8_t *d) { return (int32_t)((uint32_t)d[0] | (uint32_t)d[1] << 8 | (uint32_t)d[2] << 16 | (uint32_t)d[3] << 24); }

void can_proto_init(can_proto_t *c, uint8_t node)
{
    memset(c, 0, sizeof *c);
    c->node = node;
}

void can_proto_telemetry(const can_proto_t *c, const servo_t *s, can_frame_t *a, can_frame_t *b)
{
    a->id = 0x180 + c->node; a->len = 8;
    put32(a->data, (uint32_t)s->pos);
    put16(a->data + 4, (uint16_t)sat16((float)s->pos_err));
    a->data[6] = (uint8_t)s->state | (uint8_t)(s->mode << 4) | (uint8_t)((s->status & SV_ST_STALL) ? 0x80 : 0);
    a->data[7] = (uint8_t)s->fault;

    b->id = 0x280 + c->node; b->len = 8;
    put16(b->data,     (uint16_t)sat16(s->iq_filt * 1000.0f));
    put16(b->data + 2, (uint16_t)sat16(s->torque_nm * 1000.0f));
    put16(b->data + 4, (uint16_t)sat16(s->vel * RPM_PER_RADS * 10.0f));
    put16(b->data + 6, (uint16_t)lrintf(s->vbus * 100.0f));
}

int can_proto_rx(can_proto_t *c, servo_t *s, const can_frame_t *in, can_frame_t *out)
{
    if (in->id != 0x100 + c->node || in->len < 1) return 0;
    const uint8_t *d = in->data;
    int n = 0, status = 0;
    switch (d[0]) {
    case 0x01: if (in->len < 2) status = 1; else servo_enable(s, d[1] != 0); break;
    case 0x02: if (in->len < 2 || d[1] > 2) status = 1; else servo_set_mode(s, (sv_mode_t)d[1]); break;
    case 0x10: if (in->len < 3) status = 1; else servo_set_torque(s, get16(d + 1) * 0.001f); break;
    case 0x11: if (in->len < 5) status = 1; else servo_set_velocity(s, get32(d + 1) * 0.1f / RPM_PER_RADS); break;
    case 0x12: if (in->len < 5) status = 1; else servo_set_position(s, get32(d + 1)); break;
    case 0x20: if (in->len < 3) status = 1; else servo_set_current_limit(s, (uint16_t)get16(d + 1) * 0.001f); break;
    case 0x21: if (in->len < 3) status = 1; else { c->stream_ms = (uint16_t)get16(d + 1); c->countdown_ms = c->stream_ms; } break;
    case 0x30:
        if (in->len < 6 || d[1] > 2) { status = 1; break; }
        { float g; memcpy(&g, d + 2, 4);
          if (!(g >= 0.0f) || g > 1e6f) { status = 1; break; }
          if (d[1] == 0) s->cfg.pos_kp = g; else if (d[1] == 1) s->cfg.vel_kp = g; else s->cfg.vel_ki = g; }
        break;
    case 0x40: can_proto_telemetry(c, s, &out[0], &out[1]); n = 2; break;
    case 0x7F: servo_clear_fault(s); break;
    default: status = 2; break;
    }
    if (d[0] != 0x40) {
        out[n].id = 0x080 + c->node; out[n].len = 2;
        out[n].data[0] = d[0]; out[n].data[1] = (uint8_t)status;
        n++;
    }
    return n;
}

int can_proto_tick_ms(can_proto_t *c, const servo_t *s, can_frame_t *out)
{
    if (c->stream_ms == 0) return 0;
    if (c->countdown_ms > 1) { c->countdown_ms--; return 0; }
    c->countdown_ms = c->stream_ms;
    can_proto_telemetry(c, s, &out[0], &out[1]);
    return 2;
}
