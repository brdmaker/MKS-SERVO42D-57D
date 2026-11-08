#include "can_proto.h"
#include "params.h"
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
    c->unlock_ms = 10000;
}

void can_proto_telemetry(const can_proto_t *c, const servo_t *s, can_frame_t *a, can_frame_t *b)
{
    a->id = 0x180 + c->node; a->len = 8;
    put32(a->data, (uint32_t)s->pos);
    put16(a->data + 4, (uint16_t)sat16((float)s->pos_err));
    a->data[6] = (uint8_t)s->state | (uint8_t)(s->mode << 4) | (uint8_t)(c->unlock_left_ms ? 0x40 : 0) | (uint8_t)((s->status & SV_ST_STALL) ? 0x80 : 0);
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
    case 0x30: case 0x31: case 0x32: case 0x33: {
        int want = d[0] == 0x30 ? 7 : (d[0] == 0x32 ? 4 : 3);
        if (in->len < want) { out[n].len = 8; out[n].id = 0x380 + c->node; out[n].data[0] = d[0]; out[n].data[1] = PS_BAD_ARG;
                              memset(out[n].data + 2, 0, 6); n++; return n; }
        uint16_t id = (uint16_t)(d[1] | d[2] << 8);
        uint32_t val = 0; int st = PS_OK; uint8_t sel = 0;
        if (d[0] == 0x30) {
            uint32_t raw = (uint32_t)d[3] | (uint32_t)d[4] << 8 | (uint32_t)d[5] << 16 | (uint32_t)d[6] << 24;
            st = params_set_locked(s, c, id, raw);
            if (st == PS_OK || st == PS_RANGE || st == PS_RUNNING || st == PS_LOCKED) params_get(s, c, id, &val);   /* what is stored now */
        } else if (d[0] == 0x31) {
            st = params_get(s, c, id, &val);
        } else if (d[0] == 0x32) {
            const param_def_t *pd = params_find(id);
            sel = d[3];
            if (!pd) st = PS_UNKNOWN;
            else if (sel == 0) val = (uint32_t)pd->type | (uint32_t)pd->flags << 8;
            else if (sel == 1) memcpy(&val, &pd->min, 4);
            else if (sel == 2) memcpy(&val, &pd->max, 4);
            else st = PS_BAD_ARG;
        } else {
            if (id == 0xFFFF) val = (uint32_t)params_count();
            else { const param_def_t *pd = params_at(id); if (pd) val = pd->id; else st = PS_UNKNOWN; }
        }
        out[n].id = 0x380 + c->node; out[n].len = 8;
        out[n].data[0] = d[0]; out[n].data[1] = (uint8_t)(st | (sel << 4));
        out[n].data[2] = d[1]; out[n].data[3] = d[2];
        put32(out[n].data + 4, val);
        n++;
        return n; }
    case 0x35: {
        uint8_t blob[192];
        size_t len = params_save(s, c, blob, sizeof blob);
        if (!params_unlocked(c)) status = PS_LOCKED;
        else if (!c->save_cb) status = PS_UNAVAIL;
        else if (len == 0 || c->save_cb(blob, (uint16_t)len) != 0) status = 1;
        else c->unlock_left_ms = c->unlock_ms;
        break; }
    case 0x36:
        if (!params_unlocked(c)) status = PS_LOCKED;
        else { status = params_defaults(s, c); if (status == PS_OK) c->unlock_left_ms = c->unlock_ms; }
        break;
    case 0x50: {
        uint32_t key = in->len >= 5 ? ((uint32_t)d[1] | (uint32_t)d[2] << 8 | (uint32_t)d[3] << 16 | (uint32_t)d[4] << 24) : 0;
        if (in->len < 5 || key != SV_UNLOCK_KEY) status = PS_BAD_ARG;
        else if (!params_unlocked(c)) status = PS_LOCKED;
        else if (s->state != SV_OFF && s->state != SV_FAULT) status = PS_RUNNING;
        else if (!c->boot_cb) status = PS_UNAVAIL;
        else { out[n].id = 0x080 + c->node; out[n].len = 2; out[n].data[0] = d[0]; out[n].data[1] = 0; n++;
               c->boot_cb(); return n; }          /* the reply goes out first; on the MCU the callback resets */
        break; }
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
    if (c->unlock_left_ms) c->unlock_left_ms--;
    if (c->stream_ms == 0) return 0;
    if (c->countdown_ms > 1) { c->countdown_ms--; return 0; }
    c->countdown_ms = c->stream_ms;
    can_proto_telemetry(c, s, &out[0], &out[1]);
    return 2;
}
