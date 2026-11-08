#include "params.h"
#include <string.h>
#include <math.h>

#define S(f)  offsetof(servo_t, f)
#define SC(f) offsetof(servo_t, cfg.f)
#define C(f)  offsetof(can_proto_t, f)
#define FS    (0.9f * SV_I_FULLSCALE)

/* Ids are part of the protocol: never reuse or renumber one. */
static const param_def_t table[] = {
    /* identity */
    { 0x0001, "fw_version",   "",     PT_I32, 0, S(fw_version), 0, 0, PF_RO },
    { 0x0002, "board_id",     "",     PT_I32, 0, S(board_id),   0, 0, PF_RO },
    /* state and measurements */
    { 0x0100, "state",        "",     PT_I32, 0, S(state),      0, 0, PF_RO },
    { 0x0101, "fault",        "bits", PT_I32, 0, S(fault),      0, 0, PF_RO },
    { 0x0102, "status",       "bits", PT_I32, 0, S(status),     0, 0, PF_RO },
    { 0x0103, "pos",          "cnt",  PT_I32, 0, S(pos),        0, 0, PF_RO },
    { 0x0104, "pos_err",      "cnt",  PT_I32, 0, S(pos_err),    0, 0, PF_RO },
    { 0x0105, "vel",          "rad/s",PT_F32, 0, S(vel),        0, 0, PF_RO },
    { 0x0106, "ia",           "A",    PT_F32, 0, S(ia),         0, 0, PF_RO },
    { 0x0107, "ib",           "A",    PT_F32, 0, S(ib),         0, 0, PF_RO },
    { 0x0108, "id",           "A",    PT_F32, 0, S(id),         0, 0, PF_RO },
    { 0x0109, "iq",           "A",    PT_F32, 0, S(iq),         0, 0, PF_RO },
    { 0x010A, "iq_filt",      "A",    PT_F32, 0, S(iq_filt),    0, 0, PF_RO },
    { 0x010B, "torque",       "Nm",   PT_F32, 0, S(torque_nm),  0, 0, PF_RO },
    { 0x010C, "vbus",         "V",    PT_F32, 0, S(vbus),       0, 0, PF_RO },
    { 0x010D, "offset_a",     "LSB",  PT_F32, 0, S(off_a),      0, 0, PF_RO },
    { 0x010E, "offset_b",     "LSB",  PT_F32, 0, S(off_b),      0, 0, PF_RO },
    { 0x010F, "theta_off",    "rad",  PT_F32, 0, S(theta_off),  0, 0, PF_RO },
    { 0x0110, "enc_dir",      "",     PT_I32, 0, S(enc_dir),    0, 0, PF_RO },
    { 0x0111, "iq_ref",       "A",    PT_F32, 0, S(iq_ref),     0, 0, PF_RO },
    /* commands (the same as the command frames) */
    { 0x0200, "mode",         "",     PT_I32, 0, S(mode),       0, 2, 0 },
    { 0x0201, "cmd_iq",       "A",    PT_F32, 0, S(cmd_iq),     -FS, FS, 0 },
    { 0x0202, "cmd_vel",      "rad/s",PT_F32, 0, S(cmd_vel),    -1000, 1000, 0 },
    { 0x0203, "cmd_pos",      "cnt",  PT_I32, 0, S(cmd_pos),    -2e9f, 2e9f, 0 },
    { 0x0204, "ilim",         "A",    PT_F32, 0, S(ilim),       0, FS, 0 },
    /* configuration: saved */
    { 0x0300, "motor_r",      "ohm",  PT_F32, 0, SC(r_ohm),     0.05f, 50.0f, PF_OFF_ONLY | PF_PERSIST },
    { 0x0301, "motor_l",      "H",    PT_F32, 0, SC(l_h),       1e-5f, 0.1f,  PF_OFF_ONLY | PF_PERSIST },
    { 0x0302, "motor_kt",     "Nm/A", PT_F32, 0, SC(kt),        0.005f, 5.0f, PF_PERSIST },
    { 0x0303, "pole_pairs",   "",     PT_I32, 0, SC(pole_pairs),1, 200,       PF_OFF_ONLY | PF_PERSIST },
    { 0x0304, "i_max",        "A",    PT_F32, 0, SC(i_max),     0, FS,        PF_PERSIST },
    { 0x0305, "i_align",      "A",    PT_F32, 0, SC(i_align),   0, FS,        PF_OFF_ONLY | PF_PERSIST },
    { 0x0306, "cur_bw",       "Hz",   PT_F32, 0, SC(cur_bw_hz), 50, 3000,     PF_OFF_ONLY | PF_PERSIST },
    { 0x0307, "pos_kp",       "1/s",  PT_F32, 0, SC(pos_kp),    0, 1000,      PF_PERSIST },
    { 0x0308, "vel_kp",       "A/(rad/s)", PT_F32, 0, SC(vel_kp), 0, 10,      PF_PERSIST },
    { 0x0309, "vel_ki",       "A/rad",PT_F32, 0, SC(vel_ki),    0, 1000,      PF_PERSIST },
    { 0x030A, "vel_max",      "rad/s",PT_F32, 0, SC(vel_max),   0, 1000,      PF_PERSIST },
    { 0x030B, "vbus_min",     "V",    PT_F32, 0, SC(vbus_min),  5, 40,        PF_PERSIST },
    { 0x030C, "vbus_max",     "V",    PT_F32, 0, SC(vbus_max),  8, 40,        PF_PERSIST },
    { 0x030D, "i_trip",       "A",    PT_F32, 0, SC(i_trip),    0.1f, FS,     PF_PERSIST },
    { 0x030E, "torque_lpf",   "Hz",   PT_F32, 0, SC(torque_lpf_hz), 1, 2000,  PF_PERSIST },
    { 0x030F, "detent_nm",    "Nm",   PT_F32, 0, SC(detent_nm), 0, 5,         PF_PERSIST },
    { 0x0310, "detent_phase", "rad",  PT_F32, 0, SC(detent_phase), -7, 7,     PF_PERSIST },
    /* CAN */
    { 0x0400, "can_node",     "",     PT_U8,  1, C(node),       1, 127,       PF_PERSIST },
    { 0x0401, "stream_ms",    "ms",   PT_U16, 1, C(stream_ms),  0, 60000,     PF_PERSIST },
};
#define NPARAM ((int)(sizeof table / sizeof table[0]))

int params_count(void) { return NPARAM; }
const param_def_t *params_at(int i) { return (i >= 0 && i < NPARAM) ? &table[i] : NULL; }
const param_def_t *params_find(uint16_t id)
{
    for (int i = 0; i < NPARAM; i++) if (table[i].id == id) return &table[i];
    return NULL;
}

static void *where(const param_def_t *d, servo_t *s, can_proto_t *c)
{
    return (uint8_t *)(d->target ? (void *)c : (void *)s) + d->off;
}

static uint32_t fetch(const param_def_t *d, const void *p)
{
    uint32_t v = 0;
    switch (d->type) {
    case PT_F32: case PT_I32: memcpy(&v, p, 4); break;
    case PT_U16: { uint16_t x; memcpy(&x, p, 2); v = x; } break;
    case PT_U8:  v = *(const uint8_t *)p; break;
    }
    return v;
}

int params_get(const servo_t *s, const can_proto_t *c, uint16_t id, uint32_t *raw)
{
    const param_def_t *d = params_find(id);
    if (!d) return PS_UNKNOWN;
    *raw = fetch(d, where(d, (servo_t *)s, (can_proto_t *)c));
    return PS_OK;
}

static bool is_off(const servo_t *s) { return s->state == SV_OFF || s->state == SV_FAULT; }

int params_set(servo_t *s, can_proto_t *c, uint16_t id, uint32_t raw)
{
    const param_def_t *d = params_find(id);
    if (!d) return PS_UNKNOWN;
    if (d->flags & PF_RO) return PS_READ_ONLY;
    if ((d->flags & PF_OFF_ONLY) && !is_off(s)) return PS_RUNNING;
    double v;
    if (d->type == PT_F32) {
        float f; memcpy(&f, &raw, 4);
        if (!isfinite(f)) return PS_BAD_ARG;
        v = f;
    } else {
        v = (double)(int32_t)raw;
    }
    if (v < d->min || v > d->max) return PS_RANGE;

    /* relations between parameters */
    if (id == 0x0204 && v > s->cfg.i_max) return PS_RANGE;       /* the runtime limit cannot exceed i_max */
    if (id == 0x030B && !(v < s->cfg.vbus_max)) return PS_RANGE;
    if (id == 0x030C && !(v > s->cfg.vbus_min)) return PS_RANGE;

    void *p = where(d, s, c);
    switch (d->type) {
    case PT_F32: { float f = (float)v; memcpy(p, &f, 4); } break;
    case PT_I32: { int32_t x = (int32_t)v; memcpy(p, &x, 4); } break;
    case PT_U16: { uint16_t x = (uint16_t)v; memcpy(p, &x, 2); } break;
    case PT_U8:  *(uint8_t *)p = (uint8_t)v; break;
    }
    /* side effects */
    switch (id) {
    case 0x0200: { sv_mode_t m = (sv_mode_t)(int)v; s->mode = (sv_mode_t)(-1); servo_set_mode(s, m); } break;
    case 0x0204: servo_set_current_limit(s, (float)v); break;
    case 0x0300: case 0x0301: case 0x0306: servo_recalc(s); break;
    case 0x0304: if (s->ilim > s->cfg.i_max) s->ilim = s->cfg.i_max; break;
    case 0x0401: c->countdown_ms = c->stream_ms; break;
    default: break;
    }
    return PS_OK;
}

int params_defaults(servo_t *s, can_proto_t *c)
{
    (void)c;
    if (!is_off(s)) return PS_RUNNING;
    s->cfg = s->cfg_default;
    s->ilim = s->cfg.i_max;
    servo_recalc(s);
    return PS_OK;
}

/* ---- saved set ---- */
#define BLOB_MAGIC 0x5337u
static uint16_t crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)*p++ << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

size_t params_save(const servo_t *s, const can_proto_t *c, uint8_t *buf, size_t cap)
{
    size_t n = 0;
    int cnt = 0;
    for (int i = 0; i < NPARAM; i++) if (table[i].flags & PF_PERSIST) cnt++;
    size_t need = 4 + (size_t)cnt * 6 + 2;
    if (cap < need) return 0;
    buf[n++] = BLOB_MAGIC & 0xFF; buf[n++] = BLOB_MAGIC >> 8;
    buf[n++] = SV_FW_VERSION; buf[n++] = (uint8_t)cnt;
    for (int i = 0; i < NPARAM; i++) {
        if (!(table[i].flags & PF_PERSIST)) continue;
        uint32_t raw = fetch(&table[i], where(&table[i], (servo_t *)s, (can_proto_t *)c));
        buf[n++] = table[i].id & 0xFF; buf[n++] = table[i].id >> 8;
        for (int k = 0; k < 4; k++) buf[n++] = (raw >> (8 * k)) & 0xFF;
    }
    uint16_t crc = crc16(buf, n);
    buf[n++] = crc & 0xFF; buf[n++] = crc >> 8;
    return n;
}

int params_load(servo_t *s, can_proto_t *c, const uint8_t *buf, size_t len)
{
    if (len < 6) return -1;
    uint16_t crc = (uint16_t)(buf[len - 2] | buf[len - 1] << 8);
    if (crc != crc16(buf, len - 2)) return -1;
    if ((uint16_t)(buf[0] | buf[1] << 8) != BLOB_MAGIC) return -1;
    size_t cnt = buf[3];
    if (len != 4 + cnt * 6 + 2) return -1;
    int applied = 0;
    if (!is_off(s) || cnt > 64) return -1;               /* the drive must be off to load */
    uint8_t done[64] = { 0 };
    /* two passes: a value rejected only because of its partner (vbus_min / vbus_max) fits the second time */
    for (int pass = 0; pass < 2; pass++) {
        for (size_t i = 0; i < cnt; i++) {
            if (done[i]) continue;
            const uint8_t *e = buf + 4 + i * 6;
            uint16_t id = (uint16_t)(e[0] | e[1] << 8);
            uint32_t raw = (uint32_t)e[2] | (uint32_t)e[3] << 8 | (uint32_t)e[4] << 16 | (uint32_t)e[5] << 24;
            const param_def_t *d = params_find(id);
            if (!d || !(d->flags & PF_PERSIST)) { done[i] = 1; continue; }
            if (params_set(s, c, id, raw) == PS_OK) { done[i] = 1; applied++; }
        }
    }
    return applied;
}
