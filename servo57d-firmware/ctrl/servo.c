#include "servo.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define TWO_PI        6.2831853f
#define M_PI_F        3.1415927f
#define CAL_SETTLE    16
#define CAL_N         128
#define ALIGN_RAMP    2000          /* 0.1 s */
#define ALIGN_SETTLE  6000          /* until 0.3 s */
#define ALIGN_AVG     6000          /* average over 0.3 s */
#define DIR_SETTLE    4000
#define DIR_AVG       4000
#define RAMP_P        4000          /* the vector turns 0 -> +90 degrees in 0.2 s, rotor following */
#define RAMP_N        8000          /* then +90 -> -90 degrees in 0.4 s */
#define PHASE_TOL     0.15f         /* of the expected step (a quarter electrical turn) */
#define STALL_TICKS   4000          /* 0.2 s at the torque limit */
#define ENC_FAULT_RUN 3
#define CAL_STILL     2             /* counts the shaft may move during CAL */
#define CAL_TIMEOUT   60000         /* 3 s without a standstill -> fault */

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline int wrap14(int d) { d &= (SV_ENC_COUNTS - 1); return d >= SV_ENC_COUNTS / 2 ? d - SV_ENC_COUNTS : d; }

void servo_init(servo_t *s, const servo_cfg_t *cfg)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    /* the trip level must lie inside the range the amplifier can show */
    if (s->cfg.i_trip > 0.9f * SV_I_FULLSCALE) s->cfg.i_trip = 0.9f * SV_I_FULLSCALE;
    s->state = SV_OFF;
    s->mode = SV_MODE_POSITION;
    s->ilim = cfg->i_max;
    s->enc_dir = 1;
    s->off_a = s->off_b = SV_VREF / SV_VDDA * SV_ADC_FULL;
    s->vbus = 0;
    float wc = TWO_PI * cfg->cur_bw_hz;
    s->kp_i = cfg->l_h * wc;                 /* pole-zero cancelling PI */
    s->ki_i = cfg->r_ohm * wc;
}

void servo_enable(servo_t *s, bool on)
{
    if (!on) {
        if (s->state != SV_FAULT) s->state = SV_OFF;
        s->int_d = s->int_q = s->int_vel = 0;
        return;
    }
    if (s->state == SV_OFF) {
        s->state = SV_CAL;
        s->phase_tick = 0;
        s->cal_total = 0;
        s->acc_a = s->acc_b = 0;
        s->int_d = s->int_q = s->int_vel = 0;
        s->status = 0;
    }
}

void servo_clear_fault(servo_t *s)
{
    s->fault = 0;
    s->par_err = 0;
    if (s->state == SV_FAULT) s->state = SV_OFF;
}

void servo_set_mode(servo_t *s, sv_mode_t m)
{
    if (m == s->mode) return;
    s->mode = m;
    s->int_vel = 0;
    if (m == SV_MODE_POSITION) s->cmd_pos = s->pos;
    if (m == SV_MODE_VELOCITY) s->cmd_vel = 0;
    if (m == SV_MODE_TORQUE)   s->cmd_iq = 0;
}
void servo_set_torque(servo_t *s, float iq_a)   { s->cmd_iq = iq_a; }
void servo_set_velocity(servo_t *s, float w)    { s->cmd_vel = w; }
void servo_set_position(servo_t *s, int32_t c)  { s->cmd_pos = c; }
void servo_set_current_limit(servo_t *s, float a) { s->ilim = clampf(a, 0.0f, s->cfg.i_max); }

static void trip(servo_t *s, uint32_t f)
{
    s->fault |= f;
    s->state = SV_FAULT;
}

/* velocity PI -> q current reference, 2 kHz */
static float vel_pi(servo_t *s, float vref)
{
    const float ts = SV_TS * SV_SLOW_DIV;
    float e = vref - s->vel;
    float in = s->int_vel + s->cfg.vel_ki * e * ts;
    float u = s->cfg.vel_kp * e + in;
    float lim = s->ilim;
    if (u > lim)       u = lim;
    else if (u < -lim) u = -lim;
    else               s->int_vel = in;       /* conditional integration */
    return u;
}

static void slow(servo_t *s)
{
    const float ts = SV_TS * SV_SLOW_DIV;
    float dpos = (float)(s->pos - s->pos_prev);
    s->pos_prev = s->pos;
    float v_raw = dpos * (TWO_PI / SV_ENC_COUNTS) / ts;
    s->vel += 0.25f * (v_raw - s->vel);

    if (s->state != SV_RUN) { s->iq_ref = 0; return; }

    float iq;
    if (s->mode == SV_MODE_TORQUE) {
        iq = clampf(s->cmd_iq, -s->ilim, s->ilim);
    } else {
        float vref;
        if (s->mode == SV_MODE_POSITION) {
            s->pos_err = s->cmd_pos - s->pos;
            vref = clampf(s->cfg.pos_kp * (float)s->pos_err * (TWO_PI / SV_ENC_COUNTS),
                          -s->cfg.vel_max, s->cfg.vel_max);
        } else {
            s->pos_err = 0;
            vref = clampf(s->cmd_vel, -s->cfg.vel_max, s->cfg.vel_max);
        }
        iq = vel_pi(s, vref);
    }
    s->iq_ref = iq;

    /* stall: at the limit and still far from the target */
    if (s->mode == SV_MODE_POSITION && fabsf(iq) >= 0.99f * s->ilim && labs(s->pos_err) > 200) {
        if (++s->stall_ticks > STALL_TICKS / SV_SLOW_DIV) s->status |= SV_ST_STALL;
    } else {
        s->stall_ticks = 0;
        s->status &= ~SV_ST_STALL;
    }
}

/* Phasing result from three equilibria: the rotor at the 0, +90 and -90 degree vectors.
 * A free shaft moves E = a quarter electrical turn between them. A step counts only if it
 * is within PHASE_TOL of E and of the right sign, so a shaft held by an endstop can still
 * be phased from the free direction, and a stopped or disturbed one is refused instead of
 * giving a wrong encoder direction (which would be positive feedback). */
static bool phase_decide(servo_t *s)
{
    float E = (float)SV_ENC_COUNTS / (4.0f * (float)s->cfg.pole_pairs);
    float dp = s->dir_mean - s->align_mean, dm = s->neg_mean - s->align_mean;
    for (int d = 1; d >= -1; d -= 2) {
        bool vp = fabsf(dp - d * E) < PHASE_TOL * E;
        bool vm = fabsf(dm + d * E) < PHASE_TOL * E;
        if (!vp && !vm) continue;
        if (!vp && (float)d * dp < -PHASE_TOL * E) return false;      /* moved the wrong way */
        if (!vm && (float)d * dm >  PHASE_TOL * E) return false;
        float sum = 0.0f; int n = 0;
        if (vp) { sum += s->dir_mean - d * E; n++; }
        if (vm) { sum += s->neg_mean + d * E; n++; }
        if (vp && vm) { sum += s->align_mean; n++; }   /* the align point is trusted only if both agree */
        float raw0 = fmodf(sum / (float)n, (float)SV_ENC_COUNTS);
        float te = fmodf((float)d * raw0 * (float)s->cfg.pole_pairs, (float)SV_ENC_COUNTS);
        s->enc_dir = d;
        s->theta_off = -te * (TWO_PI / SV_ENC_COUNTS);
        return true;
    }
    return false;
}

void servo_fast(servo_t *s, uint16_t adc_a, uint16_t adc_b, uint16_t adc_vbus,
                uint16_t enc14, uint8_t enc_flags, servo_out_t *out)
{
    /* ---- measurements ---- */
    float vb_new = (float)adc_vbus * SV_VBUS_PER_LSB;
    s->vbus = (s->vbus == 0.0f) ? vb_new : s->vbus + 0.02f * (vb_new - s->vbus);

    /* encoder: multi-turn tracking; a bad sample is skipped */
    bool enc_ok = !(enc_flags & (SV_ENCF_NOMAG | SV_ENCF_PARITY));
    if (enc_flags & SV_ENCF_PARITY) s->par_err++; else s->par_err = 0;
    if ((enc_flags & SV_ENCF_NOMAG) && s->state != SV_OFF && s->state != SV_FAULT) trip(s, SV_FLT_NOMAG);
    if (s->par_err > ENC_FAULT_RUN && s->state != SV_OFF && s->state != SV_FAULT) trip(s, SV_FLT_ENCODER);
    if (enc_ok) {
        if (s->enc_valid) s->pos_raw += wrap14((int)enc14 - (int)s->enc_last);
        s->enc_last = enc14;
        s->enc_valid = true;
    }
    s->pos = s->enc_dir * s->pos_raw;

    s->ia = SV_SENSE_SIGN_A * ((float)adc_a - s->off_a) * SV_AMPS_PER_LSB;
    s->ib = SV_SENSE_SIGN_B * ((float)adc_b - s->off_b) * SV_AMPS_PER_LSB;

    bool active = s->state == SV_ALIGN || s->state == SV_PHASE_DIR || s->state == SV_PHASE_NEG || s->state == SV_RUN;
    if (active) {
        if (fabsf(s->ia) > s->cfg.i_trip || fabsf(s->ib) > s->cfg.i_trip) trip(s, SV_FLT_OVERCURRENT);
        if (s->vbus < s->cfg.vbus_min) trip(s, SV_FLT_UNDERVOLT);
        if (s->vbus > s->cfg.vbus_max) trip(s, SV_FLT_OVERVOLT);
    }

    if (++s->slow_cnt >= SV_SLOW_DIV) { s->slow_cnt = 0; slow(s); }

    out->drive = false;
    for (int i = 0; i < 4; i++) out->duty[i] = 0.0f;

    if (s->state == SV_OFF || s->state == SV_FAULT) {
        s->int_d = s->int_q = 0;
        return;
    }

    /* ---- offset calibration: legs at 50 %, no coil voltage ---- */
    if (s->state == SV_CAL) {
        out->drive = true;
        for (int i = 0; i < 4; i++) out->duty[i] = 0.5f;
        if (s->phase_tick == 0) s->cal_pos0 = s->pos_raw;
        if (labs(s->pos_raw - s->cal_pos0) > CAL_STILL) {     /* shaft moving: back-EMF would bias the offsets */
            s->phase_tick = 0; s->acc_a = s->acc_b = 0;
            if (++s->cal_total > CAL_TIMEOUT) trip(s, SV_FLT_PHASING);
            return;
        }
        if (++s->cal_total > CAL_TIMEOUT) { trip(s, SV_FLT_PHASING); return; }
        if (s->phase_tick >= CAL_SETTLE) { s->acc_a += adc_a; s->acc_b += adc_b; }
        if (++s->phase_tick >= CAL_SETTLE + CAL_N) {
            s->off_a = s->acc_a / CAL_N;
            s->off_b = s->acc_b / CAL_N;
            s->state = SV_ALIGN;
            s->phase_tick = 0;
            s->ph_n = 0; s->ph_sum = 0;
        }
        return;
    }

    /* ---- current vector and angle for this state ---- */
    float theta, cmd_d, cmd_q;
    bool use_ff = false;
    s->iq_ff = 0.0f;
    switch (s->state) {
    case SV_ALIGN: {
        theta = 0.0f;
        float r = (float)s->phase_tick / ALIGN_RAMP;
        cmd_d = s->cfg.i_align * (r > 1.0f ? 1.0f : r);
        cmd_q = 0.0f;
        break; }
    case SV_PHASE_DIR:
    case SV_PHASE_NEG: {
        /* A slowly turning vector: the rotor follows it or is held by something, it cannot
         * jump to a different equilibrium, so a reading cannot alias into the wrong direction. */
        float phi;
        if (s->state == SV_PHASE_DIR) {
            float r = (float)s->phase_tick / RAMP_P; if (r > 1.0f) r = 1.0f;
            phi = 0.5f * (float)M_PI_F * r;
        } else {
            float r = (float)s->phase_tick / RAMP_N; if (r > 1.0f) r = 1.0f;
            phi = 0.5f * (float)M_PI_F * (1.0f - 2.0f * r);
        }
        theta = 0.0f; cmd_d = s->cfg.i_align * cosf(phi); cmd_q = s->cfg.i_align * sinf(phi);
        break; }
    default: {
        int32_t te = ((int32_t)(s->enc_dir * (int32_t)enc14 * s->cfg.pole_pairs)) & (SV_ENC_COUNTS - 1);
        float we = s->vel * (float)s->cfg.pole_pairs;
        theta = (float)te * (TWO_PI / SV_ENC_COUNTS) + s->theta_off + we * 1.5f * SV_TS;
        cmd_d = 0.0f;
        if (s->cfg.detent_nm != 0.0f)
            s->iq_ff = s->cfg.detent_nm / s->cfg.kt * sinf(4.0f * theta + s->cfg.detent_phase);
        cmd_q = clampf(clampf(s->iq_ref, -s->ilim, s->ilim) + s->iq_ff, -s->cfg.i_max, s->cfg.i_max);
        use_ff = true;
        break; }
    }
    float c = cosf(theta), sn = sinf(theta);
    s->id =  s->ia * c + s->ib * sn;
    s->iq = -s->ia * sn + s->ib * c;
    float lp = TWO_PI * s->cfg.torque_lpf_hz * SV_TS;
    s->iq_filt    += lp * (s->iq - s->iq_filt);
    s->iq_ff_filt += lp * (s->iq_ff - s->iq_ff_filt);
    s->torque_nm = s->cfg.kt * (s->iq_filt - s->iq_ff_filt);

    /* ---- dq current PI with decoupling and back-EMF feed-forward ---- */
    float ed = cmd_d - s->id, eq = cmd_q - s->iq;
    float nid = s->int_d + s->ki_i * ed * SV_TS;
    float niq = s->int_q + s->ki_i * eq * SV_TS;
    float vd = s->kp_i * ed + nid;
    float vq = s->kp_i * eq + niq;
    if (use_ff) {
        float we = s->vel * (float)s->cfg.pole_pairs;
        vd -= we * s->cfg.l_h * s->iq;
        vq += we * s->cfg.l_h * s->id + s->cfg.kt * s->vel;
    }
    float vbus = s->vbus > 1.0f ? s->vbus : 1.0f;
    float vmax = (SV_DUTY_MAX - SV_DUTY_MIN) * vbus;      /* per coil */
    float vm = sqrtf(vd * vd + vq * vq);
    if (vm > vmax) {
        float k = vmax / vm;
        vd *= k; vq *= k;                                  /* saturated: freeze integrators */
    } else {
        s->int_d = nid; s->int_q = niq;
    }
    float va = vd * c - vq * sn;
    float vb = vd * sn + vq * c;
    float xa = va / (2.0f * vbus), xb = vb / (2.0f * vbus);
    out->duty[SV_LEG_AP] = clampf(0.5f + xa, SV_DUTY_MIN, SV_DUTY_MAX);
    out->duty[SV_LEG_AM] = clampf(0.5f - xa, SV_DUTY_MIN, SV_DUTY_MAX);
    out->duty[SV_LEG_BP] = clampf(0.5f + xb, SV_DUTY_MIN, SV_DUTY_MAX);
    out->duty[SV_LEG_BM] = clampf(0.5f - xb, SV_DUTY_MIN, SV_DUTY_MAX);
    out->drive = true;

    /* ---- phasing sequencer ---- */
    s->phase_tick++;
    if (s->state == SV_ALIGN) {
        if (s->phase_tick > ALIGN_SETTLE) {
            if (s->ph_n == 0) s->ph_ref = s->pos_raw;
            s->ph_sum += s->pos_raw - s->ph_ref; s->ph_n++;
        }
        if (s->phase_tick >= ALIGN_SETTLE + ALIGN_AVG) {
            s->align_mean = (float)s->ph_ref + (float)s->ph_sum / (float)s->ph_n;
            s->state = SV_PHASE_DIR; s->phase_tick = 0; s->ph_n = 0; s->ph_sum = 0;
            s->int_d = s->int_q = 0;
        }
    } else if (s->state == SV_PHASE_DIR || s->state == SV_PHASE_NEG) {
        uint32_t pre = s->state == SV_PHASE_NEG ? RAMP_N : RAMP_P;
        if (s->phase_tick > pre + DIR_SETTLE) {
            if (s->ph_n == 0) s->ph_ref = s->pos_raw;
            s->ph_sum += s->pos_raw - s->ph_ref; s->ph_n++;
        }
        if (s->phase_tick >= pre + DIR_SETTLE + DIR_AVG) {
            float mean = (float)s->ph_ref + (float)s->ph_sum / (float)s->ph_n;
            s->phase_tick = 0; s->ph_n = 0; s->ph_sum = 0;
            s->int_d = s->int_q = 0;
            if (s->state == SV_PHASE_DIR) {
                s->dir_mean = mean;
                s->state = SV_PHASE_NEG;
                return;
            }
            s->neg_mean = mean;
            if (!phase_decide(s)) { trip(s, SV_FLT_PHASING); return; }
            s->pos = s->enc_dir * s->pos_raw;
            s->int_d = s->int_q = s->int_vel = 0;
            s->vel = 0; s->pos_prev = s->pos; s->slow_cnt = 0;
            s->cmd_pos = s->pos; s->cmd_vel = 0; s->cmd_iq = 0; s->iq_ref = 0;
            s->iq_filt = 0; s->iq_ff_filt = 0;
            s->state = SV_RUN;
        }
    }
}
