#include "sim_plant.h"
#include "sim_board.h"
#include "../ctrl/mt6816_proto.h"
#include <math.h>
#include <string.h>

#define TWO_PI 6.283185307179586

static float rnd(sim_t *s)   /* uniform -1..1 */
{
    s->rng ^= s->rng << 13; s->rng ^= s->rng >> 17; s->rng ^= s->rng << 5;
    return (float)(s->rng & 0xFFFFFF) / 8388608.0f - 1.0f;
}
static float gauss(sim_t *s) { return 0.5f * (rnd(s) + rnd(s) + rnd(s) + rnd(s)) * 0.866f; }

void sim_default_motor(sim_motor_t *m)
{
    memset(m, 0, sizeof *m);
    /* 1.8 degree hybrid stepper, typical catalogue values for the board's motor size */
    m->r = SIM_M_R; m->l = SIM_M_L; m->kt = SIM_M_KT; m->pp = 50;
    m->j = SIM_M_J; m->b = SIM_M_B; m->tdet = SIM_M_TDET;
    m->enc_sign = 1;
    m->th = 0.37;                    /* arbitrary start angle */
    m->stop_lo = -1e9; m->stop_hi = 1e9;
}

void sim_default_hw(sim_hw_t *h)
{
    memset(h, 0, sizeof *h);
    h->vin = 24.0f; h->rds = SIM_RDS; h->vf = 0.7f;
    h->amp_tau = 1.0e-6f;
    h->amp_vos_a = 0.012f; h->amp_vos_b = -0.009f;   /* offsets the calibration must remove */
    h->amp_gain_err = 0.01f;
    h->adc_noise_lsb = 1.0f;
    h->adc_hz = 16.0e6; h->adc_smp_cycles = 7.5f; h->adc_conv_cycles = 12.5f;
    h->trig_offset_ticks = -16;                      /* 1 us before the peak */
    h->enc_noise_lsb = 0.3f;
}

void sim_default_cfg(servo_cfg_t *c)
{
    c->r_ohm = SIM_M_R; c->l_h = SIM_M_L; c->kt = SIM_M_KT; c->pole_pairs = 50;
    c->i_max = SIM_I_MAX; c->i_align = SIM_I_ALIGN;
    c->cur_bw_hz = 1000.0f;
    c->pos_kp = 100.0f;
    c->vel_kp = SIM_VEL_KP; c->vel_ki = SIM_VEL_KI;
    c->vel_max = 60.0f;                              /* ~570 rpm */
    c->vbus_min = 10.0f; c->vbus_max = 30.0f;
    c->i_trip = SIM_I_TRIP;
    c->torque_lpf_hz = 200.0f;
    c->detent_nm = SIM_M_TDET; c->detent_phase = 0.0f;     /* matches the model; a real motor needs identifying */
}

void sim_init(sim_t *s)
{
    memset(s, 0, sizeof *s);
    sim_default_motor(&s->m);
    sim_default_hw(&s->hw);
    servo_cfg_t cfg; sim_default_cfg(&cfg);
    servo_init(&s->servo, &cfg);
    can_proto_init(&s->can, 1);
    s->rng = 12345u;
    s->t_isr = -1; s->t_hold_a = s->t_hold_b = s->t_hold_v = -1;
    for (int i = 0; i < 4; i++) s->leg[i].ls = 1;      /* power-up: all low, FETs braking */
}

double sim_time(const sim_t *s) { return (double)s->tick / SIM_TICK_HZ; }
double sim_theta_e(const sim_t *s) { return s->m.th * s->m.pp; }

static uint16_t adc_code(sim_t *s, double v)
{
    double c = v / SV_VDDA * SV_ADC_FULL + gauss(s) * s->hw.adc_noise_lsb;
    c = floor(c + 0.5);
    if (c < 0) c = 0;
    if (c > 4095) c = 4095;
    return (uint16_t)c;
}

static uint16_t enc_sample(sim_t *s)
{
    double rev = s->m.th * s->m.enc_sign / TWO_PI;
    double c = floor(rev * SV_ENC_COUNTS + gauss(s) * s->hw.enc_noise_lsb + 0.5);
    int32_t n = ((int32_t)c) & (SV_ENC_COUNTS - 1);
    return (uint16_t)n;
}

/* The MT6816 SPI frame for the 4-byte transfer after command 0x83. */
static void mt6816_frame(sim_t *s, uint16_t angle, uint8_t rx[4])
{
    rx[0] = 0; rx[3] = 0;
    rx[1] = (uint8_t)(angle >> 6);
    rx[2] = (uint8_t)((angle & 0x3F) << 2) | (s->hw.enc_nomag ? 0x02 : 0);
    uint16_t bits = ((uint16_t)rx[1] << 8 | rx[2]) >> 1;
    bits ^= bits >> 8; bits ^= bits >> 4; bits ^= bits >> 2; bits ^= bits >> 1;
    rx[2] |= (uint8_t)(bits & 1);
    if (s->hw.enc_parity_glitches > 0) { rx[2] ^= 0x01; s->hw.enc_parity_glitches--; }
}

static void motor_step(sim_t *s, double dt, double va, double vb)
{
    sim_motor_t *m = &s->m;
    double th_e = m->th * m->pp;
    double sn = sin(th_e), cs = cos(th_e);
    double ea = -m->kt * m->w * sn, eb = m->kt * m->w * cs;
    m->ia += dt * (va - m->r * m->ia - ea) / m->l;
    m->ib += dt * (vb - m->r * m->ib - eb) / m->l;
    double iq = -m->ia * sn + m->ib * cs;
    double t_el = m->kt * iq;
    double t = t_el - m->tdet * sin(4.0 * th_e) - m->b * m->w;
    double t_ext = m->t_load + m->t_spring * (m->th - m->th_spring0);
    m->t_elec = (float)t_el; m->iq_true = (float)iq; m->t_ext = (float)t_ext;
    if (m->locked) { m->w = 0; return; }
    m->w += dt * (t - t_ext) / m->j;
    m->th += dt * m->w;
    if (m->th > m->stop_hi) { m->th = m->stop_hi; if (m->w > 0) m->w = 0; }
    if (m->th < m->stop_lo) { m->th = m->stop_lo; if (m->w < 0) m->w = 0; }
}

/* One 62.5 ns step of the electronics. */
static void electronics_step(sim_t *s, double *va_out, double *vb_out)
{
    sim_hw_t *h = &s->hw; sim_motor_t *m = &s->m;
    uint32_t p = (uint32_t)(s->tick % SIM_PERIOD);

    if (p == 0) for (int i = 0; i < 4; i++) s->cmp[i] = s->cmp_pending[i];
    uint32_t k = p < SIM_HALF ? p : SIM_PERIOD - p;      /* triangle carrier, 0..HALF */

    double il[4] = { m->ia, -m->ia, m->ib, -m->ib };     /* current out of each leg node */
    double vn[4];
    for (int i = 0; i < 4; i++) {
        sim_leg_t *g = &s->leg[i];
        int in = (int)k < s->cmp[i];                     /* PWM mode 1: high while carrier < compare */
        if (in != g->s_prev) {                           /* EG3013: HIN = LIN tied */
            g->s_prev = in; g->want = in;
            g->hs = g->ls = 0; g->dead = SIM_DEAD;       /* break before make */
        }
        if (g->dead > 0 && --g->dead == 0) { if (g->want) g->hs = 1; else g->ls = 1; }
        if (g->hs)      vn[i] = h->vin - il[i] * h->rds;
        else if (g->ls) vn[i] = -il[i] * h->rds;
        else            vn[i] = il[i] >= 0 ? -h->vf : h->vin + h->vf;   /* body diodes */
    }
    *va_out = vn[SV_LEG_AP] - vn[SV_LEG_AM];
    *vb_out = vn[SV_LEG_BP] - vn[SV_LEG_BM];

    /* shunt currents toward ground: A+ leg carries -ia, B- leg carries +ib */
    sim_leg_t *ga = &s->leg[SV_LEG_AP], *gb = &s->leg[SV_LEG_BM];
    bool ca = ga->ls || (!ga->hs && !ga->ls && il[SV_LEG_AP] >= 0);
    bool cb = gb->ls || (!gb->hs && !gb->ls && il[SV_LEG_BM] >= 0);
    double sa = ca ? -il[SV_LEG_AP] : 0.0;
    double sb = cb ? -il[SV_LEG_BM] : 0.0;
    double g1 = SV_AMP_GAIN * (1.0 + h->amp_gain_err);
    double ta = SV_VREF + g1 * SV_SHUNT_OHM * sa + h->amp_vos_a;
    double tb = SV_VREF + g1 * SV_SHUNT_OHM * sb + h->amp_vos_b;
    ta = ta < 0.02 ? 0.02 : (ta > 3.28 ? 3.28 : ta);
    tb = tb < 0.02 ? 0.02 : (tb > 3.28 ? 3.28 : tb);
    double al = (1.0 / SIM_TICK_HZ) / h->amp_tau;
    s->amp_a += al * (ta - s->amp_a);
    s->amp_b += al * (tb - s->amp_b);

    /* ADC trigger, holds and the control interrupt */
    int64_t trig_p = ((int64_t)SIM_HALF + h->trig_offset_ticks + SIM_PERIOD) % SIM_PERIOD;
    if ((int64_t)p == trig_p) {
        double tk = SIM_TICK_HZ / h->adc_hz;               /* sim ticks per ADC clock */
        double sm = h->adc_smp_cycles * tk, cv = h->adc_conv_cycles * tk;
        s->t_hold_a = (int64_t)s->tick + (int64_t)(sm + 0.5);
        s->t_hold_b = (int64_t)s->tick + (int64_t)(cv + sm + 0.5);
        s->t_hold_v = (int64_t)s->tick + (int64_t)(2 * cv + sm + 0.5);
        s->t_isr    = (int64_t)s->tick + (int64_t)(3 * cv + 0.5) + 16;   /* + 1 us entry latency */
        s->enc_prev = s->enc_now;
        s->enc_now  = enc_sample(s);
    }
    if ((int64_t)s->tick == s->t_hold_a) {
        s->adc_a = adc_code(s, s->amp_a);
        s->st.a_samples++; if (!ca) s->st.a_invalid++;
    }
    if ((int64_t)s->tick == s->t_hold_b) {
        s->adc_b = adc_code(s, s->amp_b);
        s->st.b_samples++; if (!cb) s->st.b_invalid++;
    }
    if ((int64_t)s->tick == s->t_hold_v) s->adc_v = adc_code(s, h->vin / ((15.4 + 1.0) / 1.0));
    if ((int64_t)s->tick == s->t_isr) {
        uint8_t rx[4]; uint16_t ang; mt6816_frame(s, s->enc_prev, rx);
        uint8_t fl = mt6816_decode(rx, &ang);
        servo_out_t out;
        servo_fast(&s->servo, s->adc_a, s->adc_b, s->adc_v, ang, fl, &out);
        s->st.isr_count++;
        s->drive = out.drive;
        for (int i = 0; i < 4; i++)
            s->cmp_pending[i] = out.drive ? (int)lrintf(out.duty[i] * SIM_HALF) : 0;
    }
}

static void can_ms(sim_t *s)
{
    can_frame_t out[4];
    for (int i = 0; i < s->to_drive_n; i++) {
        int n = can_proto_rx(&s->can, &s->servo, &s->to_drive[i], out);
        for (int j = 0; j < n && s->from_drive_n < 256; j++) {
            int idx = (s->from_drive_head + s->from_drive_n) % 256;
            s->from_drive[idx] = out[j]; s->can_rx_time_ticks[idx] = s->tick; s->from_drive_n++;
        }
    }
    s->to_drive_n = 0;
    int n = can_proto_tick_ms(&s->can, &s->servo, out);
    for (int j = 0; j < n && s->from_drive_n < 256; j++) {
        int idx = (s->from_drive_head + s->from_drive_n) % 256;
        s->from_drive[idx] = out[j]; s->can_rx_time_ticks[idx] = s->tick; s->from_drive_n++;
    }
}

void sim_can_send(sim_t *s, const can_frame_t *f)
{
    if (s->to_drive_n < 256) s->to_drive[s->to_drive_n++] = *f;
}
bool sim_can_recv(sim_t *s, can_frame_t *f)
{
    if (s->from_drive_n == 0) return false;
    *f = s->from_drive[s->from_drive_head];
    s->from_drive_head = (s->from_drive_head + 1) % 256; s->from_drive_n--;
    return true;
}
void sim_can_cmd(sim_t *s, uint8_t cmd, const void *arg, int arglen)
{
    can_frame_t f; memset(&f, 0, sizeof f);
    f.id = 0x100 + s->can.node; f.len = 1 + arglen; f.data[0] = cmd;
    if (arglen) memcpy(f.data + 1, arg, arglen);
    sim_can_send(s, &f);
}

void sim_run(sim_t *s, double seconds)
{
    uint64_t end = s->tick + (uint64_t)(seconds * SIM_TICK_HZ + 0.5);
    double va_sum = 0, vb_sum = 0; int cnt = 0;
    while (s->tick < end) {
        double va, vb;
        electronics_step(s, &va, &vb);
        va_sum += va; vb_sum += vb;
        if (++cnt == SIM_MOT_DIV) {
            motor_step(s, SIM_MOT_DIV / SIM_TICK_HZ, va_sum / cnt, vb_sum / cnt);
            if (fabsf((float)s->m.ia) > s->st.ia_peak) s->st.ia_peak = (float)fabs(s->m.ia);
            if (fabsf((float)s->m.ib) > s->st.ib_peak) s->st.ib_peak = (float)fabs(s->m.ib);
            va_sum = vb_sum = 0; cnt = 0;
        }
        s->tick++;
        if (s->tick % SIM_TICKS_MS == 0) {
            can_ms(s);
            if (s->trace) fprintf(s->trace, "%.4f,%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d\n",
                sim_time(s), s->servo.pos, s->servo.vel, s->servo.iq_filt,
                s->servo.torque_nm, s->m.t_elec, s->m.t_ext, s->m.ia, s->m.ib, s->servo.state);
        }
    }
}
