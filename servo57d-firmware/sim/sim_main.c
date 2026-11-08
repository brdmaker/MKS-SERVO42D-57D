/* Regression scenarios for the control core on the virtual MKS SERVO57D.
 * `make check` runs them all; `./sim_test --demo trace.csv` writes a trace. */
#include "sim_plant.h"
#include "sim_board.h"
#include "../ctrl/mt6816_proto.h"
#include "../ctrl/params.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails, checks;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define INFO(...) do { printf("  "); printf(__VA_ARGS__); printf("\n"); } while (0)
#define TWO_PI 6.283185307179586
#define TL     ((double)SIM_T_LIM)               /* torque the drive can hold, Nm */
#define ISTEP  ((double)SIM_I_MAX * 2.0 / 3.0)   /* current step of the loop test, A */

static sim_t S;

/* ---- helpers ---- */
static void put_i16(uint8_t *d, int v) { d[0] = v & 0xFF; d[1] = (v >> 8) & 0xFF; }
static void put_i32(uint8_t *d, int32_t v) { for (int i = 0; i < 4; i++) d[i] = ((uint32_t)v >> (8 * i)) & 0xFF; }

static void cmd_enable(sim_t *s, int on) { uint8_t a = on; sim_can_cmd(s, 0x01, &a, 1); }
static void cmd_mode(sim_t *s, int m) { uint8_t a = m; sim_can_cmd(s, 0x02, &a, 1); }
static void cmd_torque_ma(sim_t *s, int ma) { uint8_t a[2]; put_i16(a, ma); sim_can_cmd(s, 0x10, a, 2); }
static void cmd_vel(sim_t *s, int dpm) { uint8_t a[4]; put_i32(a, dpm); sim_can_cmd(s, 0x11, a, 4); }
static void cmd_pos(sim_t *s, int32_t c) { uint8_t a[4]; put_i32(a, c); sim_can_cmd(s, 0x12, a, 4); }
static void cmd_stream(sim_t *s, int ms) { uint8_t a[2]; put_i16(a, ms); sim_can_cmd(s, 0x21, a, 2); }

typedef struct { int32_t pos; int pos_err; int state, mode, stall, fault;
                 int iq_ma, torque_mnm, vel_01rpm, vbus_10mv; bool have_a, have_b; } tlm_t;

static void drain(sim_t *s, tlm_t *t)
{
    can_frame_t f;
    while (sim_can_recv(s, &f)) {
        if (f.id == 0x181) {
            t->pos = (int32_t)((uint32_t)f.data[0] | f.data[1] << 8 | (uint32_t)f.data[2] << 16 | (uint32_t)f.data[3] << 24);
            t->pos_err = (int16_t)(f.data[4] | f.data[5] << 8);
            t->state = f.data[6] & 7; t->mode = (f.data[6] >> 4) & 7; t->stall = f.data[6] >> 7;
            t->fault = f.data[7]; t->have_a = true;
        } else if (f.id == 0x281) {
            t->iq_ma = (int16_t)(f.data[0] | f.data[1] << 8);
            t->torque_mnm = (int16_t)(f.data[2] | f.data[3] << 8);
            t->vel_01rpm = (int16_t)(f.data[4] | f.data[5] << 8);
            t->vbus_10mv = f.data[6] | f.data[7] << 8; t->have_b = true;
        }
    }
}

static bool run_until_state(sim_t *s, sv_state_t st, double timeout)
{
    for (double t = 0; t < timeout; t += 0.01) {
        sim_run(s, 0.01);
        if (s->servo.state == st) return true;
        if (s->servo.state == SV_FAULT) return false;
    }
    return false;
}

static void fresh(void)
{
    sim_init(&S);
}

static bool start_run(sim_t *s)
{
    cmd_enable(s, 1);
    return run_until_state(s, SV_RUN, 4.0);
}

static double enc_err_deg(sim_t *s)   /* core's flux angle minus the true one, at standstill */
{
    servo_t *v = &s->servo;
    int32_t te = ((int32_t)(v->enc_dir * (int32_t)v->enc_last * v->cfg.pole_pairs)) & (SV_ENC_COUNTS - 1);
    double th = te * TWO_PI / SV_ENC_COUNTS + v->theta_off;
    double e = th - s->m.th * s->m.pp;
    e = fmod(e, TWO_PI); if (e > M_PI) e -= TWO_PI; if (e < -M_PI) e += TWO_PI;
    return e * 180.0 / M_PI;
}

/* ---- tests ---- */

static void test_mt6816_codec(void)
{
    printf("[mt6816 frame codec]\n");
    int bad = 0, badflag = 0;
    for (int a = 0; a < 16384; a++) {
        uint8_t rx[4] = { 0, (uint8_t)(a >> 6), (uint8_t)((a & 0x3F) << 2), 0 };
        uint16_t w = ((uint16_t)rx[1] << 8 | rx[2]) >> 1;
        w ^= w >> 8; w ^= w >> 4; w ^= w >> 2; w ^= w >> 1;
        rx[2] |= w & 1;
        uint16_t ang; uint8_t fl = mt6816_decode(rx, &ang);
        if (ang != a) bad++;
        if (fl) badflag++;
        rx[2] ^= 0x01;                                   /* parity error */
        if (!(mt6816_decode(rx, &ang) & SV_ENCF_PARITY)) badflag++;
        rx[2] ^= 0x01; rx[2] ^= 0x04;                    /* angle bit flipped: parity error */
        if (!(mt6816_decode(rx, &ang) & SV_ENCF_PARITY)) badflag++;
    }
    CHECK(bad == 0, "%d angles decoded wrong", bad);
    CHECK(badflag == 0, "%d flag errors", badflag);
    uint8_t rx[4] = { 0, 0x12, 0x02, 0 }; uint16_t ang;
    CHECK(mt6816_decode(rx, &ang) & SV_ENCF_NOMAG, "No_Mag bit not reported");
}

static void test_calibration(void)
{
    printf("[offset calibration]\n");
    fresh();                                             /* the rotor starts off a detent and rolls into it */
    cmd_enable(&S, 1);
    run_until_state(&S, SV_ALIGN, 1.0);
    INFO("calibration finished after %.0f ms (7.2 ms when the shaft is still)", S.servo.cal_total / 20.0);
    CHECK(S.servo.cal_total > 160, "calibration did not wait for the moving shaft");
    double ea = (SV_VREF + S.hw.amp_vos_a) / SV_VDDA * SV_ADC_FULL;
    double eb = (SV_VREF + S.hw.amp_vos_b) / SV_VDDA * SV_ADC_FULL;
    INFO("offset A %.1f (true %.1f), B %.1f (true %.1f)", S.servo.off_a, ea, S.servo.off_b, eb);
    CHECK(fabs(S.servo.off_a - ea) < 2.0, "offset A error %.2f LSB", S.servo.off_a - ea);
    CHECK(fabs(S.servo.off_b - eb) < 2.0, "offset B error %.2f LSB", S.servo.off_b - eb);
    CHECK(S.servo.state == SV_ALIGN, "state %d after CAL", S.servo.state);
    CHECK(S.st.a_invalid == 0 && S.st.b_invalid == 0, "invalid shunt samples during CAL: %llu %llu",
          (unsigned long long)S.st.a_invalid, (unsigned long long)S.st.b_invalid);
    /* a shaft at rest calibrates in the minimum time */
    fresh();
    sim_run(&S, 0.5);                                    /* settle into the detent, drive off */
    cmd_enable(&S, 1);
    run_until_state(&S, SV_ALIGN, 1.0);
    CHECK(S.servo.cal_total < 200, "still shaft needed %u ticks", S.servo.cal_total);
    CHECK(fabs(S.servo.off_b - eb) < 2.0, "offset B error %.2f LSB with a still shaft", S.servo.off_b - eb);
}

static void test_phasing(int enc_sign, double th0)
{
    printf("[phasing, magnet orientation %+d, start angle %.2f rad]\n", enc_sign, th0);
    fresh();
    S.m.enc_sign = enc_sign; S.m.th = th0;
    bool ok = start_run(&S);
    CHECK(ok, "did not reach RUN (state %d, fault 0x%x)", S.servo.state, S.servo.fault);
    if (!ok) return;
    sim_run(&S, 0.1);
    double e = enc_err_deg(&S);
    INFO("enc_dir %d, flux angle error %.2f deg, align->dir move %.1f counts (expect %.1f)",
         S.servo.enc_dir, e, S.servo.dir_mean - S.servo.align_mean, 16384.0 / (4 * 50));
    CHECK(S.servo.enc_dir == enc_sign, "enc_dir %d", S.servo.enc_dir);
    CHECK(fabs(e) < 5.0, "flux angle error %.2f deg", e);
    CHECK(fabs(S.servo.ia) < 0.15 && fabs(S.servo.ib) < 0.15, "idle current %.3f %.3f A", S.servo.ia, S.servo.ib);
}

static void test_endstop(void)
{
    printf("[phasing with the shaft against a hard endstop]\n");
    int good = 0, safe = 0, wrong = 0, other = 0; double worst = 0;
    for (int side = -1; side <= 1; side += 2) {
        for (int k = 0; k < 6; k++) {                     /* 6 start phases across one electrical turn */
            double th0 = 0.37 + k * (TWO_PI / 50 / 6);
            fresh();
            S.m.th = th0;
            if (side > 0) S.m.stop_hi = th0; else S.m.stop_lo = th0;
            cmd_enable(&S, 1);
            run_until_state(&S, SV_RUN, 4.0);
            if (S.servo.state == SV_RUN) {
                sim_run(&S, 0.05);
                double e = enc_err_deg(&S);
                if (fabs(e) > worst) worst = fabs(e);
                if (S.servo.enc_dir == 1 && fabs(e) < 25.0) good++;
                else { wrong++; INFO("wrong: stop %+d, start %d, enc_dir %d, flux error %.0f deg (steps %.1f / %.1f counts)", side, k, S.servo.enc_dir, e, S.servo.dir_mean - S.servo.align_mean, S.servo.neg_mean - S.servo.align_mean); }
            } else if (S.servo.state == SV_FAULT && (S.servo.fault & SV_FLT_PHASING)) safe++;
            else other++;
        }
    }
    INFO("12 starts against a stop: %d phased (worst flux error %.0f deg), %d refused with a phasing fault, %d wrong, %d other",
         good, worst, safe, wrong, other);
    CHECK(wrong == 0, "%d starts accepted a wrong phasing at an endstop", wrong);
    CHECK(other == 0, "%d starts neither ran nor faulted", other);
    CHECK(good >= 4, "only %d of 12 starts at an endstop phased", good);
}

static void test_current_step(void)
{
    printf("[current loop step, rotor blocked]\n");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    cmd_mode(&S, SV_MODE_TORQUE); sim_run(&S, 0.01);
    S.m.locked = true;
    sim_run(&S, 0.02);
    cmd_torque_ma(&S, (int)(ISTEP * 1000));
    sim_run(&S, 0.0005);                                 /* CAN command lands within 1 ms */
    double t10 = -1, t90 = -1, peak = 0, t = 0;
    for (int i = 0; i < 400; i++) {                      /* 40 us steps */
        sim_run(&S, 40e-6); t += 40e-6;
        double iq = S.m.iq_true;
        if (t10 < 0 && iq >= 0.1 * ISTEP) t10 = t;
        if (t90 < 0 && iq >= 0.9 * ISTEP) t90 = t;
        if (iq > peak) peak = iq;
    }
    sim_run(&S, 0.05);
    double fin = S.m.iq_true;
    INFO("rise 10-90%% %.0f us, overshoot %.1f%%, final %.3f A (cmd %.3f), measured by the core %.3f A",
         (t90 - t10) * 1e6, (peak / fin - 1) * 100, fin, ISTEP, S.servo.iq);
    CHECK(t10 > 0 && t90 > 0, "never reached 90%%");
    CHECK((t90 - t10) > 150e-6 && (t90 - t10) < 700e-6, "rise time %.0f us out of range", (t90 - t10) * 1e6);
    CHECK(peak / fin < 1.15, "overshoot %.1f%%", (peak / fin - 1) * 100);
    CHECK(fabs(fin - ISTEP) < 0.03 * ISTEP, "steady error %.3f A", fin - ISTEP);
    CHECK(S.st.a_invalid + S.st.b_invalid == 0, "invalid shunt samples %llu", (unsigned long long)(S.st.a_invalid + S.st.b_invalid));
}

static void test_position(void)
{
    printf("[position loop]\n");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    int32_t p0 = S.servo.pos;
    int32_t steps[3] = { 2000, -5000, 16384 };
    int32_t target = p0;
    for (int k = 0; k < 3; k++) {
        target += steps[k];
        cmd_pos(&S, target);
        double settle = -1, peak = 0, t = 0;
        double start = S.servo.pos;
        for (int i = 0; i < 800; i++) {                  /* 1 s in 1.25 ms */
            sim_run(&S, 1.25e-3); t += 1.25e-3;
            double e = target - S.servo.pos;
            double over = (steps[k] > 0 ? -e : e);
            if (over > peak) peak = over;
            if (fabs(e) > 8) settle = -1; else if (settle < 0) settle = t;
        }
        double span = fabs(target - start);
        sim_run(&S, 0.3);
        double true_err = target - S.m.th * S.m.enc_sign / TWO_PI * SV_ENC_COUNTS - 0;
        INFO("step %+d counts: settled (+-8) after %.0f ms, overshoot %.1f counts (%.1f%%), final error %d counts",
             steps[k], settle * 1000, peak, 100 * peak / span, target - S.servo.pos);
        CHECK(settle > 0 && settle < 0.6, "settle time %.0f ms", settle * 1000);
        CHECK(peak / span < 0.12, "overshoot %.1f%%", 100 * peak / span);
        CHECK(abs(target - S.servo.pos) <= 3, "final error %d", target - S.servo.pos);
        (void)true_err;
    }
    /* the core's multi-turn counter must follow the true shaft */
    double shaft = (S.m.th * S.m.enc_sign / TWO_PI) * SV_ENC_COUNTS;
    INFO("shaft truth %.1f counts vs core %d", shaft, S.servo.pos);
    CHECK(fabs(shaft - S.servo.pos) < 4.0, "multi-turn count off by %.1f", shaft - S.servo.pos);
}

static void test_velocity(void)
{
    printf("[velocity loop]\n");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    cmd_mode(&S, SV_MODE_VELOCITY);
    cmd_vel(&S, 1200);                                   /* 120.0 rpm */
    sim_run(&S, 0.6);
    double th_a = S.m.th; sim_run(&S, 0.3);
    double rpm = (S.m.th - th_a) / 0.3 * 60 / TWO_PI;
    INFO("speed %.1f rpm (cmd 120.0), torque estimate %.3f Nm, friction torque %.3f Nm",
         rpm, S.servo.torque_nm, S.m.b * S.m.w);
    CHECK(fabs(rpm - 120.0) < 3.6, "speed %.1f rpm", rpm);
    CHECK(fabs(S.servo.torque_nm - S.m.b * S.m.w) < 0.037 * TL, "torque estimate %.3f", S.servo.torque_nm);
    cmd_vel(&S, -1200);
    sim_run(&S, 0.8);
    th_a = S.m.th; sim_run(&S, 0.3);
    rpm = (S.m.th - th_a) / 0.3 * 60 / TWO_PI;
    CHECK(fabs(rpm + 120.0) < 3.6, "reverse speed %.1f rpm", rpm);
}

static void test_force_feedback(void)
{
    printf("[force feedback over CAN]\n");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    cmd_stream(&S, 10);                                  /* telemetry every 10 ms */
    tlm_t t; memset(&t, 0, sizeof t);
    sim_run(&S, 0.3);
    static const float frac[] = { 0.074f, 0.222f, 0.370f, 0.593f, 0.815f, 0.0f, -0.222f, -0.593f, -0.815f };
    float loads[sizeof frac / sizeof *frac];
    for (unsigned i = 0; i < sizeof frac / sizeof *frac; i++) loads[i] = (float)(frac[i] * TL);
    double worst = 0;
    for (unsigned i = 0; i < sizeof loads / sizeof *loads; i++) {
        S.m.t_load = loads[i];
        sim_run(&S, 0.5);
        tlm_t last = t; drain(&S, &last); t = last;
        double rep = t.torque_mnm / 1000.0;
        double err = rep - loads[i];
        if (fabs(err) > worst) worst = fabs(err);
        INFO("load %+.2f Nm -> CAN torque %+.3f Nm (iq %+d mA), position error %d counts",
             loads[i], rep, t.iq_ma, t.pos_err);
        CHECK(t.have_a && t.have_b, "no telemetry received");
        CHECK(fabs(err) < 0.052 * TL + 0.05 * fabs(loads[i]), "torque %.3f vs load %.3f", rep, loads[i]);
        CHECK(abs(t.pos_err) < 6, "position error %d under load", t.pos_err);
        CHECK(t.state == SV_RUN && t.fault == 0 && !t.stall, "state %d fault 0x%x stall %d", t.state, t.fault, t.stall);
    }
    INFO("worst torque error %.3f Nm over +-%.2f Nm (limit of the drive: %.2f Nm)", worst, 0.815 * TL, TL);
    /* beyond the torque limit: stall flag, torque telemetry saturates */
    S.m.b = (float)(0.037 * TL);                                       /* a loaded shaft: the weight sinks at a finite speed */
    S.m.t_load = (float)(1.185 * TL);
    sim_run(&S, 1.0);
    drain(&S, &t);
    INFO("load %.2f Nm (over the limit): stall %d, CAN torque %.3f Nm, iq %d mA", 1.185 * TL, t.stall, t.torque_mnm / 1000.0, t.iq_ma);
    CHECK(t.stall, "stall flag not set");
    CHECK(t.fault == 0, "stall must not be a fault (0x%x)", t.fault);
    /* remove the overload: the multi-turn count kept the lost steps, the drive returns to the target */
    int32_t tgt = S.servo.cmd_pos;
    INFO("lost %d counts while overloaded", S.servo.pos - tgt);
    S.m.t_load = 0; S.m.b = SIM_M_B;
    sim_run(&S, 2.5);
    drain(&S, &t);
    INFO("overload removed: position error %d counts, stall %d", t.pos_err, t.stall);
    CHECK(abs(S.servo.pos - tgt) <= 3 && !t.stall, "no recovery after the overload (error %d)", S.servo.pos - tgt);
}

static void test_no_detent_model(void)
{
    printf("[detent feed-forward off: what the drive does with an unidentified motor]\n");
    fresh();
    S.servo.cfg.detent_nm = 0.0f;
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    cmd_pos(&S, S.servo.pos + 3000);
    sim_run(&S, 1.5);
    int e0 = abs(S.servo.cmd_pos - S.servo.pos);
    double lo = 1e9, hi = -1e9;
    for (int i = 0; i < 100; i++) {                      /* hunting amplitude over 1 s */
        sim_run(&S, 0.01);
        double e = S.servo.cmd_pos - S.servo.pos;
        if (e < lo) lo = e;
        if (e > hi) hi = e;
    }
    INFO("holding error now %d counts, range over 1 s [%.0f, %.0f] counts (0.1 electrical turn = 33 counts)", e0, lo, hi);
    CHECK(lo > -SIM_HUNT_LIM && hi < SIM_HUNT_LIM, "hunting %.0f..%.0f counts without detent compensation", lo, hi);
    S.m.t_load = (float)(0.37 * TL); sim_run(&S, 0.6);
    double tq = S.servo.torque_nm;
    INFO("%.2f Nm load: torque estimate %.3f Nm", 0.37 * TL, tq);
    CHECK(fabs(tq - 0.37 * TL) < 0.067 * TL, "torque estimate %.3f without detent compensation", tq);
}

static void test_param_mismatch(void)
{
    printf("[motor parameters configured wrong]\n");
    static const struct { float r, l, kt; const char *what; } v[] = {
        { 1.5f, 0.6f, 1.0f, "R +50%, L -40%" },
        { 0.6f, 1.5f, 1.0f, "R -33%, L +50%" },
        { 1.0f, 1.0f, 1.2f, "Kt +20%" },
    };
    for (unsigned i = 0; i < sizeof v / sizeof *v; i++) {
        fresh();
        servo_cfg_t cfg = S.servo.cfg;
        cfg.r_ohm *= v[i].r; cfg.l_h *= v[i].l; cfg.kt *= v[i].kt;
        servo_init(&S.servo, &cfg);                           /* recompute the PI gains */
        bool ok = start_run(&S);
        CHECK(ok, "%s: no RUN (fault 0x%x)", v[i].what, S.servo.fault);
        if (!ok) continue;
        cmd_pos(&S, S.servo.pos + 4000);
        sim_run(&S, 0.8);
        int err = S.servo.cmd_pos - S.servo.pos;
        S.m.t_load = (float)(0.444 * TL); sim_run(&S, 0.6);
        INFO("%s: position error %d counts, %.2f Nm load reads %.3f Nm, state %d", v[i].what, err, 0.444 * TL, S.servo.torque_nm, S.servo.state);
        CHECK(abs(err) <= 4, "%s: position error %d", v[i].what, err);
        CHECK(S.servo.state == SV_RUN, "%s: left RUN (0x%x)", v[i].what, S.servo.fault);
        double want = 0.444 * TL * v[i].kt;                          /* the estimate scales with the configured Kt */
        CHECK(fabs(S.servo.torque_nm - want) < 0.037 * TL, "%s: torque estimate %.3f, expected %.3f", v[i].what, S.servo.torque_nm, want);
    }
}


/* ---- parameter access over the CAN mailbox ---- */
static float f32of(uint32_t raw) { float f; memcpy(&f, &raw, 4); return f; }
static uint32_t bitsof(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

/* send a parameter command, run 2.5 ms, return the 0x381 reply for `code` and `id` */
static bool pxfer(sim_t *s, const uint8_t *cmd, int len, can_frame_t *rep)
{
    can_frame_t f; memset(&f, 0, sizeof f);
    f.id = 0x100 + s->can.node; f.len = len; memcpy(f.data, cmd, len);
    sim_can_send(s, &f);
    sim_run(s, 0.0025);
    can_frame_t r; bool got = false;
    while (sim_can_recv(s, &r))
        if (r.id == 0x380 + s->can.node && r.data[0] == cmd[0]) { *rep = r; got = true; }
    return got;
}
static int pget(sim_t *s, uint16_t id, uint32_t *val)
{
    uint8_t c[3] = { 0x31, (uint8_t)id, (uint8_t)(id >> 8) }; can_frame_t r;
    if (!pxfer(s, c, 3, &r)) return -1;
    *val = (uint32_t)r.data[4] | (uint32_t)r.data[5] << 8 | (uint32_t)r.data[6] << 16 | (uint32_t)r.data[7] << 24;
    return r.data[1] & 15;
}
static int pset(sim_t *s, uint16_t id, uint32_t raw, uint32_t *val)
{
    uint8_t c[7] = { 0x30, (uint8_t)id, (uint8_t)(id >> 8), (uint8_t)raw, (uint8_t)(raw >> 8), (uint8_t)(raw >> 16), (uint8_t)(raw >> 24) };
    can_frame_t r;
    if (!pxfer(s, c, 7, &r)) return -1;
    if (val) *val = (uint32_t)r.data[4] | (uint32_t)r.data[5] << 8 | (uint32_t)r.data[6] << 16 | (uint32_t)r.data[7] << 24;
    return r.data[1] & 15;
}
static int pinfo(sim_t *s, uint16_t id, int sel, uint32_t *val)
{
    uint8_t c[4] = { 0x32, (uint8_t)id, (uint8_t)(id >> 8), (uint8_t)sel }; can_frame_t r;
    if (!pxfer(s, c, 4, &r)) return -1;
    *val = (uint32_t)r.data[4] | (uint32_t)r.data[5] << 8 | (uint32_t)r.data[6] << 16 | (uint32_t)r.data[7] << 24;
    return r.data[1] & 15;
}
static int pat(sim_t *s, uint16_t index, uint32_t *val)
{
    uint8_t c[3] = { 0x33, (uint8_t)index, (uint8_t)(index >> 8) }; can_frame_t r;
    if (!pxfer(s, c, 3, &r)) return -1;
    *val = (uint32_t)r.data[4] | (uint32_t)r.data[5] << 8;
    return r.data[1] & 15;
}
static uint8_t flash_blob[256]; static int flash_len;
static int flash_cb(const uint8_t *b, uint16_t n) { memcpy(flash_blob, b, n); flash_len = n; return 0; }

static void test_params(void)
{
    printf("[parameters over CAN]\n");
    fresh();
    uint32_t v = 0; int st;
    /* enumeration */
    st = pat(&S, 0xFFFF, &v);
    int count = (int)v;
    CHECK(st == 0 && count == params_count(), "count %d, table %d", count, params_count());
    int rw = 0, saved = 0;
    for (int i = 0; i < count; i++) {
        uint32_t idv = 0; st = pat(&S, (uint16_t)i, &idv);
        const param_def_t *d = params_at(i);
        CHECK(st == 0 && d && idv == d->id, "index %d -> id 0x%04x (table 0x%04x)", i, idv, d ? d->id : 0);
        if (!d) continue;
        if (!(d->flags & PF_RO)) rw++;
        if (d->flags & PF_PERSIST) saved++;
        uint32_t t, mn, mx, got;
        CHECK(pinfo(&S, d->id, 0, &t) == 0 && (t & 0xFF) == (uint32_t)d->type && (t >> 8) == d->flags, "info type/flags of %s", d->name);
        CHECK(pinfo(&S, d->id, 1, &mn) == 0 && f32of(mn) == d->min, "min of %s", d->name);
        CHECK(pinfo(&S, d->id, 2, &mx) == 0 && f32of(mx) == d->max, "max of %s", d->name);
        CHECK(pget(&S, d->id, &got) == 0, "get %s", d->name);
        if (!(d->flags & PF_RO)) {                                 /* writable: range edges and round trip */
            uint32_t cur; pget(&S, d->id, &cur);
            CHECK(pset(&S, d->id, cur, &got) == PS_OK && got == cur, "%s: write back the current value", d->name);
            if (d->type == PT_F32) {
                CHECK(pset(&S, d->id, bitsof(d->max * 1.01f + 1e-3f), &got) == PS_RANGE && got == cur, "%s: above the maximum", d->name);
                CHECK(pset(&S, d->id, bitsof(d->min - fabsf(d->min) * 0.01f - 1e-3f), &got) == PS_RANGE && got == cur, "%s: below the minimum", d->name);
                float mid = 0.5f * (d->min + d->max);
                if (d->id == 0x0204) mid = 0.5f * S.servo.cfg.i_max;          /* the runtime limit is capped by i_max */
                CHECK(pset(&S, d->id, bitsof(mid), &got) == PS_OK && f32of(got) == mid, "%s: midpoint %g read back %g", d->name, mid, f32of(got));
            } else {
                CHECK(pset(&S, d->id, (uint32_t)(int32_t)((double)d->max + 1), &got) == PS_RANGE && got == cur, "%s: above the maximum", d->name);
                CHECK(pset(&S, d->id, (uint32_t)(int32_t)((double)d->min - 1), &got) == PS_RANGE && got == cur, "%s: below the minimum", d->name);
                int32_t mid = (int32_t)((d->min + d->max) / 2);
                if (d->id == 0x0400) mid = (int32_t)cur;               /* the node id keeps the mailbox working */
                CHECK(pset(&S, d->id, (uint32_t)mid, &got) == PS_OK && (int32_t)got == mid, "%s: midpoint %d read back %d", d->name, mid, (int)got);
            }
        } else {
            CHECK(pset(&S, d->id, 1, &got) == PS_READ_ONLY, "%s: read-only write accepted", d->name);
        }
    }
    INFO("%d parameters, %d settable, %d in the saved set", count, rw, saved);
    /* errors */
    CHECK(pget(&S, 0x7777, &v) == PS_UNKNOWN, "unknown id on get");
    CHECK(pset(&S, 0x7777, 0, &v) == PS_UNKNOWN, "unknown id on set");
    { uint8_t c[3] = { 0x30, 0x07, 0x03 }; can_frame_t r; CHECK(pxfer(&S, c, 3, &r) && (r.data[1] & 15) == PS_BAD_ARG, "short set frame"); }
    CHECK(pset(&S, 0x0307, bitsof(NAN), &v) == PS_BAD_ARG, "NaN");
    CHECK(pset(&S, 0x0307, bitsof(INFINITY), &v) == PS_BAD_ARG, "infinity");
    /* relation between vbus limits */
    fresh();
    CHECK(pset(&S, 0x030B, bitsof(35.0f), &v) == PS_RANGE, "vbus_min above vbus_max accepted");
    CHECK(pset(&S, 0x030C, bitsof(9.0f), &v) == PS_RANGE, "vbus_max below vbus_min accepted");
    /* identity */
    CHECK(pget(&S, 0x0001, &v) == 0 && v == SV_FW_VERSION, "fw version");
    CHECK(pget(&S, 0x0002, &v) == 0 && v == SV_BOARD_ID, "board id");
}

static void test_params_running(void)
{
    printf("[parameters while running]\n");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    uint32_t v, before; pget(&S, 0x0300, &before);
    CHECK(pset(&S, 0x0300, bitsof(2.0f), &v) == PS_RUNNING && v == before, "motor_r changed while running");
    uint32_t pp; pget(&S, 0x0303, &pp);
    CHECK(pset(&S, 0x0303, 20, &v) == PS_RUNNING && v == pp, "pole_pairs changed while running");
    CHECK(pset(&S, 0x0307, bitsof(80.0f), &v) == PS_OK && f32of(v) == 80.0f, "pos_kp not writable while running");
    CHECK(pset(&S, 0x0204, bitsof(1.0f), &v) == PS_OK && fabsf(S.servo.ilim - 1.0f) < 1e-6f, "current limit");
    CHECK(pset(&S, 0x0304, bitsof(0.5f), &v) == PS_OK && S.servo.ilim <= 0.5f + 1e-6f, "i_max below the limit must pull the limit down");
    /* the measured values follow the drive */
    sim_run(&S, 0.1);
    pget(&S, 0x0100, &v);   CHECK((int)v == SV_RUN, "state parameter %u", v);
    pget(&S, 0x0103, &v);   CHECK((int32_t)v == S.servo.pos || labs((int32_t)v - S.servo.pos) < 3, "pos parameter");
    pget(&S, 0x010C, &v);   CHECK(fabsf(f32of(v) - 24.0f) < 0.5f, "vbus parameter %.2f", f32of(v));
    /* defaults refuse while running */
    uint8_t c[1] = { 0x36 }; can_frame_t r;
    sim_can_cmd(&S, 0x36, NULL, 0); (void)c; (void)r;
    sim_run(&S, 0.003);
    CHECK(S.servo.cfg.pos_kp == 80.0f, "defaults were applied while running");
}

static void test_param_effect(void)
{
    printf("[a parameter change takes effect]\n");
    double rise[2];
    float bw[2] = { 1000.0f, 400.0f };
    for (int k = 0; k < 2; k++) {
        fresh();
        uint32_t v;
        CHECK(pset(&S, 0x0306, bitsof(bw[k]), &v) == PS_OK, "cur_bw");
        CHECK(pset(&S, 0x0305, bitsof((float)SIM_I_ALIGN), &v) == PS_OK, "i_align");
        if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
        cmd_mode(&S, SV_MODE_TORQUE); sim_run(&S, 0.01);
        S.m.locked = true; sim_run(&S, 0.02);
        cmd_torque_ma(&S, (int)(ISTEP * 1000)); sim_run(&S, 0.0005);
        double t10 = -1, t90 = -1, t = 0;
        for (int i = 0; i < 500; i++) {
            sim_run(&S, 40e-6); t += 40e-6;
            if (t10 < 0 && S.m.iq_true >= 0.1 * ISTEP) t10 = t;
            if (t90 < 0 && S.m.iq_true >= 0.9 * ISTEP) t90 = t;
        }
        rise[k] = (t90 - t10) * 1e6;
    }
    INFO("current loop bandwidth 1000 Hz: rise %.0f us; set to 400 Hz over CAN: rise %.0f us", rise[0], rise[1]);
    CHECK(rise[1] > 2.0 * rise[0] && rise[1] > 600 && rise[1] < 1500, "rise %.0f us at 400 Hz", rise[1]);
}

static void test_param_store(void)
{
    printf("[saving parameters]\n");
    fresh();
    uint32_t v;
    /* no flash behind the command yet */
    sim_can_cmd(&S, 0x35, NULL, 0);
    sim_run(&S, 0.003);
    { can_frame_t r; int got = 0; while (sim_can_recv(&S, &r)) if (r.id == 0x081 && r.data[0] == 0x35) { got = 1; CHECK(r.data[1] == PS_UNAVAIL, "save without a store: status %d", r.data[1]); } CHECK(got, "no reply to save"); }
    S.can.save_cb = flash_cb; flash_len = 0;
    pset(&S, 0x0307, bitsof(77.0f), &v); pset(&S, 0x0308, bitsof(0.05f), &v);
    pset(&S, 0x030F, bitsof(0.03f), &v); pset(&S, 0x0401, 250, &v); pset(&S, 0x0300, bitsof(1.25f), &v);
    sim_can_cmd(&S, 0x35, NULL, 0); sim_run(&S, 0.003);
    { can_frame_t r; while (sim_can_recv(&S, &r)) if (r.id == 0x081 && r.data[0] == 0x35) CHECK(r.data[1] == 0, "save status %d", r.data[1]); }
    CHECK(flash_len > 0 && flash_len <= (int)sizeof flash_blob, "blob length %d", flash_len);
    INFO("saved set: %d bytes", flash_len);
    /* power cycle: defaults, then load */
    uint8_t blob[256]; int len = flash_len; memcpy(blob, flash_blob, len);
    sim_init(&S);
    CHECK(S.servo.cfg.pos_kp == 100.0f && S.can.stream_ms == 0, "defaults after init");
    int applied = params_load(&S.servo, &S.can, blob, len);
    INFO("loaded %d values", applied);
    CHECK(applied == 19, "applied %d of 19", applied);
    CHECK(S.servo.cfg.pos_kp == 77.0f && S.servo.cfg.vel_kp == 0.05f && S.servo.cfg.detent_nm == 0.03f, "values after load");
    CHECK(S.can.stream_ms == 250 && S.servo.cfg.r_ohm == 1.25f, "stream / R after load");
    CHECK(fabsf(S.servo.kp_i - S.servo.cfg.l_h * 6.2831853f * S.servo.cfg.cur_bw_hz) < 1e-3f, "PI gains recomputed");
    /* damaged blobs change nothing */
    sim_init(&S);
    uint8_t bad[256]; memcpy(bad, blob, len); bad[10] ^= 0x40;
    CHECK(params_load(&S.servo, &S.can, bad, len) == -1 && S.servo.cfg.pos_kp == 100.0f, "bit flip not detected");
    CHECK(params_load(&S.servo, &S.can, blob, len - 3) == -1 && S.servo.cfg.pos_kp == 100.0f, "truncated blob accepted");
    CHECK(params_load(&S.servo, &S.can, blob, 3) == -1, "tiny blob accepted");
    uint8_t zeros[64] = { 0 };
    CHECK(params_load(&S.servo, &S.can, zeros, sizeof zeros) == -1, "zero blob accepted");
    /* not while running */
    sim_init(&S);
    if (start_run(&S)) CHECK(params_load(&S.servo, &S.can, blob, len) == -1 && S.servo.cfg.pos_kp == 100.0f, "loaded while running");
    /* restore defaults over CAN */
    sim_init(&S);
    pset(&S, 0x0307, bitsof(55.0f), &v); pset(&S, 0x0300, bitsof(3.0f), &v);
    sim_can_cmd(&S, 0x36, NULL, 0); sim_run(&S, 0.003);
    CHECK(S.servo.cfg.pos_kp == 100.0f && fabsf(S.servo.cfg.r_ohm - SIM_M_R) < 1e-6f, "defaults not restored");
    CHECK(fabsf(S.servo.kp_i - S.servo.cfg.l_h * 6.2831853f * S.servo.cfg.cur_bw_hz) < 1e-3f && fabsf(S.servo.ki_i - S.servo.cfg.r_ohm * 6.2831853f * S.servo.cfg.cur_bw_hz) < 1e-2f, "PI gains after defaults");
}

static void dump_params_md(void)
{
    printf("### %s\n\n| id | name | type | unit | range | flags |\n|---|---|---|---|---|---|\n", SV_BOARD_NAME);
    static const char *tn[] = { "f32", "i32", "u16", "u8" };
    for (int i = 0; i < params_count(); i++) {
        const param_def_t *d = params_at(i);
        char fl[32] = "";
        if (d->flags & PF_RO) strcat(fl, "read-only ");
        if (d->flags & PF_OFF_ONLY) strcat(fl, "drive off ");
        if (d->flags & PF_PERSIST) strcat(fl, "saved");
        char rng[64];
        if (d->flags & PF_RO) snprintf(rng, sizeof rng, "-"); else snprintf(rng, sizeof rng, "%g .. %g", d->min, d->max);
        printf("| 0x%04X | %s | %s | %s | %s | %s |\n", d->id, d->name, tn[d->type], d->unit, rng, fl);
    }
    printf("\n");
}

static void test_commands(void)
{
    printf("[CAN commands]\n");
    fresh();
    can_frame_t f, r[4];
    f.id = 0x101; f.len = 2; f.data[0] = 0x02; f.data[1] = 9;       /* bad mode */
    int n = can_proto_rx(&S.can, &S.servo, &f, r);
    CHECK(n == 1 && r[0].id == 0x081 && r[0].data[1] == 1, "bad mode not refused");
    f.data[0] = 0x55;
    n = can_proto_rx(&S.can, &S.servo, &f, r);
    CHECK(n == 1 && r[0].data[1] == 2, "unknown command not refused");
    f.id = 0x105; f.data[0] = 0x01; f.data[1] = 1;                   /* other node */
    n = can_proto_rx(&S.can, &S.servo, &f, r);
    CHECK(n == 0 && S.servo.state == SV_OFF, "frame for another node accepted");
    f.id = 0x101; f.len = 1;                                         /* short frame */
    n = can_proto_rx(&S.can, &S.servo, &f, r);
    CHECK(n == 1 && r[0].data[1] == 1 && S.servo.state == SV_OFF, "short enable frame accepted");
    f.len = 3; f.data[0] = 0x20; put_i16(f.data + 1, 9000);          /* limit above i_max */
    can_proto_rx(&S.can, &S.servo, &f, r);
    CHECK(S.servo.ilim <= S.servo.cfg.i_max + 1e-6, "current limit %.2f above the maximum", S.servo.ilim);
    servo_cfg_t big = S.servo.cfg; servo_t tmp;
    big.i_trip = 100.0f; servo_init(&tmp, &big);                     /* trip level above the amplifier range */
    CHECK(tmp.cfg.i_trip <= 0.9f * SV_I_FULLSCALE + 1e-6f, "trip level %.1f A beyond the sensing range %.1f A", tmp.cfg.i_trip, SV_I_FULLSCALE);
    f.len = 7; f.data[0] = 0x30; f.data[1] = 0x07; f.data[2] = 0x03;       /* pos_kp = NaN */
    float nan = NAN; memcpy(f.data + 3, &nan, 4);
    float before = S.servo.cfg.pos_kp;
    n = can_proto_rx(&S.can, &S.servo, &f, r);
    CHECK(n == 1 && r[0].id == 0x381 && r[0].data[1] == PS_BAD_ARG && S.servo.cfg.pos_kp == before, "NaN gain accepted");
}

static void test_faults(void)
{
    printf("[faults]\n");
    /* supply collapse */
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    cmd_stream(&S, 10);
    S.hw.vin = 8.0f;
    sim_run(&S, 0.1);
    tlm_t t; memset(&t, 0, sizeof t); drain(&S, &t);
    INFO("supply 8 V: state %d, fault 0x%x, drive %d", S.servo.state, S.servo.fault, S.drive);
    CHECK(S.servo.state == SV_FAULT && (S.servo.fault & SV_FLT_UNDERVOLT), "no undervoltage trip");
    CHECK(!S.drive && S.cmp[0] == 0 && S.cmp[1] == 0 && S.cmp[2] == 0 && S.cmp[3] == 0, "legs not all low after the trip");
    CHECK(t.state == SV_FAULT && (t.fault & SV_FLT_UNDERVOLT), "fault not in the telemetry");
    S.hw.vin = 24.0f;
    sim_can_cmd(&S, 0x7F, NULL, 0);
    sim_run(&S, 0.01);
    CHECK(S.servo.state == SV_OFF && S.servo.fault == 0, "fault not cleared");
    CHECK(start_run(&S), "no restart after the fault");

    /* encoder: parity glitches are skipped, a streak trips */
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    sim_run(&S, 0.1);
    int32_t p = S.servo.pos;
    S.hw.enc_parity_glitches = 2;
    sim_run(&S, 0.05);
    CHECK(S.servo.state == SV_RUN && abs(S.servo.pos - p) <= 3, "two parity glitches upset the drive (pos %d -> %d)", p, S.servo.pos);
    S.hw.enc_parity_glitches = 50;
    sim_run(&S, 0.05);
    CHECK(S.servo.state == SV_FAULT && (S.servo.fault & SV_FLT_ENCODER), "parity streak did not trip");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    S.hw.enc_nomag = true;
    sim_run(&S, 0.05);
    CHECK(S.servo.state == SV_FAULT && (S.servo.fault & SV_FLT_NOMAG), "No_Mag did not trip");

    /* no motor / locked shaft at start: phasing must refuse */
    fresh();
    S.m.locked = true;
    cmd_enable(&S, 1);
    run_until_state(&S, SV_RUN, 4.0);
    CHECK(S.servo.state == SV_FAULT && (S.servo.fault & SV_FLT_PHASING), "blocked shaft not detected (state %d fault 0x%x)",
          S.servo.state, S.servo.fault);
}

static void test_adc_timing(void)
{
    printf("[ADC trigger timing against the sensed low-side window]\n");
    /* trigger at the carrier valley: the sensed legs are mostly high */
    fresh();
    S.hw.trig_offset_ticks = SIM_HALF;
    cmd_enable(&S, 1); sim_run(&S, 0.2);
    double bad = (double)(S.st.a_invalid + S.st.b_invalid) / (S.st.a_samples + S.st.b_samples);
    INFO("trigger at the valley: %.0f%% of the shunt samples invalid", bad * 100);
    CHECK(bad > 0.3, "the model does not see a wrong trigger (%.0f%%)", bad * 100);
    /* upstream-style slow ADC (4 MHz clock, 28.5 cycle sampling) centred in the window */
    fresh();
    S.hw.adc_hz = 4.0e6; S.hw.adc_smp_cycles = 28.5f; S.hw.adc_conv_cycles = 12.5f;
    double a_hold = 28.5 / 4e6, b_hold = (12.5 + 28.5) / 4e6;
    S.hw.trig_offset_ticks = -(int)lround(0.5 * (a_hold + b_hold) * SIM_TICK_HZ);
    bool ok = start_run(&S);
    cmd_mode(&S, SV_MODE_VELOCITY); cmd_vel(&S, 2400);   /* 240 rpm, high modulation */
    sim_run(&S, 0.5);
    bad = (double)(S.st.a_invalid + S.st.b_invalid) / (S.st.a_samples + S.st.b_samples);
    INFO("4 MHz ADC, trigger %d ticks before the peak: %.2f%% invalid, speed %.0f rpm",
         -S.hw.trig_offset_ticks, bad * 100, S.m.w * 60 / TWO_PI);
    CHECK(ok, "no RUN with the slow ADC");
    CHECK(bad < 0.01, "%.2f%% invalid samples with the slow ADC centred", bad * 100);
}

static void test_speed_limits(void)
{
    printf("[current peaks and supply headroom]\n");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    S.st.ia_peak = S.st.ib_peak = 0;
    cmd_mode(&S, SV_MODE_VELOCITY); cmd_vel(&S, 3000);   /* 300 rpm */
    sim_run(&S, 1.0);
    INFO("300 rpm: measured %.0f rpm, coil peak %.2f / %.2f A (limit %.1f A)",
         S.m.w * 60 / TWO_PI, S.st.ia_peak, S.st.ib_peak, S.servo.cfg.i_max);
    CHECK(S.st.ia_peak < 1.33 * SIM_I_MAX && S.st.ib_peak < 1.33 * SIM_I_MAX, "current peak %.2f A", fmaxf(S.st.ia_peak, S.st.ib_peak));
    CHECK(S.servo.state == SV_RUN, "left RUN (fault 0x%x)", S.servo.fault);
}

/* a trace for plotting: position steps, then a load ramp, then a velocity run */
static void demo(const char *path)
{
    fresh();
    S.trace = fopen(path, "w");
    if (!S.trace) { perror(path); return; }
    fprintf(S.trace, "t,pos,vel,iq_filt,torque_est,torque_true,load,ia,ib,state\n");
    start_run(&S);
    cmd_stream(&S, 10);
    sim_run(&S, 0.2);
    cmd_pos(&S, S.servo.pos + 4000); sim_run(&S, 0.5);
    cmd_pos(&S, S.servo.pos - 2000); sim_run(&S, 0.5);
    for (int i = 0; i <= 10; i++) { S.m.t_load = (float)(0.89 * TL * i / 10); sim_run(&S, 0.25); }
    for (int i = 10; i >= -10; i--) { S.m.t_load = (float)(0.89 * TL * i / 10); sim_run(&S, 0.25); }
    S.m.t_load = 0; sim_run(&S, 0.4);
    cmd_mode(&S, SV_MODE_VELOCITY); cmd_vel(&S, 1500); sim_run(&S, 0.8);
    cmd_vel(&S, -1500); sim_run(&S, 1.0);
    fclose(S.trace);
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "--params-md")) { dump_params_md(); return 0; }
    if (argc > 2 && !strcmp(argv[1], "--demo")) { demo(argv[2]); return 0; }
    printf("virtual %s: %.1f A max, %.2f Nm, sense %.2f mA/LSB, range +-%.1f A\n\n", SV_BOARD_NAME, SIM_I_MAX, TL, SV_AMPS_PER_LSB * 1000, SV_I_FULLSCALE);
    test_mt6816_codec();
    test_commands();
    test_params();
    test_params_running();
    test_param_effect();
    test_param_store();
    test_calibration();
    test_no_detent_model();
    test_param_mismatch();
    test_phasing(+1, 0.37);
    test_phasing(-1, 2.9);
    test_phasing(+1, -4.1);
    test_endstop();
    test_current_step();
    test_position();
    test_velocity();
    test_force_feedback();
    test_speed_limits();
    test_faults();
    test_adc_timing();
    printf("\n%s: %d checks, %d failed\n", SV_BOARD_NAME, checks, fails);
    return fails ? 1 : 0;
}
