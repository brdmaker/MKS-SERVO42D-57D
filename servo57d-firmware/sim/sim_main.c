/* Regression scenarios for the control core on the virtual MKS SERVO57D.
 * `make check` runs them all; `./sim_test --demo trace.csv` writes a trace. */
#include "sim_plant.h"
#include "../ctrl/mt6816_proto.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails, checks;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define INFO(...) do { printf("  "); printf(__VA_ARGS__); printf("\n"); } while (0)
#define TWO_PI 6.283185307179586

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
    return run_until_state(s, SV_RUN, 2.0);
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

static void test_current_step(void)
{
    printf("[current loop step, rotor blocked]\n");
    fresh();
    if (!start_run(&S)) { CHECK(0, "no RUN"); return; }
    cmd_mode(&S, SV_MODE_TORQUE); sim_run(&S, 0.01);
    S.m.locked = true;
    sim_run(&S, 0.02);
    cmd_torque_ma(&S, 2000);
    sim_run(&S, 0.0005);                                 /* CAN command lands within 1 ms */
    double t10 = -1, t90 = -1, peak = 0, t = 0;
    for (int i = 0; i < 400; i++) {                      /* 40 us steps */
        sim_run(&S, 40e-6); t += 40e-6;
        double iq = S.m.iq_true;
        if (t10 < 0 && iq >= 0.2) t10 = t;
        if (t90 < 0 && iq >= 1.8) t90 = t;
        if (iq > peak) peak = iq;
    }
    sim_run(&S, 0.05);
    double fin = S.m.iq_true;
    INFO("rise 10-90%% %.0f us, overshoot %.1f%%, final %.3f A (cmd 2.000), measured by the core %.3f A",
         (t90 - t10) * 1e6, (peak / fin - 1) * 100, fin, S.servo.iq);
    CHECK(t10 > 0 && t90 > 0, "never reached 90%%");
    CHECK((t90 - t10) > 150e-6 && (t90 - t10) < 700e-6, "rise time %.0f us out of range", (t90 - t10) * 1e6);
    CHECK(peak / fin < 1.15, "overshoot %.1f%%", (peak / fin - 1) * 100);
    CHECK(fabs(fin - 2.0) < 0.06, "steady error %.3f A", fin - 2.0);
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
    CHECK(fabs(S.servo.torque_nm - S.m.b * S.m.w) < 0.05, "torque estimate %.3f", S.servo.torque_nm);
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
    float loads[] = { 0.1f, 0.3f, 0.5f, 0.8f, 1.1f, 0.0f, -0.3f, -0.8f, -1.1f };
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
        CHECK(fabs(err) < 0.07 + 0.05 * fabs(loads[i]), "torque %.3f vs load %.3f", rep, loads[i]);
        CHECK(abs(t.pos_err) < 6, "position error %d under load", t.pos_err);
        CHECK(t.state == SV_RUN && t.fault == 0 && !t.stall, "state %d fault 0x%x stall %d", t.state, t.fault, t.stall);
    }
    INFO("worst torque error %.3f Nm over +-1.1 Nm (limit of the drive: %.2f Nm)", worst, S.servo.cfg.i_max * S.servo.cfg.kt);
    /* beyond the torque limit: stall flag, torque telemetry saturates */
    S.m.b = 0.05f;                                       /* a loaded shaft: the weight sinks at a finite speed */
    S.m.t_load = 1.6f;
    sim_run(&S, 1.0);
    drain(&S, &t);
    INFO("load 1.6 Nm (over the limit): stall %d, CAN torque %.3f Nm, iq %d mA", t.stall, t.torque_mnm / 1000.0, t.iq_ma);
    CHECK(t.stall, "stall flag not set");
    CHECK(t.fault == 0, "stall must not be a fault (0x%x)", t.fault);
    /* remove the overload: the multi-turn count kept the lost steps, the drive returns to the target */
    int32_t tgt = S.servo.cmd_pos;
    INFO("lost %d counts while overloaded", S.servo.pos - tgt);
    S.m.t_load = 0; S.m.b = 3.0e-3f;
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
    CHECK(lo > -40 && hi < 40, "hunting %.0f..%.0f counts without detent compensation", lo, hi);
    S.m.t_load = 0.5f; sim_run(&S, 0.6);
    double tq = S.servo.torque_nm;
    INFO("0.50 Nm load: torque estimate %.3f Nm", tq);
    CHECK(fabs(tq - 0.5) < 0.09, "torque estimate %.3f without detent compensation", tq);
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
        S.m.t_load = 0.6f; sim_run(&S, 0.6);
        INFO("%s: position error %d counts, 0.60 Nm load reads %.3f Nm, state %d", v[i].what, err, S.servo.torque_nm, S.servo.state);
        CHECK(abs(err) <= 4, "%s: position error %d", v[i].what, err);
        CHECK(S.servo.state == SV_RUN, "%s: left RUN (0x%x)", v[i].what, S.servo.fault);
        double want = 0.6 * v[i].kt;                          /* the estimate scales with the configured Kt */
        CHECK(fabs(S.servo.torque_nm - want) < 0.05, "%s: torque estimate %.3f, expected %.3f", v[i].what, S.servo.torque_nm, want);
    }
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
    f.len = 6; f.data[0] = 0x30; f.data[1] = 0; float nan = NAN; memcpy(f.data + 2, &nan, 4);
    float before = S.servo.cfg.pos_kp;
    n = can_proto_rx(&S.can, &S.servo, &f, r);
    CHECK(r[0].data[1] == 1 && S.servo.cfg.pos_kp == before, "NaN gain accepted");
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
    run_until_state(&S, SV_RUN, 2.0);
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
    CHECK(S.st.ia_peak < 4.0 && S.st.ib_peak < 4.0, "current peak %.2f A", fmaxf(S.st.ia_peak, S.st.ib_peak));
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
    for (int i = 0; i <= 10; i++) { S.m.t_load = 1.2f * i / 10; sim_run(&S, 0.25); }
    for (int i = 10; i >= -10; i--) { S.m.t_load = 1.2f * i / 10; sim_run(&S, 0.25); }
    S.m.t_load = 0; sim_run(&S, 0.4);
    cmd_mode(&S, SV_MODE_VELOCITY); cmd_vel(&S, 1500); sim_run(&S, 0.8);
    cmd_vel(&S, -1500); sim_run(&S, 1.0);
    fclose(S.trace);
}

int main(int argc, char **argv)
{
    if (argc > 2 && !strcmp(argv[1], "--demo")) { demo(argv[2]); return 0; }
    test_mt6816_codec();
    test_commands();
    test_calibration();
    test_no_detent_model();
    test_param_mismatch();
    test_phasing(+1, 0.37);
    test_phasing(-1, 2.9);
    test_phasing(+1, -4.1);
    test_current_step();
    test_position();
    test_velocity();
    test_force_feedback();
    test_speed_limits();
    test_faults();
    test_adc_timing();
    printf("\n%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
