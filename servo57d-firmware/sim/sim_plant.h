/* Virtual MKS SERVO57D: stepper motor, 4 EG3013 half-bridges with AP050N03Q
 * FETs, shunt + difference amplifier + 12 bit ADC, MT6816 encoder, CAN
 * mailboxes, TIM3 center-aligned PWM, and the ISR scheduling, wired to the
 * real control core in ../ctrl. Time step 62.5 ns (16 MHz). */
#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "../ctrl/servo.h"
#include "../ctrl/can_proto.h"

#define SIM_TICK_HZ   16000000.0
#define SIM_PERIOD    800            /* ticks per PWM period: 20 kHz */
#define SIM_HALF      (SIM_PERIOD / 2)
#define SIM_DEAD      2              /* EG3013 dead time 120 ns -> 2 ticks */
#define SIM_MOT_DIV   8              /* motor integrated every 0.5 us */
#define SIM_TICKS_MS  16000

typedef struct {
    /* parameters */
    float r, l, kt, j, b, tdet;      /* ohm, H, Nm/A, kg m^2, Nm s/rad, Nm */
    int   pp;
    /* state */
    double ia, ib, w, th;            /* A, A, rad/s, rad (mechanical) */
    /* load and test hooks */
    float t_load;                    /* constant external torque, Nm */
    float t_spring;                  /* Nm/rad about th_spring0 (0 = none) */
    double th_spring0;
    bool  locked;                    /* rotor blocked (current-step tests) */
    double stop_lo, stop_hi;         /* hard mechanical endstops, rad (default: none) */
    int   enc_sign;                  /* magnet orientation: encoder counts up with th * enc_sign */
    /* truth for the tests */
    float t_elec;                    /* torque from the coils, Nm */
    float iq_true;                   /* A */
    float t_ext;                     /* external torque seen, Nm */
} sim_motor_t;

typedef struct {
    float vin;                       /* supply, V */
    float rds, vf;                   /* FET on resistance, body diode drop */
    float amp_tau;                   /* amplifier settling, s */
    float amp_vos_a, amp_vos_b;      /* output-referred offsets, V */
    float amp_gain_err;              /* relative gain error of both amplifiers */
    float adc_noise_lsb;
    double adc_hz; float adc_smp_cycles, adc_conv_cycles;
    int   trig_offset_ticks;         /* ADC trigger relative to the carrier peak */
    float enc_noise_lsb;
    bool  enc_nomag;                 /* inject MT6816 No_Mag */
    int   enc_parity_glitches;       /* corrupt this many next frames */
} sim_hw_t;

typedef struct {
    uint64_t a_samples, a_invalid, b_samples, b_invalid;  /* shunt not conducting at the hold */
    uint64_t isr_count;
    float    ia_peak, ib_peak;
} sim_stats_t;

typedef struct { int hs, ls, dead, want, s_prev; } sim_leg_t;

typedef struct {
    sim_motor_t m;
    sim_hw_t    hw;
    sim_stats_t st;
    servo_t     servo;
    can_proto_t can;
    sim_leg_t   leg[4];
    uint64_t    tick;
    int         cmp[4], cmp_pending[4];
    bool        drive;
    double      amp_a, amp_b;        /* amplifier outputs, V */
    /* ADC scheduling */
    int64_t     t_hold_a, t_hold_b, t_hold_v, t_isr;
    uint16_t    adc_a, adc_b, adc_v;
    uint16_t    enc_now, enc_prev;   /* latched at the trigger; the core gets the older one */
    double      va_acc, vb_acc;
    uint32_t    rng;
    /* CAN mailboxes */
    can_frame_t to_drive[256]; int to_drive_n;
    can_frame_t from_drive[256]; int from_drive_head, from_drive_n;
    uint64_t    can_rx_time_ticks[256];
    /* trace */
    FILE *trace; uint64_t trace_next;
} sim_t;

void sim_default_motor(sim_motor_t *m);
void sim_default_hw(sim_hw_t *h);
void sim_default_cfg(servo_cfg_t *c);
void sim_init(sim_t *s);                   /* defaults for everything */
void sim_run(sim_t *s, double seconds);
double sim_time(const sim_t *s);

void sim_can_send(sim_t *s, const can_frame_t *f);          /* host -> drive */
bool sim_can_recv(sim_t *s, can_frame_t *f);                /* drive -> host */
void sim_can_cmd(sim_t *s, uint8_t cmd, const void *arg, int arglen);

/* true electrical angle of the rotor flux in the alpha frame, rad */
double sim_theta_e(const sim_t *s);

#endif
