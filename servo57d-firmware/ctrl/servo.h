/* Closed-loop stepper core for the MKS SERVO57D: field-oriented current loop
 * (20 kHz), velocity / position loops (2 kHz), force (torque) estimate.
 * Pure C with float, no register access: runs on the Cortex-M4F and in the
 * host simulator unchanged. */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include "servo_hw.h"

#define SV_SLOW_DIV 10             /* fast ticks per slow tick -> 2 kHz */

typedef enum { SV_OFF, SV_CAL, SV_ALIGN, SV_PHASE_DIR, SV_RUN, SV_FAULT, SV_PHASE_NEG } sv_state_t;
typedef enum { SV_MODE_TORQUE, SV_MODE_VELOCITY, SV_MODE_POSITION } sv_mode_t;

#define SV_FLT_OVERCURRENT 0x01
#define SV_FLT_UNDERVOLT   0x02
#define SV_FLT_OVERVOLT    0x04
#define SV_FLT_NOMAG       0x08
#define SV_FLT_PHASING     0x10
#define SV_FLT_ENCODER     0x20    /* parity error streak */

#define SV_ST_STALL        0x01    /* torque limit reached with a large error */

typedef struct {
    float r_ohm, l_h;              /* coil resistance, inductance */
    float kt;                      /* Nm per A of q current (= back-EMF V s/rad) */
    int   pole_pairs;              /* 50 for a 1.8 degree motor */
    float i_max;                   /* limit of the q current command, A */
    float i_align;                 /* current for phasing, A */
    float cur_bw_hz;               /* current loop bandwidth */
    float pos_kp;                  /* velocity command per position error, 1/s */
    float vel_kp, vel_ki;          /* A per rad/s, A per rad */
    float vel_max;                 /* rad/s */
    float vbus_min, vbus_max;      /* V */
    float i_trip;                  /* hard overcurrent trip, A */
    float torque_lpf_hz;           /* filter of the force estimate */
    float detent_nm;               /* detent (cogging) torque amplitude, 4 cycles per electrical turn; 0 = off */
    float detent_phase;            /* rad, phase of the detent against the rotor flux axis */
} servo_cfg_t;

typedef struct {
    float duty[4];                 /* A+, A-, B+, B-, 0..1 */
    bool  drive;                   /* false: all legs low (brake), no regulation */
} servo_out_t;

typedef struct {
    servo_cfg_t cfg;
    servo_cfg_t cfg_default;       /* what the config returns to on `restore defaults` */
    uint32_t    fw_version, board_id;
    sv_state_t  state;
    sv_mode_t   mode;
    uint32_t    fault;
    uint32_t    status;

    /* commands */
    float   cmd_iq;                /* A */
    float   cmd_vel;               /* rad/s */
    int32_t cmd_pos;               /* counts, multi-turn, 16384 per rev */
    float   ilim;                  /* A, runtime limit <= cfg.i_max */

    /* measurements and estimates */
    float   ia, ib, id, iq;        /* coil currents and their d/q projection, A */
    float   vbus;                  /* V */
    float   iq_filt;               /* A, filtered */
    float   torque_nm;             /* force feedback: external torque, kt * (iq - detent feed-forward), filtered */
    int32_t pos;                   /* counts, positive = positive torque */
    int32_t pos_err;               /* cmd_pos - pos while in position mode */
    float   vel;                   /* rad/s, filtered */

    /* internals */
    float   off_a, off_b;          /* ADC counts at zero current */
    float   theta_off;             /* rad, electrical offset from phasing */
    int     enc_dir;               /* +1 / -1 from phasing */
    uint16_t enc_last;
    bool    enc_valid;
    uint32_t tick, phase_tick;
    float   acc_a, acc_b;
    int32_t ph_sum, ph_ref; int ph_n;
    float   align_mean, dir_mean, neg_mean;   /* encoder at the 0, +90 and -90 degree vectors */
    float   kp_i, ki_i, int_d, int_q;
    float   int_vel;
    float   iq_ref;                /* current command from the slow loop, A */
    float   iq_ff, iq_ff_filt;     /* detent feed-forward current, A */
    int32_t pos_raw;               /* encoder counts, multi-turn, before enc_dir */
    int32_t pos_prev; int slow_cnt;
    int32_t cal_pos0; uint32_t cal_total;  /* calibration waits for a standstill */
    float   vel_raw_prev;
    int     par_err;               /* consecutive encoder parity errors */
    uint32_t stall_ticks;
    float   vq_lim;
} servo_t;

void servo_init(servo_t *s, const servo_cfg_t *cfg);
void servo_enable(servo_t *s, bool on);          /* on: run CAL, ALIGN, PHASE_DIR, RUN */
void servo_clear_fault(servo_t *s);
void servo_recalc(servo_t *s);                    /* PI gains from R, L and bandwidth; call after changing them */

void servo_set_mode(servo_t *s, sv_mode_t m);
void servo_set_torque(servo_t *s, float iq_a);
void servo_set_velocity(servo_t *s, float rad_s);
void servo_set_position(servo_t *s, int32_t counts);
void servo_set_current_limit(servo_t *s, float a);

/* One 20 kHz tick. Inputs are the raw values sampled in the low-side window
 * of the sensed legs: ADC counts of current A, current B, the supply, and the
 * 14 bit encoder angle with a flag for an MT6816 fault. */
void servo_fast(servo_t *s, uint16_t adc_a, uint16_t adc_b, uint16_t adc_vbus,
                uint16_t enc14, uint8_t enc_flags, servo_out_t *out);

#define SV_ENCF_NOMAG  0x01
#define SV_ENCF_PARITY 0x02

#endif
