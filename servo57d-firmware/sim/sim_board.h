/* Motor, load and controller defaults of the virtual drive, per board. The
 * motors are typical catalogue values, not measured. */
#ifndef SIM_BOARD_H
#define SIM_BOARD_H
#include "../ctrl/servo_hw.h"

#if defined(SV_BOARD_42D)
/* NEMA17 (42 mm, 1.5 A class, about 0.44 Nm holding), 24 V */
#define SIM_M_R      1.6f
#define SIM_M_L      3.0e-3f
#define SIM_M_KT     0.21f          /* 0.44 Nm / (sqrt(2) * 1.5 A) */
#define SIM_M_J      5.4e-6f        /* 54 g cm^2 */
#define SIM_M_B      5.5e-4f        /* damping ratio about 0.03 at 1.5 A */
#define SIM_M_TDET   0.02f
#define SIM_RDS      0.015f         /* AP4008QD: assumed, no datasheet in the repo */
#define SIM_I_MAX    2.0f
#define SIM_I_ALIGN  1.5f
#define SIM_VEL_KP   0.016f
#define SIM_VEL_KI   1.2f
#define SIM_I_TRIP   4.0f           /* below 0.9 x the 5 A the amplifier can show */
/* hunting allowed at standstill without detent compensation, counts: the NEMA17's detent is
 * 4.5 % of its holding torque against 2.6 % for the NEMA23, so it hunts about twice as far */
#define SIM_HUNT_LIM 80
#else
/* NEMA23 (57 mm, 3 A class, about 1.9 Nm holding), 24 V */
#define SIM_M_R      0.9f
#define SIM_M_L      3.5e-3f
#define SIM_M_KT     0.45f          /* 1.9 Nm / (sqrt(2) * 3 A) */
#define SIM_M_J      5.0e-5f
#define SIM_M_B      3.0e-3f
#define SIM_M_TDET   0.05f
#define SIM_RDS      0.006f
#define SIM_I_MAX    3.0f
#define SIM_I_ALIGN  2.0f
#define SIM_VEL_KP   0.07f
#define SIM_VEL_KI   5.0f
#define SIM_I_TRIP   6.5f
#define SIM_HUNT_LIM 40
#endif

/* torque the drive can hold */
#define SIM_T_LIM    (SIM_I_MAX * SIM_M_KT)

#endif
