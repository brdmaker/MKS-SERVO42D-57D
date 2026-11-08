/* Hardware constants shared by the firmware core and the simulator. The board
 * is chosen at build time: -DSV_BOARD_42D or -DSV_BOARD_57D (the default). Both
 * boards have the same topology (EG3013 half-bridges with HIN = LIN, a low-side
 * shunt in leg A+ and leg B-, GS8632 difference amplifier, MT6816), so only the
 * constants in servo_hw_<board>.h differ. Items marked "verify" need a bench check. */
#ifndef SERVO_HW_H
#define SERVO_HW_H

#if defined(SV_BOARD_42D)
#include "servo_hw_42d.h"
#else
#include "servo_hw_57d.h"
#endif

/* firmware protocol version, raised when the CAN frames or the parameter ids change */
#define SV_FW_VERSION    1

#define SV_PWM_HZ        20000.0f   /* current-loop rate; one PWM period */
#define SV_TS            (1.0f / SV_PWM_HZ)

/* Current sensing: low-side shunt per coil, difference amplifier gain
 * R14/R12 = 6.65k/1k referenced to vREF = 3.3 V / 2 (LM321 buffer). ADC: 12 bit, 3.3 V. */
#define SV_VDDA          3.3f
#define SV_ADC_FULL      4095.0f
#define SV_AMP_GAIN      6.65f
#define SV_VREF          1.65f
#define SV_AMPS_PER_LSB  (SV_VDDA / SV_ADC_FULL / (SV_SHUNT_OHM * SV_AMP_GAIN))
/* current at which the amplifier output reaches a rail: the sensing range */
#define SV_I_FULLSCALE   (SV_VREF / (SV_SHUNT_OHM * SV_AMP_GAIN))

/* Supply through the divider on PA3 */
#define SV_VBUS_PER_LSB  (SV_VDDA / SV_ADC_FULL * (SV_VBUS_R_TOP + SV_VBUS_R_BOT) / SV_VBUS_R_BOT)

/* Every half-bridge has HIN and LIN of its EG3013 tied together: one PWM
 * input, 1 = high FET on, 0 = low FET on, 120 ns dead time inside the driver.
 * Legs are ordered A+, A-, B+, B- = PA6, PA7, PB0, PB1 (TIM3 CH1..CH4). */
#define SV_LEG_AP 0
#define SV_LEG_AM 1
#define SV_LEG_BP 2
#define SV_LEG_BM 3

/* Duty range that keeps the sensed low-side window and the bootstrap
 * recharge time open. */
#define SV_DUTY_MIN      0.10f
#define SV_DUTY_MAX      0.90f

#define SV_ENC_COUNTS    16384      /* MT6816, 14 bit */

#endif
