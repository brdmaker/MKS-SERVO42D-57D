/* MKS SERVO57D hardware constants, read from the V1.1 schematic
 * (see ../docs/hardware.md). Shared by the firmware core and the simulator.
 * Items marked "verify" must be checked on a real board. */
#ifndef SERVO_HW_H
#define SERVO_HW_H

#define SV_PWM_HZ        20000.0f   /* current-loop rate; one PWM period */
#define SV_TS            (1.0f / SV_PWM_HZ)

/* Current sensing: 0.02 ohm shunt (R10/R22) in the low side of one leg per
 * coil, difference amplifier GS8632 with gain R14/R12 = 6.65k/1k, referenced
 * to vREF = 3.3 V / 2 (LM321 buffer). ADC: 12 bit, 3.3 V. */
#define SV_VDDA          3.3f
#define SV_ADC_FULL      4095.0f
#define SV_SHUNT_OHM     0.02f
#define SV_AMP_GAIN      6.65f
#define SV_VREF          1.65f
#define SV_AMPS_PER_LSB  (SV_VDDA / SV_ADC_FULL / (SV_SHUNT_OHM * SV_AMP_GAIN))

/* Coil A is sensed in leg A+ (PA6), coil B in leg B- (PB1). With the coil
 * current i_A flowing A+ -> A- and i_B flowing B+ -> B-, the shunt carries
 * -i_A and +i_B toward ground. ADC counts rise with the shunt current. (verify) */
#define SV_SENSE_SIGN_A  (-1.0f)
#define SV_SENSE_SIGN_B  (+1.0f)

/* Supply: VIN -> 15.4k / 1k divider (R28 / R29) -> PA3 */
#define SV_VBUS_PER_LSB  (SV_VDDA / SV_ADC_FULL * (15.4f + 1.0f) / 1.0f)

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
