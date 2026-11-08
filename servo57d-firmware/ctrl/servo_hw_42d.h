/* MKS SERVO42D (N32L403KBQ7, NEMA17): read from the CAN V1.0_003 schematic, see
 * ../docs/hardware_42d.md. Same topology as the 57D: the differences are the shunt value
 * and the parts around it. */
#ifndef SERVO_HW_42D_H
#define SERVO_HW_42D_H
#define SV_BOARD_NAME     "SERVO42D"
#define SV_SHUNT_OHM      0.05f     /* R9, R18 */
#define SV_SENSE_SIGN_A   (-1.0f)   /* shunt in leg A+, as on the 57D (verify) */
#define SV_SENSE_SIGN_B   (+1.0f)   /* shunt in leg B-, as on the 57D (verify) */
#define SV_VBUS_R_TOP     15.4f     /* R21 */
#define SV_VBUS_R_BOT     1.0f      /* R22 */
#endif
