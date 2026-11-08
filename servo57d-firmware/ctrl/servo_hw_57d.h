/* MKS SERVO57D (N32L406CBL7, NEMA23): read from the V1.1 schematic, see ../docs/hardware.md. */
#ifndef SERVO_HW_57D_H
#define SERVO_HW_57D_H
#define SV_BOARD_NAME     "SERVO57D"
#define SV_BOARD_ID       57
#define SV_SHUNT_OHM      0.02f     /* R10, R22 */
/* Coil A is sensed in leg A+, coil B in leg B-. With i_A flowing A+ -> A- and i_B
 * flowing B+ -> B-, the shunt carries -i_A and +i_B toward ground; the ADC rises
 * with the shunt current. (verify on the board) */
#define SV_SENSE_SIGN_A   (-1.0f)
#define SV_SENSE_SIGN_B   (+1.0f)
#define SV_VBUS_R_TOP     15.4f     /* R28, kohm */
#define SV_VBUS_R_BOT     1.0f      /* R29, kohm */
#endif
