Portable control core for the MKS SERVO57D; see `../docs/control.md`.
`servo.c` current / velocity / position loops and the force estimate, `can_proto.c`
CAN commands and telemetry, `mt6816_proto.c` the encoder frame. `servo_hw.h` picks `servo_hw_57d.h` or `servo_hw_42d.h` (`-DSV_BOARD_42D`).
Tested in `../sim`, both boards.
