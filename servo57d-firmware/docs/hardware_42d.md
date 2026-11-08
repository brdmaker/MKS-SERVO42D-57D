# MKS SERVO42D: differences from the SERVO57D

Source: `Hardware/MKS SERVO42D_CAN V1.0_003 Schematic.pdf`, compared with
the 57D sheet (`hardware.md`). The 42D_485 V1.0_002 sheet was not read. Nothing here is
measured; **verify** marks what needs a bench check.

## Same as the 57D

- Gate drive: four EG3013 half-bridge drivers, HIN and LIN of each tied together (one
  PWM pin per leg, 120 ns dead time inside the driver).
- Current sensing: one low-side shunt per coil, in leg A+ (R9, coil A) and leg B- (R18,
  coil B), a GS8632 difference amplifier with R = 6.65k / 1k (gain 6.65) around vREF = 1.65 V
  (LM321 buffer). The amplifier orientation matches, so the signs in `servo_hw_42d.h` are
  the 57D's (**verify**).
- Encoder MT6816 (SPI on PB3..PB6), CAN transceiver TJA1051, TVS SMF26A, an 8 MHz crystal, and the
  15.4k / 1k supply divider (R21 / R22) on PA3.
- MCU pins that matter to the control: PA6 phaseA1, PA7 phaseA2, PB0 phaseB1, PB1 phaseB2 (TIM3),
  PA2 currentA, PA1 currentB, PA3 vBus, PA11 / PA12 CAN, PB3..PB6 SPI, PB7 nEN, PA0 nSTP,
  PA8 nDIR, PD0 LED.

## Different

| | SERVO42D | SERVO57D |
|---|---|---|
| MCU | N32L403KBQ7, 32-pin | N32L406CBL7, 48-pin |
| FETs | 4 × AP4008QD dual N-FETs (one per leg), datasheet not in the repo | 8 × AP050N03Q |
| shunt | **0.05 ohm** | 0.02 ohm |
| sense scaling | 0.3325 V/A, **2.42 mA/LSB**, range +-4.96 A | 0.133 V/A, 6.06 mA/LSB, +-12.4 A |
| supply parts | MD8942 buck, CL9195A33 LDO, DSK34 reverse diode, fuse 63 V 3 A | MP1584, AMS1117, SS54F |
| rated current | 3 A (manual) | 5.2 A |
| keys | Menu on PA14 (SWCLK), Next on PA13 (SWDIO), Enter on PA15: shared with the SWD pins | Menu PB9, Enter PB8, Next PA15 |
| other pins | OLED reset PA9, `M_IN` on PA10, no USART3 | OLED reset PB2, PB12..PB15 for `M_IN` / `M_OUT` |

The N32L403's flash and RAM were not checked; the upstream SDK is for the N32L406 and the L403 is in
the same N32L40x family, so it probably shares it (**verify**).

## What follows for the firmware

- `ctrl/servo_hw_42d.h` holds the 0.05 ohm shunt and the divider; build with `-DSV_BOARD_42D`.
- The amplifier can show +-4.96 A. `servo_init` limits the overcurrent trip to 90 % of that
  (`SV_I_FULLSCALE`), so a trip level above the sensing range cannot be configured.
- The sense signal per amp is 2.5 times larger than on the 57D, so a 1.5 A motor uses more of the ADC
  range and resolves 2.4 mA per count.
- The SWD pins are also keys: a debugger on J5 and the Menu / Next keys fight over PA13 / PA14. Pinout of J5 (and the 57D's J4): `programmer_pinout.png`.
- The motor is a NEMA17: lower inertia and torque, a larger detent in proportion. The simulated
  defaults are in `sim/sim_board.h`; the FET on-resistance there is an assumption.
