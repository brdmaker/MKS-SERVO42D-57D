# MKS SERVO57D hardware, read from the schematic

The SERVO42D has the same topology with a 0.05 ohm shunt; see `hardware_42d.md`.

Source: `Hardware/MKS SERVO57D_CAN V1.1_001 Schematic.pdf` (the same sheet is in
`Doc/`), the EG3013 and MT6816 datasheets in `Doc/`, and the pin map in
`include/board.h`. Nothing here is measured on a board. Items marked **verify**
need a bench check before the firmware relies on them.

## Blocks

| block | parts | notes |
|---|---|---|
| input | J1 8-pin (V+, GND, Com, En, Stp, Dir, 485_B/CANL, 485_A/CANH), F1 fuse, D1 SS54F, TVS SMF26A | the manual says 12–24 V |
| 5 V | MP1584 buck (U2) | gate drivers, CAN transceiver |
| 3.3 V | AMS1117-3.3 (U1) | MCU, encoder, amplifiers |
| MCU | N32L406CBL7 (U13), 64 MHz, Cortex-M4F, 128 kB flash, 32 kB RAM | `__FPU_PRESENT 1` in the SDK |
| gate drive | 4 × EG3013 half-bridge drivers (U4, U7, U8, U10), 5 V supply, 1 µF bootstrap caps | one driver per leg |
| power stage | 8 × AP050N03Q 30 V FETs, one per switch | 30 V FETs: keep the supply at or below 24 V |
| current sense | 0.02 ohm shunts R10 (coil A) and R22 (coil B), GS8632 dual amplifier (U6A/U6B), LM321 vREF buffer (U12) | see below |
| encoder | MT6816CT-ACD (U11), 14 bit, SPI | |
| CAN | TJA1051T/3 (U15), ESD diode, SW1 (apparently the 120 ohm termination, not traced) | |
| inputs / outputs | opto-isolators U3 (Stp, Dir, En) and U21 (M_IN1/2, M_OUT1/2) | |
| user interface | OLED connector, three keys, LED | |
| debug | SWD header J4: 3V3, GND, SWCLK, SWDIO (pins 1..4), see `programmer_pinout.png` | |

## MCU pins

| pin | function | pin | function |
|---|---|---|---|
| PA0 | nSTP (opto) | PB3 | SPI1 CLK, MT6816 |
| PA1 | **currentB** (ADC ch 2) | PB4 | SPI1 MISO |
| PA2 | **currentA** (ADC ch 3) | PB5 | SPI1 MOSI |
| PA3 | **vBus** (ADC ch 4), 15.4k / 1k divider | PB6 | MT6816 CS |
| PA4 / PA5 | SCL / SDA, OLED | PB7 | nEN (opto) |
| PA6 | **phaseA1** = leg A+, TIM3 CH1 | PB8 | key Enter |
| PA7 | **phaseA2** = leg A-, TIM3 CH2 | PB9 | key Menu |
| PB0 | **phaseB1** = leg B+, TIM3 CH3 | PB10 / PB11 | TX3 / RX3 (not traced) |
| PB1 | **phaseB2** = leg B-, TIM3 CH4 | PB12–PB15 | M_IN2, M_IN1, M_OUT1, M_OUT2 |
| PB2 | OLED reset | PA8 | nDIR (opto) |
| PA9 / PA10 | TX / RX (not traced; RS-485 on the `_485` board) | PA11 / PA12 | CAN RX / TX |
| PA13 / PA14 | SWDIO / SWCLK | PA15 | key Next |
| PD0 | LED (also BOOT0) | | |

`board.h` had the Next key on PA3, which is the supply divider; fixed to PA15
(the schematic and `buttons_tick()` agree).

## Gate drive

HIN and LIN of every EG3013 are tied together and driven by one MCU pin
(`phaseXn`). The EG3013's high output follows HIN and its low output is the
inverse of LIN, so for the tied pair:

- pin high: high FET on, low FET off;
- pin low: low FET on, high FET off;
- 120 ns dead time between the two, inside the driver (datasheet), with
  cross-conduction lockout.

So each leg takes one PWM output, complementary with built-in dead time. A coil is
two legs; the coil voltage is `V(leg+) - V(leg-)`, which is `Vin * (d+ - d-)`.
With `d+ = 0.5 + x` and `d- = 0.5 - x` it is `2 x Vin`.

**Disabling the drive:** do not float the pins. The datasheet's input stage has a
10k pull-down on HIN and a 10k pull-up on LIN; with both on one node the level is
undefined. The core's disabled state drives every pin low: all four low FETs on, the
coils shorted (dynamic braking). **verify** that this is what you want for a stopped
motor, and that nothing in the stock firmware uses a different safe state.

**Bootstrap:** the high side is charged while the low FET is on, so no duty above
about 0.9 (`SV_DUTY_MAX`) or below 0.1 (`SV_DUTY_MIN`). The core clamps there.

## Current sensing

```
 leg A+ low FET source ── Test_IA ──┬── R9 1k ──► amp +  ──── R8 6.65k ── vREF
                                     R10 0.02 ohm
 GND ───────────────────── R12 1k ──────────────► amp -  ──── R14 6.65k ── out = currentA
```

Both amplifiers are the same difference amplifier:

- gain 6.65 (R14 / R12 = R8 / R9 = 6.65k / 1k);
- reference vREF = 1.65 V (3.3 V through 10k / 10k, buffered by the LM321);
- `Vout = vREF + 6.65 * 0.02 ohm * I_shunt = 1.65 V + 0.133 V/A * I_shunt`.

At 12 bit and 3.3 V that is **6.06 mA per LSB**, with +-12.4 A before the amplifier
clips (the motor tops out at 5.2 A).

**Where the current is seen.** There is one shunt per coil, in the low side of one
leg only: leg A+ for coil A, leg B- for coil B. The shunt carries the coil current only
while that leg's low FET conducts (or its body diode in the dead time). When the high
FET is on, the amplifier reads vREF. So:

1. The ADC must sample inside the low-on window of both sensed legs. With the PWM
   counter running center-aligned and a leg high while the counter is below the compare
   value, the low window is centred on the counter **peak**. Its length is
   `(1 - d) * 50 us`, 5 us at `d = 0.9`.
2. Both legs must be low at the same time, so the A+ and B- duties are kept at 0.9 or
   less (`SV_DUTY_MAX`).
3. The sample is a signed measurement: the low FET conducts in both directions.

**Signs (verify).** With the coil currents `i_A` (A+ to A-) and `i_B` (B+ to B-), the
shunt carries `-i_A` toward ground in leg A+ and `+i_B` in leg B-. The ADC value rises
with the shunt current, so `SV_SENSE_SIGN_A = -1` and `SV_SENSE_SIGN_B = +1`. If one is
wrong the current loop has positive feedback on that coil; check with a small test
voltage before enabling the motor.

**ADC timing.** The upstream `adc.c` runs the ADC free-running at 4 MHz (HCLK / 16)
with 28.5 cycles of sampling: each channel is held about 7 us after its start and
the second channel 3 us after the first. That does not fit a 5 us window triggered at the
wrong time. Either use a faster ADC clock and a short sampling time for the
current channels, or trigger it from TIM3 so that the two holds straddle the
counter peak. The simulator's `test_adc_timing` shows both cases: a trigger at the
valley reads the shunt invalid on 99 % of the samples, and the 4 MHz ADC centred on
the peak works up to 240 rpm with 0 % invalid samples.

## Encoder: MT6816 SPI frame

Command byte `0x83` (read, register 3) followed by clocked bytes (upstream sends 4
bytes in one chip-select frame, polarity high, second edge, `SPI_BR_PRESCALER_4`):

- byte 1 = register 0x03 = angle<13:6>
- byte 2 = register 0x04 = angle<5:0> in bits 7..2, `No_Mag` in bit 1, parity in bit 0
- parity is even over register 3 and register 4 bits 7..1

`ctrl/mt6816_proto.c` decodes this and returns the parity and No_Mag flags. 16384 counts
per turn; the datasheet gives under 2 us delay.

## Other findings

- The upstream stepper driver is open-loop microstepping from a sine table; it does not
  read the shunts. `adc.c` samples them continuously and only prints them.
- Not traced: the opto-isolated inputs' polarity, the M_IN / M_OUT lines, the OLED
  connector, TX3/RX3, and the RS-485 variant's differences.
- Not known: whether a given board's chip is read-protected (the upstream README says
  to unlock it with the Nations tool and set level L0 to upload).
