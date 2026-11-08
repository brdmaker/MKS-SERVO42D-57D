# Upstream

Copied from https://github.com/Delsian/servo57d
(commit 5d400b01bc8afee0f2c17f4039408f82b5164584, 2024-11-12, "CAN tx"),
licence GPL-3.0 (`LICENSE`). Third-party firmware for the MKS SERVO57D
(N32L406); not MakerBase's source.

## Local changes

- `include/board.h`: `BTN3_PIN` was `GPIO_PIN_3` (PA3, the supply-voltage
  divider input); the schematic has `KEY_NEXT` on PA15 and `buttons_tick()` already
  reads bit 15. Comments on the current sensing and the gate drive.
- `ctrl/`: portable control core (current loop, velocity / position loops, force
  estimate, CAN protocol, MT6816 frame decoder) for the SERVO57D and SERVO42D
  (`-DSV_BOARD_42D`). Not yet called from `src/`.
- `sim/`: host simulator and regression tests for `ctrl/`, both boards.
- `tools/mkfwimage.py`: builds the CAN update image.
- `docs/`: hardware notes read from the schematics (57D, 42D), design notes of the control core.
