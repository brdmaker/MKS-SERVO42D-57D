# Firmware update over CAN

`ctrl/fwup.[ch]` is the update logic of a CAN bootloader: image format, receiver, staging, a
power-fail-safe apply, and the boot decision. It is plain C behind a three-function flash interface, so
the simulator (`sim/sim_fwup.c`, `make -C sim check-fw`) runs the same code on a model of the
flash that loses power half-way through any erase or write. The bootloader itself (CAN and flash drivers,
linker scripts, the jump) is **not written yet**; this is the part that decides whether a power
failure or a bad file can brick the drive, and that part is tested.

## Flash map (128 KB, offsets from 0x08000000)

| offset | size | content |
|---|---|---|
| 0x00000 | 8 KB | bootloader (CAN driver, `fwup`), never rewritten over CAN |
| 0x02000 | 56 KB | application: 512-byte image header, payload from 0x2200 (vector table first) |
| 0x10000 | 56 KB | staging: the incoming image, same layout |
| 0x1E000 | 8 KB | configuration: the saved parameter set (`params_save`) |

The page size (2 KB) and the programming unit (a 32-bit word) are confirmed by the datasheet (`Doc/N32L406CBL7.pdf`,
section 2.2.1) and the user manual, which also give the timing: 100 us per word, 2 ms (20 ms at most) per page erase. The
8 KB bootloader is still an assumption. The application is about 15 to 25 KB today. The vector-table check accepts a
stack pointer at the top of SRAM1 (0x20004000) or SRAM2 (0x20008000); the two blocks have a hole between them.

## Image

`tools/mkfwimage.py app.bin app.img --board 57 --version 3` (link the application at 0x08002200).
512-byte header: magic `S57F`, board id, version, payload size, payload CRC-32, header CRC-32; then the
payload padded to 8 bytes. The tool refuses a binary without a plausible vector table, and `--check`
verifies an image. The Python tool and the C builder write identical bytes (`make -C sim check-image`).

An image is valid when the header, the board id, the size, the payload CRC and the vector table (stack
pointer in RAM, Thumb reset vector inside the application) all check out. That is recomputed at every boot,
so no flag or journal has to survive a power failure.

## Protocol

CAN IDs, node `n`: commands `0x500+n`, data `0x700+n`, replies `0x600+n` (details in `fwup.h`).
`unlock` (the same key as the parameter write lock) -> `start` -> the image as 8-byte DATA frames in chunks
of up to 256 bytes, each chunk closed by `sync(offset after the chunk, CRC-16 of the chunk)` -> `end` ->
`apply`.

- **DATA frames only fill a RAM buffer.** `sync` writes the chunk to flash if the byte count, the offset and
  the CRC-16 all match, and otherwise drops it and answers with the device offset; the host resends that
  chunk. A lost, doubled or damaged frame costs 256 bytes and cannot shift or corrupt the data.
- **The header is checked as soon as it is in** (first chunk): an image for the other board, a damaged header
  or an oversize image is refused after 256 bytes, not after 30 KB.
- **`end`** checks the staged image completely; **`apply`** only works after a good `end`.
- **Everything that changes flash needs the unlock**, which closes by itself 10 s after the last accepted frame.
- **From the application:** command `0x50` (key, write lock open, drive off) calls `boot_cb`, which restarts
  into the bootloader. The application never writes its own flash: a flash erase stalls the CPU for tens of
  milliseconds, which a running current loop cannot take.

## Why a power failure cannot brick it

- The transfer writes only the staging area; the running application is not touched until `apply`.
- `apply` erases the application's header page first, copies the staged pages last to first, and writes the
  header last. At any moment the application is either valid or fails its CRC, and the staged copy stays valid
  until the application is.
- After reset the bootloader runs `fwb_decide`: application valid -> run it; application invalid and staged image
  valid -> finish the copy; neither -> stay and wait for a transfer.

Tested in the simulator, with torn erases (some cells erased) and torn writes (a prefix of the words, one
word half programmed):

| test | result |
|---|---|
| power cut at each of the 135 flash operations of a 30 KB transfer, 3 random tears each | 405 runs, the application was never changed or invalidated |
| power cut at each of the 31 operations of the copy, 6 tears each, half of them cut again during the recovery | 186 runs: all ended on the complete new image, none stuck, none corrupt |
| 200000 random frames to the update IDs, lock closed | flash unchanged, no flash operation; with the lock held open the same frames do reach the flash (control run) |
| wrong-board image, damaged header, oversize, bit flip in transit, truncated, no vector table | refused (at the header, or at `end`); the application unchanged |
| lost frame, doubled frame, lost + doubled in one chunk | one chunk resent each, image correct |
| every program wrote only erased cells | yes (no write onto a cell that was not erased) |

## Not done

- **The bootloader program:** CAN and flash drivers on the N32L406 SDK (`n32l40x_flash.c` is there), a linker
  script for 0x08000000 (8 KB) and one for the application at 0x08002200 with `SCB->VTOR` set, the jump, and
  the CAN bit rate. The application's `CMakeLists.txt` links at 0x08000000 today.
- **Staying in the bootloader:** after reset the bootloader should wait about 0.5 s for an `info` frame
  before it starts the application, so that an application that hangs on start (the watchdog resets it) can
  still be replaced; and `boot_cb` needs a flag that survives the reset (a RAM word the startup code does not clear).
- **What the flash driver needs** (user manual 2.2.1): unlock with the key pair `0x45670123`, `0xCDEF89AB` written to
  `FLASH_KEY` (a wrong sequence locks the controller until reset); 32-bit programming only (a half-word write is a bus
  error) and only into erased words; the internal HSI oscillator must be on while programming or erasing; check
  `FLASH_STS.BUSY` before every operation; 4 `NOP`s after each programmed word; while an operation runs the CPU stalls
  on any flash access (the bootloader's code must be able to wait, or run from SRAM); flash wait states: 0 up to 32 MHz,
  1 from 32 to 64 MHz; all flash operations stop in the low-power modes.
- **Time:** with the datasheet's numbers a 30 KB update takes about 1 s for the staging writes and erases (a
  256-byte chunk is 64 words, 6.4 ms) and at most about 1.3 s for the copy (15 pages: 20 ms erase and 51 ms
  programming each). A flash erase stalls the CPU, which is why the application never does it.
- **Recovery if the bootloader itself is broken:** the chip's ROM bootloader (system memory) programs flash over
  USART1 (also from the internal 16 MHz HSI) or USB full-speed DFU, not CAN. USART1 is PA9 / PA10 (`TX` / `RX` on the
  schematic, pads not traced); the USB D+/D- pins are the CAN pins here. To enter it BOOT0 must be high at reset
  (PD0, shared with the LED).
- **Unknowns:** whether the 8 KB bootloader fits, whether PD0 / BOOT0 (shared with the LED) interferes at
  reset, and the read protection level the first SWD flash needs. The first bootloader still goes in over SWD after unlocking the chip with the Nations tool.
- **The CRC of a running application** is only checked at boot, not while running.

Size on the M4F (`-Os`): `fwup.c` is about 1.7 kB of code and 2 kB of RAM (the page buffer of the copy and
the chunk buffer).
