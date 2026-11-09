#!/usr/bin/env python3
"""Wrap an application binary into the update image the CAN bootloader takes (ctrl/fwup.h).

    mkfwimage.py app.bin app.img --board 57 --version 3       build
    mkfwimage.py --check app.img --board 57                    verify an image

The binary is linked at 0x08002200 (flash offset 0x2000 plus the 0x200 header block; the
vector table is its first word). The image is a 512-byte header (magic "S57F", board, version,
payload size, payload CRC-32, header CRC-32) followed by the payload padded to 8 bytes with 0xFF.
"""
import argparse, struct, sys, zlib

HDR, MAGIC = 0x200, 0x46373553
APP_OFF, APP_SIZE, FLASH = 0x2000, 0xE000, 0x08000000
SRAM = ((0x20000000, 0x20004000), (0x20006000, 0x20008000))     # SRAM1 16 KB, SRAM2 8 KB, a hole between


def build(payload: bytes, board: int, version: int) -> bytes:
    payload = bytes(payload) + b"\xff" * (-len(payload) % 8)
    if len(payload) > APP_SIZE - HDR:
        raise SystemExit(f"payload {len(payload)} bytes does not fit the {APP_SIZE - HDR} byte application area")
    h = bytearray(b"\xff" * HDR)
    struct.pack_into("<I", h, 0, MAGIC)
    struct.pack_into("<HH", h, 4, 1, board)
    struct.pack_into("<III", h, 8, version, len(payload), zlib.crc32(payload))
    struct.pack_into("<I", h, 60, zlib.crc32(bytes(h[:60])))
    return bytes(h) + payload


def plausible(payload: bytes) -> str:
    sp, rv = struct.unpack_from("<II", payload, 0)
    if not any(lo < sp <= hi for lo, hi in SRAM):
        return f"initial stack pointer 0x{sp:08x} is not the top of SRAM1 or SRAM2"
    if not rv & 1 or not FLASH + APP_OFF + HDR <= rv < FLASH + APP_OFF + APP_SIZE:
        return f"reset vector 0x{rv:08x} is not in the application (link at 0x{FLASH + APP_OFF + HDR:08x})"
    return ""


def check(img: bytes, board: int) -> str:
    if len(img) < HDR + 8:
        return "too short"
    magic, = struct.unpack_from("<I", img, 0)
    ver, brd = struct.unpack_from("<HH", img, 4)
    fw, size, crc = struct.unpack_from("<III", img, 8)
    hcrc, = struct.unpack_from("<I", img, 60)
    if magic != MAGIC or ver != 1 or hcrc != zlib.crc32(img[:60]):
        return "bad header"
    if brd != board:
        return f"image is for board {brd}, not {board}"
    if size % 8 or len(img) != HDR + size or zlib.crc32(img[HDR:]) != crc:
        return "payload size or CRC does not match"
    return plausible(img[HDR:])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input"); ap.add_argument("output", nargs="?")
    ap.add_argument("--board", type=int, required=True, help="57 or 42")
    ap.add_argument("--version", type=int, default=1)
    ap.add_argument("--check", action="store_true", help="verify the image `input` instead of building")
    a = ap.parse_args()
    data = open(a.input, "rb").read()
    if a.check:
        err = check(data, a.board)
        print("ok" if not err else f"refused: {err}")
        sys.exit(1 if err else 0)
    if not a.output:
        ap.error("output file needed")
    warn = plausible(data + b"\xff" * 8)
    if warn:
        sys.exit(f"refusing to build: {warn}")
    img = build(data, a.board, a.version)
    open(a.output, "wb").write(img)
    print(f"{a.output}: {len(img)} bytes, board {a.board}, version {a.version}, payload CRC {zlib.crc32(img[HDR:]):08x}")


if __name__ == "__main__":
    main()
