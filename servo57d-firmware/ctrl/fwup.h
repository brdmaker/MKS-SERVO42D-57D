/* Firmware update over CAN: image format, staging, power-fail-safe apply, bootloader decision.
 * Plain C behind a flash interface, so the same code runs in the bootloader on the MCU and in
 * the simulator, which can cut the power at any flash operation (sim/sim_fwup.c).
 *
 * Flash (128 KB, offsets from 0x08000000; 2 KB pages and 32-bit word programming, datasheet 2.2.1):
 *   0x00000  8 KB  bootloader        CAN driver, this code; never rewritten over CAN
 *   0x02000 56 KB  application       512 B image header, then the payload (vector table at +0x200)
 *   0x10000 56 KB  staging           the incoming image, same layout; the application is untouched
 *   0x1E000  8 KB  configuration     the saved parameter set
 *
 * The application is valid when its header and the CRC-32 of its payload check out; that is
 * recomputed at every boot, so nothing else (no flag, no journal) has to survive a power failure.
 * An update is: START, the image in chunks of up to 256 bytes (8-byte DATA frames, header first),
 * each chunk closed by a SYNC that names the offset after it, END (the staged image is checked),
 * APPLY. DATA frames only fill a RAM buffer; SYNC writes the chunk to flash if exactly the expected
 * bytes arrived, otherwise it drops the chunk and answers with the device offset, so a lost or
 * doubled frame costs one chunk and cannot shift the data. APPLY erases the application header page first, copies
 * the staged pages from the last to the first and writes the header last. A power failure at any
 * point leaves either the old image or an invalid application with a valid staged image, and the
 * bootloader finishes the copy at the next start.
 *
 * CAN IDs (node n, 1..127): commands 0x500+n, data 0x700+n, replies 0x600+n.
 *   0x01 info    -> [0x01, 0, state, board u8, fw_version u32]
 *   0x30 unlock  key u32 (SV_UNLOCK_KEY, params.h)       0x31 lock
 *   0x10 start   erase-on-demand, offset 0               0x21 abort
 *   0x11 sync    offset after the chunk u32, CRC-16 of the chunk u16 -> [0x11, status, device offset u32];
 *                status 1: the chunk was not what the offset and the CRC say, it is dropped: resend
 *                from the device offset
 *   0x12 end     verify the staged image                 0x20 apply   0x40 boot the application
 * Everything that changes flash needs the unlock; it closes 10 s after the last accepted frame. */
#ifndef FWUP_H
#define FWUP_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "can_proto.h"

#define FW_PAGE        2048u
#define FW_PROG_UNIT   4u
#define FW_FLASH_SIZE  (128u * 1024u)
#define FW_BOOT_OFF    0x00000u
#define FW_APP_OFF     0x02000u
#define FW_APP_SIZE    0x0E000u
#define FW_STAGE_OFF   0x10000u
#define FW_STAGE_SIZE  0x0E000u
#define FW_CFG_OFF     0x1E000u
#define FW_HDR_SIZE    0x200u
#define FW_FLASH_ADDR  0x08000000u
#define FW_RAM_BASE    0x20000000u
#define FW_RAM_SIZE    0x6000u            /* 24 KB SRAM (datasheet; the upstream linker script says 32 KB) */

#define FW_ID_CMD   0x500u
#define FW_ID_REPLY 0x600u
#define FW_ID_DATA  0x700u

/* statuses (replies, and the return values of the checks) */
#define FWS_OK          0
#define FWS_SYNC        1
#define FWS_LOCKED      2
#define FWS_BAD_STATE   3
#define FWS_BAD_HEADER  4
#define FWS_WRONG_BOARD 5
#define FWS_TOO_BIG     6
#define FWS_CRC         7
#define FWS_FLASH       8
#define FWS_IMPLAUSIBLE 9

typedef struct {
    int (*read)(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);
    int (*erase)(void *ctx, uint32_t off);                          /* one page */
    int (*write)(void *ctx, uint32_t off, const uint8_t *buf, uint32_t n);  /* n and off multiples of FW_PROG_UNIT, erased cells only */
    void *ctx;
} fw_flash_t;

typedef struct {
    uint16_t board_id;
    uint32_t fw_version, payload_size, payload_crc;
} fw_hdr_t;

uint32_t fw_crc32(uint32_t crc, const uint8_t *p, size_t n);       /* crc starts at 0: the usual CRC-32 */
uint16_t fw_crc16(const uint8_t *p, size_t n);                      /* CRC-16/CCITT-FALSE, the chunk check */
int      fw_hdr_parse(const uint8_t b[64], fw_hdr_t *h);           /* 0 or FWS_BAD_HEADER */
void     fw_image_build(uint8_t *out, const uint8_t *payload, uint32_t payload_size,
                        uint16_t board, uint32_t fw_version);       /* out: FW_HDR_SIZE + payload_size bytes */

/* Check the image in a region: header, board, size, payload CRC, plausible vectors. 0 or FWS_*. */
int  fw_region_check(const fw_flash_t *fl, uint32_t off, uint16_t board, fw_hdr_t *h);
/* Copy the staged image over the application, power-fail safe. 0 or FWS_*. */
int  fw_apply(const fw_flash_t *fl, uint16_t board);

typedef enum { FWB_RUN_APP, FWB_APPLY, FWB_STAY } fwb_action_t;
/* What the bootloader does after reset: finish an interrupted copy, stay for a new image, or run. */
fwb_action_t fwb_decide(const fw_flash_t *fl, uint16_t board, bool stay_requested);

typedef struct {
    const fw_flash_t *fl;
    uint16_t board; uint32_t fw_version; uint8_t node;
    uint32_t unlock_ms, unlock_left_ms;
    enum { FU_IDLE = 0, FU_RECV = 1, FU_ERROR = 2, FU_STAGED = 3 } st;
    uint32_t rx_off, total;       /* bytes committed to flash, image length once the header is in */
    uint8_t  chunk[256]; uint32_t chunk_n; bool chunk_bad;
    uint8_t  hdr[64]; bool hdr_known;
    uint8_t  err;
    bool     boot_request;       /* the glue resets / jumps to the application */
} fwup_t;

void fwup_init(fwup_t *u, const fw_flash_t *fl, uint8_t node, uint16_t board, uint32_t fw_version);
/* one received frame; replies to out[] (max 1); returns the count */
int  fwup_rx(fwup_t *u, const can_frame_t *in, can_frame_t *out);
void fwup_tick_ms(fwup_t *u);

#endif
