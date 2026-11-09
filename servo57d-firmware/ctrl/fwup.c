#include "fwup.h"
#include "params.h"       /* SV_UNLOCK_KEY */
#include <string.h>

#define HDR_MAGIC 0x46373553u        /* "S57F" */

uint32_t fw_crc32(uint32_t crc, const uint8_t *p, size_t n)
{
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1));
    }
    return ~crc;
}

uint16_t fw_crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)*p++ << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void wr32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

/* header: 0 magic, 4 hdr_version u16, 6 board u16, 8 fw_version, 12 payload_size, 16 payload_crc,
 * 20..59 reserved (FF), 60 CRC-32 of bytes 0..59 */
int fw_hdr_parse(const uint8_t b[64], fw_hdr_t *h)
{
    if (rd32(b) != HDR_MAGIC) return FWS_BAD_HEADER;
    if (rd32(b + 60) != fw_crc32(0, b, 60)) return FWS_BAD_HEADER;
    if ((b[4] | b[5] << 8) != 1) return FWS_BAD_HEADER;
    h->board_id = (uint16_t)(b[6] | b[7] << 8);
    h->fw_version = rd32(b + 8);
    h->payload_size = rd32(b + 12);
    h->payload_crc = rd32(b + 16);
    return FWS_OK;
}

void fw_image_build(uint8_t *out, const uint8_t *payload, uint32_t payload_size, uint16_t board, uint32_t fw_version)
{
    memset(out, 0xFF, FW_HDR_SIZE);
    wr32(out, HDR_MAGIC);
    out[4] = 1; out[5] = 0; out[6] = board & 0xFF; out[7] = board >> 8;
    wr32(out + 8, fw_version); wr32(out + 12, payload_size);
    wr32(out + 16, fw_crc32(0, payload, payload_size));
    wr32(out + 60, fw_crc32(0, out, 60));
    memcpy(out + FW_HDR_SIZE, payload, payload_size);
}

int fw_region_check(const fw_flash_t *fl, uint32_t off, uint16_t board, fw_hdr_t *hout)
{
    uint8_t b[64]; fw_hdr_t h;
    if (fl->read(fl->ctx, off, b, 64)) return FWS_FLASH;
    if (fw_hdr_parse(b, &h)) return FWS_BAD_HEADER;
    if (h.board_id != board) return FWS_WRONG_BOARD;
    if (h.payload_size < 8 || (h.payload_size & 7) || h.payload_size > FW_APP_SIZE - FW_HDR_SIZE) return FWS_TOO_BIG;
    uint32_t crc = 0, done = 0; uint8_t buf[256];
    while (done < h.payload_size) {
        uint32_t n = h.payload_size - done; if (n > sizeof buf) n = sizeof buf;
        if (fl->read(fl->ctx, off + FW_HDR_SIZE + done, buf, n)) return FWS_FLASH;
        crc = fw_crc32(crc, buf, n); done += n;
    }
    if (crc != h.payload_crc) return FWS_CRC;
    /* a vector table: the stack pointer in RAM, the reset vector (Thumb) inside the application */
    if (fl->read(fl->ctx, off + FW_HDR_SIZE, buf, 8)) return FWS_FLASH;
    uint32_t sp = rd32(buf), rv = rd32(buf + 4);
    bool sp_ok = (sp > FW_RAM1_BASE && sp <= FW_RAM1_END) || (sp > FW_RAM2_BASE && sp <= FW_RAM2_END);   /* top of a stack in SRAM1 or SRAM2 */
    if (!sp_ok) return FWS_IMPLAUSIBLE;
    if (!(rv & 1) || rv < FW_FLASH_ADDR + FW_APP_OFF + FW_HDR_SIZE || rv >= FW_FLASH_ADDR + FW_APP_OFF + FW_APP_SIZE) return FWS_IMPLAUSIBLE;
    if (hout) *hout = h;
    return FWS_OK;
}

int fw_apply(const fw_flash_t *fl, uint16_t board)
{
    static uint8_t page[FW_PAGE];
    fw_hdr_t h;
    int st = fw_region_check(fl, FW_STAGE_OFF, board, &h);
    if (st) return st;
    uint32_t len = FW_HDR_SIZE + h.payload_size;
    uint32_t pages = (len + FW_PAGE - 1) / FW_PAGE;
    /* 1. the application header page goes first: from here the application is invalid */
    if (fl->erase(fl->ctx, FW_APP_OFF)) return FWS_FLASH;
    /* 2. the rest, last page first */
    for (uint32_t p = pages - 1; p >= 1; p--) {
        if (fl->read(fl->ctx, FW_STAGE_OFF + p * FW_PAGE, page, FW_PAGE)) return FWS_FLASH;
        if (fl->erase(fl->ctx, FW_APP_OFF + p * FW_PAGE)) return FWS_FLASH;
        if (fl->write(fl->ctx, FW_APP_OFF + p * FW_PAGE, page, FW_PAGE)) return FWS_FLASH;
    }
    /* 3. page 0: the payload part, then the header last */
    if (fl->read(fl->ctx, FW_STAGE_OFF, page, FW_PAGE)) return FWS_FLASH;
    if (fl->write(fl->ctx, FW_APP_OFF + FW_HDR_SIZE, page + FW_HDR_SIZE, FW_PAGE - FW_HDR_SIZE)) return FWS_FLASH;
    if (fl->write(fl->ctx, FW_APP_OFF, page, FW_HDR_SIZE)) return FWS_FLASH;
    return fw_region_check(fl, FW_APP_OFF, board, NULL);
}

fwb_action_t fwb_decide(const fw_flash_t *fl, uint16_t board, bool stay)
{
    bool app = fw_region_check(fl, FW_APP_OFF, board, NULL) == FWS_OK;
    if (!app) return fw_region_check(fl, FW_STAGE_OFF, board, NULL) == FWS_OK ? FWB_APPLY : FWB_STAY;
    return stay ? FWB_STAY : FWB_RUN_APP;
}

/* ---- the receiver ---- */
void fwup_init(fwup_t *u, const fw_flash_t *fl, uint8_t node, uint16_t board, uint32_t fw_version)
{
    memset(u, 0, sizeof *u);
    u->fl = fl; u->node = node; u->board = board; u->fw_version = fw_version;
    u->unlock_ms = 10000;
}

void fwup_tick_ms(fwup_t *u) { if (u->unlock_left_ms) u->unlock_left_ms--; }

static void reply(fwup_t *u, can_frame_t *out, uint8_t code, uint8_t status, uint32_t v)
{
    out->id = FW_ID_REPLY + u->node; out->len = 8;
    out->data[0] = code; out->data[1] = status;
    out->data[2] = 0; out->data[3] = 0;
    wr32(out->data + 4, v);
}

static void fail(fwup_t *u, uint8_t e) { u->st = FU_ERROR; u->err = e; }

static void data_frame(fwup_t *u, const can_frame_t *in)
{
    if (u->st != FU_RECV || in->len != 8 || !u->unlock_left_ms) return;
    if (u->chunk_n + 8 > sizeof u->chunk) { u->chunk_bad = true; return; }     /* a doubled frame */
    memcpy(u->chunk + u->chunk_n, in->data, 8);
    u->chunk_n += 8;
}

/* SYNC: the host says the chunk it just sent ends at `off`. Returns the status. */
static uint8_t commit_chunk(fwup_t *u, uint32_t off, int crc)
{
    uint32_t n = u->chunk_n; bool bad = u->chunk_bad;
    u->chunk_n = 0; u->chunk_bad = false;
    if (bad || off != u->rx_off + n) return FWS_SYNC;                           /* dropped: resend from rx_off */
    if (n && (crc < 0 || crc != fw_crc16(u->chunk, n))) return FWS_SYNC;        /* a frame lost and one doubled, or a damaged one */
    if (n == 0) return FWS_OK;
    if (u->hdr_known && u->rx_off + n > u->total) return FWS_SYNC;              /* surplus */
    if (u->rx_off + n > FW_STAGE_SIZE) { fail(u, FWS_TOO_BIG); return u->err; }
    uint32_t at = FW_STAGE_OFF + u->rx_off;
    if ((u->rx_off % FW_PAGE) == 0 && u->fl->erase(u->fl->ctx, at)) { fail(u, FWS_FLASH); return u->err; }
    if (u->fl->write(u->fl->ctx, at, u->chunk, n)) { fail(u, FWS_FLASH); return u->err; }
    if (u->rx_off < 64) memcpy(u->hdr + u->rx_off, u->chunk, (n < 64 - u->rx_off) ? n : 64 - u->rx_off);
    u->rx_off += n;
    u->unlock_left_ms = u->unlock_ms;
    if (!u->hdr_known && u->rx_off >= 64) {                                     /* the header is in: refuse early */
        fw_hdr_t h;
        if (fw_hdr_parse(u->hdr, &h)) { fail(u, FWS_BAD_HEADER); return u->err; }
        if (h.board_id != u->board) { fail(u, FWS_WRONG_BOARD); return u->err; }
        if (h.payload_size < 8 || (h.payload_size & 7) || FW_HDR_SIZE + h.payload_size > FW_STAGE_SIZE) { fail(u, FWS_TOO_BIG); return u->err; }
        u->total = FW_HDR_SIZE + h.payload_size; u->hdr_known = true;
        if (u->rx_off > u->total) { fail(u, FWS_TOO_BIG); return u->err; }
    }
    return FWS_OK;
}

int fwup_rx(fwup_t *u, const can_frame_t *in, can_frame_t *out)
{
    if (in->id == FW_ID_DATA + u->node) { data_frame(u, in); return 0; }
    if (in->id != FW_ID_CMD + u->node || in->len < 1) return 0;
    const uint8_t *d = in->data;
    bool unlocked = u->unlock_left_ms != 0;
    switch (d[0]) {
    case 0x01:
        reply(u, out, 0x01, 0, 0); out->data[2] = (uint8_t)u->st; out->data[3] = (uint8_t)u->board;
        wr32(out->data + 4, u->fw_version); return 1;
    case 0x30: {
        uint32_t key = in->len >= 5 ? rd32(d + 1) : 0;
        if (key == SV_UNLOCK_KEY) u->unlock_left_ms = u->unlock_ms; else u->unlock_left_ms = 0;
        reply(u, out, 0x30, key == SV_UNLOCK_KEY ? 0 : FWS_LOCKED, u->unlock_left_ms); return 1; }
    case 0x31: u->unlock_left_ms = 0; reply(u, out, 0x31, 0, 0); return 1;
    case 0x10:
        if (!unlocked) { reply(u, out, 0x10, FWS_LOCKED, 0); return 1; }
        u->st = FU_RECV; u->rx_off = 0; u->total = 0; u->hdr_known = false; u->err = 0; u->chunk_n = 0; u->chunk_bad = false;
        u->unlock_left_ms = u->unlock_ms;
        reply(u, out, 0x10, 0, 0); return 1;
    case 0x21:
        if (!unlocked) { reply(u, out, 0x21, FWS_LOCKED, 0); return 1; }
        u->st = FU_IDLE; reply(u, out, 0x21, 0, 0); return 1;
    case 0x11: {
        uint32_t off = in->len >= 5 ? rd32(d + 1) : 0;
        uint8_t s;
        if (!unlocked) s = FWS_LOCKED;
        else if (u->st == FU_ERROR) s = u->err;
        else if (u->st != FU_RECV) s = FWS_BAD_STATE;
        else s = commit_chunk(u, off, in->len >= 7 ? (d[5] | d[6] << 8) : -1);
        reply(u, out, 0x11, s, u->rx_off); return 1; }
    case 0x12: {
        if (!unlocked) { reply(u, out, 0x12, FWS_LOCKED, 0); return 1; }
        if (u->st == FU_ERROR) { reply(u, out, 0x12, u->err, u->rx_off); return 1; }
        if (u->st != FU_RECV || !u->hdr_known || u->rx_off != u->total || u->chunk_n != 0) { reply(u, out, 0x12, FWS_BAD_STATE, u->rx_off); return 1; }
        int s = fw_region_check(u->fl, FW_STAGE_OFF, u->board, NULL);
        if (s == FWS_OK) u->st = FU_STAGED; else fail(u, (uint8_t)s);
        reply(u, out, 0x12, (uint8_t)s, u->rx_off); return 1; }
    case 0x20: {
        if (!unlocked) { reply(u, out, 0x20, FWS_LOCKED, 0); return 1; }
        if (u->st != FU_STAGED) { reply(u, out, 0x20, FWS_BAD_STATE, 0); return 1; }
        int s = fw_apply(u->fl, u->board);
        if (s == FWS_OK) { u->boot_request = true; u->st = FU_IDLE; }
        reply(u, out, 0x20, (uint8_t)s, 0); return 1; }
    case 0x40: {
        int s = fw_region_check(u->fl, FW_APP_OFF, u->board, NULL);
        if (s == FWS_OK) u->boot_request = true;
        reply(u, out, 0x40, (uint8_t)s, 0); return 1; }
    default: return 0;
    }
}
