/* Tests of the CAN firmware update (ctrl/fwup.c) on a simulated NOR flash that can lose power at
 * any erase or write, leaving the operation half done. `make -C sim check-fw`. */
#include "../ctrl/fwup.h"
#include "../ctrl/params.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails, checks;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define INFO(...) do { printf("  "); printf(__VA_ARGS__); printf("\n"); } while (0)

/* ---- NOR flash with power loss ---- */
typedef struct {
    uint8_t mem[FW_FLASH_SIZE];
    long ops, fail_after;       /* the op with index fail_after is torn; -1: never */
    bool dead;
    uint32_t rng;
    long and_writes;            /* programs onto cells that were not erased: must stay 0 */
} sflash_t;
static sflash_t F;
static uint32_t rnd(void) { F.rng ^= F.rng << 13; F.rng ^= F.rng >> 17; F.rng ^= F.rng << 5; return F.rng; }
static int sf_read(void *c, uint32_t off, uint8_t *b, uint32_t n) { (void)c; if (off + n > FW_FLASH_SIZE) return 1; memcpy(b, F.mem + off, n); return 0; }
static int sf_erase(void *c, uint32_t off)
{
    (void)c;
    if (F.dead || off % FW_PAGE || off + FW_PAGE > FW_FLASH_SIZE) return 1;
    if (F.fail_after >= 0 && F.ops == F.fail_after) {                     /* torn: some cells erased, some not */
        for (uint32_t i = 0; i < FW_PAGE; i++) if (rnd() & 1) F.mem[off + i] = 0xFF; else if (rnd() & 1) F.mem[off + i] &= (uint8_t)rnd();
        F.dead = true; F.ops++; return 1;
    }
    F.ops++; memset(F.mem + off, 0xFF, FW_PAGE); return 0;
}
static int sf_write(void *c, uint32_t off, const uint8_t *b, uint32_t n)
{
    (void)c;
    if (F.dead || off % FW_PROG_UNIT || n % FW_PROG_UNIT || off + n > FW_FLASH_SIZE) return 1;
    uint32_t words = n / FW_PROG_UNIT, upto = words;
    bool torn = F.fail_after >= 0 && F.ops == F.fail_after;
    if (torn) upto = rnd() % (words + 1);                                  /* a prefix of the words made it */
    for (uint32_t w = 0; w < upto; w++) {
        for (uint32_t k = 0; k < FW_PROG_UNIT; k++) {
            uint32_t i = off + w * FW_PROG_UNIT + k;
            if (F.mem[i] != 0xFF) F.and_writes++;
            F.mem[i] &= b[w * FW_PROG_UNIT + k];
        }
    }
    if (torn && upto < words) {                                            /* the word being written: some bits */
        uint32_t i = off + upto * FW_PROG_UNIT;
        for (uint32_t k = 0; k < FW_PROG_UNIT; k++) F.mem[i + k] &= (uint8_t)(b[upto * FW_PROG_UNIT + k] | rnd());
    }
    F.ops++;
    if (torn) { F.dead = true; return 1; }
    return 0;
}
static fw_flash_t FL = { sf_read, sf_erase, sf_write, NULL };
static void power_cycle(void) { F.dead = false; F.fail_after = -1; }
static void flash_blank(void) { memset(F.mem, 0xFF, sizeof F.mem); F.ops = 0; F.fail_after = -1; F.dead = false; F.and_writes = 0; F.rng = 0x1234567u; }

/* ---- images ---- */
#define BOARD 57
static uint8_t *make_payload(uint32_t n, uint32_t seed)
{
    uint8_t *p = malloc(n); uint32_t x = seed * 2654435761u + 1;
    for (uint32_t i = 0; i < n; i++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; p[i] = (uint8_t)x; }
    uint32_t sp = FW_RAM1_END, rv = FW_FLASH_ADDR + FW_APP_OFF + FW_HDR_SIZE + 0x41;
    memcpy(p, &sp, 4); memcpy(p + 4, &rv, 4);
    return p;
}
static uint8_t *make_image(uint32_t payload, uint32_t seed, uint16_t board, uint32_t version, uint32_t *len)
{
    uint8_t *pl = make_payload(payload, seed), *img = malloc(FW_HDR_SIZE + payload);
    fw_image_build(img, pl, payload, board, version);
    free(pl); *len = FW_HDR_SIZE + payload; return img;
}
static void put_app(const uint8_t *img, uint32_t len, uint32_t base) { memcpy(F.mem + base, img, len); }

/* ---- the CAN side of the host ---- */
static fwup_t U;
static uint32_t cmd_id(void) { return FW_ID_CMD + U.node; }
static int cmd(uint8_t code, uint32_t arg, int len, can_frame_t *rep)
{
    can_frame_t f; memset(&f, 0, sizeof f); f.id = cmd_id(); f.len = len; f.data[0] = code;
    for (int k = 0; k < 4; k++) f.data[1 + k] = (arg >> (8 * k)) & 0xFF;
    can_frame_t r; memset(&r, 0, sizeof r);
    int n = fwup_rx(&U, &f, &r);
    if (rep) *rep = r;
    return n ? r.data[1] : -1;
}
static uint32_t rep_off(const can_frame_t *r) { return (uint32_t)r->data[4] | (uint32_t)r->data[5] << 8 | (uint32_t)r->data[6] << 16 | (uint32_t)r->data[7] << 24; }
static void send_data(const uint8_t *p) { can_frame_t f; f.id = FW_ID_DATA + U.node; f.len = 8; memcpy(f.data, p, 8); can_frame_t r; fwup_rx(&U, &f, &r); }
static void new_boot(void) { fwup_init(&U, &FL, 1, BOARD, 7); }
static int do_unlock(void) { return cmd(0x30, SV_UNLOCK_KEY, 5, NULL); }

static int sync(uint32_t end, const uint8_t *chunk, uint32_t n, can_frame_t *rep)
{
    can_frame_t f; memset(&f, 0, sizeof f); f.id = cmd_id(); f.len = 7; f.data[0] = 0x11;
    for (int k = 0; k < 4; k++) f.data[1 + k] = (end >> (8 * k)) & 0xFF;
    uint16_t c = fw_crc16(chunk, n); f.data[5] = c & 0xFF; f.data[6] = c >> 8;
    can_frame_t r; memset(&r, 0, sizeof r);
    if (!fwup_rx(&U, &f, &r)) return -1;
    if (rep) *rep = r;
    return r.data[1];
}

typedef struct { int drop_frame, dup_frame; int resent_chunks; } inject_t;
/* the reference host: returns the status of END (0 = staged), or the first error seen */
static int host_transfer(const uint8_t *img, uint32_t len, inject_t *inj)
{
    can_frame_t r; int fi = 0;
    if (inj) inj->resent_chunks = 0;
    int st = do_unlock(); if (st) return st;
    st = cmd(0x10, 0, 1, &r); if (st) return st;
    uint32_t off = 0; int guard = 0;
    while (off < len) {
        if (++guard > 100000) return -9;
        uint32_t end = off + 256 < len ? off + 256 : len;
        for (uint32_t o = off; o < end; o += 8) {
            int idx = fi++;
            if (inj && idx == inj->drop_frame) continue;
            send_data(img + o);
            if (inj && idx == inj->dup_frame) send_data(img + o);
        }
        st = sync(end, img + off, end - off, &r);
        if (st == FWS_SYNC) { off = rep_off(&r); if (inj) inj->resent_chunks++; continue; }
        if (st) return st;
        off = end;
    }
    return cmd(0x12, 0, 1, &r);
}

static uint32_t hash_region(uint32_t off, uint32_t n) { return fw_crc32(0, F.mem + off, n); }

/* ---- tests ---- */
static void test_crc_header(void)
{
    printf("[crc32 and image header]\n");
    CHECK(fw_crc32(0, (const uint8_t *)"123456789", 9) == 0xCBF43926u, "crc32 test vector");
    uint32_t len; uint8_t *img = make_image(4096, 1, BOARD, 3, &len); fw_hdr_t h;
    CHECK(fw_hdr_parse(img, &h) == 0 && h.board_id == BOARD && h.fw_version == 3 && h.payload_size == 4096, "header round trip");
    int missed = 0;
    for (int byte = 0; byte < 64; byte++) for (int bit = 0; bit < 8; bit++) {
        uint8_t b[64]; memcpy(b, img, 64); b[byte] ^= (uint8_t)(1 << bit);
        if (fw_hdr_parse(b, &h) == 0) missed++;
    }
    CHECK(missed == 0, "%d of 512 single-bit header errors went unnoticed", missed);
    /* region checks */
    flash_blank();
    CHECK(fw_region_check(&FL, FW_APP_OFF, BOARD, NULL) == FWS_BAD_HEADER, "erased flash is a valid app");
    put_app(img, len, FW_APP_OFF);
    CHECK(fw_region_check(&FL, FW_APP_OFF, BOARD, NULL) == 0, "valid image refused");
    CHECK(fw_region_check(&FL, FW_APP_OFF, 42, NULL) == FWS_WRONG_BOARD, "wrong board accepted");
    F.mem[FW_APP_OFF + FW_HDR_SIZE + 1000] ^= 0x10;
    CHECK(fw_region_check(&FL, FW_APP_OFF, BOARD, NULL) == FWS_CRC, "payload bit flip accepted");
    free(img);
    uint32_t bad_sp = 0x20010000u; img = make_image(4096, 2, BOARD, 1, &len);
    uint8_t *pl = make_payload(4096, 2); memcpy(pl, &bad_sp, 4); fw_image_build(img, pl, 4096, BOARD, 1); free(pl);
    flash_blank(); put_app(img, len, FW_APP_OFF);
    CHECK(fw_region_check(&FL, FW_APP_OFF, BOARD, NULL) == FWS_IMPLAUSIBLE, "stack pointer outside RAM accepted");
    free(img);
    /* the hole between SRAM1 and SRAM2, and the top of SRAM2 */
    { uint32_t sps[3] = { 0x20005000u, FW_RAM2_END, FW_RAM2_BASE }; int want[3] = { FWS_IMPLAUSIBLE, 0, FWS_IMPLAUSIBLE };
      for (int k = 0; k < 3; k++) { pl = make_payload(4096, 4); memcpy(pl, &sps[k], 4); img = malloc(FW_HDR_SIZE + 4096); fw_image_build(img, pl, 4096, BOARD, 1); free(pl);
        flash_blank(); put_app(img, FW_HDR_SIZE + 4096, FW_APP_OFF);
        CHECK(fw_region_check(&FL, FW_APP_OFF, BOARD, NULL) == want[k], "stack pointer 0x%08x: status %d", sps[k], fw_region_check(&FL, FW_APP_OFF, BOARD, NULL)); free(img); } }
    pl = make_payload(4096, 3); uint32_t rv = 0x08000001u; memcpy(pl + 4, &rv, 4);
    img = malloc(FW_HDR_SIZE + 4096); fw_image_build(img, pl, 4096, BOARD, 1); free(pl);
    flash_blank(); put_app(img, FW_HDR_SIZE + 4096, FW_APP_OFF);
    CHECK(fw_region_check(&FL, FW_APP_OFF, BOARD, NULL) == FWS_IMPLAUSIBLE, "reset vector in the bootloader accepted");
    free(img);
}

static void test_update(void)
{
    printf("[update over CAN]\n");
    flash_blank(); new_boot();
    uint32_t l1, l2; uint8_t *v1 = make_image(20000, 11, BOARD, 1, &l1), *v2 = make_image(30000, 22, BOARD, 2, &l2);
    put_app(v1, l1, FW_APP_OFF);
    CHECK(fwb_decide(&FL, BOARD, false) == FWB_RUN_APP, "valid app not started");
    CHECK(fwb_decide(&FL, BOARD, true) == FWB_STAY, "stay request ignored");
    can_frame_t r; cmd(0x01, 0, 1, &r);
    CHECK(r.data[3] == BOARD && rep_off(&r) == 7, "info");
    int st = host_transfer(v2, l2, NULL);
    CHECK(st == 0, "transfer status %d", st);
    CHECK(U.st == FU_STAGED, "not staged");
    CHECK(memcmp(F.mem + FW_STAGE_OFF, v2, l2) == 0, "staged bytes differ");
    CHECK(memcmp(F.mem + FW_APP_OFF, v1, l1) == 0, "the running application was touched by the transfer");
    st = cmd(0x20, 0, 1, &r);
    CHECK(st == 0 && U.boot_request, "apply status %d", st);
    CHECK(memcmp(F.mem + FW_APP_OFF, v2, l2) == 0, "application differs from the new image");
    CHECK(fwb_decide(&FL, BOARD, false) == FWB_RUN_APP, "new application not started");
    CHECK(F.and_writes == 0, "%ld programs onto cells that were not erased", F.and_writes);
    INFO("30000 byte update: %ld flash operations", F.ops);
    /* a second, smaller image: the old tail must not matter */
    uint32_t l3; uint8_t *v3 = make_image(9000, 33, BOARD, 3, &l3);
    new_boot();
    CHECK(host_transfer(v3, l3, NULL) == 0 && cmd(0x20, 0, 1, &r) == 0, "second update");
    CHECK(memcmp(F.mem + FW_APP_OFF, v3, l3) == 0 && fw_region_check(&FL, FW_APP_OFF, BOARD, NULL) == 0, "second image");
    CHECK(F.and_writes == 0, "programs onto cells that were not erased: %ld", F.and_writes);
    free(v1); free(v2); free(v3);
}

static void test_lock_fuzz(void)
{
    printf("[the write lock and stray frames]\n");
    flash_blank(); new_boot();
    uint32_t l1; uint8_t *v1 = make_image(20000, 11, BOARD, 1, &l1); put_app(v1, l1, FW_APP_OFF);
    can_frame_t r;
    CHECK(cmd(0x10, 0, 1, &r) == FWS_LOCKED, "start without the key");
    CHECK(cmd(0x20, 0, 1, &r) == FWS_LOCKED, "apply without the key");
    CHECK(cmd(0x30, SV_UNLOCK_KEY ^ 1u, 5, &r) == FWS_LOCKED && U.unlock_left_ms == 0, "wrong key");
    uint32_t h0 = fw_crc32(0, F.mem, FW_FLASH_SIZE); long ops0 = F.ops;
    for (int pass = 0; pass < 2; pass++) {
        new_boot();
        if (pass == 1) { U.unlock_left_ms = 0x7fffffff; U.unlock_ms = 0x7fffffff; }
        int accepted = 0;
        for (int i = 0; i < 200000; i++) {
            can_frame_t f; f.id = (rnd() & 1) ? FW_ID_DATA + U.node : FW_ID_CMD + U.node; f.len = (uint8_t)(1 + rnd() % 8);
            for (int k = 0; k < 8; k++) f.data[k] = (uint8_t)rnd();
            if (f.id == FW_ID_CMD + U.node) {
                static const uint8_t codes[] = { 0x10, 0x11, 0x12, 0x20, 0x21, 0x30, 0x31, 0x40, 0x01 };
                f.data[0] = codes[rnd() % sizeof codes];
                if (f.data[0] == 0x30 && pass == 0) f.data[4] ^= 0x55;     /* never the key by chance: 2^-32 */
                if (pass == 1 && (f.data[0] == 0x30 || f.data[0] == 0x31)) f.data[0] = 0x01;
                if (f.data[0] == 0x11 && (rnd() & 1)) {                     /* a sync that names what the device holds: it would commit */
                    uint32_t off = U.rx_off + U.chunk_n; uint16_t c = fw_crc16(U.chunk, U.chunk_n);
                    for (int k = 0; k < 4; k++) f.data[1 + k] = (off >> (8 * k)) & 0xFF;
                    f.data[5] = c & 0xFF; f.data[6] = c >> 8; f.len = 7;
                }
            }
            can_frame_t o;
            if (fwup_rx(&U, &f, &o) && o.data[1] == 0 && (o.data[0] == 0x10 || o.data[0] == 0x20)) accepted++;
        }
        uint32_t h1 = fw_crc32(0, F.mem, FW_FLASH_SIZE);
        if (pass == 0) {
            INFO("200000 stray frames, lock closed: flash %s, %ld flash operations, state %d", h1 == h0 ? "unchanged" : "CHANGED", F.ops - ops0, U.st);
            CHECK(h1 == h0 && F.ops == ops0 && U.st == FU_IDLE, "a stray frame touched the flash");
        } else {
            INFO("control, lock held open: flash %s, %ld flash operations, %d starts accepted", h1 == h0 ? "unchanged" : "changed", F.ops - ops0, accepted);
            CHECK(h1 != h0 && F.ops > ops0, "the control run does not reach the flash: the fuzz is too weak");
        }
    }
    free(v1);
}

static void test_rejects(void)
{
    printf("[bad images are refused and the application survives]\n");
    flash_blank(); new_boot();
    uint32_t l1; uint8_t *v1 = make_image(20000, 11, BOARD, 1, &l1); put_app(v1, l1, FW_APP_OFF);
    uint32_t happ = hash_region(FW_APP_OFF, FW_APP_SIZE);
    can_frame_t r; uint32_t l; uint8_t *img;
    /* wrong board: refused when the header arrives, before the rest is sent */
    img = make_image(20000, 5, 42, 9, &l); new_boot();
    int st = host_transfer(img, l, NULL);
    INFO("image for the other board: status %d, device offset %u of %u", st, U.rx_off, l);
    CHECK(st == FWS_WRONG_BOARD && U.rx_off <= 256, "wrong board status %d, offset %u", st, U.rx_off);
    free(img);
    /* header damaged */
    img = make_image(20000, 5, BOARD, 9, &l); img[3] ^= 1; new_boot();
    CHECK(host_transfer(img, l, NULL) == FWS_BAD_HEADER, "damaged header");
    free(img);
    /* too big */
    img = make_image(20000, 5, BOARD, 9, &l); { uint8_t *pl = make_payload(FW_STAGE_SIZE, 1); free(img); img = malloc(FW_HDR_SIZE + FW_STAGE_SIZE);
      fw_image_build(img, pl, FW_STAGE_SIZE, BOARD, 1); free(pl); l = FW_HDR_SIZE + FW_STAGE_SIZE; }
    new_boot(); st = host_transfer(img, l, NULL);
    CHECK(st == FWS_TOO_BIG, "too big: status %d", st);
    free(img);
    /* a bit flipped in transit: caught at END */
    img = make_image(20000, 5, BOARD, 9, &l); img[10000] ^= 4; new_boot();
    CHECK(host_transfer(img, l, NULL) == FWS_CRC, "bit flip in transit");
    CHECK(cmd(0x20, 0, 1, &r) != 0, "apply after a failed check");
    free(img);
    /* truncated: END before the last chunk */
    img = make_image(20000, 5, BOARD, 9, &l); new_boot();
    do_unlock(); cmd(0x10, 0, 1, &r);
    for (uint32_t o = 0; o < 4096; o += 256) { for (uint32_t k = 0; k < 256; k += 8) send_data(img + o + k); sync(o + 256, img + o, 256, &r); }
    CHECK(cmd(0x12, 0, 1, &r) == FWS_BAD_STATE, "END of a truncated image");
    CHECK(cmd(0x20, 0, 1, &r) == FWS_BAD_STATE, "apply of a truncated image");
    free(img);
    /* the right size and CRC but no vector table */
    { uint8_t *pl = malloc(8192); memset(pl, 0xA5, 8192); img = malloc(FW_HDR_SIZE + 8192); fw_image_build(img, pl, 8192, BOARD, 9); free(pl);
      new_boot(); st = host_transfer(img, FW_HDR_SIZE + 8192, NULL); CHECK(st == FWS_IMPLAUSIBLE, "no vector table: status %d", st); free(img); }
    CHECK(hash_region(FW_APP_OFF, FW_APP_SIZE) == happ, "the application changed during refused updates");
    CHECK(fwb_decide(&FL, BOARD, false) == FWB_RUN_APP, "the application is not started");
    CHECK(F.and_writes == 0, "programs onto cells that were not erased: %ld", F.and_writes);
    free(v1);
}

static void test_lost_frames(void)
{
    printf("[lost and doubled frames]\n");
    uint32_t l1, l2; uint8_t *v1 = make_image(20000, 11, BOARD, 1, &l1), *v2 = make_image(30000, 22, BOARD, 2, &l2);
    int cases[4][2] = { { 100, -1 }, { -1, 100 }, { 0, -1 }, { 3000, 3001 } };
    for (int c = 0; c < 4; c++) {
        flash_blank(); new_boot(); put_app(v1, l1, FW_APP_OFF);
        inject_t inj = { cases[c][0], cases[c][1], 0 };
        int st = host_transfer(v2, l2, &inj);
        can_frame_t r;
        INFO("%s frame %d: %d chunk(s) resent, status %d", cases[c][0] >= 0 ? "lost" : "doubled", cases[c][0] >= 0 ? cases[c][0] : cases[c][1], inj.resent_chunks, st);
        CHECK(st == 0 && inj.resent_chunks >= 1, "recovery failed (status %d, resent %d)", st, inj.resent_chunks);
        CHECK(cmd(0x20, 0, 1, &r) == 0 && memcmp(F.mem + FW_APP_OFF, v2, l2) == 0, "image differs after recovery");
        CHECK(F.and_writes == 0, "programs onto cells that were not erased: %ld", F.and_writes);
    }
    free(v1); free(v2);
}

static void test_power_fail_transfer(void)
{
    printf("[power failure during the transfer]\n");
    uint32_t l1, l2; uint8_t *v1 = make_image(20000, 11, BOARD, 1, &l1), *v2 = make_image(30000, 22, BOARD, 2, &l2);
    flash_blank(); new_boot(); put_app(v1, l1, FW_APP_OFF); host_transfer(v2, l2, NULL);
    long total = F.ops; int runs = 0, bad = 0;
    for (long n = 0; n < total; n++) for (int seed = 1; seed <= 3; seed++) {
        flash_blank(); F.rng = seed * 7919u; new_boot(); put_app(v1, l1, FW_APP_OFF);
        uint32_t happ = hash_region(FW_APP_OFF, FW_APP_SIZE);
        F.fail_after = n;
        host_transfer(v2, l2, NULL);
        power_cycle(); runs++;
        if (hash_region(FW_APP_OFF, FW_APP_SIZE) != happ || fwb_decide(&FL, BOARD, false) != FWB_RUN_APP) bad++;
    }
    INFO("%d runs, power cut at each of the %ld flash operations: %d left the application damaged", runs, total, bad);
    CHECK(bad == 0, "%d runs damaged the application", bad);
    free(v1); free(v2);
}

static void test_power_fail_apply(void)
{
    printf("[power failure during the apply]\n");
    uint32_t l1, l2; uint8_t *v1 = make_image(20000, 11, BOARD, 1, &l1), *v2 = make_image(30000, 22, BOARD, 2, &l2);
    flash_blank(); new_boot(); put_app(v1, l1, FW_APP_OFF); host_transfer(v2, l2, NULL);
    static uint8_t staged[FW_FLASH_SIZE]; memcpy(staged, F.mem, FW_FLASH_SIZE);
    F.ops = 0; fw_apply(&FL, BOARD); long total = F.ops;
    int runs = 0, to_new = 0, to_old = 0, stuck = 0, wrong = 0;
    for (long n = 0; n < total; n++) for (int seed = 1; seed <= 6; seed++) {
        memcpy(F.mem, staged, FW_FLASH_SIZE); F.ops = 0; F.dead = false; F.rng = seed * 104729u + (uint32_t)n; F.fail_after = n;
        fw_apply(&FL, BOARD);
        power_cycle(); runs++;
        /* a second power failure during the recovery, on some runs */
        if (seed & 1) {
            F.ops = 0; F.fail_after = (long)(rnd() % (unsigned)(total + 1));
            if (fwb_decide(&FL, BOARD, false) == FWB_APPLY) fw_apply(&FL, BOARD);
            power_cycle();
        }
        int guard = 0; fwb_action_t a;
        while ((a = fwb_decide(&FL, BOARD, false)) == FWB_APPLY && guard++ < 3) fw_apply(&FL, BOARD);
        if (a == FWB_STAY) { stuck++; continue; }
        if (a != FWB_RUN_APP) { stuck++; continue; }
        if (memcmp(F.mem + FW_APP_OFF, v2, l2) == 0) to_new++;
        else if (memcmp(F.mem + FW_APP_OFF, v1, l1) == 0) to_old++;
        else wrong++;
    }
    INFO("%d runs, power cut at each of the %ld flash operations of the copy (every other run cut again during the recovery):", runs, total);
    INFO("  %d ended on the new image, %d on the old image (cut before the copy began), %d stuck, %d corrupt", to_new, to_old, stuck, wrong);
    CHECK(stuck == 0 && wrong == 0, "%d stuck, %d corrupt", stuck, wrong);
    CHECK(to_new > 0, "no run reached the new image");
    free(v1); free(v2);
}

static void test_decide(void)
{
    printf("[boot decision]\n");
    uint32_t l1, l2; uint8_t *v1 = make_image(8000, 11, BOARD, 1, &l1), *v2 = make_image(8000, 22, BOARD, 2, &l2);
    for (int app = 0; app < 2; app++) for (int stage = 0; stage < 2; stage++) for (int stay = 0; stay < 2; stay++) {
        flash_blank();
        if (app) put_app(v1, l1, FW_APP_OFF);
        if (stage) put_app(v2, l2, FW_STAGE_OFF);
        fwb_action_t want = app ? (stay ? FWB_STAY : FWB_RUN_APP) : (stage ? FWB_APPLY : FWB_STAY);
        fwb_action_t got = fwb_decide(&FL, BOARD, stay);
        CHECK(got == want, "app %d stage %d stay %d: action %d, expected %d", app, stage, stay, got, want);
    }
    /* a staged image for the other board is not applied */
    flash_blank(); { uint32_t l; uint8_t *o = make_image(8000, 9, 42, 1, &l); put_app(o, l, FW_STAGE_OFF); free(o); }
    CHECK(fwb_decide(&FL, BOARD, false) == FWB_STAY, "staged image of the other board applied");
    free(v1); free(v2);
}

int main(int argc, char **argv)
{
    if (argc > 4 && !strcmp(argv[1], "--make-image")) {           /* payload.bin out.img board version, for the cross-check with tools/mkfwimage.py */
        FILE *f = fopen(argv[2], "rb"); if (!f) return 2;
        static uint8_t pl[FW_APP_SIZE]; size_t n = fread(pl, 1, sizeof pl, f); fclose(f);
        while (n & 7) pl[n++] = 0xFF;
        static uint8_t img[FW_APP_SIZE + FW_HDR_SIZE];
        fw_image_build(img, pl, (uint32_t)n, (uint16_t)atoi(argv[4]), (uint32_t)atoi(argv[5]));
        f = fopen(argv[3], "wb"); fwrite(img, 1, FW_HDR_SIZE + n, f); fclose(f); return 0;
    }
    test_crc_header();
    test_decide();
    test_update();
    test_lock_fuzz();
    test_rejects();
    test_lost_frames();
    test_power_fail_transfer();
    test_power_fail_apply();
    printf("\nfirmware update: %d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
