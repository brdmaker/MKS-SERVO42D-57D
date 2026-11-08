/* Parameter registry: every setting and every measured value has an id, a type, a range
 * and flags, so the CAN protocol (and a host tool or a flash store) can read, write,
 * enumerate and save them with one piece of code. `docs/can_params.md` is generated from
 * this table (`make -C sim docs-params`). */
#ifndef PARAMS_H
#define PARAMS_H
#include <stdint.h>
#include <stddef.h>
#include "servo.h"
#include "can_proto.h"

typedef enum { PT_F32 = 0, PT_I32 = 1, PT_U16 = 2, PT_U8 = 3 } ptype_t;

#define PF_RO        0x01   /* measured or fixed: cannot be written */
#define PF_OFF_ONLY  0x02   /* can only be written while the drive is off or faulted */
#define PF_PERSIST   0x04   /* part of the saved set */
#define PF_OPEN      0x08   /* runtime command: writable while the write lock is closed */

typedef struct {
    uint16_t id;
    const char *name;
    const char *unit;
    ptype_t type;
    uint8_t target;         /* 0: servo_t, 1: can_proto_t */
    uint16_t off;
    float min, max;
    uint8_t flags;
} param_def_t;

/* status codes, also the status byte of the CAN replies */
#define PS_OK        0
#define PS_BAD_ARG   1      /* frame too short, NaN */
#define PS_UNKNOWN   2
#define PS_READ_ONLY 3
#define PS_RANGE     4
#define PS_RUNNING   5      /* PF_OFF_ONLY while the drive is running */
#define PS_UNAVAIL   6      /* no flash store behind the save command */
#define PS_LOCKED    7      /* the write lock is closed: write the key to the lock register first */

/* The write lock. Settings, the saved set and flashing are refused until the lock register
 * (parameter 0x0003) is written with this key; a wrong value closes it again. It closes by itself
 * `unlock_ms` after the last accepted write. Meant against stray frames and host bugs. */
#define SV_UNLOCK_KEY 0xA5C3E10Fu
#define PID_WRITE_LOCK 0x0003
#define PID_UNLOCK_MS  0x0004

int  params_count(void);
const param_def_t *params_at(int index);
const param_def_t *params_find(uint16_t id);

/* raw: the float's bits for PT_F32, a (sign-extended) integer otherwise */
int  params_get(const servo_t *s, const can_proto_t *c, uint16_t id, uint32_t *raw);
int  params_set(servo_t *s, can_proto_t *c, uint16_t id, uint32_t raw);

/* The CAN path: params_set behind the write lock (the lock register and PF_OPEN
 * parameters are always writable). An accepted write keeps the lock open. */
int  params_set_locked(servo_t *s, can_proto_t *c, uint16_t id, uint32_t raw);
bool params_unlocked(const can_proto_t *c);

/* Restore the saved set to the defaults taken at servo_init. Drive must be off. */
int  params_defaults(servo_t *s, can_proto_t *c);

/* Saved set: header, (id, value) pairs of every PF_PERSIST parameter, CRC-16.
 * Returns the length written (0 if cap is too small). */
size_t params_save(const servo_t *s, const can_proto_t *c, uint8_t *buf, size_t cap);
/* Apply a saved set. Returns the number of values applied, or -1 if the blob is damaged
 * (nothing is changed then). Values out of range or unknown ids are skipped. */
int  params_load(servo_t *s, can_proto_t *c, const uint8_t *buf, size_t len);

#endif
