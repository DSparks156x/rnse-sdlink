/* Bounded SD memory service. Transport-independent; see SDMEM.txt. */
#ifndef SDMEM_H
#define SDMEM_H
#include "rnse.h"
#define SDMEM_CMD 0x30u
#define SDMEM_ACK 0x31u
#define SDMEM_CHANNEL 3u
#define SDMEM_LBA 0x2C000u
#define SDMEM_MAX_READ 65536u
#define SDMEM_MAX_PATCH 224u
#define SDMEM_CHUNK 472u
enum { SDM_READ=1, SDM_PATCH=2 };
enum { SDM_OK, SDM_FORMAT, SDM_RANGE, SDM_LENGTH, SDM_DISABLED, SDM_COMPARE, SDM_CRC, SDM_IO, SDM_UNSUPPORTED };
typedef int (*sdmem_read_fn)(void *,u32,u8 *,u32);
typedef int (*sdmem_write_fn)(void *,u32,const u8 *,u32);
typedef int (*sdmem_send_fn)(void *,u32,const u8 *);
typedef struct {
    void *opaque;
    sdmem_read_fn read;
    sdmem_write_fn write;
    sdmem_send_fn send;
    /* Physical RAM interval; an empty interval disables patches. */
    u32 write_lo, write_hi;
    /* Physical interval never writable (service's own buffers/code). */
    u32 protect_lo, protect_hi;
} sdmem_io;
typedef struct {
    sdmem_io io;
    u32 have_id, id, fingerprint, op, addr, length, offset;
    u32 seq, pending, final, sent, sent_at, chunk;
    u32 commands, bad, tx, retries, io_errors;
    u8 sector[512] __attribute__((aligned(32)));
} sdmem;
u32 sdmem_crc(const u8 *,u32);
void sdmem_init(sdmem *,const sdmem_io *);
/* Validates all framing before applying any command/ACK. Returns -1 on malformed TLVs. */
int sdmem_control(sdmem *,const u8 *,u32);
/* At most one raw-write call; frame_tick advances once per accepted superframe. */
void sdmem_pump(sdmem *,u32 frame_tick);
#endif
