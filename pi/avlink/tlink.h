/* tlink.h -- Pi <-> Teensy USB message framing (link/SPEC.md section 5.2). Little-endian, both ends are LE.
 * Must match teensy/sdemu/avstream.h. */
#ifndef AVL_TLINK_H
#define AVL_TLINK_H
#include <stdint.h>

#define TL_MAGIC 0x4B4E4C54u           /* 'TLNK' as bytes T L N K */
#define TL_HDR 16                      /* u32 magic | u16 type | u16 flags | u32 len | u32 seq */
#define TL_MAX_PAYLOAD (12 + 64 * 528) /* largest message the Teensy accepts (WIRE of 64 sectors) */

enum {
    TL_SF_BEGIN = 0x01,  /* u32 sf_seq, u16 total_sec, u16 flags */
    TL_WIRE = 0x02,      /* u32 sf_seq, u16 first_idx, u16 count, u32 fmt (4|1), count x 528|520 B */
    TL_SF_END = 0x03,    /* u32 sf_seq */
    TL_META = 0x04,
    TL_CONFIG = 0x05,    /* see tl_config */
    TL_DOWNLINK = 0x06,
    TL_FLUSH = 0x07,     /* u32 seq: drop all slots >= seq */
    TL_HOST_WRITE = 0x81,
    TL_UPLINK = 0x82,
    TL_STATUS = 0x83,    /* see tl_status */
    TL_CMDLOG = 0x84,
    TL_SERVED = 0x85,    /* u32 sf_seq, u32 t_us of its header sector */
    TL_HS_VERDICT = 0x08,  /* Pi->T: tl_hs_verdict (an older Teensy ignores it) */
    TL_RNSE_INFO = 0x86    /* T->Pi: tl_rnse_info, the unit's version announcement (docs/PROTOCOL.md 4.5) */
};

/* Version handshake = rnse/src/sdl_ver.h SDL_HS_* = teensy/sdemu/avstream.h AVS_HS_* */
#define TL_HS_LBA     0x80000u
#define TL_HS_VER     1u
#define TL_HS_PENDING 0
#define TL_HS_ACCEPT  1
#define TL_HS_REJECT  2

/* CONFIG payload (all optional: 0 = keep the Teensy's current value) */
typedef struct {
    uint16_t hold_cut_ms;     /* cut-through hold for a sector of a begun superframe (30) */
    uint16_t hold_future_ms;  /* hold for a slot not begun yet, then NOT_READY / IDLE (20) */
    uint16_t silent_ms;       /* Pi silent longer than this -> IDLE instead of NOT_READY (100) */
    uint16_t stale_ms;        /* complete, never-read, not newest slot is evicted after this (200) */
    uint8_t status_hz;        /* periodic STATUS rate (20) */
    uint8_t whole_sf;         /* 0 leave as it is, 1 off, 2 on: Teensy holds only before sector 0, until the superframe is complete (no mid-read holds) */
    uint8_t rsv[6];
} tl_config;                  /* 16 B */

/* STATUS payload, Teensy -> Pi. Credits: free_sectors - (sectors sent - rx_sectors), same for slots. */
typedef struct {
    uint16_t ver, size;       /* 1, sizeof */
    uint32_t free_sectors;    /* free wire-sector pool entries */
    uint32_t rx_sectors;      /* WIRE sectors received in total (stored or not) */
    uint32_t free_slots;      /* free superframe slots */
    uint32_t begun;           /* SF_BEGIN messages received in total */
    uint32_t newest_seq;      /* newest complete superframe (valid if flags bit0) */
    uint32_t served_seq;      /* newest superframe whose header sector was served (valid if flags bit1) */
    uint32_t sectors_served;  /* window sectors served from the pool */
    uint32_t holds;           /* start-bit holds begun (cut-through or future slot) */
    uint32_t hold_timeouts;   /* holds that ended in a synthetic sector */
    uint32_t notready;        /* NOT_READY headers served */
    uint32_t expired;         /* EXPIRED headers served */
    uint32_t idle;            /* IDLE headers served */
    uint32_t evicted;         /* slots dropped before they were read */
    uint32_t converted;       /* sectors re-encoded because the image width != bus width */
    uint32_t rx_errors;       /* malformed / unexpected messages */
    uint32_t crc7_rejects;    /* host commands with a bad CRC7 */
    uint32_t write_crc_errors;
    uint8_t card_state, bus_width, mode, flags;
    uint32_t pool_sectors;    /* pool size */
} tl_status;                  /* 80 B */

/* RNSE_INFO payload, Teensy -> Pi: the newest announcement the Teensy decoded. Sent on every new one and to every new sender. */
typedef struct {
    uint16_t ver, size;       /* 1, 32 */
    uint32_t build_id;        /* module build id (sector 107 +0x94) */
    uint8_t fmt, fmt_min;     /* stream formats the unit's decoder plays: fmt_min..fmt (rnse/src/sdl_ver.h) */
    uint8_t hs_ver, rsv0;     /* announcement word 0 (0x5D00 | layout version), low byte */
    uint32_t fw_id;           /* firmware table word 3 */
    uint16_t table_ver;       /* firmware table word 1 */
    uint16_t check;           /* announcement word 7: identifies the announcement in a verdict */
    uint32_t count;           /* announcements decoded since the Teensy booted */
    uint32_t age_ms;          /* age of the newest one when sent */
    uint32_t rsv1;
} tl_rnse_info;               /* 32 B */

/* HS_VERDICT payload, Pi -> Teensy: answered to the unit in the ack sector of its next announcement */
typedef struct {
    uint32_t build_id;
    uint16_t check;
    uint8_t verdict;          /* TL_HS_ACCEPT / TL_HS_REJECT */
    uint8_t fmt;              /* the stream format the Pi sends, 0 = none */
} tl_hs_verdict;              /* 8 B */

#endif
