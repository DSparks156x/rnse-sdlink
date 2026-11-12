/* sdl_ver.h -- SDLink stream format version and the start-handshake version announcement (docs/PROTOCOL.md 4.5).
 * One definition for tvauto (the announcement), hdr.S (module header +0x9C) and mkmodule.py (checks it).
 * Mirrors: teensy/sdemu/avstream.h (AVS_HS_*), pi/avlink/tlink.h (TL_HS_*), pi/avlink/rnse_ver.h (what the Pi chain produces).
 * Plain integers only (no u suffix): hdr.S includes this file.
 *
 * RULE: bump SDL_STREAM_FMT whenever the decoder's stream format changes in a way an older encoder/avlink cannot feed, or an
 * older decoder cannot play: the superframe header or TLVs, the audio format, the video container (video_fmt) or the RNV
 * bitstream it decodes. Raise SDL_STREAM_FMT_MIN only when this decoder stops playing an older format. The Pi streams only a
 * format in [SDL_STREAM_FMT_MIN, SDL_STREAM_FMT]. History: 1 = superframe v1, audio_fmt 1, video_fmt 2 (RNV2/RNV3), 2026-11-10. */
#ifndef SDL_VER_H
#define SDL_VER_H

#define SDL_STREAM_FMT      1    /* the newest stream format this decoder plays */
#define SDL_STREAM_FMT_MIN  1    /* the oldest one it still plays */

/* Announcement: 8 single-sector reads, read i at LBA SDL_HS_LBA + (i << 16) + w[i] (w = 16-bit words), i = 0..7, in order.
 * Window 0x80000..0xFFFFF lies inside RSAVLINK.BIN, below SILENCE.MP3 (LBA 0x1FF800) and above the info / test-pattern sectors. */
#define SDL_HS_LBA          0x80000
#define SDL_HS_SECT         0x80000
#define SDL_HS_VER          1    /* announcement layout version: w[0] = SDL_HS_W0 | SDL_HS_VER */
#define SDL_HS_W0           0x5D00
#define SDL_HS_N            8
#define SDL_HS_ACK          0x53444841   /* 'SDHA': the reply sector of read 7 from a handshake-aware Teensy */
/* ack verdict byte (sector +20) */
#define SDL_HS_PENDING      0
#define SDL_HS_ACCEPT       1
#define SDL_HS_REJECT       2

/* Unit only (not mirrored): tvauto hands sdstream the announcement in CTL words, so the reader can repeat it as a key request while
 * the decoder waits for a key (PROTOCOL.md 4.5). Word SDL_CTL_HSW = SDL_HSW_MAGIC, the next 4 = w[2k] << 16 | w[2k + 1]. */
#define SDL_CTL_HSW         57
#define SDL_HSW_MAGIC       0x53444857   /* 'SDHW' */

#endif
