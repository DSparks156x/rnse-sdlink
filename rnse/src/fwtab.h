/* fwtab.h -- SDLink firmware table (docs/FIRMWARE_CONTRACT.md). ONE definition of the entry order, shared by
 * tvauto.c (flash module), sdstream.c / avlib.c / sdmem_target.c (RAM program, built -DFWTAB) and gen_fwtab.py (parses the
 * FWT_SIGNED / FWT_VALUES lists below: keep one X(NAME) per line, nothing else on that line but an optional comment).
 *
 * Table layout (big-endian u32 words, in the firmware's own patch area, address passed in r4 by hook v2):
 *   [0] 'SDFW'  [1] version 1  [2] nwords (whole table)  [3] fw_id (free-form)  [4] ram_map 1
 *   [5 .. 5+FWE_N)           one word per entry, enum order
 *   [5+FWE_N .. +FWE_NSIG)   signature: first 4 bytes at each SIGNED entry's address, read from the firmware image
 *   [nwords-1]               CRC-32 (zlib) of all bytes before it
 * ram_map 1 = the firmware leaves 8F700000..8FA00000 free and 8F7FF800..8F7FFFFF usable (r5x/r6x map), AND the three YUY2 frame
 * buffers 8F4C0000..8F6EB000 (avsd.h FB0..FB2, Doom's heap: free whenever task 159 is ours). Our RAM stays literal.
 * Hardware registers (FFxxxxxx) stay literal: same SoC on every target.
 * Kernel ABI assumed identical on every target (stays literal): TCB stride 0x40, +4 state (u16, 0x0010 dormant), +0x20 entry. */
#ifndef FWTAB_H
#define FWTAB_H

#define FWT_MAGIC    0x53444657u   /* 'SDFW' */
#define FWT_VERSION  1u
#define FWT_RAMMAP_1 1u
#define FWT_HDR      5u
enum { FWT_W_MAGIC, FWT_W_VERSION, FWT_W_NWORDS, FWT_W_FWID, FWT_W_RAMMAP };

/* SIGNED entries: code (or firmware data) addresses; the table carries their first 4 bytes, tvauto compares them with flash. */
#define FWT_SIGNED(X) \
    X(DLY_TSK)      /* int dly_tsk(ms) */ \
    X(EXT_TSK)      /* void ext_tsk(void) */ \
    X(STA_TSK)      /* int sta_tsk(id, arg) */ \
    X(SND_OPN)      /* int Snd_opn(u32 param[9]) */ \
    X(SND_DYD)      /* int Snd_dyD(h, buf, len, aux) */ \
    X(SND_STP)      /* int Snd_stp(h) */ \
    X(SND_CLS)      /* int Snd_cls(h) */ \
    X(SD_READ)      /* int sd_raw_read(slot, lba, buf, n) (card-image loop) */ \
    X(SD_WRITE)     /* int sd_raw_write(slot, lba, buf, n) (sdmem replies) */ \
    X(SD_RDCORE)    /* read core sd_raw_read calls, with a timeout */ \
    X(SRCSEL)       /* int srcsel(idx): DSP input via the V850 */ \
    X(SRCIDX)       /* int srcidx(controller code) -> srcsel idx */ \
    X(QPOST)        /* int qpost(queue, msg) (bench keys) */ \
    X(PANEL)        /* void panel_cb(u8 bitmap[6]) (bench keys) */ \
    X(TVSEL)        /* int tvsel(ctx, arg) (bench: select TV) */ \
    X(IIS_EN)       /* stock player's once-per-boot IIS enable */ \
    X(DU_COMMIT)    /* int du_commit(flags): DU shadow commit */ \
    X(TASK_REST)    /* rest entry of the TV task's TCB (r6x: Doom entry) */ \
    X(CTRL_TAB)     /* data: u8[11] srcsel idx -> controller code */

/* VALUES: firmware RAM globals and firmware-specific constants. */
#define FWT_VALUES(X) \
    X(TICK_MS)      /* u32 1 ms tick */ \
    X(TCB_BASE)     /* TCB of task 0 */ \
    X(SRC_CTX)      /* u8 source context (TV = SRC_TV) */ \
    X(CTX_WORD)     /* u32 around the source context (diagnostics) */ \
    X(SRC_OVERLAY)  /* u8 source overlay (non-zero = temporary source) */ \
    X(SRC_CACHE)    /* u8 controller code last sent */ \
    X(SRC_STATUS)   /* u8 controller code reported */ \
    X(SCR_GROUP)    /* u16 screen group (TV = SCR_TV) */ \
    X(PWM_STATE)    /* u8 MMI power state */ \
    X(AUD_E216)     /* u8 mute state A (diagnostics) */ \
    X(AUD_E217)     /* u8 mute state B (diagnostics) */ \
    X(AUD_E219)     /* u8 audio state (diagnostics) */ \
    X(DMDL_HANDLE)  /* u32 MP3 player's sink handle */ \
    X(SD_VARIANT0)  /* u32 slot 0 clock variant */ \
    X(SD_BUSY0)     /* u32 slot 0 busy word */ \
    X(SD_BUSY1)     /* u32 slot 1 busy word */ \
    X(SD_LOCK0)     /* u32 slot 0 driver lock owner (diagnostics) */ \
    X(RING_N)       /* u32 DSP_DEC sink ring size */ \
    X(RING_RD)      /* u32 DSP_DEC sink ring read index */ \
    X(RING_WR)      /* u32 DSP_DEC sink ring write index */ \
    X(SH_MR)        /* DU array-0 shadow registers ... */ \
    X(SH_MWR) \
    X(SH_ALPHAR) \
    X(SH_DSXR) \
    X(SH_DSYR) \
    X(SH_DPXR) \
    X(SH_DPYR) \
    X(SH_DSA0) \
    X(SH_DSA1) \
    X(SH_DSA2) \
    X(SH_SPXR) \
    X(SH_SPYR) \
    X(SH_WASPR) \
    X(SH_WAMWR) \
    X(SH_BTR) \
    X(SH_TC1R) \
    X(SH_TC2R) \
    X(SH_MLR) \
    X(SH_DPPR) \
    X(SH_MASK)      /* array-0 mask mode */ \
    X(SH_DIRTY)     /* commit dirty flags */ \
    X(SH_LAYER0)    /* array-0 layer state */ \
    X(TASK_TV)      /* task id the module runs as (159) */ \
    X(TASK_AU)      /* free task id: sdstream audio task */ \
    X(TASK_RD)      /* free task id: sdstream reader task */ \
    X(SRC_TV)       /* SRC_CTX value of the TV source */ \
    X(SCR_TV)       /* SCR_GROUP value of the TV screen */ \
    X(PWM_ON)       /* PWM_STATE value: MMI on */ \
    X(MR_TV_A)      /* stock TV plane MR values */ \
    X(MR_TV_B) \
    X(SRCSEL_FM)    /* srcsel indices */ \
    X(SRCSEL_IIS) \
    X(SRCSEL_TV) \
    X(CTRL_FM)      /* controller codes */ \
    X(CTRL_IIS) \
    X(CTRL_TV) \
    X(IDX_LO)       /* SRC_CTX 0..7  -> srcsel idx, nibble i = ctx i, F = none */ \
    X(IDX_HI)       /* SRC_CTX 8..15 -> srcsel idx */ \
    X(KEYQ)         /* panel key queue id (bench keys) */ \
    X(KEYMSG)       /* panel key message word 0 (bench keys) */ \
    X(TVSEL_ARG)    /* tvsel arg for TV (bench) */

#define FWT_ENUM_(n) FWE_##n,
enum { FWT_SIGNED(FWT_ENUM_) FWE_NSIG, FWE_V0_ = FWE_NSIG - 1, FWT_VALUES(FWT_ENUM_) FWE_N };
#undef FWT_ENUM_
#define FWT_NWORDS  (FWT_HDR + FWE_N + FWE_NSIG + 1u)
#define FWT_SIG0    (FWT_HDR + FWE_N)

/* the includer defines FWT_BASE_PTR (an expression giving const u32 * to the table) */
#define FW(e)       ((FWT_BASE_PTR)[FWT_HDR + (e)])
#endif
