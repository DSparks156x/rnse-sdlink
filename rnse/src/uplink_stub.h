/* uplink_stub.h -- sim/reference transport for capenc (see uplink_stub.c) */
#ifndef UPLINK_STUB_H
#define UPLINK_STUB_H
#include "rnse.h"
void uplink_stub_init(u8 *ring, u32 nsect, u32 space_per_frame, u32 drop_every);
void uplink_stub_tick(u32 space_per_frame);   /* new frame: reset the per-frame budget */
u32 uplink_stub_sectors(void);                /* sectors written so far */
u32 uplink_stub_lost(void);                   /* messages deliberately lost */
void uplink_stub_refuse(u32 every);           /* refuse every Nth uplink_put (returns -1) */
#endif
