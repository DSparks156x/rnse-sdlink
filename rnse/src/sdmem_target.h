/* r56b/r57 adapter; no I/O happens until a valid command or reply pump. */
#ifndef SDMEM_TARGET_H
#define SDMEM_TARGET_H
#include "sdmem.h"
int sdmem_target_init(sdmem *,u32 slot,u32 arm,u32 lo,u32 hi);
#endif
