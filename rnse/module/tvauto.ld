/* SDLink v2 SDST module (same as r61): everything read-only at BASE (flash XIP 80D60080 = sector 107, see docs/FIRMWARE_CONTRACT.md). No writable sections:
 * state is at fixed RAM addresses (tvauto.c TVA_BASE). */
ENTRY(_tva_hdr)
SECTIONS {
  . = BASE;
  .text : { KEEP(*(.text.start)) *(.text*) }
  .rodata : { *(.rodata .rodata.str* .rodata.cst*) . = ALIGN(4); KEEP(*(.rodata.image)) . = ALIGN(4); }
  __module_end = .;
  .data : { *(.data*) *(.bss*) *(COMMON) }
  ASSERT(SIZEOF(.data) == 0, "SDST module must not have .data/.bss")
  /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
