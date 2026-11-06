/* r61 SDST module: everything read-only at BASE (flash XIP 80C00080, or RAM for the bench harness). No writable sections:
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
