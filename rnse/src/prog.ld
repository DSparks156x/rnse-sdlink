/* rnse_proto RAM program: one flat image at BASE (set with --defsym BASE=...), entry first, .bss inside the image
 * window (zeroed by crt0). Window size WIN (--defsym; 0x4000 for the probes, 0x20000 for decbench/sdstream). */
ENTRY(_start)
SECTIONS {
  . = BASE;
  .text : { KEEP(*(.text.start)) *(.text*) }
  .rodata : { *(.rodata*) }
  .data : { *(.data*) }
  . = ALIGN(4);
  __image_end = .;
  .bss : { __bss_start = .; *(.bss*) *(COMMON) . = ALIGN(4); __bss_end = .; }
  ASSERT(__bss_end <= BASE + WIN, "program window overflow")
  /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
