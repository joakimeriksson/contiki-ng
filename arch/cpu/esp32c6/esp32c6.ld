/* ESP32-C6 Linker Script for Contiki-NG */
/* Simplified version - for full implementation use ESP-IDF linker scripts */

MEMORY
{
  /* Internal SRAM */
  iram (rwx) : ORIGIN = 0x40800000, LENGTH = 0x50000  /* 320 KB IRAM */
  dram (rw)  : ORIGIN = 0x40850000, LENGTH = 0x14000  /* 80 KB DRAM */

  /* External Flash (code) */
  flash (rx) : ORIGIN = 0x42000000, LENGTH = 0x800000 /* 8 MB */
}

ENTRY(_start)

SECTIONS
{
  .text : {
    _stext = .;
    *(.text.entry)
    *(.text .text.*)
    *(.rodata .rodata.*)
    _etext = .;
  } > flash

  .data : {
    _sdata = .;
    *(.data .data.*)
    _edata = .;
  } > dram AT > flash

  .bss (NOLOAD) : {
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    _ebss = .;
  } > dram

  _heap_start = .;
  _heap_end = ORIGIN(dram) + LENGTH(dram);

  /* Stack symbols */
  _stack_top = ORIGIN(iram) + LENGTH(iram);
  _stack = ORIGIN(iram) + LENGTH(iram) - 0x4000;  /* 16KB stack */
  _stack_origin = ORIGIN(iram) + LENGTH(iram);
}
