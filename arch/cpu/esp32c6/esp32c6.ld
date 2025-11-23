/*
 * ESP32-C6 Linker Script for ESP-IDF bootloader compatibility
 */

MEMORY
{
    /* Flash mapped memory for code and rodata */
    irom (rx)  : ORIGIN = 0x42000020, LENGTH = 0x400000 - 0x20  /* 4MB flash */

    /* Internal SRAM for data */
    dram (rw)  : ORIGIN = 0x40800000, LENGTH = 0x50000  /* 320 KB */
}

ENTRY(_start)

SECTIONS
{
    /* Single flash section with app descriptor first */
    .flash : ALIGN(4)
    {
        /* App descriptor must be first */
        *(.rodata_desc)
        . = ALIGN(4);

        /* Entry point */
        _stext = .;
        *(.text.entry)
        *(.text .text.*)
        . = ALIGN(4);
        _etext = .;

        /* Read-only data */
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
        . = ALIGN(4);
    } > irom

    /* Initialized data in DRAM */
    .data : ALIGN(4)
    {
        _sdata = .;
        *(.data .data.*)
        *(.sdata .sdata.*)
        . = ALIGN(4);
        _edata = .;
    } > dram

    /* Uninitialized data in DRAM */
    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > dram

    /* Heap and stack */
    _heap_start = .;
    _heap_end = ORIGIN(dram) + LENGTH(dram) - 0x4000;

    /* Stack at end of DRAM */
    _stack_top = ORIGIN(dram) + LENGTH(dram);
    _stack = _stack_top - 0x4000;
    _stack_origin = _stack_top;
}
