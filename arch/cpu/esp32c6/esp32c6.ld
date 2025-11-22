/*
 * ESP32-C6 Linker Script for Direct ROM Boot
 * Code runs from IRAM, loaded directly by ROM bootloader
 */

MEMORY
{
    /* Internal SRAM - all code and data here */
    iram (rwx) : ORIGIN = 0x40800000, LENGTH = 0x60000  /* 384 KB */
}

ENTRY(_start)

SECTIONS
{
    /* Code section */
    .text : ALIGN(4)
    {
        _stext = .;
        *(.text.entry)
        *(.text .text.*)
        . = ALIGN(4);
        _etext = .;
    } > iram

    /* Read-only data */
    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > iram

    /* Initialized data */
    .data : ALIGN(4)
    {
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > iram

    /* Uninitialized data */
    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > iram

    /* Heap and stack */
    _heap_start = .;
    _heap_end = ORIGIN(iram) + LENGTH(iram) - 0x4000;

    /* Stack at end of memory */
    _stack_top = ORIGIN(iram) + LENGTH(iram);
    _stack = _stack_top - 0x4000;  /* 16KB stack */
    _stack_origin = _stack_top;
}
