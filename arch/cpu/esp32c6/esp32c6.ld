/*
 * ESP32-C6 Linker Script with Zephyr-style loader
 * Based on Zephyr's approach for ESP32-C6
 */

MEMORY
{
    /* Physical flash storage - where sections are actually stored in flash */
    flash_seg (R): ORIGIN = 0x0, LENGTH = 0x400000  /* 4MB flash */

    /* IRAM - internal RAM for code that needs to run from RAM */
    iram_seg (RWX): ORIGIN = 0x40800000, LENGTH = 0x10000  /* 64KB for loader */

    /* DRAM - remaining internal RAM for data */
    dram_seg (RW): ORIGIN = 0x40810000, LENGTH = 0x40000  /* 256KB */

    /* IROM - flash cache-mapped virtual address for code execution */
    irom_seg (RX): ORIGIN = 0x42000000, LENGTH = 0x400000  /* 4MB mapped */

    /* DROM - flash cache-mapped virtual address for read-only data */
    drom_seg (R): ORIGIN = 0x42800000, LENGTH = 0x400000  /* 4MB mapped */
}

ENTRY(loader_main)

SECTIONS
{
    /* Metadata section - contains load information for the loader */
    .metadata : ALIGN(4)
    {
        . = ALIGN(4);
        _metadata_start = ABSOLUTE(.);

        /* Magic number for loader */
        LONG(0xace637d3)

        /* Entry point address */
        KEEP(*(.entry_addr))

        /* IRAM metadata: VMA, LMA, Size */
        LONG(ADDR(.iram0.text))
        LONG(LOADADDR(.iram0.text))
        LONG(SIZEOF(.iram0.text))

        /* DRAM metadata: VMA, LMA, Size */
        LONG(ADDR(.dram0.data))
        LONG(LOADADDR(.dram0.data))
        LONG(SIZEOF(.dram0.data))

        /* Reserved for RTC (not used) */
        LONG(0)
        LONG(0)
        LONG(0)
        LONG(0)
        LONG(0)
        LONG(0)

        /* IROM metadata: VMA, LMA, Size */
        LONG(ADDR(.flash.text))
        LONG(LOADADDR(.flash.text))
        LONG(SIZEOF(.flash.text))

        /* DROM metadata: VMA, LMA, Size */
        LONG(ADDR(.flash.rodata))
        LONG(LOADADDR(.flash.rodata))
        LONG(SIZEOF(.flash.rodata))

        . = ALIGN(4);
        _metadata_end = ABSOLUTE(.);
    } > irom_seg AT > flash_seg

    /* Loader and critical code in IRAM */
    .iram0.text : ALIGN(4)
    {
        . = ALIGN(4);
        _iram_text_start = ABSOLUTE(.);
        _stext = .;

        /* Loader code must be in IRAM */
        *esp32c6-loader.o(.literal .literal.* .text .text.*)
        *startup.o(.literal .literal.* .text .text.*)

        . = ALIGN(16);
        _iram_text_end = ABSOLUTE(.);
    } > iram_seg AT > flash_seg

    /* Application code in flash (IROM) - will be memory mapped by loader */
    .flash.text : ALIGN(64)
    {
        . = ALIGN(64);
        _flash_text_start = ABSOLUTE(.);

        *(.text .text.*)

        . = ALIGN(4);
        _flash_text_end = ABSOLUTE(.);
    } > irom_seg AT > flash_seg

    /* Read-only data in flash (DROM) - will be memory mapped by loader */
    .flash.rodata : ALIGN(64)
    {
        . = ALIGN(64);
        _flash_rodata_start = ABSOLUTE(.);

        *(.rodata .rodata.*)
        *(.srodata .srodata.*)

        . = ALIGN(4);
        _flash_rodata_end = ABSOLUTE(.);
    } > drom_seg AT > flash_seg

    /* Initialized data in DRAM - copied from flash by ROM bootloader */
    .dram0.data : ALIGN(4)
    {
        . = ALIGN(4);
        _data_start = ABSOLUTE(.);
        __data_start = .;

        *(.data .data.*)
        *(.sdata .sdata.*)

        . = ALIGN(4);
        _data_end = ABSOLUTE(.);
        __data_end = .;
    } > dram_seg AT > flash_seg

    /* BSS in DRAM */
    .dram0.bss (NOLOAD) : ALIGN(4)
    {
        _bss_start = ABSOLUTE(.);
        __bss_start = .;

        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)

        . = ALIGN(4);
        _bss_end = ABSOLUTE(.);
        __bss_end = .;
    } > dram_seg

    /* Heap */
    .heap (NOLOAD) : ALIGN(16)
    {
        _heap_start = ABSOLUTE(.);
        __heap_start = .;
        . += 0x10000;
        _heap_end = ABSOLUTE(.);
        __heap_end = .;
    } > dram_seg

    /* Stack */
    .stack (NOLOAD) :
    {
        __stack_bottom = .;
        __stack_size_min = 0x4000;
        __stack_top = ORIGIN(dram_seg) + LENGTH(dram_seg);
        ASSERT(__stack_bottom + __stack_size_min < __stack_top, "Error: no space for stack");
    } > dram_seg

    _stack_top = ORIGIN(dram_seg) + LENGTH(dram_seg);

    /* Store image addresses for loader */
    _image_irom_start = LOADADDR(.flash.text);
    _image_irom_vaddr = ADDR(.flash.text);
    _image_irom_size = SIZEOF(.flash.text);

    _image_drom_start = LOADADDR(.flash.rodata);
    _image_drom_vaddr = ADDR(.flash.rodata);
    _image_drom_size = SIZEOF(.flash.rodata);
}
