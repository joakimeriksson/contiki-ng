/*
 * ESP32-C6 RISC-V Startup Code for Loader-based Boot
 * This is called by the loader after MMU setup
 * Data is already copied by ROM bootloader, we only need to clear BSS
 */

  .section .text.entry
  .global __start
  .type __start, @function

__start:
  /* Set up stack pointer */
  la sp, _stack_top

  /* Clear BSS section */
  la t0, __bss_start
  la t1, __bss_end
clear_bss:
  bge t0, t1, bss_done
  sw zero, 0(t0)
  addi t0, t0, 4
  j clear_bss
bss_done:

  /* Jump to main */
  call main

  /* Loop forever if main returns */
halt:
  j halt

  .size __start, . - __start
