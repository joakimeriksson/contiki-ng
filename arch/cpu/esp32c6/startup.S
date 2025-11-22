/*
 * ESP32-C6 RISC-V Startup Code for Direct ROM Boot
 */

  .section .text.entry
  .global _start
  .type _start, @function

_start:
  /* Set up stack pointer */
  la sp, _stack_top

  /* Clear BSS section */
  la t0, _sbss
  la t1, _ebss
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

  .size _start, . - _start
