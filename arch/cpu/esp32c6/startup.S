/*
 * ESP32-C6 RISC-V Startup Code for Contiki-NG
 * Simplified version - initializes stack and jumps to main
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

  /* Copy data section from flash to RAM */
  la t0, _sdata
  la t1, _edata
  la t2, _etext  /* Load address in flash */
copy_data:
  bge t0, t1, data_done
  lw t3, 0(t2)
  sw t3, 0(t0)
  addi t0, t0, 4
  addi t2, t2, 4
  j copy_data
data_done:

  /* Jump to main */
  call main

  /* Loop forever if main returns */
halt:
  j halt

  .size _start, . - _start
