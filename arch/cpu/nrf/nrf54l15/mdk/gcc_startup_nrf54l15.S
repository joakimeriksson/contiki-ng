/*
 *
 * Copyright (c) 2009-2025 ARM Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NOTICE: This file has been adapted for the Nordic nRF54L15 application core.
 */

    .syntax unified
    .arch armv8-m.main

#ifdef __STARTUP_CONFIG
#include "startup_config.h"
#ifndef __STARTUP_CONFIG_STACK_ALIGNEMENT
#define __STARTUP_CONFIG_STACK_ALIGNEMENT 3
#endif
#endif

    .section .stack
#if defined(__STARTUP_CONFIG)
    .align __STARTUP_CONFIG_STACK_ALIGNEMENT
    .equ    Stack_Size, __STARTUP_CONFIG_STACK_SIZE
#elif defined(__STACK_SIZE)
    .align 3
    .equ    Stack_Size, __STACK_SIZE
#else
    .align 3
    .equ    Stack_Size, 16384
#endif
    .globl __StackTop
    .globl __StackLimit
__StackLimit:
    .space Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#if defined(__STARTUP_CONFIG)
    .equ Heap_Size, __STARTUP_CONFIG_HEAP_SIZE
#elif defined(__HEAP_SIZE)
    .equ Heap_Size, __HEAP_SIZE
#else
    .equ Heap_Size, 16384
#endif
    .globl __HeapBase
    .globl __HeapLimit
__HeapBase:
    .if Heap_Size
    .space Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector, "ax"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                  /* Top of Stack */
    .long   Reset_Handler
    .long   NMI_Handler
    .long   HardFault_Handler
    .long   MemoryManagement_Handler
    .long   BusFault_Handler
    .long   UsageFault_Handler
    .long   SecureFault_Handler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SVC_Handler
    .long   DebugMon_Handler
    .long   0                           /*Reserved */
    .long   PendSV_Handler
    .long   SysTick_Handler

  /* External Interrupts */
    .rept   28                          /* IRQ 0-27 */
    .long   0
    .endr
    .long   SWI00_IRQHandler            /* 28 */
    .long   SWI01_IRQHandler            /* 29 */
    .long   SWI02_IRQHandler            /* 30 */
    .long   SWI03_IRQHandler            /* 31 */
    .rept   (64-32)                     /* 32-63 */
    .long   0
    .endr
    .long   SPU00_IRQHandler            /* 64 */
    .long   MPC00_IRQHandler            /* 65 */
    .rept   (70-66)                     /* 66-69 */
    .long   0
    .endr
    .long   AAR00_CCM00_IRQHandler      /* 70 */
    .long   ECB00_IRQHandler            /* 71 */
    .long   CRACEN_IRQHandler           /* 72 */
    .long   0                           /* 73 reserved */
    .long   SERIAL00_IRQHandler         /* 74 */
    .long   RRAMC_IRQHandler            /* 75 */
    .long   VPR00_IRQHandler            /* 76 */
    .rept   (82-77)                     /* 77-81 */
    .long   0
    .endr
    .long   CTRLAP_IRQHandler           /* 82 */
    .long   0                           /* 83 reserved */
    .long   CM33SS_IRQHandler           /* 84 */
    .long   TIMER00_IRQHandler          /* 85 */
    .rept   (128-86)                    /* 86-127 */
    .long   0
    .endr
    .long   SPU10_IRQHandler            /* 128 */
    .rept   (133-129)                   /* 129-132 */
    .long   0
    .endr
    .long   TIMER10_IRQHandler          /* 133 */
    .long   0                           /* 134 reserved */
    .long   EGU10_IRQHandler            /* 135 */
    .long   0                           /* 136 reserved */
    .long   0                           /* 137 reserved */
    .long   RADIO_0_IRQHandler          /* 138 */
    .long   RADIO_1_IRQHandler          /* 139 */
    .rept   (192-140)                   /* 140-191 */
    .long   0
    .endr
    .long   SPU20_IRQHandler            /* 192 */
    .rept   (198-193)                   /* 193-197 */
    .long   0
    .endr
    .long   SERIAL20_IRQHandler         /* 198 */
    .long   SERIAL21_IRQHandler         /* 199 */
    .long   SERIAL22_IRQHandler         /* 200 */
    .long   EGU20_IRQHandler            /* 201 */
    .long   TIMER20_IRQHandler          /* 202 */
    .long   TIMER21_IRQHandler          /* 203 */
    .long   TIMER22_IRQHandler          /* 204 */
    .long   TIMER23_IRQHandler          /* 205 */
    .long   TIMER24_IRQHandler          /* 206 */
    .long   0                           /* 207 reserved */
    .long   PDM20_IRQHandler            /* 208 */
    .long   PDM21_IRQHandler            /* 209 */
    .long   PWM20_IRQHandler            /* 210 */
    .long   PWM21_IRQHandler            /* 211 */
    .long   PWM22_IRQHandler            /* 212 */
    .long   SAADC_IRQHandler            /* 213 */
    .long   NFCT_IRQHandler             /* 214 */
    .long   TEMP_IRQHandler             /* 215 */
    .long   0                           /* 216 reserved */
    .long   0                           /* 217 reserved */
    .long   GPIOTE20_0_IRQHandler       /* 218 */
    .long   GPIOTE20_1_IRQHandler       /* 219 */
    .long   TAMPC_IRQHandler            /* 220 */
    .long   I2S20_IRQHandler            /* 221 */
    .long   0                           /* 222 reserved */
    .long   0                           /* 223 reserved */
    .long   QDEC20_IRQHandler           /* 224 */
    .long   QDEC21_IRQHandler           /* 225 */
    .long   GRTC_0_IRQHandler           /* 226 */
    .long   GRTC_1_IRQHandler           /* 227 */
    .long   GRTC_2_IRQHandler           /* 228 */
    .long   GRTC_3_IRQHandler           /* 229 */
    .rept   (256-230)                   /* 230-255 */
    .long   0
    .endr
    .long   SPU30_IRQHandler            /* 256 */
    .long   0                           /* 257 reserved */
    .long   0                           /* 258 reserved */
    .long   0                           /* 259 reserved */
    .long   SERIAL30_IRQHandler         /* 260 */
    .long   CLOCK_POWER_IRQHandler      /* 261 */
    .long   COMP_LPCOMP_IRQHandler      /* 262 */
    .long   0                           /* 263 reserved */
    .long   WDT30_IRQHandler            /* 264 */
    .long   WDT31_IRQHandler            /* 265 */
    .long   0                           /* 266 reserved */
    .long   0                           /* 267 reserved */
    .long   GPIOTE30_0_IRQHandler       /* 268 */
    .long   GPIOTE30_1_IRQHandler       /* 269 */

    .size __isr_vector, . - __isr_vector

/* Reset Handler */


    .text
    .thumb
    .thumb_func
    .align 1
    .globl Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:

#ifndef __STARTUP_SKIP_ETEXT
    ldr r1, =__etext
    ldr r2, =__data_start__
    ldr r3, =__bss_start__

    subs r3, r3, r2
    ble .L_loop1_done

.L_loop1:
    subs r3, r3, #4
    ldr r0, [r1,r3]
    str r0, [r2,r3]
    bgt .L_loop1

.L_loop1_done:
#endif

#ifdef __STARTUP_CLEAR_BSS
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__
    movs r0, #0

    subs r2, r2, r1
    ble .L_loop3_done

.L_loop3:
    subs r2, r2, #4
    str r0, [r1, r2]
    bgt .L_loop3

.L_loop3_done:
#endif /* __STARTUP_CLEAR_BSS */

    bl SystemInit

#ifndef __START
#define __START _start
#endif
    bl __START

    .pool
    .size   Reset_Handler,.-Reset_Handler

    .section ".text"

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    b       .
    .size   NMI_Handler, . - NMI_Handler

    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    b       .
    .size   HardFault_Handler, . - HardFault_Handler

    .weak   MemoryManagement_Handler
    .type   MemoryManagement_Handler, %function
MemoryManagement_Handler:
    b       .
    .size   MemoryManagement_Handler, . - MemoryManagement_Handler

    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    b       .
    .size   BusFault_Handler, . - BusFault_Handler

    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    b       .
    .size   UsageFault_Handler, . - UsageFault_Handler

    .weak   SecureFault_Handler
    .type   SecureFault_Handler, %function
SecureFault_Handler:
    b       .
    .size   SecureFault_Handler, . - SecureFault_Handler

    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    b       .
    .size   SVC_Handler, . - SVC_Handler

    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    b       .
    .size   DebugMon_Handler, . - DebugMon_Handler

    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    b       .
    .size   PendSV_Handler, . - PendSV_Handler

    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    b       .
    .size   SysTick_Handler, . - SysTick_Handler

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ  SWI00_IRQHandler
    IRQ  SWI01_IRQHandler
    IRQ  SWI02_IRQHandler
    IRQ  SWI03_IRQHandler
    IRQ  SPU00_IRQHandler
    IRQ  MPC00_IRQHandler
    IRQ  AAR00_CCM00_IRQHandler
    IRQ  ECB00_IRQHandler
    IRQ  CRACEN_IRQHandler
    IRQ  SERIAL00_IRQHandler
    IRQ  RRAMC_IRQHandler
    IRQ  VPR00_IRQHandler
    IRQ  CTRLAP_IRQHandler
    IRQ  CM33SS_IRQHandler
    IRQ  TIMER00_IRQHandler
    IRQ  SPU10_IRQHandler
    IRQ  TIMER10_IRQHandler
    IRQ  EGU10_IRQHandler
    IRQ  RADIO_0_IRQHandler
    IRQ  RADIO_1_IRQHandler
    IRQ  SPU20_IRQHandler
    IRQ  SERIAL20_IRQHandler
    IRQ  SERIAL21_IRQHandler
    IRQ  SERIAL22_IRQHandler
    IRQ  EGU20_IRQHandler
    IRQ  TIMER20_IRQHandler
    IRQ  TIMER21_IRQHandler
    IRQ  TIMER22_IRQHandler
    IRQ  TIMER23_IRQHandler
    IRQ  TIMER24_IRQHandler
    IRQ  PDM20_IRQHandler
    IRQ  PDM21_IRQHandler
    IRQ  PWM20_IRQHandler
    IRQ  PWM21_IRQHandler
    IRQ  PWM22_IRQHandler
    IRQ  SAADC_IRQHandler
    IRQ  NFCT_IRQHandler
    IRQ  TEMP_IRQHandler
    IRQ  GPIOTE20_0_IRQHandler
    IRQ  GPIOTE20_1_IRQHandler
    IRQ  TAMPC_IRQHandler
    IRQ  I2S20_IRQHandler
    IRQ  QDEC20_IRQHandler
    IRQ  QDEC21_IRQHandler
    IRQ  GRTC_0_IRQHandler
    IRQ  GRTC_1_IRQHandler
    IRQ  GRTC_2_IRQHandler
    IRQ  GRTC_3_IRQHandler
    IRQ  SPU30_IRQHandler
    IRQ  SERIAL30_IRQHandler
    IRQ  CLOCK_POWER_IRQHandler
    IRQ  COMP_LPCOMP_IRQHandler
    IRQ  WDT30_IRQHandler
    IRQ  WDT31_IRQHandler
    IRQ  GPIOTE30_0_IRQHandler
    IRQ  GPIOTE30_1_IRQHandler

