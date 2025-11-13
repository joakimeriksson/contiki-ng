/*
 * Copyright (c) 2025, Contiki-NG ESP32-C6 Port
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *      ESP32-C6 clock/timer driver implementation
 */

#include "contiki.h"
#include "sys/clock.h"
#include "sys/etimer.h"
#include "clock-arch.h"

#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* ESP32-C6 System Timer registers (52-bit system timer) */
#define SYSTIMER_BASE           0x60023000
#define SYSTIMER_UNIT0_VALUE_LO (SYSTIMER_BASE + 0x0004)
#define SYSTIMER_UNIT0_VALUE_HI (SYSTIMER_BASE + 0x0008)
#define SYSTIMER_CONF_REG       (SYSTIMER_BASE + 0x0000)

/*---------------------------------------------------------------------------*/
static volatile unsigned long clock_ticks = 0;

/*---------------------------------------------------------------------------*/
void
clock_arch_init(void)
{
  /* Initialize system timer
   * On ESP32-C6, the system timer runs at 16 MHz (XTAL frequency)
   * We need to set up a periodic interrupt at 1 kHz (1ms interval)
   *
   * In a complete implementation, this would:
   * 1. Configure the system timer
   * 2. Set up a comparator for periodic interrupts
   * 3. Enable the timer interrupt
   */

  clock_ticks = 0;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_arch_time(void)
{
  return clock_ticks;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief System timer interrupt handler
 *
 * This should be called every 1ms by the hardware timer interrupt
 */
void
clock_arch_isr(void)
{
  clock_ticks++;

  /* Notify etimer that time has advanced */
  if(etimer_pending()) {
    etimer_request_poll();
  }
}
/*---------------------------------------------------------------------------*/
/* Delay function using system timer */
void
clock_arch_delay_us(uint32_t us)
{
  /* Read system timer and busy-wait
   * System timer runs at 16 MHz on ESP32-C6
   * Each tick is 62.5 ns, so for 1us we need 16 ticks
   */

  /* In real implementation:
   * uint64_t start = read_systimer();
   * uint64_t end = start + (us * 16);
   * while (read_systimer() < end) {}
   */

  /* Simplified busy-wait */
  volatile uint32_t i;
  for(i = 0; i < us * 160; i++) {
    /* Each iteration is roughly 1/160 of a microsecond at 160MHz */
  }
}
/*---------------------------------------------------------------------------*/
