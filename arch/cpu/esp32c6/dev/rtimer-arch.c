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
 *      ESP32-C6 rtimer (real-time timer) implementation
 */

#include "contiki.h"
#include "sys/rtimer.h"
#include "rtimer-arch.h"

#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* ESP32-C6 uses general purpose timer for rtimer
 * Timer runs at APB_CLK (typically 80 MHz) but can be divided
 * We configure it for 1 MHz (1us resolution)
 */
#define TIMG0_BASE              0x6001F000
#define TIMG_T0CONFIG_REG       (TIMG0_BASE + 0x0000)
#define TIMG_T0LO_REG           (TIMG0_BASE + 0x0004)
#define TIMG_T0HI_REG           (TIMG0_BASE + 0x0008)
#define TIMG_T0UPDATE_REG       (TIMG0_BASE + 0x000C)
#define TIMG_T0ALARMLO_REG      (TIMG0_BASE + 0x0010)
#define TIMG_T0ALARMHI_REG      (TIMG0_BASE + 0x0014)
#define TIMG_T0LOADLO_REG       (TIMG0_BASE + 0x0018)
#define TIMG_T0LOADHI_REG       (TIMG0_BASE + 0x001C)

/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  /* Initialize general purpose timer 0 for rtimer
   * Configure for 1 MHz (1us per tick)
   *
   * In a complete implementation:
   * 1. Set timer divider to get 1 MHz from APB_CLK
   * 2. Enable timer in count-up mode
   * 3. Enable alarm interrupts
   * 4. Start the timer
   */
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now(void)
{
  /* Read current timer value
   * On ESP32-C6, need to trigger update and read 64-bit value
   *
   * In real implementation:
   * *((volatile uint32_t *)TIMG_T0UPDATE_REG) = 1;
   * uint32_t lo = *((volatile uint32_t *)TIMG_T0LO_REG);
   * return (rtimer_clock_t)lo;
   */

  /* Simplified - would use actual timer in real implementation */
  static rtimer_clock_t timer_value = 0;
  timer_value++;
  return timer_value;
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  /* Set alarm for time t
   *
   * In real implementation:
   * *((volatile uint32_t *)TIMG_T0ALARMLO_REG) = t;
   * *((volatile uint32_t *)TIMG_T0ALARMHI_REG) = 0;
   * Enable alarm interrupt
   */
}
/*---------------------------------------------------------------------------*/
/**
 * \brief RTimer interrupt handler
 *
 * Called when the timer alarm triggers
 */
void
rtimer_arch_isr(void)
{
  /* Clear interrupt flag and call rtimer callback */
  rtimer_run_next();
}
/*---------------------------------------------------------------------------*/
