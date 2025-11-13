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
 *      Platform implementation for ESP32-C6-DevKitC
 */

#include "contiki.h"
#include "contiki-net.h"

#include "dev/gpio-hal.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "sys/platform.h"
#include "sys/rtimer.h"
#include "sys/node-id.h"

/* CPU drivers */
#include "uart-arch.h"
#include "clock-arch.h"
#include "rtimer-arch.h"
#include "esp32c6-radio.h"

#include <stdio.h>
#include <stdint.h>

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "ESP32C6"
#define LOG_LEVEL LOG_LEVEL_MAIN

/*---------------------------------------------------------------------------*/
/* Unique ID for this node (from MAC address or random) */
static uint16_t node_id = 0x1234;

/*---------------------------------------------------------------------------*/
void
platform_init_stage_one(void)
{
  /* Stage 1: Basic hardware initialization
   * - Initialize GPIO
   * - Initialize LEDs
   * - Set up basic clocks
   */

  LOG_INFO("Platform init stage 1\n");

  /* Initialize GPIO HAL */
  gpio_hal_arch_init();

  /* Initialize LEDs */
  leds_init();

  /* Blink LED to show we're alive */
  leds_on(LEDS_ALL);
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_two(void)
{
  /* Stage 2: Peripheral initialization
   * - Initialize UART
   * - Initialize timers
   * - Initialize radio
   * - Set up serial line
   */

  LOG_INFO("Platform init stage 2\n");

  /* Initialize UART for debug output */
  uart_arch_init(UART0_CONF_BAUD_RATE);

  /* Initialize clock and rtimer */
  clock_arch_init();
  rtimer_arch_init();

  /* Set up serial line for console input */
  serial_line_init();

  /* Set input handler for UART */
  uart_arch_set_input(serial_line_input_byte);

  /* Set node ID */
  node_id_burn(node_id);
  node_id_restore();

  LOG_INFO("Node ID: 0x%04x\n", node_id);

  /* Turn off LED after init */
  leds_off(LEDS_ALL);
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_three(void)
{
  /* Stage 3: Network stack and process initialization
   * - Start network processes
   * - Initialize radio
   */

  LOG_INFO("Platform init stage 3\n");

  /* Radio is initialized by the network stack */
  LOG_INFO("IEEE 802.15.4 radio driver ready\n");

  /* Print platform info */
  LOG_INFO("ESP32-C6-DevKitC platform initialized\n");
  LOG_INFO("CPU: RISC-V @ %lu MHz\n", F_CPU / 1000000);
  LOG_INFO("RAM: %u KB\n", SRAM_SIZE / 1024);
}
/*---------------------------------------------------------------------------*/
void
platform_idle(void)
{
  /* Low-power idle state
   * Put CPU into light sleep or wait for interrupt
   *
   * In real implementation:
   * - Use ESP32-C6 light sleep mode
   * - Wake on timer or GPIO interrupt
   */

  /* For now, just wait for interrupt */
  /* __WFI(); */ /* Wait for interrupt (RISC-V: WFI instruction) */
}
/*---------------------------------------------------------------------------*/
void
platform_main_loop(void)
{
  /* Main event loop is handled by Contiki-NG core */
  /* This function can be used for platform-specific main loop customization */
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Provide platform-specific logging
 */
int
dbg_putchar(int c)
{
  uart_arch_write_byte((uint8_t)c);
  return c;
}
/*---------------------------------------------------------------------------*/
