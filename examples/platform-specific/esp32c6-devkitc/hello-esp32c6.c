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
 *      Hello World example for ESP32-C6-DevKitC
 *      Demonstrates basic platform features
 */

#include "contiki.h"
#include "dev/leds.h"
#include "sys/etimer.h"
#include "dev/radio.h"

#include <stdio.h>

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

/*---------------------------------------------------------------------------*/
PROCESS(hello_esp32c6_process, "Hello ESP32-C6");
AUTOSTART_PROCESSES(&hello_esp32c6_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_esp32c6_process, ev, data)
{
  static struct etimer timer;
  static int count = 0;
  static radio_value_t channel, txpower, pan_id;

  PROCESS_BEGIN();

  printf("\n");
  printf("=================================\n");
  printf("  ESP32-C6-DevKitC Demo\n");
  printf("=================================\n");
  printf("\n");

  /* Print platform information */
  LOG_INFO("Platform: " PLATFORM_NAME "\n");
  LOG_INFO("CPU: RISC-V @ %lu MHz\n", F_CPU / 1000000);
  LOG_INFO("RAM: %u KB\n", SRAM_SIZE / 1024);
  LOG_INFO("\n");

  /* Get radio information */
  if(NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel) == RADIO_RESULT_OK) {
    LOG_INFO("Radio channel: %d\n", channel);
  }
  if(NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &txpower) == RADIO_RESULT_OK) {
    LOG_INFO("Radio TX power: %d dBm\n", txpower);
  }
  if(NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &pan_id) == RADIO_RESULT_OK) {
    LOG_INFO("Radio PAN ID: 0x%04X\n", pan_id);
  }
  LOG_INFO("\n");

  /* Turn on the radio */
  NETSTACK_RADIO.on();
  LOG_INFO("Radio is ON\n");

  /* Set the periodic timer */
  etimer_set(&timer, CLOCK_SECOND * 2);

  printf("\n");
  printf("Starting LED blink demo...\n");
  printf("LEDs will toggle every 2 seconds\n");
  printf("\n");

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    /* Toggle LED */
    leds_toggle(LEDS_ALL);

    /* Print status */
    count++;
    LOG_INFO("Heartbeat %d - LED toggled\n", count);

    /* Reset the timer */
    etimer_reset(&timer);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
