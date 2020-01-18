/*
 * Copyright (c) 2018, RISE SICS AB.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "contiki.h"
#include "dev/button-hal.h"
#include "bmp-280-sensor.h"
#include "rgbleds.h"
#include <stdbool.h>
#include "mic.h"
/*---------------------------------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO
/*---------------------------------------------------------------------------*/
PROCESS(test_process, "BMP280 tester");
AUTOSTART_PROCESSES(&test_process);
/*---------------------------------------------------------------------------*/
#define SAMPLE_COUNT 1024
static uint16_t buffer[SAMPLE_COUNT];

static uint16_t r, g, b;


PROCESS_THREAD(test_process, ev, data)
{
  static struct etimer periodic_timer;
  static int enable = 0;
  float var;
  float mean = 0;
  float level = 0;
  int32_t temp;
  uint32_t pressure;

  PROCESS_BEGIN();

  etimer_set(&periodic_timer, CLOCK_SECOND / 4);

  MIC_init(8000, buffer, SAMPLE_COUNT);

  while(true) {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_TIMER) {

      temp = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP);
      pressure = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
      if(!MIC_isBusy()) {
        mean = MIC_getMean() + 1.123;
        level = MIC_getSoundLevel(&var);
        MIC_start(SAMPLE_COUNT);

        /* Green */
        r = 0x0000;
        g = 0xa000;
        b = 0x0000;
        if(level > -70) {
          r = 0x0000 + (level + 70) * 0x280;
          g = 0xa000 - (level + 70) * 0x280;
        }

      }

      LOG_INFO("Time: %6lu  Temp: %2"PRId32".%03"PRId32"  Pressure: %4"PRIu32".%03"PRIu32"  Mean: %d.%03d SoundLevel: %d.%03d Var: %d.%03d\n, g = %d",
               (unsigned long)clock_time(), (temp / 1000), (temp % 1000),
               (pressure / 1000), (pressure % 1000),
               (int)(mean), ((int)(mean * 1000)) % 1000,
               (int)(level), ((int)(abs((int)(level * 1000)))) % 1000,
               (int)(var), ((int)(var * 1000)) % 1000, g
               );

      rgbleds_enable(enable++ & 0xf);

      /* rgbleds_setcolor((clock_time() >> 0) & 0xffff, */
      /*                  (clock_time() >> 2) & 0xffff, */
      /*                  (clock_time() >> 4) & 0xffff); */
      rgbleds_setcolor(r, g, b);
      etimer_restart(&periodic_timer);
    }

    if(ev == sensors_event) {
      struct sensors_sensor *sensor = data;
      LOG_INFO("%s triggered!\n", sensor->type);
    }

    if(ev == button_hal_press_event) {
      button_hal_button_t *btn = data;
      LOG_INFO("%s pressed\n", BUTTON_HAL_GET_DESCRIPTION(btn));
    } else if(ev == button_hal_release_event) {
      button_hal_button_t *btn = data;
      LOG_INFO("%s released\n", BUTTON_HAL_GET_DESCRIPTION(btn));
    } else if(ev == button_hal_periodic_event) {
      button_hal_button_t *btn = data;
      LOG_INFO("%s still pressed, %u seconds\n",
               BUTTON_HAL_GET_DESCRIPTION(btn), btn->press_duration_seconds);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
