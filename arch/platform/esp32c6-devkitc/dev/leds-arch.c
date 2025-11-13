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
 *      LED driver for ESP32-C6-DevKitC (WS2812 RGB LED on GPIO8)
 */

#include "contiki.h"
#include "dev/leds.h"
#include "dev/gpio-hal.h"

/*---------------------------------------------------------------------------*/
/* LED pin definitions */
#define LED_RGB_PIN  8  /* WS2812 RGB LED on GPIO8 */

/*---------------------------------------------------------------------------*/
/* LED state */
static uint8_t led_state = 0;

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  /* Initialize GPIO for LED control
   * For WS2812, we need precise timing control
   * In a complete implementation, this would use RMT or SPI peripheral
   */

  gpio_hal_arch_pin_cfg_set(LED_RGB_PIN, GPIO_HAL_PIN_CFG_OUTPUT);
  gpio_hal_arch_write(LED_RGB_PIN, 0);

  led_state = 0;
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return led_state;
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
  led_state = leds;

  /* WS2812 control would normally require precise timing
   * For this lightweight implementation, we'll use simple GPIO toggle
   * to simulate LED control
   *
   * In a complete implementation:
   * - Use RMT (Remote Control) peripheral for WS2812 protocol
   * - Send 24-bit color data (GRB format)
   * - Each bit is encoded with precise timing (0.4us/0.8us pulses)
   */

  if(leds == 0) {
    /* All LEDs off - send zeros to WS2812 */
    gpio_hal_arch_write(LED_RGB_PIN, 0);
  } else {
    /* At least one LED on - toggle to show activity */
    /* In real implementation, would send proper RGB data */
    static uint8_t toggle = 0;
    toggle = !toggle;
    gpio_hal_arch_write(LED_RGB_PIN, toggle);
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Send WS2812 data
 * \param r Red value (0-255)
 * \param g Green value (0-255)
 * \param b Blue value (0-255)
 *
 * WS2812 protocol:
 * - GRB color order
 * - 0 bit: 0.4us high, 0.85us low
 * - 1 bit: 0.8us high, 0.45us low
 * - Reset: >50us low
 */
void
ws2812_send_pixel(uint8_t r, uint8_t g, uint8_t b)
{
  /* This is a simplified placeholder
   * Real implementation would use RMT or bit-banging with precise timing
   *
   * Example using RMT (pseudocode):
   * - Configure RMT channel for WS2812 timing
   * - Pack GRB data into RMT buffer
   * - Transmit via RMT peripheral
   */

  /* For now, just use GPIO to show some activity */
  if(r || g || b) {
    gpio_hal_arch_write(LED_RGB_PIN, 1);
  } else {
    gpio_hal_arch_write(LED_RGB_PIN, 0);
  }
}
/*---------------------------------------------------------------------------*/
