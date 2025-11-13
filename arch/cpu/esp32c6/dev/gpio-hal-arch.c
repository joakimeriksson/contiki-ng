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
 *      ESP32-C6 GPIO HAL implementation
 */

#include "contiki.h"
#include "gpio-hal-arch.h"

#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* ESP32-C6 GPIO registers */
#define GPIO_BASE               0x60091000
#define GPIO_OUT_REG            (GPIO_BASE + 0x0004)
#define GPIO_OUT_W1TS_REG       (GPIO_BASE + 0x0008)
#define GPIO_OUT_W1TC_REG       (GPIO_BASE + 0x000C)
#define GPIO_ENABLE_REG         (GPIO_BASE + 0x0020)
#define GPIO_ENABLE_W1TS_REG    (GPIO_BASE + 0x0024)
#define GPIO_ENABLE_W1TC_REG    (GPIO_BASE + 0x0028)
#define GPIO_IN_REG             (GPIO_BASE + 0x003C)

/* GPIO function registers - one per pin */
#define GPIO_PIN0_REG           (GPIO_BASE + 0x0074)
#define GPIO_FUNC_OUT_SEL_CFG_REG(n)  (GPIO_BASE + 0x0554 + (n) * 4)

/* IO MUX registers */
#define IO_MUX_BASE             0x60090000
#define IO_MUX_GPIO_REG(n)      (IO_MUX_BASE + 0x0004 + (n) * 4)

/*---------------------------------------------------------------------------*/
/* GPIO state tracking */
static uint32_t gpio_output_state = 0;

/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_init(void)
{
  /* Initialize GPIO subsystem
   * Reset all pins to safe default state
   */
  gpio_output_state = 0;
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_pin_cfg_set(gpio_hal_arch_pin_t pin, uint32_t cfg)
{
  if(pin >= GPIO_HAL_PIN_COUNT) {
    return;
  }

  /* Configure pin direction and properties
   * In real implementation:
   * - Set IO_MUX for the pin (function select, pull-up/down, drive strength)
   * - Set GPIO enable register for output pins
   * - Configure open-drain if needed
   */

  if(cfg & GPIO_HAL_PIN_CFG_OUTPUT) {
    /* Enable output */
    /* *((volatile uint32_t *)GPIO_ENABLE_W1TS_REG) = (1 << pin); */
    gpio_output_state |= (1 << pin);
  } else if(cfg & GPIO_HAL_PIN_CFG_INPUT) {
    /* Disable output (configure as input) */
    /* *((volatile uint32_t *)GPIO_ENABLE_W1TC_REG) = (1 << pin); */
    gpio_output_state &= ~(1 << pin);
  }

  /* Configure pull-up/down via IO_MUX
   * Set function to GPIO (usually function 1)
   */
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_pin_interrupt_enable(gpio_hal_arch_pin_t pin)
{
  if(pin >= GPIO_HAL_PIN_COUNT) {
    return;
  }

  /* Enable GPIO interrupt for this pin
   * In real implementation:
   * - Configure interrupt type (edge/level)
   * - Enable interrupt in GPIO interrupt enable register
   * - Enable in interrupt controller
   */
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_pin_interrupt_disable(gpio_hal_arch_pin_t pin)
{
  if(pin >= GPIO_HAL_PIN_COUNT) {
    return;
  }

  /* Disable GPIO interrupt for this pin */
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_write(gpio_hal_arch_pin_t pin, uint8_t value)
{
  if(pin >= GPIO_HAL_PIN_COUNT) {
    return;
  }

  /* Write to GPIO output register
   * In real implementation:
   */
  if(value) {
    /* Set pin high */
    /* *((volatile uint32_t *)GPIO_OUT_W1TS_REG) = (1 << pin); */
    gpio_output_state |= (1 << pin);
  } else {
    /* Set pin low */
    /* *((volatile uint32_t *)GPIO_OUT_W1TC_REG) = (1 << pin); */
    gpio_output_state &= ~(1 << pin);
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
gpio_hal_arch_read(gpio_hal_arch_pin_t pin)
{
  if(pin >= GPIO_HAL_PIN_COUNT) {
    return 0;
  }

  /* Read from GPIO input register
   * In real implementation:
   * uint32_t val = *((volatile uint32_t *)GPIO_IN_REG);
   * return (val & (1 << pin)) ? 1 : 0;
   */

  /* For now, read back output state */
  return (gpio_output_state & (1 << pin)) ? 1 : 0;
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_toggle(gpio_hal_arch_pin_t pin)
{
  if(pin >= GPIO_HAL_PIN_COUNT) {
    return;
  }

  /* Toggle pin state */
  if(gpio_output_state & (1 << pin)) {
    gpio_hal_arch_write(pin, 0);
  } else {
    gpio_hal_arch_write(pin, 1);
  }
}
/*---------------------------------------------------------------------------*/
