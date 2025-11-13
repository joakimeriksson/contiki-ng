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
 *      ESP32-C6 GPIO HAL architecture-specific header
 */

#ifndef GPIO_HAL_ARCH_H_
#define GPIO_HAL_ARCH_H_

#include "contiki.h"
#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* GPIO pin definitions */
#define GPIO_HAL_PIN_COUNT 30  /* ESP32-C6 has 30 GPIO pins */

/*---------------------------------------------------------------------------*/
typedef uint8_t gpio_hal_arch_pin_t;

/*---------------------------------------------------------------------------*/
/* GPIO configuration */
#define gpio_hal_arch_interrupt_enable(p)   gpio_hal_arch_pin_interrupt_enable(p)
#define gpio_hal_arch_interrupt_disable(p)  gpio_hal_arch_pin_interrupt_disable(p)

#define gpio_hal_arch_pin_set_input(p)      gpio_hal_arch_pin_cfg_set(p, GPIO_HAL_PIN_CFG_INPUT)
#define gpio_hal_arch_pin_set_output(p)     gpio_hal_arch_pin_cfg_set(p, GPIO_HAL_PIN_CFG_OUTPUT)

#define gpio_hal_arch_set_pin(p)            gpio_hal_arch_write_pin(p, 1)
#define gpio_hal_arch_clear_pin(p)          gpio_hal_arch_write_pin(p, 0)
#define gpio_hal_arch_toggle_pin(p)         gpio_hal_arch_toggle(p)
#define gpio_hal_arch_read_pin(p)           gpio_hal_arch_read(p)
#define gpio_hal_arch_write_pin(p, v)       gpio_hal_arch_write(p, v)

/*---------------------------------------------------------------------------*/
/* Pin configuration flags */
#define GPIO_HAL_PIN_CFG_INPUT          0x01
#define GPIO_HAL_PIN_CFG_OUTPUT         0x02
#define GPIO_HAL_PIN_CFG_PULL_UP        0x04
#define GPIO_HAL_PIN_CFG_PULL_DOWN      0x08
#define GPIO_HAL_PIN_CFG_OPEN_DRAIN     0x10

/*---------------------------------------------------------------------------*/
void gpio_hal_arch_init(void);
void gpio_hal_arch_pin_cfg_set(gpio_hal_arch_pin_t pin, uint32_t cfg);
void gpio_hal_arch_pin_interrupt_enable(gpio_hal_arch_pin_t pin);
void gpio_hal_arch_pin_interrupt_disable(gpio_hal_arch_pin_t pin);
void gpio_hal_arch_write(gpio_hal_arch_pin_t pin, uint8_t value);
uint8_t gpio_hal_arch_read(gpio_hal_arch_pin_t pin);
void gpio_hal_arch_toggle(gpio_hal_arch_pin_t pin);

/*---------------------------------------------------------------------------*/
#endif /* GPIO_HAL_ARCH_H_ */
