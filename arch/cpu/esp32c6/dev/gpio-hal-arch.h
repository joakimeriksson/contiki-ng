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

#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* GPIO configuration for Contiki-NG GPIO HAL */

/* ESP32-C6 has only one "port" (port 0) with 30 GPIOs */
#define GPIO_HAL_ARCH_PORT_COUNT 1

/*---------------------------------------------------------------------------*/
/* Function declarations for port-based GPIO HAL */

void gpio_hal_arch_init(void);

void gpio_hal_arch_port_pin_set_output(uint8_t port, uint8_t pin);
void gpio_hal_arch_port_pin_set_input(uint8_t port, uint8_t pin);
void gpio_hal_arch_port_set_pins(uint8_t port, uint32_t pins);
void gpio_hal_arch_port_clear_pins(uint8_t port, uint32_t pins);
uint32_t gpio_hal_arch_port_read_pins(uint8_t port, uint32_t pins);
void gpio_hal_arch_port_toggle_pins(uint8_t port, uint32_t pins);
void gpio_hal_arch_port_write_pins(uint8_t port, uint32_t pins, uint32_t value);

void gpio_hal_arch_port_pin_cfg_set(uint8_t port, uint8_t pin, uint32_t cfg);
uint32_t gpio_hal_arch_port_pin_cfg_get(uint8_t port, uint8_t pin);

void gpio_hal_arch_port_interrupt_enable(uint8_t port, uint8_t pin);
void gpio_hal_arch_port_interrupt_disable(uint8_t port, uint8_t pin);

/*---------------------------------------------------------------------------*/
#endif /* GPIO_HAL_ARCH_H_ */
