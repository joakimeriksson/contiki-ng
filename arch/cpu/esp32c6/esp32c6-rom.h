/*
 * Copyright (c) 2025, Contiki-NG ESP32-C6 Port
 * All rights reserved.
 */

/**
 * \file
 *      ESP32-C6 ROM function declarations
 */

#ifndef ESP32C6_ROM_H_
#define ESP32C6_ROM_H_

#include <stdint.h>
#include <stdarg.h>

/* ROM UART functions */
void ets_printf(const char *fmt, ...);
int ets_putc(int c);
void ets_install_putc1(void (*p)(char c));

/* ROM utility functions */
void ets_delay_us(uint32_t us);
uint32_t ets_get_cpu_frequency(void);

/* ROM cache functions */
void Cache_Enable_ICache(uint32_t autoload);

/* Simple wrapper for printf */
#define rom_printf ets_printf

#endif /* ESP32C6_ROM_H_ */
