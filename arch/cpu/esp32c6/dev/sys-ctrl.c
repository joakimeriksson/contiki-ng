/*
 * Copyright (c) 2025, Contiki-NG ESP32-C6 Port
 * All rights reserved.
 */

/**
 * \file
 *      ESP32-C6 system control stubs (clock, watchdog, etc.)
 */

#include "contiki.h"
#include "sys/clock.h"
#include "dev/watchdog.h"
#include <stdio.h>

/*---------------------------------------------------------------------------*/
static volatile clock_time_t current_clock = 0;
static volatile unsigned long current_seconds = 0;

/*---------------------------------------------------------------------------*/
void
clock_init(void)
{
  current_clock = 0;
  current_seconds = 0;
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  return current_clock;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  return current_seconds;
}
/*---------------------------------------------------------------------------*/
void
clock_wait(clock_time_t i)
{
  clock_time_t start = clock_time();
  while(clock_time() - start < i) {
    /* Busy wait */
  }
}
/*---------------------------------------------------------------------------*/
void
clock_delay_usec(uint16_t dt)
{
  /* Simple delay loop */
  volatile uint32_t i;
  for(i = 0; i < dt * 16; i++) {
    /* ~1us per 16 iterations at 160MHz */
  }
}
/*---------------------------------------------------------------------------*/
void
watchdog_init(void)
{
  /* Watchdog initialization stub */
}
/*---------------------------------------------------------------------------*/
void
watchdog_start(void)
{
  /* Watchdog start stub */
}
/*---------------------------------------------------------------------------*/
void
watchdog_periodic(void)
{
  /* Watchdog periodic refresh stub */
}
/*---------------------------------------------------------------------------*/
void
watchdog_stop(void)
{
  /* Watchdog stop stub */
}
/*---------------------------------------------------------------------------*/
void
watchdog_reboot(void)
{
  /* System reset */
  while(1) {
    /* Hang - in real implementation would trigger reset */
  }
}
/*---------------------------------------------------------------------------*/
/* Picolibc stdio support */
/*---------------------------------------------------------------------------*/
static int
uart_putc(char c, FILE *file)
{
  (void)file;
  /* In real implementation, write to UART */
  return c;
}

static int
uart_getc(FILE *file)
{
  (void)file;
  return EOF;
}

static FILE __stdio = FDEV_SETUP_STREAM(uart_putc, uart_getc, NULL, _FDEV_SETUP_RW);
FILE *const stdin = &__stdio;
FILE *const stdout = &__stdio;
FILE *const stderr = &__stdio;
/*---------------------------------------------------------------------------*/
