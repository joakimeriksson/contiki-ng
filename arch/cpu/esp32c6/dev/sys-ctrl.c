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
#include <sys/stat.h>
#include <errno.h>
#include <reent.h>

/* ROM function for character output */
extern int ets_putc(int c);

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
/* Newlib syscall stubs for ESP-IDF toolchain */
/*---------------------------------------------------------------------------*/
_ssize_t
_read_r(struct _reent *r, int fd, void *buf, size_t len)
{
  (void)r;
  (void)fd;
  (void)buf;
  (void)len;
  return -1;
}
/*---------------------------------------------------------------------------*/
_ssize_t
_write_r(struct _reent *r, int fd, const void *buf, size_t len)
{
  (void)r;
  (void)fd;
  const char *p = (const char *)buf;
  for(size_t i = 0; i < len; i++) {
    ets_putc(p[i]);
  }
  return len;
}
/*---------------------------------------------------------------------------*/
_off_t
_lseek_r(struct _reent *r, int fd, _off_t offset, int whence)
{
  (void)r;
  (void)fd;
  (void)offset;
  (void)whence;
  return -1;
}
/*---------------------------------------------------------------------------*/
int
_close_r(struct _reent *r, int fd)
{
  (void)r;
  (void)fd;
  return -1;
}
/*---------------------------------------------------------------------------*/
int
_fstat_r(struct _reent *r, int fd, struct stat *st)
{
  (void)r;
  (void)fd;
  st->st_mode = S_IFCHR;
  return 0;
}
/*---------------------------------------------------------------------------*/
int
_isatty_r(struct _reent *r, int fd)
{
  (void)r;
  (void)fd;
  return 1;
}
/*---------------------------------------------------------------------------*/
void *
_sbrk_r(struct _reent *r, ptrdiff_t incr)
{
  (void)r;
  (void)incr;
  return (void *)-1;
}
/*---------------------------------------------------------------------------*/
int
_getpid_r(struct _reent *r)
{
  (void)r;
  return 1;
}
/*---------------------------------------------------------------------------*/
int
_kill_r(struct _reent *r, int pid, int sig)
{
  (void)r;
  (void)pid;
  (void)sig;
  return -1;
}
/*---------------------------------------------------------------------------*/
struct _reent *
__getreent(void)
{
  return _GLOBAL_REENT;
}
/*---------------------------------------------------------------------------*/
int
pthread_setcancelstate(int state, int *oldstate)
{
  (void)state;
  (void)oldstate;
  return 0;
}
/*---------------------------------------------------------------------------*/
