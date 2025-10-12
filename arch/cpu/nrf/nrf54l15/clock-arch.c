/*
 * Minimal GRTC-backed clock implementation for nRF54L15.
 *
 * NOTE: This ignores advanced low-power tuning and only provides the
 *       tick functionality required for basic Contiki timers.
 */

#include "contiki.h"

#include "nrfx_clock.h"
#include "soc/nrfx_coredep.h"  /* Delay functions in v3.x */

/*
 * nrfx HAL conditionally redefines GRTC_IRQn based on NRF_APPLICATION:
 * - For nRF54L15 (LUMOS) application core: GRTC_IRQn = GRTC_2_IRQn
 * - For nRF54L15 FLPR core: GRTC_IRQn = GRTC_0_IRQn
 * This is defined in hal/nrf_grtc.h, which is included by nrfx_grtc.h
 */
#include "nrfx_grtc.h"
#include "sys/etimer.h"

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

#define GRTC_IRQ_PRIORITY      6
#define GRTC_TICK_FREQUENCY_HZ 1000000UL

#if CLOCK_SIZE != 4
#error CLOCK_CONF_SIZE must be 4 (32 bit)
#endif

static volatile clock_time_t ticks;
static uint32_t tick_interval_us;
static nrfx_grtc_channel_t tick_channel;
static bool is_initialized;
volatile nrfx_err_t last_schedule_err;
volatile uint8_t tick_channel_id;

static void schedule_next_tick(void);

static void
clock_update(void)
{
  ticks++;
  if(etimer_pending() && !CLOCK_LT(ticks, etimer_next_expiration_time())) {
    etimer_request_poll();
  }
}

static void
grtc_tick_handler(int32_t id, uint64_t cc_value, void *context)
{
  (void)id;
  (void)cc_value;
  (void)context;

  clock_update();
  schedule_next_tick();
}

static void wait_for_lfclk_ready(void);
static void wait_for_syscounter_ready(void);

static void
schedule_next_tick(void)
{
  nrfx_err_t err = nrfx_grtc_syscounter_cc_relative_set(&tick_channel,
                                                        tick_interval_us,
                                                        true,
                                                        NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
  last_schedule_err = err;
  if(err == NRFX_ERROR_INTERNAL) {
    wait_for_syscounter_ready();
    err = nrfx_grtc_syscounter_cc_relative_set(&tick_channel,
                                               tick_interval_us,
                                               true,
                                               NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
    last_schedule_err = err;
  }

  if(err != NRFX_SUCCESS) {
    return;
  }
}

static void
lfclk_init(void)
{
  nrfx_err_t err = nrfx_clock_init(NULL);
  if(err != NRFX_SUCCESS && err != NRFX_ERROR_ALREADY) {
    return;
  }

  nrfx_clock_enable();
  nrfx_clock_lfclk_start();
  wait_for_lfclk_ready();
}

void
clock_init(void)
{
  if(is_initialized) {
    return;
  }

  ticks = 0;
  tick_interval_us = GRTC_TICK_FREQUENCY_HZ / CLOCK_SECOND;
  if(tick_interval_us == 0) {
    tick_interval_us = 1;
  }

  lfclk_init();

  nrfx_err_t err = nrfx_grtc_init(GRTC_IRQ_PRIORITY);
  if(err != NRFX_SUCCESS && err != NRFX_ERROR_ALREADY) {
    return;
  }

  /* Start the GRTC syscounter with busy wait (Zephyr approach)
   * This allocates a main CC channel automatically */
  uint8_t main_cc_channel = 0;
  err = nrfx_grtc_syscounter_start(true, &main_cc_channel);
  if(err != NRFX_SUCCESS && err != NRFX_ERROR_ALREADY) {
    return;
  }
  wait_for_syscounter_ready();

  /* Now allocate a separate GRTC channel for our ticking */
  uint8_t channel = 0;
  err = nrfx_grtc_channel_alloc(&channel);
  if(err != NRFX_SUCCESS) {
    return;
  }

  nrfx_grtc_syscounter_cc_int_enable(channel);

  /* Setup channel structure */
  tick_channel.channel = channel;
  tick_channel.handler = grtc_tick_handler;
  tick_channel.p_context = NULL;
  tick_channel_id = channel;

  schedule_next_tick();

  is_initialized = true;
}

clock_time_t
clock_time(void)
{
  return ticks;
}

unsigned long
clock_seconds(void)
{
  return (unsigned long)(ticks / CLOCK_SECOND);
}

void
clock_wait(clock_time_t i)
{
  clock_time_t start = clock_time();
  while(clock_time() - start < i) {
    __WFE();
  }
}

void
clock_delay_usec(uint16_t dt)
{
  nrfx_coredep_delay_us(dt);
}

void
clock_delay(unsigned int i)
{
  clock_delay_usec(i);
}

static void
wait_for_lfclk_ready(void)
{
  while(!nrfx_clock_lfclk_is_running()) {
    __NOP();
  }
}

static void
wait_for_syscounter_ready(void)
{
  while(!nrfx_grtc_ready_check()) {
    __NOP();
  }
}
