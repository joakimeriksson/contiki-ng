/*
 * Copyright (c) 2025, Contiki-NG.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Low Power Timer + SL Timer + Timer Coordinator for nrf_802154 on nRF54L15.
 *
 * Replaces the Zephyr-dependent sl/sl_opensource/src/nrf_802154_sl_timer.c.
 * Uses GRTC as the 1 MHz free-running counter (1 lptick = 1 us).
 *
 * GRTC CC channels are dynamically allocated via the nrfx GRTC driver so that
 * the nrfx IRQ dispatcher correctly routes compare events to our callbacks.
 */

#include "nrf_802154_sl_timer.h"
#include "platform/nrf_802154_platform_sl_lptimer.h"
#include "timer/nrf_802154_timer_coord.h"
#include "nrfx_grtc.h"

#include "nrf.h"

#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* Non-blocking SYSCOUNTER read from the active GRTC interrupt group.
 *
 * The standard nrfx_grtc_syscounter_get() reads from SYSCOUNTER[0] (the
 * "powered" domain) which requires cross-domain synchronization.  The HAL
 * busy-waits on the BUSY bit; if the GRTC is transitioning between power
 * states the wait can be unbounded — a deadlock when called with interrupts
 * disabled.
 *
 * On nRF54L15 application core the secure build uses GRTC group 2 and the
 * non-secure build uses group 1. The active group is directly readable while
 * the local CPU is running. No BUSY polling is needed here.
 */
static inline uint64_t
grtc_syscounter_read_active(void)
{
  uint32_t hi, lo;
  do {
    hi = NRF_GRTC->SYSCOUNTER[GRTC_IRQ_GROUP].SYSCOUNTERH;
    lo = NRF_GRTC->SYSCOUNTER[GRTC_IRQ_GROUP].SYSCOUNTERL;
  } while(hi != NRF_GRTC->SYSCOUNTER[GRTC_IRQ_GROUP].SYSCOUNTERH);
  return ((uint64_t)(hi & 0x001FFFFFUL) << 32) | lo;
}
/*---------------------------------------------------------------------------*/

static volatile bool alarm_pending;
static volatile bool sync_pending;

/* Dynamically allocated GRTC channels */
static nrfx_grtc_channel_t alarm_channel;
static nrfx_grtc_channel_t sync_channel;
static uint8_t alarm_ch_id;
static uint8_t sync_ch_id;
static bool channels_allocated;

/* Forward declaration */
static nrf_802154_sl_timer_t *active_timer;

/*---------------------------------------------------------------------------*/
/* nrfx GRTC callbacks */
/*---------------------------------------------------------------------------*/
static void
alarm_grtc_callback(int32_t id, uint64_t cc_value, void *context)
{
  (void)id;
  (void)cc_value;
  (void)context;
  alarm_pending = false;
  if(active_timer != NULL &&
     (active_timer->action_type & NRF_802154_SL_TIMER_ACTION_TYPE_CALLBACK) &&
     active_timer->action.callback.callback != NULL) {
    nrf_802154_sl_timer_t *t = active_timer;
    active_timer = NULL;
    t->action.callback.callback(t);
  }
}

static void
sync_grtc_callback(int32_t id, uint64_t cc_value, void *context)
{
  (void)id;
  (void)cc_value;
  (void)context;
  sync_pending = false;
}

/*---------------------------------------------------------------------------*/
/* LP Timer Platform API */
/*---------------------------------------------------------------------------*/
void
nrf_802154_platform_sl_lp_timer_init(void)
{
  alarm_pending = false;
  sync_pending = false;

  if(channels_allocated) {
    return;
  }

  /* Allocate alarm channel */
  if(nrfx_grtc_channel_alloc(&alarm_ch_id) != NRFX_SUCCESS) {
    return;
  }
  alarm_channel.channel = alarm_ch_id;
  alarm_channel.handler = alarm_grtc_callback;
  alarm_channel.p_context = NULL;
  nrfx_grtc_syscounter_cc_int_enable(alarm_ch_id);

  /* Allocate sync channel */
  if(nrfx_grtc_channel_alloc(&sync_ch_id) != NRFX_SUCCESS) {
    return;
  }
  sync_channel.channel = sync_ch_id;
  sync_channel.handler = sync_grtc_callback;
  sync_channel.p_context = NULL;
  nrfx_grtc_syscounter_cc_int_enable(sync_ch_id);

  channels_allocated = true;
}

void
nrf_802154_platform_sl_lp_timer_deinit(void)
{
  alarm_pending = false;
}

uint64_t
nrf_802154_platform_sl_lptimer_current_lpticks_get(void)
{
  return grtc_syscounter_read_active();
}

uint64_t
nrf_802154_platform_sl_lptimer_us_to_lpticks_convert(uint64_t us, bool round_up)
{
  (void)round_up;
  /* 1 lptick = 1 us for GRTC at 1 MHz */
  return us;
}

uint64_t
nrf_802154_platform_sl_lptimer_lpticks_to_us_convert(uint64_t lpticks)
{
  return lpticks;
}

void
nrf_802154_platform_sl_lptimer_schedule_at(uint64_t fire_lpticks)
{
  if(!channels_allocated) {
    return;
  }
  alarm_pending = true;
  nrfx_grtc_syscounter_cc_absolute_set(&alarm_channel, fire_lpticks, true);
}

void
nrf_802154_platform_sl_lptimer_disable(void)
{
  alarm_pending = false;
  if(!channels_allocated) {
    return;
  }
  nrfx_grtc_syscounter_cc_disable(alarm_ch_id);
}

void
nrf_802154_platform_sl_lptimer_critical_section_enter(void)
{
  if(!channels_allocated) {
    return;
  }
  nrfx_grtc_syscounter_cc_int_disable(alarm_ch_id);
}

void
nrf_802154_platform_sl_lptimer_critical_section_exit(void)
{
  if(!channels_allocated) {
    return;
  }
  if(alarm_pending) {
    nrfx_grtc_syscounter_cc_int_enable(alarm_ch_id);
  }
}

nrf_802154_sl_lptimer_platform_result_t
nrf_802154_platform_sl_lptimer_hw_task_prepare(uint64_t fire_lpticks,
                                               uint32_t ppi_channel)
{
  (void)ppi_channel;
  if(!channels_allocated) {
    return NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
  }
  nrfx_grtc_syscounter_cc_absolute_set(&alarm_channel, fire_lpticks, true);
  return NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
}

nrf_802154_sl_lptimer_platform_result_t
nrf_802154_platform_sl_lptimer_hw_task_cleanup(void)
{
  if(!channels_allocated) {
    return NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
  }
  nrfx_grtc_syscounter_cc_disable(alarm_ch_id);
  return NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
}

nrf_802154_sl_lptimer_platform_result_t
nrf_802154_platform_sl_lptimer_hw_task_update_ppi(uint32_t ppi_channel)
{
  (void)ppi_channel;
  return NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
}

void
nrf_802154_platform_sl_lptimer_sync_schedule_now(void)
{
  if(!channels_allocated) {
    return;
  }
  uint64_t now = grtc_syscounter_read_active();
  sync_pending = true;
  nrfx_grtc_syscounter_cc_absolute_set(&sync_channel, now + 10, true);
}

void
nrf_802154_platform_sl_lptimer_sync_schedule_at(uint64_t fire_lpticks)
{
  if(!channels_allocated) {
    return;
  }
  sync_pending = true;
  nrfx_grtc_syscounter_cc_absolute_set(&sync_channel, fire_lpticks, true);
}

void
nrf_802154_platform_sl_lptimer_sync_abort(void)
{
  if(!channels_allocated) {
    return;
  }
  nrfx_grtc_syscounter_cc_disable(sync_ch_id);
  sync_pending = false;
}

uint32_t
nrf_802154_platform_sl_lptimer_sync_event_get(void)
{
  return (uint32_t)&NRF_GRTC->EVENTS_COMPARE[sync_ch_id];
}

uint64_t
nrf_802154_platform_sl_lptimer_sync_lpticks_get(void)
{
  return ((uint64_t)NRF_GRTC->CC[sync_ch_id].CCH << 32) |
         NRF_GRTC->CC[sync_ch_id].CCL;
}

uint32_t
nrf_802154_platform_sl_lptimer_granularity_get(void)
{
  return 1; /* 1 us granularity */
}

/*---------------------------------------------------------------------------*/
/* SL Timer API -- replaces Zephyr k_timer based implementation */
/*---------------------------------------------------------------------------*/

void
nrf_802154_sl_timer_module_init(void)
{
  active_timer = NULL;
  nrf_802154_platform_sl_lp_timer_init();
}

void
nrf_802154_sl_timer_module_uninit(void)
{
  active_timer = NULL;
}

uint64_t
nrf_802154_sl_timer_current_time_get(void)
{
  return grtc_syscounter_read_active();
}

void
nrf_802154_sl_timer_init(nrf_802154_sl_timer_t *p_timer)
{
  (void)p_timer;
}

void
nrf_802154_sl_timer_deinit(nrf_802154_sl_timer_t *p_timer)
{
  (void)p_timer;
}

nrf_802154_sl_timer_ret_t
nrf_802154_sl_timer_add(nrf_802154_sl_timer_t *p_timer)
{
  active_timer = p_timer;
  nrf_802154_platform_sl_lptimer_schedule_at(p_timer->trigger_time);
  return NRF_802154_SL_TIMER_RET_SUCCESS;
}

nrf_802154_sl_timer_ret_t
nrf_802154_sl_timer_remove(nrf_802154_sl_timer_t *p_timer)
{
  if(active_timer == p_timer) {
    nrf_802154_platform_sl_lptimer_disable();
    active_timer = NULL;
    return NRF_802154_SL_TIMER_RET_SUCCESS;
  }
  return NRF_802154_SL_TIMER_RET_INACTIVE;
}

nrf_802154_sl_timer_ret_t
nrf_802154_sl_timer_update_ppi(nrf_802154_sl_timer_t *p_timer, uint32_t ppi_chn)
{
  (void)p_timer;
  (void)ppi_chn;
  return NRF_802154_SL_TIMER_RET_SUCCESS;
}

/*---------------------------------------------------------------------------*/
/* Timer Coordinator -- SL-opensource runs without frame timestamping.
 * Report timestamps as unavailable until a real capture implementation exists.
 */
/*---------------------------------------------------------------------------*/
void
nrf_802154_timer_coord_init(void)
{
  /* No-op. */
}

void
nrf_802154_timer_coord_uninit(void)
{
  /* No-op. */
}

void
nrf_802154_timer_coord_start(void)
{
  /* No-op. */
}

void
nrf_802154_timer_coord_stop(void)
{
  /* No-op. */
}

void
nrf_802154_timer_coord_timestamp_prepare(const nrf_802154_sl_event_handle_t *p_event)
{
  (void)p_event;
}

bool
nrf_802154_timer_coord_timestamp_get(uint64_t *p_timestamp)
{
  (void)p_timestamp;
  return false;
}
