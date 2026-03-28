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
#include "helpers/nrfx_gppi.h"
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
static uint32_t critical_section_depth;

/* Dynamically allocated GRTC channels */
static nrfx_grtc_channel_t alarm_channel;
static nrfx_grtc_channel_t sync_channel;
static nrfx_grtc_channel_t hw_task_channel;
static uint8_t alarm_ch_id;
static uint8_t sync_ch_id;
static uint8_t hw_task_ch_id;
static bool channels_allocated;

/* Active software timers, ordered by trigger_time. */
static nrf_802154_sl_timer_t *alarm_head;
static uint64_t sync_fire_lpticks;

enum hw_task_state
{
  HW_TASK_STATE_IDLE,
  HW_TASK_STATE_SETTING_UP,
  HW_TASK_STATE_READY,
  HW_TASK_STATE_UPDATING,
  HW_TASK_STATE_CLEANING,
};

static enum hw_task_state hw_task_state;
static uint32_t hw_task_ppi_channel = NRF_802154_SL_HW_TASK_PPI_INVALID;
static uint64_t hw_task_fire_lpticks;

static inline uint32_t
irq_lock_local(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  __DMB();

  return primask;
}

static inline void
irq_unlock_local(uint32_t primask)
{
  __DMB();
  __set_PRIMASK(primask);
}

static inline nrf_802154_sl_timer_t *
timer_next_get(nrf_802154_sl_timer_t *timer)
{
  return (nrf_802154_sl_timer_t *)(uintptr_t)timer->priv.placeholder[0];
}

static inline void
timer_next_set(nrf_802154_sl_timer_t *timer, nrf_802154_sl_timer_t *next)
{
  timer->priv.placeholder[0] = (uint64_t)(uintptr_t)next;
}

static inline bool
timer_is_active(nrf_802154_sl_timer_t *timer)
{
  return timer->priv.placeholder[1] != 0U;
}

static inline void
timer_active_set(nrf_802154_sl_timer_t *timer, bool active)
{
  timer->priv.placeholder[1] = active ? 1U : 0U;
}

__attribute__((weak)) void
nrf_802154_sl_timestamper_synchronized(void)
{
}

static inline bool
hw_task_state_set_locked(enum hw_task_state expected, enum hw_task_state new_state)
{
  if(hw_task_state != expected) {
    return false;
  }

  hw_task_state = new_state;

  return true;
}

static inline uint32_t
hw_task_event_address_get(void)
{
  return nrfx_grtc_event_compare_address_get(hw_task_ch_id);
}

static inline bool
hw_task_triggered_check_locked(void)
{
  return NRF_GRTC->EVENTS_COMPARE[hw_task_ch_id] != 0U ||
         grtc_syscounter_read_active() >= hw_task_fire_lpticks;
}

static inline void
hw_task_ppi_bind_locked(uint32_t ppi_channel)
{
  if(ppi_channel == NRF_802154_SL_HW_TASK_PPI_INVALID) {
    return;
  }

  nrfx_gppi_event_endpoint_setup((uint8_t)ppi_channel, hw_task_event_address_get());
}

static inline void
hw_task_ppi_unbind_locked(uint32_t ppi_channel)
{
  if(ppi_channel == NRF_802154_SL_HW_TASK_PPI_INVALID) {
    return;
  }

  nrfx_gppi_event_endpoint_clear((uint8_t)ppi_channel, hw_task_event_address_get());
}

static void
alarm_reschedule_locked(void)
{
  if(!channels_allocated) {
    alarm_pending = false;
    return;
  }

  if(alarm_head == NULL) {
    alarm_pending = false;
    nrfx_grtc_syscounter_cc_disable(alarm_ch_id);
    return;
  }

  alarm_pending = true;
  nrfx_grtc_syscounter_cc_absolute_set(&alarm_channel, alarm_head->trigger_time, true);
}

static nrf_802154_sl_timer_ret_t
timer_remove_locked(nrf_802154_sl_timer_t *timer)
{
  nrf_802154_sl_timer_t *prev = NULL;
  nrf_802154_sl_timer_t *curr = alarm_head;

  while(curr != NULL) {
    if(curr == timer) {
      nrf_802154_sl_timer_t *next = timer_next_get(curr);

      if(prev == NULL) {
        alarm_head = next;
      } else {
        timer_next_set(prev, next);
      }

      timer_next_set(curr, NULL);
      timer_active_set(curr, false);
      alarm_reschedule_locked();
      return NRF_802154_SL_TIMER_RET_SUCCESS;
    }

    prev = curr;
    curr = timer_next_get(curr);
  }

  return NRF_802154_SL_TIMER_RET_INACTIVE;
}

/*---------------------------------------------------------------------------*/
/* nrfx GRTC callbacks */
/*---------------------------------------------------------------------------*/
static void
alarm_grtc_callback(int32_t id, uint64_t cc_value, void *context)
{
  (void)id;
  (void)cc_value;
  (void)context;

  while(true) {
    nrf_802154_sl_timer_t *timer;
    uint32_t primask = irq_lock_local();

    timer = alarm_head;

    if(timer == NULL) {
      alarm_pending = false;
      irq_unlock_local(primask);
      break;
    }

    if(timer->trigger_time > grtc_syscounter_read_active()) {
      alarm_reschedule_locked();
      irq_unlock_local(primask);
      break;
    }

    alarm_head = timer_next_get(timer);
    timer_next_set(timer, NULL);
    timer_active_set(timer, false);
    alarm_reschedule_locked();

    irq_unlock_local(primask);

    if((timer->action_type & NRF_802154_SL_TIMER_ACTION_TYPE_CALLBACK) &&
       timer->action.callback.callback != NULL) {
      timer->action.callback.callback(timer);
    }
  }
}

static void
sync_grtc_callback(int32_t id, uint64_t cc_value, void *context)
{
  (void)id;
  (void)cc_value;
  (void)context;

  sync_fire_lpticks = cc_value;
  sync_pending = false;
  nrf_802154_sl_timestamper_synchronized();
}

/*---------------------------------------------------------------------------*/
/* LP Timer Platform API */
/*---------------------------------------------------------------------------*/
void
nrf_802154_platform_sl_lp_timer_init(void)
{
  nrfx_err_t err;

  alarm_pending = false;
  sync_pending = false;
  critical_section_depth = 0;
  sync_fire_lpticks = 0;
  hw_task_state = HW_TASK_STATE_IDLE;
  hw_task_ppi_channel = NRF_802154_SL_HW_TASK_PPI_INVALID;
  hw_task_fire_lpticks = 0;

  if(channels_allocated) {
    return;
  }

  /* Allocate alarm channel */
  err = nrfx_grtc_channel_alloc(&alarm_ch_id);
  if(err != NRFX_SUCCESS) {
    return;
  }
  alarm_channel.channel = alarm_ch_id;
  alarm_channel.handler = alarm_grtc_callback;
  alarm_channel.p_context = NULL;
  nrfx_grtc_syscounter_cc_int_enable(alarm_ch_id);

  /* Allocate sync channel */
  err = nrfx_grtc_channel_alloc(&sync_ch_id);
  if(err != NRFX_SUCCESS) {
    nrfx_grtc_channel_free(alarm_ch_id);
    return;
  }
  sync_channel.channel = sync_ch_id;
  sync_channel.handler = sync_grtc_callback;
  sync_channel.p_context = NULL;
  nrfx_grtc_syscounter_cc_int_enable(sync_ch_id);

  /* Allocate a dedicated compare channel for hardware task scheduling. */
  err = nrfx_grtc_channel_alloc(&hw_task_ch_id);
  if(err != NRFX_SUCCESS) {
    nrfx_grtc_channel_free(sync_ch_id);
    nrfx_grtc_channel_free(alarm_ch_id);
    return;
  }
  hw_task_channel.channel = hw_task_ch_id;
  hw_task_channel.handler = NULL;
  hw_task_channel.p_context = NULL;

  channels_allocated = true;
}

void
nrf_802154_platform_sl_lp_timer_deinit(void)
{
  alarm_pending = false;
  sync_pending = false;
  critical_section_depth = 0;
  hw_task_state = HW_TASK_STATE_IDLE;
  hw_task_ppi_channel = NRF_802154_SL_HW_TASK_PPI_INVALID;
  hw_task_fire_lpticks = 0;
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
  uint32_t primask;

  if(!channels_allocated) {
    return;
  }

  primask = irq_lock_local();

  if(critical_section_depth++ == 0U) {
    nrfx_grtc_syscounter_cc_int_disable(alarm_ch_id);
    nrfx_grtc_syscounter_cc_int_disable(sync_ch_id);
  }

  irq_unlock_local(primask);
}

void
nrf_802154_platform_sl_lptimer_critical_section_exit(void)
{
  uint32_t primask;

  if(!channels_allocated) {
    return;
  }

  primask = irq_lock_local();

  if(critical_section_depth != 0U) {
    critical_section_depth--;
  }

  if(critical_section_depth == 0U) {
    if(sync_pending) {
      nrfx_grtc_syscounter_cc_int_enable(sync_ch_id);
    }

    if(alarm_pending) {
      nrfx_grtc_syscounter_cc_int_enable(alarm_ch_id);
    }
  }

  irq_unlock_local(primask);
}

nrf_802154_sl_lptimer_platform_result_t
nrf_802154_platform_sl_lptimer_hw_task_prepare(uint64_t fire_lpticks,
                                               uint32_t ppi_channel)
{
  if(!channels_allocated) {
    return NRF_802154_SL_LPTIMER_PLATFORM_NO_RESOURCES;
  }

  {
    uint32_t primask = irq_lock_local();

    if(!hw_task_state_set_locked(HW_TASK_STATE_IDLE, HW_TASK_STATE_SETTING_UP)) {
      irq_unlock_local(primask);
      return NRF_802154_SL_LPTIMER_PLATFORM_NO_RESOURCES;
    }

    hw_task_ppi_unbind_locked(hw_task_ppi_channel);
    hw_task_ppi_channel = NRF_802154_SL_HW_TASK_PPI_INVALID;
    hw_task_fire_lpticks = fire_lpticks;
    NRF_GRTC->EVENTS_COMPARE[hw_task_ch_id] = 0;

    irq_unlock_local(primask);
  }

  if(nrfx_grtc_syscounter_cc_absolute_set(&hw_task_channel, fire_lpticks, false) != NRFX_SUCCESS) {
    uint32_t primask = irq_lock_local();
    hw_task_state = HW_TASK_STATE_IDLE;
    irq_unlock_local(primask);
    return NRF_802154_SL_LPTIMER_PLATFORM_TOO_LATE;
  }

  {
    uint32_t primask = irq_lock_local();

    if((grtc_syscounter_read_active() + 2U) > fire_lpticks || hw_task_triggered_check_locked()) {
      irq_unlock_local(primask);
      nrfx_grtc_syscounter_cc_disable(hw_task_ch_id);
      primask = irq_lock_local();
      hw_task_state = HW_TASK_STATE_IDLE;
      irq_unlock_local(primask);
      return NRF_802154_SL_LPTIMER_PLATFORM_TOO_LATE;
    }

    hw_task_ppi_bind_locked(ppi_channel);
    hw_task_ppi_channel = ppi_channel;
    hw_task_state = HW_TASK_STATE_READY;

    irq_unlock_local(primask);
  }

  return NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
}

nrf_802154_sl_lptimer_platform_result_t
nrf_802154_platform_sl_lptimer_hw_task_cleanup(void)
{
  if(!channels_allocated) {
    return NRF_802154_SL_LPTIMER_PLATFORM_WRONG_STATE;
  }

  {
    uint32_t primask = irq_lock_local();

    if(!hw_task_state_set_locked(HW_TASK_STATE_READY, HW_TASK_STATE_CLEANING)) {
      irq_unlock_local(primask);
      return NRF_802154_SL_LPTIMER_PLATFORM_WRONG_STATE;
    }

    irq_unlock_local(primask);
  }

  nrfx_grtc_syscounter_cc_disable(hw_task_ch_id);

  {
    uint32_t primask = irq_lock_local();

    hw_task_ppi_unbind_locked(hw_task_ppi_channel);
    hw_task_ppi_channel = NRF_802154_SL_HW_TASK_PPI_INVALID;
    hw_task_state = HW_TASK_STATE_IDLE;

    irq_unlock_local(primask);
  }

  return NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
}

nrf_802154_sl_lptimer_platform_result_t
nrf_802154_platform_sl_lptimer_hw_task_update_ppi(uint32_t ppi_channel)
{
  bool too_late;

  if(!channels_allocated) {
    return NRF_802154_SL_LPTIMER_PLATFORM_WRONG_STATE;
  }

  {
    uint32_t primask = irq_lock_local();

    if(!hw_task_state_set_locked(HW_TASK_STATE_READY, HW_TASK_STATE_UPDATING)) {
      irq_unlock_local(primask);
      return NRF_802154_SL_LPTIMER_PLATFORM_WRONG_STATE;
    }

    hw_task_ppi_unbind_locked(hw_task_ppi_channel);
    hw_task_ppi_bind_locked(ppi_channel);
    hw_task_ppi_channel = ppi_channel;
    too_late = hw_task_triggered_check_locked();
    hw_task_state = HW_TASK_STATE_READY;

    irq_unlock_local(primask);
  }

  return too_late ? NRF_802154_SL_LPTIMER_PLATFORM_TOO_LATE :
                    NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS;
}

void
nrf_802154_platform_sl_lptimer_sync_schedule_now(void)
{
  if(!channels_allocated) {
    return;
  }
  uint64_t now = grtc_syscounter_read_active();
  sync_fire_lpticks = now + 3;
  sync_pending = true;
  nrfx_grtc_syscounter_cc_absolute_set(&sync_channel, sync_fire_lpticks, true);
}

void
nrf_802154_platform_sl_lptimer_sync_schedule_at(uint64_t fire_lpticks)
{
  if(!channels_allocated) {
    return;
  }
  sync_fire_lpticks = fire_lpticks;
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
  return sync_fire_lpticks;
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
  alarm_head = NULL;
  nrf_802154_platform_sl_lp_timer_init();
}

void
nrf_802154_sl_timer_module_uninit(void)
{
  alarm_head = NULL;
}

uint64_t
nrf_802154_sl_timer_current_time_get(void)
{
  return grtc_syscounter_read_active();
}

void
nrf_802154_sl_timer_init(nrf_802154_sl_timer_t *p_timer)
{
  timer_next_set(p_timer, NULL);
  timer_active_set(p_timer, false);
}

void
nrf_802154_sl_timer_deinit(nrf_802154_sl_timer_t *p_timer)
{
  (void)nrf_802154_sl_timer_remove(p_timer);
}

nrf_802154_sl_timer_ret_t
nrf_802154_sl_timer_add(nrf_802154_sl_timer_t *p_timer)
{
  uint32_t primask = irq_lock_local();
  nrf_802154_sl_timer_t *prev = NULL;
  nrf_802154_sl_timer_t *curr = alarm_head;

  if(timer_is_active(p_timer)) {
    (void)timer_remove_locked(p_timer);
  }

  while(curr != NULL && curr->trigger_time <= p_timer->trigger_time) {
    prev = curr;
    curr = timer_next_get(curr);
  }

  timer_next_set(p_timer, curr);
  timer_active_set(p_timer, true);

  if(prev == NULL) {
    alarm_head = p_timer;
    alarm_reschedule_locked();
  } else {
    timer_next_set(prev, p_timer);
  }

  irq_unlock_local(primask);
  return NRF_802154_SL_TIMER_RET_SUCCESS;
}

nrf_802154_sl_timer_ret_t
nrf_802154_sl_timer_remove(nrf_802154_sl_timer_t *p_timer)
{
  nrf_802154_sl_timer_ret_t ret;
  uint32_t primask = irq_lock_local();

  ret = timer_remove_locked(p_timer);

  irq_unlock_local(primask);

  return ret;
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
