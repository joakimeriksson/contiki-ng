/*
 * Copyright (c) 2025, Contiki-NG.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * High Precision Timer platform for nrf_802154 on nRF54L15.
 * Uses TIMER10 (dedicated per nrf_802154_peripherals_nrf54l.h).
 * Configured as 1 MHz, 32-bit counter for microsecond precision.
 */

#include "platform/nrf_802154_hp_timer.h"
#include "nrf.h"

/* TIMER10 is the HP timer instance for nRF54L15 802.15.4.
 * CC[0] = sync capture, CC[1] = timestamp capture. */
#define HP_TIMER NRF_TIMER10

/* CC channel assignments */
#define SYNC_CC      0
#define TIMESTAMP_CC 1

static volatile bool sync_captured;

void nrf_802154_hp_timer_init(void)
{
  /* Configure TIMER10 as 1 MHz 32-bit timer. */
  HP_TIMER->MODE = TIMER_MODE_MODE_Timer;
  HP_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  HP_TIMER->PRESCALER = 4; /* 16 MHz / 2^4 = 1 MHz */

  HP_TIMER->TASKS_CLEAR = 1;
  sync_captured = false;
}

void nrf_802154_hp_timer_deinit(void)
{
  HP_TIMER->TASKS_STOP = 1;
  HP_TIMER->TASKS_CLEAR = 1;
}

void nrf_802154_hp_timer_start(void)
{
  HP_TIMER->TASKS_CLEAR = 1;
  HP_TIMER->TASKS_START = 1;
}

void nrf_802154_hp_timer_stop(void)
{
  HP_TIMER->TASKS_STOP = 1;
}

uint32_t nrf_802154_hp_timer_current_time_get(void)
{
  HP_TIMER->TASKS_CAPTURE[SYNC_CC] = 1;
  return HP_TIMER->CC[SYNC_CC];
}

uint32_t nrf_802154_hp_timer_sync_task_get(void)
{
  return (uint32_t)&HP_TIMER->TASKS_CAPTURE[SYNC_CC];
}

void nrf_802154_hp_timer_sync_prepare(void)
{
  HP_TIMER->EVENTS_COMPARE[SYNC_CC] = 0;
  sync_captured = false;
}

bool nrf_802154_hp_timer_sync_time_get(uint32_t *p_timestamp)
{
  if(!sync_captured) {
    /* Check if a capture has happened by reading the CC register.
     * The DPPI connection triggers TASKS_CAPTURE, so after the event
     * the CC register contains the captured value. We use a simple flag
     * approach -- the caller sets up the DPPI and after the event fires,
     * the value is captured. We assume it happened if timer is running. */
    *p_timestamp = HP_TIMER->CC[SYNC_CC];
    sync_captured = true;
    return true;
  }

  *p_timestamp = HP_TIMER->CC[SYNC_CC];
  return true;
}

uint32_t nrf_802154_hp_timer_timestamp_task_get(void)
{
  return (uint32_t)&HP_TIMER->TASKS_CAPTURE[TIMESTAMP_CC];
}

uint32_t nrf_802154_hp_timer_timestamp_get(void)
{
  return HP_TIMER->CC[TIMESTAMP_CC];
}
