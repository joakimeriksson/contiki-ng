/*

Copyright (c) 2010 - 2025, Nordic Semiconductor ASA All rights reserved.

SPDX-License-Identifier: BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of Nordic Semiconductor ASA nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef NRF54L15_H
#define NRF54L15_H

#ifdef __cplusplus
    extern "C" {
#endif

/* MDK version info required by nrfx */
#define MDK_MAJOR_VERSION 8
#define MDK_MINOR_VERSION 60
#define MDK_MICRO_VERSION 3

/* Define GPIOTE_CH_NUM for HAL compatibility (use max of GPIOTE20/30) */
#ifndef GPIOTE_CH_NUM
#define GPIOTE_CH_NUM 8
#endif

/*
 * nRF54L15 CPUAPP uses interrupt group 1 in secure mode and group 0 in
 * non-secure mode, matching Zephyr's CPUAPP DTS setup. FLPR uses group 0.
 *
 * This must be visible before nrfx pulls in the interim header because the
 * HAL token-pastes the group number into register names.
 */
#ifndef GPIOTE_IRQ_GROUP
#if defined(NRF_FLPR) || defined(NRF_TRUSTZONE_NONSECURE)
#define GPIOTE_IRQ_GROUP 0
#else
#define GPIOTE_IRQ_GROUP 1
#endif
#endif

/*
 * Force NRF_GPIOTE_PORT_ID to 0 by defining LUMOS_XXAA for GPIOTE
 * nRF54L15 EVENTS_PORT array has only [1] element, so PORT_ID must be 0
 * (not 1 as would be set by NRF_APPLICATION in nrf_gpiote.h)
 * LUMOS_XXAA is another nRF54 series chip that also uses PORT_ID=0
 */
#ifndef LUMOS_XXAA
#define LUMOS_XXAA
#endif

/* nRF54L15 has DPPIC - define DPPI_PRESENT for nrfx compatibility */
#ifndef DPPI_PRESENT
#define DPPI_PRESENT 1
#endif

/* Define DPPI subscribe/publish enable bit */
#ifndef NRF_SUBSCRIBE_PUBLISH_ENABLE
#define NRF_SUBSCRIBE_PUBLISH_ENABLE (1UL << 31)
#endif

/* Define GRTC compatibility macros for the application domain. In the
 * TrustZone split, non-secure runs on interrupt/syscounter group 1 while the
 * secure world keeps group 2 for radio-side timing.
 */
#ifndef GRTC_IRQn
#if defined(NRF_TRUSTZONE_NONSECURE)
#define GRTC_IRQn GRTC_1_IRQn
#else
#define GRTC_IRQn GRTC_2_IRQn
#endif
#endif
#ifndef GRTC_IRQ_GROUP
#if defined(NRF_TRUSTZONE_NONSECURE)
#define GRTC_IRQ_GROUP 1
#else
#define GRTC_IRQ_GROUP 2
#endif
#endif

#include "nrf54l15_types.h"

/*
 * Define GPIOTE register name compatibility aliases for HAL.
 * The HAL uses numbered INTENSET/INTENCLR names on nRF54L15, but it also
 * probes for a few unnumbered compatibility symbols.
 */
#define GPIOTE_INTENSET_IN0_Msk GPIOTE_INTENSET0_IN0_Msk
#define GPIOTE_INTENSET_IN1_Msk GPIOTE_INTENSET0_IN1_Msk
#define GPIOTE_INTENSET_IN2_Msk GPIOTE_INTENSET0_IN2_Msk
#define GPIOTE_INTENSET_IN3_Msk GPIOTE_INTENSET0_IN3_Msk
#define GPIOTE_INTENSET_IN4_Msk GPIOTE_INTENSET0_IN4_Msk
#define GPIOTE_INTENSET_IN5_Msk GPIOTE_INTENSET0_IN5_Msk
#define GPIOTE_INTENSET_IN6_Msk GPIOTE_INTENSET0_IN6_Msk
#define GPIOTE_INTENSET_IN7_Msk GPIOTE_INTENSET0_IN7_Msk
#define GPIOTE_INTENSET_PORT0SECURE_Msk GPIOTE_INTENSET0_PORT0SECURE_Msk

#include "nrf54l15_global.h"
#include "nrf54l15_application.h"
#include "nrf54l15_flpr.h"

/* DPPIC channel/group count aliases for nrf_dppi.h compatibility */
#ifndef DPPIC00_CH_NUM
#define DPPIC00_CH_NUM DPPIC00_CH_NUM_SIZE
#endif
#ifndef DPPIC10_CH_NUM
#define DPPIC10_CH_NUM DPPIC10_CH_NUM_SIZE
#endif
#ifndef DPPIC20_CH_NUM
#define DPPIC20_CH_NUM DPPIC20_CH_NUM_SIZE
#endif
#ifndef DPPIC30_CH_NUM
#define DPPIC30_CH_NUM DPPIC30_CH_NUM_SIZE
#endif
#ifndef DPPIC00_GROUP_NUM
#define DPPIC00_GROUP_NUM DPPIC00_GROUP_NUM_SIZE
#endif
#ifndef DPPIC10_GROUP_NUM
#define DPPIC10_GROUP_NUM DPPIC10_GROUP_NUM_SIZE
#endif
#ifndef DPPIC20_GROUP_NUM
#define DPPIC20_GROUP_NUM DPPIC20_GROUP_NUM_SIZE
#endif
#ifndef DPPIC30_GROUP_NUM
#define DPPIC30_GROUP_NUM DPPIC30_GROUP_NUM_SIZE
#endif

/* GPIO pin count aliases for nrfx GPIOTE driver */
#ifndef P0_PIN_NUM
#define P0_PIN_NUM P0_PIN_NUM_SIZE
#endif
#ifndef P1_PIN_NUM
#define P1_PIN_NUM P1_PIN_NUM_SIZE
#endif
#ifndef P2_PIN_NUM
#define P2_PIN_NUM P2_PIN_NUM_SIZE  /* P2 has 11 pins on nRF54L15 */
#endif

/* GPIOTE instance defines for nrfx driver */
#ifndef GPIOTE20_CH_NUM
#define GPIOTE20_CH_NUM 8
#endif
#ifndef GPIOTE30_CH_NUM
#define GPIOTE30_CH_NUM 4
#endif
#ifndef GPIOTE20_AVAILABLE_GPIO_PORTS
#define GPIOTE20_AVAILABLE_GPIO_PORTS 0x00000002UL /* P1 */
#endif
#ifndef GPIOTE30_AVAILABLE_GPIO_PORTS
#define GPIOTE30_AVAILABLE_GPIO_PORTS 0x00000001UL /* P0 */
#endif

/* Peripheral aliases for nrfx driver compatibility */
#ifndef NRF_WDT0
#define NRF_WDT0 NRF_WDT30
#endif

#ifndef NRF_TIMER0
#define NRF_TIMER0 NRF_TIMER00
#endif

#ifndef RADIO_IRQn
#define RADIO_IRQn RADIO_0_IRQn
#endif

#ifdef __cplusplus
}
#endif
#endif /* NRF54L15_H */
