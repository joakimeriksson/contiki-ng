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
 *      Contiki configuration for ESP32-C6-DevKitC platform
 */

#ifndef CONTIKI_CONF_H_
#define CONTIKI_CONF_H_

#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* Include CPU and platform-specific configuration */
#include "esp32c6-def.h"
#include "esp32c6-conf.h"

/*---------------------------------------------------------------------------*/
/* Platform name */
#define PLATFORM_NAME "ESP32-C6-DevKitC"

/*---------------------------------------------------------------------------*/
/* Clock and timer configuration */
#define CLOCK_CONF_SECOND 1000

/* Rtimer configuration */
#ifndef RTIMER_CONF_SECOND
#define RTIMER_CONF_SECOND RTIMER_ARCH_SECOND
#endif

/*---------------------------------------------------------------------------*/
/* Network configuration */

/* IEEE 802.15.4 */
#define IEEE802154_CONF_PANID 0xABCD

/* 6LoWPAN */
#define LINKADDR_CONF_SIZE 8

/* Network buffer size */
#define PACKETBUF_CONF_SIZE 128
#define QUEUEBUF_CONF_NUM 8

/* UIP buffer size */
#define UIP_CONF_BUFFER_SIZE 1280

/* Neighbor table size */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 16

/* Routing table size */
#define UIP_CONF_MAX_ROUTES 16

/*---------------------------------------------------------------------------*/
/* Radio driver configuration */
#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO esp32c6_radio_driver
#endif

/*---------------------------------------------------------------------------*/
/* Logging configuration */
#ifndef LOG_CONF_LEVEL_MAIN
#define LOG_CONF_LEVEL_MAIN LOG_LEVEL_INFO
#endif

#ifndef LOG_CONF_LEVEL_IPV6
#define LOG_CONF_LEVEL_IPV6 LOG_LEVEL_WARN
#endif

#ifndef LOG_CONF_LEVEL_RPL
#define LOG_CONF_LEVEL_RPL LOG_LEVEL_WARN
#endif

#ifndef LOG_CONF_LEVEL_MAC
#define LOG_CONF_LEVEL_MAC LOG_LEVEL_WARN
#endif

/*---------------------------------------------------------------------------*/
/* GPIO and LED configuration */
#define LEDS_CONF_RED    1
#define LEDS_CONF_GREEN  2
#define LEDS_CONF_BLUE   4
#define LEDS_CONF_ALL    7

/*---------------------------------------------------------------------------*/
/* Memory configuration */
#define MMEM_CONF_SIZE 4096

/*---------------------------------------------------------------------------*/
/* Printf configuration */
#define DBG_CONF_USB 0
#define DBG_CONF_UART 1

/*---------------------------------------------------------------------------*/
/* Serial line configuration */
#ifndef SERIAL_LINE_CONF_BUFSIZE
#define SERIAL_LINE_CONF_BUFSIZE 128
#endif

/*---------------------------------------------------------------------------*/
/* Platform-specific types */
typedef uint32_t clock_time_t;
typedef uint32_t uip_stats_t;

/*---------------------------------------------------------------------------*/
#endif /* CONTIKI_CONF_H_ */
