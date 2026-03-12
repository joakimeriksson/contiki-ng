/*
 * Copyright (c) 2023, RISE Research Institutes of Sweden
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
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

/*
 * Authors: Niclas Finne <niclas.finne@ri.se>
 *          Nicolas Tsiftes <nicolas.tsiftes@ri.se>
 */

#include "contiki.h"
#include "lib/dbg-io/dbg.h"
#include "net/linkaddr.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "sys/autostart.h"
#include "tz-api.h"

#include <arm_cmse.h>
#include <stdarg.h>
#include <string.h>

static struct tz_api tz_api;
static bool initialized;
static volatile unsigned poll_wait_counter;

/*---------------------------------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "TZAPI"
#define LOG_LEVEL LOG_LEVEL_INFO
/*---------------------------------------------------------------------------*/
#define TZ_API_PRINTLN_MAX_LEN 256
/*---------------------------------------------------------------------------*/
process_event_t trustzone_init_event;
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL bool
tz_api_init(struct tz_api *apip)
{
  if(initialized || apip == NULL) {
    return false;
  }

  trustzone_init_event = process_alloc_event();

  apip = cmse_check_address_range(apip, sizeof(*apip), CMSE_NONSECURE);
  if(apip == NULL ||
     apip->request_poll == NULL ||
     apip->process_packet_data == NULL ||
     apip->packet_data == NULL ||
     apip->packet_data_size < 2) {
    return false;
  }

  if(cmse_check_address_range(apip->request_poll, sizeof(apip->request_poll),
                              CMSE_NONSECURE) == NULL) {
    return false;
  }

  if(cmse_check_address_range(apip->process_packet_data,
                              sizeof(apip->process_packet_data),
                              CMSE_NONSECURE) == NULL) {
    return false;
  }

  if(cmse_check_address_range(apip->packet_data,
                              apip->packet_data_size,
                              CMSE_NONSECURE) == NULL) {
    return false;
  }

  memcpy(&tz_api, apip, sizeof(tz_api));

  initialized = true;

  for(size_t i = 0; autostart_processes[i] != NULL; i++) {
    process_post(autostart_processes[i], trustzone_init_event, NULL);
  }
  tz_api_poll();

  return true;
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL bool
tz_api_poll(void)
{
  static bool is_poll_running;

  if(!initialized || is_poll_running) {
    return false;
  }
  is_poll_running = true;

  process_num_events_t event_count = process_nevents();

  if(event_count > 0) {
    LOG_DBG("Processing %u event%s at %lu\n", event_count,
	    event_count == 1 ? "" : "s", (unsigned long)clock_time());
  }

  while(event_count-- > 0) {
    process_run();
    watchdog_periodic();
  }

  is_poll_running = false;
  poll_wait_counter = 0;

  return process_nevents() > 0;
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL void
tz_api_println(const char *text)
{
  char secure_text[TZ_API_PRINTLN_MAX_LEN];
  dbg_output_context_t previous_context;
  size_t i = 0;

  if(text == NULL) {
    secure_text[0] = '(';
    secure_text[1] = 'n';
    secure_text[2] = 'u';
    secure_text[3] = 'l';
    secure_text[4] = 'l';
    secure_text[5] = ')';
    secure_text[6] = '\0';
  } else {
    while(i < sizeof(secure_text) - 1) {
      const char *ns_charp = cmse_check_address_range((void *)(text + i), 1,
                                                      CMSE_NONSECURE);
      char c;

      if(ns_charp == NULL) {
        break;
      }

      c = *ns_charp;
      if(c == '\0' || c == '\n' || c == '\r') {
        break;
      }

      secure_text[i++] = c;
    }

    secure_text[i] = '\0';
  }

  previous_context = dbg_output_context_swap(DBG_OUTPUT_CONTEXT_NONSECURE);
  dbg_putchar('n');
  dbg_putchar('>');
  dbg_putchar(' ');
  for(i = 0; secure_text[i] != '\0'; i++) {
    dbg_putchar(secure_text[i]);
  }
  dbg_putchar('\n');
  dbg_output_context_swap(previous_context);
}
/*---------------------------------------------------------------------------*/
bool
tz_api_request_poll_from_ns(void)
{
  if(!initialized) {
    return false;
  }
  if(poll_wait_counter > 0) {
    poll_wait_counter--;
    return false;
  }
  /* Wait some time for the poll before timeout and request again */
  poll_wait_counter = 128;
  return tz_api.request_poll();
}
/*---------------------------------------------------------------------------*/
void
tz_api_process_packet_data(void)
{
  if(!initialized || packetbuf_datalen() > tz_api.packet_data_size) {
    return;
  }

  memcpy(tz_api.packet_data, packetbuf_dataptr(), packetbuf_datalen());
  tz_api.process_packet_data(packetbuf_datalen());
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_prepare(const void *payload, unsigned short payload_len)
{
  return NETSTACK_RADIO.prepare(payload, payload_len);
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_transmit(unsigned short transmit_len)
{
  return NETSTACK_RADIO.transmit(transmit_len);
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_send(const void *payload, unsigned short payload_len)
{
  return NETSTACK_RADIO.send(payload, payload_len);
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_read(void *buf, unsigned short buf_len)
{
  return NETSTACK_RADIO.read(buf, buf_len);
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_channel_clear(void)
{
  return NETSTACK_RADIO.channel_clear();
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_receiving_packet(void)
{
  return NETSTACK_RADIO.receiving_packet();
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_pending_packet(void)
{
  return NETSTACK_RADIO.pending_packet();
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL int
tz_api_radio_set_power_mode(bool on)
{
  return on ? NETSTACK_RADIO.on() : NETSTACK_RADIO.off();
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL radio_result_t
tz_api_radio_get_value(radio_param_t param, radio_value_t *value)
{
  return NETSTACK_RADIO.get_value(param, value);
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL radio_result_t
tz_api_radio_set_value(radio_param_t param, radio_value_t value)
{
  return NETSTACK_RADIO.set_value(param, value);
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL radio_result_t
tz_api_radio_get_object(radio_param_t param, void *dest, size_t size)
{
  if(param == RADIO_PARAM_64BIT_ADDR && dest != NULL && size >= 8) {
    memset(dest, 0, size);
    memcpy((uint8_t *)dest + size - LINKADDR_SIZE, &linkaddr_node_addr,
           LINKADDR_SIZE);
    return RADIO_RESULT_OK;
  }

  return NETSTACK_RADIO.get_object(param, dest, size);
}
/*---------------------------------------------------------------------------*/
CC_TRUSTZONE_SECURE_CALL radio_result_t
tz_api_radio_set_object(radio_param_t param, const void *src, size_t size)
{
  return NETSTACK_RADIO.set_object(param, src, size);
}
/*---------------------------------------------------------------------------*/
