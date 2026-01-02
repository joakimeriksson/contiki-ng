/*
 * RPL-UDP Server (DAG root) with button events and RSSI monitoring
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "net/ipv6/uipbuf.h"
#include "dev/button-hal.h"
#include "dev/leds.h"
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#include "shell.h"
#include "shell-commands.h"

/* For dynamic log level control */
void log_set_level(const char *module, int level);

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678
#define UDP_MCAST_PORT  5679

/* Message types */
#define MSG_TYPE_KEEPALIVE 'k'
#define MSG_TYPE_BUTTON    'b'
#define MSG_TYPE_ACK       'a'
#define MSG_TYPE_CONFIG    'c'
#define MSG_TYPE_MCAST     'm'

/* Client keepalive interval (seconds) - 0 means don't send in ACK */
static int client_interval = 0;

static struct simple_udp_connection udp_conn;
static struct simple_udp_connection mcast_conn;
static uint32_t seq_num = 0;
static uint32_t msg_rx_count = 0;
static uint32_t mcast_rx_count = 0;

/* Store last client address for button-triggered messages */
static uip_ipaddr_t last_client_addr;
static uint8_t have_client = 0;

PROCESS(udp_server_process, "UDP server");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/
static PT_THREAD(cmd_interval(struct pt *pt, shell_output_func output, char *args))
{
  PT_BEGIN(pt);

  if(args == NULL || strlen(args) == 0) {
    SHELL_OUTPUT(output, "Current interval: %d seconds (0=not sent)\n", client_interval);
  } else {
    int val = atoi(args);
    if(val >= 0 && val <= 3600) {
      client_interval = val;
      SHELL_OUTPUT(output, "Interval set to %d seconds\n", client_interval);
    } else {
      SHELL_OUTPUT(output, "Invalid interval (0-3600)\n");
    }
  }

  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
static const struct shell_command_t app_commands[] = {
  { "interval", cmd_interval, "'> interval [sec]': get/set client keepalive interval" },
  { NULL, NULL, NULL },
};
static struct shell_command_set_t app_shell_command_set = {
  .next = NULL,
  .commands = app_commands,
};
/*---------------------------------------------------------------------------*/
static void
send_ack(const uip_ipaddr_t *dest, uint32_t ack_seq, int16_t rssi)
{
  static char msg[80];

  if(client_interval > 0) {
    snprintf(msg, sizeof(msg), "{\"t\":\"%c\",\"s\":%" PRIu32 ",\"r\":%d,\"int\":%d}",
             MSG_TYPE_ACK, ack_seq, rssi, client_interval);
  } else {
    snprintf(msg, sizeof(msg), "{\"t\":\"%c\",\"s\":%" PRIu32 ",\"r\":%d}",
             MSG_TYPE_ACK, ack_seq, rssi);
  }

  LOG_INFO("Tx ACK seq=%" PRIu32 " rssi=%d to ", ack_seq, rssi);
  LOG_INFO_6ADDR(dest);
  LOG_INFO_("\n");

  simple_udp_sendto(&udp_conn, msg, strlen(msg), dest);
}
/*---------------------------------------------------------------------------*/
static void
send_button_to_clients(int button_id)
{
  static char msg[64];

  if(!have_client) {
    LOG_INFO("No client known yet\n");
    return;
  }

  snprintf(msg, sizeof(msg), "{\"t\":\"%c\",\"s\":%" PRIu32 ",\"b\":%d}",
           MSG_TYPE_BUTTON, seq_num, button_id);

  LOG_INFO("Tx button=%d seq=%" PRIu32 " to ", button_id, seq_num);
  LOG_INFO_6ADDR(&last_client_addr);
  LOG_INFO_("\n");

  simple_udp_sendto(&udp_conn, msg, strlen(msg), &last_client_addr);
  seq_num++;
}
/*---------------------------------------------------------------------------*/
static void
mcast_rx_callback(struct simple_udp_connection *c,
                  const uip_ipaddr_t *sender_addr,
                  uint16_t sender_port,
                  const uip_ipaddr_t *receiver_addr,
                  uint16_t receiver_port,
                  const uint8_t *data,
                  uint16_t datalen)
{
  LOG_INFO("MCAST Rx '%.*s' from ", datalen, (char *)data);
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");
  mcast_rx_count++;
  leds_single_toggle(LEDS_LED1);
}
/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
                const uip_ipaddr_t *sender_addr,
                uint16_t sender_port,
                const uip_ipaddr_t *receiver_addr,
                uint16_t receiver_port,
                const uint8_t *data,
                uint16_t datalen)
{
  char type = '?';
  uint32_t msg_seq = 0;
  int button_id = -1;
  int16_t rx_rssi;

  /* Get RSSI of this received packet */
  rx_rssi = (int16_t)uipbuf_get_attr(UIPBUF_ATTR_RSSI);

  LOG_INFO("Rx '%.*s' rssi=%d from ", datalen, (char *)data, rx_rssi);
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");

  /* Remember this client */
  uip_ipaddr_copy(&last_client_addr, sender_addr);
  have_client = 1;

  /* Simple JSON parsing */
  const char *tp = strstr((const char *)data, "\"t\":\"");
  if(tp != NULL) {
    type = tp[5];
  }

  const char *sp = strstr((const char *)data, "\"s\":");
  if(sp != NULL) {
    msg_seq = atoi(sp + 4);
  }

  const char *bp = strstr((const char *)data, "\"b\":");
  if(bp != NULL) {
    button_id = atoi(bp + 4);
  }

  msg_rx_count++;

  /* Flash LED on receive */
  leds_single_on(LEDS_LED1);

  if(type == MSG_TYPE_KEEPALIVE) {
    /* Parse data from keepalive */
    int bat = 0, client_rssi = 0;

    const char *bat_p = strstr((const char *)data, "\"bat\":");
    if(bat_p != NULL) bat = atoi(bat_p + 6);

    const char *rp = strstr((const char *)data, "\"r\":");
    if(rp != NULL) client_rssi = atoi(rp + 4);

    LOG_INFO("Keepalive seq=%" PRIu32 " client_rssi=%d bat=%dmV\n",
             msg_seq, client_rssi, bat);

    send_ack(sender_addr, msg_seq, rx_rssi);

  } else if(type == MSG_TYPE_BUTTON) {
    LOG_INFO("Button %d from client, seq=%" PRIu32 "\n", button_id, msg_seq);
    send_ack(sender_addr, msg_seq, rx_rssi);
    /* Toggle LED on remote button press */
    leds_single_toggle(LEDS_LED2);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  static struct etimer led_timer;
  button_hal_button_t *btn;

  PROCESS_BEGIN();

  /* Lower log levels at startup for performance (ACK timing sensitive)
   * Use shell "log mac 4" etc. to enable debug when needed */
  log_set_level("mac", LOG_LEVEL_INFO);   /* See CSMA tx retries */
  log_set_level("framer", LOG_LEVEL_WARN);
  log_set_level("radio", LOG_LEVEL_WARN);
  log_set_level("rpl", LOG_LEVEL_INFO);
  log_set_level("ipv6", LOG_LEVEL_WARN);

  /* Register shell commands */
  shell_command_set_register(&app_shell_command_set);

  /* Initialize DAG root */
  NETSTACK_ROUTING.root_start();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                      UDP_CLIENT_PORT, udp_rx_callback);

  /* Initialize multicast UDP connection */
  simple_udp_register(&mcast_conn, UDP_MCAST_PORT, NULL,
                      UDP_MCAST_PORT, mcast_rx_callback);

  LOG_INFO("UDP server started (DAG root)\n");
  LOG_INFO("Listening for multicast on port %d\n", UDP_MCAST_PORT);
  LOG_INFO("Button count: %u\n", button_hal_button_count);

  while(1) {
    PROCESS_WAIT_EVENT();

    /* Handle button press - send to known client */
    if(ev == button_hal_press_event) {
      btn = (button_hal_button_t *)data;
      LOG_INFO("Button %d (%s) pressed\n", btn->unique_id,
               BUTTON_HAL_GET_DESCRIPTION(btn));
      leds_single_on(LEDS_LED2);
      send_button_to_clients(btn->unique_id);
    }

    /* Handle button release */
    if(ev == button_hal_release_event) {
      leds_single_off(LEDS_LED2);
    }

    /* Turn off RX LED after delay */
    if(ev == PROCESS_EVENT_TIMER && data == &led_timer) {
      leds_single_off(LEDS_LED1);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
