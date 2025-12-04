/*
 * RPL-UDP Client with button events, sensor data, and RSSI monitoring
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "random.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "net/ipv6/uipbuf.h"
#include "dev/button-hal.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include "batmon-sensor.h"

#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

/* For dynamic log level control */
void log_set_level(const char *module, int level);

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

/* Keepalive interval - send every 10 seconds */
#define KEEPALIVE_INTERVAL (10 * CLOCK_SECOND)

/* Message types */
#define MSG_TYPE_KEEPALIVE 'k'
#define MSG_TYPE_BUTTON    'b'
#define MSG_TYPE_ACK       'a'

static struct simple_udp_connection udp_conn;
static uint32_t seq_num = 0;
static uint32_t rx_count = 0;
static uint32_t tx_count = 0;

/* Last received RSSI (to include in keepalives) */
static int16_t last_rssi = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static int
read_battery(void)
{
  /* Read battery voltage (available on all CC13xx/CC26xx) */
  int bat_raw = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  return (bat_raw * 125) >> 5;  /* Convert to mV */
}
/*---------------------------------------------------------------------------*/
static void
send_keepalive(void)
{
  static char msg[64];
  uip_ipaddr_t dest_ipaddr;
  int bat;

  if(!NETSTACK_ROUTING.node_is_reachable() ||
     !NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
    LOG_INFO("Not reachable yet\n");
    return;
  }

  bat = read_battery();

  snprintf(msg, sizeof(msg),
           "{\"t\":\"%c\",\"s\":%" PRIu32 ",\"r\":%d,\"bat\":%d}",
           MSG_TYPE_KEEPALIVE, seq_num, last_rssi, bat);

  LOG_INFO("Tx keepalive seq=%" PRIu32 " rssi=%d bat=%dmV to ", seq_num, last_rssi, bat);
  LOG_INFO_6ADDR(&dest_ipaddr);
  LOG_INFO_("\n");

  simple_udp_sendto(&udp_conn, msg, strlen(msg), &dest_ipaddr);
  seq_num++;
  tx_count++;

  leds_single_on(LEDS_LED1);
}
/*---------------------------------------------------------------------------*/
static void
send_button(int button_id)
{
  static char msg[64];
  uip_ipaddr_t dest_ipaddr;

  if(!NETSTACK_ROUTING.node_is_reachable() ||
     !NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
    LOG_INFO("Not reachable yet\n");
    return;
  }

  snprintf(msg, sizeof(msg), "{\"t\":\"%c\",\"s\":%" PRIu32 ",\"b\":%d}",
           MSG_TYPE_BUTTON, seq_num, button_id);

  LOG_INFO("Tx button=%d seq=%" PRIu32 " to ", button_id, seq_num);
  LOG_INFO_6ADDR(&dest_ipaddr);
  LOG_INFO_("\n");

  simple_udp_sendto(&udp_conn, msg, strlen(msg), &dest_ipaddr);
  seq_num++;
  tx_count++;
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
  int16_t rx_rssi;

  /* Get RSSI of this received packet */
  rx_rssi = (int16_t)uipbuf_get_attr(UIPBUF_ATTR_RSSI);
  last_rssi = rx_rssi;

  LOG_INFO("Rx '%.*s' rssi=%d from ", datalen, (char *)data, rx_rssi);
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");

  /* Simple parse: look for "t":"X" */
  const char *tp = strstr((const char *)data, "\"t\":\"");
  if(tp != NULL) {
    type = tp[5];
  }

  /* Flash LED on receive */
  leds_single_on(LEDS_LED1);

  if(type == MSG_TYPE_ACK) {
    /* Parse RSSI from ACK to see how well server received us */
    const char *rp = strstr((const char *)data, "\"r\":");
    if(rp != NULL) {
      int server_rssi = atoi(rp + 4);
      LOG_INFO("Server heard us at %d dBm\n", server_rssi);
    }
  } else if(type == MSG_TYPE_BUTTON) {
    LOG_INFO("Server button press received!\n");
    /* Toggle LED on remote button */
    leds_single_toggle(LEDS_LED2);
  }

  rx_count++;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic_timer;
  static struct etimer led_timer;
  button_hal_button_t *btn;

  PROCESS_BEGIN();

  /* Lower log levels at startup for performance (ACK timing sensitive)
   * Use shell "log mac 4" etc. to enable debug when needed */
  log_set_level("mac", LOG_LEVEL_WARN);
  log_set_level("framer", LOG_LEVEL_WARN);
  log_set_level("radio", LOG_LEVEL_WARN);
  log_set_level("rpl", LOG_LEVEL_INFO);
  log_set_level("ipv6", LOG_LEVEL_WARN);

  /* Initialize sensors */
  SENSORS_ACTIVATE(batmon_sensor);

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, udp_rx_callback);

  /* Start keepalive timer with some jitter */
  etimer_set(&periodic_timer, random_rand() % KEEPALIVE_INTERVAL);

  LOG_INFO("UDP client started\n");
  LOG_INFO("Button count: %u\n", button_hal_button_count);

  while(1) {
    PROCESS_WAIT_EVENT();

    /* Handle button press */
    if(ev == button_hal_press_event) {
      btn = (button_hal_button_t *)data;
      LOG_INFO("Button %d (%s) pressed\n", btn->unique_id,
               BUTTON_HAL_GET_DESCRIPTION(btn));
      leds_single_on(LEDS_LED2);
      send_button(btn->unique_id);
    }

    /* Handle button release */
    if(ev == button_hal_release_event) {
      leds_single_off(LEDS_LED2);
    }

    /* Handle keepalive timer */
    if(ev == PROCESS_EVENT_TIMER && data == &periodic_timer) {
      send_keepalive();

      /* Turn off TX LED after short delay */
      etimer_set(&led_timer, CLOCK_SECOND / 4);

      /* Print stats every 10 keepalives */
      if((seq_num % 10) == 0) {
        LOG_INFO("Stats: Tx=%" PRIu32 " Rx=%" PRIu32 "\n", tx_count, rx_count);
      }

      /* Reset keepalive timer with jitter */
      etimer_set(&periodic_timer, KEEPALIVE_INTERVAL
                 - CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)));
    }

    /* Turn off LED after delay */
    if(ev == PROCESS_EVENT_TIMER && data == &led_timer) {
      leds_single_off(LEDS_LED1);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
