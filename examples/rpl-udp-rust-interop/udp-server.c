/*
 * UDP Server with Rust IPv6 Stack
 *
 * This server uses the Rust-based IPv6 stack to demonstrate
 * interoperability with C-based clients.
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"

#include "sys/log.h"
#define LOG_MODULE "Rust-Server"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Include Rust stack if enabled */
#if UIP6_RUST_CONF_ENABLE
#include "uip6-rust-conf.h"
#include "uip6-rust-glue.h"
#endif

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

static struct simple_udp_connection udp_conn;
static uint32_t rx_count = 0;
static uint32_t tx_count = 0;

PROCESS(udp_server_process, "UDP server (Rust)");
AUTOSTART_PROCESSES(&udp_server_process);
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
  rx_count++;

  LOG_INFO("[RUST] Received request #%"PRIu32": '%.*s' from ",
           rx_count, datalen, (char *) data);
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");

#if WITH_SERVER_REPLY
  /* Send back the same string to the client as an echo reply */
  LOG_INFO("[RUST] Sending response #%"PRIu32"\n", tx_count);
  simple_udp_sendto(&udp_conn, data, datalen, sender_addr);
  tx_count++;
#endif /* WITH_SERVER_REPLY */

  /* Print statistics every 10 messages */
  if(rx_count % 10 == 0) {
    LOG_INFO("[RUST] Statistics - Received: %"PRIu32", Sent: %"PRIu32"\n",
             rx_count, tx_count);

#if UIP6_RUST_ENABLE
    /* Print Rust stack stats */
    uip6_rust_glue_get_stats();
#endif
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  PROCESS_BEGIN();

  LOG_INFO("======================================\n");
  LOG_INFO("  RPL-UDP Server with Rust IPv6\n");
  LOG_INFO("======================================\n");

#if UIP6_RUST_ENABLE
  LOG_INFO("Using Rust IPv6 stack\n");

  /* Initialize Rust IPv6 stack */
  uip6_rust_glue_init();

  #if UIP6_RUST_HYBRID_MODE
  LOG_INFO("Mode: Hybrid (Rust + C fallback)\n");
  #else
  LOG_INFO("Mode: Rust only\n");
  #endif

  /* Get Rust stack version */
  uip6_rust_glue_get_stats();
#else
  LOG_INFO("Using C IPv6 stack\n");
#endif

  LOG_INFO("======================================\n");

  /* Initialize DAG root */
  LOG_INFO("Initializing as RPL DAG root...\n");
  NETSTACK_ROUTING.root_start();

  /* Initialize UDP connection */
  LOG_INFO("Registering UDP server on port %d\n", UDP_SERVER_PORT);
  simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                      UDP_CLIENT_PORT, udp_rx_callback);

  LOG_INFO("Server ready! Waiting for clients...\n");
  LOG_INFO("\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
