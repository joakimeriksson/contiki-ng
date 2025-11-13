/*
 * Rust TCP/IP Stack - C Wrapper for Contiki-NG Process System
 *
 * This file provides the Contiki-NG PROCESS infrastructure and helper
 * functions for the Rust TCP/IP implementation. The core TCP/IP logic
 * is implemented in Rust (tcpip.rs), while this file handles:
 * - Process management (PROCESS macros)
 * - Event handling
 * - C function wrappers for Rust to call
 */

#include "contiki.h"
#include "contiki-net.h"
#include "net/ipv6/uip-packetqueue.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/linkaddr.h"
#include "net/routing/routing.h"

#include <string.h>

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "TCP/IP-Rust"
#define LOG_LEVEL LOG_LEVEL_TCPIP

/* Rust function declarations */
extern void tcpip_rust_init(void);
extern int tcpip_rust_packet_input(void);
extern void tcpip_rust_input(void);
extern int tcpip_rust_ipv6_output(void);
extern int tcpip_rust_output(const uint8_t *lladdr);
extern void tcpip_rust_eventhandler(uint8_t ev, void *data);

/* Process events */
process_event_t tcpip_event;
#if UIP_CONF_ICMP6
process_event_t tcpip_icmp6_event;
#endif

/* Periodic timer */
static struct etimer periodic;

#if UIP_CONF_IPV6_REASSEMBLY
extern struct etimer uip_reass_timer;
#endif

/* Event types */
enum {
  TCP_POLL,
  UDP_POLL,
  PACKET_INPUT
};

/*---------------------------------------------------------------------------*/
/* Helper functions for Rust to call C code */
/*---------------------------------------------------------------------------*/

/* Logging functions */
void tcpip_rust_log_info(const char *msg)
{
  LOG_INFO("%s", msg);
}

void tcpip_rust_log_warn(const char *msg)
{
  LOG_WARN("%s", msg);
}

void tcpip_rust_log_err(const char *msg)
{
  LOG_ERR("%s", msg);
}

/* Network layer output */
int tcpip_rust_network_output(const uint8_t *lladdr)
{
  return NETSTACK_NETWORK.output((const linkaddr_t *)lladdr);
}

/* Routing helpers */
int tcpip_rust_get_nexthop(const uip_ipaddr_t *dest, uip_ipaddr_t *out)
{
  uip_ds6_route_t *route;
  const uip_ipaddr_t *nexthop;

  /* Check if destination is on-link */
  if(uip_ds6_is_addr_onlink(dest)) {
    uip_ipaddr_copy(out, dest);
    return 0;
  }

  /* Check route table */
  route = uip_ds6_route_lookup(dest);
  if(route == NULL) {
    /* Use default route */
    nexthop = uip_ds6_defrt_choose();
    if(nexthop == NULL) {
      return -1;
    }
  } else {
    nexthop = uip_ds6_route_nexthop(route);
    if(nexthop == NULL) {
      return -1;
    }
  }

  uip_ipaddr_copy(out, nexthop);
  return 0;
}

int tcpip_rust_is_addr_onlink(const uip_ipaddr_t *addr)
{
  return uip_ds6_is_addr_onlink(addr) ? 1 : 0;
}

int tcpip_rust_is_my_addr(const uip_ipaddr_t *addr)
{
  return uip_ds6_is_my_addr(addr) ? 1 : 0;
}

/*---------------------------------------------------------------------------*/
/* TCP/IP output function (called by applications) */
/*---------------------------------------------------------------------------*/
uint8_t
tcpip_output(const uip_lladdr_t *a)
{
  int ret;

  /* Tag Traffic Class if needed */
#if UIP_TAG_TC_WITH_VARIABLE_RETRANSMISSIONS
  if(uipbuf_get_attr(UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS) !=
     UIP_MAX_MAC_TRANSMISSIONS_UNDEFINED) {
    UIP_IP_BUF->vtc = 0x60 | (UIP_TC_MAC_TRANSMISSION_COUNTER_BIT >> 4);
    UIP_IP_BUF->tcflow =
      uipbuf_get_attr(UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS) << 4;
  }
#endif

  /* Call Rust output function */
  if(netstack_process_ip_callback(NETSTACK_IP_OUTPUT, (const linkaddr_t *)a) ==
     NETSTACK_IP_PROCESS) {
    ret = tcpip_rust_output((const uint8_t *)a);
    return ret;
  } else {
    uipbuf_clear();
    return 0;
  }
}

/*---------------------------------------------------------------------------*/
/* TCP/IP input function (called by lower layers) */
/*---------------------------------------------------------------------------*/
void
tcpip_input(void)
{
  if(netstack_process_ip_callback(NETSTACK_IP_INPUT, NULL) ==
     NETSTACK_IP_PROCESS) {
    process_post_synch(&tcpip_process, PACKET_INPUT, NULL);
  }
  uipbuf_clear();
}

/*---------------------------------------------------------------------------*/
/* IPv6 output with routing */
/*---------------------------------------------------------------------------*/
void
tcpip_ipv6_output(void)
{
  tcpip_rust_ipv6_output();
}

/*---------------------------------------------------------------------------*/
/* TCP/UDP polling functions (for compatibility) */
/*---------------------------------------------------------------------------*/
#if UIP_UDP
void
tcpip_poll_udp(struct uip_udp_conn *conn)
{
  process_post(&tcpip_process, UDP_POLL, conn);
}
#endif

#if UIP_TCP
void
tcpip_poll_tcp(struct uip_conn *conn)
{
  process_post(&tcpip_process, TCP_POLL, conn);
}
#endif

/*---------------------------------------------------------------------------*/
/* Main TCP/IP process */
/*---------------------------------------------------------------------------*/
PROCESS(tcpip_process, "TCP/IP stack (Rust)");

PROCESS_THREAD(tcpip_process, ev, data)
{
  PROCESS_BEGIN();

  LOG_INFO("Starting Rust TCP/IP stack\n");

  /* Initialize Rust stack */
  tcpip_rust_init();

  /* Allocate events */
  tcpip_event = process_alloc_event();
#if UIP_CONF_ICMP6
  tcpip_icmp6_event = process_alloc_event();
#endif

  /* Start periodic timer */
  etimer_set(&periodic, CLOCK_SECOND / 2);

  /* Initialize routing protocol */
  NETSTACK_ROUTING.init();

  /* Main event loop */
  while(1) {
    PROCESS_YIELD();

    /* Dispatch to Rust event handler */
    if(ev == PROCESS_EVENT_TIMER) {
      /* Handle periodic timer */
      if(data == &periodic && etimer_expired(&periodic)) {
        etimer_restart(&periodic);
        /* Periodic processing handled by routing protocol */
      }

#if UIP_CONF_IPV6_REASSEMBLY
      if(data == &uip_reass_timer && etimer_expired(&uip_reass_timer)) {
        uip_reass_over();
        tcpip_ipv6_output();
      }
#endif

      /* DS6 periodic timer */
      if(data == &uip_ds6_timer_periodic && etimer_expired(&uip_ds6_timer_periodic)) {
        uip_ds6_periodic();
        tcpip_ipv6_output();
      }

#if !UIP_CONF_ROUTER
      /* Router solicitation timer */
      if(data == &uip_ds6_timer_rs && etimer_expired(&uip_ds6_timer_rs)) {
        uip_ds6_send_rs();
        tcpip_ipv6_output();
      }
#endif
    } else if(ev == PACKET_INPUT) {
      /* Process packet with Rust */
      tcpip_rust_packet_input();
    } else if(ev == TCP_POLL) {
      /* TCP polling - will be handled when TCP is ported to Rust */
      LOG_DBG("TCP poll event (not implemented)\n");
    } else if(ev == UDP_POLL) {
      /* UDP polling - will be handled when UDP is ported to Rust */
      LOG_DBG("UDP poll event (not implemented)\n");
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
