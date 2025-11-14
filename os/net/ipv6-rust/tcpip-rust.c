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

/* Rust UDP function declarations */
extern struct uip_udp_conn *uip_udp_new_rust(const uip_ipaddr_t *ripaddr, uint16_t rport);
extern void uip_udp_bind_rust(struct uip_udp_conn *conn, uint16_t lport);
extern struct uip_udp_conn *uip_udp_conn_rust(void);
extern struct uip_udp_conn *uip_udp_conns_rust(void);

/* Rust buffer management declarations */
extern uint8_t *uip_buf_ptr(void);
extern uint16_t *uip_len_ptr(void);
extern void *uip_aligned_buf_ptr(void);
extern void uip_set_len(uint16_t len);
extern uint16_t uip_get_len(void);

/* Debug logging helpers for Rust */
void rust_debug_log(const char *msg) {
  printf("%s", msg);
  fflush(stdout);
}

void rust_debug_log_int(const char *msg, int val) {
  printf("%s %d\n", msg, val);
  fflush(stdout);
}

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

/* Event types (used as data with tcpip_event) */
enum {
  TCP_POLL,
  UDP_POLL,
  PACKET_INPUT
};

/* Track event type for current event */
static int current_event_type;

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
  if(uip_ds6_is_addr_onlink((uip_ipaddr_t *)dest)) {
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
  return uip_ds6_is_addr_onlink((uip_ipaddr_t *)addr) ? 1 : 0;
}

int tcpip_rust_is_my_addr(const uip_ipaddr_t *addr)
{
  return uip_ds6_is_my_addr((uip_ipaddr_t *)addr) ? 1 : 0;
}

/*---------------------------------------------------------------------------*/
/* UDP functions (wrappers for Rust implementation) */
/*---------------------------------------------------------------------------*/

/* Override UDP functions to use Rust implementation */
#undef udp_new
#undef uip_udp_bind

struct uip_udp_conn *
udp_new(const uip_ipaddr_t *ripaddr, uint16_t rport, void *appstate)
{
  struct uip_udp_conn *c;

  /* Create new UDP connection via Rust */
  c = uip_udp_new_rust(ripaddr, rport);

  if(c == NULL) {
    return NULL;
  }

  /* Set TTL from ds6 interface */
  c->ttl = uip_ds6_if.cur_hop_limit;

  /* Store application state pointers for compatibility */
  c->appstate.p = PROCESS_CURRENT();
  c->appstate.state = appstate;

  return c;
}

struct uip_udp_conn *
uip_udp_new(const uip_ipaddr_t *ripaddr, uint16_t rport)
{
  struct uip_udp_conn *c = uip_udp_new_rust(ripaddr, rport);
  if(c != NULL) {
    c->ttl = uip_ds6_if.cur_hop_limit;
  }
  return c;
}

/* Note: uip_udp_conn is now defined in the buffer management section above */

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
  /* Force output to see if function is called */
  printf("[RUST-TCPIP] tcpip_input() CALLED! uip_len=%d\n", uip_len);
  LOG_INFO("[C] tcpip_input() called, uip_len=%d\n", uip_len);

  if(netstack_process_ip_callback(NETSTACK_IP_INPUT, NULL) ==
     NETSTACK_IP_PROCESS) {
    printf("[RUST-TCPIP] netstack callback OK, posting event\n");
    LOG_INFO("[C] Posting PACKET_INPUT event to tcpip_process\n");
    current_event_type = PACKET_INPUT;
    process_post_synch(&tcpip_process, tcpip_event, (void *)&current_event_type);
  } else {
    printf("[RUST-TCPIP] netstack callback FAILED!\n");
    LOG_INFO("[C] netstack_process_ip_callback returned != NETSTACK_IP_PROCESS\n");
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
/* Buffer management - redirect to Rust */
/*---------------------------------------------------------------------------*/

/* Provide actual global variables that point to Rust buffers */
/* These must be actual symbols that can be linked by other C files */
/* Note: uip_aligned_buf must match the structure that Rust provides */
uip_buf_t uip_aligned_buf;   /* Actual buffer structure */
/* Note: uip_buf is defined as a macro in uip.h: #define uip_buf (uip_aligned_buf.u8) */

uint16_t uip_len = 0;        /* Packet length */

/* Additional buffer variables from uip6.c that we need to provide */
uint16_t uip_ext_len = 0;    /* Extension header length */
uint16_t uip_slen = 0;       /* Send length */
uint8_t uip_last_proto = 0;  /* Last protocol number */

/* Additional uIP global variables needed by C code */
void *uip_appdata = NULL;    /* Application data pointer */

/* TCP/UDP connection pointers (stubs for now) */
struct uip_conn *uip_conn = NULL;         /* Current TCP connection */
struct uip_udp_conn *uip_udp_conn = NULL; /* Current UDP connection */
uint8_t uip_flags = 0;                    /* Connection flags */

/* Link-layer address */
uip_lladdr_t uip_lladdr;

/* Initialize buffer pointers to use Rust buffers */
static void init_buffer_ptrs(void) {
  uip_buf_t *rust_aligned_buf = (uip_buf_t *)uip_aligned_buf_ptr();
  uint16_t *rust_len = uip_len_ptr();

  /* Copy Rust buffer contents to our local buffer */
  /* This ensures C code sees the same data as Rust */
  memcpy(&uip_aligned_buf, rust_aligned_buf, sizeof(uip_buf_t));

  /* Sync length - we'll need to keep these in sync during packet processing */
  uip_len = *rust_len;
}

/*---------------------------------------------------------------------------*/
/* uIP function stubs */
/*---------------------------------------------------------------------------*/

/* tcpip_uipcall - stub for application callbacks */
void tcpip_uipcall(void)
{
  /* TODO: Implement proper application callbacks when TCP is ported */
  LOG_DBG("tcpip_uipcall() called (stub)\n");
}

/* uip_process - stub for processing (replaced by Rust) */
void uip_process(uint8_t flag)
{
  /* This is replaced by Rust packet processing */
  LOG_DBG("uip_process() called (stub)\n");
}

/* uip_icmp6chksum - ICMPv6 checksum calculation */
/* This should ideally call Rust checksum functions */
uint16_t uip_icmp6chksum(void)
{
  /* TODO: Call Rust checksum function */
  /* For now, return 0 (checksum disabled) */
  LOG_WARN("uip_icmp6chksum() called (stub - returning 0)\n");
  return 0;
}

/* uip_send - TCP send function (stub) */
void uip_send(const void *data, int len)
{
  /* TODO: Implement TCP send when TCP is ported */
  LOG_DBG("uip_send() called (stub)\n");
}

/* uip_remove_ext_hdr - Remove IPv6 extension header */
bool uip_remove_ext_hdr(void)
{
  /* TODO: Implement extension header removal */
  LOG_DBG("uip_remove_ext_hdr() called (stub)\n");
  return false;
}

/* uip_htonl - Host to network byte order (32-bit) */
uint32_t uip_htonl(uint32_t val)
{
  return ((val & 0xff000000) >> 24) |
         ((val & 0x00ff0000) >> 8) |
         ((val & 0x0000ff00) << 8) |
         ((val & 0x000000ff) << 24);
}

/*---------------------------------------------------------------------------*/
/* Main TCP/IP process */
/*---------------------------------------------------------------------------*/
PROCESS(tcpip_process, "TCP/IP stack (Rust)");

PROCESS_THREAD(tcpip_process, ev, data)
{
  PROCESS_BEGIN();

  printf("[RUST-TCPIP] ===== Starting Rust TCP/IP stack =====\n");
  LOG_INFO("Starting Rust TCP/IP stack\n");

  /* Initialize buffer pointers */
  init_buffer_ptrs();

  /* Initialize Rust stack */
  tcpip_rust_init();
  printf("[RUST-TCPIP] Rust stack initialized, entering event loop\n");

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
  printf("[RUST-TCPIP] Event loop started, tcpip_event=%d, PROCESS_EVENT_TIMER=%d\n",
         tcpip_event, PROCESS_EVENT_TIMER);

  while(1) {
    PROCESS_YIELD();

    printf("[RUST-TCPIP] Event: %d (tcpip_event=%d, TIMER=%d)\n",
           ev, tcpip_event, PROCESS_EVENT_TIMER);
    printf("[RUST-TCPIP] Comparisons: ev==TIMER=%d, ev==tcpip_event=%d\n",
           (ev == PROCESS_EVENT_TIMER), (ev == tcpip_event));
    LOG_DBG("[C] Event received: %d (tcpip_event=%d)\n", ev, tcpip_event);

    /* Dispatch to Rust event handler */
    if(ev == PROCESS_EVENT_TIMER) {
      printf("[RUST-TCPIP] --> Taking TIMER branch\n");
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
    } else if(ev == tcpip_event) {
      printf("[RUST-TCPIP] --> Taking TCPIP_EVENT branch!\n");
      /* Check event type from data pointer */
      int *event_type = (int *)data;
      printf("[RUST-TCPIP] data=%p, event_type=%d\n", data, event_type ? *event_type : -1);
      printf("[RUST-TCPIP] PACKET_INPUT constant=%d, TCP_POLL=%d, UDP_POLL=%d\n",
             PACKET_INPUT, TCP_POLL, UDP_POLL);
      printf("[RUST-TCPIP] Checking: event_type=%d, *event_type=%d, PACKET_INPUT=%d\n",
             (event_type != NULL), event_type ? *event_type : -999, PACKET_INPUT);
      printf("[RUST-TCPIP] Comparison result: (*event_type == PACKET_INPUT) = %d\n",
             event_type ? (*event_type == PACKET_INPUT) : -1);
      LOG_INFO("[C] tcpip_event received, type=%d\n", event_type ? *event_type : -1);

      if(event_type && *event_type == PACKET_INPUT) {
        printf("[RUST-TCPIP] --> PACKET_INPUT branch entered!\n");
        printf("[RUST-TCPIP] Step 1: uip_len=%d\n", uip_len);
        LOG_INFO("[C] PACKET_INPUT: uip_len=%d\n", uip_len);

        /* Sync C buffer length to Rust */
        printf("[RUST-TCPIP] Step 2: Calling uip_set_len(%d)\n", uip_len);
        uip_set_len(uip_len);
        printf("[RUST-TCPIP] Step 3: uip_set_len returned\n");

        /* Copy C buffer to Rust buffer */
        printf("[RUST-TCPIP] Step 4: Getting rust_buf pointer\n");
        uint8_t *rust_buf = uip_buf_ptr();
        printf("[RUST-TCPIP] Step 5: rust_buf=%p, copying %d bytes\n", rust_buf, uip_len);
        memcpy(rust_buf, uip_aligned_buf.u8, uip_len);
        printf("[RUST-TCPIP] Step 6: memcpy completed\n");

        LOG_INFO("[C] Synced buffer to Rust, calling tcpip_rust_packet_input()\n");

        /* Process packet with Rust */
        printf("[RUST-TCPIP] Step 7: Calling tcpip_rust_packet_input()\n");
        /* Test debug helpers from C */
        rust_debug_log("[C-TEST] Testing rust_debug_log from C\n");
        rust_debug_log_int("[C-TEST] Testing rust_debug_log_int from C:", 12345);
        fflush(stdout);  /* Force output before calling Rust */
        int rust_result = tcpip_rust_packet_input();
        fflush(stdout);  /* Force output after Rust returns */
        printf("[RUST-TCPIP] Step 8: tcpip_rust_packet_input() returned %d\n", rust_result);

        /* Sync Rust buffer back to C (for replies) */
        uint16_t *rust_len_ptr = uip_len_ptr();
        uip_len = *rust_len_ptr;
        printf("[RUST-TCPIP] Step 9: After Rust processing, uip_len=%d\n", uip_len);
        LOG_INFO("[C] After Rust processing: uip_len=%d\n", uip_len);
      } else if(event_type && *event_type == TCP_POLL) {
        printf("[RUST-TCPIP] --> TCP_POLL branch (not implemented)\n");
        /* TCP polling - will be handled when TCP is ported to Rust */
        LOG_DBG("[C] TCP poll event (not implemented)\n");
      } else if(event_type && *event_type == UDP_POLL) {
        printf("[RUST-TCPIP] --> UDP_POLL branch (not implemented)\n");
        /* UDP polling - will be handled when UDP is ported to Rust */
        LOG_DBG("[C] UDP poll event (not implemented)\n");
      } else {
        printf("[RUST-TCPIP] --> No event_type match! event_type=%p, *event_type=%d\n",
               event_type, event_type ? *event_type : -999);
      }
    } else {
      printf("[RUST-TCPIP] --> Taking ELSE branch (other event: %d)\n", ev);
      LOG_DBG("[C] Other event: %d\n", ev);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
