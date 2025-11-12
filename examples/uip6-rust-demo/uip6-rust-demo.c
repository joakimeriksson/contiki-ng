/**
 * \file uip6-rust-demo.c
 * \brief Demo application showing Rust IPv6 stack configuration
 *
 * This application demonstrates how to enable and use the Rust-based
 * IPv6 stack using the configuration system.
 */

#include "contiki.h"
#include "contiki-net.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-ds6.h"

/* Include Rust stack headers if enabled */
#include "uip6-rust-conf.h"
#if UIP6_RUST_ENABLE
#include "uip6-rust-glue.h"
#endif

#include "sys/log.h"
#define LOG_MODULE "Demo"
#define LOG_LEVEL LOG_LEVEL_INFO

/*---------------------------------------------------------------------------*/
PROCESS(demo_process, "uIP6-Rust Demo");
AUTOSTART_PROCESSES(&demo_process);
/*---------------------------------------------------------------------------*/

static void
print_stack_info(void)
{
  LOG_INFO("=================================\n");
  LOG_INFO("  IPv6 Stack Configuration\n");
  LOG_INFO("=================================\n");

#if UIP6_RUST_ENABLE
  LOG_INFO("Stack: Rust-based IPv6\n");

  #if UIP6_RUST_HYBRID_MODE
  LOG_INFO("Mode: Hybrid (C + Rust)\n");
  #else
  LOG_INFO("Mode: Rust only\n");
  #endif

  #if UIP6_RUST_LOG_LEVEL > LOG_LEVEL_NONE
  LOG_INFO("Logging: Enabled\n");
  #else
  LOG_INFO("Logging: Disabled\n");
  #endif

  /* Get Rust stack stats */
  uip6_rust_glue_get_stats();

#else
  LOG_INFO("Stack: C-based IPv6 (uip6)\n");
  LOG_INFO("Mode: Standard C\n");
#endif

  LOG_INFO("=================================\n");
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(demo_process, ev, data)
{
  static struct etimer timer;
  uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();

  LOG_INFO("\n");
  LOG_INFO("**********************************\n");
  LOG_INFO("  uIP6-Rust Demo Application\n");
  LOG_INFO("**********************************\n");
  LOG_INFO("\n");

  /* Print stack information */
  print_stack_info();

#if UIP6_RUST_ENABLE
  /* Initialize Rust stack */
  LOG_INFO("Initializing Rust IPv6 stack...\n");
  uip6_rust_glue_init();

  /* Add a link-local address */
  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0001);
  LOG_INFO("Adding link-local address: ");
  LOG_INFO_6ADDR(&ipaddr);
  LOG_INFO_("\n");

  if(uip6_rust_addr_add(&ipaddr, 2) == 0) {
    LOG_INFO("✓ Address added successfully\n");
  } else {
    LOG_ERR("✗ Failed to add address\n");
  }

  /* Add a global address */
  uip_ip6addr(&ipaddr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0001);
  LOG_INFO("Adding global address: ");
  LOG_INFO_6ADDR(&ipaddr);
  LOG_INFO_("\n");

  if(uip6_rust_addr_add(&ipaddr, 0) == 0) {
    LOG_INFO("✓ Address added successfully\n");
  } else {
    LOG_ERR("✗ Failed to add address\n");
  }

  #if UIP6_RUST_HYBRID_MODE
  /* In hybrid mode, sync addresses to Rust stack */
  LOG_INFO("Synchronizing addresses to Rust stack...\n");
  uip6_rust_glue_sync_addresses();
  #endif

#else
  /* Using C stack */
  LOG_INFO("Using standard C IPv6 stack\n");

  /* Add addresses using C API */
  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0001);
  if(uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL) != NULL) {
    LOG_INFO("✓ Link-local address added\n");
  }

  uip_ip6addr(&ipaddr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0001);
  if(uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF) != NULL) {
    LOG_INFO("✓ Global address added\n");
  }
#endif

  LOG_INFO("\n");
  LOG_INFO("Demo application running.\n");
  LOG_INFO("The application will print status every 10 seconds.\n");
  LOG_INFO("\n");

  /* Set periodic timer */
  etimer_set(&timer, CLOCK_SECOND * 10);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    LOG_INFO("--- Status Update ---\n");

#if UIP6_RUST_ENABLE
    uip6_rust_glue_get_stats();
#else
    LOG_INFO("C IPv6 stack running\n");
#endif

    etimer_reset(&timer);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
