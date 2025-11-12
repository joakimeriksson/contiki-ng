/**
 * \file example-usage.c
 * \brief Example usage of the uIP6-Rust IPv6 stack
 *
 * This file demonstrates how to use the Rust-based IPv6 stack in a
 * Contiki-NG application.
 */

#include "contiki.h"
#include "uip6-rust.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-ds6.h"
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
PROCESS(rust_ipv6_example, "Rust IPv6 Example");
AUTOSTART_PROCESSES(&rust_ipv6_example);
/*---------------------------------------------------------------------------*/

/**
 * \brief Print an IPv6 address in human-readable format
 */
static void
print_ipv6_addr(const uip_ip6addr_t *addr)
{
  uint16_t a;
  int i, f;

  for(i = 0, f = 0; i < sizeof(uip_ip6addr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        printf("::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        printf(":");
      }
      printf("%x", a);
    }
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rust_ipv6_example, ev, data)
{
  static struct etimer timer;
  uip_ip6addr_t addr;
  uip_ip6addr_t src_addr;
  linkaddr_t lladdr;
  int result;

  PROCESS_BEGIN();

  printf("\n=== uIP6-Rust Example ===\n");

  /* Get version information */
  const uint8_t *version = uip6_rust_version();
  printf("Version: %s\n", version);

  /* Initialize the Rust IPv6 stack */
  printf("\nInitializing uIP6-Rust...\n");
  uip6_rust_init();
  printf("Initialization complete.\n");

  /* Example 1: Add a link-local address */
  printf("\n--- Example 1: Add Link-Local Address ---\n");
  uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0001);
  printf("Adding address: ");
  print_ipv6_addr(&addr);
  printf("\n");

  result = uip6_rust_addr_add(&addr, 2); /* Manual address */
  if(result == 0) {
    printf("✓ Address added successfully\n");
  } else {
    printf("✗ Failed to add address\n");
  }

  /* Example 2: Look up an address */
  printf("\n--- Example 2: Address Lookup ---\n");
  printf("Looking up address: ");
  print_ipv6_addr(&addr);
  printf("\n");

  result = uip6_rust_addr_lookup(&addr);
  if(result == 1) {
    printf("✓ Address found\n");
  } else {
    printf("✗ Address not found\n");
  }

  /* Example 3: Add a global address */
  printf("\n--- Example 3: Add Global Address ---\n");
  uip_ip6addr(&addr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0001);
  printf("Adding address: ");
  print_ipv6_addr(&addr);
  printf("\n");

  result = uip6_rust_addr_add(&addr, 0); /* Autoconf address */
  if(result == 0) {
    printf("✓ Address added successfully\n");
  } else {
    printf("✗ Failed to add address\n");
  }

  /* Example 4: Source address selection */
  printf("\n--- Example 4: Source Address Selection ---\n");
  uip_ip6addr(&addr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0002);
  printf("Destination: ");
  print_ipv6_addr(&addr);
  printf("\n");

  result = uip6_rust_select_src(&addr, &src_addr);
  if(result == 0) {
    printf("✓ Selected source: ");
    print_ipv6_addr(&src_addr);
    printf("\n");
  } else {
    printf("✗ No suitable source address found\n");
  }

  /* Example 5: Add a neighbor */
  printf("\n--- Example 5: Add Neighbor ---\n");
  uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0002);
  memset(&lladdr, 0, sizeof(lladdr));
  lladdr.u8[0] = 0x02;
  lladdr.u8[7] = 0x02;

  printf("Adding neighbor: ");
  print_ipv6_addr(&addr);
  printf("\n");
  printf("Link-layer address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         lladdr.u8[0], lladdr.u8[1], lladdr.u8[2], lladdr.u8[3],
         lladdr.u8[4], lladdr.u8[5], lladdr.u8[6], lladdr.u8[7]);

  result = uip6_rust_nbr_add(&addr, &lladdr, 0); /* Not a router */
  if(result == 0) {
    printf("✓ Neighbor added successfully\n");
  } else {
    printf("✗ Failed to add neighbor\n");
  }

  /* Example 6: Look up a neighbor */
  printf("\n--- Example 6: Neighbor Lookup ---\n");
  printf("Looking up neighbor: ");
  print_ipv6_addr(&addr);
  printf("\n");

  linkaddr_t found_lladdr;
  result = uip6_rust_nbr_lookup(&addr, &found_lladdr);
  if(result == 0) {
    printf("✓ Neighbor found\n");
    printf("Link-layer address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           found_lladdr.u8[0], found_lladdr.u8[1], found_lladdr.u8[2], found_lladdr.u8[3],
           found_lladdr.u8[4], found_lladdr.u8[5], found_lladdr.u8[6], found_lladdr.u8[7]);
  } else {
    printf("✗ Neighbor not found\n");
  }

  /* Example 7: Address type checks */
  printf("\n--- Example 7: Address Type Checks ---\n");

  /* Test multicast address */
  uip_ip6addr(&addr, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);
  printf("Address: ");
  print_ipv6_addr(&addr);
  printf("\n");
  printf("  Multicast: %s\n", uip6_rust_is_multicast(&addr) ? "yes" : "no");
  printf("  Link-local: %s\n", uip6_rust_is_link_local(&addr) ? "yes" : "no");

  /* Test link-local address */
  uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0, 0, 0, 0x0001);
  printf("\nAddress: ");
  print_ipv6_addr(&addr);
  printf("\n");
  printf("  Multicast: %s\n", uip6_rust_is_multicast(&addr) ? "yes" : "no");
  printf("  Link-local: %s\n", uip6_rust_is_link_local(&addr) ? "yes" : "no");

  /* Test global address */
  uip_ip6addr(&addr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0001);
  printf("\nAddress: ");
  print_ipv6_addr(&addr);
  printf("\n");
  printf("  Multicast: %s\n", uip6_rust_is_multicast(&addr) ? "yes" : "no");
  printf("  Link-local: %s\n", uip6_rust_is_link_local(&addr) ? "yes" : "no");

  printf("\n=== Example Complete ===\n\n");

  /* Set a periodic timer for demonstration */
  etimer_set(&timer, CLOCK_SECOND * 30);

  while(1) {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_TIMER && data == &timer) {
      printf("Periodic check: Stack is running\n");
      etimer_reset(&timer);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
