/**
 * \file uip6-rust-test.c
 * \brief Test application for the Rust IPv6 stack
 *
 * This application tests the uIP6-Rust implementation by:
 * - Initializing the stack
 * - Adding addresses
 * - Testing ICMPv6 echo (ping)
 * - Verifying neighbor discovery
 * - Processing test packets
 */

#include "contiki.h"
#include "contiki-net.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/netstack.h"
#include "uip6-rust.h"

#include "sys/log.h"
#define LOG_MODULE "Rust-Test"
#define LOG_LEVEL LOG_LEVEL_INFO

#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
PROCESS(uip6_rust_test_process, "uIP6-Rust Test Process");
AUTOSTART_PROCESSES(&uip6_rust_test_process);
/*---------------------------------------------------------------------------*/

/* Test counters */
static int tests_passed = 0;
static int tests_failed = 0;
static int tests_total = 0;

/* Test packet buffer */
static uint8_t test_packet[256];

/*---------------------------------------------------------------------------*/
/**
 * \brief Print test result
 */
static void
print_test_result(const char *test_name, int passed)
{
  tests_total++;
  if(passed) {
    tests_passed++;
    LOG_INFO("✓ PASS: %s\n", test_name);
  } else {
    tests_failed++;
    LOG_ERR("✗ FAIL: %s\n", test_name);
  }
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Print IPv6 address
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
/**
 * \brief Test 1: Stack initialization
 */
static void
test_init(void)
{
  LOG_INFO("Test 1: Stack Initialization\n");

  /* Initialize the Rust IPv6 stack */
  uip6_rust_init();

  /* Get version */
  const uint8_t *version = uip6_rust_version();
  LOG_INFO("Version: %s\n", version);

  print_test_result("Stack initialization", version != NULL);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Test 2: Address management
 */
static void
test_addresses(void)
{
  uip_ip6addr_t addr;
  int result;

  LOG_INFO("Test 2: Address Management\n");

  /* Test 2.1: Add link-local address */
  uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0001);
  LOG_INFO("  Adding link-local: ");
  print_ipv6_addr(&addr);
  LOG_INFO_("\n");

  result = uip6_rust_addr_add(&addr, 2); /* Manual address */
  print_test_result("Add link-local address", result == 0);

  /* Test 2.2: Lookup the address */
  result = uip6_rust_addr_lookup(&addr);
  print_test_result("Lookup link-local address", result == 1);

  /* Test 2.3: Add global address */
  uip_ip6addr(&addr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0001);
  LOG_INFO("  Adding global: ");
  print_ipv6_addr(&addr);
  LOG_INFO_("\n");

  result = uip6_rust_addr_add(&addr, 0); /* Autoconf address */
  print_test_result("Add global address", result == 0);

  /* Test 2.4: Lookup global address */
  result = uip6_rust_addr_lookup(&addr);
  print_test_result("Lookup global address", result == 1);

  /* Test 2.5: Source address selection */
  uip_ip6addr_t dest, src;
  uip_ip6addr(&dest, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0002);
  result = uip6_rust_select_src(&dest, &src);

  if(result == 0) {
    LOG_INFO("  Selected source: ");
    print_ipv6_addr(&src);
    LOG_INFO_("\n");
  }
  print_test_result("Source address selection", result == 0);

  /* Test 2.6: Remove address */
  uip_ip6addr(&addr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0001);
  result = uip6_rust_addr_rm(&addr);
  print_test_result("Remove address", result == 0);

  /* Test 2.7: Verify removal */
  result = uip6_rust_addr_lookup(&addr);
  print_test_result("Verify address removed", result == 0);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Test 3: Neighbor management
 */
static void
test_neighbors(void)
{
  uip_ip6addr_t ipaddr;
  linkaddr_t lladdr, found_lladdr;
  int result;

  LOG_INFO("Test 3: Neighbor Management\n");

  /* Test 3.1: Add neighbor */
  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0002);
  memset(&lladdr, 0, sizeof(lladdr));

#if LINKADDR_SIZE == 8
  lladdr.u8[0] = 0x02;
  lladdr.u8[7] = 0x02;
#elif LINKADDR_SIZE == 2
  lladdr.u8[0] = 0x02;
  lladdr.u8[1] = 0x02;
#endif

  LOG_INFO("  Adding neighbor: ");
  print_ipv6_addr(&ipaddr);
  LOG_INFO_(" -> ");
  for(int i = 0; i < LINKADDR_SIZE; i++) {
    LOG_INFO_("%02x%s", lladdr.u8[i], (i < LINKADDR_SIZE-1) ? ":" : "");
  }
  LOG_INFO_("\n");

  result = uip6_rust_nbr_add(&ipaddr, &lladdr, 0);
  print_test_result("Add neighbor", result == 0);

  /* Test 3.2: Lookup neighbor */
  result = uip6_rust_nbr_lookup(&ipaddr, &found_lladdr);
  print_test_result("Lookup neighbor", result == 0);

  /* Test 3.3: Verify link-layer address */
  if(result == 0) {
    int match = memcmp(&lladdr, &found_lladdr, sizeof(linkaddr_t)) == 0;
    print_test_result("Verify neighbor link-layer address", match);
  } else {
    print_test_result("Verify neighbor link-layer address", 0);
  }

  /* Test 3.4: Add router neighbor */
  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0003);
#if LINKADDR_SIZE == 8
  lladdr.u8[7] = 0x03;
#elif LINKADDR_SIZE == 2
  lladdr.u8[1] = 0x03;
#endif
  result = uip6_rust_nbr_add(&ipaddr, &lladdr, 1); /* Is router */
  print_test_result("Add router neighbor", result == 0);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Test 4: Address type checks
 */
static void
test_address_types(void)
{
  uip_ip6addr_t addr;
  int result;

  LOG_INFO("Test 4: Address Type Checks\n");

  /* Test 4.1: Multicast address */
  uip_ip6addr(&addr, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);
  result = uip6_rust_is_multicast(&addr);
  print_test_result("Multicast address detection", result == 1);

  /* Test 4.2: Not multicast */
  uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0, 0, 0, 0x0001);
  result = uip6_rust_is_multicast(&addr);
  print_test_result("Non-multicast address detection", result == 0);

  /* Test 4.3: Link-local address */
  result = uip6_rust_is_link_local(&addr);
  print_test_result("Link-local address detection", result == 1);

  /* Test 4.4: Not link-local */
  uip_ip6addr(&addr, 0x2001, 0x0db8, 0, 0, 0, 0, 0, 0x0001);
  result = uip6_rust_is_link_local(&addr);
  print_test_result("Non-link-local address detection", result == 0);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Build a test ICMPv6 Echo Request packet
 */
static uint16_t
build_echo_request(uint8_t *buf, const uip_ip6addr_t *src, const uip_ip6addr_t *dst)
{
  uint16_t offset = 0;

  /* IPv6 header */
  buf[offset++] = 0x60; /* Version 6, TC = 0 */
  buf[offset++] = 0x00; /* TC + Flow label */
  buf[offset++] = 0x00; /* Flow label */
  buf[offset++] = 0x00; /* Flow label */
  buf[offset++] = 0x00; /* Payload length MSB */
  buf[offset++] = 0x08; /* Payload length LSB (8 bytes for ICMP) */
  buf[offset++] = 58;   /* Next header: ICMPv6 */
  buf[offset++] = 64;   /* Hop limit */

  /* Source address */
  memcpy(&buf[offset], src, 16);
  offset += 16;

  /* Destination address */
  memcpy(&buf[offset], dst, 16);
  offset += 16;

  /* ICMPv6 Echo Request */
  buf[offset++] = 128;  /* Type: Echo Request */
  buf[offset++] = 0;    /* Code */
  buf[offset++] = 0;    /* Checksum (will be calculated) */
  buf[offset++] = 0;
  buf[offset++] = 0;    /* Identifier */
  buf[offset++] = 0x01;
  buf[offset++] = 0;    /* Sequence number */
  buf[offset++] = 0x01;

  /* Calculate ICMPv6 checksum */
  /* For simplicity, we'll let the stack handle it */
  /* In a real implementation, we'd calculate it here */

  return offset;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Test 5: Packet processing
 */
static void
test_packet_processing(void)
{
  uip_ip6addr_t src, dst;
  uint16_t len;
  int result;

  LOG_INFO("Test 5: Packet Processing\n");

  /* Set up addresses */
  uip_ip6addr(&src, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0002);
  uip_ip6addr(&dst, 0xfe80, 0, 0, 0, 0x0200, 0, 0, 0x0001);

  /* Add destination address to our interface */
  uip6_rust_addr_add(&dst, 2);

  /* Test 5.1: Build and process Echo Request */
  len = build_echo_request(test_packet, &src, &dst);
  LOG_INFO("  Processing ICMPv6 Echo Request (%d bytes)\n", len);

  result = uip6_rust_input(test_packet, len);
  print_test_result("Process ICMPv6 Echo Request", result == 0);

  /* Test 5.2: Invalid packet (too short) */
  result = uip6_rust_input(test_packet, 10);
  print_test_result("Reject too-short packet", result < 0);

  /* Test 5.3: NULL buffer */
  result = uip6_rust_input(NULL, 100);
  print_test_result("Reject NULL buffer", result < 0);

  /* Test 5.4: Zero length */
  result = uip6_rust_input(test_packet, 0);
  print_test_result("Reject zero-length packet", result < 0);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Print test summary
 */
static void
print_summary(void)
{
  LOG_INFO("\n");
  LOG_INFO("=== Test Summary ===\n");
  LOG_INFO("Total tests:  %d\n", tests_total);
  LOG_INFO("Passed:       %d\n", tests_passed);
  LOG_INFO("Failed:       %d\n", tests_failed);
  LOG_INFO("Success rate: %d%%\n", (tests_passed * 100) / tests_total);
  LOG_INFO("==================\n");

  if(tests_failed == 0) {
    LOG_INFO("✓ ALL TESTS PASSED\n");
  } else {
    LOG_ERR("✗ SOME TESTS FAILED\n");
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uip6_rust_test_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();

  LOG_INFO("\n");
  LOG_INFO("====================================\n");
  LOG_INFO("  uIP6-Rust Test Suite\n");
  LOG_INFO("====================================\n");
  LOG_INFO("\n");

  /* Wait a bit for the system to stabilize */
  etimer_set(&timer, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

  /* Run tests */
  test_init();
  test_addresses();
  test_neighbors();
  test_address_types();
  test_packet_processing();

  /* Print summary */
  print_summary();

  /* Keep process alive for simulator */
  LOG_INFO("\nTest complete. Process will continue running.\n");

  etimer_set(&timer, CLOCK_SECOND * 10);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    LOG_INFO("Test process still running (passed: %d, failed: %d)\n",
             tests_passed, tests_failed);
    etimer_reset(&timer);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
