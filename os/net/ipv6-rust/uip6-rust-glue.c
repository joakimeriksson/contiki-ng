/**
 * \file uip6-rust-glue.c
 * \brief Glue layer between Contiki-NG and Rust IPv6 stack
 *
 * This file provides the integration layer that allows the Rust IPv6 stack
 * to be used as a drop-in replacement for the C stack, or in hybrid mode
 * where both stacks run in parallel.
 */

#include "contiki.h"
#include "uip6-rust-conf.h"

#if UIP6_RUST_ENABLE

#include "uip6-rust.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/tcpip.h"
#include "net/netstack.h"
#include "sys/log.h"

#define LOG_MODULE "Rust-Glue"
#define LOG_LEVEL UIP6_RUST_LOG_LEVEL

/*---------------------------------------------------------------------------*/
/* State tracking */
/*---------------------------------------------------------------------------*/

static uint8_t rust_initialized = 0;

/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize the Rust IPv6 stack
 *
 * This is called during system initialization to set up the Rust stack.
 */
void
uip6_rust_glue_init(void)
{
  if(rust_initialized) {
    LOG_WARN("Rust stack already initialized\n");
    return;
  }

  LOG_INFO("Initializing Rust IPv6 stack\n");

  /* Initialize the Rust stack */
  uip6_rust_init();

  rust_initialized = 1;

  /* Print version information */
  const uint8_t *version = uip6_rust_version();
  LOG_INFO("Rust IPv6 stack initialized: %s\n", version);

#if UIP6_RUST_HYBRID_MODE
  LOG_INFO("Running in HYBRID mode (C + Rust)\n");
#else
  LOG_INFO("Running in RUST-ONLY mode\n");
#endif
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Process incoming packet with Rust stack
 *
 * This function is called to process an incoming IPv6 packet.
 * In hybrid mode, it tries Rust first, then falls back to C.
 * In Rust-only mode, it only uses Rust.
 *
 * \param buf Pointer to packet buffer
 * \param len Length of packet
 * \return 0 on success, negative on error
 */
int
uip6_rust_glue_input(uint8_t *buf, uint16_t len)
{
  int result;

  if(!rust_initialized) {
    LOG_WARN("Rust stack not initialized, initializing now\n");
    uip6_rust_glue_init();
  }

  LOG_DBG("Processing packet (%u bytes) with Rust stack\n", len);

  /* Process with Rust stack */
  result = uip6_rust_input(buf, len);

#if UIP6_RUST_HYBRID_MODE
  if(result < 0) {
    LOG_DBG("Rust processing failed, falling back to C stack\n");
    /* Fall back to C stack */
    memcpy(uip_buf, buf, len);
    uip_len = len;
    tcpip_input();
    return 0;
  }
#endif

  if(result < 0) {
    LOG_WARN("Rust stack failed to process packet\n");
  }

  return result;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Synchronize addresses from C stack to Rust stack
 *
 * This function copies addresses from the C uip_ds6 to the Rust stack.
 * Useful when running in hybrid mode or during migration.
 */
void
uip6_rust_glue_sync_addresses(void)
{
  uip_ds6_addr_t *addr;
  int i;

  if(!rust_initialized) {
    return;
  }

  LOG_INFO("Synchronizing addresses to Rust stack\n");

  /* Sync all configured addresses */
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    addr = &uip_ds6_if.addr_list[i];
    if(addr->isused) {
      uint8_t addr_type = 2; /* Manual by default */

      if(addr->type == ADDR_AUTOCONF) {
        addr_type = 0;
      } else if(addr->type == ADDR_DHCP) {
        addr_type = 1;
      }

      LOG_DBG("Syncing address to Rust\n");
      uip6_rust_addr_add(&addr->ipaddr, addr_type);
    }
  }
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Hook for address addition
 *
 * When an address is added to the C stack, also add it to Rust.
 */
void
uip6_rust_glue_addr_add_hook(const uip_ipaddr_t *addr, uint8_t type)
{
  if(!rust_initialized || !UIP6_RUST_ENABLE) {
    return;
  }

  LOG_DBG("Adding address to Rust stack\n");
  uip6_rust_addr_add(addr, type);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Hook for address removal
 *
 * When an address is removed from the C stack, also remove from Rust.
 */
void
uip6_rust_glue_addr_rm_hook(const uip_ipaddr_t *addr)
{
  if(!rust_initialized || !UIP6_RUST_ENABLE) {
    return;
  }

  LOG_DBG("Removing address from Rust stack\n");
  uip6_rust_addr_rm(addr);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Get statistics from Rust stack
 *
 * Returns information about the Rust stack status.
 */
void
uip6_rust_glue_get_stats(void)
{
  if(!rust_initialized) {
    LOG_INFO("Rust stack not initialized\n");
    return;
  }

  const uint8_t *version = uip6_rust_version();
  LOG_INFO("Rust IPv6 Stack Status:\n");
  LOG_INFO("  Version: %s\n", version);
  LOG_INFO("  Initialized: yes\n");
#if UIP6_RUST_HYBRID_MODE
  LOG_INFO("  Mode: Hybrid (C + Rust)\n");
#else
  LOG_INFO("  Mode: Rust only\n");
#endif
}

#else /* UIP6_RUST_ENABLE */

/* Stub functions when Rust is disabled */
void uip6_rust_glue_init(void) { }
int uip6_rust_glue_input(uint8_t *buf, uint16_t len) { return -1; }
void uip6_rust_glue_sync_addresses(void) { }
void uip6_rust_glue_addr_add_hook(const uip_ipaddr_t *addr, uint8_t type) { }
void uip6_rust_glue_addr_rm_hook(const uip_ipaddr_t *addr) { }
void uip6_rust_glue_get_stats(void) { }

#endif /* UIP6_RUST_ENABLE */
