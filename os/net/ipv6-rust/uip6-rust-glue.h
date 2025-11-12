/**
 * \file uip6-rust-glue.h
 * \brief Glue layer between Contiki-NG and Rust IPv6 stack
 */

#ifndef UIP6_RUST_GLUE_H_
#define UIP6_RUST_GLUE_H_

#include "net/ipv6/uip.h"

/**
 * \brief Initialize the Rust IPv6 stack glue layer
 *
 * Call this during system initialization to set up the Rust stack.
 */
void uip6_rust_glue_init(void);

/**
 * \brief Process incoming packet with Rust stack
 *
 * \param buf Pointer to packet buffer
 * \param len Length of packet in bytes
 * \return 0 on success, negative on error
 */
int uip6_rust_glue_input(uint8_t *buf, uint16_t len);

/**
 * \brief Synchronize addresses from C stack to Rust stack
 *
 * Copies all configured addresses from the C uip_ds6 to Rust.
 */
void uip6_rust_glue_sync_addresses(void);

/**
 * \brief Hook called when address is added
 *
 * \param addr The IPv6 address
 * \param type Address type (0=autoconf, 1=DHCP, 2=manual)
 */
void uip6_rust_glue_addr_add_hook(const uip_ipaddr_t *addr, uint8_t type);

/**
 * \brief Hook called when address is removed
 *
 * \param addr The IPv6 address
 */
void uip6_rust_glue_addr_rm_hook(const uip_ipaddr_t *addr);

/**
 * \brief Get and print Rust stack statistics
 */
void uip6_rust_glue_get_stats(void);

#endif /* UIP6_RUST_GLUE_H_ */
