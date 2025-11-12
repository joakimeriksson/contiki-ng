/**
 * \file uip6-rust.h
 * \brief Rust-based IPv6 stack for Contiki-NG
 *
 * This file provides the C interface to the Rust implementation of the
 * uIP6 IPv6 stack, offering improved memory safety and security.
 */

#ifndef UIP6_RUST_H
#define UIP6_RUST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*/
/* Type definitions */
/*---------------------------------------------------------------------------*/

/**
 * \brief IPv6 address structure (compatible with uip_ip6addr_t)
 */
typedef union {
    uint8_t  u8[16];
    uint16_t u16[8];
} uip_ip6addr_t;

/**
 * \brief Link-layer address structure
 */
typedef struct {
    uint8_t u8[8];
} linkaddr_t;

/*---------------------------------------------------------------------------*/
/* Core functions */
/*---------------------------------------------------------------------------*/

/**
 * \brief Initialize the Rust uIP6 stack
 *
 * This function must be called before any other uIP6-Rust functions.
 * It initializes the IPv6 data structures, neighbor cache, and ICMPv6.
 */
void uip6_rust_init(void);

/**
 * \brief Process an incoming IPv6 packet
 * \param buf Pointer to the packet buffer
 * \param len Length of the packet in bytes
 * \return 0 on success, negative value on error
 *
 * This function processes an incoming IPv6 packet, performing:
 * - IPv6 header validation
 * - Extension header processing
 * - Upper-layer protocol dispatch (ICMPv6, UDP, TCP)
 * - Neighbor Discovery processing
 */
int32_t uip6_rust_input(uint8_t *buf, uint16_t len);

/**
 * \brief Get the version string
 * \return Pointer to a null-terminated version string
 */
const uint8_t *uip6_rust_version(void);

/*---------------------------------------------------------------------------*/
/* Address management functions */
/*---------------------------------------------------------------------------*/

/**
 * \brief Add an IPv6 address to the interface
 * \param addr Pointer to the IPv6 address to add
 * \param addr_type Address type (0=autoconf, 1=DHCP, 2=manual)
 * \return 0 on success, -1 on error
 */
int32_t uip6_rust_addr_add(const uip_ip6addr_t *addr, uint8_t addr_type);

/**
 * \brief Look up an IPv6 address
 * \param addr Pointer to the IPv6 address to look up
 * \return 1 if address exists, 0 if not found
 */
int32_t uip6_rust_addr_lookup(const uip_ip6addr_t *addr);

/**
 * \brief Remove an IPv6 address from the interface
 * \param addr Pointer to the IPv6 address to remove
 * \return 0 on success, -1 on error
 */
int32_t uip6_rust_addr_rm(const uip_ip6addr_t *addr);

/**
 * \brief Select source address for a destination
 * \param dest Pointer to the destination IPv6 address
 * \param src_out Pointer to store the selected source address
 * \return 0 on success, -1 if no suitable source address found
 */
int32_t uip6_rust_select_src(const uip_ip6addr_t *dest, uip_ip6addr_t *src_out);

/*---------------------------------------------------------------------------*/
/* Neighbor cache functions */
/*---------------------------------------------------------------------------*/

/**
 * \brief Add or update a neighbor cache entry
 * \param ipaddr Pointer to the neighbor's IPv6 address
 * \param lladdr Pointer to the neighbor's link-layer address
 * \param is_router 1 if neighbor is a router, 0 otherwise
 * \return 0 on success, -1 on error
 */
int32_t uip6_rust_nbr_add(const uip_ip6addr_t *ipaddr,
                          const linkaddr_t *lladdr,
                          uint8_t is_router);

/**
 * \brief Look up a neighbor's link-layer address
 * \param ipaddr Pointer to the neighbor's IPv6 address
 * \param lladdr_out Pointer to store the link-layer address (can be NULL)
 * \return 0 if neighbor found, -1 if not found
 */
int32_t uip6_rust_nbr_lookup(const uip_ip6addr_t *ipaddr,
                             linkaddr_t *lladdr_out);

/*---------------------------------------------------------------------------*/
/* Utility functions */
/*---------------------------------------------------------------------------*/

/**
 * \brief Check if an address is multicast
 * \param addr Pointer to the IPv6 address to check
 * \return 1 if multicast, 0 otherwise
 */
int32_t uip6_rust_is_multicast(const uip_ip6addr_t *addr);

/**
 * \brief Check if an address is link-local
 * \param addr Pointer to the IPv6 address to check
 * \return 1 if link-local, 0 otherwise
 */
int32_t uip6_rust_is_link_local(const uip_ip6addr_t *addr);

#ifdef __cplusplus
}
#endif

#endif /* UIP6_RUST_H */
