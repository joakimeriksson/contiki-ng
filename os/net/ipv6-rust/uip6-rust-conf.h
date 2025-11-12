/**
 * \file uip6-rust-conf.h
 * \brief Configuration for Rust-based IPv6 stack
 *
 * This file provides configuration options for enabling and using the
 * Rust-based IPv6 stack as a replacement for the C implementation.
 */

#ifndef UIP6_RUST_CONF_H_
#define UIP6_RUST_CONF_H_

/*---------------------------------------------------------------------------*/
/* Configuration Options */
/*---------------------------------------------------------------------------*/

/**
 * \brief Enable Rust-based IPv6 stack
 *
 * Set to 1 to use the Rust implementation instead of the C implementation.
 * This can be set in project-conf.h or as a compiler flag.
 *
 * Example in project-conf.h:
 *   #define UIP6_RUST_CONF_ENABLE 1
 *
 * Example as compiler flag:
 *   CFLAGS += -DUIP6_RUST_CONF_ENABLE=1
 */
#ifdef UIP6_RUST_CONF_ENABLE
#define UIP6_RUST_ENABLE UIP6_RUST_CONF_ENABLE
#else
#define UIP6_RUST_ENABLE 0
#endif

/**
 * \brief Hybrid mode: Use both C and Rust stacks
 *
 * When enabled, both stacks run in parallel for testing/validation.
 * The Rust stack processes packets first, then falls back to C if needed.
 *
 * This is useful for:
 * - Validating Rust implementation against C
 * - Gradual migration
 * - A/B testing
 */
#ifdef UIP6_RUST_CONF_HYBRID_MODE
#define UIP6_RUST_HYBRID_MODE UIP6_RUST_CONF_HYBRID_MODE
#else
#define UIP6_RUST_HYBRID_MODE 0
#endif

/**
 * \brief Enable Rust stack logging
 *
 * When enabled, the Rust stack will log debug information.
 */
#ifdef UIP6_RUST_CONF_LOG_LEVEL
#define UIP6_RUST_LOG_LEVEL UIP6_RUST_CONF_LOG_LEVEL
#else
#define UIP6_RUST_LOG_LEVEL LOG_LEVEL_NONE
#endif

/*---------------------------------------------------------------------------*/
/* Validation */
/*---------------------------------------------------------------------------*/

#if UIP6_RUST_ENABLE && !UIP_CONF_IPV6
#error "UIP6_RUST requires UIP_CONF_IPV6 to be enabled"
#endif

#if UIP6_RUST_HYBRID_MODE && !UIP6_RUST_ENABLE
#error "UIP6_RUST_HYBRID_MODE requires UIP6_RUST_ENABLE to be set"
#endif

/*---------------------------------------------------------------------------*/
/* Auto-configuration */
/*---------------------------------------------------------------------------*/

#if UIP6_RUST_ENABLE
/* When Rust stack is enabled, we can optionally disable some C components */

/* You can uncomment these to disable C components when using Rust only */
/* #define UIP_CONF_ND6_SEND_NA 0 */
/* #define UIP_CONF_ND6_SEND_NS 0 */
/* #define UIP_CONF_ND6_SEND_RA 0 */

#endif /* UIP6_RUST_ENABLE */

#endif /* UIP6_RUST_CONF_H_ */
