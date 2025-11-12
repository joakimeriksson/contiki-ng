/**
 * \file project-conf-example.h
 * \brief Example project configuration for using Rust IPv6 stack
 *
 * Copy the desired configuration sections to your project-conf.h file
 * to enable the Rust-based IPv6 stack.
 */

#ifndef PROJECT_CONF_EXAMPLE_H_
#define PROJECT_CONF_EXAMPLE_H_

/*---------------------------------------------------------------------------*/
/* Example 1: Enable Rust IPv6 stack (full replacement) */
/*---------------------------------------------------------------------------*/

/* Uncomment to use Rust stack instead of C */
// #define UIP6_RUST_CONF_ENABLE 1

/* Optional: Enable debug logging from Rust stack */
// #define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_DBG

/*---------------------------------------------------------------------------*/
/* Example 2: Hybrid mode (both C and Rust) */
/*---------------------------------------------------------------------------*/

/* Uncomment to run both stacks for testing/validation */
// #define UIP6_RUST_CONF_ENABLE 1
// #define UIP6_RUST_CONF_HYBRID_MODE 1

/*---------------------------------------------------------------------------*/
/* Example 3: Platform-specific configuration */
/*---------------------------------------------------------------------------*/

/* Use Rust on native platform, C on others */
// #ifdef CONTIKI_TARGET_NATIVE
// #define UIP6_RUST_CONF_ENABLE 1
// #else
// #define UIP6_RUST_CONF_ENABLE 0
// #endif

/*---------------------------------------------------------------------------*/
/* Required configuration for IPv6 */
/*---------------------------------------------------------------------------*/

/* Make sure IPv6 is enabled (required for Rust stack) */
#define UIP_CONF_IPV6 1

/* Buffer size must be at least 1280 (IPv6 minimum MTU) */
#define UIP_CONF_BUFFER_SIZE 1280

/* Configure neighbor and route tables */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 8
#define UIP_CONF_MAX_ROUTES 8

/*---------------------------------------------------------------------------*/
/* Optional: Disable C components when using Rust-only mode */
/*---------------------------------------------------------------------------*/

#if UIP6_RUST_CONF_ENABLE && !UIP6_RUST_CONF_HYBRID_MODE
/* When using Rust-only, you can optionally reduce C code size */
/* by disabling C implementations of features handled by Rust */

/* Note: Only do this after testing that Rust handles everything! */
// #define UIP_CONF_ND6_SEND_NA 0
// #define UIP_CONF_ND6_SEND_NS 0
#endif

#endif /* PROJECT_CONF_EXAMPLE_H_ */
