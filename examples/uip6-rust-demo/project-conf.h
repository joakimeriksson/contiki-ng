/**
 * \file project-conf.h
 * \brief Project configuration for uIP6-Rust demo
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*---------------------------------------------------------------------------*/
/* IPv6 Configuration */
/*---------------------------------------------------------------------------*/

/* Enable IPv6 */
#define UIP_CONF_IPV6 1

/* Buffer size (must be at least 1280 for IPv6) */
#define UIP_CONF_BUFFER_SIZE 1280

/*---------------------------------------------------------------------------*/
/* Rust Stack Configuration */
/*---------------------------------------------------------------------------*/

/*
 * Option 1: Enable Rust stack (full replacement)
 * Uncomment to use Rust instead of C
 */
// #define UIP6_RUST_CONF_ENABLE 1

/*
 * Option 2: Hybrid mode (both C and Rust)
 * Uncomment both lines to run both stacks in parallel
 */
// #define UIP6_RUST_CONF_ENABLE 1
// #define UIP6_RUST_CONF_HYBRID_MODE 1

/*
 * Option 3: Enable Rust logging
 * Uncomment to see debug output from Rust stack
 */
// #define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_DBG

/*
 * Note: UIP6_RUST_CONF_ENABLE can also be set via Makefile:
 *   make UIP6_RUST_CONF_ENABLE=1 TARGET=native
 *   make UIP6_RUST_CONF_ENABLE=0 TARGET=native
 */

/*---------------------------------------------------------------------------*/
/* Network Configuration */
/*---------------------------------------------------------------------------*/

/* Neighbor table size */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 8

/* Route table size */
#define UIP_CONF_MAX_ROUTES 8

/* Logging */
#define LOG_CONF_LEVEL_IPV6 LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_TCPIP LOG_LEVEL_INFO

#endif /* PROJECT_CONF_H_ */
