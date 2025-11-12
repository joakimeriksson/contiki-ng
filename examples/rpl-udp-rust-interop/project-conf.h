/**
 * \file project-conf.h
 * \brief Project configuration for RPL-UDP Rust Interoperability Demo
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*---------------------------------------------------------------------------*/
/* Network Configuration */
/*---------------------------------------------------------------------------*/

/* Enable IPv6 */
#define UIP_CONF_IPV6 1

/* Buffer size */
#define UIP_CONF_BUFFER_SIZE 1280

/* Neighbor and route table sizes */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 16
#define UIP_CONF_MAX_ROUTES 16

/*---------------------------------------------------------------------------*/
/* Rust Stack Configuration for Server */
/*---------------------------------------------------------------------------*/

/*
 * Note: UIP6_RUST_CONF_ENABLE can be set via:
 * 1. Makefile: make UIP6_RUST_CONF_ENABLE=1 udp-server.native
 * 2. Compiler flag: CFLAGS+=-DUIP6_RUST_CONF_ENABLE=1
 * 3. This file (if not already defined)
 */

#ifndef UIP6_RUST_CONF_ENABLE
  /* Default: Don't enable Rust (use C) */
  #define UIP6_RUST_CONF_ENABLE 0
#endif

#if UIP6_RUST_CONF_ENABLE
  /* Optional: Enable Rust logging for debugging */
  #ifndef UIP6_RUST_CONF_LOG_LEVEL
    #define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_INFO
  #endif

  /* Optional: Use hybrid mode for testing */
  // #define UIP6_RUST_CONF_HYBRID_MODE 1
#endif

/*---------------------------------------------------------------------------*/
/* RPL Configuration */
/*---------------------------------------------------------------------------*/

/* Use RPL-Lite */
#define ROUTING_CONF_RPL_LITE 1

/* RPL DAO timer */
#define RPL_CONF_DAO_DELAY (CLOCK_SECOND * 10)

/*---------------------------------------------------------------------------*/
/* MAC Configuration */
/*---------------------------------------------------------------------------*/

/* Use CSMA for simplicity and compatibility */
/* Note: Can also use TSCH if needed */
#define NETSTACK_CONF_MAC csma_driver

/*---------------------------------------------------------------------------*/
/* Logging Configuration */
/*---------------------------------------------------------------------------*/

#define LOG_CONF_LEVEL_RPL LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_TCPIP LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_IPV6 LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_MAC LOG_LEVEL_INFO

/*---------------------------------------------------------------------------*/
/* Application Configuration */
/*---------------------------------------------------------------------------*/

/* Enable server replies */
#define WITH_SERVER_REPLY 1

/* Send interval for clients (in seconds) */
#define SEND_INTERVAL (10 * CLOCK_SECOND)

#endif /* PROJECT_CONF_H_ */
