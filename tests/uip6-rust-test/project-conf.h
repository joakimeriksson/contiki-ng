/*
 * Project configuration for uIP6-Rust test
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Enable IPv6 */
#define UIP_CONF_IPV6 1

/* Logging configuration */
#define LOG_CONF_LEVEL_IPV6 LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_TCPIP LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_MAC LOG_LEVEL_INFO

/* Increase buffer size for testing */
#define UIP_CONF_BUFFER_SIZE 1280

/* Enable neighbor table */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 8

/* Enable route table */
#define UIP_CONF_MAX_ROUTES 8

/* Network stack configuration */
#define NETSTACK_CONF_WITH_IPV6 1

#endif /* PROJECT_CONF_H_ */
