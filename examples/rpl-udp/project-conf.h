#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Logging - keep low for performance, use shell "log <module> 4" to enable debug */
#define LOG_CONF_LEVEL_RPL       LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_IPV6      LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_MAC       LOG_LEVEL_INFO  /* See retransmissions */
#define LOG_CONF_LEVEL_FRAMER    LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_RADIO     LOG_LEVEL_INFO  /* See TX/RX counts */

/* Zolertia Firefly: Use CC1200 Sub-GHz radio */
#define ZOUL_CONF_USE_CC1200_RADIO 1

/* SimpleLink: Increase ACK wait time for sub-GHz prop-mode (~10ms) */
#ifdef CONTIKI_TARGET_SIMPLELINK
#define CSMA_CONF_ACK_WAIT_TIME (RTIMER_SECOND / 100)
#endif

#endif /* PROJECT_CONF_H_ */
