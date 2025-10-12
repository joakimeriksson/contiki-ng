/*
 * Minimal configuration header for the nRF54L15 Cortex-M33 application core.
 *
 * Mirrors the existing nRF52/nRF53 style so platform code can include a
 * CPU-specific configuration header if needed. Extend as drivers mature.
 */
#ifndef NRF54L15_CONF_H_
#define NRF54L15_CONF_H_

/* Disable networking until radio driver is fully ported */
#ifndef NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_WITH_IPV6 0
#endif

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 0
#endif

/* Disable watchdog until properly tested on nRF54L15 */
#ifndef WATCHDOG_CONF_ENABLE
#define WATCHDOG_CONF_ENABLE 0
#endif


#endif /* NRF54L15_CONF_H_ */
