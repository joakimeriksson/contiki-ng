/*
 * Minimal configuration header for the nRF54L15 Cortex-M33 application core.
 *
 * Mirrors the existing nRF52/nRF53 style so platform code can include a
 * CPU-specific configuration header if needed. Extend as drivers mature.
 */
#ifndef NRF54L15_CONF_H_
#define NRF54L15_CONF_H_

/* Enable IPv6 networking -- nrf_802154 radio driver is now integrated */
#ifndef NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_WITH_IPV6 1
#endif

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 4
#endif

/* Disable watchdog until properly tested on nRF54L15 */
#ifndef WATCHDOG_CONF_ENABLE
#define WATCHDOG_CONF_ENABLE 0
#endif

/* Enable extended HardFault handler with register dump. */
#ifndef NRF_CONF_HARDFAULT_HANDLER_EXTENDED
#define NRF_CONF_HARDFAULT_HANDLER_EXTENDED 1
#endif

#endif /* NRF54L15_CONF_H_ */
