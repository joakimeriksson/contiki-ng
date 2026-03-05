/*
 * nRF54L15 compatibility wrapper for nrf.h
 *
 * The main nrfx mdk/nrf.h does not yet support nRF54L15,
 * so we provide this wrapper to include the nRF54L15-specific headers.
 */

#ifndef NRF_H
#define NRF_H

/* Define series macro so nrf_802154 peripherals header picks the right variant. */
#ifndef NRF54L_SERIES
#define NRF54L_SERIES
#endif

#include "nrf54l15.h"
#include "system_nrf54l.h"

/* nRF54L15 MDK uses _SIZE suffix for channel counts; add aliases expected
   by the nrfx HAL layer. */
#ifndef EGU10_CH_NUM
#define EGU10_CH_NUM EGU10_CH_NUM_SIZE
#endif
#ifndef EGU20_CH_NUM
#define EGU20_CH_NUM EGU20_CH_NUM_SIZE
#endif

#endif /* NRF_H */
