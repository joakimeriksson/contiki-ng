/*
 * nRF54L15 compatibility wrapper for nrf.h
 *
 * The main nrfx mdk/nrf.h does not yet support nRF54L15,
 * so we provide this wrapper to include the nRF54L15-specific headers.
 */

#ifndef NRF_H
#define NRF_H

#include "nrf54l15.h"
#include "system_nrf54l.h"

#endif /* NRF_H */
