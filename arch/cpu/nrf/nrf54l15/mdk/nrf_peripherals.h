/*
 * nRF54L15 peripheral availability header
 *
 * This is a minimal version for nRF54L15 support in Contiki-NG.
 * Based on Nordic's nrf_peripherals.h structure.
 */

#ifndef NRF_PERIPHERALS_H__
#define NRF_PERIPHERALS_H__

#if defined(NRF54L15_XXAA)
    #include "nrf54l15_application_peripherals.h"
#else
    #error "Device must be defined. Expected NRF54L15_XXAA."
#endif

#endif // NRF_PERIPHERALS_H__
