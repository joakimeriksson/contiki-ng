/*
 * Copyright (c) 2025, Contiki-NG ESP32-C6 Port
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *      ESP32-C6 IEEE 802.15.4 radio driver implementation
 */

#include "contiki.h"
#include "dev/radio.h"
#include "esp32c6-radio.h"
#include "net/packetbuf.h"
#include "net/netstack.h"

#include <stdint.h>
#include <string.h>

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "ESP32C6-Radio"
#define LOG_LEVEL LOG_LEVEL_DBG

/*---------------------------------------------------------------------------*/
/* ESP32-C6 IEEE 802.15.4 MAC registers (simplified) */
#define IEEE802154_BASE         0x60080000
#define IEEE802154_CONF_REG     (IEEE802154_BASE + 0x0000)
#define IEEE802154_STATUS_REG   (IEEE802154_BASE + 0x0004)
#define IEEE802154_CMD_REG      (IEEE802154_BASE + 0x0008)
#define IEEE802154_CHANNEL_REG  (IEEE802154_BASE + 0x000C)
#define IEEE802154_TXPOWER_REG  (IEEE802154_BASE + 0x0010)
#define IEEE802154_PANID_REG    (IEEE802154_BASE + 0x0014)
#define IEEE802154_SHORTADDR_REG (IEEE802154_BASE + 0x0018)

/*---------------------------------------------------------------------------*/
/* Radio state */
static uint8_t radio_on = 0;
static uint8_t radio_channel = 26;
static int8_t radio_txpower = 5;
static uint16_t radio_pan_id = 0xABCD;
static uint16_t radio_short_addr = 0;

/* RX buffer */
#define RX_BUF_SIZE 128
static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t rx_buf_len = 0;

/*---------------------------------------------------------------------------*/
/* Forward declarations */
static int esp32c6_radio_init(void);
static int esp32c6_radio_prepare(const void *payload, unsigned short payload_len);
static int esp32c6_radio_transmit(unsigned short transmit_len);
static int esp32c6_radio_send(const void *payload, unsigned short payload_len);
static int esp32c6_radio_read(void *buf, unsigned short buf_len);
static int esp32c6_radio_channel_clear(void);
static int esp32c6_radio_receiving_packet(void);
static int esp32c6_radio_pending_packet(void);
static int esp32c6_radio_on(void);
static int esp32c6_radio_off(void);
static radio_result_t esp32c6_radio_get_value(radio_param_t param, radio_value_t *value);
static radio_result_t esp32c6_radio_set_value(radio_param_t param, radio_value_t value);
static radio_result_t esp32c6_radio_get_object(radio_param_t param, void *dest, size_t size);
static radio_result_t esp32c6_radio_set_object(radio_param_t param, const void *src, size_t size);

/*---------------------------------------------------------------------------*/
/* Radio driver instance */
const struct radio_driver esp32c6_radio_driver = {
  esp32c6_radio_init,
  esp32c6_radio_prepare,
  esp32c6_radio_transmit,
  esp32c6_radio_send,
  esp32c6_radio_read,
  esp32c6_radio_channel_clear,
  esp32c6_radio_receiving_packet,
  esp32c6_radio_pending_packet,
  esp32c6_radio_on,
  esp32c6_radio_off,
  esp32c6_radio_get_value,
  esp32c6_radio_set_value,
  esp32c6_radio_get_object,
  esp32c6_radio_set_object
};
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_init(void)
{
  LOG_INFO("Initializing ESP32-C6 IEEE 802.15.4 radio\n");

  /* Initialize radio hardware
   * In a real implementation:
   * 1. Enable IEEE 802.15.4 peripheral clock
   * 2. Reset radio
   * 3. Configure default parameters
   * 4. Set up interrupts
   * 5. Calibrate radio
   */

  radio_on = 0;
  radio_channel = 26;
  radio_txpower = 5;
  radio_pan_id = 0xABCD;

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_prepare(const void *payload, unsigned short payload_len)
{
  if(payload_len > 127) {
    LOG_ERR("Payload too large: %u\n", payload_len);
    return RADIO_TX_ERR;
  }

  /* Copy payload to TX buffer
   * In real implementation:
   * - Write to radio TX FIFO
   * - Set frame length
   */

  LOG_DBG("Prepared %u bytes for transmission\n", payload_len);
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_transmit(unsigned short transmit_len)
{
  if(!radio_on) {
    LOG_ERR("Radio is off, cannot transmit\n");
    return RADIO_TX_ERR;
  }

  /* Transmit the prepared packet
   * In real implementation:
   * 1. Trigger TX start
   * 2. Wait for TX complete interrupt
   * 3. Check if ACK was received (if requested)
   */

  LOG_DBG("Transmitting %u bytes on channel %u\n", transmit_len, radio_channel);

  /* Simulate transmission delay */
  /* In real hardware, this would be interrupt-driven */

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_send(const void *payload, unsigned short payload_len)
{
  int ret;

  ret = esp32c6_radio_prepare(payload, payload_len);
  if(ret != RADIO_TX_OK) {
    return ret;
  }

  return esp32c6_radio_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_read(void *buf, unsigned short buf_len)
{
  if(rx_buf_len == 0) {
    return 0;
  }

  if(buf_len < rx_buf_len) {
    LOG_ERR("Buffer too small: %u < %u\n", buf_len, rx_buf_len);
    return 0;
  }

  /* Copy received data to buffer */
  memcpy(buf, rx_buf, rx_buf_len);
  uint8_t len = rx_buf_len;

  /* Clear RX buffer */
  rx_buf_len = 0;

  LOG_DBG("Read %u bytes from radio\n", len);
  return len;
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_channel_clear(void)
{
  if(!radio_on) {
    return 1;
  }

  /* Check if channel is clear (CCA)
   * In real implementation:
   * - Perform CCA (Clear Channel Assessment)
   * - Return 1 if clear, 0 if busy
   */

  /* For now, assume channel is always clear */
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_receiving_packet(void)
{
  /* Check if currently receiving a packet
   * In real implementation:
   * - Check radio status register
   * - Return 1 if receiving, 0 otherwise
   */

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_pending_packet(void)
{
  /* Check if a packet is ready to be read */
  return (rx_buf_len > 0);
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_on(void)
{
  if(radio_on) {
    return 1;
  }

  LOG_INFO("Turning radio on, channel %u\n", radio_channel);

  /* Turn on radio
   * In real implementation:
   * 1. Power on radio
   * 2. Set channel
   * 3. Enter RX mode
   * 4. Enable RX interrupt
   */

  radio_on = 1;
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
esp32c6_radio_off(void)
{
  if(!radio_on) {
    return 1;
  }

  LOG_INFO("Turning radio off\n");

  /* Turn off radio
   * In real implementation:
   * 1. Disable interrupts
   * 2. Enter sleep mode
   * 3. Power down radio
   */

  radio_on = 0;
  return 1;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
esp32c6_radio_get_value(radio_param_t param, radio_value_t *value)
{
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    *value = radio_on ? RADIO_POWER_MODE_ON : RADIO_POWER_MODE_OFF;
    return RADIO_RESULT_OK;

  case RADIO_PARAM_CHANNEL:
    *value = radio_channel;
    return RADIO_RESULT_OK;

  case RADIO_PARAM_PAN_ID:
    *value = radio_pan_id;
    return RADIO_RESULT_OK;

  case RADIO_PARAM_16BIT_ADDR:
    *value = radio_short_addr;
    return RADIO_RESULT_OK;

  case RADIO_PARAM_TXPOWER:
    *value = radio_txpower;
    return RADIO_RESULT_OK;

  case RADIO_CONST_CHANNEL_MIN:
    *value = ESP32C6_RADIO_CHANNEL_MIN;
    return RADIO_RESULT_OK;

  case RADIO_CONST_CHANNEL_MAX:
    *value = ESP32C6_RADIO_CHANNEL_MAX;
    return RADIO_RESULT_OK;

  case RADIO_CONST_TXPOWER_MIN:
    *value = ESP32C6_RADIO_TXPOWER_MIN;
    return RADIO_RESULT_OK;

  case RADIO_CONST_TXPOWER_MAX:
    *value = ESP32C6_RADIO_TXPOWER_MAX;
    return RADIO_RESULT_OK;

  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
esp32c6_radio_set_value(radio_param_t param, radio_value_t value)
{
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      esp32c6_radio_on();
      return RADIO_RESULT_OK;
    } else if(value == RADIO_POWER_MODE_OFF) {
      esp32c6_radio_off();
      return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_INVALID_VALUE;

  case RADIO_PARAM_CHANNEL:
    if(value < ESP32C6_RADIO_CHANNEL_MIN || value > ESP32C6_RADIO_CHANNEL_MAX) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    radio_channel = (uint8_t)value;
    LOG_INFO("Set channel to %u\n", radio_channel);
    /* In real implementation: write to hardware register */
    return RADIO_RESULT_OK;

  case RADIO_PARAM_PAN_ID:
    radio_pan_id = (uint16_t)value;
    LOG_INFO("Set PAN ID to 0x%04X\n", radio_pan_id);
    /* In real implementation: write to hardware register */
    return RADIO_RESULT_OK;

  case RADIO_PARAM_16BIT_ADDR:
    radio_short_addr = (uint16_t)value;
    LOG_INFO("Set short address to 0x%04X\n", radio_short_addr);
    /* In real implementation: write to hardware register */
    return RADIO_RESULT_OK;

  case RADIO_PARAM_TXPOWER:
    if(value < ESP32C6_RADIO_TXPOWER_MIN || value > ESP32C6_RADIO_TXPOWER_MAX) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    radio_txpower = (int8_t)value;
    LOG_INFO("Set TX power to %d dBm\n", radio_txpower);
    /* In real implementation: write to hardware register */
    return RADIO_RESULT_OK;

  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
esp32c6_radio_get_object(radio_param_t param, void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
esp32c6_radio_set_object(radio_param_t param, const void *src, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Radio RX interrupt handler
 *
 * This would be called by the actual ISR when a packet is received
 */
void
esp32c6_radio_rx_isr(void)
{
  /* Read packet from hardware RX buffer
   * In real implementation:
   * 1. Read frame length
   * 2. Read frame data
   * 3. Read RSSI and LQI
   * 4. Clear interrupt flag
   */

  /* For now, this is a stub */
  if(rx_buf_len == 0) {
    /* Simulate received packet notification */
    /* NETSTACK_MAC.input(); */
  }
}
/*---------------------------------------------------------------------------*/
