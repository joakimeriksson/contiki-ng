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
 *      ESP32-C6 UART driver implementation
 */

#include "contiki.h"
#include "uart-arch.h"
#include <stdio.h>
#include <stdint.h>

/* ESP32-C6 UART0 registers (simplified memory-mapped access) */
#define UART0_BASE              0x60000000
#define UART_FIFO_REG(i)        (UART0_BASE + 0x0000)
#define UART_INT_ST_REG(i)      (UART0_BASE + 0x0008)
#define UART_INT_CLR_REG(i)     (UART0_BASE + 0x0010)
#define UART_CLKDIV_REG(i)      (UART0_BASE + 0x0014)
#define UART_STATUS_REG(i)      (UART0_BASE + 0x001C)
#define UART_CONF0_REG(i)       (UART0_BASE + 0x0020)
#define UART_CONF1_REG(i)       (UART0_BASE + 0x0024)

/* UART status bits */
#define UART_TXFIFO_CNT         0x3FF
#define UART_RXFIFO_CNT_M       0x3FF
#define UART_RXFIFO_CNT_S       0

/*---------------------------------------------------------------------------*/
static int (*input_handler)(unsigned char c) = NULL;

/*---------------------------------------------------------------------------*/
void
uart_arch_init(uint32_t baud_rate)
{
  /* Note: In a real implementation, this would configure the UART hardware
   * For now, we provide a stub that would integrate with ESP-IDF's UART driver
   * or direct register access. For compilation purposes, we keep it simple.
   */

  /* Configure UART parameters:
   * - Baud rate divider = (APB_CLK / baud_rate)
   * - 8 data bits, no parity, 1 stop bit
   * - Enable RX interrupt
   */

  /* This is a placeholder - actual implementation would:
   * 1. Configure GPIO pins for UART function
   * 2. Set baud rate divider
   * 3. Configure frame format
   * 4. Enable UART and interrupts
   */
}
/*---------------------------------------------------------------------------*/
void
uart_arch_write_byte(uint8_t c)
{
  /* Write byte to UART FIFO */
  /* In real implementation:
   * while (!uart_arch_tx_ready()) {}
   * *((volatile uint32_t *)UART_FIFO_REG(0)) = c;
   */

  /* For now, use standard output for debugging/simulation */
  putchar(c);
}
/*---------------------------------------------------------------------------*/
void
uart_arch_set_input(int (*input)(unsigned char c))
{
  input_handler = input;
}
/*---------------------------------------------------------------------------*/
int
uart_arch_tx_ready(void)
{
  /* Check if TX FIFO has space */
  /* In real implementation:
   * uint32_t status = *((volatile uint32_t *)UART_STATUS_REG(0));
   * return ((status >> 16) & UART_TXFIFO_CNT) < 128;
   */
  return 1;
}
/*---------------------------------------------------------------------------*/
/* UART interrupt handler (would be called by ISR) */
void
uart_rx_interrupt_handler(void)
{
  if(input_handler) {
    /* Read all available bytes from RX FIFO */
    /* In real implementation:
     * while (rx_fifo_not_empty) {
     *   uint8_t byte = *((volatile uint32_t *)UART_FIFO_REG(0)) & 0xFF;
     *   input_handler(byte);
     * }
     */
  }
}
/*---------------------------------------------------------------------------*/
