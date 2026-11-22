#ifndef APP_UART_TX_UART_TX_STM32_H
#define APP_UART_TX_UART_TX_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "uart_tx/uart_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USART1 TX through DMA1 stream 0 (set up in usart.c, USER CODE blocks), driven by uart_tx.c.
 *
 * Interrupt priorities (NVIC priority group 4, lower number = more urgent):
 *     0   EXTI3   DATA_RDY timestamp  (never masked by this module)
 *     2   DMA1_Stream0, USART1        TX path  (UART_TX_IRQ_PRIO)
 *    15   SysTick
 *
 * The frame slots live in AXI SRAM (section .dma_buf): this project's linker script puts
 * .data / .bss / stack in DTCM, which DMA1 cannot reach.  The data cache is off at the moment;
 * if it is turned on, mark that region non-cacheable in the MPU (or clean it before each start).
 */
#define UART_TX_IRQ_PRIO 2u

void uart_tx_stm32_init(void);

/** Non-blocking.  Returns false if the frame was dropped (queue full); counted in g_uart_tx.dropped. */
bool uart_tx_stm32_submit(const uint8_t *frame, uint16_t len);

/** Counters (submitted / sent / dropped / start_fail / errors); read with a debugger. */
extern uart_tx_t g_uart_tx;

#ifdef __cplusplus
}
#endif

#endif /* APP_UART_TX_UART_TX_STM32_H */
