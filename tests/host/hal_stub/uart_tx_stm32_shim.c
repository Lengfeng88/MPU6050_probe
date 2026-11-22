/* Host replacement for App/uart_tx/uart_tx_stm32.c in the main.c glue test.
 * The glue test compares UART bytes and the HAL call trace with the pre-refactor firmware, so
 * the shim hands every frame straight to the mock HAL_UART_Transmit with the old timeout.
 * The queue / DMA logic itself is tested in test_units.c (uart_tx). */
#include "uart_tx/uart_tx_stm32.h"

#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart1;

uart_tx_t g_uart_tx;

void uart_tx_stm32_init(void) {}

bool uart_tx_stm32_submit(const uint8_t *frame, uint16_t len)
{
    return HAL_UART_Transmit(&huart1, frame, len, 50) == HAL_OK;
}
