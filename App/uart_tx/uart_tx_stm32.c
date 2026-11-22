/* STM32H7 glue for uart_tx.c: DMA start, BASEPRI lock, IRQ handlers.  Target build only
 * (the host tests replace this file with tests/host/hal_stub/uart_tx_stm32_shim.c). */
#include "uart_tx/uart_tx_stm32.h"

#include "main.h"
#include "usart.h"

__attribute__((section(".dma_buf"), aligned(32)))
static uint8_t s_slots[UART_TX_SLOTS * UART_TX_SLOT_SIZE];

uart_tx_t g_uart_tx;

static bool port_start(const uint8_t *buf, uint16_t len)
{
    return HAL_UART_Transmit_DMA(&huart1, buf, len) == HAL_OK;
}

/* Raise BASEPRI so that priorities >= UART_TX_IRQ_PRIO are masked; EXTI3 (0) still runs. */
static uint32_t port_lock(void)
{
    uint32_t old = __get_BASEPRI();
    uint32_t lvl = UART_TX_IRQ_PRIO << (8u - __NVIC_PRIO_BITS);
    if (old == 0u || old > lvl)
        __set_BASEPRI(lvl);
    __ISB();
    return old;
}

static void port_unlock(uint32_t token)
{
    __set_BASEPRI(token);
}

void uart_tx_stm32_init(void)
{
    static const uart_tx_port_t port = { port_start, port_lock, port_unlock };
    uart_tx_init(&g_uart_tx, s_slots, &port);
}

bool uart_tx_stm32_submit(const uint8_t *frame, uint16_t len)
{
    return uart_tx_submit(&g_uart_tx, frame, len);
}

/* ---- interrupt plumbing (CubeMX did not generate these: DMA is configured in usart.c) ---- */

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(huart1.hdmatx);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        uart_tx_on_complete(&g_uart_tx);
}

/* The same callback also reports RX errors (framing / overrun once the Pi's TX line is wired
 * for the return channel).  Those say nothing about the TX frame in flight; only a DMA error
 * does, so only that abandons a frame. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1 && (huart->ErrorCode & HAL_UART_ERROR_DMA) != 0u)
        uart_tx_on_error(&g_uart_tx);
}
