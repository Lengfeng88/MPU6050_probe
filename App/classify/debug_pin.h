#ifndef APP_CLASSIFY_DEBUG_PIN_H
#define APP_CLASSIFY_DEBUG_PIN_H

/*
 * One GPIO that is high while the classifier runs, so a scope or logic analyzer can measure its run time
 * independently of the DWT cycle counter.  Default PB0; change the two macros to move it.
 * On the host (tests) it compiles to nothing; define IMU_NO_DEBUG_PIN to switch it off on the board.
 *
 *   debug_pin_init();            once, after the clocks are up
 *   debug_pin_high(); work; debug_pin_low();
 */
#if defined(__arm__) && !defined(IMU_NO_DEBUG_PIN)
#include "main.h"

#ifndef DEBUG_PIN_PORT
#define DEBUG_PIN_PORT  GPIOB
#define DEBUG_PIN_MASK  GPIO_PIN_0
#endif

static inline void debug_pin_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();                 /* match DEBUG_PIN_PORT if you change it */
    HAL_GPIO_WritePin(DEBUG_PIN_PORT, DEBUG_PIN_MASK, GPIO_PIN_RESET);
    g.Pin = DEBUG_PIN_MASK;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(DEBUG_PIN_PORT, &g);
}

static inline void debug_pin_high(void) { DEBUG_PIN_PORT->BSRR = DEBUG_PIN_MASK; }
static inline void debug_pin_low(void)  { DEBUG_PIN_PORT->BSRR = (uint32_t)DEBUG_PIN_MASK << 16; }
#else
static inline void debug_pin_init(void) {}
static inline void debug_pin_high(void) {}
static inline void debug_pin_low(void) {}
#endif

#endif /* APP_CLASSIFY_DEBUG_PIN_H */
