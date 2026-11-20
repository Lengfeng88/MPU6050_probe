#ifndef APP_MEASURE_M0_PINS_H
#define APP_MEASURE_M0_PINS_H

/*
 * M0 baseline instrumentation: two extra GPIOs for a logic analyzer.  Opt-in: nothing is compiled in
 * (and no pin is touched) unless IMU_M0_PINS is defined, and then YOU must say which pins are free on
 * your board; there is deliberately no default.
 *
 *   -DIMU_M0_PINS -DM0_ISR_PORT=GPIOx -DM0_ISR_PIN=GPIO_PIN_n    high from the start of the DATA_RDY
 *                                                                callback, low when it has queued the event
 *   -DM0_I2C_PORT=GPIOx -DM0_I2C_PIN=GPIO_PIN_n                  high while mpu6050_read_raw() runs
 *
 * The ISR pin goes high just before TIM2 is read, so (DRDY edge -> ISR pin edge) is the delay between the
 * real sensor event and the moment the timestamp is taken: the bias of ts_us.  It includes the NVIC entry,
 * HAL_GPIO_EXTI_IRQHandler and the callback prologue.
 *
 * Edges are single BSRR writes (a few cycles).  On the host (tests) everything compiles to nothing.
 */
#if defined(__arm__) && defined(IMU_M0_PINS)
#include "main.h"

#if !defined(M0_ISR_PORT) || !defined(M0_ISR_PIN) || !defined(M0_I2C_PORT) || !defined(M0_I2C_PIN)
#error "IMU_M0_PINS needs M0_ISR_PORT, M0_ISR_PIN, M0_I2C_PORT and M0_I2C_PIN (free pins of YOUR board)"
#endif

static inline void m0_pin_setup(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef g = {0};
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    g.Pin = pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &g);
}

static inline void m0_pins_init(void)
{
    /* A GPIO port whose RCC clock is off ignores writes silently, so enable every port a pin may be on. */
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE(); __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE(); __HAL_RCC_GPIOK_CLK_ENABLE();
    m0_pin_setup(M0_ISR_PORT, M0_ISR_PIN);
    m0_pin_setup(M0_I2C_PORT, M0_I2C_PIN);
}
static inline void m0_isr_high(void) { M0_ISR_PORT->BSRR = M0_ISR_PIN; }
static inline void m0_isr_low(void)  { M0_ISR_PORT->BSRR = (uint32_t)M0_ISR_PIN << 16; }
static inline void m0_i2c_high(void) { M0_I2C_PORT->BSRR = M0_I2C_PIN; }
static inline void m0_i2c_low(void)  { M0_I2C_PORT->BSRR = (uint32_t)M0_I2C_PIN << 16; }
#else
static inline void m0_pins_init(void) {}
static inline void m0_isr_high(void) {}
static inline void m0_isr_low(void) {}
static inline void m0_i2c_high(void) {}
static inline void m0_i2c_low(void) {}
#endif

#endif /* APP_MEASURE_M0_PINS_H */
