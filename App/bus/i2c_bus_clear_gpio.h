#ifndef APP_BUS_I2C_BUS_CLEAR_GPIO_H
#define APP_BUS_I2C_BUS_CLEAR_GPIO_H

#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "bus/i2c_bus_clear.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GPIO_TypeDef *port;      /* SCL and SDA must be on the same port */
    uint16_t      scl_pin;   /* GPIO_PIN_x */
    uint16_t      sda_pin;
} i2c_bus_pins_t;

/**
 * STM32 HAL binding of i2c_bus_clear().
 *
 * Preconditions:
 *   - the port's GPIO clock is enabled (__HAL_RCC_GPIOx_CLK_ENABLE())
 *   - HAL_Init() has run (HAL_Delay is used; the delays are therefore ~1 ms
 *     granular, which is fine for a path that only does real work on a stuck bus)
 *   - the I2C peripheral has NOT been initialised yet, i.e. call this before
 *     MX_I2C1_Init(); the pins are returned in their reset state so that
 *     HAL_I2C_MspInit() can configure them as usual.
 *
 * i2c_bus_clear_gpio():        on an idle bus costs one HAL_Delay(1), changes nothing.
 * i2c_bus_clear_gpio_always(): additionally issues a STOP on an idle bus (5 x HAL_Delay(1)).
 */
i2c_bus_status_t i2c_bus_clear_gpio(const i2c_bus_pins_t *pins, unsigned *clocks_used);
i2c_bus_status_t i2c_bus_clear_gpio_always(const i2c_bus_pins_t *pins, unsigned *clocks_used);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUS_I2C_BUS_CLEAR_GPIO_H */
