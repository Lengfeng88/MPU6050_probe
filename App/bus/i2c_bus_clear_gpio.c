#include "bus/i2c_bus_clear_gpio.h"

static void set_scl(void *c, bool released)
{
    const i2c_bus_pins_t *p = c;
    HAL_GPIO_WritePin(p->port, p->scl_pin, released ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_sda(void *c, bool released)
{
    const i2c_bus_pins_t *p = c;
    HAL_GPIO_WritePin(p->port, p->sda_pin, released ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool get_scl(void *c)
{
    const i2c_bus_pins_t *p = c;
    return HAL_GPIO_ReadPin(p->port, p->scl_pin) == GPIO_PIN_SET;
}

static bool get_sda(void *c)
{
    const i2c_bus_pins_t *p = c;
    return HAL_GPIO_ReadPin(p->port, p->sda_pin) == GPIO_PIN_SET;
}

static void delay_us(void *c, uint32_t us)
{
    (void)c;
    HAL_Delay((us + 999u) / 1000u);        /* SysTick based: rounds every delay up to 1 ms */
}

static i2c_bus_status_t clear_gpio(const i2c_bus_pins_t *pins, unsigned *clocks_used, bool force)
{
    const uint16_t both = (uint16_t)(pins->scl_pin | pins->sda_pin);
    GPIO_InitTypeDef gi = {0};

    /* Latch "released" in ODR BEFORE the pins become outputs; otherwise the
     * reset value (0) would pull both lines low for a moment. */
    HAL_GPIO_WritePin(pins->port, both, GPIO_PIN_SET);

    gi.Pin   = both;
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(pins->port, &gi);

    const i2c_bus_ops_t ops = {
        .set_scl  = set_scl,
        .set_sda  = set_sda,
        .get_scl  = get_scl,
        .get_sda  = get_sda,
        .delay_us = delay_us,
        .ctx      = (void *)pins,
    };
    i2c_bus_status_t st = force ? i2c_bus_clear_always(&ops, clocks_used)
                                : i2c_bus_clear(&ops, clocks_used);

    HAL_GPIO_DeInit(pins->port, both);     /* back to reset state for HAL_I2C_MspInit() */
    return st;
}

i2c_bus_status_t i2c_bus_clear_gpio(const i2c_bus_pins_t *pins, unsigned *clocks_used)
{
    return clear_gpio(pins, clocks_used, false);
}

i2c_bus_status_t i2c_bus_clear_gpio_always(const i2c_bus_pins_t *pins, unsigned *clocks_used)
{
    return clear_gpio(pins, clocks_used, true);
}
