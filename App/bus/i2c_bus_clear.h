#ifndef APP_BUS_I2C_BUS_CLEAR_H
#define APP_BUS_I2C_BUS_CLEAR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * I2C bus clear (NXP UM10204, section 3.1.16).
 *
 * A slave interrupted in the middle of a byte (typically because the MCU was
 * reset mid-transfer, e.g. by the debugger) can keep SDA pulled low while it
 * waits for more clock pulses.  The I2C peripheral then sees "arbitration
 * lost" on its next START and every transfer fails until the slave is power
 * cycled.  Clocking SCL by hand lets the slave finish its byte and let go.
 *
 * This file is pure logic: all pin access goes through i2c_bus_ops_t, so it
 * is tested on the host against a simulated stuck slave.  The STM32 HAL
 * binding is in bus/i2c_bus_clear_gpio.h.
 *
 * Call it while the I2C peripheral is NOT driving the pins (before the
 * peripheral is initialised).
 */

typedef struct {
    /* true = release the line (open-drain, pulled up), false = pull low */
    void (*set_scl)(void *ctx, bool released);
    void (*set_sda)(void *ctx, bool released);
    /* level actually seen on the pin: true = high */
    bool (*get_scl)(void *ctx);
    bool (*get_sda)(void *ctx);
    void (*delay_us)(void *ctx, uint32_t us);
    void *ctx;
} i2c_bus_ops_t;

typedef enum {
    I2C_BUS_IDLE = 0,      /* both lines were high; nothing was done        */
    I2C_BUS_RECOVERED,     /* SDA was stuck low; released, STOP issued      */
    I2C_BUS_SCL_STUCK,     /* SCL held low (stretching forever or shorted)  */
    I2C_BUS_SDA_STUCK,     /* SDA still low after 9 clocks (or shorted)     */
} i2c_bus_status_t;

#define I2C_BUS_CLEAR_MAX_CLOCKS   9u   /* 8 data bits + ACK slot */
#define I2C_BUS_HALF_PERIOD_US     5u   /* ~100 kHz */
#define I2C_BUS_SCL_WAIT_POLLS     20u  /* tolerated clock stretching, in half periods */

/**
 * Only acts if SDA is seen low.  @param clocks_used optional; receives the
 * number of SCL pulses generated.  Leaves both lines released in every case.
 */
i2c_bus_status_t i2c_bus_clear(const i2c_bus_ops_t *ops, unsigned *clocks_used);

/**
 * Same, but ALSO issues a STOP when SDA is already high (status I2C_BUS_IDLE,
 * 0 clocks).  A slave interrupted mid-transfer does not necessarily hold SDA
 * low at that moment (it may be waiting for a bit that happens to read 1) and
 * then misbehaves on the next START.  A STOP puts every slave back to idle
 * without writing anything (no data clocks are sent while SDA is high).
 */
i2c_bus_status_t i2c_bus_clear_always(const i2c_bus_ops_t *ops, unsigned *clocks_used);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUS_I2C_BUS_CLEAR_H */
