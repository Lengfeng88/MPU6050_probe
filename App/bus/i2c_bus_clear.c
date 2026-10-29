#include "bus/i2c_bus_clear.h"

/* Release SCL and wait for it to actually go high (a slave may stretch it). */
static bool release_scl_and_wait(const i2c_bus_ops_t *o)
{
    o->set_scl(o->ctx, true);
    for (unsigned i = 0; i < I2C_BUS_SCL_WAIT_POLLS; i++) {
        o->delay_us(o->ctx, I2C_BUS_HALF_PERIOD_US);
        if (o->get_scl(o->ctx))
            return true;
    }
    return false;
}

static i2c_bus_status_t clear_impl(const i2c_bus_ops_t *o, unsigned *clocks, bool force)
{
    o->set_sda(o->ctx, true);
    if (!release_scl_and_wait(o))
        return I2C_BUS_SCL_STUCK;
    const bool was_stuck = !o->get_sda(o->ctx);
    if (!was_stuck && !force)
        return I2C_BUS_IDLE;                       /* healthy bus: touch nothing else */

    /* SDA is held low: clock until the slave lets go, at most 9 pulses.
     * (Skipped when SDA is high: forced mode then only issues the STOP below.) */
    while (!o->get_sda(o->ctx) && *clocks < I2C_BUS_CLEAR_MAX_CLOCKS) {
        o->set_scl(o->ctx, false);
        o->delay_us(o->ctx, I2C_BUS_HALF_PERIOD_US);
        if (!release_scl_and_wait(o))
            return I2C_BUS_SCL_STUCK;
        (*clocks)++;
    }
    if (!o->get_sda(o->ctx))
        return I2C_BUS_SDA_STUCK;

    /* Finish with a proper STOP so every device on the bus is idle:
     * SDA low while SCL is low, then SCL high, then SDA high. */
    o->set_scl(o->ctx, false);
    o->delay_us(o->ctx, I2C_BUS_HALF_PERIOD_US);
    o->set_sda(o->ctx, false);
    o->delay_us(o->ctx, I2C_BUS_HALF_PERIOD_US);
    if (!release_scl_and_wait(o))
        return I2C_BUS_SCL_STUCK;
    o->set_sda(o->ctx, true);
    o->delay_us(o->ctx, I2C_BUS_HALF_PERIOD_US);

    if (!(o->get_sda(o->ctx) && o->get_scl(o->ctx)))
        return I2C_BUS_SDA_STUCK;
    return was_stuck ? I2C_BUS_RECOVERED : I2C_BUS_IDLE;
}

static i2c_bus_status_t run(const i2c_bus_ops_t *ops, unsigned *clocks_used, bool force)
{
    unsigned clocks = 0;
    i2c_bus_status_t st = clear_impl(ops, &clocks, force);

    /* never leave a line pulled low, whatever happened */
    ops->set_scl(ops->ctx, true);
    ops->set_sda(ops->ctx, true);

    if (clocks_used)
        *clocks_used = clocks;
    return st;
}

i2c_bus_status_t i2c_bus_clear(const i2c_bus_ops_t *ops, unsigned *clocks_used)
{
    return run(ops, clocks_used, false);
}

i2c_bus_status_t i2c_bus_clear_always(const i2c_bus_ops_t *ops, unsigned *clocks_used)
{
    return run(ops, clocks_used, true);
}
