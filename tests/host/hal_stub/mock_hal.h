#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <stdint.h>

#include "bus_sim.h"

/* Fault injection for the simulated MPU-6050 (all zero = healthy). */
typedef struct {
    uint8_t who_am_i;                       /* value returned for reg 0x75 */
    int     fail_id_read;                   /* make the WHO_AM_I read return HAL_ERROR */
    int     fail_write_n;                   /* fail the Nth register write (1-based) */
    int     fail_burst_from, fail_burst_to; /* fail burst reads from..to (1-based, inclusive) */
    int     poison_relapse;                 /* with poison_at_txn: the slave fails that transaction but leaves SDA high (confused) */
    int     poison_at_txn;                  /* the Nth I2C transaction wedges the bus (slave grabs SDA for 5 clocks) */
} mock_cfg_t;

extern mock_cfg_t mock_cfg;

/* Mock GPIO: PB8 is wired to the simulated bus' SCL, PB9 to SDA (see bus_sim.h). */
extern bus_sim_t g_bus;
extern int       mock_gpio_glitches;        /* pins driven low by HAL_GPIO_Init because ODR was still 0 */
extern int       mock_gpio_deinits;
extern uint16_t  mock_gpio_deinit_pins;

/** Clear trace, UART capture, counters and the bus; healthy sensor (WHO_AM_I = 0x68). */
void mock_reset(void);

/** NUL-terminated text log of every HAL call since mock_reset(). */
const char *mock_trace(void);

#endif /* MOCK_HAL_H */
