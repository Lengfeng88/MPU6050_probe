#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <stdint.h>

/* Fault injection for the simulated MPU-6050 (all zero = healthy). */
typedef struct {
    uint8_t who_am_i;                       /* value returned for reg 0x75 */
    int     fail_id_read;                   /* make the WHO_AM_I read return HAL_ERROR */
    int     fail_write_n;                   /* fail the Nth register write (1-based) */
    int     fail_burst_from, fail_burst_to; /* fail burst reads from..to (1-based, inclusive) */
} mock_cfg_t;

extern mock_cfg_t mock_cfg;

/** Clear trace, UART capture and counters; healthy sensor (WHO_AM_I = 0x68). */
void mock_reset(void);

/** NUL-terminated text log of every HAL call since mock_reset(). */
const char *mock_trace(void);

#endif /* MOCK_HAL_H */
