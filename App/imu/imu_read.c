#include "imu/imu_read.h"

void imu_read_init(imu_read_t *r, uint8_t *raw, const imu_read_port_t *port, uint32_t timeout_us)
{
    *r = (imu_read_t){0};
    r->raw        = raw;
    r->port       = *port;
    r->timeout_us = timeout_us;
    r->state      = IMU_READ_IDLE;
}

static void mark(const imu_read_t *r, bool busy)
{
    if (r->port.mark)
        r->port.mark(busy);
}

static void recover_now(imu_read_t *r)
{
    uint32_t t0 = r->port.now_us();
    r->port.recover();
    uint32_t dt = r->port.now_us() - t0;
    r->recoveries++;
    if (dt > r->recover_us_max)
        r->recover_us_max = dt;
}

bool imu_read_start(imu_read_t *r, uint16_t seq, uint32_t ts)
{
    if (r->state != IMU_READ_IDLE) {
        r->rejected++;
        return false;
    }
    r->seq     = seq;
    r->ts      = ts;
    r->t_start = r->port.now_us();
    r->state   = IMU_READ_BUSY;          /* before start(): the DMA interrupt may fire at once */
    mark(r, true);
    if (!r->port.start(r->raw)) {
        r->state = IMU_READ_IDLE;
        mark(r, false);
        r->start_fail++;
        recover_now(r);
        return false;
    }
    r->started++;
    return true;
}

imu_read_result_t imu_read_poll(imu_read_t *r)
{
    switch (r->state) {
    case IMU_READ_DONE:
        r->state = IMU_READ_IDLE;
        return IMU_READ_OK;

    case IMU_READ_ERROR:
        recover_now(r);
        r->state = IMU_READ_IDLE;
        return IMU_READ_FAILED;

    case IMU_READ_BUSY: {
        if ((uint32_t)(r->port.now_us() - r->t_start) <= r->timeout_us)
            return IMU_READ_NONE;
        /* The completion interrupt may have fired since the check above: decide under the lock. */
        uint32_t tok = r->port.lock();
        bool give_up = (r->state == IMU_READ_BUSY);
        if (give_up)
            r->state = IMU_READ_RECOVERING;   /* from here on_complete / on_error are ignored */
        r->port.unlock(tok);
        if (!give_up)
            return IMU_READ_NONE;             /* it finished meanwhile: the next poll reports it */
        r->timeouts++;
        mark(r, false);
        recover_now(r);
        r->state = IMU_READ_IDLE;
        return IMU_READ_FAILED;
    }

    default:                                  /* IDLE (and RECOVERING, never seen from the loop) */
        return IMU_READ_NONE;
    }
}

void imu_read_on_complete(imu_read_t *r)
{
    if (r->state != IMU_READ_BUSY) {
        r->spurious++;
        return;
    }
    uint32_t el = r->port.now_us() - r->t_start;
    r->last_us = el;
    if (el > r->max_us)
        r->max_us = el;
    r->completed++;
    mark(r, false);
    r->state = IMU_READ_DONE;
}

void imu_read_on_error(imu_read_t *r)
{
    if (r->state != IMU_READ_BUSY) {
        r->spurious++;
        return;
    }
    r->errors++;
    mark(r, false);
    r->state = IMU_READ_ERROR;
}
