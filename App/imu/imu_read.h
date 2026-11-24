#ifndef APP_IMU_IMU_READ_H
#define APP_IMU_IMU_READ_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * One IMU burst read at a time, started by the main loop, finished by an interrupt (I2C + DMA),
 * with a software timeout and bus recovery (the HAL's DMA mode has no timeout of its own).
 * Hardware independent: everything that touches the chip comes in through imu_read_port_t, so
 * the logic is tested on the host (tests/host/test_units.c, imu_read).
 *
 *   main loop:  imu_read_start()  imu_read_poll()  imu_read_idle()
 *   interrupt:  imu_read_on_complete()  imu_read_on_error()     (I2C / DMA IRQ, one priority level)
 *
 * Life of one read:   IDLE --start--> BUSY --on_complete--> DONE --poll--> IDLE   (poll returns OK)
 *                                     BUSY --on_error-----> FAILED --poll--> recover, IDLE (FAILED)
 *                                     BUSY --timeout in poll--> recover, IDLE            (FAILED)
 * A FAILED read produces no packet: its sequence number is a hole that the receiver sees.
 *
 * Rules the port must keep:
 *  - start() may complete (and call on_complete) before it returns, so the state is BUSY first.
 *  - recover() runs in the main loop only (it may take tens of ms: HAL_Delay based bus clear).
 *    When it returns, no completion / error interrupt of the abandoned transfer may still be
 *    pending: a late callback would complete the NEXT read with half a buffer.
 *  - lock() masks the completion / error interrupt (not the DATA_RDY one) and returns a token.
 */
typedef struct {
    /** Begin a DMA burst read into buf (MPU6050_RAW_LEN bytes).  false: not started. */
    bool     (*start)(uint8_t *buf);
    /** Abort the transfer, free the bus, re-initialise the peripheral. */
    void     (*recover)(void);
    uint32_t (*lock)(void);
    void     (*unlock)(uint32_t token);
    /** Free-running microsecond counter (wraps at 2^32). */
    uint32_t (*now_us)(void);
    /** Optional: called with true when a read starts, false when it ends (ok, error or timeout). */
    void     (*mark)(bool busy);
} imu_read_port_t;

typedef enum {
    IMU_READ_NONE = 0,     /* nothing finished */
    IMU_READ_OK,           /* raw[] holds the 14 bytes of event `seq` / `ts` */
    IMU_READ_FAILED,       /* the event is lost; recovery has run */
} imu_read_result_t;

typedef enum {
    IMU_READ_IDLE = 0,
    IMU_READ_BUSY,
    IMU_READ_DONE,
    IMU_READ_ERROR,        /* on_error() was called; poll() recovers */
    IMU_READ_RECOVERING,   /* poll() gave up on the transfer */
} imu_read_state_t;

typedef struct {
    uint8_t              *raw;              /* DMA-capable RAM, owned by the caller, 14 bytes */
    imu_read_port_t       port;
    uint32_t              timeout_us;
    volatile imu_read_state_t state;
    uint16_t              seq;              /* event being read / last event delivered */
    uint32_t              ts;
    uint32_t              t_start;
    volatile uint32_t     started;
    volatile uint32_t     completed;        /* on_complete accepted */
    volatile uint32_t     errors;           /* on_error accepted */
    volatile uint32_t     timeouts;
    volatile uint32_t     start_fail;
    volatile uint32_t     recoveries;
    volatile uint32_t     spurious;         /* callbacks that arrived when no read was in flight */
    volatile uint32_t     rejected;         /* start() while not idle */
    volatile uint32_t     last_us;          /* start -> completion of the last good read */
    volatile uint32_t     max_us;           /* the largest of those: "maximum observed", not a bound */
    volatile uint32_t     recover_us_max;
} imu_read_t;

void imu_read_init(imu_read_t *r, uint8_t *raw, const imu_read_port_t *port, uint32_t timeout_us);

static inline bool imu_read_idle(const imu_read_t *r) { return r->state == IMU_READ_IDLE; }

/** Begin reading event (seq, ts).  false if a read is already in flight (counted) or the port
 *  refused to start (counted; recovery has run).  Never blocks except in that recovery. */
bool imu_read_start(imu_read_t *r, uint16_t seq, uint32_t ts);

/** Call every pass of the main loop. */
imu_read_result_t imu_read_poll(imu_read_t *r);

/** Interrupt context. */
void imu_read_on_complete(imu_read_t *r);
void imu_read_on_error(imu_read_t *r);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_IMU_READ_H */
