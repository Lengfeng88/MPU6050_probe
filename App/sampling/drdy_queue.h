#ifndef APP_SAMPLING_DRDY_QUEUE_H
#define APP_SAMPLING_DRDY_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Must be a power of two.  Usable capacity is DRDY_QUEUE_SIZE - 1. */
#define DRDY_QUEUE_SIZE 16u

/*
 * Single-producer / single-consumer queue of DATA_RDY events (timestamp + sequence number).
 *
 * The sequence number is assigned HERE, in the ISR, one per DATA_RDY event, whether or
 * not the event fits into the queue.  A dropped event (queue full) therefore leaves a
 * hole in the sequence numbers that the consumer sees, and so does a main-loop event whose
 * I2C read later fails and produces no packet.
 *
 *   producer: the EXTI ISR           -> drdy_queue_push_from_isr()
 *   consumer: the main loop          -> drdy_queue_pop()
 *
 * Lock-free: only the producer writes `head` and the slots, only the
 * consumer writes `tail`.  This is correct for ONE producer and ONE consumer
 * on a single core, which is how it is used here.  Do not call push from two
 * different interrupt priorities, and do not call pop from an ISR.
 */
typedef struct {
    volatile uint32_t ts_us[DRDY_QUEUE_SIZE];
    volatile uint16_t seq[DRDY_QUEUE_SIZE];
    volatile uint16_t next_seq;  /* written by the ISR: number of the next event, wraps at 65536 */
    volatile uint32_t head;      /* written by the ISR   */
    volatile uint32_t tail;      /* written by main loop */
    volatile uint32_t dropped;   /* pushes rejected because the queue was full */
} drdy_queue_t;

void drdy_queue_init(drdy_queue_t *q);

/** Takes the next sequence number; returns false (and counts a drop) when full. */
bool drdy_queue_push_from_isr(drdy_queue_t *q, uint32_t ts_us);

/** Returns false when empty. */
bool drdy_queue_pop(drdy_queue_t *q, uint32_t *ts_us, uint16_t *seq);

uint32_t drdy_queue_dropped(const drdy_queue_t *q);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAMPLING_DRDY_QUEUE_H */
