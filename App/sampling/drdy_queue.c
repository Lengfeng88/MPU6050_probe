#include "sampling/drdy_queue.h"

_Static_assert((DRDY_QUEUE_SIZE & (DRDY_QUEUE_SIZE - 1u)) == 0u,
               "DRDY_QUEUE_SIZE must be a power of two");

#define MASK (DRDY_QUEUE_SIZE - 1u)

void drdy_queue_init(drdy_queue_t *q)
{
    for (uint32_t i = 0; i < DRDY_QUEUE_SIZE; i++) {
        q->ts_us[i] = 0;
        q->seq[i]   = 0;
    }
    q->head    = 0;
    q->tail    = 0;
    q->dropped = 0;
    q->next_seq = 0;
}

bool drdy_queue_push_from_isr(drdy_queue_t *q, uint32_t ts_us)
{
    uint32_t head = q->head;
    uint32_t next = (head + 1u) & MASK;
    uint16_t seq  = q->next_seq;
    q->next_seq   = (uint16_t)(seq + 1u);   /* consumed by the event even if it is dropped */
    if (next == q->tail) {            /* full */
        q->dropped++;
        return false;
    }
    q->ts_us[head] = ts_us;           /* publish the slot before moving head */
    q->seq[head] = seq;
    q->head = next;
    return true;
}

bool drdy_queue_pop(drdy_queue_t *q, uint32_t *ts_us, uint16_t *seq)
{
    uint32_t tail = q->tail;
    if (tail == q->head)              /* empty */
        return false;
    *ts_us = q->ts_us[tail];
    *seq   = q->seq[tail];
    q->tail = (tail + 1u) & MASK;
    return true;
}

uint32_t drdy_queue_dropped(const drdy_queue_t *q)
{
    return q->dropped;
}
