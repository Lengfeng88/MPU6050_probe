#include "uart_tx/uart_tx.h"

#include <string.h>

_Static_assert((UART_TX_SLOTS & (UART_TX_SLOTS - 1u)) == 0u, "UART_TX_SLOTS must be a power of two");
_Static_assert(UART_TX_SLOT_SIZE <= 255u, "len[] is a byte per slot");

#define MASK (UART_TX_SLOTS - 1u)

void uart_tx_init(uart_tx_t *q, uint8_t *slots, const uart_tx_port_t *port)
{
    memset(q, 0, sizeof *q);
    q->slots = slots;
    q->port  = *port;
}

/* Start the frame at `tail` if there is one.  Caller holds the lock or is the consumer IRQ. */
static void kick(uart_tx_t *q)
{
    if (q->head == q->tail) {
        q->busy = false;
        return;
    }
    uint32_t t = q->tail;
    q->busy = q->port.start(&q->slots[t * UART_TX_SLOT_SIZE], q->len[t]);
    if (!q->busy)
        q->start_fail++;      /* frame stays queued; the next submit / completion retries */
}

bool uart_tx_submit(uart_tx_t *q, const uint8_t *frame, uint16_t len)
{
    if (len == 0u || len > UART_TX_SLOT_SIZE)
        return false;

    uint32_t token = q->port.lock();
    uint32_t head  = q->head;
    uint32_t next  = (head + 1u) & MASK;
    bool ok = (next != q->tail);
    if (ok) {
        memcpy(&q->slots[head * UART_TX_SLOT_SIZE], frame, len);
        q->len[head] = (uint8_t)len;
        q->head = next;           /* publish after the copy */
        q->submitted++;
        if (!q->busy)
            kick(q);
    } else {
        q->dropped++;
    }
    q->port.unlock(token);
    return ok;
}

/* Frame at `tail` is done (sent or abandoned): free its slot, start the next one. */
static void finish_one(uart_tx_t *q)
{
    if (q->head == q->tail) {     /* spurious callback: nothing was in flight */
        q->busy = false;
        return;
    }
    q->tail = (q->tail + 1u) & MASK;
    kick(q);
}

void uart_tx_on_complete(uart_tx_t *q)
{
    if (q->head != q->tail)
        q->sent++;
    finish_one(q);
}

void uart_tx_on_error(uart_tx_t *q)
{
    q->errors++;
    finish_one(q);
}
