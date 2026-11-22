#ifndef APP_UART_TX_UART_TX_H
#define APP_UART_TX_UART_TX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Power of two.  Usable capacity is UART_TX_SLOTS - 1 frames. */
#define UART_TX_SLOTS      16u
/* Largest frame; the IMU packet is 24 bytes, a later status / result frame must fit too. */
#define UART_TX_SLOT_SIZE  32u

/*
 * Frame queue in front of a DMA-driven UART.  Hardware independent: the three things that
 * touch hardware come in through uart_tx_port_t, so the logic is tested on the host.
 *
 *   producer: thread context (main loop)      -> uart_tx_submit()
 *   consumer: TX-complete / TX-error IRQ      -> uart_tx_on_complete() / uart_tx_on_error()
 *
 * The producer and the consumer both touch `busy`, so submit() runs inside port->lock().
 * On the target the lock raises BASEPRI above the UART/DMA interrupt priority ONLY; the
 * DATA_RDY interrupt (higher priority) is never masked, so timestamps do not depend on it.
 *
 * Nothing here blocks: a full queue drops the new frame and counts it.
 */
typedef struct {
    /** Start sending `len` bytes at `buf` (DMA).  The buffer stays valid until on_complete. */
    bool     (*start)(const uint8_t *buf, uint16_t len);
    /** Mask the UART TX-complete IRQ; returns a token for unlock().  Must not mask DRDY. */
    uint32_t (*lock)(void);
    void     (*unlock)(uint32_t token);
} uart_tx_port_t;

typedef struct {
    uint8_t       *slots;                  /* UART_TX_SLOTS * UART_TX_SLOT_SIZE bytes, in DMA-capable RAM */
    uint8_t        len[UART_TX_SLOTS];
    uart_tx_port_t port;
    volatile uint32_t head;                /* next slot to fill (producer) */
    volatile uint32_t tail;                /* oldest frame not fully sent; in flight while busy */
    volatile bool     busy;                /* a DMA transfer of slot `tail` is running */
    volatile uint32_t submitted;           /* frames accepted */
    volatile uint32_t sent;                /* frames fully transmitted */
    volatile uint32_t dropped;             /* frames rejected because the queue was full */
    volatile uint32_t start_fail;          /* port->start() returned false */
    volatile uint32_t errors;              /* UART/DMA errors (frame abandoned) */
} uart_tx_t;

void uart_tx_init(uart_tx_t *q, uint8_t *slots, const uart_tx_port_t *port);

/** Queue one frame (len <= UART_TX_SLOT_SIZE) and start the transfer if the line is idle.
 *  Returns false if the frame was not queued (queue full, or bad length). */
bool uart_tx_submit(uart_tx_t *q, const uint8_t *frame, uint16_t len);

/** Call from the TX-complete IRQ: the frame at `tail` has left the shift register. */
void uart_tx_on_complete(uart_tx_t *q);

/** Call from the UART/DMA error IRQ: abandon the frame in flight and carry on. */
void uart_tx_on_error(uart_tx_t *q);

#ifdef __cplusplus
}
#endif

#endif /* APP_UART_TX_UART_TX_H */
