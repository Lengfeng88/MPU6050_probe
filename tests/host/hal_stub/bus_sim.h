#ifndef BUS_SIM_H
#define BUS_SIM_H

/*
 * Simulated open-drain I2C bus with one misbehaving slave, for host tests.
 *
 *  - sda_hold_edges:    the slave holds SDA low until it has seen this many SCL
 *                       falling edges (a slave interrupted mid-byte outputs zeros
 *                       until clocked through the rest of the byte); 0 = healthy
 *  - scl_stretch_reads: every time the master releases SCL (after having pulled it
 *                       low) it keeps reading low this many times; -1 = forever
 *  - scl_forced_low:    SCL is shorted low from the start
 *
 * The sim also counts SCL falling edges and START / STOP conditions (an SDA
 * edge while SCL is high).
 */
#include <stdbool.h>

typedef struct {
    /* scenario: set before bus_sim_start() */
    int  sda_hold_edges;
    int  scl_stretch_reads;
    bool scl_forced_low;
    bool confused;           /* slave stopped mid-transfer: SDA looks high, but it grabs SDA
                                low at the next transaction unless a STOP came first */

    /* master drive, true = released */
    bool m_scl, m_sda;

    /* observations */
    int  falls;
    int  starts, stops;

    /* internals */
    int  stretch_left;
    bool last_scl, last_sda;
} bus_sim_t;

static inline bool bus_sim_sda_level(const bus_sim_t *b)
{
    return b->m_sda && !(b->falls < b->sda_hold_edges);
}

static inline bool bus_sim_scl_level(const bus_sim_t *b)
{
    return b->m_scl && b->stretch_left == 0 && !b->scl_forced_low;
}

static inline void bus_sim_update(bus_sim_t *b)
{
    bool scl = bus_sim_scl_level(b);
    if (b->last_scl && !scl)
        b->falls++;
    bool sda = bus_sim_sda_level(b);
    if (b->last_scl && scl) {                       /* SCL stayed high: an SDA edge is a START/STOP */
        if (b->last_sda && !sda) b->starts++;
        if (!b->last_sda && sda) { b->stops++; b->confused = false; }
    }
    b->last_scl = scl;
    b->last_sda = sda;
}

static inline void bus_sim_start(bus_sim_t *b)
{
    b->m_scl = b->m_sda = true;
    b->falls = b->starts = b->stops = 0;
    b->stretch_left = 0;
    b->last_scl = bus_sim_scl_level(b);
    b->last_sda = bus_sim_sda_level(b);
}

static inline void bus_sim_set_scl(bus_sim_t *b, bool released)
{
    bool was_released = b->m_scl;
    b->m_scl = released;
    if (!released)
        b->stretch_left = 0;                        /* master holds it low anyway */
    else if (!was_released)
        b->stretch_left = b->scl_stretch_reads;     /* the slave may stretch after every release */
    bus_sim_update(b);
}

static inline void bus_sim_set_sda(bus_sim_t *b, bool released)
{
    b->m_sda = released;
    bus_sim_update(b);
}

static inline bool bus_sim_get_sda(bus_sim_t *b)
{
    return bus_sim_sda_level(b);
}

static inline bool bus_sim_get_scl(bus_sim_t *b)
{
    if (!b->m_scl || b->scl_forced_low)
        return false;
    if (b->stretch_left != 0) {                     /* stretched: this read still sees low */
        if (b->stretch_left > 0)
            b->stretch_left--;
        b->last_scl = (b->stretch_left == 0);
        return false;
    }
    return true;
}

#endif /* BUS_SIM_H */
