/*
 * Unit tests for App/ (host build, ASan + UBSan).
 *
 * The ref_* functions are copied verbatim from the pre-refactor main.c and
 * serve as the oracle: the new modules must match them byte for byte.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu/mpu6050.h"
#include "imu/imu_read.h"
#include "packet/crc16.h"
#include "packet/imu_packet.h"
#include "sampling/drdy_queue.h"
#include "uart_tx/uart_tx.h"
#include "bus/i2c_bus_clear.h"
#include "bus/i2c_bus_clear_gpio.h"
#include "mock_hal.h"

static int g_checks, g_fail;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static uint32_t rs = 0x12345678u;                       /* xorshift32, deterministic */
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }

/* ---- oracle 1: verbatim from the pre-refactor main.c ------------------------- */
static uint16_t ref_crc16_ccitt(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void ref_encode(uint8_t pkt[24], uint16_t seq, uint32_t ts, const uint8_t b[14])
{
    pkt[0]  = 0xAA;
    pkt[1]  = 0x55;
    pkt[2]  = (uint8_t)(seq);        pkt[3]  = (uint8_t)(seq >> 8);
    pkt[4]  = (uint8_t)(ts);         pkt[5]  = (uint8_t)(ts >> 8);
    pkt[6]  = (uint8_t)(ts >> 16);   pkt[7]  = (uint8_t)(ts >> 24);
    for (int i = 0; i < 14; i++) pkt[8 + i] = b[i];
    uint16_t crc = ref_crc16_ccitt(&pkt[2], 20);
    pkt[22] = (uint8_t)(crc);        pkt[23] = (uint8_t)(crc >> 8);
}

/* ---- oracle 2: the pre-refactor ISR/main-loop ring, verbatim logic ------------ */
static volatile uint32_t ref_ring[16], ref_head, ref_tail, ref_dropped;
static void ref_isr(uint32_t ts)
{
    uint32_t next = (ref_head + 1u) & (16u - 1u);
    if (next == ref_tail) { ref_dropped++; return; }
    ref_ring[ref_head] = ts;
    ref_head = next;
}
static bool ref_pop(uint32_t *ts)
{
    if (ref_tail != ref_head) {
        *ts = ref_ring[ref_tail];
        ref_tail = (ref_tail + 1u) & (16u - 1u);
        return true;
    }
    return false;
}

/* ---- oracle 3: independent table-driven CRC-16/CCITT-FALSE -------------------- */
static uint16_t tbl[256];
static void tbl_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint16_t c = (uint16_t)(i << 8);
        for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
        tbl[i] = c;
    }
}
static uint16_t crc_tbl(const uint8_t *d, size_t n)
{
    uint16_t c = 0xFFFF;
    while (n--) c = (uint16_t)((c << 8) ^ tbl[(uint8_t)((c >> 8) ^ *d++)]);
    return c;
}

/* ------------------------------------------------------------------------------ */

static void test_crc(void)
{
    puts("crc16");
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1);   /* published check value */
    CHECK(crc16_ccitt((const uint8_t *)"", 0) == 0xFFFF);

    uint8_t buf[64];
    for (int t = 0; t < 20000; t++) {
        size_t n = rnd() % sizeof buf;
        for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)rnd();
        uint16_t got = crc16_ccitt(buf, n);
        if (got != crc_tbl(buf, n) || got != ref_crc16_ccitt(buf, n)) { CHECK(0); break; }
    }
}

static void test_packet(void)
{
    puts("imu_packet");
    uint8_t raw[14], out[24];
    for (int i = 0; i < 14; i++) raw[i] = (uint8_t)i;

    imu_packet_encode(out, 0x1234, 0xAABBCCDDu, raw);
    CHECK(out[0] == 0xAA && out[1] == 0x55);
    CHECK(out[2] == 0x34 && out[3] == 0x12);                                   /* seq, LE */
    CHECK(out[4] == 0xDD && out[5] == 0xCC && out[6] == 0xBB && out[7] == 0xAA); /* ts, LE */
    CHECK(memcmp(&out[8], raw, 14) == 0);                                      /* raw verbatim */
    uint16_t crc = crc_tbl(&out[2], 20);
    CHECK(out[22] == (uint8_t)crc && out[23] == (uint8_t)(crc >> 8));          /* crc, LE, over 2..21 */

    /* byte-for-byte equal to the pre-refactor encoder, including seq/ts extremes */
    static const uint16_t seqs[] = { 0, 1, 0x00FF, 0x0100, 0x7FFF, 0x8000, 0xFFFF };
    static const uint32_t tss[]  = { 0, 1, 0xFF, 0x100, 0xFFFFu, 0x10000u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu };
    uint8_t exp[24];
    for (size_t a = 0; a < sizeof seqs / sizeof *seqs; a++)
        for (size_t b = 0; b < sizeof tss / sizeof *tss; b++) {
            for (int i = 0; i < 14; i++) raw[i] = (uint8_t)rnd();
            imu_packet_encode(out, seqs[a], tss[b], raw);
            ref_encode(exp, seqs[a], tss[b], raw);
            CHECK(memcmp(out, exp, 24) == 0);
        }
    int mismatches = 0;
    for (int t = 0; t < 200000; t++) {
        uint16_t s = (uint16_t)rnd();
        uint32_t ts = rnd();
        for (int i = 0; i < 14; i++) raw[i] = (uint8_t)rnd();
        imu_packet_encode(out, s, ts, raw);
        ref_encode(exp, s, ts, raw);
        mismatches += memcmp(out, exp, 24) != 0;
    }
    CHECK(mismatches == 0);
}

static void test_queue(void)
{
    puts("drdy_queue");
    drdy_queue_t q;
    uint32_t v;
    uint16_t sq;

    drdy_queue_init(&q);
    CHECK(!drdy_queue_pop(&q, &v, &sq));

    for (uint32_t i = 0; i < DRDY_QUEUE_SIZE - 1u; i++) CHECK(drdy_queue_push_from_isr(&q, 100 + i));
    CHECK(!drdy_queue_push_from_isr(&q, 999));                 /* capacity is SIZE-1 */
    CHECK(drdy_queue_dropped(&q) == 1);
    for (uint32_t i = 0; i < DRDY_QUEUE_SIZE - 1u; i++) { CHECK(drdy_queue_pop(&q, &v, &sq)); CHECK(v == 100 + i && sq == i); }
    CHECK(!drdy_queue_pop(&q, &v, &sq));

    /* the dropped event used up sequence number 15: the next accepted event is 16 */
    for (uint32_t i = 0; i < 1000; i++) {                      /* wrap-around many times */
        CHECK(drdy_queue_push_from_isr(&q, i));
        CHECK(drdy_queue_pop(&q, &v, &sq) && v == i && sq == (uint16_t)(DRDY_QUEUE_SIZE + i));
    }
    CHECK(drdy_queue_dropped(&q) == 1);

    /* sequence numbers wrap at 65536 like the packet field */
    drdy_queue_init(&q);
    q.next_seq = 0xFFFE;
    CHECK(drdy_queue_push_from_isr(&q, 1) && drdy_queue_push_from_isr(&q, 2) && drdy_queue_push_from_isr(&q, 3));
    CHECK(drdy_queue_pop(&q, &v, &sq) && sq == 0xFFFE);
    CHECK(drdy_queue_pop(&q, &v, &sq) && sq == 0xFFFF);
    CHECK(drdy_queue_pop(&q, &v, &sq) && sq == 0x0000);

    /* random push/pop mix must match the pre-refactor ring exactly */
    drdy_queue_init(&q);
    ref_head = ref_tail = ref_dropped = 0;
    int bad = 0;
    uint16_t shadow_seq[DRDY_QUEUE_SIZE] = {0};                /* independent model of the sequence numbers */
    uint16_t events = 0;
    for (int t = 0; t < 2000000; t++) {
        bool push = (rnd() % 100) < ((t / 50000) % 2 ? 70 : 40);   /* alternate fill/drain phases */
        if (push) {
            uint32_t ts = rnd();
            uint32_t before = ref_dropped, slot = ref_head;
            ref_isr(ts);
            if (ref_dropped == before) shadow_seq[slot] = events;
            events++;                                           /* every event takes a number, dropped or not */
            bool ok = drdy_queue_push_from_isr(&q, ts);
            bad += ok != (ref_dropped == before);
        } else {
            uint32_t a = 0, b = 0, slot = ref_tail;
            uint16_t sb = 0;
            bool ra = ref_pop(&a), rb = drdy_queue_pop(&q, &b, &sb);
            bad += (ra != rb) || (ra && (a != b || sb != shadow_seq[slot]));
        }
        bad += drdy_queue_dropped(&q) != ref_dropped;
    }
    CHECK(bad == 0);
}

static int count_lines_with(const char *text, const char *prefix)
{
    int n = 0;
    size_t pl = strlen(prefix);
    for (const char *p = text; *p; ) {
        if (strncmp(p, prefix, pl) == 0) n++;
        const char *nl = strchr(p, '\n');
        if (!nl) break;
        p = nl + 1;
    }
    return n;
}

static void test_driver(void)
{
    puts("mpu6050");
    static I2C_HandleTypeDef bus;
    mpu6050_t dev;

    /* Expected sequence is derived by hand from the pre-refactor main.c init code
     * (reads/writes with 0x68<<1 = 0xD0, timeouts 100, delays 100 and 10). */
    static const char expected[] =
        "R dev=0xD0 reg=0x75 len=1 to=100\n"
        "W dev=0xD0 reg=0x6B val=0x80 to=100\n"
        "DELAY 100\n"
        "W dev=0xD0 reg=0x6B val=0x01 to=100\n"
        "DELAY 10\n"
        "W dev=0xD0 reg=0x1A val=0x03 to=100\n"
        "W dev=0xD0 reg=0x19 val=0x09 to=100\n"
        "W dev=0xD0 reg=0x38 val=0x01 to=100\n";

    mock_reset();
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW) == MPU6050_OK);
    CHECK(strcmp(mock_trace(), expected) == 0);
    if (strcmp(mock_trace(), expected) != 0) printf("--- got ---\n%s", mock_trace());

    mock_reset(); mock_cfg.who_am_i = 0x70;                    /* wrong chip */
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW) == MPU6050_ERR_ID);
    CHECK(count_lines_with(mock_trace(), "W ") == 0);          /* nothing written to a wrong chip */

    mock_reset(); mock_cfg.fail_id_read = 1;                   /* no ACK */
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW) == MPU6050_ERR_BUS);
    CHECK(count_lines_with(mock_trace(), "W ") == 0);

    for (int k = 1; k <= 5; k++) {                             /* any single failing write stops init */
        mock_reset(); mock_cfg.fail_write_n = k;
        CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW) == MPU6050_ERR_BUS);
        CHECK(count_lines_with(mock_trace(), "W ") == k);
    }

    uint8_t raw[MPU6050_RAW_LEN];
    mock_reset();
    mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW);
    CHECK(mpu6050_read_raw(&dev, raw));
    CHECK(strstr(mock_trace(), "R dev=0xD0 reg=0x3B len=14 to=20\n") != NULL);

    mock_reset(); mock_cfg.fail_burst_from = 1; mock_cfg.fail_burst_to = 1;
    CHECK(!mpu6050_read_raw(&dev, raw));
}

/* ---- i2c bus clear: pure logic against the simulated stuck slave -------------- */
static void sim_set_scl(void *c, bool r) { bus_sim_set_scl(c, r); }
static void sim_set_sda(void *c, bool r) { bus_sim_set_sda(c, r); }
static bool sim_get_scl(void *c)         { return bus_sim_get_scl(c); }
static bool sim_get_sda(void *c)         { return bus_sim_get_sda(c); }
static void sim_delay(void *c, uint32_t us) { (void)c; (void)us; }

static i2c_bus_status_t run_clear_ex(int hold_edges, int stretch_reads, bool scl_low, unsigned *clocks)
{
    mock_reset();
    g_bus.sda_hold_edges    = hold_edges;
    g_bus.scl_stretch_reads = stretch_reads;
    g_bus.scl_forced_low    = scl_low;
    bus_sim_start(&g_bus);
    const i2c_bus_ops_t ops = { sim_set_scl, sim_set_sda, sim_get_scl, sim_get_sda, sim_delay, &g_bus, false };
    return i2c_bus_clear(&ops, clocks);
}

static i2c_bus_status_t run_clear(int hold_edges, int stretch_reads, unsigned *clocks)
{
    return run_clear_ex(hold_edges, stretch_reads, false, clocks);
}

static void test_bus_clear(void)
{
    puts("i2c_bus_clear");
    unsigned n;

    CHECK(run_clear(0, 0, &n) == I2C_BUS_IDLE);                 /* healthy bus: do nothing */
    CHECK(n == 0 && g_bus.falls == 0 && g_bus.stops == 0 && g_bus.starts == 0);

    for (int z = 1; z <= 9; z++) {                              /* slave needs z clocks to let go */
        CHECK(run_clear(z, 0, &n) == I2C_BUS_RECOVERED);
        CHECK(n == (unsigned)z);
        CHECK(g_bus.stops == 1 && g_bus.starts == 0);           /* ends with exactly one STOP */
        CHECK(g_bus.falls == z + 1);                            /* z pulses + the one before the STOP */
        CHECK(bus_sim_get_sda(&g_bus) && g_bus.m_scl && g_bus.m_sda);
    }

    CHECK(run_clear(10, 0, &n) == I2C_BUS_SDA_STUCK);           /* needs more than 9: give up */
    CHECK(n == 9 && g_bus.stops == 0 && g_bus.m_scl && g_bus.m_sda);   /* lines left released */

    CHECK(run_clear(0, 5, &n) == I2C_BUS_IDLE);                 /* clock stretching is tolerated */
    CHECK(run_clear(3, 5, &n) == I2C_BUS_RECOVERED && n == 3);

    CHECK(run_clear_ex(3, 0, true, &n) == I2C_BUS_SCL_STUCK);   /* SCL shorted low: no pulses attempted */
    CHECK(n == 0 && g_bus.falls == 0 && g_bus.m_scl && g_bus.m_sda);

    CHECK(run_clear(3, -1, &n) == I2C_BUS_SCL_STUCK);           /* slave stretches forever after the first pulse */
    CHECK(n == 0 && g_bus.falls == 1 && g_bus.m_scl && g_bus.m_sda);
}

/* ---- same scenarios through the HAL GPIO binding ------------------------------ */
/* ---- forced variant: a STOP even when SDA is already high ---------------------- */
static i2c_bus_status_t run_always(int hold_edges, bool confused, unsigned *clocks)
{
    mock_reset();
    g_bus.sda_hold_edges = hold_edges;
    g_bus.confused = confused;
    bus_sim_start(&g_bus);
    const i2c_bus_ops_t ops = { sim_set_scl, sim_set_sda, sim_get_scl, sim_get_sda, sim_delay, &g_bus, false };
    return i2c_bus_clear_always(&ops, clocks);
}

static void test_bus_clear_always(void)
{
    puts("i2c_bus_clear_always");
    unsigned n;

    CHECK(run_always(0, false, &n) == I2C_BUS_IDLE);            /* healthy: reports IDLE ... */
    CHECK(n == 0 && g_bus.stops == 1 && g_bus.starts == 0 && g_bus.falls == 1);   /* ... but did send one STOP, no data clocks */
    CHECK(g_bus.m_scl && g_bus.m_sda);

    CHECK(run_always(0, true, &n) == I2C_BUS_IDLE);             /* confused slave, SDA high: the STOP calms it */
    CHECK(!g_bus.confused && n == 0);

    for (int z = 1; z <= 9; z++) {                              /* SDA-low case behaves exactly like i2c_bus_clear */
        CHECK(run_always(z, false, &n) == I2C_BUS_RECOVERED && n == (unsigned)z);
        CHECK(g_bus.stops == 1 && g_bus.starts == 0);
    }
    CHECK(run_always(10, false, &n) == I2C_BUS_SDA_STUCK && n == 9 && g_bus.m_scl && g_bus.m_sda);

    /* the old, conditional function leaves a confused slave alone: that was the board failure */
    unsigned m;
    mock_reset(); g_bus.confused = true; bus_sim_start(&g_bus);
    const i2c_bus_ops_t ops = { sim_set_scl, sim_set_sda, sim_get_scl, sim_get_sda, sim_delay, &g_bus, false };
    CHECK(i2c_bus_clear(&ops, &m) == I2C_BUS_IDLE && g_bus.stops == 0 && g_bus.confused);
}

static void test_bus_clear_gpio(void)
{
    puts("i2c_bus_clear_gpio");
    static GPIO_TypeDef port;
    const i2c_bus_pins_t pins = { &port, GPIO_PIN_8, GPIO_PIN_9 };
    unsigned n;

    mock_reset();
    bus_sim_start(&g_bus);
    CHECK(i2c_bus_clear_gpio(&pins, &n) == I2C_BUS_IDLE);
    CHECK(mock_gpio_glitches == 0);                             /* ODR latched high before the pins turn into outputs */
    CHECK(mock_gpio_deinits == 1 && mock_gpio_deinit_pins == (GPIO_PIN_8 | GPIO_PIN_9));
    CHECK(g_bus.falls == 0);

    for (int z = 1; z <= 9; z++) {
        mock_reset();
        g_bus.sda_hold_edges = z;
        bus_sim_start(&g_bus);
        CHECK(i2c_bus_clear_gpio(&pins, &n) == I2C_BUS_RECOVERED);
        CHECK(n == (unsigned)z && g_bus.stops == 1 && g_bus.starts == 0);
        CHECK(mock_gpio_glitches == 0);
        CHECK(mock_gpio_deinits == 1);
        CHECK(g_bus.m_scl && g_bus.m_sda);                      /* both pins released after DeInit */
    }

    mock_reset();
    g_bus.sda_hold_edges = 12;                                  /* hopeless slave */
    bus_sim_start(&g_bus);
    CHECK(i2c_bus_clear_gpio(&pins, &n) == I2C_BUS_SDA_STUCK);
    CHECK(mock_gpio_deinits == 1 && g_bus.m_scl && g_bus.m_sda);
}

static void test_stuck_bus_story(void)
{
    puts("stuck bus: driver fails, bus clear fixes it");
    static I2C_HandleTypeDef bus;
    static GPIO_TypeDef port;
    const i2c_bus_pins_t pins = { &port, GPIO_PIN_8, GPIO_PIN_9 };
    mpu6050_t dev;
    unsigned n = 0;

    mock_reset();
    g_bus.sda_hold_edges = 5;                                   /* slave left holding SDA low by a reset */
    bus_sim_start(&g_bus);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW) == MPU6050_ERR_BUS);   /* what the board showed */
    CHECK(i2c_bus_clear_gpio(&pins, &n) == I2C_BUS_RECOVERED && n == 5);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW) == MPU6050_OK);
}

/* ---- uart_tx: frame queue in front of a DMA UART --------------------------------------- */
/* Fake DMA: start() remembers the pointer; the bytes are read only when the transfer
 * "completes" (like the real DMA, which reads the buffer while it runs).  An overwrite of a
 * slot that is still in flight therefore shows up as wrong bytes on the wire. */
static struct {
    const uint8_t *buf; uint16_t len; bool inflight;
    unsigned fail_starts;            /* next N start() calls return false */
    unsigned locks, unlocks, depth, max_depth, starts_while_inflight;
    uint8_t wire[1 << 16]; size_t wire_len;
} fdma;

static bool fdma_start(const uint8_t *buf, uint16_t len)
{
    if (fdma.fail_starts) { fdma.fail_starts--; return false; }
    if (fdma.inflight) fdma.starts_while_inflight++;
    fdma.buf = buf; fdma.len = len; fdma.inflight = true;
    return true;
}
static uint32_t fdma_lock(void)   { fdma.locks++;   if (++fdma.depth > fdma.max_depth) fdma.max_depth = fdma.depth; return 7u; }
static void     fdma_unlock(uint32_t t) { fdma.unlocks++; fdma.depth--; CHECK(t == 7u); }

static void fdma_reset(void) { memset(&fdma, 0, sizeof fdma); }

/* the DMA finishes: bytes appear on the wire, then the TX-complete IRQ runs */
static bool fdma_finish(uart_tx_t *q, bool error)
{
    if (!fdma.inflight) return false;
    fdma.inflight = false;
    if (!error) { memcpy(fdma.wire + fdma.wire_len, fdma.buf, fdma.len); fdma.wire_len += fdma.len; }
    if (error) uart_tx_on_error(q); else uart_tx_on_complete(q);
    return true;
}

static void frame_fill(uint8_t *f, uint16_t len, uint32_t id)
{
    for (uint16_t i = 0; i < len; i++) f[i] = (uint8_t)(id * 31u + i * 7u + 1u);
}

static void test_uart_tx(void)
{
    puts("uart_tx");
    static uint8_t slots[UART_TX_SLOTS * UART_TX_SLOT_SIZE];
    const uart_tx_port_t port = { fdma_start, fdma_lock, fdma_unlock };
    uart_tx_t q;
    uint8_t f[UART_TX_SLOT_SIZE];

    /* one frame: starts at once, nothing queued behind it */
    fdma_reset(); uart_tx_init(&q, slots, &port);
    frame_fill(f, 24, 1);
    CHECK(uart_tx_submit(&q, f, 24));
    CHECK(fdma.inflight && fdma.len == 24 && q.busy);
    CHECK(fdma_finish(&q, false) && !q.busy && q.sent == 1 && fdma.wire_len == 24);
    CHECK(memcmp(fdma.wire, f, 24) == 0);

    /* bad lengths are refused and do not touch the lock / counters */
    CHECK(!uart_tx_submit(&q, f, 0) && !uart_tx_submit(&q, f, UART_TX_SLOT_SIZE + 1));
    CHECK(q.submitted == 1 && q.dropped == 0);

    /* overload: nothing completes; the in-flight frame keeps its slot, so capacity is SLOTS-1 */
    fdma_reset(); uart_tx_init(&q, slots, &port);
    unsigned ok = 0;
    for (uint32_t i = 0; i < 40; i++) { frame_fill(f, 24, i); ok += uart_tx_submit(&q, f, 24); }
    CHECK(ok == UART_TX_SLOTS - 1 && q.dropped == 40 - ok && q.submitted == ok);
    while (fdma_finish(&q, false)) {}
    CHECK(q.sent == ok && fdma.wire_len == ok * 24u);
    bool order = true;                                     /* the first `ok` frames, in order, intact */
    for (uint32_t i = 0; i < ok; i++) { frame_fill(f, 24, i); order &= memcmp(fdma.wire + i * 24u, f, 24) == 0; }
    CHECK(order);
    CHECK(fdma.starts_while_inflight == 0);                /* never two transfers at once */

    /* start() refuses (HAL busy / state stuck): frame stays queued and goes out on a later event */
    fdma_reset(); uart_tx_init(&q, slots, &port);
    fdma.fail_starts = 2;
    frame_fill(f, 24, 5); CHECK(uart_tx_submit(&q, f, 24));        /* start fails -> idle, queued */
    CHECK(!fdma.inflight && !q.busy && q.start_fail == 1);
    frame_fill(f, 24, 6); CHECK(uart_tx_submit(&q, f, 24));        /* retry fails once more */
    CHECK(!fdma.inflight && q.start_fail == 2);
    frame_fill(f, 24, 7); CHECK(uart_tx_submit(&q, f, 24));        /* now it starts: frame 5 first */
    CHECK(fdma.inflight);
    while (fdma_finish(&q, false)) {}
    CHECK(fdma.wire_len == 72);
    order = true;
    for (uint32_t i = 0; i < 3; i++) { frame_fill(f, 24, 5 + i); order &= memcmp(fdma.wire + i * 24u, f, 24) == 0; }
    CHECK(order && q.sent == 3);

    /* DMA error abandons the frame in flight, the next one still goes out */
    fdma_reset(); uart_tx_init(&q, slots, &port);
    frame_fill(f, 24, 10); uart_tx_submit(&q, f, 24);
    frame_fill(f, 24, 11); uart_tx_submit(&q, f, 24);
    CHECK(fdma_finish(&q, true) && q.errors == 1 && fdma.inflight);   /* frame 10 lost, 11 started */
    CHECK(fdma_finish(&q, false) && fdma.wire_len == 24);
    frame_fill(f, 24, 11); CHECK(memcmp(fdma.wire, f, 24) == 0);
    CHECK(!q.busy);

    /* spurious completion on an idle queue must not underflow */
    fdma_reset(); uart_tx_init(&q, slots, &port);
    uart_tx_on_complete(&q); uart_tx_on_error(&q);
    CHECK(q.head == q.tail && !q.busy && q.sent == 0);
    frame_fill(f, 24, 3); CHECK(uart_tx_submit(&q, f, 24) && fdma.inflight);

    /* randomized: submits and completions interleaved, mixed frame lengths, many wraps.
     * Model: every accepted frame must appear on the wire once, in order, byte-exact. */
    fdma_reset(); uart_tx_init(&q, slots, &port);
    static uint8_t model[1 << 16]; size_t model_len = 0;
    uint32_t id = 0;
    for (unsigned step = 0; step < 100000; step++) {
        if (rnd() % 100 < 55) {
            uint16_t len = 1u + rnd() % UART_TX_SLOT_SIZE;
            frame_fill(f, len, id++);
            if (uart_tx_submit(&q, f, len)) {
                if (model_len + len > sizeof model) { while (fdma_finish(&q, false)) {} break; }
                memcpy(model + model_len, f, len); model_len += len;
            }
        } else if (fdma.wire_len + UART_TX_SLOT_SIZE < sizeof fdma.wire) {
            fdma_finish(&q, false);
        }
        if (fdma.wire_len + UART_TX_SLOT_SIZE >= sizeof fdma.wire || model_len + UART_TX_SLOT_SIZE >= sizeof model) break;
    }
    while (fdma_finish(&q, false)) {}
    CHECK(fdma.wire_len == model_len && memcmp(fdma.wire, model, model_len) == 0);
    CHECK(q.sent == q.submitted && fdma.starts_while_inflight == 0 && q.head == q.tail && !q.busy);
    CHECK(fdma.locks == fdma.unlocks && fdma.depth == 0 && fdma.max_depth == 1);
    printf("  (%u frames through the queue, %u dropped when full)\n", (unsigned)q.sent, (unsigned)q.dropped);
}


/* ---- imu_read: one DMA burst read at a time, timeout + recovery --------------------------- */
extern struct { unsigned calls; uint16_t dev, reg, add_size, len; uint8_t *buf; HAL_StatusTypeDef ret; } mock_dma_read;

static struct {
    uint32_t now;                 /* the fake microsecond clock */
    unsigned starts, recovers, marks_hi, marks_lo, locks, unlocks;
    bool     start_ok;
    uint32_t recover_cost_us;     /* the fake clock advances by this much inside recover() */
    int      depth, max_depth;
    imu_read_t *fire_in_lock;     /* call on_complete from inside lock(): models the IRQ winning the race */
    bool     fire_error;
    bool     recovering;
    bool     callback_during_recover;
} fr;

static bool     fr_start(uint8_t *buf) { fr.starts++; for (int i = 0; i < 14; i++) buf[i] = (uint8_t)(0x40 + i); return fr.start_ok; }
static imu_read_t *fr_self;
static bool     fr_fast_start(uint8_t *b) { (void)b; imu_read_on_complete(fr_self); return true; }
static void     fr_recover(void) { fr.recovers++; fr.now += fr.recover_cost_us; }
static uint32_t fr_lock(void)
{
    fr.locks++; fr.depth++; if (fr.depth > fr.max_depth) fr.max_depth = fr.depth;
    if (fr.fire_in_lock) { imu_read_t *r = fr.fire_in_lock; fr.fire_in_lock = NULL;
        if (fr.fire_error) imu_read_on_error(r); else imu_read_on_complete(r); }
    return 7;
}
static void     fr_unlock(uint32_t t) { fr.unlocks++; fr.depth--; CHECK(t == 7); }
static uint32_t fr_now(void) { return fr.now; }
static void     fr_mark(bool busy) { if (busy) fr.marks_hi++; else fr.marks_lo++; }

static void fr_reset(imu_read_t *r, uint8_t *buf, uint32_t timeout, uint32_t t0)
{
    static const imu_read_port_t port = { fr_start, fr_recover, fr_lock, fr_unlock, fr_now, fr_mark };
    memset(&fr, 0, sizeof fr);
    fr.start_ok = true; fr.now = t0;
    imu_read_init(r, buf, &port, timeout);
}

static void test_imu_read(void)
{
    puts("imu_read");
    imu_read_t r; uint8_t buf[14];

    /* normal read: start, completes after 430 us, poll reports OK once, D4-style marks balanced */
    fr_reset(&r, buf, 3000, 1000);
    CHECK(imu_read_idle(&r) && imu_read_poll(&r) == IMU_READ_NONE);
    CHECK(imu_read_start(&r, 77, 123456) && !imu_read_idle(&r) && fr.starts == 1 && fr.marks_hi == 1);
    CHECK(imu_read_poll(&r) == IMU_READ_NONE);
    fr.now += 430; imu_read_on_complete(&r);
    CHECK(fr.marks_lo == 1 && r.last_us == 430 && r.max_us == 430 && r.completed == 1);
    CHECK(imu_read_poll(&r) == IMU_READ_OK && r.seq == 77 && r.ts == 123456 && buf[0] == 0x40 && buf[13] == 0x4D);
    CHECK(imu_read_idle(&r) && imu_read_poll(&r) == IMU_READ_NONE);       /* reported once */
    CHECK(fr.recovers == 0 && r.timeouts == 0 && r.errors == 0 && r.spurious == 0);

    /* only one read at a time */
    fr_reset(&r, buf, 3000, 0);
    CHECK(imu_read_start(&r, 1, 1) && !imu_read_start(&r, 2, 2) && r.rejected == 1 && fr.starts == 1 && r.seq == 1);

    /* timeout boundary: exactly timeout_us is still fine, one more gives up */
    fr_reset(&r, buf, 3000, 0xFFFFF000u);                 /* the clock wraps inside the read */
    CHECK(imu_read_start(&r, 5, 50));
    fr.now += 3000; CHECK(imu_read_poll(&r) == IMU_READ_NONE && fr.recovers == 0);
    fr.now += 1;    fr.recover_cost_us = 25000;
    CHECK(imu_read_poll(&r) == IMU_READ_FAILED && r.timeouts == 1 && fr.recovers == 1 && fr.marks_lo == 1);
    CHECK(imu_read_idle(&r) && r.recover_us_max == 25000 && r.recoveries == 1);
    CHECK(fr.locks == 1 && fr.unlocks == 1 && fr.depth == 0);
    /* a callback from the abandoned transfer that slips through is counted and ignored */
    imu_read_on_complete(&r); imu_read_on_error(&r);
    CHECK(r.spurious == 2 && imu_read_idle(&r) && r.completed == 0 && r.errors == 0);
    /* and the next read works normally */
    fr.recover_cost_us = 0;
    CHECK(imu_read_start(&r, 6, 60)); fr.now += 429; imu_read_on_complete(&r);
    CHECK(imu_read_poll(&r) == IMU_READ_OK && r.seq == 6);

    /* streak of lost events: failed reads, errors and refused starts count, a good read resets it */
    fr_reset(&r, buf, 3000, 0);
    CHECK(imu_read_start(&r, 1, 1)); fr.now += 3001; CHECK(imu_read_poll(&r) == IMU_READ_FAILED);   /* timeout */
    CHECK(imu_read_start(&r, 2, 2)); imu_read_on_error(&r); CHECK(imu_read_poll(&r) == IMU_READ_FAILED);
    fr.start_ok = false; CHECK(!imu_read_start(&r, 3, 3)); fr.start_ok = true;
    CHECK(r.streak == 3 && r.streak_max == 3);
    CHECK(imu_read_start(&r, 4, 4)); imu_read_on_complete(&r); CHECK(imu_read_poll(&r) == IMU_READ_OK);
    CHECK(r.streak == 0 && r.streak_max == 3);
    CHECK(imu_read_start(&r, 5, 5)); imu_read_on_error(&r); CHECK(imu_read_poll(&r) == IMU_READ_FAILED);
    CHECK(r.streak == 1 && r.streak_max == 3);                  /* the maximum is kept */

    /* the interrupt wins the race with the timeout decision: no recovery, the read is delivered */
    fr_reset(&r, buf, 3000, 0);
    CHECK(imu_read_start(&r, 9, 90)); fr.now += 3001;
    fr.fire_in_lock = &r;
    CHECK(imu_read_poll(&r) == IMU_READ_NONE && fr.recovers == 0 && r.timeouts == 0 && r.completed == 1);
    CHECK(imu_read_poll(&r) == IMU_READ_OK && r.seq == 9);

    /* an error from the HAL: recovery runs in poll(), the event is lost */
    fr_reset(&r, buf, 3000, 0);
    CHECK(imu_read_start(&r, 11, 110)); fr.now += 50; imu_read_on_error(&r);
    CHECK(r.errors == 1 && fr.marks_lo == 1 && fr.recovers == 0 && !imu_read_idle(&r));   /* recovery not in the ISR */
    CHECK(imu_read_poll(&r) == IMU_READ_FAILED && fr.recovers == 1 && imu_read_idle(&r) && r.timeouts == 0);
    CHECK(imu_read_poll(&r) == IMU_READ_NONE);
    /* the error beats the timeout, and the interrupt wins over it as well */
    fr_reset(&r, buf, 3000, 0);
    CHECK(imu_read_start(&r, 12, 120)); fr.now += 3001; fr.fire_in_lock = &r; fr.fire_error = true;
    CHECK(imu_read_poll(&r) == IMU_READ_NONE && r.timeouts == 0 && r.errors == 1);
    CHECK(imu_read_poll(&r) == IMU_READ_FAILED && fr.recovers == 1);

    /* the port refuses to start: counted, recovery runs, nothing is left busy, marks balanced */
    fr_reset(&r, buf, 3000, 0); fr.start_ok = false;
    CHECK(!imu_read_start(&r, 13, 130) && r.start_fail == 1 && fr.recovers == 1 && imu_read_idle(&r));
    CHECK(fr.marks_hi == 1 && fr.marks_lo == 1 && r.started == 0);
    fr.start_ok = true;
    CHECK(imu_read_start(&r, 14, 140) && r.started == 1);

    /* a start() that completes before it returns (a DMA that is faster than the caller) */
    fr_reset(&r, buf, 3000, 0);
    {
        /* emulate with the port calling on_complete inside start: state must already be BUSY */
        static imu_read_port_t p;
        p = (imu_read_port_t){ 0 };
        p.start = fr_fast_start; p.recover = fr_recover; p.lock = fr_lock; p.unlock = fr_unlock; p.now_us = fr_now;
        imu_read_init(&r, buf, &p, 3000); fr_self = &r;
        CHECK(imu_read_start(&r, 15, 150) && r.completed == 1 && r.spurious == 0);
        CHECK(imu_read_poll(&r) == IMU_READ_OK && r.seq == 15);
    }

    /* the driver call: right device address, register, length and buffer */
    {
        I2C_HandleTypeDef h; mpu6050_t dev = { &h, MPU6050_ADDR_AD0_LOW };
        uint8_t b[14]; memset(&mock_dma_read, 0, sizeof mock_dma_read); mock_dma_read.ret = HAL_OK;
        CHECK(mpu6050_read_raw_dma_start(&dev, b));
        CHECK(mock_dma_read.calls == 1 && mock_dma_read.dev == 0xD0 && mock_dma_read.reg == 0x3B &&
              mock_dma_read.add_size == I2C_MEMADD_SIZE_8BIT && mock_dma_read.len == 14 && mock_dma_read.buf == b);
        mock_dma_read.ret = HAL_BUSY; CHECK(!mpu6050_read_raw_dma_start(&dev, b));
    }

    /* random storm: whatever order the events come in, one read at a time, marks balanced,
     * every started read ends as OK, FAILED(error) or FAILED(timeout) exactly once */
    fr_reset(&r, buf, 3000, 0);
    unsigned ok = 0, failed = 0, begun = 0;
    for (unsigned i = 0; i < 20000; i++) {
        fr.now += 1 + rnd() % 900;
        switch (rnd() % 5) {
        case 0: if (imu_read_start(&r, (uint16_t)i, i)) begun++; break;
        case 1: if (rnd() % 3 == 0) imu_read_on_complete(&r); break;
        case 2: if (rnd() % 8 == 0) imu_read_on_error(&r); break;
        default: {
            imu_read_result_t x = imu_read_poll(&r);
            if (x == IMU_READ_OK) ok++; else if (x == IMU_READ_FAILED) failed++;
        }}
        if (fr.depth != 0) { CHECK(0); break; }
    }
    for (int k = 0; k < 4; k++) { fr.now += 4000; imu_read_result_t x = imu_read_poll(&r);
        if (x == IMU_READ_OK) ok++; else if (x == IMU_READ_FAILED) failed++; }
    CHECK(imu_read_idle(&r) && begun == ok + failed);
    CHECK(fr.marks_hi == fr.marks_lo && fr.max_depth <= 1);
    CHECK(r.timeouts + r.errors == failed && r.completed == ok);
    printf("  (%u reads: %u ok, %u timeouts, %u errors, %u late callbacks ignored)\n",
           begun, ok, (unsigned)r.timeouts, (unsigned)r.errors, (unsigned)r.spurious);
}

/* ---- the closing STOP: with and without an SCL edge ------------------------------------- */
static i2c_bus_status_t run_style(int hold_edges, unsigned bits, int bits_len, bool force, bool no_edge, unsigned *clocks)
{
    mock_reset();
    g_bus.sda_hold_edges = hold_edges;
    g_bus.slave_bits = bits;
    g_bus.slave_bits_len = bits_len;
    bus_sim_start(&g_bus);
    const i2c_bus_ops_t ops = { sim_set_scl, sim_set_sda, sim_get_scl, sim_get_sda, sim_delay, &g_bus, no_edge };
    i2c_bus_status_t st = force ? i2c_bus_clear_always(&ops, clocks) : i2c_bus_clear(&ops, clocks);
    g_bus.slave_bits = 0; g_bus.slave_bits_len = 0;
    return st;
}

static void test_bus_clear_stop_styles(void)
{
    puts("i2c_bus_clear: STOP with / without an SCL edge");
    unsigned n;

    /* a slave that sends 0,1,0,1 (bit k presented after k SCL falls), then lets go */
    CHECK(run_style(0, 0xAu, 4, true, false, &n) == I2C_BUS_SDA_STUCK);   /* old STOP: one clock frees SDA (bit 1), the STOP's SCL fall */
    CHECK(n == 1);                                                         /* brings bit 2 = 0 and the slave holds SDA when we let go */
    CHECK(run_style(0, 0xAu, 4, true, true, &n) == I2C_BUS_RECOVERED && n == 1);
    CHECK(g_bus.falls == 1 && g_bus.starts == 1 && g_bus.stops == 1);    /* the one clock pulse, then START + STOP, no more SCL falls */
    CHECK(g_bus.m_scl && g_bus.m_sda);

    /* SDA is high when the clear starts (bit 0 = 1) but the next bit is 0: 0 clocks, still failed with the old STOP */
    CHECK(run_style(0, 0x1u, 3, true, false, &n) == I2C_BUS_SDA_STUCK && n == 0);   /* bits 1,0,0 */
    CHECK(run_style(0, 0x1u, 3, true, true, &n) == I2C_BUS_IDLE && n == 0);
    CHECK(g_bus.falls == 0 && g_bus.starts == 1 && g_bus.stops == 1);

    /* a slave that is not in the way: both styles agree on the outcome */
    for (int z = 1; z <= 9; z++) {
        CHECK(run_style(z, 0, 0, false, true, &n) == I2C_BUS_RECOVERED && n == (unsigned)z);
        CHECK(g_bus.falls == z && g_bus.stops == 1 && g_bus.starts == 1);
        CHECK(run_style(z, 0, 0, true, false, &n) == I2C_BUS_RECOVERED && n == (unsigned)z);
    }
    CHECK(run_style(10, 0, 0, false, true, &n) == I2C_BUS_SDA_STUCK && n == 9 && g_bus.stops == 0);   /* still gives up */
    CHECK(run_style(0, 0, 0, false, true, &n) == I2C_BUS_IDLE && g_bus.stops == 0 && g_bus.starts == 0);   /* healthy, not forced: touches nothing */
    CHECK(run_style(0, 0, 0, true, true, &n) == I2C_BUS_IDLE && g_bus.stops == 1);                        /* forced: one STOP on a healthy bus */
}

int main(void)
{
    tbl_init();
    test_crc();
    test_packet();
    test_queue();
    test_uart_tx();
    test_imu_read();
    test_driver();
    test_bus_clear();
    test_bus_clear_always();
    test_bus_clear_stop_styles();
    test_bus_clear_gpio();
    test_stuck_bus_story();
    printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
