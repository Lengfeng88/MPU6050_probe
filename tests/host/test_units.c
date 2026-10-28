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
#include "packet/crc16.h"
#include "packet/imu_packet.h"
#include "sampling/drdy_queue.h"
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

    drdy_queue_init(&q);
    CHECK(!drdy_queue_pop(&q, &v));

    for (uint32_t i = 0; i < DRDY_QUEUE_SIZE - 1u; i++) CHECK(drdy_queue_push_from_isr(&q, 100 + i));
    CHECK(!drdy_queue_push_from_isr(&q, 999));                 /* capacity is SIZE-1 */
    CHECK(drdy_queue_dropped(&q) == 1);
    for (uint32_t i = 0; i < DRDY_QUEUE_SIZE - 1u; i++) { CHECK(drdy_queue_pop(&q, &v)); CHECK(v == 100 + i); }
    CHECK(!drdy_queue_pop(&q, &v));

    for (uint32_t i = 0; i < 1000; i++) {                      /* wrap-around many times */
        CHECK(drdy_queue_push_from_isr(&q, i));
        CHECK(drdy_queue_pop(&q, &v) && v == i);
    }
    CHECK(drdy_queue_dropped(&q) == 1);

    /* random push/pop mix must match the pre-refactor ring exactly */
    drdy_queue_init(&q);
    ref_head = ref_tail = ref_dropped = 0;
    int bad = 0;
    for (int t = 0; t < 2000000; t++) {
        bool push = (rnd() % 100) < ((t / 50000) % 2 ? 70 : 40);   /* alternate fill/drain phases */
        if (push) {
            uint32_t ts = rnd();
            uint32_t before = ref_dropped;
            ref_isr(ts);
            bool ok = drdy_queue_push_from_isr(&q, ts);
            bad += ok != (ref_dropped == before);
        } else {
            uint32_t a = 0, b = 0;
            bool ra = ref_pop(&a), rb = drdy_queue_pop(&q, &b);
            bad += (ra != rb) || (ra && a != b);
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

int main(void)
{
    tbl_init();
    test_crc();
    test_packet();
    test_queue();
    test_driver();
    printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
