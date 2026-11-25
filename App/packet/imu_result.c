#include "packet/imu_result.h"
#include "crc16.h"

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)(v >> 24);
}

void imu_result_encode(uint8_t out[IMU_RES_LEN], const imu_result_t *r)
{
    out[0] = IMU_RES_SYNC0;
    out[1] = IMU_RES_SYNC1;
    put16(&out[2], r->win_seq);
    put16(&out[4], r->seq_last);
    put32(&out[6], r->ts_last_us);
    put32(&out[10], r->ts_done_us);
    out[14] = r->cls;
    out[15] = r->flags;
    put16(&out[16], r->tree_id);
    put16(&out[18], r->uart_dropped);
    put32(&out[20], r->cycles);
    put16(&out[24], crc16_ccitt(&out[2], IMU_RES_CRC_SPAN));
}
