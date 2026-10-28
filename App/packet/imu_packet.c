#include "packet/imu_packet.h"
#include "packet/crc16.h"

_Static_assert(IMU_PKT_OFF_SEQ == 2u && IMU_PKT_OFF_TS == IMU_PKT_OFF_SEQ + 2u,
               "seq is 2 bytes at offset 2, ts follows");
_Static_assert(IMU_PKT_OFF_RAW == IMU_PKT_OFF_TS + 4u, "raw follows the 4-byte ts");
_Static_assert(IMU_PKT_OFF_CRC == IMU_PKT_OFF_RAW + IMU_PKT_RAW_LEN, "crc follows raw");
_Static_assert(IMU_PKT_OFF_CRC + 2u == IMU_PKT_LEN, "crc is the last 2 bytes");
_Static_assert(IMU_PKT_CRC_SPAN == IMU_PKT_OFF_CRC - IMU_PKT_OFF_SEQ, "crc covers seq..raw");

void imu_packet_encode(uint8_t out[IMU_PKT_LEN], uint16_t seq, uint32_t ts_us,
                       const uint8_t raw[IMU_PKT_RAW_LEN])
{
    out[0] = IMU_PKT_SYNC0;
    out[1] = IMU_PKT_SYNC1;

    out[IMU_PKT_OFF_SEQ + 0] = (uint8_t)(seq);
    out[IMU_PKT_OFF_SEQ + 1] = (uint8_t)(seq >> 8);

    out[IMU_PKT_OFF_TS + 0] = (uint8_t)(ts_us);
    out[IMU_PKT_OFF_TS + 1] = (uint8_t)(ts_us >> 8);
    out[IMU_PKT_OFF_TS + 2] = (uint8_t)(ts_us >> 16);
    out[IMU_PKT_OFF_TS + 3] = (uint8_t)(ts_us >> 24);

    for (unsigned i = 0; i < IMU_PKT_RAW_LEN; i++)
        out[IMU_PKT_OFF_RAW + i] = raw[i];

    uint16_t crc = crc16_ccitt(&out[IMU_PKT_OFF_SEQ], IMU_PKT_CRC_SPAN);
    out[IMU_PKT_OFF_CRC + 0] = (uint8_t)(crc);
    out[IMU_PKT_OFF_CRC + 1] = (uint8_t)(crc >> 8);
}
