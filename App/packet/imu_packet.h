#ifndef APP_PACKET_IMU_PACKET_H
#define APP_PACKET_IMU_PACKET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IMU sample packet, 24 bytes, sent over USART1.  All multi-byte header
 * fields are little-endian.  See docs/packet_protocol.md.
 *
 *   offset  size  field
 *   0       2     sync     0xAA 0x55
 *   2       2     seq      uint16, +1 per packet, wraps at 65536
 *   4       4     ts_us    uint32, TIM2 count (1 us) at the DATA_RDY edge
 *   8       14    raw      MPU-6050 regs 0x3B..0x48 verbatim (big-endian):
 *                          ax ay az temp gx gy gz, int16 each
 *   22      2     crc16    CRC-16/CCITT-FALSE over bytes 2..21
 */
#define IMU_PKT_SYNC0         0xAAu
#define IMU_PKT_SYNC1         0x55u

#define IMU_PKT_RAW_LEN       14u
#define IMU_PKT_LEN           24u

#define IMU_PKT_OFF_SYNC      0u
#define IMU_PKT_OFF_SEQ       2u
#define IMU_PKT_OFF_TS        4u
#define IMU_PKT_OFF_RAW       8u
#define IMU_PKT_OFF_CRC       22u
#define IMU_PKT_CRC_SPAN      20u   /* bytes 2..21 */

/**
 * Build one packet.  Pure function, no hardware dependency.
 *
 * @param out    receives IMU_PKT_LEN bytes
 * @param seq    sequence number
 * @param ts_us  timestamp in microseconds
 * @param raw    IMU_PKT_RAW_LEN bytes, copied verbatim
 */
void imu_packet_encode(uint8_t out[IMU_PKT_LEN], uint16_t seq, uint32_t ts_us,
                       const uint8_t raw[IMU_PKT_RAW_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* APP_PACKET_IMU_PACKET_H */
