#ifndef APP_PACKET_IMU_RESULT_H
#define APP_PACKET_IMU_RESULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Classifier result frame, 26 bytes, sent over USART1 once per completed
 * window.  All multi-byte fields are little-endian.  See
 * docs/packet_protocol.md ("Result frame").
 *
 *   offset  size  field
 *   0       2     sync          0xAA 0x5A  (sample frames use 0xAA 0x55)
 *   2       2     win_seq       uint16, +1 per completed window
 *   4       2     seq_last      uint16, seq of the last sample in the window
 *   6       4     ts_last_us    uint32, ts_us of that sample
 *   10      4     ts_done_us    uint32, TIM2 count when the class was ready
 *   14      1     cls           class index (see imu_tree.h)
 *   15      1     flags         bit 0: tree trained on simulated data
 *   16      2     tree_id       uint16, identifies the tree in the firmware
 *   18      2     uart_dropped  uint16, g_uart_tx.dropped (saturating)
 *   20      4     cycles        uint32, DWT cycles of this window
 *   24      2     crc16         CRC-16/CCITT-FALSE over bytes 2..23
 */
#define IMU_RES_SYNC0         0xAAu
#define IMU_RES_SYNC1         0x5Au
#define IMU_RES_LEN           26u
#define IMU_RES_CRC_SPAN      22u   /* bytes 2..23 */

#define IMU_RES_FLAG_SIMULATED_TREE  0x01u

typedef struct {
    uint16_t win_seq;
    uint16_t seq_last;
    uint32_t ts_last_us;
    uint32_t ts_done_us;
    uint8_t  cls;
    uint8_t  flags;
    uint16_t tree_id;
    uint16_t uart_dropped;
    uint32_t cycles;
} imu_result_t;

/** Build one result frame.  Pure function, no hardware dependency. */
void imu_result_encode(uint8_t out[IMU_RES_LEN], const imu_result_t *r);

#ifdef __cplusplus
}
#endif

#endif /* APP_PACKET_IMU_RESULT_H */
