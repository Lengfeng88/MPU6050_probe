#ifndef APP_PACKET_CRC16_H
#define APP_PACKET_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, MSB first, no reflection,
 * no final XOR.  Check value: crc16_ccitt("123456789", 9) == 0x29B1.
 *
 * Pure function with no hardware dependency, so it builds on the host.
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* APP_PACKET_CRC16_H */
