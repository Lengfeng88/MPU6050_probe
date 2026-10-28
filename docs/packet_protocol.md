# IMU packet protocol

One-way stream, MCU to host, over USART1 at 115200 baud (`tools/parse_imu.py` opens the
port with pyserial defaults, i.e. 8N1). One packet per MPU-6050 DATA_RDY interrupt,
100 Hz (`SMPLRT_DIV = 9`, `DLPF_CFG = 3`).

## Frame (24 bytes)

| offset | size | field   | notes |
|-------:|-----:|---------|-------|
| 0      | 2    | sync    | `0xAA 0x55` |
| 2      | 2    | `seq`   | uint16 LE, +1 per packet, wraps at 65536 |
| 4      | 4    | `ts_us` | uint32 LE, TIM2 count (1 µs per tick) captured in the DATA_RDY ISR; wraps every ~71.6 min |
| 8      | 14   | `raw`   | MPU-6050 registers `0x3B..0x48` verbatim: `ax ay az temp gx gy gz`, each int16 **big-endian** |
| 22     | 2    | `crc16` | uint16 LE, CRC-16/CCITT-FALSE over bytes 2..21 |

CRC-16/CCITT-FALSE: poly `0x1021`, init `0xFFFF`, MSB first, no reflection, no final XOR.
Check value: `crc16("123456789") = 0x29B1`.

Implementation: `App/packet/imu_packet.[ch]`, `App/packet/crc16.[ch]`.

## Decoding notes

* The device is left at its reset full-scale ranges (±2 g, ±250 °/s): 16384 LSB/g and
  131 LSB/(°/s). The driver does not write `ACCEL_CONFIG` / `GYRO_CONFIG`.
* Temperature in °C = `temp / 340 + 36.53`.
* Host resync (as in `parse_imu.py`): search for `AA 55`; if the CRC fails, drop 2 bytes and
  search again.

## What `seq` can and cannot detect

`seq` is assigned when a packet is built, after the I²C burst read succeeded. So:

* A DATA_RDY event that was **dropped because the ISR queue was full**, or whose **I²C read
  failed**, never gets a `seq`. It is invisible in `seq` and shows up only as a gap in `ts_us`
  (a multiple of ~10 ms).
* `seq` gaps therefore reveal only packets lost after encoding (UART side, host side).
* The queue's `dropped` counter is not transmitted.

Reproduced in the host simulation (`tests/host`, scenarios `burst` and `readfail`).
