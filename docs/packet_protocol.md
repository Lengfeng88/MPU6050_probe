# IMU packet protocol

One-way stream, MCU to host, over USART1 at 1,000,000 baud (`tools/parse_imu.py` opens the
port with pyserial defaults otherwise, i.e. 8N1; the firmware sent at 115200 before commit "UART TX through DMA"). One packet per MPU-6050 DATA_RDY interrupt,
100 Hz (`SMPLRT_DIV = 9`, `DLPF_CFG = 3`).

## Frame (24 bytes)

| offset | size | field   | notes |
|-------:|-----:|---------|-------|
| 0      | 2    | sync    | `0xAA 0x55` |
| 2      | 2    | `seq`   | uint16 LE, DATA_RDY event number (+1 per interrupt), wraps at 65536 |
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

## What `seq` detects

`seq` numbers DATA_RDY **events**: it is assigned in the EXTI ISR (`App/sampling/drdy_queue.c`),
one number per interrupt, whether or not the event fits into the ISR queue. So:

* an event **dropped because the queue was full**, or whose **I²C read failed**, leaves a hole in
  `seq` (and a gap of a multiple of ~10 ms in `ts_us`);
* packets lost on the UART or on the host also leave a hole;
* `seq` wraps at 65536; a gap of N numbers means N events produced no packet.

`seq` cannot say *why* an event is missing (queue full / read failed / link loss); `ts_us` and the
gap length help: a burst of several consecutive missing numbers points at a stalled main loop.
The queue's `dropped` counter is still not transmitted.

History: before this change `seq` was assigned after the I²C read succeeded (+1 per packet), so
drops and failed reads were invisible in `seq`; see `tests/host/run_glue.py`, scenarios `burst`
and `readfail`, for both behaviours side by side.

## Result frame (26 bytes) - DRAFT

Sent once per completed classifier window (every 100 samples, about 1 Hz), on the same USART1 stream as
the sample frames. The sample frame above is **unchanged**; the result frame has its own sync bytes, so
recorded sessions, `frames.bin` and the existing tools keep working.

| offset | size | field          | notes |
|-------:|-----:|----------------|-------|
| 0      | 2    | sync           | `0xAA 0x5A` (sample frames use `0xAA 0x55`) |
| 2      | 2    | `win_seq`      | uint16 LE, +1 per completed window, wraps at 65536. A hole means a result frame was lost (windows that never complete after a `seq` hole do not use a number) |
| 4      | 2    | `seq_last`     | uint16 LE, `seq` of the last sample in the window. The window is the 200 consecutive `seq` values ending here (a window never spans a `seq` hole) |
| 6      | 4    | `ts_last_us`   | uint32 LE, `ts_us` of that last sample |
| 10     | 4    | `ts_done_us`   | uint32 LE, TIM2 count (1 us) when the class was ready. `ts_done_us - ts_last_us` (mod 2^32) is the MCU-side latency, no clock sync needed |
| 14     | 1    | `cls`          | uint8, class index as in the header comment of `App/classify/imu_tree.h` |
| 15     | 1    | `flags`        | bit 0: tree was trained on simulated data; other bits 0 |
| 16     | 2    | `tree_id`      | uint16 LE, identifies the tree in the firmware (CRC-16 of the generated tree data, written into `imu_tree.h` by `imu_train.py`), so a prediction can be compared with the right Python tree |
| 18     | 2    | `uart_dropped` | uint16 LE, `g_uart_tx.dropped`, cumulative, saturating |
| 20     | 4    | `cycles`       | uint32 LE, DWT cycles of this window's feature + tree computation |
| 24     | 2    | `crc16`        | uint16 LE, CRC-16/CCITT-FALSE over bytes 2..23 |

Test vector (`win_seq=1, seq_last=299, ts_last_us=3000000, ts_done_us=3003686, cls=2, flags=1,
tree_id=0x1234, uart_dropped=0, cycles=127519`):

```
aa 5a 01 00 2b 01 c0 c6 2d 00 26 d5 2d 00 02 01 34 12 00 00 1f f2 01 00 df 9e
```

### Host decoding

* Search for either sync (`AA 55` or `AA 5A`), take the earliest, read 24 or 26 bytes accordingly,
  check the CRC; on failure drop 2 bytes and search again (same rule as for sample frames).
* A decoder that only knows sample frames still gets every sample: it skips the result frame
  and counts its 26 bytes as `resync_bytes` (checked in a prototype on 20,000 samples with 199 results).
  Only the old tools' `resync_bytes` statistic becomes non-zero; update the tools to
  separate the two frame types.
* `FrameDecoder` keeps yielding only sample `Frame`s; result frames are collected in a separate list
  and counted separately (`result_frames`, `result_crc_bad`), so `imu_record.py` and `imu_convert.py`
  do not change. Recorded result frames go to their own file next to `frames.bin`.

### What the result frame tells the host

* Missing results: gaps in `win_seq`. Missing samples: gaps in `seq` (unchanged).
* Where the window is: `seq_last`, so the host can recompute the 17 features and the tree from the raw
  samples and compare with `cls` for the same window.
* MCU-side latency per window and the cost of the computation: `ts_done_us - ts_last_us`, `cycles`.
* Whether the MCU is dropping frames: `uart_dropped`.
