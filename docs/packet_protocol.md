# IMU packet protocol

One-way stream, MCU to host, over USART1 at 115200 baud (`tools/parse_imu.py` opens the
port with pyserial defaults, i.e. 8N1). One packet per MPU-6050 DATA_RDY interrupt,
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

Limit: the EXTI pending flag is a single bit, not a counter. If interrupts are masked or the
core is halted (debugger) for longer than one sample period, several DATA_RDY events collapse
into ONE interrupt and `seq` advances by only 1. Such a stall is visible only as a gap in
`ts_us`. Verified on hardware with a debugger halt: `dt max` ~264 ms, `seq_gaps=0`.
A main-loop stall with interrupts enabled (queue full) is visible in `seq`: temporary 300 ms
stall every 500 events gave `seq_gaps=4`, `lost=60` (15 per stall, queue capacity 15).

History: before this change `seq` was assigned after the I²C read succeeded (+1 per packet), so
drops and failed reads were invisible in `seq`; see `tests/host/run_glue.py`, scenarios `burst`
and `readfail`, for both behaviours side by side.
