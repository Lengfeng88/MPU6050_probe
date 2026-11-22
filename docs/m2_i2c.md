# M2: I2C read, from a blocking 100 kHz burst to DMA (plan, M2a first)

State before M2 (`m1-uart-dma`, `docs/m1_uart_dma.md`): `mpu6050_read_raw()` is a blocking `HAL_I2C_Mem_Read` at 100 kHz and takes 1596 us of every 9979 us (D4 pulse). It is now the largest block the main loop spends.
I2C1 runs from a 100 MHz kernel clock (`RCC.I2C123Freq_Value=100000000` in the `.ioc`), `Timing = 0x10C0ECFF`.

Order, each step measured against the one before:
- **M2a** 400 kHz, still blocking. One value changed (`Timing`), plus a way to see the bus (SCL on D5, SDA on D6 of the analyser).
- **M2b** DMA read, started by the main loop, with a timeout and bus recovery (the HAL DMA mode has none).
- **M2c** (optional) the highest sample rate each configuration sustains.

## M2a: the value

`tools/i2c_timing.py` (RM0433 formulas, written from memory of the manual: check them; unit-tested in `tests/py/test_i2c_timing.py`).
`python3 tools/i2c_timing.py --search 400e3` returns `0x30961326`: PRESC 3 (tPRESC 40 ns), SCLDEL 9, SDADEL 6, SCLH 19, SCLL 38; programmed low 1560 ns, high 800 ns (fast-mode minimums 1300 and 600 ns).
The search keeps the fastest case (rise and fall time 0, analogue filter 50 ns) at or below 400 kHz, which the MPU-6050 supports as far as I remember its datasheet (the PDF is in `docs/`: check it).
With an assumed rise and fall time of 100 ns the estimate is 318 to 370 kHz; the real bus (pull-ups, wire capacitance of the Fire module and its leads) is not known. `i2c.c` and the `.ioc` carry the same value so that regenerating the code does not undo it
(the old value, for going back to 100 kHz: `0x10C0ECFF`; the decoder gives 94 to 98 kHz for it).

Prediction, written before the measurement so that it can be wrong:
- the model on the current 100 kHz firmware: every I2C read has 153 SCL rising edges (address + register + repeated address + 14 data bytes, 9 clocks each) and a mean SCL frequency of about 92 to 98 kHz;
- 400 kHz firmware: still 153 edges per read, mean frequency about 310 to 370 kHz, and the D4 read pulse about 410 to 500 us (153 clocks of 2.7 to 3.1 us, plus the HAL's own setup).

## Procedure

1. Wire SCL (PB8) to D5 and SDA (PB9) to D6 (done). `tools/m0_latency.py` reads D5 by default (`--scl`), D6 is not used yet.
2. Step 1, the model against the firmware that is on the board (100 kHz): one capture, look at the two `SCL` rows.
3. Step 2, build and flash the 400 kHz firmware, 25 captures merged, then a 120 s Pi recording at 1 Mbaud (`imu_record.py`).

Keep the 400 kHz setting only if all of these hold: every read has 153 clocks in at least 10 000 events; the read width and the SCL frequency are inside the predicted band or the difference is explained; the Pi recording has
`crc_bad`, `resync_bytes`, `seq_gaps` and `lost_events` at 0 (a failed read is a skipped frame, i.e. a sequence gap); and a look at SCL / SDA at 400 kHz shows no obvious problem (the analyser samples at 24 MHz: 60 samples per
SCL period, enough for the frequency, not for the shape of the edges).

## Results

(not measured yet)
