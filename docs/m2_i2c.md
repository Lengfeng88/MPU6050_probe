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

The 153 was a miscount; step 1 below gave 155 and explains the two extra rises. The frequency prediction held. The 400 kHz prediction is revised after step 1:
- 155 rising edges per read;
- nominal SCL period = programmed 2360 ns + 390 ns of sync delay measured at 100 kHz = about 2.75 us, i.e. about 360 kHz (340 to 370 kHz); mean frequency over a read a little lower because of the repeated-start gap (330 to 365 kHz);
- D4 read pulse about 420 to 470 us (17 us of HAL setup and teardown plus 154 periods; the repeated-start gap grows less than the clocks shrink).
This revision uses the sync delay of the 100 kHz capture, so it is not an independent prediction.

## Procedure

1. Wire SCL (PB8) to D5 and SDA (PB9) to D6 (done). `tools/m0_latency.py` reads D5 by default (`--scl`), D6 is not used yet.
2. Step 1, the model against the firmware that is on the board (100 kHz): one capture, look at the two `SCL` rows.
3. Step 2, build and flash the 400 kHz firmware, 25 captures merged, then a 120 s Pi recording at 1 Mbaud (`imu_record.py`).

Keep the 400 kHz setting only if all of these hold: every read has 155 clocks in at least 10 000 events; the read width and the SCL frequency are inside the predicted band or the difference is explained; the Pi recording has
`crc_bad`, `resync_bytes`, `seq_gaps` and `lost_events` at 0 (a failed read is a skipped frame, i.e. a sequence gap); and a look at SCL / SDA at 400 kHz shows no obvious problem (the analyser samples at 24 MHz: 60 samples per
SCL period, enough for the frequency, not for the shape of the edges).

## Results

### Step 1: the 100 kHz firmware (M1 build `b86bd15`) with D5 / D6 attached, 2026-11-22, one capture of 4.167 s, 417 reads

| Quantity | min | p50 | max |
|----------|----:|----:|----:|
| SCL rising edges per I2C read | 155 | 155 | 155 |
| SCL mean frequency during a read [kHz] | 97.322 | 97.325 | 97.327 |
| I2C read width, D4 [us] | 1599.208 | 1599.396 | 1599.667 |

One read looked at edge by edge (`/tmp/cap.bin`, the sixth read): median SCL period 10.25 us (97.6 kHz); first rising edge 11.12 us after the D4 pulse starts, last 5.96 us before it ends.
SDA sampled at the rising edges reads `110100000` for edges 0 to 8 (address 0xD0, write, then the ACK), `001110110` for 9 to 17 (register 0x3B, ACK), `1` at edge 18 followed by a 15.5 us gap
(the repeated start is being prepared), then `110100010` for 19 to 27 (address 0xD1, read, ACK). So 155 = 18 + 1 + 9 + 126 + 1: 153 byte clocks, one rise for the repeated start and one more, at the end, which I take to be the
rise before the STOP (the last byte is NACKed with SDA high, so SDA must go low before the stop); that last one is inferred from the count, SDA was not looked at there.

Reading it:
- The model's frequency prediction held (92 to 98 kHz). The nominal period 10.25 us is the programmed 9.86 us plus about 0.39 us of sync delay: the fast end of the estimate (10.2 to 10.64 us), so this bus has faster edges than the 100 ns I assumed.
- With D5 and D6 attached the read is 1599.4 us, against 1596.3 us in every capture without them (runs 2 and 4 of `m0_baseline.md`, `m1_uart_dma.md`): 3.1 us longer, which is 155 clocks of about 20 ns. A slower rising edge from the extra capacitance of
  the two probes would do that (a hypothesis, not tested). At 400 kHz the probes matter more; the 100 kHz and 400 kHz numbers of this document are with the probes on.

### Step 2: the 400 kHz firmware (`Timing = 0x30961326`, flash 24584 B, probes on D5 / D6), 2026-11-22

Analyser, 25 captures merged, 10416 DRDY events (104 s), same pins and flags as M1:

| Quantity | min | p50 | max |
|----------|----:|----:|----:|
| SCL rising edges per I2C read (10440 reads) | 155 | 155 | 155 |
| SCL mean frequency during a read [kHz] | 363.315 | 363.458 | 363.493 |
| I2C read width, D4 [us] | 429.125 | 429.500 | 429.833 |
| DRDY -> ISR pin [us] | 0.375 | 0.417 | 0.458 |
| DRDY -> first UART start bit [us] | 437.208 | 437.583 | 438.042 |
| DRDY -> classifier push start [us] | 437.708 | 438.083 | 438.542 |
| window classification width [us] (n = 105) | 338.083 | 338.125 | 338.375 |

Counters after the 25 captures: `submitted` = `sent` = 43111, `dropped`, `start_fail`, `errors` all 0. 10311 pushes + 105 windows = 10416 events.

Against the step 1 / M1 numbers (the probes are on in both, so the comparison is fair): read width 1599.4 -> 429.5 us (-1169.9 us); DRDY -> push start 1604.5 -> 438.1 us; from DRDY to the start of the push the loop takes 438 us of each 9979 us period (4.4 %); in M1 the read alone took 16 % (1599 / 9979). The push, 1 us, and the window classification (every 100th event, 338 us) come on top; the loop's total busy time was not measured.

The predictions: 155 edges held; 363.4 kHz is inside the revised 330 to 365 kHz band, at its top; the D4 pulse of 429.5 us is inside the revised 420 to 470 us. The first prediction of "about 450 to 500 us" given in chat was too high (it was not written to this file).

The window classification is 338 us here against 352 us in M1 and 309 us in the reference build. The only change between M1 and M2a is a 32-bit constant (Flash +4 B). That points at code placement with the instruction cache off, but that is a hypothesis and not tested.

Pi side (`imu_record.py`, 1 Mbaud, board lying still; the Pi is `Xu`):
- 120 s: 12031 frames, `crc_bad`, `resync_bytes`, `seq_gaps`, `lost_events` all 0, `ts_us` spacing 9978 to 9979 us.
- 600 s (`m2a-400k-long`, started 5 s after the key press): 60134 frames, the same four counters 0.

The CRC is computed on the MCU after the I2C read, so it cannot see a bit error on the bus. `tools/imu_compare.py` compares a still recording with the earlier 100 kHz one (`m1-linktest`, 12032 samples) channel by channel; an outlier is more than 6 robust sigma (MAD) from the median.
- The 120 s recording started with a burst (0.09 to 0.26 s, ax / ay / az / gx at the same samples, alternating sign): a disturbance of the desk, not a bus error, in my reading (the cause, a key press, is a guess).
- The 600 s recording has 95 outliers in 4 events. Three events, at 230.9, 233.2 and 235.0 s, are one disturbance of about 5 s with several channels moving together. The fourth, at 507.27 s, is two outliers (az -234, ay -176) 30 samples apart.
- The 100 kHz recording has 4 outliers in 1 event (ay, ax, 51 samples apart).
- Left unexplained: three isolated single-channel samples at 400 kHz, idx 343 of the 120 s recording (az -256) and the two of the 507 s event, in about 72000 samples. None of them is a clean power of two (the median has noise of about 35 counts, so -256 is not evidence). The 100 kHz recording has 12032 samples, so at that rate about 0.5 would be expected: it cannot tell the two bus speeds apart.
- Means: ax and ay differ by 25 to 29 counts between the 100 kHz and the 400 kHz recordings (placement of the board between the recordings was not controlled), `temp` rose about 640 counts (about 1.9 deg C) over the 600 s run and `gz` moved 7.7 counts (1.8 sigma), consistent with a temperature-dependent gyro offset (not tested).

What this does not show: it finds only large errors; a flipped low bit is hidden in the sensor noise. SCL and SDA were not looked at for edge shape at 400 kHz (24 MHz sampling, 60 samples per period: enough for the frequency only). The 400 kHz setting is kept, with "no evidence of bus bit errors, three unexplained samples" as the stated result, not "no errors".

Not verified: the probe-loading explanation of the extra 3.1 us at 100 kHz; the nominal 400 kHz of the MPU-6050 and the rise time of this bus against the I2C fast-mode limits (datasheet and a scope are needed for that; the analyser cannot show it).
