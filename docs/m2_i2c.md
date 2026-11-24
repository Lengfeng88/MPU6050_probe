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

## M2b: the read by DMA, with a timeout and bus recovery

State before M2b (`m2a`, `0x30961326`, blocking): the loop sits in `HAL_I2C_Mem_Read` for 429.5 us of every 9979 us; nothing can interrupt a read that hangs except the HAL's own 20 ms timeout.

What changed (code: `App/imu/imu_read.*`, `imu_read_stm32.*`, `i2c.c` USER CODE blocks, the loop in `main.c`):
- The main loop pops a DRDY event and starts `HAL_I2C_Mem_Read_DMA` (DMA1 stream 1, request I2C1_RX, buffer in AXI SRAM `.dma_buf`), then goes on. The I2C event / error interrupts and the DMA interrupt (all priority 2, as the UART DMA; EXTI3 stays at 0) finish it. One read in flight at a time; the rest wait in the DRDY queue.
- The state machine (`imu_read.c`) has no HAL in it and is tested on the host (`test_units.c`, `imu_read`): normal read, error, timeout (3000 us, i.e. about 7 times the read), the interrupt winning the race against the timeout, a callback arriving after the transfer was abandoned (counted as `spurious`, ignored), a start that fails, a storm of random orders.
- A read that fails (HAL error) or does not finish in 3000 us gives no packet: its sequence number is a hole on the Pi side (the number is assigned in the DATA_RDY ISR). Recovery runs in the main loop: `HAL_I2C_DeInit` (which switches the three IRQs off and de-initialises the DMA stream), clear pending IRQs, `i2c_bus_clear_gpio_always` (HAL_Delay based: about 5 ms on an idle bus, up to roughly 25 ms if SDA is held low and 9 clocks are needed: from the code, not measured), `MX_I2C1_Init`.
- During a recovery the loop does not run; EXTI3 does, so timestamps and sequence numbers stay right and up to 15 events wait in the queue (about 150 ms).
- D4 (`M0_I2C_PIN`) now goes high when the read starts and low in the completion interrupt (or at the timeout), so its width is the read itself, not the loop's pass.
- Counters (`g_imu_rd`, read with openocd): `mdw <address of g_imu_rd> 22`. Offsets in bytes: state 32 (one byte) and seq 34 (two bytes) share the word at 32, ts 36, t_start 40, started 44, completed 48, errors 52, timeouts 56, start_fail 60, recoveries 64, spurious 68, rejected 72, last_us 76, max_us 80 (start to completion interrupt, "maximum observed"), recover_us_max 84. The firmware build asserts these offsets (arm-none-eabi-gcc makes enums one byte; my first version of this line had been checked with clang's 4-byte enums and was off by 4 from byte 32 on). Interrupt cost: `g_imu_irq_calls` (32 bit), `g_imu_irq_cycles` (64 bit), `g_imu_irq_cycles_max`; the cycles include any EXTI3 that nests inside the handler (about 0.25 us each).
- Fault injection, opt-in: `-DM2B_FAULT_EVERY=N` (and `-DM2B_FAULT_AFTER_US=150`): after every N-th read the firmware waits 150 us and switches the I2C peripheral off in the middle of the transfer. The DMA never finishes and the sensor may be left holding SDA low, so the timeout and the bus clear are needed. `g_imu_faults_injected` counts them. It is a software fault, not an electrical one (a shorted or floating line is a different test).

### Predictions, written before the first measurement

These are estimates from a model of the extra interrupt entries, not from any measurement of this firmware; the numbers can be wrong.
- Read width (D4) 430 to 445 us: the same 155 clocks, plus the interrupt entry before the register byte and before the restart. `max_us` is a few us larger than the D4 width (it also counts the start call).
- Four interrupt calls per read (address/register byte, restart, DMA complete, STOP), `g_imu_irq_calls / started` = 4 (3 to 5 would be explained; much more would not), and 3 to 10 us of CPU in them per read (`g_imu_irq_cycles / started` = 1200 to 4000 cycles).
- So the loop is free for about 400 us of each read, and uses about 15 to 30 us of it (start call plus interrupts). Nothing else in the loop needs that time yet: the classifier needs the sample. This step buys a loop that can time out and recover, not lower latency.
- DRDY -> first UART start bit: p50 440 to 455 us (M2a: 437.6). DRDY -> ISR pin unchanged at 0.375 to 0.458 us.
- Window classification time: no prediction (it moved from 309 to 352 to 338 us with code layout; this build changes the layout again).
- Without fault injection over at least 10 000 events: `errors`, `timeouts`, `start_fail`, `spurious`, `rejected`, `recoveries` all 0, `started` = `completed` = `g_uart_tx.sent`, no packet lost on the Pi (`seq_gaps` 0).
- With `M2B_FAULT_EVERY=50` over 10 000 events (200 faults): `timeouts` = `g_imu_faults_injected`, `recoveries` = `timeouts`, `recover_us_max` between 4000 and 30 000 us, `g_drdy.dropped` 0, and the Pi sees `lost_events` = number of faults, each a single missing sequence number. The `ts_us` spacing across a hole is about 19 958 us (the timestamps are DRDY times, not delivery times).

### Procedure

1. Build as in M2a (flags `-DIMU_M0_PINS -DM0_ISR_PORT=GPIOH -DM0_ISR_PIN=GPIO_PIN_2 -DM0_I2C_PORT=GPIOH -DM0_I2C_PIN=GPIO_PIN_3`), flash, 25 captures merged, the counters, the Pi 120 s recording.
2. Rebuild with `-DM2B_FAULT_EVERY=50` added, flash, the counters and a Pi recording (about 120 s = 12 000 events = 240 faults).
3. Keep M2b only if the no-fault criteria hold and the fault run recovers every time.

## Results (M2b)

Build `m2b` (flash 29 636 B, AXI SRAM 544 B), Release `-Os`, 400 MHz, caches off, `Timing = 0x30961326`, probes on D5 / D6 as in M2a, board lying still, 2026-11-23.

### Without fault injection

Analyser, 25 captures merged, 10416 DRDY events (104 s):

| Quantity | min | p50 | max |
|----------|----:|----:|----:|
| SCL rising edges per I2C read (10439 reads) | 155 | 155 | 155 |
| SCL mean frequency during a read [kHz] | 362.994 | 363.172 | 363.208 |
| I2C read width, D4 [us] | 432.458 | 432.500 | 432.792 |
| DRDY -> D4 rises [us] | 1.250 | 1.500 | 1.792 |
| DRDY -> ISR pin [us] | 0.333 | 0.375 | 0.458 |
| DRDY -> classifier push start [us] | 442.083 | 442.375 | 443.000 |
| window classification width [us] (n = 105) | 315.500 | 315.583 | 315.875 |
| DRDY -> first UART start bit [us] | no data | no data | no data |

D2 (USART1 TX) showed no edges in this run or in a later single capture, while the counters and the Pi say the UART worked; the cause was not found (the same happened in the first M1 run). So the UART row is missing and the comparison with M2a is DRDY -> push start: 438.1 -> 442.4 us (+4.3 us, of which the read width is +3.0 us; the other 1.3 us was not taken apart).

Counters after the captures (read one after the other with openocd, about 1 s apart in total, so they are not from one instant):
- `g_imu_rd`: `started` = `completed` = 52 096; `errors`, `timeouts`, `start_fail`, `recoveries`, `spurious`, `rejected` all 0; `last_us` 432, `max_us` 433. `g_uart_tx`: `submitted` = `sent` = 52 204 (108 more, which is about the 1 s between the reads), `dropped`, `start_fail`, `errors` 0.
- Interrupts: 156 372 calls (3.00 per read), 123 722 458 cycles in them (about 2374 per read = 5.9 us; about 791 per call = 2.0 us), largest single call 1086 cycles = 2.7 us (includes an EXTI3 that nests inside it, if one did).

Pi, two 120 s recordings at 1 Mbaud: 12 031 and 12 030 frames, `crc_bad`, `resync_bytes`, `seq_gaps`, `lost_events` all 0, `ts_us` spacing 9979 to 9980 us (M2a: 9978 to 9979; the sensor clock had been drifting upward all session, not investigated). Compared with the 600 s M2a recording (`imu_compare`): means of ax, ay, az, gy within 0.22 sigma, no outliers in ax, ay, gx, gz, 2 in az (not listed); `gz` rose again, 76.2 -> 77.6 -> 83.9 -> 94.1 over the four 400 kHz / 100 kHz recordings while `temp` rose by about 3.8 deg C, which fits a temperature-dependent gyro offset (not tested).

### With fault injection (`-DM2B_FAULT_EVERY=50`, flash 29 736 B)

Board, 20 s after flashing: `started` 2024, `completed` 1984, `timeouts` 40, `recoveries` 40, `errors` / `start_fail` / `spurious` / `rejected` 0, `recover_us_max` 10 005 us, `max_us` 433, `g_drdy.dropped` 0; faults injected 41 (read about 0.3 s after the others: one more fault fits). Last bus clear: status 0 (idle bus), 0 clocks.

Pi, 120 s (`m2b-fault`): 11 788 frames, `crc_bad` 0, `resync_bytes` 0, 240 gaps of exactly one sequence number, every missing number is 49 mod 50 (the 50th, 100th ... read), the `ts_us` step across a gap is 19 958 to 19 959 us, sequence numbers 20 605 to 32 632: 11 788 + 240 = 12 028 = last - first + 1. So each injected fault lost its own event and nothing else: no event was lost to the recovery itself (a drop in the DRDY queue would show as an extra hole), and the stream went on after all 240 recoveries.

### Against the predictions

- Held: read width 430 to 445 us (432.5); 155 edges; 3 to 10 us of interrupt time (5.9); ISR latency unchanged; all no-fault counters 0 over 52 096 reads; fault run: `timeouts` = `recoveries` = injected faults, `recover_us_max` in 4000 to 30 000 us, `dropped` 0, `lost_events` = faults with one number each, gap step about 19 958 us.
- Missed: 4 interrupt calls per read were predicted, 3.00 were measured (inside the 3 to 5 tolerance I had written, but the point estimate was wrong, and I do not know which of the four does not happen).
- Not checkable: DRDY -> first UART start bit (D2 without data); the inferred value from the push start is in the predicted band.

### What this does not show

- The CPU time the loop gets back during a read is not measured: the interrupts take 5.9 us, but the duration of the start call (HAL setup, DMA start) was not timed. Nothing in the loop uses the freed time yet.
- Whether the fault run ever needed the bus clear's clock pulses: only the last recovery was seen (idle bus, 0 clocks). The counters `g_imu_rd_bus_idle`, `g_imu_rd_bus_recovered`, `g_imu_rd_bus_failed` and `g_imu_rd_clocks_max` were added after this run for that; see the next run.
- The injected fault is a switched-off peripheral, not an electrical fault (shorted or floating line, a slave that never releases SDA).
- DRDY arriving while a read is in flight: not tested here (the DRDY pulse never falls inside a read at 100 Hz); that needs the stall experiment (`M0_STALL_*`) on this build, which was not run.
- Window classification time is 315.6 us in this build; the classifier source did not change.
