# M1: UART TX by DMA at 1 Mbaud, measured against the M0 reference

Firmware: `ff71532` (`realtime-baremetal`), built with the same M0 pins as the reference (`-DIMU_M0_PINS -DM0_ISR_PORT=GPIOH -DM0_ISR_PIN=GPIO_PIN_2 -DM0_I2C_PORT=GPIOH -DM0_I2C_PIN=GPIO_PIN_3`, no stall),
Release `-Os`, 400 MHz, caches off, USART1 at 1 000 000 baud with TX by DMA1 stream 0, MPU-6050 at 100 Hz, flash 24 580 B (reference: 18 940 B), AXI SRAM 512 B (the 16 x 32 B frame slots).
Reference: tag `m0-reference`, `docs/m0_baseline.md` runs 2 and 4. Measured on 2026-11-22 with `tools/m0_latency.py`, 25 captures of 4.167 s merged (the second 25-capture run of this firmware; see the note on D2 below). Board untouched, at rest and any Pi activity during the captures not recorded.

## Logic analyser, 25 captures, 10415 events (maximum OBSERVED values)

| Quantity | reference (run 4): p50 / max | M1: p50 / max |
|----------|------:|------:|
| DRDY to ISR pin [us] | 0.417 / 0.458 | 0.375 / 0.417 |
| ISR pin width [us] | 0.250 / 0.292 | 0.250 / 0.292 |
| DRDY to I2C start [us] | 1.042 / 1.333 | 0.958 / 1.208 |
| I2C read width [us] | 1596.292 / 1596.500 | 1596.292 / 1596.500 |
| DRDY to first UART start bit [us] | 1610.292 / 1610.667 | 1604.042 / 1604.292 |
| DRDY to per-sample push start [us] | 3694.125 / 3694.458 | 1604.542 / 1604.833 |
| per-sample push width [us] | 1.083 / 1.375 | 1.083 / 1.375 |
| window classification width [us] (n = 108 / 104) | 309.083 / 309.333 | 352.083 / 352.458 |
| DRDY period, analyser clock [us] | 9979.375 / 9979.458 | 9979.542 / 9979.625 |

Consistency: 10311 per-sample pushes + 104 window classifications = 10415 = events; every DRDY had an ISR pulse; 4 DRDY edges at the capture ends found no I2C read.

Reading it:
- The loop no longer waits for the UART. The push now starts 1604.5 us after DRDY instead of 3694.1 us. That is 2089.6 us earlier, about the 2083 us the 24 bytes took at 115 200 baud plus a few us.
  The CPU is busy about 1.6 ms of every 9.98 ms (16 %) instead of about 3.7 ms (37 %); what is left is the 1.6 ms blocking I2C read.
- The UART start bit appears 6.8 us after the I2C read ends (1604.04 against 1597.25 us: packet encode, CRC, queueing and starting the DMA) and the push 0.5 us after that; before, the start bit came 13 us after the read and the push waited for the frame.
- The ISR latency did not change (0.375 to 0.417 us, the same two samples as before). The one-sample difference of the p50 between builds is not a result.
- **The window classification is 352 us in this build**, 14 % above the reference 309 us, from the same classifier source. With runs 3 and 5 of `m0_baseline.md` that makes 309, 335 and 352 us in three builds,
  and README's 318.8 us a fourth point. The cause is not established. Code layout with the instruction cache off is a candidate; enabling the I-cache (and the D-cache with an MPU region for the DMA buffer)
  is an experiment to try later, measured, not a conclusion.

## Counters (read with openocd, `g_uart_tx`)

| read | submitted | sent | dropped | start_fail | errors |
|------|----------:|-----:|--------:|-----------:|-------:|
| about 15 s after the first flash | 1549 | 1549 | 0 | 0 | 0 |
| after the first 25-capture run of this firmware (13 captures completed) | 54573 | 54573 | 0 | 0 | 0 |
| after the first full 25 captures | 82406 | 82406 | 0 | 0 | 0 |
| after the 25 captures of this table | 153172 | 153172 | 0 | 0 | 0 |

The board was powered up on 2026-11-22 (it had been off overnight, not running); the last three rows are from that power-on: 153 172 frames is about 25 minutes at 100 Hz, which is the whole span the last row covers. The first row is from the previous day, before the power-off.

## Pi link test (Pi 5, `/dev/ttyAMA0`, 1 Mbaud, `tools/imu_record.py`, 120 s)

12 032 frames, `crc_bad` 0, `resync_bytes` 0, `seq_gaps` 0, `lost_events` 0, `ts_us` difference between consecutive frames 9978 to 9979 us (the 1 us tick of TIM2).
Mean period on the MCU's clock: 120.049643 s / 12 031 = 9978.36 us (100.217 Hz). The analyser's merged period was 9979.25 us (13 captures), 9979.33 us and 9979.54 us in the three runs of the following 25 minutes:
0.9 to 1.2 us (90 to 120 ppm) above the MCU's figure and drifting upward in that session. Both clocks are crystals with a tolerance of that size and the sensor's clock can drift with temperature, so this data does not say which is closer to the truth.

## Not verified / gaps

- D2 (PA9) showed no edge at all in an earlier 25-capture run of this firmware (a 1 s check gave 0 edges, 100 % high) while the Pi had received the stream error-free minutes before and the counters kept increasing. The cause was not established (a loose analyser clip is the candidate; what was done to restore the channel was not recorded). In the run of the table above the channel shows frames again, and every other row of the two runs agrees to within one or two samples.
- The 240 us frame time at 1 Mbaud is arithmetic, not measured: the tool reports the start bit only.
- No stall / overlap experiment was run on the M1 build. The DMA interrupt runs at priority 2 and masks nothing that EXTI3 (priority 0) needs; that is by construction (BASEPRI) and not yet measured with DRDY arriving during a DMA completion.
- The link test covers 120 s without load or Pi-side activity; the counters cover about 14 minutes.
- Window classification time is build-specific (see above); the 352 us is for `ff71532` with these flags only.
