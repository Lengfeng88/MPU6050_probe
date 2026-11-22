# M0: bare-metal baseline

Goal: numbers for the CURRENT firmware (blocking I2C, blocking UART, classifier in the main loop) before anything is
changed, so that later steps (UART DMA, I2C DMA, FreeRTOS) can be compared against them. Baseline firmware = commit
`1c350ee` plus the M0 pins (this patch). Do not flash the UART-DMA / 1 Mbaud patch for this measurement: it changes the
timing being measured, and the Pi tools would need `--baud 115200` for the baseline firmware.

## Wiring (8-channel logic analyzer, README: sigrok `fx2lafw`, 24 MHz, about 42 ns per sample)

| Channel | Signal | Source |
|---------|--------|--------|
| D0 | classifier running | PB0, already in the firmware |
| D1 | MPU-6050 INT = DRDY | PC3, already in the README wiring |
| D2 | USART1 TX | PA9 (idles high; first falling edge of a frame = start bit) |
| D3 | DATA_RDY callback running | NEW, a free pin of YOUR board |
| D4 | `mpu6050_read_raw()` running | NEW, a free pin of YOUR board |
| GND | common ground | |

Suggestion for the Fire (野火) H743X core board + base board, read from their schematics (not tested on the board): **PH2 = D3, PH3 = D4** (base board header CN4 pins 54 and 52). On the base board they go nowhere but the header; on the core board they are the IO0/IO1 data lines of the second QSPI flash, whose CE# (PG6) is pulled high, and this firmware never touches QUADSPI. Build flags: `-DIMU_M0_PINS -DM0_ISR_PORT=GPIOH -DM0_ISR_PIN=GPIO_PIN_2 -DM0_I2C_PORT=GPIOH -DM0_I2C_PIN=GPIO_PIN_3`. Note PB0 (D0) is also the LED_R pin on that board, so the LED flickers.

D3/D4 have no default: pick two header pins that nothing else on the board uses, then build with
`-DIMU_M0_PINS -DM0_ISR_PORT=GPIOx -DM0_ISR_PIN=GPIO_PIN_n -DM0_I2C_PORT=GPIOx -DM0_I2C_PIN=GPIO_PIN_n`
(e.g. through `CMAKE_C_FLAGS`). Without `IMU_M0_PINS` nothing is compiled in; with it but without the four macros the build stops
with an `#error`. Edges are single BSRR writes.

## Capture and analysis

```bash
sigrok-cli -d fx2lafw --config samplerate=24m --samples 100m -O binary -o /tmp/cap.bin    # ~4 s: ~400 DRDY events
python3 tools/m0_latency.py /tmp/cap.bin --rate 24e6 --json m0.json
```

100 events are too few for p99; for p99.9 take at least 10,000 events (about 100 s of capture, several files) and say so.
`m0_latency.py` keeps only the small per-event numbers of each capture, so many captures can be merged without keeping the raw files:

```bash
for i in $(seq 1 25); do
  sigrok-cli -d fx2lafw --config samplerate=24m --samples 100m -O binary -o /tmp/cap.bin
  python3 tools/m0_latency.py /tmp/cap.bin --events-out /tmp/ev_$i.npz >/dev/null && rm /tmp/cap.bin
done
python3 tools/m0_latency.py /tmp/ev_*.npz --json m0.json        # 25 x 4.2 s, about 10,000 events
```

PB0 (D0) is high around EVERY `imu_activity_push_raw()` (a few us) and for about 300 us when that push completes a 2 s window. The tool
splits the two by pulse width (`--long-us`, default 50). (An earlier version of this document said the pulse comes once per second; that was wrong.)

## The four experiments

| # | Question | Measured by | Notes |
|---|----------|-------------|-------|
| 1 | Actual sampling period and its spread | D1 rising-to-rising | The period comes from the MPU-6050's own clock (README: mean 9979.2 us, i.e. 100.2 Hz), not from the MCU crystal. |
| 2 | Delay DRDY edge to timestamp | D1 to D3 | = the bias of `ts_us`. It contains NVIC entry, `HAL_GPIO_EXTI_IRQHandler` and the callback prologue (caches are off, code runs from Flash). Under load it is the interesting number, but at the stock 100 Hz it is never loaded: every DRDY arrives while the loop is idle (runs 1 and 2, the loop is busy ~3.7 ms of 10). The overlap experiment below creates the load. |
| 3 | Processing chain | D1 to D4 (I2C start), D4 width (I2C), D1 to D2 (first UART bit), D1 to D0 (classifier start) | Per-stage cycle counts still come from DWT (`g_act_cycles_*`). |
| 4 | Sensor to Pi | partly | The physical link to the MCU's TX pin is measured here. Pi receive time needs the MCU/Pi clock mapping (M6); until then `imu_record.py` timestamps are batch times (`ser.read(512)` with a 0.1 s timeout) and must not be used as latency numbers. |

Conditions to write next to every result: build type (Release `-Os`), clock 400 MHz, caches off, baud, what the sensor was doing
(still / shaking), whether `imu_record.py` was running on the Pi, capture length and number of events. The maximum is a
maximum OBSERVED execution time, not a WCET.

## Overlap experiment: DRDY while the main loop is busy

Runs 1 and 2 only cover the idle case. `m0_pins.h` has an opt-in stall: after every `M0_STALL_EVERY`-th event the main loop busy-waits `M0_STALL_US`
microseconds (DWT cycle counter), optionally ramped (`M0_STALL_STEP_US`, `M0_STALL_STEPS`: S, S+D, ... S+(N-1)D, then again). Later DRDY edges then arrive while the loop is
blocked, and the queued events are handled back to back, so a DRDY can land inside an I2C read or inside the UART transmit. Nothing changes unless these macros are defined.

```bash
cmake --preset Release -DCMAKE_C_FLAGS="-DIMU_M0_PINS -DM0_ISR_PORT=GPIOH -DM0_ISR_PIN=GPIO_PIN_2 -DM0_I2C_PORT=GPIOH -DM0_I2C_PIN=GPIO_PIN_3 \
  -DM0_STALL_US=8000 -DM0_STALL_EVERY=10 -DM0_STALL_STEP_US=250 -DM0_STALL_STEPS=49"
cmake --build --preset Release          # flash as usual; stalls of 8.0, 8.25 ... 20.0 ms, one every 10 events (every 0.1 s)
```

The flags stay in the CMake cache: to go back to the stock M0 build, configure again with the first line only (no `M0_STALL_*`). Capture exactly as in
"Capture and analysis" (25 captures, merged). The ramp is there to sweep the phase; with run 2's numbers (DRDY at 0, I2C 1.0 to 1597 us, UART tx until about 3.69 ms, stall starting
after the push) the second DRDY after the stall lands in the UART transmit for a stall of about 12.6 to 14.7 ms and in the I2C read for about 14.7 to 16.3 ms.
That is arithmetic from run 2, not a measurement; the tool reports what actually happened. Window classifications are NOT targeted by this stall (they need a stall
aligned to the window event); that case is still open.

New rows in the table: `DRDY -> I2C start, paired in order` is the loop's latency (it is large here; the other `DRDY -> ...` rows pair each DRDY with the first
pulse before the next DRDY and mislead once the loop falls behind, so read them only from the idle runs). The three `DRDY -> ISR pin, DRDY inside ...` rows are the ISR latency of
exactly those DRDY edges that arrived while the loop was in an I2C read, in the UART transmit / packet encode, or in a window classification (taken from the capture
itself). Prediction to check, not a result: they stay at 0.375 to 0.46 us, because EXTI3 has priority 0 and the BASEPRI critical sections do not mask it. If they do not,
something masks it (PRIMASK, a HAL call) and that is the finding.

Assumptions of the in-order pairing: no event is dropped (stalls of up to 20 ms are far below the 15-slot queue, about 150 ms). The tool prints how many DRDY edges at the end of a
capture found no I2C read; a few per capture is just the capture ending, more means drops (a drop shifts every later pair by one period).

## Results

### Run 1: 2026-11-21, `realtime-baremetal` at `1bfa6c2` (baseline firmware + M0 pins on PH2/PH3), first look

Conditions: Release `-Os`, 400 MHz, caches off, USART1 at 115200 baud, MPU-6050 at 100 Hz, board at rest (as instructed; not independently
checked), 24 MHz capture, one capture of 4.167 s, 417 analysed DRDY events. **Too few events for p99 / p99.9** (they equal the max). Numbers from
the first version of `tools/m0_latency.py`; its classifier row is the per-sample push (see below), not the window classification.

| Quantity | min | p50 | max |
|----------|----:|----:|----:|
| DRDY period | 9.979 ms | 9.979 ms | 9.979 ms (printed to 1 us: no spread visible) |
| DRDY to ISR pin (bias of `ts_us`) | 0.375 us | 0.417 us | 0.458 us |
| ISR pin width | 0.250 us | 0.292 us | 0.292 us |
| DRDY to I2C read start | 0.958 us | 1.042 us | 1.292 us |
| I2C read width | 1596.0 us | 1596.3 us | 1596.4 us |
| DRDY to first UART start bit | 1610.0 us | 1610.3 us | 1610.5 us |
| DRDY to classifier push start | 3693.8 us | 3694.1 us | 3694.4 us |

Reading it (one sample = 41.7 ns, so the 83 ns spread of the ISR latency is 2 samples):
- The ISR latency is about 0.42 us, roughly 167 CPU cycles at 400 MHz (NVIC entry, `HAL_GPIO_EXTI_IRQHandler`, callback prologue, from Flash, caches off). Its
  spread is at the resolution of the instrument and far below the 1 us tick of TIM2, so the jitter of `ts_us` is dominated by that quantisation.
- The I2C read takes 1.6 ms, in line with a 100 kHz bus, and then the UART frame starts about 13 us later (packet build, CRC, HAL setup). The frame lasts 2.08 ms
  (24 bytes at 115200 baud), and the push starts right after it: 3694 us is about 1610 + 2083. The CPU is busy about 3.7 ms of every 9.979 ms, almost all of it blocked in I2C and UART waits.
- **What this run does not show**: every DRDY arrives while the main loop is idle (it finishes about 6 ms earlier), so the case "DRDY while the loop is inside the I2C read or the
  UART transmit" was never exercised, and neither was the window-completing classification (319 us) overlapping anything. Measure that before calling 0.46 us a worst case
  (for instance with the sensor at a higher output rate so that the loop cannot keep up, or a deliberately long busy-wait in the loop).

### Run 2: 2026-11-21, same firmware (`1bfa6c2`), 25 captures merged, 10414 events

Conditions: as run 1 (Release `-Os`, 400 MHz, caches off, 115200 baud, MPU-6050 at 100 Hz), board untouched during the run (at rest is not independently
checked; whether `imu_record.py` was running on the Pi was not recorded). 25 separate captures of 4.167 s (104.2 s of data in total, with gaps between captures), merged with
`tools/m0_latency.py ... --events-out` and then `tools/m0_latency.py ev_*.npz`. Tool v3. 10414 events, so p99.9 is now meaningful as a statistic, but with a spread this
small it equals the max to within one sample.

| Quantity | n | min | p50 | p99 | p99.9 | max |
|----------|--:|----:|----:|----:|------:|----:|
| DRDY period [us] | 10414 | 9979.083 | 9979.167 | 9979.208 | 9979.250 | 9979.250 |
| DRDY to ISR pin (bias of `ts_us`) [us] | 10414 | 0.375 | 0.417 | 0.458 | 0.458 | 0.458 |
| ISR pin width [us] | 10414 | 0.250 | 0.250 | 0.292 | 0.292 | 0.292 |
| DRDY to I2C read start [us] | 10414 | 0.958 | 1.042 | 1.167 | 1.208 | 1.333 |
| I2C read width [us] | 10414 | 1596.000 | 1596.292 | 1596.417 | 1596.417 | 1596.542 |
| DRDY to first UART start bit [us] | 10414 | 1609.958 | 1610.333 | 1610.500 | 1610.542 | 1610.625 |
| DRDY to per-sample push start [us] | 10308 | 3693.708 | 3694.125 | 3694.375 | 3694.458 | 3694.542 |
| per-sample push width [us] | 10308 | 1.042 | 1.083 | 1.083 | 1.083 | 1.375 |
| DRDY to window classification start [us] | 106 | 3693.750 | 3694.125 | 3694.373 | 3694.412 | 3694.417 |
| window classification width [us] | 106 | 309.042 | 309.083 | 309.333 | 309.333 | 309.333 |

Reading it (all values are maximum OBSERVED over this run, one sample = 41.7 ns):
- Consistency: 10308 per-sample pushes + 106 window classifications = 10414 = the number of DRDY events, and every DRDY had an ISR pulse. No event was lost or doubled in this run.
- The ISR latency stayed within 0.375 to 0.458 us over 10414 events (2 samples of spread), again below the 1 us tick of `ts_us`.
- The DRDY period is 9979.17 us (100.209 Hz, +0.21 % against the nominal 100 Hz). Run 1 gave 9978.79 us; the 0.38 us (38 ppm) difference is of the order of the
  analyser's own crystal tolerance and of temperature drift of the sensor clock, and this data cannot separate the two. The absolute rate carries the analyser's clock error; the +0.21 % does not.
- The window classification took 309.0 to 309.3 us in 106 windows. README's 318.8 us (maximum of 1496 windows, an earlier build) is larger. 106 windows cannot exclude a rare slower
  window, and the code-placement explanation for the difference is still unverified. Do not quote 309 us as a maximum until a longer run exists.
- What this run still does not show (unchanged from run 1): DRDY while the loop is inside I2C, UART or the window classification, and the classifier under real motion.

## Not verified, check before relying on it

- MPU-6050 interrupt pin mode. The driver never writes `INT_PIN_CFG` (0x37), so the chip is at its reset value. From the
  datasheet as I remember it, that is active-high, push-pull, a short pulse, not latched, cleared by reading `INT_STATUS`
  (0x3A). That has not been checked on this sensor: D1 shows the pulse width directly (the synthetic tests assume about 50 us).
  If D1 stays high between events, the mode is latched and the DRDY edges are not what they look like.
- D1 and D3 edges one-to-one: checked in run 2 (10414 DRDY events, 10414 ISR pulses); not checked while the main loop is stalled.
