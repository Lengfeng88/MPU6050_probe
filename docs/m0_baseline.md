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

### Run 3: 2026-11-21, `realtime-baremetal` at `7c577b0` built with the stall, 25 captures merged, 10411 events

Build: as run 2 plus `-DM0_STALL_US=8000 -DM0_STALL_EVERY=10 -DM0_STALL_STEP_US=250 -DM0_STALL_STEPS=49` (stalls of 8.0 to 20.0 ms after every 10th event, 49 steps, repeated).
Same conditions otherwise (Release `-Os`, 400 MHz, caches off, 115200 baud, MPU-6050 at 100 Hz, board untouched; at rest and Pi activity not recorded). 25 captures of 4.167 s, merged.

| Quantity | n | min | p50 | p99 | p99.9 | max |
|----------|--:|----:|----:|----:|------:|----:|
| DRDY period [us] | 10411 | 9978.708 | 9978.917 | 9979.042 | 9979.042 | 9979.083 |
| DRDY to ISR pin, all events [us] | 10411 | 0.292 | 0.375 | 0.417 | 0.417 | 0.417 |
| ISR pin width [us] | 10411 | 0.167 | 0.250 | 0.250 | 0.250 | 0.250 |
| I2C read width [us] | 10093 | 1595.958 | 1596.083 | 1596.333 | 1596.375 | 1596.417 |
| DRDY to I2C start, paired in order [us] | 10429 | 0.917 | 1.083 | 12520.8 | 13710.6 | 14044.1 |
| DRDY to ISR pin, DRDY inside an I2C read [us] | 149 | 0.375 | 0.375 | 0.417 | 0.417 | 0.417 |
| DRDY to ISR pin, DRDY inside UART tx / encode [us] | 273 | 0.333 | 0.375 | 0.417 | 0.417 | 0.417 |
| DRDY to ISR pin, DRDY inside a window classification [us] | 0 | no data | | | | |
| window classification width [us] | 103 | 334.750 | 334.750 | 335.000 | 335.000 | 335.000 |

Reading it (maximum OBSERVED values, one sample = 41.7 ns):
- With the main loop blocked, the ISR latency did not change: 149 DRDY edges arrived during an I2C read and 273 during the UART transmit / packet encode, and all of them
  were 0.333 to 0.417 us, the same range as the idle events. This is consistent with EXTI3 at priority 0 not being masked by the loop's critical sections (BASEPRI).
  It is not proof for other code paths: a DRDY during a window classification was NOT observed (the stall follows the window event, it does not precede it), and
  nothing here covers an ISR of higher or equal priority (the UART DMA interrupt of the next milestone is not in this build).
- The loop's own latency grew as intended: p50 1.08 us, max 14.04 ms. The maximum matches the arithmetic: the stall follows a window event (service time about 4.03 ms),
  so the longest wait is 20.0 + 4.03 - 9.98 = 14.05 ms.
- No event was dropped: `g_drdy.dropped` read through openocd after the run was 0 (it counts since the last reset, which is the whole stall run). 6 DRDY edges at the ends of the 25 captures
  found no I2C read, which is what the capture ending first is expected to cause (about 16 % of captures, so about 4 of 25).
- Rows not to read in this run: `DRDY -> I2C start`, `-> first UART start bit` and `-> classifier push start` (window pairing) pair a DRDY with the first pulse before the next DRDY, which is wrong
  when the loop is behind (the UART row even shows a minimum of 0.000 us). Their p50 is still the idle value because most events are not stalled.
- The ISR pin is 2 samples narrower and 1 sample lower in latency than in run 2 (0.292 to 0.417 against 0.375 to 0.458 us). Different build, same instrument resolution; no claim beyond that.

**The numbers are specific to a build.** Same source for the sampling path and the classifier, different code layout (run 2 against run 3, medians of the unstalled majority):

| | run 2 (`1bfa6c2`) | run 3 (`7c577b0`, with stall code) |
|---|---:|---:|
| window classification width | 309.08 us | 334.75 us (+8.3 %) |
| I2C read end to first UART start bit | about 13.0 us | about 5.3 us |
| DRDY to per-sample push start | 3694.1 us | 3686.2 us |

README's 3.686 ms and 318.8 us are two more points of the same kind. The cause is not established (code placement, alignment and Flash fetch with caches off are the candidates). Run 4
below shows the offset only appears when the stall code is compiled in. Consequence: any number quoted from this document needs its commit and build flags, and a milestone is compared with the baseline only through
a measurement of the same build.

### Run 4: 2026-11-21, `realtime-baremetal` at `3d29d4b`, reference build (M0 pins, no stall), 25 captures merged, 10417 events

Build flags: `-DIMU_M0_PINS -DM0_ISR_PORT=GPIOH -DM0_ISR_PIN=GPIO_PIN_2 -DM0_I2C_PORT=GPIOH -DM0_I2C_PIN=GPIO_PIN_3` (no `M0_STALL_*`), Release `-Os`, 400 MHz, caches off, 115200 baud, MPU-6050 at 100 Hz,
flash 18940 B (the same size as run 2's build), ELF sha256 `c8ebc0ae4e0b53efb51339ab5398c3ed73d8aa1c73f521a9be8740de2f60c3d0`. Board untouched (at rest and Pi activity not recorded).

| Quantity | run 2 (`1bfa6c2`, 10414 ev.) p50 / max | run 4 (`3d29d4b`, 10417 ev.) p50 / max |
|----------|------:|------:|
| DRDY period [us] | 9979.167 / 9979.250 | 9979.375 / 9979.458 |
| DRDY to ISR pin [us] | 0.417 / 0.458 | 0.417 / 0.458 |
| ISR pin width [us] | 0.250 / 0.292 | 0.250 / 0.292 |
| DRDY to I2C start [us] | 1.042 / 1.333 | 1.042 / 1.333 |
| I2C read width [us] | 1596.292 / 1596.542 | 1596.292 / 1596.500 |
| DRDY to first UART start bit [us] | 1610.333 / 1610.625 | 1610.292 / 1610.667 |
| DRDY to per-sample push start [us] | 3694.125 / 3694.542 | 3694.125 / 3694.458 |
| per-sample push width [us] | 1.083 / 1.375 | 1.083 / 1.375 |
| window classification width [us] (n=106 / n=108) | 309.083 / 309.333 | 309.083 / 309.333 |

Reading it:
- Two independent 25-capture runs, one commit apart, agree to within one sample (41.7 ns) in every row except the DRDY period, so the measurement repeats. This is the reference for the later milestones.
- The no-op `m0_stall_after_event()` call and the added header code did not move the sampling path or the classifier: the window classification is 309 us again, not the 335 us of the stall build (run 3).
  That is numerical agreement, not proof of an identical binary (the binaries of run 2 and run 4 were not compared).
- The DRDY period moved 0.2 us between these runs (9979.17 to 9979.38 us) and was 9978.6 to 9978.9 us in runs 1 and 3: 0.75 us (75 ppm) over five measurements. Sensor temperature and the analyser's clock are both candidates; this data does not separate them.
- 10309 per-sample pushes + 108 window classifications = 10417 = events; every DRDY had an ISR pulse; 7 DRDY edges at the ends of the captures found no I2C read (capture ending first).

### Run 5: 2026-11-21, reference build (`3d29d4b` firmware), still against shaken, 8 captures each. Inconclusive on data dependence.

Procedure: 8 captures of 4.167 s with the board left alone, then 8 captures while the MPU-6050 was shaken by hand (shaking started 2 s before each capture and continued through it).
Separate event files, merged per group. The shaking was not instrumented: how hard it was is not recorded.

| | still (8 captures, 3332 events) | shaken (8 captures, 3333 events) |
|---|---:|---:|
| window classification width, n | 32 | 32 |
| min / p50 / max [us] | 309.042 / 309.083 / 309.292 | 309.042 / 309.083 / 309.333 |

Independent check with the DWT cycle counter (`g_act_cycles_window_max`, read with openocd before and after the 16 captures, no reset between): `0x1e347` = 123719 cycles = 309.30 us at 400 MHz both times,
while `g_act_windows` went from 474 to 827. None of the 353 windows in between took longer. The logic analyser (GPIO pulse, max 309.29 to 309.33 us) and the DWT counter agree.

What this does and does not show:
- On these windows the classification time did not move: the whole observed range is 309.04 to 309.33 us (about 116 cycles, 0.1 %).
- It does not show that the input changed. `g_act_class` read 3 (`still`, `imu_tree.h`) before and during the shaking, so the classifier saw nothing that made it leave "still".
  Whether the shaking was strong enough to matter is unknown, so this run says nothing about worst-case input.
- From the code (`imu_window.c`): the loops are fixed (200 samples, 6 channels, two passes) and `sqrtf` is one FPU instruction, but three things depend on the data: the min / max updates of the
  acceleration magnitude, the zero-crossing branch, and the path through the tree. A rough upper bound for the branch costs would be a few microseconds; that is an estimate from reading the code, not a measurement.
  A bound needs a synthetic worst-case input (for example alternating signs for the zero crossings, a rising ramp for the min / max updates) run on the board in one build, because the build layout alone moved the
  window time by 8 % (run 3).
- `imu_tree.h` says it was trained on simulated data. Nothing here supports a claim about recognising real activities; this milestone measures timing only.

## Not verified, check before relying on it

- MPU-6050 interrupt pin mode: CHECKED, one 4.167 s capture of the reference build (417 DRDY pulses, tool at `79985fa`). The driver never writes `INT_PIN_CFG` (0x37), so the chip is at its
  reset value. D1 shows a pulse of 48.79 to 48.92 us (p50 48.83 us) that falls by itself about every 9.98 ms: the line is active-high (the rising edge is what the EXTI uses) and NOT latched.
  Not determined: push-pull against open-drain. The width is 2.3 % under the 50 us I remembered from the datasheet (clock accuracy of the sensor is one candidate; not looked into).
  One capture is a check of the mode, not a distribution.
- D1 and D3 edges one-to-one: checked in run 2 (10414 DRDY events, 10414 ISR pulses) and in run 3 with the loop stalled (10411 and 10411).
- DRDY arriving during a window classification (about 309 us): not observed. The stall of the overlap experiment follows the window event instead of preceding it.
- Worst-case classifier input (see run 5): not measured.
