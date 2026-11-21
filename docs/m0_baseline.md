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
| 2 | Delay DRDY edge to timestamp | D1 to D3 | = the bias of `ts_us`. It contains NVIC entry, `HAL_GPIO_EXTI_IRQHandler` and the callback prologue (caches are off, code runs from Flash). Under load it is the interesting number: repeat with the I2C read and the UART frame of the previous event in progress (they are, at 100 Hz: the loop is busy ~3.7 ms of 10). |
| 3 | Processing chain | D1 to D4 (I2C start), D4 width (I2C), D1 to D2 (first UART bit), D1 to D0 (classifier start) | Per-stage cycle counts still come from DWT (`g_act_cycles_*`). |
| 4 | Sensor to Pi | partly | The physical link to the MCU's TX pin is measured here. Pi receive time needs the MCU/Pi clock mapping (M6); until then `imu_record.py` timestamps are batch times (`ser.read(512)` with a 0.1 s timeout) and must not be used as latency numbers. |

Conditions to write next to every result: build type (Release `-Os`), clock 400 MHz, caches off, baud, what the sensor was doing
(still / shaking), whether `imu_record.py` was running on the Pi, capture length and number of events. The maximum is a
maximum OBSERVED execution time, not a WCET.

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

## Not verified, check before relying on it

- MPU-6050 interrupt pin mode. The driver never writes `INT_PIN_CFG` (0x37), so the chip is at its reset value. From the
  datasheet as I remember it, that is active-high, push-pull, a short pulse, not latched, cleared by reading `INT_STATUS`
  (0x3A). That has not been checked on this sensor: D1 shows the pulse width directly (the synthetic tests assume about 50 us).
  If D1 stays high between events, the mode is latched and the DRDY edges are not what they look like.
- Whether the D1 edges and the D3 edges are one-to-one on the real board (`m0_latency.py` prints how many DRDY events had no
  ISR pulse).
