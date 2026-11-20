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

100 events are too few for p99; for p99.9 take at least 10,000 events (several captures, e.g. 10 minutes in total) and say so.
`tools/logic_pulses.py` still gives the classifier-window numbers (the once-per-second long pulses).

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

## Not verified, check before relying on it

- MPU-6050 interrupt pin mode. The driver never writes `INT_PIN_CFG` (0x37), so the chip is at its reset value. From the
  datasheet as I remember it, that is active-high, push-pull, a short pulse, not latched, cleared by reading `INT_STATUS`
  (0x3A). That has not been checked on this sensor: D1 shows the pulse width directly (the synthetic tests assume about 50 us).
  If D1 stays high between events, the mode is latched and the DRDY edges are not what they look like.
- Whether the D1 edges and the D3 edges are one-to-one on the real board (`m0_latency.py` prints how many DRDY events had no
  ISR pulse).
