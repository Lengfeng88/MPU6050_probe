# MPU6050_probe

STM32H743 firmware that streams raw MPU-6050 samples (accelerometer, temperature, gyroscope)
over USART1 as 24-byte binary packets at 100 Hz, plus the host-side tools to check the stream.

- MCU: STM32H743, project generated with STM32CubeMX (CMake + Ninja presets)
- Sensor: MPU-6050 on I2C1 (SCL = PB8, SDA = PB9), DATA_RDY interrupt on EXTI line 3
- Link: USART1, 1,000,000 baud, 8N1, one-way (MCU to host); TX by DMA (`App/uart_tx`)
- Debug probe: CMSIS-DAP over SWD, flashed with OpenOCD

## Layout

| Path | What it is |
|------|------------|
| `Core/` | CubeMX-generated code. Own code in `main.c` lives only inside `USER CODE` blocks, so regenerating from CubeMX keeps it. |
| `App/packet/` | CRC-16/CCITT-FALSE and the 24-byte packet encoder. |
| `App/imu/` | MPU-6050 driver (init, 14-byte burst read). |
| `App/sampling/` | ISR to main-loop queue of DATA_RDY events (timestamp + sequence number). |
| `App/uart_tx/` | Non-blocking UART TX: frame queue (`uart_tx.c`, host-tested) + STM32 DMA / BASEPRI / IRQ glue (`uart_tx_stm32.c`). |
| `App/bus/` | I2C bus recovery (clocks + STOP) used at boot and when sensor init fails. |
| `App/classify/` | On-board activity classifier: 200-sample window, 17 features, decision tree (`imu_tree.h` is generated), cycle counter and a debug pin for timing. |
| `docs/packet_protocol.md` | The packet format and what `seq` can and cannot detect. |
| `docs/data_format.md` | Format of recorded sessions (`frames.bin`, `meta.json`, CSV / NPZ). |
| `tools/parse_imu.py` | Reads the stream, checks CRC / `seq` / timestamps, prints a summary. |
| `tools/imu_record.py` | Records a labelled session to disk (run it on the Raspberry Pi). |
| `tools/imu_convert.py` | Turns a recorded session into CSV / NumPy. |
| `tools/imu_proto.py` | Packet decoder shared by the tools above. |
| `tools/rec.sh` | Starts a delayed, detached recording in tmux (survives an SSH drop). |
| `tools/imu_sim.py` | Simulated sessions in the recorded format (for pipeline tests only, see below). |
| `tools/imu_features.py` | Cuts sessions into windows and computes the 17 features (needs numpy). |
| `tools/imu_train.py` | Session-wise evaluation, decision tree training, C export (needs numpy). |
| `tools/logic_pulses.py` | Measures the classifier pulse width and its delay after DRDY in a logic-analyzer capture. |
| `tools/reset_stress.sh` | Resets the MCU at random moments and checks it keeps streaming. |
| `tests/host/` | Host-side tests of the firmware code (no board needed). |
| `tests/py/` | Tests of the Python tools. |

`App/*/*.c` is picked up by a glob in `CMakeLists.txt`; add new modules as `App/<dir>/<file>.c`.

## How it works

1. At boot, before the I2C peripheral is initialised, the I2C bus is put back to idle:
   up to 9 clock pulses if SDA is held low, then a STOP (`App/bus`).
2. The sensor is configured for 100 Hz with DATA_RDY interrupts (`SMPLRT_DIV = 9`, `DLPF_CFG = 3`).
   If initialisation fails with a bus error, the I2C peripheral is released, the bus is cleared
   again, the peripheral is re-initialised and the init is retried (3 attempts in total).
   A wrong `WHO_AM_I` is not retried.
3. In the EXTI ISR each DATA_RDY event gets a timestamp (TIM2, 1 us per tick) and a sequence
   number and is pushed into a small lock-free queue.
4. The main loop pops an event, reads the 14 sensor bytes over I2C (still blocking, ~1.6 ms), builds the
   packet and queues it for USART1. `App/uart_tx` sends it by DMA (DMA1 stream 0) and returns at once;
   a full queue drops the new frame and counts it in `g_uart_tx.dropped` (not transmitted yet).

Because `seq` is assigned in the ISR, events dropped on a full queue or lost to a failed I2C
read leave a hole in `seq`. Events merged by the hardware (interrupts masked or the core halted
longer than 10 ms) do not; those show up only as a gap in `ts_us`. Details in
`docs/packet_protocol.md`.

## Build and flash

```bash
cmake --build --preset Debug

openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "adapter speed 1000" \
  -c "program build/Debug/MPU6050_probe.elf verify reset exit"
```

Use `build/Release/` and `--preset Release` for timing measurements (`-Os`; Debug is `-O0`).

If OpenOCD fails with `Error connecting DP: cannot read IDR`, this variant connects under reset
at a lower speed and worked on this board:

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "adapter speed 500" \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -c "program build/Debug/MPU6050_probe.elf verify reset exit"
```

## Check the stream

```bash
pip install pyserial
python3 tools/parse_imu.py /dev/ttyUSB0 30      # port, seconds [, baud; default 1000000]
```

Healthy output on this board:

```
packets ok=3015  crc_bad=0  seq_gaps=0  lost=0  resync_bytes=0
dt(us): min=9979 max=9980 mean=9979.2
```

- `crc_bad`: packets whose CRC failed. `resync_bytes`: bytes skipped to find the next `AA 55`;
  a small number right after opening the port is normal (the tool starts mid-packet).
- `seq_gaps` / `lost`: number of holes in `seq` / number of missing sequence numbers.
- `dt(us)`: time between consecutive packets, from the MCU timestamps.

## Record data

```bash
python3 tools/imu_record.py /dev/ttyUSB0 --label walking --subject kelvin \
        --note "right wrist, strap" --duration 120
python3 tools/imu_convert.py data/<session dir>            # samples.csv in physical units
python3 tools/imu_convert.py data/<session dir> --npz      # also samples.npz (needs numpy)
```

Each run creates `data/<UTC time>_<label>/` with `frames.bin` (the valid packets, unchanged)
and `meta.json` (label, subject, note, start time, reception statistics). A healthy run has
`crc_bad`, `seq_gaps` and `lost_events` all 0. Format and time handling: `docs/data_format.md`.
Add `data/` to `.gitignore`; recordings do not belong in the repository.

The recorder only needs a serial port, so it runs the same on a laptop or on a Raspberry Pi
(pyserial and Python 3.8+; no other dependency). How the Pi is wired to the board is not part of this repo.

## Activity classifier on the MCU

`App/classify/` keeps the newest 200 samples (2 s), and every 100 samples computes 17 features
(means, standard deviations, |a| and |w| statistics, jerk, zero crossings of gy; the same code
path as `tools/imu_features.py`, checked against it in `tests/py/test_classify_c.py`) and runs a
decision tree on them. A window never spans lost events: after a hole in `seq` it restarts.
The result is not sent anywhere yet; read the `volatile` globals `g_act_class`, `g_act_windows`,
`g_act_cycles_window_max` and `g_act_cycles_push_max` of `main.c` with a debugger.
Class indices are in the header comment of `App/classify/imu_tree.h`.

**The tree in the repository was trained on SIMULATED data** (`tools/imu_sim.py`). It shows that
the pipeline works end to end; it says nothing about how well real activities are recognised, and
no accuracy on real data has been measured. Retrain on real recordings:

```bash
python3 tools/imu_convert.py data/<session> --npz              # once per session
python3 tools/imu_features.py data/<sessions...> --out data/features.npz
python3 tools/imu_train.py data/features.npz --depth 4 --export-c App/classify/imu_tree.h
```

`imu_train.py` evaluates by session (a held-out session of every class per fold), because the
windows of one session overlap and a random split would leak. It prints a warning first when
the data is simulated, and the exported header carries the same warning.

### Measured cost

Release build (`-Os`), 400 MHz, **CPU caches not enabled** (the firmware never turns them on, so code runs straight from Flash), one board, firmware with the simulated tree, board standing still on a desk:

| Item | Result | How |
|------|--------|-----|
| Flash, `App/classify` code and tree | 1237 B (whole firmware 18.6 KB of 2 MB) | `arm-none-eabi-size` on the two objects |
| RAM, window buffer | 4800 B (static) | 200 samples x 6 floats |
| Stack, classifier frames | 1728 B (features) + 184 B (push), plus the callers; `_Min_Stack_Size` is 4 KB | `-fstack-usage` |
| Window completed (about once per second) | 318.8 us worst of 1496 windows (127,519 cycles) | DWT cycle counter |
| Same, other instruments | 319 us (oscilloscope), 318.6 us (logic analyzer, 2 windows) | pulse width on PB0 |
| Ordinary sample | 1.09 us mean (logic analyzer), 511 cycles = 1.28 us worst (DWT) | |
| DRDY to classifier start | 3.686 ms (I2C read about 1.6 ms, UART frame about 2.1 ms) | logic analyzer, 208 events |
| Stream with the classifier running | 6014 frames in 60 s, `crc_bad`, `seq_gaps`, `lost` all 0 | `tools/imu_record.py` on the Pi |

The classifier runs after the UART transmit, so the packet stream does not wait for it. These
are elapsed times: an interrupt that arrives during a window computation is included. The same
classifier code measured 309 us (DWT, 196 windows) in an earlier build of the firmware, so
expect a few percent from code placement alone (cause not verified; with the caches off, code
alignment in Flash can matter). The worst case under real motion has not been measured.

PB0 is high while the classifier runs (`App/classify/debug_pin.h`; define `IMU_NO_DEBUG_PIN` to
switch it off). To reproduce the logic-analyzer numbers with a 24 MHz, 8-channel analyzer
(D0 = PB0, D1 = PC3 = DRDY, D2 = PA9 = UART TX; close PulseView first):

```bash
sigrok-cli -d fx2lafw --config samplerate=24m --samples 50m -O binary -o /tmp/cap.bin
python3 tools/logic_pulses.py /tmp/cap.bin --rate 24e6
```

## Reset stress test

```bash
tools/reset_stress.sh 60        # number of resets, optional serial port as 2nd argument
```

Resets the MCU a second time 150-400 ms after the first (while it is streaming), waits, and
requires at least 150 packets in 2 s. It stops at the first failure.

This exists because a reset in the middle of an I2C transfer can leave the MPU-6050 stuck,
after which every boot failed with `Error_Handler` (`hi2c1.ErrorCode = 0x2`, arbitration lost)
until the board was power cycled. Results on this board:

| Firmware | Result |
|----------|--------|
| no bus recovery | failed at reset #4 and #2 |
| bus clear only when SDA is low | failed at reset #17 |
| STOP at boot + retry (current) | 60 of 60 and 300 of 300 resets recovered |

The failure mechanism (the sensor stopped mid-transfer) is inferred from the symptoms and from
the fix working; it was not captured on a logic analyser.

## Tests

Python tools (the receiver tools use only the standard library and the recorder is driven through a
fake serial port; the feature, training and pulse tools need numpy, and some tests also use
scikit-learn or a C compiler and are skipped when those are missing):

```bash
python3 -m unittest discover -s tests/py -v
```

Firmware code on the host, needs only `gcc`, `make`, `python3` and `git`:

```bash
make -C tests/host clean
make -C tests/host test OLD_REV=<commit before the App/ split>
```

- `units`: the `App/` modules against the original code (CRC, packet layout, queue), the driver
  against a simulated MPU-6050, and the bus recovery against a simulated stuck slave.
- `glue`: the USER CODE of `main.c` before and after the refactor is compiled for the host and run
  against the same simulated sensor; UART bytes and HAL call traces are compared, and the output
  goes through the real `tools/parse_imu.py`. Scenarios cover steady streaming, bursts, failed
  reads, wrong chip ID, stuck and confused bus, and a bus that never recovers.
- The classifier's C code is also compiled with the host compiler by `tests/py/test_classify_c.py`
  and compared with numpy (double build: rounding noise; float build, as on the MCU: below 2e-4).
  `gy_zcr` can differ by a few counts in a window where samples sit exactly on the mean
  (a nearly still, quantised gyro); the test accepts that only in such windows.

The Makefile does not track header changes, so run `make clean` first after pulling changes.
These tests use a mock HAL; they do not replace testing on the board.

## Known limits

- Raw sensor data only: no calibration, filtering or unit conversion in the firmware.
  The sensor stays at its reset ranges (+-2 g, +-250 deg/s).
- One-way link, no flow control: if the host does not read, the MCU does not notice.
- The queue's `dropped` counter is kept in RAM but not transmitted.
- The classifier's accuracy on real data is unknown (see above), and its result is only visible
  through a debugger.

## Non-blocking UART TX (1 Mbaud)

Before: `HAL_UART_Transmit(..., 50)` in the main loop blocked for the whole frame (24 B at 115200 baud = 2.08 ms) and, on a
stuck line, for up to 50 ms. Now `uart_tx_stm32_submit()` copies the frame into a 16-slot queue and starts DMA; the
TX-complete interrupt (priority 2) starts the next frame. EXTI3 (DATA_RDY, priority 0) is never masked by it: the queue lock
raises BASEPRI above priority 2 only.

- 1,000,000 baud: a 24-byte frame takes 240 us on the wire, 2.4 % line load at 100 Hz. Both clocks divide exactly
  (USART kernel clock 100 MHz / 100; the Pi's PL011 divisor also comes out exact if its UART clock is 48 or 50 MHz, which is
  from memory, not checked on this Pi). Verify on the Pi: `parse_imu.py` for 10 minutes with `crc_bad=0`.
- The frame slots are in AXI SRAM (`.dma_buf` section added to `STM32H743xx_FLASH.ld`): `.data`/`.bss`/stack are in DTCM, which DMA1
  cannot read. If the data cache is ever enabled, make that region non-cacheable in the MPU.
- DMA, the two interrupts and their priorities are set up in the USER CODE blocks of `usart.c` and in `App/uart_tx`, not in the
  `.ioc` (only the baud rate is). The `.ld` edit and these blocks are hand edits that CubeMX regeneration can remove.
- `printf` / `_write` must not be used next to the packet stream any more (see the comment in `main.c`).
- Host tests: `tests/host` `units` covers the queue with a fake DMA (order, overload, start failure, DMA error, 100k random ops).
  The `glue` test checks that `main.c` still produces the same bytes and HAL trace (a shim stands in for the DMA glue).
  **Not yet measured on the board**: the DMA/IRQ glue only passed a syntax-level cross-compile check here. To measure: main-loop
  time per sample and `g_uart_tx.{submitted,sent,dropped,start_fail,errors}` after a 10-minute run.
