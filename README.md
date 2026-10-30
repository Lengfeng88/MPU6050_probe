# MPU6050_probe

STM32H743 firmware that streams raw MPU-6050 samples (accelerometer, temperature, gyroscope)
over USART1 as 24-byte binary packets at 100 Hz, plus the host-side tools to check the stream.

- MCU: STM32H743, project generated with STM32CubeMX (CMake + Ninja presets)
- Sensor: MPU-6050 on I2C1 (SCL = PB8, SDA = PB9), DATA_RDY interrupt on EXTI line 3
- Link: USART1, 115200 baud, 8N1, one-way (MCU to host)
- Debug probe: CMSIS-DAP over SWD, flashed with OpenOCD

## Layout

| Path | What it is |
|------|------------|
| `Core/` | CubeMX-generated code. Own code in `main.c` lives only inside `USER CODE` blocks, so regenerating from CubeMX keeps it. |
| `App/packet/` | CRC-16/CCITT-FALSE and the 24-byte packet encoder. |
| `App/imu/` | MPU-6050 driver (init, 14-byte burst read). |
| `App/sampling/` | ISR to main-loop queue of DATA_RDY events (timestamp + sequence number). |
| `App/bus/` | I2C bus recovery (clocks + STOP) used at boot and when sensor init fails. |
| `docs/packet_protocol.md` | The packet format and what `seq` can and cannot detect. |
| `docs/data_format.md` | Format of recorded sessions (`frames.bin`, `meta.json`, CSV / NPZ). |
| `tools/parse_imu.py` | Reads the stream, checks CRC / `seq` / timestamps, prints a summary. |
| `tools/imu_record.py` | Records a labelled session to disk (run it on the Raspberry Pi). |
| `tools/imu_convert.py` | Turns a recorded session into CSV / NumPy. |
| `tools/imu_proto.py` | Packet decoder shared by the tools above. |
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
4. The main loop pops an event, reads the 14 sensor bytes over I2C, builds the packet and sends
   it over USART1.

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
python3 tools/parse_imu.py /dev/ttyUSB0 30      # port, seconds
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

Python tools (standard library only; the recorder is driven through a fake serial port):

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

The Makefile does not track header changes, so run `make clean` first after pulling changes.
These tests use a mock HAL; they do not replace testing on the board.

## Known limits

- Raw sensor data only: no calibration, filtering or unit conversion in the firmware.
  The sensor stays at its reset ranges (+-2 g, +-250 deg/s).
- One-way link, no flow control: if the host does not read, the MCU does not notice.
- The queue's `dropped` counter is kept in RAM but not transmitted.
