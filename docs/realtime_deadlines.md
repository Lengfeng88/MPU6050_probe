# Real-time requirements (proposal, step 0)

These are PROPOSED numbers to be confirmed or changed by the project owner; nothing here has been
measured on the board unless stated. "Real time" here means: bounded, measured worst cases and
detectable failure, not a hard-real-time guarantee on the Raspberry Pi (Linux).

## Split

| Part | Class | Why |
|------|-------|-----|
| STM32: DATA_RDY timestamp, I2C read, framing, UART TX, classifier | hard-ish real time (bounded by design and measured) | single core, interrupts, no OS |
| Link (UART) | bounded transfer time, loss detectable | CRC + `seq` |
| Raspberry Pi: reader, logging | soft real time, measured | Linux; PREEMPT_RT + pinned FIFO thread improves the bound but is not a proof |

## Deadlines (100 Hz, period 10 ms)

| # | Requirement | Today | How to measure |
|---|-------------|-------|----------------|
| D1 | DRDY edge to timestamp captured: jitter <= 5 us | dt 9979..9980 us over 3015 packets (README), i.e. about 1 us, board idle | `parse_imu.py` dt line; repeat with everything running |
| D2 | DRDY to first byte of the packet on the wire <= 3 ms (worst case, not mean) | DRDY to classifier start 3.686 ms, of which I2C ~1.6 ms and UART ~2.1 ms (README) | logic analyzer D1 (DRDY) vs D2 (PA9), `tools/logic_pulses.py` |
| D3 | No main-loop pass longer than 5 ms, including I2C failure paths | I2C timeout is 20 ms, UART was 50 ms: not bounded below the period | per-stage pin / DWT max, fault injection |
| D4 | Zero silent loss: every lost DATA_RDY event shows up as a `seq` gap; queue drops counted | `dropped` kept but not transmitted | add counters to a status frame (later step) |
| D5 | Window result (every 100 samples) emitted within 1 ms of the window's last sample | result only in a debugger variable; compute 319 us worst, board still | later step: result frame |
| D6 | Pi: frame available to the application within 10 ms of DRDY, p99.9 | unmeasured; `imu_record.py` reads in batches of up to 100 ms | later steps: time sync + C reader |

Classification latency as a whole is dominated by the 2 s window (hop 1 s), not by D1..D6.
