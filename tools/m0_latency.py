#!/usr/bin/env python3
"""M0 baseline: timing chain of one DATA_RDY event, from a logic-analyzer capture.

Capture (close PulseView first), one byte per sample, bit n = channel Dn:
    sigrok-cli -d fx2lafw --config samplerate=24m --samples 100m -O binary -o /tmp/cap.bin
    python3 tools/m0_latency.py /tmp/cap.bin --rate 24e6

Default wiring (change with --drdy/--isr/--i2c/--uart/--cls):
    D0 PB0  classifier running (high)      D3 your pin  DATA_RDY callback running (high), see App/measure/m0_pins.h
    D1 PC3  MPU-6050 INT = DRDY            D4 your pin  mpu6050_read_raw() running (high)
    D2 PA9  USART1 TX (idles high; the first falling edge of a frame is its start bit)

Per DRDY rising edge it reports, as min / p50 / p99 / p99.9 / max (percentiles need many events: n is printed):
    DRDY period, DRDY -> ISR pin (= bias of ts_us), ISR pin width, DRDY -> I2C start, I2C width,
    DRDY -> first UART start bit, DRDY -> classifier start, and how many DRDY edges had no ISR pulse.
Resolution is one sample (about 42 ns at 24 MHz).  The maximum is an OBSERVED maximum, not a WCET.
Needs numpy.
"""
import argparse
import json
import sys

import numpy as np

from logic_pulses import edges, pulses


def pct(v, scale=1.0):
    v = np.asarray(v, dtype=float) * scale
    if len(v) == 0:
        return None
    q = np.percentile(v, [50, 99, 99.9])
    return {"n": int(len(v)), "min": float(v.min()), "p50": float(q[0]), "p99": float(q[1]),
            "p99.9": float(q[2]), "max": float(v.max())}


def first_in_window(starts, dr):
    """For each DRDY edge dr[i]: index of the first element of `starts` in [dr[i], dr[i+1]) or -1."""
    nxt = np.append(dr[1:], np.iinfo(np.int64).max)
    j = np.searchsorted(starts, dr, side="left")
    ok = j < len(starts)
    hit = np.full(len(dr), -1, dtype=np.int64)
    jj = np.minimum(j, max(len(starts) - 1, 0))
    if len(starts):
        good = ok & (starts[jj] < nxt)
        hit[good] = jj[good]
    return hit


def analyse(data, rate, drdy=1, isr=3, i2c=4, uart=2, cls=0):
    us = 1e6 / rate
    bit = lambda ch: (data >> ch) & 1
    dr, _ = edges(bit(drdy))
    out = {"duration_s": len(data) / rate, "drdy_edges": int(len(dr))}
    if len(dr) < 3:
        return out
    dr = dr[:-1]                                   # the last DRDY has no closing window
    out["drdy_period_ms"] = pct(np.diff(edges(bit(drdy))[0]), us / 1000.0)

    def chain(name, starts, widths=None, width_name=None):
        hit = first_in_window(starts, dr)
        seen = hit >= 0
        out[name] = pct(starts[hit[seen]] - dr[seen], us)
        out[name + "_missing"] = int((~seen).sum())
        if widths is not None:
            out[width_name] = pct(widths[hit[seen]], us)

    s, w = pulses(bit(isr));  chain("drdy_to_isr_us", s, w, "isr_width_us")
    s, w = pulses(bit(i2c));  chain("drdy_to_i2c_start_us", s, w, "i2c_width_us")
    uf = edges(bit(uart))[1]                       # falling edges on TX = start bits
    chain("drdy_to_uart_start_us", uf)
    s, w = pulses(bit(cls))
    chain("drdy_to_classifier_start_us", s)
    return out


def show(out):
    print(f"{out['duration_s']:.3f} s, {out['drdy_edges']} DRDY edges")
    if "drdy_period_ms" not in out:
        print("too few DRDY edges: wrong channel, or the board is not running?")
        return
    rows = [("DRDY period [ms]", "drdy_period_ms", None),
            ("DRDY -> ISR pin [us] (bias of ts_us)", "drdy_to_isr_us", "drdy_to_isr_us_missing"),
            ("ISR pin width [us]", "isr_width_us", None),
            ("DRDY -> I2C start [us]", "drdy_to_i2c_start_us", "drdy_to_i2c_start_us_missing"),
            ("I2C read width [us]", "i2c_width_us", None),
            ("DRDY -> first UART start bit [us]", "drdy_to_uart_start_us", "drdy_to_uart_start_us_missing"),
            ("DRDY -> classifier start [us]", "drdy_to_classifier_start_us", "drdy_to_classifier_start_us_missing")]
    print(f"{'':42s}{'n':>7s}{'min':>10s}{'p50':>10s}{'p99':>10s}{'p99.9':>10s}{'max':>10s}")
    for label, key, miss in rows:
        v = out.get(key)
        if v is None:
            print(f"{label:42s}  no data on that channel")
            continue
        extra = f"   ({out[miss]} DRDY events without one)" if miss and out.get(miss) else ""
        print(f"{label:42s}{v['n']:7d}{v['min']:10.3f}{v['p50']:10.3f}{v['p99']:10.3f}{v['p99.9']:10.3f}{v['max']:10.3f}{extra}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--rate", type=float, default=24e6)
    for name, d in (("drdy", 1), ("isr", 3), ("i2c", 4), ("uart", 2), ("cls", 0)):
        ap.add_argument(f"--{name}", type=int, default=d, help=f"channel of {name} (default D{d})")
    ap.add_argument("--json", help="also write the numbers to this file")
    a = ap.parse_args(argv)
    data = np.fromfile(a.capture, dtype=np.uint8)
    if len(data) < 1000:
        print("capture is empty or too short", file=sys.stderr)
        return 1
    out = analyse(data, a.rate, a.drdy, a.isr, a.i2c, a.uart, a.cls)
    show(out)
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
