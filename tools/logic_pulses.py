#!/usr/bin/env python3
"""Measure the classifier pulse (D0) and its delay after the DRDY interrupt (D1) in a logic-analyzer capture.

Capture (close PulseView first, it holds the USB device), one byte per sample, bit n = channel Dn:
    sigrok-cli -d fx2lafw --config samplerate=24m --samples 50m -O binary -o /tmp/cap.bin
    python3 tools/logic_pulses.py /tmp/cap.bin --rate 24e6

D0 = PB0 (high while the classifier runs), D1 = PC3 (MPU-6050 INT / DRDY), edges counted on rising.
Pulses shorter than --long-us are ordinary samples, the rest are window completions (about 1 per second).
Needs numpy.  Resolution is one sample (about 42 ns at 24 MHz).
"""
import argparse
import sys

import numpy as np


def edges(bits: np.ndarray):
    """-> (rising, falling) sample indices of a 0/1 array (index of the first sample at the new level)."""
    d = np.diff(bits.astype(np.int8))
    return np.flatnonzero(d == 1) + 1, np.flatnonzero(d == -1) + 1


def pulses(bits: np.ndarray):
    """-> (start, width) in samples of every complete high pulse (a pulse cut by either end is dropped)."""
    r, f = edges(bits)
    if len(r) and len(f) and f[0] < r[0]:
        f = f[1:]
    n = min(len(r), len(f))
    return r[:n], f[:n] - r[:n]


def stats(v, unit, scale):
    v = np.asarray(v, dtype=float) * scale
    return f"n={len(v)}  min {v.min():.3f}  mean {v.mean():.3f}  max {v.max():.3f} {unit}"


def analyse(data: np.ndarray, rate: float, long_us: float = 50.0, pulse_ch: int = 0, drdy_ch: int = 1) -> dict:
    us = 1e6 / rate
    start, width = pulses((data >> pulse_ch) & 1)
    dr, _ = edges((data >> drdy_ch) & 1)
    out = {"duration_s": len(data) / rate, "pulses": len(start), "drdy_edges": len(dr)}
    long_mask = width * us >= long_us
    out["short_us"] = width[~long_mask] * us
    out["long_us"] = width[long_mask] * us
    out["long_start_s"] = start[long_mask] / rate
    if len(dr) > 1:
        out["drdy_period_ms"] = np.diff(dr) * us / 1000.0
    # delay from the last DRDY edge before each pulse start to that pulse's start
    i = np.searchsorted(dr, start, side="right") - 1
    ok = i >= 0
    out["delay_ms"] = (start[ok] - dr[i[ok]]) * us / 1000.0
    out["delay_long_ms"] = ((start[ok] - dr[i[ok]]) * us / 1000.0)[long_mask[ok]]
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", help="raw binary capture, one byte per sample")
    ap.add_argument("--rate", type=float, default=24e6, help="sample rate in Hz (default 24e6)")
    ap.add_argument("--long-us", type=float, default=50.0, help="pulses at least this wide count as window pulses")
    args = ap.parse_args(argv)

    data = np.fromfile(args.capture, dtype=np.uint8)
    if len(data) < 1000:
        print("capture is empty or too short", file=sys.stderr)
        return 1
    r = analyse(data, args.rate, args.long_us)
    print(f"{len(data)} samples, {r['duration_s']:.3f} s, {r['pulses']} complete pulses on D0, {r['drdy_edges']} DRDY edges on D1")
    if "drdy_period_ms" in r:
        print("DRDY period:       " + stats(r["drdy_period_ms"], "ms", 1))
    if len(r["short_us"]):
        print("ordinary pulses:   " + stats(r["short_us"], "us", 1))
    else:
        print("ordinary pulses:   none")
    if len(r["long_us"]):
        print("window pulses:     " + stats(r["long_us"], "us", 1))
        for t, w, d in zip(r["long_start_s"], r["long_us"], r["delay_long_ms"]):
            print(f"   at {t:8.4f} s   width {w:8.2f} us   starts {d:6.3f} ms after DRDY")
    else:
        print(f"window pulses:     none (no pulse >= {args.long_us} us; capture at least 1.1 s)")
    if len(r["delay_ms"]):
        print("DRDY -> pulse start: " + stats(r["delay_ms"], "ms", 1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
