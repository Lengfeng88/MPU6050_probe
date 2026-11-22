#!/usr/bin/env python3
"""M0 baseline: timing chain of one DATA_RDY event, from logic-analyzer captures.

Capture (close PulseView first), one byte per sample, bit n = channel Dn:
    sigrok-cli -d fx2lafw --config samplerate=24m --samples 100m -O binary -o /tmp/cap.bin
    python3 tools/m0_latency.py /tmp/cap.bin --rate 24e6

Many captures: analyse each, keep only the per-event numbers (small), merge at the end:
    for i in $(seq 1 25); do
      sigrok-cli -d fx2lafw --config samplerate=24m --samples 100m -O binary -o /tmp/cap.bin
      python3 tools/m0_latency.py /tmp/cap.bin --events-out /tmp/ev_$i.npz >/dev/null && rm /tmp/cap.bin
    done
    python3 tools/m0_latency.py /tmp/ev_*.npz --json m0.json          # .npz files are merged, not re-analysed

Default wiring (change with --drdy/--isr/--i2c/--uart/--cls):
    D0 PB0  high around EVERY classifier push (a few us), and for ~300 us when the push completes a window
    D1 PC3  MPU-6050 INT = DRDY            D3 your pin  DATA_RDY callback running, see App/measure/m0_pins.h
    D2 PA9  USART1 TX (idles high; the first falling edge of a frame is its start bit)
                                           D4 your pin  mpu6050_read_raw() running

Per DRDY rising edge it reports min / p50 / p99 / p99.9 / max (n is printed; below ~10,000 events p99.9 is just the max):
    DRDY period, DRDY -> ISR pin (= bias of ts_us), ISR pin width, DRDY -> I2C start, I2C width,
    DRDY -> first UART start bit, DRDY -> start of the per-sample classifier push and its width,
    DRDY -> start of a window-completing classification (pulses longer than --long-us) and its width,
    and how many DRDY edges had no ISR / I2C / UART / push pulse.
Resolution is one sample (about 42 ns at 24 MHz).  The maximum is an OBSERVED maximum, not a WCET.
Needs numpy.
"""
import argparse
import json
import sys

import numpy as np

from logic_pulses import edges, pulses

ROWS = [("DRDY period [us]", "drdy_period_us", None),
        ("DRDY (INT) pulse width [us]", "drdy_width_us", None),
        ("DRDY -> ISR pin [us] (bias of ts_us)", "drdy_to_isr_us", "drdy_to_isr_us_missing"),
        ("ISR pin width [us]", "isr_width_us", None),
        ("DRDY -> I2C start [us]", "drdy_to_i2c_start_us", "drdy_to_i2c_start_us_missing"),
        ("I2C read width [us]", "i2c_width_us", None),
        ("DRDY -> first UART start bit [us]", "drdy_to_uart_start_us", "drdy_to_uart_start_us_missing"),
        ("DRDY -> classifier push start [us]", "drdy_to_push_start_us", "push_missing"),
        ("classifier push width [us]", "push_width_us", None),
        ("DRDY -> window classification start [us]", "drdy_to_window_start_us", None),
        ("window classification width [us]", "window_width_us", None),
        ("DRDY -> I2C start, paired in order [us]", "i2c_start_seq_us", "i2c_start_seq_unmatched"),
        ("DRDY -> ISR pin, DRDY inside an I2C read [us]", "isr_latency_in_i2c_us", None),
        ("DRDY -> ISR pin, DRDY inside UART tx / encode [us]", "isr_latency_in_uart_us", None),
        ("DRDY -> ISR pin, DRDY inside a window classif. [us]", "isr_latency_in_window_us", None)]
MISS_TEXT = {"i2c_start_seq_unmatched": "DRDY events at the end without an I2C read: capture ended first (a few per capture) or events were dropped"}
SCALARS = ("events", "duration_s")          # per-file scalars, stored as 1-element arrays and summed when merged


def pct(v):
    v = np.asarray(v, dtype=float)
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


def inside(t, starts, ends):
    """Per element of t: does it lie in [starts[j], ends[j]) for some j?  (starts sorted, intervals disjoint)"""
    if len(starts) == 0:
        return np.zeros(len(t), dtype=bool)
    j = np.searchsorted(starts, t, side="right") - 1
    return (j >= 0) & (t < ends[np.maximum(j, 0)])


def paired_in_order(starts, dr, us, sync_us=3.0):
    """k-th DRDY with k-th I2C start, valid when the loop falls behind (the window pairing above then pairs wrongly).
    Starts at the first DRDY whose own I2C read began within sync_us (the loop was idle).  Assumes no event is dropped:
    a drop shifts every later pair by one period, so also returns how many DRDY edges at the end found no I2C read
    (a few per capture are just the capture ending first; more means drops)."""
    hit = first_in_window(starts, dr)
    own = (hit >= 0) & ((starts[np.maximum(hit, 0)] - dr) * us < sync_us) if len(starts) else np.zeros(len(dr), bool)
    ok = np.flatnonzero(own)
    if len(ok) == 0:
        return np.array([]), 0
    k0, j0 = ok[0], hit[ok[0]]
    n = min(len(dr) - k0, len(starts) - j0)
    return (starts[j0:j0 + n] - dr[k0:k0 + n]) * us, int(len(dr) - k0 - n)


def collect(data, rate, drdy=1, isr=3, i2c=4, uart=2, cls=0, long_us=50.0):
    """One capture -> per-event arrays (microseconds) and a few counters.  Small enough to keep and merge."""
    us = 1e6 / rate
    bit = lambda ch: (data >> ch) & 1
    rises = edges(bit(drdy))[0]
    ev = {"duration_s": np.array([len(data) / rate]), "events": np.array([0])}
    if len(rises) < 3:
        return ev
    dr = rises[:-1]                                # the last DRDY has no closing window
    ev["events"] = np.array([len(dr)])
    ev["drdy_period_us"] = np.diff(rises) * us
    ev["drdy_width_us"] = pulses(bit(drdy))[1] * us        # a latched INT line would never fall: no complete pulse, no data

    def chain(name, starts, widths=None, width_name=None):
        hit = first_in_window(starts, dr)
        seen = hit >= 0
        ev[name] = (starts[hit[seen]] - dr[seen]) * us
        ev[name + "_missing"] = np.array([int((~seen).sum())])
        if widths is not None:
            ev[width_name] = widths[hit[seen]] * us

    s, w = pulses(bit(isr));  chain("drdy_to_isr_us", s, w, "isr_width_us")
    s, w = pulses(bit(i2c));  chain("drdy_to_i2c_start_us", s, w, "i2c_width_us")
    chain("drdy_to_uart_start_us", edges(bit(uart))[1])      # falling edges on TX = start bits

    # PB0 is high around every push; the one that completes a window is much longer.  Split by width.
    s, w = pulses(bit(cls))
    hit = first_in_window(s, dr)
    seen = hit >= 0
    start = (s[hit[seen]] - dr[seen]) * us
    width = w[hit[seen]] * us
    short = width < long_us
    ev["push_missing"] = np.array([int((~seen).sum())])
    ev["drdy_to_push_start_us"], ev["push_width_us"] = start[short], width[short]
    ev["drdy_to_window_start_us"], ev["window_width_us"] = start[~short], width[~short]

    # Overlap: DRDY edges that arrive while the main loop is busy.  Busy = I2C read, UART tx + packet encode
    # (end of the I2C pulse until the next classifier pulse), window classification (long classifier pulses).
    si, wi = pulses(bit(i2c))
    sc, wc = pulses(bit(cls))
    lat = np.full(len(dr), np.nan)
    isr_s, _ = pulses(bit(isr))
    h = first_in_window(isr_s, dr)
    lat[h >= 0] = (isr_s[h[h >= 0]] - dr[h >= 0]) * us
    ends_i = si + wi
    k = np.searchsorted(sc, ends_i)
    keep = k < len(sc)
    long_ = wc * us >= long_us
    for name, (a, b) in (("i2c", (si, ends_i)), ("uart", (ends_i[keep], sc[k[keep]])),
                         ("window", (sc[long_], sc[long_] + wc[long_]))):
        m = inside(dr, a, b) & ~np.isnan(lat)
        ev[f"isr_latency_in_{name}_us"] = lat[m]
    ev["i2c_start_seq_us"], unmatched = paired_in_order(si, rises, us)   # all DRDY edges: no closing window needed
    ev["i2c_start_seq_unmatched"] = np.array([unmatched])
    return ev


def merge(evs):
    keys = {k for e in evs for k in e}
    return {k: np.concatenate([e[k] for e in evs if k in e]) for k in keys}


def summarise(ev):
    out = {"duration_s": float(ev["duration_s"].sum()), "drdy_edges": int(ev["events"].sum())}
    for _, key, miss in ROWS:
        if key in ev:
            out[key] = pct(ev[key])
        if miss and miss in ev:
            out[miss] = int(ev[miss].sum())
    return out


def analyse(data, rate, drdy=1, isr=3, i2c=4, uart=2, cls=0, long_us=50.0):
    return summarise(collect(data, rate, drdy, isr, i2c, uart, cls, long_us))


def show(out):
    print(f"{out['duration_s']:.3f} s, {out['drdy_edges']} DRDY events analysed")
    if "drdy_period_us" not in out:
        print("too few DRDY edges: wrong channel, or the board is not running?")
        return
    print(f"{'':44s}{'n':>7s}{'min':>11s}{'p50':>11s}{'p99':>11s}{'p99.9':>11s}{'max':>11s}")
    small = False
    for label, key, miss in ROWS:
        v = out.get(key)
        if v is None:
            print(f"{label:44s}  no data")
            continue
        small |= key == "drdy_to_isr_us" and v["n"] < 10000
        extra = f"   ({out[miss]} {MISS_TEXT.get(miss, 'DRDY events without one')})" if miss and out.get(miss) else ""
        print(f"{label:44s}{v['n']:7d}{v['min']:11.3f}{v['p50']:11.3f}{v['p99']:11.3f}{v['p99.9']:11.3f}{v['max']:11.3f}{extra}")
    if small:
        print("note: fewer than 10,000 events: p99 / p99.9 are not meaningful yet, read min / p50 / max.")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="raw captures (.bin) and/or event files (.npz) written by --events-out")
    ap.add_argument("--rate", type=float, default=24e6)
    for name, d in (("drdy", 1), ("isr", 3), ("i2c", 4), ("uart", 2), ("cls", 0)):
        ap.add_argument(f"--{name}", type=int, default=d, help=f"channel of {name} (default D{d})")
    ap.add_argument("--long-us", type=float, default=50.0, help="classifier pulses longer than this are window completions")
    ap.add_argument("--events-out", help="save the per-event numbers (merged) to this .npz")
    ap.add_argument("--json", help="also write the summary to this file")
    a = ap.parse_args(argv)
    evs = []
    for path in a.inputs:
        try:
            if path.endswith(".npz"):
                with np.load(path) as z:
                    evs.append({k: z[k] for k in z.files})
                continue
            data = np.fromfile(path, dtype=np.uint8)
        except OSError as e:
            print(f"cannot read {path}: {e.strerror}", file=sys.stderr)
            return 1
        if len(data) < 1000:
            print(f"{path}: empty or too short", file=sys.stderr)
            return 1
        evs.append(collect(data, a.rate, a.drdy, a.isr, a.i2c, a.uart, a.cls, a.long_us))
    ev = merge(evs)
    if a.events_out:
        np.savez_compressed(a.events_out, **ev)
    out = summarise(ev)
    show(out)
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
