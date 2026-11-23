#!/usr/bin/env python3
"""Compare two recorded sessions of a *stationary* sensor, channel by channel.

Why: the packet CRC is computed on the MCU *after* the I2C read, so a bit error on the
I2C bus (e.g. at 400 kHz) produces a valid CRC with wrong data.  On a board that lies
still, such errors show up as outliers or as a shifted mean/std versus a known-good
recording (the 100 kHz one).  This is a statistical check, not a proof: it can only
show a difference, and a small error rate may stay below what 12000 samples reveal.

usage: imu_compare.py A B      (each a session directory with frames.bin, or a frames.bin)
"""
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_proto  # noqa: E402

CHANNELS = ("ax", "ay", "az", "temp", "gx", "gy", "gz")
OUTLIER_K = 6.0          # robust z-score threshold (MAD-based)


def load_samples(path):
    """-> list of 7-tuples of raw register values, in frame order."""
    if os.path.isdir(path):
        path = os.path.join(path, "frames.bin")
    with open(path, "rb") as f:
        data = f.read()
    dec = imu_proto.FrameDecoder()
    return [imu_proto.decode_raw(fr.raw) for fr in dec.feed(data)]


def channel_stats(samples):
    """-> list (per channel) of dicts; plus the count of identical consecutive samples."""
    out = []
    for c in range(7):
        xs = [s[c] for s in samples]
        med = statistics.median(xs)
        mad = statistics.median(abs(x - med) for x in xs)
        scale = max(1.4826 * mad, 0.5)       # floor: values are integers
        out.append({
            "n": len(xs),
            "mean": statistics.fmean(xs),
            "std": statistics.pstdev(xs),
            "min": min(xs), "max": max(xs),
            "median": med,
            "outliers": sum(1 for x in xs if abs(x - med) > OUTLIER_K * scale),
        })
    repeats = sum(1 for a, b in zip(samples, samples[1:]) if a == b)
    return out, repeats


def compare(a, b):
    """Rows for the report: per channel, the stats of both sides and the mean shift
    (B - A) in units of the larger of the two standard deviations."""
    sa, ra = channel_stats(a)
    sb, rb = channel_stats(b)
    rows = []
    for name, x, y in zip(CHANNELS, sa, sb):
        sd = max(x["std"], y["std"])
        shift = (y["mean"] - x["mean"]) / sd if sd > 0 else (0.0 if y["mean"] == x["mean"] else float("inf"))
        rows.append((name, x, y, shift))
    return rows, ra, rb


def report(name_a, name_b, a, b):
    rows, ra, rb = compare(a, b)
    lines = [f"A = {name_a}  ({len(a)} samples)", f"B = {name_b}  ({len(b)} samples)", "",
             f"{'ch':<5}{'meanA':>10}{'meanB':>10}{'stdA':>8}{'stdB':>8}"
             f"{'minA':>7}{'maxA':>7}{'minB':>7}{'maxB':>7}{'outA':>6}{'outB':>6}{'shift/std':>10}"]
    for name, x, y, sh in rows:
        lines.append(f"{name:<5}{x['mean']:>10.2f}{y['mean']:>10.2f}{x['std']:>8.2f}{y['std']:>8.2f}"
                     f"{x['min']:>7}{x['max']:>7}{y['min']:>7}{y['max']:>7}"
                     f"{x['outliers']:>6}{y['outliers']:>6}{sh:>10.2f}")
    lines += ["", f"identical consecutive samples (all 7 channels): A {ra}, B {rb}",
              f"outlier = more than {OUTLIER_K:g} robust sigma (MAD) from the median"]
    return "\n".join(lines)


def main(argv):
    if len(argv) != 3:
        print(__doc__)
        return 2
    a, b = load_samples(argv[1]), load_samples(argv[2])
    if not a or not b:
        print("no valid frames in", argv[1] if not a else argv[2])
        return 1
    print(report(argv[1], argv[2], a, b))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
