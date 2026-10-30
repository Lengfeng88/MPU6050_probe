#!/usr/bin/env python3
"""Convert a recorded session (frames.bin + meta.json) to CSV and/or NumPy.

    python3 tools/imu_convert.py data/20261029-180000_walking            # -> samples.csv
    python3 tools/imu_convert.py data/20261029-180000_walking --npz      # also samples.npz (needs numpy)

Columns: seq (event number, starts at 0), t_s (seconds since the first sample, from the MCU
clock), gap (1 = events are missing right before this sample), ax_g ay_g az_g, gx_dps gy_dps
gz_dps, temp_c.  With --raw the raw int16 register values are written instead of physical units.
Samples are NOT resampled: after a gap the time step is simply larger.
"""
import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_proto as P  # noqa: E402


def read_frames(session: str):
    path = os.path.join(session, "frames.bin")
    data = open(path, "rb").read()
    if len(data) % P.FRAME_LEN:
        print(f"warning: {path} ends with a partial frame ({len(data) % P.FRAME_LEN} bytes), ignored",
              file=sys.stderr)
    dec = P.FrameDecoder()
    frames = list(dec.feed(data[:len(data) - len(data) % P.FRAME_LEN]))
    if dec.crc_bad or dec.resync_bytes:
        print(f"warning: frames.bin is damaged (crc_bad={dec.crc_bad}, resync_bytes={dec.resync_bytes})",
              file=sys.stderr)
    return frames


def samples(frames, raw: bool):
    """Yield (seq, t_s, gap, values...) with unwrapped seq and time."""
    useq, uts = P.Unwrapper(16), P.Unwrapper(32)
    last = None
    for f in frames:
        s, t = useq(f.seq), uts(f.ts_us)
        gap = int(last is not None and s - last != 1)
        last = s
        if raw:
            ax, ay, az, tr, gx, gy, gz = P.decode_raw(f.raw)
            vals = (ax, ay, az, gx, gy, gz, tr)
        else:
            vals = P.to_physical(f.raw)
        yield (s, t / 1e6, gap) + vals


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="session directory made by imu_record.py")
    ap.add_argument("--raw", action="store_true", help="raw int16 values instead of g / deg/s / deg C")
    ap.add_argument("--csv", default=None, help="CSV path (default: <session>/samples.csv)")
    ap.add_argument("--npz", nargs="?", const="", default=None, metavar="PATH",
                    help="also write a NumPy .npz (default path: <session>/samples.npz)")
    args = ap.parse_args(argv)

    rows = list(samples(read_frames(args.session), args.raw))
    if not rows:
        print("no samples in session", file=sys.stderr)
        return 1

    unit = ("ax", "ay", "az", "gx", "gy", "gz", "temp_raw") if args.raw else \
           ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "temp_c")
    out = args.csv or os.path.join(args.session, "samples.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("seq", "t_s", "gap") + unit)
        for r in rows:
            if args.raw:
                w.writerow((r[0], f"{r[1]:.6f}", r[2]) + r[3:])
            else:
                w.writerow((r[0], f"{r[1]:.6f}", r[2]) + tuple(f"{v:.6f}" for v in r[3:]))
    print(f"{len(rows)} samples -> {out}")

    if args.npz is not None:
        import numpy as np
        meta = {}
        mp = os.path.join(args.session, "meta.json")
        if os.path.exists(mp):
            meta = json.load(open(mp, encoding="utf-8"))
        a = np.array(rows, dtype=np.float64)
        npz = args.npz or os.path.join(args.session, "samples.npz")
        np.savez(npz, seq=a[:, 0].astype(np.int64), t_s=a[:, 1], gap=a[:, 2].astype(np.uint8),
                 accel=a[:, 3:6], gyro=a[:, 6:9], temp=a[:, 9],
                 label=np.array(meta.get("label", "")), subject=np.array(meta.get("subject", "")),
                 units=np.array("raw" if args.raw else "g,dps,degC"))
        print(f"-> {npz}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
