#!/usr/bin/env python3
"""Cut recorded sessions into windows and compute cheap per-window features (needs numpy).

    python3 tools/imu_convert.py data/<session> --npz               # once per session
    python3 tools/imu_features.py data/sim-* --out data/features_sim.npz

Windows are a fixed number of samples (default 200 = 2 s at ~100 Hz) with 50 % overlap.  A window
never contains a gap (a sample flagged gap=1 inside it): such windows are dropped, not interpolated.
Every window keeps the name of its session ("group"), so train / test can be split by session --
windows of one session overlap and look alike, so they must never end up on both sides.

All features are simple sums / means / min / max over the window, cheap to compute on a
microcontroller (no FFT).  They are computed in physical units (g, deg/s) from the converter's npz.
"""
import argparse
import json
import os
import sys

import numpy as np

FEATURES = (
    "ax_mean", "ay_mean", "az_mean",                  # gravity direction = orientation of the sensor
    "ax_std", "ay_std", "az_std",                     # movement energy per axis
    "gx_std", "gy_std", "gz_std",
    "amag_mean", "amag_std", "amag_min", "amag_max",  # |a| in g
    "gmag_mean", "gmag_std",                          # |w| in deg/s
    "amag_jerk",                                      # mean |d|a|/dt| between neighbouring samples
    "gy_zcr",                                         # zero crossings of the mean-removed gy (cadence)
)


def window_starts(gap: np.ndarray, size: int, step: int) -> np.ndarray:
    """Start indices of the windows that contain no gap.

    gap[i] == 1 means events are missing right *before* sample i, so a window [s, s+size) is broken
    if any of gap[s+1 : s+size] is set (gap[s] describes the hole before the window, which is fine).
    """
    n = len(gap)
    if n < size:
        return np.zeros(0, dtype=np.int64)
    starts = np.arange(0, n - size + 1, step)
    cum = np.concatenate(([0], np.cumsum(gap.astype(np.int64))))   # cum[i] = number of gaps in gap[:i]
    broken = cum[starts + size] - cum[starts + 1]
    return starts[broken == 0]


def window_features(acc: np.ndarray, gyr: np.ndarray) -> np.ndarray:
    """acc, gyr: (size, 3) arrays in g and deg/s -> feature vector in FEATURES order."""
    amag = np.linalg.norm(acc, axis=1)
    gmag = np.linalg.norm(gyr, axis=1)
    gy = gyr[:, 1] - gyr[:, 1].mean()
    s = np.signbit(gy).astype(np.int8)
    zcr = float(np.count_nonzero(s[1:] != s[:-1]))
    return np.array([
        *acc.mean(axis=0), *acc.std(axis=0), *gyr.std(axis=0),
        amag.mean(), amag.std(), amag.min(), amag.max(),
        gmag.mean(), gmag.std(),
        np.abs(np.diff(amag)).mean(),
        zcr,
    ], dtype=np.float64)


def load_session(path: str) -> dict:
    npz = os.path.join(path, "samples.npz")
    if not os.path.exists(npz):
        raise FileNotFoundError(f"{npz} not found (run: python3 tools/imu_convert.py {path} --npz)")
    d = np.load(npz)
    return {k: d[k] for k in d.files}


def session_is_simulated(path: str) -> bool:
    """True if meta.json says the session came from tools/imu_sim.py (not a real recording)."""
    try:
        with open(os.path.join(path, "meta.json"), encoding="utf-8") as f:
            return bool(json.load(f).get("simulated", False))
    except (OSError, ValueError):
        return False


def session_windows(path: str, size: int = 200, step: int = 100):
    """-> (X, label, n_dropped) for one session directory."""
    s = load_session(path)
    starts = window_starts(s["gap"], size, step)
    total = 0 if len(s["gap"]) < size else len(range(0, len(s["gap"]) - size + 1, step))
    X = np.array([window_features(s["accel"][i:i + size], s["gyro"][i:i + size]) for i in starts])
    return X.reshape(len(starts), len(FEATURES)), str(s["label"]), total - len(starts)


def build(paths, size: int = 200, step: int = 100):
    Xs, ys, gs, dropped = [], [], [], 0
    for p in paths:
        X, label, d = session_windows(p, size, step)
        dropped += d
        Xs.append(X)
        ys += [label] * len(X)
        gs += [os.path.basename(os.path.normpath(p))] * len(X)
    X = np.concatenate(Xs) if Xs else np.zeros((0, len(FEATURES)))
    return X, np.array(ys), np.array(gs), dropped


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", help="session directories that contain samples.npz")
    ap.add_argument("--size", type=int, default=200, help="samples per window (default 200 = 2 s)")
    ap.add_argument("--step", type=int, default=100, help="samples between window starts (default 100 = 50%% overlap)")
    ap.add_argument("--out", default="data/features.npz")
    args = ap.parse_args(argv)
    if args.size < 2 or args.step < 1:
        ap.error("need --size >= 2 and --step >= 1")

    X, y, g, dropped = build(args.sessions, args.size, args.step)
    if len(X) == 0:
        print("no windows (sessions shorter than one window?)", file=sys.stderr)
        return 1
    sim_by_group = {os.path.basename(os.path.normpath(p)): session_is_simulated(p) for p in args.sessions}
    simulated = np.array([sim_by_group[name] for name in g], dtype=bool)
    np.savez(args.out, X=X, y=y, group=g, features=np.array(FEATURES),
             size=args.size, step=args.step, simulated=simulated)
    print(f"{len(X)} windows from {len(set(g))} sessions ({dropped} dropped because they contained a gap) -> {args.out}")
    if simulated.any():
        print(f"  note: {len(set(g[simulated]))} of these sessions are SIMULATED" +
              ("" if simulated.all() else " (mixed with real ones - do not train on them together)"))
    for label in sorted(set(y)):
        print(f"  {label:6s} {int((y == label).sum()):5d} windows, {len(set(g[y == label]))} sessions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
