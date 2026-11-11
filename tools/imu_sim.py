#!/usr/bin/env python3
"""Generate SIMULATED IMU sessions in the same on-disk format as imu_record.py.

    python3 tools/imu_sim.py walk --duration 60 --seed 3
    python3 tools/imu_sim.py run  --duration 60 --seed 10 --sessions 5   # seeds 10..14

Creates  <out>/sim-s<seed>_<activity>/  with frames.bin (real 24-byte packets, CRC included) and
meta.json (same fields as a real recording, plus "simulated": true and "sim_params"), so
imu_convert.py and everything after it work unchanged.

THIS IS NOT REAL DATA.  It is a physics-flavoured model of a sensor in a trouser pocket / on the
thigh (x along the thigh toward the knee): gravity direction from the thigh angle, a gait swing
(thigh angle with a second harmonic), the tangential / centripetal acceleration of the swing,
body bounce and heel-strike impulses, per-session bias / frequency / amplitude variation, and the
noise level and timestamp step measured on the real board.  The sensor ranges (+-2 g, +-250 deg/s)
are applied, so hard running clips like the real firmware would.  Use it to test the pipeline and
to benchmark code; a classifier trained on it only learns this model, so its accuracy says nothing
about the real world.

Activities: still (sensor lying on a desk), sit, stand, walk, run.
"""
import argparse
import datetime as dt
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_proto as P  # noqa: E402

FORMAT_VERSION = 1
NOMINAL_RATE_HZ = 100
ACTIVITIES = ("still", "sit", "stand", "walk", "run")
G = 9.81
SENSOR_R_M = 0.15            # distance of the sensor from the hip joint along the thigh
DEG = math.pi / 180.0


def draw_params(activity: str, rng: random.Random) -> dict:
    """Per-session random parameters ("a different person / sensor placement / day")."""
    p = {
        "activity": activity,
        "accel_bias_g": [round(rng.gauss(0, 0.03), 5) for _ in range(3)],
        "gyro_bias_dps": [round(rng.gauss(0, 2.0), 4) for _ in range(3)],
        "accel_noise_g": 0.002,      # measured on the real board, sensor flat on a desk
        "gyro_noise_dps": 0.025,
        "temp_c0": round(rng.uniform(20.0, 30.0), 3),
        "phase0": round(rng.uniform(0, 2 * math.pi), 5),
    }
    if activity == "walk":
        p.update(stride_hz=rng.uniform(0.80, 1.10), swing_deg=rng.uniform(15, 25),
                 bounce_g=rng.uniform(0.15, 0.30), impact_g=rng.uniform(0.2, 0.5),
                 theta0_deg=rng.uniform(0, 8))
    elif activity == "run":
        p.update(stride_hz=rng.uniform(1.25, 1.60), swing_deg=rng.uniform(28, 42),
                 bounce_g=rng.uniform(0.6, 1.2), impact_g=rng.uniform(0.8, 1.8),
                 theta0_deg=rng.uniform(5, 15))
    elif activity == "sit":
        p.update(theta0_deg=rng.uniform(75, 100), sway_deg=0.4)
    elif activity == "stand":
        p.update(theta0_deg=rng.uniform(0, 6), sway_deg=0.6)
    if activity in ("sit", "stand"):
        # slow postural sway: three low-frequency components (Hz, relative amplitude, phase)
        p["sway"] = [[round(rng.uniform(lo, hi), 4), round(rng.uniform(0.4, 1.0), 4),
                      round(rng.uniform(0, 2 * math.pi), 4)]
                     for lo, hi in ((0.08, 0.2), (0.2, 0.45), (0.45, 0.8))]
    elif activity == "still":
        p.update(tilt_x=rng.gauss(0, 0.08), tilt_y=rng.gauss(0, 0.08))
    for k, v in list(p.items()):
        if isinstance(v, float):
            p[k] = round(v, 5)
    return p


class Model:
    """Evaluates the noise-free sensor signal (specific force in g, angular rate in deg/s)."""

    def __init__(self, p: dict, rng: random.Random):
        self.p, self.rng = p, rng

    def _sway(self, t: float):
        """-> (angle_deg, rate_dps) of a smooth, slow postural sway (sum of sinusoids)."""
        comps = self.p["sway"]
        norm = math.sqrt(sum(c[1] ** 2 for c in comps) / 2.0)        # so the std dev equals sway_deg
        amp = self.p["sway_deg"] / norm
        ang = rate = 0.0
        for f, r, ph in comps:
            w = 2 * math.pi * f
            ang += amp * r * math.sin(w * t + ph)
            rate += amp * r * w * math.cos(w * t + ph)
        return ang, rate

    def at(self, t: float):
        p, act = self.p, self.p["activity"]
        if act == "still":
            x, y = p["tilt_x"], p["tilt_y"]
            n = math.sqrt(x * x + y * y + 1)
            return (x / n, y / n, 1 / n), (0.0, 0.0, 0.0)
        if act in ("sit", "stand"):
            sway, rate = self._sway(t)
            th = (p["theta0_deg"] + sway) * DEG
            return (-math.cos(th), 0.0, math.sin(th)), (0.0, rate, 0.0)
        # walk / run: thigh swings about the hip (y axis)
        f, A = p["stride_hz"], p["swing_deg"]
        w = 2 * math.pi * f
        ph = w * t + p["phase0"]
        th_deg = p["theta0_deg"] + A * math.sin(ph) + 0.25 * A * math.sin(2 * ph + 0.7)
        om_deg = (A * math.cos(ph) + 0.5 * A * math.cos(2 * ph + 0.7)) * w          # deg/s
        al_deg = (-A * math.sin(ph) - A * math.sin(2 * ph + 0.7)) * w * w           # deg/s^2
        th, om, al = th_deg * DEG, om_deg * DEG, al_deg * DEG
        step_ph = (2 * ph / (2 * math.pi) + 0.2) % 1.0                              # two steps per stride
        d = min(step_ph, 1 - step_ph) / (2 * f)                                     # seconds from a footfall
        a_vert = p["bounce_g"] * math.sin(2 * ph + 1.0) + p["impact_g"] * math.exp(-(d / 0.015) ** 2)
        up = (-math.cos(th), 0.0, math.sin(th))
        swing = (-SENSOR_R_M * om * om / G, 0.0, -SENSOR_R_M * al / G)
        fx = tuple(up[i] * (1.0 + a_vert) + swing[i] for i in range(3))
        aw = A * w
        return fx, (0.12 * aw * math.sin(ph + 1.1), om_deg, 0.08 * aw * math.sin(2 * ph + 0.3))


def clip16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


def utc_iso(t: float) -> str:
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).isoformat(timespec="milliseconds")


def simulate_session(args, seed: int) -> str:
    rng = random.Random(seed)
    p = draw_params(args.activity, rng)
    model = Model(p, rng)
    start_seq = args.start_seq if args.start_seq is not None else rng.randrange(65536)
    ts0 = args.start_ts if args.start_ts is not None else rng.randrange(1 << 32)
    n_events = int(args.duration * 1e6 / args.period_us)

    session = os.path.join(args.out, f"sim-s{seed:04d}_{args.activity}")
    os.makedirs(session, exist_ok=False)

    stored = 0
    seq_gaps = lost = 0
    dts = []
    first_ts = last_ts = None
    last_stored_event = None
    drop_left = 0
    with open(os.path.join(session, "frames.bin"), "wb") as out:
        for k in range(n_events):
            t_us = round(k * args.period_us + rng.gauss(0, args.jitter_us))
            t = t_us / 1e6
            f, w = model.at(t)                       # always advance the model, even for lost events
            if drop_left == 0 and args.drop_prob > 0 and rng.random() < args.drop_prob:
                drop_left = rng.randint(1, 3)
            if drop_left:
                drop_left -= 1
                continue
            temp = p["temp_c0"] + 0.002 * t + rng.gauss(0, 0.1)
            ab, gb = p["accel_bias_g"], p["gyro_bias_dps"]
            a = [clip16((f[i] + ab[i] + rng.gauss(0, p["accel_noise_g"])) * P.ACCEL_LSB_PER_G) for i in range(3)]
            g = [clip16((w[i] + gb[i] + rng.gauss(0, p["gyro_noise_dps"])) * P.GYRO_LSB_PER_DPS) for i in range(3)]
            tr = clip16((temp - 36.53) * 340.0)
            raw = b"".join(v.to_bytes(2, "big", signed=True) for v in (a[0], a[1], a[2], tr, g[0], g[1], g[2]))
            out.write(P.encode((start_seq + k) & 0xFFFF, (ts0 + t_us) & 0xFFFFFFFF, raw))
            stored += 1
            if last_stored_event is not None:
                dts.append(t_us - last_ts)
                if k - last_stored_event != 1:
                    seq_gaps += 1
                    lost += k - last_stored_event - 1
            if first_ts is None:
                first_ts, first_event = t_us, k
            last_ts, last_stored_event = t_us, k

    now = dt.datetime.now(dt.timezone.utc).timestamp()
    meta = {
        "format_version": FORMAT_VERSION,
        "status": "complete",
        "label": args.activity,
        "subject": args.subject or f"sim{seed}",
        "note": args.note or "SIMULATED data (tools/imu_sim.py), not a real recording",
        "port": "simulated",
        "baud": 0,
        "nominal_rate_hz": NOMINAL_RATE_HZ,
        "accel_lsb_per_g": P.ACCEL_LSB_PER_G,
        "gyro_lsb_per_dps": P.GYRO_LSB_PER_DPS,
        "recording_started_utc": utc_iso(now),
        "frames": stored,
        "crc_bad": 0,
        "resync_bytes": 0,
        "seq_gaps": seq_gaps,
        "lost_events": lost,
        "dt_us_min": min(dts) if dts else None,
        "dt_us_max": max(dts) if dts else None,
        "recording_ended_utc": utc_iso(now),
        "simulated": True,
        "sim_params": {**p, "seed": seed, "period_us": args.period_us, "jitter_us": args.jitter_us,
                       "drop_prob": args.drop_prob, "placement": "thigh/pocket (model)"},
    }
    if stored:
        meta["first_frame_utc"] = utc_iso(now)
        meta["last_frame_utc"] = utc_iso(now)
        meta["mcu_elapsed_s"] = (last_ts - first_ts) / 1e6
        meta["first_ts_us"] = (ts0 + first_ts) & 0xFFFFFFFF
        meta["first_seq"] = (start_seq + first_event) & 0xFFFF
    tmp = os.path.join(session, "meta.json.tmp")
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2)
        fh.write("\n")
    os.replace(tmp, os.path.join(session, "meta.json"))
    print(f"simulated {args.activity} seed={seed}: {stored} frames, seq_gaps={seq_gaps}, lost={lost} -> {session}")
    return session


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("activity", choices=ACTIVITIES)
    ap.add_argument("--duration", type=float, default=60.0, help="seconds per session (default 60)")
    ap.add_argument("--seed", type=int, default=1, help="first seed; the same seed gives byte-identical frames.bin")
    ap.add_argument("--sessions", type=int, default=1, help="number of sessions (seeds seed, seed+1, ...)")
    ap.add_argument("--out", default="data", help="parent directory (default ./data)")
    ap.add_argument("--subject", default="", help="default: sim<seed>")
    ap.add_argument("--note", default="")
    ap.add_argument("--period-us", type=float, default=9978.0, help="mean time between samples (real board: ~9978)")
    ap.add_argument("--jitter-us", type=float, default=4.0, help="timestamp jitter, std dev in microseconds")
    ap.add_argument("--drop-prob", type=float, default=0.0, help="chance per event of starting a 1-3 event loss")
    ap.add_argument("--start-seq", type=int, default=None, help="first seq (default random; use 65530 to test wrap)")
    ap.add_argument("--start-ts", type=int, default=None, help="first ts_us (default random; use 4294960000 to test wrap)")
    args = ap.parse_args(argv)
    if args.duration <= 0 or args.sessions < 1 or not (0 <= args.drop_prob < 1):
        ap.error("need --duration > 0, --sessions >= 1, 0 <= --drop-prob < 1")
    for i in range(args.sessions):
        simulate_session(args, args.seed + i)
    return 0


if __name__ == "__main__":
    sys.exit(main())
