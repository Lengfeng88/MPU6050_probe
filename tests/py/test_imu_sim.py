"""Tests for tools/imu_sim.py: format compatibility, determinism, and the physical sanity of each activity.

Run with:  python3 -m unittest discover -s tests/py -v
"""
import csv
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TOOLS = os.path.join(ROOT, "tools")
sys.path.insert(0, TOOLS)
import imu_proto as P  # noqa: E402


def run(script, *args):
    return subprocess.run([sys.executable, os.path.join(TOOLS, script), *args], capture_output=True, text=True)


def simulate(tmp, activity, *extra, seed=1, duration=20):
    r = run("imu_sim.py", activity, "--duration", str(duration), "--seed", str(seed), "--out", tmp, *extra)
    assert r.returncode == 0, r.stderr
    d = os.path.join(tmp, f"sim-s{seed:04d}_{activity}")
    r = run("imu_convert.py", d)
    assert r.returncode == 0, r.stderr
    with open(os.path.join(d, "samples.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    cols = {k: [float(x[k]) for x in rows] for k in rows[0]}
    return d, cols


def read_bytes(path):
    with open(path, "rb") as f:
        return f.read()


def read_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def mean(v):
    return sum(v) / len(v)


def std(v):
    m = mean(v)
    return math.sqrt(mean([(x - m) ** 2 for x in v]))


def peak_freq(x, fs, lo=0.4, hi=4.0, step=0.02):
    """Frequency with the largest DFT magnitude in [lo, hi] (plain Python, no numpy)."""
    m = mean(x)
    x = [v - m for v in x]
    best, bf = -1.0, 0.0
    f = lo
    while f <= hi:
        re = sum(v * math.cos(2 * math.pi * f * i / fs) for i, v in enumerate(x))
        im = sum(v * math.sin(2 * math.pi * f * i / fs) for i, v in enumerate(x))
        if re * re + im * im > best:
            best, bf = re * re + im * im, f
        f += step
    return bf


class SimFormat(unittest.TestCase):
    def test_frames_are_valid_packets_and_meta_matches_recorder_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            d, _ = simulate(tmp, "walk", duration=10)
            data = read_bytes(os.path.join(d, "frames.bin"))
            dec = P.FrameDecoder()
            frames = list(dec.feed(data))
            self.assertEqual((dec.crc_bad, dec.resync_bytes), (0, 0))
            meta = read_json(os.path.join(d, "meta.json"))
            self.assertEqual(len(frames), meta["frames"])
            self.assertEqual(len(data), 24 * meta["frames"])
            self.assertTrue(meta["simulated"])
            self.assertEqual(meta["status"], "complete")
            # every field the real recorder writes (docs/data_format.md) is present
            for k in ("format_version", "status", "label", "subject", "note", "port", "baud", "nominal_rate_hz",
                      "accel_lsb_per_g", "gyro_lsb_per_dps", "recording_started_utc", "recording_ended_utc",
                      "first_frame_utc", "last_frame_utc", "first_seq", "first_ts_us", "frames", "crc_bad",
                      "resync_bytes", "seq_gaps", "lost_events", "dt_us_min", "dt_us_max", "mcu_elapsed_s"):
                self.assertIn(k, meta)
            self.assertEqual((meta["seq_gaps"], meta["lost_events"]), (0, 0))
            self.assertAlmostEqual(meta["mcu_elapsed_s"], 10.0, delta=0.05)

    def test_same_seed_same_bytes_different_seed_different_bytes(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b, tempfile.TemporaryDirectory() as c:
            da, _ = simulate(a, "run", duration=5, seed=7)
            db, _ = simulate(b, "run", duration=5, seed=7)
            dc, _ = simulate(c, "run", duration=5, seed=8)
            fa, fb, fc = (read_bytes(os.path.join(x, "frames.bin")) for x in (da, db, dc))
            self.assertEqual(fa, fb)
            self.assertNotEqual(fa, fc)

    def test_seq_and_timestamp_wraparound_is_unwrapped_by_the_converter(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, c = simulate(tmp, "still", "--start-seq", "65530", "--start-ts", "4294960000", duration=3)
            seq = [int(v) for v in c["seq"]]
            self.assertEqual(seq, list(range(len(seq))))                 # unwrapped across 65535 -> 0
            t = c["t_s"]
            self.assertTrue(all(b > a for a, b in zip(t, t[1:])))        # monotonic across the ts wrap
            self.assertAlmostEqual(t[-1], 3.0, delta=0.05)

    def test_drops_become_seq_gaps_that_match_meta_and_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            d, c = simulate(tmp, "walk", "--drop-prob", "0.02", duration=30)
            meta = read_json(os.path.join(d, "meta.json"))
            self.assertGreater(meta["seq_gaps"], 0)
            self.assertEqual(sum(int(g) for g in c["gap"]), meta["seq_gaps"])
            seq = [int(v) for v in c["seq"]]
            self.assertEqual(seq[-1] - seq[0] + 1 - len(seq), meta["lost_events"])

    def test_sample_rate_matches_the_real_board(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, c = simulate(tmp, "still", duration=20)
            rate = (len(c["t_s"]) - 1) / (c["t_s"][-1] - c["t_s"][0])
            self.assertAlmostEqual(rate, 100.22, delta=0.1)              # 9978 us step measured on the real board


class SimPhysics(unittest.TestCase):
    def test_gravity_magnitude_for_static_activities(self):
        with tempfile.TemporaryDirectory() as tmp:
            for act in ("still", "sit", "stand"):
                _, c = simulate(tmp, act, duration=20)
                mag = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(c["ax_g"], c["ay_g"], c["az_g"])]
                self.assertAlmostEqual(mean(mag), 1.0, delta=0.12, msg=act)    # 1 g plus the random bias
                self.assertLess(std(mag), 0.03, act)

    def test_sit_and_stand_differ_in_gravity_direction(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, sit = simulate(tmp, "sit", duration=20)
            _, stand = simulate(tmp, "stand", duration=20)
            self.assertLess(mean(stand["ax_g"]), -0.8)                   # thigh vertical: gravity along the thigh axis
            self.assertGreater(mean(sit["az_g"]), 0.8)                   # thigh horizontal: gravity across it
            self.assertLess(abs(mean(sit["ax_g"])), 0.4)

    def test_still_has_almost_no_angular_rate_and_sit_stand_a_little(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, still = simulate(tmp, "still", duration=20)
            _, stand = simulate(tmp, "stand", duration=20)
            self.assertLess(std(still["gy_dps"]), 0.1)
            self.assertLess(std(stand["gy_dps"]), 5.0)                   # smooth sway, not jitter

    def test_gait_frequency_matches_the_drawn_stride_frequency(self):
        with tempfile.TemporaryDirectory() as tmp:
            for act in ("walk", "run"):
                d, c = simulate(tmp, act, duration=20)
                want = read_json(os.path.join(d, "meta.json"))["sim_params"]["stride_hz"]
                fs = (len(c["t_s"]) - 1) / (c["t_s"][-1] - c["t_s"][0])
                self.assertAlmostEqual(peak_freq(c["gy_dps"], fs), want, delta=0.06, msg=act)

    def test_energy_increases_still_stand_walk_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            e = {}
            for act in ("stand", "walk", "run"):
                _, c = simulate(tmp, act, duration=20)
                e[act] = std(c["gy_dps"])
            self.assertLess(e["stand"], e["walk"])
            self.assertLess(e["walk"], e["run"])

    def test_sensor_ranges_clip_like_the_real_firmware(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, c = simulate(tmp, "run", duration=20)
            # full scale is int16 (32768 counts) divided by the firmware's LSB-per-unit factors
            a_max = 32768 / P.ACCEL_LSB_PER_G + 1e-4
            g_max = 32768 / P.GYRO_LSB_PER_DPS + 1e-4
            self.assertLessEqual(max(abs(v) for v in c["ax_g"] + c["ay_g"] + c["az_g"]), a_max)
            self.assertLessEqual(max(abs(v) for v in c["gx_dps"] + c["gy_dps"] + c["gz_dps"]), g_max)
            self.assertGreater(max(abs(v) for v in c["gy_dps"]), 249.0)       # and running does reach the rail

    def test_bad_arguments_are_rejected(self):
        self.assertEqual(run("imu_sim.py", "flying").returncode, 2)
        self.assertEqual(run("imu_sim.py", "walk", "--duration", "0").returncode, 2)
        self.assertEqual(run("imu_sim.py", "walk", "--drop-prob", "1").returncode, 2)


if __name__ == "__main__":
    unittest.main()
