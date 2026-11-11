"""Tests for tools/imu_features.py (skipped when numpy is not installed)."""
import math
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TOOLS = os.path.join(ROOT, "tools")
sys.path.insert(0, TOOLS)

try:
    import numpy as np
    import imu_features as F
except ImportError:                       # numpy missing: the receiver tools do not need it
    np = None


def run(script, *args):
    r = subprocess.run([sys.executable, os.path.join(TOOLS, script), *args], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r


def make_session(tmp, activity, *extra, seed=1, duration=20):
    run("imu_sim.py", activity, "--duration", str(duration), "--seed", str(seed), "--out", tmp, *extra)
    d = os.path.join(tmp, f"sim-s{seed:04d}_{activity}")
    run("imu_convert.py", d, "--npz")
    return d


@unittest.skipIf(np is None, "numpy not installed")
class Windows(unittest.TestCase):
    def test_window_count_without_gaps(self):
        s = F.window_starts(np.zeros(1000, dtype=np.uint8), 200, 100)
        self.assertEqual(list(s), list(range(0, 801, 100)))

    def test_too_short_gives_no_windows(self):
        self.assertEqual(len(F.window_starts(np.zeros(150, dtype=np.uint8), 200, 100)), 0)

    def test_a_gap_drops_exactly_the_windows_that_contain_it(self):
        gap = np.zeros(1000, dtype=np.uint8)
        gap[450] = 1                                   # events missing right before sample 450
        self.assertEqual(list(F.window_starts(gap, 200, 100)), [0, 100, 200, 500, 600, 700, 800])
        gap = np.zeros(1000, dtype=np.uint8)
        gap[400] = 1                                   # hole right before the window that starts at 400: fine
        self.assertEqual(list(F.window_starts(gap, 200, 100)), [0, 100, 200, 400, 500, 600, 700, 800])

    def test_gap_at_first_sample_of_window_is_allowed_but_second_is_not(self):
        gap = np.zeros(400, dtype=np.uint8)
        gap[100] = 1
        self.assertIn(100, F.window_starts(gap, 200, 100))
        gap = np.zeros(400, dtype=np.uint8)
        gap[101] = 1
        self.assertNotIn(100, F.window_starts(gap, 200, 100))


@unittest.skipIf(np is None, "numpy not installed")
class Features(unittest.TestCase):
    def test_vector_length_matches_names(self):
        v = F.window_features(np.zeros((200, 3)), np.zeros((200, 3)))
        self.assertEqual(len(v), len(F.FEATURES))
        self.assertEqual(len(set(F.FEATURES)), len(F.FEATURES))

    def test_constant_signal(self):
        acc = np.tile([0.0, 0.0, 1.0], (200, 1))
        v = dict(zip(F.FEATURES, F.window_features(acc, np.zeros((200, 3)))))
        self.assertEqual((v["ax_mean"], v["ay_mean"], v["az_mean"]), (0.0, 0.0, 1.0))
        for k in ("ax_std", "gx_std", "gy_std", "gz_std", "amag_std", "gmag_mean", "amag_jerk", "gy_zcr"):
            self.assertEqual(v[k], 0.0, k)
        self.assertEqual((v["amag_mean"], v["amag_min"], v["amag_max"]), (1.0, 1.0, 1.0))

    def test_known_values(self):
        n = 200
        acc = np.zeros((n, 3))
        acc[:, 0] = np.where(np.arange(n) % 2 == 0, 1.0, -1.0)         # +-1 g on x, mean 0, std 1
        t = np.arange(n) / 100.0
        gyr = np.zeros((n, 3))
        gyr[:, 1] = 100.0 * np.sin(2 * math.pi * 1.0 * t + 0.3)          # 1 Hz, 2 s: ~4 zero crossings
        v = dict(zip(F.FEATURES, F.window_features(acc, gyr)))
        self.assertAlmostEqual(v["ax_std"], 1.0)
        self.assertAlmostEqual(v["amag_mean"], 1.0)
        self.assertAlmostEqual(v["gy_std"], 100.0 / math.sqrt(2), delta=1.0)
        self.assertTrue(3 <= v["gy_zcr"] <= 5, v["gy_zcr"])


@unittest.skipIf(np is None, "numpy not installed")
class EndToEnd(unittest.TestCase):
    def test_windows_never_span_a_gap_in_real_session_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = make_session(tmp, "walk", "--drop-prob", "0.01", duration=60)
            s = F.load_session(d)
            self.assertGreater(int(s["gap"].sum()), 0)
            starts = F.window_starts(s["gap"], 200, 100)
            self.assertGreater(len(starts), 0)
            for i in starts:
                self.assertEqual(int(s["seq"][i + 199] - s["seq"][i]), 199)   # contiguous events
            X, label, dropped = F.session_windows(d)
            self.assertEqual(label, "walk")
            self.assertEqual(X.shape, (len(starts), len(F.FEATURES)))
            self.assertGreater(dropped, 0)

    def test_build_keeps_session_names_as_groups(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = make_session(tmp, "stand", seed=3)
            b = make_session(tmp, "run", seed=4)
            X, y, g, dropped = F.build([a, b])
            self.assertEqual(set(g), {"sim-s0003_stand", "sim-s0004_run"})
            self.assertEqual(set(y), {"stand", "run"})
            self.assertEqual(dropped, 0)
            self.assertEqual(len(X), len(y))

    def test_features_separate_the_simulated_activities_in_the_expected_way(self):
        with tempfile.TemporaryDirectory() as tmp:
            feats = {}
            for i, act in enumerate(("stand", "sit", "walk", "run")):
                X, _, _ = F.session_windows(make_session(tmp, act, seed=10 + i))
                feats[act] = dict(zip(F.FEATURES, X.mean(axis=0)))
            self.assertLess(feats["stand"]["ax_mean"], -0.8)               # thigh vertical
            self.assertGreater(feats["sit"]["az_mean"], 0.8)               # thigh horizontal
            self.assertLess(feats["stand"]["gmag_std"], feats["walk"]["gmag_std"])
            self.assertLess(feats["walk"]["gmag_std"], feats["run"]["gmag_std"])
            self.assertLess(feats["walk"]["gy_zcr"], feats["run"]["gy_zcr"] + 1)

    def test_cli_writes_npz_and_reports_counts(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = make_session(tmp, "walk", seed=5)
            out = os.path.join(tmp, "f.npz")
            r = run("imu_features.py", a, "--out", out)
            self.assertIn("windows from 1 sessions", r.stdout)
            d = np.load(out)
            self.assertEqual(d["X"].shape[1], len(F.FEATURES))
            self.assertEqual(list(d["features"]), list(F.FEATURES))
            self.assertTrue(d["simulated"].all())                       # sim sessions are flagged as such
            self.assertEqual(len(d["simulated"]), len(d["X"]))
            self.assertIn("SIMULATED", r.stdout)

    def test_real_sessions_are_not_flagged(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = make_session(tmp, "walk", seed=6)
            import json
            with open(os.path.join(a, "meta.json"), encoding="utf-8") as f:
                meta = json.load(f)
            del meta["simulated"]                                       # what a recording from imu_record.py looks like
            with open(os.path.join(a, "meta.json"), "w", encoding="utf-8") as f:
                json.dump(meta, f)
            self.assertFalse(F.session_is_simulated(a))
            self.assertFalse(F.session_is_simulated(os.path.join(tmp, "does-not-exist")))

    def test_missing_npz_is_a_clear_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            run("imu_sim.py", "walk", "--duration", "5", "--out", tmp)
            r = subprocess.run([sys.executable, os.path.join(TOOLS, "imu_features.py"),
                                os.path.join(tmp, "sim-s0001_walk"), "--out", os.path.join(tmp, "f.npz")],
                               capture_output=True, text=True)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn("samples.npz", r.stderr)


if __name__ == "__main__":
    unittest.main()
