"""Parity tests: App/classify (C, built with the host compiler) against tools/imu_features.py (numpy).

The C feature code is compiled twice: with double (must match numpy to rounding noise) and with
float (what the MCU runs; must match to float precision).  Skipped without numpy or a C compiler.
"""
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TOOLS = os.path.join(ROOT, "tools")
CLASSIFY = os.path.join(ROOT, "App", "classify")
sys.path.insert(0, TOOLS)

try:
    import numpy as np
    import imu_features as F
    import imu_train as T
except ImportError:
    np = None

CC = shutil.which("cc") or shutil.which("gcc")

HARNESS = r'''
#include <stdio.h>
#include "classify/imu_activity.h"
int main(void)
{
    static imu_activity_t a;
    imu_activity_init(&a);
    unsigned seq, b[14];
    while (scanf("%u", &seq) == 1) {
        uint8_t raw[14];
        for (int i = 0; i < 14; i++) { if (scanf("%u", &b[i]) != 1) return 1; raw[i] = (uint8_t)b[i]; }
        IMU_FP_T f[IMU_N_FEAT];
        int c = imu_activity_push_raw(&a, (uint16_t)seq, raw, f);
        if (c >= 0) {
            printf("%d", c);
            for (unsigned i = 0; i < IMU_N_FEAT; i++) printf(" %.17g", (double)f[i]);
            printf("\n");
        }
    }
    return 0;
}
'''


def to_raw_bytes(accel, gyro):
    """physical -> the 14 register bytes (temp = 0), the inverse of what imu_convert.py does"""
    a = np.rint(accel * 16384.0).astype(">i2")
    g = np.rint(gyro * 131.0).astype(">i2")
    out = np.zeros((len(a), 14), dtype=np.uint8)
    out[:, 0:6] = a.view(np.uint8).reshape(-1, 6)
    out[:, 8:14] = g.view(np.uint8).reshape(-1, 6)
    return out


def build(tmp, fp, tree_header):
    for name in os.listdir(CLASSIFY):
        shutil.copy(os.path.join(CLASSIFY, name), tmp)
    with open(os.path.join(tmp, "imu_tree.h"), "w") as f:      # a tree that matches the data under test
        f.write(tree_header)
    os.makedirs(os.path.join(tmp, "classify"))
    for name in ("imu_window.h", "imu_activity.h"):
        shutil.copy(os.path.join(CLASSIFY, name), os.path.join(tmp, "classify", name))
    with open(os.path.join(tmp, "h.c"), "w") as f:
        f.write(HARNESS)
    exe = os.path.join(tmp, "h_" + fp)
    r = subprocess.run([CC, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O1", "-DIMU_FP_T=" + fp, "-I", tmp,
                        "-o", exe, os.path.join(tmp, "h.c"), os.path.join(tmp, "imu_window.c"),
                        os.path.join(tmp, "imu_activity.c"), "-lm"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return exe


def feed(exe, seqs, raw):
    text = "\n".join("%d " % s + " ".join(str(int(v)) for v in row) for s, row in zip(seqs, raw))
    out = subprocess.run([exe], input=text, capture_output=True, text=True, check=True).stdout.splitlines()
    rows = [[float(v) for v in line.split()] for line in out]
    return np.array([r[0] for r in rows], dtype=int), np.array([r[1:] for r in rows]).reshape(len(rows), -1)


def run(script, *args):
    r = subprocess.run([sys.executable, os.path.join(TOOLS, script), *args], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r


@unittest.skipIf(np is None or CC is None, "numpy or a C compiler not available")
class Parity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        t = cls.tmp
        dirs = []
        for i, act in enumerate(("still", "sit", "stand", "walk", "run")):
            for k in range(2):
                seed = 10 * (i + 1) + k
                run("imu_sim.py", act, "--duration", "20", "--seed", str(seed), "--out", t)
                d = os.path.join(t, f"sim-s{seed:04d}_{act}")
                run("imu_convert.py", d, "--npz")
                dirs.append(d)
        cls.dirs = dirs
        X, y, g, _ = F.build(dirs)
        classes = sorted(set(y))
        cls.classes = classes
        cls.tree = T.Tree(3, 10).fit(X, np.array([classes.index(v) for v in y]), len(classes))
        header = cls.tree.to_c(list(F.FEATURES), classes)
        for sub in ("d", "f"):
            os.makedirs(os.path.join(t, sub))
        cls.exe_d = build(os.path.join(t, "d"), "double", header)
        cls.exe_f = build(os.path.join(t, "f"), "float", header)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def session(self, i):
        s = F.load_session(self.dirs[i])
        raw = to_raw_bytes(s["accel"], s["gyro"])
        # the values the C code sees (raw / scale), which is also what the converter produced
        np.testing.assert_allclose(raw[:, 0:6].copy().view(">i2").reshape(-1, 3) / 16384.0, s["accel"], atol=1e-12)
        return s, raw

    def assert_zcr_close(self, got, X, s):
        """gy_zcr counts sign changes of (gy - mean).  The gyro is quantised, so in a nearly still window some
        samples can sit exactly ON the mean (one still session has a window with 19 of 200), and then the sign of
        0 vs -1e-17 decides the count: numpy's pairwise sum and a plain loop disagree, and so would float vs
        double.  That is a property of the feature, not of the C code, so a difference is accepted only in
        windows that have such a tie; everywhere else the count must be exactly equal."""
        gy = s["gyro"][:, 1]
        tie = np.array([bool((np.abs(gy[i:i + 200] - gy[i:i + 200].mean()) < 1e-4).any())
                        for i in F.window_starts(s["gap"], 200, 100)])
        bad = got[:, 16] != X[:, 16]
        self.assertFalse((bad & ~tie).any(), np.where(bad & ~tie)[0])

    def test_double_build_matches_numpy_features_to_rounding_noise(self):
        for i in (0, 3, 5, 8):                                    # still, sit, stand, walk, run: spread over classes
            s, raw = self.session(i)
            X, _, _ = F.session_windows(self.dirs[i])
            cls_, got = feed(self.exe_d, s["seq"], raw)
            self.assertEqual(got.shape, X.shape)
            np.testing.assert_allclose(got[:, :16], X[:, :16], rtol=1e-9, atol=1e-9)
            self.assert_zcr_close(got, X, s)

    def test_float_build_matches_to_float_precision(self):
        for i in (1, 4, 7, 9):
            s, raw = self.session(i)
            X, _, _ = F.session_windows(self.dirs[i])
            cls_, got = feed(self.exe_f, s["seq"], raw)
            self.assertEqual(got.shape, X.shape)
            scale = np.maximum(np.abs(X[:, :16]), 1.0)
            self.assertLess(float(np.max(np.abs(got[:, :16] - X[:, :16]) / scale[:, :16])), 2e-4)
            self.assert_zcr_close(got, X, s)

    def test_classes_agree_with_the_python_tree(self):
        agree = total = 0
        for i in range(len(self.dirs)):
            s, raw = self.session(i)
            X, _, _ = F.session_windows(self.dirs[i])
            c, _ = feed(self.exe_f, s["seq"], raw)
            want = self.tree.predict(X, dtype=np.float32)
            agree += int((c == want).sum())
            total += len(want)
        self.assertGreater(total, 100)
        self.assertGreaterEqual(agree / total, 0.99)

    def test_a_gap_restarts_the_window_and_no_window_spans_it(self):
        s, raw = self.session(6)
        n = 700
        seq = np.arange(n)
        seq[350:] += 5                                           # 5 events lost before sample 350
        c, f = feed(self.exe_d, seq, raw[:n])
        # windows end at samples 199 and 299; the one ending at 399 would contain the gap (so it is dropped);
        # the run restarts at sample 350, giving windows 350..549 and 450..649
        self.assertEqual(len(c), 4)
        w = [raw[i:i + 200] for i in (0, 100, 350, 450)]
        for k, rows in enumerate(w):
            acc = rows[:, 0:6].copy().view(">i2").reshape(-1, 3) / 16384.0
            gyr = rows[:, 8:14].copy().view(">i2").reshape(-1, 3) / 131.0
            np.testing.assert_allclose(f[k], F.window_features(acc, gyr), rtol=1e-9, atol=1e-9)

    def test_sequence_wraparound_is_not_a_gap(self):
        s, raw = self.session(2)
        n = 400
        seq = (np.arange(n) + 65400) % 65536                       # crosses 65535 -> 0
        c, _ = feed(self.exe_d, seq, raw[:n])
        self.assertEqual(len(c), 3)                                # windows at 200, 300, 400 samples


if __name__ == "__main__":
    unittest.main()
