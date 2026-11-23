import os
import random
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "tools"))
import imu_compare  # noqa: E402
import imu_proto  # noqa: E402


def make(samples):
    return [struct.pack(">7h", *s) for s in samples]


def write_session(d, samples, tail=b""):
    blob = b"".join(imu_proto.encode(i, i * 10000, raw) for i, raw in enumerate(make(samples)))
    with open(os.path.join(d, "frames.bin"), "wb") as f:
        f.write(blob + tail)


def noisy(n, seed, bit_error_at=()):
    r = random.Random(seed)
    out = []
    for i in range(n):
        s = [int(r.gauss(m, 5)) for m in (100, -50, 16384, -9000, 10, -20, 3)]
        if i in bit_error_at:
            s[1] ^= 0x4000       # one flipped bit in ay
        out.append(tuple(s))
    return out


class Compare(unittest.TestCase):
    def test_load_ignores_partial_tail_frame(self):
        with tempfile.TemporaryDirectory() as d:
            write_session(d, noisy(50, 1), tail=b"\xaa\x55\x01\x02")
            self.assertEqual(len(imu_compare.load_samples(d)), 50)

    def test_same_distribution_has_small_shift_and_no_outliers(self):
        rows, _, _ = imu_compare.compare(noisy(3000, 1), noisy(3000, 2))
        for name, x, y, sh in rows:
            self.assertLess(abs(sh), 0.2, name)
            self.assertEqual(y["outliers"], 0, name)

    def test_single_flipped_bit_is_flagged_only_in_that_channel(self):
        rows, _, _ = imu_compare.compare(noisy(3000, 1), noisy(3000, 2, bit_error_at={1234}))
        by = {r[0]: r for r in rows}
        self.assertEqual(by["ay"][2]["outliers"], 1)
        self.assertEqual(by["ay"][1]["outliers"], 0)
        for ch in ("ax", "az", "gx", "gy", "gz", "temp"):
            self.assertEqual(by[ch][2]["outliers"], 0, ch)

    def test_mean_shift_is_signed_b_minus_a(self):
        a = noisy(3000, 1)
        b = [(s[0] + 10,) + s[1:] for s in noisy(3000, 2)]
        by = {r[0]: r for r in imu_compare.compare(a, b)[0]}
        self.assertGreater(by["ax"][3], 1.0)          # std 5, shift +10 -> about 2
        self.assertLess(abs(by["ay"][3]), 0.2)

    def test_repeated_identical_samples_counted(self):
        a = noisy(100, 1)
        a = a[:50] + [a[49]] * 3 + a[50:]             # 3 repeats of sample 49
        self.assertEqual(imu_compare.channel_stats(a)[1], 3)

    def test_constant_channel_does_not_divide_by_zero(self):
        a = [(1, 2, 3, 4, 5, 6, 7)] * 20
        rows, _, _ = imu_compare.compare(a, a)
        self.assertTrue(all(r[3] == 0.0 for r in rows))
        self.assertTrue(all(r[1]["outliers"] == 0 for r in rows))

    def test_one_count_jitter_on_a_flat_channel_is_not_an_outlier(self):
        a = [(0, 0, 0, 0, 0, 0, 0)] * 100
        a[10] = (1, 0, 0, 0, 0, 0, 0)                 # MAD is 0; +-1 LSB must not count
        a[20] = (50, 0, 0, 0, 0, 0, 0)                # this one must
        self.assertEqual(imu_compare.channel_stats(a)[0][0]["outliers"], 1)

    def test_report_and_cli(self):
        with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
            write_session(d1, noisy(200, 1)); write_session(d2, noisy(200, 2))
            txt = imu_compare.report(d1, d2, imu_compare.load_samples(d1), imu_compare.load_samples(d2))
            self.assertIn("shift/std", txt)
            self.assertEqual(imu_compare.main(["x", d1, d2]), 0)


if __name__ == "__main__":
    unittest.main()
