"""Tests for tools/i2c_timing.py: the decoder against the value that is known to give about 100 kHz, and the search limits."""
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import i2c_timing as T  # noqa: E402

CLK = 100e6


class I2cTiming(unittest.TestCase):
    def test_decode_the_current_100khz_value(self):
        d = T.decode(0x10C0ECFF, CLK)
        self.assertEqual((d["presc"], d["scldel"], d["sdadel"], d["sclh"], d["scll"]), (1, 12, 0, 0xEC, 0xFF))
        self.assertAlmostEqual(d["t_scll"], 5.12e-6, delta=1e-9)
        self.assertAlmostEqual(d["t_sclh"], 4.74e-6, delta=1e-9)
        self.assertTrue(90e3 < d["f_min_hz"] < d["f_max_hz"] < 100e3)       # about 100 kHz, below it because of the sync delays

    def test_encode_and_fields_round_trip(self):
        for v in (0x10C0ECFF, 0x30961326, 0x00000000, 0xF0FF_FFFF & 0xF0FFFFFF):
            self.assertEqual(T.encode(**T.fields(v)), v & 0xF0FFFFFF)

    def test_search_never_exceeds_the_target_even_on_the_fastest_bus(self):
        d = T.search(CLK, 400e3)
        self.assertIsNotNone(d)
        self.assertLessEqual(T.decode(d["timingr"], CLK, 0.0, 0.0)["f_max_hz"], 400e3)
        self.assertGreater(d["f_bound_hz"], 380e3)                          # and it is not needlessly slow
        self.assertGreaterEqual(d["t_scll"], 1300e-9)                       # fast-mode low and high minimums
        self.assertGreaterEqual(d["t_sclh"], 600e-9)

    def test_search_respects_the_delay_formulas(self):
        d = T.search(CLK, 400e3)
        tp = (d["presc"] + 1) / CLK
        self.assertGreaterEqual((d["scldel"] + 1) * tp, T.FM["tr"] + T.FM["t_su"] - 1e-12)
        self.assertGreaterEqual(d["sdadel"] * tp, T.FM["tf"] - T.TAF[0] - 3 / CLK - 1e-12)
        self.assertLessEqual(d["sdadel"] * tp, T.FM["t_hd_max"] - T.FM["tr"] - T.TAF[1] - 4 / CLK + 1e-12)

    def test_an_impossible_target_returns_none(self):
        self.assertIsNone(T.search(CLK, 1e3))

    def test_the_value_in_the_firmware_is_the_one_the_search_returns(self):
        d = T.search(CLK, 400e3)
        src = open(os.path.join(ROOT, "Core", "Src", "i2c.c"), encoding="utf-8", errors="replace").read()
        self.assertIn("hi2c1.Init.Timing = 0x%08X;" % d["timingr"], src)
        ioc = open(os.path.join(ROOT, "MPU6050_probe.ioc"), encoding="utf-8", errors="replace").read()
        self.assertIn("I2C1.Timing=0x%08X" % d["timingr"], ioc)


if __name__ == "__main__":
    unittest.main()
