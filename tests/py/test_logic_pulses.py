"""Tests for tools/logic_pulses.py on synthetic captures (skipped without numpy)."""
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
    import logic_pulses as L
except ImportError:
    np = None

RATE = 24e6


def synth(seconds=2.2, long_us=319.0, short_us=1.3, drdy_period_us=9978.0, delay_us=3700.0, long_every=100):
    n = int(seconds * RATE)
    d = np.zeros(n, dtype=np.uint8)
    k = 0
    t = 2000.0
    while t + delay_us + 400 < seconds * 1e6:
        a = int(t * RATE / 1e6)
        d[a:a + int(50 * RATE / 1e6)] |= 2                                    # D1: 50 us DRDY pulse
        w = long_us if long_every and k % long_every == 5 else short_us   # long_every=0: never
        b = int((t + delay_us) * RATE / 1e6)
        d[b:b + int(round(w * RATE / 1e6))] |= 1                              # D0: classifier pulse
        t += drdy_period_us
        k += 1
    return d


@unittest.skipIf(np is None, "numpy not installed")
class Pulses(unittest.TestCase):
    def test_edges_and_pulses_of_a_known_signal(self):
        bits = np.array([0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0])
        s, w = L.pulses(bits)
        self.assertEqual((list(s), list(w)), ([1, 5, 7], [2, 1, 3]))

    def test_pulses_cut_by_the_ends_are_dropped(self):
        s, w = L.pulses(np.array([1, 1, 0, 1, 0, 1, 1]))
        self.assertEqual((list(s), list(w)), ([3], [1]))                      # leading high and trailing high dropped

    def test_widths_delay_and_period_are_recovered(self):
        d = synth()
        r = L.analyse(d, RATE)
        self.assertGreater(len(r["short_us"]), 150)
        self.assertAlmostEqual(float(np.median(r["short_us"])), 1.3, delta=0.1)
        self.assertEqual(len(r["long_us"]), 3)                                  # k = 5, 105, 205 in 2.2 s
        for w in r["long_us"]:
            self.assertAlmostEqual(float(w), 319.0, delta=0.1)
        self.assertAlmostEqual(float(np.mean(r["drdy_period_ms"])), 9.978, delta=0.001)
        self.assertAlmostEqual(float(np.mean(r["delay_ms"])), 3.7, delta=0.001)
        for dl in r["delay_long_ms"]:
            self.assertAlmostEqual(float(dl), 3.7, delta=0.001)

    def test_no_long_pulse_is_reported_as_none(self):
        r = L.analyse(synth(seconds=0.5, long_every=0), RATE)
        self.assertEqual(len(r["long_us"]), 0)

    def test_cli_prints_the_window_pulse_width(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "cap.bin")
            synth().tofile(p)
            r = subprocess.run([sys.executable, os.path.join(TOOLS, "logic_pulses.py"), p, "--rate", "24e6"],
                               capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("window pulses:     n=3", r.stdout)
            self.assertIn("319.0", r.stdout)

    def test_empty_capture_is_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "cap.bin")
            open(p, "wb").close()
            r = subprocess.run([sys.executable, os.path.join(TOOLS, "logic_pulses.py"), p], capture_output=True, text=True)
            self.assertEqual(r.returncode, 1)


if __name__ == "__main__":
    unittest.main()
