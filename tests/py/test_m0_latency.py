"""Tests for tools/m0_latency.py on synthetic captures with known delays (skipped without numpy)."""
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

try:
    import numpy as np
    import m0_latency as M
except ImportError:
    np = None

RATE = 24e6


def synth(n_events=300, period_us=9978.0, isr_us=0.9, isr_w=1.3, i2c_us=40.0, i2c_w=1600.0, uart_us=2100.0,
          cls_us=3700.0, cls_w=1.3, skip_isr=(), jitter=None, window_every=None, window_w=319.0):
    n = int((n_events + 2) * period_us * RATE / 1e6) + 100000
    d = np.zeros(n, dtype=np.uint8)
    smp = lambda t: int(round(t * RATE / 1e6))
    t0 = 1000.0
    for k in range(n_events):
        t = t0 + k * period_us + (0 if jitter is None else jitter[k])
        d[smp(t):smp(t) + smp(50)] |= 2                                         # D1 DRDY, 50 us pulse
        if k not in skip_isr:
            d[smp(t + isr_us):smp(t + isr_us) + smp(isr_w)] |= 8                # D3 ISR pin
        d[smp(t + i2c_us):smp(t + i2c_us + i2c_w)] |= 16                        # D4 I2C busy
        w = window_w if (window_every and k % window_every == window_every - 1) else cls_w
        d[smp(t + cls_us):smp(t + cls_us) + smp(w)] |= 1                        # D0 classifier push / window
    d |= 4                                                                      # D2 UART TX idles high
    for k in range(n_events):
        t = t0 + k * period_us + (0 if jitter is None else jitter[k]) + uart_us
        for b in range(24):                                                     # 24 bytes: start bit low, then data
            s = smp(t + b * 10.0)                                               # 1 Mbaud: 10 us per byte
            d[s:s + smp(1.0)] &= ~np.uint8(4)                                   # start bit
    return d


@unittest.skipIf(np is None, "numpy not installed")
class M0(unittest.TestCase):
    def test_recovers_known_delays(self):
        r = M.analyse(synth(), RATE)
        self.assertEqual(r["drdy_edges"], 299)                          # 300 DRDY edges; the last has no closing window
        self.assertAlmostEqual(r["drdy_to_isr_us"]["p50"], 0.9, delta=0.1)
        self.assertAlmostEqual(r["isr_width_us"]["p50"], 1.3, delta=0.1)
        self.assertAlmostEqual(r["drdy_to_i2c_start_us"]["p50"], 40.0, delta=0.1)
        self.assertAlmostEqual(r["i2c_width_us"]["p50"], 1600.0, delta=0.1)
        self.assertAlmostEqual(r["drdy_to_uart_start_us"]["p50"], 2100.0, delta=0.1)   # first start bit of the frame
        self.assertAlmostEqual(r["drdy_to_push_start_us"]["p50"], 3700.0, delta=0.1)
        self.assertAlmostEqual(r["drdy_period_us"]["p50"], 9978.0, delta=0.1)
        self.assertAlmostEqual(r["push_width_us"]["p50"], 1.3, delta=0.1)
        self.assertEqual(r["drdy_to_isr_us_missing"], 0)

    def test_counts_events_without_isr_pulse_and_sees_a_late_one(self):
        r = M.analyse(synth(skip_isr=(10, 11, 200)), RATE)
        self.assertEqual(r["drdy_to_isr_us_missing"], 3)
        # 300 DRDY edges; the last one has no closing window and is not analysed: 299 - 3 skipped
        self.assertEqual(r["drdy_to_isr_us"]["n"], 296)

    def test_max_is_the_worst_event_not_the_mean(self):
        d = synth(n_events=200)
        # delay one ISR pulse by 25 us: move it
        smp = lambda t: int(round(t * RATE / 1e6))
        t = 1000.0 + 50 * 9978.0
        d[smp(t + 0.9):smp(t + 0.9) + smp(1.3)] &= ~np.uint8(8)
        d[smp(t + 25.0):smp(t + 25.0) + smp(1.3)] |= 8
        r = M.analyse(d, RATE)
        self.assertAlmostEqual(r["drdy_to_isr_us"]["max"], 25.0, delta=0.1)
        self.assertAlmostEqual(r["drdy_to_isr_us"]["p50"], 0.9, delta=0.1)

    def test_period_jitter_shows_in_max_min(self):
        jit = np.zeros(300); jit[100] = 12.0                                    # one DRDY 12 us late
        r = M.analyse(synth(jitter=jit), RATE)
        self.assertAlmostEqual(r["drdy_period_us"]["max"], 9978.0 + 12.0, delta=0.1)
        self.assertAlmostEqual(r["drdy_period_us"]["min"], 9978.0 - 12.0, delta=0.1)

    def test_missing_channel_is_reported_not_crashed(self):
        d = synth() & np.uint8(0b11)                                            # only D0, D1 kept
        r = M.analyse(d, RATE)
        self.assertIsNone(r["drdy_to_isr_us"])
        self.assertGreater(r["drdy_to_isr_us_missing"], 0)

    def test_window_completions_are_split_from_ordinary_pushes(self):
        r = M.analyse(synth(n_events=300, window_every=100), RATE)      # events 99, 199, 299 are long
        # 299 events are analysed (the last DRDY has no closing window); long ones are k = 99 and 199 (k = 299 is the unanalysed last)
        self.assertEqual(r["push_width_us"]["n"], 297)
        self.assertEqual(r["drdy_to_window_start_us"]["n"], 2)
        self.assertAlmostEqual(r["window_width_us"]["p50"], 319.0, delta=0.2)
        self.assertAlmostEqual(r["push_width_us"]["max"], 1.3, delta=0.1)
        self.assertAlmostEqual(r["drdy_to_window_start_us"]["p50"], 3700.0, delta=0.1)

    def test_merging_event_files_adds_up_and_matches_one_long_capture(self):
        import tempfile
        a, b = synth(n_events=200), synth(n_events=150, isr_us=1.1)
        ea, eb = M.collect(a, RATE), M.collect(b, RATE)
        with tempfile.TemporaryDirectory() as d:
            pa, pb = os.path.join(d, "a.npz"), os.path.join(d, "b.npz")
            np.savez_compressed(pa, **ea); np.savez_compressed(pb, **eb)
            out = os.path.join(d, "out.json")
            self.assertEqual(M.main([pa, pb, "--json", out]), 0)
            import json
            r = json.load(open(out))
        self.assertEqual(r["drdy_edges"], 199 + 149)
        self.assertEqual(r["drdy_to_isr_us"]["n"], 199 + 149)
        self.assertAlmostEqual(r["drdy_to_isr_us"]["min"], 0.9, delta=0.1)
        self.assertAlmostEqual(r["drdy_to_isr_us"]["max"], 1.1, delta=0.1)

    def test_unreadable_file_is_a_message_not_a_traceback(self):
        self.assertEqual(M.main(["/nonexistent/cap.bin"]), 1)


if __name__ == "__main__":
    unittest.main()
