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


def loop_capture(drdy_us, stall_after=None, stall_us=0.0, window=()):
    """A main loop that handles events strictly in order, one at a time: I2C 1600 us, then 2.1 ms of UART tx / encode,
    then a classifier push (1.3 us, or 319 us for events in `window`).  Returns (capture, truth), truth = per event
    (drdy, i2c_start, i2c_end, push_start, push_end).  The generator is the oracle, not the tool."""
    smp = lambda t: int(round(t * RATE / 1e6))
    n = smp(drdy_us[-1] + 60000.0)
    d = np.zeros(n, dtype=np.uint8)
    d |= 4
    truth, free = [], 0.0
    for k, t in enumerate(drdy_us):
        d[smp(t):smp(t) + smp(50)] |= 2
        d[smp(t + 0.4):smp(t + 0.4) + smp(0.3)] |= 8
        a = max(t + 1.0, free)
        e = a + 1600.0
        ps = e + 2100.0
        pe = ps + (319.0 if k in window else 1.3)
        d[smp(a):smp(e)] |= 16
        d[smp(ps):smp(pe)] |= 1
        truth.append((t, a, e, ps, pe))
        free = pe + (stall_us if k == stall_after else 0.0)
    return d, truth


def add_scl(d, truth, period_us, clocks, skip=()):
    """Put `clocks` SCL pulses (50 % duty) into bit 5, starting 5 us into every I2C read of `truth` (from loop_capture)."""
    smp = lambda t: int(round(t * RATE / 1e6))
    for k, (_, a, e, _, _) in enumerate(truth):
        n = clocks if k not in skip else 20                       # a truncated read, as after a NACK
        for c in range(n):
            t = a + 5.0 + c * period_us
            d[smp(t):smp(t + period_us / 2)] |= 32


@unittest.skipIf(np is None, "numpy not installed")
class Scl(unittest.TestCase):
    def test_clock_count_and_frequency_of_each_read(self):
        d, truth = loop_capture([1000.0 + 9979.0 * k for k in range(30)])
        add_scl(d, truth, 2.5, 153)
        ev = M.collect(d, RATE)
        self.assertEqual(set(ev["scl_clocks"].tolist()), {153})
        self.assertAlmostEqual(float(np.median(ev["scl_khz"])), 400.0, delta=1.0)
        self.assertEqual(len(M.summarise(ev)["scl_clocks"].keys() & {"min", "max"}), 2)

    def test_a_truncated_read_shows_as_fewer_clocks(self):
        d, truth = loop_capture([1000.0 + 9979.0 * k for k in range(30)])
        add_scl(d, truth, 10.0, 153, skip=(7,))
        ev = M.collect(d, RATE)
        self.assertEqual(int(ev["scl_clocks"].min()), 20)
        self.assertEqual(int((ev["scl_clocks"] == 153).sum()), 29)       # 30 reads (every I2C pulse is counted), one truncated
        self.assertEqual(len(ev["scl_clocks"]), 30)
        self.assertAlmostEqual(float(np.median(ev["scl_khz"])), 100.0, delta=1.0)

    def test_without_an_scl_signal_the_rows_are_empty_not_wrong(self):
        d, _ = loop_capture([1000.0 + 9979.0 * k for k in range(30)])
        ev = M.collect(d, RATE)
        self.assertNotIn("scl_clocks", ev)
        self.assertIsNone(M.summarise(ev).get("scl_clocks"))


@unittest.skipIf(np is None, "numpy not installed")
class Overlap(unittest.TestCase):
    def test_idle_loop_has_no_overlap_and_in_order_pairing_agrees(self):
        d, _ = loop_capture([1000.0 + 9979.0 * k for k in range(60)])
        ev = M.collect(d, RATE)
        self.assertEqual(len(ev["isr_latency_in_i2c_us"]) + len(ev["isr_latency_in_uart_us"])
                         + len(ev["isr_latency_in_window_us"]), 0)
        self.assertAlmostEqual(float(np.median(ev["i2c_start_seq_us"])), 1.0, delta=0.1)
        self.assertEqual(int(ev["i2c_start_seq_unmatched"][0]), 0)

    def test_stall_delays_are_paired_in_order_and_overlaps_are_counted(self):
        dr = [1000.0 + 10000.0 * k for k in range(40)]
        d, truth = loop_capture(dr, stall_after=3, stall_us=14000.0)
        ev = M.collect(d, RATE)
        n = len(ev["i2c_start_seq_us"])
        want = np.array([a - t for t, a, _, _, _ in truth])[:n]
        self.assertGreater(want.max(), 5000.0)                     # events really waited several ms
        self.assertLess(float(np.abs(ev["i2c_start_seq_us"] - want).max()), 0.2)
        self.assertEqual(int(ev["i2c_start_seq_unmatched"][0]), 0)
        # DRDY edges that fall inside a busy interval of the loop (truth from the generator)
        last = len(dr) - 1                                         # the last DRDY has no closing window
        inside_i2c = sum(1 for k in range(last) for (_, a, e, _, _) in truth if a <= dr[k] < e)
        inside_uart = sum(1 for k in range(last) for (_, _, e, ps, _) in truth if e <= dr[k] < ps)
        self.assertGreater(inside_i2c + inside_uart, 0)
        self.assertEqual(len(ev["isr_latency_in_i2c_us"]), inside_i2c)
        self.assertEqual(len(ev["isr_latency_in_uart_us"]), inside_uart)
        for key in ("isr_latency_in_i2c_us", "isr_latency_in_uart_us"):
            self.assertTrue(np.all(np.abs(ev[key] - 0.4) < 0.1))

    def test_capture_that_starts_while_the_loop_is_behind_syncs_on_an_idle_event(self):
        dr = [1000.0 + 10000.0 * k for k in range(40)]
        d, truth = loop_capture(dr, stall_after=3, stall_us=14000.0)
        d = d[int(42000.0 * RATE / 1e6):]                # starts after DRDY 4, before its read: event 4's DRDY is cut off
        ev = M.collect(d, RATE)
        self.assertGreater(len(ev["i2c_start_seq_us"]), 20)
        self.assertGreater(float(ev["i2c_start_seq_us"].min()), 0.0)      # never pairs event 5's DRDY with event 4's read
        self.assertLess(float(ev["i2c_start_seq_us"].max()), 5.0)         # synced on the first idle event, loop caught up

    def test_drdy_inside_window_classification(self):
        dr = [1000.0 + 10000.0 * k for k in range(10)]
        dr.insert(5, dr[4] + 1.0 + 1600.0 + 2100.0 + 100.0)     # 100 us after window 4's push began
        d, truth = loop_capture(dr, window=(4,))
        ev = M.collect(d, RATE)
        self.assertEqual(len(ev["isr_latency_in_window_us"]), 1)
        self.assertAlmostEqual(float(ev["isr_latency_in_window_us"][0]), 0.4, delta=0.1)

    def test_a_dropped_event_is_flagged_by_the_unmatched_count(self):
        dr = [1000.0 + 10000.0 * k for k in range(40)]
        d, truth = loop_capture(dr)
        # remove event 10's I2C and classifier pulses (the loop never read it): the in-order pairing slips by one
        smp = lambda t: int(round(t * RATE / 1e6))
        t, a, e, ps, pe = truth[10]
        d[smp(a):smp(pe) + 10] &= ~np.uint8(17)
        ev = M.collect(d, RATE)
        self.assertEqual(int(ev["i2c_start_seq_unmatched"][0]), 1)


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
        self.assertAlmostEqual(r["drdy_width_us"]["p50"], 50.0, delta=0.1)               # the synthetic INT pulse is 50 us
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
