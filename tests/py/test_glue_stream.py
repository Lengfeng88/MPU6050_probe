"""tests/host/glue_stream.py: taking the firmware stream apart and checking result frames."""
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "tests", "host"))
import imu_proto as P  # noqa: E402
import glue_stream as G  # noqa: E402


def stream(seqs, window_ends, tree_id=0x5C4B, flags=1):
    out, ts = bytearray(), {}
    for s in seqs:
        ts[s] = 1000 + s * 10000
        out += P.encode(s, ts[s], bytes([s & 0xFF]) * 14)
        if s in window_ends:
            k = sorted(window_ends).index(s) + 1
            out += P.encode_result(P.ResultFrame(k, s, ts[s], ts[s] + 3700, 1, flags, tree_id, 0, 127519))
    return bytes(out)


class GlueStreamTest(unittest.TestCase):
    def test_clean_stream(self):
        samples, results = G.decode_stream(stream(range(300), {199, 299}))
        self.assertEqual((len(samples), len(results)), (300, 2))
        self.assertEqual(G.result_problems(samples, results, tree_id=0x5C4B, simulated=True), [])
        self.assertEqual(G.sample_bytes(samples), stream(range(300), set()))

    def test_window_spanning_a_hole_is_flagged(self):
        seqs = [s for s in range(300) if s != 250]               # sample 250 lost, result at 299 claims a full window
        samples, results = G.decode_stream(stream(seqs, {199, 299}))
        bad = G.result_problems(samples, results)
        self.assertEqual(len(bad), 1)
        self.assertIn("window 2", bad[0])

    def test_wrong_timestamp_tree_id_flag_and_win_seq(self):
        samples, _ = G.decode_stream(stream(range(300), set()))
        r = [P.ResultFrame(2, 199, 5, 10, 0, 0, 0x1111, 0, 0)]    # win_seq should start at 1, ts wrong, tree/flag wrong
        bad = " | ".join(G.result_problems(samples, r, tree_id=0x5C4B, simulated=True))
        for needle in ("first win_seq", "ts_last_us", "tree_id", "simulated flag"):
            self.assertIn(needle, bad)

    def test_done_before_last_is_flagged(self):
        samples, _ = G.decode_stream(stream(range(200), set()))
        r = [P.ResultFrame(1, 199, samples[199].ts_us, samples[199].ts_us - 5, 0, 0, 0, 0, 0)]
        self.assertTrue(any("before" in p for p in G.result_problems(samples, r)))

    def test_stray_bytes_and_bad_frames_are_rejected(self):
        with self.assertRaises(AssertionError):
            G.decode_stream(b"\x00" + stream(range(5), set()))
        bad = bytearray(stream(range(5), set()))
        bad[30] ^= 0xFF
        with self.assertRaises(AssertionError):
            G.decode_stream(bytes(bad))

    def test_trace_extras(self):
        old = ["DELAY 1", "I2C_READ n=14", "TX len=24 to=50", "DELAY 1", "I2C_READ n=14", "TX len=24 to=50"]
        new = ["DELAY 1", "I2C_READ n=14", "TX len=24 to=50", "TX len=26 to=50", "DELAY 1", "I2C_READ n=14",
               "TX len=24 to=50"]
        self.assertEqual(G.trace_extras(old, new), (["TX len=26 to=50"], None))
        self.assertTrue(G.is_result_tx("TX len=26 to=50"))
        self.assertFalse(G.is_result_tx("TX len=24 to=50"))
        self.assertFalse(G.is_result_tx("TX len=260 to=50"))
        self.assertEqual(G.trace_extras(old, old), ([], None))
        extras, problem = G.trace_extras(old, new[:2] + new[3:])          # a line removed
        self.assertIsNotNone(problem)
        swapped = [old[0], old[2], old[1]] + old[3:]                      # same lines, other order
        self.assertIsNotNone(G.trace_extras(old, swapped)[1])
        changed = [l.replace("n=14", "n=13") if i == 1 else l for i, l in enumerate(old)]
        self.assertIsNotNone(G.trace_extras(old, changed)[1])

    def test_seq_wrap(self):
        seqs = [(65400 + i) & 0xFFFF for i in range(300)]
        samples, results = G.decode_stream(stream(seqs, {seqs[199], seqs[299]}))
        self.assertEqual(G.result_problems(samples, results, tree_id=0x5C4B), [])


if __name__ == "__main__":
    unittest.main()
