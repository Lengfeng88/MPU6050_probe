"""tools/parse_imu.py with result frames in the stream, through a fake `serial` module."""
import contextlib
import io
import os
import re
import runpy
import sys
import tempfile
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import imu_proto as P  # noqa: E402

PARSER = os.path.join(ROOT, "tools", "parse_imu.py")
# set PARSE_IMU_OLD=/path/to/the/previous/parse_imu.py to also compare with it
OLD = os.environ.get("PARSE_IMU_OLD")


class FakeSerial:
    def __init__(self, port, baud, timeout=0):
        with open(port, "rb") as f:
            self.data = f.read()
        self.pos = 0

    def reset_input_buffer(self):
        pass

    def read(self, n):
        chunk, self.pos = self.data[self.pos:self.pos + n], self.pos + n
        return chunk

    def close(self):
        pass


def run_tool(script, data, seconds="0.3"):
    with tempfile.NamedTemporaryFile(delete=False) as f:
        f.write(data)
    mod = types.ModuleType("serial")
    mod.Serial = FakeSerial
    old_argv, old_serial = sys.argv, sys.modules.get("serial")
    sys.modules["serial"], sys.argv = mod, [script, f.name, seconds]
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            runpy.run_path(script, run_name="__main__")
    finally:
        sys.argv = old_argv
        if old_serial is None:
            del sys.modules["serial"]
        else:
            sys.modules["serial"] = old_serial
        os.unlink(f.name)
    return buf.getvalue()


def make(n, results=True, skip=()):
    out, ts = bytearray(), 1000
    for i in range(n):
        ts += 10000
        if i in skip:
            continue
        out += P.encode(i, ts, bytes([i & 0xFF]) * 14)
        if results and i >= 199 and (i - 199) % 100 == 0:
            k = (i - 199) // 100 + 1
            out += P.encode_result(P.ResultFrame(k, i, ts, ts + 3700, i % 5, 1, 0x5C4B, 0, 127519))
    return bytes(out)


class ParseImuResultsTest(unittest.TestCase):
    def test_sample_only_stream_gives_the_same_report_as_before(self):
        old = OLD or os.path.join(os.path.dirname(__file__), "parse_imu_before.py")
        if not os.path.exists(old):
            self.skipTest("set PARSE_IMU_OLD to the previous parse_imu.py to compare")
        data = make(500, results=False)
        self.assertEqual(run_tool(PARSER, data), run_tool(old, data))

    def test_mixed_stream_has_no_resync_and_reports_results(self):
        out = run_tool(PARSER, make(500))
        self.assertIn("packets ok=500  crc_bad=0  seq_gaps=0  lost=0  resync_bytes=0", out)
        self.assertIn("results ok=4  crc_bad=0  win_gaps=0  lost_windows=0", out)
        self.assertIn("min=3700 max=3700", out)
        self.assertIn("tree_id=0x5C4B", out)
        self.assertIn("WARNING: the tree was trained on SIMULATED data", out)

    def test_lost_samples_and_lost_result_are_counted_separately(self):
        data = bytearray(make(500, skip={10, 11}))
        # drop the second result frame (win_seq 2) from the stream
        r2 = P.encode_result(P.ResultFrame(2, 299, 1000 + 300 * 10000, 1000 + 300 * 10000 + 3700, 299 % 5, 1, 0x5C4B, 0, 127519))
        i = bytes(data).find(r2)
        self.assertGreater(i, 0)
        del data[i:i + P.RESULT_LEN]
        out = run_tool(PARSER, bytes(data))
        self.assertIn("seq_gaps=1  lost=2", out)
        self.assertIn("win_gaps=1  lost_windows=1", out)

    def test_corrupt_result_is_counted_not_misread(self):
        data = bytearray(make(300))
        i = bytes(data).find(b"\xAA\x5A")
        data[i + 12] ^= 0xFF
        out = run_tool(PARSER, bytes(data))
        self.assertRegex(out, r"results ok=1  crc_bad=1")
        self.assertIn("packets ok=300  crc_bad=0", out)

    def test_no_result_line_without_results(self):
        self.assertNotIn("results ok", run_tool(PARSER, make(100, results=False)))


if __name__ == "__main__":
    unittest.main()
