"""Tests for the host-side receiver tools.  Run:  python3 -m unittest discover -s tests/py -v

Standard library only (the recorder is driven through tests/host/fake_serial instead of pyserial).
"""
import csv
import json
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
TOOLS = os.path.join(ROOT, "tools")
sys.path.insert(0, TOOLS)
import imu_proto as P  # noqa: E402

# Produced by the firmware's own C encoder (App/packet/imu_packet.c): (seq, ts_us, raw hex, packet hex)
GOLDEN = [
(0, 0, "031425364758697a8b9cadbecfe0", "aa55000000000000031425364758697a8b9cadbecfe07f99"),
(4660, 2309737967, "2233445566778899aabbccddeeff", "aa553412efcdab892233445566778899aabbccddeeff682d"),
(65535, 4294967295, "415263748596a7b8c9daebfc0d1e", "aa55ffffffffffff415263748596a7b8c9daebfc0d1e245b")
]


def rd(path, mode="rb"):
    with open(path, mode) as f:
        return f.read()


def wr(path, data):
    with open(path, "wb") as f:
        f.write(data)


def raw_for(i):
    return bytes((i * 7 + k * 13) & 0xFF for k in range(14))


def stream(seqs, t0=1000, step=10000, ts_wrap=True):
    """Packets for the given (wrapping) seq numbers, 10 ms apart."""
    out = b""
    for n, s in enumerate(seqs):
        ts = t0 + n * step
        out += P.encode(s, ts & 0xFFFFFFFF if ts_wrap else ts, raw_for(n))
    return out


class Proto(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(P.crc16_ccitt(b"123456789"), 0x29B1)

    def test_python_encoder_matches_c_encoder(self):
        for seq, ts, raw_hex, pkt_hex in GOLDEN:
            self.assertEqual(P.encode(seq, ts, bytes.fromhex(raw_hex)).hex(), pkt_hex)

    def test_decoder_reads_c_packets(self):
        d = P.FrameDecoder()
        frames = list(d.feed(b"".join(bytes.fromhex(g[3]) for g in GOLDEN)))
        self.assertEqual([(f.seq, f.ts_us, f.raw.hex()) for f in frames], [(g[0], g[1], g[2]) for g in GOLDEN])
        self.assertEqual((d.crc_bad, d.resync_bytes), (0, 0))

    def test_any_chunking_gives_same_frames(self):
        data = stream(range(5))
        want = list(P.FrameDecoder().feed(data))
        for cut in range(len(data) + 1):
            d = P.FrameDecoder()
            got = list(d.feed(data[:cut])) + list(d.feed(data[cut:]))
            self.assertEqual(got, want, cut)
        d = P.FrameDecoder()                                  # one byte at a time
        self.assertEqual([f for b in data for f in d.feed(bytes([b]))], want)

    def test_garbage_before_stream_is_skipped_and_counted(self):
        d = P.FrameDecoder()
        garbage = b"\x01\x02\xAA\x03\x55\xAA"            # includes a lone AA and AA at the very end
        frames = list(d.feed(garbage + stream(range(3))))
        self.assertEqual(len(frames), 3)
        self.assertEqual(d.resync_bytes, len(garbage))

    def test_corrupt_packet_dropped_stream_recovers(self):
        pk = [P.encode(i, 1000 + i, raw_for(i)) for i in range(6)]
        bad = bytearray(pk[2]); bad[10] ^= 0x40
        d = P.FrameDecoder()
        frames = list(d.feed(pk[0] + pk[1] + bytes(bad) + pk[3] + pk[4] + pk[5]))
        self.assertEqual([f.seq for f in frames], [0, 1, 3, 4, 5])
        self.assertEqual(d.crc_bad, 1)

    def test_false_sync_does_not_swallow_the_next_valid_frame(self):
        d = P.FrameDecoder()
        data = b"\xAA\x55" + b"\x01" * 10 + stream(range(3))     # a fake sync whose 24-byte window overlaps frame 0
        frames = list(d.feed(data))
        self.assertEqual([f.seq for f in frames], [0, 1, 2])
        self.assertEqual(d.crc_bad, 1)

    def test_sync_bytes_inside_payload_do_not_confuse(self):
        raw = b"\xAA\x55" * 7
        d = P.FrameDecoder()
        frames = list(d.feed(P.encode(1, 2, raw) + P.encode(2, 3, raw)))
        self.assertEqual([f.seq for f in frames], [1, 2])

    def test_physical_units(self):
        raw = (16384).to_bytes(2, "big", signed=True) + (0).to_bytes(2, "big") + (-16384).to_bytes(2, "big", signed=True) \
            + (3400).to_bytes(2, "big", signed=True) + (131).to_bytes(2, "big", signed=True) \
            + (-262).to_bytes(2, "big", signed=True) + (0).to_bytes(2, "big")
        ax, ay, az, gx, gy, gz, temp = P.to_physical(raw)
        self.assertAlmostEqual(ax, 1.0); self.assertAlmostEqual(az, -1.0)
        self.assertAlmostEqual(temp, 3400 / 340 + 36.53)
        self.assertAlmostEqual(gx, 1.0); self.assertAlmostEqual(gy, -2.0)

    def test_unwrapper(self):
        u = P.Unwrapper(16)
        self.assertEqual([u(x) for x in (65534, 65535, 0, 1, 3)], [0, 1, 2, 3, 5])
        t = P.Unwrapper(32)
        self.assertEqual([t(x) for x in (0xFFFFFFF0, 0x10)], [0, 0x20])
        u = P.Unwrapper(16)
        self.assertEqual([u(x) for x in (10, 12, 11)], [0, 2, 1])    # a small step back is not a wrap


def run_tool(script, *args, **kw):
    env = dict(os.environ, PYTHONPATH=os.path.join(ROOT, "tests", "host", "fake_serial"))
    return subprocess.run([sys.executable, os.path.join(TOOLS, script), *args], env=env,
                          capture_output=True, text=True, **kw)


class RecordConvert(unittest.TestCase):
    def record(self, data, tmp, **extra):
        port = os.path.join(tmp, "uart.bin")
        wr(port, data)
        args = [port, "--label", "walking", "--subject", "kelvin", "--note", "wrist", "--duration", "0.4", "--out", tmp]
        r = run_tool("imu_record.py", *args)
        dirs = [d for d in os.listdir(tmp) if d.endswith("_walking")]
        return r, (os.path.join(tmp, dirs[0]) if dirs else None)

    def test_record_with_gaps_and_wrap(self):
        # seq wraps 65534 -> 0; events 65536+1 .. +3 missing (lost 3); ts wraps too
        seqs = [65533, 65534, 65535, 0, 1, 5, 6]
        data = b"\x00\x11" + stream(seqs, t0=0xFFFFFFFF - 25000, step=10000)    # 2 garbage bytes first
        with tempfile.TemporaryDirectory() as tmp:
            r, d = self.record(data, tmp)
            self.assertEqual(r.returncode, 0, r.stderr)
            meta = json.loads(rd(os.path.join(d, "meta.json")))
            self.assertEqual(meta["status"], "complete")
            self.assertEqual((meta["label"], meta["subject"], meta["note"]), ("walking", "kelvin", "wrist"))
            self.assertEqual((meta["frames"], meta["crc_bad"], meta["seq_gaps"], meta["lost_events"]), (7, 0, 1, 3))
            self.assertEqual(meta["resync_bytes"], 2)
            self.assertEqual((meta["dt_us_min"], meta["dt_us_max"]), (10000, 10000))
            self.assertAlmostEqual(meta["mcu_elapsed_s"], 0.06)
            self.assertEqual(os.path.getsize(os.path.join(d, "frames.bin")), 7 * 24)
            # frames.bin is exactly the valid packets
            self.assertEqual(rd(os.path.join(d, "frames.bin")), stream(seqs, t0=0xFFFFFFFF - 25000))

            r = run_tool("imu_convert.py", d)
            self.assertEqual(r.returncode, 0, r.stderr)
            rows = list(csv.DictReader(rd(os.path.join(d, "samples.csv"), "r").splitlines()))
            self.assertEqual([int(x["seq"]) for x in rows], [0, 1, 2, 3, 4, 8, 9])   # unwrapped, hole of 3 kept
            self.assertEqual([int(x["gap"]) for x in rows], [0, 0, 0, 0, 0, 1, 0])
            self.assertEqual([x["t_s"] for x in rows], ["0.000000", "0.010000", "0.020000", "0.030000", "0.040000", "0.050000", "0.060000"])

    def test_empty_stream_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            r, d = self.record(b"\x00\x01\x02", tmp)
            self.assertEqual(r.returncode, 3)
            self.assertEqual(json.loads(rd(os.path.join(d, "meta.json")))["frames"], 0)

    def test_bad_port_is_recorded_as_failed(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = run_tool("imu_record.py", os.path.join(tmp, "missing"), "--label", "x", "--out", tmp, "--duration", "0.1")
            self.assertEqual(r.returncode, 2)
            d = [x for x in os.listdir(tmp) if x.endswith("_x")][0]
            self.assertEqual(json.loads(rd(os.path.join(tmp, d, "meta.json")))["status"], "failed")

    def test_label_is_validated(self):
        r = run_tool("imu_record.py", "p", "--label", "../evil")
        self.assertEqual(r.returncode, 2)

    def test_convert_raw_and_partial_trailing_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            r, d = self.record(stream(range(3)), tmp)
            with open(os.path.join(d, "frames.bin"), "ab") as f:
                f.write(b"\xAA\x55\x00")                            # a crash left a partial frame
            r = run_tool("imu_convert.py", d, "--raw")
            self.assertEqual(r.returncode, 0)
            self.assertIn("partial frame", r.stderr)
            rows = list(csv.DictReader(rd(os.path.join(d, "samples.csv"), "r").splitlines()))
            self.assertEqual(len(rows), 3)
            self.assertEqual(int(rows[0]["ax"]), int.from_bytes(raw_for(0)[0:2], "big", signed=True))


if __name__ == "__main__":
    unittest.main()
