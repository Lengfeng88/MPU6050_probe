"""Result frame: Python encoder / decoder and the firmware C encoder (docs/packet_protocol.md)."""
import os
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import imu_proto as P  # noqa: E402

# Same vector as in docs/packet_protocol.md.
VEC_FIELDS = P.ResultFrame(win_seq=1, seq_last=299, ts_last_us=3000000, ts_done_us=3003686,
                           cls=2, flags=1, tree_id=0x1234, uart_dropped=0, cycles=127519)
VEC_HEX = "aa 5a 01 00 2b 01 c0 c6 2d 00 26 d5 2d 00 02 01 34 12 00 00 1f f2 01 00 df 9e"


def mixed_stream(n, rng):
    out, samples, results = bytearray(), [], []
    ts = 1000
    for i in range(n):
        seq, ts = i & 0xFFFF, (ts + 9979) & 0xFFFFFFFF
        raw = bytes(rng.getrandbits(8) for _ in range(P.RAW_LEN))
        out += P.encode(seq, ts, raw)
        samples.append(P.Frame(seq, ts, raw))
        if i >= 199 and (i - 199) % 100 == 0:
            r = P.ResultFrame((i - 199) // 100, seq, ts, (ts + 3700) & 0xFFFFFFFF,
                              rng.randrange(4), 1, 0xBEEF, 0, 127519)
            out += P.encode_result(r)
            results.append(r)
    return bytes(out), samples, results


def feed_chunks(dec, data, rng):
    got, i = [], 0
    while i < len(data):
        k = rng.randrange(1, 40)
        got += list(dec.feed(data[i:i + k]))
        i += k
    return got


class ResultFrameTest(unittest.TestCase):
    def test_vector(self):
        self.assertEqual(P.encode_result(VEC_FIELDS).hex(" "), VEC_HEX)
        self.assertEqual(len(P.encode_result(VEC_FIELDS)), P.RESULT_LEN)

    def test_latency_wraps(self):
        r = VEC_FIELDS._replace(ts_last_us=0xFFFFFF00, ts_done_us=0x00000E66)
        self.assertEqual(r.mcu_latency_us, 0xE66 + 0x100)

    def test_mixed_stream_random_chunks(self):
        rng = random.Random(1)
        data, samples, results = mixed_stream(5000, rng)
        d = P.FrameDecoder()
        self.assertEqual(feed_chunks(d, data, rng), samples)
        self.assertEqual(d.results, results)
        self.assertEqual((d.crc_bad, d.result_crc_bad, d.resync_bytes), (0, 0, 0))
        self.assertEqual((d.frames, d.result_frames), (len(samples), len(results)))

    def test_sample_only_stream_unchanged(self):
        rng = random.Random(2)
        data = b"".join(P.encode(i, i * 10000, bytes(14)) for i in range(300))
        d = P.FrameDecoder()
        self.assertEqual(len(feed_chunks(d, data, rng)), 300)
        self.assertEqual(d.results, [])

    def test_sample_decoder_semantics_unchanged_for_midstream_start(self):
        data = P.encode(1, 1, bytes(14)) + P.encode(2, 2, bytes(14))
        d = P.FrameDecoder()
        got = list(d.feed(data[5:]))                # starts inside the first packet
        self.assertEqual([f.seq for f in got], [2])
        self.assertEqual(d.resync_bytes, len(data[5:]) - 24)

    def test_payload_containing_sync_bytes(self):
        s = P.encode(5, 9, b"\xAA\x5A" * 7)
        s2 = P.encode(6, 19, b"\xAA\x55" * 7)
        r = P.encode_result(VEC_FIELDS)
        d = P.FrameDecoder()
        got = list(d.feed(s + r + s2 + r))
        self.assertEqual([f.seq for f in got], [5, 6])
        self.assertEqual(len(d.results), 2)
        self.assertEqual((d.crc_bad, d.result_crc_bad, d.resync_bytes), (0, 0, 0))

    def test_corrupted_result_is_dropped_not_misread(self):
        r = bytearray(P.encode_result(VEC_FIELDS))
        r[10] ^= 0x01
        s = P.encode(7, 70, bytes(14))
        d = P.FrameDecoder()
        got = list(d.feed(bytes(r) + s))
        self.assertEqual([f.seq for f in got], [7])
        self.assertEqual(d.results, [])
        self.assertEqual(d.result_crc_bad, 1)

    def test_corruption_never_yields_wrong_frames(self):
        rng = random.Random(3)
        data, samples, results = mixed_stream(3000, rng)
        bad = bytearray(data)
        for _ in range(100):
            bad[rng.randrange(len(bad))] ^= 0xFF
        d = P.FrameDecoder()
        got = list(d.feed(bytes(bad)))
        self.assertTrue(set(got) <= set(samples))
        self.assertTrue(set(d.results) <= set(results))
        self.assertGreater(len(got), len(samples) * 0.9)

    def test_old_style_decoder_still_gets_every_sample(self):
        """A decoder that knows only AA 55 skips result frames (counts them as resync bytes)."""
        rng = random.Random(4)
        data, samples, results = mixed_stream(4000, rng)
        got, buf, i = [], bytearray(), 0
        buf += data
        while True:                                   # the pre-change algorithm
            j = buf.find(P.SYNC)
            if j < 0 or len(buf) - j < P.FRAME_LEN:
                break
            del buf[:j]
            pkt = bytes(buf[:24])
            if P.crc16_ccitt(pkt[2:22]) != pkt[22] | (pkt[23] << 8):
                del buf[:2]
                continue
            del buf[:24]
            got.append(pkt)
        self.assertEqual(len(got), len(samples))


@unittest.skipUnless(shutil.which("gcc"), "gcc not found")
class FirmwareEncoderTest(unittest.TestCase):
    """App/packet/imu_result.c must produce the same bytes as imu_proto.encode_result."""

    C_MAIN = r'''
#include <stdio.h>
#include <stdlib.h>
#include "packet/imu_result.h"
int main(void){
  unsigned long a[9]; imu_result_t r; uint8_t out[IMU_RES_LEN];
  while (scanf("%lu %lu %lu %lu %lu %lu %lu %lu %lu",&a[0],&a[1],&a[2],&a[3],&a[4],&a[5],&a[6],&a[7],&a[8])==9){
    r.win_seq=(uint16_t)a[0]; r.seq_last=(uint16_t)a[1]; r.ts_last_us=(uint32_t)a[2];
    r.ts_done_us=(uint32_t)a[3]; r.cls=(uint8_t)a[4]; r.flags=(uint8_t)a[5];
    r.tree_id=(uint16_t)a[6]; r.uart_dropped=(uint16_t)a[7]; r.cycles=(uint32_t)a[8];
    imu_result_encode(out,&r);
    for(unsigned i=0;i<IMU_RES_LEN;i++) printf("%02x",out[i]);
    printf("\n");
  }
  return 0;
}
'''

    def test_matches_python(self):
        rng = random.Random(5)
        cases = [VEC_FIELDS, P.ResultFrame(0, 0, 0, 0, 0, 0, 0, 0, 0),
                 P.ResultFrame(0xFFFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 255, 255, 0xFFFF, 0xFFFF, 0xFFFFFFFF)]
        for _ in range(500):
            cases.append(P.ResultFrame(rng.getrandbits(16), rng.getrandbits(16), rng.getrandbits(32),
                                       rng.getrandbits(32), rng.getrandbits(8), rng.getrandbits(8),
                                       rng.getrandbits(16), rng.getrandbits(16), rng.getrandbits(32)))
        with tempfile.TemporaryDirectory() as tmp:
            main_c = os.path.join(tmp, "main.c")
            with open(main_c, "w") as f:
                f.write(self.C_MAIN)
            exe = os.path.join(tmp, "enc")
            pk = os.path.join(ROOT, "App", "packet")
            subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", os.path.join(ROOT, "App"), "-o", exe,
                            main_c, os.path.join(pk, "imu_result.c"), os.path.join(pk, "crc16.c")],
                           check=True)
            inp = "\n".join(" ".join(str(v) for v in c) for c in cases) + "\n"
            out = subprocess.run([exe], input=inp, capture_output=True, text=True, check=True).stdout.split()
        self.assertEqual(len(out), len(cases))
        for c, hx in zip(cases, out):
            self.assertEqual(hx, P.encode_result(c).hex())


if __name__ == "__main__":
    unittest.main()
