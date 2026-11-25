"""Decoder for the MCU's 24-byte IMU packets (see docs/packet_protocol.md).

Pure Python, standard library only.  Shared by imu_record.py and imu_convert.py.
"""
import re
import struct
from typing import Iterator, NamedTuple

SYNC = b"\xAA\x55"
FRAME_LEN = 24
RAW_LEN = 14

# Classifier result frame (docs/packet_protocol.md, "Result frame").
RESULT_SYNC = b"\xAA\x5A"
RESULT_LEN = 26
RESULT_FLAG_SIMULATED_TREE = 0x01
_RESULT_FMT = "<HHIIBBHHI"          # the 22 bytes between sync and crc16
_ANY_SYNC = re.compile(b"\xAA[\x55\x5A]")

# The firmware leaves the MPU-6050 at its reset ranges (+-2 g, +-250 deg/s).
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, MSB first, no final XOR."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Frame(NamedTuple):
    seq: int          # uint16, DATA_RDY event number (wraps at 65536)
    ts_us: int        # uint32, MCU timer in microseconds (wraps every ~71.6 min)
    raw: bytes        # 14 bytes, MPU-6050 registers 0x3B..0x48 verbatim


class ResultFrame(NamedTuple):
    win_seq: int         # uint16, +1 per completed window
    seq_last: int        # uint16, seq of the last sample in the window
    ts_last_us: int      # uint32, ts_us of that sample
    ts_done_us: int      # uint32, MCU timer when the class was ready
    cls: int             # class index (see App/classify/imu_tree.h)
    flags: int           # bit 0: tree trained on simulated data
    tree_id: int         # uint16
    uart_dropped: int    # uint16, cumulative, saturating
    cycles: int          # uint32, DWT cycles of this window

    @property
    def mcu_latency_us(self) -> int:
        """Sample DATA_RDY edge to class ready, on the MCU clock (mod 2**32)."""
        return (self.ts_done_us - self.ts_last_us) & 0xFFFFFFFF


def encode_result(r: ResultFrame) -> bytes:
    """Build a result frame exactly like the firmware does (used by the tests)."""
    body = struct.pack(_RESULT_FMT, *r)
    return RESULT_SYNC + body + struct.pack("<H", crc16_ccitt(body))


def encode(seq: int, ts_us: int, raw: bytes) -> bytes:
    """Build a packet exactly like the firmware does (used by the tests)."""
    assert len(raw) == RAW_LEN
    body = struct.pack("<HI", seq & 0xFFFF, ts_us & 0xFFFFFFFF) + raw
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


def frame_bytes(f: Frame) -> bytes:
    return encode(f.seq, f.ts_us, f.raw)


def decode_raw(raw: bytes):
    """-> (ax, ay, az, temp_raw, gx, gy, gz) as signed ints (registers are big-endian)."""
    return struct.unpack(">7h", raw)


def to_physical(raw: bytes):
    """-> (ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c)"""
    ax, ay, az, t, gx, gy, gz = decode_raw(raw)
    return (ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G,
            gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS,
            t / 340.0 + 36.53)


class FrameDecoder:
    """Incremental decoder: feed() arbitrary chunks of the serial stream, get valid frames.

    feed() yields only sample Frames, as before.  Classifier result frames are appended to
    ``results`` (the caller may drain it with ``results.clear()``) and counted separately, so
    code that only cares about samples does not change.

    Resynchronisation: look for AA 55 (sample) or AA 5A (result), whichever comes first; if
    the CRC of the candidate fails, drop its two sync bytes and look again.
    """

    def __init__(self):
        self._buf = bytearray()
        self.results = []
        self.frames = 0
        self.crc_bad = 0
        self.resync_bytes = 0
        self.result_frames = 0
        self.result_crc_bad = 0

    def feed(self, data: bytes) -> Iterator[Frame]:
        buf = self._buf
        buf += data
        while True:
            m = _ANY_SYNC.search(buf)
            if m is None:
                keep = 1 if buf[-1:] == SYNC[:1] else 0     # a lone AA may be the start of a sync
                self.resync_bytes += len(buf) - keep
                del buf[:len(buf) - keep]
                return
            i = m.start()
            if i > 0:
                self.resync_bytes += i
                del buf[:i]
            is_result = buf[1] == RESULT_SYNC[1]
            n = RESULT_LEN if is_result else FRAME_LEN
            if len(buf) < n:
                return
            pkt = bytes(buf[:n])
            if crc16_ccitt(pkt[2:n - 2]) != pkt[n - 2] | (pkt[n - 1] << 8):
                if is_result:
                    self.result_crc_bad += 1
                else:
                    self.crc_bad += 1
                del buf[:2]
                continue
            del buf[:n]
            if is_result:
                self.result_frames += 1
                self.results.append(ResultFrame(*struct.unpack_from(_RESULT_FMT, pkt, 2)))
                continue
            self.frames += 1
            seq, ts = struct.unpack_from("<HI", pkt, 2)
            yield Frame(seq, ts, pkt[8:22])


class Unwrapper:
    """Turns a wrapping counter into a monotonically growing one.

    Assumes consecutive values are less than half the range apart (seq: 32768 events =
    5.5 min; ts_us: 2**31 us = 35.8 min), which holds for a stream that is being received.
    """

    def __init__(self, bits: int):
        self._mod = 1 << bits
        self._half = self._mod >> 1
        self._last = None
        self._base = 0

    def __call__(self, x: int) -> int:
        if self._last is not None:
            d = (x - self._last) % self._mod
            if d >= self._half:          # went backwards: treat as a small negative step
                d -= self._mod
            self._base += d
        else:
            self._base = 0               # first value defines zero
        self._last = x
        return self._base
