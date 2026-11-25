"""Helpers for run_glue.py: take the firmware's UART byte stream apart into sample frames and
classifier result frames (docs/packet_protocol.md), and check what a result frame promises."""
import pathlib
import sys
from collections import Counter

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import imu_proto as P  # noqa: E402


def decode_stream(b):
    """-> (sample Frames, result frames).  The stream must consist of whole, valid frames only."""
    d = P.FrameDecoder()
    samples = list(d.feed(b))
    assert (d.crc_bad, d.result_crc_bad, d.resync_bytes) == (0, 0, 0), \
        f"stream has bad frames: crc_bad={d.crc_bad} result_crc_bad={d.result_crc_bad} resync={d.resync_bytes}"
    assert len(samples) * P.FRAME_LEN + len(d.results) * P.RESULT_LEN == len(b), "stray bytes in the stream"
    return samples, d.results


def sample_bytes(samples):
    """Sample frames re-encoded: identical to what the firmware sent (encode() is exact)."""
    return b"".join(P.frame_bytes(f) for f in samples)


def is_result_tx(line):
    """A result frame's UART call in the HAL trace, e.g. 'TX len=26 to=50' (a sample frame is len=24)."""
    return line.startswith(f"TX len={P.RESULT_LEN} ")


def trace_extras(old, new):
    """Compare two HAL call traces where `new` may only ADD lines (the UART call of each result frame).

    Returns (extras, problem): `extras` are the lines in `new` that `old` does not account for;
    `problem` is None if `old` is a subsequence of `new` (nothing removed, nothing reordered or changed),
    else a short description."""
    missing = Counter(old) - Counter(new)
    if missing:
        return [], f"{sum(missing.values())} old line(s) not in the new trace, e.g. {next(iter(missing))!r}"
    it = iter(new)
    for line in old:
        if not any(line == n for n in it):
            return [], f"old line {line!r} is out of order in the new trace"
    return list((Counter(new) - Counter(old)).elements()), None


def result_problems(samples, results, window=200, tree_id=None, simulated=None):
    """What a result frame promises, checked against the sample frames of the same stream.
    Returns a list of problems (empty = fine)."""
    bad = []
    by_seq = {f.seq: f for f in samples}
    for k, r in enumerate(results):
        if k == 0:
            if r.win_seq != 1:
                bad.append(f"first win_seq is {r.win_seq}, expected 1")
        elif (r.win_seq - results[k - 1].win_seq) & 0xFFFF != 1:
            bad.append(f"win_seq {results[k - 1].win_seq} -> {r.win_seq}: not consecutive")
        missing = [(r.seq_last - j) & 0xFFFF for j in range(window) if ((r.seq_last - j) & 0xFFFF) not in by_seq]
        if missing:
            bad.append(f"window {r.win_seq}: {len(missing)} of its {window} samples are missing from the stream "
                       f"(a window must never span a seq hole), e.g. seq {missing[0]}")
        last = by_seq.get(r.seq_last)
        if last is not None and last.ts_us != r.ts_last_us:
            bad.append(f"window {r.win_seq}: ts_last_us {r.ts_last_us} != ts_us {last.ts_us} of seq {r.seq_last}")
        if r.mcu_latency_us >= 1 << 31:
            bad.append(f"window {r.win_seq}: ts_done_us is before ts_last_us")
        if tree_id is not None and r.tree_id != tree_id:
            bad.append(f"window {r.win_seq}: tree_id 0x{r.tree_id:04X}, expected 0x{tree_id:04X}")
        if simulated is not None and bool(r.flags & P.RESULT_FLAG_SIMULATED_TREE) != bool(simulated):
            bad.append(f"window {r.win_seq}: simulated flag {r.flags & 1}, expected {int(bool(simulated))}")
    return bad
