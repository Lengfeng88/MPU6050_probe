#!/usr/bin/env python3
import re, struct, sys, time
from collections import Counter
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
BAUD = int(sys.argv[3]) if len(sys.argv) > 3 else 1000000   # firmware: 1 Mbaud (115200 before the DMA change)

# Two frame types share the stream (docs/packet_protocol.md): 24-byte samples (AA 55) and
# 26-byte classifier results (AA 5A, about once a second).
SYNC_RE = re.compile(rb"\xAA[\x55\x5A]")
SAMPLE_LEN, RESULT_LEN = 24, 26
RESULT_FLAG_SIMULATED_TREE = 0x01

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.reset_input_buffer()

buf = bytearray()
n_ok = n_crc_bad = n_gap = lost = 0
resync = 0
last_seq = None
last_ts = None
dts = []
ax = ay = az = t = gx = gy = gz = 0
r_ok = r_crc_bad = r_gaps = r_lost = 0
r_last_win = None
r_classes = Counter()
r_lat = []
r_cycles_max = 0
r_dropped = 0
r_tree_ids = set()
r_flags = set()

t_end = time.time() + DURATION
while time.time() < t_end:
    buf += ser.read(256)
    while True:
        m = SYNC_RE.search(buf)
        if m is None:
            buf = buf[-1:]
            break
        i = m.start()
        if i > 0:
            resync += i
            del buf[:i]
        is_result = buf[1] == 0x5A
        n = RESULT_LEN if is_result else SAMPLE_LEN
        if len(buf) < n:
            break
        pkt = bytes(buf[:n])
        crc_rx = pkt[n - 2] | (pkt[n - 1] << 8)
        if crc16_ccitt(pkt[2:n - 2]) != crc_rx:
            if is_result:
                r_crc_bad += 1
            else:
                n_crc_bad += 1
            del buf[:2]
            continue
        del buf[:n]
        if is_result:
            win, seq_last, ts_last, ts_done, cls, flags, tree_id, dropped, cycles = \
                struct.unpack_from("<HHIIBBHHI", pkt, 2)
            r_ok += 1
            if r_last_win is not None:
                g = (win - r_last_win) & 0xFFFF
                if g != 1:
                    r_gaps += 1
                    r_lost += g - 1
            r_last_win = win
            r_classes[cls] += 1
            r_lat.append((ts_done - ts_last) & 0xFFFFFFFF)
            r_cycles_max = max(r_cycles_max, cycles)
            r_dropped = dropped
            r_tree_ids.add(tree_id)
            r_flags.add(flags)
            continue
        seq, ts = struct.unpack_from("<HI", pkt, 2)
        ax, ay, az, t, gx, gy, gz = struct.unpack_from(">7h", pkt, 8)
        n_ok += 1
        if last_seq is not None:
            gap = (seq - last_seq) & 0xFFFF
            if gap != 1:
                n_gap += 1
                lost += gap - 1
        if last_ts is not None:
            dts.append((ts - last_ts) & 0xFFFFFFFF)
        last_seq, last_ts = seq, ts

ser.close()
print(f"packets ok={n_ok}  crc_bad={n_crc_bad}  seq_gaps={n_gap}  lost={lost}  resync_bytes={resync}")
if dts:
    print(f"dt(us): min={min(dts)} max={max(dts)} mean={sum(dts)/len(dts):.1f}")
    print(f"last sample: a=({ax},{ay},{az}) g=({gx},{gy},{gz}) temp={t/340+36.53:.1f}C")
if r_ok or r_crc_bad:
    print(f"results ok={r_ok}  crc_bad={r_crc_bad}  win_gaps={r_gaps}  lost_windows={r_lost}")
    print("  classes: " + ", ".join(f"{c}={n}" for c, n in sorted(r_classes.items())) + "   (index, see App/classify/imu_tree.h)")
    if r_lat:
        print(f"  MCU latency, DATA_RDY edge to class ready (us): min={min(r_lat)} max={max(r_lat)} mean={sum(r_lat)/len(r_lat):.1f}")
    print(f"  window cycles max={r_cycles_max}  uart_dropped(last)={r_dropped}  "
          f"tree_id={','.join('0x%04X' % x for x in sorted(r_tree_ids))}")
    if any(f & RESULT_FLAG_SIMULATED_TREE for f in r_flags):
        print("  WARNING: the tree was trained on SIMULATED data; the classes say nothing about real activities")
