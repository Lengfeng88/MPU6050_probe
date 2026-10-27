#!/usr/bin/env python3
import struct, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

ser = serial.Serial(PORT, 115200, timeout=0.1)
ser.reset_input_buffer()

buf = bytearray()
n_ok = n_crc_bad = n_gap = lost = 0
resync = 0
last_seq = None
last_ts = None
dts = []
ax = ay = az = t = gx = gy = gz = 0

t_end = time.time() + DURATION
while time.time() < t_end:
    buf += ser.read(256)
    while True:
        i = buf.find(b"\xAA\x55")
        if i < 0:
            buf = buf[-1:]
            break
        if i > 0:
            resync += i
            del buf[:i]
        if len(buf) < 24:
            break
        pkt = bytes(buf[:24])
        crc_rx = pkt[22] | (pkt[23] << 8)
        if crc16_ccitt(pkt[2:22]) != crc_rx:
            n_crc_bad += 1
            del buf[:2]
            continue
        del buf[:24]
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
