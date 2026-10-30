#!/usr/bin/env python3
"""Record one labelled IMU session from the MCU's serial stream.

    python3 tools/imu_record.py /dev/ttyUSB0 --label walking --subject kelvin --duration 120

Creates  <out>/<UTC date-time>_<label>/  containing
    frames.bin   every CRC-valid 24-byte packet, byte for byte as the MCU sent it
    meta.json    label, start time, sampling info, receive statistics

Stop with Ctrl-C (or --duration); the files are always closed cleanly.
Format: docs/data_format.md.   Convert to CSV / NumPy with tools/imu_convert.py.
"""
import argparse
import datetime as dt
import json
import os
import re
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_proto as P  # noqa: E402

FORMAT_VERSION = 1
NOMINAL_RATE_HZ = 100


def utc_iso(t: float) -> str:
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).isoformat(timespec="milliseconds")


def write_json_atomic(path: str, obj: dict) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


class Recorder:
    def __init__(self, session_dir: str, args):
        self.dir = session_dir
        self.meta_path = os.path.join(session_dir, "meta.json")
        self.frames = open(os.path.join(session_dir, "frames.bin"), "wb")
        self.dec = P.FrameDecoder()
        self.useq, self.uts = P.Unwrapper(16), P.Unwrapper(32)
        self.n = 0
        self.seq_gaps = self.lost = 0
        self.last_useq = None
        self.dt_min = self.dt_max = None
        self.last_uts = None
        self.first_wall = self.last_wall = None
        self.first = self.last = None
        self.meta = {
            "format_version": FORMAT_VERSION,
            "status": "recording",
            "label": args.label,
            "subject": args.subject,
            "note": args.note,
            "port": args.port,
            "baud": args.baud,
            "nominal_rate_hz": NOMINAL_RATE_HZ,
            "accel_lsb_per_g": P.ACCEL_LSB_PER_G,
            "gyro_lsb_per_dps": P.GYRO_LSB_PER_DPS,
            "recording_started_utc": utc_iso(time.time()),
        }
        write_json_atomic(self.meta_path, self.meta)

    def feed(self, data: bytes, now: float) -> None:
        for f in self.dec.feed(data):
            self.frames.write(P.frame_bytes(f))
            useq, uts = self.useq(f.seq), self.uts(f.ts_us)
            if self.last_useq is not None and useq - self.last_useq != 1:
                self.seq_gaps += 1
                self.lost += useq - self.last_useq - 1
            if self.last_uts is not None:
                d = uts - self.last_uts
                self.dt_min = d if self.dt_min is None else min(self.dt_min, d)
                self.dt_max = d if self.dt_max is None else max(self.dt_max, d)
            self.last_useq, self.last_uts = useq, uts
            if self.first is None:
                self.first, self.first_wall = f, now
            self.last, self.last_wall = f, now
            self.n += 1

    def stats(self) -> dict:
        s = {
            "frames": self.n,
            "crc_bad": self.dec.crc_bad,
            "resync_bytes": self.dec.resync_bytes,
            "seq_gaps": self.seq_gaps,
            "lost_events": self.lost,
            "dt_us_min": self.dt_min,
            "dt_us_max": self.dt_max,
        }
        if self.first is not None:
            s["first_frame_utc"] = utc_iso(self.first_wall)
            s["last_frame_utc"] = utc_iso(self.last_wall)
            s["mcu_elapsed_s"] = (self.last_uts - 0) / 1e6      # ts of last frame, first frame = 0
            s["first_ts_us"] = self.first.ts_us
            s["first_seq"] = self.first.seq
        return s

    def close(self, status: str, error: str = None) -> dict:
        self.frames.flush()
        os.fsync(self.frames.fileno())
        self.frames.close()
        self.meta.update(self.stats())
        self.meta["status"] = status
        if error:
            self.meta["error"] = error
        self.meta["recording_ended_utc"] = utc_iso(time.time())
        write_json_atomic(self.meta_path, self.meta)
        return self.meta


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--label", required=True, help="activity label, e.g. walking (letters, digits, - and _)")
    ap.add_argument("--subject", default="", help="who is wearing / moving the sensor")
    ap.add_argument("--note", default="", help="free text: sensor placement, surface, ...")
    ap.add_argument("--duration", type=float, default=None, help="stop after this many seconds (default: until Ctrl-C)")
    ap.add_argument("--out", default="data", help="parent directory for sessions (default: ./data)")
    args = ap.parse_args(argv)

    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.label):
        ap.error("--label may only contain letters, digits, '-' and '_'")

    import serial                                   # pyserial; imported late so --help works without it

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d-%H%M%S")
    session = os.path.join(args.out, f"{stamp}_{args.label}")
    os.makedirs(session, exist_ok=False)

    stop = {"flag": False}
    def on_signal(signum, _frame):
        stop["flag"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    rec = Recorder(session, args)
    status, error = "complete", None
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        ser.reset_input_buffer()
    except Exception as e:                          # port missing / busy
        rec.close("failed", f"cannot open {args.port}: {e}")
        print(f"cannot open {args.port}: {e}", file=sys.stderr)
        return 2

    t0 = time.monotonic()
    next_report = t0 + 5.0
    print(f"recording '{args.label}' into {session}  (Ctrl-C to stop)")
    try:
        while not stop["flag"]:
            if args.duration is not None and time.monotonic() - t0 >= args.duration:
                break
            data = ser.read(512)
            now = time.time()
            if data:
                rec.feed(data, now)
            if time.monotonic() >= next_report:
                next_report += 5.0
                print(f"  {rec.n} frames  crc_bad={rec.dec.crc_bad}  seq_gaps={rec.seq_gaps}  lost={rec.lost}")
        if stop["flag"]:
            status = "stopped"
    except Exception as e:                          # e.g. USB adapter unplugged
        status, error = "interrupted", f"{type(e).__name__}: {e}"
    finally:
        try:
            ser.close()
        except Exception:
            pass
        meta = rec.close(status, error)

    print(f"{meta['status']}: {meta['frames']} frames, crc_bad={meta['crc_bad']}, "
          f"seq_gaps={meta['seq_gaps']}, lost={meta['lost_events']}, resync_bytes={meta['resync_bytes']}")
    if error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    if meta["frames"] == 0:
        print("warning: no valid frames received (wrong port or baud rate? MCU not running?)", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
