# Recorded session format (version 1)

One recording = one directory, written by `tools/imu_record.py`:

```
data/20261029-180000_walking/
  frames.bin     the sensor data
  meta.json      who / what / when, and how well the reception went
  samples.csv    (optional) made by tools/imu_convert.py
  samples.npz    (optional) made by tools/imu_convert.py --npz
```

The directory name is `<UTC date>-<UTC time>_<label>`.

## frames.bin

The CRC-valid packets from the MCU, **byte for byte** (24 bytes each, layout in
`packet_protocol.md`), back to back, no header and no index. Nothing is converted, so the
original stream can always be re-decoded later (for example with a different scale, or after
a bug fix in the converter). Packets that failed the CRC and bytes skipped while looking for
the next `AA 55` are counted in `meta.json` but not stored.

Size: 2.4 kB per second, 8.6 MB per hour.

If the recorder is killed (power loss on the Pi), the file can end with a partial frame;
`imu_convert.py` ignores it with a warning.

## meta.json

| field | meaning |
|-------|---------|
| `format_version` | `1` |
| `status` | `recording` (still open or the recorder died), `complete` (`--duration` reached), `stopped` (Ctrl-C / SIGTERM), `interrupted` (read error, e.g. USB adapter unplugged; see `error`), `failed` (port could not be opened) |
| `label`, `subject`, `note` | what was recorded, who moved the sensor, free text (sensor placement, surface, ...) |
| `port`, `baud` | serial settings used |
| `nominal_rate_hz` | `100` (firmware setting; the real rate comes from `ts_us`) |
| `accel_lsb_per_g`, `gyro_lsb_per_dps` | `16384`, `131`: the firmware leaves the sensor at +-2 g and +-250 deg/s |
| `recording_started_utc`, `recording_ended_utc` | when the recorder ran (Pi clock) |
| `first_frame_utc`, `last_frame_utc` | Pi clock when the first / last valid frame arrived |
| `first_seq`, `first_ts_us` | counters of the first stored frame (`seq` and `ts_us` do not start at 0 after an MCU reset) |
| `frames` | stored packets |
| `crc_bad`, `resync_bytes` | damaged packets seen / bytes skipped to resynchronise |
| `seq_gaps`, `lost_events` | holes in `seq` / number of missing events (see below) |
| `dt_us_min`, `dt_us_max` | smallest and largest time between consecutive stored packets, from the MCU clock |
| `mcu_elapsed_s` | MCU time between first and last stored packet |

## Time

- `ts_us` is the MCU timer value captured in the DATA_RDY interrupt: it is the time of the
  *sample*, not of its arrival on the Pi. It wraps every 71.6 minutes; the tools unwrap it
  (assuming consecutive packets are less than 35.8 minutes apart).
- `t_s` in the CSV is seconds since the first stored packet, from the MCU clock only.
- There is no per-sample Pi timestamp. To get wall-clock time of a sample, use
  `first_frame_utc + t_s`. The first-frame time has an error of the serial latency (milliseconds)
  and the MCU crystal drifts relative to the Pi clock (compare `mcu_elapsed_s` with
  `last_frame_utc - first_frame_utc` to see how much). Fine for activity recognition, not for
  aligning with another sensor to the millisecond.
- If the MCU is reset during a recording, `ts_us` restarts and the unwrapper cannot tell; start a
  new session instead (a reset also shows up as a `seq` jump).

## Gaps and `seq`

`seq` numbers DATA_RDY events (+1 per 10 ms event), assigned in the MCU's interrupt handler.
A hole in `seq` means events happened but no packet arrived: the MCU queue was full, an I2C read
failed, or bytes were lost on the serial link. `seq_gaps` counts the holes, `lost_events` the
missing numbers. Not detected by `seq`: events merged by the hardware when interrupts were
blocked; those show as a long `dt` only.

Samples are stored as they arrived, **not** resampled. After a gap the time step is larger and
the CSV row after it has `gap = 1`. A window for a classifier should not span a gap (or should
interpolate explicitly); that is the job of the training code, not the recorder.

## samples.csv

`seq,t_s,gap,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c`, one row per stored packet.
`seq` is unwrapped and starts at 0. With `--raw`: `seq,t_s,gap,ax,ay,az,gx,gy,gz,temp_raw`
(signed 16-bit register values).

Temperature is `raw / 340 + 36.53` (MPU-6050 datasheet formula, only an approximation).

## samples.npz

`seq` (int64), `t_s` (float64), `gap` (uint8), `accel` (N x 3, g), `gyro` (N x 3, deg/s),
`temp` (deg C), and `label`, `subject`, `units` as scalar strings.

## Recording tips

- One session per activity and per run; keep them 1-5 minutes for easy handling and labelling.
- Put the sensor placement in `--note`; a model trained on one placement does not carry over
  to another.
- Check `meta.json` after the run: `crc_bad`, `seq_gaps` and `lost_events` should be 0 on a
  healthy link.
