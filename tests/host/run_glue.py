#!/usr/bin/env python3
"""A/B test of the main.c glue: pre-refactor vs current.

Both versions are compiled for the host (see gen_glue.py / Makefile) and run
against the same simulated MPU-6050.  Compared per scenario:
  * exit code
  * the UART bytes: identical, except that the new firmware numbers packets by DATA_RDY
    event (assigned in the ISR), so after a dropped event or a failed read `seq` skips;
    ts_us and the 14 sensor bytes of every packet must still match the old stream
  * the trace of every I2C / UART / delay call, identical apart from
      - where HAL_TIM_Base_Start sits (it moved earlier on purpose), and
      - the boot-time I2C bus clear (on a healthy bus: 5 x HAL_Delay(1) = clock-free STOP)
The current UART bytes are then fed to the real, unmodified tools/parse_imu.py
through a fake serial port.

Scenarios where the two versions are SUPPOSED to differ are checked separately:
  writefail  the new driver checks register writes (and the new main retries once)
  midbyte    slave stopped mid-transfer, SDA still HIGH at boot: only a STOP un-confuses it
  wedge      the bus wedges during our own init (reset hit later): DeInit + clear + retry
  stuckbus   a slave holds SDA low after a reset (the failure seen on the board):
             the old firmware dies in Error_Handler, the new one clears the bus
  deadbus    SDA never lets go: both versions stop in Error_Handler
"""
import os
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
B = HERE / "build"
PARSER = ROOT / "tools" / "parse_imu.py"
failures = []


def check(cond, msg):
    print(("  ok    " if cond else "  FAIL  ") + msg)
    if not cond:
        failures.append(msg)


def run(binname, scen):
    uart, trace = B / f"{binname}.{scen}.uart", B / f"{binname}.{scen}.trace"
    for p in (uart, trace):
        p.unlink(missing_ok=True)
    env = dict(os.environ, SIM_SCENARIO=scen, SIM_UART=str(uart), SIM_TRACE=str(trace),
               ASAN_OPTIONS="detect_leaks=0")
    rc = subprocess.run([str(B / binname)], env=env).returncode
    return rc, uart, (uart.read_bytes() if uart.exists() else b""), \
        (trace.read_text().splitlines() if trace.exists() else [])


def parse(uart):
    env = dict(os.environ, PYTHONPATH=str(HERE / "fake_serial"))
    out = subprocess.run([sys.executable, str(PARSER), str(uart), "0.5"], env=env,
                         capture_output=True, text=True, check=True).stdout
    m = re.search(r"ok=(\d+)\s+crc_bad=(\d+)\s+seq_gaps=(\d+)\s+lost=(\d+)\s+resync_bytes=(\d+)", out)
    assert m, out
    r = dict(zip(("ok", "crc_bad", "gaps", "lost", "resync"), map(int, m.groups())))
    d = re.search(r"dt\(us\): min=(\d+) max=(\d+)", out)
    r["dt_min"], r["dt_max"] = (int(d.group(1)), int(d.group(2))) if d else (None, None)
    return r


def frames(b):
    assert len(b) % 24 == 0
    return [(int.from_bytes(b[i + 2:i + 4], "little"), int.from_bytes(b[i + 4:i + 8], "little"), b[i + 8:i + 22])
            for i in range(0, len(b), 24)]


def strip_tim(t):
    return [l for l in t if l != "TIM_START"]


def split_recovery(t):
    """Remove the bus-clear prologue (leading HAL_Delay(1) lines); return (rest, how many)."""
    i = 0
    while i < len(t) and t[i] == "DELAY 1":
        i += 1
    return t[i:], i


SCENARIOS = {
    # name:      expected exit code, expected parse_imu.py results on the current stream
    "steady":   (0,  dict(ok=300, crc_bad=0, gaps=0, lost=0, resync=0)),
    "burst":    (0,  dict(ok=295, crc_bad=0, gaps=1, lost=5, resync=0)),   # 20 IRQs at once -> 5 dropped = seq hole of 5
    "readfail": (0,  dict(ok=298, crc_bad=0, gaps=1, lost=2, resync=0)),   # 2 failed I2C bursts = seq hole of 2
    "badid":    (42, dict(ok=0,   crc_bad=0, gaps=0, lost=0, resync=0)),
}

for scen, (exp_rc, exp) in SCENARIOS.items():
    print(f"== {scen}")
    orc, _, obytes, otrace = run("glue_old", scen)
    nrc, nuart, nbytes, ntrace = run("glue_new", scen)

    check(orc == exp_rc and nrc == exp_rc, f"exit code old={orc} new={nrc} (expected {exp_rc})")
    if scen in ("burst", "readfail"):
        of, nf = frames(obytes), frames(nbytes)
        check([f[1:] for f in of] == [f[1:] for f in nf],
              f"same packets (ts_us + sensor bytes): old {len(of)}, new {len(nf)}")
        check([f[0] for f in of] == list(range(len(of))), "old: seq is 0..n-1, blind to the loss")
        miss = sorted(set(range(300)) - {f[0] for f in nf})
        check(len(nf) + len(miss) == 300 and nf[-1][0] == 299,
              f"new: seq = event number; the {len(miss)} missing numbers {miss[0]}..{miss[-1]} are exactly the lost events")
    else:
        check(obytes == nbytes, f"UART bytes identical (old {len(obytes)} B, new {len(nbytes)} B)")
    ncore, nrec = split_recovery(ntrace)
    check(nrec == 5, f"healthy bus: the boot-time bus clear (STOP only) costs exactly 5 x HAL_Delay(1) (saw {nrec})")
    check(strip_tim(otrace) == strip_tim(ncore),
          f"HAL call trace identical apart from TIM_START and the bus clear ({len(otrace)} vs {len(ncore)} lines)")
    if any("reg=0x38" in l for l in ntrace):
        for name, t in (("old", otrace), ("new", ntrace)):
            int_en = next(i for i, l in enumerate(t) if "reg=0x38" in l)
            check(t.index("TIM_START") < int_en, f"{name}: timer started before DATA_RDY is enabled")

    r = parse(nuart)
    check(all(r[k] == v for k, v in exp.items()), f"parse_imu.py on current stream: { {k: r[k] for k in exp} }")
    if scen == "steady":
        check(10000 <= r["dt_min"] and r["dt_max"] <= 10003, f"dt(us) {r['dt_min']}..{r['dt_max']} (sim: 10000+0..3)")
    if scen == "burst":
        check(r["dt_max"] >= 60000, f"5 dropped events: {r['dt_max']} us timestamp gap AND a seq gap (lost={r['lost']})")
    if scen == "readfail":
        check(r["dt_max"] >= 30000, f"2 failed reads: {r['dt_max']} us timestamp gap AND a seq gap (lost={r['lost']})")

print("== writefail  (intended difference: the new driver checks register writes; main retries)")
orc, _, ob, _ = run("glue_old", "writefail")
nrc, _, nb, nt = run("glue_new", "writefail")
check(orc == 0 and len(ob) == 300 * 24, f"old: failed CONFIG write ignored, keeps streaming (exit {orc}, {len(ob)} B)")
check(nrc == 0 and len(nb) == 300 * 24 and nt.count("I2C_DEINIT") == 1,
      f"new: failed CONFIG write noticed, I2C re-initialised once, init retried OK (exit {nrc}, {len(nb)} B, {nt.count('I2C_DEINIT')} retry)")

print("== stuckbus  (intended difference: a slave holds SDA low after a reset; needs 5 clocks to let go)")
orc, _, ob, ot = run("glue_old", "stuckbus")
nrc, nuart, nb, nt = run("glue_new", "stuckbus")
check(orc == 42 and len(ob) == 0 and ot[-1] == "ERROR_HANDLER",
      f"old: first I2C transfer fails, halts in Error_Handler (exit {orc}, {len(ob)} B)  <- the failure seen on the board")
check(nrc == 0 and len(nb) == 300 * 24, f"new: bus cleared at boot, streams normally (exit {nrc}, {len(nb)} B)")
check(nt.count("DELAY 1") == 15, f"new: 5 clock pulses + STOP = 15 x HAL_Delay(1) (saw {nt.count('DELAY 1')})")
r = parse(nuart)
check(r["ok"] == 300 and r["crc_bad"] == 0 and r["gaps"] == 0 and r["lost"] == 0,
      f"parse_imu.py on the recovered stream: ok={r['ok']} crc_bad={r['crc_bad']} gaps={r['gaps']} lost={r['lost']}")

print("== midbyte  (SDA is high at boot, but the slave misbehaves at the next START unless a STOP came first)")
orc, _, ob, ot = run("glue_old", "midbyte")
nrc, nuart, nb, nt = run("glue_new", "midbyte")
check(orc == 42 and len(ob) == 0, f"old: first I2C transfer fails, halts in Error_Handler (exit {orc}, {len(ob)} B)")
check(nrc == 0 and len(nb) == 300 * 24 and "I2C_DEINIT" not in nt,
      f"new: the boot-time STOP is enough, no retry needed (exit {nrc}, {len(nb)} B)")
r = parse(nuart)
check(r["ok"] == 300 and r["crc_bad"] == 0 and r["gaps"] == 0, f"parse_imu.py: ok={r['ok']} crc_bad={r['crc_bad']} gaps={r['gaps']}")

print("== wedge  (bus wedges during our own init, after the boot-time clear already ran)")
orc, _, ob, _ = run("glue_old", "wedge")
nrc, nuart, nb, nt = run("glue_new", "wedge")
check(orc == 42 and len(ob) == 0, f"old: halts in Error_Handler (exit {orc}, {len(ob)} B)")
check(nrc == 0 and len(nb) == 300 * 24, f"new: recovers (exit {nrc}, {len(nb)} B)")
check(nt.count("I2C_DEINIT") == 1 and nt.count("I2C_INIT") == 1 and nt.index("I2C_DEINIT") < nt.index("I2C_INIT"),
      "new: peripheral released, bus cleared, peripheral re-initialised, in that order")
r = parse(nuart)
check(r["ok"] == 300 and r["crc_bad"] == 0 and r["gaps"] == 0, f"parse_imu.py: ok={r['ok']} crc_bad={r['crc_bad']} gaps={r['gaps']}")

print("== relapse  (our first transfer fails with SDA still high: the retry's bus clear must send a STOP)")
orc, _, ob, _ = run("glue_old", "relapse")
nrc, nuart, nb, nt = run("glue_new", "relapse")
check(orc == 42 and len(ob) == 0, f"old: halts in Error_Handler (exit {orc}, {len(ob)} B)")
check(nrc == 0 and len(nb) == 300 * 24 and nt.count("I2C_DEINIT") == 1, f"new: one retry is enough (exit {nrc}, {len(nb)} B, {nt.count('I2C_DEINIT')} retry)")

print("== deadbus  (SDA never lets go: nothing can be done in software)")
orc, _, ob, _ = run("glue_old", "deadbus")
nrc, _, nb, nt = run("glue_new", "deadbus")
check(orc == 42 and len(ob) == 0, f"old: halts in Error_Handler (exit {orc})")
check(nrc == 42 and len(nb) == 0 and nt[-1] == "ERROR_HANDLER",
      f"new: gives up and halts in Error_Handler (exit {nrc}), does not hang in the bus clear")
check(nt.count("I2C_DEINIT") == 2, f"new: exactly 3 attempts (2 retries), saw {nt.count('I2C_DEINIT')} retries")

print()
if failures:
    print(f"{len(failures)} FAILED")
    sys.exit(1)
print("all glue checks passed")
