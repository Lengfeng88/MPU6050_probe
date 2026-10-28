#!/usr/bin/env python3
"""A/B test of the main.c glue: pre-refactor vs post-refactor.

Both versions are compiled for the host (see gen_glue.py / Makefile) and run
against the same simulated MPU-6050.  Compared per scenario:
  * exit code
  * the UART bytes (must be identical)
  * the trace of every I2C / UART / delay call (identical apart from where
    HAL_TIM_Base_Start sits, which moved earlier on purpose)
The post-refactor UART bytes are then fed to the real, unmodified
tools/parse_imu.py through a fake serial port.
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


def strip_tim(t):
    return [l for l in t if l != "TIM_START"]


SCENARIOS = {
    # name:      expected exit code, expected parse_imu.py results on the NEW stream
    "steady":   (0,  dict(ok=300, crc_bad=0, gaps=0, lost=0, resync=0)),
    "burst":    (0,  dict(ok=295, crc_bad=0, gaps=0, lost=0, resync=0)),   # 20 IRQs at once -> 5 dropped
    "readfail": (0,  dict(ok=298, crc_bad=0, gaps=0, lost=0, resync=0)),   # 2 failed I2C bursts
    "badid":    (42, dict(ok=0,   crc_bad=0, gaps=0, lost=0, resync=0)),
}

for scen, (exp_rc, exp) in SCENARIOS.items():
    print(f"== {scen}")
    orc, _, obytes, otrace = run("glue_old", scen)
    nrc, nuart, nbytes, ntrace = run("glue_new", scen)

    check(orc == exp_rc and nrc == exp_rc, f"exit code old={orc} new={nrc} (expected {exp_rc})")
    check(obytes == nbytes, f"UART bytes identical (old {len(obytes)} B, new {len(nbytes)} B)")
    check(strip_tim(otrace) == strip_tim(ntrace),
          f"HAL call trace identical apart from TIM_START ({len(otrace)} vs {len(ntrace)} lines)")
    if any("reg=0x38" in l for l in ntrace):
        for name, t in (("old", otrace), ("new", ntrace)):
            int_en = next(i for i, l in enumerate(t) if "reg=0x38" in l)
            check(t.index("TIM_START") < int_en, f"{name}: timer started before DATA_RDY is enabled")

    r = parse(nuart)
    check(all(r[k] == v for k, v in exp.items()), f"parse_imu.py on new stream: { {k: r[k] for k in exp} }")
    if scen == "steady":
        check(10000 <= r["dt_min"] and r["dt_max"] <= 10003, f"dt(us) {r['dt_min']}..{r['dt_max']} (sim: 10000+0..3)")
    if scen == "burst":
        check(r["dt_max"] >= 60000, f"5 dropped events show up as a {r['dt_max']} us timestamp gap, NOT as a seq gap")
    if scen == "readfail":
        check(r["dt_max"] >= 30000, f"2 failed reads show up as a {r['dt_max']} us timestamp gap, NOT as a seq gap")

print("== writefail  (intended difference: the new driver checks register writes)")
orc, _, ob, _ = run("glue_old", "writefail")
nrc, _, nb, _ = run("glue_new", "writefail")
check(orc == 0 and len(ob) == 300 * 24, f"old: failed CONFIG write ignored, keeps streaming (exit {orc}, {len(ob)} B)")
check(nrc == 42 and len(nb) == 0, f"new: failed CONFIG write halts in Error_Handler (exit {nrc}, {len(nb)} B)")

print()
if failures:
    print(f"{len(failures)} FAILED")
    sys.exit(1)
print("all glue checks passed")
