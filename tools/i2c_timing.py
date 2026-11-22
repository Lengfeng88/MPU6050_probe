#!/usr/bin/env python3
"""STM32 I2C TIMINGR: decode a value into times and a frequency range, or search one for a target SCL frequency.

    python3 tools/i2c_timing.py --decode 0x10C0ECFF --clk 100e6            # what the current 100 kHz value gives
    python3 tools/i2c_timing.py --search 400e3 --clk 100e6                 # a value for at most 400 kHz

Formulas of the STM32H7 reference manual (RM0433, I2C timings):
    tSCL   = tSYNC1 + tSYNC2 + ((SCLH + 1) + (SCLL + 1)) * (PRESC + 1) * tI2CCLK
    tSYNC1 = tf + tAF + (DNF + 2..3) * tI2CCLK        tSYNC2 = tr + tAF + (DNF + 2..3) * tI2CCLK
    SCLDEL >= (tr + tSU;DAT) / tPRESC - 1
    SDADEL >= (tf - tHD;DAT(min) - tAF(min) - (DNF + 3) * tI2CCLK) / tPRESC
    SDADEL <= (tHD;DAT(max) - tr - tAF(max) - (DNF + 4) * tI2CCLK) / tPRESC
These were written from memory of the manual, so check them against it.  The analogue filter delay (50 to 260 ns) and the rise and
fall times of YOUR bus (tr, tf: pull-ups, wire capacitance) are inputs, not facts: the result is an estimate.  Measure SCL on the bus.
The search never returns a value whose fastest case (tr = tf = 0) is above the target.
"""
import argparse
import math
import sys

TAF = (50e-9, 260e-9)          # analogue filter delay min / max (STM32H7 datasheet value, from memory)
FM = dict(t_low=1300e-9, t_high=600e-9, t_su=100e-9, t_hd_max=900e-9, tr=300e-9, tf=300e-9)    # I2C fast mode limits (NXP UM10204)


def fields(timingr):
    return dict(presc=timingr >> 28 & 0xF, scldel=timingr >> 20 & 0xF, sdadel=timingr >> 16 & 0xF,
                sclh=timingr >> 8 & 0xFF, scll=timingr & 0xFF)


def encode(presc, scldel, sdadel, sclh, scll):
    return presc << 28 | scldel << 20 | sdadel << 16 | sclh << 8 | scll


def decode(timingr, clk, tr=100e-9, tf=100e-9, dnf=0):
    """-> dict with the fields, the programmed low / high times and the SCL frequency range [Hz] for the given tr / tf."""
    f = fields(timingr)
    tcl = 1.0 / clk
    tp = (f["presc"] + 1) * tcl
    t_low, t_high = (f["scll"] + 1) * tp, (f["sclh"] + 1) * tp
    sync = lambda af, k: (tf + af + (dnf + k) * tcl) + (tr + af + (dnf + k) * tcl)
    t_min = t_low + t_high + sync(TAF[0], 2)
    t_max = t_low + t_high + sync(TAF[1], 3)
    f.update(t_scll=t_low, t_sclh=t_high, t_scl_min=t_min, t_scl_max=t_max, f_max_hz=1 / t_min, f_min_hz=1 / t_max)
    return f


def search(clk, f_target, tr=100e-9, tf=100e-9, dnf=0, lim=FM):
    """Best TIMINGR whose fastest case does not exceed f_target; None if there is none.
    The limit is checked with tr = tf = 0 (the fastest bus there could be), the printed estimate uses the given tr / tf."""
    tcl = 1.0 / clk
    best = None
    for presc in range(16):
        tp = (presc + 1) * tcl
        scldel = max(0, math.ceil((lim["tr"] + lim["t_su"]) / tp - 1e-9) - 1)
        sdadel = max(0, math.ceil((lim["tf"] - TAF[0] - (dnf + 3) * tcl) / tp - 1e-9))
        sdadel_max = math.floor((lim["t_hd_max"] - lim["tr"] - TAF[1] - (dnf + 4) * tcl) / tp + 1e-9)
        if scldel > 15 or sdadel > 15 or sdadel > sdadel_max:
            continue
        for scll in range(256):
            if (scll + 1) * tp < lim["t_low"]:
                continue
            for sclh in range(256):
                if (sclh + 1) * tp < lim["t_high"]:
                    continue
                v = encode(presc, scldel, sdadel, sclh, scll)
                bound = decode(v, clk, 0.0, 0.0, dnf)["f_max_hz"]
                if bound > f_target:
                    continue
                key = (bound, -abs((scll + 1) / (sclh + 1) - 2.0))             # as fast as allowed, low about twice the high
                if best is None or key > best[0]:
                    best = (key, dict(decode(v, clk, tr, tf, dnf), f_bound_hz=bound))
    return None if best is None else dict(best[1], timingr=encode(*(best[1][k] for k in ("presc", "scldel", "sdadel", "sclh", "scll"))))


def show(d):
    print(f"PRESC {d['presc']}  SCLDEL {d['scldel']}  SDADEL {d['sdadel']}  SCLH {d['sclh']} (0x{d['sclh']:02X})  SCLL {d['scll']} (0x{d['scll']:02X})")
    print(f"programmed low {d['t_scll']*1e9:.0f} ns, high {d['t_sclh']*1e9:.0f} ns; SCL period {d['t_scl_min']*1e9:.0f} to {d['t_scl_max']*1e9:.0f} ns "
          f"= {d['f_min_hz']/1e3:.1f} to {d['f_max_hz']/1e3:.1f} kHz (estimate)")
    if "f_bound_hz" in d:
        print(f"upper bound for any bus (tr = tf = 0): {d['f_bound_hz']/1e3:.1f} kHz")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--decode", help="TIMINGR value, e.g. 0x10C0ECFF")
    g.add_argument("--search", type=float, help="target SCL frequency in Hz (an upper limit)")
    ap.add_argument("--clk", type=float, default=100e6, help="I2C kernel clock in Hz (default 100e6)")
    ap.add_argument("--tr", type=float, default=100e-9, help="bus rise time estimate, s")
    ap.add_argument("--tf", type=float, default=100e-9, help="bus fall time estimate, s")
    a = ap.parse_args(argv)
    if a.decode:
        show(decode(int(a.decode, 0), a.clk, a.tr, a.tf))
        return 0
    d = search(a.clk, a.search, a.tr, a.tf)
    if d is None:
        print("no value meets the limits", file=sys.stderr)
        return 1
    print(f"TIMINGR = 0x{d['timingr']:08X}")
    show(d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
