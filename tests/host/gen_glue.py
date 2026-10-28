#!/usr/bin/env python3
"""Turn the USER CODE blocks of a CubeMX main.c into a program that compiles on the host.

usage: gen_glue.py <main.c> <out.c>

Only text the developer wrote is extracted (Includes, PV, 0, 2, 3), so the same
script works on the pre-refactor and post-refactor main.c.  `while (1)` becomes
`while (sim_continue())` so the simulated sensor can fire interrupts and end the run.
"""
import re
import sys

src = open(sys.argv[1], encoding="utf-8", errors="replace", newline="").read().replace("\r\n", "\n")


def block(name):
    pat = r"/\* USER CODE BEGIN %s \*/\n(.*?)/\* USER CODE END %s \*/" % (re.escape(name), re.escape(name))
    m = re.search(pat, src, re.S)
    if not m:
        sys.exit("gen_glue: missing USER CODE block '%s' in %s" % (name, sys.argv[1]))
    return m.group(1)


parts = [
    '#include "glue_prelude.h"',
    block("Includes"),
    block("PV"),
    block("0"),
    "int main(void)\n{",
    block("2"),
    "  while (sim_continue()) {",
    block("3"),            # CubeMX puts the loop's closing '}' inside this block
    "  return 0;\n}\n",
]
open(sys.argv[2], "w", encoding="utf-8").write("\n".join(parts))
