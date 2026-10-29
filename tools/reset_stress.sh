#!/usr/bin/env bash
# Reset the MCU at a random moment while it is streaming, N times in a row, and check
# after every reset that it boots and keeps streaming.  Needs the board flashed, the
# debug probe and the UART connected.
#
#   tools/reset_stress.sh [N=60] [PORT=/dev/ttyUSB0]
#
# Why: a reset in the middle of an I2C transfer can leave the MPU-6050 holding SDA low,
# after which every boot fails with "arbitration lost" until the sensor is power cycled.
# Without bus recovery this script stops at the first such failure (and you must power
# cycle the board); with recovery it should get through all N resets.
set -u

N=${1:-60}
PORT=${2:-/dev/ttyUSB0}
HERE=$(cd "$(dirname "$0")" && pwd)
OCD=(openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg -c "adapter speed 1000")

for i in $(seq 1 "$N"); do
    d=$(( RANDOM % 250 + 150 ))      # ms after the first reset: firmware is streaming by then
    "${OCD[@]}" -c "init; reset run; sleep $d; reset run; sleep 300; exit" >/dev/null 2>&1 \
        || { echo "openocd failed at reset #$i"; exit 2; }
    out=$(python3 "$HERE/parse_imu.py" "$PORT" 2 | head -1)
    ok=$(sed -n 's/.*ok=\([0-9]*\).*/\1/p' <<<"$out")
    if [ "${ok:-0}" -lt 150 ]; then
        echo
        echo "FAIL at reset #$i (second reset ${d} ms after the first): $out"
        echo "-> $((i-1)) resets succeeded before this one"
        exit 1
    fi
    printf '.'
done
echo
echo "all $N resets recovered and kept streaming"
