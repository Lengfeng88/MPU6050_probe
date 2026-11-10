#!/usr/bin/env bash
# Start a detached, delayed IMU recording that survives SSH disconnects.
#   tools/rec.sh LABEL DURATION_S [DELAY_S] [NOTE]
# Env: IMU_PORT (default /dev/ttyAMA0), IMU_SUBJECT (default kelvin)
set -euo pipefail

label=${1:-}; dur=${2:-}; delay=${3:-15}; note=${4:-}
if [[ -z $label || -z $dur ]]; then
  echo "usage: $0 LABEL DURATION_S [DELAY_S] [NOTE]" >&2; exit 2
fi
[[ $label =~ ^[A-Za-z0-9_-]+$ ]] || { echo "label: letters, digits, - and _ only" >&2; exit 2; }
[[ $dur   =~ ^[0-9]+$ ]] || { echo "DURATION_S must be a whole number of seconds" >&2; exit 2; }
[[ $delay =~ ^[0-9]+$ ]] || { echo "DELAY_S must be a whole number of seconds" >&2; exit 2; }
command -v tmux >/dev/null || { echo "tmux is not installed" >&2; exit 1; }

port=${IMU_PORT:-/dev/ttyAMA0}
subject=${IMU_SUBJECT:-kelvin}
root=$(cd "$(dirname "$0")/.." && pwd)
name="rec-$label"
mkdir -p "$root/data"

if tmux has-session -t "$name" 2>/dev/null; then
  echo "tmux session '$name' is already running" >&2; exit 1
fi

tmux new-session -d -s "$name" -c "$root" bash -c '
  echo "recording starts in $1 s ..."; sleep "$1"
  python3 tools/imu_record.py "$2" --label "$3" --subject "$4" --note "$5" --duration "$6" --out data 2>&1 \
    | tee -a "data/rec_$3.log"
  echo "recorder exit code: ${PIPESTATUS[0]}" | tee -a "data/rec_$3.log"
  sleep 5
' _ "$delay" "$port" "$label" "$subject" "$note" "$dur"

echo "started '$name': waits ${delay}s, records ${dur}s. Log: data/rec_${label}.log"
echo "attach: tmux attach -t $name    (detach: Ctrl-b d)"
