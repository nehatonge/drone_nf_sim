#!/usr/bin/env bash
# One complete, repeatable simulated flight:
#   fresh Gazebo + SITL (batch) -> flight_test.py -> analyze_flight.py -> stop
#
#   ./test_run.sh [--gui] [--model nf_hexa] [--wind 0] [--gust 0] -- <flight_test args>
#   (headless unless --gui)
#   e.g. ./test_run.sh --model nf_hexa_trimmed --wind 5 --gust 2 -- scan --freq-ghz 3
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ARGS=()
MODEL=nf_hexa
HEADLESS_ARG=--headless
while [[ $# -gt 0 && "$1" != "--" ]]; do
  [[ "$1" == "--gui" ]] && { HEADLESS_ARG=""; shift; continue; }   # show the Gazebo window
  [[ "$1" == "--model" ]] && MODEL="$2"
  SIM_ARGS+=("$1"); shift
done
shift
source /opt/ros/jazzy/setup.bash
source "$HOME/venv-ardupilot/bin/activate"

"$HERE/stop_sim.sh" >/dev/null
"$HERE/run_sim.sh" --batch $HEADLESS_ARG "${SIM_ARGS[@]}" > "$HOME/nf_runs/sim_console.log" 2>&1 &
sleep 12
export NF_MODEL="$MODEL"
LOG="$HOME/nf_runs/flight_test.log"
python3 -u "$HERE/flight_test.py" "$@" 2>&1 | tee "$LOG"
"$HERE/stop_sim.sh" >/dev/null
OUT=$(grep RESULT_CSV "$LOG" | tail -1 | awk '{print $2}')
FID=$(grep FLIGHT_ID "$LOG" | head -1 | awk '{print $2}')
DB=$(grep FLIGHT_ID "$LOG" | head -1 | awk '{print $4}')       # "FLIGHT_ID <id> -> <database>"
if [[ -n "$OUT" ]]; then
  python3 "$HERE/../analysis/analyze_flight.py" "$OUT" 2>/dev/null
  PNG="${OUT%.csv}.png"
  if [[ -n "$FID" ]]; then
    [[ -f "$PNG" ]] && python3 "$HERE/scan_db.py" --db "$DB" attach "$FID" flight_plot "$PNG" >/dev/null
    python3 "$HERE/scan_db.py" --db "$DB" attach "$FID" flight_test_log "$LOG" >/dev/null
  fi
fi
