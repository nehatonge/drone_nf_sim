#!/usr/bin/env bash
# One complete, repeatable simulated flight:
#   fresh Gazebo + SITL (batch) -> flight_test.py -> analyze_flight.py -> stop
#
#   ./test_run.sh [--model nf_hexa] [--wind 0] [--gust 0] -- <flight_test args>
#   e.g. ./test_run.sh --model nf_hexa_trimmed --wind 5 --gust 2 -- scan --freq-ghz 3
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ARGS=()
MODEL=nf_hexa
while [[ $# -gt 0 && "$1" != "--" ]]; do
  [[ "$1" == "--model" ]] && MODEL="$2"
  SIM_ARGS+=("$1"); shift
done
shift
source /opt/ros/jazzy/setup.bash
source "$HOME/venv-ardupilot/bin/activate"

"$HERE/stop_sim.sh" >/dev/null
"$HERE/run_sim.sh" --batch --headless "${SIM_ARGS[@]}" > "$HOME/nf_runs/sim_console.log" 2>&1 &
sleep 12
export NF_MODEL="$MODEL"
OUT=$(python3 -u "$HERE/flight_test.py" "$@" 2>&1 | tee /dev/stderr | grep RESULT_CSV | awk '{print $2}')
"$HERE/stop_sim.sh" >/dev/null
if [[ -n "$OUT" ]]; then
  python3 "$HERE/../analysis/analyze_flight.py" "$OUT" 2>/dev/null
fi
