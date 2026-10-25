#!/usr/bin/env bash
# Launch Gazebo (nf_range world) + ArduPilot SITL for the near-field hexacopter.
#
#   ./run_sim.sh [--model nf_hexa|nf_hexa_trimmed] [--wind MPS] [--gust MPS] [--headless]
#
# Gazebo runs in the background; SITL (MAVProxy console) runs in the foreground.
# MAVLink for scripts: udp:127.0.0.1:14550
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL=nf_hexa
WIND=0
GUST=0
HEADLESS=0
BATCH=0
PARAM_ARGS=("--add-param-file=$HERE/params/nf_hexa.parm")
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model) MODEL="$2"; shift 2 ;;
    --wind) WIND="$2"; shift 2 ;;
    --gust) GUST="$2"; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --batch) HEADLESS=1; BATCH=1; shift ;;
    --params) PARAM_ARGS+=("--add-param-file=$(realpath "$2")"); shift 2 ;;
    *) echo "unknown arg $1"; exit 1 ;;
  esac
done

# WSLg: render Gazebo on the NVIDIA GPU through D3D12
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

export GZ_SIM_RESOURCE_PATH="$HERE/models:$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

# Build the world for this run (model + wind)
RUN_DIR="$HOME/nf_runs"
mkdir -p "$RUN_DIR"
WORLD="$RUN_DIR/nf_range_run.sdf"
WIND_XML=""
if [[ "$WIND" != "0" || "$GUST" != "0" ]]; then
  # Wind from the north-west quarter, blowing across the scan plane and onto the AUT.
  WIND_XML="<wind><linear_velocity>$(python3 -c "import math;w=$WIND;print(f'{-w*math.cos(math.radians(30)):.3f} {-w*math.sin(math.radians(30)):.3f} 0')")</linear_velocity></wind>
    <plugin filename=\"gz-sim-wind-effects-system\" name=\"gz::sim::systems::WindEffects\">
      <force_approximation_scaling_factor>0.35</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>5</time_for_rise>
          <sin><amplitude_percent>$(python3 -c "w=$WIND;g=$GUST;print(f'{(g/w if w>0 else 0):.3f}')")</amplitude_percent><period>8</period></sin>
          <noise type=\"gaussian\"><mean>0</mean><stddev>$(python3 -c "print(f'{0.15*$GUST:.3f}')")</stddev></noise>
        </magnitude>
        <direction>
          <time_for_rise>20</time_for_rise>
          <sin><amplitude>8</amplitude><period>30</period></sin>
          <noise type=\"gaussian\"><mean>0</mean><stddev>2</stddev></noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type=\"gaussian\"><mean>0</mean><stddev>$(python3 -c "print(f'{0.05*$GUST:.3f}')")</stddev></noise>
      </vertical>
    </plugin>"
fi
python3 - "$HERE/worlds/nf_range.sdf" "$WORLD" "$MODEL" "$WIND_XML" <<'EOF'
import sys
src, dst, model, wind = sys.argv[1:5]
s = open(src).read().replace("MODEL_NAME", model).replace("<!--WIND_PLUGIN-->", wind)
open(dst, "w").write(s)
EOF
echo "world: $WORLD  model: $MODEL  wind: $WIND m/s  gust: $GUST m/s"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

GZ_ARGS="-r -v2 $WORLD"
[[ $HEADLESS == 1 ]] && GZ_ARGS="-s $GZ_ARGS"
gz sim $GZ_ARGS > "$RUN_DIR/gz.log" 2>&1 &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null; pkill -f "gz sim" 2>/dev/null' EXIT
sleep 5

# ArduPilot SITL, hexa frame, physics from Gazebo (JSON)
[[ -f "$HOME/venv-ardupilot/bin/activate" ]] && source "$HOME/venv-ardupilot/bin/activate"
cd "$RUN_DIR"
if [[ $BATCH == 1 ]]; then
  # no MAVProxy: scripts connect straight to SITL on tcp:127.0.0.1:5760
  "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" -v ArduCopter -f hexa --model JSON \
    "${PARAM_ARGS[@]}" --no-rebuild --no-mavproxy -w
else
  "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" -v ArduCopter -f hexa --model JSON \
    "${PARAM_ARGS[@]}" \
    --no-rebuild --console --out=udp:127.0.0.1:14550 -w
fi
