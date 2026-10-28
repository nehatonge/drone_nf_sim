#!/usr/bin/env bash
# Launch Gazebo (CATR lab world) + ArduPilot SITL + UWB positioning for the near-field hexacopter.
#
#   ./run_sim.sh [--model nf_hexa|nf_hexa_trimmed|nf_hexa_down] [--wind MPS] [--gust MPS] [--headless] [--batch]
#                [--params FILE] [--uwb "ARGS"]     e.g. --uwb "--range-sigma 0.10 --nlos-prob 0.05"
#                [--aut-ghz F] [--n-lambda N] [--spawn "X Y"]
#   The take-off point is computed from the antenna frequency (catr_layout.takeoff_point:
#   scan distance N*lambda + probe offset, limited to the flat floor) unless --spawn is given.
#
# Gazebo and uwb_extnav.py run in the background; SITL (MAVProxy console) in the foreground.
# --wind/--gust stand in for air drafts (HVAC, downwash recirculation) in the lab.
# MAVLink for scripts: udp:127.0.0.1:14550 (or tcp:127.0.0.1:5760 with --batch)
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL=nf_hexa
WIND=0
GUST=0
HEADLESS=0
BATCH=0
UWB_ARGS=""
AUT_GHZ=5.0
N_LAMBDA=5
SPAWN=""
PARAM_ARGS=("--add-param-file=$HERE/params/nf_hexa.parm")
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model) MODEL="$2"; shift 2 ;;
    --wind) WIND="$2"; shift 2 ;;
    --gust) GUST="$2"; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --batch) BATCH=1; shift ;;         # no MAVProxy; scripts connect on tcp:5760 (add --headless for no Gazebo window)
    --params) PARAM_ARGS+=("--add-param-file=$(realpath "$2")"); shift 2 ;;
    --uwb) UWB_ARGS="$2"; shift 2 ;;
    --aut-ghz) AUT_GHZ="$2"; shift 2 ;;
    --n-lambda) N_LAMBDA="$2"; shift 2 ;;
    --spawn) SPAWN="$2"; shift 2 ;;
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
WORLD="$RUN_DIR/catr_lab_run.sdf"
WIND_XML=""
if [[ "$WIND" != "0" || "$GUST" != "0" ]]; then
  # Draft blowing down-range and across the scan plane, onto the AUT.
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
[[ -z "$SPAWN" ]] && SPAWN=$(cd "$HERE" && python3 catr_layout.py "$AUT_GHZ" "$N_LAMBDA")
read -r SPAWN_X SPAWN_Y <<< "$SPAWN"
python3 - "$HERE/worlds/catr_lab.sdf" "$WORLD" "$MODEL" "$WIND_XML" "$SPAWN_X" "$SPAWN_Y" <<'EOF'
import sys
src, dst, model, wind, sx, sy = sys.argv[1:7]
s = open(src).read().replace("MODEL_NAME", model).replace("<!--WIND_PLUGIN-->", wind)
s = s.replace("SPAWN_X", sx).replace("SPAWN_Y", sy)
open(dst, "w").write(s)
EOF
echo "world: $WORLD  model: $MODEL  take-off: ($SPAWN_X, $SPAWN_Y)  wind: $WIND m/s  gust: $GUST m/s"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

GZ_ARGS="-r -v2 $WORLD"
[[ $HEADLESS == 1 ]] && GZ_ARGS="-s $GZ_ARGS"
gz sim $GZ_ARGS > "$RUN_DIR/gz.log" 2>&1 &
GZ_PID=$!
sleep 5

[[ -f "$HOME/venv-ardupilot/bin/activate" ]] && source "$HOME/venv-ardupilot/bin/activate"

# UWB positioning (no GNSS indoors); connects to SITL SERIAL1 once SITL is up
export NF_MODEL="$MODEL"
python3 -u "$HERE/uwb_extnav.py" $UWB_ARGS > "$RUN_DIR/uwb.log" 2>&1 &
UWB_PID=$!
trap 'kill $UWB_PID $GZ_PID 2>/dev/null; pkill -f "gz sim" 2>/dev/null' EXIT

# ArduPilot SITL, hexa frame, physics from Gazebo (JSON)
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
