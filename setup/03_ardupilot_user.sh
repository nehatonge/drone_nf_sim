#!/usr/bin/env bash
# Python venv + ArduCopter SITL build + ardupilot_gazebo plugin build. Run as user neha.
set -euo pipefail

VENV="$HOME/venv-ardupilot"
python3 -m venv --system-site-packages "$VENV"
source "$VENV/bin/activate"
pip install -U pip wheel
pip install future lxml pymavlink pyserial MAVProxy geocoder empy==3.3.4 ptyprocess dronecan \
  numpy pyparsing psutil matplotlib scipy pyyaml intelhex pandas

grep -q venv-ardupilot ~/.bashrc || cat >> ~/.bashrc <<'EOF'

# --- drone near-field sim
source /opt/ros/jazzy/setup.bash
source $HOME/venv-ardupilot/bin/activate
export PATH=$HOME/ardupilot/Tools/autotest:/usr/lib/ccache:$PATH
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/mnt/c/Users/nehat/drone_nf_sim/sim/models:$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
EOF

# ArduCopter SITL
cd "$HOME/ardupilot"
./waf configure --board sitl
./waf copter -j"$(nproc)"

# ardupilot_gazebo plugin (Gazebo Harmonic)
export GZ_VERSION=harmonic
cd "$HOME/gz_ws/src/ardupilot_gazebo"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j"$(nproc)"
echo "ARDUPILOT USER BUILD DONE"
