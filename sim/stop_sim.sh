#!/usr/bin/env bash
# Stops Gazebo, ArduPilot SITL and MAVProxy started by run_sim.sh
for p in arducopter sim_vehicle.py mavproxy.py "gz sim" "gz-sim" xterm; do
  pkill -f -- "$p" 2>/dev/null
done
sleep 1
pgrep -af "arducopter|gz sim" >/dev/null && pkill -9 -f "arducopter|gz sim" 2>/dev/null
echo "simulation stopped"
