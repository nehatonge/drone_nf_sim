# drone_nf_sim

Simulation of a drone-based near-field antenna measurement system: a heavy-lift
hexacopter carries a waveguide probe (WGP) through a planar raster in front of an
antenna under test (AUT). The goal is to find out, before any hardware is built,
how accurately the drone can hold and know the probe position, and what that does
to the reconstructed far-field pattern.

Companion to the report *Drone-Based Near-Field Antenna Measurement System:
Architecture Report* (Oct 2026). Section and figure numbers below refer to it.

**Stack:** Ubuntu 24.04 (WSL2) · ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8) ·
ArduPilot Copter 4.6 SITL · `ardupilot_gazebo` plugin · Python 3

## How the report's components appear in the simulation

Numbers match Table 1 / Figure 3 of the report.

| # | Component | In the simulation | Where |
|---|---|---|---|
| 1 | Frame, 1400 mm wheelbase | `base_link` with lumped mass and inertia of plates, arms, landing gear | `sim/gen_nf_hexa.py` |
| 2 | 6 × T-Motor P60 / U8 Lite, 170 KV | 6 revolute rotor joints, velocity-controlled; ω_max 576 rad/s, 4.85 kg peak thrust each | `sim/gen_nf_hexa.py` |
| 3 | 24 in carbon props, 3 CW + 3 CCW | 2 LiftDrag blades per rotor, sized so T = kω² and reaction torque match | `sim/gen_nf_hexa.py` |
| 4 | ESC (6 channels) | Implicit: ArduPilotPlugin maps PWM 1100–1900 µs to rotor speed | model plugin block |
| 5 | Battery 12S 22 Ah | 5.6 kg mass, position is the CG trim (`--battery-x`); `BATT_CAPACITY` | generator, `params/nf_hexa.parm` |
| 6 | Cube Orange | ArduCopter SITL (Hexa-X, `FRAME_CLASS 2`, `FRAME_TYPE 1`) + 1 kHz IMU sensor | `sim/run_sim.sh`, `imu_link` |
| 7 | Jetson Orin Nano | Played by `flight_test.py`: commands GUIDED setpoints over MAVLink and logs the probe pose. Not yet a ROS 2 node. | `sim/flight_test.py` |
| 8 | RTK-GNSS + compass | SITL GPS at 10 Hz for flight; RTK error modelled in post-processing as a Gauss–Markov drift (σ 10 mm, τ 60 s) | `params/nf_hexa.parm`, `analysis/analyze_flight.py` |
| 9 | Telemetry radio | MAVLink over UDP 14550 (MAVProxy) / TCP 5760 (scripts) | `sim/run_sim.sh` |
| 10 | Rangefinder (down) | **Disabled** (`RNGFND1_TYPE 0`): the JSON interface does not pass it to SITL | `params/nf_hexa.parm` |
| 11 | HereFlow optical flow | SITL optical flow (`FLOW_TYPE 10`) | `params/nf_hexa.parm` |
| 12 | VNA / RF receiver | Mass only (0.40 kg). **No RF measurement yet** (report step 5) | `sim/gen_nf_hexa.py` |
| 13 | Probe mount | Mass only (0.20 kg) | `sim/gen_nf_hexa.py` |
| 14 | Probe boom + coax | `boom` link, GFRP tube 0.10–1.20 m forward, 0.45 kg | `sim/gen_nf_hexa.py` |
| 15 | Waveguide probe | `probe` link at x = 1.25 m, z = −0.15 m (WR-284 S-band OEWG, 0.8 kg) | `sim/gen_nf_hexa.py` |
| G | AUT + RF source | 1 m × 1 m aperture on a mast at (0, 0, 4 m), facing +x | `sim/worlds/nf_range.sdf` |
| G | RTK base, GCS | Visual only | `sim/worlds/nf_range.sdf` |
| G | 10 MHz reference | Not modelled | — |
| — | Wind and gusts | Gazebo WindEffects, set per run (`--wind`, `--gust`) | `sim/run_sim.sh` |

Masses marked `EST` in the code are estimates. Replace them with datasheet values
when the parts are chosen, then regenerate the model.

## Pipeline status (report Figure 4)

| Step | Description | Status |
|---|---|---|
| 1 | Scenario setup (band, AUT, scan plane, error budget) | `analysis/sizing_check.py`, CLI args |
| 2 | Scan path planner | `analysis/scan_path.py`, `raster_traj()` in `flight_test.py` |
| 3 | Drone simulation, flight controller in the loop | Gazebo + ArduPilot SITL (ROS 2 bridge not used yet) |
| 4 | Pose + data log | CSV of true probe pose from Gazebo (not rosbag yet) |
| 5 | EM near-field model at logged poses | To do |
| 6 | NF-FF transformation (probe correction, plane-wave spectrum) | To do |
| 7 | Far-field pattern + error metrics | Position/phase-error verdict per band in `analyze_flight.py`; pattern comparison to do |

## Layout

```
setup/      install scripts for WSL Ubuntu 24.04 (run in order 01, 02, 03)
sim/
  gen_nf_hexa.py       generates the hexacopter SDF from the component list
  models/              generated models: nf_hexa (as built), nf_hexa_trimmed (battery moved aft)
  worlds/nf_range.sdf  test range: AUT, scan plane outline, RTK base, GCS
  params/              ArduCopter parameter files
  run_sim.sh           start Gazebo + SITL
  flight_test.py       hold / hover / scan flights, logs true probe pose
  test_run.sh          one complete repeatable run: sim -> flight -> analysis -> stop
  stop_sim.sh          kill everything
analysis/
  sizing_check.py      mass, thrust, CG, prop clearance, endurance, tolerance, scan time
  scan_path.py         planar serpentine raster as JSON
  analyze_flight.py    tracking/knowledge error and phase error per band, plots
```

## Setup (once)

In WSL Ubuntu 24.04, with ArduPilot cloned to `~/ardupilot` and `ardupilot_gazebo`
cloned to `~/gz_ws/src/ardupilot_gazebo`:

```bash
sudo bash setup/01_install_ros2_gazebo.sh
sudo bash setup/02_ardupilot_root.sh
bash setup/03_ardupilot_user.sh        # venv, SITL build, plugin build, ~/.bashrc entries
```

`03` writes the repo path `/mnt/c/Users/nehat/drone_nf_sim` into `~/.bashrc`.
Change it there if you clone the repo somewhere else.

## Running

```bash
cd /mnt/c/Users/nehat/drone_nf_sim

python3 analysis/sizing_check.py                    # static checks, no simulator

# regenerate models after changing masses/geometry
python3 sim/gen_nf_hexa.py
python3 sim/gen_nf_hexa.py --name nf_hexa_trimmed --battery-x -0.24

# interactive: Gazebo GUI + MAVProxy console
./sim/run_sim.sh --model nf_hexa_trimmed

# one complete batch run (headless), e.g. S-band scan in 5 m/s wind with 2 m/s gusts
./sim/test_run.sh --model nf_hexa_trimmed --wind 5 --gust 2 -- scan --freq-ghz 3
./sim/test_run.sh --model nf_hexa_trimmed -- hover --hold 60
./sim/test_run.sh --model nf_hexa_trimmed --params sim/params/gains_try1.parm -- hold
```

Results (CSV + PNG) go to `~/nf_runs/`, outside the repo.

## Coordinate frames

- Gazebo world ENU: x east, y north, z up. AUT aperture in the x = 0 plane, centre
  at height 4 m, facing +x. Drone spawns at (6, 0) facing the AUT (yaw 180°).
- Model body: x forward (towards the probe), y left, z up.
- Motor numbering follows ArduPilot Hexa-X. It differs from Figure 3 of the report;
  spin directions are the same.
