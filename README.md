# drone_nf_sim

Simulation of a drone-based near-field antenna measurement system inside a compact
antenna test range (CATR) lab. A heavy-lift hexacopter carries a waveguide probe
(WGP) through a planar raster in front of an antenna under test (AUT) in the quiet
zone; the CATR's own far-field measurement of the same AUT is the reference. The goal
is to find out, before any hardware is built, how accurately the drone can hold and
know the probe position indoors, and what that does to the far-field pattern.

Companion to the report *Drone-Based Near-Field Antenna Measurement System:
Architecture Report* (Oct 2026). Section and figure numbers below refer to it. The
report describes an outdoor RTK-GNSS setup; this simulation moves it indoors, where
UWB replaces RTK-GNSS.

**Stack:** Ubuntu 24.04 (WSL2) · ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8) ·
ArduPilot Copter 4.6 SITL · `ardupilot_gazebo` plugin · Python 3

## The CATR lab

25 × 12 × 10 m interior (to the absorber tips), lined with pyramidal absorber on the
walls and ceiling and on the floor between the reflector and the scan plane. Layout in
[sim/catr_layout.py](sim/catr_layout.py); every value marked EST there is a placeholder
until the real lab drawing is available.

| Item | In the world |
|---|---|
| Reflector | 6 × 6 m panel at the −x end, tilted towards the quiet zone (serrations not modelled) |
| Feed | horn on an absorber-covered tower at (−5, 0, 1.8 m) aimed at the reflector |
| Quiet zone | 3 m sphere centred on the AUT (transparent green) |
| AUT | 1 × 1 m aperture at height 4 m in the x = 0 plane, on a roll-over-azimuth positioner, turned to face +x for the drone scan (it faces the reflector for the CATR reference measurement) |
| UWB anchors | 8 on poles just inside the absorber, 4 at 1.2 m and 4 at 8.5 m |
| Take-off pad | flat floor on the AUT boresight, placed from the antenna frequency (x = 5.03 m at 5 GHz), drone facing the AUT |
| AUT marker | 0.5 m ArUco marker (DICT_4X4_50, id 0) on a plate 0.9 m above the aperture centre |

Absorber is visual geometry: Gazebo does not simulate RF. Flat collision boxes at the
absorber tips stop a drone that drifts into a wall.

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
| 7 | Jetson Orin Nano | Played by `flight_test.py`: commands GUIDED setpoints over MAVLink, finds the AUT with the camera, runs the VNA trigger handshake, logs everything to SQLite. Not yet a ROS 2 node. | `sim/flight_test.py` |
| — | Forward camera | 640 × 480, 69° FOV, 10 Hz at the front of the frame (`/nf/camera`); ArUco marker detection and pose with OpenCV | `sim/gen_nf_hexa.py`, `sim/marker_detect.py` |
| 8 | RTK-GNSS + compass | **Indoors: UWB tag on the mast** + compass. `uwb_extnav.py` turns true tag position into noisy anchor ranges, solves the position and sends it as external navigation (`EK3_SRC1_POSXY/POSZ 6`). GPS off. | `sim/uwb_extnav.py`, `params/nf_hexa.parm` |
| 9 | Telemetry radio | MAVLink over UDP 14550 (MAVProxy) / TCP 5760 (scripts); UWB on TCP 5762 | `sim/run_sim.sh` |
| 10 | Rangefinder (down) | Single-beam Gazebo lidar → ArduPilotPlugin → `RNGFND1_TYPE 100`. Logged, not used for height: over pyramidal floor absorber it reads tips and valleys. | `sim/gen_nf_hexa.py` |
| 11 | HereFlow optical flow | SITL optical flow (`FLOW_TYPE 10`), fused for velocity (`EK3_SRC1_VELXY 5`) | `params/nf_hexa.parm` |
| 12 | VNA / RF receiver | Simulated LibreVNA-class VNA with TRIGGER / READY handshake lines; each sweep measures 21 frequencies one after another at the probe pose of that instant (EM model); injectable faults | `sim/vna_sim.py`, `sim/em_model.py` |
| 13 | Probe mount | Mass only (0.20 kg) | `sim/gen_nf_hexa.py` |
| 14 | Probe boom + coax | `boom` link, GFRP tube 0.10–1.20 m forward, 0.45 kg | `sim/gen_nf_hexa.py` |
| 15 | Waveguide probe | `probe` link at the boom tip, 0.8 kg. Forward-facing (`nf_hexa`, `nf_hexa_trimmed`) or downward-facing (`nf_hexa_down`). RF: open-ended WR-187 (3.95–5.85 GHz) | `sim/gen_nf_hexa.py`, `sim/em_model.py` |
| G | AUT + RF source | AUT on the CATR positioner in the quiet zone, ArUco marker plate above the aperture | `sim/gen_catr_lab.py` |
| G | CATR reflector + feed | Reference far-field measurement of the AUT | `sim/gen_catr_lab.py` |
| G | UWB anchors (replace RTK base) | 8 anchors, positions in `catr_layout.py` | `sim/gen_catr_lab.py` |
| G | 10 MHz reference | Not modelled | — |
| — | Air drafts | Gazebo WindEffects, set per run (`--wind`, `--gust`) | `sim/run_sim.sh` |

Masses marked `EST` in the code are estimates. Replace them with datasheet values
when the parts are chosen, then regenerate the model.

## Pipeline status (report Figure 4)

| Step | Description | Status |
|---|---|---|
| 1 | Scenario setup (band, AUT, scan plane, error budget) | `analysis/sizing_check.py`, CLI args |
| 2 | Scan path planner | `analysis/scan_path.py`, `raster_traj()` in `flight_test.py` |
| 3 | Drone simulation, flight controller in the loop | Gazebo + ArduPilot SITL + UWB (ROS 2 bridge not used yet) |
| 4 | Pose + data log | CSV of true probe pose from Gazebo; grid scans in SQLite (not rosbag yet) |
| 5 | EM near-field model at logged poses | `sim/em_model.py`: 5 GHz 1 × 1 m AUT, WR-187 probe; checked by `sim/em_selftest.py` |
| 6 | NF-FF transformation (probe correction, plane-wave spectrum) | To do |
| 7 | Far-field pattern + error metrics | Position/phase-error verdict per band in `analyze_flight.py`; comparison with the CATR reference pattern to do |

## Planar near-field scan (`nfscan`)

The measurement mission. `flight_test.py nfscan`:

1. **Take-off point from the antenna frequency** (`catr_layout.takeoff_point`): straight
   behind the scan position on the AUT boresight (scan distance n·λ + probe offset), but
   never closer than x = 5.03 m, where the probe still hangs over the flat floor. At
   5 GHz: scan distance 5λ = 30 cm, take-off at x = 5.03 m. `run_sim.sh` places the drone
   there (`--aut-ghz`, `--n-lambda`, or `--spawn "X Y"`).
2. **Find the AUT** with the forward camera: ArUco marker above the aperture, pose by
   `solvePnP`, converted to the world with the drone's own pose estimate. If the marker is
   not in view, the drone turns in 30° steps.
3. **Align**: fly to a stand-off point 1 m beyond the scan plane, square to the AUT at its
   centre height; look again from there (more accurate), then approach the scan plane and
   **lock the height**. In simulation the AUT is found to ~11 mm and < 0.1°.
4. **Serpentine 2 × 2 m grid** on the plane parallel to the aperture, trigger points every
   10 cm (21 × 21 = 441): from the grid centre to the **bottom-right** corner (seen from the
   drone), right → left, step up, left → right, … to the top row.
5. **At every trigger point** (closed loop, feedback before and after):
   - before: probe within **2 cm** of the point for **1 s** (drone's own estimate) and the
     VNA idle (READY high)
   - TRIGGER high → VNA acknowledges (READY low) → sweep → READY high → read → TRIGGER low
   - after: acknowledge received, sweep finished in time, probe stayed within 2 cm during
     the sweep, data valid
   - any failure (`not_stable`, `vna_not_ready`, `missed_trigger`, `sweep_timeout`,
     `moved_during_sweep`, `bad_data`): **back to the same point and trigger again**; never
     on to the next point until it succeeds. After 5 failed attempts the scan stops
     (resume with `--resume`).
   - hover the rest of the **10 s**, then save the point.
6. Back to the grid centre, retreat, return to the pad, land.

About 441 × ~12.5 s ≈ 1.5 h for the full grid. 10 cm is 1.7 λ at 5 GHz: right for
testing path and trigger logic; a far-field reconstruction needs ≤ λ/2 = 3 cm
(`--nf-spacing 0.03`).

### VNA: LibreVNA (simulated)

Chosen for the drone: 100 kHz – 6 GHz (covers the 5 GHz AUT and L/S/C), small and
USB-powered from the Jetson, 10 MHz reference in/out (shared-reference phase coherence,
report section 4), two ports (port 2 can be the reference channel), open source with SCPI
control, low cost. Trade-offs: lower dynamic range than a benchtop VNA, and the trigger
handshake goes over USB/SCPI; if a hard 5 V TTL trigger line is required, a Copper Mountain
compact USB VNA is the step up. `vna_sim.py` models generic TRIGGER / READY lines, so the
flight logic works with either. Check the datasheet figures before buying.

On the real drone, the Jetson Orin Nano's GPIO is **3.3 V and not 5 V tolerant**: use a
level shifter for any 5 V TTL trigger lines.

## Grid scan (stop-and-go)

`flight_test.py grid` flies a horizontal serpentine grid at the AUT's height (4 m),
from the lab corner (probe at x 10.25, y −4.0) to just in front of the AUT
(x ≈ 1.1, y 0), 15 cm spacing: 62 × 27 = 1674 points. At each point:

1. move there at 0.2 m/s
2. wait until the drone's **own** estimate (UWB + optical flow) is within 5 cm for 1 s
   (20 s timeout; the point is still measured and flagged `arrived = 0`)
3. hover 10 s; log true and estimated probe position; take a VNA sweep every second
   at the true probe pose (`em_model.py`)
4. save the point and its averaged sweep to SQLite, then continue

The full grid takes about 7 h, far more than one battery, so every point is committed
on its own and `--resume <scan_id>` continues where a scan stopped. The script stops if
the drone disarms, so no point is saved from the floor.

**Spacing:** 15 cm is fine for testing path following, but it is 2.5 λ at 5 GHz. A
near-field scan for NF-FF needs ≤ λ/2 = 3 cm (`--spacing 0.03`).

The database is `data/nf_scans.db` (Windows: `C:\Users\nehat\drone_nf_sim\data\nf_scans.db`,
open with DB Browser for SQLite):

| Table | One row per | Main columns |
|---|---|---|
| `scans` | scan | start time, model, probe, frequency sweep, spacing, hold time, grid and settings (JSON) |
| `points` | grid point (once measured) | target, estimated (EKF) and true (Gazebo) probe position with std over the hold, grid offsets, attempts, flight |
| `measurements` | point × frequency | S21 (re, im), amplitude dB, phase deg (and std over sweeps for `grid`) |
| `flights` | flight | test, take-off point, AUT detection and its error vs truth, outcome, trigger counts, log folder |
| `triggers` | trigger attempt | handshake times (trigger, ack, done), position error before / during the sweep, result |
| `events` | event | take-off, detection, alignment, height lock, retries, landing, errors |
| `telemetry` | 0.1 s of flight | true and reference probe position, attitude, phase of the mission |
| `files` | archived log | ArduPilot `.BIN`, flight CSV and plot, UWB / Gazebo / console logs in `data/flights/<flight_id>/` |

`python3 sim/scan_db.py` lists scans and flights; `python3 sim/scan_db.py triggers <flight_id>`
lists every trigger attempt.

Amplitudes are relative to the probe facing the AUT on boresight at 1 m (0 dB).

## Known issues

- **Gains are inertia-scaled, not auto-tuned.** The boom makes pitch inertia ~3× roll;
  with equal gains the pitch loop oscillated and the drone crashed on any horizontal
  move. Pitch rate gains are now ~3× roll (`params/nf_hexa.parm`), verified on the grid
  transits (max 3–6° tilt). `hover`/`scan` with the forward probe are not re-tested yet.
  Real hardware will need its own AUTOTUNE.
- **UWB is too coarse to know the probe position for phase.** ~30 mm fix error is
  ≈ 108° at S-band against a λ/50 (7.2°) criterion. The drone can fly on UWB, but the
  near-field data needs a more accurate position reference (motion capture or laser
  tracker) logged alongside.
- The first arm attempt always reports "Gyros inconsistent": ArduPilot wants the two
  IMUs to agree for 10 s from the first check. `flight_test.py` retries.

## Layout

```
setup/      install scripts for WSL Ubuntu 24.04 (run in order 01, 02, 03)
sim/
  catr_layout.py       CATR lab dimensions, reflector, feed, AUT, UWB anchors (edit here)
  em_model.py          EM model: AUT aperture -> probe S21 over a frequency sweep
  em_selftest.py       checks the EM model against the analytical far-field pattern
  scan_db.py           SQLite store: scans, points, triggers, events, telemetry, flight logs
  vna_sim.py           simulated VNA with TRIGGER / READY handshake and fault injection
  marker_detect.py     camera + ArUco: finds the AUT centre and facing direction
  gen_catr_lab.py      generates the lab model (absorber meshes) and worlds/catr_lab.sdf
  gen_nf_hexa.py       generates the hexacopter SDF from the component list
  models/              generated: catr_lab, nf_hexa (as built), nf_hexa_trimmed (battery 24 cm aft),
                       nf_hexa_down (trimmed, probe facing down)
  worlds/catr_lab.sdf  generated world (drone filled in by run_sim.sh)
  params/              ArduCopter parameter files
  uwb_extnav.py        UWB positioning -> ArduPilot external navigation
  run_sim.sh           start Gazebo + UWB + SITL
  flight_test.py       hold / hover / scan flights, logs true probe pose
  test_run.sh          one complete repeatable run: sim -> flight -> analysis -> stop
  stop_sim.sh          kill everything
analysis/
  sizing_check.py      mass, thrust, CG, prop clearance, endurance, tolerance, scan time
  scan_path.py         planar serpentine raster as JSON
  analyze_flight.py    tracking/knowledge error and phase error per band, plots
```

**Step-by-step run guide and troubleshooting: [docs/RUNNING.md](docs/RUNNING.md).**

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

# regenerate after changing the lab layout or the drone
python3 sim/gen_catr_lab.py
python3 sim/gen_nf_hexa.py
python3 sim/gen_nf_hexa.py --name nf_hexa_trimmed --battery-x -0.24
python3 sim/gen_nf_hexa.py --name nf_hexa_down --battery-x -0.24 --probe down

python3 sim/em_selftest.py                          # EM model check, no simulator

# planar near-field scan of the 5 GHz AUT: short test first, then the full 441 points
./sim/test_run.sh --model nf_hexa_trimmed -- nfscan --max-points 10
./sim/test_run.sh --model nf_hexa_trimmed -- nfscan
./sim/test_run.sh --model nf_hexa_trimmed -- nfscan --resume 2       # continue scan 2
python3 sim/scan_db.py triggers 1                   # trigger attempts of flight 1

# grid scan: a short test first, then the full grid, resumable
./sim/test_run.sh --model nf_hexa_down -- grid --max-points 10
./sim/test_run.sh --model nf_hexa_down -- grid --corner 10.25 -4.0 --end 9.65 -3.70   # 5 x 3 points
./sim/test_run.sh --model nf_hexa_down -- grid --resume 1
python3 sim/scan_db.py                              # list scans
python3 sim/scan_db.py 1                            # points of scan 1

# interactive: Gazebo GUI + MAVProxy console
./sim/run_sim.sh --model nf_hexa_trimmed
#   wait ~20 s for "EKF3 IMU0 is using external nav data", then at the MAV> prompt:
#   mode guided / arm throttle / takeoff 4   (repeat arm after 10 s if it says "Gyros inconsistent")

# one complete batch run (headless)
./sim/test_run.sh --model nf_hexa_trimmed -- hold --hold 30
./sim/test_run.sh --model nf_hexa_trimmed -- scan --freq-ghz 3
./sim/test_run.sh --model nf_hexa_trimmed --uwb "--range-sigma 0.10 --nlos-prob 0.05" -- hold
./sim/test_run.sh --model nf_hexa_trimmed --wind 1 --gust 0.5 -- hold     # air drafts
```

Results (CSV + PNG) go to `~/nf_runs/` (from Windows: `\\wsl$\Ubuntu-24.04\home\neha\nf_runs`).
`~/nf_runs/uwb.log` reports the UWB fix error every 10 s.

## Coordinate frames

- Gazebo world ENU: x along the range (reflector at −x), y across, z up. Origin on the
  floor below the AUT aperture centre; aperture in the x = 0 plane facing +x, centre at
  height 4 m. The drone spawns on the AUT boresight facing it (yaw 180°), at the take-off
  point computed from the antenna frequency (x = 5.03 m at 5 GHz) or `--spawn "X Y"`.
- Scan grid offsets: u to the drone's right, w up, on the plane parallel to the aperture.
- ArduPilot's local NED origin is the world origin; `flight_test.py` measures it before
  take-off instead of assuming it.
- Model body: x forward (towards the probe), y left, z up.
- Motor numbering follows ArduPilot Hexa-X. It differs from Figure 3 of the report;
  spin directions are the same.
