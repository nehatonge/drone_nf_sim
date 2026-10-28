# Running the simulation

Step-by-step guide to set up, run and check the drone near-field simulation, and what
to do when something goes wrong. For what the simulation contains, see the
[README](../README.md).

Everything runs inside **WSL (Ubuntu 24.04)**. On Windows, open a terminal and type
`wsl` first. Git, Python and Gazebo are installed inside WSL, not on Windows.

---

## 1. One-time setup

Skip this if `gz`, `~/ardupilot` and `~/gz_ws/src/ardupilot_gazebo` already exist.

```bash
# get the repo (or work in /mnt/c/Users/<you>/drone_nf_sim if it is already there)
git clone git@github.com:nehatonge/drone_nf_sim.git
cd drone_nf_sim

# ArduPilot and the Gazebo plugin sources
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && git checkout Copter-4.6.3 && git submodule update --init --recursive && cd -
mkdir -p ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/gz_ws/src/ardupilot_gazebo

# install and build (20-40 min the first time)
sudo bash setup/01_install_ros2_gazebo.sh     # ROS 2 Jazzy + Gazebo Harmonic
sudo bash setup/02_ardupilot_root.sh          # ArduPilot system packages
bash setup/03_ardupilot_user.sh               # Python venv, SITL build, plugin build, ~/.bashrc
```

`03` adds environment variables to `~/.bashrc`, including the path to this repo's
models (`/mnt/c/Users/nehat/drone_nf_sim/sim/models`). If your copy of the repo is
somewhere else, edit that line in `~/.bashrc`.

Close the terminal and open a new one so `~/.bashrc` is loaded, then check:

```bash
gz sim --versions                     # 8.x
ls ~/ardupilot/build/sitl/bin/arducopter
ls ~/gz_ws/src/ardupilot_gazebo/build/libArduPilotPlugin.so
python3 -c "import pymavlink, numpy; print('python OK')"
```

---

## 2. Check without the simulator (seconds)

```bash
cd /mnt/c/Users/nehat/drone_nf_sim
python3 analysis/sizing_check.py      # mass, thrust, CG, endurance, tolerances
python3 sim/em_selftest.py            # EM model vs analytical pattern: must print PASS
```

---

## 3. Run the simulation

There are three ways, from simplest to most hands-on.

### A. Automatic test run (no window unless `--gui`)

Starts everything, flies, analyses, stops. Best for repeatable tests. Gazebo runs
without a window; add `--gui` before `--` to see it
(`./sim/test_run.sh --gui --model nf_hexa_down -- grid --max-points 10`), or attach a
window to a running test from another terminal with `gz sim -g`.

```bash
cd /mnt/c/Users/nehat/drone_nf_sim
# planar near-field scan of the 5 GHz AUT (camera alignment + VNA trigger handshake)
./sim/test_run.sh --model nf_hexa_trimmed -- nfscan --max-points 10   # 10 trigger points, ~5 min
./sim/test_run.sh --model nf_hexa_trimmed -- nfscan                   # all 441 points, ~1.5 h
./sim/test_run.sh --model nf_hexa_trimmed -- nfscan --resume 2        # continue scan 2

# another antenna frequency: take-off point and sweep follow it
./sim/test_run.sh --model nf_hexa_trimmed --aut-ghz 3 -- nfscan --aut-ghz 3 --max-points 10

# path-following grid with the downward probe (earlier test)
./sim/test_run.sh --model nf_hexa_down -- grid --max-points 10
./sim/test_run.sh --model nf_hexa_trimmed -- hold --hold 30        # hover stability only
```

Everything after `--` goes to `flight_test.py`; see `python3 sim/flight_test.py --help`.

### B. Watch the grid flight in Gazebo (two terminals)

**Terminal 1** — simulator with the Gazebo window:

```bash
cd /mnt/c/Users/nehat/drone_nf_sim
./sim/run_sim.sh --batch --model nf_hexa_trimmed            # near-field scan (probe forward)
# or: ./sim/run_sim.sh --batch --model nf_hexa_down         # path-following grid (probe down)
```

Wait until the Gazebo window shows the lab (10-20 s). To find the drone: in the Entity
Tree on the right, right-click `nf_hexa_down` → **Move to**.

**Terminal 2** — the grid script (plays the Jetson):

```bash
cd /mnt/c/Users/nehat/drone_nf_sim
source ~/venv-ardupilot/bin/activate
NF_MODEL=nf_hexa_trimmed python3 sim/flight_test.py nfscan --max-points 10
# or: NF_MODEL=nf_hexa_down python3 sim/flight_test.py grid --max-points 10
```

`NF_MODEL` must match `--model` in terminal 1. When finished, in either terminal:

```bash
./sim/stop_sim.sh
```

### C. Fly by hand (MAVProxy console)

```bash
cd /mnt/c/Users/nehat/drone_nf_sim
./sim/run_sim.sh --model nf_hexa_down
```

Wait ~20 s for `EKF3 IMU0 is using external nav data`, then type at the `MAV>` prompt
(in the terminal, not in the Console window):

```
mode guided
arm throttle          # repeat after 10 s if it says "Gyros inconsistent"
takeoff 4
mode land
```

---

## 4. Look at the results

| What | Where |
|---|---|
| Scans, points, triggers, events, telemetry (SQLite) | `data/nf_scans.db` — Windows: `C:\Users\nehat\drone_nf_sim\data\nf_scans.db`, open with [DB Browser for SQLite](https://sqlitebrowser.org/) |
| Summary | `python3 sim/scan_db.py` (scans and flights), `python3 sim/scan_db.py 1` (points of scan 1), `python3 sim/scan_db.py triggers 1` (every trigger attempt of flight 1) |
| All log files of a flight | `data/flights/<flight_id>/` (ArduPilot `.BIN`, CSV, plot, UWB / Gazebo / console logs) |
| Camera image check | `cd sim && NF_MODEL=nf_hexa_trimmed python3 marker_detect.py` (while a sim runs) → `~/nf_runs/camera.png` |
| Flight CSV + plot | `~/nf_runs/` — Windows: `\\wsl$\Ubuntu-24.04\home\neha\nf_runs` |
| UWB fix error | `~/nf_runs/uwb.log` (printed every 10 s) |
| Gazebo log | `~/nf_runs/gz.log` |
| SITL / script console | `~/nf_runs/sim_console.log` |
| ArduPilot flight logs | `~/nf_runs/logs/*.BIN` (open with Mission Planner or `mavlogdump.py`) |

Example SQL (DB Browser → Execute SQL):

```sql
-- centre-frequency amplitude and phase of every point in scan 1
SELECT p.idx, p.target_x, p.target_y, m.amp_db, m.phase_deg
FROM points p JOIN measurements m ON m.point_id = p.id
WHERE p.scan_id = 1 AND m.freq_hz = 5.0e9
ORDER BY p.idx;
```

---

## 5. Rebuild and regenerate

Changing Python scripts or `params/nf_hexa.parm` needs **nothing** — they are read at
every start (`run_sim.sh` wipes old SITL parameters with `-w`).

| If you changed… | Run |
|---|---|
| Lab layout (`sim/catr_layout.py`) | `python3 sim/gen_catr_lab.py` |
| Drone masses / geometry (`sim/gen_nf_hexa.py`) | `python3 sim/gen_nf_hexa.py --name nf_hexa_down --battery-x -0.24 --probe down` (and the other models, see README) |
| ArduPilot source | `cd ~/ardupilot && ./waf configure --board sitl && ./waf copter -j$(nproc)` |
| `ardupilot_gazebo` source | `export GZ_VERSION=harmonic; cd ~/gz_ws/src/ardupilot_gazebo/build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j$(nproc)` |

After a rebuild, a quick check that keeps the real database clean:

```bash
./sim/test_run.sh --model nf_hexa_down -- grid --max-points 3 --db /tmp/test.db
```

---

## 6. Troubleshooting

Start with `./sim/stop_sim.sh` — most odd behaviour comes from a previous run still
going in the background.

### The Gazebo window does not appear
- `--headless` and `test_run.sh` run Gazebo without a window on purpose; use
  `./sim/test_run.sh --gui ...` to get one.
- Attach a window to a running simulation from another terminal: `gz sim -g`
- Check `~/nf_runs/gz.log` for errors.
- Graphics: `~/.bashrc` sets `GALLIUM_DRIVER=d3d12` and
  `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` for WSLg on an NVIDIA GPU. On a machine
  without an NVIDIA GPU, remove the second line. Test with `glxinfo -B | grep renderer`.
- Lab or drone missing in a `gz sim -g` window: that terminal lacks the models path.
  `echo $GZ_SIM_RESOURCE_PATH` must contain `drone_nf_sim/sim/models` (set by
  `~/.bashrc`; `run_sim.sh` sets it for its own window).

### An "ArduCopter" window opens and looks stuck
Normal. It is the SITL flight controller; it prints its start-up and then stays quiet.
Do not type in it or close it during a run. `xterm: cannot load font` warnings are
harmless.

### The drone sits on the pad and does nothing
- In mode C it waits for your commands (`mode guided`, `arm throttle`, `takeoff 4`).
- In mode B, start `flight_test.py` in a second terminal.

### "Arm: Gyros inconsistent"
Normal for the first ~10 s: ArduPilot wants both IMUs to agree for 10 s from the first
check. `flight_test.py` retries automatically; by hand, wait and `arm throttle` again.

### "Arm: Rangefinder 1: No Data"
The drone model has no rangefinder or the plugin is not getting it.
- Regenerate the models (section 5); the lidar is added by `gen_nf_hexa.py`.
- `gz topic -l | grep rangefinder` must show `/nf/rangefinder` while the sim runs.
- Check that `ardupilot_gazebo` is built (section 1).

### "EKF never started using external nav"
UWB positioning is not reaching ArduPilot.
- `cat ~/nf_runs/uwb.log` — it should print a fix error every 10 s.
- `uwb: ... no pose` or nothing at all: `NF_MODEL` does not match the model in Gazebo.
- `run_sim.sh` starts `uwb_extnav.py` itself; if you start SITL another way, start it too:
  `NF_MODEL=nf_hexa_down python3 sim/uwb_extnav.py`

### "no pose for model 'X' on /world/catr_lab/pose/info"
`NF_MODEL` (terminal 2) differs from `--model` (terminal 1), or Gazebo is not running.

### "SITL not reachable on tcp:127.0.0.1:5760" / heartbeat timeout
- SITL is not running yet (wait longer) or was started without `--batch`. Without
  `--batch`, MAVProxy uses port 5760; then use `--url udp:127.0.0.1:14550`.
- A previous run still holds the port: `./sim/stop_sim.sh`, then start again.

### The drone oscillates, tilts a lot or crashes when it moves
- Make sure `params/nf_hexa.parm` is loaded: the start-up line
  `Adding parameters from (.../nf_hexa.parm)` must appear.
- The pitch gains are scaled to the boom's pitch inertia. If you change the probe, boom
  or battery position, the gains may need adjusting (`ATC_RAT_PIT_*`).
- Look at desired vs actual attitude in the newest `~/nf_runs/logs/*.BIN` (`ATT`, `RATE`).

### "AUT marker not found"
The camera never saw the ArUco marker, even after turning a full circle.
- Use the nf_hexa models generated after the camera was added (`gz topic -l` must show
  `/nf/camera`); regenerate them if not (section 5).
- Regenerate the lab (`python3 sim/gen_catr_lab.py`): it writes the marker texture
  `sim/models/catr_lab/materials/textures/aut_marker.png`.
- Look at what the camera sees: `marker_detect.py` saves `~/nf_runs/camera.png`.
- `--search-alt` sets the height the drone looks from (default 3.5 m).

### "point N failed 5 times - scan M stopped"
Five triggers in a row failed at one point. `python3 sim/scan_db.py triggers <flight_id>`
shows why (`not_stable`, `missed_trigger`, `sweep_timeout`, `moved_during_sweep`,
`bad_data`, `vna_not_ready`). In simulation the VNA faults come from
`--vna-miss-prob`, `--vna-hang-prob`, `--vna-bad-prob` (set them to 0 for a fault-free
run); many `not_stable` means the drone cannot hold 2 cm (`--trigger-tol`). Continue with
`--resume M`.

### "vehicle disarmed during ... (crash?) - scan N stopped"
The grid script stopped so no point is saved from the floor. Fix the cause, then
continue with `--resume N`; saved points are kept.

### Points are "NOT arrived"
The drone's own estimate never stayed within `--arrive-tol` (5 cm) for 1 s within
`--arrive-timeout` (20 s). The point is still measured and saved with `arrived = 0`.
Check `~/nf_runs/uwb.log` (fix error) and try a slower `--step-speed`.

### "database is locked"
Close DB Browser for SQLite (or anything else reading `nf_scans.db`) while a scan runs.
SQLite on the Windows drive does not share a file well between two programs.

### "ModuleNotFoundError: No module named 'pymavlink'"
Activate the venv: `source ~/venv-ardupilot/bin/activate` (already in `~/.bashrc` after
setup `03`).

### `git` not found on Windows
Use Git inside WSL, or install Git for Windows, or open the folder in VS Code through
the WSL extension.

### Simulation runs slowly
Check the real-time factor in the Gazebo window (bottom right). Close other heavy
programs; run headless (`test_run.sh`) for long scans.
