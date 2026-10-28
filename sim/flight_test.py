#!/usr/bin/env python3
"""
Flight tests for the near-field hexacopter in Gazebo + ArduPilot SITL.

  python3 flight_test.py hold  [--alt 4] [--hold 40]        take off, hover in place (stability)
  python3 flight_test.py hover [--alt 4] [--hold 40]        hover with the probe on the scan plane
  python3 flight_test.py scan  [--freq-ghz 3] [--width 3] [--height 3] [--speed 0.5]
  python3 flight_test.py grid  [--spacing 0.15] [--hold-time 10] [--max-points N] [--resume ID]
        stop-and-go grid from the lab corner to the AUT: at each point arrive, hover
        --hold-time s, measure S21 over the sweep with em_model.py, save to SQLite (scan_db.py)
  python3 flight_test.py nfscan [--aut-ghz 5] [--nf-size 2] [--nf-spacing 0.1] [--max-points N] [--resume ID]
        planar near-field scan: take off, find the AUT with the camera (ArUco marker), align
        parallel to it at n_lambda wavelengths and lock height, then a 2 x 2 m serpentine
        from the bottom-right corner upwards; at every 10 cm trigger point: stable within
        2 cm for 1 s -> VNA trigger handshake (vna_sim.py) -> check -> hover 10 s; any
        failure: back to the same point and trigger again. Everything goes to SQLite.

The script commands the vehicle through MAVLink (GUIDED mode, like a GCS / the
Jetson would) and records the TRUE probe pose from Gazebo (/world/<w>/pose/info)
for the whole flight, so the result is the real probe tracking error,
independent of the EKF. The AUT is approached in two stages (stand-off point
first, then a slow move in) so an overshoot cannot hit it.

Frames: Gazebo world ENU (x east, y north, z up). The AUT aperture is the x = 0
plane, the drone faces -x (yaw 180 deg ENU). The probe sits at PROBE_OFFSET in
the body frame (x forward, y left, z up).
"""
import argparse
import csv
import glob
import json
import math
import os
import subprocess
import threading
import time

import numpy as np
from pymavlink import mavutil

WORLD = "catr_lab"
MODEL = os.environ.get("NF_MODEL", "nf_hexa")
# Probe aperture centre, boresight and E-plane direction in the body frame (x fwd, y left,
# z up); must match gen_nf_hexa.py. Selected in main() with --probe.
PROBES = {
    "forward": dict(offset=(1.325, 0.0, -0.15), normal=(1, 0, 0), e=(0, 1, 0)),   # aperture face at the boom tip
    "down": dict(offset=(1.25, 0.0, -0.30), normal=(0, 0, -1), e=(0, 1, 0)),
}
PROBE = PROBES["forward"]
PROBE_OFFSET = PROBE["offset"]
HOME_ENU = (6.0, 0.0, 0.40)        # model origin on the pad; replaced by the true spawn pose in main()
YAW_ENU = math.pi                  # facing the AUT; set_yaw() after the camera alignment
YAW_NED = math.pi / 2 - YAW_ENU    # NED heading (rad): -90 deg = west


def set_yaw(yaw_enu):
    global YAW_ENU, YAW_NED
    YAW_ENU = yaw_enu
    YAW_NED = math.pi / 2 - yaw_enu
STANDOFF_X = 3.0                   # probe x of the safe stand-off point
APPROACH_SPEED = 0.3               # m/s for the final move towards the AUT
OUT_DIR = os.path.expanduser("~/nf_runs")


# ----------------------------------------------------------------- geometry
def quat_rot(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def quat_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def body_for_probe(p):
    """Model-origin ENU position that puts the probe at p (level, yaw = YAW_ENU)."""
    c, s = math.cos(YAW_ENU), math.sin(YAW_ENU)
    ox, oy, oz = PROBE_OFFSET
    return (p[0] - (c * ox - s * oy), p[1] - (s * ox + c * oy), p[2] - oz)


def probe_for_body(b):
    c, s = math.cos(YAW_ENU), math.sin(YAW_ENU)
    ox, oy, oz = PROBE_OFFSET
    return (b[0] + (c * ox - s * oy), b[1] + (s * ox + c * oy), b[2] + oz)


# World ENU position of ArduPilot's local NED origin (EKF origin). In SITL this is the
# Gazebo world origin, not the spawn point; measured on the ground by calibrate_origin().
local_origin_enu = list(HOME_ENU)


def enu_to_local_ned(p):
    """ENU world -> ArduPilot local NED relative to the EKF origin."""
    o = local_origin_enu
    return (p[1] - o[1], p[0] - o[0], -(p[2] - o[2]))


# ----------------------------------------------------------------- truth
class Truth:
    """Streams the model pose from Gazebo via `gz topic` (JSON)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.latest = None          # (t, pos, quat)
        self.proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "--json-output"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        for line in self.proc.stdout:
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            st = msg.get("header", {}).get("stamp", {})
            t = int(st.get("sec", 0)) + int(st.get("nsec", 0)) * 1e-9
            for p in msg.get("pose", []):
                if p.get("name") == MODEL:
                    pos = p.get("position", {})
                    o = p.get("orientation", {})
                    pos = (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0))
                    q = (o.get("w", 1.0), o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0))
                    with self.cond:
                        self.latest = (t, pos, q)
                        self.cond.notify_all()
                    break

    def probe(self):
        """(sim_t, probe_pos, quat, body_pos) or None."""
        with self.lock:
            s = self.latest
        if s is None:
            return None
        t, pos, q = s
        r = quat_rot(q, PROBE_OFFSET)
        return t, (pos[0] + r[0], pos[1] + r[1], pos[2] + r[2]), q, pos

    def wait_new(self, last_t, timeout=1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.latest is not None and self.latest[0] != last_t, timeout)
        return self.probe()

    def now(self):
        s = self.probe()
        return s[0] if s else 0.0

    def close(self):
        self.proc.terminate()


# ----------------------------------------------------------------- recorder
class Recorder:
    """Logs every truth sample of the whole flight with the current phase and reference."""

    def __init__(self, truth, name):
        os.makedirs(OUT_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(OUT_DIR, f"{name}_{MODEL}_{stamp}.csv")
        self.truth = truth
        self.phase, self.ref = "ground", truth.probe()[1]
        self.running = True
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def set(self, phase, ref):
        self.phase, self.ref = phase, ref

    def _run(self):
        with open(self.path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "phase", "probe_x", "probe_y", "probe_z", "ref_x", "ref_y", "ref_z",
                        "body_x", "body_y", "body_z", "roll_deg", "pitch_deg", "yaw_deg"])
            last = None
            while self.running:
                s = self.truth.wait_new(last, 0.5)
                if s is None or s[0] == last:
                    continue
                last = s[0]
                t, probe, q, body = s
                r, p, y = quat_rpy(q)
                ref = self.ref
                w.writerow([f"{t:.4f}", self.phase, *[f"{v:.5f}" for v in probe],
                            *[f"{v:.5f}" for v in ref], *[f"{v:.5f}" for v in body],
                            f"{math.degrees(r):.3f}", f"{math.degrees(p):.3f}", f"{math.degrees(y):.3f}"])

    def close(self):
        self.running = False
        self.th.join(2)
        print("log:", self.path)


# ----------------------------------------------------------------- vehicle
class Vehicle:
    def __init__(self, url):
        print(f"connecting {url} ...")
        for attempt in range(30):
            try:
                self.m = mavutil.mavlink_connection(url, source_system=250, retries=1)
                break
            except ConnectionRefusedError:
                time.sleep(2)
        else:
            raise RuntimeError(f"SITL not reachable on {url}")
        self.m.wait_heartbeat(timeout=60)
        print(f"heartbeat from sys {self.m.target_system}")
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.armed = False
        self.mode_name = ""
        self.local_ned = None
        self.att = None
        self.texts = []
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        """Keep reading MAVLink so the socket never backs up; print FC messages."""
        while True:
            msg = self.m.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "STATUSTEXT":
                if "Rangefinder" not in msg.text:
                    print("  FC:", msg.text)
                self.texts.append(msg.text)
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode_name = mavutil.mode_string_v10(msg)
            elif t == "LOCAL_POSITION_NED":
                self.local_ned = (msg.x, msg.y, msg.z)
            elif t == "ATTITUDE":
                self.att = (msg.roll, msg.pitch, msg.yaw)

    def wait_ready(self, timeout=180):
        print("waiting for EKF / UWB position ...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any("using external nav" in t for t in self.texts):
                time.sleep(5)
                return
            time.sleep(0.2)
        raise RuntimeError("EKF never started using external nav (is uwb_extnav.py running? see ~/nf_runs/uwb.log)")

    def calibrate_origin(self, truth, seconds=3.0):
        """Find where ArduPilot's local NED origin sits in the Gazebo world: true body
        position minus the EKF local position, averaged on the ground."""
        acc, t_end = [], time.time() + seconds
        while time.time() < t_end:
            if self.local_ned is not None:
                n, e, d = self.local_ned
                b = truth.probe()[3]
                acc.append((b[0] - e, b[1] - n, b[2] + d))
            time.sleep(0.1)
        if not acc:
            raise RuntimeError("no LOCAL_POSITION_NED from the vehicle")
        for k in range(3):
            local_origin_enu[k] = sum(a[k] for a in acc) / len(acc)
        print("EKF origin in world ENU:", tuple(round(c, 2) for c in local_origin_enu))

    def mode(self, name):
        for _ in range(10):
            self.m.set_mode(self.m.mode_mapping()[name])
            time.sleep(1)
            if self.mode_name == name:
                return
        raise RuntimeError(f"mode {name} not confirmed")

    def arm(self):
        for _ in range(30):
            self.m.arducopter_arm()
            time.sleep(2)
            if self.armed:
                print("armed")
                return
        raise RuntimeError("could not arm")

    def takeoff(self, alt):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

    def goto_ned(self, n, e, d, vn=0.0, ve=0.0, vd=0.0, yaw_ned=None):
        # use position + velocity + yaw; ignore accel (bits 6-8) and yaw rate (bit 11)
        mask = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 11)
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e, d, vn, ve, vd, 0, 0, 0,
            YAW_NED if yaw_ned is None else yaw_ned, 0)

    def goto_probe(self, p, vel=(0, 0, 0)):
        n, e, d = enu_to_local_ned(body_for_probe(p))
        self.goto_ned(n, e, d, vel[1], vel[0], -vel[2])

    def est_body_pose(self):
        """Drone pose as it knows it (EKF: UWB + optical flow): ENU position of the body
        origin and the body (x fwd, y left, z up) -> world ENU rotation matrix."""
        if self.local_ned is None or self.att is None:
            return None
        n, e, d = self.local_ned
        cr, sr = math.cos(self.att[0]), math.sin(self.att[0])
        cp, sp = math.cos(self.att[1]), math.sin(self.att[1])
        cy, sy = math.cos(self.att[2]), math.sin(self.att[2])
        R_ned_frd = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                              [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                              [-sp, cp * sr, cp * cr]])
        T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])      # NED <-> ENU
        F = np.diag([1, -1, -1])                              # FLU <-> FRD
        o = local_origin_enu
        return np.array([o[0] + e, o[1] + n, o[2] - d]), T @ R_ned_frd @ F

    def est_probe(self):
        """Probe position as the drone itself knows it (EKF: UWB + optical flow), world ENU."""
        pose = self.est_body_pose()
        if pose is None:
            return None
        p, R = pose
        return tuple(p + R @ np.array(PROBE_OFFSET))


# ----------------------------------------------------------------- manoeuvres
def hold(v, truth, rec, p, seconds, phase):
    """Hold the probe at p for `seconds` of SIM time."""
    rec.set(phase, p)
    t_end = truth.now() + seconds
    while truth.now() < t_end:
        v.goto_probe(p)
        time.sleep(0.05)


def move(v, truth, rec, p0, p1, speed, phase):
    """Move the probe along a straight line p0 -> p1 at `speed` (sim time)."""
    L = math.dist(p0, p1)
    if L < 1e-3:
        return
    u = [(p1[i] - p0[i]) / L for i in range(3)]
    vel = tuple(speed * c for c in u)
    t0 = truth.now()
    while True:
        s = min((truth.now() - t0) * speed, L)
        p = tuple(p0[i] + u[i] * s for i in range(3))
        rec.set(phase, p)
        v.goto_probe(p, vel if s < L else (0, 0, 0))
        if s >= L:
            return
        time.sleep(0.05)


def take_off(v, truth, rec, probe_alt):
    v.wait_ready()
    v.calibrate_origin(truth)
    v.mode("GUIDED")
    v.arm()
    start = truth.probe()[1]
    body_alt = body_for_probe((0, 0, probe_alt))[2] - HOME_ENU[2]
    target = (start[0], start[1], probe_alt)
    rec.set("takeoff", target)
    v.takeoff(body_alt)
    t_end = truth.now() + 30
    while truth.now() < t_end:
        if truth.probe()[3][2] > HOME_ENU[2] + 0.95 * body_alt:
            break
        time.sleep(0.2)
    hold(v, truth, rec, target, 5, "climb_settle")
    return target


def approach(v, truth, rec, here, target):
    standoff = (max(STANDOFF_X, target[0]), target[1], target[2])
    move(v, truth, rec, here, standoff, 1.0, "to_standoff")
    hold(v, truth, rec, standoff, 5, "standoff")
    move(v, truth, rec, standoff, target, APPROACH_SPEED, "approach")
    return standoff


def leave_and_land(v, truth, rec, here, standoff):
    move(v, truth, rec, here, standoff, APPROACH_SPEED, "retreat")
    rec.set("land", standoff)
    v.mode("LAND")
    t_end = truth.now() + 40
    while truth.now() < t_end and v.armed:
        time.sleep(0.5)


def run_hold(v, truth, rec, a):
    p = take_off(v, truth, rec, a.alt)
    hold(v, truth, rec, p, 5, "settle")
    print(f"holding {a.hold} s")
    hold(v, truth, rec, p, a.hold, "hover")
    rec.set("land", p)
    v.mode("LAND")
    t_end = truth.now() + 40
    while truth.now() < t_end and v.armed:
        time.sleep(0.5)


def run_hover(v, truth, a, rec):
    here = take_off(v, truth, rec, a.alt)
    target = (a.scan_dist, 0.0, a.alt)
    print(f"approaching, probe -> {target}")
    standoff = approach(v, truth, rec, here, target)
    hold(v, truth, rec, target, 8, "settle")
    print(f"hovering on the scan plane {a.hold} s")
    hold(v, truth, rec, target, a.hold, "hover")
    leave_and_land(v, truth, rec, target, standoff)


def raster_traj(a):
    """Probe reference: list of (t, (x,y,z), (vx,vy,vz), phase), lines along y, stepping up in z."""
    lam = 0.299792458 / a.freq_ghz
    dz = a.line_spacing or lam / 2
    n_lines = int(round(a.height / dz)) + 1
    z0 = a.centre_z - a.height / 2
    pts, t, dt = [], 0.0, 0.05
    for i in range(n_lines):
        z = z0 + i * dz
        y0, y1 = (-a.width / 2, a.width / 2) if i % 2 == 0 else (a.width / 2, -a.width / 2)
        sgn = 1 if y1 > y0 else -1
        n = int(abs(y1 - y0) / (a.speed * dt))
        for k in range(n + 1):
            pts.append((t, (a.scan_dist, y0 + sgn * a.speed * k * dt, z), (0, sgn * a.speed, 0), f"line{i}"))
            t += dt
        if i < n_lines - 1:      # step up to the next line, then pause (turn is not sampled)
            tstep = max(1.0, dz / 0.2)
            m = int(tstep / dt)
            for k in range(m + 1):
                pts.append((t, (a.scan_dist, y1, z + dz * k / m), (0, 0, dz / tstep), "turn"))
                t += dt
            for _ in range(int(1.5 / dt)):
                pts.append((t, (a.scan_dist, y1, z + dz), (0, 0, 0), "turn"))
                t += dt
    return pts


def run_scan(v, truth, a, rec):
    traj = raster_traj(a)
    T = traj[-1][0]
    print(f"raster: {sum(1 for p in traj if p[3] != 'turn')} line setpoints, {T / 60:.1f} min")
    here = take_off(v, truth, rec, traj[0][1][2])
    standoff = approach(v, truth, rec, here, traj[0][1])
    hold(v, truth, rec, traj[0][1], 8, "settle")
    t0, idx = truth.now(), 0
    while True:
        tr = truth.now() - t0                 # follow SIM time, robust to real-time factor < 1
        if tr > T:
            break
        while idx + 1 < len(traj) and traj[idx + 1][0] <= tr:
            idx += 1
        _, ref, vel, phase = traj[idx]
        rec.set(phase, ref)
        v.goto_probe(ref, vel)
        if idx % 600 == 0:
            print(f"  t={tr:6.1f}/{T:.0f}s {phase}")
        time.sleep(0.02)
    end = traj[-1][1]
    leave_and_land(v, truth, rec, end, (standoff[0], end[1], end[2]))


# ----------------------------------------------------------------- grid scan
def grid_points(a):
    """Serpentine grid from the lab corner (--corner) to the AUT end (--end): lines along
    x towards the AUT, stepping in y. Returns [(idx, row, col, (x, y, z))]."""
    (cx, cy), (ex, ey) = a.corner, a.end
    s = a.spacing
    nx = int(abs(ex - cx) / s + 1e-9) + 1
    ny = int(abs(ey - cy) / s + 1e-9) + 1
    sx, sy = math.copysign(s, ex - cx), math.copysign(s, ey - cy)
    pts = []
    for r in range(ny):
        for c in (range(nx) if r % 2 == 0 else range(nx - 1, -1, -1)):
            pts.append((len(pts), r, c, (round(cx + c * sx, 4), round(cy + r * sy, 4), a.grid_z)))
    return pts


def wait_arrival(v, truth, rec, p, tol, timeout, settle=1.0):
    """Hold the setpoint until the drone's own position estimate stays within tol of p for
    `settle` s. Returns (arrived, seconds taken)."""
    rec.set("arrive", p)
    t0, t_in = truth.now(), None
    while truth.now() - t0 < timeout:
        v.goto_probe(p)
        e = v.est_probe()
        if e is not None and math.dist(e, p) <= tol:
            t_in = t_in or truth.now()
            if truth.now() - t_in >= settle:
                return True, truth.now() - t0
        else:
            t_in = None
        time.sleep(0.05)
    return False, timeout


def hold_and_measure(v, truth, rec, p, model, seconds, sweep_interval):
    """Hover on p for `seconds` (sim time); log true and estimated probe position at every
    pose update and take one VNA sweep every sweep_interval at the TRUE probe pose."""
    rec.set("hold", p)
    true_pos, est_pos, sweeps = [], [], []
    t0 = truth.now()
    next_sweep, last = t0, None
    while truth.now() < t0 + seconds:
        v.goto_probe(p)
        s = truth.wait_new(last, 0.2)
        if s is None or s[0] == last:
            continue
        last, (t, ppos, q, _) = s[0], s
        true_pos.append(ppos)
        e = v.est_probe()
        if e is not None:
            est_pos.append(e)
        if t >= next_sweep:
            sweeps.append(model.measure(ppos, quat_rot(q, PROBE["normal"]), quat_rot(q, PROBE["e"]))[0])
            next_sweep += sweep_interval
    return np.array(true_pos), np.array(est_pos), np.array(sweeps), t0, truth.now()


def run_grid(v, truth, a, rec):
    from em_model import NearFieldModel, catr_aut, wr187_probe
    from scan_db import ScanDB

    db = ScanDB(a.db)
    if a.resume:
        grid, settings = db.scan(a.resume)
        pts = [(g[0], g[1], g[2], tuple(g[3])) for g in grid]
        for k, val in settings.items():           # keep the original scan's settings
            setattr(a, k, val)
        scan_id = a.resume
    else:
        pts = grid_points(a)
        settings = {k: getattr(a, k) for k in ("spacing", "grid_z", "corner", "end", "hold_time", "arrive_tol",
                                                "arrive_timeout", "step_speed", "transit_speed", "sweep_interval",
                                                "freq_start", "freq_stop", "freq_points", "noise_db")}
        scan_id = db.new_scan(pts, settings, model=MODEL, probe=a.probe,
                              aut="1 x 1 m TE10 aperture at (0,0,4) facing +x, E along y; WR-187 probe",
                              freq_start_hz=a.freq_start * 1e9, freq_stop_hz=a.freq_stop * 1e9,
                              freq_points=a.freq_points, spacing_m=a.spacing, hold_s=a.hold_time)
    done = db.done(scan_id)
    todo = [p for p in pts if p[0] not in done]
    if a.max_points:
        todo = todo[:a.max_points]
    if not todo:
        print(f"scan {scan_id}: all {len(pts)} points already saved")
        return
    est = len(todo) * (a.hold_time + 1.0 + a.spacing / a.step_speed + 1.0) / 60
    print(f"scan {scan_id}: {len(pts)} points, {len(done)} saved, flying {len(todo)} now (~{est:.0f} min) "
          f"-> {db.path}")

    freqs = np.linspace(a.freq_start * 1e9, a.freq_stop * 1e9, a.freq_points)
    model = NearFieldModel(catr_aut(), wr187_probe(), freqs, noise_db=a.noise_db)

    def check_flying(what):
        if not v.armed:
            raise RuntimeError(f"vehicle disarmed during {what} (crash?) - scan {scan_id} stopped, "
                               f"resume with --resume {scan_id}")

    pad = take_off(v, truth, rec, a.grid_z)
    move(v, truth, rec, pad, todo[0][3], a.transit_speed, "transit")
    check_flying("transit")
    prev = todo[0][3]
    for n, (idx, row, col, p) in enumerate(todo):
        move(v, truth, rec, prev, p, a.step_speed, "step")
        arrived, t_arr = wait_arrival(v, truth, rec, p, a.arrive_tol, a.arrive_timeout)
        tp, ep, sw, t_h0, t_h1 = hold_and_measure(v, truth, rec, p, model, a.hold_time, a.sweep_interval)
        check_flying(f"point {idx}")               # never save a point measured on the floor
        mean = sw.mean(axis=0)
        amp = 20 * np.log10(np.abs(sw))
        dphi = np.angle(sw / mean)                    # phase spread around the mean
        point = dict(idx=idx, row=row, col=col,
                     target_x=p[0], target_y=p[1], target_z=p[2],
                     arrived=int(arrived), t_arrive_s=t_arr, t_hold_start=t_h0, t_hold_end=t_h1,
                     n_sweeps=len(sw))
        for name, arr in (("est", ep), ("true", tp)):
            if len(arr):
                for k, axis in enumerate("xyz"):
                    point[f"{name}_{axis}"] = float(arr[:, k].mean())
                    point[f"{name}_std_{axis}"] = float(arr[:, k].std())
        meas = [dict(freq_hz=float(f), s21_re=float(mean[i].real), s21_im=float(mean[i].imag),
                     amp_db=float(20 * np.log10(abs(mean[i]))), phase_deg=float(np.degrees(np.angle(mean[i]))),
                     amp_std_db=float(amp[:, i].std()), phase_std_deg=float(np.degrees(dphi[:, i].std())))
                for i, f in enumerate(freqs)]
        db.save_point(scan_id, point, meas)
        c = len(freqs) // 2
        err = math.dist((point["est_x"], point["est_y"], point["est_z"]),
                        (point["true_x"], point["true_y"], point["true_z"])) if "est_x" in point else math.nan
        print(f"  [{n + 1}/{len(todo)}] point {idx} (row {row}, col {col}) at {p[0]:.2f},{p[1]:.2f}: "
              f"{'arrived' if arrived else 'NOT arrived'} in {t_arr:.1f}s, est-true {err * 1000:.0f} mm, "
              f"S21 {meas[c]['amp_db']:.1f} dB {meas[c]['phase_deg']:.0f} deg @ {freqs[c] / 1e9:.2f} GHz",
              flush=True)
        prev = p

    move(v, truth, rec, prev, pad, a.transit_speed, "return")
    hold(v, truth, rec, pad, 3, "return_settle")
    rec.set("land", pad)
    v.mode("LAND")
    t_end = truth.now() + 40
    while truth.now() < t_end and v.armed:
        time.sleep(0.5)
    print(f"scan {scan_id}: {len(db.done(scan_id))}/{len(pts)} points saved")


# ----------------------------------------------------------------- near-field scan: camera + trigger
class Keeper:
    """Sends the position setpoint at most every 50 ms while a loop polls faster."""

    def __init__(self, v):
        self.v, self.t = v, 0.0

    def __call__(self, p):
        if time.time() - self.t >= 0.05:
            self.v.goto_probe(p)
            self.t = time.time()


def look_for_aut(v, truth, rec, cam, here, seconds, min_dets=8):
    """Hover at `here` and detect the AUT marker on every new camera frame for `seconds`.
    Returns (aperture centre, normal, n detections) as the drone sees them, or None."""
    from marker_detect import marker_to_aperture
    rec.set("detect", here)
    keep = Keeper(v)
    cs, ns, last = [], [], None
    t_end = truth.now() + seconds
    while truth.now() < t_end:
        keep(here)
        d = cam.detect()
        pose = v.est_body_pose()
        if d is not None and pose is not None and d[0] != last:
            last = d[0]
            c, n = marker_to_aperture(pose[1], pose[0], d[1], d[2])
            cs.append(c)
            ns.append(n)
        time.sleep(0.02)
    if len(cs) < min_dets:
        return None
    n = np.median(ns, axis=0)
    return np.median(cs, axis=0), n / np.linalg.norm(n), len(cs)


def nf_grid(size, spacing):
    """Serpentine on the scan plane as in the sketch: from the bottom-right corner (seen
    from the drone) right -> left, step up, left -> right, ... up to the top row.
    Returns [(idx, row, col, u, w)], u = offset to the drone's right, w = offset up (m)."""
    k = int(round(size / 2 / spacing))
    offs = [round(i * spacing, 4) for i in range(-k, k + 1)]
    pts = []
    for r, w in enumerate(offs):
        for c in (range(len(offs) - 1, -1, -1) if r % 2 == 0 else range(len(offs))):
            pts.append((len(pts), r, c, offs[c], w))
    return pts


def grid_world(centre, n, u, w):
    """Grid offsets -> world position on the plane parallel to the aperture."""
    face = -np.asarray(n)                                 # drone looks along -normal
    right = np.cross(face, [0, 0, 1])
    return tuple(np.asarray(centre) + u * right + w * np.array([0, 0, 1.0]))


def trigger_sweep(v, truth, vna, p, tol, ack_timeout, sweep_timeout):
    """One trigger handshake with the VNA while holding p, with feedback before and after:
    TRIGGER HIGH -> READY LOW (ack) -> READY HIGH (done) -> read -> TRIGGER LOW.
    Fails if there is no ack, no done, the probe left the tolerance during the sweep, or
    the data is invalid."""
    keep = Keeper(v)

    def err():
        e = v.est_probe()
        return math.dist(e, p) if e is not None else math.inf

    e0 = err()
    out = dict(t_trigger=truth.now(), t_ack=None, t_done=None, err_before_mm=e0 * 1000, data=None)
    t_wait = truth.now()
    while not vna.ready:                          # feedback before: the VNA must be idle (READY HIGH)
        keep(p)
        if truth.now() - t_wait > ack_timeout + 0.3:
            vna.abort()
            return dict(out, result="vna_not_ready", err_max_during_mm=e0 * 1000)
        time.sleep(0.002)
    out["t_trigger"] = truth.now()
    vna.set_trigger(True)
    while truth.now() - out["t_trigger"] < ack_timeout:
        keep(p)
        if not vna.ready:
            out["t_ack"] = truth.now()
            break
        time.sleep(0.001)
    emax = e0
    if out["t_ack"] is None:
        vna.set_trigger(False)
        return dict(out, result="missed_trigger", err_max_during_mm=emax * 1000)
    while not vna.ready:
        keep(p)
        emax = max(emax, err())
        if truth.now() - out["t_ack"] > sweep_timeout:
            vna.set_trigger(False)
            vna.abort()
            return dict(out, result="sweep_timeout", err_max_during_mm=emax * 1000)
        time.sleep(0.002)
    out["t_done"] = truth.now()
    data = vna.read()
    vna.set_trigger(False)
    out["err_max_during_mm"] = emax * 1000
    if emax > tol:
        return dict(out, result="moved_during_sweep")
    if data is None or data[1] != "ok" or not np.all(np.isfinite(data[0])):
        return dict(out, result="bad_data")
    return dict(out, result="ok", data=data[0])


def hover_log(v, truth, rec, p, until, phase="hold"):
    """Hover on p until sim time `until`; return true and estimated probe positions."""
    rec.set(phase, p)
    keep = Keeper(v)
    tp, ep, last = [], [], None
    while truth.now() < until:
        keep(p)
        s = truth.wait_new(last, 0.2)
        if s is None or s[0] == last:
            continue
        last = s[0]
        tp.append(s[1])
        e = v.est_probe()
        if e is not None:
            ep.append(e)
    return np.array(tp), np.array(ep)


def run_nfscan(v, truth, a, rec, db, flight_id):
    from catr_layout import scan_distance
    from em_model import NearFieldModel, catr_aut, wr187_probe
    from marker_detect import MarkerCamera
    from vna_sim import SimVNA

    def ev(kind, text=""):
        db.event(flight_id, truth.now(), kind, text)
        print(f"  [{kind}] {text}", flush=True)

    def check_flying(what):
        if not v.armed:
            raise RuntimeError(f"vehicle disarmed during {what} (crash?)")

    scan_dist = scan_distance(a.aut_ghz, a.n_lambda)
    freqs = np.linspace(a.freq_start * 1e9, a.freq_stop * 1e9, a.freq_points)
    model = NearFieldModel(catr_aut(), wr187_probe(), freqs, noise_db=a.noise_db)
    vna = SimVNA(model, truth, lambda q: (quat_rot(q, PROBE["normal"]), quat_rot(q, PROBE["e"])),
                 sweep_time=a.vna_sweep_time, miss_prob=a.vna_miss_prob, hang_prob=a.vna_hang_prob,
                 bad_prob=a.vna_bad_prob, seed=a.vna_seed)
    cam = MarkerCamera()
    db.update_flight(flight_id, scan_dist_m=scan_dist)

    # 1. take off on the pad and find the AUT with the camera, turning if it is not in view
    here = take_off(v, truth, rec, a.search_alt)
    ev("takeoff", f"probe at {here[2]:.2f} m, scan distance {scan_dist * 100:.0f} cm "
                  f"({a.n_lambda:g} lambda at {a.aut_ghz:g} GHz)")
    det = None
    for turn in range(12):
        det = look_for_aut(v, truth, rec, cam, here, 3.0)
        if det:
            break
        set_yaw(YAW_ENU + math.radians(30))
        ev("search", f"AUT not in view, turning to {math.degrees(YAW_ENU) % 360:.0f} deg")
        hold(v, truth, rec, here, 3, "turn")
    if det is None:
        raise RuntimeError("AUT marker not found")
    c, n, k = det
    ev("detected", f"{k} frames: AUT centre ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:.3f}) m, "
                   f"facing ({n[0]:+.3f}, {n[1]:+.3f})")

    # 2. square up to the AUT at its centre height, look again from closer, refine
    set_yaw(math.atan2(-n[1], -n[0]))
    standoff = tuple(c + n * (scan_dist + a.standoff_extra))
    move(v, truth, rec, here, standoff, a.transit_speed, "to_standoff")
    hold(v, truth, rec, standoff, 3, "standoff")
    det2 = look_for_aut(v, truth, rec, cam, standoff, 3.0)
    if det2:
        c, n, k = det2
        set_yaw(math.atan2(-n[1], -n[0]))
    true_c, true_n = model.aut.c, model.aut.n
    c_err = float(np.linalg.norm(c - true_c)) * 1000
    n_err = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n, true_n))))))
    db.update_flight(flight_id, aut_est_x=float(c[0]), aut_est_y=float(c[1]), aut_est_z=float(c[2]),
                     aut_est_nx=float(n[0]), aut_est_ny=float(n[1]), aut_err_mm=c_err, aut_normal_err_deg=n_err)
    ev("aligned", f"AUT centre ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:.3f}) m, heading "
                  f"{math.degrees(YAW_ENU):.1f} deg; vs truth {c_err:.0f} mm, {n_err:.2f} deg")
    centre = tuple(c + n * scan_dist)
    standoff = tuple(c + n * (scan_dist + a.standoff_extra))
    move(v, truth, rec, standoff, centre, a.step_speed, "approach")
    hold(v, truth, rec, centre, 3, "height_locked")
    check_flying("alignment")
    ev("height_locked", f"scan plane {scan_dist * 100:.0f} cm in front of the AUT, centre at "
                        f"({centre[0]:.3f}, {centre[1]:.3f}, {centre[2]:.3f})")

    # 3. the grid, relative to the AUT as detected on this flight
    if a.resume:
        grid, _ = db.scan(a.resume)
        scan_id = a.resume
    else:
        grid = nf_grid(a.nf_size, a.nf_spacing)
        settings = {k_: getattr(a, k_) for k_ in ("aut_ghz", "n_lambda", "nf_size", "nf_spacing", "trigger_tol",
                                                  "steady", "hold_time", "max_attempts", "vna_sweep_time")}
        scan_id = db.new_scan(grid, settings, model=MODEL, probe="forward",
                              aut=f"1 x 1 m TE10 aperture, {a.aut_ghz:g} GHz, found by camera; WR-187 probe",
                              freq_start_hz=freqs[0], freq_stop_hz=freqs[-1], freq_points=len(freqs),
                              spacing_m=a.nf_spacing, hold_s=a.hold_time)
    db.update_flight(flight_id, scan_id=scan_id)
    done = db.done(scan_id)
    todo = [g for g in grid if g[0] not in done]
    if a.max_points:
        todo = todo[:a.max_points]
    ev("grid", f"scan {scan_id}: {len(grid)} trigger points, {len(done)} saved, flying {len(todo)} "
               f"(~{len(todo) * (a.hold_time + 2.5) / 60:.0f} min)")

    prev = centre
    n_trig = n_fail = n_done = 0
    for idx, row, col, u, w in todo:
        p = grid_world(c + n * scan_dist, n, u, w)
        for attempt in range(1, a.max_attempts + 1):
            move(v, truth, rec, prev, p, a.step_speed, "step")
            stable, t_st = wait_arrival(v, truth, rec, p, a.trigger_tol, a.arrive_timeout, settle=a.steady)
            check_flying(f"point {idx}")
            if not stable:
                n_trig += 1
                n_fail += 1
                e = v.est_probe()
                db.trigger(flight_id=flight_id, scan_id=scan_id, point_idx=idx, attempt=attempt,
                           t_trigger=truth.now(), result="not_stable",
                           err_before_mm=math.dist(e, p) * 1000 if e else None, steady_s=t_st)
                ev("retry", f"point {idx} attempt {attempt}: not stable within {a.trigger_tol * 100:.0f} cm "
                            f"-> back to the point")
                prev = e or p
                continue
            t_stable = truth.now()
            rec.set("trigger", p)
            tr = trigger_sweep(v, truth, vna, p, a.trigger_tol, a.ack_timeout, 4 * a.vna_sweep_time + 1.0)
            n_trig += 1
            db.trigger(flight_id=flight_id, scan_id=scan_id, point_idx=idx, attempt=attempt,
                       t_trigger=tr["t_trigger"], t_ack=tr["t_ack"], t_done=tr["t_done"], result=tr["result"],
                       err_before_mm=tr["err_before_mm"], err_max_during_mm=tr["err_max_during_mm"], steady_s=t_st)
            if tr["result"] != "ok":
                n_fail += 1
                ev("retry", f"point {idx} attempt {attempt}: {tr['result']} -> back to the point, trigger again")
                prev = v.est_probe() or p
                continue
            tp, ep = hover_log(v, truth, rec, p, t_stable + a.hold_time)
            check_flying(f"point {idx}")
            s21 = tr["data"]
            point = dict(idx=idx, row=row, col=col, grid_u=u, grid_v=w, flight_id=flight_id, attempts=attempt,
                         target_x=p[0], target_y=p[1], target_z=p[2], arrived=1, t_arrive_s=t_st,
                         t_hold_start=t_stable, t_hold_end=truth.now(), n_sweeps=1)
            for name, arr in (("est", ep), ("true", tp)):
                if len(arr):
                    for kx, axis in enumerate("xyz"):
                        point[f"{name}_{axis}"] = float(arr[:, kx].mean())
                        point[f"{name}_std_{axis}"] = float(arr[:, kx].std())
            meas = [dict(freq_hz=float(f), s21_re=float(s21[i].real), s21_im=float(s21[i].imag),
                         amp_db=float(20 * np.log10(abs(s21[i]))), phase_deg=float(np.degrees(np.angle(s21[i]))))
                    for i, f in enumerate(freqs)]
            db.save_point(scan_id, point, meas)
            n_done += 1
            ci = len(freqs) // 2
            print(f"  point {idx:3d} (row {row:2d}, col {col:2d}, u {u:+.2f}, w {w:+.2f}): attempt {attempt}, "
                  f"trigger at {tr['err_before_mm']:.0f} mm, sweep max {tr['err_max_during_mm']:.0f} mm, "
                  f"S21 {meas[ci]['amp_db']:.1f} dB {meas[ci]['phase_deg']:.0f} deg", flush=True)
            break
        else:
            db.update_flight(flight_id, points_done=n_done, triggers_total=n_trig, triggers_failed=n_fail)
            raise RuntimeError(f"point {idx} failed {a.max_attempts} times - scan {scan_id} stopped, "
                               f"resume with --resume {scan_id}")
        prev = p
        db.update_flight(flight_id, points_done=n_done, triggers_total=n_trig, triggers_failed=n_fail)
    vna.close()
    ev("grid_done", f"{n_done} points, {n_trig} triggers, {n_fail} retried")

    # 4. back to the grid centre, away from the AUT, over the pad, land
    move(v, truth, rec, prev, centre, a.step_speed, "to_centre")
    move(v, truth, rec, centre, standoff, a.step_speed, "retreat")
    move(v, truth, rec, standoff, here, a.transit_speed, "return")
    hold(v, truth, rec, here, 3, "return_settle")
    rec.set("land", here)
    v.mode("LAND")
    t_end = truth.now() + 40
    while truth.now() < t_end and v.armed:
        time.sleep(0.5)
    ev("landed", f"scan {scan_id}: {len(db.done(scan_id))}/{len(grid)} points saved")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=["hold", "hover", "scan", "grid", "nfscan"])
    ap.add_argument("--probe", choices=list(PROBES), default=None,
                    help="probe direction (default: 'down' for *_down models, else 'forward')")
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--alt", type=float, default=4.0, help="probe height (m)")
    ap.add_argument("--hold", type=float, default=40)
    ap.add_argument("--freq-ghz", type=float, default=3.0)
    ap.add_argument("--width", type=float, default=3.0)
    ap.add_argument("--height", type=float, default=3.0)
    ap.add_argument("--centre-z", type=float, default=4.0)
    ap.add_argument("--scan-dist", type=float, default=1.0)
    ap.add_argument("--line-spacing", type=float, default=None)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--tag", default="")
    g = ap.add_argument_group("grid")
    g.add_argument("--corner", type=float, nargs=2, default=(10.25, -4.0),
                   help="first grid point (probe x y): lab corner, 1.5 m clear of the walls and anchor poles")
    g.add_argument("--end", type=float, nargs=2, default=(1.0, 0.0), help="opposite grid corner, in front of the AUT")
    g.add_argument("--grid-z", type=float, default=4.0, help="probe height (m), AUT centre height")
    g.add_argument("--spacing", type=float, default=0.15)
    g.add_argument("--hold-time", type=float, default=10.0)
    g.add_argument("--arrive-tol", type=float, default=0.05, help="estimated probe position tolerance (m)")
    g.add_argument("--arrive-timeout", type=float, default=20.0)
    g.add_argument("--step-speed", type=float, default=0.2, help="m/s between grid points")
    g.add_argument("--transit-speed", type=float, default=0.3, help="m/s to the first point and back")
    g.add_argument("--sweep-interval", type=float, default=1.0, help="s between VNA sweeps during the hold")
    g.add_argument("--freq-start", type=float, default=None, help="GHz (default: AUT frequency - 4 %%)")
    g.add_argument("--freq-stop", type=float, default=None, help="GHz (default: AUT frequency + 4 %%)")
    g.add_argument("--freq-points", type=int, default=21)
    g.add_argument("--noise-db", type=float, default=-70.0, help="VNA noise floor re boresight at 1 m")
    g.add_argument("--max-points", type=int, default=0, help="stop after N points (0 = all)")
    g.add_argument("--resume", type=int, default=0, help="continue scan ID from the database")
    g.add_argument("--db", default=None, help="SQLite file (default data/nf_scans.db)")
    nf = ap.add_argument_group("nfscan (planar near-field scan with camera alignment and VNA trigger)")
    nf.add_argument("--aut-ghz", type=float, default=5.0, help="AUT frequency (GHz)")
    nf.add_argument("--n-lambda", type=float, default=5.0, help="scan distance in wavelengths")
    nf.add_argument("--nf-size", type=float, default=2.0, help="scan grid side (m), centred on the AUT")
    nf.add_argument("--nf-spacing", type=float, default=0.10, help="trigger point spacing (m)")
    nf.add_argument("--search-alt", type=float, default=3.5, help="probe height for finding the AUT (m)")
    nf.add_argument("--standoff-extra", type=float, default=1.0, help="stand-off beyond the scan plane (m)")
    nf.add_argument("--trigger-tol", type=float, default=0.02, help="position tolerance to trigger (m)")
    nf.add_argument("--steady", type=float, default=1.0, help="s inside the tolerance before triggering")
    nf.add_argument("--max-attempts", type=int, default=5, help="tries per trigger point before the scan stops")
    nf.add_argument("--ack-timeout", type=float, default=0.2, help="s for the VNA to acknowledge the trigger")
    nf.add_argument("--vna-sweep-time", type=float, default=0.5, help="s per VNA sweep")
    nf.add_argument("--vna-miss-prob", type=float, default=0.02, help="simulated missed triggers")
    nf.add_argument("--vna-hang-prob", type=float, default=0.01, help="simulated sweeps that never finish")
    nf.add_argument("--vna-bad-prob", type=float, default=0.01, help="simulated invalid sweeps")
    nf.add_argument("--vna-seed", type=int, default=None)
    a = ap.parse_args()

    global PROBE, PROBE_OFFSET, HOME_ENU
    a.probe = a.probe or ("down" if "down" in MODEL else "forward")
    PROBE = PROBES[a.probe]
    PROBE_OFFSET = PROBE["offset"]
    a.freq_start = a.freq_start or round(a.aut_ghz * 0.96, 4)
    a.freq_stop = a.freq_stop or round(a.aut_ghz * 1.04, 4)
    if a.db is None:
        from scan_db import DEFAULT_DB
        a.db = DEFAULT_DB
    from scan_db import ScanDB, utc

    truth = Truth()
    t0 = time.time()
    while truth.probe() is None:
        if time.time() - t0 > 30:
            raise RuntimeError(f"no pose for model '{MODEL}' on /world/{WORLD}/pose/info")
        time.sleep(0.2)
    HOME_ENU = tuple(truth.probe()[3])                    # where run_sim.sh put the drone
    print("gazebo truth OK, drone at", tuple(round(c, 3) for c in HOME_ENU),
          "probe at", tuple(round(c, 3) for c in truth.probe()[1]))
    db = ScanDB(a.db)
    flight_id = db.new_flight(test=a.test, model=MODEL, aut_ghz=a.aut_ghz, n_lambda=a.n_lambda,
                              takeoff_x=HOME_ENU[0], takeoff_y=HOME_ENU[1], args_json=json.dumps(vars(a)))
    print(f"FLIGHT_ID {flight_id} -> {a.db}")
    started = time.time()
    v = Vehicle(a.url)
    name = {"hold": "hold", "hover": "hover", "scan": f"scan{a.freq_ghz:g}GHz", "grid": "grid",
            "nfscan": f"nfscan{a.aut_ghz:g}GHz"}[a.test] + a.tag
    rec = Recorder(truth, name)
    status, reason = "completed", ""
    try:
        if a.test == "hold":
            run_hold(v, truth, rec, a)
        elif a.test == "hover":
            run_hover(v, truth, a, rec)
        elif a.test == "grid":
            run_grid(v, truth, a, rec)
        elif a.test == "nfscan":
            run_nfscan(v, truth, a, rec, db, flight_id)
        else:
            run_scan(v, truth, a, rec)
    except BaseException as exc:
        status, reason = "aborted", f"{type(exc).__name__}: {exc}"
        db.event(flight_id, truth.now(), "error", reason)
        if v.armed:                                       # never leave it hovering next to the AUT
            try:
                v.mode("LAND")
            except Exception:
                pass
        raise
    finally:
        rec.close()
        truth.close()
        db.update_flight(flight_id, ended_utc=utc(), status=status, reason=reason)
        n_tel = db.import_telemetry(flight_id, rec.path)
        bins = [b for b in glob.glob(os.path.join(OUT_DIR, "logs", "*.BIN")) if os.path.getmtime(b) >= started]
        files = [("flight_csv", rec.path), ("ardupilot_bin", max(bins, key=os.path.getmtime) if bins else None),
                 ("uwb_log", os.path.join(OUT_DIR, "uwb.log")), ("gazebo_log", os.path.join(OUT_DIR, "gz.log")),
                 ("sim_console", os.path.join(OUT_DIR, "sim_console.log"))]
        d = db.archive(flight_id, files)
        print(f"flight {flight_id} {status}: {n_tel} telemetry rows, logs archived to {d}")
        print("RESULT_CSV", rec.path)


if __name__ == "__main__":
    main()
