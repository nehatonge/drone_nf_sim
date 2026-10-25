#!/usr/bin/env python3
"""
Flight tests for the near-field hexacopter in Gazebo + ArduPilot SITL.

  python3 flight_test.py hold  [--alt 4] [--hold 40]        take off, hover in place (stability)
  python3 flight_test.py hover [--alt 4] [--hold 40]        hover with the probe on the scan plane
  python3 flight_test.py scan  [--freq-ghz 3] [--width 3] [--height 3] [--speed 0.5]

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
import json
import math
import os
import subprocess
import threading
import time

from pymavlink import mavutil

WORLD = "nf_range"
MODEL = os.environ.get("NF_MODEL", "nf_hexa")
PROBE_OFFSET = (1.25, 0.0, -0.15)
HOME_ENU = (6.0, 0.0, 0.40)        # spawn pose of the model origin
YAW_ENU = math.pi                  # facing the AUT
YAW_NED = math.pi / 2 - YAW_ENU    # NED heading (rad): -90 deg = west
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


def enu_to_local_ned(p):
    """ENU world -> ArduPilot local NED relative to home (the spawn point)."""
    return (p[1] - HOME_ENU[1], p[0] - HOME_ENU[0], -(p[2] - HOME_ENU[2]))


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
        threading.Thread(target=self._pump, daemon=True).start()
        self.armed = False
        self.mode_name = ""

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
                self.last_text = msg.text
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode_name = mavutil.mode_string_v10(msg)

    def wait_ready(self, timeout=180):
        print("waiting for EKF / GPS ...")
        self.last_text = ""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if "using GPS" in getattr(self, "last_text", ""):
                time.sleep(5)
                return
            time.sleep(0.2)
        raise RuntimeError("EKF never started using GPS")

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

    def goto_ned(self, n, e, d, vn=0.0, ve=0.0, vd=0.0, yaw_ned=YAW_NED):
        # use position + velocity + yaw; ignore accel (bits 6-8) and yaw rate (bit 11)
        mask = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 11)
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e, d, vn, ve, vd, 0, 0, 0, yaw_ned, 0)

    def goto_probe(self, p, vel=(0, 0, 0)):
        n, e, d = enu_to_local_ned(body_for_probe(p))
        self.goto_ned(n, e, d, vel[1], vel[0], -vel[2])


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=["hold", "hover", "scan"])
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
    a = ap.parse_args()

    truth = Truth()
    t0 = time.time()
    while truth.probe() is None:
        if time.time() - t0 > 30:
            raise RuntimeError(f"no pose for model '{MODEL}' on /world/{WORLD}/pose/info")
        time.sleep(0.2)
    print("gazebo truth OK, probe at", tuple(round(c, 3) for c in truth.probe()[1]))
    v = Vehicle(a.url)
    name = {"hold": "hold", "hover": "hover", "scan": f"scan{a.freq_ghz:g}GHz"}[a.test] + a.tag
    rec = Recorder(truth, name)
    try:
        if a.test == "hold":
            run_hold(v, truth, rec, a)
        elif a.test == "hover":
            run_hover(v, truth, a, rec)
        else:
            run_scan(v, truth, a, rec)
    finally:
        rec.close()
        truth.close()
    print("RESULT_CSV", rec.path)


if __name__ == "__main__":
    main()
