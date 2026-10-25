#!/usr/bin/env python3
"""
UWB positioning for the CATR lab: stands in for the UWB tag on the drone and its
position engine (no GNSS indoors).

Every 1/rate s:
  true tag position from Gazebo -> range to each anchor + noise -> least-squares
  position fix -> VISION_POSITION_ESTIMATE to ArduPilot (EK3_SRC1_POSXY = 6, ExternalNav)

Range noise model, per anchor: white noise (--range-sigma), a fixed bias drawn at
start-up (--bias-sigma, antenna delay / calibration error) and, with probability
--nlos-prob, a positive non-line-of-sight excess (exponential, mean --nlos-mean).
The position error then follows from the anchor geometry, as on the real system.

It also sets the EKF origin, which a GPS would normally do.

  NF_MODEL=nf_hexa_trimmed python3 uwb_extnav.py [--url tcp:127.0.0.1:5762] [--rate 10] [--range-sigma 0.05]
(run_sim.sh starts it automatically.)
"""
import argparse
import time

import numpy as np
from pymavlink import mavutil

from catr_layout import UWB_ANCHORS, UWB_TAG_OFFSET
from flight_test import Truth, quat_rot

ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = -35.363262, 149.165237, 584.0   # world spherical_coordinates


def fix(anchors, ranges, x0):
    """Gauss-Newton multilateration from the previous fix."""
    x = np.array(x0, dtype=float)
    for _ in range(10):
        d = x - anchors
        n = np.linalg.norm(d, axis=1)
        J = d / n[:, None]
        dx, *_ = np.linalg.lstsq(J, ranges - n, rcond=None)
        x += dx
        if np.linalg.norm(dx) < 1e-5:
            break
    return x


def connect(url):
    while True:
        try:
            m = mavutil.mavlink_connection(url, source_system=1, source_component=197, retries=1)
            m.wait_heartbeat(timeout=10)
            return m
        except (ConnectionRefusedError, OSError, TimeoutError):
            time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="tcp:127.0.0.1:5762", help="SITL SERIAL1")
    ap.add_argument("--rate", type=float, default=10.0, help="position updates per second")
    ap.add_argument("--range-sigma", type=float, default=0.05, help="white range noise, 1-sigma (m)")
    ap.add_argument("--bias-sigma", type=float, default=0.02, help="per-anchor constant range bias, 1-sigma (m)")
    ap.add_argument("--nlos-prob", type=float, default=0.0, help="probability of an NLOS range per update")
    ap.add_argument("--nlos-mean", type=float, default=0.3, help="mean NLOS excess range (m)")
    ap.add_argument("--seed", type=int, default=None)
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    anchors = np.array(UWB_ANCHORS, dtype=float)
    bias = rng.normal(0, a.bias_sigma, len(anchors))

    truth = Truth()
    while truth.probe() is None:
        time.sleep(0.2)
    m = connect(a.url)
    print(f"uwb: {len(anchors)} anchors, sigma {a.range_sigma * 100:.0f} cm, bias {a.bias_sigma * 100:.0f} cm, "
          f"NLOS {a.nlos_prob:.0%}, {a.rate:g} Hz -> {a.url}", flush=True)

    x = None
    origin_sent, last_origin = 0, 0.0
    errs = []
    t_report = time.time() + 10
    period = 1.0 / a.rate
    next_t = time.time()
    while True:
        next_t += period
        _, _, q, body = truth.probe()
        off = quat_rot(q, UWB_TAG_OFFSET)
        tag = np.array(body) + np.array(off)
        r = np.linalg.norm(anchors - tag, axis=1) + bias + rng.normal(0, a.range_sigma, len(anchors))
        if a.nlos_prob > 0:
            r += (rng.random(len(anchors)) < a.nlos_prob) * rng.exponential(a.nlos_mean, len(anchors))
        x = fix(anchors, r, tag if x is None else x)
        errs.append(x - tag)

        # ENU world -> NED; VISO_POS_* tells the EKF where the tag sits on the frame
        m.mav.vision_position_estimate_send(int(time.time() * 1e6), x[1], x[0], -x[2], 0, 0, 0)

        # no GPS indoors: give the EKF its origin (repeated a few times, ignored once set)
        if origin_sent < 10 and time.time() - last_origin > 2:
            m.mav.set_gps_global_origin_send(m.target_system, int(ORIGIN_LAT * 1e7), int(ORIGIN_LON * 1e7),
                                             int(ORIGIN_ALT * 1000), int(time.time() * 1e6))
            last_origin = time.time()
            origin_sent += 1

        while m.recv_match(blocking=False) is not None:     # keep the socket drained
            pass

        if time.time() > t_report:
            e = np.array(errs)
            rms = np.sqrt(np.mean(e**2, axis=0)) * 1000
            print(f"uwb: fix error rms x {rms[0]:.0f} mm, y {rms[1]:.0f} mm, z {rms[2]:.0f} mm "
                  f"({len(e)} fixes)", flush=True)
            errs.clear()
            t_report = time.time() + 10
        time.sleep(max(0.0, next_t - time.time()))


if __name__ == "__main__":
    main()
