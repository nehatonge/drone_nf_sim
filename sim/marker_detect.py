#!/usr/bin/env python3
"""
Finds the AUT with the drone's forward camera: detects the ArUco marker above the
aperture and returns the aperture centre and normal in the world frame.

  camera image (Gazebo /nf/camera) -> ArUco corners -> solvePnP -> marker pose in the
  camera -> world, using the drone's OWN pose estimate (EKF: UWB + flow), as the
  Jetson would -> aperture centre = marker centre - MARKER_ABOVE (vertical)

  python3 marker_detect.py        print detections for 10 s (needs a running sim);
                                  saves the last image to ~/nf_runs/camera.png
"""
import os
import threading
import time

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

from catr_layout import MARKER_ABOVE, MARKER_DICT, MARKER_ID, MARKER_SIZE

CAM_TOPIC = "/nf/camera"
CAM_OFFSET = np.array([0.20, 0.0, 0.0])     # camera in the body frame (gen_nf_hexa.CAMERA)
CAM_HFOV = 1.2
# camera optical frame (x right, y down, z forward) -> camera/body frame (x fwd, y left, z up)
R_BODY_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


class MarkerCamera:
    def __init__(self, topic=CAM_TOPIC):
        self.lock = threading.Lock()
        self.frame = None            # (stamp_s, BGR image)
        self.node = Node()
        if not self.node.subscribe(Image, topic, self._cb):
            raise RuntimeError(f"cannot subscribe to {topic}")
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # whole-pixel corners: ~5 % range error at 20 px
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_DICT)),
                                                params)
        s = MARKER_SIZE / 2
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

    def _cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        st = msg.header.stamp
        with self.lock:
            self.frame = (st.sec + st.nsec * 1e-9, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    def latest(self):
        with self.lock:
            return self.frame

    def detect(self):
        """Marker pose in the camera optical frame: (stamp, R_opt_marker, t_opt, pixel size) or None."""
        f = self.latest()
        if f is None:
            return None
        stamp, img = f
        h, w = img.shape[:2]
        fx = (w / 2) / np.tan(CAM_HFOV / 2)
        K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]])
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if ids is None or MARKER_ID not in ids.flatten():
            return None
        c = corners[list(ids.flatten()).index(MARKER_ID)].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(self.obj, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        return stamp, R, tvec.reshape(3), float(np.linalg.norm(c[0] - c[1]))


def marker_to_aperture(R_wb, p_b, R_opt_m, t_opt):
    """Marker pose in the camera -> aperture centre and normal in the world.
    R_wb, p_b: drone body rotation and position in the world (estimate)."""
    R_bm = R_BODY_OPT @ R_opt_m
    p_m = p_b + R_wb @ (CAM_OFFSET + R_BODY_OPT @ t_opt)
    n = R_wb @ R_bm[:, 2]                    # marker z axis points out of the plate, towards the drone
    n[2] = 0.0                               # AUT stands vertical: keep the horizontal facing direction
    n /= np.linalg.norm(n)
    centre = p_m - np.array([0, 0, MARKER_ABOVE])
    return centre, n


def main():
    """Stand-alone check against Gazebo truth (pose from /world/catr_lab/pose/info)."""
    from flight_test import Truth
    cam = MarkerCamera()
    truth = Truth()
    t_end = time.time() + 10
    while time.time() < t_end:
        d = cam.detect()
        s = truth.probe()
        if d is None or s is None:
            print("no marker" if s else "no pose")
            time.sleep(0.5)
            continue
        _, R, t, px = d
        q = s[2]
        w, x, y, z = q
        R_wb = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        c, n = marker_to_aperture(R_wb, np.array(s[3]), R, t)
        print(f"marker {px:.0f} px, range {np.linalg.norm(t):.2f} m -> aperture centre "
              f"({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:.3f}) normal ({n[0]:+.3f}, {n[1]:+.3f})  [true (0, 0, 4.000) (+1, 0)]")
        time.sleep(0.5)
    f = cam.latest()
    if f is not None:
        path = os.path.expanduser("~/nf_runs/camera.png")
        cv2.imwrite(path, f[1])
        print("image:", path)


if __name__ == "__main__":
    main()
