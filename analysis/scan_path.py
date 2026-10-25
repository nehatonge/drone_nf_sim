#!/usr/bin/env python3
"""
Planar serpentine raster for the near-field scan (report Fig. 2).

Coordinate frame (local ENU, origin on the ground below the AUT centre):
  AUT aperture lies in the x = 0 plane, facing +x.
  Scan plane is x = SCAN_DIST, spanning y (horizontal) and z (height).
The probe, not the drone centre, has to follow the raster: the drone centre is
offset by the boom length along -x (the boom points at the AUT, +x of the drone
is towards the AUT, so the drone yaw faces -x... i.e. yaw = 180 deg in ENU).
"""
import argparse
import json
import math


def raster(width, height, centre_y, centre_z, line_spacing):
    """Serpentine lines along y, stepping in z (bottom to top). Returns list of (y0, y1, z)."""
    n_lines = int(round(height / line_spacing)) + 1
    lines = []
    for i in range(n_lines):
        z = centre_z - height / 2 + i * line_spacing
        y0, y1 = centre_y - width / 2, centre_y + width / 2
        if i % 2:
            y0, y1 = y1, y0
        lines.append((y0, y1, z))
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq-ghz", type=float, default=3.0)
    ap.add_argument("--width", type=float, default=3.0)
    ap.add_argument("--height", type=float, default=3.0)
    ap.add_argument("--centre-z", type=float, default=4.0, help="AUT centre height (m)")
    ap.add_argument("--scan-dist", type=float, default=1.0, help="probe-to-AUT distance (m)")
    ap.add_argument("--boom", type=float, default=1.25, help="probe offset ahead of drone centre (m)")
    ap.add_argument("--line-spacing", type=float, default=None, help="default lambda/2")
    ap.add_argument("--out", default="scan_path.json")
    a = ap.parse_args()

    lam = 0.299792458 / a.freq_ghz
    dz = a.line_spacing or lam / 2
    lines = raster(a.width, a.height, 0.0, a.centre_z, dz)

    # Drone centre sits boom-length further from the AUT than the probe; drone faces the AUT.
    drone_x = a.scan_dist + a.boom
    plan = {
        "freq_ghz": a.freq_ghz,
        "lambda_m": lam,
        "line_spacing_m": dz,
        "sample_spacing_m": lam / 2,
        "scan_dist_m": a.scan_dist,
        "boom_m": a.boom,
        "drone_x_m": drone_x,
        "yaw_enu_deg": 180.0,
        "lines": [{"y0": y0, "y1": y1, "z": z} for (y0, y1, z) in lines],
    }
    with open(a.out, "w") as f:
        json.dump(plan, f, indent=1)
    length = sum(abs(l[1] - l[0]) for l in lines) + a.height
    print(f"lambda={lam*1000:.1f} mm, {len(lines)} lines, path {length:.0f} m -> {a.out}")


if __name__ == "__main__":
    main()
