"""
Layout of the compact antenna test range (CATR) lab, shared by gen_catr_lab.py
(Gazebo world) and uwb_extnav.py (UWB positioning).

World frame ENU, metres, origin on the floor below the AUT aperture centre:
  x along the range: reflector end at -x, back wall at +x
  y across the range, z up
The AUT aperture lies in the x = 0 plane facing +x (positioner turned 180 deg from
its CATR reference orientation, which faces the reflector at -x), so the drone
scans on the down-range side where there is room. All values marked EST are
placeholders until the real lab drawing is available.
"""

# Interior of the lab, measured to the absorber tips (25 x 12 x 10 m)
ROOM_X = (-12.0, 13.0)
ROOM_Y = (-6.0, 6.0)
ROOM_Z = (0.0, 10.0)

# Pyramidal absorber: base pitch, pyramid height, backing thickness (EST)
ABS_PITCH = 0.6
ABS_HEIGHT = 0.6
ABS_BACKING = 0.1
FLOOR_ABS_HEIGHT = 0.45           # shorter floor absorber
FLOOR_ABS_X = (ROOM_X[0], 3.5)    # floor absorber from the reflector up to just past the scan plane;
                                  # flat walk-on floor beyond, where the drone takes off and lands

# CATR (EST): offset-fed reflector at the -x end, feed on a tower below the range axis
QZ_CENTRE = (0.0, 0.0, 4.0)       # quiet-zone centre = AUT aperture centre
QZ_DIAMETER = 3.0
REFLECTOR_CENTRE = (-10.6, 0.0, 5.5)
REFLECTOR_SIZE = (6.0, 6.0)       # width (y), height (z), serrated edge not modelled
REFLECTOR_TILT_DEG = 15.0         # rotation about y, aims the offset paraboloid at the quiet zone
FEED_POS = (-5.0, 0.0, 1.8)

# AUT on a roll-over-azimuth positioner
AUT_SIZE = (1.0, 1.0)             # aperture width (y), height (z)
AUT_POSITIONER_X = -0.6

# ArUco marker on a plate just above the aperture, in the aperture plane, for the
# drone camera to find and align to the AUT. The aperture centre is MARKER_ABOVE
# metres below the marker centre.
MARKER_DICT = "DICT_4X4_50"
MARKER_ID = 0
MARKER_SIZE = 0.50                # black square side (m)
MARKER_PLATE = 0.60               # white plate side (m)
MARKER_ABOVE = AUT_SIZE[1] / 2 + 0.10 + MARKER_PLATE / 2
MARKER_CENTRE = (0.004, 0.0, QZ_CENTRE[2] + MARKER_ABOVE)

# Drone default spawn: flat floor, facing the AUT (run_sim.sh normally computes it
# from the antenna frequency with takeoff_point())
SPAWN = (6.0, 0.0)
PROBE_AHEAD = 1.325               # probe aperture ahead of the drone centre (gen_nf_hexa.py, forward probe)
# the probe hangs 0.25 m above the floor at take-off, so it must be over the flat floor
MIN_TAKEOFF_X = FLOOR_ABS_X[1] + PROBE_AHEAD + 0.2


def scan_distance(freq_ghz, n_lambda=5.0):
    """Probe-to-aperture distance for a planar near-field scan: n_lambda wavelengths."""
    return n_lambda * 0.299792458 / freq_ghz


def takeoff_point(freq_ghz, n_lambda=5.0):
    """Take-off point on the AUT boresight: straight behind the drone's scan position
    (scan distance + probe offset), but never closer than the flat floor allows."""
    x = QZ_CENTRE[0] + scan_distance(freq_ghz, n_lambda) + PROBE_AHEAD
    return (round(max(x, MIN_TAKEOFF_X), 3), QZ_CENTRE[1])


if __name__ == "__main__":
    import sys
    print(*takeoff_point(float(sys.argv[1]) if len(sys.argv) > 1 else 5.0,
                         float(sys.argv[2]) if len(sys.argv) > 2 else 5.0))

# UWB anchors (x, y, z): on poles just inside the absorber so the absorber does not
# block them, 4 low + 4 high around the scan volume for a good vertical geometry (EST)
UWB_ANCHORS = [
    (-1.5, -5.3, 1.2), (-1.5, 5.3, 1.2), (9.0, -5.3, 1.2), (9.0, 5.3, 1.2),
    (-1.5, -5.3, 8.5), (-1.5, 5.3, 8.5), (9.0, -5.3, 8.5), (9.0, 5.3, 8.5),
]
UWB_TAG_OFFSET = (-0.05, 0.0, 0.31)   # tag on the top-deck mast (body x fwd, y left, z up), where the GNSS antenna was
