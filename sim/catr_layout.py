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

# Drone spawn: flat floor, facing the AUT
SPAWN = (6.0, 0.0)

# UWB anchors (x, y, z): on poles just inside the absorber so the absorber does not
# block them, 4 low + 4 high around the scan volume for a good vertical geometry (EST)
UWB_ANCHORS = [
    (-1.5, -5.3, 1.2), (-1.5, 5.3, 1.2), (9.0, -5.3, 1.2), (9.0, 5.3, 1.2),
    (-1.5, -5.3, 8.5), (-1.5, 5.3, 8.5), (9.0, -5.3, 8.5), (9.0, 5.3, 8.5),
]
UWB_TAG_OFFSET = (-0.05, 0.0, 0.31)   # tag on the top-deck mast (body x fwd, y left, z up), where the GNSS antenna was
