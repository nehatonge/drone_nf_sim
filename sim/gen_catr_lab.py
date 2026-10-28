#!/usr/bin/env python3
"""
Generates the CATR lab for Gazebo (Harmonic) from catr_layout.py:
  models/catr_lab/        static model: absorber-lined room, reflector, feed,
                          quiet zone, AUT on its positioner, UWB anchors
  models/catr_lab/meshes/ pyramidal absorber as binary STL, one mesh per surface
  worlds/catr_lab.sdf     world: lab + the drone (MODEL_NAME, filled in by run_sim.sh)

Absorber is visual geometry only: Gazebo does not simulate RF. Each surface also
gets a flat collision box at the pyramid tips so a drone that drifts hits it.
"""
import math
import os
import struct

from catr_layout import (ABS_BACKING, ABS_HEIGHT, ABS_PITCH, AUT_POSITIONER_X, AUT_SIZE,
                         FEED_POS, FLOOR_ABS_HEIGHT, FLOOR_ABS_X, MARKER_CENTRE, MARKER_DICT,
                         MARKER_ID, MARKER_PLATE, MARKER_SIZE, QZ_CENTRE, QZ_DIAMETER,
                         REFLECTOR_CENTRE, REFLECTOR_SIZE, REFLECTOR_TILT_DEG, ROOM_X,
                         ROOM_Y, ROOM_Z, UWB_ANCHORS)

HERE = os.path.dirname(os.path.abspath(__file__))
ABSORBER_RGB = "0.13 0.17 0.38 1"
BACKING_RGB = "0.08 0.10 0.22 1"


# ----------------------------------------------------------------- vectors
def add(*v):
    return tuple(sum(c) for c in zip(*v))


def mul(v, s):
    return tuple(c * s for c in v)


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def unit(v):
    n = math.sqrt(dot(v, v))
    return mul(v, 1 / n)


# ----------------------------------------------------------------- absorber mesh
def pyramids(base0, u, v, n, u_len, v_len, height):
    """Triangles of a pyramid array: base plane at base0 spanned by unit vectors u, v,
    tips pointing along n. The grid is centred on the span."""
    nu, nv = int(u_len // ABS_PITCH), int(v_len // ABS_PITCH)
    ou, ov = (u_len - nu * ABS_PITCH) / 2, (v_len - nv * ABS_PITCH) / 2
    h = ABS_PITCH / 2
    tris = []
    for i in range(nu):
        for j in range(nv):
            c = add(base0, mul(u, ou + (i + 0.5) * ABS_PITCH), mul(v, ov + (j + 0.5) * ABS_PITCH))
            tip = add(c, mul(n, height))
            corners = [add(c, mul(u, su * h), mul(v, sv * h)) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            for k in range(4):
                a, b = corners[k], corners[(k + 1) % 4]
                nrm = cross(sub(b, a), sub(tip, a))
                if dot(nrm, n) < 0:           # make every face point away from the wall
                    a, b = b, a
                    nrm = mul(nrm, -1)
                tris.append((unit(nrm), a, b, tip))
    return tris


def write_marker_png(path, px=600):
    """ArUco marker with its white border, as the plate texture."""
    import cv2
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_DICT))
    inner = int(round(px * MARKER_SIZE / MARKER_PLATE))
    img = cv2.aruco.generateImageMarker(d, MARKER_ID, inner)
    pad = (px - inner) // 2
    img = cv2.copyMakeBorder(img, pad, px - inner - pad, pad, px - inner - pad, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(path, img)


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"catr_lab absorber".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for nrm, a, b, c in tris:
            f.write(struct.pack("<12fH", *nrm, *a, *b, *c, 0))


def surfaces():
    """(name, base0, u, v, n, u_len, v_len, height): absorber tips sit on the room interior."""
    x0, x1 = ROOM_X
    y0, y1 = ROOM_Y
    z0, z1 = ROOM_Z
    H = ABS_HEIGHT
    lx, ly, lz = x1 - x0, y1 - y0, z1 - z0
    X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    return [
        ("wall_reflector_end", (x0 - H, y0, z0), Y, Z, X, ly, lz, H),
        ("wall_back", (x1 + H, y0, z0), Y, Z, (-1, 0, 0), ly, lz, H),
        ("wall_right", (x0, y0 - H, z0), X, Z, Y, lx, lz, H),
        ("wall_left", (x0, y1 + H, z0), X, Z, (0, -1, 0), lx, lz, H),
        ("ceiling", (x0, y0, z1 + H), X, Y, (0, 0, -1), lx, ly, H),
        ("floor", (FLOOR_ABS_X[0], y0, z0), X, Y, Z, FLOOR_ABS_X[1] - FLOOR_ABS_X[0], ly, FLOOR_ABS_HEIGHT),
    ]


# ----------------------------------------------------------------- SDF snippets
def material(rgb, extra=""):
    return f"<material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse>{extra}</material>"


def box(name, pose, size, rgb, collide=False, rpy=(0, 0, 0)):
    p = f"{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} {rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"
    g = f"<geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>"
    s = f'      <visual name="{name}"><pose>{p}</pose>{g}{material(rgb)}</visual>\n'
    if collide:
        s += f'      <collision name="{name}_col"><pose>{p}</pose>{g}</collision>\n'
    return s


def cylinder(name, pose, r, length, rgb, collide=False, rpy=(0, 0, 0)):
    p = f"{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} {rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"
    g = f"<geometry><cylinder><radius>{r}</radius><length>{length:.3f}</length></cylinder></geometry>"
    s = f'      <visual name="{name}"><pose>{p}</pose>{g}{material(rgb)}</visual>\n'
    if collide:
        s += f'      <collision name="{name}_col"><pose>{p}</pose>{g}</collision>\n'
    return s


def build_model(mesh_dir):
    x0, x1 = ROOM_X
    y0, y1 = ROOM_Y
    z0, z1 = ROOM_Z
    H, B = ABS_HEIGHT, ABS_BACKING
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    lx, ly, lz = x1 - x0, y1 - y0, z1 - z0
    out = []

    # absorber meshes
    n_pyr = 0
    for name, base0, u, v, n, ul, vl, h in surfaces():
        tris = pyramids(base0, u, v, n, ul, vl, h)
        n_pyr += len(tris) // 4
        write_stl(os.path.join(mesh_dir, f"{name}.stl"), tris)
        out.append(f'      <visual name="{name}_absorber"><geometry><mesh><uri>model://catr_lab/meshes/{name}.stl</uri>'
                   f'</mesh></geometry>{material(ABSORBER_RGB)}</visual>\n')

    # backing behind the pyramids (visual) and collision at the tip plane
    out.append(box("backing_reflector_end", (x0 - H - B / 2, cy, cz), (B, ly + 2 * H, lz + 2 * H), BACKING_RGB))
    out.append(box("backing_back", (x1 + H + B / 2, cy, cz), (B, ly + 2 * H, lz + 2 * H), BACKING_RGB))
    out.append(box("backing_right", (cx, y0 - H - B / 2, cz), (lx + 2 * H, B, lz + 2 * H), BACKING_RGB))
    out.append(box("backing_left", (cx, y1 + H + B / 2, cz), (lx + 2 * H, B, lz + 2 * H), BACKING_RGB))
    out.append(box("backing_ceiling", (cx, cy, z1 + H + B / 2), (lx + 2 * H, ly + 2 * H, B), BACKING_RGB))
    for name, pose, size in [
        ("tips_reflector_end", (x0 - H / 2, cy, cz), (H, ly, lz)),
        ("tips_back", (x1 + H / 2, cy, cz), (H, ly, lz)),
        ("tips_right", (cx, y0 - H / 2, cz), (lx, H, lz)),
        ("tips_left", (cx, y1 + H / 2, cz), (lx, H, lz)),
        ("tips_ceiling", (cx, cy, z1 + H / 2), (lx, ly, H)),
        ("tips_floor", ((FLOOR_ABS_X[0] + FLOOR_ABS_X[1]) / 2, cy, FLOOR_ABS_HEIGHT / 2),
         (FLOOR_ABS_X[1] - FLOOR_ABS_X[0], ly, FLOOR_ABS_HEIGHT)),
    ]:
        g = f"<geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>"
        out.append(f'      <collision name="{name}"><pose>{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} 0 0 0</pose>{g}</collision>\n')

    # flat floor (walk-on area beyond the floor absorber); the take-off pad is in the world file
    fx0 = FLOOR_ABS_X[1]
    out.append(box("floor_flat", ((fx0 + x1) / 2, cy, -0.005), (x1 - fx0, ly, 0.01), "0.55 0.55 0.55 1"))

    # reflector (offset paraboloid shown as a tilted panel) on a support frame
    t = math.radians(REFLECTOR_TILT_DEG)
    rx, ry, rz = REFLECTOR_CENTRE
    # pitch +t turns the panel normal (+x) down towards the feed and the quiet zone
    out.append(box("reflector", (rx, ry, rz), (0.08, REFLECTOR_SIZE[0], REFLECTOR_SIZE[1]), "0.85 0.86 0.9 1",
                   collide=True, rpy=(0, t, 0)))
    out.append(box("reflector_frame", (rx - 0.5, ry, rz), (0.6, REFLECTOR_SIZE[0] * 0.6, REFLECTOR_SIZE[1] * 0.8),
                   "0.35 0.35 0.38 1", rpy=(0, t, 0)))

    # feed tower (absorber-covered) and feed horn aimed at the reflector
    fx, fy, fz = FEED_POS
    out.append(box("feed_tower", (fx, fy, (fz - 0.25) / 2), (0.6, 0.6, fz - 0.25), ABSORBER_RGB, collide=True))
    aim = math.atan2(rz - fz, rx - fx)        # angle of the reflector centre seen from the feed (x-z plane)
    out.append(box("feed_horn", (fx, fy, fz), (0.35, 0.22, 0.18), "0.9 0.6 0.2 1", rpy=(0, -aim, 0)))

    # quiet zone (no collision)
    qx, qy, qz = QZ_CENTRE
    out.append(f'      <visual name="quiet_zone"><pose>{qx} {qy} {qz} 0 0 0</pose>'
               f'<geometry><sphere><radius>{QZ_DIAMETER / 2}</radius></sphere></geometry>'
               f'{material("0.3 0.9 0.4 0.12")}<transparency>0.88</transparency></visual>\n')

    # AUT on a roll-over-azimuth positioner, aperture in the x = 0 plane facing +x
    aw, ah = AUT_SIZE
    ped_top = qz - ah / 2 - 0.35
    out.append(cylinder("positioner_pedestal", (AUT_POSITIONER_X, 0, ped_top / 2), 0.18, ped_top, "0.4 0.4 0.45 1", collide=True))
    out.append(box("positioner_head", (AUT_POSITIONER_X, 0, ped_top + 0.15), (0.5, 0.5, 0.3), "0.3 0.3 0.35 1", collide=True))
    out.append(box("aut_bracket", (AUT_POSITIONER_X / 2 - 0.05, 0, qz - ah / 2 - 0.1), (abs(AUT_POSITIONER_X), 0.12, 0.3), "0.3 0.3 0.35 1"))
    out.append(box("aut_aperture", (-0.025, 0, qz), (0.05, aw, ah), "0.6 0.4 0.9 1", collide=True))

    # ArUco marker plate above the aperture, facing +x like the aperture
    mx, my, mz = MARKER_CENTRE
    out.append(box("marker_backing", (mx - 0.015, my, mz), (0.02, MARKER_PLATE + 0.04, MARKER_PLATE + 0.04),
                   "0.3 0.3 0.35 1", collide=True))
    out.append(f'      <visual name="aut_marker"><pose>{mx:.3f} {my:.3f} {mz:.3f} 0 0 0</pose>'
               f'<geometry><box><size>0.004 {MARKER_PLATE} {MARKER_PLATE}</size></box></geometry>'
               f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>'
               f'<pbr><metal><albedo_map>model://catr_lab/materials/textures/aut_marker.png</albedo_map>'
               f'<roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual>\n')

    # UWB anchors on poles
    for i, (ax, ay, az) in enumerate(UWB_ANCHORS):
        out.append(cylinder(f"uwb_pole_{i}", (ax, ay, az / 2), 0.02, az, "0.5 0.5 0.5 1"))
        out.append(box(f"uwb_anchor_{i}", (ax, ay, az + 0.05), (0.08, 0.08, 0.1), "1 0.45 0.1 1"))

    sdf = f"""<?xml version="1.0"?>
<!-- generated by gen_catr_lab.py: {lx:.0f} x {ly:.0f} x {lz:.0f} m CATR lab, {n_pyr} absorber pyramids,
     {len(UWB_ANCHORS)} UWB anchors. Edit catr_layout.py and regenerate. -->
<sdf version="1.9">
  <model name="catr_lab">
    <static>true</static>
    <link name="link">
{''.join(out)}    </link>
  </model>
</sdf>
"""
    return sdf, n_pyr


WORLD = """<?xml version="1.0" ?>
<!--
  Compact antenna test range lab (generated by gen_catr_lab.py, layout in catr_layout.py).
  ENU: x along the range (reflector at -x), y across, z up. The AUT aperture lies in
  the x = 0 plane facing +x, centre at height {qz} m. The drone starts on the pad at
  (SPAWN_X, SPAWN_Y) facing the AUT; run_sim.sh fills these in from the antenna
  frequency (catr_layout.takeoff_point). No GNSS indoors: position comes from UWB.
-->
<sdf version="1.9">
  <world name="catr_lab">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <!--WIND_PLUGIN-->

    <scene>
      <ambient>0.55 0.55 0.6</ambient>
      <background>0.05 0.05 0.08</background>
      <grid>false</grid>
    </scene>

    <!-- only used as the EKF origin sent by uwb_extnav.py -->
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <!-- indoor lighting: directional lights without shadows pass through the ceiling -->
    <light type="directional" name="ceiling_lights">
      <cast_shadows>false</cast_shadows>
      <diffuse>0.75 0.75 0.75 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.2 0.1 -1</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <diffuse>0.3 0.3 0.32 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>-0.6 -0.4 -0.3</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 40</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>

    <include><uri>model://catr_lab</uri></include>

    <model name="pad">
      <static>true</static>
      <pose>SPAWN_X SPAWN_Y 0.002 0 0 0</pose>
      <link name="link">
        <visual name="pad"><geometry><box><size>2.0 2.0 0.004</size></box></geometry>
          <material><ambient>0.85 0.7 0.1 1</ambient><diffuse>0.85 0.7 0.1 1</diffuse></material></visual>
      </link>
    </model>

    <include>
      <uri>model://MODEL_NAME</uri>
      <pose degrees="true">SPAWN_X SPAWN_Y 0.40 0 0 180</pose>
    </include>
  </world>
</sdf>
"""


def main():
    mdir = os.path.join(HERE, "models", "catr_lab")
    os.makedirs(os.path.join(mdir, "meshes"), exist_ok=True)
    os.makedirs(os.path.join(mdir, "materials", "textures"), exist_ok=True)
    write_marker_png(os.path.join(mdir, "materials", "textures", "aut_marker.png"))
    sdf, n_pyr = build_model(os.path.join(mdir, "meshes"))
    with open(os.path.join(mdir, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(mdir, "model.config"), "w") as f:
        f.write("""<?xml version="1.0"?>
<model><name>catr_lab</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>Compact antenna test range lab with pyramidal RF absorber (generated)</description></model>
""")
    with open(os.path.join(HERE, "worlds", "catr_lab.sdf"), "w") as f:
        f.write(WORLD.format(qz=QZ_CENTRE[2]))
    print(f"catr_lab: {n_pyr} absorber pyramids, {len(UWB_ANCHORS)} UWB anchors -> {mdir}, worlds/catr_lab.sdf")


if __name__ == "__main__":
    main()
