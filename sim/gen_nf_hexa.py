#!/usr/bin/env python3
"""
Generates the Gazebo (Harmonic) SDF model of the near-field measurement hexacopter
for ArduPilot SITL, from the same numbers used in analysis/sizing_check.py.

Model frame: x forward (towards the AUT / probe), y left, z up.
Motor numbering and spin follow ArduPilot Hexa-X (FRAME_CLASS=2, FRAME_TYPE=1):
  angle measured clockwise from the nose, CW/CCW seen from above
    M1  +90 CW   M2  -90 CCW   M3  -30 CW
    M4 +150 CCW  M5  +30 CCW   M6 -150 CW
  (Same spin directions as Fig. 3 of the report; only the numbering differs.)

Rotor thrust uses two LiftDrag blades per rotor, sized so that
  T = k w^2,   k = THRUST_MAX_N / W_MAX^2
and the blade drag gives the reaction torque Q at full thrust.
"""
import argparse
import math
import os

G = 9.81

# ----------------------------------------------------------------- airframe
WHEELBASE = 1.40
PROP_D = 24 * 0.0254
THRUST_MAX_N = 4.85 * G          # per motor
W_MAX = 576.0                    # rad/s at full throttle (~5500 rpm loaded, 170 KV on 12S) - EST
Q_MAX = 1.5                      # N.m prop torque at full thrust - EST, typical for 24 in
PROP_MASS = 0.09
ROTOR_Z = 0.12                   # rotor plane above base origin

# lumped masses on base_link: (name, mass, (x, y, z), (sx, sy, sz) box for inertia/visual)
BASE_ITEMS = [
    ("centre_plates", 2.00, (0, 0, 0.0), (0.30, 0.30, 0.06)),   # + 1.0 kg arms added below = 3.0 kg frame
    ("landing_gear",           0.60, (0, 0, -0.25), (0.50, 0.50, 0.30)),
    ("esc",                    0.60, (0, 0, -0.04), (0.12, 0.12, 0.03)),
    ("cube_orange",            0.10, (0, 0, 0.09), (0.09, 0.05, 0.04)),
    ("gnss_mast",              0.20, (-0.05, 0, 0.25), (0.08, 0.08, 0.20)),
    ("telemetry",              0.05, (-0.08, 0, 0.09), (0.05, 0.03, 0.02)),
    ("jetson",                 0.33, (0, 0, 0.04), (0.11, 0.10, 0.04)),
    ("rangefinder_flow",       0.07, (0, 0, -0.10), (0.05, 0.05, 0.03)),
    ("wiring_misc",            0.50, (0, 0, 0.0), (0.25, 0.25, 0.05)),
    ("vna",                    0.40, (0.10, 0, -0.10), (0.15, 0.10, 0.04)),
    ("probe_mount",            0.20, (0.15, 0, -0.13), (0.20, 0.06, 0.03)),
]
MOTOR_MASS = 0.37
BATTERY = dict(mass=5.60, size=(0.25, 0.14, 0.11), z=-0.10)

BOOM = dict(mass=0.45, x0=0.10, x1=1.20, z=-0.15, r=0.015)    # GFRP tube + coax
PROBE = dict(mass=0.80, x=1.25, z=-0.15, size=(0.15, 0.08, 0.05))  # WR-284 OEWG + adapter

# ArduPilot Hexa-X: (motor number, angle deg clockwise from nose, spin)
MOTORS = [(1, 90, "CW"), (2, -90, "CCW"), (3, -30, "CW"),
          (4, 150, "CCW"), (5, 30, "CCW"), (6, -150, "CW")]

# LiftDrag blade constants
CLA, A0 = 4.25, 0.3
CP_R = 0.7 * PROP_D / 2
RHO = 1.2041


def box_inertia(m, sx, sy, sz):
    return (m * (sy**2 + sz**2) / 12, m * (sx**2 + sz**2) / 12, m * (sx**2 + sy**2) / 12)


def lumped(items):
    """items: (m, (x,y,z), (ixx,iyy,izz) about own CG). Returns m, cg, inertia about cg (diag)."""
    m = sum(i[0] for i in items)
    cg = [sum(i[0] * i[1][k] for i in items) / m for k in range(3)]
    ixx = iyy = izz = 0.0
    for mi, p, (a, b, c) in items:
        dx, dy, dz = (p[0] - cg[0], p[1] - cg[1], p[2] - cg[2])
        ixx += a + mi * (dy**2 + dz**2)
        iyy += b + mi * (dx**2 + dz**2)
        izz += c + mi * (dx**2 + dy**2)
    return m, cg, (ixx, iyy, izz)


def motor_xy(angle_deg, r):
    a = math.radians(angle_deg)
    return r * math.cos(a), -r * math.sin(a)


def inertial(m, pose, I):
    return f"""      <inertial>
        <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 0</pose>
        <mass>{m:.4f}</mass>
        <inertia><ixx>{I[0]:.6f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{I[1]:.6f}</iyy><iyz>0</iyz><izz>{I[2]:.6f}</izz></inertia>
      </inertial>"""


def box_visual(name, pose, size, rgb, rpy=(0, 0, 0)):
    return f"""      <visual name="{name}">
        <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} {rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}</pose>
        <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
        <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>
      </visual>"""


def build(battery_x, name):
    r = WHEELBASE / 2
    k = THRUST_MAX_N / W_MAX**2
    area = k / (RHO * CLA * A0 * CP_R**2)
    # drag: per-blade D = Q / (2 r_cp) at full thrust; cd = cda * alpha -> cda = cla * D / L
    lift_blade = THRUST_MAX_N / 2
    drag_blade = Q_MAX / (2 * CP_R)
    cda = CLA * drag_blade / lift_blade

    # ---- base link lumped inertia
    items = []
    for _, m, p, s in BASE_ITEMS:
        items.append((m, p, box_inertia(m, *s)))
    for _, ang, _ in MOTORS:
        x, y = motor_xy(ang, r)
        items.append((MOTOR_MASS, (x, y, ROTOR_Z - 0.04), (1e-4, 1e-4, 1e-4)))
    # carbon arm tubes: 1.0 kg of the 3.0 kg frame, spread along the arms
    arm_m = 1.0 / 6
    for _, ang, _ in MOTORS:
        x, y = motor_xy(ang, r / 2)
        Irod = arm_m * r**2 / 12
        items.append((arm_m, (x, y, ROTOR_Z - 0.06), (Irod, Irod, Irod)))
    items.append((BATTERY["mass"], (battery_x, 0, BATTERY["z"]), box_inertia(BATTERY["mass"], *BATTERY["size"])))
    mb, cgb, Ib = lumped(items)

    boom_len = BOOM["x1"] - BOOM["x0"]
    boom_cx = (BOOM["x0"] + BOOM["x1"]) / 2
    Iboom = (BOOM["mass"] * BOOM["r"]**2 / 2, BOOM["mass"] * boom_len**2 / 12, BOOM["mass"] * boom_len**2 / 12)
    Iprobe = box_inertia(PROBE["mass"], *PROBE["size"])
    Iprop = (PROP_MASS * 0.04**2 / 12, PROP_MASS * PROP_D**2 / 12, PROP_MASS * PROP_D**2 / 12)

    total = mb + BOOM["mass"] + PROBE["mass"] + 6 * PROP_MASS
    cg_x = (mb * cgb[0] + BOOM["mass"] * boom_cx + PROBE["mass"] * PROBE["x"]) / total

    out = []
    out.append(f"""<?xml version="1.0"?>
<!-- generated by gen_nf_hexa.py: total mass {total:.2f} kg, CG x = {cg_x*100:+.1f} cm,
     rotor k = {k:.3e} N/(rad/s)^2, blade area = {area*1e4:.2f} cm^2, cda = {cda:.3f} -->
<sdf version="1.9">
  <model name="{name}">
    <pose>0 0 0.40 0 0 0</pose>
    <link name="base_link">
      <enable_wind>true</enable_wind>
{inertial(mb, cgb, Ib)}
      <collision name="gear_collision">
        <pose>0 0 -0.37 0 0 0</pose>
        <geometry><box><size>0.60 0.60 0.04</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <collision name="body_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.30 0.30 0.20</size></box></geometry>
      </collision>""")
    # visuals
    out.append(box_visual("deck_top", (0, 0, 0.07), (0.30, 0.26, 0.006), "0.2 0.2 0.2 1"))
    out.append(box_visual("deck_mid", (0, 0, 0.0), (0.32, 0.28, 0.006), "0.2 0.2 0.2 1"))
    out.append(box_visual("deck_bot", (0, 0, -0.07), (0.30, 0.26, 0.006), "0.2 0.2 0.2 1"))
    out.append(box_visual("cube_orange", (0, 0, 0.095), (0.09, 0.05, 0.04), "1 0.5 0.1 1"))
    out.append(box_visual("jetson", (0, 0, 0.03), (0.11, 0.10, 0.04), "0.1 0.7 0.3 1"))
    out.append(box_visual("battery", (battery_x, 0, BATTERY["z"]), BATTERY["size"], "0.6 0.6 0.6 1"))
    out.append(box_visual("vna", (0.10, 0, -0.10), (0.15, 0.10, 0.04), "0.2 0.4 1 1"))
    out.append(box_visual("gnss_mast", (-0.05, 0, 0.20), (0.012, 0.012, 0.22), "0.3 0.3 0.3 1"))
    out.append(box_visual("gnss", (-0.05, 0, 0.31), (0.08, 0.08, 0.02), "1 0.5 0.1 1"))
    for i, (_, ang, _) in enumerate(MOTORS):
        x, y = motor_xy(ang, r)
        a = math.atan2(y, x)
        out.append(box_visual(f"arm_{i}", (x / 2, y / 2, ROTOR_Z - 0.06), (r, 0.03, 0.03), "0.15 0.15 0.15 1", (0, 0, a)))
        out.append(f"""      <visual name="motor_{i}">
        <pose>{x:.4f} {y:.4f} {ROTOR_Z - 0.03:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.045</radius><length>0.05</length></cylinder></geometry>
        <material><ambient>0.9 0.4 0.1 1</ambient><diffuse>0.9 0.4 0.1 1</diffuse></material>
      </visual>""")
    for i, (lx, ly) in enumerate([(0.25, 0.3), (0.25, -0.3), (-0.25, 0.3), (-0.25, -0.3)]):
        out.append(box_visual(f"leg_{i}", (lx * 0.8, ly * 0.8, -0.22), (0.02, 0.02, 0.30), "0.3 0.3 0.3 1"))
    out.append(box_visual("skid_l", (0, 0.24, -0.37), (0.60, 0.025, 0.025), "0.3 0.3 0.3 1"))
    out.append(box_visual("skid_r", (0, -0.24, -0.37), (0.60, 0.025, 0.025), "0.3 0.3 0.3 1"))
    out.append("    </link>")

    # IMU (Cube Orange at the frame centre)
    out.append("""    <link name="imu_link">
      <pose>0 0 0.095 0 0 0</pose>
      <inertial><mass>0.01</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>imu_link</gz_frame_id>
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000.0</update_rate>
      </sensor>
    </link>
    <joint name="imu_joint" type="fixed"><parent>base_link</parent><child>imu_link</child></joint>""")

    # Boom and probe
    out.append(f"""    <link name="boom">
      <enable_wind>true</enable_wind>
{inertial(BOOM['mass'], (boom_cx, 0, BOOM['z']), Iboom)}
      <visual name="boom_visual">
        <pose>{boom_cx:.4f} 0 {BOOM['z']:.4f} 0 1.5708 0</pose>
        <geometry><cylinder><radius>{BOOM['r']}</radius><length>{boom_len:.4f}</length></cylinder></geometry>
        <material><ambient>0.9 0.9 0.8 1</ambient><diffuse>0.9 0.9 0.8 1</diffuse></material>
      </visual>
    </link>
    <joint name="boom_joint" type="fixed"><parent>base_link</parent><child>boom</child></joint>
    <link name="probe">
      <pose>{PROBE['x']:.4f} 0 {PROBE['z']:.4f} 0 0 0</pose>
      <enable_wind>true</enable_wind>
{inertial(PROBE['mass'], (0, 0, 0), Iprobe)}
      <visual name="probe_visual">
        <geometry><box><size>{PROBE['size'][0]} {PROBE['size'][1]} {PROBE['size'][2]}</size></box></geometry>
        <material><ambient>0.2 0.4 1 1</ambient><diffuse>0.2 0.4 1 1</diffuse></material>
      </visual>
      <collision name="probe_collision">
        <geometry><box><size>{PROBE['size'][0]} {PROBE['size'][1]} {PROBE['size'][2]}</size></box></geometry>
      </collision>
    </link>
    <joint name="probe_joint" type="fixed"><parent>boom</parent><child>probe</child></joint>""")

    # Rotors
    for i, (num, ang, spin) in enumerate(MOTORS):
        x, y = motor_xy(ang, r)
        rgb = "0.1 0.1 0.1 0.8" if spin == "CW" else "0.4 0.4 0.4 0.8"
        out.append(f"""    <link name="rotor_{i}">
      <pose>{x:.4f} {y:.4f} {ROTOR_Z:.4f} 0 0 0</pose>
{inertial(PROP_MASS, (0, 0, 0), Iprop)}
      <visual name="prop">
        <geometry><box><size>{PROP_D:.3f} 0.045 0.005</size></box></geometry>
        <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>
      </visual>
      <visual name="disc">
        <geometry><cylinder><radius>{PROP_D/2:.3f}</radius><length>0.001</length></cylinder></geometry>
        <material><ambient>0.9 0.5 0.3 0.15</ambient><diffuse>0.9 0.5 0.3 0.15</diffuse></material>
        <transparency>0.85</transparency>
      </visual>
    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent><child>rotor_{i}</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.0002</damping></dynamics></axis>
    </joint>""")

    out.append("""    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>false</static_publisher>
      <update_frequency>100</update_frequency>
    </plugin>""")

    for i, (num, ang, spin) in enumerate(MOTORS):
        sgn = 1 if spin == "CCW" else -1     # CCW from above = +z rotation
        for cpx in (CP_R, -CP_R):
            fwd_y = sgn * (1 if cpx > 0 else -1)
            out.append(f"""    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>{A0}</a0><alpha_stall>1.4</alpha_stall><cla>{CLA}</cla><cda>{cda:.4f}</cda><cma>0</cma>
      <cla_stall>-0.025</cla_stall><cda_stall>0</cda_stall><cma_stall>0</cma_stall>
      <area>{area:.6f}</area><air_density>{RHO}</air_density>
      <cp>{cpx:.4f} 0 0</cp><forward>0 {fwd_y} 0</forward><upward>0 0 1</upward>
      <link_name>rotor_{i}</link_name>
    </plugin>""")

    for i in range(6):
        out.append(f"""    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_{i}_joint</joint_name>
    </plugin>""")

    out.append("""    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_link::imu_sensor</imuName>""")
    for i, (num, ang, spin) in enumerate(MOTORS):
        sgn = 1 if spin == "CCW" else -1
        out.append(f"""      <control channel="{num - 1}">
        <jointName>rotor_{i}_joint</jointName>
        <useForce>1</useForce>
        <multiplier>{sgn * W_MAX:.1f}</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.5</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
        <i_max>0</i_max><i_min>0</i_min>
        <cmd_max>4.0</cmd_max><cmd_min>-4.0</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>""")
    out.append("""    </plugin>
  </model>
</sdf>
""")
    return "\n".join(out), total, cg_x, k, area, cda


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--battery-x", type=float, default=0.0, help="battery x position (m), negative = aft")
    ap.add_argument("--name", default="nf_hexa")
    ap.add_argument("--models-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "models"))
    a = ap.parse_args()
    sdf, total, cg_x, k, area, cda = build(a.battery_x, a.name)
    d = os.path.join(a.models_dir, a.name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(d, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model><name>{a.name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>Hexacopter for drone-based near-field antenna measurement (generated)</description></model>
""")
    hover_w = math.sqrt(total * G / 6 / k)
    print(f"{a.name}: mass {total:.2f} kg, CG x {cg_x*100:+.1f} cm, k {k:.3e}, "
          f"blade area {area*1e4:.2f} cm2, cda {cda:.3f}, hover {hover_w:.0f} rad/s "
          f"= {hover_w/W_MAX*100:.0f} % of w_max -> {d}")


if __name__ == "__main__":
    main()
