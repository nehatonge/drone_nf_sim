#!/usr/bin/env python3
"""
Static compatibility check for the drone-based near-field measurement hexacopter.

Checks, from the architecture report (Oct 2026):
  1. Mass budget vs. the 15 kg all-up weight implied by "2.5 kg/motor at 50 % thrust"
  2. Thrust-to-weight and hover throttle (including the extra load on the front
     motors caused by the forward probe boom)
  3. Propeller clearance for the chosen wheelbase and prop size
  4. Centre-of-gravity shift from the boom, and the counter-moment needed
  5. Hover flight time
  6. Probe position tolerance per band vs. UWB positioning accuracy (indoor CATR lab)
  7. Scan-time for a planar raster at lambda/2 spacing (stop-and-go vs. on-the-fly)

Every mass marked "EST" is an estimate - replace with datasheet values when the
components are chosen. Standard library only.
"""
import math

G = 9.81

# ---------------------------------------------------------------- configuration
WHEELBASE_M = 1.40          # motor-to-opposite-motor (report: 1.2 - 1.6 m)
PROP_IN = 24                # report: 22 - 24 in
N_MOTORS = 6

# Motor thrust per report: 2.5 kg at 50 % throttle, ~4.85 kg peak (P60 / U8 Lite class)
THRUST_HOVER_REF_KG = 2.5
THRUST_MAX_KG = 4.85
HOVER_EFF_G_PER_W = 9.0     # EST, typical for 22-24 in props near hover; check motor datasheet

BATTERY_S = 12
BATTERY_AH = 22.0           # EST 12S 22 Ah pack
BATTERY_USABLE = 0.80

# (name, mass_kg, x_forward_m from frame centre)   x>0 = towards the probe
COMPONENTS = [
    ("Frame, plates, arms (1400 mm)",  3.00, 0.00),   # EST
    ("Landing gear",                   0.60, 0.00),   # EST
    ("Motors x6 (P60 class)",          6 * 0.37, 0.00),  # EST
    ("Props x6 (24 in CF)",            6 * 0.09, 0.00),  # EST
    ("ESCs (6 channels)",              0.60, 0.00),   # EST
    ("Battery 12S 22 Ah",              5.60, 0.00),   # EST - position is the CG trim knob
    ("Cube Orange + carrier",          0.10, 0.00),
    ("UWB tag + compass + mast",       0.20, 0.00),   # EST
    ("Telemetry radio",                0.05, 0.00),
    ("Jetson Orin Nano + case + BEC",  0.33, 0.00),   # EST
    ("Rangefinder + HereFlow",         0.07, 0.00),
    ("Wiring, PDB, misc",              0.50, 0.00),   # EST
    ("VNA / RF receiver (compact)",    0.40, 0.10),   # EST
    ("Probe mount rail + clamp",       0.20, 0.15),   # EST
    ("Probe boom (GFRP tube)",         0.30, 0.65),   # EST, centroid of a 1.3 m boom
    ("Phase-stable coax",              0.15, 0.65),   # EST
    ("S-band OEWG probe (WR-284)",     0.80, 1.25),   # EST, probe at boom tip
]

AUW_TARGET_KG = 15.0        # implied by the report (6 x 2.5 kg at 50 %)

# RF / scan
BANDS_GHZ = {"L (1.5 GHz)": 1.5, "S (3 GHz)": 3.0, "C (5.5 GHz)": 5.5, "X (10 GHz)": 10.0}
POS_HORIZ_M = 0.030         # UWB fix, 1-sigma horizontal (sim/uwb_extnav.py with 5 cm range noise)
POS_VERT_M = 0.045          # UWB fix, 1-sigma vertical
SCAN_W_M, SCAN_H_M = 3.0, 3.0   # EST planar scan area (depends on AUT size + truncation angle)
SCAN_SPEED_MPS = 0.5        # on-the-fly raster speed
STOP_GO_S_PER_POINT = 3.0   # settle + measure per point if hovering at each sample


def thrust_at(u):
    """Per-motor thrust (kg) at throttle u in [0,1], power law through the two report points."""
    a = math.log(THRUST_MAX_KG / THRUST_HOVER_REF_KG) / math.log(2.0)
    return THRUST_MAX_KG * u ** a


def throttle_for(t_kg):
    a = math.log(THRUST_MAX_KG / THRUST_HOVER_REF_KG) / math.log(2.0)
    return (t_kg / THRUST_MAX_KG) ** (1.0 / a)


def status(ok, warn=False):
    return "OK  " if ok else ("WARN" if warn else "FAIL")


def main():
    print("=" * 78)
    print(" Hexacopter compatibility check - drone-based near-field antenna measurement")
    print("=" * 78)

    # 1. mass
    m = sum(c[1] for c in COMPONENTS)
    print("\n[1] Mass budget")
    for name, mass, x in COMPONENTS:
        print(f"    {name:34s} {mass:6.2f} kg   x={x:+.2f} m")
    print(f"    {'TOTAL':34s} {m:6.2f} kg   (target from report: {AUW_TARGET_KG:.1f} kg)")
    print(f"    {status(m <= AUW_TARGET_KG, m <= 1.1 * AUW_TARGET_KG)} mass "
          f"{'within' if m <= AUW_TARGET_KG else 'over'} budget by {abs(m - AUW_TARGET_KG):.2f} kg")

    # 2. CG and per-motor load (hexa X: motors at 30, 90, 150, ... deg; front = +x)
    x_cg = sum(c[1] * c[2] for c in COMPONENTS) / m
    r = WHEELBASE_M / 2
    motor_x = [r * math.cos(math.radians(30 + 60 * i)) for i in range(6)]
    sx2 = sum(x * x for x in motor_x)
    loads = [m / 6 + m * x_cg * x / sx2 for x in motor_x]   # min-norm thrust split
    t_max_motor = max(loads)
    print("\n[2] Thrust and hover throttle")
    tw = N_MOTORS * THRUST_MAX_KG / m
    print(f"    Thrust-to-weight            {tw:.2f}   {status(tw >= 2.0, tw >= 1.7)} (want >= 2.0 for a heavy-lift survey drone)")
    u_avg = throttle_for(m / 6)
    u_front = throttle_for(t_max_motor)
    print(f"    Hover throttle (balanced)   {100 * u_avg:.0f} %")
    print(f"    Hover throttle (front pair) {100 * u_front:.0f} %   {status(u_front <= 0.6, u_front <= 0.7)} "
          f"(front motors carry {t_max_motor:.2f} kg vs {m / 6:.2f} kg mean)")
    print(f"    Control margin front motors {THRUST_MAX_KG / t_max_motor:.2f}x")

    # 3. prop clearance - in a hexagon the adjacent-motor distance equals the radius
    prop_d = PROP_IN * 0.0254
    adj = r
    gap = adj - prop_d
    print("\n[3] Propeller clearance")
    print(f"    Adjacent motor spacing      {adj * 1000:.0f} mm, prop diameter {prop_d * 1000:.0f} mm, tip gap {gap * 1000:.0f} mm")
    print(f"    {status(gap >= 0.1 * prop_d, gap > 0)} (rule of thumb: gap >= 10 % of prop diameter)")
    for p in (22, 24):
        print(f"      min wheelbase for {p} in props: {2 * 1.1 * p * 0.0254 * 1000:.0f} mm")
    prop_tip_r = r + prop_d / 2
    probe_x = max(c[2] for c in COMPONENTS)
    print(f"    Front prop tips reach x = {r * math.cos(math.radians(30)) + prop_d / 2:.2f} m; "
          f"probe at x = {probe_x:.2f} m  "
          f"{status(probe_x > r * math.cos(math.radians(30)) + prop_d / 2 + 0.2)}")

    # 4. CG
    print("\n[4] Centre of gravity")
    print(f"    CG shift forward            {x_cg * 100:.1f} cm")
    moment = m * x_cg
    batt = next(c for c in COMPONENTS if c[0].startswith("Battery"))
    print(f"    Forward moment              {moment:.2f} kg.m")
    print(f"    Battery must move aft by    {moment / batt[1] * 100:.0f} cm to re-centre "
          f"(or add {moment / 0.5:.1f} kg counterweight at 0.5 m aft)")
    print(f"    {status(abs(x_cg) < 0.02, abs(x_cg) < 0.05)} (want |CG| < 2 cm after trim)")

    # 5. flight time
    p_hover = m * 1000 / HOVER_EFF_G_PER_W
    e = BATTERY_S * 3.7 * BATTERY_AH * BATTERY_USABLE
    t_min = e / p_hover * 60
    print("\n[5] Endurance")
    print(f"    Hover power ~{p_hover:.0f} W, usable energy {e:.0f} Wh -> hover time ~{t_min:.0f} min")

    # 6. position tolerance
    print("\n[6] Probe position tolerance vs UWB positioning")
    print(f"    UWB (1-sigma): horizontal {POS_HORIZ_M * 1000:.0f} mm, vertical {POS_VERT_M * 1000:.0f} mm")
    print(f"    {'band':12s} {'lambda':>8s} {'lam/50':>8s} {'lam/100':>8s} {'phase err @UWB':>16s}  verdict")
    for name, f in BANDS_GHZ.items():
        lam = 3e8 / (f * 1e9)
        ph = 360 * POS_HORIZ_M / lam
        ok = POS_HORIZ_M <= lam / 50
        print(f"    {name:12s} {lam * 1000:6.0f}mm {lam / 50 * 1000:6.1f}mm {lam / 100 * 1000:6.1f}mm "
              f"{ph:12.0f} deg    {status(ok)}")
    print("    Note: the range axis (normal to the scan plane) is horizontal for a vertical AUT,")
    print("    so the horizontal position error drives the phase error. Hover-hold error adds to this")
    print("    and is what the Gazebo + ArduPilot SITL runs will measure.")

    # 7. scan time
    print("\n[7] Planar raster scan time (scan area "
          f"{SCAN_W_M:.1f} x {SCAN_H_M:.1f} m, lambda/2 spacing)")
    for name, f in BANDS_GHZ.items():
        dx = 3e8 / (f * 1e9) / 2
        nx = int(SCAN_W_M / dx) + 1
        ny = int(SCAN_H_M / dx) + 1
        n = nx * ny
        t_stop = n * STOP_GO_S_PER_POINT / 60
        t_fly = (ny * SCAN_W_M + SCAN_H_M) / SCAN_SPEED_MPS / 60
        print(f"    {name:12s} {nx:4d} x {ny:4d} = {n:7d} pts | stop-and-go {t_stop:7.0f} min | "
              f"on-the-fly @{SCAN_SPEED_MPS} m/s {t_fly:5.0f} min "
              f"{status(t_fly < 0.7 * t_min, t_fly < t_min)}")
    print("\n" + "=" * 78)


if __name__ == "__main__":
    main()
