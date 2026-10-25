#!/usr/bin/env python3
"""
Turns a flight_test.py CSV into the position-error verdict for each band.

Two error sources for the near-field data:
  tracking error  = true probe position - planned grid point   (flight dynamics, wind)
  knowledge error = logged position     - true probe position  (RTK-GNSS + attitude error)

If NF-FF uses the PLANNED grid, the phase error comes from the tracking error.
If NF-FF uses the LOGGED positions (report section 4), it comes from the knowledge
error only - but the samples are then off-grid in the plane.

Only the range axis (x, normal to the scan plane) maps straight into phase:
  dphi = 360 deg * dx / lambda. Criterion: |dx| <= lambda/50 (about 7 deg).

  python3 analyze_flight.py <csv> [--rtk-sigma 0.01] [--rtk-tau 60]
"""
import argparse
import math
import os

import numpy as np

BANDS = {"L 1.5 GHz": 1.5, "S 3 GHz": 3.0, "C 5.5 GHz": 5.5, "X 10 GHz": 10.0}


def load(path):
    import csv
    with open(path) as f:
        r = csv.DictReader(f)
        rows = list(r)
    cols = {k: [] for k in rows[0]}
    for row in rows:
        for k, v in row.items():
            cols[k].append(v)
    out = {k: (np.array(v, dtype=float) if k != "phase" else np.array(v)) for k, v in cols.items()}
    return out


def gauss_markov(t, sigma, tau, rng):
    """First-order Gauss-Markov process: slowly drifting RTK error, std sigma, correlation time tau."""
    e = np.zeros_like(t)
    e[0] = rng.normal(0, sigma)
    for i in range(1, len(t)):
        dt = max(t[i] - t[i - 1], 0)
        a = math.exp(-dt / tau)
        e[i] = a * e[i - 1] + rng.normal(0, sigma * math.sqrt(1 - a * a))
    return e


def stats(x):
    return dict(rms=float(np.sqrt(np.mean(x**2))), p95=float(np.percentile(np.abs(x), 95)),
                max=float(np.max(np.abs(x))), mean=float(np.mean(x)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--rtk-sigma", type=float, default=0.010, help="RTK horizontal 1-sigma (m)")
    ap.add_argument("--rtk-sigma-v", type=float, default=0.020, help="RTK vertical 1-sigma (m)")
    ap.add_argument("--rtk-tau", type=float, default=60.0, help="RTK error correlation time (s)")
    ap.add_argument("--att-sigma-deg", type=float, default=0.2, help="attitude error (deg) -> lever-arm error")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    d = load(a.csv)
    sel = np.array([p.startswith("line") or p == "hover" for p in d["phase"]])
    t = d["t"][sel]
    ex = d["probe_x"][sel] - d["ref_x"][sel]          # range axis (normal to the scan plane)
    ey = d["probe_y"][sel] - d["ref_y"][sel]          # along the raster line
    ez = d["probe_z"][sel] - d["ref_z"][sel]          # across lines (height)
    roll, pitch, yaw = d["roll_deg"][sel], d["pitch_deg"][sel], d["yaw_deg"][sel]

    rng = np.random.default_rng(a.seed)
    # knowledge error on the range axis: RTK horizontal + attitude error x 1.25 m lever arm
    lever = 1.25
    kx = gauss_markov(t, a.rtk_sigma, a.rtk_tau, rng) + \
        lever * np.radians(gauss_markov(t, a.att_sigma_deg, 5.0, rng))
    kz = gauss_markov(t, a.rtk_sigma_v, a.rtk_tau, rng)

    print("=" * 76)
    print(f" Flight: {os.path.basename(a.csv)}   samples {sel.sum()}   duration {t[-1] - t[0]:.0f} s")
    print("=" * 76)
    for name, e in [("range axis x (-> phase)", ex), ("along line y", ey), ("across lines z", ez)]:
        s = stats(e)
        print(f" tracking {name:24s} rms {s['rms']*1000:7.1f} mm  p95 {s['p95']*1000:7.1f} mm  "
              f"max {s['max']*1000:7.1f} mm  bias {s['mean']*1000:+6.1f} mm")
    print(f" attitude: roll rms {np.std(roll):.2f} deg, pitch mean {np.mean(pitch):+.2f} deg "
          f"(rms {np.std(pitch):.2f}), yaw std {np.std(np.unwrap(np.radians(yaw)))*57.3:.2f} deg")
    sk = stats(kx)
    print(f" knowledge range axis (RTK {a.rtk_sigma*1000:.0f} mm, tau {a.rtk_tau:.0f} s, "
          f"att {a.att_sigma_deg} deg)  rms {sk['rms']*1000:.1f} mm  p95 {sk['p95']*1000:.1f} mm")

    print("\n Phase error on the range axis (rms / p95), criterion lambda/50 = 7.2 deg")
    print(f" {'band':10s} {'lambda/50':>9s} | {'planned grid':>22s} | {'logged RTK positions':>22s}")
    verdict = {}
    for name, f in BANDS.items():
        lam = 0.299792458 / f
        st, sk = stats(ex - np.mean(ex)), stats(kx)     # constant bias = range offset, correctable
        ph_t = (360 * st["rms"] / lam, 360 * st["p95"] / lam)
        ph_k = (360 * sk["rms"] / lam, 360 * sk["p95"] / lam)
        ok_t = st["rms"] <= lam / 50
        ok_k = sk["rms"] <= lam / 50
        verdict[name] = (ok_t, ok_k)
        print(f" {name:10s} {lam/50*1000:7.1f}mm | {ph_t[0]:6.1f}/{ph_t[1]:6.1f} deg  {'OK  ' if ok_t else 'FAIL'}   "
              f"| {ph_k[0]:6.1f}/{ph_k[1]:6.1f} deg  {'OK  ' if ok_k else 'FAIL'}")

    # plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        # whole flight, measurement phase shaded
        T = d["t"] - d["t"][0]
        fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        for axi in ax:
            axi.fill_between(T, 0, 1, where=sel, transform=axi.get_xaxis_transform(),
                             color="tab:green", alpha=0.08, lw=0)
        ax[0].plot(T, d["probe_x"], lw=0.8, label="probe x (true)")
        ax[0].plot(T, d["ref_x"], lw=0.8, ls="--", label="probe x (reference)")
        ax[0].plot(T, d["probe_z"], lw=0.8, label="probe z (true)")
        ax[0].plot(T, d["ref_z"], lw=0.8, ls="--", label="probe z (reference)")
        ax[0].axhline(0, color="purple", lw=0.8)
        ax[0].text(T[0], 0.05, " AUT plane x=0", fontsize=7, color="purple")
        ax[0].set_ylabel("position [m]")
        ax[0].legend(fontsize=7, ncol=2)
        ax[1].plot(T, (d["probe_x"] - d["ref_x"]) * 1000, lw=0.8, label="range axis x")
        ax[1].plot(T, (d["probe_y"] - d["ref_y"]) * 1000, lw=0.8, label="along line y")
        ax[1].plot(T, (d["probe_z"] - d["ref_z"]) * 1000, lw=0.8, label="across lines z")
        ax[1].set_ylabel("tracking error [mm]")
        ax[1].legend(fontsize=7)
        tt = t - d["t"][0]
        ax[2].plot(tt, (ex - np.mean(ex)) * 1000, lw=0.8, label="tracking x, measurement phase (bias removed)")
        ax[2].plot(tt, kx * 1000, lw=0.8, label="knowledge x (RTK model)")
        for name, f in BANDS.items():
            lam = 0.299792458 / f
            ax[2].axhline(lam / 50 * 1000, ls=":", lw=0.7, color="gray")
            ax[2].text(tt[-1], lam / 50 * 1000, f" {name.split()[0]} λ/50", fontsize=7, va="center")
        ax[2].set_ylabel("range-axis error [mm]")
        ax[2].legend(fontsize=7)
        ax[3].plot(T, d["roll_deg"], lw=0.8, label="roll")
        ax[3].plot(T, d["pitch_deg"], lw=0.8, label="pitch")
        ax[3].set_ylabel("attitude [deg]")
        ax[3].set_xlabel("time [s]  (green = measurement phase)")
        ax[3].legend(fontsize=7)
        fig.suptitle(os.path.basename(a.csv))
        fig.tight_layout()
        png = os.path.splitext(a.csv)[0] + ".png"
        fig.savefig(png, dpi=120)
        print("\n plot:", png)
    except Exception as exc:  # plotting is optional
        print("plot skipped:", exc)


if __name__ == "__main__":
    main()
