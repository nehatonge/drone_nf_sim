#!/usr/bin/env python3
"""
Checks em_model.py against itself:
  1. far-field limit: the near-field summation at 200 m (beyond 2D^2/lambda = 67 m
     for the 1 x 1 m AUT at 5 GHz) must match the analytical aperture pattern
     in the E- and H-plane, down to -30 dB
  2. reference: 0 dB on boresight at 1 m, and the near-field amplitude/phase along
     boresight, for a feel of the numbers
  python3 em_selftest.py      (exit code 1 on failure)
"""
import sys

import numpy as np

from em_model import Aperture, NearFieldModel, catr_aut

F = 5.0e9
TOL_DB = 0.5


def plane_cut(model, plane, R=200.0):
    aut = model.aut
    th = np.radians(np.linspace(-40, 40, 161))
    other = aut.e if plane == "E" else aut.h
    u = np.cos(th)[:, None] * aut.n + np.sin(th)[:, None] * other
    p = aut.c + R * u
    pe = aut.e - (u @ aut.e)[:, None] * u          # probe co-polar, transverse to the line of sight
    s = model.measure(p, -u, pe, noisy=False)[:, 0]
    nf = 20 * np.log10(np.abs(s) / np.abs(s).max())
    ff = model.far_field(u, model.k[0])
    ff = 20 * np.log10(np.abs(ff) / np.abs(ff).max())
    sel = ff > -30
    return np.degrees(th), nf, ff, np.max(np.abs(nf[sel] - ff[sel]))


def main():
    aut = catr_aut()
    tiny = Aperture((0, 0, 0), (1, 0, 0), (0, 1, 0), a_h=0.001, b_e=0.001)   # near-isotropic probe
    model = NearFieldModel(aut, tiny, [F])
    print(f"AUT {aut.a} x {aut.b} m at {F / 1e9:g} GHz, {len(model.w)} Huygens elements")
    ok = True
    for plane in ("E", "H"):
        th, nf, ff, err = plane_cut(model, plane)
        i3 = th[np.argmin(np.abs(ff[th >= 0] + 3)) + np.count_nonzero(th < 0)]
        print(f"  {plane}-plane: max |near-field sum - analytic| = {err:.2f} dB above -30 dB "
              f"(-3 dB at {i3:.1f} deg)  {'OK' if err < TOL_DB else 'FAIL'}")
        ok &= err < TOL_DB

    from em_model import wr187_probe
    m = NearFieldModel(aut, wr187_probe(), [F])
    print("  WR-187 probe facing the AUT on boresight:")
    for d in (0.5, 1.0, 2.0, 4.0, 8.0):
        s = m.measure(aut.c + d * aut.n, -aut.n, aut.e, noisy=False)[0, 0]
        print(f"    {d:4.1f} m: {20 * np.log10(abs(s)):6.1f} dB  phase {np.degrees(np.angle(s)):7.1f} deg")
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
