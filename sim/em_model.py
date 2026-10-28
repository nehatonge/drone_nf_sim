"""
Electromagnetic model for the simulation (report Figure 4, step 5): what the probe
measures at a given pose.

AUT and probe are both rectangular apertures with a TE10-like distribution
(cosine along the H dimension, uniform along E) and linear polarisation along E:
  AUT    1 x 1 m aperture in the CATR quiet zone (catr_layout.py), 5 GHz
  probe  open-ended WR-187 waveguide (47.55 x 22.15 mm, 3.95-5.85 GHz)

Probe output = direct summation over Huygens elements of the AUT aperture:

  V = (j k / 4 pi) sum_n  w_n exp(-j k R_n) / R_n  Obl_A(u_n)  F_P(-u_n)  (p_A . p_P)

  w_n       aperture distribution x element area
  u_n, R_n  unit vector and distance, element -> probe
  Obl_A     (1 + cos theta) / 2 about the AUT normal
  F_P       probe receive pattern for a wave arriving from -u_n
  p_A, p_P  AUT and probe co-polar unit vectors (Ludwig-3 about each aperture's
            own axes), transverse to u_n

Valid from about one wavelength out. It keeps the near-field (Fresnel) behaviour
the NF-FF step needs and tends to the aperture far-field pattern at large range
(checked in em_selftest.py). Not modelled: multiple reflections between probe and
AUT, the drone body, and lab reflections (the absorber is assumed ideal).
"""
import numpy as np

C0 = 299792458.0


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


class Aperture:
    """Rectangular aperture: centre, normal (boresight), E direction, H size a, E size b (m)."""

    def __init__(self, centre, normal, e_dir, a_h, b_e):
        self.c = np.asarray(centre, dtype=float)
        self.n = _unit(normal)
        e = np.asarray(e_dir, dtype=float)
        self.e = _unit(e - np.dot(e, self.n) * self.n)
        self.h = np.cross(self.n, self.e)
        self.a, self.b = a_h, b_e

    def pattern(self, u, k, n=None, e=None):
        """Field pattern for unit directions u (..., 3). n, e override the orientation
        (arrays broadcastable to u) when the aperture moves, e.g. a probe on the drone."""
        n = self.n if n is None else n
        e = self.e if e is None else e
        h = np.cross(n, e)
        cos_t = np.sum(u * n, axis=-1)
        ue, uh = np.sum(u * e, axis=-1), np.sum(u * h, axis=-1)
        f_e = np.sinc(k * self.b * ue / (2 * np.pi))                   # uniform along E
        x = k * self.a * uh / np.pi
        near = np.abs(np.abs(x) - 1) < 1e-6
        f_h = np.where(near, np.pi / 4,
                       np.cos(np.pi * x / 2) / np.where(near, 1.0, 1 - x**2))  # cosine along H
        return (1 + cos_t) / 2 * f_e * f_h

    def copol(self, u, n=None, e=None):
        """Ludwig-3 co-polar unit vector (..., 3) for directions u: equals e on boresight,
        stays transverse to u and has unit length (a Huygens source has no extra cos theta)."""
        n = self.n if n is None else n
        e = self.e if e is None else e
        h = np.cross(n, e)
        c = np.sum(u * n, axis=-1)[..., None]
        ue, uh = np.sum(u * e, axis=-1)[..., None], np.sum(u * h, axis=-1)[..., None]
        c1 = np.maximum(1 + c, 1e-9)
        return -uh * ue / c1 * h + (1 - ue**2 / c1) * e - ue * n

    def elements(self, spacing):
        """Huygens element positions (N, 3) and weights (N,) with the TE10 distribution."""
        na = max(int(np.ceil(self.a / spacing)), 1)
        nb = max(int(np.ceil(self.b / spacing)), 1)
        sa = (np.arange(na) + 0.5) / na * self.a - self.a / 2
        sb = (np.arange(nb) + 0.5) / nb * self.b - self.b / 2
        A, B = np.meshgrid(sa, sb, indexing="ij")
        pos = self.c + A.reshape(-1, 1) * self.h + B.reshape(-1, 1) * self.e
        w = np.cos(np.pi * A / self.a).reshape(-1) * (self.a / na) * (self.b / nb)
        return pos, w


class NearFieldModel:
    """AUT -> probe coupling over a frequency sweep."""

    def __init__(self, aut, probe, freqs_hz, noise_db=-70.0, ref_dist=1.0, seed=None):
        self.aut, self.probe = aut, probe
        self.f = np.asarray(freqs_hz, dtype=float)
        self.k = 2 * np.pi * self.f / C0
        lam_min = C0 / self.f.max()
        self.pos, self.w = aut.elements(lam_min / 4)
        self.noise = 10 ** (noise_db / 20)
        self.rng = np.random.default_rng(seed)
        # reference: probe co-polar on the AUT boresight at ref_dist, facing the AUT
        p0 = aut.c + ref_dist * aut.n
        self.ref = np.abs(self._raw(p0[None], -aut.n[None], aut.e[None]))[0]     # (F,)

    def _raw(self, p, pn, pe, idx=None):
        """p, pn, pe: (M, 3) probe aperture centre, normal, E direction -> V (M, F),
        or (M, len(idx)) for the frequency indices idx."""
        idx = range(len(self.f)) if idx is None else idx
        out = np.empty((len(p), len(idx)), dtype=complex)
        for m in range(len(p)):
            n_p = _unit(pn[m])
            e_p = _unit(pe[m] - np.dot(pe[m], n_p) * n_p)
            d = p[m] - self.pos                                   # (N, 3)
            R = np.linalg.norm(d, axis=1)
            u = d / R[:, None]
            obl = (1 + u @ self.aut.n) / 2
            pol = np.sum(self.aut.copol(u) * self.probe.copol(-u, n_p, e_p), axis=1)
            for j, i in enumerate(idx):
                k = self.k[i]
                fp = self.probe.pattern(-u, k, n_p, e_p)
                out[m, j] = 1j * k / (4 * np.pi) * np.sum(self.w * np.exp(-1j * k * R) / R * obl * fp * pol)
        return out

    def measure(self, p, pn, pe, noisy=True, freq_index=None):
        """S21 (M, F), normalised to the boresight reference at ref_dist (0 dB there);
        (M, 1) for one frequency with freq_index (a VNA sweeping point by point)."""
        p, pn, pe = (np.atleast_2d(np.asarray(x, dtype=float)) for x in (p, pn, pe))
        idx = None if freq_index is None else [freq_index]
        s = self._raw(p, pn, pe, idx) / (self.ref if idx is None else self.ref[idx])
        if noisy and self.noise > 0:
            s += self.noise / np.sqrt(2) * (self.rng.standard_normal(s.shape) + 1j * self.rng.standard_normal(s.shape))
        return s

    def far_field(self, u, k=None):
        """AUT far-field pattern for unit directions u (analytical reference, step 7)."""
        return self.aut.pattern(np.asarray(u, dtype=float), self.k[len(self.k) // 2] if k is None else k)


def wr187_probe():
    """Open-ended WR-187 waveguide; orientation is set per sample from the drone pose."""
    return Aperture((0, 0, 0), (1, 0, 0), (0, 1, 0), a_h=0.04755, b_e=0.02215)


def catr_aut():
    """1 x 1 m AUT aperture in the x = 0 plane facing +x, E along y (catr_layout.py)."""
    from catr_layout import AUT_SIZE, QZ_CENTRE
    return Aperture(QZ_CENTRE, (1, 0, 0), (0, 1, 0), a_h=AUT_SIZE[1], b_e=AUT_SIZE[0])
