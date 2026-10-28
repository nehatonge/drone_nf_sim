"""
Simulated VNA on the drone (stand-in for a LibreVNA) with a TTL-style trigger handshake.

Lines, as seen from the Jetson:
  TRIGGER  output -> VNA   rising edge starts one sweep
  READY    VNA -> input    HIGH = idle / sweep complete, LOW = sweeping

Handshake for one sweep:
  Jetson TRIGGER HIGH -> VNA READY LOW (acknowledge, within ack_delay)
  -> VNA sweeps the frequency points one after another over sweep_time, each at the
     probe pose of that instant (em_model.py) -> READY HIGH (done) -> Jetson reads the
     sweep and sets TRIGGER LOW

Fault injection, so the retry logic in flight_test.py is exercised:
  miss_prob  the VNA ignores the trigger edge (READY never goes LOW)
  hang_prob  the VNA starts but never finishes (READY stays LOW)
  bad_prob   the sweep finishes but the data is invalid (status "error")

All timing is in simulation time (Gazebo clock).
"""
import threading
import time

import numpy as np


class SimVNA:
    def __init__(self, model, truth, probe_axes, sweep_time=0.5, ack_delay=0.005,
                 miss_prob=0.02, hang_prob=0.01, bad_prob=0.01, seed=None):
        """model: em_model.NearFieldModel; truth: flight_test.Truth;
        probe_axes(q) -> (normal, e) of the probe in the world for drone attitude q."""
        self.model, self.truth, self.probe_axes = model, truth, probe_axes
        self.sweep_time, self.ack_delay = sweep_time, ack_delay
        self.miss_prob, self.hang_prob, self.bad_prob = miss_prob, hang_prob, bad_prob
        self.rng = np.random.default_rng(seed)
        self.lock = threading.Lock()
        self._trigger = False
        self._ready = True
        self._data = None             # (s21 (F,), status, t_start, t_end)
        self._edge = False
        self._abort = False
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    # ---- Jetson side
    def set_trigger(self, level):
        with self.lock:
            if level and not self._trigger:
                self._edge = True
            self._trigger = level

    @property
    def ready(self):
        with self.lock:
            return self._ready

    def read(self):
        with self.lock:
            d, self._data = self._data, None
            return d

    def abort(self):
        """Reset the VNA (front-panel / SCPI abort) after a timeout."""
        with self.lock:
            self._abort = True

    def close(self):
        self.running = False

    # ---- VNA side
    def _wait_sim(self, t):
        while self.running and self.truth.now() < t:
            with self.lock:
                if self._abort:
                    return False
            time.sleep(0.001)
        return self.running

    def _run(self):
        while self.running:
            with self.lock:
                edge, self._edge = self._edge, False
                if self._abort:
                    self._abort, self._ready, self._data = False, True, None
            if not edge:
                time.sleep(0.001)
                continue
            r = self.rng.random()
            if r < self.miss_prob:
                continue                                   # trigger edge lost
            t0 = self.truth.now()
            if not self._wait_sim(t0 + self.ack_delay):
                continue
            with self.lock:
                self._ready = False                        # acknowledge: sweeping
            hang = r < self.miss_prob + self.hang_prob
            bad = not hang and r < self.miss_prob + self.hang_prob + self.bad_prob
            nf = len(self.model.f)
            s21 = np.empty(nf, dtype=complex)
            t_start = self.truth.now()
            ok = True
            for i in range(nf):                            # one frequency point after another
                if not self._wait_sim(t_start + i * self.sweep_time / nf):
                    ok = False
                    break
                _, p, q, _ = self.truth.probe()
                n, e = self.probe_axes(q)
                s21[i] = self.model.measure(p, n, e, freq_index=i)[0, 0]
            if not ok:
                continue
            if hang:
                while self.running:                        # stuck until abort()
                    with self.lock:
                        if self._abort:
                            break
                    time.sleep(0.005)
                continue
            if not self._wait_sim(t_start + self.sweep_time):
                continue
            if bad:
                s21[self.rng.integers(nf)] = np.nan
            with self.lock:
                self._data = (s21, "error" if bad else "ok", t_start, self.truth.now())
                self._ready = True                         # done
