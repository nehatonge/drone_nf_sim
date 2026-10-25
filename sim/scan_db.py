#!/usr/bin/env python3
"""
SQLite store for grid scans. One file, readable from WSL and from Windows
(e.g. DB Browser for SQLite): data/nf_scans.db in the repo by default.

  scans         one row per scan: grid, frequency sweep, model, settings
  points        one row per grid point: target, estimated and true probe position
                (mean and std over the hold), timing, whether it arrived in tolerance
  measurements  one row per point and frequency: S21 averaged over the sweeps of the hold

Each point is committed on its own, so a crash or a battery swap loses at most the
point in progress; flight_test.py grid --resume <scan_id> continues from there.

  python3 scan_db.py                 list scans
  python3 scan_db.py <scan_id>       points of one scan with the centre-frequency result
"""
import json
import os
import sqlite3
import sys
import time

DEFAULT_DB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "nf_scans.db")

SCHEMA = """
CREATE TABLE IF NOT EXISTS scans (
  id              INTEGER PRIMARY KEY,
  started_utc     TEXT NOT NULL,
  model           TEXT,
  probe           TEXT,
  aut             TEXT,
  freq_start_hz   REAL,
  freq_stop_hz    REAL,
  freq_points     INTEGER,
  spacing_m       REAL,
  hold_s          REAL,
  n_points        INTEGER,
  grid_json       TEXT,
  settings_json   TEXT
);
CREATE TABLE IF NOT EXISTS points (
  id              INTEGER PRIMARY KEY,
  scan_id         INTEGER NOT NULL REFERENCES scans(id),
  idx             INTEGER NOT NULL,
  row             INTEGER,
  col             INTEGER,
  target_x REAL, target_y REAL, target_z REAL,
  est_x REAL, est_y REAL, est_z REAL,
  est_std_x REAL, est_std_y REAL, est_std_z REAL,
  true_x REAL, true_y REAL, true_z REAL,
  true_std_x REAL, true_std_y REAL, true_std_z REAL,
  arrived         INTEGER,
  t_arrive_s      REAL,
  t_hold_start    REAL,
  t_hold_end      REAL,
  n_sweeps        INTEGER,
  saved_utc       TEXT,
  UNIQUE (scan_id, idx)
);
CREATE TABLE IF NOT EXISTS measurements (
  point_id        INTEGER NOT NULL REFERENCES points(id),
  freq_hz         REAL NOT NULL,
  s21_re REAL, s21_im REAL,
  amp_db          REAL,
  phase_deg       REAL,
  amp_std_db      REAL,
  phase_std_deg   REAL,
  PRIMARY KEY (point_id, freq_hz)
);
"""


def utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class ScanDB:
    def __init__(self, path=DEFAULT_DB):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self.con = sqlite3.connect(path)
        self.con.executescript(SCHEMA)

    def new_scan(self, grid, settings, **meta):
        cols = ["started_utc", "grid_json", "settings_json", "n_points"] + list(meta)
        vals = [utc(), json.dumps(grid), json.dumps(settings), len(grid)] + list(meta.values())
        with self.con:
            cur = self.con.execute(f"INSERT INTO scans ({','.join(cols)}) VALUES ({','.join('?' * len(cols))})", vals)
        return cur.lastrowid

    def scan(self, scan_id):
        cur = self.con.execute("SELECT grid_json, settings_json FROM scans WHERE id = ?", (scan_id,))
        row = cur.fetchone()
        if row is None:
            raise KeyError(f"scan {scan_id} not in {self.path}")
        return json.loads(row[0]), json.loads(row[1])

    def done(self, scan_id):
        return {r[0] for r in self.con.execute("SELECT idx FROM points WHERE scan_id = ?", (scan_id,))}

    def save_point(self, scan_id, point, measurements):
        """point: dict of points columns; measurements: list of dicts of measurements columns."""
        point = dict(point, scan_id=scan_id, saved_utc=utc())
        with self.con:
            cur = self.con.execute(f"INSERT INTO points ({','.join(point)}) VALUES ({','.join('?' * len(point))})",
                                   list(point.values()))
            pid = cur.lastrowid
            for m in measurements:
                m = dict(m, point_id=pid)
                self.con.execute(f"INSERT INTO measurements ({','.join(m)}) VALUES ({','.join('?' * len(m))})",
                                 list(m.values()))
        return pid


def main():
    db = ScanDB()
    if len(sys.argv) == 1:
        print(f"{db.path}")
        for r in db.con.execute("""SELECT s.id, s.started_utc, s.model, s.n_points, COUNT(p.id),
                                   s.freq_start_hz, s.freq_stop_hz, s.freq_points, s.spacing_m
                                   FROM scans s LEFT JOIN points p ON p.scan_id = s.id GROUP BY s.id"""):
            print(f"  scan {r[0]:3d}  {r[1]}  {r[2]:16s}  {r[4]:5d}/{r[3]:<5d} points  "
                  f"{r[5] / 1e9:.2f}-{r[6] / 1e9:.2f} GHz x {r[7]}  spacing {r[8] * 100:.0f} cm")
        return
    sid = int(sys.argv[1])
    print(" idx  row col   target x     y     z  | est-true x   y   z (mm) | arrived |  S21 @ centre freq")
    for r in db.con.execute("""
        SELECT p.idx, p.row, p.col, p.target_x, p.target_y, p.target_z,
               p.est_x - p.true_x, p.est_y - p.true_y, p.est_z - p.true_z, p.arrived, m.amp_db, m.phase_deg, m.freq_hz
        FROM points p JOIN measurements m ON m.point_id = p.id
        WHERE p.scan_id = ? AND m.freq_hz = (SELECT freq_hz FROM measurements WHERE point_id = p.id
                                             ORDER BY ABS(freq_hz - (SELECT (freq_start_hz + freq_stop_hz) / 2
                                                                     FROM scans WHERE id = ?)) LIMIT 1)
        ORDER BY p.idx""", (sid, sid)):
        print(f" {r[0]:4d} {r[1]:3d} {r[2]:3d}  {r[3]:6.2f} {r[4]:6.2f} {r[5]:5.2f} | "
              f"{r[6] * 1000:5.0f} {r[7] * 1000:4.0f} {r[8] * 1000:4.0f}      |   {'yes' if r[9] else 'no '}   | "
              f"{r[10]:6.1f} dB {r[11]:7.1f} deg @ {r[12] / 1e9:.3f} GHz")


if __name__ == "__main__":
    main()
