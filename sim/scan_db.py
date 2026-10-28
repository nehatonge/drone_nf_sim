#!/usr/bin/env python3
"""
SQLite store for grid scans. One file, readable from WSL and from Windows
(e.g. DB Browser for SQLite): data/nf_scans.db in the repo by default.

  scans         one row per scan: grid, frequency sweep, model, settings
  points        one row per grid point (saved once it is measured successfully): target,
                estimated and true probe position (mean and std over the hold), timing
  measurements  one row per point and frequency: S21 averaged over the sweeps of the hold
  flights       one row per flight: take-off point, AUT detection, outcome, log folder
  triggers      every trigger attempt: handshake times, position error before/during the
                sweep, result (ok / not_stable / vna_not_ready / missed_trigger /
                sweep_timeout / moved_during_sweep / bad_data)
  events        flight events (take-off, detection, alignment, retries, landing, errors)
  telemetry     flight log at 10 Hz: true and reference probe position, attitude, phase
  files         archived flight log files (ArduPilot .BIN, CSV, plots, UWB/Gazebo logs)
                copied to data/flights/<flight_id>/

Each point is committed on its own, so a crash or a battery swap loses at most the
point in progress; flight_test.py <test> --resume <scan_id> continues from there.

  python3 scan_db.py                 list scans and flights
  python3 scan_db.py <scan_id>       points of one scan with the centre-frequency result
  python3 scan_db.py triggers <flight_id>   trigger attempts of one flight
  python3 scan_db.py --db FILE ...   any of the above on another database file
"""
import csv
import json
import os
import shutil
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
CREATE TABLE IF NOT EXISTS flights (
  id              INTEGER PRIMARY KEY,
  scan_id         INTEGER REFERENCES scans(id),
  test            TEXT,
  started_utc     TEXT,
  ended_utc       TEXT,
  status          TEXT,
  reason          TEXT,
  model           TEXT,
  aut_ghz         REAL,
  n_lambda        REAL,
  scan_dist_m     REAL,
  takeoff_x REAL, takeoff_y REAL,
  aut_est_x REAL, aut_est_y REAL, aut_est_z REAL,
  aut_est_nx REAL, aut_est_ny REAL,
  aut_err_mm      REAL,
  aut_normal_err_deg REAL,
  points_done     INTEGER,
  triggers_total  INTEGER,
  triggers_failed INTEGER,
  files_dir       TEXT,
  args_json       TEXT
);
CREATE TABLE IF NOT EXISTS triggers (
  id              INTEGER PRIMARY KEY,
  flight_id       INTEGER REFERENCES flights(id),
  scan_id         INTEGER,
  point_idx       INTEGER,
  attempt         INTEGER,
  t_trigger       REAL,
  t_ack           REAL,
  t_done          REAL,
  result          TEXT,
  err_before_mm   REAL,
  err_max_during_mm REAL,
  steady_s        REAL
);
CREATE TABLE IF NOT EXISTS events (
  id              INTEGER PRIMARY KEY,
  flight_id       INTEGER REFERENCES flights(id),
  t               REAL,
  utc             TEXT,
  kind            TEXT,
  text            TEXT
);
CREATE TABLE IF NOT EXISTS telemetry (
  flight_id       INTEGER REFERENCES flights(id),
  t REAL, phase TEXT,
  probe_x REAL, probe_y REAL, probe_z REAL,
  ref_x REAL, ref_y REAL, ref_z REAL,
  roll_deg REAL, pitch_deg REAL, yaw_deg REAL
);
CREATE TABLE IF NOT EXISTS files (
  flight_id       INTEGER REFERENCES flights(id),
  kind            TEXT,
  path            TEXT,
  bytes           INTEGER
);
"""
# columns added after the first version; added to an existing database on open
MIGRATIONS = [("points", "flight_id", "INTEGER"), ("points", "attempts", "INTEGER"),
              ("points", "grid_u", "REAL"), ("points", "grid_v", "REAL")]


def utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class ScanDB:
    def __init__(self, path=DEFAULT_DB):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self.flights_dir = os.path.join(os.path.dirname(os.path.abspath(path)), "flights")   # next to this database
        self.con = sqlite3.connect(path)
        self.con.executescript(SCHEMA)
        for table, col, typ in MIGRATIONS:
            cols = {r[1] for r in self.con.execute(f"PRAGMA table_info({table})")}
            if col not in cols:
                self.con.execute(f"ALTER TABLE {table} ADD COLUMN {col} {typ}")
        self.con.commit()

    # ---- flights, events, triggers, logs
    def _insert(self, table, row):
        with self.con:
            cur = self.con.execute(f"INSERT INTO {table} ({','.join(row)}) VALUES ({','.join('?' * len(row))})",
                                   list(row.values()))
        return cur.lastrowid

    def new_flight(self, **row):
        return self._insert("flights", dict(row, started_utc=utc(), status="flying"))

    def update_flight(self, flight_id, **row):
        with self.con:
            self.con.execute(f"UPDATE flights SET {','.join(k + ' = ?' for k in row)} WHERE id = ?",
                             list(row.values()) + [flight_id])

    def event(self, flight_id, t, kind, text=""):
        self._insert("events", dict(flight_id=flight_id, t=t, utc=utc(), kind=kind, text=text))

    def trigger(self, **row):
        return self._insert("triggers", row)

    def import_telemetry(self, flight_id, csv_path, every=10):
        """Flight CSV from flight_test.Recorder (~100 Hz) -> telemetry table at ~10 Hz."""
        cols = ["t", "phase", "probe_x", "probe_y", "probe_z", "ref_x", "ref_y", "ref_z",
                "roll_deg", "pitch_deg", "yaw_deg"]
        with open(csv_path) as f:
            rows = [[r[c] for c in cols] for i, r in enumerate(csv.DictReader(f)) if i % every == 0]
        with self.con:
            self.con.executemany(f"INSERT INTO telemetry (flight_id,{','.join(cols)}) VALUES (?{',?' * len(cols)})",
                                 [[flight_id] + r for r in rows])
        return len(rows)

    def archive(self, flight_id, paths):
        """Copy flight log files to flights/<flight_id>/ next to the database and list them in `files`."""
        d = os.path.join(self.flights_dir, f"{flight_id:04d}")
        os.makedirs(d, exist_ok=True)
        for kind, src in paths:
            if not src or not os.path.exists(src):
                continue
            dst = os.path.join(d, os.path.basename(src))
            shutil.copy2(src, dst)
            self._insert("files", dict(flight_id=flight_id, kind=kind, path=dst, bytes=os.path.getsize(dst)))
        self.update_flight(flight_id, files_dir=d)
        return d

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
    path = DEFAULT_DB
    if len(sys.argv) > 2 and sys.argv[1] == "--db":           # scan_db.py --db FILE ...
        path = sys.argv[2]
        del sys.argv[1:3]
    db = ScanDB(path)
    if len(sys.argv) == 1:
        print(f"{db.path}")
        for r in db.con.execute("""SELECT s.id, s.started_utc, s.model, s.n_points, COUNT(p.id),
                                   s.freq_start_hz, s.freq_stop_hz, s.freq_points, s.spacing_m
                                   FROM scans s LEFT JOIN points p ON p.scan_id = s.id GROUP BY s.id"""):
            print(f"  scan {r[0]:3d}  {r[1]}  {r[2]:16s}  {r[4]:5d}/{r[3]:<5d} points  "
                  f"{r[5] / 1e9:.2f}-{r[6] / 1e9:.2f} GHz x {r[7]}  spacing {r[8] * 100:.0f} cm")
        for r in db.con.execute("""SELECT id, scan_id, test, started_utc, status, points_done, triggers_total,
                                   triggers_failed, aut_err_mm, aut_normal_err_deg, reason FROM flights"""):
            det = f"AUT found ±{r[8]:.0f} mm / {r[9]:.1f} deg" if r[8] is not None else "no AUT detection"
            print(f"  flight {r[0]:3d}  scan {r[1]}  {r[2]}  {r[3]}  {r[4]:9s}  {r[5] or 0} points, "
                  f"{r[6] or 0} triggers ({r[7] or 0} failed), {det}  {r[10] or ''}")
        return
    if sys.argv[1] == "attach":                    # scan_db.py attach <flight_id> <kind> <file>
        print(db.archive(int(sys.argv[2]), [(sys.argv[3], sys.argv[4])]))
        return
    if sys.argv[1] == "triggers":
        fid = int(sys.argv[2])
        print(" point attempt  t_trigger  ack(ms) sweep(s)  err before/during (mm)  result")
        for r in db.con.execute("""SELECT point_idx, attempt, t_trigger, t_ack, t_done, err_before_mm,
                                   err_max_during_mm, result FROM triggers WHERE flight_id = ? ORDER BY id""", (fid,)):
            ack = f"{(r[3] - r[2]) * 1000:6.1f}" if r[3] else "   -  "
            sw = f"{r[4] - r[3]:6.2f}" if r[4] and r[3] else "   -  "
            print(f" {r[0]:5d} {r[1]:7d} {r[2]:10.2f}  {ack}  {sw}      {r[5] or 0:5.1f} / {r[6] or 0:5.1f}       {r[7]}")
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
