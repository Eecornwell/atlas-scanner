#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Reads a session rosbag and reports cross-sensor timing statistics
# (LiDAR, camera, odometry, IMU) using message header timestamps to quantify
# synchronisation quality.  Reports both raw stream gaps AND the interpolation
# residual that actually drives pose error in the merged point cloud.
"""
Synchronization benchmark for atlas-scanner sessions.

Metrics reported
----------------
1. Raw stream gaps  — nearest-neighbour delta between topic header stamps.
   Tells you how well the hardware clocks are aligned.

2. Interpolation residual — for each camera frame (scan centre), the distance
   to the bracketing odometry samples used by interp_pose().  This is the
   quantity that directly drives pose error in the merged cloud: if the two
   bracketing odom samples are t0 and t1, the worst-case interpolation error
   is bounded by (t1-t0)/2 × angular_velocity, not by the nearest-neighbour
   gap.  At 10 Hz odom the bracket half-width is ~50 ms regardless of camera
   rate.

3. Odom coverage — fraction of the LiDAR/camera window that has odometry.
   Gaps here mean extrapolation (clamped to nearest) rather than interpolation.

4. Per-topic arrival jitter — std of inter-message gaps, a proxy for
   scheduling noise and DDS queue pressure.

Usage:
  python3 sync_benchmark.py <session_dir> [--walk-speed 0.5] [--out report.json]
"""

import sys
import sqlite3
import argparse
import json
import struct
import subprocess
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# Thresholds (mean of nearest-neighbour delta, ms)
# ---------------------------------------------------------------------------
GOOD_MS = 33.0   # sub-frame at 10 Hz LiDAR
OK_MS   = 100.0  # one full LiDAR frame period


# ---------------------------------------------------------------------------
# Bag helpers
# ---------------------------------------------------------------------------

def open_db3(bag_dir: Path):
    """Returns (connection, tmp_path_or_None)."""
    db3 = sorted(bag_dir.glob("*.db3"))
    if db3:
        return sqlite3.connect(str(db3[0])), None
    zstd = sorted(bag_dir.glob("*.db3.zstd"))
    if not zstd:
        raise FileNotFoundError(f"No .db3 in {bag_dir}")
    out = Path(str(zstd[0]).replace(".zstd", ""))
    r = subprocess.run(["zstd", "-d", str(zstd[0]), "-o", str(out), "-f"],
                       capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"zstd decompress failed: {zstd[0]}")
    return sqlite3.connect(str(out)), out


def _header_stamp(data: bytes) -> float:
    """Extract ROS2 CDR header stamp (sec+nanosec at bytes 4-12) as float seconds."""
    sec  = struct.unpack_from('<i', data, 4)[0]
    nsec = struct.unpack_from('<I', data, 8)[0]
    return sec + nsec / 1e9


def bag_stamps(con: sqlite3.Connection, fragment: str) -> np.ndarray:
    """Return sorted array of header timestamps (s) for the first topic matching fragment."""
    topics = {r[0]: r[1] for r in con.execute("SELECT id, name FROM topics")}
    # Prefer exact match, then substring
    tid = next((t for t, n in topics.items() if n == fragment), None)
    if tid is None:
        tid = next((t for t, n in topics.items() if fragment in n), None)
    if tid is None:
        return np.array([])
    rows = con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()
    stamps = []
    for (data,) in rows:
        try:
            stamps.append(_header_stamp(bytes(data)))
        except Exception:
            pass
    return np.sort(np.array(stamps))


def topic_names(con: sqlite3.Connection) -> list:
    return [r[0] for r in con.execute("SELECT name FROM topics")]


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def nearest_delta(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """For each stamp in `a` within b's window, nearest distance to any stamp in `b`."""
    if len(b) == 0:
        return np.array([])
    a = a[(a >= b[0]) & (a <= b[-1])]
    if len(a) == 0:
        return np.array([])
    idx = np.searchsorted(b, a)
    idx = np.clip(idx, 1, len(b) - 1)
    return np.minimum(np.abs(a - b[idx - 1]), np.abs(a - b[idx]))


def interp_bracket_width(camera: np.ndarray, odom: np.ndarray) -> np.ndarray:
    """
    For each camera stamp, return the width of the odometry bracket used by
    interp_pose() — i.e. (t_right - t_left) for the two surrounding odom samples.
    Camera stamps outside the odom window are excluded (those use clamped extrapolation).
    Returns half-widths in seconds (the actual interpolation uncertainty is
    proportional to this × angular/linear velocity).
    """
    if len(odom) < 2:
        return np.array([])
    cam_in = camera[(camera >= odom[0]) & (camera <= odom[-1])]
    if len(cam_in) == 0:
        return np.array([])
    idx = np.searchsorted(odom, cam_in)
    idx = np.clip(idx, 1, len(odom) - 1)
    return (odom[idx] - odom[idx - 1]) / 2.0  # half-bracket = max interpolation error bound


def stats(arr: np.ndarray, scale=1000.0) -> dict:
    ms = arr * scale
    return {
        "mean":   float(np.mean(ms)),
        "std":    float(np.std(ms)),
        "median": float(np.median(ms)),
        "p95":    float(np.percentile(ms, 95)),
        "max":    float(np.max(ms)),
        "n":      int(len(ms)),
    }


def grade(mean_ms: float) -> str:
    return "GOOD" if mean_ms < GOOD_MS else ("OK" if mean_ms < OK_MS else "POOR")


def fmt_stat(label: str, s: dict, walk_speed: float, unit="ms") -> str:
    pos_err_cm = s["mean"] * 1e-3 * walk_speed * 100
    g = grade(s["mean"])
    return (f"  {label:30s}  mean={s['mean']:6.1f}{unit}  std={s['std']:5.1f}  "
            f"p95={s['p95']:6.1f}  max={s['max']:6.1f}  "
            f"pos_err≈{pos_err_cm:.1f}cm  [{g}]")


def jitter(stamps: np.ndarray) -> dict:
    if len(stamps) < 2:
        return {}
    gaps = np.diff(stamps) * 1000.0
    return {
        "mean_gap_ms": float(np.mean(gaps)),
        "std_gap_ms":  float(np.std(gaps)),
        "max_gap_ms":  float(np.max(gaps)),
        "min_gap_ms":  float(np.min(gaps)),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def benchmark(session_dir: str, walk_speed: float, out_path: str | None):
    session = Path(session_dir)

    # Find bag: continuous mode has rosbag_* at session root;
    # stationary mode passes the scan dir so rosbag_* is one level down.
    bag_dirs = sorted(session.glob("rosbag_*"))
    if not bag_dirs:
        bag_dirs = sorted(session.glob("*/rosbag_*"))
    if not bag_dirs:
        print("✗ No rosbag_* directory found"); sys.exit(1)

    con = tmp_db3 = None
    for bag_dir in bag_dirs:
        try:
            con, tmp_db3 = open_db3(bag_dir)
            break
        except Exception as e:
            print(f"  ⚠ Skipping {bag_dir.name}: {e}")
    if con is None:
        print("✗ No readable bag found"); sys.exit(1)

    names = topic_names(con)

    # Prefer raw odometry over buffered so timestamps are LIO-native
    odom_topic = next((n for n in names if n == "/rko_lio/odometry"), None)
    if odom_topic is None:
        odom_topic = next((n for n in names if "odometry" in n), None)

    lidar  = bag_stamps(con, "/livox/lidar")
    camera = bag_stamps(con, "fisheye")
    odom   = bag_stamps(con, odom_topic) if odom_topic else np.array([])
    imu    = bag_stamps(con, "/livox/imu")
    con.close()
    if tmp_db3 and tmp_db3.exists():
        tmp_db3.unlink()

    print(f"\n=== Sync Benchmark: {session.name} ===")
    def _span(arr, label):
        if len(arr):
            print(f"  {label:8s}: {len(arr):4d} msgs  span={arr[-1]-arr[0]:.1f}s  "
                  f"rate={len(arr)/(arr[-1]-arr[0]):.1f}Hz" if arr[-1] > arr[0] else
                  f"  {label:8s}: {len(arr):4d} msgs")
        else:
            print(f"  {label:8s}: NOT FOUND")
    _span(lidar,  "LiDAR")
    _span(camera, "Camera")
    _span(odom,   f"Odom ({odom_topic.split('/')[-1] if odom_topic else '?'})")
    _span(imu,    "IMU")

    report = {
        "session": str(session),
        "walk_speed_m_s": walk_speed,
        "odom_topic": odom_topic,
        "pairs": {},
        "interpolation": {},
        "jitter": {},
    }

    # ── 1. Raw stream nearest-neighbour gaps ──────────────────────────────
    print(f"\n--- Raw stream gaps (nearest-neighbour, anchor = camera) ---")
    raw_pairs = []
    if len(lidar)  and len(camera): raw_pairs.append(("Camera↔LiDAR",  camera, lidar))
    if len(odom)   and len(camera): raw_pairs.append(("Camera↔Odom",   camera, odom))
    if len(imu)    and len(camera): raw_pairs.append(("Camera↔IMU",    camera, imu))
    if len(lidar)  and len(odom):   raw_pairs.append(("LiDAR↔Odom",    lidar,  odom))

    for name, a, b in raw_pairs:
        d = nearest_delta(a, b)
        if len(d) == 0:
            gap_s = max(b[0] - a[-1], a[0] - b[-1], 0.0)
            print(f"  {name:30s}  [NO OVERLAP — gap {gap_s*1000:.0f}ms]")
            continue
        s = stats(d)
        s["grade"] = grade(s["mean"])
        report["pairs"][name] = s
        print(fmt_stat(name, s, walk_speed))

    # ── 2. Interpolation residual (what actually drives pose error) ───────
    print(f"\n--- Interpolation residual (pose accuracy at scan centres) ---")
    print(f"  Each camera frame is a scan centre; interp_pose() brackets it")
    print(f"  between two odom samples.  Half-bracket width bounds pose error.")

    if len(camera) and len(odom) >= 2:
        hw = interp_bracket_width(camera, odom)
        if len(hw):
            s = stats(hw)
            report["interpolation"]["odom_half_bracket_ms"] = s
            pos_err_cm = s["mean"] * 1e-3 * walk_speed * 100
            g = grade(s["mean"])
            print(f"  {'Odom bracket half-width':30s}  mean={s['mean']:6.1f}ms  std={s['std']:5.1f}  "
                  f"p95={s['p95']:6.1f}  max={s['max']:6.1f}  "
                  f"pos_err≈{pos_err_cm:.1f}cm  [{g}]")

            # Fraction of camera frames that fall outside the odom window (extrapolated)
            n_extrap = int(np.sum((camera < odom[0]) | (camera > odom[-1])))
            extrap_frac = n_extrap / len(camera) if len(camera) else 0.0
            report["interpolation"]["extrapolated_frames"] = n_extrap
            report["interpolation"]["extrapolated_frac"] = extrap_frac
            if n_extrap:
                print(f"  ⚠ {n_extrap}/{len(camera)} camera frames ({extrap_frac*100:.0f}%) "
                      f"outside odom window — pose clamped to nearest, not interpolated")
        else:
            print("  No camera frames within odom window")
    else:
        print("  Insufficient odom data for interpolation analysis")

    # ── 3. Odom coverage ──────────────────────────────────────────────────
    if len(odom) and len(lidar):
        lidar_span = lidar[-1] - lidar[0]
        overlap = min(odom[-1], lidar[-1]) - max(odom[0], lidar[0])
        coverage = max(0.0, overlap) / lidar_span if lidar_span > 0 else 0.0
        start_delay = max(0.0, odom[0] - lidar[0])
        report["odom_coverage"] = {"fraction": coverage, "start_delay_s": start_delay}
        print(f"\n--- Odometry coverage ---")
        print(f"  Coverage: {coverage*100:.0f}% of LiDAR window")
        if start_delay > 0.5:
            print(f"  ⚠ Odom starts {start_delay:.2f}s after LiDAR (RKO-LIO init delay)")

    # ── 4. Per-topic arrival jitter ───────────────────────────────────────
    print(f"\n--- Per-topic arrival jitter (scheduling noise) ---")
    for label, stamps in [("LiDAR", lidar), ("Camera", camera),
                           ("Odom", odom), ("IMU", imu)]:
        if len(stamps) < 2:
            continue
        j = jitter(stamps)
        report["jitter"][label] = j
        flag = "  ⚠ HIGH" if j["std_gap_ms"] > 20.0 else ""
        print(f"  {label:8s}  mean_gap={j['mean_gap_ms']:.1f}ms  "
              f"std={j['std_gap_ms']:.1f}ms  max_gap={j['max_gap_ms']:.1f}ms{flag}")

    # ── 5. Overall grade ──────────────────────────────────────────────────
    grades = [v["grade"] for v in report["pairs"].values()]
    interp_grade = report["interpolation"].get("odom_half_bracket_ms", {}).get("grade", "")
    if interp_grade:
        grades.append(interp_grade)
    overall = "POOR" if "POOR" in grades else ("OK" if "OK" in grades else "GOOD")
    report["overall"] = overall

    print(f"\n  Thresholds: GOOD < {GOOD_MS:.0f}ms | OK < {OK_MS:.0f}ms | POOR >= {OK_MS:.0f}ms")
    print(f"  Walk speed: {walk_speed} m/s  (1ms ≈ {walk_speed*0.1:.2f}cm positional error)")
    print(f"\n  Overall: {overall}\n")

    if out_path:
        Path(out_path).write_text(json.dumps(report, indent=2))
        print(f"  Report saved: {out_path}")

    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir")
    parser.add_argument("--walk-speed", type=float, default=0.5)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    benchmark(args.session_dir, args.walk_speed, args.out)
