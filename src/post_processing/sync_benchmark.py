#!/usr/bin/env python3
"""
Synchronization benchmark for atlas-scanner sessions.

Reads a rosbag and reports timing statistics between LiDAR, camera, and odometry.
Produces a per-frame CSV and a summary with pass/fail grades against thresholds
derived from sensor frame periods and walking-speed coloring error budget.

Usage:
  python3 sync_benchmark.py <session_dir> [--walk-speed 0.5] [--out report.json]
"""

import sys
import sqlite3
import argparse
import json
import subprocess
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------
# Mid360 publishes at ~10 Hz → 100 ms frame period.
# At 0.5 m/s walking speed, 1 ms sync error ≈ 0.5 mm positional error.
# We grade against three tiers:
#   GOOD   < 33 ms  (sub-frame, ~1.6 cm at 0.5 m/s)
#   OK     < 100 ms (one LiDAR frame, ~5 cm)
#   POOR   >= 100 ms

GOOD_MS  = 33.0
OK_MS    = 100.0


# ---------------------------------------------------------------------------
# Bag helpers (no ROS runtime needed — reads SQLite directly)
# ---------------------------------------------------------------------------

def open_db3(bag_dir: Path) -> sqlite3.Connection:
    db3 = sorted(bag_dir.glob("*.db3"))
    if db3:
        return sqlite3.connect(str(db3[0]))
    zstd = sorted(bag_dir.glob("*.db3.zstd"))
    if not zstd:
        raise FileNotFoundError(f"No .db3 in {bag_dir}")
    out = str(zstd[0]).replace(".zstd", "")
    r = subprocess.run(["zstd", "-d", str(zstd[0]), "-o", out, "-f"], capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"zstd decompress failed: {zstd[0]}")
    return sqlite3.connect(out)


def bag_stamps(con: sqlite3.Connection, fragment: str) -> np.ndarray:
    """Return sorted array of bag-receive timestamps (ns→s) for a topic."""
    topics = {r[0]: r[1] for r in con.execute("SELECT id, name FROM topics")}
    tid = next((t for t, n in topics.items() if fragment in n), None)
    if tid is None:
        return np.array([])
    rows = con.execute(
        "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()
    return np.array([r[0] / 1e9 for r in rows])


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def nearest_delta(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """For each stamp in `a`, find the minimum absolute difference to any stamp in `b`."""
    if len(b) == 0:
        return np.array([])
    idx = np.searchsorted(b, a)
    idx = np.clip(idx, 1, len(b) - 1)
    left  = np.abs(a - b[idx - 1])
    right = np.abs(a - b[idx])
    return np.minimum(left, right)


def stats(deltas_s: np.ndarray) -> dict:
    ms = deltas_s * 1000.0
    return {
        "mean_ms":   float(np.mean(ms)),
        "std_ms":    float(np.std(ms)),
        "median_ms": float(np.median(ms)),
        "p95_ms":    float(np.percentile(ms, 95)),
        "max_ms":    float(np.max(ms)),
        "n":         int(len(ms)),
    }


def grade(mean_ms: float) -> str:
    if mean_ms < GOOD_MS:
        return "GOOD"
    if mean_ms < OK_MS:
        return "OK"
    return "POOR"


def interpret(pair: str, s: dict, walk_speed: float) -> str:
    pos_err_cm = s["mean_ms"] * 1e-3 * walk_speed * 100
    g = grade(s["mean_ms"])
    return (
        f"{pair:25s}  mean={s['mean_ms']:6.1f}ms  std={s['std_ms']:5.1f}ms  "
        f"p95={s['p95_ms']:6.1f}ms  max={s['max_ms']:6.1f}ms  "
        f"pos_err≈{pos_err_cm:.1f}cm  [{g}]"
    )


# ---------------------------------------------------------------------------
# Inter-arrival jitter (clock drift / scheduling noise)
# ---------------------------------------------------------------------------

def jitter(stamps: np.ndarray) -> dict:
    if len(stamps) < 2:
        return {}
    gaps = np.diff(stamps) * 1000.0  # ms
    return {
        "mean_gap_ms":  float(np.mean(gaps)),
        "std_gap_ms":   float(np.std(gaps)),
        "max_gap_ms":   float(np.max(gaps)),
        "min_gap_ms":   float(np.min(gaps)),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def benchmark(session_dir: str, walk_speed: float, out_path: str | None):
    session = Path(session_dir)
    bag_dirs = sorted(session.glob("rosbag_*"))
    if not bag_dirs:
        print("✗ No rosbag_* directory found"); sys.exit(1)

    con = open_db3(bag_dirs[0])

    lidar  = bag_stamps(con, "/livox/lidar")
    camera = bag_stamps(con, "fisheye")
    odom   = bag_stamps(con, "odometry")
    imu    = bag_stamps(con, "imu")
    con.close()

    print(f"\n=== Sync Benchmark: {session.name} ===")
    print(f"  LiDAR  : {len(lidar)}  frames  span={lidar[-1]-lidar[0]:.1f}s" if len(lidar) else "  LiDAR  : NOT FOUND")
    print(f"  Camera : {len(camera)} frames  span={camera[-1]-camera[0]:.1f}s" if len(camera) else "  Camera : NOT FOUND")
    print(f"  Odom   : {len(odom)}  poses   span={odom[-1]-odom[0]:.1f}s"   if len(odom)   else "  Odom   : NOT FOUND")
    print(f"  IMU    : {len(imu)}   msgs    span={imu[-1]-imu[0]:.1f}s"     if len(imu)    else "  IMU    : NOT FOUND")

    report = {"session": str(session), "walk_speed_m_s": walk_speed, "pairs": {}}

    print(f"\n--- Cross-sensor timing (anchor = camera frame) ---")
    pairs = []
    if len(lidar)  and len(camera): pairs.append(("Camera↔LiDAR",  camera, lidar))
    if len(odom)   and len(camera): pairs.append(("Camera↔Odom",   camera, odom))
    if len(imu)    and len(camera): pairs.append(("Camera↔IMU",    camera, imu))
    if len(lidar)  and len(odom):   pairs.append(("LiDAR↔Odom",    lidar,  odom))

    for name, a, b in pairs:
        d = nearest_delta(a, b)
        s = stats(d)
        s["grade"] = grade(s["mean_ms"])
        report["pairs"][name] = s
        print(" ", interpret(name, s, walk_speed))

    print(f"\n--- Per-topic arrival jitter (scheduling noise) ---")
    for name, stamps in [("LiDAR", lidar), ("Camera", camera), ("Odom", odom), ("IMU", imu)]:
        if len(stamps) < 2:
            continue
        j = jitter(stamps)
        report.setdefault("jitter", {})[name] = j
        print(f"  {name:8s}  mean_gap={j['mean_gap_ms']:.1f}ms  "
              f"std={j['std_gap_ms']:.1f}ms  max_gap={j['max_gap_ms']:.1f}ms")

    # Overall grade: worst of all pairs
    grades = [v["grade"] for v in report["pairs"].values()]
    overall = "POOR" if "POOR" in grades else ("OK" if "OK" in grades else "GOOD")
    report["overall"] = overall

    print(f"\n  Thresholds: GOOD < {GOOD_MS:.0f}ms | OK < {OK_MS:.0f}ms | POOR >= {OK_MS:.0f}ms")
    print(f"  Walk speed: {walk_speed} m/s  (1ms error ≈ {walk_speed*0.1:.2f}cm positional error)")
    print(f"\n  Overall: {overall}\n")

    if out_path:
        Path(out_path).write_text(json.dumps(report, indent=2))
        print(f"  Report saved: {out_path}")

    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", help="Path to session directory (contains rosbag_*)")
    parser.add_argument("--walk-speed", type=float, default=0.5,
                        help="Walking speed in m/s for positional error estimate (default: 0.5)")
    parser.add_argument("--out", default=None,
                        help="Optional path to write JSON report")
    args = parser.parse_args()
    benchmark(args.session_dir, args.walk_speed, args.out)
