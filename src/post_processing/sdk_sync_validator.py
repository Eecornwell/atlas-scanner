#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Validates LiDAR-camera timing synchronisation for SDK-stitch
# continuous sessions where no camera topic exists in the rosbag.
#
# In SDK stitch mode the shutter time comes from:
#   1. fusion_scan_NNN/capture_N.shutter_event  — written at TakePhoto() return
#   2. fusion_scan_NNN/*.insp.capture_time       — same value, persisted with file
#
# The LiDAR and odometry timestamps are in the Livox hardware clock.
# The shutter timestamps are in the host system clock.
# This tool estimates the host→Livox offset from the bag, then reports:
#
#   • Shutter-to-LiDAR gap   — nearest LiDAR frame to each shutter time
#                               after clock correction.  Should be <50ms.
#   • Shutter-to-odom gap    — nearest odometry sample to each shutter.
#                               Drives pose interpolation error.
#   • LiDAR window coverage  — fraction of the ±lidar_window/2 window around
#                               each shutter that has LiDAR frames.
#   • Interval regularity    — std of actual inter-shot intervals vs configured.
#   • capture_time vs shutter_event consistency — detects mismatched sidecars.
#
# Usage:
#   python3 sdk_sync_validator.py <session_dir> [--lidar-window 0.6] [--walk-speed 0.5]

import sys
import os
import json
import struct
import sqlite3
import argparse
import subprocess
from pathlib import Path

import numpy as np

_ALLOWED_DATA = Path(os.path.expanduser("~/atlas_ws/data")).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _RCLPY = True
except ImportError:
    _RCLPY = False


# ---------------------------------------------------------------------------
# Bag helpers
# ---------------------------------------------------------------------------

def open_db3(bag_dir: Path):
    bag_dir = _safe_data(bag_dir)
    db3 = sorted(bag_dir.glob("*.db3"))
    if db3:
        return sqlite3.connect(str(_safe_data(db3[0]))), None
    zstd = sorted(bag_dir.glob("*.db3.zstd"))
    if not zstd:
        raise FileNotFoundError(f"No .db3 in {bag_dir}")
    safe_zstd = _safe_data(zstd[0])
    out = _safe_data(Path(str(safe_zstd).replace(".zstd", "")))
    r = subprocess.run(["zstd", "-d", str(safe_zstd), "-o", str(out), "-f"],
                       capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"zstd decompress failed: {safe_zstd}")
    return sqlite3.connect(str(out)), out


def _topic_map(con):
    return {r[0]: (r[1], r[2]) for r in con.execute("SELECT id, name, type FROM topics")}


def _bag_stamps_raw(con, fragment) -> np.ndarray:
    """Header timestamps for first topic matching fragment, without rclpy."""
    topics = {r[0]: r[1] for r in con.execute("SELECT id, name FROM topics")}
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
            sec  = struct.unpack_from('<i', bytes(data), 4)[0]
            nsec = struct.unpack_from('<I', bytes(data), 8)[0]
            stamps.append(sec + nsec / 1e9)
        except Exception:
            pass
    return np.sort(np.array(stamps))


def _host_to_livox_offset(con) -> float:
    """
    Estimate host-clock → Livox-hardware-clock offset from LiDAR header stamps
    vs bag receipt timestamps.  Median of (bag_ts - header_ts) over first 200
    frames; robust to scheduling jitter outliers.
    """
    if not _RCLPY:
        return 0.0
    topics = _topic_map(con)
    lid_tid = next((t for t, (n, _) in topics.items() if n == "/livox/lidar"), None)
    if lid_tid is None:
        return 0.0
    LidarType = get_message(topics[lid_tid][1])
    rows = con.execute(
        "SELECT data, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 200",
        (lid_tid,)
    ).fetchall()
    diffs = []
    for data, bag_ns in rows:
        try:
            msg = deserialize_message(bytes(data), LidarType)
            hdr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            diffs.append(bag_ns / 1e9 - hdr_t)
        except Exception:
            pass
    if not diffs:
        return 0.0
    measured = float(np.median(diffs))
    # When livox_time_sync (or PTP) has synced clocks, the measured value is
    # just DDS delivery latency, not a real clock offset. Zero it.
    if abs(measured) < 0.1:
        return 0.0
    return measured


# ---------------------------------------------------------------------------
# Shutter event readers
# ---------------------------------------------------------------------------

def read_shutter_events(session: Path) -> list[dict]:
    """
    Build timing records for each scan in the session.

    Primary source: .insp.capture_time sidecars written by the SDK daemon
    alongside each downloaded .insp file.  These survive reconstruct_from_bag
    (which wipes and rebuilds fusion_scan_*/ dirs but preserves .insp files).

    Fallback: capture_N.shutter_event files written during the live session.
    These are wiped by reconstruct_from_bag so are only present if the
    validator runs before post-processing.

    Returns list of dicts sorted by scan index, each with:
      scan_name, shot_index, shutter_t, capture_time (or None), insp_file (or None)
    """
    events = []
    seen_scans = set()

    # ── Primary: .insp.capture_time sidecars ─────────────────────────────
    for ct_path in sorted(session.glob("fusion_scan_*/*.insp.capture_time")):
        try:
            _safe_data(ct_path)
            # Format: "t_before" (old) or "t_before t_after" (new).
            # t_before = TakePhoto() call entry = shutter fires.
            capture_time = float(ct_path.read_text().strip().split()[0])
        except Exception:
            continue

        insp_path = Path(str(ct_path).replace(".capture_time", ""))
        insp_file = insp_path.name if insp_path.exists() else None
        scan_name = ct_path.parent.name

        # Derive shot index from scan directory number (fusion_scan_NNN → N-1)
        try:
            shot_idx = int(scan_name.split("_")[-1]) - 1
        except (IndexError, ValueError):
            shot_idx = len(events)

        seen_scans.add(scan_name)
        events.append({
            "scan_name":    scan_name,
            "shot_index":   shot_idx,
            "shutter_t":    capture_time,   # capture_time IS the shutter time
            "capture_time": capture_time,
            "insp_file":    insp_file,
            "source":       "capture_time",
        })

    # ── Fallback: capture_N.shutter_event files (pre-reconstruct sessions) ─
    for se in sorted(session.glob("fusion_scan_*/capture_*.shutter_event")):
        scan_name = se.parent.name
        if scan_name in seen_scans:
            continue  # already have a capture_time record for this scan
        try:
            _safe_data(se)
            shutter_t = float(se.read_text().strip())
        except Exception:
            continue

        try:
            shot_idx = int(se.stem.split("_")[1])
        except (IndexError, ValueError):
            shot_idx = len(events)

        insp_files = sorted(se.parent.glob("*.insp"))
        insp_file = insp_files[0].name if insp_files else None

        capture_time = None
        if insp_files:
            ct_p = Path(str(insp_files[0]) + ".capture_time")
            if ct_p.exists():
                try:
                    capture_time = float(ct_p.read_text().strip().split()[0])
                except Exception:
                    pass

        seen_scans.add(scan_name)
        events.append({
            "scan_name":    scan_name,
            "shot_index":   shot_idx,
            "shutter_t":    shutter_t,
            "capture_time": capture_time,
            "insp_file":    insp_file,
            "source":       "shutter_event",
        })

    events.sort(key=lambda e: e["shot_index"])
    return events


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def nearest_delta(target: float, stamps: np.ndarray) -> float:
    """Nearest distance from target to any stamp in the sorted array."""
    if len(stamps) == 0:
        return float("inf")
    idx = np.searchsorted(stamps, target)
    candidates = []
    if idx > 0:
        candidates.append(abs(target - stamps[idx - 1]))
    if idx < len(stamps):
        candidates.append(abs(target - stamps[idx]))
    return min(candidates)


def lidar_window_coverage(shutter_t: float, lidar_stamps: np.ndarray,
                           half_window: float) -> float:
    """
    Fraction of expected LiDAR frames within [shutter-half, shutter+half].
    Uses the median LiDAR rate to compute the expected frame count.
    """
    if len(lidar_stamps) < 2:
        return 0.0
    median_gap = float(np.median(np.diff(lidar_stamps)))
    expected_frames = max(1, int(round(2 * half_window / median_gap)))
    in_window = np.sum(
        (lidar_stamps >= shutter_t - half_window) &
        (lidar_stamps <= shutter_t + half_window)
    )
    return min(1.0, int(in_window) / expected_frames)


def grade(mean_ms: float) -> str:
    return "GOOD" if mean_ms < 33 else ("OK" if mean_ms < 100 else "POOR")


# ---------------------------------------------------------------------------
# Main validator
# ---------------------------------------------------------------------------

def validate(session_dir: str, lidar_window: float = 0.6, walk_speed: float = 0.5):
    try:
        session = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    # ── Find session bag ──────────────────────────────────────────────────
    bag_dirs = sorted(d for d in session.glob("rosbag_*") if "_imu" not in d.name)
    if not bag_dirs:
        print("✗ No rosbag_* directory found (continuous mode bag required)")
        sys.exit(1)

    con = tmp = None
    for bd in bag_dirs:
        try:
            con, tmp = open_db3(bd)
            bag_dir = bd
            break
        except Exception as e:
            print(f"  ⚠ Skipping {bd.name}: {e}")
    if con is None:
        print("✗ No readable bag found")
        sys.exit(1)

    # ── Extract timestamps from bag ───────────────────────────────────────
    lidar_stamps = _bag_stamps_raw(con, "/livox/lidar")
    odom_stamps  = _bag_stamps_raw(con, "/rko_lio/odometry")

    offset = _host_to_livox_offset(con)
    con.close()
    if tmp and tmp.exists():
        tmp.unlink()

    print(f"\n=== SDK Sync Validator: {session.name} ===")
    if not _RCLPY:
        print("  ⚠ rclpy not available — host→Livox offset cannot be computed")
        print("    Source your ROS workspace for full analysis")

    if len(lidar_stamps):
        lidar_rate = len(lidar_stamps) / (lidar_stamps[-1] - lidar_stamps[0])
        print(f"  LiDAR:  {len(lidar_stamps)} frames  "
              f"span={lidar_stamps[-1]-lidar_stamps[0]:.1f}s  rate={lidar_rate:.1f}Hz")
    else:
        print("  LiDAR:  NOT FOUND in bag")

    if len(odom_stamps):
        odom_rate = len(odom_stamps) / (odom_stamps[-1] - odom_stamps[0])
        print(f"  Odom:   {len(odom_stamps)} poses   "
              f"span={odom_stamps[-1]-odom_stamps[0]:.1f}s  rate={odom_rate:.1f}Hz")
    else:
        print("  Odom:   NOT FOUND in bag")

    print(f"  Host→Livox clock offset: {offset*1000:+.1f} ms"
          if _RCLPY else "  Host→Livox clock offset: unknown (no rclpy)")

    # ── Read shutter events ───────────────────────────────────────────────
    events = read_shutter_events(session)
    if not events:
        print("\n  ✗ No timing data found in fusion_scan_*/ dirs")
        print("    Expected: fusion_scan_NNN/*.insp.capture_time sidecars")
        print("    These are written by insta360_capture alongside each .insp file.")
        sys.exit(1)

    sources = set(e["source"] for e in events)
    print(f"  Shutter events: {len(events)} shots  "
          f"(source: {', '.join(sorted(sources))})")

    # ── Per-shot analysis ─────────────────────────────────────────────────
    half = lidar_window / 2.0

    lidar_deltas_ms   = []
    odom_deltas_ms    = []
    coverages         = []
    ct_vs_se_deltas   = []
    missing_insp      = []
    missing_ct        = []

    print(f"\n{'Shot':>4}  {'Scan':12}  {'Shutter(host)':>16}  "
          f"{'→LiDAR':>10}  {'→Odom':>8}  {'Coverage':>9}  {'ct-se':>8}  Status")
    print("─" * 90)

    for ev in events:
        shot     = ev["shot_index"]
        shutter  = ev["shutter_t"]
        shutter_livox = shutter - offset   # convert host → Livox clock

        lidar_d = nearest_delta(shutter_livox, lidar_stamps) * 1000 if len(lidar_stamps) else float("inf")
        odom_d  = nearest_delta(shutter_livox, odom_stamps)  * 1000 if len(odom_stamps)  else float("inf")
        cov     = lidar_window_coverage(shutter_livox, lidar_stamps, half)

        # capture_time vs shutter_event consistency
        ct_delta = None
        if ev["capture_time"] is not None:
            ct_delta = (ev["capture_time"] - shutter) * 1000  # ms, should be ~0
            ct_vs_se_deltas.append(abs(ct_delta))

        # Status flags
        flags = []
        if lidar_d > 100:
            flags.append("⚠LIDAR_GAP")
        if odom_d > 100:
            flags.append("⚠ODOM_GAP")
        if cov < 0.5:
            flags.append("⚠LOW_COVERAGE")
        if ct_delta is not None and abs(ct_delta) > 50:
            flags.append("⚠CT_MISMATCH")
        if ev["insp_file"] is None:
            flags.append("⚠NO_INSP")
            missing_insp.append(ev["scan_name"])
        if ev["capture_time"] is None and ev["insp_file"] is not None:
            flags.append("⚠NO_CT")
            missing_ct.append(ev["scan_name"])

        status = " ".join(flags) if flags else "✓"

        ct_str = f"{ct_delta:+.1f}ms" if ct_delta is not None else "  n/a  "
        print(f"  {shot:>3}  {ev['scan_name']:12}  {shutter:>16.3f}  "
              f"{lidar_d:>8.1f}ms  {odom_d:>6.1f}ms  {cov:>8.0%}  {ct_str:>8}  {status}")

        if lidar_d < 1e5:
            lidar_deltas_ms.append(lidar_d)
        if odom_d < 1e5:
            odom_deltas_ms.append(odom_d)
        coverages.append(cov)

    # ── Summary statistics ────────────────────────────────────────────────
    print("\n" + "─" * 90)
    print(f"\n  Summary ({len(events)} shots, walk_speed={walk_speed}m/s):\n")

    def _summary(label, vals_ms, walk_speed=walk_speed):
        if not vals_ms:
            print(f"  {label:35s}  no data")
            return
        a = np.array(vals_ms)
        pos_err = np.mean(a) * 1e-3 * walk_speed * 100
        g = grade(float(np.mean(a)))
        print(f"  {label:35s}  mean={np.mean(a):6.1f}ms  std={np.std(a):5.1f}  "
              f"p95={np.percentile(a,95):6.1f}  max={np.max(a):6.1f}  "
              f"pos_err≈{pos_err:.1f}cm  [{g}]")

    _summary("Shutter → nearest LiDAR frame", lidar_deltas_ms)
    _summary("Shutter → nearest odom pose",   odom_deltas_ms)

    if coverages:
        mean_cov = np.mean(coverages)
        low = sum(1 for c in coverages if c < 0.8)
        print(f"  {'LiDAR window coverage':35s}  mean={mean_cov:.0%}  "
              f"low-coverage shots={low}/{len(coverages)}")

    if ct_vs_se_deltas:
        a = np.array(ct_vs_se_deltas)
        print(f"  {'capture_time vs shutter_event':35s}  mean={np.mean(a):.1f}ms  "
              f"max={np.max(a):.1f}ms  "
              f"({'consistent ✓' if np.max(a) < 50 else 'MISMATCH ✗'})")

    # ── Interval regularity ───────────────────────────────────────────────
    if len(events) >= 2:
        shutters = np.array([e["shutter_t"] for e in events])
        intervals = np.diff(shutters)
        print(f"\n  Interval regularity:")
        print(f"    mean={np.mean(intervals):.2f}s  std={np.std(intervals):.3f}s  "
              f"min={np.min(intervals):.2f}s  max={np.max(intervals):.2f}s")
        if np.std(intervals) > 1.0:
            print(f"    ⚠ High interval jitter (std={np.std(intervals):.2f}s) "
                  f"— TakePhoto() blocking time varies")

    # ── Diagnostics ───────────────────────────────────────────────────────
    print(f"\n  Diagnostics:")
    print(f"    Host→Livox offset applied: {offset*1000:+.2f} ms" if _RCLPY else
          f"    Host→Livox offset: unknown — source ROS workspace")
    if missing_insp:
        print(f"    ⚠ Missing .insp files: {', '.join(missing_insp)}")
    if missing_ct:
        print(f"    ⚠ Missing .capture_time sidecars: {', '.join(missing_ct)}")

    print(f"\n  Thresholds: GOOD < 33ms | OK < 100ms | POOR ≥ 100ms")
    print(f"  1ms timing error ≈ {walk_speed*0.1:.2f}cm positional error at {walk_speed}m/s\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Validate LiDAR-camera sync for SDK-stitch continuous sessions"
    )
    parser.add_argument("session_dir",
                        help="Path to session directory (contains rosbag_* and fusion_scan_*/)")
    parser.add_argument("--lidar-window", type=float, default=0.6,
                        help="LiDAR accumulation half-window in seconds (default: 0.6)")
    parser.add_argument("--walk-speed",   type=float, default=0.5,
                        help="Scanner walk speed m/s for position error estimate (default: 0.5)")
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    validate(args.session_dir, args.lidar_window, args.walk_speed)
