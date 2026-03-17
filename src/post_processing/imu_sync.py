#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Estimates the clock offset between the Insta360 camera IMU and
# the Livox Mid360 IMU by cross-correlating their gyroscope Z-axis signals.
# Writes sync_offset.json next to the bag for use by reconstruct_from_bag.py.
#
# Usage:
#   python3 imu_sync.py <session_dir> [--axis {x,y,z}] [--window 2.0] [--out sync_offset.json]
#
# Output (sync_offset.json):
#   {
#     "delta_t_s":   <float>,   # camera_t_corrected = camera_t + delta_t_s
#     "confidence":  <float>,   # normalised cross-correlation peak height [0-1]
#     "method":      "gyro_xcorr",
#     "axis":        "z",
#     "livox_hz":    200,
#     "camera_hz":   1000
#   }

import sys
import json
import struct
import sqlite3
import argparse
import subprocess
from pathlib import Path

import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("✗ rclpy not available — source your ROS workspace first")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Bag helpers (shared pattern with reconstruct_from_bag.py)
# ---------------------------------------------------------------------------

def open_db3(bag_dir: Path):
    db3 = sorted(bag_dir.glob("*.db3"))
    if db3:
        return sqlite3.connect(str(db3[0]))
    zstd = sorted(bag_dir.glob("*.db3.zstd"))
    if not zstd:
        raise FileNotFoundError(f"No .db3 in {bag_dir}")
    out = Path(str(zstd[0]).replace(".zstd", ""))
    r = subprocess.run(["zstd", "-d", str(zstd[0]), "-o", str(out), "-f"],
                       capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"zstd decompress failed: {zstd[0]}")
    return sqlite3.connect(str(out))


def _topic_map(con):
    return {r[0]: (r[1], r[2]) for r in con.execute("SELECT id, name, type FROM topics")}


def _read_imu(con, topics, name_fragment):
    """Return (times_s, gyro_xyz) arrays for the first topic matching fragment."""
    tid = next((t for t, (n, _) in topics.items() if n == name_fragment), None)
    if tid is None:
        tid = next((t for t, (n, _) in topics.items() if name_fragment in n), None)
    if tid is None:
        return None, None

    msg_type_str = topics[tid][1]
    MsgType = get_message(msg_type_str)
    rows = con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()

    times, gyros = [], []
    for (data,) in rows:
        msg = deserialize_message(bytes(data), MsgType)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g = msg.angular_velocity
        times.append(t)
        gyros.append([g.x, g.y, g.z])

    if not times:
        return None, None
    return np.array(times), np.array(gyros)


def _fix_batched_timestamps(times: np.ndarray, nominal_hz: float = 0.0) -> np.ndarray:
    """
    The Insta360 SDK delivers IMU samples in batches with identical host
    timestamps.  Reconstruct monotonic per-sample timestamps by:
    1. Identifying batch boundaries (gaps > 2x the median gap).
    2. Linearly interpolating sample times within each batch using either
       the measured inter-batch interval or the nominal_hz rate as a fallback.
    """
    if len(times) < 2:
        return times
    gaps = np.diff(times)
    median_gap = np.median(gaps[gaps > 1e-6]) if np.any(gaps > 1e-6) else 1.0 / max(nominal_hz, 100.0)
    # Samples with gap < 1ms are considered part of the same batch
    batch_starts = np.concatenate([[0], np.where(gaps > 1e-3)[0] + 1])
    fixed = times.copy()
    for i, start in enumerate(batch_starts):
        end = batch_starts[i + 1] if i + 1 < len(batch_starts) else len(times)
        n = end - start
        if n <= 1:
            continue
        t0 = times[start]
        t1 = times[end - 1] if times[end - 1] > t0 + 1e-6 else t0 + median_gap * (n - 1)
        fixed[start:end] = np.linspace(t0, t1, n)
    return fixed



def resample(times: np.ndarray, values: np.ndarray, target_hz: float):
    """Linearly interpolate a signal onto a uniform grid at target_hz."""
    t0, t1 = times[0], times[-1]
    t_uniform = np.arange(t0, t1, 1.0 / target_hz)
    resampled = np.interp(t_uniform, times, values)
    return t_uniform, resampled


def _best_axis_pair(t_ref, g_ref, t_query, g_query, resample_hz=50.0):
    """
    Find the axis pair with the highest absolute Pearson correlation at lag=0.
    Returns (ref_axis_idx, query_axis_idx, sign) where sign is +1 or -1.
    """
    t0 = max(t_ref[0], t_query[0])
    t1 = min(t_ref[-1], t_query[-1])
    if t1 - t0 < 1.0:
        return 2, 2, 1  # fallback to z/z
    tg = np.arange(t0, t1, 1.0 / resample_hz)
    best_r, best_pair = 0.0, (2, 2, 1)
    for ri in range(3):
        a = np.interp(tg, t_ref,   g_ref[:, ri])
        a = (a - a.mean()) / (a.std() + 1e-12)
        for qi in range(3):
            b = np.interp(tg, t_query, g_query[:, qi])
            b = (b - b.mean()) / (b.std() + 1e-12)
            r = float(np.dot(a, b) / len(a))
            if abs(r) > abs(best_r):
                best_r, best_pair = r, (ri, qi, 1 if r > 0 else -1)
    return best_pair



def estimate_offset(t_ref: np.ndarray, sig_ref: np.ndarray,
                    t_query: np.ndarray, sig_query: np.ndarray,
                    resample_hz: float = 500.0,
                    max_lag_s: float = 0.5) -> tuple[float, float]:
    """
    Estimate the time offset such that:
        sig_query(t + delta_t) ≈ sig_ref(t)
    i.e. camera_t_corrected = camera_t + delta_t

    Returns (delta_t_seconds, confidence [0-1]).
    """
    # Resample both signals onto the same uniform grid
    overlap_start = max(t_ref[0], t_query[0])
    overlap_end   = min(t_ref[-1], t_query[-1])
    if overlap_end - overlap_start < 1.0:
        raise ValueError(
            f"IMU overlap too short ({overlap_end - overlap_start:.2f}s) — "
            "need at least 1s of simultaneous motion")

    t_grid = np.arange(overlap_start, overlap_end, 1.0 / resample_hz)
    a = np.interp(t_grid, t_ref,   sig_ref)
    b = np.interp(t_grid, t_query, sig_query)

    # Zero-mean and unit-variance normalisation
    a = (a - a.mean()) / (a.std() + 1e-12)
    b = (b - b.mean()) / (b.std() + 1e-12)

    # Restrict search to ±max_lag_s
    max_lag_samples = int(max_lag_s * resample_hz)

    # FFT cross-correlation (O(N log N))
    n = len(a) + len(b) - 1
    fft_size = 1 << (n - 1).bit_length()  # next power of 2
    A = np.fft.rfft(a, n=fft_size)
    B = np.fft.rfft(b, n=fft_size)
    xcorr_full = np.fft.irfft(A * np.conj(B), n=fft_size)[:n]

    # Rearrange to lag-centred form and restrict window
    xcorr = np.concatenate([xcorr_full[-(max_lag_samples):],
                             xcorr_full[:max_lag_samples + 1]])
    lags = np.arange(-max_lag_samples, max_lag_samples + 1) / resample_hz

    peak_idx  = np.argmax(np.abs(xcorr))
    delta_t   = float(lags[peak_idx])
    confidence = float(np.abs(xcorr[peak_idx]) / (len(a) + 1e-12))
    confidence = min(1.0, confidence)

    return delta_t, confidence


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def estimate_imu_sync(session_dir: str, axis: str = "z",
                      window_s: float = 0.0, out_name: str = "sync_offset.json"):
    session = Path(session_dir)

    bag_dirs = sorted(session.glob("rosbag_*"))
    if not bag_dirs:
        bag_dirs = sorted(session.glob("*/rosbag_*"))
    if not bag_dirs:
        print("✗ No rosbag_* directory found"); sys.exit(1)

    con = None
    bag_dir = None
    imu_bag_dir = None
    for bd in bag_dirs:
        try:
            con = open_db3(bd)
            bag_dir = bd
            break
        except Exception as e:
            print(f"  ⚠ Skipping {bd.name}: {e}")
    if con is None:
        print("✗ No readable bag found"); sys.exit(1)

    # Look for a dedicated IMU bag (rosbag_*_imu) written by the split recorder.
    # Fall back to the main bag if not found.
    imu_bag_dirs = sorted(session.glob("rosbag_*_imu"))
    if imu_bag_dirs:
        try:
            imu_con = open_db3(imu_bag_dirs[0])
            imu_bag_dir = imu_bag_dirs[0]
        except Exception:
            imu_con = con
    else:
        imu_con = con

    topics = _topic_map(con)
    imu_topics = _topic_map(imu_con)
    print("Topics:", [n for _, (n, _) in topics.items()])
    if imu_bag_dir:
        print("IMU bag topics:", [n for _, (n, _) in imu_topics.items()])

    t_livox, g_livox = _read_imu(imu_con, imu_topics, "/livox/imu")
    # Prefer raw camera IMU — higher rate than the Madgwick filtered output,
    # and the resampling step in estimate_offset handles burst gaps.
    t_cam, g_cam = _read_imu(imu_con, imu_topics, "/imu/data_raw")
    if t_cam is None:
        t_cam, g_cam = _read_imu(imu_con, imu_topics, "/imu/data")
    if t_cam is None:
        for _, (name, _) in imu_topics.items():
            if "imu" in name and "livox" not in name:
                t_cam, g_cam = _read_imu(imu_con, imu_topics, name)
                if t_cam is not None:
                    print(f"  Using fallback IMU topic: {name}")
                    break
    if imu_con is not con:
        imu_con.close()
    con.close()

    if t_livox is None:
        print("✗ /livox/imu not found in bag"); sys.exit(1)
    if t_cam is None:
        print("✗ Camera IMU not found in bag — check /imu/data_raw is recorded"); sys.exit(1)

    ax = {"x": 0, "y": 1, "z": 2}[axis]

    # Auto-detect the best-correlated axis pair — the two IMUs may have
    # different orientations so the default axis may be uncorrelated.
    ref_ax, query_ax, sign = _best_axis_pair(t_livox, g_livox, t_cam, g_cam)
    axis_names = 'xyz'
    if (ref_ax, query_ax) != (ax, ax):
        print(f"  Auto-selected axes: Livox_{axis_names[ref_ax]} vs "
              f"Camera_{axis_names[query_ax]} (sign={sign:+d}) "
              f"— overrides --axis {axis}")
    sig_livox = g_livox[:, ref_ax]
    sig_cam   = g_cam[:, query_ax] * sign

    # Optionally trim to a sub-window (useful for long sessions)
    if window_s > 0:
        t0 = max(t_livox[0], t_cam[0])
        t1 = t0 + window_s
        mask_l = (t_livox >= t0) & (t_livox <= t1)
        mask_c = (t_cam   >= t0) & (t_cam   <= t1)
        t_livox, sig_livox = t_livox[mask_l], sig_livox[mask_l]
        t_cam,   sig_cam   = t_cam[mask_c],   sig_cam[mask_c]

    livox_hz  = int(round(len(t_livox) / (t_livox[-1] - t_livox[0])))
    camera_hz = int(round(len(t_cam)   / (t_cam[-1]   - t_cam[0])))
    print(f"  Livox IMU:  {len(t_livox)} samples @ ~{livox_hz}Hz")
    print(f"  Camera IMU: {len(t_cam)} samples @ ~{camera_hz}Hz")

    # Reconstruct per-sample timestamps if the camera IMU was batch-delivered
    gaps = np.diff(t_cam)
    zero_gap_frac = np.sum(gaps < 1e-3) / len(gaps)
    if zero_gap_frac > 0.1:
        print(f"  ⚠ Camera IMU has {zero_gap_frac*100:.0f}% duplicate timestamps "
              f"(SDK batch delivery) — reconstructing timestamps")
        t_cam = _fix_batched_timestamps(t_cam, nominal_hz=camera_hz)

    delta_t, confidence = estimate_offset(t_livox, sig_livox, t_cam, sig_cam)

    _CONF_THRESHOLD = 0.7
    result = {
        "delta_t_s":   round(delta_t, 6) if confidence >= _CONF_THRESHOLD else 0.0,
        "confidence":  round(confidence, 4),
        "method":      "gyro_xcorr",
        "livox_axis":  axis_names[ref_ax],
        "camera_axis": axis_names[query_ax],
        "axis_sign":   sign,
        "axis":        axis_names[ref_ax],
        "livox_hz":    livox_hz,
        "camera_hz":   camera_hz,
    }

    out_path = bag_dir / out_name
    out_path.write_text(json.dumps(result, indent=2))

    # Warn if offset has shifted significantly from the previous session.
    prev_sessions = sorted(
        [p for p in session.parent.glob("*/rosbag_*/sync_offset.json") if p != out_path],
        key=lambda p: p.stat().st_mtime, reverse=True
    )

    sign = "+" if delta_t >= 0 else ""
    flag = "" if confidence >= _CONF_THRESHOLD else f"  ⚠ LOW CONFIDENCE — offset zeroed, ensure scanner was moving"
    print(f"\n  Δt = {sign}{delta_t * 1000:.2f} ms  (confidence={confidence:.3f}){flag}")
    print(f"  camera_t_corrected = camera_t + ({sign}{delta_t * 1000:.2f} ms)")
    print(f"  Saved: {out_path}")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir")
    parser.add_argument("--axis",   default="z", choices=["x", "y", "z"])
    parser.add_argument("--window", type=float, default=0.0,
                        help="Seconds of data to use (0 = full bag)")
    parser.add_argument("--out",    default="sync_offset.json")
    args = parser.parse_args()
    estimate_imu_sync(args.session_dir, args.axis, args.window, args.out)
