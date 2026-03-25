#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Reconstructs a full scan session from a recorded rosbag by grouping LiDAR frames into time-windowed scans and matching each to the closest camera frame and odometry pose.
"""
Reconstruct scan sessions from a rosbag using ROS header timestamps.

Instead of interval-based live capture, this reads the bag after recording
and groups LiDAR frames into scans, then finds the temporally closest
fisheye image and odometry pose for each scan using header stamps.

Usage:
  python3 reconstruct_from_bag.py <session_dir> [--interval 3.0] [--lidar-window 2.0]
"""

import sys
import os
import json
import struct
import sqlite3
import argparse
import subprocess
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation, Slerp

import cv2

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("✗ rclpy not available — source your ROS workspace first")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Bag reading helpers
# ---------------------------------------------------------------------------

def open_db3(bag_dir):
    bag_path = Path(bag_dir)
    db3_files = sorted(bag_path.glob("*.db3"))
    if db3_files:
        return sqlite3.connect(str(db3_files[0]))
    zstd = sorted(bag_path.glob("*.db3.zstd"))
    if not zstd:
        raise FileNotFoundError(f"No .db3 file in {bag_dir}")
    out = str(zstd[0]).replace(".zstd", "")
    print(f"  Decompressing {zstd[0].name}...")
    result = subprocess.run(["zstd", "-d", str(zstd[0]), "-o", out, "-f"],
                            capture_output=True)
    if result.returncode != 0 or not Path(out).exists():
        raise RuntimeError(
            f"Bag is truncated/corrupt and cannot be recovered: {zstd[0].name}\n"
            f"The recording was likely interrupted during compression.\n"
            f"Future sessions use uncompressed recording to prevent this.")
    return sqlite3.connect(out)


def topic_map(con):
    return {r[0]: (r[1], r[2]) for r in con.execute("SELECT id, name, type FROM topics")}


def read_topic(con, topics, name_fragment):
    """Return list of (ros_stamp_sec, data_bytes) sorted by stamp, matched by name fragment."""
    tid = next((tid for tid, (n, _) in topics.items() if n == name_fragment), None)
    if tid is None:
        tid = next((tid for tid, (n, _) in topics.items() if name_fragment in n), None)
    if tid is None:
        return None, None
    msg_type_str = topics[tid][1]
    MsgType = get_message(msg_type_str)
    rows = con.execute(
        "SELECT data, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,),
    ).fetchall()
    msgs = []
    for data, bag_ts_ns in rows:
        msg = deserialize_message(bytes(data), MsgType)
        # Prefer header stamp when available
        if hasattr(msg, "header"):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            stamp = bag_ts_ns / 1e9
        msgs.append((stamp, msg))
    msgs.sort(key=lambda x: x[0])
    return msgs, MsgType


# ---------------------------------------------------------------------------
# Point cloud helpers
# ---------------------------------------------------------------------------

def unpack_lidar(msg):
    points = []
    intensity_offset = next((f.offset for f in msg.fields if f.name == "intensity"), None)
    for i in range(0, len(msg.data), msg.point_step):
        if i + 12 > len(msg.data):
            break
        x, y, z = (struct.unpack("<f", msg.data[i + o: i + o + 4])[0] for o in (0, 4, 8))
        if not (abs(x) < 200 and abs(y) < 200 and abs(z) < 100):
            continue
        intensity = 0.5
        if intensity_offset and i + intensity_offset + 4 <= len(msg.data):
            try:
                intensity = max(0.0, min(1.0,
                    struct.unpack("<f", msg.data[i + intensity_offset: i + intensity_offset + 4])[0] / 255.0))
            except Exception:
                pass
        points.append([x, y, z, intensity])
    return points


def save_ply(path, points, has_intensity=True):
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if has_intensity:
            f.write("property float intensity\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")
            if has_intensity:
                f.write(f" {p[3]:.6f}")
            f.write("\n")


# ---------------------------------------------------------------------------
# Closest-message lookup
# ---------------------------------------------------------------------------

def closest(msgs, target_stamp):
    """Return the message whose stamp is closest to target_stamp."""
    return min(msgs, key=lambda x: abs(x[0] - target_stamp))


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def pose_to_matrix(odom_msg):
    pos = odom_msg.pose.pose.position
    ori = odom_msg.pose.pose.orientation
    q = np.array([ori.x, ori.y, ori.z, ori.w])
    q /= np.linalg.norm(q)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = [pos.x, pos.y, pos.z]
    return T


def interp_pose(all_odom, target_stamp):
    """Return (t_interp, q_interp) interpolated at target_stamp."""
    times = np.array([ts for ts, _ in all_odom])
    idx = np.searchsorted(times, target_stamp)

    if idx == 0 or idx >= len(all_odom):
        _, odom_msg = all_odom[max(0, idx - 1)]
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        q = np.array([ori.x, ori.y, ori.z, ori.w])
        q /= np.linalg.norm(q)
        return np.array([pos.x, pos.y, pos.z]), q

    ts0, om0 = all_odom[idx - 1]
    ts1, om1 = all_odom[idx]
    alpha = (target_stamp - ts0) / (ts1 - ts0) if ts1 != ts0 else 0.0

    p0, p1 = om0.pose.pose.position, om1.pose.pose.position
    t_interp = np.array([p0.x, p0.y, p0.z]) + alpha * (
        np.array([p1.x, p1.y, p1.z]) - np.array([p0.x, p0.y, p0.z]))

    o0, o1 = om0.pose.pose.orientation, om1.pose.pose.orientation
    q0 = np.array([o0.x, o0.y, o0.z, o0.w]); q0 /= np.linalg.norm(q0)
    q1 = np.array([o1.x, o1.y, o1.z, o1.w]); q1 /= np.linalg.norm(q1)
    q_interp = Slerp([0.0, 1.0], Rotation.from_quat([q0, q1]))(alpha).as_quat()
    return t_interp, q_interp


def write_trajectory_json(scan_dir, scan_name, capture_stamp, all_odom):
    """Interpolate odometry at capture_stamp and write trajectory.json."""
    t_interp, q_interp = interp_pose(all_odom, capture_stamp)

    lidar_pose = {
        "position": {"x": float(t_interp[0]), "y": float(t_interp[1]), "z": float(t_interp[2])},
        "orientation": {"x": float(q_interp[0]), "y": float(q_interp[1]),
                        "z": float(q_interp[2]), "w": float(q_interp[3])},
    }

    start_ts = all_odom[0][0]
    full_traj = []
    for ts, om in all_odom:
        op = om.pose.pose.position
        oo = om.pose.pose.orientation
        full_traj.append({
            "timestamp": ts,
            "relative_time": ts - start_ts,
            "position": {"x": op.x, "y": op.y, "z": op.z},
            "orientation": {"x": oo.x, "y": oo.y, "z": oo.z, "w": oo.w},
            "lidar_pose": {
                "position": {"x": op.x, "y": op.y, "z": op.z},
                "orientation": {"x": oo.x, "y": oo.y, "z": oo.z, "w": oo.w},
            },
        })

    data = {
        "scan_info": {
            "name": scan_name,
            "timestamp": datetime.now().isoformat(),
            "capture_time": capture_stamp,
            "scan_request_time": capture_stamp,
            "scan_pose_time": capture_stamp,
        },
        "current_pose": {
            "timestamp": capture_stamp,
            "relative_time": capture_stamp - start_ts,
            "lidar_pose": lidar_pose,
            "position": lidar_pose["position"],
            "orientation": lidar_pose["orientation"],
        },
        "full_trajectory": full_traj,
    }

    with open(os.path.join(scan_dir, "trajectory.json"), "w") as f:
        json.dump(data, f, indent=2)


# ---------------------------------------------------------------------------
# Fisheye decode — handles JPEG and H.264 CompressedImage
# ---------------------------------------------------------------------------

def _find_keyframes(image_msgs_raw):
    """Return sorted list of message indices that contain an IDR (keyframe) NAL unit."""
    keyframes = []
    for i, (_, msg) in enumerate(image_msgs_raw):
        raw = bytes(msg.data)
        j = 0
        while j < len(raw) - 4:
            if raw[j:j+4] == b'\x00\x00\x00\x01':
                if j + 4 < len(raw) and (raw[j+4] & 0x1f) == 5:  # IDR
                    keyframes.append(i)
                    break
                j += 4
            else:
                j += 1
    return keyframes


def _h264_frame_size(image_msgs_raw):
    """Decode a single frame to determine pixel dimensions."""
    raw = b''.join(bytes(m.data) for _, m in image_msgs_raw[:30])
    proc = subprocess.Popen(
        ['ffmpeg', '-f', 'h264', '-i', 'pipe:0', '-frames:v', '1',
         '-f', 'rawvideo', '-pix_fmt', 'bgr24', 'pipe:1'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    out, err = proc.communicate(input=raw)
    import re
    m = re.search(rb'(\d{3,4})x(\d{3,4})', err)
    if m:
        return int(m.group(2)), int(m.group(1))  # h, w
    return 1280, 2560


def decode_h264_frame(image_msgs_raw, target_idx, h, w, keyframes):
    """Decode a single H.264 frame by index, starting from the preceding keyframe."""
    kf = max((k for k in keyframes if k <= target_idx), default=0)
    raw = b''.join(bytes(m.data) for _, m in image_msgs_raw[kf: target_idx + 1])
    skip = target_idx - kf
    frame_bytes = h * w * 3
    proc = subprocess.Popen(
        ['ffmpeg', '-f', 'h264', '-i', 'pipe:0',
         '-vf', f'select=gte(n\\,{skip})', '-frames:v', '1',
         '-f', 'rawvideo', '-pix_fmt', 'bgr24', 'pipe:1'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )
    out, _ = proc.communicate(input=raw)
    if len(out) < frame_bytes:
        return None
    return np.frombuffer(out[:frame_bytes], dtype=np.uint8).reshape(h, w, 3)


def closest_image_index(image_msgs_raw, target_stamp):
    """Return (index, stamp, msg) of the closest frame by stamp."""
    idx = min(range(len(image_msgs_raw)), key=lambda i: abs(image_msgs_raw[i][0] - target_stamp))
    return idx, image_msgs_raw[idx][0], image_msgs_raw[idx][1]


def decode_jpeg_frame(msg):
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def extract_back_fisheye(dual_img):
    back = dual_img[:, : dual_img.shape[1] // 2]
    return cv2.rotate(back, cv2.ROTATE_90_CLOCKWISE)


# ---------------------------------------------------------------------------
# Stationary mode: one bag per scan already in fusion_scan_* dirs
# ---------------------------------------------------------------------------

def _reconstruct_stationary(session_path, per_scan_bags, camera_mode):
    """Re-process per-scan bags that were recorded in stationary mode.
    Each bag already corresponds to one scan; we just extract the image,
    LiDAR, and odometry and run colorization."""
    pp = Path(__file__).resolve().parent
    scan_count = 0

    for bag_dir in per_scan_bags:
        scan_dir = bag_dir.parent
        scan_name = scan_dir.name
        print(f"\nProcessing {scan_name} from {bag_dir.name}...")

        try:
            con = open_db3(bag_dir)
        except Exception as e:
            print(f"  ✗ Could not open bag: {e}")
            continue

        topics = topic_map(con)
        print("  Topics:", ", ".join(n for _, (n, _) in topics.items()))

        lidar_msgs, _ = read_topic(con, topics, "/livox/lidar")
        image_msgs_raw, _ = read_topic(con, topics, "fisheye")
        odom_msgs, _ = read_topic(con, topics, "/rko_lio/odometry")
        con.close()

        if not lidar_msgs:
            print(f"  ✗ No LiDAR messages, skipping")
            continue

        # --- LiDAR: accumulate all points from the bag ---
        all_points = []
        for _, msg in lidar_msgs:
            all_points.extend(unpack_lidar(msg))

        if not all_points:
            print(f"  ✗ Zero valid points, skipping")
            continue

        # Only write PLY if not already present from live capture
        sensor_ply = scan_dir / "sensor_lidar.ply"
        if not sensor_ply.exists():
            save_ply(str(sensor_ply), all_points)
            print(f"  Saved sensor_lidar.ply ({len(all_points)} pts)")
        else:
            print(f"  sensor_lidar.ply already exists, skipping")

        centre = lidar_msgs[len(lidar_msgs) // 2][0]  # mid-bag timestamp
        ts_str = datetime.fromtimestamp(centre).strftime("%Y%m%d_%H%M%S")

        # --- Camera: decode middle frame ---
        if image_msgs_raw:
            is_h264 = 'h264' in getattr(image_msgs_raw[0][1], 'format', '').lower()
            img_out = scan_dir / f"fisheye_{ts_str}.jpg"
            if not any(scan_dir.glob("fisheye_*.jpg")):
                if is_h264:
                    img_h, img_w = _h264_frame_size(image_msgs_raw)
                    keyframes = _find_keyframes(image_msgs_raw)
                    mid_idx = len(image_msgs_raw) // 2
                    bgr = decode_h264_frame(image_msgs_raw, mid_idx, img_h, img_w, keyframes)
                else:
                    mid_idx = len(image_msgs_raw) // 2
                    bgr = decode_jpeg_frame(image_msgs_raw[mid_idx][1])
                if bgr is not None:
                    out_img = bgr if camera_mode == "dual_fisheye" else extract_back_fisheye(bgr)
                    cv2.imwrite(str(img_out), out_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    print(f"  Saved {img_out.name}")
            else:
                print(f"  fisheye image already exists, skipping")

        # --- Odometry: write trajectory.json if not present ---
        if odom_msgs and not (scan_dir / "trajectory.json").exists():
            write_trajectory_json(str(scan_dir), scan_name, centre, odom_msgs)
            print(f"  Saved trajectory.json")

            import json as _json
            with open(str(scan_dir / "trajectory.json")) as _f:
                _traj = _json.load(_f)
            _lp = _traj["current_pose"]["lidar_pose"]
            q = np.array([_lp["orientation"][k] for k in ("x", "y", "z", "w")])
            q /= np.linalg.norm(q)
            R_mat = Rotation.from_quat(q).as_matrix()
            t_vec = np.array([_lp["position"][k] for k in ("x", "y", "z")])
            world_ply = scan_dir / "world_lidar.ply"
            if not world_ply.exists():
                world_pts = [(R_mat @ np.array(p[:3]) + t_vec).tolist() + [p[3]]
                             for p in all_points]
                save_ply(str(world_ply), world_pts)
                print(f"  Saved world_lidar.ply")

        scan_count += 1
        print(f"  ✓ {scan_name} ready")

    print(f"\n✓ Processed {scan_count} scans")

    # --- Colorization ---
    print("\nColorizing...")
    if camera_mode == "dual_fisheye":
        fisheye_to_erp = str(pp / "fisheye_to_erp.py")
        for scan_dir in sorted(session_path.glob("fusion_scan_*")):
            if not scan_dir.is_dir():
                continue
            for fisheye_jpg in sorted(scan_dir.glob("fisheye_*.jpg")):
                erp_path = scan_dir / "equirect_dual_fisheye.jpg"
                if not erp_path.exists():
                    subprocess.run(
                        [sys.executable, fisheye_to_erp, str(fisheye_jpg), str(erp_path), "--dual"],
                        check=False,
                    )
        subprocess.run(
            [sys.executable, str(pp / "post_process_coloring.py"), str(session_path), "--use-exact"],
            check=False,
        )
    else:
        for scan_dir in sorted(session_path.glob("fusion_scan_*")):
            if not scan_dir.is_dir() or not list(scan_dir.glob("fisheye_*.jpg")):
                continue
            print(f"  Coloring {scan_dir.name}...")
            subprocess.run(
                [sys.executable, str(pp / "color_with_fisheye.py"), str(scan_dir)],
                check=False,
            )

    return scan_count


# ---------------------------------------------------------------------------
# Main reconstruction
# ---------------------------------------------------------------------------

def reconstruct(session_dir, interval=3.0, lidar_window=2.0, camera_mode="single_fisheye", max_gyro=0.3, trim_ends=2):
    session_path = Path(session_dir)

    # Continuous mode: single bag at session root
    bag_dirs = sorted(session_path.glob("rosbag_*"))

    # Stationary mode: per-scan bags inside fusion_scan_*/rosbag_*
    if not bag_dirs:
        per_scan_bags = sorted(session_path.glob("fusion_scan_*/rosbag_*"))
        if per_scan_bags:
            return _reconstruct_stationary(session_path, per_scan_bags, camera_mode)
        print("✗ No rosbag_* directory found in session")
        sys.exit(1)

    bag_dir = bag_dirs[0]
    print(f"Reading bag: {bag_dir.name}")

    con = open_db3(bag_dir)
    topics = topic_map(con)

    print("  Topics in bag:")
    for tid, (name, typ) in topics.items():
        count = con.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,)).fetchone()[0]
        print(f"    {name}  ({count} msgs)")

    lidar_msgs, _ = read_topic(con, topics, "/livox/lidar")
    image_msgs_raw, _ = read_topic(con, topics, "fisheye")
    odom_msgs, _ = read_topic(con, topics, "/rko_lio/odometry")
    con.close()

    # Load Livox IMU for gyro-based motion detection.
    imu_msgs = None
    imu_bag_dirs = sorted(session_path.glob("rosbag_*_imu"))
    _imu_bag_dir = imu_bag_dirs[0] if imu_bag_dirs else bag_dir
    try:
        imu_con = open_db3(_imu_bag_dir)
        imu_topics = topic_map(imu_con)
        imu_msgs, _ = read_topic(imu_con, imu_topics, "/livox/imu")
        imu_con.close()
        if imu_msgs:
            print(f"  IMU: {len(imu_msgs)} samples from {_imu_bag_dir.name}  "
                  f"(~{len(imu_msgs)/(imu_msgs[-1][0]-imu_msgs[0][0]):.0f}Hz)")
    except Exception as e:
        print(f"  ⚠ Could not load IMU data: {e} — gyro filter disabled")

    # Load IMU-derived clock offset if available (produced by imu_sync.py).
    # Only applied when confidence >= 0.7 — below that the cross-correlation
    # peak is unreliable and applying a spurious offset makes alignment worse.
    # Note: the camera driver stamps frames with rclpy.clock() (same ROS system
    # clock as the Livox driver), so the true offset is typically <5ms and a
    # low-confidence estimate should never override that.
    _CONF_THRESHOLD = 0.7
    _sync_path = bag_dir / "sync_offset.json"
    _cam_dt = 0.0
    if _sync_path.exists():
        import json as _json
        _sync = _json.loads(_sync_path.read_text())
        _cam_dt = float(_sync.get("delta_t_s", 0.0))
        _conf   = float(_sync.get("confidence", 0.0))
        if _conf < _CONF_THRESHOLD:
            print(f"  IMU sync offset: {_cam_dt * 1000:+.2f} ms  (confidence={_conf:.3f}) "
                  f"— BELOW threshold ({_CONF_THRESHOLD}), not applied")
            _cam_dt = 0.0
        else:
            print(f"  IMU sync offset: {_cam_dt * 1000:+.2f} ms  (confidence={_conf:.3f}) — applied")
            if image_msgs_raw and _cam_dt != 0.0:
                image_msgs_raw = [(_t + _cam_dt, _m) for _t, _m in image_msgs_raw]
    else:
        print("  No sync_offset.json found — run imu_sync.py for sub-10ms accuracy")

    is_h264 = False
    img_h, img_w = 1280, 2560
    keyframes = []
    if image_msgs_raw:
        fmt = getattr(image_msgs_raw[0][1], 'format', 'unknown').lower()
        is_h264 = 'h264' in fmt or 'h.264' in fmt
        print(f"  Camera format: {fmt}  ({len(image_msgs_raw)} frames)")
        if is_h264:
            img_h, img_w = _h264_frame_size(image_msgs_raw)
            keyframes = _find_keyframes(image_msgs_raw)
            print(f"  Frame size: {img_w}x{img_h}, GOP size: ~{keyframes[1]-keyframes[0] if len(keyframes)>1 else 'N/A'}")

    if not lidar_msgs:
        print("✗ No LiDAR messages in bag")
        sys.exit(1)

    print(f"  LiDAR: {len(lidar_msgs)} frames  "
          f"({lidar_msgs[-1][0] - lidar_msgs[0][0]:.1f}s)")
    if image_msgs_raw:
        print(f"  Camera: {len(image_msgs_raw)} frames  "
              f"({image_msgs_raw[-1][0] - image_msgs_raw[0][0]:.1f}s)")
    if odom_msgs:
        print(f"  Odometry: {len(odom_msgs)} poses  "
              f"({odom_msgs[-1][0] - odom_msgs[0][0]:.1f}s)")

    # -----------------------------------------------------------------------
    # Build scan centres from camera frame timestamps so each scan's image
    # is the exact frame that drove the capture.  For H.264 we use keyframe
    # indices directly; for JPEG every frame is a candidate.
    # `interval` acts as a minimum gap between scans to avoid duplicates.
    # LiDAR points are then collected within `lidar_window` of each centre.
    # -----------------------------------------------------------------------
    t_start = lidar_msgs[0][0]
    t_end = lidar_msgs[-1][0]
    duration = t_end - t_start

    if image_msgs_raw and odom_msgs:
        # Select scan centres at low-motion moments using IMU gyroscope magnitude.
        # Falls back to odom-based motion detection if IMU is unavailable.

        def gyro_magnitude_at(t, window=0.5):
            if imu_msgs is None or len(imu_msgs) == 0:
                return None
            samples = [np.linalg.norm([m.angular_velocity.x,
                                       m.angular_velocity.y,
                                       m.angular_velocity.z])
                       for ts, m in imu_msgs
                       if abs(ts - t) <= window / 2.0]
            return float(np.mean(samples)) if samples else None

        odom_times = np.array([ts for ts, _ in odom_msgs])

        def motion_score_at(t):
            gyro = gyro_magnitude_at(t)
            if gyro is not None:
                return gyro
            # Fallback: odom-based
            w = 0.5
            idx0 = np.searchsorted(odom_times, t - w)
            idx1 = np.searchsorted(odom_times, t + w)
            if idx1 <= idx0:
                return float('inf')
            t0_p, q0 = interp_pose(odom_msgs, odom_times[idx0])
            t1_p, q1 = interp_pose(odom_msgs, odom_times[min(idx1, len(odom_msgs)-1)])
            d_pos = float(np.linalg.norm(t1_p - t0_p))
            dR = Rotation.from_quat(q0).inv() * Rotation.from_quat(q1)
            d_angle = float(np.degrees(np.linalg.norm(dR.as_rotvec())))
            return d_pos + d_angle * 0.01

        if is_h264:
            candidate_indices = keyframes if keyframes else list(range(len(image_msgs_raw)))
        else:
            candidate_indices = list(range(len(image_msgs_raw)))

        scored = []
        for ci in candidate_indices:
            t = image_msgs_raw[ci][0]
            if t < t_start or t > t_end:
                continue
            if not (odom_times[0] <= t <= odom_times[-1]):
                continue
            scored.append((motion_score_at(t), t, ci))
        scored.sort(key=lambda x: x[0])

        using_gyro = imu_msgs is not None and len(imu_msgs) > 0
        print(f"  Motion-gated scan selection using {'IMU gyro' if using_gyro else 'odom'}")

        centres = []
        for score, t, ci in scored:
            if using_gyro and score > max_gyro:
                continue
            if all(abs(t - tc) >= interval for tc, _ in centres):
                centres.append((t, ci))
        centres.sort(key=lambda x: x[0])

        if not centres:
            print(f"  ⚠ No frames below max_gyro={max_gyro:.2f} rad/s — using best available")
            for _, t, ci in scored:
                if all(abs(t - tc) >= interval for tc, _ in centres):
                    centres.append((t, ci))
            centres.sort(key=lambda x: x[0])

        for tc, _ in centres:
            score = motion_score_at(tc)
            label = f'{score:.3f} rad/s' if using_gyro else f'score={score:.4f}'
            print(f"    t+{tc - t_start:.1f}s: {label}")

    elif image_msgs_raw:
        if is_h264:
            candidate_indices = keyframes if keyframes else list(range(len(image_msgs_raw)))
        else:
            candidate_indices = list(range(len(image_msgs_raw)))
        centres = []
        last_t = -float('inf')
        for ci in candidate_indices:
            t = image_msgs_raw[ci][0]
            if t < t_start or t > t_end:
                continue
            if t - last_t >= interval:
                centres.append((t, ci))
                last_t = t
        if not centres:
            centres = [(image_msgs_raw[candidate_indices[0]][0], candidate_indices[0])]
    else:
        # No camera — fall back to LiDAR-derived centres.
        centres = []
        t = t_start + lidar_window / 2.0
        while t < t_end - lidar_window / 2.0:
            centres.append((t, None))
            t += interval
        if not centres:
            centres = [((t_start + t_end) / 2.0, None)]

    # Restrict centres to the odometry window so every scan gets a properly
    # interpolated pose rather than a clamped boundary extrapolation.
    # Frames before odom starts (RKO-LIO init delay) are silently dropped.
    if odom_msgs:
        odom_t0 = odom_msgs[0][0]
        odom_t1 = odom_msgs[-1][0]
        filtered = [(t, i) for t, i in centres if odom_t0 <= t <= odom_t1]
        if len(filtered) < len(centres):
            dropped = len(centres) - len(filtered)
            print(f"  ⚠ Dropping {dropped} scan centre(s) outside odom window "
                  f"(odom starts {odom_t0 - lidar_msgs[0][0]:.2f}s after LiDAR)")
        centres = filtered if filtered else centres  # keep all if odom window is empty

    if trim_ends > 0 and len(centres) > trim_ends * 2:
        centres = centres[trim_ends:-trim_ends]
        print(f"  Trimmed {trim_ends} scan(s) from each end (SLAM startup/shutdown) — {len(centres)} remaining")
    elif trim_ends > 0:
        print(f"  ⚠ Not enough scans to trim {trim_ends} from each end — keeping all {len(centres)}")

    print(f"\n  Session duration: {duration:.1f}s  →  {len(centres)} scans "
          f"(camera-driven, min_gap={interval}s, lidar_window={lidar_window}s)")

    scan_count = 0
    for idx, (centre, img_idx) in enumerate(centres):
        scan_num = idx + 1
        scan_name = f"fusion_scan_{scan_num:03d}"
        scan_dir = session_path / scan_name
        scan_dir.mkdir(exist_ok=True)

        ts_str = datetime.fromtimestamp(centre).strftime("%Y%m%d_%H%M%S")

        # --- LiDAR: per-frame motion compensation within the window ---
        # Each frame is transformed to world frame using its own interpolated pose,
        # eliminating motion blur from accumulating frames with a single pose.
        half = lidar_window / 2.0
        window_lidar = [(ts, msg) for ts, msg in lidar_msgs if abs(ts - centre) <= half]
        if not window_lidar:
            print(f"  ⚠ {scan_name}: no LiDAR frames in window, skipping")
            continue

        all_points = []       # sensor-frame (for sensor_lidar.ply)
        world_points = []     # motion-compensated world-frame (for world_lidar.ply)
        seen = set()
        for frame_ts, msg in window_lidar:
            mid = id(msg)
            if mid in seen:
                continue
            seen.add(mid)
            pts = unpack_lidar(msg)
            all_points.extend(pts)
            if odom_msgs:
                t_f, q_f = interp_pose(odom_msgs, frame_ts)
                R_f = Rotation.from_quat(q_f).as_matrix()
                for p in pts:
                    world_points.append((R_f @ np.array(p[:3]) + t_f).tolist() + [p[3]])

        if not all_points:
            print(f"  ⚠ {scan_name}: zero valid points, skipping")
            continue

        ply_path = scan_dir / "sensor_lidar.ply"
        save_ply(str(ply_path), all_points)

        # --- Camera: decode the exact frame whose timestamp is the centre ---
        fisheye_path = None
        dt_img = 0.0
        if image_msgs_raw and img_idx is not None:
            img_out = scan_dir / f"fisheye_{ts_str}.jpg"
            if is_h264:
                bgr = decode_h264_frame(image_msgs_raw, img_idx, img_h, img_w, keyframes)
            else:
                bgr = decode_jpeg_frame(image_msgs_raw[img_idx][1])
            if bgr is not None:
                out_img = bgr if camera_mode == "dual_fisheye" else extract_back_fisheye(bgr)
                cv2.imwrite(str(img_out), out_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                fisheye_path = str(img_out)
        elif image_msgs_raw and img_idx is None:
            # LiDAR-fallback path: find closest image
            img_idx_fb, img_stamp_fb, img_msg_fb = closest_image_index(image_msgs_raw, centre)
            dt_img = abs(img_stamp_fb - centre)
            if dt_img < interval:
                img_out = scan_dir / f"fisheye_{ts_str}.jpg"
                bgr = decode_jpeg_frame(img_msg_fb)
                if bgr is not None:
                    out_img = bgr if camera_mode == "dual_fisheye" else extract_back_fisheye(bgr)
                    cv2.imwrite(str(img_out), out_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    fisheye_path = str(img_out)

        # --- Odometry: write trajectory.json at camera frame stamp; save motion-compensated world cloud ---
        dt_odom = float('inf')
        if odom_msgs:
            odom_stamp, odom_msg = closest(odom_msgs, centre)
            dt_odom = abs(odom_stamp - centre)
            if dt_odom < 5.0:
                write_trajectory_json(str(scan_dir), scan_name, centre, odom_msgs)
                if world_points:
                    save_ply(str(scan_dir / "world_lidar.ply"), world_points)
            else:
                print(f"    ⚠ {scan_name}: closest odom is {dt_odom:.2f}s away")

        scan_count += 1
        print(f"  ✓ {scan_name}: {len(all_points)} pts  "
              f"img_dt={dt_img:.3f}s  odom_dt={dt_odom:.3f}s")

    print(f"\n✓ Reconstructed {scan_count} scans from bag")

    # --- ERP conversion + colorization ---
    pp = Path(__file__).resolve().parent

    print("\nConverting fisheye → ERP and colorizing...")
    if camera_mode == "dual_fisheye":
        fisheye_to_erp = str(pp / "fisheye_to_erp.py")
        for scan_dir in sorted(session_path.glob("fusion_scan_*")):
            if not scan_dir.is_dir():
                continue
            for fisheye_jpg in sorted(scan_dir.glob("fisheye_*.jpg")):
                erp_path = scan_dir / "equirect_dual_fisheye.jpg"
                subprocess.run(
                    [sys.executable, fisheye_to_erp, str(fisheye_jpg), str(erp_path), "--dual"],
                    check=False,
                )
        subprocess.run(
            [sys.executable, str(pp / "post_process_coloring.py"), str(session_path), "--use-exact"],
            check=False,
        )
    else:
        for scan_dir in sorted(session_path.glob("fusion_scan_*")):
            if not scan_dir.is_dir() or not sorted(scan_dir.glob("fisheye_*.jpg")):
                continue
            print(f"  Coloring {scan_dir.name}...")
            subprocess.run(
                [sys.executable, str(pp / "color_with_fisheye.py"), str(scan_dir)],
                check=False,
            )

    return scan_count


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", help="Path to the session directory (contains rosbag_*)")
    parser.add_argument("--interval", type=float, default=3.0,
                        help="Seconds between scan centres (default: 3.0)")
    parser.add_argument("--lidar-window", type=float, default=2.0,
                        help="LiDAR accumulation window in seconds (default: 2.0)")
    parser.add_argument("--camera-mode", default="single_fisheye",
                        choices=["dual_fisheye", "single_fisheye"])
    parser.add_argument("--max-gyro", type=float, default=0.3,
                        help="Max gyro magnitude (rad/s) to accept a scan centre (default: 0.3)")
    parser.add_argument("--trim-ends", type=int, default=2,
                        help="Scans to drop from each end for SLAM startup/shutdown (default: 2)")
    args = parser.parse_args()

    reconstruct(args.session_dir, args.interval, args.lidar_window, args.camera_mode, args.max_gyro, args.trim_ends)
