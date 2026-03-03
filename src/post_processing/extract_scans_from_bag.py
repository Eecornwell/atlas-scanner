#!/usr/bin/env python3
"""
Inject accurate trajectory poses from the session rosbag into existing
fusion_scan_NNN/ directories created by continuous_fisheye_capture.py.

Each scan dir already has fisheye_*.jpg and sensor_lidar_*.ply from live
capture. This script reads the bag odometry and writes trajectory.json
using the pose closest to each scan's capture time.

Usage:
  python3 extract_scans_from_bag.py <session_dir>
"""

import sys
import os
import json
import sqlite3
import argparse
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_odometry(bag_dir):
    bag_dir = Path(bag_dir)
    db3_files = sorted(bag_dir.glob('*.db3'))
    if not db3_files:
        zstd_files = sorted(bag_dir.glob('*.db3.zstd'))
        if not zstd_files:
            print(f'✗ No .db3 file in {bag_dir}')
            sys.exit(1)
        import subprocess
        db3_path = str(zstd_files[0]).replace('.zstd', '')
        print(f'  Decompressing {zstd_files[0].name}...')
        subprocess.run(['zstd', '-d', str(zstd_files[0]), '-o', db3_path, '-f'], check=True)
        db3_files = [Path(db3_path)]

    con = sqlite3.connect(str(db3_files[0]))
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}

    odom_tid = next((tid for tid, (name, _) in topics.items()
                     if name == '/rko_lio/odometry'), None)
    if odom_tid is None:  # fall back to buffered
        odom_tid = next((tid for tid, (name, _) in topics.items()
                         if 'odometry' in name), None)
    if odom_tid is None:
        print('✗ No odometry topic in bag')
        sys.exit(1)

    odom_type = get_message(topics[odom_tid][1])
    odom_msgs = []
    for (data, ts_ns) in con.execute(
            'SELECT data, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp',
            (odom_tid,)):
        msg = deserialize_message(bytes(data), odom_type)
        odom_msgs.append((ts_ns / 1e9, msg))

    con.close()
    print(f'  Odometry: {len(odom_msgs)} poses, '
          f'{odom_msgs[-1][0] - odom_msgs[0][0]:.1f}s duration')
    return odom_msgs


def write_trajectory_json(scan_name, scan_dir, capture_ts, odom_msg, all_odom, first_T):
    pos = odom_msg.pose.pose.position
    ori = odom_msg.pose.pose.orientation
    q = np.array([ori.x, ori.y, ori.z, ori.w])
    q /= np.linalg.norm(q)
    R_mat = Rotation.from_quat(q).as_matrix()

    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [pos.x, pos.y, pos.z]

    lidar_pose = {
        'position': {'x': float(pos.x), 'y': float(pos.y), 'z': float(pos.z)},
        'orientation': {'x': float(q[0]), 'y': float(q[1]), 'z': float(q[2]), 'w': float(q[3])}
    }

    start_ts = all_odom[0][0]
    full_traj = []
    for ts, om in all_odom:
        op = om.pose.pose.position
        oo = om.pose.pose.orientation
        full_traj.append({
            'timestamp': ts,
            'relative_time': ts - start_ts,
            'position': {'x': op.x, 'y': op.y, 'z': op.z},
            'orientation': {'x': oo.x, 'y': oo.y, 'z': oo.z, 'w': oo.w},
            'lidar_pose': {
                'position': {'x': op.x, 'y': op.y, 'z': op.z},
                'orientation': {'x': oo.x, 'y': oo.y, 'z': oo.z, 'w': oo.w},
            },
        })

    data = {
        'scan_info': {
            'name': scan_name,
            'timestamp': datetime.now().isoformat(),
            'capture_time': capture_ts,
            'scan_request_time': capture_ts,
            'scan_pose_time': capture_ts,
        },
        'current_pose': {
            'timestamp': capture_ts,
            'relative_time': capture_ts - start_ts,
            'lidar_pose': lidar_pose,
            'position': lidar_pose['position'],
            'orientation': lidar_pose['orientation'],
        },
        'full_trajectory': full_traj,
    }

    with open(os.path.join(scan_dir, 'trajectory.json'), 'w') as f:
        json.dump(data, f, indent=2)

    return T


def get_scan_capture_time(scan_dir):
    """Infer capture time from sensor_lidar_*.ply filename timestamp."""
    plys = sorted(Path(scan_dir).glob('sensor_lidar_*.ply'))
    if not plys:
        return None
    ts_str = plys[0].stem.replace('sensor_lidar_', '')
    try:
        return datetime.strptime(ts_str, '%Y%m%d_%H%M%S').timestamp()
    except ValueError:
        return None


def inject_poses(session_dir):
    session_path = Path(session_dir)

    scan_dirs = sorted([d for d in session_path.iterdir()
                        if d.is_dir() and d.name.startswith('fusion_scan_')])
    if not scan_dirs:
        print('✗ No fusion_scan_* directories found')
        sys.exit(1)

    bag_dirs = sorted(session_path.glob('rosbag_*'))
    if not bag_dirs:
        print('✗ No rosbag found')
        sys.exit(1)

    print(f'Reading bag: {bag_dirs[0].name}')
    odom_msgs = read_odometry(bag_dirs[0])

    first_T = None
    injected = 0

    for scan_dir in scan_dirs:
        capture_ts = get_scan_capture_time(scan_dir)
        if capture_ts is None:
            print(f'  ⚠ {scan_dir.name}: cannot determine capture time, skipping')
            continue

        # Find median pose over a ±2s window around capture time (filters LIO noise)
        window = [msg for ts, msg in odom_msgs if abs(ts - capture_ts) <= 2.0]
        if not window:
            best_ts, best_msg = min(odom_msgs, key=lambda x: abs(x[0] - capture_ts))
            dt = abs(best_ts - capture_ts)
        else:
            # Use the pose whose position is closest to the median position in the window
            xs = [m.pose.pose.position.x for m in window]
            ys = [m.pose.pose.position.y for m in window]
            zs = [m.pose.pose.position.z for m in window]
            mx, my, mz = float(np.median(xs)), float(np.median(ys)), float(np.median(zs))
            best_msg = min(window, key=lambda m: (
                (m.pose.pose.position.x-mx)**2 +
                (m.pose.pose.position.y-my)**2 +
                (m.pose.pose.position.z-mz)**2))
            best_ts = min(odom_msgs, key=lambda x: abs(x[0] - capture_ts))[0]
            dt = abs(best_ts - capture_ts)
        if dt > 5.0:
            print(f'  ⚠ {scan_dir.name}: closest pose is {dt:.1f}s away, skipping')
            continue

        ref_T = first_T if first_T is not None else np.eye(4)
        T = write_trajectory_json(scan_dir.name, str(scan_dir), capture_ts,
                                  best_msg, odom_msgs, ref_T)
        if first_T is None:
            first_T = T

        pos = best_msg.pose.pose.position
        print(f'  ✓ {scan_dir.name}: pose=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}) dt={dt:.2f}s')
        injected += 1

    print(f'\n✓ Injected poses into {injected}/{len(scan_dirs)} scans')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('session_dir')
    args = parser.parse_args()
    inject_poses(args.session_dir)
