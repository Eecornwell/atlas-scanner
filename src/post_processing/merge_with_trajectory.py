#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Merges colored sensor-frame point clouds from all scans in a session into a single world-frame PLY by applying each scan's trajectory pose.
"""
Merge colored point clouds using trajectory poses.

KEY: Points in sensor_colored_exact.ply are in SENSOR frame.
Use R1.T @ R.T to align rotations (double transpose).
"""

import sys
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R, Slerp

def load_ply(ply_file):
    """Load colored points from PLY, skipping uncolored (black) points"""
    points = []
    colors = []

    with open(ply_file, 'r') as f:
        raw = f.read()
    lines = raw.split(r'\n') if ('\n' not in raw and r'\n' in raw) else raw.splitlines()

    header_end = next(i+1 for i, line in enumerate(lines) if line.strip() == 'end_header')
    
    for line in lines[header_end:]:
        parts = line.strip().split()
        if len(parts) >= 6:
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                if r == 0 and g == 0 and b == 0:  # skip uncolored points
                    continue
                points.append([x, y, z])
                colors.append([r, g, b])
            except ValueError:
                continue
    
    return np.array(points), np.array(colors)

def merge_scans_simple(session_dir):
    """Simple merge without trajectory - just concatenate all scans"""
    session_path = Path(session_dir)
    
    scan_dirs = sorted([d for d in session_path.iterdir() 
                       if d.is_dir() and d.name.startswith('fusion_scan_')])
    
    all_points = []
    all_colors = []
    
    for scan_dir in scan_dirs:
        candidates = [
            "world_colored_exact.ply", "world_colored_pointcloud.ply",
            "world_colored.ply", "sensor_colored_exact.ply",
            "sensor_colored_pointcloud.ply", "sensor_colored.ply",
        ]
        colored_ply = next((scan_dir / n for n in candidates if (scan_dir / n).exists()), None)

        if colored_ply is None:
            print(f"Warning: No colored point cloud for {scan_dir.name}, skipping")
            continue

        points, colors = load_ply(colored_ply)
        all_points.append(points)
        all_colors.append(colors)
        print(f"  {scan_dir.name}: {len(points)} points")

    if not all_points:
        print("No points to merge")
        return False

    merged_points = np.vstack(all_points)
    merged_colors = np.vstack(all_colors)

    output_file = session_path / "merged_pointcloud.ply"
    save_ply(output_file, merged_points, merged_colors)

    print(f"\n✓ Merged point cloud saved: {output_file}")
    print(f"  Total points: {len(merged_points)}")
    print("  Note: Merged without trajectory alignment (scans may not be aligned)")

    return True

def save_ply(output_file, points, colors):
    """Save merged point cloud to PLY"""
    with open(output_file, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {len(points)}\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property uchar red\n')
        f.write('property uchar green\n')
        f.write('property uchar blue\n')
        f.write('end_header\n')
        
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i]
            f.write(f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n')

def transform_points(points, position, orientation):
    """Transform points from sensor frame to world frame"""
    # Build rotation matrix from quaternion
    x, y, z, w = orientation['x'], orientation['y'], orientation['z'], orientation['w']
    rot = R.from_quat([x, y, z, w])
    R_mat = rot.as_matrix()
    
    # Translation vector
    t = np.array([position['x'], position['y'], position['z']])
    
    # Transform: p_world = R * p_sensor + t
    points_world = (R_mat @ points.T).T + t
    
    return points_world

def pose_matrix_from_trajectory(traj):
    """Return 4x4 pose matrix interpolated to capture_time from full_trajectory."""
    si = traj.get('scan_info', {})
    target = si.get('capture_time') or si.get('scan_request_time')
    full = traj.get('full_trajectory', [])

    if target and full:
        times = np.array([p['timestamp'] for p in full])
        idx = np.searchsorted(times, target)

        if idx == 0:
            entry = full[0]
        elif idx >= len(full):
            entry = full[-1]
        else:
            p0, p1 = full[idx - 1], full[idx]
            t0, t1 = times[idx - 1], times[idx]
            alpha = (target - t0) / (t1 - t0) if t1 != t0 else 0.0

            pos0 = np.array([p0['position']['x'], p0['position']['y'], p0['position']['z']])
            pos1 = np.array([p1['position']['x'], p1['position']['y'], p1['position']['z']])
            t_interp = pos0 + alpha * (pos1 - pos0)

            q0 = [p0['orientation']['x'], p0['orientation']['y'],
                  p0['orientation']['z'], p0['orientation']['w']]
            q1 = [p1['orientation']['x'], p1['orientation']['y'],
                  p1['orientation']['z'], p1['orientation']['w']]
            r_interp = Slerp([0.0, 1.0], R.from_quat([q0, q1]))(alpha)

            T = np.eye(4)
            T[:3, :3] = r_interp.as_matrix()
            T[:3, 3] = t_interp
            return T

        pos = entry.get('position', {})
        ori = entry.get('orientation', {})
    else:
        cp = traj.get('current_pose', {})
        entry = cp.get('lidar_pose', cp)
        pos = entry.get('position', {})
        ori = entry.get('orientation', {})

    T = np.eye(4)
    T[:3, :3] = R.from_quat([ori.get('x', 0), ori.get('y', 0),
                              ori.get('z', 0), ori.get('w', 1)]).as_matrix()
    T[:3, 3] = [pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)]
    return T


def merge_scans_with_trajectory(session_dir):
    """Merge scans into the reference frame of the first scan.

    Each scan's sensor-frame points are transformed by T1_inv @ TN, where T1
    is the first scan's absolute odom pose and TN is the current scan's pose.
    This cancels the scanner's absolute orientation so all scans share the
    first scan's coordinate frame.
    """
    session_path = Path(session_dir)

    scan_dirs = sorted([d for d in session_path.iterdir()
                        if d.is_dir() and d.name.startswith('fusion_scan_')])

    if not scan_dirs:
        print("No scan directories found")
        return False

    print(f"Merging {len(scan_dirs)} scans using trajectory poses...")

    if not (scan_dirs[0] / "trajectory.json").exists():
        print("✗ No trajectory data found - RKO-LIO was not running.")
        print("  Cannot merge without poses. Re-run with a working LIO or enable ICP alignment.")
        return False

    # sensor_colored_exact.ply is always sensor-frame (world_colored_exact.ply is identical
    # despite its name — exact_match_fusion saves sensor coords under both filenames).
    SENSOR_CANDIDATES = [
        "sensor_colored_exact.ply", "sensor_colored_pointcloud.ply", "sensor_colored.ply",
        "world_colored_exact.ply", "world_colored_pointcloud.ply", "world_colored.ply",
    ]

    # Build pose matrices for every scan first so we can compute T1_inv once.
    pose_matrices = {}
    for scan_dir in scan_dirs:
        traj_file = scan_dir / "trajectory.json"
        if traj_file.exists():
            with open(traj_file) as f:
                traj = json.load(f)
            pose_matrices[scan_dir.name] = pose_matrix_from_trajectory(traj)

    if not pose_matrices:
        print("✗ No trajectory data found in any scan - RKO-LIO was not running.")
        print("  Cannot merge without poses. Re-run with a working LIO or enable ICP alignment.")
        return False

    # The LiDAR has a static mounting tilt so the odom Z-axis is not true vertical.
    # Extract roll/pitch from the first scan's pose and build a gravity-alignment
    # correction that levels the merged cloud (yaw left at 0 to keep X forward).
    first_name = scan_dirs[0].name
    T_first = pose_matrices[first_name]
    euler_first = R.from_matrix(T_first[:3, :3]).as_euler('xyz', degrees=True)
    print(f"  Odom tilt: roll={euler_first[0]:.2f} pitch={euler_first[1]:.2f} yaw={euler_first[2]:.2f} deg")
    t_ref = T_first[:3, 3]

    all_points = []
    all_colors = []

    for scan_dir in scan_dirs:
        colored_ply = next((scan_dir / n for n in SENSOR_CANDIDATES if (scan_dir / n).exists()), None)
        if colored_ply is None:
            print(f"Warning: No colored point cloud for {scan_dir.name}, skipping")
            continue

        points, colors = load_ply(colored_ply)

        if len(points) == 0:
            print(f"⚠ No colored points in {scan_dir.name}, skipping")
            continue

        if scan_dir.name not in pose_matrices:
            print(f"⚠ No trajectory for {scan_dir.name}, using identity")
            points_transformed = points
        else:
            T = pose_matrices[scan_dir.name]
            # If world_lidar.ply exists the points are already motion-compensated in
            # absolute world frame — just apply the relative translation to the first
            # scan's origin so all scans share the same reference frame.
            world_ply_exists = (scan_dir / "world_lidar.ply").exists() and colored_ply.name.startswith("world_")
            if world_ply_exists:
                t_rel = T[:3, 3] - t_ref
                points_transformed = points + t_rel
            else:
                T_rel = np.linalg.inv(T_first) @ T
                pts_h = np.hstack([points, np.ones((len(points), 1))])
                points_transformed = (T_rel @ pts_h.T).T[:, :3]
            t_rel = T[:3, 3] - t_ref
            print(f"  {scan_dir.name}: {len(points)} points  rel_t=[{t_rel[0]:.3f},{t_rel[1]:.3f},{t_rel[2]:.3f}]")

        all_points.append(points_transformed)
        all_colors.append(colors)

    if not all_points:
        print("No points to merge")
        return False

    merged_points = np.vstack(all_points)
    merged_colors = np.vstack(all_colors)

    output_file = session_path / "merged_pointcloud.ply"
    save_ply(output_file, merged_points, merged_colors)

    print(f"\n✓ Merged point cloud saved: {output_file}")
    print(f"  Total points: {len(merged_points)}")

    return True

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 merge_with_trajectory.py <session_directory>")
        sys.exit(1)
    
    merge_scans_with_trajectory(sys.argv[1])
