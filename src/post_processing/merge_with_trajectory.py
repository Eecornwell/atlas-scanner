#!/usr/bin/env python3
"""
Merge colored point clouds using trajectory poses.

KEY: Points in sensor_colored_exact.ply are in SENSOR frame.
Use R1.T @ R.T to align rotations (double transpose).
"""

import sys
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

def load_ply(ply_file):
    """Load points and colors from PLY file"""
    points = []
    colors = []
    
    with open(ply_file, 'r') as f:
        lines = f.readlines()
    
    header_end = next(i+1 for i, line in enumerate(lines) if line.strip() == 'end_header')
    
    for line in lines[header_end:]:
        parts = line.strip().split()
        if len(parts) >= 6:
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
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
    """Return 4x4 pose matrix for this scan, using the full_trajectory entry
    closest to scan_request_time."""
    request_time = traj.get('scan_info', {}).get('scan_request_time')
    full = traj.get('full_trajectory', [])
    if request_time and full:
        best = min(full, key=lambda p: abs(p['timestamp'] - request_time))
    else:
        cp = traj.get('current_pose', {})
        best = cp.get('lidar_pose', cp)
    pos = best.get('position', {})
    ori = best.get('orientation', {})
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

    # Reference frame = first scan's pose
    first_name = scan_dirs[0].name
    T_ref_inv = np.linalg.inv(pose_matrices[first_name])

    all_points = []
    all_colors = []

    for scan_dir in scan_dirs:
        colored_ply = next((scan_dir / n for n in SENSOR_CANDIDATES if (scan_dir / n).exists()), None)
        if colored_ply is None:
            print(f"Warning: No colored point cloud for {scan_dir.name}, skipping")
            continue

        points, colors = load_ply(colored_ply)

        if scan_dir.name not in pose_matrices:
            print(f"⚠ No trajectory for {scan_dir.name}, using identity")
            points_transformed = points
        else:
            T = pose_matrices[scan_dir.name]
            # Transform: sensor -> odom -> ref frame of scan_001
            T_rel = T_ref_inv @ T
            pts_h = np.hstack([points, np.ones((len(points), 1))])
            points_transformed = (T_rel @ pts_h.T).T[:, :3]
            t = T_rel[:3, 3]
            print(f"  {scan_dir.name}: {len(points)} points  rel_t=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}]")

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
