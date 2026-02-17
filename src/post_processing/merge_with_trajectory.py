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
        colored_ply = scan_dir / "world_colored_exact.ply"
        if not colored_ply.exists():
            colored_ply = scan_dir / "sensor_colored_exact.ply"
        
        if not colored_ply.exists():
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

def merge_scans_with_trajectory(session_dir):
    """Merge scans by transforming all using trajectory poses"""
    session_path = Path(session_dir)
    
    scan_dirs = sorted([d for d in session_path.iterdir() 
                       if d.is_dir() and d.name.startswith('fusion_scan_')])
    
    if not scan_dirs:
        print("No scan directories found")
        return False
    
    print(f"Merging {len(scan_dirs)} scans using trajectory poses...")
    
    # Check if first scan has trajectory
    traj_file = scan_dirs[0] / "trajectory.json"
    if not traj_file.exists():
        print("⚠ No trajectory data available, merging without transformation")
        return merge_scans_simple(session_dir)
    
    all_points = []
    all_colors = []
    
    for i, scan_dir in enumerate(scan_dirs):
        colored_ply = scan_dir / "world_colored_exact.ply"
        if not colored_ply.exists():
            colored_ply = scan_dir / "sensor_colored_exact.ply"
        
        if not colored_ply.exists():
            print(f"Warning: No colored point cloud for {scan_dir.name}, skipping")
            continue
        
        points, colors = load_ply(colored_ply)
        
        traj_file = scan_dir / "trajectory.json"
        if not traj_file.exists():
            print(f"⚠ No trajectory for {scan_dir.name}, using identity transform")
            points_transformed = points
        else:
            with open(traj_file) as f:
                traj = json.load(f)
            
            # Get lidar pose (position and orientation)
            if 'current_pose' in traj and 'lidar_pose' in traj['current_pose']:
                lidar_pose = traj['current_pose']['lidar_pose']
                position = lidar_pose['position']
                orientation = lidar_pose['orientation']
                
                # Build transformation matrix
                pos = np.array([position['x'], position['y'], position['z']])
                quat = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
                rot = R.from_quat(quat)
                R_mat = rot.as_matrix()
                
                # Transform points: p_world = R * p_sensor + t
                points_transformed = (R_mat @ points.T).T + pos
                
                # Debug: show sample transformation
                if len(points) > 0:
                    sample_before = points[0]
                    sample_after = points_transformed[0]
                    print(f"    Sample: {sample_before} -> {sample_after}")
                    print(f"    Pose: t=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            else:
                print(f"⚠ Invalid trajectory format for {scan_dir.name}, using identity")
                points_transformed = points
        
        all_points.append(points_transformed)
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
    
    return True

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 merge_with_trajectory.py <session_directory>")
        sys.exit(1)
    
    merge_scans_with_trajectory(sys.argv[1])
