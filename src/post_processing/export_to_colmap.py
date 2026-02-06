#!/usr/bin/env python3
"""
Export scan session to COLMAP format.
Creates sparse reconstruction with camera poses and colored point cloud.
"""

import os
import sys
import json
import yaml
import shutil
import numpy as np
import cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as R

def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    """Convert quaternion to rotation matrix"""
    return R.from_quat([qx, qy, qz, qw]).as_matrix()

def rotation_matrix_to_quaternion(rot_mat):
    """Convert rotation matrix to quaternion (w, x, y, z)"""
    q = R.from_matrix(rot_mat).as_quat()  # Returns [x, y, z, w]
    return [q[3], q[0], q[1], q[2]]  # Return [w, x, y, z]

def export_to_colmap(session_dir):
    """Export scan session to COLMAP format"""
    session_path = Path(session_dir)
    
    # Create COLMAP directory structure
    colmap_dir = session_path / "colmap"
    sparse_dir = colmap_dir / "sparse" / "0"
    images_dir = colmap_dir / "images"
    
    sparse_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Exporting to COLMAP format: {colmap_dir}")
    
    # Find all scan directories
    scan_dirs = sorted([d for d in session_path.iterdir() 
                       if d.is_dir() and d.name.startswith('fusion_scan_')])
    
    if not scan_dirs:
        print("No scan directories found")
        return False
    
    # Load camera calibration
    calib_path = Path.home() / "atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml"
    with open(calib_path, 'r') as f:
        calib = yaml.safe_load(f)
    
    img_width = calib.get('image_width', 1920)
    img_height = calib.get('image_height', 960)
    
    # Write cameras.txt (equirectangular camera model)
    with open(sparse_dir / "cameras.txt", 'w') as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        f.write(f"1 SIMPLE_RADIAL {img_width} {img_height} {img_width/2} {img_width/2} {img_height/2} 0\n")
    
    # Write rigs.txt (empty - not used)
    with open(sparse_dir / "rigs.txt", 'w') as f:
        f.write("# Rig configuration (empty)\n")
    
    # Note: frames.txt is not written - COLMAP doesn't use it in text format
    # Collect images and poses
    images_data = []
    all_points = []
    
    for idx, scan_dir in enumerate(scan_dirs, start=1):
        # Find trajectory file - prefer ICP-refined if available
        traj_file_refined = scan_dir / "trajectory_icp_refined.json"
        traj_file = scan_dir / "trajectory.json"
        
        if traj_file_refined.exists():
            traj_file = traj_file_refined
            print(f"Using ICP-refined pose for {scan_dir.name}")
        
        # Use identity pose if no trajectory
        if not traj_file.exists():
            print(f"No trajectory for {scan_dir.name}, using identity pose")
            # Identity transformation
            qw_c2w, qx_c2w, qy_c2w, qz_c2w = 1.0, 0.0, 0.0, 0.0
            t_c2w = np.array([0.0, 0.0, 0.0])
            timestamp = 0.0
        else:
            with open(traj_file, 'r') as f:
                traj = json.load(f)
            
            # Get current pose (use camera_pose if available, fallback to lidar_pose)
            if 'current_pose' in traj:
                pose_data = traj['current_pose']
                if 'camera_pose' in pose_data:
                    pose = pose_data['camera_pose']
                elif 'lidar_pose' in pose_data:
                    pose = pose_data['lidar_pose']
                else:
                    pose = {'position': pose_data['position'], 'orientation': pose_data['orientation']}
                timestamp = pose_data.get('timestamp', 0.0)
            elif 'poses' in traj and traj['poses']:
                pose = traj['poses'][-1]
                timestamp = pose.get('timestamp', 0.0)
            else:
                print(f"Empty trajectory for {scan_dir.name}, using identity pose")
                qw_c2w, qx_c2w, qy_c2w, qz_c2w = 1.0, 0.0, 0.0, 0.0
                t_c2w = np.array([0.0, 0.0, 0.0])
                timestamp = 0.0
                pose = None
            
            if pose:
                tx, ty, tz = pose['position']['x'], pose['position']['y'], pose['position']['z']
                qx, qy, qz, qw = pose['orientation']['x'], pose['orientation']['y'], pose['orientation']['z'], pose['orientation']['w']
                
                # Trajectory: camera position (C) + c2w quaternion
                # COLMAP: w2c quaternion (qw,qx,qy,qz) + T where C = -R_w2c^T * T
                # So: T = -R_w2c * C
                
                R_c2w = R.from_quat([qx, qy, qz, qw]).as_matrix()
                R_w2c = R_c2w.T
                C = np.array([tx, ty, tz])
                T = -R_w2c @ C
                
                quat_w2c = R.from_matrix(R_w2c).as_quat()
                qw_c2w, qx_c2w, qy_c2w, qz_c2w = quat_w2c[3], quat_w2c[0], quat_w2c[1], quat_w2c[2]
                t_c2w = T
        
        # Find equirectangular image - prioritize masked blended version
        image_candidates = [
            scan_dir / "equirect_*_masked.png",  # Highest priority: masked + blended
            scan_dir / "equirect_*_raw_masked.png",  # Second: masked only
            scan_dir / "equirect_*.jpg"  # Fallback: original
        ]
        
        erp_image = None
        for pattern in image_candidates:
            matches = list(scan_dir.glob(pattern.name))
            if matches:
                erp_image = matches[0]
                break
        
        if not erp_image:
            print(f"Warning: No ERP image for {scan_dir.name}, skipping")
            continue
        image_name = f"scan_{idx:03d}.png"
        mask_name = f"scan_{idx:03d}.png.mask.png"
        
        # Load and process image
        img = cv2.imread(str(erp_image), cv2.IMREAD_UNCHANGED)
        
        if len(img.shape) == 3 and img.shape[2] == 4:  # Has alpha channel
            # Extract alpha as mask
            # COLMAP expects: 0 (black) = ignore, 255 (white) = use
            # Our alpha: 255 = valid, 0 = masked (rig)
            # So we can use alpha directly as the mask
            mask = img[:, :, 3]
            # Save RGB image
            cv2.imwrite(str(images_dir / image_name), img[:, :, :3])
            # Save mask image (alpha channel where 255=valid, 0=masked)
            cv2.imwrite(str(images_dir / mask_name), mask)
            print(f"  Created mask for {scan_dir.name}")
        else:
            # No alpha, just copy image (no mask needed)
            shutil.copy(erp_image, images_dir / image_name)
        
        # Store image data for images.txt
        images_data.append({
            'image_id': idx,
            'qw': qw_c2w, 'qx': qx_c2w, 'qy': qy_c2w, 'qz': qz_c2w,
            'tx': t_c2w[0], 'ty': t_c2w[1], 'tz': t_c2w[2],
            'camera_id': 1,
            'name': image_name
        })
        
        # Load colored point cloud
        colored_ply = scan_dir / "world_colored_exact.ply"
        if not colored_ply.exists():
            colored_ply = scan_dir / "world_colored.ply"
        
        if colored_ply.exists():
            points = load_ply_points(colored_ply)
            all_points.extend(points)
    
    # Write images.txt
    with open(sparse_dir / "images.txt", 'w') as f:
        f.write("# Image list with two lines of data per image:\n")
        f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
        for img in images_data:
            f.write(f"{img['image_id']} {img['qw']} {img['qx']} {img['qy']} {img['qz']} ")
            f.write(f"{img['tx']} {img['ty']} {img['tz']} {img['camera_id']} {img['name']}\n")
            f.write("\n")  # Empty POINTS2D line
    
    # Write points3D.txt (empty as requested)
    with open(sparse_dir / "points3D.txt", 'w') as f:
        f.write("# 3D point list (empty - use sparse.ply instead)\n")
        f.write("# POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n")
    
    # Save merged colored point cloud as sparse.ply
    if all_points:
        save_ply(sparse_dir / "sparse.ply", all_points)
        print(f"✓ Saved {len(all_points)} points to sparse.ply")
    
    print(f"✓ Exported {len(images_data)} scans to COLMAP format")
    print(f"  - Images: {images_dir}")
    print(f"  - Sparse model: {sparse_dir}")
    
    return True

def load_ply_points(ply_file):
    """Load points from PLY file"""
    points = []
    with open(ply_file, 'r') as f:
        lines = f.readlines()
    
    header_end = next(i+1 for i, line in enumerate(lines) if line.strip() == 'end_header')
    
    for line in lines[header_end:]:
        parts = line.strip().split()
        if len(parts) >= 6:
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                points.append([x, y, z, r, g, b])
            except ValueError:
                continue
    
    return points

def save_ply(output_file, points):
    """Save points to PLY file"""
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
        
        for p in points:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]} {p[4]} {p[5]}\n')

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 export_to_colmap.py <session_directory>")
        sys.exit(1)
    
    export_to_colmap(sys.argv[1])
