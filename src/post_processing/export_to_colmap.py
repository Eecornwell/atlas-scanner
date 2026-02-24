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

# ROS odom (X-forward, Y-left, Z-up) -> COLMAP world
# ROS+X(fwd)->COLMAP+X, ROS+Z(up)->COLMAP-Y (up in GUI), ROS+Y(left)->COLMAP+Z
R_ROS2COLMAP = np.array([
    [ 1,  0,  0],   # ROS+X -> COLMAP+X
    [ 0,  0, -1],   # ROS+Z -> COLMAP-Y (up in GUI, COLMAP+Y=down)
    [ 0,  1,  0],   # ROS+Y -> COLMAP+Z
], dtype=float)

def _load_T_lidar_camera():
    """Return T_lidar_camera (4x4): camera position/orientation in lidar frame."""
    calib_path = Path.home() / "atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml"
    with open(calib_path) as f:
        calib = yaml.safe_load(f)
    # fusion_calibration.yaml stores T_camera_lidar (lidar->camera)
    T_camera_lidar = np.eye(4)
    T_camera_lidar[:3, :3] = R.from_euler('xyz', [
        calib['roll_offset'], calib['pitch_offset'], calib['yaw_offset']
    ]).as_matrix()
    T_camera_lidar[:3, 3] = [calib['x_offset'], calib['y_offset'], calib['z_offset']]
    return np.linalg.inv(T_camera_lidar)  # invert once to get T_lidar_camera

# Precompute the azimuth offset of cam_Z in the lidar frame (constant for a given calibration).
# The Insta360 SDK gravity-stabilizes the ERP: image top = world up, lon=0 = the horizontal
# azimuth direction the physical camera forward axis (cam_Z) points toward.
# Since roll=~178 deg, cam_Z points nearly straight down, but its horizontal projection
# defines the lon=0 direction of the panorama.
def _cam_Z_azimuth_offset(T_lidar_camera):
    cam_Z_lidar = T_lidar_camera[:3, 2]
    return np.arctan2(cam_Z_lidar[1], cam_Z_lidar[0])


def ros_pose_to_colmap_w2c(lidar_pos_xyz, lidar_quat_xyzw, T_lidar_camera, camera_quat_xyzw=None):
    """Convert a ROS odom lidar pose to COLMAP w2c (qw,qx,qy,qz) + T.

    The Insta360 SDK stitches the ERP in the camera's own frame (not gravity-stabilised).
    We use the full camera c2w rotation from camera_pose.orientation directly.
    ERP convention: +X=right, +Y=down, +Z=forward in camera frame.
    """
    R_lidar_w = R.from_quat(lidar_quat_xyzw).as_matrix()

    if camera_quat_xyzw is not None:
        # Full camera c2w rotation in ROS world — use directly, no yaw-only extraction
        R_c2w = R_ROS2COLMAP @ R.from_quat(camera_quat_xyzw).as_matrix()
    else:
        # Fallback: derive camera orientation from lidar pose + calibration
        R_cam_c2w = R_lidar_w @ T_lidar_camera[:3, :3]
        R_c2w = R_ROS2COLMAP @ R_cam_c2w

    R_w2c = R_c2w.T

    C_ros = np.array(lidar_pos_xyz) + R_lidar_w @ T_lidar_camera[:3, 3]
    C_colmap = R_ROS2COLMAP @ C_ros
    T = -R_w2c @ C_colmap

    q = R.from_matrix(R_w2c).as_quat()
    return [q[3], q[0], q[1], q[2]], T.tolist()

def export_to_colmap(session_dir):
    """Export scan session to COLMAP format"""
    session_path = Path(session_dir)
    
    # Create COLMAP directory structure
    colmap_dir = session_path / "colmap"
    sparse_dir = colmap_dir / "init_sparse" / "0"
    images_dir = colmap_dir / "pano_images"
    
    sparse_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Exporting to COLMAP format: {colmap_dir}")
    
    # Load camera-lidar calibration
    T_lidar_camera = _load_T_lidar_camera()
    
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
            qw_c2w, qx_c2w, qy_c2w, qz_c2w = 1.0, 0.0, 0.0, 0.0
            t_c2w = np.array([0.0, 0.0, 0.0])
            timestamp = 0.0
        else:
            with open(traj_file, 'r') as f:
                traj = json.load(f)
            
            # Use lidar_pose orientation (gravity-aligned) with camera_pose position.
            # The ERP image up-direction is defined by the lidar's gravity-aligned frame.
            # Camera is physically inverted but ERP image is stitched right-side up.
            if 'current_pose' in traj:
                pose_data = traj['current_pose']
                timestamp = pose_data.get('timestamp', 0.0)
                cam_quat_xyzw = None

                if 'camera_pose' in pose_data and 'lidar_pose' in pose_data:
                    lid_pos = pose_data['lidar_pose']['position']
                    lid_ori = pose_data['lidar_pose']['orientation']
                    pos_xyz = [lid_pos['x'], lid_pos['y'], lid_pos['z']]
                    quat_xyzw = [lid_ori['x'], lid_ori['y'], lid_ori['z'], lid_ori['w']]
                    cam_ori = pose_data['camera_pose']['orientation']
                    cam_quat_xyzw = [cam_ori['x'], cam_ori['y'], cam_ori['z'], cam_ori['w']]
                elif 'camera_pose' in pose_data:
                    cam = pose_data['camera_pose']
                    pos_xyz = [cam['position']['x'], cam['position']['y'], cam['position']['z']]
                    quat_xyzw = [cam['orientation']['x'], cam['orientation']['y'], cam['orientation']['z'], cam['orientation']['w']]
                elif 'lidar_pose' in pose_data:
                    p = pose_data['lidar_pose']
                    pos_xyz = [p['position']['x'], p['position']['y'], p['position']['z']]
                    quat_xyzw = [p['orientation']['x'], p['orientation']['y'], p['orientation']['z'], p['orientation']['w']]
                else:
                    p = pose_data
                    pos_xyz = [p['position']['x'], p['position']['y'], p['position']['z']]
                    quat_xyzw = [p['orientation']['x'], p['orientation']['y'], p['orientation']['z'], p['orientation']['w']]
            elif 'poses' in traj and traj['poses']:
                p = traj['poses'][-1]
                timestamp = p.get('timestamp', 0.0)
                pos_xyz = [p['position']['x'], p['position']['y'], p['position']['z']]
                quat_xyzw = [p['orientation']['x'], p['orientation']['y'], p['orientation']['z'], p['orientation']['w']]
            else:
                print(f"Empty trajectory for {scan_dir.name}, using identity pose")
                pos_xyz = [0.0, 0.0, 0.0]
                quat_xyzw = [0.0, 0.0, 0.0, 1.0]
                timestamp = 0.0

            (qw_c2w, qx_c2w, qy_c2w, qz_c2w), t_c2w_list = ros_pose_to_colmap_w2c(pos_xyz, quat_xyzw, T_lidar_camera, cam_quat_xyzw)
            t_c2w = np.array(t_c2w_list)
        
        # Find equirectangular image - prioritize blended masked version
        image_candidates = [
            scan_dir / "equirect_blended_masked.png",  # Highest priority: blended + masked
            scan_dir / "equirect_*_masked.png",  # Second: any masked version
            scan_dir / "equirect_*_raw_masked.png",  # Third: raw masked
            scan_dir / "equirect_*.jpg"  # Fallback: original
        ]
        
        erp_image = None
        for pattern in image_candidates:
            if '*' in pattern.name:
                matches = list(scan_dir.glob(pattern.name))
                if matches:
                    erp_image = matches[0]
                    break
            else:
                if pattern.exists():
                    erp_image = pattern
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
    
    # Use merged_pointcloud.ply as the authoritative lidar source if available,
    # otherwise fall back to accumulating per-scan colored clouds.
    merged_src = session_path / "merged_pointcloud.ply"
    if merged_src.exists():
        all_points = load_ply_points(merged_src)
        for p in all_points:
            xyz = R_ROS2COLMAP @ np.array(p[:3])
            p[0], p[1], p[2] = xyz[0], xyz[1], xyz[2]
        print(f"Using merged_pointcloud.ply ({len(all_points)} points)")
    else:
        for scan_dir in scan_dirs:
            colored_ply = scan_dir / "world_colored_exact.ply"
            if not colored_ply.exists():
                colored_ply = scan_dir / "world_colored.ply"
            if colored_ply.exists():
                points = load_ply_points(colored_ply)
                for p in points:
                    xyz = R_ROS2COLMAP @ np.array(p[:3])
                    p[0], p[1], p[2] = xyz[0], xyz[1], xyz[2]
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
    
    # Write points3D.txt with lidar points for Gaussian splat initialization
    with open(sparse_dir / "points3D.txt", 'w') as f:
        f.write("# POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]\n")
        for i, p in enumerate(all_points, start=1):
            f.write(f"{i} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(p[3])} {int(p[4])} {int(p[5])} 0\n")
    
    # Save merged colored point cloud as sparse.ply
    if all_points:
        save_ply(sparse_dir / "sparse.ply", all_points)
        print(f"✓ Saved {len(all_points)} points to sparse.ply")
    
    print(f"✓ Exported {len(images_data)} scans to COLMAP format")
    print(f"  - Images: {images_dir}")
    print(f"  - Sparse model: {sparse_dir}")
    
    return True

def load_ply_points(ply_file):
    """Load points from PLY file (ASCII or binary)."""
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(str(ply_file))
    pts = np.asarray(pcd.points)
    cols = (np.asarray(pcd.colors) * 255).astype(int) if pcd.has_colors() else np.zeros((len(pts), 3), int)
    return [[*pts[i].tolist(), *cols[i].tolist()] for i in range(len(pts))]

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
