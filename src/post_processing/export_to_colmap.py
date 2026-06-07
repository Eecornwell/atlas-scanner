#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Exports a full scan session to COLMAP sparse reconstruction format, writing cameras.txt, images.txt, and points3D.txt from trajectory poses and colored point clouds.
"""
Export scan session to COLMAP format.
Creates sparse reconstruction with camera poses and colored point cloud.

COORDINATE FRAME CONVENTION:
  - All poses and points are expressed in CAMERA-world space.
  - sensor_colored_exact.ply is in lidar-sensor frame.
  - For each scan we compute T_world_camera and transform the points:
      p_camera_world = R_world_camera @ T_camera_lidar @ p_lidar_sensor + t_world_camera
  - Camera poses (w2c) are the inverse of T_world_camera.
  - Finally R_ROS2COLMAP converts from ROS camera-world to COLMAP world.

  This ensures depth rendered from camera poses matches the point positions,
  which is required for depth-supervised Gaussian splatting training.
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

# ROS world (X-forward, Y-left, Z-up) -> COLMAP world
R_ROS2COLMAP = np.array([
    [ 1,  0,  0],
    [ 0,  0, -1],
    [ 0,  1,  0],
], dtype=float)


def _load_T_camera_lidar():
    """Return T_camera_lidar (4x4): transforms points from lidar frame to camera frame."""
    calib_path = Path.home() / "atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml"
    with open(calib_path) as f:
        calib = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [
        calib['roll_offset'], calib['pitch_offset'], calib['yaw_offset']
    ]).as_matrix()
    T[:3, 3] = [calib['x_offset'], calib['y_offset'], calib['z_offset']]
    return T


# Keep old name as alias for backwards compat with erp_to_perspective_colmap.py
_load_T_lidar_camera = _load_T_camera_lidar


def _get_T_world_camera(pos_xyz, quat_xyzw, T_camera_lidar):
    """Compute T_world_camera (4x4) from a lidar pose and calibration.

    T_world_camera = T_world_lidar @ inv(T_camera_lidar)
    """
    T_lidar_camera = np.linalg.inv(T_camera_lidar)
    T_world_lidar = np.eye(4)
    T_world_lidar[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
    T_world_lidar[:3, 3] = pos_xyz
    return T_world_lidar @ T_lidar_camera


def ros_pose_to_colmap_w2c(pos_xyz, quat_xyzw, T_camera_lidar, camera_quat_xyzw=None):
    """Convert a ROS lidar pose to COLMAP w2c quaternion + translation.

    The camera center and orientation are derived in camera-world space,
    then converted to COLMAP convention.
    """
    if camera_quat_xyzw is not None:
        R_c2w_ros = R.from_quat(camera_quat_xyzw).as_matrix()
        C_ros = np.array(pos_xyz)
    else:
        T_world_camera = _get_T_world_camera(pos_xyz, quat_xyzw, T_camera_lidar)
        R_c2w_ros = T_world_camera[:3, :3]
        C_ros = T_world_camera[:3, 3]

    # Convert to COLMAP world
    R_c2w = R_ROS2COLMAP @ R_c2w_ros
    R_w2c = R_c2w.T
    C_colmap = R_ROS2COLMAP @ C_ros
    T = -R_w2c @ C_colmap
    q = R.from_matrix(R_w2c).as_quat()  # [x,y,z,w]
    return [q[3], q[0], q[1], q[2]], T.tolist()


MIN_BASELINE_M = 0.10


def _filter_by_baseline(images_data, min_dist):
    """Greedy filter: keep a scan only if it is >= min_dist from all kept scans."""
    kept = []
    positions = []
    for img in images_data:
        pos = np.array([img['tx'], img['ty'], img['tz']])
        if not positions or min(np.linalg.norm(pos - p) for p in positions) >= min_dist:
            kept.append(img)
            positions.append(pos)
    if len(kept) < len(images_data):
        print(f"  Filtered {len(images_data) - len(kept)} near-duplicate scan(s) (baseline < {min_dist*100:.0f}cm)")
    return kept


def _load_pose_from_scan(scan_dir):
    """Load lidar pose (pos_xyz, quat_xyzw) from a scan's trajectory file."""
    traj_file = scan_dir / "trajectory_icp_refined.json"
    if not traj_file.exists():
        traj_file = scan_dir / "trajectory.json"
    if not traj_file.exists():
        return None

    with open(traj_file, 'r') as f:
        traj = json.load(f)

    if 'current_pose' in traj:
        pose_data = traj['current_pose']
        lp = pose_data.get('lidar_pose') or pose_data
    elif 'poses' in traj and traj['poses']:
        lp = traj['poses'][-1]
    else:
        return None

    pos_xyz = [lp['position']['x'], lp['position']['y'], lp['position']['z']]
    quat_xyzw = [lp['orientation']['x'], lp['orientation']['y'],
                 lp['orientation']['z'], lp['orientation']['w']]
    return pos_xyz, quat_xyzw


def _transform_scan_points_to_camera_world(points, pos_xyz, quat_xyzw, T_camera_lidar):
    """Transform sensor-frame lidar points to ROS world space using T_world_lidar.

    sensor_colored_exact.ply is in lidar-sensor frame. Applying T_world_lidar
    puts them in the same ROS world where camera centers (from T_world_camera =
    T_world_lidar @ T_lidar_camera) also live. Both get R_ROS2COLMAP after this.
    """
    # T_world_lidar: transforms lidar-sensor points to world
    T_world_lidar = np.eye(4)
    T_world_lidar[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
    T_world_lidar[:3, 3] = pos_xyz

    # Transform points: p_world = T_world_lidar @ p_sensor
    pts_h = np.hstack([points, np.ones((len(points), 1))])
    pts_world = (T_world_lidar @ pts_h.T).T[:, :3]
    return pts_world


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
    T_camera_lidar = _load_T_camera_lidar()

    # Find all scan directories
    scan_dirs = sorted([d for d in session_path.iterdir()
                       if d.is_dir() and d.name.startswith('fusion_scan_')])

    if not scan_dirs:
        print("No scan directories found")
        return False

    # Load camera calibration for image dimensions
    calib_path = Path.home() / "atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml"
    with open(calib_path, 'r') as f:
        calib = yaml.safe_load(f)

    img_width = calib.get('image_width', 1920)
    img_height = calib.get('image_height', 960)

    # Write cameras.txt (SPHERICAL model for equirectangular images)
    with open(sparse_dir / "cameras.txt", 'w') as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        f.write(f"1 SPHERICAL {img_width} {img_height}\n")

    # Collect images, poses, and per-scan point clouds
    images_data = []
    scan_point_data = []  # [(points_ros_world, colors), ...] per kept scan

    for idx, scan_dir in enumerate(scan_dirs, start=1):
        pose_result = _load_pose_from_scan(scan_dir)
        if pose_result is None:
            print(f"No trajectory for {scan_dir.name}, skipping")
            continue

        pos_xyz, quat_xyzw = pose_result

        # Compute COLMAP w2c pose (camera-world -> COLMAP convention)
        (qw, qx, qy, qz), t_list = ros_pose_to_colmap_w2c(
            pos_xyz, quat_xyzw, T_camera_lidar
        )

        # Find equirectangular image
        erp_image = _find_erp_image(scan_dir)
        if not erp_image:
            print(f"Warning: No ERP image for {scan_dir.name}, skipping")
            continue

        image_name = f"scan_{idx:03d}.png"
        mask_name = f"scan_{idx:03d}.png.mask.png"

        # Load and process image
        img = cv2.imread(str(erp_image), cv2.IMREAD_UNCHANGED)
        if img is None:
            print(f"Warning: Cannot read image for {scan_dir.name}, skipping")
            continue

        if len(img.shape) == 3 and img.shape[2] == 4:
            mask = img[:, :, 3]
            cv2.imwrite(str(images_dir / image_name), img[:, :, :3])
            cv2.imwrite(str(images_dir / mask_name), mask)
        else:
            shutil.copy(erp_image, images_dir / image_name)

        images_data.append({
            'image_id': idx,
            'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
            'tx': t_list[0], 'ty': t_list[1], 'tz': t_list[2],
            'camera_id': 1,
            'name': image_name,
            'scan_dir': scan_dir,
            'pos_xyz': pos_xyz,
            'quat_xyzw': quat_xyzw,
        })

    # Filter out scans too close together
    images_data = _filter_by_baseline(images_data, MIN_BASELINE_M)

    # Load and transform point clouds for kept scans only
    # Points are transformed from sensor-frame to ROS world using T_world_lidar,
    # then to COLMAP world with R_ROS2COLMAP. This is the SAME world frame
    # that the camera centers live in (ros_pose_to_colmap_w2c uses the same chain).
    all_points = []
    for img in images_data:
        scan_dir = img['scan_dir']
        # Prefer sensor_colored_exact.ply (guaranteed sensor frame)
        colored_ply = None
        for name in ["sensor_colored_exact.ply", "world_colored_exact.ply",
                     "sensor_colored.ply", "world_colored.ply"]:
            p = scan_dir / name
            if p.exists():
                colored_ply = p
                break

        if colored_ply is None:
            continue

        points, colors = _load_ply_points_and_colors(colored_ply)
        if len(points) == 0:
            continue

        # Transform from lidar-sensor frame to ROS world
        pts_world = _transform_scan_points_to_camera_world(
            points, img['pos_xyz'], img['quat_xyzw'], T_camera_lidar
        )

        # Then to COLMAP world
        pts_colmap = (R_ROS2COLMAP @ pts_world.T).T

        for i in range(len(pts_colmap)):
            all_points.append([*pts_colmap[i].tolist(), *colors[i].tolist()])

    print(f"✓ Loaded {len(all_points)} points from {len(images_data)} kept scans")

    # Remove internal keys before writing
    for img in images_data:
        img.pop('scan_dir', None)
        img.pop('pos_xyz', None)
        img.pop('quat_xyzw', None)

    # Write images.txt
    with open(sparse_dir / "images.txt", 'w') as f:
        f.write("# Image list with two lines of data per image:\n")
        f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
        for img in images_data:
            f.write(f"{img['image_id']} {img['qw']} {img['qx']} {img['qy']} {img['qz']} ")
            f.write(f"{img['tx']} {img['ty']} {img['tz']} {img['camera_id']} {img['name']}\n")
            f.write("\n")  # Empty POINTS2D line

    # Write points3D.txt
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


def _find_erp_image(scan_dir):
    """Find the best equirectangular image in a scan directory."""
    candidates = [
        "equirect_blended_masked.png",
        "equirect_dual_fisheye_masked.png",
        "equirect_dual_fisheye_raw_masked.png",
        "equirect_dual_fisheye.jpg",
    ]
    for name in candidates:
        p = scan_dir / name
        if p.exists():
            return p
    # Glob fallback
    for pattern in ["equirect_*_masked.png", "equirect_*.jpg", "equirect_*.png"]:
        matches = list(scan_dir.glob(pattern))
        if matches:
            return matches[0]
    return None


def _load_ply_points_and_colors(ply_file):
    """Load points and colors from PLY file."""
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(str(ply_file))
    pts = np.asarray(pcd.points)
    if pcd.has_colors():
        cols = (np.asarray(pcd.colors) * 255).astype(int)
    else:
        cols = np.zeros((len(pts), 3), int)
    return pts, cols


def load_ply_points(ply_file):
    """Load points from PLY file as list of [x,y,z,r,g,b]."""
    pts, cols = _load_ply_points_and_colors(ply_file)
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
