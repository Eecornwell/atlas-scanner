#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Colors a sensor-frame PLY by projecting points into the equirectangular image using the exact same bearing-vector projection convention as direct_visual_lidar_calibration.
"""
Sensor fusion that EXACTLY matches direct_visual_lidar_calibration projection.

Key: The calibration tool uses:
  lon = atan2(bearing[0], bearing[2])  # X forward, Z right
  lat = -asin(bearing[1])              # Y up

The input is always sensor_lidar.ply (sensor frame) — the ERP image is also
in sensor frame (the camera is rigidly attached to the scanner), so
T_camera_lidar maps sensor-frame LiDAR points directly into the camera frame.

In SDK stitch continuous mode, sensor_lidar.ply is motion-compensated to the
capture-time sensor frame by reconstruct_from_bag.py, so it already matches
the ERP orientation. Each SDK-stitched ERP is independently gravity-aligned
by the camera's internal IMU (DYNAMICSTITCH), so each scan's ERP correctly
represents that scan's own sensor-frame orientation. No cross-scan ERP
reference or EIS correction is needed or correct.

The resulting sensor_colored_exact.ply is in sensor frame. The posegraph /
merge step applies the trajectory pose (T_world_lidar) to bring it to world
frame, exactly as in stationary mode.
"""

import numpy as np
import cv2
import sys
import os
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def load_points(ply_file):
    try:
        safe = _safe_data(ply_file)
    except ValueError as e:
        raise ValueError(f"load_points: {e}") from e
    with open(safe, 'rb') as f:
        header_bytes = b''
        while True:
            line = f.readline()
            header_bytes += line
            if line.strip() == b'end_header':
                break
        header = header_bytes.decode('ascii')
        binary = 'binary_little_endian' in header
        n_verts = int(next(l.split()[-1] for l in header.splitlines() if l.startswith('element vertex')))
        fields = [l.split()[-1] for l in header.splitlines() if l.startswith('property float')]
        if binary:
            data = np.frombuffer(f.read(n_verts * len(fields) * 4), dtype=np.float32).reshape(n_verts, len(fields))
            return data[:, :3]
        else:
            lines = f.read().decode('ascii').splitlines()
            pts = []
            for line in lines:
                parts = line.strip().split()
                if len(parts) >= 3:
                    try: pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except ValueError: continue
            return np.array(pts)

def exact_match_calibration_tool(scan_dir):
    try:
        safe_scan = _safe_data(scan_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return False

    sensor_ply = next((
        str(_safe_data(safe_scan / f))
        for f in os.listdir(str(safe_scan))
        if 'sensor_lidar' in f and f.endswith('.ply')
        and _ALLOWED_DATA in [_safe_data(safe_scan / f), *_safe_data(safe_scan / f).parents]
    ), None)

    # Find image - prefer masked PNG, then regular ERP JPG
    mask_file = None
    image_file = None
    for f in os.listdir(str(safe_scan)):
        if '_raw' in f:
            continue
        try:
            candidate = _safe_data(safe_scan / f)
        except ValueError:
            continue
        if f.endswith('_masked.png'):
            mask_file = str(candidate)
        elif ('equirect' in f or 'equirectangular' in f) and f.endswith('.jpg'):
            image_file = str(candidate)

    if mask_file and os.path.exists(mask_file):
        image = cv2.imread(mask_file, cv2.IMREAD_UNCHANGED)
        if image is not None and image.ndim == 3 and image.shape[2] == 4:
            alpha_mask = image[:, :, 3] >= 128
            image = image[:, :, :3]
        else:
            image = cv2.imread(mask_file)
            alpha_mask = None
    elif image_file and os.path.exists(image_file):
        image = cv2.imread(image_file)
        alpha_mask = None
    else:
        image = None

    if sensor_ply is None or image is None:
        print("Missing sensor PLY or image")
        return False

    points = load_points(sensor_ply)
    if points.ndim < 2 or len(points) == 0:
        print("Empty point cloud, skipping")
        return False
    img_height, img_width = image.shape[:2]

    # Load calibration — use per-camera calibration if .cam_index exists
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from camera_hw import calibration_path, cam_index_for_scan, camera_hw_for_session
    _session_dir = os.path.dirname(scan_dir)
    _hw = camera_hw_for_session(_session_dir)
    _cam_idx = cam_index_for_scan(scan_dir)
    config_path = str(calibration_path(_hw, _cam_idx))
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    roll  = config['roll_offset']
    pitch = config['pitch_offset']
    yaw   = config['yaw_offset']
    t_x   = config['x_offset']
    t_y   = config['y_offset']
    t_z   = config['z_offset']
    skip_rate = config.get('skip_rate', 1)

    print(f"Transformation (T_camera_lidar):")
    print(f"  Translation: [{t_x:.4f}, {t_y:.4f}, {t_z:.4f}]")
    print(f"  Rotation (RPY): [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")

    if skip_rate > 1:
        points = points[::skip_rate]

    # Build T_camera_lidar: sensor LiDAR frame -> camera frame
    R_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T_camera_lidar = np.eye(4)
    T_camera_lidar[:3, :3] = R_matrix
    T_camera_lidar[:3, 3] = [t_x, t_y, t_z]

    # Filter points
    distances = np.linalg.norm(points, axis=1)
    valid_mask = (distances > 0.8) & (distances < 8.0) & (points[:, 2] > 0.05)
    points = points[valid_mask]

    print(f"Processing {len(points)} points...")

    points_h = np.hstack([points, np.ones((len(points), 1))])
    points_camera = (T_camera_lidar @ points_h.T).T[:, :3]

    # Equirectangular projection (matches direct_visual_lidar_calibration)
    bearing = points_camera / np.linalg.norm(points_camera, axis=1, keepdims=True)
    lat = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
    lon = np.arctan2(bearing[:, 0], bearing[:, 2])
    u = img_width  * (0.5 + lon / (2 * np.pi))
    v = img_height * (0.5 - lat / np.pi)

    print(f"Projection range: u=[{u.min():.1f}, {u.max():.1f}], v=[{v.min():.1f}, {v.max():.1f}]")

    valid_pixels = (u >= 0) & (u < img_width) & (v >= 0) & (v < img_height)

    if alpha_mask is not None:
        u_int = np.clip(np.round(u).astype(int), 0, img_width - 1)
        v_int = np.clip(np.round(v).astype(int), 0, img_height - 1)
        valid_pixels = valid_pixels & alpha_mask[v_int, u_int]

    valid_points = points[valid_pixels]
    valid_u = u[valid_pixels]
    valid_v = v[valid_pixels]

    print(f"Valid projections: {len(valid_points)} points")

    # Bilinear interpolation — blends 4 surrounding pixels for each projected
    # point instead of snapping to nearest pixel, giving sharper color edges.
    u0 = np.clip(np.floor(valid_u).astype(int), 0, img_width - 1)
    v0 = np.clip(np.floor(valid_v).astype(int), 0, img_height - 1)
    u1 = np.clip(u0 + 1, 0, img_width - 1)
    v1 = np.clip(v0 + 1, 0, img_height - 1)
    fu = (valid_u - np.floor(valid_u))[:, np.newaxis]
    fv = (valid_v - np.floor(valid_v))[:, np.newaxis]
    c00 = image[v0, u0].astype(np.float32)
    c10 = image[v0, u1].astype(np.float32)
    c01 = image[v1, u0].astype(np.float32)
    c11 = image[v1, u1].astype(np.float32)
    colors_bgr = (c00 * (1 - fu) * (1 - fv) + c10 * fu * (1 - fv) +
                  c01 * (1 - fu) * fv       + c11 * fu * fv)
    colors = np.clip(colors_bgr, 0, 255).astype(np.uint8)[:, [2, 1, 0]]  # BGR to RGB

    def write_ply(path, pts, cols):
        try:
            safe_path = _safe_data(path)
        except ValueError as e:
            print(f"Error: write_ply path rejected: {e}")
            return
        with open(safe_path, 'w') as f:
            f.write('ply\nformat ascii 1.0\n')
            f.write(f'element vertex {len(pts)}\n')
            f.write('property float x\nproperty float y\nproperty float z\n')
            f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
            for i in range(len(pts)):
                x, y, z = pts[i]
                r, g, b = cols[i]
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n')

    sensor_output = str(_safe_data(safe_scan / "sensor_colored_exact.ply"))
    write_ply(sensor_output, valid_points, colors)
    print(f"\u2713 Saved sensor-frame colored points to {sensor_output}")

    # world_colored_exact.ply: same sensor-frame data (world transform applied by posegraph/merge)
    world_output = str(_safe_data(safe_scan / "world_colored_exact.ply"))
    write_ply(world_output, valid_points, colors)
    return True

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 exact_match_fusion.py <scan_directory>")
        sys.exit(1)
    try:
        _safe_data(sys.argv[1])
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    exact_match_calibration_tool(sys.argv[1])
