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

    # Fallback: any PNG in the scan dir that isn't masked/raw (e.g. oak1_TIMESTAMP.png)
    # For oak1: prefer the undistorted image since calibration used zero distortion.
    if not mask_file and not image_file:
        undistorted_candidates = []
        regular_candidates = []
        for f in sorted(os.listdir(str(safe_scan))):
            if not f.endswith('.png') or '_masked' in f or '_raw' in f:
                continue
            try:
                candidate = str(_safe_data(safe_scan / f))
            except ValueError:
                continue
            if '_undistorted' in f:
                undistorted_candidates.append(candidate)
            else:
                regular_candidates.append(candidate)
        # Prefer undistorted for oak1 (calibrated with zero distortion)
        candidates = undistorted_candidates or regular_candidates
        if candidates:
            image_file = candidates[0]

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

    # Load calibration — resolve by serial from .cam_index so multi-camera
    # sessions use the correct per-slot calibration regardless of what
    # camera_hw session_config.json records (which is always the primary hw).
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from camera_hw import calibration_path, cam_index_for_scan, camera_hw_for_session, load_camera_profile
    import yaml as _yaml
    _session_dir = os.path.dirname(scan_dir)
    _session_hw = camera_hw_for_session(_session_dir)  # from session_config.json

    # Resolve cam_index by serial when available (new .cam_index format written
    # by main.cpp / main_multi.cpp).  For old no-serial .cam_index files the
    # raw sdk_index (always 0 for single-camera) is returned — in that case
    # use session_config.json's camera_hw directly rather than trusting the
    # slot lookup, which would map sdk_index=0 → cam_0 (x5) incorrectly.
    _ci_path = os.path.join(scan_dir, '.cam_index')
    _ci_has_serial = False
    try:
        _ci_parts = open(_ci_path).read().strip().split()
        _ci_has_serial = len(_ci_parts) >= 2
    except OSError:
        pass
    _cam_idx = cam_index_for_scan(scan_dir)

    # hw: use serial-resolved slot when available; fall back to multi_camera.yaml
    # by cam_index alone (index is correct even without serial), then session hw.
    # Special case: if the scan contains an oak1_* image, it is always an OAK-1
    # scan regardless of .cam_index serial — override hw directly.
    _is_oak1_scan = any(f.startswith('oak1_') for f in os.listdir(str(safe_scan)))
    _hw = 'oak1' if _is_oak1_scan else _session_hw
    if not _is_oak1_scan:
        try:
            _src = os.path.join(os.path.dirname(__file__), '..')
            _mc_path = os.path.join(_src, 'config', 'multi_camera.yaml')
            if os.path.exists(_mc_path):
                _mc = _yaml.safe_load(open(_mc_path).read()) or {}
                _slot_hw = _mc.get('cameras', {}).get(f'cam_{_cam_idx}', {}).get('camera_hw', '')
                # Only trust the slot index when a serial was present in .cam_index —
                # without a serial, sdk_index=0 always maps to cam_0 regardless of
                # which physical camera was used, so slot lookup would give the wrong hw
                # (e.g. cam_0=x5 for a single-camera oak1 session).
                if _slot_hw and _ci_has_serial:
                    _hw = _slot_hw
        except Exception:
            pass

    # _is_perspective must be based on the SCAN's actual hw, not the session hw.
    # In a multi-camera session (e.g. x5+oak1), oak1 scans need pinhole projection
    # even though session_hw=x5 which has perspective=False.
    _profile = load_camera_profile(_hw)
    _is_perspective = _profile.get('perspective', False)

    config_path = str(calibration_path(_hw, _cam_idx))
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    roll  = config['roll_offset']
    pitch = config['pitch_offset']
    yaw   = config['yaw_offset']
    t_x   = config['x_offset']
    t_y   = config['y_offset']
    t_z   = config['z_offset']

    print(f"Transformation (T_camera_lidar):")
    print(f"  Translation: [{t_x:.4f}, {t_y:.4f}, {t_z:.4f}]")
    print(f"  Rotation (RPY): [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")

    # Build T_camera_lidar: sensor LiDAR frame -> camera frame
    R_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T_camera_lidar = np.eye(4)
    T_camera_lidar[:3, :3] = R_matrix
    T_camera_lidar[:3, 3] = [t_x, t_y, t_z]

    # Transform all points to camera frame first, then filter.
    # Filtering in sensor frame (e.g. points[:,2] > 0.05) is wrong because the
    # LiDAR Z-axis is not aligned with the camera Z-axis after T_camera_lidar.
    # The only correct depth filter is camera-frame Z > 0 (point is in front
    # of the camera), which also implicitly rejects behind-camera points that
    # would otherwise wrap around the ERP and get wrong colors.
    distances = np.linalg.norm(points, axis=1)
    range_mask = (distances > 0.3) & (distances < 30.0)
    points = points[range_mask]

    print(f"Processing {len(points)} points...")

    points_h = np.hstack([points, np.ones((len(points), 1))])
    points_camera = (T_camera_lidar @ points_h.T).T[:, :3]

    # Equirectangular projection (matches direct_visual_lidar_calibration)
    cam_depths = np.linalg.norm(points_camera, axis=1)
    depth_valid = cam_depths > 0.1
    points        = points[depth_valid]
    points_camera = points_camera[depth_valid]
    cam_depths    = cam_depths[depth_valid]

    # Projection: pinhole for perspective cameras (oak1), ERP for fisheye
    if _is_perspective:
        # Pinhole projection using calibration intrinsics
        # Load intrinsics from camera_info.yaml (scaled to actual image size)
        _ci_path = os.path.join(scan_dir, 'camera_info.yaml')
        if not os.path.exists(_ci_path):
            _ci_path = os.path.join(_session_dir, 'camera_info.yaml')
        _ci = _yaml.safe_load(open(_ci_path).read())
        _sx = img_width  / _ci['width']
        _sy = img_height / _ci['height']
        _fx = _ci['fx'] * _sx
        _fy = _ci['fy'] * _sy
        _cx = _ci['cx'] * _sx
        _cy = _ci['cy'] * _sy
        # Simple pinhole projection (zero distortion — using undistorted image)
        _front = points_camera[:, 2] > 0.1
        u = np.full(len(points_camera), -1.0)
        v = np.full(len(points_camera), -1.0)
        u[_front] = _fx * points_camera[_front, 0] / points_camera[_front, 2] + _cx
        v[_front] = _fy * points_camera[_front, 1] / points_camera[_front, 2] + _cy
    else:
        # Equirectangular projection (fisheye cameras)
        bearing = points_camera / cam_depths[:, np.newaxis]
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

    # Bilinear interpolation
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
    valid_colors = np.clip(colors_bgr, 0, 255).astype(np.uint8)[:, [2, 1, 0]]  # BGR to RGB

    out_points = valid_points
    colors = valid_colors

    def write_ply(path, pts, cols):
        try:
            safe_path = _safe_data(path)
        except ValueError as e:
            print(f"Error: write_ply path rejected: {e}")
            return
        n = len(pts)
        header = (
            'ply\nformat binary_little_endian 1.0\n'
            f'element vertex {n}\n'
            'property float x\nproperty float y\nproperty float z\n'
            'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'
        )
        # Pack as struct-of-arrays: 3×float32 + 3×uint8 per point.
        # numpy structured array gives a single tobytes() call instead of
        # N string-format + N write() calls (300k points: ~50× faster).
        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                       ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        rec = np.empty(n, dtype=dt)
        rec['x'] = pts[:, 0]; rec['y'] = pts[:, 1]; rec['z'] = pts[:, 2]
        rec['r'] = cols[:, 0]; rec['g'] = cols[:, 1]; rec['b'] = cols[:, 2]
        with open(safe_path, 'wb') as f:
            f.write(header.encode())
            f.write(rec.tobytes())

    sensor_output = str(_safe_data(safe_scan / "sensor_colored_exact.ply"))
    write_ply(sensor_output, out_points, colors)
    print(f"\u2713 Saved sensor-frame colored points to {sensor_output}")
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
