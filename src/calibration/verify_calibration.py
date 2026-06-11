#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Visualises the LiDAR-camera extrinsic calibration by projecting sensor-frame
# LiDAR points onto the ERP image and saving the overlay. Good calibration shows points
# landing on the correct surfaces; smearing or offset indicates a bad extrinsic.

import sys
import os
import numpy as np
import cv2
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R

CALIB_PATH = Path(__file__).parent.parent / 'config' / 'fusion_calibration.yaml'

_ALLOWED_DATA   = Path(os.path.expanduser('~/atlas_ws/data')).resolve()
_ALLOWED_OUTPUT = Path(os.path.expanduser('~/atlas_ws/output')).resolve()


def _safe_scan(p) -> Path:
    """Accept paths under the data or output tree."""
    resolved = Path(p).resolve()
    for root in (_ALLOWED_DATA, _ALLOWED_OUTPUT):
        if root in [resolved, *resolved.parents]:
            return resolved
    raise ValueError(f"Path '{resolved}' is outside allowed roots")


def load_ply_points(ply_file):
    safe = _safe_scan(ply_file)
    with open(safe, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            header += line
            if line.strip() == b'end_header':
                break
        header_str = header.decode('ascii')
        binary = 'binary_little_endian' in header_str
        n = int(next(l.split()[-1] for l in header_str.splitlines() if l.startswith('element vertex')))
        fields = [l.split()[-1] for l in header_str.splitlines() if l.startswith('property float')]
        if binary:
            data = np.frombuffer(f.read(n * len(fields) * 4), dtype=np.float32).reshape(n, len(fields))
            return data[:, :3]
        lines = f.read().decode('ascii').splitlines()
    pts = []
    for line in lines:
        parts = line.strip().split()
        if len(parts) >= 3:
            try:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
            except ValueError:
                continue
    return np.array(pts)


def verify_calibration(scan_dir, output_path=None):
    try:
        scan_dir = _safe_scan(scan_dir)
    except ValueError as e:
        print(f'Error: {e}')
        return False

    # Find sensor PLY
    sensor_ply = next((scan_dir / f for f in os.listdir(scan_dir)
                       if 'sensor_lidar' in f and f.endswith('.ply')), None)
    if sensor_ply is None:
        print(f"No sensor_lidar*.ply found in {scan_dir}")
        return False
    try:
        sensor_ply = _safe_scan(sensor_ply)
    except ValueError as e:
        print(f'Error: {e}')
        return False

    # Find ERP image — prefer unmasked jpg for clearest overlay
    image_path = next((scan_dir / f for f in sorted(os.listdir(scan_dir))
                       if ('equirect' in f) and f.endswith('.jpg')), None)
    if image_path is None:
        print(f"No equirect*.jpg found in {scan_dir}")
        return False
    try:
        image_path = _safe_scan(image_path)
    except ValueError as e:
        print(f'Error: {e}')
        return False

    print(f"PLY:   {sensor_ply.name}")
    print(f"Image: {image_path.name}")

    with open(CALIB_PATH) as f:
        cfg = yaml.safe_load(f)

    roll  = cfg['roll_offset']  + cfg.get('manual_roll_adjustment',  0.0)
    pitch = cfg['pitch_offset'] + cfg.get('manual_pitch_adjustment', 0.0)
    yaw   = cfg['yaw_offset']   + cfg.get('manual_yaw_adjustment',   0.0)
    tx, ty, tz = cfg['x_offset'], cfg['y_offset'], cfg['z_offset']

    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3]  = [tx, ty, tz]

    print(f"T_camera_lidar RPY (deg): [{np.degrees(roll):.3f}, {np.degrees(pitch):.3f}, {np.degrees(yaw):.3f}]")
    print(f"T_camera_lidar t   (m):   [{tx:.4f}, {ty:.4f}, {tz:.4f}]")

    points = load_ply_points(str(sensor_ply))
    dist = np.linalg.norm(points, axis=1)
    points = points[(dist > 0.5) & (dist < 10.0)]

    pts_h = np.hstack([points, np.ones((len(points), 1))])
    pts_cam = (T @ pts_h.T).T[:, :3]

    bearing = pts_cam / np.linalg.norm(pts_cam, axis=1, keepdims=True)
    lat = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
    lon =  np.arctan2(bearing[:, 0], bearing[:, 2])

    img = cv2.imread(str(image_path))
    h, w = img.shape[:2]

    u = (w * (0.5 + lon / (2 * np.pi))).astype(np.float32)
    v = (h * (0.5 - lat / np.pi)).astype(np.float32)

    valid = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v = u[valid], v[valid]
    pts_valid = pts_cam[valid]
    depths = np.linalg.norm(pts_valid, axis=1)
    depths = np.linalg.norm(pts_valid, axis=1)

    # Colour dots by distance using continuous jet colormap for maximum visual contrast
    depths = np.linalg.norm(pts_valid, axis=1)
    d_min, d_max = depths.min(), depths.max()
    norm_d = ((depths - d_min) / (d_max - d_min + 1e-6) * 255).astype(np.uint8)
    colours = cv2.applyColorMap(norm_d.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

    overlay = img.copy()
    for i in range(len(u)):
        cv2.circle(overlay, (int(round(u[i])), int(round(v[i]))), 2,
                   (int(colours[i, 0]), int(colours[i, 1]), int(colours[i, 2])), -1)

    result = cv2.addWeighted(img, 0.4, overlay, 0.6, 0)

    # Legend
    legend_img = np.zeros((20, 200, 3), dtype=np.uint8)
    for x in range(200):
        c = cv2.applyColorMap(np.array([[int(x * 255 / 199)]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
        legend_img[:, x] = c
    cv2.putText(result, f'{d_min:.1f}m', (10, result.shape[0] - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(result, f'{d_max:.1f}m', (result.shape[1] - 60, result.shape[0] - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    result[result.shape[0]-20:, (result.shape[1]-200)//2:(result.shape[1]+200)//2] = legend_img

    if output_path is None:
        output_path = scan_dir / 'calib_verify_overlay.jpg'
    try:
        output_path = _safe_scan(output_path)
    except ValueError as e:
        print(f'Error: {e}')
        return False
    cv2.imwrite(str(output_path), result)
    print(f"✓ Saved overlay ({len(u)} points projected) → {output_path}")
    print(f"  Blue=near, Red=far (continuous jet scale {d_min:.1f}m - {d_max:.1f}m)")
    print(f"  If colours are offset from surfaces, adjust manual_roll/pitch/yaw_adjustment in fusion_calibration.yaml")
    return True


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 verify_calibration.py <scan_dir> [output.jpg]")
        sys.exit(1)
    output = None
    if len(sys.argv) > 2:
        try:
            output = _safe_scan(sys.argv[2])
        except ValueError as e:
            print(f'Error: {e}')
            sys.exit(1)
    verify_calibration(sys.argv[1], output)
