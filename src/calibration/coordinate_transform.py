#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Reads the raw T_lidar_camera result from calib.json, inverts it to T_camera_lidar, and writes the final extrinsic parameters to fusion_calibration.yaml.
"""
Calibration transformation that properly handles coordinate systems.

direct_visual_lidar_calibration outputs T_lidar_camera: maps a point in
camera frame to lidar frame (camera -> lidar).

For projection we need T_camera_lidar: maps a lidar point into camera frame
(lidar -> camera). This is obtained by inverting T_lidar_camera.

Key insight: T_camera_lidar = inverse(T_lidar_camera)
"""
import sys
import json
import yaml
import os
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_ALLOWED_SRC = Path(os.path.expanduser("~/atlas_ws/src")).resolve()


def _safe_src(root_path: str, *parts) -> Path:
    resolved = Path(root_path).joinpath(*parts).resolve()
    if _ALLOWED_SRC not in [resolved, *resolved.parents]:
        raise ValueError(
            f"Path '{resolved}' is outside the allowed source root '{_ALLOWED_SRC}'"
        )
    return resolved


def _camera_hw_erp_dims(root_path, camera_hw):
    """Return (erp_width, erp_height) from the camera model YAML."""
    hw_yaml = Path(root_path) / 'config' / 'camera_models' / f'{camera_hw}.yaml'
    if hw_yaml.exists():
        try:
            cfg = yaml.safe_load(hw_yaml.read_text()) or {}
            return cfg.get('erp_width', 5760), cfg.get('erp_height', 2880)
        except Exception:
            pass
    defaults = {'onex2': (5760, 2880), 'x5': (7680, 3840)}
    return defaults.get(camera_hw, (5760, 2880))


def calibration_transform(root_path, use_existing=False, camera_hw='onex2'):
    # Write to both the shared config and the per-model calibration file
    config_path     = _safe_src(root_path, 'config', 'fusion_calibration.yaml')
    hw_config_path  = _safe_src(root_path, 'config', 'calibrations', camera_hw, 'fusion_calibration.yaml')
    
    if use_existing:
        if config_path.exists():
            print(f"Using existing calibration for camera_hw={camera_hw}")
            print_calibration_comparison(root_path)
            return
        else:
            print("No existing calibration found, generating new one...")
            use_existing = False
    
    # Load calib.json from workspace level
    calib_path = '/home/orion/atlas_ws/output/calib.json'
    
    if not os.path.exists(calib_path):
        if config_path.exists():
            print(f"⚠ calib.json not found — keeping existing fusion_calibration.yaml")
            print_calibration_comparison(root_path)
            return
        # Truly fresh environment: write identity so the session can proceed
        print(f"⚠ calib.json not found at: {calib_path}")
        print("  Writing identity calibration (no extrinsic correction).")
        print("  Run direct_visual_lidar_calibration to generate calib.json.")
        config = {
            'roll_offset': 0.0,
            'pitch_offset': 0.0,
            'yaw_offset': 0.0,
            'manual_roll_adjustment': 0.0,
            'manual_pitch_adjustment': 0.0,
            'manual_yaw_adjustment': 0.0,
            'azimuth_offset': 0.0,
            'elevation_offset': 0.0,
            'x_offset': 0.0,
            'y_offset': 0.0,
            'z_offset': 0.0,
            'flip_x': False,
            'flip_y': False,
            'image_width': 5760,
            'image_height': 2880,
            'use_fisheye': False,
            'skip_rate': 1,
        }
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config_path.write_text(yaml.dump(config, default_flow_style=False, sort_keys=False))
        print(f"  ✓ Wrote identity calibration to: {config_path}")
        return
    
    with open(calib_path, 'r') as f:
        calib_data = json.load(f)
    
    # direct_visual_lidar_calibration outputs T_lidar_camera (camera->lidar).
    # For projection we need T_camera_lidar (lidar->camera) = inv(T_lidar_camera).
    if 'results' in calib_data:
        vec = calib_data['results']['T_lidar_camera']
        Tx, Ty, Tz, Qx, Qy, Qz, Qw = vec
    elif 'T_lidar_camera' in calib_data:
        T_matrix = np.array(calib_data['T_lidar_camera'])
        Tx, Ty, Tz = T_matrix[:3, 3]
        rot = R.from_matrix(T_matrix[:3, :3])
        Qx, Qy, Qz, Qw = rot.as_quat()
    else:
        raise ValueError("Invalid calib.json format")

    print("\n=== CALIBRATION TOOL OUTPUT (T_lidar_camera, camera->lidar) ===")
    print(f"Translation: [{Tx:.4f}, {Ty:.4f}, {Tz:.4f}]")
    print(f"Quaternion: [{Qx:.4f}, {Qy:.4f}, {Qz:.4f}, {Qw:.4f}]")

    # Invert to get T_camera_lidar (lidar->camera) for projection
    T_lidar_camera_mat = np.eye(4)
    T_lidar_camera_mat[:3, :3] = R.from_quat([Qx, Qy, Qz, Qw]).as_matrix()
    T_lidar_camera_mat[:3, 3] = [Tx, Ty, Tz]
    T_camera_lidar_mat = np.linalg.inv(T_lidar_camera_mat)
    
    print("\nT_camera_lidar matrix (lidar->camera, after inversion):")
    print(T_camera_lidar_mat)
    
    R_camera_lidar = T_camera_lidar_mat[:3, :3]
    t_camera_lidar = T_camera_lidar_mat[:3, 3]
    
    # Convert to euler angles (XYZ convention)
    rot_camera_lidar = R.from_matrix(R_camera_lidar)
    euler_xyz = rot_camera_lidar.as_euler('xyz', degrees=False)
    
    print(f"\nRotation (euler XYZ): [{euler_xyz[0]:.6f}, {euler_xyz[1]:.6f}, {euler_xyz[2]:.6f}]")
    print(f"Translation: [{t_camera_lidar[0]:.6f}, {t_camera_lidar[1]:.6f}, {t_camera_lidar[2]:.6f}]")
    
    # Load existing config to preserve manual adjustments
    existing_config = {}
    if config_path.exists():
        try:
            existing_config = yaml.safe_load(config_path.read_text()) or {}
        except Exception:
            pass
    
    # SDK stitch always outputs at the camera model's native ERP resolution.
    # Stream mode uses the ROS equirectangular node config.
    # Since exact_match_fusion.py reads dimensions from the actual image,
    # this value is informational only.
    erp_width, erp_height = _camera_hw_erp_dims(root_path, camera_hw)

    # Create config with DIRECT values (no complex transformations)
    config = {
        'roll_offset': float(euler_xyz[0]),
        'pitch_offset': float(euler_xyz[1]),
        'yaw_offset': float(euler_xyz[2]),
        'manual_roll_adjustment': existing_config.get('manual_roll_adjustment', 0.0),
        'manual_pitch_adjustment': existing_config.get('manual_pitch_adjustment', 0.0),
        'manual_yaw_adjustment': existing_config.get('manual_yaw_adjustment', 0.0),
        'azimuth_offset': existing_config.get('azimuth_offset', 0.0),
        'elevation_offset': existing_config.get('elevation_offset', 0.0),
        'x_offset': float(t_camera_lidar[0]),
        'y_offset': float(t_camera_lidar[1]),
        'z_offset': float(t_camera_lidar[2]),
        'flip_x': existing_config.get('flip_x', False),
        'flip_y': existing_config.get('flip_y', False),
        'image_width': erp_width,
        'image_height': erp_height,
        'use_fisheye': existing_config.get('use_fisheye', False),
        'skip_rate': existing_config.get('skip_rate', 5)
    }
    
    # Save to shared config (active calibration) and per-model calibration file
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(yaml.dump(config, default_flow_style=False, sort_keys=False))
    hw_config_path.parent.mkdir(parents=True, exist_ok=True)
    hw_config_path.write_text(yaml.dump(config, default_flow_style=False, sort_keys=False))

    print(f"\n✓ Saved calibration for {camera_hw} to: {hw_config_path}")
    print(f"✓ Active calibration updated: {config_path}")

def print_calibration_comparison(root_path):
    """Compare old vs new transformation approach"""
    config_path = _safe_src(root_path, 'config', 'fusion_calibration.yaml')

    if not config_path.exists():
        return

    config = yaml.safe_load(config_path.read_text())
    
    print("\n=== CURRENT CALIBRATION VALUES ===")
    print(f"Roll:  {config.get('roll_offset', 0.0):.6f} rad")
    print(f"Pitch: {config.get('pitch_offset', 0.0):.6f} rad")
    print(f"Yaw:   {config.get('yaw_offset', 0.0):.6f} rad")
    print(f"Translation: [{config.get('x_offset', 0.0):.4f}, {config.get('y_offset', 0.0):.4f}, {config.get('z_offset', 0.0):.4f}]")
    print(f"Azimuth offset: {config.get('azimuth_offset', 0.0):.6f} rad ({np.degrees(config.get('azimuth_offset', 0.0)):.1f}°)")
    print(f"Flip X: {config.get('flip_x', False)}, Flip Y: {config.get('flip_y', False)}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python coordinate_transform.py <root_path> [--use-existing] [--camera-hw onex2|x5]")
        print("Example: python coordinate_transform.py /home/orion/atlas_ws/src/atlas-scanner/src --camera-hw x5")
        sys.exit(1)

    root_path    = sys.argv[1]
    use_existing = '--use-existing' in sys.argv
    camera_hw    = 'onex2'
    if '--camera-hw' in sys.argv:
        idx = sys.argv.index('--camera-hw')
        if idx + 1 < len(sys.argv):
            camera_hw = sys.argv[idx + 1]

    try:
        _safe_src(root_path)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    calibration_transform(root_path, use_existing, camera_hw)
