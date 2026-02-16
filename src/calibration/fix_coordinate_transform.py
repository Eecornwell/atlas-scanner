#!/usr/bin/env python3
"""
Fixed calibration transformation that properly handles coordinate systems.

The calibration tool provides T_lidar_camera (camera → lidar transform).
For projection, we need T_camera_lidar (lidar → camera transform).

Key insight: T_camera_lidar = inverse(T_lidar_camera)
"""
import sys
import json
import yaml
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

def fix_calibration_transform(root_path, use_existing=False):
    config_path = os.path.join(root_path, 'config', 'fusion_calibration.yaml')
    
    if use_existing:
        if os.path.exists(config_path):
            print("Using existing fusion_calibration.yaml")
            print_calibration_comparison(root_path)
            return
        else:
            print("No existing calibration found, generating new one...")
            use_existing = False
    
    # Load calib.json
    calib_paths = [
        '/home/orion/atlas_ws/output/calib.json',
        os.path.join(root_path, 'output', 'calib.json')
    ]
    
    calib_path = None
    for path in calib_paths:
        if os.path.exists(path):
            calib_path = path
            break
    
    if calib_path is None:
        raise FileNotFoundError(f"calib.json not found in: {calib_paths}")
    
    with open(calib_path, 'r') as f:
        calib_data = json.load(f)
    
    # Extract T_lidar_camera
    if 'results' in calib_data:
        T_lidar_camera = calib_data['results']['T_lidar_camera']
        Tx, Ty, Tz, Qx, Qy, Qz, Qw = T_lidar_camera
    elif 'T_lidar_camera' in calib_data:
        T_matrix = np.array(calib_data['T_lidar_camera'])
        Tx, Ty, Tz = T_matrix[:3, 3]
        rot = R.from_matrix(T_matrix[:3, :3])
        Qx, Qy, Qz, Qw = rot.as_quat()
    else:
        raise ValueError("Invalid calib.json format")
    
    print("\n=== CALIBRATION TOOL OUTPUT (T_lidar_camera) ===")
    print(f"Translation: [{Tx:.4f}, {Ty:.4f}, {Tz:.4f}]")
    print(f"Quaternion: [{Qx:.4f}, {Qy:.4f}, {Qz:.4f}, {Qw:.4f}]")
    
    # Build T_lidar_camera matrix
    T_lidar_camera_mat = np.eye(4)
    T_lidar_camera_mat[:3, :3] = R.from_quat([Qx, Qy, Qz, Qw]).as_matrix()
    T_lidar_camera_mat[:3, 3] = [Tx, Ty, Tz]
    
    print("\nT_lidar_camera matrix:")
    print(T_lidar_camera_mat)
    
    # CORRECT APPROACH: Invert to get T_camera_lidar
    T_camera_lidar_mat = np.linalg.inv(T_lidar_camera_mat)
    
    print("\n=== INVERTED FOR PROJECTION (T_camera_lidar) ===")
    print(T_camera_lidar_mat)
    
    # Extract rotation and translation from T_camera_lidar
    R_camera_lidar = T_camera_lidar_mat[:3, :3]
    t_camera_lidar = T_camera_lidar_mat[:3, 3]
    
    # Convert to euler angles (XYZ convention)
    rot_camera_lidar = R.from_matrix(R_camera_lidar)
    euler_xyz = rot_camera_lidar.as_euler('xyz', degrees=False)
    
    print(f"\nRotation (euler XYZ): [{euler_xyz[0]:.6f}, {euler_xyz[1]:.6f}, {euler_xyz[2]:.6f}]")
    print(f"Translation: [{t_camera_lidar[0]:.6f}, {t_camera_lidar[1]:.6f}, {t_camera_lidar[2]:.6f}]")
    
    # Load existing config to preserve manual adjustments
    existing_config = {}
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                existing_config = yaml.safe_load(f) or {}
        except:
            pass
    
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
        'image_width': existing_config.get('image_width', 1920),
        'image_height': existing_config.get('image_height', 960),
        'use_fisheye': existing_config.get('use_fisheye', False),
        'skip_rate': existing_config.get('skip_rate', 5)
    }
    
    # Save
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"\n✓ Saved corrected calibration to: {config_path}")

def print_calibration_comparison(root_path):
    """Compare old vs new transformation approach"""
    config_path = os.path.join(root_path, 'config', 'fusion_calibration.yaml')
    
    if not os.path.exists(config_path):
        return
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    print("\n=== CURRENT CALIBRATION VALUES ===")
    print(f"Roll:  {config.get('roll_offset', 0.0):.6f} rad")
    print(f"Pitch: {config.get('pitch_offset', 0.0):.6f} rad")
    print(f"Yaw:   {config.get('yaw_offset', 0.0):.6f} rad")
    print(f"Translation: [{config.get('x_offset', 0.0):.4f}, {config.get('y_offset', 0.0):.4f}, {config.get('z_offset', 0.0):.4f}]")
    print(f"Azimuth offset: {config.get('azimuth_offset', 0.0):.6f} rad ({np.degrees(config.get('azimuth_offset', 0.0)):.1f}°)")
    print(f"Flip X: {config.get('flip_x', False)}, Flip Y: {config.get('flip_y', False)}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python fix_coordinate_transform.py <root_path> [--use-existing]")
        print("Example: python fix_coordinate_transform.py /home/orion/atlas_ws/src/atlas-scanner/src")
        sys.exit(1)
    
    root_path = sys.argv[1]
    use_existing = len(sys.argv) > 2 and sys.argv[2] == "--use-existing"
    
    fix_calibration_transform(root_path, use_existing)
