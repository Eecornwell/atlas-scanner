#!/usr/bin/env python3
"""
Sensor fusion that EXACTLY matches direct_visual_lidar_calibration projection.

Key: The calibration tool uses:
  lon = atan2(bearing[0], bearing[2])  # X forward, Z right
  lat = -asin(bearing[1])              # Y up

Our previous scripts used:
  azimuth = atan2(y_cam, x_cam)  # WRONG convention!
"""

import numpy as np
import cv2
import sys
import os
import yaml
from scipy.spatial.transform import Rotation as R

def load_points(ply_file):
    points = []
    with open(ply_file, 'r') as f:
        lines = f.readlines()
    
    header_end = next(i+1 for i, line in enumerate(lines) if line.strip() == 'end_header')
    
    for line in lines[header_end:]:
        parts = line.strip().split()
        if len(parts) >= 3:
            try:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            except ValueError:
                continue
    
    return np.array(points)

def exact_match_calibration_tool(scan_dir):
    # Find files
    sensor_ply = next((os.path.join(scan_dir, f) for f in os.listdir(scan_dir) 
                      if 'sensor_lidar' in f and f.endswith('.ply')), None)
    
    # Find image
    mask_file = None
    image_file = None
    
    for f in os.listdir(scan_dir):
        if f.endswith('_masked.png'):
            mask_file = os.path.join(scan_dir, f)
        elif ('equirect' in f or 'equirectangular' in f) and f.endswith('.jpg'):
            image_file = os.path.join(scan_dir, f)
    
    if mask_file and os.path.exists(mask_file):
        image = cv2.imread(mask_file, cv2.IMREAD_UNCHANGED)
        if image.shape[2] == 4:
            alpha_mask = image[:, :, 3] > 0
            image = image[:, :, :3]
        else:
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
    img_height, img_width = image.shape[:2]
    
    # Load calibration
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'fusion_calibration.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Get transformation (this is T_camera_lidar from inverted calib.json)
    roll = config['roll_offset']
    pitch = config['pitch_offset']
    yaw = config['yaw_offset']
    
    t_x = config['x_offset']
    t_y = config['y_offset']
    t_z = config['z_offset']
    
    skip_rate = config.get('skip_rate', 1)
    
    print(f"Transformation (T_camera_lidar):")
    print(f"  Translation: [{t_x:.4f}, {t_y:.4f}, {t_z:.4f}]")
    print(f"  Rotation (RPY): [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")
    print(f"  Using exact calibration tool projection (no offsets/flips)")
    
    # Apply skip rate
    if skip_rate > 1:
        points = points[::skip_rate]
    
    # Filter points
    distances = np.linalg.norm(points, axis=1)
    valid_mask = (distances > 0.8) & (distances < 8.0) & (points[:, 2] > 0.05)
    points = points[valid_mask]
    
    print(f"Processing {len(points)} points...")
    
    # Build T_camera_lidar matrix
    R_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T_camera_lidar = np.eye(4)
    T_camera_lidar[:3, :3] = R_matrix
    T_camera_lidar[:3, 3] = [t_x, t_y, t_z]
    
    # Transform points to camera frame (EXACTLY like calibration tool)
    points_homogeneous = np.hstack([points, np.ones((len(points), 1))])
    points_camera = (T_camera_lidar @ points_homogeneous.T).T[:, :3]
    
    # EXACT projection from calibration tool (equirectangular.hpp)
    # bearing = point_3d.normalized()
    bearing = points_camera / np.linalg.norm(points_camera, axis=1, keepdims=True)
    
    # lat = -asin(bearing[1])
    # lon = atan2(bearing[0], bearing[2])
    lat = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
    lon = np.arctan2(bearing[:, 0], bearing[:, 2])
    
    # x = width * (0.5 + lon / (2*PI))
    # y = height * (0.5 - lat / PI)
    u = img_width * (0.5 + lon / (2 * np.pi))
    v = img_height * (0.5 - lat / np.pi)
    
    print(f"Projection range: u=[{u.min():.1f}, {u.max():.1f}], v=[{v.min():.1f}, {v.max():.1f}]")
    
    # Filter valid pixels
    valid_pixels = (u >= 0) & (u < img_width) & (v >= 0) & (v < img_height)
    
    # Remove transparent pixels
    if alpha_mask is not None:
        u_int = np.clip(np.round(u).astype(int), 0, img_width - 1)
        v_int = np.clip(np.round(v).astype(int), 0, img_height - 1)
        non_alpha = alpha_mask[v_int, u_int]
        valid_pixels = valid_pixels & non_alpha
    
    valid_points = points[valid_pixels]
    valid_u = u[valid_pixels]
    valid_v = v[valid_pixels]
    
    print(f"Valid projections: {len(valid_points)} points")
    
    # Sample colors
    u_int = np.clip(np.round(valid_u).astype(int), 0, img_width - 1)
    v_int = np.clip(np.round(valid_v).astype(int), 0, img_height - 1)
    colors = image[v_int, u_int][:, [2, 1, 0]]  # BGR to RGB
    
    # Save sensor-frame colored points
    sensor_output = os.path.join(scan_dir, "sensor_colored_exact.ply")
    with open(sensor_output, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {len(valid_points)}\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property uchar red\n')
        f.write('property uchar green\n')
        f.write('property uchar blue\n')
        f.write('end_header\n')
        
        for i in range(len(valid_points)):
            x, y, z = valid_points[i]
            r, g, b = colors[i]
            f.write(f'{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n')
    
    print(f"✓ Saved sensor-frame colored points to {sensor_output}")
    
    # Also save as world_colored_exact.ply for backward compatibility
    # (Note: despite the name, these are still sensor coordinates!)
    output_file = os.path.join(scan_dir, "world_colored_exact.ply")
    with open(output_file, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {len(valid_points)}\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property uchar red\n')
        f.write('property uchar green\n')
        f.write('property uchar blue\n')
        f.write('end_header\n')
        
        for i in range(len(valid_points)):
            x, y, z = valid_points[i]
            r, g, b = colors[i]
            f.write(f'{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n')
    
    print(f"✓ Also saved as {output_file} (for backward compatibility)")
    print("This uses EXACT projection from calibration tool (no azimuth_offset, no flip)")
    return True

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 exact_match_fusion.py <scan_directory>")
        sys.exit(1)
    
    exact_match_calibration_tool(sys.argv[1])
