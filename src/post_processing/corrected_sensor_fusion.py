#!/usr/bin/env python3
"""
Corrected sensor fusion that properly applies T_camera_lidar transformation.

This version directly applies the rotation matrix instead of decomposing
into euler angles and recomposing with multiple rotation steps.
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

def corrected_fusion(scan_dir):
    # Find files
    sensor_ply = next((os.path.join(scan_dir, f) for f in os.listdir(scan_dir) 
                      if 'sensor_lidar' in f and f.endswith('.ply')), None)
    
    # Find masked ERP image first, fallback to regular image
    mask_file = None
    image_file = None
    
    for f in os.listdir(scan_dir):
        if f.endswith('_masked.png'):
            mask_file = os.path.join(scan_dir, f)
        elif ('equirect' in f or 'equirectangular' in f) and f.endswith('.jpg'):
            image_file = os.path.join(scan_dir, f)
    
    # Use masked image if available
    if mask_file and os.path.exists(mask_file):
        image = cv2.imread(mask_file, cv2.IMREAD_UNCHANGED)
        print(f"Using masked image: {os.path.basename(mask_file)}")
        if image.shape[2] == 4:
            alpha_mask = image[:, :, 3] > 0
            image = image[:, :, :3]
        else:
            alpha_mask = None
    elif image_file and os.path.exists(image_file):
        image = cv2.imread(image_file)
        alpha_mask = None
        print(f"Using regular image: {os.path.basename(image_file)}")
    else:
        image = None
    
    if sensor_ply is None or image is None:
        print("Missing sensor PLY or image")
        return False
    
    points = load_points(sensor_ply)
    img_height, img_width = image.shape[:2]
    
    # Load calibration
    config_paths = [
        os.path.join(os.path.dirname(__file__), '..', 'config', 'fusion_calibration.yaml')
    ]
    
    config_path = None
    for path in config_paths:
        if os.path.exists(path):
            config_path = path
            print(f"Using calibration from: {config_path}")
            break
    
    if config_path is None:
        raise FileNotFoundError(f"fusion_calibration.yaml not found")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Get transformation parameters
    roll = config['roll_offset'] + config.get('manual_roll_adjustment', 0.0)
    pitch = config['pitch_offset'] + config.get('manual_pitch_adjustment', 0.0)
    yaw = config['yaw_offset'] + config.get('manual_yaw_adjustment', 0.0)
    
    t_x = config['x_offset']
    t_y = config['y_offset']
    t_z = config['z_offset']
    
    azimuth_offset = config.get('azimuth_offset', 0.0)
    elevation_offset = config.get('elevation_offset', 0.0)
    flip_x = config.get('flip_x', False)
    flip_y = config.get('flip_y', False)
    skip_rate = config.get('skip_rate', 1)
    
    print(f"\\nTransformation parameters:")
    print(f"  Rotation (RPY): [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")
    print(f"  Translation: [{t_x:.4f}, {t_y:.4f}, {t_z:.4f}]")
    print(f"  Azimuth offset: {azimuth_offset:.4f} ({np.degrees(azimuth_offset):.1f}°)")
    print(f"  Flip X: {flip_x}, Flip Y: {flip_y}")
    
    # Apply skip rate
    if skip_rate > 1:
        points = points[::skip_rate]
    
    # Filter points
    distances = np.linalg.norm(points, axis=1)
    valid_mask = (distances > 0.8) & (distances < 8.0) & (points[:, 2] > 0.05)
    points = points[valid_mask]
    
    print(f"Processing {len(points)} points...")
    
    # Build rotation matrix from euler angles (XYZ convention)
    R_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    
    # Build full transformation matrix
    T_camera_lidar = np.eye(4)
    T_camera_lidar[:3, :3] = R_matrix
    T_camera_lidar[:3, 3] = [t_x, t_y, t_z]
    
    # Transform points to camera frame
    points_homogeneous = np.hstack([points, np.ones((len(points), 1))])
    points_camera = (T_camera_lidar @ points_homogeneous.T).T[:, :3]
    
    # Extract camera coordinates
    x_cam = points_camera[:, 0]
    y_cam = points_camera[:, 1]
    z_cam = points_camera[:, 2]
    
    # Project to spherical coordinates
    # Azimuth: angle in XY plane
    azimuth = np.arctan2(y_cam, x_cam) + azimuth_offset
    
    # Elevation: angle from XY plane
    r_xy = np.sqrt(x_cam**2 + y_cam**2)
    elevation = np.arctan2(z_cam, r_xy) + elevation_offset
    
    # Map to equirectangular image coordinates
    # Azimuth: -π to π maps to 0 to width
    u = (azimuth + np.pi) / (2 * np.pi) * img_width
    
    # Elevation: -π/2 to π/2 maps to height to 0 (top to bottom)
    v = img_height/2 - (elevation / (np.pi/2)) * (img_height/2)
    
    # Apply flips
    if flip_x:
        u = img_width - 1 - u
    
    if flip_y:
        v = img_height - 1 - v
    
    # Filter valid pixels
    valid_pixels = (u >= 0) & (u < img_width) & (v >= 0) & (v < img_height)
    
    # Remove points that project to transparent pixels
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
    
    # Save colored point cloud
    output_file = os.path.join(scan_dir, "world_colored.ply")
    with open(output_file, 'w') as f:
        f.write('ply\\nformat ascii 1.0\\n')
        f.write(f'element vertex {len(valid_points)}\\n')
        f.write('property float x\\nproperty float y\\nproperty float z\\n')
        f.write('property uchar red\\nproperty uchar green\\nproperty uchar blue\\n')
        f.write('end_header\\n')
        
        for i in range(len(valid_points)):
            x, y, z = valid_points[i]
            r, g, b = colors[i]
            f.write(f'{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\\n')
    
    print(f"✓ Saved {len(valid_points)} colored points to {output_file}")
    return True

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 corrected_sensor_fusion.py <scan_directory>")
        sys.exit(1)
    
    corrected_fusion(sys.argv[1])
