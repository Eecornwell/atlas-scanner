#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Extracts T_lidar_camera from calib.json, undoes the R_align pre-rotation
# applied during lidar intensity image generation, and writes fusion_calibration.yaml.
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# R_align applied in generate_intensity_images.py before projecting lidar points,
# followed by a 180-degree image rotation. The effective transform seen by the tool is
# R_align_eff = diag(-1,-1,1) @ R_align_proj = [[0,1,0],[0,0,1],[1,0,0]]
# The tool finds R_found = R_align_eff @ T_cam_lidar_rot, so:
# T_cam_lidar_rot = R_align_eff.T @ R_found
R_ALIGN = np.array([[0,1,0],[0,0,1],[1,0,0]], dtype=float)

# Load calibration results
with open('/home/orion/atlas_ws/output/calib.json', 'r') as f:
    calib = json.load(f)

# Extract the final calibration (T_lidar_camera if available, otherwise init_T_lidar_camera)
if calib['results']['T_lidar_camera'][0] != 0.0:
    quat_trans = calib['results']['T_lidar_camera']
    print("Using final calibration result (T_lidar_camera)")
else:
    quat_trans = calib['results']['init_T_lidar_camera']
    print("Using initial guess (init_T_lidar_camera)")

tx, ty, tz = quat_trans[0], quat_trans[1], quat_trans[2]
qx, qy, qz, qw = quat_trans[3], quat_trans[4], quat_trans[5], quat_trans[6]

# The tool outputs T_lidar_camera where the camera frame includes R_align.
# Undo R_align to recover the true T_camera_lidar rotation:
# T_lidar_camera_found = R_align @ T_cam_lidar, so T_cam_lidar = R_align.T @ T_lidar_camera_found
# Then invert to get T_camera_lidar (what fusion_calibration.yaml expects).
R_lc_found = R.from_quat([qx, qy, qz, qw]).as_matrix()
R_lc_true = R_ALIGN.T @ R_lc_found
# Invert to get T_camera_lidar
R_cl = R_lc_true.T
t_cl = -R_lc_true.T @ np.array([tx, ty, tz])

rotation = R.from_matrix(R_cl)
euler_rad = rotation.as_euler('xyz', degrees=False)
euler_deg = rotation.as_euler('xyz', degrees=True)

print("=== DIRECT VISUAL LIDAR CALIBRATION RESULTS ===")
print(f"Translation (meters): x={t_cl[0]:.4f}, y={t_cl[1]:.4f}, z={t_cl[2]:.4f}")
print(f"Rotation (degrees): roll={euler_deg[0]:.2f}, pitch={euler_deg[1]:.2f}, yaw={euler_deg[2]:.2f}")
print(f"Rotation (radians): roll={euler_rad[0]:.4f}, pitch={euler_rad[1]:.4f}, yaw={euler_rad[2]:.4f}")
print()

# Update the fusion calibration with new values
new_config = f"""roll_offset: {euler_rad[0]}
pitch_offset: {euler_rad[1]}
yaw_offset: {euler_rad[2]}
manual_roll_adjustment: 0.0
manual_pitch_adjustment: 0.0
manual_yaw_adjustment: 0.0
azimuth_offset: 0.0
elevation_offset: 0.0
x_offset: {t_cl[0]}
y_offset: {t_cl[1]}
z_offset: {t_cl[2]}
flip_x: false
flip_y: false
image_width: 1920
image_height: 960
use_fisheye: false
skip_rate: 5
"""

# Write updated configuration
with open('/home/orion/atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml', 'w') as f:
    f.write(new_config)

print("✓ Updated fusion_calibration.yaml with new calibration values")
print(f"✓ Saved to: /home/orion/atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml")