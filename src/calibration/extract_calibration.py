#!/usr/bin/env python3
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

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

# Convert quaternion to rotation matrix then to Euler angles
rotation = R.from_quat([qx, qy, qz, qw])
euler_rad = rotation.as_euler('xyz', degrees=False)
euler_deg = rotation.as_euler('xyz', degrees=True)

print("=== DIRECT VISUAL LIDAR CALIBRATION RESULTS ===")
print(f"Translation (meters): x={tx:.4f}, y={ty:.4f}, z={tz:.4f}")
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
x_offset: {tx}
y_offset: {ty}
z_offset: {tz}
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