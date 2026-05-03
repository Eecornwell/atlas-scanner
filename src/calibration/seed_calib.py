#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Seeds ~/atlas_ws/output/calib.json with the current
# fusion_calibration.yaml values as the initial guess for calibrate.
# Run this before ./calibrate to start from the known-good pose.

import json
import yaml
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

YAML_PATH  = Path.home() / 'atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml'
CALIB_PATH = Path.home() / 'atlas_ws/output/calib.json'

with open(YAML_PATH) as f:
    cfg = yaml.safe_load(f)

roll  = cfg['roll_offset']  + cfg.get('manual_roll_adjustment',  0.0)
pitch = cfg['pitch_offset'] + cfg.get('manual_pitch_adjustment', 0.0)
yaw   = cfg['yaw_offset']   + cfg.get('manual_yaw_adjustment',   0.0)

T = np.eye(4)
T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
T[:3, 3]  = [cfg['x_offset'], cfg['y_offset'], cfg['z_offset']]

Ti = np.linalg.inv(T)
q  = R.from_matrix(Ti[:3, :3]).as_quat()  # xyzw
vec = [float(v) for v in list(Ti[:3, 3]) + list(q)]

with open(CALIB_PATH) as f:
    d = json.load(f)
d['results']['init_T_lidar_camera'] = vec
d['results']['T_lidar_camera']      = vec
with open(CALIB_PATH, 'w') as f:
    json.dump(d, f, indent=2)

print(f'✓ Seeded {CALIB_PATH}')
print(f'  T_lidar_camera: {[round(v,4) for v in vec]}')
