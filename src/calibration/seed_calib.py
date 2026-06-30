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

# Detect which camera hw was used for the output dataset via session_config.json files
import glob as _glob
_hw = 'onex2'
_source_jsons = sorted(_glob.glob(str(Path.home() / 'atlas_ws/output/*_source.json')))
if _source_jsons:
    try:
        import json as _json
        _src = _json.loads(Path(_source_jsons[0]).read_text())
        _scan_cfg = Path(_src.get('scan_dir', '')) / '..' / '..' / 'session_config.json'
        if _scan_cfg.exists():
            _hw = _json.loads(_scan_cfg.read_text()).get('camera_hw', 'onex2')
    except Exception:
        pass

# Prefer the per-hw calibration file; fall back to shared
_hw_yaml = Path.home() / f'atlas_ws/src/atlas-scanner/src/config/calibrations/{_hw}/fusion_calibration.yaml'
if _hw_yaml.exists():
    YAML_PATH = _hw_yaml
    print(f'Using {_hw} calibration: {_hw_yaml}')

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
if all(abs(v) < 1e-3 for v in vec[:3]) and abs(vec[3] - 1.0) < 0.01:
    print('  ⚠ WARNING: seeded value looks like identity — calibration yaml may be corrupt')
