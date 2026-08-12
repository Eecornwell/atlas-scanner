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

import os as _os, glob as _glob
_src_root = Path(__file__).parent.parent

# Priority 1: per-camera-slot path from multi_camera.yaml (same source as
# generate_intensity_images.py uses), so the seed and intensity projection
# are always consistent.
_cam_idx = _os.environ.get('ATLAS_CALIBRATION_CAM_INDEX', '')
if _cam_idx:
    try:
        import yaml as _y
        _mc_path = _src_root / 'config' / 'multi_camera.yaml'
        if _mc_path.exists():
            _mc = _y.safe_load(_mc_path.read_text()) or {}
            # Try exact slot, then fall back to hw match via source.json
            _slot_cfg = _mc.get('cameras', {}).get(f'cam_{_cam_idx}', {})
            if not _slot_cfg.get('calibration'):
                _source_jsons0 = sorted(_glob.glob(str(Path.home() / 'atlas_ws/output/*_source.json')))
                if _source_jsons0:
                    import json as _json0
                    _sess_hw = _json0.loads(Path(_source_jsons0[0]).read_text()).get('camera_hw', '')
                    if _sess_hw:
                        _slot_cfg = next(
                            (v for v in _mc.get('cameras', {}).values()
                             if v.get('camera_hw') == _sess_hw), {})
            _calib_rel = _slot_cfg.get('calibration', '')
            if _calib_rel:
                _slot_path = _src_root / 'config' / _calib_rel
                if _slot_path.exists():
                    YAML_PATH = _slot_path
                    _hw = _slot_cfg.get('camera_hw', 'onex2')
                    print(f'Using {_hw} calibration: {_slot_path}')
    except Exception:
        pass

# Priority 2: per-hw calibration detected from output dataset
if YAML_PATH == Path.home() / 'atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml':
    _hw = 'onex2'
    _source_jsons = sorted(_glob.glob(str(Path.home() / 'atlas_ws/output/*_source.json')))
    if _source_jsons:
        try:
            import json as _json
            _src = _json.loads(Path(_source_jsons[0]).read_text())
            _scan_cfg = Path(_src.get('scan_dir', '')) / '..' / 'session_config.json'
            if _scan_cfg.exists():
                _hw = _json.loads(_scan_cfg.read_text()).get('camera_hw', 'onex2')
        except Exception:
            pass
    _hw_yaml = _src_root / 'config' / 'calibrations' / _hw / 'fusion_calibration.yaml'
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
