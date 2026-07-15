#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Extracts T_lidar_camera from calib.json, undoes the R_align pre-rotation
# applied during lidar intensity image generation, and writes fusion_calibration.yaml
# to both the shared config and the per-model calibrations/<camera_hw>/ directory.
#
# Usage:
#   python3 extract_calibration.py [--camera-hw onex2|x5]

import argparse
import json
import sys
import yaml
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

_SRC = Path(__file__).resolve().parent.parent  # atlas-scanner/src/
sys.path.insert(0, str(_SRC))
from camera_hw import load_camera_profile

# R_align applied in generate_intensity_images.py before projecting lidar points,
# followed by a 180-degree image rotation. The effective transform seen by the tool is
# R_align_eff = diag(-1,-1,1) @ R_align_proj = [[0,1,0],[0,0,1],[1,0,0]]
# The tool finds R_found = R_align_eff @ T_cam_lidar_rot, so:
# T_cam_lidar_rot = R_align_eff.T @ R_found
R_ALIGN = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)

CALIB_JSON = Path.home() / 'atlas_ws/output/calib.json'


def extract(camera_hw='onex2'):
    if not CALIB_JSON.exists():
        print(f'✗ calib.json not found at {CALIB_JSON}')
        sys.exit(1)

    with open(CALIB_JSON) as f:
        calib = json.load(f)

    results = calib.get('results', {})
    if results.get('T_lidar_camera', [0])[0] != 0.0:
        quat_trans = results['T_lidar_camera']
        print('Using final calibration result (T_lidar_camera)')
    else:
        quat_trans = results['init_T_lidar_camera']
        print('Using initial guess (init_T_lidar_camera)')

    tx, ty, tz       = quat_trans[0], quat_trans[1], quat_trans[2]
    qx, qy, qz, qw  = quat_trans[3], quat_trans[4], quat_trans[5], quat_trans[6]

    R_lc_found = R.from_quat([qx, qy, qz, qw]).as_matrix()
    R_lc_true  = R_ALIGN.T @ R_lc_found
    R_cl       = R_lc_true.T
    t_cl       = -R_lc_true.T @ np.array([tx, ty, tz])

    euler_rad = R.from_matrix(R_cl).as_euler('xyz', degrees=False)
    euler_deg = R.from_matrix(R_cl).as_euler('xyz', degrees=True)

    print('=== DIRECT VISUAL LIDAR CALIBRATION RESULTS ===')
    print(f'Translation (m): x={t_cl[0]:.4f}, y={t_cl[1]:.4f}, z={t_cl[2]:.4f}')
    print(f'Rotation (deg):  roll={euler_deg[0]:.2f}  pitch={euler_deg[1]:.2f}  yaw={euler_deg[2]:.2f}')

    # Load ERP resolution from camera model profile
    profile    = load_camera_profile(camera_hw)
    erp_width  = profile['erp_width']
    erp_height = profile['erp_height']
    print(f'Camera HW: {profile["display_name"]}  ERP {erp_width}x{erp_height}')

    config = {
        'roll_offset':             float(euler_rad[0]),
        'pitch_offset':            float(euler_rad[1]),
        'yaw_offset':              float(euler_rad[2]),
        'manual_roll_adjustment':  0.0,
        'manual_pitch_adjustment': 0.0,
        'manual_yaw_adjustment':   0.0,
        'azimuth_offset':          0.0,
        'elevation_offset':        0.0,
        'x_offset':                float(t_cl[0]),
        'y_offset':                float(t_cl[1]),
        'z_offset':                float(t_cl[2]),
        'flip_x':                  False,
        'flip_y':                  False,
        'image_width':             erp_width,
        'image_height':            erp_height,
        'use_fisheye':             False,
        'skip_rate':               5,
    }
    text = yaml.dump(config, default_flow_style=False, sort_keys=False)

    # Write to per-model calibration file
    hw_path = _SRC / 'config' / 'calibrations' / camera_hw / 'fusion_calibration.yaml'
    hw_path.parent.mkdir(parents=True, exist_ok=True)
    hw_path.write_text(text)
    print(f'\n\u2713 Saved to: {hw_path}')

    # Also update the shared active calibration
    shared_path = _SRC / 'config' / 'fusion_calibration.yaml'
    shared_path.write_text(text)
    print(f'\u2713 Active calibration updated: {shared_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-hw', default='onex2', choices=['onex2', 'x3', 'x5'],
                        help='Camera hardware model (default: onex2)')
    args = parser.parse_args()
    extract(args.camera_hw)
