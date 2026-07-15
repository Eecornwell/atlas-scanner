#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Composes the initial_guess_auto result with the seed calibration.
#
# When the LiDAR intensity image is generated with the seed calibration pre-applied,
# initial_guess_auto finds a RESIDUAL rotation (identity if seed is perfect).
# The true T_lidar_camera = T_residual_auto * T_seed.
#
# This script reads both from calib.json, composes them, and writes the result
# back as init_T_lidar_camera for the calibrate step to use.

import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R


def quat_to_matrix(q_xyzw):
    """Convert [x,y,z,w] quaternion to 4x4 homogeneous matrix."""
    return R.from_quat(q_xyzw).as_matrix()


def vec7_to_T(vec):
    """Convert [tx,ty,tz,qx,qy,qz,qw] to 4x4 matrix."""
    T = np.eye(4)
    T[:3, 3] = vec[:3]
    T[:3, :3] = R.from_quat(vec[3:7]).as_matrix()
    return T


def T_to_vec7(T):
    """Convert 4x4 matrix to [tx,ty,tz,qx,qy,qz,qw]."""
    q = R.from_matrix(T[:3, :3]).as_quat()  # xyzw
    return list(T[:3, 3]) + list(q)


def main():
    calib_path = Path.home() / 'atlas_ws' / 'output' / 'calib.json'
    if not calib_path.exists():
        print("No calib.json found")
        return

    with open(calib_path) as f:
        d = json.load(f)

    results = d.get('results', {})
    seed_vec = results.get('init_T_lidar_camera')
    auto_vec = results.get('init_T_lidar_camera_auto')

    if not seed_vec or not auto_vec:
        print("Missing seed or auto initial guess in calib.json")
        return

    # T_lidar_camera_seed: the pre-applied rotation used to generate intensity image
    T_seed = vec7_to_T(seed_vec)

    # T_lidar_camera_auto: the residual found by initial_guess_auto
    # This is what the tool thinks is the full T_lidar_camera, but since the
    # intensity image already had T_seed applied, it's actually just the residual.
    T_auto = vec7_to_T(auto_vec)

    # Compose: T_true = T_auto * T_seed
    # Because: point_camera = T_cam_lidar_true * point_lidar
    #        = T_cam_lidar_auto * T_cam_lidar_seed * point_lidar
    # And T_lidar_camera = inv(T_camera_lidar)
    # So: T_lidar_camera_true = inv(inv(T_seed) * inv(T_auto))
    #                         = T_auto * T_seed
    T_composed = T_auto @ T_seed
    composed_vec = T_to_vec7(T_composed)

    # Write composed result as the init_T_lidar_camera for calibrate to use
    results['init_T_lidar_camera'] = composed_vec
    results['T_lidar_camera'] = composed_vec
    d['results'] = results

    with open(calib_path, 'w') as f:
        json.dump(d, f, indent=2)

    print(f"Seed T_lidar_camera:     {[round(v,4) for v in seed_vec]}")
    print(f"Auto residual:           {[round(v,4) for v in auto_vec]}")
    print(f"Composed T_lidar_camera: {[round(v,4) for v in composed_vec]}")
    print(f"\n✓ Composed initial guess written to calib.json")


if __name__ == '__main__':
    main()
