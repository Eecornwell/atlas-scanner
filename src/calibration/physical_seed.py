#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Computes a calibration seed from physical measurements of camera
# position relative to the LiDAR. Uses the X5 calibration as the reference
# orientation (camera mounted vertically with front lens down, back lens up)
# and adjusts translation and yaw based on user-provided offsets.
#
# Usage:
#   python3 physical_seed.py --camera-hw x3 --forward 3.25 --left 2.25 --up 0 --yaw 4
#
# All distances are in inches. Yaw is in degrees (positive = CCW from above).
# The reference frame is the LiDAR: +X=forward, +Y=left, +Z=up.

import argparse
import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_SRC = Path(__file__).resolve().parent.parent
_CALIB_DIR = _SRC / 'config' / 'calibrations'
_MODELS_DIR = _SRC / 'config' / 'camera_models'

INCHES_TO_METERS = 0.0254


def compute_physical_seed(camera_hw, forward_in, left_in, up_in, yaw_deg):
    """Compute T_camera_lidar from physical measurements.

    Uses the X5 calibration as the base orientation (roll/pitch) and adjusts
    yaw and translation based on the camera's physical position.
    """
    # Load X5 calibration as reference orientation
    x5_path = _CALIB_DIR / 'x5' / 'fusion_calibration.yaml'
    if not x5_path.exists():
        raise FileNotFoundError(f"X5 reference calibration not found: {x5_path}")

    with open(x5_path) as f:
        x5 = yaml.safe_load(f)

    x5_roll = x5['roll_offset']
    x5_pitch = x5['pitch_offset']
    x5_yaw = x5['yaw_offset']

    # Apply yaw offset (positive = CCW from above = positive in ROS Z-up frame)
    new_yaw = x5_yaw + np.radians(yaw_deg)

    # Build rotation matrix
    R_cam_lidar = R.from_euler('xyz', [x5_roll, x5_pitch, new_yaw]).as_matrix()

    # Camera position in LiDAR frame (inches to meters)
    cam_pos_lidar = np.array([
        forward_in * INCHES_TO_METERS,
        left_in * INCHES_TO_METERS,
        up_in * INCHES_TO_METERS,
    ])

    # T_camera_lidar translation = R_cam_lidar @ (-cam_pos_lidar)
    # (position of LiDAR origin expressed in camera frame)
    t_cam = R_cam_lidar @ (-cam_pos_lidar)

    # Get image dimensions from camera model
    hw_yaml = _MODELS_DIR / f'{camera_hw}.yaml'
    img_w, img_h = 5760, 2880
    if hw_yaml.exists():
        cfg = yaml.safe_load(hw_yaml.read_text()) or {}
        img_w = cfg.get('erp_width', img_w)
        img_h = cfg.get('erp_height', img_h)

    return {
        'roll_offset': x5_roll,
        'pitch_offset': x5_pitch,
        'yaw_offset': float(new_yaw),
        'manual_roll_adjustment': 0.0,
        'manual_pitch_adjustment': 0.0,
        'manual_yaw_adjustment': 0.0,
        'azimuth_offset': 0.0,
        'elevation_offset': 0.0,
        'x_offset': float(t_cam[0]),
        'y_offset': float(t_cam[1]),
        'z_offset': float(t_cam[2]),
        'flip_x': False,
        'flip_y': False,
        'image_width': img_w,
        'image_height': img_h,
        'use_fisheye': False,
        'skip_rate': 5,
    }


def main():
    parser = argparse.ArgumentParser(
        description='Compute calibration seed from physical camera measurements')
    parser.add_argument('--camera-hw', required=True,
                        help='Camera hardware model (x5, x3, onex2)')
    parser.add_argument('--forward', type=float, required=True,
                        help='Distance forward from LiDAR center (inches, negative=behind)')
    parser.add_argument('--left', type=float, required=True,
                        help='Distance left from LiDAR center (inches, negative=right)')
    parser.add_argument('--up', type=float, required=True,
                        help='Distance up from LiDAR center (inches, negative=below)')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='Yaw rotation in degrees (positive=CCW from above)')
    args = parser.parse_args()

    calib = compute_physical_seed(args.camera_hw, args.forward, args.left, args.up, args.yaw)

    # Write to per-hw calibration file
    out_path = _CALIB_DIR / args.camera_hw / 'fusion_calibration.yaml'
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))

    # Also update the active calibration
    active_path = _SRC / 'config' / 'fusion_calibration.yaml'
    active_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))

    print(f"Physical seed for {args.camera_hw}:")
    print(f"  Position (LiDAR frame): fwd={args.forward}\" left={args.left}\" up={args.up}\"")
    print(f"  Yaw offset: {args.yaw}° from X5 reference")
    print(f"  Translation (cam frame): [{calib['x_offset']:.4f}, {calib['y_offset']:.4f}, {calib['z_offset']:.4f}]")
    print(f"  Yaw: {np.degrees(calib['yaw_offset']):.2f}°")
    print(f"  Image: {calib['image_width']}x{calib['image_height']}")
    print(f"\n✓ Saved to: {out_path}")
    print(f"✓ Active calibration updated: {active_path}")


if __name__ == '__main__':
    main()
