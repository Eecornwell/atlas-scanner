#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Computes a calibration seed from physical measurements of camera
# position relative to the LiDAR center.
#
# ─── Coordinate System ───────────────────────────────────────────────────────
#
# Stand behind the scanner looking at it from the BACK (cable side of MID360
# facing you). The LiDAR center is the origin.
#
#   +X = FORWARD  (away from you, toward the front of the scanner)
#   +Y = LEFT     (to your left as you face the scanner's front)
#   +Z = UP       (toward the ceiling)
#
# The MID360's cable connector points in the -X direction (toward you).
#
# ─── Measurements ────────────────────────────────────────────────────────────
#
#   forward:  How far the camera is in front of the LiDAR center (inches)
#             Positive = toward scanner front, Negative = toward cable/back
#
#   left:     How far the camera is to the left (inches)
#             Positive = left side, Negative = right side
#             (as seen from behind the scanner)
#
#   up:       How far the camera is above the LiDAR center (inches)
#             Positive = above, Negative = below
#
#   yaw:      Camera rotation about the vertical axis (degrees)
#             0° = camera faces same direction as LiDAR front (+X)
#             Positive = camera rotated CCW (looking down from above)
#             e.g. +90° = camera faces left, -90° = camera faces right
#
# ─── Examples ────────────────────────────────────────────────────────────────
#
#   Camera mounted 3" forward, 2" to the left, same height, facing forward:
#     --forward 3 --left 2 --up 0 --yaw 0
#
#   Camera mounted 1" behind, 3" to the right, 0.5" below, facing right:
#     --forward -1 --left -3 --up -0.5 --yaw -90
#
#   Camera mounted directly above LiDAR, 4" up, rotated 45° left:
#     --forward 0 --left 0 --up 4 --yaw 45
#
# ─── Usage ───────────────────────────────────────────────────────────────────
#
#   python3 physical_seed.py --camera-hw x3 --forward 3.0 --left 2.25 --up 0 --yaw 0
#

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

    The X5 reference defines the camera mounting orientation (vertical, front
    lens down). All cameras are assumed to share this mounting orientation —
    only position and yaw differ between cameras.
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

    # Apply yaw offset relative to X5's yaw
    # yaw_deg=0 means camera faces same direction as the X5 (scanner front)
    new_yaw = x5_yaw + np.radians(yaw_deg)

    # Build rotation matrix for T_camera_lidar
    R_cam_lidar = R.from_euler('xyz', [x5_roll, x5_pitch, new_yaw]).as_matrix()

    # Camera position in LiDAR frame (inches to meters)
    cam_pos_lidar = np.array([
        forward_in * INCHES_TO_METERS,
        left_in * INCHES_TO_METERS,
        up_in * INCHES_TO_METERS,
    ])

    # T_camera_lidar translation = R_cam_lidar @ (-cam_pos_lidar)
    # This expresses the LiDAR origin in the camera's coordinate frame
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
        description='Compute calibration seed from physical camera position measurements.\n\n'
                    'Stand behind the scanner (MID360 cable facing you).\n'
                    '+X = forward (away from you), +Y = left, +Z = up.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera-hw', required=True,
                        help='Camera hardware model (x5, x3, onex2)')
    parser.add_argument('--forward', type=float, required=True,
                        help='Camera distance forward from LiDAR center (inches). '
                             'Positive=toward front, Negative=toward cable/back')
    parser.add_argument('--left', type=float, required=True,
                        help='Camera distance left from LiDAR center (inches). '
                             'Positive=left, Negative=right (as seen from behind)')
    parser.add_argument('--up', type=float, required=True,
                        help='Camera distance above LiDAR center (inches). '
                             'Positive=above, Negative=below')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='Camera yaw rotation (degrees). '
                             '0=faces forward like LiDAR, +90=faces left, -90=faces right')
    args = parser.parse_args()

    calib = compute_physical_seed(args.camera_hw, args.forward, args.left, args.up, args.yaw)

    # Write to per-hw calibration file
    out_path = _CALIB_DIR / args.camera_hw / 'fusion_calibration.yaml'
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))

    # Also write to the per-camera-slot path from multi_camera.yaml if cam index is known
    import os as _os
    _cam_idx = _os.environ.get('ATLAS_CALIBRATION_CAM_INDEX', '')
    if _cam_idx:
        try:
            mc_path = _SRC / 'config' / 'multi_camera.yaml'
            if mc_path.exists():
                mc = yaml.safe_load(mc_path.read_text()) or {}
                cam_cfg = mc.get('cameras', {}).get(f'cam_{_cam_idx}', {})
                calib_rel = cam_cfg.get('calibration', '')
                if calib_rel:
                    slot_path = _SRC / 'config' / calib_rel
                    slot_path.parent.mkdir(parents=True, exist_ok=True)
                    slot_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))
                    print(f'✓ Also saved to slot path: {slot_path}')
        except Exception as e:
            print(f'  (could not write slot path: {e})')

    # Also update the active calibration
    active_path = _SRC / 'config' / 'fusion_calibration.yaml'
    active_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))

    print(f"Physical seed for {args.camera_hw}:")
    print(f"")
    print(f"  Reference: stand behind scanner, MID360 cable facing you")
    print(f"  +X=forward  +Y=left  +Z=up")
    print(f"")
    print(f"  Position (from LiDAR center):")
    print(f"    Forward: {args.forward:+.2f}\"  ({args.forward * INCHES_TO_METERS * 100:+.1f} cm)")
    print(f"    Left:    {args.left:+.2f}\"  ({args.left * INCHES_TO_METERS * 100:+.1f} cm)")
    print(f"    Up:      {args.up:+.2f}\"  ({args.up * INCHES_TO_METERS * 100:+.1f} cm)")
    print(f"")
    print(f"  Yaw: {args.yaw:+.1f}° (0=forward, +90=left, -90=right)")
    print(f"")
    print(f"  Computed T_camera_lidar:")
    print(f"    Translation: [{calib['x_offset']:.4f}, {calib['y_offset']:.4f}, {calib['z_offset']:.4f}] m")
    print(f"    Euler XYZ:   [{np.degrees(calib['roll_offset']):.2f}°, "
          f"{np.degrees(calib['pitch_offset']):.2f}°, "
          f"{np.degrees(calib['yaw_offset']):.2f}°]")
    print(f"    Image:       {calib['image_width']}x{calib['image_height']}")
    print(f"")
    print(f"✓ Saved to: {out_path}")
    print(f"✓ Active calibration updated: {active_path}")


if __name__ == '__main__':
    main()
