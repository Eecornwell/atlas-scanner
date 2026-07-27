#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Writes a calibration seed from physical measurements of camera
# position relative to the LiDAR center.
#
# The rotation (roll/pitch/yaw) is always copied from the X5 calibration —
# all cameras on this rig share the same mounting orientation so the rotation
# matrix is identical.  Only the translation (camera position) differs between
# cameras and is computed from the physical measurements here.
#
# Fine-tune the rotation with the interactive viewer after running this if
# there are small mounting angle differences.
#
# ─── Coordinate System ───────────────────────────────────────────────────────
#
# Stand behind the scanner (MID360 cable facing you). LiDAR center = origin.
#
#   +X = FORWARD  (away from you, toward the front of the scanner)
#   +Y = LEFT     (to your left as you face the scanner front)
#   +Z = UP
#
# ─── Usage ───────────────────────────────────────────────────────────────────
#
#   python3 physical_seed.py --camera-hw onex2 --forward 3.0 --left -3.0 --up 0

import argparse
import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_SRC      = Path(__file__).resolve().parent.parent
_CALIB_DIR  = _SRC / 'config' / 'calibrations'
_MODELS_DIR = _SRC / 'config' / 'camera_models'

INCHES_TO_METERS = 0.0254

# Per-camera reference orientations — fallback only if multi_camera.yaml
# does not have a mounting_orientation for this camera.
_ORIENTATION_FALLBACK = {
    'x5':    (-179.77,  0.95, -91.77),
    'x3':    (-178.54,  2.92, -81.56),
    'onex2': (-179.0,  3.0, -98.0),
}


def _reference_orientation(camera_hw):
    """Return (roll_deg, pitch_deg, yaw_deg) for this camera.

    Resolution order:
    1. mounting_orientation from the matching slot in multi_camera.yaml
       (matched by camera_hw, so single-camera sessions work too).
    2. Hardcoded fallback table.
    """
    mc_path = _SRC / 'config' / 'multi_camera.yaml'
    if mc_path.exists():
        try:
            mc = yaml.safe_load(mc_path.read_text()) or {}
            import os as _os
            cam_idx = _os.environ.get('ATLAS_CALIBRATION_CAM_INDEX', '')
            # Try exact slot match first
            if cam_idx:
                slot = mc.get('cameras', {}).get(f'cam_{cam_idx}', {})
                if slot.get('camera_hw') == camera_hw and slot.get('mounting_orientation'):
                    r, p, y = slot['mounting_orientation']
                    return float(r), float(p), float(y)
            # Fall back to first slot matching camera_hw
            for slot in mc.get('cameras', {}).values():
                if slot.get('camera_hw') == camera_hw and slot.get('mounting_orientation'):
                    r, p, y = slot['mounting_orientation']
                    return float(r), float(p), float(y)
        except Exception:
            pass
    return _ORIENTATION_FALLBACK.get(camera_hw, _ORIENTATION_FALLBACK['x5'])


def compute_physical_seed(camera_hw, forward_in, left_in, up_in):
    """Compute T_camera_lidar from physical position measurements.

    Rotation is taken from the per-camera reference orientation table.
    Translation is computed from the physical camera position in the lidar frame.
    """
    roll_deg, pitch_deg, yaw_deg = _reference_orientation(camera_hw)
    R_mat = R.from_euler('xyz', np.radians([roll_deg, pitch_deg, yaw_deg])).as_matrix()
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    cam_pos_lidar = np.array([
        forward_in * INCHES_TO_METERS,
        left_in    * INCHES_TO_METERS,
        up_in      * INCHES_TO_METERS,
    ])
    # T_camera_lidar translation: expresses lidar origin in camera frame
    t = R_mat @ (-cam_pos_lidar)

    hw_yaml = _MODELS_DIR / f'{camera_hw}.yaml'
    img_w, img_h = 5760, 2880
    if hw_yaml.exists():
        cfg = yaml.safe_load(hw_yaml.read_text()) or {}
        img_w = cfg.get('erp_width',  img_w)
        img_h = cfg.get('erp_height', img_h)

    return {
        'roll_offset':             float(roll),
        'pitch_offset':            float(pitch),
        'yaw_offset':              float(yaw),
        'manual_roll_adjustment':  0.0,
        'manual_pitch_adjustment': 0.0,
        'manual_yaw_adjustment':   0.0,
        'azimuth_offset':          0.0,
        'elevation_offset':        0.0,
        'x_offset':                float(t[0]),
        'y_offset':                float(t[1]),
        'z_offset':                float(t[2]),
        'flip_x':                  False,
        'flip_y':                  False,
        'image_width':             img_w,
        'image_height':            img_h,
        'use_fisheye':             False,
        'skip_rate':               5,
    }


def main():
    parser = argparse.ArgumentParser(
        description='Write calibration seed from physical camera position.\n\n'
                    'Rotation is copied from the X5 calibration (same for all\n'
                    'cameras on this rig).  Fine-tune with the interactive viewer.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera-hw', required=True,
                        help='Camera hardware model (x5, x3, onex2)')
    parser.add_argument('--forward', type=float, required=True,
                        help='Camera distance forward from LiDAR center (inches). '
                             'Positive=toward front, Negative=toward cable/back')
    parser.add_argument('--left', type=float, required=True,
                        help='Camera distance left from LiDAR center (inches). '
                             'Negative = right side of rig')
    parser.add_argument('--up', type=float, required=True,
                        help='Camera distance above LiDAR center (inches)')
    args = parser.parse_args()

    calib = compute_physical_seed(args.camera_hw, args.forward, args.left, args.up)

    import os as _os
    _cam_idx = _os.environ.get('ATLAS_CALIBRATION_CAM_INDEX', '')

    out_path = _CALIB_DIR / args.camera_hw / 'fusion_calibration.yaml'
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))

    if _cam_idx:
        try:
            mc_path = _SRC / 'config' / 'multi_camera.yaml'
            if mc_path.exists():
                mc = yaml.safe_load(mc_path.read_text()) or {}
                calib_rel = mc.get('cameras', {}).get(f'cam_{_cam_idx}', {}).get('calibration', '')
                if calib_rel:
                    slot_path = _SRC / 'config' / calib_rel
                    slot_path.parent.mkdir(parents=True, exist_ok=True)
                    slot_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))
                    print(f'✓ Saved to slot path: {slot_path}')
        except Exception as e:
            print(f'  (could not write slot path: {e})')

    active_path = _SRC / 'config' / 'fusion_calibration.yaml'
    active_path.write_text(yaml.dump(calib, default_flow_style=False, sort_keys=False))

    r_deg, p_deg, y_deg = _reference_orientation(args.camera_hw)
    R_mat = R.from_euler('xyz', np.radians([r_deg, p_deg, y_deg])).as_matrix()
    pos = -R_mat.T @ np.array([calib['x_offset'], calib['y_offset'], calib['z_offset']])
    print(f'Physical seed for {args.camera_hw}:')
    print(f'  Position (from measurements): Fwd={args.forward:+.2f}" Left={args.left:+.2f}" Up={args.up:+.2f}"')
    print(f'  Position (decoded check):     Fwd={pos[0]/INCHES_TO_METERS:+.2f}" Left={pos[1]/INCHES_TO_METERS:+.2f}" Up={pos[2]/INCHES_TO_METERS:+.2f}"')
    print(f'  Rotation: roll={np.degrees(calib["roll_offset"]):.2f}° pitch={np.degrees(calib["pitch_offset"]):.2f}° yaw={np.degrees(calib["yaw_offset"]):.2f}°')
    print(f'  Translation: [{calib["x_offset"]:.4f}, {calib["y_offset"]:.4f}, {calib["z_offset"]:.4f}] m')
    print(f'✓ Saved to: {out_path}')
    print(f'  → Open Interactive Viewer to fine-tune if needed')


if __name__ == '__main__':
    main()
