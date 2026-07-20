#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Interactive calibration seed viewer with sliders.
# Adjusts Forward/Left/Up (inches) and Yaw (degrees) in real-time,
# showing the LiDAR projection overlaid on the camera ERP.
# Press 'S' to save the current values as the calibration seed.
# Press 'Q' or ESC to quit without saving.
#
# Usage:
#   python3 interactive_seed.py <session_dir> [--camera-hw x3]

import argparse
import cv2
import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_SRC = Path(__file__).resolve().parent.parent
_CALIB_DIR = _SRC / 'config' / 'calibrations'
_MODELS_DIR = _SRC / 'config' / 'camera_models'

INCHES_TO_METERS = 0.0254


def load_x5_orientation():
    """Load the X5 reference rotation (roll/pitch/yaw)."""
    x5_path = _CALIB_DIR / 'x5' / 'fusion_calibration.yaml'
    with open(x5_path) as f:
        cfg = yaml.safe_load(f)
    return cfg['roll_offset'], cfg['pitch_offset'], cfg['yaw_offset']


def load_seed_values(camera_hw, cam_index=None):
    """Load saved translation and yaw from the camera's slot calibration file.
    Resolves through calibration_path() so it always reads the same file
    that exact_match_fusion.py uses during coloring."""
    import sys as _sys
    _sys.path.insert(0, str(_SRC))
    from camera_hw import calibration_path as _calib_path
    calib_path = _calib_path(camera_hw, cam_index)
    if not calib_path.exists():
        return 3.0, 0.0, 0.0, 0.0
    with open(calib_path) as f:
        cfg = yaml.safe_load(f)

    x5_path = _CALIB_DIR / 'x5' / 'fusion_calibration.yaml'
    with open(x5_path) as f:
        x5 = yaml.safe_load(f)

    yaw_deg = float(np.degrees(cfg['yaw_offset'] - x5['yaw_offset']))

    R_mat = R.from_euler('xyz', [
        cfg['roll_offset'], cfg['pitch_offset'], cfg['yaw_offset']
    ]).as_matrix()
    t_cam = np.array([cfg['x_offset'], cfg['y_offset'], cfg['z_offset']])
    cam_pos_lidar = -R_mat.T @ t_cam

    fwd_in  = cam_pos_lidar[0] / INCHES_TO_METERS
    left_in = cam_pos_lidar[1] / INCHES_TO_METERS
    up_in   = cam_pos_lidar[2] / INCHES_TO_METERS
    return fwd_in, left_in, up_in, yaw_deg


def compute_T(roll, pitch, yaw, fwd_m, left_m, up_m):
    """Compute T_camera_lidar from orientation and position."""
    R_mat = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    cam_pos_lidar = np.array([fwd_m, left_m, up_m])
    t = R_mat @ (-cam_pos_lidar)
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = t
    return T


def project_points(points, T_cam_lidar, width, height):
    """Project points into ERP using T_camera_lidar. Returns u, v, valid_mask, depths."""
    pts_cam = (T_cam_lidar[:3, :3] @ points.T).T + T_cam_lidar[:3, 3]
    norm = np.linalg.norm(pts_cam, axis=1)
    valid = norm > 0.3
    bearing = pts_cam[valid] / norm[valid, None]

    lat = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
    lon = np.arctan2(bearing[:, 0], bearing[:, 2])
    u = (width * (0.5 + lon / (2 * np.pi))).astype(int) % width
    v = np.clip((height * (0.5 - lat / np.pi)).astype(int), 0, height - 1)
    return u, v, valid, norm[valid]


def main():
    parser = argparse.ArgumentParser(description='Interactive calibration seed viewer')
    parser.add_argument('session_dir', help='Session directory with fusion_scan_* subdirs')
    parser.add_argument('--camera-hw', default=None, help='Camera hardware (auto-detected if not set)')
    parser.add_argument('--cam-index', type=int, default=None,
                        help='Camera slot index (0/1/2). Required for multi-camera rigs to '
                             'load and save the correct slot calibration.')
    args = parser.parse_args()

    session = Path(args.session_dir)

    # Detect camera_hw
    camera_hw = args.camera_hw
    if not camera_hw:
        sess_cfg = session / 'session_config.json'
        if sess_cfg.exists():
            import json
            camera_hw = json.loads(sess_cfg.read_text()).get('camera_hw', 'x5')
        else:
            camera_hw = 'x5'

    # Resolve the slot calibration path — same file coloring uses
    import sys as _sys
    _sys.path.insert(0, str(_SRC))
    from camera_hw import calibration_path as _calib_path
    slot_calib_path = _calib_path(camera_hw, args.cam_index)
    print(f"Camera HW: {camera_hw}  cam_index={args.cam_index}")
    print(f"Calibration file: {slot_calib_path}")

    # Find first scan
    scan_dirs = sorted(d for d in session.iterdir()
                      if d.is_dir() and d.name.startswith('fusion_scan_'))
    if not scan_dirs:
        print("No fusion_scan_* directories found")
        return

    first_scan = scan_dirs[0]

    # Load camera image (prefer masked)
    cam_files = sorted(first_scan.glob('equirect_*_masked.png'))
    if not cam_files:
        cam_files = sorted(first_scan.glob('equirect_dual_fisheye.jpg'))
    if not cam_files:
        print("No ERP image found")
        return

    cam_img_full = cv2.imread(str(cam_files[0]))
    full_h, full_w = cam_img_full.shape[:2]

    # Downsample for interactive display
    scale = min(1920 / full_w, 960 / full_h, 1.0)
    disp_w = int(full_w * scale)
    disp_h = int(full_h * scale)
    cam_img = cv2.resize(cam_img_full, (disp_w, disp_h))

    # Load PLY
    ply_files = sorted(first_scan.glob('sensor_lidar*.ply'))
    if not ply_files:
        print("No sensor_lidar PLY found")
        return

    from plyfile import PlyData
    print(f"Loading {ply_files[0].name}...")
    ply = PlyData.read(str(ply_files[0]))
    v = ply['vertex']
    points = np.column_stack([v['x'], v['y'], v['z']]).astype(np.float32)

    # Load calibration mask from multi_camera.yaml
    cam_mask = np.ones((disp_h, disp_w), dtype=bool)
    mc_path = _SRC / 'config' / 'multi_camera.yaml'
    mask_name = ''
    if mc_path.exists():
        mc = yaml.safe_load(mc_path.read_text()) or {}
        for cam in mc.get('cameras', {}).values():
            if cam.get('camera_hw', '') == camera_hw:
                mask_name = cam.get('mask_calibration', cam.get('mask_dual', ''))
                break
    if not mask_name:
        hw_yaml = _MODELS_DIR / f'{camera_hw}.yaml'
        if hw_yaml.exists():
            _cfg = yaml.safe_load(hw_yaml.read_text()) or {}
            mask_name = _cfg.get('lidar_mask_dual', '')
    if mask_name:
        mask_path = _SRC / 'config' / 'masks' / mask_name
        if mask_path.exists():
            m = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            m = cv2.resize(m, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)
            cam_mask = m > 128
            print(f"Calibration mask: {mask_name}")

    # Load X5 reference orientation and current seed values
    x5_roll, x5_pitch, x5_yaw = load_x5_orientation()
    init_fwd, init_left, init_up, init_yaw = load_seed_values(camera_hw, args.cam_index)
    print(f"Loaded seed: fwd={init_fwd:.2f}\" left={init_left:.2f}\" up={init_up:.2f}\" yaw={init_yaw:.1f}°")

    # Slider ranges: generous enough to not max out
    # Translation: -12" to +12" (0.05" per tick, 480 ticks)
    # Rotation: -30 to +30 degrees (0.1 deg per tick, 600 ticks)
    T_RANGE = 480   # half-range in ticks
    T_SCALE = 0.05  # inches per tick
    R_RANGE = 300   # half-range in ticks
    R_SCALE = 0.1   # degrees per tick

    win = 'Calibration Seed - S=Save Q=Quit'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, disp_w, disp_h + 200)

    cv2.createTrackbar('Fwd (in)',    win, T_RANGE + int(round(init_fwd  / T_SCALE)), T_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Left (in)',   win, T_RANGE + int(round(init_left / T_SCALE)), T_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Up (in)',     win, T_RANGE + int(round(init_up   / T_SCALE)), T_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Roll (deg)',  win, R_RANGE, R_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Pitch (deg)', win, R_RANGE, R_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Yaw (deg)',   win, R_RANGE + int(round(init_yaw  / R_SCALE)), R_RANGE * 2, lambda x: None)

    print(f"\nInteractive Seed Viewer - {camera_hw}")
    print(f"  Adjust sliders to align LiDAR with camera features")
    print(f"  Translation: +/-{T_RANGE * T_SCALE:.1f}\" ({T_SCALE}\" per tick)")
    print(f"  Rotation: +/-{R_RANGE * R_SCALE:.1f}deg ({R_SCALE}deg per tick)")
    print(f"  S = Save calibration    Q/ESC = Quit without saving")
    print(f"  Image: {disp_w}x{disp_h} (scaled from {full_w}x{full_h})")

    saved = False
    show_edges = False  # press V to toggle edge overlay on top of dots
    while True:
        # Read slider values
        fwd_in   = (cv2.getTrackbarPos('Fwd (in)',   win) - T_RANGE) * T_SCALE
        left_in  = (cv2.getTrackbarPos('Left (in)',  win) - T_RANGE) * T_SCALE
        up_in    = (cv2.getTrackbarPos('Up (in)',    win) - T_RANGE) * T_SCALE
        roll_deg  = (cv2.getTrackbarPos('Roll (deg)', win) - R_RANGE) * R_SCALE
        pitch_deg = (cv2.getTrackbarPos('Pitch (deg)', win) - R_RANGE) * R_SCALE
        yaw_deg   = (cv2.getTrackbarPos('Yaw (deg)',  win) - R_RANGE) * R_SCALE

        # Compute transform (X5 base + adjustments)
        roll_rad = x5_roll + np.radians(roll_deg)
        pitch_rad = x5_pitch + np.radians(pitch_deg)
        yaw_rad = x5_yaw + np.radians(yaw_deg)
        T = compute_T(roll_rad, pitch_rad, yaw_rad,
                     fwd_in * INCHES_TO_METERS,
                     left_in * INCHES_TO_METERS,
                     up_in * INCHES_TO_METERS)

        # Project
        u, v_px, valid, depths = project_points(points, T, disp_w, disp_h)

        # Create overlay — original behavior: TURBO dots on camera
        # Press V to toggle edge overlay on top
        display = cam_img.copy()
        display[~cam_mask] = display[~cam_mask] // 4

        depth_img = np.zeros((disp_h, disp_w), dtype=np.float32)
        depth_img[v_px, u] = depths
        # Larger kernel than original (9x9 vs 3x3) for better visibility
        depth_img = cv2.dilate(depth_img,
                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
        depth_img[~cam_mask] = 0

        d_valid = depth_img[depth_img > 0]
        if len(d_valid) > 0:
            d_min, d_max = np.percentile(d_valid, [5, 95])
            depth_norm = np.clip((depth_img - d_min) / max(d_max - d_min, 0.1), 0, 1)
            depth_u8 = (depth_norm * 255).astype(np.uint8)
        else:
            depth_u8 = np.zeros((disp_h, disp_w), dtype=np.uint8)
        depth_u8[~cam_mask] = 0
        lid_mask_bool = depth_u8 > 0

        # Original: TURBO colormap blended onto camera (70% lidar, 30% camera)
        lid_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        display[lid_mask_bool] = cv2.addWeighted(
            cam_img, 0.3, lid_color, 0.7, 0)[lid_mask_bool]

        # Optional: edge overlay toggled with V
        if show_edges:
            cam_gray = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)
            cam_edges = cv2.Canny(cam_gray, 30, 90)
            cam_edges[~cam_mask] = 0
            cam_edges = cv2.dilate(cam_edges, np.ones((2, 2), np.uint8))
            lid_edges = cv2.Canny(depth_u8, 10, 40)
            lid_edges = cv2.dilate(lid_edges, np.ones((2, 2), np.uint8))
            display[cam_edges > 0] = (50, 50, 255)
            display[lid_edges > 0] = (50, 255, 50)
            display[(cam_edges > 0) & (lid_edges > 0)] = (0, 255, 255)
            edge_hint = '  V=edges OFF'
        else:
            edge_hint = '  V=edges ON'

        # Info text with black outline so it reads on any background
        def _put(img, txt, pos, scale=0.6, color=(255, 255, 255)):
            cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                        scale, color, 1, cv2.LINE_AA)

        _put(display, f'T: Fwd={fwd_in:.2f}" Left={left_in:.2f}" Up={up_in:.2f}"',
             (10, 28))
        _put(display, f'R: Roll={roll_deg:.1f} Pitch={pitch_deg:.1f} Yaw={yaw_deg:.1f} deg',
             (10, 54))
        _put(display, f'S=Save  Q=Quit{edge_hint}', (10, 80),
             scale=0.5, color=(0, 255, 255))

        cv2.imshow(win, display)
        key = cv2.waitKey(30)
        if key == -1:
            pass
        elif key & 0xFF in (ord('q'), 27):
            break
        elif key & 0xFF == ord('v'):
            show_edges = not show_edges
        elif key & 0xFF == ord('s'):
            calib = {
                'roll_offset': float(roll_rad),
                'pitch_offset': float(pitch_rad),
                'yaw_offset': float(yaw_rad),
                'manual_roll_adjustment': 0.0,
                'manual_pitch_adjustment': 0.0,
                'manual_yaw_adjustment': 0.0,
                'azimuth_offset': 0.0,
                'elevation_offset': 0.0,
                'x_offset': float(T[0, 3]),
                'y_offset': float(T[1, 3]),
                'z_offset': float(T[2, 3]),
                'flip_x': False,
                'flip_y': False,
                'image_width': full_w,
                'image_height': full_h,
                'use_fisheye': False,
                'skip_rate': 5,
            }
            text = yaml.dump(calib, default_flow_style=False, sort_keys=False)
            slot_calib_path.parent.mkdir(parents=True, exist_ok=True)
            slot_calib_path.write_text(text)
            hw_path = _CALIB_DIR / camera_hw / 'fusion_calibration.yaml'
            hw_path.parent.mkdir(parents=True, exist_ok=True)
            hw_path.write_text(text)
            (_SRC / 'config' / 'fusion_calibration.yaml').write_text(text)
            print(f'\n\u2713 Saved calibration:')
            print(f'  Slot: {slot_calib_path}')
            print(f'  T: Fwd={fwd_in:.2f} Left={left_in:.2f} Up={up_in:.2f} (inches)')
            print(f'  R: Roll={roll_deg:.1f} Pitch={pitch_deg:.1f} Yaw={yaw_deg:.1f} deg')
            saved = True
            break
    cv2.destroyAllWindows()
    if not saved:
        print("\nQuit without saving.")


if __name__ == '__main__':
    main()
