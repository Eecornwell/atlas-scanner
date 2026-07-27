#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Interactive calibration seed viewer with sliders.
# Adjusts Forward/Left/Up (inches) and Roll/Pitch/Yaw (degrees) in real-time,
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

# Rotation sliders: full ±360° range at 1°/tick so every orientation is reachable
# regardless of which side of the ±180° Euler discontinuity the camera lands on.
R_RANGE  = 360   # half-range in ticks  (±360° total)
R_SCALE  = 1.0   # degrees per tick
ROLL_CENTER  = 0.0
PITCH_CENTER = 0.0
YAW_CENTER   = 0.0

# Translation slider: ±24" in 0.05" steps
T_RANGE = 480   # half-range in ticks
T_SCALE = 0.05  # inches per tick


def _load_calibration(camera_hw, cam_index):
    """Return the current calibration dict for this camera, or None."""
    import sys as _sys
    _sys.path.insert(0, str(_SRC))
    from camera_hw import calibration_path as _calib_path
    p = _calib_path(camera_hw, cam_index)
    if p.exists():
        with open(p) as f:
            return yaml.safe_load(f)
    return None


_ORIENTATION_FALLBACK = {
    'x5':    (-179.77,  0.95, -91.77),
    'x3':    (-178.54,  2.92, -81.56),
    'onex2': (-179.0,  3.0, -98.0),
}


def _mounting_orientation(camera_hw, cam_index):
    """Return (roll_deg, pitch_deg, yaw_deg) from multi_camera.yaml mounting_orientation.
    Falls back to the hardcoded table if not set."""
    mc_path = _SRC / 'config' / 'multi_camera.yaml'
    if mc_path.exists():
        try:
            mc = yaml.safe_load(mc_path.read_text()) or {}
            # Try exact slot index first
            if cam_index is not None:
                slot = mc.get('cameras', {}).get(f'cam_{cam_index}', {})
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


def _init_angles(camera_hw, cam_index):
    """Return (roll_deg, pitch_deg, yaw_deg) to initialise the sliders.
    Uses the camera's own calibration when it has non-zero translation.
    Falls back to mounting_orientation from multi_camera.yaml.
    """
    cfg = _load_calibration(camera_hw, cam_index)
    if cfg is not None:
        t_mag = (cfg.get('x_offset', 0.0)**2 +
                 cfg.get('y_offset', 0.0)**2 +
                 cfg.get('z_offset', 0.0)**2) ** 0.5
        if t_mag > 0.001:
            return (np.degrees(cfg['roll_offset']),
                    np.degrees(cfg['pitch_offset']),
                    np.degrees(cfg['yaw_offset']))
    return _mounting_orientation(camera_hw, cam_index)


def _init_translation(camera_hw, cam_index):
    """Return (fwd_in, left_in, up_in) decoded from the current calibration."""
    cfg = _load_calibration(camera_hw, cam_index)
    if cfg is None:
        return 0.0, 0.0, 0.0
    R_mat = R.from_euler('xyz', [
        cfg['roll_offset'], cfg['pitch_offset'], cfg['yaw_offset']
    ]).as_matrix()
    t_cam = np.array([cfg['x_offset'], cfg['y_offset'], cfg['z_offset']])
    cam_pos_lidar = -R_mat.T @ t_cam
    return (cam_pos_lidar[0] / INCHES_TO_METERS,
            cam_pos_lidar[1] / INCHES_TO_METERS,
            cam_pos_lidar[2] / INCHES_TO_METERS)


def compute_T(roll_deg, pitch_deg, yaw_deg, fwd_m, left_m, up_m):
    """Compute T_camera_lidar from absolute orientation (degrees) and position."""
    R_mat = R.from_euler('xyz', np.radians([roll_deg, pitch_deg, yaw_deg])).as_matrix()
    cam_pos_lidar = np.array([fwd_m, left_m, up_m])
    t = R_mat @ (-cam_pos_lidar)
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = t
    return T


def project_points(points, T_cam_lidar, width, height):
    """Project points into ERP using T_camera_lidar."""
    pts_cam = (T_cam_lidar[:3, :3] @ points.T).T + T_cam_lidar[:3, 3]
    norm = np.linalg.norm(pts_cam, axis=1)
    valid = norm > 0.3
    bearing = pts_cam[valid] / norm[valid, None]
    lat = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
    lon = np.arctan2(bearing[:, 0], bearing[:, 2])
    u = (width * (0.5 + lon / (2 * np.pi))).astype(int) % width
    v = np.clip((height * (0.5 - lat / np.pi)).astype(int), 0, height - 1)
    return u, v, valid, norm[valid]


def _clamp_to_slider(val, half_range, scale, center=0.0):
    """Convert an absolute value to a slider tick position, clamped to range."""
    ticks = int(round((val - center) / scale))
    ticks = max(-half_range, min(half_range, ticks))
    return half_range + ticks


def _read_slider(pos, half_range, scale, center=0.0):
    """Convert a slider tick position back to an absolute value."""
    return center + (pos - half_range) * scale


def main():
    parser = argparse.ArgumentParser(description='Interactive calibration seed viewer')
    parser.add_argument('session_dir', help='Session directory with fusion_scan_* subdirs')
    parser.add_argument('--camera-hw', default=None)
    parser.add_argument('--cam-index', type=int, default=None)
    args = parser.parse_args()

    session = Path(args.session_dir)

    camera_hw = args.camera_hw
    if not camera_hw:
        sess_cfg = session / 'session_config.json'
        if sess_cfg.exists():
            import json
            camera_hw = json.loads(sess_cfg.read_text()).get('camera_hw', 'x5')
        else:
            camera_hw = 'x5'

    import sys as _sys
    _sys.path.insert(0, str(_SRC))
    from camera_hw import calibration_path as _calib_path
    slot_calib_path = _calib_path(camera_hw, args.cam_index)
    print(f"Camera HW: {camera_hw}  cam_index={args.cam_index}")
    print(f"Calibration file: {slot_calib_path}")

    scan_dirs = sorted(d for d in session.iterdir()
                       if d.is_dir() and d.name.startswith('fusion_scan_'))
    if not scan_dirs:
        print("No fusion_scan_* directories found")
        return

    first_scan = scan_dirs[0]
    cam_files = sorted(first_scan.glob('equirect_*_masked.png'))
    if not cam_files:
        cam_files = sorted(first_scan.glob('equirect_dual_fisheye.jpg'))
    if not cam_files:
        print("No ERP image found")
        return

    cam_img_full = cv2.imread(str(cam_files[0]))
    full_h, full_w = cam_img_full.shape[:2]
    scale = min(1920 / full_w, 960 / full_h, 1.0)
    disp_w = int(full_w * scale)
    disp_h = int(full_h * scale)
    cam_img = cv2.resize(cam_img_full, (disp_w, disp_h))

    ply_files = sorted(first_scan.glob('sensor_lidar*.ply'))
    if not ply_files:
        print("No sensor_lidar PLY found")
        return

    from plyfile import PlyData
    print(f"Loading {ply_files[0].name}...")
    ply = PlyData.read(str(ply_files[0]))
    v = ply['vertex']
    points = np.column_stack([v['x'], v['y'], v['z']]).astype(np.float32)

    # Load calibration mask
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

    # Initialise sliders from the current calibration (absolute degrees/inches)
    init_roll_deg, init_pitch_deg, init_yaw_deg = _init_angles(camera_hw, args.cam_index)
    init_fwd, init_left, init_up = _init_translation(camera_hw, args.cam_index)

    print(f"Init angles:  roll={init_roll_deg:.1f}° pitch={init_pitch_deg:.1f}° yaw={init_yaw_deg:.1f}°")
    print(f"Init position: fwd={init_fwd:.2f}\" left={init_left:.2f}\" up={init_up:.2f}\"")

    win = 'Calibration Seed - S=Save Q=Quit'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, disp_w, disp_h + 200)

    # Rotation sliders: centred at ROLL_CENTER/PITCH_CENTER/YAW_CENTER so the
    # typical camera orientations sit in the middle of the range, not at the edge.
    cv2.createTrackbar('Roll (deg)',  win, _clamp_to_slider(init_roll_deg,  R_RANGE, R_SCALE, ROLL_CENTER),  R_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Pitch (deg)', win, _clamp_to_slider(init_pitch_deg, R_RANGE, R_SCALE, PITCH_CENTER), R_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Yaw (deg)',   win, _clamp_to_slider(init_yaw_deg,   R_RANGE, R_SCALE, YAW_CENTER),   R_RANGE * 2, lambda x: None)
    # Translation sliders: centred at 0
    cv2.createTrackbar('Fwd (in)',    win, _clamp_to_slider(init_fwd,   T_RANGE, T_SCALE), T_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Left (in)',   win, _clamp_to_slider(init_left,  T_RANGE, T_SCALE), T_RANGE * 2, lambda x: None)
    cv2.createTrackbar('Up (in)',     win, _clamp_to_slider(init_up,    T_RANGE, T_SCALE), T_RANGE * 2, lambda x: None)

    print(f"\nInteractive Seed Viewer - {camera_hw}")
    print(f"  Roll  slider: {ROLL_CENTER:.0f}° ± {R_RANGE}°   Pitch: {PITCH_CENTER:.0f}° ± {R_RANGE}°   Yaw: {YAW_CENTER:.0f}° ± {R_RANGE}°")
    print(f"  Translation: ±{T_RANGE * T_SCALE:.0f}\" range, {T_SCALE}\"/tick")
    print(f"  S = Save    Q/ESC = Quit without saving    V = toggle edge overlay")

    saved = False
    show_edges = False
    while True:
        roll_deg  = _read_slider(cv2.getTrackbarPos('Roll (deg)',  win), R_RANGE, R_SCALE, ROLL_CENTER)
        pitch_deg = _read_slider(cv2.getTrackbarPos('Pitch (deg)', win), R_RANGE, R_SCALE, PITCH_CENTER)
        yaw_deg   = _read_slider(cv2.getTrackbarPos('Yaw (deg)',   win), R_RANGE, R_SCALE, YAW_CENTER)
        fwd_in    = _read_slider(cv2.getTrackbarPos('Fwd (in)',    win), T_RANGE, T_SCALE)
        left_in   = _read_slider(cv2.getTrackbarPos('Left (in)',   win), T_RANGE, T_SCALE)
        up_in     = _read_slider(cv2.getTrackbarPos('Up (in)',     win), T_RANGE, T_SCALE)

        T = compute_T(roll_deg, pitch_deg, yaw_deg,
                      fwd_in * INCHES_TO_METERS,
                      left_in * INCHES_TO_METERS,
                      up_in * INCHES_TO_METERS)

        u, v_px, valid, depths = project_points(points, T, disp_w, disp_h)

        display = cam_img.copy()
        display[~cam_mask] = display[~cam_mask] // 4

        depth_img = np.zeros((disp_h, disp_w), dtype=np.float32)
        depth_img[v_px, u] = depths
        depth_img = cv2.dilate(depth_img,
                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
        depth_img[~cam_mask] = 0

        d_valid = depth_img[depth_img > 0]
        if len(d_valid) > 0:
            d_min, d_max = np.percentile(d_valid, [5, 95])
            depth_u8 = np.clip((depth_img - d_min) / max(d_max - d_min, 0.1), 0, 1)
            depth_u8 = (depth_u8 * 255).astype(np.uint8)
        else:
            depth_u8 = np.zeros((disp_h, disp_w), dtype=np.uint8)
        depth_u8[~cam_mask] = 0
        lid_mask_bool = depth_u8 > 0

        lid_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        display[lid_mask_bool] = cv2.addWeighted(
            cam_img, 0.3, lid_color, 0.7, 0)[lid_mask_bool]

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

        def _put(img, txt, pos, scale=0.6, color=(255, 255, 255)):
            cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                        scale, color, 1, cv2.LINE_AA)

        _put(display, f'T: Fwd={fwd_in:.2f}" Left={left_in:.2f}" Up={up_in:.2f}"', (10, 28))
        _put(display, f'R: Roll={roll_deg:.0f} Pitch={pitch_deg:.0f} Yaw={yaw_deg:.0f} deg (absolute)', (10, 54))
        _put(display, f'S=Save  Q=Quit{edge_hint}', (10, 80), scale=0.5, color=(0, 255, 255))

        cv2.imshow(win, display)
        key = cv2.waitKey(30)
        if key == -1:
            pass
        elif key & 0xFF in (ord('q'), 27):
            break
        elif key & 0xFF == ord('v'):
            show_edges = not show_edges
        elif key & 0xFF == ord('s'):
            roll_rad  = np.radians(roll_deg)
            pitch_rad = np.radians(pitch_deg)
            yaw_rad   = np.radians(yaw_deg)
            calib = {
                'roll_offset':             float(roll_rad),
                'pitch_offset':            float(pitch_rad),
                'yaw_offset':              float(yaw_rad),
                'manual_roll_adjustment':  0.0,
                'manual_pitch_adjustment': 0.0,
                'manual_yaw_adjustment':   0.0,
                'azimuth_offset':          0.0,
                'elevation_offset':        0.0,
                'x_offset':                float(T[0, 3]),
                'y_offset':                float(T[1, 3]),
                'z_offset':                float(T[2, 3]),
                'flip_x':                  False,
                'flip_y':                  False,
                'image_width':             full_w,
                'image_height':            full_h,
                'use_fisheye':             False,
                'skip_rate':               5,
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
            print(f'  T: Fwd={fwd_in:.2f}" Left={left_in:.2f}" Up={up_in:.2f}"')
            print(f'  R: Roll={roll_deg:.1f} Pitch={pitch_deg:.1f} Yaw={yaw_deg:.1f} deg')
            saved = True
            break

    cv2.destroyAllWindows()
    if not saved:
        print("\nQuit without saving.")


if __name__ == '__main__':
    main()
