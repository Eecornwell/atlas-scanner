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
    'oak1':  (-45.0,  0.0, -90.0),
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


def project_points_pinhole(points, T_cam_lidar, K, dist, width, height):
    """Project points onto a rectilinear image using pinhole + distortion model."""
    pts_cam = (T_cam_lidar[:3, :3] @ points.T).T + T_cam_lidar[:3, 3]
    # Only points in front of the camera
    valid = pts_cam[:, 2] > 0.3
    pts_f = pts_cam[valid]
    norm = np.linalg.norm(pts_f, axis=1)
    # Normalised image coords
    xn = pts_f[:, 0] / pts_f[:, 2]
    yn = pts_f[:, 1] / pts_f[:, 2]
    # Rational polynomial distortion (k1..k6, p1, p2)
    r2 = xn**2 + yn**2
    r4, r6 = r2**2, r2**3
    k1, k2, p1, p2 = dist[0], dist[1], dist[2], dist[3]
    k3 = dist[4] if len(dist) > 4 else 0.0
    k4 = dist[5] if len(dist) > 5 else 0.0
    k5 = dist[6] if len(dist) > 6 else 0.0
    k6 = dist[7] if len(dist) > 7 else 0.0
    num = 1 + k1*r2 + k2*r4 + k3*r6
    den = 1 + k4*r2 + k5*r4 + k6*r6
    radial = num / np.where(den == 0, 1e-9, den)
    xd = xn*radial + 2*p1*xn*yn + p2*(r2 + 2*xn**2)
    yd = yn*radial + p1*(r2 + 2*yn**2) + 2*p2*xn*yn
    u = (K[0, 0]*xd + K[0, 2]).astype(int)
    v = (K[1, 1]*yd + K[1, 2]).astype(int)
    in_frame = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    # Return in same shape as ERP version: u, v, valid_mask, depths
    u_out = u[in_frame]
    v_out = v[in_frame]
    # Build full-length valid mask combining front-of-camera and in-frame
    full_valid = np.zeros(len(points), dtype=bool)
    idx = np.where(valid)[0][in_frame]
    full_valid[idx] = True
    return u_out, v_out, full_valid, norm[in_frame]


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
    # OAK-1: rectilinear PNG
    oak1_files = sorted(first_scan.glob('oak1_*.png'))
    oak1_files = [f for f in oak1_files if '_undistorted' not in f.name]
    is_pinhole = False
    pinhole_K = None
    pinhole_dist = None
    if not cam_files and oak1_files:
        cam_files = oak1_files
        is_pinhole = True
        # Load intrinsics from camera_info.yaml in the scan dir
        info_path = first_scan / 'camera_info.yaml'
        if not info_path.exists():
            info_path = session / 'camera_info.yaml'
        if info_path.exists():
            _info = yaml.safe_load(info_path.read_text())
            # Scale K to actual image size (depthai delivers 4000x3000 not 4056x3040)
            _img_probe = cv2.imread(str(cam_files[0]))
            _ah, _aw = _img_probe.shape[:2] if _img_probe is not None else (_info['height'], _info['width'])
            sx, sy = _aw / _info['width'], _ah / _info['height']
            pinhole_K = np.array([
                [_info['fx']*sx, 0, _info['cx']*sx],
                [0, _info['fy']*sy, _info['cy']*sy],
                [0, 0, 1]], dtype=np.float64)
            pinhole_dist = np.array(_info['D'][:8], dtype=np.float64)
        else:
            print("⚠ No camera_info.yaml found — projection may be inaccurate")
    if not cam_files:
        print("No camera image found (equirect or oak1 PNG)")
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
    # For pinhole: track crop region so user can zoom into where points land
    crop_enabled = is_pinhole
    crop_cx, crop_cy = disp_w // 2, disp_h // 2
    CROP_W, CROP_H = min(960, disp_w), min(720, disp_h)

    def _put(img, txt, pos, scale=0.55, color=(255, 255, 255)):
        cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)

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

        if is_pinhole and pinhole_K is not None:
            u, v_px, valid, depths = project_points_pinhole(
                points, T, pinhole_K, pinhole_dist, disp_w, disp_h)
        else:
            u, v_px, valid, depths = project_points(points, T, disp_w, disp_h)

        display = cam_img.copy()
        display[~cam_mask] = display[~cam_mask] // 4

        # Build depth image
        depth_img = np.zeros((disp_h, disp_w), dtype=np.float32)
        if len(u) > 0:
            depth_img[v_px, u] = depths

        # --- Range-discontinuity edges: much more readable than depth colormap ---
        # Dilate slightly just to fill gaps between sparse points, then find
        # where depth changes sharply (structural edges in the scene).
        depth_filled = cv2.dilate(depth_img,
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        depth_filled[~cam_mask] = 0

        d_valid = depth_filled[depth_filled > 0]
        if len(d_valid) > 0:
            d_min, d_max = np.percentile(d_valid, [2, 98])
            depth_u8 = np.clip((depth_filled - d_min) / max(d_max - d_min, 0.1), 0, 1)
            depth_u8 = (depth_u8 * 255).astype(np.uint8)
        else:
            depth_u8 = np.zeros((disp_h, disp_w), dtype=np.uint8)
        depth_u8[~cam_mask] = 0

        # Compute range-gradient edges from the depth image
        # Sobel on depth highlights depth discontinuities = structural edges
        if depth_u8.max() > 0:
            dx = cv2.Sobel(depth_u8.astype(np.float32), cv2.CV_32F, 1, 0, ksize=3)
            dy = cv2.Sobel(depth_u8.astype(np.float32), cv2.CV_32F, 0, 1, ksize=3)
            grad = np.sqrt(dx**2 + dy**2)
            grad_thresh = np.percentile(grad[grad > 0], 70) if (grad > 0).any() else 1
            lidar_edges = (grad > grad_thresh).astype(np.uint8) * 255
            lidar_edges = cv2.dilate(lidar_edges, np.ones((2, 2), np.uint8))
        else:
            lidar_edges = np.zeros((disp_h, disp_w), dtype=np.uint8)

        # Camera image edges
        cam_gray = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)
        cam_edges = cv2.Canny(cam_gray, 20, 60)
        cam_edges[~cam_mask] = 0

        # Composite: dim image, overlay LiDAR depth as thin colormap dots,
        # then overlay LiDAR structural edges in bright green,
        # camera edges in dim blue, coincident edges in yellow.
        display = (cam_img.astype(np.float32) * 0.5).astype(np.uint8)

        # Thin dot overlay for depth context (small dilation only)
        depth_dots = cv2.dilate(depth_img,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        depth_dots[~cam_mask] = 0
        if depth_dots.max() > 0:
            d2 = np.clip((depth_dots - d_min) / max(d_max - d_min, 0.1), 0, 1)
            d2_u8 = (d2 * 255).astype(np.uint8)
            dot_color = cv2.applyColorMap(d2_u8, cv2.COLORMAP_TURBO)
            dot_mask = depth_dots > 0
            display[dot_mask] = cv2.addWeighted(
                cam_img, 0.2, dot_color, 0.8, 0)[dot_mask]

        # LiDAR structural edges: bright green
        display[lidar_edges > 0] = (30, 220, 30)
        # Camera edges: dim blue
        display[cam_edges > 0] = np.clip(
            display[cam_edges > 0].astype(int) + [60, 30, 0], 0, 255).astype(np.uint8)
        # Coincident (aligned) edges: bright yellow
        coincident = (lidar_edges > 0) & (cam_edges > 0)
        display[coincident] = (0, 255, 255)

        n_pts = len(u)
        n_coincident = int(coincident.sum())

        # --- Crop view for pinhole: zoom into where points actually land ---
        if crop_enabled and n_pts > 0:
            # Centre crop on median of projected points
            crop_cx = int(np.median(u))
            crop_cy = int(np.median(v_px))
            crop_cx = max(CROP_W//2, min(disp_w - CROP_W//2, crop_cx))
            crop_cy = max(CROP_H//2, min(disp_h - CROP_H//2, crop_cy))
            x0 = crop_cx - CROP_W//2; x1 = x0 + CROP_W
            y0 = crop_cy - CROP_H//2; y1 = y0 + CROP_H
            crop = display[y0:y1, x0:x1].copy()
            # Scale crop up to fill display width
            scale_up = min(disp_w / CROP_W, disp_h / CROP_H)
            cw = int(CROP_W * scale_up); ch = int(CROP_H * scale_up)
            crop_big = cv2.resize(crop, (cw, ch), interpolation=cv2.INTER_NEAREST)
            # Draw crop rectangle on full view
            cv2.rectangle(display, (x0, y0), (x1, y1), (0, 200, 255), 2)
            # Show side by side: full (left) | crop zoomed (right)
            full_resized = cv2.resize(display, (disp_w, disp_h))
            # Pad crop to same height
            pad_top = (disp_h - ch) // 2
            pad_bot = disp_h - ch - pad_top
            if pad_top >= 0 and pad_bot >= 0:
                crop_padded = cv2.copyMakeBorder(crop_big, pad_top, pad_bot, 0, 0,
                                                  cv2.BORDER_CONSTANT, value=(20, 20, 20))
                combined = np.hstack([full_resized, crop_padded])
            else:
                combined = full_resized
            out_img = combined
        else:
            out_img = display

        _put(out_img, f'T: Fwd={fwd_in:.2f}" Left={left_in:.2f}" Up={up_in:.2f}"', (10, 28))
        _put(out_img, f'R: Roll={roll_deg:.0f} Pitch={pitch_deg:.0f} Yaw={yaw_deg:.0f} deg', (10, 54))
        _put(out_img, f'pts={n_pts}  aligned={n_coincident}  S=Save Q=Quit C=crop', (10, 80),
             scale=0.5, color=(0, 255, 255))
        _put(out_img, 'GREEN=LiDAR edges  YELLOW=aligned  BLUE=camera edges', (10, out_img.shape[0] - 10),
             scale=0.45, color=(180, 180, 180))

        cv2.imshow(win, out_img)
        key = cv2.waitKey(30)
        if key == -1:
            pass
        elif key & 0xFF in (ord('q'), 27):
            break
        elif key & 0xFF == ord('c'):
            crop_enabled = not crop_enabled
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
