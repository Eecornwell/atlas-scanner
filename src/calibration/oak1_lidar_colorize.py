#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# OAK-1 calibration: direct visual alignment.
#
# Projects LiDAR points onto the camera image using the current calibration.
# The user adjusts sliders until the projected dots align with visible features.
# This avoids the PnP degeneracy problem (single-wall geometry) entirely.
#
# The key insight: with a 45-deg upward camera, the image shows a wall at
# ~3m depth.  Small rotation errors shift ALL dots left/right/up/down uniformly.
# Small translation errors shift dots by a fixed pixel offset.
# This makes visual alignment intuitive and unambiguous.
#
# Layout:
#   Main panel: camera image with LiDAR dots overlaid.
#               Dots coloured by depth (blue=far, red=close).
#               Well-calibrated: dots land on the correct surfaces.
#
# Sliders: Roll / Pitch / Yaw (1 deg steps) + Fwd / Left / Up (0.1" steps)
# Fine mode (F key): 0.1 deg / 0.01" steps
# S = save    Q/Esc = quit without saving

import argparse, sys, cv2, yaml, numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_SRC = Path(__file__).resolve().parent.parent
_CALIB_DIR = _SRC / 'config' / 'calibrations'
INCHES = 0.0254

R_HALF, R_SCALE_C, R_SCALE_F = 360, 1.0, 0.1   # coarse/fine deg per tick
T_HALF, T_SCALE_C, T_SCALE_F = 480, 0.1, 0.01   # coarse/fine inches per tick


def _load_calib(camera_hw, cam_index):
    sys.path.insert(0, str(_SRC))
    from camera_hw import calibration_path
    p = calibration_path(camera_hw, cam_index)
    if p.exists():
        return yaml.safe_load(p.read_text()), p
    hw_p = _CALIB_DIR / camera_hw / 'fusion_calibration.yaml'
    if hw_p.exists():
        return yaml.safe_load(hw_p.read_text()), p
    return None, p


def _save_calib(c, slot_path, camera_hw):
    """Write calibration to the slot path, hw-level path, and active shared path.
    The slot path is resolved from multi_camera.yaml if available, so changes
    are picked up by the system regardless of which path it reads."""
    txt = yaml.dump(c, default_flow_style=False, sort_keys=False)
    # Always write to the slot path passed in (from calibration_path())
    slot_path.parent.mkdir(parents=True, exist_ok=True)
    slot_path.write_text(txt)
    # Also write to hw-level path
    hw_p = _CALIB_DIR / camera_hw / 'fusion_calibration.yaml'
    hw_p.parent.mkdir(parents=True, exist_ok=True)
    hw_p.write_text(txt)
    # Also write to active shared path
    (_SRC / 'config' / 'fusion_calibration.yaml').write_text(txt)
    print(f'  Written to: {slot_path}')
    print(f'  Written to: {hw_p}')


def _undistort(img, info):
    h, w = img.shape[:2]
    sx, sy = w / info['width'], h / info['height']
    K = np.array([[info['fx']*sx, 0, info['cx']*sx],
                  [0, info['fy']*sy, info['cy']*sy],
                  [0, 0, 1]], dtype=np.float64)
    dist = np.array(info['D'][:8], dtype=np.float64)
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0.0)
    return cv2.undistort(img, K, dist, None, new_K), new_K


def _project(pts, Rm, t, K, iw, ih):
    """Project LiDAR points into undistorted image. Returns u, v, depth arrays (in-frame only)."""
    pc = (Rm @ pts.T).T + t
    front = pc[:, 2] > 0.1
    if not front.any():
        return np.array([]), np.array([]), np.array([])
    pf = pc[front]
    u = K[0, 0] * pf[:, 0] / pf[:, 2] + K[0, 2]
    v = K[1, 1] * pf[:, 1] / pf[:, 2] + K[1, 2]
    d = np.linalg.norm(pf, axis=1)
    inf = (u >= 0) & (u < iw) & (v >= 0) & (v < ih)
    return u[inf], v[inf], d[inf]


def _overlay(img_bgr, u, v, depths, dot_r=3):
    """Draw depth-coloured dots on image."""
    out = img_bgr.copy()
    if len(u) == 0:
        return out
    d_min, d_max = np.percentile(depths, 2), np.percentile(depths, 98)
    d_u8 = np.clip((depths - d_min) / max(d_max - d_min, 0.01) * 255, 0, 255).astype(np.uint8)
    lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(1, -1),
                            cv2.COLORMAP_TURBO)[0]
    ui = np.round(u).astype(int)
    vi = np.round(v).astype(int)
    for i in range(len(ui)):
        col = tuple(int(c) for c in lut[d_u8[i]])
        cv2.circle(out, (ui[i], vi[i]), dot_r, col, -1)
    return out


def _put(img, txt, pos, scale=0.5, color=(220, 220, 220)):
    cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def _sv(pos, half, scale):
    return (pos - half) * scale


def _sp(val, half, scale):
    return max(0, min(half * 2, int(round(val / scale)) + half))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session_dir')
    ap.add_argument('--camera-hw', default='oak1')
    ap.add_argument('--cam-index', type=int, default=None)
    args = ap.parse_args()

    session   = Path(args.session_dir)
    camera_hw = args.camera_hw

    # Find scan
    scan_dir = img_path = ply_path = None
    for sd in sorted(d for d in session.iterdir()
                     if d.is_dir() and d.name.startswith('fusion_scan_')):
        imgs = [f for f in sorted(sd.glob('oak1_*.png'))
                if '_undistorted' not in f.name]
        plys = sorted(sd.glob('sensor_lidar*.ply'))
        if imgs and plys:
            scan_dir, img_path, ply_path = sd, imgs[0], plys[0]
            break
    if scan_dir is None:
        print('No scan with oak1 image + sensor_lidar PLY'); sys.exit(1)
    print(f'Scan: {scan_dir.name}  |  {img_path.name}')

    # Load and undistort image
    img_full = cv2.imread(str(img_path))
    info_p = scan_dir / 'camera_info.yaml'
    if not info_p.exists():
        info_p = session / 'camera_info.yaml'
    info = yaml.safe_load(info_p.read_text())
    ih, iw = img_full.shape[:2]
    info_s = dict(info, width=iw, height=ih,
                  fx=info['fx']*iw/info['width'], fy=info['fy']*ih/info['height'],
                  cx=info['cx']*iw/info['width'], cy=info['cy']*ih/info['height'])
    undist, K = _undistort(img_full, info_s)

    # Display size
    DISP_W = 1200
    sc = DISP_W / iw
    img_disp = cv2.resize(undist, (DISP_W, int(ih * sc)))
    disp_h = img_disp.shape[0]
    K_disp = K.copy()
    K_disp[0] *= sc; K_disp[1] *= sc

    # Load points — subsample for speed
    from plyfile import PlyData
    ply = PlyData.read(str(ply_path))
    vv  = ply['vertex']
    pts_all = np.column_stack([vv['x'], vv['y'], vv['z']]).astype(np.float64)
    # Use all points (60k is fast enough for projection)
    pts = pts_all
    print(f'{len(pts)} points')

    # Load calibration
    calib, slot_path = _load_calib(camera_hw, args.cam_index)
    if calib is None:
        calib = {'roll_offset': -0.7854, 'pitch_offset': 0.0,
                 'yaw_offset': -1.5708, 'x_offset': 0.0,
                 'y_offset': 0.0762, 'z_offset': 0.0}
    Rm0 = R.from_euler('xyz', [calib['roll_offset'],
                                calib['pitch_offset'],
                                calib['yaw_offset']]).as_matrix()
    t0  = np.array([calib['x_offset'], calib['y_offset'], calib['z_offset']])
    p0  = -Rm0.T @ t0

    WIN = 'OAK-1 Calibration  |  Align dots to features  |  S=Save  Q=Quit  F=Fine/Coarse'
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, DISP_W, disp_h + 160)

    fine = [False]

    def _make_sliders(r_scale, t_scale):
        cv2.createTrackbar('Roll  deg', WIN,
                           _sp(np.degrees(calib['roll_offset']),  R_HALF, r_scale), R_HALF*2, lambda x:None)
        cv2.createTrackbar('Pitch deg', WIN,
                           _sp(np.degrees(calib['pitch_offset']), R_HALF, r_scale), R_HALF*2, lambda x:None)
        cv2.createTrackbar('Yaw   deg', WIN,
                           _sp(np.degrees(calib['yaw_offset']),   R_HALF, r_scale), R_HALF*2, lambda x:None)
        cv2.createTrackbar('Fwd   in ', WIN,
                           _sp(p0[0]/INCHES, T_HALF, t_scale), T_HALF*2, lambda x:None)
        cv2.createTrackbar('Left  in ', WIN,
                           _sp(p0[1]/INCHES, T_HALF, t_scale), T_HALF*2, lambda x:None)
        cv2.createTrackbar('Up    in ', WIN,
                           _sp(p0[2]/INCHES, T_HALF, t_scale), T_HALF*2, lambda x:None)

    _make_sliders(R_SCALE_C, T_SCALE_C)

    saved = False
    last_vals = [None]

    print()
    print('Adjust sliders until the coloured dots align with visible features.')
    print('  Dots coloured by depth: blue=far, red=close.')
    print('  Well-calibrated: wall edges, corners, fixtures have dots on them.')
    print('  F = toggle fine mode (0.1 deg / 0.01 in steps)')
    print('  S = save    Q = quit')
    print()

    while True:
        r_scale = R_SCALE_F if fine[0] else R_SCALE_C
        t_scale = T_SCALE_F if fine[0] else T_SCALE_C

        roll  = _sv(cv2.getTrackbarPos('Roll  deg', WIN), R_HALF, r_scale)
        pitch = _sv(cv2.getTrackbarPos('Pitch deg', WIN), R_HALF, r_scale)
        yaw   = _sv(cv2.getTrackbarPos('Yaw   deg', WIN), R_HALF, r_scale)
        fwd   = _sv(cv2.getTrackbarPos('Fwd   in ', WIN), T_HALF, t_scale)
        left  = _sv(cv2.getTrackbarPos('Left  in ', WIN), T_HALF, t_scale)
        up    = _sv(cv2.getTrackbarPos('Up    in ', WIN), T_HALF, t_scale)

        Rm = R.from_euler('xyz', np.radians([roll, pitch, yaw])).as_matrix()
        pos = np.array([fwd, left, up]) * INCHES
        t   = Rm @ (-pos)

        u, v, depths = _project(pts, Rm, t, K_disp, DISP_W, disp_h)
        frame = _overlay(img_disp, u, v, depths)

        # HUD
        mode_str = 'FINE (0.1deg/0.01in)' if fine[0] else 'COARSE (1deg/0.1in)'
        _put(frame, f'Roll={roll:+.1f}  Pitch={pitch:+.1f}  Yaw={yaw:+.1f} deg  [{mode_str}]',
             (10, 28), 0.6, (220, 220, 50))
        _put(frame, f'Fwd={fwd:+.2f}"  Left={left:+.2f}"  Up={up:+.2f}"  '
             f'Pts in frame: {len(u)}',
             (10, 58), 0.6, (220, 220, 50))
        _put(frame, 'S=Save  Q=Quit  F=Fine/Coarse  |  dots: blue=far  red=close',
             (10, disp_h - 12), 0.5, (180, 180, 180))

        cv2.imshow(WIN, frame)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('f'):
            # Toggle fine/coarse — read current values and re-create sliders
            fine[0] = not fine[0]
            new_r = R_SCALE_F if fine[0] else R_SCALE_C
            new_t = T_SCALE_F if fine[0] else T_SCALE_C
            # Temporarily store current values
            cur = dict(roll=roll, pitch=pitch, yaw=yaw, fwd=fwd, left=left, up=up)
            _make_sliders(new_r, new_t)
            # Restore values in new scale
            cv2.setTrackbarPos('Roll  deg', WIN, _sp(cur['roll'],  R_HALF, new_r))
            cv2.setTrackbarPos('Pitch deg', WIN, _sp(cur['pitch'], R_HALF, new_r))
            cv2.setTrackbarPos('Yaw   deg', WIN, _sp(cur['yaw'],   R_HALF, new_r))
            cv2.setTrackbarPos('Fwd   in ', WIN, _sp(cur['fwd'],   T_HALF, new_t))
            cv2.setTrackbarPos('Left  in ', WIN, _sp(cur['left'],  T_HALF, new_t))
            cv2.setTrackbarPos('Up    in ', WIN, _sp(cur['up'],    T_HALF, new_t))
            print(f'  Switched to {"FINE" if fine[0] else "COARSE"} mode')
        elif key == ord('s'):
            c = {'roll_offset':  float(np.radians(roll)),
                 'pitch_offset': float(np.radians(pitch)),
                 'yaw_offset':   float(np.radians(yaw)),
                 'manual_roll_adjustment':  0.0,
                 'manual_pitch_adjustment': 0.0,
                 'manual_yaw_adjustment':   0.0,
                 'azimuth_offset':   0.0,
                 'elevation_offset': 0.0,
                 'x_offset': float(t[0]),
                 'y_offset': float(t[1]),
                 'z_offset': float(t[2]),
                 'flip_x': False, 'flip_y': False,
                 'image_width':  int(img_full.shape[1]),
                 'image_height': int(img_full.shape[0]),
                 'use_fisheye': False, 'skip_rate': 5}
            _save_calib(c, slot_path, camera_hw)
            print(f'\n✓ Saved: {slot_path}')
            print(f'  Roll={roll:.2f}°  Pitch={pitch:.2f}°  Yaw={yaw:.2f}°')
            print(f'  Fwd={fwd:.3f}"  Left={left:.3f}"  Up={up:.3f}"')
            saved = True
            break

    cv2.destroyAllWindows()
    if not saved:
        print('Quit without saving.')


if __name__ == '__main__':
    main()
