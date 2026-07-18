#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Generates visual overlays of LiDAR intensity projected onto the
# camera ERP image using the current calibration seed. Produces three views:
#   - seed_edges.jpg: red=camera edges, green=lidar edges, yellow=overlap
#   - seed_checker.jpg: checkerboard blend of camera and lidar
#   - seed_split.jpg: left=camera, right=lidar with center divider
#
# Usage:
#   python3 verify_seed_overlay.py [output_dir] [--open]

import argparse
import cv2
import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_SRC = Path(__file__).resolve().parent.parent


def load_calibration(camera_hw):
    """Load T_camera_lidar from the per-hw calibration."""
    calib_path = _SRC / 'config' / 'calibrations' / camera_hw / 'fusion_calibration.yaml'
    if not calib_path.exists():
        calib_path = _SRC / 'config' / 'fusion_calibration.yaml'
    with open(calib_path) as f:
        cfg = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [
        cfg['roll_offset'], cfg['pitch_offset'], cfg['yaw_offset']
    ]).as_matrix()
    T[:3, 3] = [cfg['x_offset'], cfg['y_offset'], cfg['z_offset']]
    return T, cfg.get('image_width', 5760), cfg.get('image_height', 2880)


def project_ply_to_erp(ply_path, T_cam_lidar, width, height):
    """Project a PLY point cloud into an ERP intensity image using T_camera_lidar."""
    from plyfile import PlyData
    ply = PlyData.read(str(ply_path))
    v = ply['vertex']
    points = np.column_stack([v['x'], v['y'], v['z']])

    # Transform to camera frame
    pts_cam = (T_cam_lidar[:3, :3] @ points.T).T + T_cam_lidar[:3, 3]

    # Filter
    norm = np.linalg.norm(pts_cam, axis=1)
    valid = norm > 0.3
    bearing = pts_cam[valid] / norm[valid, None]

    # Equirectangular projection (matches equirectangular.hpp)
    lat = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
    lon = np.arctan2(bearing[:, 0], bearing[:, 2])
    u = (width * (0.5 + lon / (2 * np.pi))).astype(int) % width
    v_px = np.clip((height * (0.5 - lat / np.pi)).astype(int), 0, height - 1)

    # Intensity
    if 'intensity' in v.data.dtype.names:
        intensity = np.array(v['intensity'], dtype=np.float32)[valid]
        intensity = np.clip(intensity / max(intensity.max(), 1) * 255, 0, 255).astype(np.uint8)
    else:
        intensity = np.full(valid.sum(), 200, dtype=np.uint8)

    img = np.zeros((height, width), dtype=np.uint8)
    img[v_px, u] = intensity
    return img


def _get_calibration_mask(camera_hw, cam_w, cam_h):
    """Load the calibration mask for a camera from multi_camera.yaml.
    Falls back to the camera model's lidar_mask_dual if not set."""
    mc_path = _SRC / 'config' / 'multi_camera.yaml'
    mask_name = ''
    if mc_path.exists():
        import yaml as _y
        mc = _y.safe_load(mc_path.read_text()) or {}
        for cam in mc.get('cameras', {}).values():
            if cam.get('camera_hw', '') == camera_hw:
                mask_name = cam.get('mask_calibration', cam.get('mask_dual', ''))
                break
    if not mask_name:
        hw_yaml = _SRC / 'config' / 'camera_models' / f'{camera_hw}.yaml'
        if hw_yaml.exists():
            import yaml as _y
            _cfg = _y.safe_load(hw_yaml.read_text()) or {}
            mask_name = _cfg.get('lidar_mask_dual', '')
    if not mask_name:
        return None
    mask_path = _SRC / 'config' / 'masks' / mask_name
    if not mask_path.exists():
        return None
    mask_img = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask_img is None:
        return None
    return cv2.resize(mask_img, (cam_w, cam_h), interpolation=cv2.INTER_NEAREST)


def main():
    parser = argparse.ArgumentParser(description='Verify seed calibration with visual overlay')
    parser.add_argument('session_dir', nargs='?', default=str(Path.home() / 'atlas_ws/output'),
                        help='Session directory or calibration output directory')
    parser.add_argument('--open', action='store_true', help='Open the edge image after generating')
    args = parser.parse_args()

    output = Path(args.session_dir)

    # Detect camera_hw from session_config.json
    import json, glob
    camera_hw = 'x5'
    sess_cfg = output / 'session_config.json'
    if sess_cfg.exists():
        camera_hw = json.loads(sess_cfg.read_text()).get('camera_hw', 'x5')
    else:
        # Fallback: check source sidecars (calibration output dir)
        source_jsons = sorted(glob.glob(str(output / '*_source.json')))
        if source_jsons:
            src = json.loads(Path(source_jsons[0]).read_text())
            sc = Path(src.get('scan_dir', '')) / '..' / 'session_config.json'
            if sc.exists():
                camera_hw = json.loads(sc.read_text()).get('camera_hw', 'x5')

    print(f"Camera HW: {camera_hw}")

    # Load calibration — prefer the per-slot path from multi_camera.yaml
    # (e.g. calibrations/x3/left/fusion_calibration.yaml for cam_1) so the
    # overlay reflects the same transform used during calibration, not the
    # hw-level file which may be stale or belong to a different camera slot.
    import os as _os
    _src_root = Path(__file__).resolve().parent.parent
    _calib_override = _os.environ.get('ATLAS_CALIBRATION_FILE', '')
    if _calib_override and Path(_calib_override).exists():
        # Patch load_calibration to use this path
        _T, _w, _h = load_calibration(camera_hw)  # loads hw-level as base
        with open(_calib_override) as _f:
            _cfg_slot = yaml.safe_load(_f)
        _T = np.eye(4)
        _T[:3, :3] = R.from_euler('xyz', [
            _cfg_slot['roll_offset'], _cfg_slot['pitch_offset'], _cfg_slot['yaw_offset']
        ]).as_matrix()
        _T[:3, 3] = [_cfg_slot['x_offset'], _cfg_slot['y_offset'], _cfg_slot['z_offset']]
        T_cam_lidar, img_w, img_h = _T, _cfg_slot.get('image_width', 5760), _cfg_slot.get('image_height', 2880)
        print(f'Using slot calibration: {_calib_override}')
    else:
        # Fallback: check multi_camera.yaml for a slot path matching camera_hw
        _cam_idx = _os.environ.get('ATLAS_CALIBRATION_CAM_INDEX', '')
        if _cam_idx:
            _mc_path = _src_root / 'config' / 'multi_camera.yaml'
            if _mc_path.exists():
                _mc = yaml.safe_load(_mc_path.read_text()) or {}
                _calib_rel = _mc.get('cameras', {}).get(f'cam_{_cam_idx}', {}).get('calibration', '')
                if _calib_rel:
                    _slot_path = _src_root / 'config' / _calib_rel
                    if _slot_path.exists():
                        with open(_slot_path) as _f:
                            _cfg_slot = yaml.safe_load(_f)
                        T_cam_lidar = np.eye(4)
                        T_cam_lidar[:3, :3] = R.from_euler('xyz', [
                            _cfg_slot['roll_offset'], _cfg_slot['pitch_offset'], _cfg_slot['yaw_offset']
                        ]).as_matrix()
                        T_cam_lidar[:3, 3] = [_cfg_slot['x_offset'], _cfg_slot['y_offset'], _cfg_slot['z_offset']]
                        img_w = _cfg_slot.get('image_width', 5760)
                        img_h = _cfg_slot.get('image_height', 2880)
                        print(f'Using slot calibration (cam_{_cam_idx}): {_slot_path}')
                    else:
                        T_cam_lidar, img_w, img_h = load_calibration(camera_hw)
                else:
                    T_cam_lidar, img_w, img_h = load_calibration(camera_hw)
            else:
                T_cam_lidar, img_w, img_h = load_calibration(camera_hw)
        else:
            T_cam_lidar, img_w, img_h = load_calibration(camera_hw)
    euler = R.from_matrix(T_cam_lidar[:3, :3]).as_euler('xyz', degrees=True)
    print(f"Seed rotation (deg): roll={euler[0]:.1f} pitch={euler[1]:.1f} yaw={euler[2]:.1f}")
    print(f"Seed translation: [{T_cam_lidar[0,3]:.4f}, {T_cam_lidar[1,3]:.4f}, {T_cam_lidar[2,3]:.4f}]")

    # Find files - support both session dir and calibration output dir
    ply_files = []
    cam_files = []

    # Check if this is a session directory (has fusion_scan_* subdirs)
    scan_dirs = sorted(d for d in output.iterdir()
                      if d.is_dir() and d.name.startswith('fusion_scan_'))
    if scan_dirs:
        first_scan = scan_dirs[0]
        ply_files = sorted(first_scan.glob('sensor_lidar*.ply'))
        cam_files = sorted(first_scan.glob('equirect_*_masked.png'))
        if not cam_files:
            cam_files = sorted(first_scan.glob('equirect_dual_fisheye.jpg'))
    else:
        # Calibration output directory
        ply_files = sorted(output.glob('*.ply'))
        cam_files = sorted(output.glob('images/000000.png'))
        if not cam_files:
            cam_files = sorted(output.glob('000000.png'))

    if not ply_files or not cam_files:
        print(f"Missing files: PLY={len(ply_files)}, cam={len(cam_files)}")
        return

    cam_img = cv2.imread(str(cam_files[0]))
    cam_h, cam_w = cam_img.shape[:2]

    # Apply calibration mask (from multi_camera.yaml mask_calibration field)
    mask_img = _get_calibration_mask(camera_hw, cam_w, cam_h)
    if mask_img is not None:
        cam_img[mask_img < 128] = 0
        print(f"Applied calibration mask for {camera_hw}")
    else:
        print(f"No calibration mask found for {camera_hw}")

    print(f"Projecting {ply_files[0].name} ({cam_w}x{cam_h})...")
    lid_img = project_ply_to_erp(ply_files[0], T_cam_lidar, cam_w, cam_h)

    # Mask LiDAR where camera image is black (scanner body / masked regions)
    cam_gray = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)
    cam_mask = cam_gray > 30
    # Erode mask to remove edges at mask boundaries
    cam_mask = cv2.erode(cam_mask.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    lid_img[~cam_mask] = 0
    lid_mask = lid_img > 10

    # 1. Edge alignment: red=camera, green=lidar, yellow=overlap
    cam_gray = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)
    cam_edges = cv2.Canny(cam_gray, 50, 150)
    cam_edges[~cam_mask] = 0
    lid_edges = cv2.Canny(lid_img, 20, 80)
    edge_viz = np.zeros_like(cam_img)
    edge_viz[cam_edges > 0] = [0, 0, 255]
    edge_viz[lid_edges > 0] = [0, 255, 0]
    edge_viz[(cam_edges > 0) & (lid_edges > 0)] = [0, 255, 255]
    cv2.imwrite(str(output / 'seed_edges.jpg'), edge_viz, [cv2.IMWRITE_JPEG_QUALITY, 90])

    # 2. Checkerboard blend
    block = max(cam_w // 16, 1)
    lid_color = cv2.applyColorMap(lid_img, cv2.COLORMAP_INFERNO)
    checker = cam_img.copy()
    for i in range(0, cam_h, block):
        for j in range(0, cam_w, block):
            if ((i // block) + (j // block)) % 2 == 0:
                roi = lid_mask[i:i+block, j:j+block]
                if roi.any():
                    checker[i:i+block, j:j+block][roi] = lid_color[i:i+block, j:j+block][roi]
    cv2.imwrite(str(output / 'seed_checker.jpg'), checker, [cv2.IMWRITE_JPEG_QUALITY, 90])

    # 3. Split view
    mid = cam_w // 2
    lid_rgb = cv2.applyColorMap(lid_img, cv2.COLORMAP_INFERNO)
    split = cam_img.copy()
    split[:, mid:] = np.where(lid_mask[:, mid:, None], lid_rgb[:, mid:], split[:, mid:])
    cv2.line(split, (mid, 0), (mid, cam_h), (0, 255, 255), 2)
    cv2.imwrite(str(output / 'seed_split.jpg'), split, [cv2.IMWRITE_JPEG_QUALITY, 90])

    print(f"\n\u2713 Verification images saved:")
    print(f"  {output / 'seed_edges.jpg'}   - red=camera, green=lidar, yellow=aligned")
    print(f"  {output / 'seed_checker.jpg'} - checkerboard (features continue across blocks = good)")
    print(f"  {output / 'seed_split.jpg'}   - left=camera, right=lidar (edges align at divider)")
    print(f"\n  Shifted horizontally \u2192 adjust Yaw")
    print(f"  Shifted vertically \u2192 adjust Up")
    print(f"  No lidar coverage in some areas \u2192 normal (LiDAR FOV is limited)")

    if args.open:
        import subprocess
        subprocess.Popen(['xdg-open', str(output / 'seed_edges.jpg')])


if __name__ == '__main__':
    main()
