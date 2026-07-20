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


def load_calibration(camera_hw, cam_index=None):
    """Load T_camera_lidar via calibration_path() so it always resolves
    to the same slot file that exact_match_fusion.py uses."""
    import sys as _sys
    _sys.path.insert(0, str(_SRC))
    from camera_hw import calibration_path as _cp
    calib_path = _cp(camera_hw, cam_index)
    with open(calib_path) as f:
        cfg = yaml.safe_load(f)
    print(f"  Calibration: {calib_path}")
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
    parser.add_argument('--cam-index', type=int, default=None,
                        help='Camera slot index (0/1/2). Ensures the correct slot calibration is loaded.')
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

    T_cam_lidar, img_w, img_h = load_calibration(camera_hw, args.cam_index)
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

    # Dilate LiDAR for visibility before edge detection
    lid_dilated = cv2.dilate(lid_img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    lid_dilated[~cam_mask] = 0

    # --- 1. Edge alignment: camera=red, LiDAR=green, overlap=yellow ---
    # Shown on a dimmed camera background so context is visible.
    cam_gray = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)
    cam_edges = cv2.Canny(cam_gray, 40, 120)
    cam_edges[~cam_mask] = 0
    lid_edges = cv2.Canny(lid_dilated, 15, 60)
    edge_viz = (cam_img.astype(np.float32) * 0.3).astype(np.uint8)
    edge_viz[cam_edges > 0] = [0, 0, 220]       # red  = camera edges
    edge_viz[lid_edges > 0] = [0, 220, 0]        # green = LiDAR edges
    edge_viz[(cam_edges > 0) & (lid_edges > 0)] = [0, 220, 220]  # yellow = aligned
    cv2.imwrite(str(output / 'seed_edges.jpg'), edge_viz, [cv2.IMWRITE_JPEG_QUALITY, 92])

    # --- 2. Overlay: camera + TURBO-colored LiDAR dots (near=white, far=blue) ---
    # Thick dots (radius 5) on full-brightness camera so both are easy to read.
    lid_color = cv2.applyColorMap(lid_dilated, cv2.COLORMAP_TURBO)
    overlay = cam_img.copy()
    overlay[lid_mask] = cv2.addWeighted(cam_img, 0.15, lid_color, 0.85, 0)[lid_mask]
    # Draw a thin white halo around each LiDAR point for contrast on dark surfaces
    halo = cv2.dilate(lid_mask.astype(np.uint8), np.ones((7,7), np.uint8)) -            cv2.erode(lid_mask.astype(np.uint8), np.ones((3,3), np.uint8))
    overlay[halo > 0] = [255, 255, 255]
    cv2.imwrite(str(output / 'seed_overlay.jpg'), overlay, [cv2.IMWRITE_JPEG_QUALITY, 92])

    # --- 3. Composite: edge view (left) | overlay (right) in one image ---
    # Single file to open — shows both alignment quality and context side by side.
    divider = np.full((cam_h, 6, 3), (0, 255, 255), dtype=np.uint8)
    # Scale both halves to same width for clean side-by-side
    half_w = cam_w // 2
    edge_half    = cv2.resize(edge_viz, (half_w, cam_h))
    overlay_half = cv2.resize(overlay,  (half_w, cam_h))
    div_thin = np.full((cam_h, 4, 3), (0, 255, 255), dtype=np.uint8)
    composite = np.hstack([edge_half, div_thin, overlay_half])
    # Legend
    legend_y = cam_h - 18
    cv2.putText(composite, "RED=camera  GREEN=lidar  YELLOW=aligned",
                (10, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,220,220), 1, cv2.LINE_AA)
    cv2.putText(composite, "TURBO dots: near=white far=blue",
                (half_w + 14, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1, cv2.LINE_AA)
    cv2.imwrite(str(output / 'seed_composite.jpg'), composite, [cv2.IMWRITE_JPEG_QUALITY, 92])

    # Keep split view for backward compat
    mid = cam_w // 2
    split = cam_img.copy()
    split[:, mid:] = np.where(lid_mask[:, mid:, None], lid_color[:, mid:], split[:, mid:])
    cv2.line(split, (mid, 0), (mid, cam_h), (0, 255, 255), 2)
    cv2.imwrite(str(output / 'seed_split.jpg'), split, [cv2.IMWRITE_JPEG_QUALITY, 92])

    print(f"\n\u2713 Verification images saved:")
    print(f"  {output / 'seed_composite.jpg'} \u2190 OPEN THIS: edge alignment + overlay side by side")
    print(f"  {output / 'seed_edges.jpg'}     - red=camera, green=lidar, yellow=aligned (on dim bg)")
    print(f"  {output / 'seed_overlay.jpg'}   - TURBO lidar dots on full camera")
    print(f"  {output / 'seed_split.jpg'}     - left=camera, right=lidar")
    print(f"\n  Shifted horizontally \u2192 adjust Yaw")
    print(f"  Shifted vertically   \u2192 adjust Up/Pitch")
    print(f"  Rotated              \u2192 adjust Roll")
    print(f"  No lidar coverage in some areas \u2192 normal (LiDAR FOV is limited)")

    if args.open:
        import subprocess
        subprocess.Popen(['xdg-open', str(output / 'seed_edges.jpg')])


if __name__ == '__main__':
    main()
