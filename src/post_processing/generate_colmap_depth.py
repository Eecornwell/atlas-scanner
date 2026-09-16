#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Generates uint16 depth images (mm) for every perspective tile in
# the COLMAP model. Uses the non-downsampled sensor_colored_exact.ply from each
# scan, transformed to each tile's exact camera frame via the BA-refined pose
# from images.bin and the T_camera_lidar calibration.
#
# Output: colmap/depth_images/face_XX/pano_NNN.png  (uint16, mm, same size as tiles)
#
# Usage:
#   python3 generate_colmap_depth.py <session_dir>

import struct
import json
import os
import sys
import argparse
import numpy as np
import cv2
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()
_ALLOWED_SRC  = Path(os.path.expanduser('~/atlas_ws/src')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def _safe_src(p) -> Path:
    resolved = Path(p).resolve()
    for root in (_ALLOWED_SRC, _ALLOWED_DATA):
        if root in [resolved, *resolved.parents]:
            return resolved
    raise ValueError(f"Path '{resolved}' is outside allowed roots")


# ---------------------------------------------------------------------------
# Binary readers
# ---------------------------------------------------------------------------

def _read_cameras(path):
    cameras = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            cam_id = struct.unpack('I', f.read(4))[0]
            model  = struct.unpack('i', f.read(4))[0]
            w, h   = struct.unpack('QQ', f.read(16))
            nparams = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8, 5: 8, 6: 8, 7: 0}.get(model, 0)
            params = list(struct.unpack(f'{nparams}d', f.read(8 * nparams)))
            cameras[cam_id] = {'w': int(w), 'h': int(h), 'params': params}
    return cameras


def _read_images(path):
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            img_id          = struct.unpack('I', f.read(4))[0]
            qw, qx, qy, qz = struct.unpack('dddd', f.read(32))
            tx, ty, tz      = struct.unpack('ddd',  f.read(24))
            cam_id          = struct.unpack('I',    f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00':
                    break
                name += c
            n_pts = struct.unpack('Q', f.read(8))[0]
            f.read(n_pts * 24)
            images[img_id] = {
                'name':      name.decode(),
                'qvec':      np.array([qw, qx, qy, qz]),
                'tvec':      np.array([tx, ty, tz]),
                'camera_id': cam_id,
            }
    return images


# ---------------------------------------------------------------------------
# PLY reader (sensor_colored_exact.ply — ASCII)
# ---------------------------------------------------------------------------

def _read_ply_points(path):
    pts = []
    with open(path, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            header += line
            if line.strip() == b'end_header':
                break
        hdr = header.decode('ascii', errors='replace')
        binary = 'binary_little_endian' in hdr
        n_verts = int(next(l.split()[-1] for l in hdr.splitlines()
                           if l.startswith('element vertex')))
        fields = [l.split()[-1] for l in hdr.splitlines()
                  if l.startswith('property float')]
        n_fields = len(fields)
        if binary:
            data = np.frombuffer(f.read(n_verts * n_fields * 4),
                                 dtype=np.float32).reshape(n_verts, n_fields)
            return data[:, :3].astype(np.float64)
        else:
            for line in f.read().decode('ascii', errors='replace').splitlines():
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except ValueError:
                        continue
    return np.array(pts, dtype=np.float64)


# ---------------------------------------------------------------------------
# Depth rendering
# ---------------------------------------------------------------------------

def _joint_bilateral_fill(depth_u16, rgb_guide, window=15,
                          sigma_space=7.0, sigma_color=20.0):
    """
    Fill holes in depth_u16 guided by the RGB image.
    Two-pass strategy:
      Pass 1 (coarse): discontinuity-aware nearest-neighbour fill.
                       Each hole pixel inherits the nearest valid depth only
                       if the RGB colour difference is below a hard threshold,
                       preventing background depth from bleeding across edges.
      Pass 2 (fine):   bilateral fill restricted to same-layer neighbours
                       (|depth_neighbour - depth_hole| < 20% of hole depth)
                       to snap residual holes to the correct depth layer.
    """
    from scipy.ndimage import distance_transform_edt

    depth = depth_u16.astype(np.float32)
    valid_mask = depth > 0

    if not valid_mask.all():
        # ── Pass 1: colour-gated nearest-neighbour propagation ──────────────
        # Find the spatially nearest valid pixel for every hole pixel.
        _, nearest_idx = distance_transform_edt(~valid_mask, return_indices=True)
        depth_nn = depth[nearest_idx[0], nearest_idx[1]]

        if rgb_guide is not None:
            # Gate propagation on RGB similarity: only accept the nearest-neighbour
            # depth if the colour difference is small (same surface), otherwise
            # leave the hole for the bilateral pass to handle.
            gray = cv2.cvtColor(rgb_guide, cv2.COLOR_BGR2GRAY).astype(np.float32)
            hole_y, hole_x = np.where(~valid_mask)
            nn_y, nn_x = nearest_idx[0][~valid_mask], nearest_idx[1][~valid_mask]
            color_diff = np.abs(gray[hole_y, hole_x] - gray[nn_y, nn_x])
            # Accept propagation only where colour is similar (same surface)
            accept = color_diff < 15.0
            depth[hole_y[accept], hole_x[accept]] = depth_nn[~valid_mask][accept]
        else:
            depth = np.where(valid_mask, depth, depth_nn)

    # ── Pass 2: depth-layer-aware bilateral fill ─────────────────────────────
    # Re-compute hole mask after pass 1 (some holes may still be unfilled).
    # For each remaining hole, accumulate weighted depth from neighbours that
    # are in the same depth layer (within 20% relative depth), preventing
    # foreground/background mixing at silhouette edges.
    still_holes = depth == 0
    if still_holes.any() and rgb_guide is not None:
        gray = cv2.cvtColor(rgb_guide, cv2.COLOR_BGR2GRAY).astype(np.float32)
        half = window // 2
        spatial_w = np.exp(
            -(np.mgrid[-half:half+1, -half:half+1][0] ** 2 +
              np.mgrid[-half:half+1, -half:half+1][1] ** 2)
            / (2 * sigma_space ** 2)
        ).astype(np.float32)

        depth_pad = np.pad(depth, half, mode='edge')
        gray_pad  = np.pad(gray,  half, mode='edge')
        vy, vx = np.where(still_holes)
        acc_d = np.zeros(len(vy), dtype=np.float32)
        acc_w = np.zeros(len(vy), dtype=np.float32)
        gc = gray_pad[vy + half, vx + half]
        for dy in range(-half, half + 1):
            for dx in range(-half, half + 1):
                ny, nx = vy + dy + half, vx + dx + half
                d_nb = depth_pad[ny, nx]
                g_nb = gray_pad[ny, nx]
                # Depth-layer gate: reject neighbours from a different depth layer.
                # Use a relative threshold so the gate scales with scene depth.
                depth_ref = np.maximum(acc_d / np.maximum(acc_w, 1e-6), 100.0)
                same_layer = (d_nb == 0) | (np.abs(d_nb - depth_ref) < 0.20 * depth_ref)
                color_w = np.exp(-(g_nb - gc) ** 2 / (2 * sigma_color ** 2))
                w_ij = spatial_w[dy + half, dx + half] * color_w * same_layer
                acc_d += w_ij * d_nb
                acc_w += w_ij
        filled = acc_w > 1e-6
        depth[vy[filled], vx[filled]] = acc_d[filled] / acc_w[filled]

    # Any pixels still zero after both passes: fall back to unconditional NN
    still_zero = depth == 0
    if still_zero.any():
        _, nearest_idx = distance_transform_edt(~(depth > 0), return_indices=True)
        depth[still_zero] = depth[nearest_idx[0][still_zero], nearest_idx[1][still_zero]]

    return depth.astype(np.uint16)


# Angular gap between Livox Mid-360 scan lines ≈ 0.33°.
# At depth z (metres), this gap projects to f_px * gap_rad / z pixels.
# We use this to set a per-point splat radius that exactly covers the gap,
# preventing holes without over-blurring distant surfaces.
_LIDAR_ANGULAR_GAP_RAD = np.radians(0.5)   # 0.5° — conservative (Mid-360 ~0.33°)
_SPLAT_RADIUS_MIN = 1                        # never smaller than 1px
_SPLAT_RADIUS_MAX = 12                       # cap to avoid blurring thin structures


def _render_depth(pts_world, R_w2c, t_w2c, f_px, cx, cy, w, h, radius=3,
                  rgb_guide=None):
    """
    Project pts_world (N,3) into the camera, splat each point as a
    depth-adaptive disc whose radius scales with f_px/z so the splat
    exactly covers the angular gap between LiDAR scan lines at any range.
    Returns uint16 depth map in mm.
    """
    pts_cam = (R_w2c @ pts_world.T).T + t_w2c

    valid = (pts_cam[:, 2] > 0.1) & (pts_cam[:, 2] < 100.0)
    pts_cam = pts_cam[valid]
    if len(pts_cam) == 0:
        return np.zeros((h, w), dtype=np.uint16)

    z  = pts_cam[:, 2]
    ui = np.round(pts_cam[:, 0] / z * f_px + cx).astype(np.int32)
    vi = np.round(pts_cam[:, 1] / z * f_px + cy).astype(np.int32)

    in_bounds = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    ui, vi, z = ui[in_bounds], vi[in_bounds], z[in_bounds]

    # Depth-adaptive splat radius: r = clamp(f_px * gap_rad / z, min, max)
    # Splat far-to-near so close points overwrite background depth correctly.
    splat_r = np.clip(
        np.round(f_px * _LIDAR_ANGULAR_GAP_RAD / z).astype(np.int32),
        _SPLAT_RADIUS_MIN, _SPLAT_RADIUS_MAX
    )

    depth_f = np.full((h, w), np.inf, dtype=np.float32)
    order = np.argsort(z)[::-1]   # far → near
    ui, vi, z, splat_r = ui[order], vi[order], z[order].astype(np.float32), splat_r[order]

    # Group by radius to batch the disc-splatting loop (avoids per-point Python overhead).
    # Discontinuity guard: a disc pixel is only written if the centre point is
    # closer than the current pixel OR the current pixel is still unset (inf).
    # This prevents background splat halos bleeding over foreground silhouettes —
    # the far-to-near order ensures foreground points always win at their centre,
    # and the guard stops background discs from overwriting already-set closer pixels.
    for r in np.unique(splat_r):
        mask = splat_r == r
        ui_r, vi_r, z_r = ui[mask], vi[mask], z[mask]
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                vj = np.clip(vi_r + dy, 0, h - 1)
                uj = np.clip(ui_r + dx, 0, w - 1)
                closer = z_r < depth_f[vj, uj]
                depth_f[vj[closer], uj[closer]] = z_r[closer]

    depth_mm = np.clip(depth_f * 1000.0, 0, 65535)
    depth_mm[depth_f == np.inf] = 0
    depth_u16 = depth_mm.astype(np.uint16)

    depth_u16 = _joint_bilateral_fill(depth_u16, rgb_guide)
    return depth_u16


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def _load_T_camera_lidar(session_path=None):
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from camera_hw import camera_hw_for_session, calibration_path
    hw         = camera_hw_for_session(session_path) if session_path else 'onex2'
    calib_file = calibration_path(hw)
    with open(calib_file) as f:
        calib = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler(
        'xyz', [calib['roll_offset'], calib['pitch_offset'], calib['yaw_offset']]
    ).as_matrix()
    T[:3, 3] = [calib['x_offset'], calib['y_offset'], calib['z_offset']]
    return T


# ---------------------------------------------------------------------------
# ROS2COLMAP (same as panorama_sfm_colmap.py)
# ---------------------------------------------------------------------------

R_ROS2COLMAP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)


def _pano_index(name):
    try:
        return int(name.split('pano_')[1].split('.')[0])
    except Exception:
        return -1


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def generate_depth_images(session_dir, sparse_subdir='colmap/sparse/0', radius=3):
    try:
        session = _safe_data(session_dir)
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)

    sparse_dir = session / sparse_subdir
    if not sparse_dir.exists():
        print(f'No sparse model at {sparse_dir}')
        sys.exit(1)

    cameras = _read_cameras(sparse_dir / 'cameras.bin')
    images  = _read_images(sparse_dir  / 'images.bin')

    T_camera_lidar = _load_T_camera_lidar(session)
    T_lidar_camera = np.linalg.inv(T_camera_lidar)

    depth_root = _safe_data(session / 'colmap' / 'depth_images')
    depth_root.mkdir(exist_ok=True)

    # Use the merged filtered point cloud from colmap/sparse/merged.ply.
    # This is the full session cloud — filtered, colored, spatially coherent —
    # giving dense uniform depth coverage vs the raw per-scan sensor_lidar.ply
    # which has scan-line gaps and single-return noise.
    merged_ply = session / 'colmap' / 'sparse' / 'merged.ply'
    use_merged = merged_ply.exists()
    if use_merged:
        print(f'  Using merged cloud: {merged_ply.name}')
        import open3d as _o3d
        _pcd = _o3d.io.read_point_cloud(str(merged_ply))
        merged_pts_colmap = np.asarray(_pcd.points).astype(np.float64)
        print(f'  Merged cloud: {len(merged_pts_colmap)} points')
    else:
        print('  ⚠ No merged.ply found, falling back to per-scan sensor_lidar.ply')
        merged_pts_colmap = None

    scan_dirs = {int(d.name.split('_')[-1]): d
                 for d in sorted(session.glob('fusion_scan_*'))
                 if d.is_dir()
                 and not (d / '.blur_skip').exists()
                 and not (d / '.corrupt_bag').exists()}

    by_pano = {}
    for img in images.values():
        pi = _pano_index(img['name'])
        if pi >= 0:
            by_pano.setdefault(pi, []).append(img)

    n_written = 0

    for pi, img_list in sorted(by_pano.items()):
        if use_merged:
            pts_colmap = merged_pts_colmap
        else:
            scan_dir = scan_dirs.get(pi)
            if scan_dir is None:
                print(f'  ⚠ No scan dir for pano_{pi:03d}, skipping')
                continue
            ply_path = scan_dir / 'sensor_lidar.ply'
            if not ply_path.exists():
                ply_path = scan_dir / 'sensor_colored_exact.ply'
            if not ply_path.exists():
                print(f'  ⚠ No PLY for {scan_dir.name}, skipping')
                continue
            pts_sensor = _read_ply_points(str(ply_path))
            if len(pts_sensor) == 0:
                continue
            scan_dir = scan_dirs.get(pi)
            traj_file = scan_dir / 'trajectory.json'
            if not traj_file.exists():
                continue
            with open(traj_file) as f:
                traj = json.load(f)
            lp  = traj['current_pose']['lidar_pose']
            pos = np.array([lp['position']['x'], lp['position']['y'], lp['position']['z']])
            q   = np.array([lp['orientation']['x'], lp['orientation']['y'],
                            lp['orientation']['z'], lp['orientation']['w']])
            R_lidar_world = Rotation.from_quat(q).as_matrix()
            pts_ros    = (R_lidar_world @ pts_sensor.T).T + pos
            pts_colmap = (R_ROS2COLMAP @ pts_ros.T).T

        if use_merged:
            print(f'  pano_{pi:03d}: {len(pts_colmap)} pts (merged)')
        for img in img_list:
            cam   = cameras[img['camera_id']]
            w, h  = cam['w'], cam['h']
            f_px  = cam['params'][0]
            cx    = cam['params'][1]
            cy    = cam['params'][2]

            # w2c from images.bin (BA-refined)
            qvec = img['qvec']   # [qw, qx, qy, qz]
            R_w2c = Rotation.from_quat(
                [qvec[1], qvec[2], qvec[3], qvec[0]]).as_matrix()
            t_w2c = img['tvec']

            # Load the RGB tile before rendering so it can guide hole filling
            rel           = img['name']                  # e.g. face_00/pano_001.png
            rgb_tile_path = session / 'colmap' / 'images' / rel
            rgb_guide     = cv2.imread(str(rgb_tile_path)) if rgb_tile_path.exists() else None

            depth_img = _render_depth(pts_colmap, R_w2c, t_w2c, f_px, cx, cy, w, h,
                                      radius=radius, rgb_guide=rgb_guide)
            out   = depth_root / rel
            out.parent.mkdir(parents=True, exist_ok=True)
            try:
                _safe_data(out)
            except ValueError:
                continue
            cv2.imwrite(str(out), depth_img)
            n_written += 1

        print(f'    wrote {len(img_list)} depth tiles')

    print(f'\n✓ {n_written} depth images -> {depth_root}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--sparse', default='colmap/sparse/0')
    parser.add_argument('--radius', type=int, default=3,
                        help='Point splat radius in pixels (default: 3)')
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)
    generate_depth_images(args.session_dir, args.sparse, radius=args.radius)
