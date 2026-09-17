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

def _guided_depth_completion(sparse_depth_u16, rgb_guide):
    """
    Convert a sparse splatted depth map into a dense continuous depth surface.

    Uses PromptDA (depth-anything/promptda_vitl) when available — a model
    purpose-built for LiDAR-anchored dense depth completion that produces
    smooth planar surfaces with sharp RGB-aligned edges.

    Falls back to iterative guided image filtering (cv2.ximgproc) when
    PromptDA is not installed, which is faster but produces blockier output.
    """
    if rgb_guide is None:
        return sparse_depth_u16

    # ── Try PromptDA first ────────────────────────────────────────────────────────────────
    try:
        import torch
        from promptda.promptda import PromptDA
        from PIL import Image as _PILImage

        device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # Cache model across calls to avoid reloading per tile
        if not hasattr(_guided_depth_completion, '_promptda_model'):
            print('  Loading PromptDA model (first tile)...')
            _guided_depth_completion._promptda_model = (
                PromptDA.from_pretrained('depth-anything/promptda_vitl')
                .to(device).eval()
            )
        model = _guided_depth_completion._promptda_model

        # PromptDA expects:
        #   image: PIL RGB image
        #   prompt_depth: (1,1,H,W) float32 tensor in metres
        rgb_pil = _PILImage.fromarray(cv2.cvtColor(rgb_guide, cv2.COLOR_BGR2RGB))
        sparse_m = sparse_depth_u16.astype(np.float32) / 1000.0  # mm -> metres
        prompt = torch.from_numpy(sparse_m).unsqueeze(0).unsqueeze(0).to(device)

        with torch.no_grad():
            depth_pred = model.predict(image=rgb_pil, prompt_depth=prompt)

        depth_m = depth_pred.squeeze().cpu().numpy()  # (H, W) float32, metres
        return np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)

    except (ImportError, Exception):
        pass  # PromptDA not installed or failed — fall through to guided filter

    # ── Guided filter fallback ────────────────────────────────────────────────────────────────
    depth = sparse_depth_u16.astype(np.float32)
    valid = (depth > 0).astype(np.float32)
    guide = cv2.cvtColor(rgb_guide, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0

    try:
        for radius, eps in [(64, 0.01), (32, 0.005), (16, 0.002), (8, 0.001)]:
            d_filt = cv2.ximgproc.guidedFilter(
                guide=guide, src=depth * valid,
                radius=radius, eps=eps * (depth.max() ** 2 + 1e-6)
            )
            c_filt = cv2.ximgproc.guidedFilter(
                guide=guide, src=valid, radius=radius, eps=eps
            )
            propagated = np.where(c_filt > 0.01, d_filt / np.maximum(c_filt, 0.01), 0.0)
            depth = np.where(valid > 0.5, depth, propagated)
            valid = (depth > 0).astype(np.float32)
        if depth.max() > 0:
            d_norm = np.clip(depth / depth.max() * 255, 0, 255).astype(np.uint8)
            d_sharp = cv2.ximgproc.jointBilateralFilter(
                joint=rgb_guide, src=d_norm, d=9, sigmaColor=25, sigmaSpace=5)
            depth = d_sharp.astype(np.float32) / 255.0 * depth.max()
    except (cv2.error, AttributeError):
        # ximgproc not available — fall back to TELEA inpainting
        hole_mask = (sparse_depth_u16 == 0).astype(np.uint8)
        if hole_mask.any():
            d_max = depth.max()
            if d_max > 0:
                d_norm = np.clip(depth / d_max * 255, 0, 255).astype(np.uint8)
                d_inp  = cv2.inpaint(d_norm, hole_mask, inpaintRadius=5,
                                     flags=cv2.INPAINT_TELEA)
                depth  = np.where(sparse_depth_u16 > 0, depth,
                                  d_inp.astype(np.float32) / 255.0 * d_max)

    # Final NN fill for any remaining zeros
    still_zero = depth == 0
    if still_zero.any():
        from scipy.ndimage import distance_transform_edt
        _, idx = distance_transform_edt(still_zero, return_indices=True)
        depth[still_zero] = depth[idx[0][still_zero], idx[1][still_zero]]

    return np.clip(depth, 0, 65535).astype(np.uint16)


def _joint_bilateral_fill(depth_u16, rgb_guide, window=15,
                          sigma_space=7.0, sigma_color=20.0):
    """
    Fill holes in depth_u16 guided by the RGB image.
    Uses OpenCV TELEA inpainting on the hole mask to propagate depth along
    RGB isophotes (follows object edges) rather than Euclidean nearest-neighbour
    (which produces Voronoi staircase boundaries). A bilateral pass then
    corrects any remaining depth-layer mixing at silhouette edges.
    """
    from scipy.ndimage import distance_transform_edt

    depth = depth_u16.astype(np.float32)
    valid_mask = depth > 0

    if not valid_mask.all():
        hole_mask = (~valid_mask).astype(np.uint8)

        if rgb_guide is not None:
            # Normalise depth to uint8 range for inpainting, then scale back.
            # TELEA inpainting fills holes by propagating values inward along
            # RGB isophotes, so boundaries follow the colour edges in the guide
            # image rather than forming Voronoi staircases.
            d_max = depth.max()
            if d_max > 0:
                d_norm = np.clip(depth / d_max * 255, 0, 255).astype(np.uint8)
                d_inpainted = cv2.inpaint(d_norm, hole_mask, inpaintRadius=5,
                                          flags=cv2.INPAINT_TELEA)
                depth_filled = d_inpainted.astype(np.float32) / 255.0 * d_max
                depth = np.where(valid_mask, depth, depth_filled)
            else:
                depth = np.zeros_like(depth)
        else:
            _, nearest_idx = distance_transform_edt(~valid_mask, return_indices=True)
            depth = np.where(valid_mask, depth,
                             depth[nearest_idx[0], nearest_idx[1]])

    # Bilateral pass: correct depth-layer mixing at silhouette edges.
    # Only runs on pixels that were originally holes and are adjacent to a
    # depth discontinuity in the filled result.
    still_holes = depth_u16 == 0
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
                depth_ref = np.maximum(acc_d / np.maximum(acc_w, 1e-6), 100.0)
                same_layer = (d_nb == 0) | (np.abs(d_nb - depth_ref) < 0.20 * depth_ref)
                color_w = np.exp(-(g_nb - gc) ** 2 / (2 * sigma_color ** 2))
                w_ij = spatial_w[dy + half, dx + half] * color_w * same_layer
                acc_d += w_ij * d_nb
                acc_w += w_ij
        filled = acc_w > 1e-6
        depth[vy[filled], vx[filled]] = acc_d[filled] / acc_w[filled]

    # Final fallback: any remaining zeros get unconditional NN fill
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
    Project pts_world (N,3) into the camera using sub-pixel splatting.
    Each point is projected to its exact floating-point (u,v) coordinate
    and distributed across its 2x2 pixel neighbourhood with bilinear weights,
    eliminating the staircase jagging caused by hard integer rounding.
    A depth-adaptive Gaussian disc then fills the scan-line gap around each
    sub-pixel centre. Returns uint16 depth map in mm.
    """
    pts_cam = (R_w2c @ pts_world.T).T + t_w2c

    valid = (pts_cam[:, 2] > 0.1) & (pts_cam[:, 2] < 100.0)
    pts_cam = pts_cam[valid]
    if len(pts_cam) == 0:
        return np.zeros((h, w), dtype=np.uint16)

    z  = pts_cam[:, 2]
    # Exact sub-pixel projection — do NOT round yet
    uf = pts_cam[:, 0] / z * f_px + cx
    vf = pts_cam[:, 1] / z * f_px + cy

    in_bounds = (uf >= 0) & (uf < w) & (vf >= 0) & (vf < h)
    uf, vf, z = uf[in_bounds], vf[in_bounds], z[in_bounds]

    splat_r = np.clip(
        np.round(f_px * _LIDAR_ANGULAR_GAP_RAD / z).astype(np.int32),
        _SPLAT_RADIUS_MIN, _SPLAT_RADIUS_MAX
    )

    # Float accumulators: weighted depth sum and total weight per pixel.
    # Bilinear sub-pixel weights spread each point across its 2x2 neighbourhood
    # so depth boundaries land at the true projection position rather than a
    # rounded integer, eliminating staircase edges at object silhouettes.
    depth_acc  = np.zeros((h, w), dtype=np.float64)
    weight_acc = np.zeros((h, w), dtype=np.float64)

    order = np.argsort(z)[::-1]   # far -> near
    uf, vf, z, splat_r = uf[order], vf[order], z[order], splat_r[order]

    u0 = np.floor(uf).astype(np.int32)
    v0 = np.floor(vf).astype(np.int32)
    du = (uf - u0).astype(np.float32)
    dv = (vf - v0).astype(np.float32)

    for r in np.unique(splat_r):
        mask = splat_r == r
        u0_r, v0_r, z_r = u0[mask], v0[mask], z[mask]
        bw00 = ((1 - du) * (1 - dv))[mask]
        bw10 = (     du  * (1 - dv))[mask]
        bw01 = ((1 - du) *      dv )[mask]
        bw11 = (     du  *      dv )[mask]
        bilinear_r = [(0, 0, bw00), (1, 0, bw10), (0, 1, bw01), (1, 1, bw11)]
        # Sigma covers the full splat radius so the Gaussian decays to ~1%
        # at the edge. No hard disc cutoff — the weight tapers smoothly to
        # zero, eliminating the jagged circle boundary entirely.
        sigma = max(r / 2.0, 0.5)
        extent = r + 1  # one extra pixel so the tail blends into neighbours

        for dy in range(-extent, extent + 1):
            for dx in range(-extent, extent + 1):
                # Pure Gaussian — no hard disc boundary check
                gauss_w = float(np.exp(-(dx * dx + dy * dy) / (2 * sigma * sigma)))
                if gauss_w < 0.01:   # skip negligible contributions
                    continue

                for bdu, bdv, bw in bilinear_r:
                    vj = np.clip(v0_r + dy + bdv, 0, h - 1)
                    uj = np.clip(u0_r + dx + bdu, 0, w - 1)
                    tw = gauss_w * bw

                    existing = np.where(
                        weight_acc[vj, uj] > 0,
                        depth_acc[vj, uj] / weight_acc[vj, uj],
                        np.inf
                    )
                    write = z_r < existing
                    depth_acc[vj[write], uj[write]]  = tw[write] * z_r[write]
                    weight_acc[vj[write], uj[write]] = tw[write]

    valid_w  = weight_acc > 0
    depth_f  = np.where(valid_w, depth_acc / np.maximum(weight_acc, 1e-9), 0.0)
    depth_mm = np.clip(depth_f * 1000.0, 0, 65535).astype(np.uint16)

    depth_mm = _guided_depth_completion(depth_mm, rgb_guide)
    return depth_mm


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
