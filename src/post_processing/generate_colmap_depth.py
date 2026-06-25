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

def _render_depth(pts_world, R_w2c, t_w2c, f_px, cx, cy, w, h, radius=2):
    """
    Project pts_world (N,3) into the camera defined by w2c pose and
    SIMPLE_PINHOLE intrinsics. Each point is splatted as a filled disc
    of `radius` pixels (z-buffered). Returns uint16 depth map in mm.
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

    depth_f = np.full((h, w), np.inf, dtype=np.float64)

    if radius <= 1:
        # Single-pixel, closest wins
        order = np.argsort(z)[::-1]
        depth_f[vi[order], ui[order]] = z[order]
    else:
        # Splat each point as a filled disc, z-buffered (closest wins)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                vj = np.clip(vi + dy, 0, h - 1)
                uj = np.clip(ui + dx, 0, w - 1)
                # Only write where this point is closer than what's there
                closer = z < depth_f[vj, uj]
                depth_f[vj[closer], uj[closer]] = z[closer]

    depth_mm = np.clip(depth_f * 1000.0, 0, 65535)
    depth_mm[depth_f == np.inf] = 0
    return depth_mm.astype(np.uint16)


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

def generate_depth_images(session_dir, sparse_subdir='colmap/sparse/0', radius=2):
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

    # Output directory mirrors the images/ layout
    depth_root = _safe_data(session / 'colmap' / 'depth_images')
    depth_root.mkdir(exist_ok=True)

    # Group images by panorama index to load each scan's PLY once
    scan_dirs = {int(d.name.split('_')[-1]): d
                 for d in sorted(session.glob('fusion_scan_*')) if d.is_dir()}

    # Build pano_idx -> list of image dicts
    by_pano = {}
    for img in images.values():
        pi = _pano_index(img['name'])
        if pi >= 0:
            by_pano.setdefault(pi, []).append(img)

    n_written = 0
    n_panos   = len(by_pano)

    for pi, img_list in sorted(by_pano.items()):
        scan_dir = scan_dirs.get(pi)
        if scan_dir is None:
            print(f'  ⚠ No scan dir for pano_{pi:03d}, skipping')
            continue

        # Use the full unfiltered scan — sensor_lidar.ply has ~200k pts
        # vs sensor_colored_exact.ply which is pre-filtered to ~23k.
        ply_path = scan_dir / 'sensor_lidar.ply'
        if not ply_path.exists():
            ply_path = scan_dir / 'sensor_colored_exact.ply'
        if not ply_path.exists():
            print(f'  ⚠ No PLY for {scan_dir.name}, skipping')
            continue

        pts_sensor = _read_ply_points(str(ply_path))
        if len(pts_sensor) == 0:
            print(f'  ⚠ Empty PLY for {scan_dir.name}, skipping')
            continue

        # pts_sensor are in sensor (lidar) frame.
        # Each tile's w2c pose is in COLMAP world frame where:
        #   p_colmap = R_ROS2COLMAP @ (R_lidar_world @ p_sensor + t_lidar_world)
        #   p_colmap = R_ROS2COLMAP @ p_ros_world
        #
        # To get pts into COLMAP world we need the pano's lidar world pose.
        # Read it from trajectory.json (the same pose used to build the init model).
        traj_file = scan_dir / 'trajectory.json'
        if not traj_file.exists():
            print(f'  ⚠ No trajectory.json for {scan_dir.name}, skipping')
            continue
        with open(traj_file) as f:
            traj = json.load(f)
        lp  = traj['current_pose']['lidar_pose']
        pos = np.array([lp['position']['x'], lp['position']['y'], lp['position']['z']])
        q   = np.array([lp['orientation']['x'], lp['orientation']['y'],
                        lp['orientation']['z'], lp['orientation']['w']])
        R_lidar_world = Rotation.from_quat(q).as_matrix()

        # Sensor frame -> ROS world -> COLMAP world
        pts_ros    = (R_lidar_world @ pts_sensor.T).T + pos
        pts_colmap = (R_ROS2COLMAP @ pts_ros.T).T        # (N, 3) in COLMAP world

        print(f'  pano_{pi:03d}: {len(pts_colmap)} pts  ({scan_dir.name})')

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

            depth_img = _render_depth(pts_colmap, R_w2c, t_w2c, f_px, cx, cy, w, h,
                                       radius=radius)

            # Mirror directory structure: face_XX/pano_NNN.png
            rel   = img['name']                          # e.g. face_00/pano_001.png
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
    parser.add_argument('--radius', type=int, default=2,
                        help='Point splat radius in pixels (default: 2)')
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)
    generate_depth_images(args.session_dir, args.sparse, radius=args.radius)
