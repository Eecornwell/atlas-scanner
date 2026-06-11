#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Merges world_lidar.ply files from all scans using only trajectory
# poses (no ICP). Useful for diagnosing alignment issues before ICP post-processing.

import sys
import json
import os
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def load_ply_points(path):
    """Load XYZ and optional RGB from a PLY file. Returns (N,3) or (N,6) array."""
    pts = []
    safe = _safe_data(path)
    has_rgb = False
    with open(safe, 'rb') as f:
        while True:
            line = f.readline().decode('ascii', errors='replace')
            if 'end_header' in line:
                break
            if 'element vertex' in line:
                n = int(line.split()[-1])
            if 'property uchar red' in line:
                has_rgb = True
        for _ in range(n):
            line = f.readline().decode('ascii', errors='replace').split()
            try:
                if has_rgb and len(line) >= 6:
                    pts.append([float(line[0]), float(line[1]), float(line[2]),
                                float(line[3]) / 255.0, float(line[4]) / 255.0, float(line[5]) / 255.0])
                elif len(line) >= 3:
                    pts.append([float(line[0]), float(line[1]), float(line[2])])
            except ValueError:
                pass
    cols = 6 if has_rgb else 3
    return np.array(pts) if pts else np.zeros((0, cols))


def save_ply(path, points):
    """Save XYZ or XYZ+RGB point array to PLY. RGB expected as floats 0-1."""
    safe = _safe_data(path)
    has_rgb = points.shape[1] == 6
    with open(safe, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(points)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        if has_rgb:
            f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write('end_header\n')
        for p in points:
            if has_rgb:
                r, g, b = int(p[3] * 255), int(p[4] * 255), int(p[5] * 255)
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {r} {g} {b}\n')
            else:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


def merge_trajectory_only(session_dir):
    try:
        session = _safe_data(session_dir)
    except ValueError as e:
        print(f'Error: {e}')
        return
    scan_dirs = sorted(session.glob('fusion_scan_*'))

    all_points = []
    T_first_inv = None

    for scan_dir in scan_dirs:
        colored_ply = scan_dir / 'world_colored_exact.ply'
        world_ply = scan_dir / 'world_lidar.ply'
        traj_file = scan_dir / 'trajectory.json'
        if not traj_file.exists():
            print(f'  Skipping {scan_dir.name}: missing trajectory.json')
            continue
        use_colored = colored_ply.exists()
        src_ply = colored_ply if use_colored else world_ply
        if not src_ply.exists():
            print(f'  Skipping {scan_dir.name}: missing world_lidar.ply or world_colored_exact.ply')
            continue
        try:
            src_ply = _safe_data(src_ply)
            traj_file = _safe_data(traj_file)
        except ValueError:
            print(f'  Skipping {scan_dir.name}: path outside allowed root')
            continue
        with open(traj_file) as f:
            traj = json.load(f)
        lp = traj['current_pose']['lidar_pose']
        pos = lp['position']
        ori = lp['orientation']
        q = np.array([ori['x'], ori['y'], ori['z'], ori['w']])
        q /= np.linalg.norm(q)
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[:3, 3] = [pos['x'], pos['y'], pos['z']]

        if T_first_inv is None:
            T_first_inv = np.linalg.inv(T)

        T_rel = T_first_inv @ T
        euler = Rotation.from_matrix(T_rel[:3, :3]).as_euler('xyz', degrees=True)
        print(f'  {scan_dir.name}: pos=({T_rel[0,3]:.3f},{T_rel[1,3]:.3f},{T_rel[2,3]:.3f})'
              f'  euler=({euler[0]:.1f},{euler[1]:.1f},{euler[2]:.1f})deg'
              + (' [colored]' if use_colored else ''))

        pts = load_ply_points(src_ply)
        if len(pts) == 0:
            continue
        # Apply T_first_inv to XYZ only; carry RGB columns unchanged
        xyz = pts[:, :3]
        xyz_h = np.hstack([xyz, np.ones((len(xyz), 1))])
        xyz_rel = (T_first_inv @ xyz_h.T).T[:, :3]
        if pts.shape[1] == 6:
            all_points.append(np.hstack([xyz_rel, pts[:, 3:]]))
        else:
            all_points.append(xyz_rel)

    if not all_points:
        print('No points to merge')
        return

    merged = np.vstack(all_points)
    out = _safe_data(session / 'merged_trajectory_only.ply')
    save_ply(out, merged)
    print(f'\n✓ Saved {len(merged)} points to {out}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: python3 merge_trajectory_only.py <session_dir>')
        sys.exit(1)
    try:
        _safe_data(sys.argv[1])
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)
    merge_trajectory_only(sys.argv[1])
