#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Merges world_lidar.ply files from all scans using only trajectory
# poses (no ICP). Useful for diagnosing alignment issues before ICP post-processing.

import sys
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def load_ply_points(path):
    pts = []
    with open(path, 'rb') as f:
        while True:
            line = f.readline().decode('ascii', errors='replace')
            if 'end_header' in line:
                break
            if 'element vertex' in line:
                n = int(line.split()[-1])
        for _ in range(n):
            line = f.readline().decode('ascii', errors='replace').split()
            if len(line) >= 3:
                try:
                    pts.append([float(line[0]), float(line[1]), float(line[2])])
                except ValueError:
                    pass
    return np.array(pts) if pts else np.zeros((0, 3))


def save_ply(path, points):
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(points)}\n')
        f.write('property float x\nproperty float y\nproperty float z\nend_header\n')
        for p in points:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


def merge_trajectory_only(session_dir):
    session = Path(session_dir)
    scan_dirs = sorted(session.glob('fusion_scan_*'))

    all_points = []
    T_first_inv = None

    for scan_dir in scan_dirs:
        world_ply = scan_dir / 'world_lidar.ply'
        traj_file = scan_dir / 'trajectory.json'
        if not world_ply.exists() or not traj_file.exists():
            print(f'  Skipping {scan_dir.name}: missing world_lidar.ply or trajectory.json')
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
              f'  euler=({euler[0]:.1f},{euler[1]:.1f},{euler[2]:.1f})deg')

        # world_lidar.ply is already in world frame — just apply T_first_inv
        pts = load_ply_points(str(world_ply))
        if len(pts) == 0:
            continue
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts_rel = (T_first_inv @ pts_h.T).T[:, :3]
        all_points.append(pts_rel)

    if not all_points:
        print('No points to merge')
        return

    merged = np.vstack(all_points)
    out = session / 'merged_trajectory_only.ply'
    save_ply(str(out), merged)
    print(f'\n✓ Saved {len(merged)} points to {out}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: python3 merge_trajectory_only.py <session_dir>')
        sys.exit(1)
    merge_trajectory_only(sys.argv[1])
