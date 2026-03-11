#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Applies statistical outlier removal and voxel downsampling to a merged colored PLY to reduce noise and file size before export.
"""
Clean a merged colored PLY point cloud.

Steps:
  1. Statistical outlier removal  — removes isolated noise points
  2. Voxel downsampling           — merges duplicate/overlapping points

Usage:
  python3 clean_pointcloud.py <input.ply> [--voxel 0.01] [--nb-neighbors 20] [--std-ratio 2.0]
"""

import sys
import argparse
import numpy as np
from pathlib import Path
import open3d as o3d


def clean(input_path, voxel_size=0.01, nb_neighbors=20, std_ratio=2.0):
    pcd = o3d.io.read_point_cloud(str(input_path))
    n_before = len(pcd.points)
    print(f"  Input:  {n_before} points")

    if nb_neighbors > 0:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        print(f"  After outlier removal: {len(pcd.points)} points  "
              f"(removed {n_before - len(pcd.points)})")

    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f"  After voxel downsample ({voxel_size}m): {len(pcd.points)} points")

    out_path = Path(input_path).with_stem(Path(input_path).stem + "_clean")
    o3d.io.write_point_cloud(str(out_path), pcd)
    print(f"  ✓ Saved: {out_path.name}")
    return str(out_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_ply")
    parser.add_argument("--voxel", type=float, default=0.01,
                        help="Voxel size in metres (0 = skip, default 0.01)")
    parser.add_argument("--nb-neighbors", type=int, default=20,
                        help="Neighbours for outlier removal (default 20)")
    parser.add_argument("--std-ratio", type=float, default=2.0,
                        help="Std-dev threshold for outlier removal (default 2.0)")
    args = parser.parse_args()
    clean(args.input_ply, args.voxel, args.nb_neighbors, args.std_ratio)
