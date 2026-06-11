#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Orchestrates per-scan point cloud coloring: converts the captured fisheye image to ERP and then calls exact_match_fusion to project and color the sensor-frame PLY.
"""
Color point clouds using fisheye images.
Converts dual-fisheye to ERP first, then uses the calibrated ERP coloring pipeline.
"""

import sys
import os
import glob
from pathlib import Path
from fisheye_to_erp import fisheye_jpg_to_erp
from exact_match_fusion import exact_match_calibration_tool

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def color_scan_fisheye(scan_dir, config_path=None, mask_path=None):
    print(f"\n{'='*60}")
    print(f"Coloring scan: {scan_dir}")
    print(f"{'='*60}")

    try:
        safe_scan = _safe_data(scan_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return False

    fisheye_files = [
        f for f in glob.glob(os.path.join(str(safe_scan), 'fisheye_*.jpg'))
        if _ALLOWED_DATA in [Path(f).resolve(), *Path(f).resolve().parents]
    ]
    if not fisheye_files:
        print(f"\u2717 No fisheye images found in {safe_scan}")
        return False

    fisheye_img_path = fisheye_files[0]
    print(f"\u2713 Fisheye image: {os.path.basename(fisheye_img_path)}")

    # Convert single fisheye to ERP using single-lens config (zeroed alignment)
    erp_config = os.path.expanduser(
        '~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml')
    timestamp = os.path.basename(fisheye_img_path).replace('fisheye_', '').replace('.jpg', '')
    erp_path = str(_safe_data(safe_scan / f'equirect_{timestamp}.jpg'))

    if not os.path.exists(erp_path):
        print("\u2713 Converting fisheye \u2192 ERP...")
        fisheye_jpg_to_erp(fisheye_img_path, erp_config, erp_path, dual=False)
    else:
        print(f"\u2713 ERP already exists: {os.path.basename(erp_path)}")

    # Apply lidar mask to produce the masked PNG that exact_match_fusion prefers
    masked_path = str(_safe_data(Path(erp_path).with_suffix('').parent /
                                  (Path(erp_path).stem + '_masked.png')))
    if not os.path.exists(masked_path):
        print("\u2713 Applying LiDAR mask...")
        from apply_lidar_mask import apply_mask
        mask_file = os.path.expanduser('~/atlas_ws/src/atlas-scanner/src/lidar_mask_single.png')
        apply_mask(erp_path, mask_file, masked_path)

    # Use the calibrated ERP coloring pipeline
    print("\u2713 Coloring point cloud using ERP + calibrated extrinsics...")
    return exact_match_calibration_tool(str(safe_scan))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 color_with_fisheye.py <scan_dir> [config_path] [mask_path]")
        print("\nExample:")
        print("  python3 color_with_fisheye.py ~/atlas_ws/data/synchronized_scans/sync_fusion_20250101_120000/fusion_scan_001")
        sys.exit(1)
    
    scan_dir = sys.argv[1]
    config_path = sys.argv[2] if len(sys.argv) > 2 else None
    mask_path = sys.argv[3] if len(sys.argv) > 3 else None

    try:
        _safe_data(scan_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    # Auto-detect config if not provided
    if config_path is None:
        default_config = os.path.expanduser('~/atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml')
        if os.path.exists(default_config):
            config_path = default_config
    
    # Auto-detect mask if not provided
    if mask_path is None:
        default_mask = os.path.expanduser('~/atlas_ws/src/atlas-scanner/src/lidar_mask_single.png')
        if os.path.exists(default_mask):
            mask_path = default_mask
    
    success = color_scan_fisheye(scan_dir, config_path, mask_path)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
