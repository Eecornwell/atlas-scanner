#!/usr/bin/env python3
"""
Color point clouds using fisheye images.
Converts dual-fisheye to ERP first, then uses the calibrated ERP coloring pipeline.
"""

import sys
import os
import glob
from fisheye_to_erp import fisheye_jpg_to_erp
from exact_match_fusion import exact_match_calibration_tool


def color_scan_fisheye(scan_dir, config_path=None, mask_path=None):
    print(f"\n{'='*60}")
    print(f"Coloring scan: {scan_dir}")
    print(f"{'='*60}")

    fisheye_files = glob.glob(os.path.join(scan_dir, 'fisheye_*.jpg'))
    if not fisheye_files:
        print(f"✗ No fisheye images found in {scan_dir}")
        return False

    fisheye_img_path = fisheye_files[0]
    print(f"✓ Fisheye image: {os.path.basename(fisheye_img_path)}")

    # Convert dual-fisheye to ERP using equirectangular calibration
    erp_config = os.path.expanduser(
        '~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml')
    timestamp = os.path.basename(fisheye_img_path).replace('fisheye_', '').replace('.jpg', '')
    erp_path = os.path.join(scan_dir, f'equirect_{timestamp}.jpg')

    if not os.path.exists(erp_path):
        print("✓ Converting fisheye → ERP...")
        fisheye_jpg_to_erp(fisheye_img_path, erp_config, erp_path)
    else:
        print(f"✓ ERP already exists: {os.path.basename(erp_path)}")

    # Use the calibrated ERP coloring pipeline
    print("✓ Coloring point cloud using ERP + calibrated extrinsics...")
    return exact_match_calibration_tool(scan_dir)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 color_with_fisheye.py <scan_dir> [config_path] [mask_path]")
        print("\nExample:")
        print("  python3 color_with_fisheye.py ~/atlas_ws/data/synchronized_scans/sync_fusion_20250101_120000/fusion_scan_001")
        sys.exit(1)
    
    scan_dir = sys.argv[1]
    config_path = sys.argv[2] if len(sys.argv) > 2 else None
    mask_path = sys.argv[3] if len(sys.argv) > 3 else None
    
    # Auto-detect config if not provided
    if config_path is None:
        default_config = os.path.expanduser('~/atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml')
        if os.path.exists(default_config):
            config_path = default_config
    
    # Auto-detect mask if not provided
    if mask_path is None:
        default_mask = os.path.expanduser('~/atlas_ws/src/atlas-scanner/src/lidar_mask.png')
        if os.path.exists(default_mask):
            mask_path = default_mask
    
    success = color_scan_fisheye(scan_dir, config_path, mask_path)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
