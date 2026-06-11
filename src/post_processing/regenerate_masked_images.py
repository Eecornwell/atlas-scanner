#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Re-applies the LiDAR mask to all equirectangular images in a session directory. Useful after re-running seam blending or updating the mask file.
import sys
import os
from pathlib import Path

# Reuse apply_mask logic
sys.path.insert(0, str(Path(__file__).parent))
from apply_lidar_mask import apply_mask

MASK_BASE = Path(os.path.expanduser('~/atlas_ws/src/atlas-scanner/src'))
_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()
_ALLOWED_SRC  = Path(os.path.expanduser('~/atlas_ws/src')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def regenerate_masked_images(session_dir, camera_mode="dual_fisheye", sdk_stitch=False):
    """Regenerate masked images from blended ERP images"""
    try:
        session_path = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return
    if sdk_stitch and camera_mode == "dual_fisheye":
        suffix = "dual_sdk"
    elif camera_mode == "dual_fisheye":
        suffix = "dual"
    else:
        suffix = "single"
    mask_file = str((MASK_BASE / f"lidar_mask_{suffix}.png").resolve())
    
    count = 0
    for scan_dir in sorted(session_path.glob("fusion_scan_*")):
        if scan_dir.is_dir():
            # Find blended ERP images
            for img_file in scan_dir.glob("equirect_*.jpg"):
                if '_bak' not in img_file.name:
                    try:
                        safe_img = _safe_data(img_file)
                        safe_out = _safe_data(img_file.with_name(img_file.stem + "_masked.png"))
                    except ValueError:
                        continue
                    if apply_mask(str(safe_img), mask_file, str(safe_out)):
                        count += 1
    
    print(f"✓ Regenerated {count} masked images from blended ERPs")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir")
    parser.add_argument("--camera-mode", default="dual_fisheye", choices=["dual_fisheye", "single_fisheye"])
    parser.add_argument("--sdk-stitch", action="store_true")
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    regenerate_masked_images(args.session_dir, args.camera_mode, args.sdk_stitch)
