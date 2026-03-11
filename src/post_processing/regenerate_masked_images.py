#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Re-applies the LiDAR mask to all equirectangular images in a session directory. Useful after re-running seam blending or updating the mask file.
import sys
from pathlib import Path

# Reuse apply_mask logic
sys.path.insert(0, str(Path(__file__).parent))
from apply_lidar_mask import apply_mask

MASK_BASE = "/home/orion/atlas_ws/src/atlas-scanner/src"

def regenerate_masked_images(session_dir, camera_mode="dual_fisheye"):
    """Regenerate masked images from blended ERP images"""
    session_path = Path(session_dir)
    suffix = "dual" if camera_mode == "dual_fisheye" else "single"
    mask_file = f"{MASK_BASE}/lidar_mask_{suffix}.png"
    
    count = 0
    for scan_dir in sorted(session_path.glob("fusion_scan_*")):
        if scan_dir.is_dir():
            # Find blended ERP images
            for img_file in scan_dir.glob("equirect_*.jpg"):
                if '_bak' not in img_file.name:
                    output = img_file.with_name(img_file.stem + "_masked.png")
                    if apply_mask(str(img_file), mask_file, str(output)):
                        count += 1
    
    print(f"✓ Regenerated {count} masked images from blended ERPs")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir")
    parser.add_argument("--camera-mode", default="dual_fisheye", choices=["dual_fisheye", "single_fisheye"])
    args = parser.parse_args()
    regenerate_masked_images(args.session_dir, args.camera_mode)
