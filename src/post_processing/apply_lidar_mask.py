#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Applies a binary mask PNG to an equirectangular image, zeroing out regions (e.g. the scanner body and tripod) that should be excluded from point cloud coloring.
import cv2
import sys
import os
from pathlib import Path

MASK_BASE = "/home/orion/atlas_ws/src/atlas-scanner/src"

def apply_mask(image_path, mask_path, output_path):
    import cv2
    import numpy as np
    
    img = cv2.imread(image_path)
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    
    if img is None or mask is None:
        print(f"Failed to load image or mask")
        return False
    
    # Resize mask to match image if needed
    if mask.shape != img.shape[:2]:
        mask = cv2.resize(mask, (img.shape[1], img.shape[0]))
    
    # Create RGBA image
    b, g, r = cv2.split(img)
    rgba = cv2.merge([b, g, r, mask])
    
    cv2.imwrite(output_path, rgba)
    return True

if __name__ == "__main__":
    import argparse
    if len(sys.argv) == 4:
        apply_mask(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        parser = argparse.ArgumentParser()
        parser.add_argument("scan_dir")
        parser.add_argument("--camera-mode", default="dual_fisheye", choices=["dual_fisheye", "single_fisheye"])
        args = parser.parse_args()
        suffix = "dual" if args.camera_mode == "dual_fisheye" else "single"
        mask_path = f"{MASK_BASE}/lidar_mask_{suffix}.png"
        for img_file in Path(args.scan_dir).glob("equirect*.jpg"):
            output = img_file.with_name(img_file.stem + "_masked.png")
            apply_mask(str(img_file), mask_path, str(output))
