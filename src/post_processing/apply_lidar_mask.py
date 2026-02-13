#!/usr/bin/env python3
import cv2
import sys
import os
from pathlib import Path

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
    if len(sys.argv) == 2:
        # Apply to all images in scan directory
        scan_dir = Path(sys.argv[1])
        mask_path = "/home/orion/atlas_ws/src/atlas-scanner/src/lidar_mask.png"
        
        for img_file in scan_dir.glob("equirect*.jpg"):
            output = img_file.with_name(img_file.stem + "_masked.png")
            apply_mask(str(img_file), mask_path, str(output))
    elif len(sys.argv) == 4:
        # Apply to single image
        apply_mask(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print("Usage: python3 apply_lidar_mask.py <scan_dir>")
        print("   or: python3 apply_lidar_mask.py <image> <mask> <output>")
