#!/usr/bin/env python3
import cv2
import numpy as np
import sys
from pathlib import Path

def blend_erp_seams(image_path, output_path=None, blend_width=5):
    """Blend fisheye seams in equirectangular image"""
    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Failed to load {image_path}")
        return False
    
    # Save backup
    backup_path = str(image_path).replace('.jpg', '_raw.jpg')
    cv2.imwrite(backup_path, img)
    
    height, width = img.shape[:2]
    seam_positions = [481, 1440]  # Exact seam positions for 1920px image
    
    for seam_x in seam_positions:
        # Blend across the seam with multi-column averaging
        for x in range(max(0, seam_x - blend_width), min(width, seam_x + blend_width)):
            # Distance from seam (-1 to +1, negative=left, positive=right)
            offset = x - seam_x
            # Weight: 0=use left side, 1=use right side
            weight = (offset + blend_width) / (2 * blend_width)
            weight = np.clip(weight, 0, 1)
            
            # Sample multiple columns and average to reduce streaking
            x_left = seam_x - blend_width - 3
            x_right = seam_x + blend_width + 3
            
            if 0 <= x_left < width and 0 <= x_right < width:
                # Average 5 columns on each side
                left_cols = img[:, max(0, x_left-2):x_left+3].mean(axis=1).astype(float)
                right_cols = img[:, x_right-2:min(width, x_right+3)].mean(axis=1).astype(float)
                img[:, x] = (left_cols * (1 - weight) + right_cols * weight).astype('uint8')
    
    # Overwrite original file
    cv2.imwrite(str(image_path), img)
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 blend_erp_seams.py <image_or_directory>")
        sys.exit(1)
    
    path = Path(sys.argv[1])
    
    if path.is_file():
        blend_erp_seams(path)
        print(f"✓ Blended: {path}")
    elif path.is_dir():
        count = 0
        for scan_dir in sorted(path.glob("fusion_scan_*")):
            if scan_dir.is_dir():
                for img_file in scan_dir.glob("equirect_*.jpg"):
                    if blend_erp_seams(img_file):
                        count += 1
        print(f"✓ Blended {count} images")
    else:
        print(f"Path not found: {path}")
