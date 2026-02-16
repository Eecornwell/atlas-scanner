#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import sys
from pathlib import Path

def blend_seam(img, seam_pos, seam_width=10):
    """Blend seam in-place at given position, single smooth transition"""
    rows, width = img.shape[:2]
    half_width = seam_width // 2
    
    # Extract regions on both sides of seam
    left_region = img[:, seam_pos-half_width:seam_pos].copy()
    right_region = img[:, seam_pos:seam_pos+half_width].copy()
    
    # Blend across the entire seam width with single gradient
    for i in range(seam_width):
        alpha = i / (seam_width - 1) if seam_width > 1 else 0.5
        
        left_idx = i if i < half_width else half_width - 1
        right_idx = i - half_width if i >= half_width else 0
        
        pos = seam_pos - half_width + i
        if 0 <= pos < width:
            img[:, pos] = left_region[:, left_idx] * (1 - alpha) + right_region[:, right_idx] * alpha
    
    return img

def blend_erp_seams(image_path, output_path=None):
    """Blend ERP image seams at 1/4 and 3/4 positions using simple weighted blending"""
    img = cv.imread(str(image_path), cv.IMREAD_UNCHANGED)
    if img is None:
        print(f"Failed to load {image_path}")
        return False
    
    has_alpha = img.shape[2] == 4 if len(img.shape) == 3 else False
    
    if has_alpha:
        alpha = img[:, :, 3]
        img_rgb = img[:, :, :3].astype(float)
    else:
        img_rgb = img.astype(float)
    
    backup_path = str(image_path).replace('.jpg', '_raw.jpg').replace('.png', '_raw.png')
    if not Path(backup_path).exists():
        if has_alpha:
            cv.imwrite(backup_path, np.dstack([np.clip(img_rgb, 0, 255).astype(np.uint8), alpha]))
        else:
            cv.imwrite(backup_path, np.clip(img_rgb, 0, 255).astype(np.uint8))
    
    height, width = img_rgb.shape[:2]
    quarter = width // 4
    three_quarter = 3 * width // 4
    
    # Blend at seam positions in-place
    img_rgb = blend_seam(img_rgb, quarter)
    img_rgb = blend_seam(img_rgb, three_quarter)
    
    result = np.clip(img_rgb, 0, 255).astype(np.uint8)
    
    if has_alpha:
        result = np.dstack([result, alpha])
    
    cv.imwrite(str(image_path), result)
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 blend_erp_seams_simple.py <image_or_directory>")
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
                    if "_raw" not in str(img_file):
                        if blend_erp_seams(img_file):
                            count += 1
                            print(f"  Blended: {img_file.name}")
                for img_file in scan_dir.glob("equirect_*_masked.png"):
                    if "_raw" not in str(img_file):
                        if blend_erp_seams(img_file):
                            count += 1
                            print(f"  Blended: {img_file.name}")
        print(f"✓ Blended {count} images")
    else:
        print(f"Path not found: {path}")
