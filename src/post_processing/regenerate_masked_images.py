#!/usr/bin/env python3
import sys
from pathlib import Path

# Reuse apply_mask logic
sys.path.insert(0, str(Path(__file__).parent))
from apply_lidar_mask import apply_mask

def regenerate_masked_images(session_dir):
    """Regenerate masked images from blended ERP images"""
    session_path = Path(session_dir)
    mask_file = "/home/orion/atlas_ws/src/atlas-scanner/src/lidar_mask.png"
    
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
    if len(sys.argv) != 2:
        print("Usage: python3 regenerate_masked_images.py <session_directory>")
        sys.exit(1)
    
    regenerate_masked_images(sys.argv[1])
