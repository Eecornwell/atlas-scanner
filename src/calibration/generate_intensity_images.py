#!/usr/bin/env python3
import numpy as np
import cv2
import sys
import os
from pathlib import Path

def generate_intensity_image(ply_file, output_image, width=1920, height=960):
    """Generate intensity image from PLY point cloud for SuperGlue"""
    
    # Read PLY file
    points = []
    intensities = []
    
    with open(ply_file, 'r') as f:
        lines = f.readlines()
    
    header_end = next(i+1 for i, line in enumerate(lines) if line.strip() == 'end_header')
    has_intensity = any('intensity' in line for line in lines[:header_end])
    
    for line in lines[header_end:]:
        parts = line.strip().split()
        if len(parts) >= 3:
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                intensity = float(parts[3]) if has_intensity and len(parts) > 3 else 0.5
                points.append([x, y, z])
                intensities.append(intensity)
            except ValueError:
                continue
    
    points = np.array(points)
    intensities = np.array(intensities)
    
    if len(points) == 0:
        print(f"No points found in {ply_file}")
        return False
    
    # Project to spherical coordinates
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    
    azimuth = np.arctan2(y, x)
    r_xy = np.sqrt(x**2 + y**2)
    elevation = np.arctan2(z, r_xy)
    
    # Map to image coordinates
    u = ((azimuth + np.pi) / (2 * np.pi) * width).astype(int)
    v = (height/2 - (elevation / (np.pi/2)) * (height/2)).astype(int)
    
    # Filter valid pixels
    valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u = u[valid]
    v = v[valid]
    intensities = intensities[valid]
    
    # Create intensity image
    intensity_image = np.zeros((height, width), dtype=np.uint8)
    
    # Normalize intensities to 0-255
    if intensities.max() > 1.0:
        intensities = intensities / 255.0
    intensities = np.clip(intensities * 255, 0, 255).astype(np.uint8)
    
    # Fill image
    intensity_image[v, u] = intensities
    
    # Save
    cv2.imwrite(output_image, intensity_image)
    print(f"✓ Generated {output_image}")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 generate_intensity_images.py <output_directory>")
        sys.exit(1)
    
    output_dir = Path(sys.argv[1])
    
    # Find all PLY files
    ply_files = sorted(output_dir.glob("*.ply"))
    
    if not ply_files:
        print(f"No PLY files found in {output_dir}")
        sys.exit(1)
    
    print(f"Generating intensity images for {len(ply_files)} PLY files...")
    
    for ply_file in ply_files:
        base_name = ply_file.stem
        output_image = output_dir / f"{base_name}_lidar_intensities.png"
        generate_intensity_image(str(ply_file), str(output_image))
    
    print(f"\n✓ Generated {len(ply_files)} intensity images")
    print("Now you can run SuperGlue:")
    print(f"  cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration")
    print(f"  python3 ./find_matches_superglue.py {output_dir}")
