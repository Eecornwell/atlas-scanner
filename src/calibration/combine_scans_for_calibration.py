#!/usr/bin/env python3

import os
import shutil
import json
import numpy as np
from pathlib import Path

def combine_scans_for_calibration(base_dir, output_dir):
    """Combine multiple scan sessions into one calibration dataset"""
    
    os.makedirs(f"{output_dir}/images", exist_ok=True)
    os.makedirs(f"{output_dir}/points", exist_ok=True)
    
    base_path = Path(base_dir)
    
    # Find all fusion_scan directories
    fusion_dirs = sorted([d for d in base_path.iterdir() if d.is_dir() and d.name.startswith('fusion_scan')])
    
    timestamps = []
    total_count = 0
    
    for fusion_dir in fusion_dirs:
        # Find masked equirectangular image first, fallback to regular
        masked_files = list(fusion_dir.glob("equirect_*_masked.png"))
        equirect_files = list(fusion_dir.glob("equirect_*.jpg")) + list(fusion_dir.glob("equirect_*.png"))
        
        # Prefer masked images for calibration (will fill with gray noise)
        if masked_files:
            equirect_files = masked_files
            print(f"  Using masked image from {fusion_dir.name}")
        else:
            equirect_files = [f for f in equirect_files if '_masked' not in f.name]
        
        ply_files = list(fusion_dir.glob("sensor_lidar_*.ply")) or list(fusion_dir.glob("world_lidar.ply"))
        
        if equirect_files and ply_files:
            # Copy and convert image to PNG (handle alpha channel if present)
            src_img = equirect_files[0]
            dst_img = f"{output_dir}/images/{total_count:06d}.png"
            
            import cv2
            # Read with alpha channel if present
            img = cv2.imread(str(src_img), cv2.IMREAD_UNCHANGED)
            if img is not None:
                # If image has alpha channel (masked), fill transparent areas with gray noise instead of black
                if img.shape[2] == 4:
                    # Create gray noise background
                    bgr = img[:, :, :3]
                    alpha = img[:, :, 3] / 255.0
                    # Generate gray noise (mean 128, std 20)
                    noise = np.random.normal(128, 20, bgr.shape).astype('uint8')
                    # Blend: use image where alpha=1, use noise where alpha=0
                    img = (bgr * alpha[:,:,np.newaxis] + noise * (1 - alpha[:,:,np.newaxis])).astype('uint8')
                    
                    # Blur the transition areas to hide seams
                    mask_edges = cv2.dilate((alpha < 0.99).astype('uint8'), np.ones((15,15), np.uint8))
                    img = cv2.GaussianBlur(img, (15, 15), 0) * mask_edges[:,:,np.newaxis] + img * (1 - mask_edges[:,:,np.newaxis])
                    img = img.astype('uint8')
                    
                    print(f"    Applied mask with gray noise fill and blurred edges")
                
                cv2.imwrite(dst_img, img)
                
                # Also copy to root directory for SuperGlue
                root_img = f"{output_dir}/{total_count:06d}.png"
                cv2.imwrite(root_img, img)
                
                # Copy PLY file
                src_ply = ply_files[0]
                dst_ply = f"{output_dir}/points/{total_count:06d}.ply"
                shutil.copy2(src_ply, dst_ply)
                
                # Also copy to root directory
                root_ply = f"{output_dir}/{total_count:06d}.ply"
                shutil.copy2(src_ply, root_ply)
                
                # Create timestamp entry
                timestamps.append({
                    "image_id": total_count,
                    "timestamp": total_count * 1000000000,
                    "image_file": f"images/{total_count:06d}.png",
                    "points_file": f"points/{total_count:06d}.ply"
                })
                
                print(f"  Added scan {total_count}: {fusion_dir.name}")
                total_count += 1
    
    if total_count == 0:
        print("No valid scans found!")
        return 0
    
    # Create metadata files
    bag_names = [f"{i:06d}" for i in range(total_count)]
    
    calib_data = {
        "camera": {
            "camera_model": "equirectangular",
            "distortion_coeffs": [],
            "intrinsics": [1920.0, 960.0]
        },
        "meta": {
            "bag_names": bag_names,
            "camera_info_topic": "/camera/camera_info",
            "data_path": output_dir,
            "image_topic": "/equirectangular/image",
            "intensity_channel": "intensity",
            "points_topic": "/livox/lidar"
        },
        "results": {
            "T_lidar_camera": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "init_T_lidar_camera": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        }
    }
    
    with open(f"{output_dir}/calib.json", 'w') as f:
        json.dump(calib_data, f, indent=2)
    
    # Create preprocessing metadata
    metadata = {
        "camera_model": "equirectangular",
        "image_size": [1920, 960],
        "camera_intrinsics": [610, 610, 1920, 960],
        "camera_distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
        "image_topic": "/equirectangular/image",
        "points_topic": "/livox/lidar",
        "intensity_channel": "intensity",
        "num_images": total_count,
        "num_points": total_count,
        "timestamps": timestamps
    }
    
    with open(f"{output_dir}/preprocessing_result.json", 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"\n✓ Combined calibration dataset created!")
    print(f"Total images: {total_count}")
    print(f"Output: {output_dir}")
    
    return total_count

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) != 2:
        print("Usage: python3 combine_scans_for_calibration.py <session_directory>")
        print("Example: python3 combine_scans_for_calibration.py /home/orion/atlas_ws/data/synchronized_scans/sync_fusion_20260121_170652")
        sys.exit(1)
    
    base_dir = sys.argv[1]
    output_dir = "/home/orion/atlas_ws/output"
    
    if not os.path.exists(base_dir):
        print(f"Error: Input directory {base_dir} does not exist")
        sys.exit(1)
    
    # Clean output directory
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    
    count = combine_scans_for_calibration(base_dir, output_dir)
    
    if count > 1:
        print(f"\n✓ Ready for calibration with {count} images!")
        print("\nNext steps:")
        print("  cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration")
        print("  python3 ./find_matches_superglue.py ~/atlas_ws/output")
        print("  ./initial_guess_manual --data_path ~/atlas_ws/output")
        print("  ./calibrate --data_path ~/atlas_ws/output")
    else:
        print("Need more scan data for calibration")
