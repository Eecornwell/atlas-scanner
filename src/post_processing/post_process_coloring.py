#!/usr/bin/env python3
import os
import sys
from pathlib import Path

def process_scan_coloring(scan_dir, use_exact=False):
    """Apply coloring to a single scan directory"""
    scan_path = Path(scan_dir)
    
    # Find PLY and image files
    world_ply = scan_path / "world_lidar.ply"
    sensor_ply = scan_path / "sensor_lidar.ply"
    equirect_imgs = list(scan_path.glob("equirect_*.jpg"))
    
    # Determine which PLY to use
    if world_ply.exists():
        lidar_ply = world_ply
        ply_type = "world"
    elif sensor_ply.exists():
        lidar_ply = sensor_ply
        ply_type = "sensor"
    else:
        print(f"No LiDAR PLY found in {scan_dir}")
        return False
    
    if not equirect_imgs:
        print(f"No equirectangular image found in {scan_dir}")
        return False
    
    equirect_img = equirect_imgs[0]  # Use first image found
    
    print(f"Processing {scan_path.name}: {lidar_ply.name} + {equirect_img.name}")
    
    # Apply coloring using fusion script
    fusion_script = "exact_match_fusion.py" if use_exact else "sensor_fusion.py"
    
    try:
        os.system(f"cd /home/orion/atlas_ws && python3 src/atlas-scanner/src/post_processing/{fusion_script} '{scan_dir}'")
        
        # Check if colored PLY was created
        colored_candidates = [
            scan_path / "world_colored_exact.ply",
            scan_path / "world_colored.ply",
            scan_path / "sensor_colored.ply"
        ]
        
        for colored_ply in colored_candidates:
            if colored_ply.exists():
                print(f"✓ Created {colored_ply.name}")
                return True
        
        print(f"✗ Failed to create colored PLY for {scan_path.name}")
        return False
            
    except Exception as e:
        print(f"✗ Error processing {scan_path.name}: {e}")
        return False

def post_process_session_coloring(session_dir, use_exact=False):
    """Apply coloring to all scans in a session directory"""
    session_path = Path(session_dir)
    if not session_path.exists():
        print(f"Session directory not found: {session_dir}")
        return
    
    # Find all scan subdirectories
    scan_dirs = sorted([d for d in session_path.iterdir() if d.is_dir() and d.name.startswith('fusion_scan_')])
    
    if not scan_dirs:
        print("No scan directories found")
        return
    
    method = "exact match (calibration tool projection)" if use_exact else "original method"
    print(f"Post-processing coloring for {len(scan_dirs)} scans using {method}...")
    
    success_count = 0
    for scan_dir in scan_dirs:
        if process_scan_coloring(scan_dir, use_exact):
            success_count += 1
    
    print(f"\n✓ Coloring complete: {success_count}/{len(scan_dirs)} scans processed successfully")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python post_process_coloring.py <session_directory> [--use-exact]")
        sys.exit(1)
    
    use_exact = "--use-exact" in sys.argv
    post_process_session_coloring(sys.argv[1], use_exact)