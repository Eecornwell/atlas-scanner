#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Applies ERP-based point cloud coloring to a single scan directory, selecting between world-frame and sensor-frame PLY inputs automatically.
import subprocess
import sys
import os
from pathlib import Path

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def process_scan_coloring(scan_dir, use_exact=False):
    """Apply coloring to a single scan directory"""
    try:
        scan_path = _safe_data(scan_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return False
    
    # Find PLY and image files
    world_ply = scan_path / "world_lidar.ply"
    sensor_ply = scan_path / "sensor_lidar.ply"
    equirect_imgs = list(scan_path.glob("equirect_*.jpg"))
    
    # Always use sensor_lidar.ply for coloring — exact_match_fusion.py assumes
    # sensor frame (T_camera_lidar maps sensor->camera directly).
    # world_lidar.ply is in world frame and requires a different transform chain.
    if sensor_ply.exists():
        lidar_ply = sensor_ply
        ply_type = "sensor"
    elif world_ply.exists():
        lidar_ply = world_ply
        ply_type = "world"
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
    script_path = Path(__file__).resolve().parent / fusion_script

    try:
        subprocess.run(
            [sys.executable, str(script_path), str(scan_dir)],
            check=False,
            cwd="/home/orion/atlas_ws",
        )
        
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
    try:
        session_path = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return
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
        try:
            _safe_data(scan_dir)
        except ValueError:
            continue
        if process_scan_coloring(scan_dir, use_exact):
            success_count += 1
    
    print(f"\n✓ Coloring complete: {success_count}/{len(scan_dirs)} scans processed successfully")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python post_process_coloring.py <session_directory> [--use-exact]")
        sys.exit(1)
    try:
        _safe_data(sys.argv[1])
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    use_exact = "--use-exact" in sys.argv
    post_process_session_coloring(sys.argv[1], use_exact)