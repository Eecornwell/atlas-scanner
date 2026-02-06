#!/usr/bin/env python3
import sys
import numpy as np

def ply_to_e57(ply_path, e57_path):
    """Convert PLY file to E57 format"""
    try:
        import pye57
    except ImportError:
        print("Error: pye57 library not installed. Install with: pip3 install pye57")
        return False
    
    try:
        # Read PLY file
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(ply_path)
        
        if len(pcd.points) == 0:
            print(f"Error: No points in {ply_path}")
            return False
        
        # Prepare data for E57
        points = np.asarray(pcd.points)
        
        # Check if colors exist
        has_colors = len(pcd.colors) > 0
        if has_colors:
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
        
        # Create E57 file
        e57 = pye57.E57(e57_path, mode='w')
        
        # Prepare data dictionary
        data = {
            "cartesianX": points[:, 0],
            "cartesianY": points[:, 1],
            "cartesianZ": points[:, 2],
        }
        
        if has_colors:
            data["colorRed"] = colors[:, 0]
            data["colorGreen"] = colors[:, 1]
            data["colorBlue"] = colors[:, 2]
        
        # Write scan
        e57.write_scan_raw(data)
        e57.close()
        
        print(f"✓ Saved E57: {e57_path} ({len(points)} points)")
        return True
        
    except Exception as e:
        print(f"Error converting to E57: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 ply_to_e57.py <input.ply> <output.e57>")
        sys.exit(1)
    
    success = ply_to_e57(sys.argv[1], sys.argv[2])
    sys.exit(0 if success else 1)
