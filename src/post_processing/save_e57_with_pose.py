#!/usr/bin/env python3

import numpy as np
import json
import sys

try:
    import pye57
except ImportError:
    print("Installing pye57...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pye57"])
    import pye57

def save_e57_with_pose(points, filename, pose_data=None, colors=None):
    """Save E57 file with embedded pose and metadata"""
    
    # Create E57 file
    e57 = pye57.E57(filename, mode='w')
    
    # Prepare point data
    data = {
        "cartesianX": points[:, 0].astype(np.float64),
        "cartesianY": points[:, 1].astype(np.float64), 
        "cartesianZ": points[:, 2].astype(np.float64)
    }
    
    if colors is not None:
        data["colorRed"] = colors[:, 0].astype(np.uint8)
        data["colorGreen"] = colors[:, 1].astype(np.uint8)
        data["colorBlue"] = colors[:, 2].astype(np.uint8)
    
    # Write scan to E57 - pye57 doesn't support pose in header via constructor
    # Write basic scan first
    e57.write_scan_raw(data)
    
    # Try to add pose via header modification (if supported)
    if pose_data:
        try:
            # Get the header that was just created
            header = e57.get_header(0)
            
            pos = pose_data.get('position', {})
            ori = pose_data.get('orientation', {})
            
            # Convert quaternion to rotation matrix
            qx, qy, qz, qw = ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)
            
            rotation_matrix = np.array([
                [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
            ])
            
            translation = np.array([pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0)])
            
            # Try to set pose data if header supports it
            if hasattr(header, 'rotation_matrix'):
                header.rotation_matrix = rotation_matrix
            if hasattr(header, 'translation'):
                header.translation = translation
                
        except Exception as e:
            print(f"Note: Could not embed pose in E57 header: {e}")
            print("Pose data available in PLY and JSON files")
    e57.close()
    
    print(f"✓ Saved E57 file: {filename}")

def load_ply_points(filename):
    """Load points from PLY file"""
    points = []
    colors = []
    
    with open(filename, 'r') as f:
        in_header = True
        has_color = False
        
        for line in f:
            line = line.strip()
            if in_header:
                if line == "end_header":
                    in_header = False
                elif "property uchar red" in line:
                    has_color = True
                continue
            
            parts = line.split()
            if len(parts) >= 3:
                point = [float(parts[0]), float(parts[1]), float(parts[2])]
                points.append(point)
                
                if has_color and len(parts) >= 6:
                    color = [int(parts[3]), int(parts[4]), int(parts[5])]
                    colors.append(color)
    
    points = np.array(points)
    colors = np.array(colors) if colors else None
    
    return points, colors

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 save_e57_with_pose.py <input_ply> [trajectory_json] [output_e57]")
        sys.exit(1)
    
    input_ply = sys.argv[1]
    trajectory_json = sys.argv[2] if len(sys.argv) > 2 else None
    output_e57 = sys.argv[3] if len(sys.argv) > 3 else input_ply.replace('.ply', '.e57')
    
    # Load trajectory data
    pose_data = None
    if trajectory_json and trajectory_json.endswith('.json'):
        try:
            with open(trajectory_json, 'r') as f:
                trajectory_data = json.load(f)
                pose_data = trajectory_data.get('pose', trajectory_data)
        except Exception as e:
            print(f"Warning: Could not load trajectory data: {e}")
    
    # Load PLY points
    points, colors = load_ply_points(input_ply)
    print(f"Loaded {len(points)} points from {input_ply}")
    
    # Save as E57
    save_e57_with_pose(points, output_e57, pose_data, colors)

if __name__ == '__main__':
    main()