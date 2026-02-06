#!/usr/bin/env python3

import numpy as np
import json
import sys
from datetime import datetime

def save_ply_with_pose(points, filename, pose_data=None, colors=None):
    """Save PLY file with embedded pose data in header"""
    
    # Prepare header
    header_lines = [
        "ply",
        "format ascii 1.0"
    ]
    
    # Add pose data as comments in header
    if pose_data:
        header_lines.append("comment Generated with 6-DOF pose data")
        header_lines.append(f"comment timestamp {pose_data.get('timestamp', datetime.now().isoformat())}")
        header_lines.append("comment coordinate_system ROS_REP_103")
        header_lines.append("comment coordinate_frame odom")
        header_lines.append("comment coordinate_convention X_forward_Y_left_Z_up")
        header_lines.append("comment units meters_radians")
        
        # Position
        pos = pose_data.get('position', {})
        header_lines.append(f"comment pose_position {pos.get('x', 0.0)} {pos.get('y', 0.0)} {pos.get('z', 0.0)}")
        
        # Orientation (quaternion)
        ori = pose_data.get('orientation', {})
        header_lines.append(f"comment pose_orientation {ori.get('x', 0.0)} {ori.get('y', 0.0)} {ori.get('z', 0.0)} {ori.get('w', 1.0)}")
        
        # Transforms
        transforms = pose_data.get('transforms', {})
        if 'lidar_to_camera' in transforms:
            ltc = transforms['lidar_to_camera']
            ltc_trans = ltc.get('translation', {})
            ltc_rot = ltc.get('rotation', {})
            header_lines.append(f"comment lidar_to_camera_translation {ltc_trans.get('x', 0.0)} {ltc_trans.get('y', 0.0)} {ltc_trans.get('z', 0.0)}")
            header_lines.append(f"comment lidar_to_camera_rotation {ltc_rot.get('x', 0.0)} {ltc_rot.get('y', 0.0)} {ltc_rot.get('z', 0.0)} {ltc_rot.get('w', 1.0)}")
    
    # Vertex count and properties
    header_lines.append(f"element vertex {len(points)}")
    header_lines.append("property float x")
    header_lines.append("property float y") 
    header_lines.append("property float z")
    
    if colors is not None:
        header_lines.append("property uchar red")
        header_lines.append("property uchar green")
        header_lines.append("property uchar blue")
    
    header_lines.append("end_header")
    
    # Write file
    with open(filename, 'w') as f:
        # Write header
        for line in header_lines:
            f.write(line + '\n')
        
        # Write points
        for i, point in enumerate(points):
            if colors is not None:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {int(colors[i][0])} {int(colors[i][1])} {int(colors[i][2])}\n")
            else:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")

def extract_pose_from_ply(filename):
    """Extract pose data from PLY header comments"""
    pose_data = {}
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line == "end_header":
                break
            
            if line.startswith("comment timestamp"):
                pose_data['timestamp'] = line.split(' ', 2)[2]
            elif line.startswith("comment pose_position"):
                parts = line.split()
                pose_data['position'] = {
                    'x': float(parts[2]),
                    'y': float(parts[3]),
                    'z': float(parts[4])
                }
            elif line.startswith("comment pose_orientation"):
                parts = line.split()
                pose_data['orientation'] = {
                    'x': float(parts[2]),
                    'y': float(parts[3]),
                    'z': float(parts[4]),
                    'w': float(parts[5])
                }
            elif line.startswith("comment lidar_to_camera_translation"):
                parts = line.split()
                if 'transforms' not in pose_data:
                    pose_data['transforms'] = {}
                if 'lidar_to_camera' not in pose_data['transforms']:
                    pose_data['transforms']['lidar_to_camera'] = {}
                pose_data['transforms']['lidar_to_camera']['translation'] = {
                    'x': float(parts[2]),
                    'y': float(parts[3]),
                    'z': float(parts[4])
                }
            elif line.startswith("comment lidar_to_camera_rotation"):
                parts = line.split()
                if 'transforms' not in pose_data:
                    pose_data['transforms'] = {}
                if 'lidar_to_camera' not in pose_data['transforms']:
                    pose_data['transforms']['lidar_to_camera'] = {}
                pose_data['transforms']['lidar_to_camera']['rotation'] = {
                    'x': float(parts[2]),
                    'y': float(parts[3]),
                    'z': float(parts[4]),
                    'w': float(parts[5])
                }
    
    return pose_data

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 save_ply_with_pose.py <input_ply> [trajectory_json] [output_ply]")
        print("  or: python3 save_ply_with_pose.py --extract <ply_file>")
        sys.exit(1)
    
    if sys.argv[1] == "--extract":
        # Extract pose from PLY
        pose_data = extract_pose_from_ply(sys.argv[2])
        print(json.dumps(pose_data, indent=2))
        return
    
    input_ply = sys.argv[1]
    trajectory_json = sys.argv[2] if len(sys.argv) > 2 else None
    output_ply = sys.argv[3] if len(sys.argv) > 3 else input_ply.replace('.ply', '_with_pose.ply')
    
    # Load trajectory data
    pose_data = None
    if trajectory_json and trajectory_json.endswith('.json'):
        try:
            with open(trajectory_json, 'r') as f:
                trajectory_data = json.load(f)
                pose_data = trajectory_data.get('pose', trajectory_data)
        except Exception as e:
            print(f"Warning: Could not load trajectory data: {e}")
    
    # Load PLY points (simple parser)
    points = []
    colors = None
    
    with open(input_ply, 'r') as f:
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
                    if colors is None:
                        colors = []
                    colors.append([int(parts[3]), int(parts[4]), int(parts[5])])
    
    # Save with pose data
    save_ply_with_pose(points, output_ply, pose_data, colors)
    print(f"✓ Saved PLY with pose data: {output_ply}")

if __name__ == '__main__':
    main()