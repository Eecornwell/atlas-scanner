#!/usr/bin/env python3
"""Export depth maps from lidar for each camera view"""
import numpy as np
import cv2
from pathlib import Path
import sys
import json

def project_lidar_to_depth(points, R_w2c, T, fx, fy, cx, cy, width, height):
    """Project lidar points to camera and create depth map"""
    # Transform points to camera frame
    pts_cam = (R_w2c @ points.T).T + T
    
    # Filter points behind camera
    valid = pts_cam[:, 2] > 0
    pts_cam = pts_cam[valid]
    
    # Project to image
    u = fx * pts_cam[:, 0] / pts_cam[:, 2] + cx
    v = fy * pts_cam[:, 1] / pts_cam[:, 2] + cy
    depth = pts_cam[:, 2]
    
    # Filter points outside image
    valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, depth = u[valid], v[valid], depth[valid]
    
    # Create depth map (keep closest depth per pixel)
    depth_map = np.zeros((height, width), dtype=np.float32)
    u_int, v_int = u.astype(int), v.astype(int)
    
    for i in range(len(u_int)):
        if depth_map[v_int[i], u_int[i]] == 0 or depth[i] < depth_map[v_int[i], u_int[i]]:
            depth_map[v_int[i], u_int[i]] = depth[i]
    
    return depth_map

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 export_depth_maps.py <session_dir>")
        sys.exit(1)
    
    session_dir = Path(sys.argv[1]).expanduser()
    colmap_dir = session_dir / "colmap"
    
    # Load lidar points (already in COLMAP frame)
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(str(colmap_dir / "init_sparse" / "0" / "sparse.ply"))
    points = np.asarray(pcd.points)
    
    # Load camera metadata
    with open(colmap_dir / "images" / "metadata.json") as f:
        metadata = json.load(f)
    
    depth_dir = colmap_dir / "depths"
    depth_dir.mkdir(exist_ok=True)
    
    # Camera intrinsics (pinhole 90° FOV)
    width = height = 1024
    fx = fy = 512.0
    cx = cy = 512.0
    
    for item in metadata:
        from scipy.spatial.transform import Rotation as R
        qw, qx, qy, qz = item['quat']
        R_w2c = R.from_quat([qx, qy, qz, qw]).as_matrix()
        T = np.array(item['trans'])
        
        depth_map = project_lidar_to_depth(points, R_w2c, T, fx, fy, cx, cy, width, height)
        
        # Save as 16-bit PNG (scale to mm)
        depth_mm = (depth_map * 1000).astype(np.uint16)
        cv2.imwrite(str(depth_dir / f"{item['name']}.png"), depth_mm)
    
    print(f"✓ Exported {len(metadata)} depth maps to {depth_dir}")
