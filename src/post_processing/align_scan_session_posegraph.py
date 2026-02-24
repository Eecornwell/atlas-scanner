#!/usr/bin/env python3
"""
Pose graph optimization for point cloud alignment.

Uses trajectory poses as initialization and refines with ICP + pose graph optimization.
Better for scan sets where not all scans overlap with the first scan.
"""

import sys
from pathlib import Path
import open3d as o3d
import numpy as np
import json

def load_trajectory_pose(scan_dir):
    """Load initial trajectory pose from trajectory.json or metadata.json."""
    trajectory_file = scan_dir / "trajectory.json"
    if trajectory_file.exists():
        with open(trajectory_file, 'r') as f:
            trajectory = json.load(f)
            if 'current_pose' in trajectory and 'lidar_pose' in trajectory['current_pose']:
                lidar_pose = trajectory['current_pose']['lidar_pose']
                pose = np.eye(4)
                pos = lidar_pose['position']
                pose[:3, 3] = [pos['x'], pos['y'], pos['z']]
                quat = lidar_pose['orientation']
                from scipy.spatial.transform import Rotation as R
                rot = R.from_quat([quat['x'], quat['y'], quat['z'], quat['w']])
                pose[:3, :3] = rot.as_matrix()
                return pose
    
    metadata_file = scan_dir / "metadata.json"
    if metadata_file.exists():
        with open(metadata_file, 'r') as f:
            metadata = json.load(f)
            if 'trajectory_pose' in metadata:
                pose_data = metadata['trajectory_pose']
                pose = np.eye(4)
                pose[:3, :3] = np.array(pose_data['rotation']).reshape(3, 3)
                pose[:3, 3] = np.array(pose_data['translation'])
                return pose
    
    return np.eye(4)

def pairwise_icp(source, target, voxel_size, init_transform):
    """Multi-scale ICP refinement."""
    current_transform = init_transform
    loss = o3d.pipelines.registration.TukeyLoss(k=voxel_size * 2.0)
    
    scales = [
        (voxel_size * 15, 50),
        (voxel_size * 5, 30),
        (voxel_size * 2, 20),
        (voxel_size * 1.0, 14)
    ]
    
    for i, (max_dist, max_iter) in enumerate(scales):
        result = o3d.pipelines.registration.registration_icp(
            source, target, max_dist, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter))
        current_transform = result.transformation
    
    return result

def register_pose_graph(session_dir):
    """Pose graph optimization with trajectory initialization."""
    
    session_path = Path(session_dir)
    scan_dirs = sorted(session_path.glob("fusion_scan_*"))
    
    if len(scan_dirs) < 2:
        print(f"Need at least 2 scans, found {len(scan_dirs)}")
        return
    
    print(f"Found {len(scan_dirs)} scan directories")
    
    # Find point clouds
    ply_files = []
    for scan_dir in scan_dirs:
        ply = scan_dir / "world_colored_exact.ply"
        if not ply.exists():
            ply = scan_dir / "world_colored_pointcloud.ply"
        if ply.exists():
            ply_files.append(ply)
            pcd_temp = o3d.io.read_point_cloud(str(ply))
            center = np.mean(np.asarray(pcd_temp.points), axis=0)
            print(f"  {scan_dir.name}/{ply.name}: center = [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
    
    if len(ply_files) < 2:
        print(f"Need at least 2 point clouds, found {len(ply_files)}")
        return
    
    # Load trajectory poses
    trajectory_poses = []
    for scan_dir in scan_dirs:
        pose = load_trajectory_pose(scan_dir)
        trajectory_poses.append(pose)
        print(f"Trajectory {scan_dir.name}: t=[{pose[0,3]:.3f}, {pose[1,3]:.3f}, {pose[2,3]:.3f}]")
    
    # Preprocess clouds
    voxel_size = 0.05
    pcds = []
    for f in ply_files:
        pcd = o3d.io.read_point_cloud(str(f))
        pcd.colors = o3d.utility.Vector3dVector()
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd = pcd.voxel_down_sample(voxel_size)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)
        pcds.append(pcd)
        print(f"Preprocessed {f.parent.name}: {len(pcd.points)} points")
    
    # Build pose graph
    print("\nBuilding pose graph...")
    pose_graph = o3d.pipelines.registration.PoseGraph()
    
    # Add nodes with trajectory poses
    for i, traj_pose in enumerate(trajectory_poses):
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(traj_pose))
    
    # Add edges for overlapping scans
    n_scans = len(pcds)
    
    # Sequential edges (always add)
    for i in range(n_scans - 1):
        print(f"\nRefining edge {i} -> {i+1} (sequential)")
        
        # Relative transform: T_i_to_i+1 = inv(T_i+1) @ T_i
        init_relative = np.linalg.inv(trajectory_poses[i+1]) @ trajectory_poses[i]
        
        result = pairwise_icp(pcds[i], pcds[i+1], voxel_size, init_relative)
        
        print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
        
        if result.fitness > 0.3:
            information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                pcds[i], pcds[i+1], voxel_size * 2, result.transformation)
            
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(
                    i, i+1, result.transformation, information, uncertain=False))
    
    # KNN edges (add for better connectivity)
    centers = [np.mean(np.asarray(pcd.points), axis=0) for pcd in pcds]
    
    for i in range(n_scans):
        # Find 2 closest neighbors (excluding sequential neighbors)
        dists = []
        for j in range(n_scans):
            if abs(i - j) > 1:  # Skip sequential neighbors
                dist = np.linalg.norm(centers[i] - centers[j])
                dists.append((j, dist))
        
        dists.sort(key=lambda x: x[1])
        
        # Try to add closest non-sequential neighbor
        for j, dist in dists[:1]:
            if dist > 1.5:  # Skip if too far
                continue
            
            src_idx = min(i, j)
            tgt_idx = max(i, j)
            
            print(f"\nRefining edge {src_idx} -> {tgt_idx} (KNN, dist={dist:.2f}m)")
            
            init_relative = np.linalg.inv(trajectory_poses[tgt_idx]) @ trajectory_poses[src_idx]
            result = pairwise_icp(pcds[src_idx], pcds[tgt_idx], voxel_size, init_relative)
            
            print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
            
            if result.fitness > 0.3:
                information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    pcds[src_idx], pcds[tgt_idx], voxel_size * 2, result.transformation)
                
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        src_idx, tgt_idx, result.transformation, information, uncertain=True))
    
    # Optimize pose graph
    print(f"\nOptimizing pose graph ({len(pose_graph.nodes)} nodes, {len(pose_graph.edges)} edges)...")
    
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=voxel_size * 2,
        edge_prune_threshold=0.25,
        reference_node=0)
    
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
    
    print("Optimization complete!")
    
    # Save results
    print("\nSaving aligned results...")
    transforms = [node.pose for node in pose_graph.nodes]
    
    for i, scan_dir in enumerate(scan_dirs):
        # Save refined trajectory
        traj_file = scan_dir / "trajectory.json"
        if traj_file.exists():
            with open(traj_file, 'r') as f:
                traj = json.load(f)
            
            # Update with refined pose
            T_refined = transforms[i]
            from scipy.spatial.transform import Rotation as R
            
            traj_refined = traj.copy()
            traj_refined['current_pose'] = traj_refined.get('current_pose', {})
            traj_refined['current_pose']['lidar_pose'] = {
                'position': {
                    'x': float(T_refined[0, 3]),
                    'y': float(T_refined[1, 3]),
                    'z': float(T_refined[2, 3])
                },
                'orientation': dict(zip(['x', 'y', 'z', 'w'], 
                    R.from_matrix(T_refined[:3, :3]).as_quat().tolist()))
            }
            
            # Save refined trajectory
            with open(scan_dir / "trajectory_icp_refined.json", 'w') as f:
                json.dump(traj_refined, f, indent=2)
        
        # Save aligned colored point cloud
        colored_file = scan_dir / "world_colored_exact.ply"
        if not colored_file.exists():
            colored_file = scan_dir / "world_colored_pointcloud.ply"
        if colored_file.exists():
            pcd_colored = o3d.io.read_point_cloud(str(colored_file))
            pcd_colored.transform(transforms[i])
            output_colored = scan_dir / f"{colored_file.stem}_aligned.ply"
            o3d.io.write_point_cloud(str(output_colored), pcd_colored)
        
        pose = transforms[i]
        translation = pose[:3, 3]
        rotation_matrix = pose[:3, :3]
        
        sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = 0
        
        euler_deg = np.degrees([x, y, z])
        print(f"{scan_dir.name}:")
        print(f"  Position: [{translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}]")
        print(f"  Rotation: [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]")
    
    # Create merged colored point cloud
    print("\nCreating merged colored point cloud...")
    merged_colored = o3d.geometry.PointCloud()
    
    for i, scan_dir in enumerate(scan_dirs):
        colored_file = scan_dir / "world_colored_exact.ply"
        if not colored_file.exists():
            colored_file = scan_dir / "world_colored_pointcloud.ply"
        if colored_file.exists():
            pcd = o3d.io.read_point_cloud(str(colored_file))
            pcd.transform(transforms[i])
            merged_colored = merged_colored + pcd
    
    if len(merged_colored.points) > 0:
        merged_colored = merged_colored.voxel_down_sample(0.005)
        merged_colored_file = session_path / "merged_aligned_colored.ply"
        o3d.io.write_point_cloud(str(merged_colored_file), merged_colored)
        print(f"Merged: {merged_colored_file} ({len(merged_colored.points)} points)")

def main():
    if len(sys.argv) != 2:
        print("Usage: python align_scan_session_posegraph.py <session_dir>")
        return
    
    register_pose_graph(sys.argv[1])

if __name__ == "__main__":
    main()
