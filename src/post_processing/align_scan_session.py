#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import os
import sys
from pathlib import Path

def pairwise_registration(source, target, voxel_size=0.1):
    """Compute transformation between two point clouds using robust Generalized ICP"""
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)
    
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=30))
    target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=30))
    
    # Use Generalized ICP (plane-to-plane) which is more robust
    result = o3d.pipelines.registration.registration_generalized_icp(
        source_down, target_down,
        max_correspondence_distance=voxel_size * 3,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
    
    return result

def full_registration(pcds, voxel_size=0.1):
    """Multi-way registration using pose graph optimization"""
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.eye(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    
    n_pcds = len(pcds)
    
    # Build pose graph with pairwise registrations
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            print(f"  Registering {source_id} -> {target_id}")
            
            result = pairwise_registration(pcds[source_id], pcds[target_id], voxel_size)
            
            # Only add edge if registration is reasonable
            R = result.transformation[:3, :3]
            t = result.transformation[:3, 3]
            angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
            trans_dist = np.linalg.norm(t)
            
            # Accept if reasonable fitness and not extreme rotation (< 60 degrees)
            if result.fitness > 0.3 and np.degrees(angle) < 60:
                print(f"    fitness={result.fitness:.3f}, angle={np.degrees(angle):.1f}°, trans={trans_dist:.3f}m - ACCEPTED")
                
                # Add edge to pose graph
                information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    pcds[source_id], pcds[target_id], voxel_size * 2, result.transformation)
                
                if source_id == target_id - 1:
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id, target_id,
                                                                 result.transformation,
                                                                 information, uncertain=False))
                else:
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id, target_id,
                                                                 result.transformation,
                                                                 information, uncertain=True))
            else:
                print(f"    fitness={result.fitness:.3f}, angle={np.degrees(angle):.1f}°, trans={trans_dist:.3f}m - REJECTED")
    
    # Add remaining nodes
    for i in range(1, n_pcds):
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))
    
    # Optimize pose graph with conservative settings
    print("\nOptimizing pose graph...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=voxel_size * 2,
        edge_prune_threshold=0.1,
        preference_loop_closure=0.1,
        reference_node=0)
    
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
    
    return pose_graph

def align_scan_session(session_dir):
    """Align all scans in a session directory"""
    session_path = Path(session_dir)
    if not session_path.exists():
        print(f"Session directory not found: {session_dir}")
        return
    
    # Find all scan subdirectories
    scan_dirs = sorted([d for d in session_path.iterdir() if d.is_dir() and d.name.startswith('fusion_scan_')])
    
    if len(scan_dirs) < 2:
        print(f"Need at least 2 scans for alignment. Found: {len(scan_dirs)}")
        return
    
    print(f"Found {len(scan_dirs)} scans to align")
    
    # Find colored point clouds in each scan
    scan_files = []
    for scan_dir in scan_dirs:
        # Try different colored PLY names in order of preference
        colored_candidates = [
            scan_dir / "world_colored.ply",
            scan_dir / "world_colored_pointcloud.ply",
            scan_dir / "sensor_colored_pointcloud.ply"
        ]
        
        colored_ply = None
        for candidate in colored_candidates:
            if candidate.exists():
                colored_ply = candidate
                break
        
        if not colored_ply:
            # Try any colored PLY file
            alternatives = list(scan_dir.glob("*colored*.ply"))
            if alternatives:
                colored_ply = alternatives[0]
            else:
                print(f"No colored point cloud found in {scan_dir}")
                continue
        
        scan_files.append(colored_ply)
    
    if len(scan_files) < 2:
        print("Need at least 2 colored point clouds for alignment")
        return
    
    print(f"Aligning {len(scan_files)} point clouds...")
    
    # Load all point clouds
    pcds = []
    for scan_file in scan_files:
        pcd = o3d.io.read_point_cloud(str(scan_file))
        pcds.append(pcd)
        print(f"Loaded {scan_file.name}: {len(pcd.points)} points")
    
    # Perform multi-way registration with pose graph optimization
    print("\nBuilding pose graph...")
    pose_graph = full_registration(pcds, voxel_size=0.1)
    
    # Save aligned scans
    aligned_dir = session_path
    print("\nSaving aligned scans...")
    
    for i, (scan_file, pcd) in enumerate(zip(scan_files, pcds)):
        # Apply optimized transformation
        pcd_aligned = o3d.geometry.PointCloud(pcd)
        pcd_aligned.transform(pose_graph.nodes[i].pose)
        
        aligned_file = aligned_dir / f"scan_{i+1:03d}_aligned.ply"
        o3d.io.write_point_cloud(str(aligned_file), pcd_aligned)
        print(f"✓ Saved {aligned_file.name}")
    
    print(f"\n✓ Alignment complete. Individual aligned scans saved in: {aligned_dir}")
    
    print("\n=== ICP Alignment Summary ===")
    for i in range(len(scan_files)):
        print(f"Scan {i+1}: Optimized with pose graph")
    
    # Create merged point cloud
    print("\nCreating merged point cloud...")
    merged_pcd = o3d.geometry.PointCloud()
    
    for i in range(1, len(scan_files) + 1):
        aligned_file = aligned_dir / f"scan_{i:03d}_aligned.ply"
        if aligned_file.exists():
            pcd = o3d.io.read_point_cloud(str(aligned_file))
            merged_pcd += pcd
            print(f"Added scan {i}: {len(pcd.points)} points")
    
    # Save merged point cloud
    merged_file = aligned_dir / "merged_aligned_pointcloud.ply"
    o3d.io.write_point_cloud(str(merged_file), merged_pcd)
    
    print(f"\n✓ Merged point cloud saved: {merged_file}")
    print(f"  Total points: {len(merged_pcd.points)}")
    
    # Save ICP-refined poses for COLMAP
    print("\nSaving ICP-refined poses...")
    import json
    
    for i, scan_dir in enumerate(scan_dirs):
        traj_file = scan_dir / "trajectory.json"
        if not traj_file.exists():
            continue
        
        with open(traj_file) as f:
            traj = json.load(f)
        
        # Get optimized pose from pose graph
        T_optimized = pose_graph.nodes[i].pose
        
        # Original pose
        orig_mat_str = traj['current_pose']['transformation_matrix_cloudcompare_absolute']
        T_orig = np.array([float(x) for x in orig_mat_str.split()]).reshape(4, 4)
        
        # Apply optimization: T_refined = T_optimized @ T_orig
        T_refined = T_optimized @ T_orig
        
        # Update trajectory with refined pose
        traj_refined = traj.copy()
        mat_str_refined = ' '.join([f"{x:.6f}" for x in T_refined.flatten()])
        traj_refined['current_pose']['transformation_matrix_cloudcompare_absolute'] = mat_str_refined
        
        # Update position and orientation from refined matrix
        from scipy.spatial.transform import Rotation as R
        traj_refined['current_pose']['lidar_pose']['position'] = {
            'x': float(T_refined[0, 3]),
            'y': float(T_refined[1, 3]),
            'z': float(T_refined[2, 3])
        }
        quat = R.from_matrix(T_refined[:3, :3]).as_quat()
        traj_refined['current_pose']['lidar_pose']['orientation'] = {
            'x': float(quat[0]),
            'y': float(quat[1]),
            'z': float(quat[2]),
            'w': float(quat[3])
        }
        
        # Save refined trajectory
        traj_refined_file = scan_dir / "trajectory_icp_refined.json"
        with open(traj_refined_file, 'w') as f:
            json.dump(traj_refined, f, indent=2)
        print(f"  Saved: {scan_dir.name}/trajectory_icp_refined.json")
    
    print("✓ ICP-refined poses saved")
    
    print(f"\n=== Output Files ===")
    for i in range(1, len(scan_files) + 1):
        aligned_file = aligned_dir / f"scan_{i:03d}_aligned.ply"
        if aligned_file.exists():
            pcd = o3d.io.read_point_cloud(str(aligned_file))
            print(f"  - {aligned_file.name} ({len(pcd.points)} points)")
    print(f"  - merged_aligned_pointcloud.ply ({len(merged_pcd.points)} points)")
    
    corrected_traj = aligned_dir / "corrected_trajectory.json"
    if corrected_traj.exists():
        print(f"  - corrected_trajectory.json (ICP-corrected poses)")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python align_scan_session.py <session_directory>")
        sys.exit(1)
    
    align_scan_session(sys.argv[1])