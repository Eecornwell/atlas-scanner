#!/usr/bin/env python3
"""
Geometry-based point cloud alignment with RANSAC + Multi-scale ICP.

Algorithm Overview:
------------------
1. RANSAC Initialization:
   - Uses FPFH (Fast Point Feature Histograms) geometric features
   - 200K iterations for robust initial alignment
   - Purely geometry-based (no color influence)

2. Multi-scale ICP Refinement:
   - 4 scales: coarse to fine (voxel_size * 15 -> 1.0)
   - Point-to-plane ICP with Tukey robust loss function
   - Handles outliers and noise effectively

3. Sequential Alignment Strategy:
   - All scans aligned to first scan (reference)
   - Avoids drift accumulation from sequential chaining
   - More reliable than pose graph optimization for small scan sets

4. Coordinate Frame Handling:
   - world_lidar.ply: Already in world frame (poses baked in)
   - world_colored_exact.ply: In sensor/camera frame (needs alignment)
   - Algorithm aligns colored point clouds directly (sensor frame)
   - Colors stripped during registration to focus on 3D geometry
   - Refined transforms applied to colored point clouds for output

5. Geometry-First Registration:
   - Alignment computed using geometry only (colors removed)
   - FPFH features based on surface normals and curvature
   - Point-to-plane ICP uses geometric distances only
   - Produces accurate alignment independent of color/texture

Output:
-------
- Individual aligned colored point clouds: world_colored_exact_aligned.ply in each scan directory
- Merged colored point cloud: merged_aligned_colored.ply in session directory

Note: world_lidar.ply files are NOT aligned as they're already in world frame.
"""

import sys
from pathlib import Path
import open3d as o3d
import numpy as np
import json

def load_trajectory_pose(scan_dir):
    """Load initial trajectory pose from trajectory.json or metadata.json."""
    # Try trajectory.json first (newer format)
    trajectory_file = scan_dir / "trajectory.json"
    if trajectory_file.exists():
        with open(trajectory_file, 'r') as f:
            trajectory = json.load(f)
            if 'current_pose' in trajectory and 'lidar_pose' in trajectory['current_pose']:
                lidar_pose = trajectory['current_pose']['lidar_pose']
                pose = np.eye(4)
                # Extract position
                pos = lidar_pose['position']
                pose[:3, 3] = [pos['x'], pos['y'], pos['z']]
                # Extract orientation (quaternion) and convert to rotation matrix
                quat = lidar_pose['orientation']
                from scipy.spatial.transform import Rotation as R
                rot = R.from_quat([quat['x'], quat['y'], quat['z'], quat['w']])
                pose[:3, :3] = rot.as_matrix()
                return pose
    
    # Fallback to metadata.json (older format)
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

def get_point_cloud_center_ply(ply_path):
    """Get center of PLY point cloud."""
    pcd = o3d.io.read_point_cloud(str(ply_path))
    points = np.asarray(pcd.points)
    return np.mean(points, axis=0)

def get_sequential_and_knn_pairs(ply_files):
    """Get sequential pairs + KNN pairs for better connectivity."""
    
    # Sequential pairs (ensures connectivity)
    sequential_pairs = []
    for i in range(len(ply_files) - 1):
        sequential_pairs.append([i, i + 1])
    
    # KNN pairs based on centers
    centers = [get_point_cloud_center_ply(s) for s in ply_files]
    
    knn_pairs = []
    for i in range(len(centers)):
        # Find 2 closest neighbors
        dists = []
        for j in range(len(centers)):
            if i != j:
                dist = np.linalg.norm(centers[i] - centers[j])
                dists.append((j, dist))
        
        dists.sort(key=lambda x: x[1])
        
        # Add closest 2 neighbors
        for j, _ in dists[:2]:
            pair = [min(i, j), max(i, j)]
            if pair not in sequential_pairs and pair not in knn_pairs:
                knn_pairs.append(pair)
    
    return sequential_pairs, knn_pairs

def ransac_initial_alignment(source, target, voxel_size):
    """RANSAC-based initial alignment using FPFH features."""
    radius_feature = voxel_size * 5
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh, True,
        max_correspondence_distance=voxel_size * 2.0,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.8),
                  o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 2.0)],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(200000, 0.999))
    
    print(f"    RANSAC fitness: {result_ransac.fitness:.3f}, RMSE: {result_ransac.inlier_rmse:.4f}")
    return result_ransac.transformation, result_ransac.fitness

def pairwise_registration_ba_style(source, target, voxel_size=0.05):
    """
    RANSAC + Multi-scale ICP with Tukey loss.
    
    Steps:
    1. RANSAC with FPFH features for initial alignment
    2. Multi-scale ICP (4 scales) with point-to-plane + Tukey loss
    3. Returns transformation, information matrix, and fitness score
    """
    
    # RANSAC initialization
    current_transform, ransac_fitness = ransac_initial_alignment(source, target, voxel_size)
    
    # Skip ICP if RANSAC failed badly
    if ransac_fitness < 0.05:
        print(f"    WARNING: RANSAC fitness too low ({ransac_fitness:.3f}), using identity")
        current_transform = np.eye(4)
    
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
        print(f"    ICP scale {i+1}: fitness={result.fitness:.3f}, RMSE={result.inlier_rmse:.4f}")
    
    information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, scales[-1][0], current_transform)
    
    return current_transform, information, result.fitness

def register_final_ba(session_dir):
    """
    Sequential registration with RANSAC + multi-scale ICP.
    
    Process:
    1. Load colored point clouds (sensor frame) for alignment
    2. Strip colors and preprocess (outlier removal, downsampling, normals)
    3. Align each scan to reference using RANSAC + ICP
    4. Apply refined transforms to original colored point clouds
    5. Save individual aligned colored clouds and merged result
    
    Note: world_lidar.ply files already have poses baked in (world frame),
    while world_colored_exact.ply files are in sensor/camera frame and need alignment.
    """
    
    session_path = Path(session_dir)
    
    # Find scan subdirectories
    scan_dirs = sorted(session_path.glob("fusion_scan_*"))
    
    if len(scan_dirs) < 2:
        print(f"Need at least 2 scans for alignment, found {len(scan_dirs)}")
        return
    
    print(f"Found {len(scan_dirs)} scan directories")
    
    # Find point clouds - use colored for alignment since lidar is already in world frame
    ply_files = []
    for scan_dir in scan_dirs:
        # Use colored point clouds (sensor frame) for alignment
        ply = scan_dir / "world_colored_exact.ply"
        if not ply.exists():
            ply = scan_dir / "world_colored_pointcloud.ply"
        if not ply.exists():
            ply = scan_dir / "sensor_lidar.ply"  # Fallback to sensor frame lidar
        if ply.exists():
            ply_files.append(ply)
            # Debug: check point cloud center
            pcd_temp = o3d.io.read_point_cloud(str(ply))
            center = np.mean(np.asarray(pcd_temp.points), axis=0)
            print(f"  Using {scan_dir.name}/{ply.name}: center = [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
    
    if len(ply_files) < 2:
        print(f"Need at least 2 point clouds, found {len(ply_files)}")
        return
    
    print(f"Found {len(ply_files)} point clouds")
    
    # Load trajectory poses to unbake from colored point clouds
    trajectory_poses = []
    for scan_dir in scan_dirs:
        pose = load_trajectory_pose(scan_dir)
        trajectory_poses.append(pose)
        # Debug: check if pose is identity
        is_identity = np.allclose(pose, np.eye(4))
        print(f"Loaded trajectory pose for {scan_dir.name}: {'Identity' if is_identity else 'Non-identity'}")
        if not is_identity:
            print(f"  Translation: {pose[:3, 3]}")
    
    # Remove debug output since we're now using colored directly
    # Load and preprocess clouds
    voxel_size = 0.05
    pcds = []
    
    for f in ply_files:
        pcd = o3d.io.read_point_cloud(str(f))
        # Remove colors to focus purely on geometry
        pcd.colors = o3d.utility.Vector3dVector()
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd = pcd.voxel_down_sample(voxel_size)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)
        pcds.append(pcd)
        print(f"Preprocessed {f.parent.name}/{f.name}: {len(pcd.points)} points")
    
    # Sequential alignment to first scan (reference)
    print("\nAligning scans sequentially to reference...")
    reference_pcd = pcds[0]
    transforms = [np.eye(4)]  # Reference has identity transform
    
    for i in range(1, len(pcds)):
        print(f"\nAligning scan {i+1} ({ply_files[i].parent.name}) to reference...")
        
        transformation, information, fitness = pairwise_registration_ba_style(
            pcds[i], reference_pcd, voxel_size)
        
        print(f"  Final fitness: {fitness:.3f}")
        transforms.append(transformation)
    
    # Save results - colored point clouds with refined alignment
    print("\nSaving aligned results...")
    for i, scan_dir in enumerate(scan_dirs):
        # Save aligned colored (this is what we aligned)
        colored_file = scan_dir / "world_colored_exact.ply"
        if not colored_file.exists():
            colored_file = scan_dir / "world_colored_pointcloud.ply"
        if colored_file.exists():
            pcd_colored = o3d.io.read_point_cloud(str(colored_file))
            pcd_colored.transform(transforms[i])
            output_colored = scan_dir / f"{colored_file.stem}_aligned.ply"
            o3d.io.write_point_cloud(str(output_colored), pcd_colored)
        
        # Extract rotation (as Euler angles) and translation
        pose = transforms[i]
        translation = pose[:3, 3]
        rotation_matrix = pose[:3, :3]
        
        # Convert rotation matrix to Euler angles (ZYX convention)
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
        print(f"Merged colored: {merged_colored_file} ({len(merged_colored.points)} points)")

def main():
    if len(sys.argv) != 2:
        print("Usage: python align_scan_session.py <session_dir>")
        print("Example: python align_scan_session.py ~/atlas_ws/data/synchronized_scans/sync_fusion_20260216_203306")
        return
    
    register_final_ba(sys.argv[1])

if __name__ == "__main__":
    main()
