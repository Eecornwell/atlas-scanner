#!/usr/bin/env python3
"""
BA-style registration with improved connectivity.
"""

import sys
from pathlib import Path
import open3d as o3d
import numpy as np

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

def pairwise_registration_ba_style(source, target, voxel_size=0.05):
    """BA-style pairwise registration with Tukey loss."""
    
    sigmaf = 3.0
    sigma = sigmaf * voxel_size
    loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    
    # Coarse registration
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss))
    
    # Fine registration
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine, icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss))
    
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine, icp_fine.transformation)
    
    return transformation_icp, information_icp, icp_fine.fitness

def register_final_ba(input_dir, output_dir):
    """Final BA-style registration with guaranteed connectivity."""
    
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Find aligned PLY files
    ply_files = sorted(input_path.glob("scan_*_aligned.ply"))
    
    print(f"Found {len(ply_files)} PLY files")
    
    # Get pairs with guaranteed connectivity
    sequential_pairs, knn_pairs = get_sequential_and_knn_pairs(ply_files)
    print(f"Sequential pairs: {len(sequential_pairs)}, KNN pairs: {len(knn_pairs)}")
    
    # Load and preprocess clouds
    voxel_size = 0.05
    pcds = []
    for f in ply_files:
        pcd = o3d.io.read_point_cloud(str(f))
        pcd = pcd.voxel_down_sample(voxel_size)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcds.append(pcd)
        print(f"Preprocessed {f.name}: {len(pcd.points)} points")
    
    # Create pose graph
    pose_graph = o3d.pipelines.registration.PoseGraph()
    
    # Add nodes
    for i in range(len(pcds)):
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))
    
    # Add sequential edges (certain)
    print("Adding sequential edges...")
    for p in sequential_pairs:
        print(f"Sequential: {ply_files[p[0]].name} -> {ply_files[p[1]].name}")
        
        transformation, information, fitness = pairwise_registration_ba_style(
            pcds[p[0]], pcds[p[1]], voxel_size)
        
        print(f"  Fitness: {fitness:.3f}")
        
        pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            p[0], p[1], transformation, information, uncertain=False))
    
    # Add KNN edges (uncertain)
    print("Adding KNN edges...")
    for p in knn_pairs:
        print(f"KNN: {ply_files[p[0]].name} -> {ply_files[p[1]].name}")
        
        transformation, information, fitness = pairwise_registration_ba_style(
            pcds[p[0]], pcds[p[1]], voxel_size)
        
        print(f"  Fitness: {fitness:.3f}")
        
        # Only add edge if fitness is reasonable
        if fitness > 0.1:
            pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                p[0], p[1], transformation, information, uncertain=True))
    
    # Global optimization
    print("Running global optimization...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=voxel_size * 1.5,
        edge_prune_threshold=0.25,
        reference_node=0)
    
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
    
    # Save results
    print("Saving results...")
    for i, ply_file in enumerate(ply_files):
        pcd = o3d.io.read_point_cloud(str(ply_file))
        pcd.transform(pose_graph.nodes[i].pose)
        
        output_file = output_path / f"{ply_file.stem}_registered.ply"
        o3d.io.write_point_cloud(str(output_file), pcd)
        
        translation = pose_graph.nodes[i].pose[:3, 3]
        print(f"{ply_file.name}: [{translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}]")
    
    # Merged result
    merged = o3d.geometry.PointCloud()
    for i, ply_file in enumerate(ply_files):
        pcd = o3d.io.read_point_cloud(str(ply_file))
        pcd.transform(pose_graph.nodes[i].pose)
        merged = merged + pcd
    
    merged = merged.voxel_down_sample(0.005)
    merged_file = output_path / "merged_result.ply"
    o3d.io.write_point_cloud(str(merged_file), merged)
    print(f"Merged: {merged_file} ({len(merged.points)} points)")

def main():
    if len(sys.argv) != 3:
        print("Usage: python final_ba_registration.py <input_dir> <output_dir>")
        return
    
    register_final_ba(sys.argv[1], sys.argv[2])

if __name__ == "__main__":
    main()
