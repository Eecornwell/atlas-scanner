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
from scipy.spatial.transform import Rotation, Slerp


def _interp_pose(full_traj, target_time):
    """Interpolate (lerp + slerp) between the two nearest trajectory poses."""
    if not full_traj:
        return np.eye(4)

    times = np.array([p['timestamp'] for p in full_traj])
    idx = np.searchsorted(times, target_time)

    if idx == 0:
        p = full_traj[0]
    elif idx >= len(full_traj):
        p = full_traj[-1]
    else:
        p0, p1 = full_traj[idx - 1], full_traj[idx]
        t0, t1 = times[idx - 1], times[idx]
        alpha = (target_time - t0) / (t1 - t0) if t1 != t0 else 0.0

        pos0 = np.array([p0['position']['x'], p0['position']['y'], p0['position']['z']])
        pos1 = np.array([p1['position']['x'], p1['position']['y'], p1['position']['z']])
        t_interp = pos0 + alpha * (pos1 - pos0)

        q0 = [p0['orientation']['x'], p0['orientation']['y'],
              p0['orientation']['z'], p0['orientation']['w']]
        q1 = [p1['orientation']['x'], p1['orientation']['y'],
              p1['orientation']['z'], p1['orientation']['w']]
        rots = Rotation.from_quat([q0, q1])
        r_interp = Slerp([0.0, 1.0], rots)(alpha)

        T = np.eye(4)
        T[:3, :3] = r_interp.as_matrix()
        T[:3, 3] = t_interp
        return T

    # Single-point fallback
    pos = p.get('position', {})
    ori = p.get('orientation', {})
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(
        [ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)]
    ).as_matrix()
    T[:3, 3] = [pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)]
    return T


def load_trajectory_pose(scan_dir):
    """Load pose using capture_time-interpolated lookup (matches merge_with_trajectory.py)."""
    traj_file = scan_dir / "trajectory.json"
    if traj_file.exists():
        with open(traj_file) as f:
            traj = json.load(f)
        si = traj.get('scan_info', {})
        capture_time = si.get('capture_time') or si.get('scan_request_time')
        full = traj.get('full_trajectory', [])
        if capture_time and full:
            return _interp_pose(full, capture_time)
        # Fallback: current_pose
        cp = traj.get('current_pose', {})
        lp = cp.get('lidar_pose', cp)
        pos = lp.get('position', {})
        ori = lp.get('orientation', {})
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(
            [ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)]
        ).as_matrix()
        T[:3, 3] = [pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)]
        return T

    metadata_file = scan_dir / "metadata.json"
    if metadata_file.exists():
        with open(metadata_file) as f:
            metadata = json.load(f)
        if 'trajectory_pose' in metadata:
            pd = metadata['trajectory_pose']
            T = np.eye(4)
            T[:3, :3] = np.array(pd['rotation']).reshape(3, 3)
            T[:3, 3] = np.array(pd['translation'])
            return T

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
    
    # Find point clouds — prefer trajectory-aligned world clouds
    ply_files = []
    for scan_dir in scan_dirs:
        for name in ("world_colored_exact.ply", "world_colored_pointcloud.ply",
                     "sensor_colored_exact.ply", "sensor_colored.ply"):
            ply = scan_dir / name
            if ply.exists():
                ply_files.append(ply)
                break
    
    if len(ply_files) < 2:
        print(f"Need at least 2 point clouds, found {len(ply_files)}")
        return
    
    # Load trajectory poses (absolute odom frame)
    traj_poses_abs = []
    for scan_dir in scan_dirs:
        pose = load_trajectory_pose(scan_dir)
        traj_poses_abs.append(pose)
        print(f"Trajectory {scan_dir.name}: t=[{pose[0,3]:.3f}, {pose[1,3]:.3f}, {pose[2,3]:.3f}]")

    # Convert to relative-to-first-scan frame (matches merge_with_trajectory.py convention).
    # Pose graph nodes live in this relative frame so scan 0 stays at identity.
    T_first_inv = np.linalg.inv(traj_poses_abs[0])
    trajectory_poses = [T_first_inv @ T for T in traj_poses_abs]

    # Zero out Z translation in relative poses: terrestrial scanner moves on a
    # flat plane so any Z offset is LiDAR tilt integration artifact, not real
    # vertical displacement.
    for T in trajectory_poses:
        T[2, 3] = 0.0

    # Preprocess clouds (geometry only — strip colors for ICP)
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

    # Build pose graph (nodes in relative frame)
    print("\nBuilding pose graph...")
    pose_graph = o3d.pipelines.registration.PoseGraph()

    for rel_pose in trajectory_poses:
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(rel_pose))

    n_scans = len(pcds)

    # Sequential edges
    for i in range(n_scans - 1):
        print(f"\nRefining edge {i} -> {i+1} (sequential)")
        # Relative transform: bring scan i into scan i+1's frame
        init_relative = np.linalg.inv(trajectory_poses[i+1]) @ trajectory_poses[i]
        result = pairwise_icp(pcds[i], pcds[i+1], voxel_size, init_relative)
        print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
        information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            pcds[i], pcds[i+1], voxel_size * 2, result.transformation)
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                i, i+1, result.transformation, information, uncertain=False))

    # KNN loop-closure edges
    centers = [np.mean(np.asarray(pcd.points), axis=0) for pcd in pcds]
    added_pairs = {(i, i + 1) for i in range(n_scans - 1)}

    for i in range(n_scans):
        dists = sorted(
            [(j, np.linalg.norm(centers[i] - centers[j]))
             for j in range(n_scans) if abs(i - j) > 1],
            key=lambda x: x[1])
        for j, dist in dists[:2]:
            if dist > 1.5:
                break
            pair = (min(i, j), max(i, j))
            if pair in added_pairs:
                continue
            added_pairs.add(pair)
            src, tgt = pair
            print(f"\nRefining edge {src} -> {tgt} (KNN, dist={dist:.2f}m)")
            init_relative = np.linalg.inv(trajectory_poses[tgt]) @ trajectory_poses[src]
            result = pairwise_icp(pcds[src], pcds[tgt], voxel_size, init_relative)
            print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
            if result.fitness > 0.15:
                information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    pcds[src], pcds[tgt], voxel_size * 2, result.transformation)
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        src, tgt, result.transformation, information, uncertain=True))
    
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
    # transforms[i] are in relative-to-first-scan frame (sensor-frame clouds plug straight in)
    print("\nSaving aligned results...")
    transforms = [node.pose for node in pose_graph.nodes]

    SENSOR_CANDIDATES = [
        "sensor_colored_exact.ply", "sensor_colored_pointcloud.ply", "sensor_colored.ply",
        "world_colored_exact.ply", "world_colored_pointcloud.ply", "world_colored.ply",
    ]

    for i, scan_dir in enumerate(scan_dirs):
        T_rel = transforms[i]
        t = T_rel[:3, 3]
        euler_deg = np.degrees(Rotation.from_matrix(T_rel[:3, :3]).as_euler('xyz'))
        print(f"{scan_dir.name}:")
        print(f"  Position: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]")
        print(f"  Rotation: [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]")

        # Save refined trajectory (store relative pose so merge_with_trajectory.py can reuse it)
        traj_file = scan_dir / "trajectory.json"
        if traj_file.exists():
            with open(traj_file) as f:
                traj = json.load(f)
            traj_refined = traj.copy()
            traj_refined['current_pose'] = traj_refined.get('current_pose', {})
            traj_refined['current_pose']['lidar_pose'] = {
                'position': {'x': float(T_rel[0, 3]), 'y': float(T_rel[1, 3]), 'z': float(T_rel[2, 3])},
                'orientation': dict(zip(['x', 'y', 'z', 'w'],
                    Rotation.from_matrix(T_rel[:3, :3]).as_quat().tolist()))
            }
            with open(scan_dir / "trajectory_icp_refined.json", 'w') as f:
                json.dump(traj_refined, f, indent=2)

        # Save per-scan aligned cloud (sensor-frame points × relative transform)
        colored_file = next((scan_dir / n for n in SENSOR_CANDIDATES if (scan_dir / n).exists()), None)
        if colored_file:
            pcd_colored = o3d.io.read_point_cloud(str(colored_file))
            pcd_colored.transform(T_rel)
            o3d.io.write_point_cloud(
                str(scan_dir / f"{colored_file.stem}_aligned.ply"), pcd_colored)

    # Merge all aligned colored clouds
    print("\nCreating merged colored point cloud...")
    merged_colored = o3d.geometry.PointCloud()

    for i, scan_dir in enumerate(scan_dirs):
        colored_file = next((scan_dir / n for n in SENSOR_CANDIDATES if (scan_dir / n).exists()), None)
        if colored_file:
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
