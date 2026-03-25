#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Aligns scan point clouds using trajectory poses as initialisation and refines with ICP and pose graph optimisation. Better suited than sequential ICP for large sessions where not all scans overlap with the first.
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
import sqlite3
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

def pairwise_icp(source, target, voxel_size, init_transform,
                 source_colored=None, target_colored=None):
    """Multi-scale geometry ICP, then colored ICP fine-tuning if colored clouds provided."""
    current_transform = init_transform
    loss = o3d.pipelines.registration.TukeyLoss(k=voxel_size * 2.0)

    for max_dist, max_iter in [
        (voxel_size * 15, 50),
        (voxel_size * 5,  30),
        (voxel_size * 2,  20),
        (voxel_size * 1.0, 14),
    ]:
        result = o3d.pipelines.registration.registration_icp(
            source, target, max_dist, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter))
        current_transform = result.transformation

    if source_colored is not None and target_colored is not None:
        result = o3d.pipelines.registration.registration_colored_icp(
            source_colored, target_colored,
            voxel_size * 1.0, current_transform,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))

    return result

def _load_imu_from_bag(session_path):
    """Load /livox/imu angular velocity from the session's IMU bag.
    Returns list of (timestamp, gyro_magnitude) or empty list if unavailable."""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return []
    imu_bag = next(session_path.glob("rosbag_*_imu"), None) or \
              next(session_path.glob("rosbag_*"), None)
    if imu_bag is None:
        return []
    db3 = next(imu_bag.glob("*.db3"), None)
    if db3 is None:
        zstd = next(imu_bag.glob("*.db3.zstd"), None)
        if zstd is None:
            return []
        import subprocess
        out = str(zstd).replace(".zstd", "")
        subprocess.run(["zstd", "-d", str(zstd), "-o", out, "-f"], capture_output=True)
        db3 = Path(out)
    try:
        con = sqlite3.connect(str(db3))
        topics = {r[0]: (r[1], r[2]) for r in con.execute("SELECT id, name, type FROM topics")}
        tid = next((tid for tid, (n, _) in topics.items() if "/livox/imu" in n), None)
        if tid is None:
            con.close(); return []
        MsgType = get_message(topics[tid][1])
        rows = con.execute(
            "SELECT data, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
        ).fetchall()
        con.close()
        result = []
        for data, _ in rows:
            msg = deserialize_message(bytes(data), MsgType)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            g = msg.angular_velocity
            result.append((stamp, (g.x**2 + g.y**2 + g.z**2) ** 0.5))
        return result
    except Exception:
        return []


def _gyro_at(imu_data, t, window=0.5):
    """Mean gyro magnitude over a window centred on t. Returns None if no data."""
    samples = [g for ts, g in imu_data if abs(ts - t) <= window / 2.0]
    return float(np.mean(samples)) if samples else None


def register_pose_graph(session_dir, max_gyro=0.3):
    """Pose graph optimization with trajectory initialization."""
    
    session_path = Path(session_dir)
    scan_dirs = sorted(session_path.glob("fusion_scan_*"))
    
    if len(scan_dirs) < 2:
        print(f"Need at least 2 scans, found {len(scan_dirs)}")
        return
    
    print(f"Found {len(scan_dirs)} scan directories")

    # Load IMU data for gyro filter
    imu_data = _load_imu_from_bag(session_path)
    if imu_data:
        print(f"  Loaded {len(imu_data)} IMU samples for motion filtering")
    else:
        print("  ⚠ No IMU data found — skipping gyro filter")

    # Prefer sensor-frame clouds so we can apply the trajectory pose ourselves.
    # world_colored_* are already in absolute world frame (motion-compensated) —
    # using them here would double-transform when we apply T_rel later.
    SENSOR_FIRST = [
        "sensor_colored_exact.ply", "sensor_colored_pointcloud.ply", "sensor_colored.ply",
        "world_colored_exact.ply", "world_colored_pointcloud.ply", "world_colored.ply",
    ]
    ply_files = []
    valid_scan_dirs = []
    for scan_dir in scan_dirs:
        # Gyro filter: skip scans captured during high motion
        if imu_data:
            traj_file = scan_dir / "trajectory.json"
            if traj_file.exists():
                with open(traj_file) as f:
                    traj = json.load(f)
                capture_time = traj.get('scan_info', {}).get('capture_time') or \
                               traj.get('current_pose', {}).get('timestamp')
                if capture_time is not None:
                    gyro = _gyro_at(imu_data, float(capture_time))
                    if gyro is not None and gyro > max_gyro:
                        print(f"  Skipping {scan_dir.name}: gyro={gyro:.3f} rad/s > {max_gyro:.2f}")
                        continue
        for name in SENSOR_FIRST:
            ply = scan_dir / name
            if ply.exists():
                ply_files.append(ply)
                valid_scan_dirs.append(scan_dir)
                break

    scan_dirs = valid_scan_dirs
    
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

    # Preprocess clouds — pre-transform into the reference frame using trajectory poses
    # so ICP sees overlapping geometry and only needs to find the small residual correction.
    # The pose graph edge transformation T_ij is then the small residual in reference frame.
    voxel_size = 0.05  # 50mm matches the good session that successfully corrected diverged poses
    pcds = []         # geometry-only, pre-transformed into reference frame
    pcds_colored = [] # colored, pre-transformed into reference frame
    for i, f in enumerate(ply_files):
        pcd = o3d.io.read_point_cloud(str(f))
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd = pcd.voxel_down_sample(voxel_size)
        # Pre-transform into reference frame using trajectory pose
        pcd.transform(trajectory_poses[i])
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)

        pcd_c = o3d.io.read_point_cloud(str(f)).voxel_down_sample(voxel_size)
        pcd_c.transform(trajectory_poses[i])
        pcd_c.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        has_color = pcd_c.has_colors()

        pcd_geom = o3d.geometry.PointCloud(pcd)
        pcd_geom.colors = o3d.utility.Vector3dVector()
        pcds.append(pcd_geom)
        pcds_colored.append(pcd_c if has_color else None)
        print(f"Preprocessed {f.parent.name}: {len(pcd.points)} points  color={'yes' if has_color else 'no'}")

    # Build pose graph — nodes start at identity since clouds are already in reference frame.
    # ICP finds a small residual correction T_ij (near-identity) between overlapping clouds.
    # The final node pose = identity @ T_ij_chain, which global_optimization refines.
    print("\nBuilding pose graph...")
    pose_graph = o3d.pipelines.registration.PoseGraph()

    for _ in trajectory_poses:
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(np.eye(4)))

    n_scans = len(pcds)

    # Sequential edges — clouds are already aligned so init is identity
    for i in range(n_scans - 1):
        print(f"\nRefining edge {i} -> {i+1} (sequential)")
        result = pairwise_icp(pcds[i], pcds[i+1], voxel_size, np.eye(4),
                              pcds_colored[i], pcds_colored[i+1])
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
            result = pairwise_icp(pcds[src], pcds[tgt], voxel_size, np.eye(4),
                                  pcds_colored[src], pcds_colored[tgt])
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
    
    # transforms[i] are residual corrections in reference frame.
    # Full transform for sensor-frame cloud i = trajectory_poses[i] @ transforms[i]
    print("\nSaving aligned results...")
    transforms = [node.pose for node in pose_graph.nodes]
    full_transforms = [trajectory_poses[i] @ transforms[i] for i in range(len(transforms))]

    for i, scan_dir in enumerate(scan_dirs):
        T_rel = full_transforms[i]
        t = T_rel[:3, 3]
        euler_deg = np.degrees(Rotation.from_matrix(T_rel[:3, :3]).as_euler('xyz'))
        print(f"{scan_dir.name}:")
        print(f"  Position: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]")
        print(f"  Rotation: [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]")

        traj_file = scan_dir / "trajectory.json"
        if traj_file.exists():
            with open(traj_file) as f:
                traj = json.load(f)
            traj_refined = traj.copy()
            # Store the ICP-refined pose in current_pose.lidar_pose so
            # pose_matrix_from_trajectory() and merge_with_trajectory.py read it correctly.
            # T_rel is already in relative-to-first-scan frame.
            q_refined = Rotation.from_matrix(T_rel[:3, :3]).as_quat()
            traj_refined['current_pose'] = dict(traj_refined.get('current_pose', {}))
            traj_refined['current_pose']['lidar_pose'] = {
                'position': {'x': float(T_rel[0, 3]), 'y': float(T_rel[1, 3]), 'z': float(T_rel[2, 3])},
                'orientation': {'x': float(q_refined[0]), 'y': float(q_refined[1]),
                                'z': float(q_refined[2]), 'w': float(q_refined[3])}
            }
            traj_refined['scan_info'] = dict(traj_refined.get('scan_info', {}))
            traj_refined['scan_info']['icp_refined'] = True
            with open(scan_dir / "trajectory_icp_refined.json", 'w') as f:
                json.dump(traj_refined, f, indent=2)

        colored_file = next((scan_dir / n for n in SENSOR_FIRST if (scan_dir / n).exists()), None)
        if colored_file:
            pcd_colored = o3d.io.read_point_cloud(str(colored_file))
            pcd_colored.transform(T_rel)
            o3d.io.write_point_cloud(
                str(scan_dir / f"{colored_file.stem}_aligned.ply"), pcd_colored)

    print("\nCreating merged colored point cloud...")
    merged_colored = o3d.geometry.PointCloud()

    for i, scan_dir in enumerate(scan_dirs):
        colored_file = next((scan_dir / n for n in SENSOR_FIRST if (scan_dir / n).exists()), None)
        if colored_file:
            pcd = o3d.io.read_point_cloud(str(colored_file))
            pcd.transform(full_transforms[i])
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
