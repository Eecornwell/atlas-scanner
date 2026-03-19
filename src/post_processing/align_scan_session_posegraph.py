#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Aligns scan point clouds using trajectory poses as initialisation and refines with ICP and pose graph optimisation. Better suited than sequential ICP for large sessions where not all scans overlap with the first.
"""
Pose graph optimization for point cloud alignment.

Uses trajectory poses as initialization and refines with ICP + pose graph optimization.
Global registration (FPFH+RANSAC) is used as fallback on raw sensor-frame clouds
when trajectory-initialised ICP gives low fitness.
"""

import sys
from pathlib import Path
import open3d as o3d
import numpy as np
import json
import sqlite3
import struct
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

    pos = p.get('position', {})
    ori = p.get('orientation', {})
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(
        [ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)]
    ).as_matrix()
    T[:3, 3] = [pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)]
    return T


def load_trajectory_pose(scan_dir):
    """Load pose from current_pose.lidar_pose — already interpolated at pose_stamp
    (median LiDAR timestamp) by reconstruct_from_bag.py.
    Never re-interpolate from full_trajectory: capture_time is the camera frame
    timestamp, not the pose timestamp, so re-interpolating would undo the fix."""
    traj_file = scan_dir / "trajectory.json"
    if traj_file.exists():
        with open(traj_file) as f:
            traj = json.load(f)
        cp = traj.get('current_pose', {})
        lp = cp.get('lidar_pose', cp)
        pos = lp.get('position', {})
        ori = lp.get('orientation', {})
        if pos and ori and ori.get('w', 0) != 0:
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


def preprocess_cloud(pcd_raw, voxel_size):
    """Downsample, remove outliers, estimate normals. Returns geometry-only cloud."""
    pcd, _ = pcd_raw.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.voxel_down_sample(voxel_size)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(30)
    pcd_geom = o3d.geometry.PointCloud(pcd)
    pcd_geom.colors = o3d.utility.Vector3dVector()
    return pcd_geom


def global_registration(source, target, voxel_size):
    """FPFH+RANSAC global registration on raw sensor-frame clouds."""
    radius_feature = voxel_size * 5
    src_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    tgt_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, src_fpfh, tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=voxel_size * 3,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 3),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def pairwise_icp(source, target, voxel_size, init_transform,
                 source_colored=None, target_colored=None):
    """Multi-scale geometry ICP. Returns the geometry ICP result.
    Colored ICP is intentionally not used: its fitness metric mixes color and
    geometry so it is incomparable to the geometry-only fitness threshold used
    in best_pairwise, and it frequently diverges when residual misalignment
    after geometry ICP exceeds the tight 0.03 m correspondence distance."""
    current_transform = init_transform
    loss = o3d.pipelines.registration.TukeyLoss(k=voxel_size * 2.0)

    for max_dist, max_iter in [
        (voxel_size * 15, 50),
        (voxel_size * 5,  30),
        (voxel_size * 2,  20),
        (voxel_size * 0.8, 20),
    ]:
        result = o3d.pipelines.registration.registration_icp(
            source, target, max_dist, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter))
        current_transform = result.transformation

    return result


def best_pairwise(src_raw, tgt_raw, voxel_size, traj_init):
    """Try trajectory-init ICP first; fall back to global+ICP if fitness < 0.40."""
    result = pairwise_icp(src_raw, tgt_raw, voxel_size, traj_init)
    if result.fitness >= 0.40:
        return result, 'traj'

    print(f"  Low fitness ({result.fitness:.3f}) — retrying with global registration...")
    global_result = global_registration(src_raw, tgt_raw, voxel_size)
    if global_result.fitness > result.fitness:
        refined = pairwise_icp(src_raw, tgt_raw, voxel_size, global_result.transformation)
        print(f"  After global+ICP: Fitness: {refined.fitness:.3f}, RMSE: {refined.inlier_rmse:.4f}")
        if refined.fitness > result.fitness:
            return refined, 'global'
    return result, 'traj'


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
        subprocess.run(["zstd", "-d", str(zstd), "-o", out, "-f"],
                       capture_output=True)
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
        for data, bag_ts_ns in rows:
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
    print(f"  Gyro filter: max_gyro={max_gyro:.2f} rad/s")

    session_path = Path(session_dir)
    scan_dirs = sorted(session_path.glob("fusion_scan_*"))

    if len(scan_dirs) < 2:
        print(f"Need at least 2 scans, found {len(scan_dirs)}")
        return

    print(f"Found {len(scan_dirs)} scan directories")

    # Load IMU data for gyro filter (motion during capture)
    imu_data = _load_imu_from_bag(session_path)
    if imu_data:
        print(f"  Loaded {len(imu_data)} IMU samples for motion filtering")
    else:
        print("  ⚠ No IMU data found — skipping gyro filter")

    SENSOR_FIRST = [
        "sensor_colored_exact.ply", "sensor_colored_pointcloud.ply", "sensor_colored.ply",
        "world_colored_exact.ply", "world_colored_pointcloud.ply", "world_colored.ply",
    ]
    ply_files = []
    valid_scan_dirs = []
    for scan_dir in scan_dirs:
        # Check gyro at this scan's capture time
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

    # Filter out SLAM-unconverged scans using pose consistency.
    # RKO-LIO drifts during initialisation — the first few scans often have
    # positions scattered far from the stable cluster. Detect this by finding
    # the median position across all scans and dropping any scan whose position
    # is more than 3x the median absolute deviation (MAD) from the median.
    # MAD is robust to outliers so a minority of bad scans won't corrupt it.
    positions = np.array([T[:3, 3] for T in traj_poses_abs])
    median_pos = np.median(positions, axis=0)
    dists = np.linalg.norm(positions - median_pos, axis=1)
    mad = np.median(dists)
    # Use a minimum floor of 0.15m so we don't filter in tight stationary sessions
    drift_threshold = max(3.0 * mad, 0.15)
    filtered_dirs, filtered_poses, filtered_plys = [], [], []
    for i, (scan_dir, pose, ply) in enumerate(zip(scan_dirs, traj_poses_abs, ply_files)):
        if dists[i] > drift_threshold:
            print(f"  Skipping {scan_dir.name}: pose dist={dists[i]:.3f}m from median "
                  f"(threshold={drift_threshold:.3f}m, MAD={mad:.3f}m) — likely SLAM drift")
        else:
            filtered_dirs.append(scan_dir)
            filtered_poses.append(pose)
            filtered_plys.append(ply)
    scan_dirs, traj_poses_abs, ply_files = filtered_dirs, filtered_poses, filtered_plys

    if len(ply_files) < 2:
        print(f"Need at least 2 scans after filtering, found {len(ply_files)}")
        return

    T_first_inv = np.linalg.inv(traj_poses_abs[0])
    # trajectory_poses[i] = pose of scan i relative to scan 0 (reference frame)
    trajectory_poses = [T_first_inv @ T for T in traj_poses_abs]

    voxel_size = 0.03

    # Pre-transform every cloud into reference frame using its trajectory pose.
    # ICP then only needs to find small residual corrections rather than large
    # rotations, which is critical for scans with large orientation changes
    # (e.g. w=0.13 quaternions = ~165° rotation) where sensor-frame clouds share
    # no geometric overlap and ICP cannot converge from a cold start.
    pcds_ref = []  # geometry-only clouds in reference frame
    for i, f in enumerate(ply_files):
        pcd_raw = o3d.io.read_point_cloud(str(f))
        pcd = preprocess_cloud(pcd_raw, voxel_size)
        pcd.transform(trajectory_poses[i])
        pcds_ref.append(pcd)
        print(f"Preprocessed {f.parent.name}: {len(pcd.points)} points")

    # Pose graph nodes: identity init because clouds are already in reference frame.
    # ICP refines the residual; the node pose accumulates the correction.
    print("\nBuilding pose graph...")
    pose_graph = o3d.pipelines.registration.PoseGraph()
    for _ in trajectory_poses:
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))

    n_scans = len(pcds_ref)

    TRAJ_PRIOR_INFO = np.eye(6) * 1e4
    ICP_FITNESS_THRESHOLD = 0.40

    # Count reliable ICP edges before deciding whether to run the optimizer.
    # If fewer than half the sequential edges are reliable, the pose graph
    # has too many trajectory-prior edges to improve on the SLAM poses —
    # in that case skip optimization and fall back to the trajectory merge.
    icp_edge_count = 0
    icp_edges = []  # (i, j, transformation, information)
    traj_edges = []  # (i, j) — will use identity transform + prior

    for i in range(n_scans - 1):
        print(f"\nRefining edge {i} -> {i+1} (sequential)")
        result, method = best_pairwise(
            pcds_ref[i], pcds_ref[i + 1], voxel_size, np.eye(4))
        print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}  [{method}]")
        if result.fitness >= ICP_FITNESS_THRESHOLD:
            information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                pcds_ref[i], pcds_ref[i + 1], voxel_size * 1.5, result.transformation)
            icp_edges.append((i, i + 1, result.transformation, information))
            icp_edge_count += 1
        else:
            print(f"  ICP unreliable — using trajectory prior for edge {i}->{i+1}")
            traj_edges.append((i, i + 1))

    reliable_ratio = icp_edge_count / (n_scans - 1)
    print(f"\nReliable ICP edges: {icp_edge_count}/{n_scans - 1} ({reliable_ratio:.0%})")

    if reliable_ratio < 0.5:
        print("  ⚠ Fewer than 50% of sequential edges are reliable — "
              "ICP cannot improve on SLAM poses. Skipping pose graph optimization.")
        print("  The trajectory-based merge (merged_pointcloud.ply) is the best result.")
        return

    # Enough reliable edges — build and optimize the pose graph
    for i, j, transformation, information in icp_edges:
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                i, j, transformation, information, uncertain=False))
    for i, j in traj_edges:
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                i, j, np.eye(4), TRAJ_PRIOR_INFO, uncertain=False))

    # KNN loop-closure edges
    centers_ref = [np.asarray(p.points).mean(axis=0) if len(p.points) else np.zeros(3)
                   for p in pcds_ref]

    added_pairs = {(i, i + 1) for i in range(n_scans - 1)}
    for i in range(n_scans):
        dists = sorted(
            [(j, np.linalg.norm(centers_ref[i] - centers_ref[j]))
             for j in range(n_scans) if abs(i - j) > 1],
            key=lambda x: x[1])
        for j, dist in dists[:3]:
            if dist > 2.0:
                break
            pair = (min(i, j), max(i, j))
            if pair in added_pairs:
                continue
            added_pairs.add(pair)
            src, tgt = pair
            print(f"\nRefining edge {src} -> {tgt} (KNN, dist={dist:.2f}m)")
            result, method = best_pairwise(
                pcds_ref[src], pcds_ref[tgt], voxel_size, np.eye(4))
            print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}  [{method}]")
            if result.fitness >= ICP_FITNESS_THRESHOLD:
                information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    pcds_ref[src], pcds_ref[tgt], voxel_size * 1.5, result.transformation)
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        src, tgt, result.transformation, information, uncertain=False))

    print(f"\nOptimizing pose graph ({len(pose_graph.nodes)} nodes, {len(pose_graph.edges)} edges)...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=voxel_size * 1.5,
        edge_prune_threshold=0.1,
        reference_node=0)
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
    print("Optimization complete!")

    # Node poses after optimization are residual corrections in reference frame.
    # Full transform = trajectory_pose @ residual_correction.
    print("\nSaving aligned results...")
    full_transforms = [trajectory_poses[i] @ node.pose
                       for i, node in enumerate(pose_graph.nodes)]

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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir")
    parser.add_argument("--max-gyro", type=float, default=0.3,
                        help="Max gyro magnitude (rad/s) to include a scan (default: 0.3)")
    args = parser.parse_args()
    register_pose_graph(args.session_dir, max_gyro=args.max_gyro)


if __name__ == "__main__":
    main()
