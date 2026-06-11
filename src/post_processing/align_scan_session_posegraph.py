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
import os
from pathlib import Path
import open3d as o3d
import numpy as np
import json
import sqlite3
from scipy.spatial.transform import Rotation, Slerp

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


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
        try:
            traj_file = _safe_data(traj_file)
        except ValueError:
            return np.eye(4)
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
        try:
            metadata_file = _safe_data(metadata_file)
        except ValueError:
            return np.eye(4)
        with open(metadata_file) as f:
            metadata = json.load(f)
        if 'trajectory_pose' in metadata:
            pd = metadata['trajectory_pose']
            T = np.eye(4)
            T[:3, :3] = np.array(pd['rotation']).reshape(3, 3)
            T[:3, 3] = np.array(pd['translation'])
            return T

    return np.eye(4)

def coarse_icp(source, target, voxel_size, init_transform=None):
    """Coarse ICP using trajectory pose as initialization."""
    if init_transform is None:
        init_transform = np.eye(4)
    # Use point-to-point for coarse stage (more robust when normals are noisy)
    result = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size * 4, init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
    # Refine with point-to-plane
    loss = o3d.pipelines.registration.TukeyLoss(k=voxel_size * 2.0)
    result = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size * 2, result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
    return result


def fine_icp(source, target, voxel_size, init_transform):
    """Fine ICP starting from a good coarse initialisation.
    Runs tight Tukey stages to sub-voxel accuracy.
    """
    current_transform = init_transform
    for max_dist, max_iter, tukey_k in [
        (voxel_size * 2,  30, voxel_size * 1.0),
        (voxel_size * 1.0, 30, voxel_size * 0.5),
        (voxel_size * 0.5, 20, voxel_size * 0.25),
    ]:
        loss = o3d.pipelines.registration.TukeyLoss(k=tukey_k)
        result = o3d.pipelines.registration.registration_icp(
            source, target, max_dist, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter))
        current_transform = result.transformation
    return result


def pairwise_icp(source, target, voxel_size, init_transform,
                 source_colored=None, target_colored=None):
    """Two-stage ICP: coarse pass initialized from trajectory, fine pass to refine.
    Fitness is evaluated at 2x voxel_size after fine ICP for a fair comparison.
    """
    coarse = coarse_icp(source, target, voxel_size, init_transform)

    MIN_FITNESS_FOR_FINE = 0.15
    if coarse.fitness < MIN_FITNESS_FOR_FINE:
        # Re-evaluate fitness at 2x voxel_size for consistent comparison
        eval_result = o3d.pipelines.registration.evaluate_registration(
            source, target, voxel_size * 2, coarse.transformation)
        coarse = type('R', (), {'fitness': eval_result.fitness,
                                'inlier_rmse': eval_result.inlier_rmse,
                                'transformation': coarse.transformation})()
        return coarse

    result = fine_icp(source, target, voxel_size, coarse.transformation)
    # Re-evaluate fitness at 2x voxel_size so it's not penalized by the tight fine radius
    eval_result = o3d.pipelines.registration.evaluate_registration(
        source, target, voxel_size * 2, result.transformation)
    result = type('R', (), {'fitness': eval_result.fitness,
                            'inlier_rmse': eval_result.inlier_rmse,
                            'transformation': result.transformation})()
    return result

def _load_imu_from_bag(session_path):
    """Load /livox/imu angular velocity from the session's IMU bag.
    Checks session-level bags first (continuous mode), then per-scan bags
    (stationary mode where each scan has its own rosbag subdirectory).
    Returns list of (timestamp, gyro_magnitude) or empty list if unavailable."""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return []

    # Collect candidate bag directories: session-level first, then per-scan
    candidates = list(session_path.glob("rosbag_*_imu")) + \
                 list(session_path.glob("rosbag_*")) + \
                 [b for sd in sorted(session_path.glob("fusion_scan_*"))
                  for b in sorted(sd.glob("rosbag_*"))]
    if not candidates:
        return []

    def _read_imu_db3(db3):
        con = sqlite3.connect(str(db3))
        topics = {r[0]: (r[1], r[2]) for r in con.execute("SELECT id, name, type FROM topics")}
        tid = next((tid for tid, (n, _) in topics.items() if "/livox/imu" in n), None)
        if tid is None:
            con.close()
            return []
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

    all_samples = []
    for imu_bag in candidates:
        db3 = next(imu_bag.glob("*.db3"), None)
        if db3 is None:
            zstd = next(imu_bag.glob("*.db3.zstd"), None)
            if zstd is None:
                continue
            import subprocess
            out = str(zstd).replace(".zstd", "")
            subprocess.run(["zstd", "-d", str(zstd), "-o", out, "-f"], capture_output=True)
            db3 = Path(out)
        try:
            all_samples.extend(_read_imu_db3(db3))
        except Exception:
            continue

    # Deduplicate by timestamp and sort
    seen = set()
    result = []
    for ts, g in sorted(all_samples):
        if ts not in seen:
            seen.add(ts)
            result.append((ts, g))
    return result


def _gyro_at(imu_data, t, window=0.2):
    """75th-percentile gyro magnitude over a window centred on t."""
    samples = [g for ts, g in imu_data if abs(ts - t) <= window / 2.0]
    return float(np.percentile(samples, 75)) if samples else None


def register_pose_graph(session_dir, max_gyro=0.25, iterations=1):
    """Pose graph optimization with trajectory initialization."""

    def pose_to_T_from_file(traj_file):
        return load_trajectory_pose(Path(traj_file).parent)
    
    session_path = Path(session_dir)
    try:
        session_path = _safe_data(session_path)
    except ValueError as e:
        print(f"Error: {e}")
        return
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

    # Use world_lidar.ply for ICP — already in absolute odom world frame.
    # Transform to relative-to-first using T_first_inv so all clouds share
    # the same coordinate frame. This gives much better ICP fitness than
    # sensor-frame clouds which require large trajectory pre-transforms.
    LIDAR_CANDIDATES = ["world_lidar.ply", "sensor_lidar.ply"]
    COLORED_CANDIDATES = [
        "sensor_colored_exact.ply", "sensor_colored_pointcloud.ply", "sensor_colored.ply",
        "world_colored_exact.ply", "world_colored_pointcloud.ply", "world_colored.ply",
    ]
    lidar_files = []
    ply_files = []  # colored, for output
    valid_scan_dirs = []
    for scan_dir in scan_dirs:
        lidar = next((scan_dir / n for n in LIDAR_CANDIDATES if (scan_dir / n).exists()), None)
        colored = next((scan_dir / n for n in COLORED_CANDIDATES if (scan_dir / n).exists()), None)
        if lidar and colored:
            # Gyro filter
            if imu_data:
                traj_file = scan_dir / "trajectory.json"
                if traj_file.exists():
                    try:
                        traj_file = _safe_data(traj_file)
                    except ValueError:
                        continue
                    with open(traj_file) as f:
                        traj = json.load(f)
                    capture_time = traj.get('scan_info', {}).get('capture_time') or \
                                   traj.get('current_pose', {}).get('timestamp')
                    if capture_time is not None:
                        gyro = _gyro_at(imu_data, float(capture_time))
                        if gyro is not None and gyro > max_gyro:
                            print(f"  Skipping {scan_dir.name}: gyro={gyro:.3f} rad/s > {max_gyro:.2f}")
                            continue
            lidar_files.append(lidar)
            ply_files.append(colored)
            valid_scan_dirs.append(scan_dir)

    scan_dirs = valid_scan_dirs

    # Drop scans whose trajectory position is too close to an already-kept scan.
    # Near-duplicate poses add no new geometry and produce degenerate baselines for
    # Gaussian splatting. 10 cm is a safe minimum for a stationary scanner.
    MIN_BASELINE_M = 0.10
    kept_indices = []
    kept_positions = []
    for i, scan_dir in enumerate(scan_dirs):
        pos = load_trajectory_pose(scan_dir)[:3, 3]
        if not kept_positions or min(np.linalg.norm(pos - p) for p in kept_positions) >= MIN_BASELINE_M:
            kept_indices.append(i)
            kept_positions.append(pos)
        else:
            print(f"  Skipping {scan_dir.name}: too close to an existing scan (< {MIN_BASELINE_M*100:.0f}cm)")
    scan_dirs   = [scan_dirs[i]   for i in kept_indices]
    lidar_files = [lidar_files[i] for i in kept_indices]
    ply_files   = [ply_files[i]   for i in kept_indices]

    if len(lidar_files) < 2:
        print(f"Need at least 2 point clouds, found {len(lidar_files)}")
        return

    # Load trajectory poses for output only (not for ICP pre-transform)
    traj_poses_abs = []
    for scan_dir in scan_dirs:
        pose = load_trajectory_pose(scan_dir)
        traj_poses_abs.append(pose)
        print(f"Trajectory {scan_dir.name}: t=[{pose[0,3]:.3f}, {pose[1,3]:.3f}, {pose[2,3]:.3f}]")


    T_first_inv = np.linalg.inv(traj_poses_abs[0])
    trajectory_poses = [T_first_inv @ T for T in traj_poses_abs]

    # Preprocess world_lidar clouds.
    # world_lidar is in absolute odom world frame. Transform to relative-to-first-scan
    # frame (same as merge_with_trajectory.py) so ICP corrections are small residuals
    # Use world_lidar.ply for ICP (already motion-compensated world frame).
    # Transform to relative-to-first for ICP, then apply the same correction
    # to the colored clouds (also transformed to relative-to-first via trajectory_poses).
    voxel_size = 0.05  # 5cm voxels for ICP reference matching
    pcds = []
    pcds_colored = []
    for i, f in enumerate(lidar_files):
        pcd = o3d.io.read_point_cloud(str(f))
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd = pcd.voxel_down_sample(voxel_size)
        # world_lidar.ply is already in absolute odom world frame.
        # Apply only T_first_inv to bring all clouds into relative-to-first frame.
        # DO NOT apply trajectory_poses[i] again — world_lidar already has it baked in.
        pcd.transform(T_first_inv)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)
        pcds.append(pcd)

        pcd_c = o3d.io.read_point_cloud(str(ply_files[i])).voxel_down_sample(voxel_size)
        pcd_c.transform(trajectory_poses[i])  # sensor -> relative-to-first
        pcd_c.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcds_colored.append(pcd_c if pcd_c.has_colors() else None)

        print(f"Preprocessed {lidar_files[i].parent.name}: {len(pcd.points)} points  color={'yes' if pcd_c.has_colors() else 'no'}")

    # Iterative pose graph refinement — each pass uses the previous corrected poses
    # as initialization, so ICP finds progressively smaller residuals.
    for iteration in range(iterations):
        print(f"\n=== Pose graph iteration {iteration+1}/{iterations} ===")

        # Rebuild pcds from accumulated trajectory_poses on iterations > 0
        if iteration > 0:
            pcds = []
            for i, f in enumerate(lidar_files):
                pcd = o3d.io.read_point_cloud(str(f))
                pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                pcd = pcd.voxel_down_sample(voxel_size)
                # world_lidar in abs frame; apply T_icp_prev @ T_first_inv to refine
                T_prev_corr = icp_corrections[i]  # refined relative-to-first pose
                # world_pts -> T_first_inv -> rel frame -> T_icp_corr -> refined
                # = T_icp_corr @ T_first_inv @ world_pts
                # But T_icp_corr already IS the refined relative-to-first pose.
                # So we need T_icp_corr @ inv(T_first_inv @ T_abs) @ T_first_inv
                # = T_icp_corr @ T_first_inv since world = T_abs @ sensor
                pcd.transform(T_first_inv)  # world -> rel-to-first
                # Apply the delta between icp_correction and trajectory_pose
                T_delta = T_prev_corr @ np.linalg.inv(trajectory_poses[i])
                pcd.transform(T_delta)  # apply the ICP correction delta
                pcd.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
                pcd.orient_normals_consistent_tangent_plane(30)
                pcds.append(pcd)

        n_scans = len(pcds)
        pose_graph = o3d.pipelines.registration.PoseGraph()
        for _ in scan_dirs:
            pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))

        # Sequential edges.
        # world_lidar.ply clouds are already in world frame and brought to
        # relative-to-first by T_first_inv, so they are already roughly aligned.
        # Use identity init — trajectory_poses include scanner tilt which is
        # NOT present in world_lidar (motion-compensated per-frame).
        seq_edge_fitnesses = []
        for i in range(n_scans - 1):
            print(f"\nRefining edge {i} -> {i+1} (sequential)")
            # Initialize ICP with the trajectory relative pose to avoid local minima.
            # The trajectory gives the approximate relative transform; ICP refines it.
            T_traj_rel = trajectory_poses[i+1] @ np.linalg.inv(trajectory_poses[i])
            result = pairwise_icp(pcds[i], pcds[i+1], voxel_size, T_traj_rel)
            print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
            T_icp = result.transformation
            if result.fitness < 0.25:
                print(f"  ⚠ Low fitness ({result.fitness:.3f}) — using identity")
                T_icp = np.eye(4)
            else:
                seq_edge_fitnesses.append(result.fitness)
            information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                pcds[i], pcds[i+1], voxel_size * 2, T_icp)
            information *= 10.0
            pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                i, i+1, T_icp, information, uncertain=False))

        # KNN loop-closure edges — use 3 neighbors and extend range to 2.0m
        centers = [np.mean(np.asarray(pcd.points), axis=0) for pcd in pcds]
        added_pairs = {(i, i+1) for i in range(n_scans - 1)}
        for i in range(n_scans):
            dists = sorted([(j, np.linalg.norm(centers[i] - centers[j]))
                            for j in range(n_scans) if abs(i-j) > 1], key=lambda x: x[1])
            for j, dist in dists[:3]:
                if dist > 2.0:
                    break
                pair = (min(i,j), max(i,j))
                if pair in added_pairs:
                    continue
                added_pairs.add(pair)
                src, tgt = pair
                print(f"\nLoop closure edge {src} -> {tgt} (dist={dist:.2f}m)")
                T_traj_rel_lc = trajectory_poses[tgt] @ np.linalg.inv(trajectory_poses[src])
                result = pairwise_icp(pcds[src], pcds[tgt], voxel_size, T_traj_rel_lc)
                print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
                if result.fitness > 0.25:
                    T_icp = result.transformation
                    information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                        pcds[src], pcds[tgt], voxel_size * 2, T_icp)
                    pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                        src, tgt, T_icp, information, uncertain=True))

        seq_good = sum(1 for e in pose_graph.edges if not e.uncertain)
        mean_seq_fitness = np.mean(seq_edge_fitnesses) if seq_edge_fitnesses else 0.0
        print(f"\nSequential edges: {seq_good}/{n_scans-1} passed  mean_fitness={mean_seq_fitness:.3f}")

        if mean_seq_fitness < 0.05 or n_scans < 3:
            print("⚠ Insufficient ICP quality — using trajectory poses directly")
            icp_corrections = list(trajectory_poses)
        else:
            print(f"Refining poses: leave-one-out ICP against merged reference...")
            # Each scan is refined against the merged geometry of ALL OTHER scans.
            # The cloud is pre-transformed by the trajectory pose, so ICP from
            # identity finds ONLY the small residual correction needed.
            # Corrected pose = T_icp_residual @ trajectory_pose[i]
            icp_corrections = [trajectory_poses[0]]  # scan_001 is reference
            for i in range(1, n_scans):
                # Build reference from all scans except current
                ref = o3d.geometry.PointCloud()
                for j, p in enumerate(pcds):
                    if j != i:
                        ref = ref + p
                ref = ref.voxel_down_sample(voxel_size)
                ref.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

                # Two-stage ICP from identity — cloud already in trajectory frame.
                # Stage 1: coarse pass with a wide search radius to handle trajectory
                # drift up to ~50 cm (common in continuous handheld mode).
                # Stage 2: fine pass at voxel_size to lock in sub-voxel accuracy.
                coarse_dist = voxel_size * 8  # ~40 cm for voxel_size=0.05
                result = o3d.pipelines.registration.registration_icp(
                    pcds[i], ref, coarse_dist, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
                if result.fitness >= 0.15:
                    loss = o3d.pipelines.registration.TukeyLoss(k=voxel_size * 1.0)
                    result = o3d.pipelines.registration.registration_icp(
                        pcds[i], ref, voxel_size * 2, result.transformation,
                        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))

                T_icp = result.transformation
                rot_deg = np.degrees(np.arccos(np.clip((np.trace(T_icp[:3,:3]) - 1) / 2, -1, 1)))
                trans_m = np.linalg.norm(T_icp[:3, 3])
                euler = np.degrees(Rotation.from_matrix(T_icp[:3,:3]).as_euler('xyz'))

                if result.fitness < 0.15 or rot_deg > 20.0 or trans_m > 1.0:
                    print(f"  ⚠ {scan_dirs[i].name}: rejected fitness={result.fitness:.3f} rot={rot_deg:.1f}deg trans={trans_m*100:.0f}cm")
                    icp_corrections.append(trajectory_poses[i])
                else:
                    # T_icp is the residual correction in the already-trajectory-aligned frame.
                    # Apply it on top of the trajectory pose.
                    T_corr = T_icp @ trajectory_poses[i]
                    print(f"  ✓ {scan_dirs[i].name}: fitness={result.fitness:.3f}  "
                          f"t=({T_icp[0,3]*100:+.1f},{T_icp[1,3]*100:+.1f},{T_icp[2,3]*100:+.1f})cm  "
                          f"euler=({euler[0]:+.1f},{euler[1]:+.1f},{euler[2]:+.1f})deg")
                    icp_corrections.append(T_corr)
            print("Refinement complete!")

    # icp_corrections[i] holds the final absolute pose in relative-to-first frame
    # (either the optimized pose from the pose graph, or the trajectory pose if reverted).
    full_transforms = icp_corrections

    for i, scan_dir in enumerate(scan_dirs):
        T_rel = full_transforms[i]
        t = T_rel[:3, 3]
        euler_deg = np.degrees(Rotation.from_matrix(T_rel[:3, :3]).as_euler('xyz'))
        # correction_applied: optimized pose differs from trajectory pose
        T_delta = T_rel @ np.linalg.inv(trajectory_poses[i])
        correction_applied = not np.allclose(T_delta, np.eye(4), atol=1e-4)
        print(f"{scan_dir.name}: {'ICP corrected' if correction_applied else 'trajectory (identity)'}")
        print(f"  Position: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]")
        print(f"  Rotation: [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]")

        traj_file = scan_dir / "trajectory.json"
        if traj_file.exists():
            try:
                traj_file = _safe_data(traj_file)
            except ValueError:
                continue
            with open(traj_file) as f:
                traj = json.load(f)
            traj_refined = traj.copy()
            # icp_corrections[i] is the corrected pose in relative-to-first frame.
            # Convert back to absolute odom frame for storage, matching
            # the original trajectory.json convention so pose_from_trajectory()
            # computes T_first_inv @ T_abs correctly.
            T_abs_corrected = traj_poses_abs[0] @ icp_corrections[i]
            q_refined = Rotation.from_matrix(T_abs_corrected[:3, :3]).as_quat()
            traj_refined['current_pose'] = dict(traj_refined.get('current_pose', {}))
            traj_refined['current_pose']['lidar_pose'] = {
                'position': {'x': float(T_abs_corrected[0, 3]), 'y': float(T_abs_corrected[1, 3]), 'z': float(T_abs_corrected[2, 3])},
                'orientation': {'x': float(q_refined[0]), 'y': float(q_refined[1]),
                                'z': float(q_refined[2]), 'w': float(q_refined[3])}
            }
            # Recompute camera_pose from the refined lidar pose using calibration.
            # T_world_camera = T_world_lidar @ T_lidar_camera
            try:
                import yaml
                calib_path = Path.home() / 'atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml'
                with open(calib_path) as cf:
                    calib = yaml.safe_load(cf)
                T_lidar_camera = np.eye(4)
                T_lidar_camera[:3, :3] = Rotation.from_euler('xyz', [
                    calib['roll_offset'], calib['pitch_offset'], calib['yaw_offset']
                ]).as_matrix()
                T_lidar_camera[:3, 3] = [calib['x_offset'], calib['y_offset'], calib['z_offset']]
                T_world_camera = T_rel @ T_lidar_camera
                q_cam = Rotation.from_matrix(T_world_camera[:3, :3]).as_quat()
                traj_refined['current_pose']['camera_pose'] = {
                    'position': {'x': float(T_world_camera[0, 3]), 'y': float(T_world_camera[1, 3]),
                                 'z': float(T_world_camera[2, 3])},
                    'orientation': {'x': float(q_cam[0]), 'y': float(q_cam[1]),
                                    'z': float(q_cam[2]), 'w': float(q_cam[3])}
                }
            except Exception as e:
                print(f'  ⚠ Could not update camera_pose: {e}')
            traj_refined['scan_info'] = dict(traj_refined.get('scan_info', {}))
            traj_refined['scan_info']['icp_refined'] = True
            traj_refined['scan_info']['icp_correction_applied'] = not np.allclose(
                icp_corrections[i], np.eye(4), atol=1e-6)
            try:
                out_file = _safe_data(scan_dir / "trajectory_icp_refined.json")
            except ValueError:
                continue
            with open(out_file, 'w') as f:
                json.dump(traj_refined, f, indent=2)

        colored_file = next((scan_dir / n for n in COLORED_CANDIDATES if (scan_dir / n).exists()), None)
        if colored_file:
            pcd_colored = o3d.io.read_point_cloud(str(colored_file))
            pcd_colored.transform(T_rel)
            o3d.io.write_point_cloud(
                str(scan_dir / f"{colored_file.stem}_aligned.ply"), pcd_colored)

    print("\nCreating merged colored point cloud...")
    merged_colored = o3d.geometry.PointCloud()

    # Include gyro-filtered scans with their full transforms
    for i, scan_dir in enumerate(scan_dirs):
        colored_file = next((scan_dir / n for n in COLORED_CANDIDATES if (scan_dir / n).exists()), None)
        if colored_file:
            pcd = o3d.io.read_point_cloud(str(colored_file))
            pcd.transform(full_transforms[i])
            merged_colored = merged_colored + pcd

    # Include gyro-skipped scans using trajectory pose
    all_scan_dirs = sorted(session_path.glob('fusion_scan_*'))
    skipped_dirs = [d for d in all_scan_dirs if d not in scan_dirs]
    for scan_dir in skipped_dirs:
        colored_file = next((scan_dir / n for n in COLORED_CANDIDATES if (scan_dir / n).exists()), None)
        if not colored_file:
            continue
        traj_file = scan_dir / 'trajectory.json'
        if not traj_file.exists():
            continue
        T_abs = pose_to_T_from_file(traj_file)
        T_rel = T_first_inv @ T_abs
        pcd = o3d.io.read_point_cloud(str(colored_file))
        pcd.transform(T_rel)
        merged_colored = merged_colored + pcd

    if len(merged_colored.points) > 0:
        merged_colored = merged_colored.voxel_down_sample(0.005)
        merged_colored_file = session_path / "merged_aligned_colored.ply"
        o3d.io.write_point_cloud(str(merged_colored_file), merged_colored)
        print(f"Merged: {merged_colored_file} ({len(merged_colored.points)} points)")

def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir")
    parser.add_argument("--max-gyro", type=float, default=0.15)
    parser.add_argument("--iterations", type=int, default=1)
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    register_pose_graph(args.session_dir, max_gyro=args.max_gyro, iterations=args.iterations)

if __name__ == "__main__":
    main()
