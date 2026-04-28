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

def coarse_icp(source, target, voxel_size):
    """Fast coarse ICP to measure overlap fitness. Uses only 2 stages."""
    loss = o3d.pipelines.registration.TukeyLoss(k=voxel_size * 4.0)
    result = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size * 15, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
    result = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size * 5, result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))
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
    """Two-stage ICP: coarse pass to assess fitness, fine pass only if overlap is sufficient.

    This avoids wasting time on fine ICP for pairs with poor overlap, and prevents
    low-quality edges from polluting the pose graph.
    Returns (result, used_fine) where used_fine indicates fine ICP was run.
    """
    # Stage 1: fast coarse ICP to measure overlap
    coarse = coarse_icp(source, target, voxel_size)

    # Only run fine ICP if coarse fitness is good enough to trust
    MIN_FITNESS_FOR_FINE = 0.30
    if coarse.fitness < MIN_FITNESS_FOR_FINE:
        return coarse  # return coarse result; caller will likely reject it

    # Stage 2: fine ICP from coarse initialisation
    result = fine_icp(source, target, voxel_size, coarse.transformation)

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
    """90th-percentile gyro magnitude over a window centred on t.
    More robust than mean (catches sustained rotation) without being thrown off
    by brief vibration spikes that max would catch.
    Returns None if no data."""
    samples = [g for ts, g in imu_data if abs(ts - t) <= window / 2.0]
    return float(np.percentile(samples, 90)) if samples else None


def register_pose_graph(session_dir, max_gyro=0.12):
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

    # Use world_lidar.ply for ICP — these are already in world frame with
    # per-frame motion compensation, so no trajectory pre-transform is needed.
    # This gives much better ICP fitness than sensor-frame clouds pre-transformed
    # by tilted trajectory poses.
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
                    with open(traj_file) as f:
                        traj = json.load(f)
                    capture_time = traj.get('scan_info', {}).get('capture_time') or \
                                   traj.get('current_pose', {}).get('timestamp')
                    if capture_time is not None:
                        gyro = _gyro_at(imu_data, float(capture_time))
                        if gyro is not None and gyro > max_gyro:
                            print(f"  Skipping {scan_dir.name}: gyro={gyro:.3f} rad/s > {max_gyro:.2f}")
                            continue
                        # Velocity filter: skip scans where scanner was translating fast.
                        # Motion blur from translation degrades ICP just as much as rotation.
                        full = traj.get('full_trajectory', [])
                        if capture_time and full:
                            times_arr = np.array([p['timestamp'] for p in full])
                            idx = np.searchsorted(times_arr, float(capture_time))
                            if 1 <= idx < len(full):
                                p0, p1 = full[idx-1], full[idx]
                                dt = p1['timestamp'] - p0['timestamp']
                                if dt > 0:
                                    vel = ((p1['position']['x']-p0['position']['x'])**2 +
                                           (p1['position']['y']-p0['position']['y'])**2 +
                                           (p1['position']['z']-p0['position']['z'])**2)**0.5 / dt
                                    max_vel = 0.12  # m/s
                                    if vel > max_vel:
                                        print(f"  Skipping {scan_dir.name}: vel={vel:.3f} m/s > {max_vel:.2f}")
                                        continue
            lidar_files.append(lidar)
            ply_files.append(colored)
            valid_scan_dirs.append(scan_dir)

    scan_dirs = valid_scan_dirs
    
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
    # in a consistent upright frame.
    voxel_size = 0.05
    pcds = []
    pcds_colored = []
    for i, f in enumerate(lidar_files):
        pcd = o3d.io.read_point_cloud(str(f))
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd = pcd.voxel_down_sample(voxel_size)
        # Transform to relative-to-first frame using T_first_inv
        pcd.transform(T_first_inv)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)
        pcds.append(pcd)

        # Load colored cloud for fine-tuning — transform to same relative frame
        pcd_c = o3d.io.read_point_cloud(str(ply_files[i])).voxel_down_sample(voxel_size)
        pcd_c.transform(trajectory_poses[i])  # sensor -> relative-to-first
        pcd_c.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcds_colored.append(pcd_c if pcd_c.has_colors() else None)

        print(f"Preprocessed {lidar_files[i].parent.name}: {len(pcd.points)} points  color={'yes' if pcd_c.has_colors() else 'no'}")

    # Build pose graph — world_lidar clouds are already in world frame.
    # ICP finds small residual corrections. Node poses start at identity.
    print("\nBuilding pose graph...")
    pose_graph = o3d.pipelines.registration.PoseGraph()
    for _ in scan_dirs:
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))

    n_scans = len(pcds)

    # Sequential edges
    for i in range(n_scans - 1):
        print(f"\nRefining edge {i} -> {i+1} (sequential)")
        result = pairwise_icp(pcds[i], pcds[i+1], voxel_size, np.eye(4))
        print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
        T_icp = result.transformation
        rot_deg = np.degrees(np.arccos(np.clip((np.trace(T_icp[:3,:3]) - 1) / 2, -1, 1)))
        trans_m = np.linalg.norm(T_icp[:3, 3])
        if result.fitness < 0.30 or rot_deg > 5.0 or trans_m > 0.15:
            if result.fitness < 0.30:
                print(f"  ⚠ Low fitness ({result.fitness:.3f}) — using identity (trajectory pose)")
            else:
                print(f"  ⚠ ICP correction too large (rot={rot_deg:.1f}°, t={trans_m:.3f}m) — using identity")
            T_icp = np.eye(4)
        information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            pcds[i], pcds[i+1], voxel_size * 2, T_icp)
        information *= 10.0
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                i, i+1, T_icp, information, uncertain=False))

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
            result = pairwise_icp(pcds[src], pcds[tgt], voxel_size, np.eye(4))
            print(f"  Fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.4f}")
            if result.fitness > 0.35:
                T_icp = result.transformation
                rot_deg = np.degrees(np.arccos(np.clip((np.trace(T_icp[:3,:3]) - 1) / 2, -1, 1)))
                trans_m = np.linalg.norm(T_icp[:3, 3])
                if rot_deg > 5.0 or trans_m > 0.15:
                    print(f"  ⚠ Loop closure correction too large (rot={rot_deg:.1f}°, t={trans_m:.3f}m) — skipping")
                    continue
                information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    pcds[src], pcds[tgt], voxel_size * 2, T_icp)
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        src, tgt, T_icp, information, uncertain=True))
    
    print("\nSaving aligned results...")
    # Compute average sequential edge quality from sanity-checked edges
    seq_good = sum(1 for e in pose_graph.edges if not e.uncertain)
    avg_seq_fitness = seq_good / max(n_scans - 1, 1)
    print(f"Sequential edge quality: {seq_good}/{n_scans-1} passed sanity check")
    # ICP corrections are in world frame (small residuals on already-aligned clouds).
    # To get the final transform for sensor-frame colored clouds:
    #   T_final = T_icp_correction @ trajectory_pose
    # where trajectory_pose transforms sensor->world and T_icp_correction refines it.
    if avg_seq_fitness < 0.5 or len(pose_graph.edges) < n_scans:
        print("⚠ ICP edges unreliable — using trajectory poses directly (skipping optimization)")
        icp_corrections = [np.eye(4)] * n_scans
    else:
        print(f"Optimizing pose graph ({len(pose_graph.nodes)} nodes, {len(pose_graph.edges)} edges)...")
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=voxel_size * 2,
            edge_prune_threshold=0.25,
            reference_node=0,
            preference_loop_closure=0.1)
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)
        print("Optimization complete!")
        icp_corrections = [node.pose for node in pose_graph.nodes]

    # Full transform for sensor-frame colored clouds:
    # T_final = T_icp_correction @ T_first_inv @ T_abs
    #         = T_icp_correction @ trajectory_poses[i]
    # This matches merge_with_trajectory.py's relative-to-first convention.
    full_transforms = [icp_corrections[i] @ trajectory_poses[i] for i in range(n_scans)]

    for i, scan_dir in enumerate(scan_dirs):
        T_rel = full_transforms[i]
        t = T_rel[:3, 3]
        euler_deg = np.degrees(Rotation.from_matrix(T_rel[:3, :3]).as_euler('xyz'))
        correction_applied = not np.allclose(icp_corrections[i], np.eye(4), atol=1e-6)
        print(f"{scan_dir.name}: {'ICP corrected' if correction_applied else 'trajectory (identity)'}")
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
            with open(scan_dir / "trajectory_icp_refined.json", 'w') as f:
                json.dump(traj_refined, f, indent=2)

        colored_file = next((scan_dir / n for n in COLORED_CANDIDATES if (scan_dir / n).exists()), None)
        if colored_file:
            pcd_colored = o3d.io.read_point_cloud(str(colored_file))
            pcd_colored.transform(T_rel)
            o3d.io.write_point_cloud(
                str(scan_dir / f"{colored_file.stem}_aligned.ply"), pcd_colored)

    print("\nCreating merged colored point cloud...")
    merged_colored = o3d.geometry.PointCloud()

    for i, scan_dir in enumerate(scan_dirs):
        colored_file = next((scan_dir / n for n in COLORED_CANDIDATES if (scan_dir / n).exists()), None)
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
