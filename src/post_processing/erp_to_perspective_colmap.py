#!/usr/bin/env python3
"""
Convert equirectangular images to perspective projections for COLMAP.
Uses OpenCV and COLMAP CLI with camera mask support.
"""

import sys
import cv2
import numpy as np
from pathlib import Path
import subprocess
import json
from scipy.spatial.transform import Rotation as R

R_ROS2COLMAP = np.array([[1,0,0],[0,0,-1],[0,1,0]], dtype=float)

# Cubemap face rotations in camera frame: rows = [right_cam, down_cam, look_cam]
# Camera is mounted upside-down: physical up = cam -Z, physical down = cam +Z.
# Side faces use down=cam+Z (world-down for level scan -> upright images).
# All 6 faces are derived from the Forward frame by 90-deg rotations, guaranteeing
# a geometrically consistent cube with shared edges between adjacent faces.
FACES_IN_CAM = np.array([
    ([1,0,0], [0,-1,0], [0,0,-1]),  # Ceiling:  right=+X, down=-Y, look=-Z
    ([1,0,0], [0,+1,0], [0,0,+1]),  # Floor:    right=+X, down=+Y, look=+Z
    ([1,0,0], [0,0,+1], [0,-1,0]),  # Forward:  right=+X, down=+Z, look=-Y
    ([-1,0,0],[0,0,+1], [0,+1,0]),  # Backward: right=-X, down=+Z, look=+Y
    ([0,+1,0],[0,0,+1], [+1,0,0]),  # Left:     right=+Y, down=+Z, look=+X
    ([0,-1,0],[0,0,+1], [-1,0,0]),  # Right:    right=-Y, down=+Z, look=-X
], dtype=float)


def erp_to_perspective(erp_img, fov_deg, R_face_w2c, R_cam_w2c_ros, output_size=1024):
    """Sample a perspective face from an ERP image.
    R_face_w2c: face camera w2c in COLMAP world (rows=[right,down,look]).
    R_cam_w2c_ros: Actually c2w! (misnamed for backward compatibility)
    Rays: persp cam -> COLMAP world -> ROS world -> ERP cam.
    """
    h, w = erp_img.shape[:2]
    f = output_size / (2 * np.tan(np.radians(fov_deg) / 2))
    cx = cy = output_size / 2.0
    u, v = np.meshgrid(np.arange(output_size), np.arange(output_size))
    rays_persp = np.stack([(u-cx)/f, (v-cy)/f, np.ones((output_size,output_size))], axis=-1)
    rays_colmap = rays_persp @ R_face_w2c          # -> COLMAP world (row-vector)
    rays_ros    = rays_colmap @ R_ROS2COLMAP        # -> ROS world
    rays_erp    = rays_ros @ R_cam_w2c_ros          # -> ERP camera frame (row-vec: @ c2w, despite param name)
    lon = np.arctan2(rays_erp[...,0], rays_erp[...,2])
    lat = np.arcsin(np.clip(-rays_erp[...,1] / np.linalg.norm(rays_erp, axis=-1), -1, 1))
    erp_u = ((lon + np.pi) / (2*np.pi) * w).astype(np.float32)
    erp_v = ((np.pi/2 - lat) / np.pi * h).astype(np.float32)
    return cv2.remap(erp_img, erp_u, erp_v, cv2.INTER_CUBIC, borderMode=cv2.BORDER_WRAP)

def convert_erp_to_perspectives(session_dir):
    """Convert all ERP images to perspective projections"""
    session_path = Path(session_dir).expanduser()
    colmap_dir = session_path / "colmap"
    erp_dir = colmap_dir / "images"
    persp_dir = colmap_dir / "perspective_images"
    mask_dir = colmap_dir / "perspective_masks"
    sparse_dir = colmap_dir / "sparse" / "0"
    
    # Run export_to_colmap to ensure images/points are up to date
    import importlib.util, shutil
    _spec = importlib.util.spec_from_file_location(
        "export_to_colmap",
        Path(__file__).parent / "export_to_colmap.py"
    )
    _mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    _mod.export_to_colmap(session_dir)
    T_lidar_camera = _mod._load_T_lidar_camera()

    # Clean up previous perspective images and reconstruction
    if persp_dir.exists():
        shutil.rmtree(persp_dir)
    persp_dir.mkdir(exist_ok=True)
    
    if mask_dir.exists():
        shutil.rmtree(mask_dir)
    mask_dir.mkdir(exist_ok=True)
    
    recon_dir = colmap_dir / "perspective_reconstruction"
    if recon_dir.exists():
        shutil.rmtree(recon_dir)
    
    db_file = colmap_dir / "perspective_database.db"
    if db_file.exists():
        db_file.unlink()
    
    print(f"Converting ERP images to perspective projections...")
    print(f"  Input: {erp_dir}")
    print(f"  Output: {persp_dir}")
    print(f"  Masks: {mask_dir}")
    
    # Build poses directly from trajectory files (bypass stale images.txt)
    poses = {}
    scan_dirs = sorted([d for d in session_path.iterdir()
                        if d.is_dir() and d.name.startswith('fusion_scan_')])
    for idx, scan_dir in enumerate(scan_dirs, start=1):
        traj_file = scan_dir / 'trajectory_icp_refined.json'
        if not traj_file.exists():
            traj_file = scan_dir / 'trajectory.json'
        if not traj_file.exists():
            continue
        with open(traj_file) as f:
            traj = json.load(f)
        pose_data = traj.get('current_pose', {})
        cam_quat_xyzw = None
        if 'lidar_pose' in pose_data:
            p = pose_data['lidar_pose']
            pos_xyz = [p['position']['x'], p['position']['y'], p['position']['z']]
            q_xyzw = [p['orientation']['x'], p['orientation']['y'], p['orientation']['z'], p['orientation']['w']]
            if 'camera_pose' in pose_data:
                c = pose_data['camera_pose']['orientation']
                cam_quat_xyzw = [c['x'], c['y'], c['z'], c['w']]
        elif 'poses' in traj and traj['poses']:
            p = traj['poses'][-1]
            pos_xyz = [p['position']['x'], p['position']['y'], p['position']['z']]
            q_xyzw = [p['orientation']['x'], p['orientation']['y'], p['orientation']['z'], p['orientation']['w']]
        else:
            continue
        (qw, qx, qy, qz), T = _mod.ros_pose_to_colmap_w2c(pos_xyz, q_xyzw, T_lidar_camera, cam_quat_xyzw)
        image_name = f'scan_{idx:03d}.png'
        poses[image_name] = {'id': idx, 'quat': [qw, qx, qy, qz], 'trans': T, 'cam_quat_ros': cam_quat_xyzw}
    
    perspective_images = []

    # Process images from colmap/images directory
    for erp_file in sorted(erp_dir.glob("*.jpg")) + sorted(erp_dir.glob("*.png")):
        # Skip mask files
        if '.mask.' in erp_file.name:
            continue

        print(f"\nProcessing {erp_file.name}...")
        erp_img = cv2.imread(str(erp_file))

        if erp_img is None:
            print(f"  Failed to load {erp_file}")
            continue

        # Check for corresponding mask
        mask_file = erp_dir / f"{erp_file.name}.mask.png"
        erp_mask = None
        if mask_file.exists():
            erp_mask = cv2.imread(str(mask_file), cv2.IMREAD_GRAYSCALE)

        base_name = erp_file.stem
        pose = poses.get(erp_file.name, None)

        # Compute shared camera w2c in COLMAP world and camera center
        if pose:
            cam_q_ros = pose.get('cam_quat_ros') or [0,0,0,1]
            R_cam_c2w_col = R_ROS2COLMAP @ R.from_quat(cam_q_ros).as_matrix()
            R_cam_w2c_col = R_cam_c2w_col.T
            R_cam_c2w_ros = R.from_quat(cam_q_ros).as_matrix()  # c2w, not w2c!
            qw, qx, qy, qz = pose['quat']
            R_erp_w2c = R.from_quat([qx, qy, qz, qw]).as_matrix()
            camera_center = -R_erp_w2c.T @ np.array(pose['trans'])
        else:
            R_cam_w2c_col = np.eye(3)
            R_cam_c2w_ros = np.eye(3)
            camera_center = None

        for i, R_face_in_cam in enumerate(FACES_IN_CAM):
            # R_face_w2c: face camera w2c in COLMAP world
            R_face_w2c = R_face_in_cam @ R_cam_w2c_col
            persp_img = erp_to_perspective(erp_img, fov_deg=90, R_face_w2c=R_face_w2c, R_cam_w2c_ros=R_cam_c2w_ros)
            out_name = f"{base_name}_view{i:02d}.png"
            cv2.imwrite(str(persp_dir / out_name), persp_img)
            print(f"  Saved: {out_name}")

            if erp_mask is not None:
                persp_mask = erp_to_perspective(erp_mask, fov_deg=90, R_face_w2c=R_face_w2c, R_cam_w2c_ros=R_cam_c2w_ros)
                _, persp_mask = cv2.threshold(persp_mask, 127, 255, cv2.THRESH_BINARY)
                cv2.imwrite(str(mask_dir / f"{out_name}.png"), persp_mask)

            if camera_center is not None:
                T_face = -R_face_w2c @ camera_center
                q = R.from_matrix(R_face_w2c).as_quat()  # [x,y,z,w]
                # COLMAP requires qw >= 0; negate if needed (same rotation, different sign)
                if q[3] < 0:
                    q = -q
                persp_quat = [q[3], q[0], q[1], q[2]]
                persp_trans = T_face.tolist()
            else:
                persp_quat = [1.0, 0.0, 0.0, 0.0]
                persp_trans = [0.0, 0.0, 0.0]

            perspective_images.append({
                'name': out_name,
                'original': erp_file.name,
                'view_index': i,
                'quat': persp_quat,
                'trans': persp_trans
            })

        print(f"  Generated {len(FACES_IN_CAM)} perspective views")
    
    print(f"\n✓ Generated {len(perspective_images)} perspective images")
    
    # Save metadata
    metadata_file = persp_dir / "metadata.json"
    with open(metadata_file, 'w') as f:
        json.dump(perspective_images, f, indent=2)
    
    return persp_dir

def run_colmap_on_perspectives(persp_dir, session_dir):
    """Run COLMAP reconstruction on perspective images with known poses"""
    session_path = Path(session_dir).expanduser()
    colmap_dir = session_path / "colmap"
    database = colmap_dir / "perspective_database.db"
    output_dir = colmap_dir / "perspective_reconstruction"
    output_dir.mkdir(exist_ok=True)
    
    print(f"\nRunning COLMAP on perspective images with known poses...")
    
    # Remove old database
    if database.exists():
        database.unlink()
    
    # Load metadata to create rig configuration
    metadata_file = persp_dir / "metadata.json"
    with open(metadata_file) as f:
        metadata = json.load(f)
    
    persp_sparse_dir = output_dir / "input"
    persp_sparse_dir.mkdir(exist_ok=True)

    # Write empty points3D.txt (no prior 3D points)
    with open(persp_sparse_dir / "points3D.txt", 'w') as f:
        pass

    # Feature extraction with per-image masks
    print("\n1. Extracting features...")
    mask_dir = colmap_dir / "perspective_masks"
    mask_files = list(mask_dir.glob("*.png"))
    print(f"  Found {len(mask_files)} mask files")

    cmd = [
        "colmap", "feature_extractor",
        "--database_path", str(database),
        "--image_path", str(persp_dir),
        "--ImageReader.camera_model", "PINHOLE",
        "--ImageReader.single_camera_per_image", "1",
        "--ImageReader.camera_params", "512,512,512,512",
    ]
    if mask_files:
        cmd.extend(["--ImageReader.mask_path", str(mask_dir)])
    subprocess.run(cmd)

    # Skip rig setup - not needed for triangulation with known poses

    # Write cameras and images as BINARY files
    import sqlite3 as _sq, struct
    from scipy.spatial.transform import Rotation
    _conn = _sq.connect(str(database))
    db_images = [(img_id, cam_id, name) for img_id, cam_id, name
                 in _conn.execute('SELECT image_id, camera_id, name FROM images ORDER BY image_id').fetchall()]
    db_cameras = _conn.execute('SELECT camera_id, model, width, height, params FROM cameras ORDER BY camera_id').fetchall()
    _conn.close()
    
    # Create metadata lookup by name
    metadata_map = {item['name']: item for item in metadata}
    
    # Write cameras.bin
    with open(persp_sparse_dir / "cameras.bin", 'wb') as f:
        f.write(struct.pack('Q', len(db_cameras)))
        for cam_id, model, width, height, params_blob in db_cameras:
            f.write(struct.pack('I', cam_id))
            f.write(struct.pack('i', 1))  # PINHOLE model
            f.write(struct.pack('QQ', width, height))
            f.write(params_blob)
    
    # Write images.bin in database order
    with open(persp_sparse_dir / "images.bin", 'wb') as f:
        f.write(struct.pack('Q', len(db_images)))
        for img_id, cam_id, name in db_images:
            item = metadata_map[name]
            qw, qx, qy, qz = item['quat']
            tx, ty, tz = item['trans']
            f.write(struct.pack('I', img_id))
            f.write(struct.pack('dddd', qw, qx, qy, qz))
            f.write(struct.pack('ddd', tx, ty, tz))
            f.write(struct.pack('I', cam_id))
            f.write(name.encode('utf-8') + b'\x00')
            f.write(struct.pack('Q', 0))  # No points2D yet
    
    # Write empty points3D.bin
    with open(persp_sparse_dir / "points3D.bin", 'wb') as f:
        f.write(struct.pack('Q', 0))

    # Feature matching
    print("\n2. Matching features...")
    subprocess.run([
        "colmap", "exhaustive_matcher",
        "--database_path", str(database)
    ])
    
    # Reconstruction: Import features into model with known poses
    print("\n3. Importing features with known poses...")
    output_model_dir = output_dir / "0"
    output_model_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy input model to output (preserves exact poses)
    import shutil
    for file in persp_sparse_dir.glob("*"):
        if file.is_file():
            shutil.copy(file, output_model_dir / file.name)
    
    # Write empty points3D.bin
    with open(output_model_dir / "points3D.bin", 'wb') as f:
        f.write(struct.pack('Q', 0))  # 0 points
    
    print("  Copied input model to preserve exact poses")
    
    # Triangulate points with known poses
    subprocess.run([
        "colmap", "point_triangulator",
        "--database_path", str(database),
        "--image_path", str(persp_dir),
        "--input_path", str(output_model_dir),
        "--output_path", str(output_model_dir),
        "--Mapper.tri_min_angle", "1.0",
        "--Mapper.tri_merge_max_reproj_error", "16.0",
        "--Mapper.tri_complete_max_reproj_error", "16.0",
        "--Mapper.tri_re_max_angle_error", "8.0",
        "--clear_points", "1",
    ])

    # Skip bundle adjustment - it modifies poses even with refine_rig_from_world=0
    # Use triangulated model directly with known poses
    print("\n4. Skipping bundle adjustment (preserving known poses)...")
    if not any(output_model_dir.iterdir()):
        print("  No triangulation output")
        return False

    # Export to PLY
    model_dir = output_model_dir
    if model_dir.exists() and any(model_dir.iterdir()):
        print(f"\n✓ Reconstruction complete: {model_dir}")
        
        ply_output = output_dir / "reconstructed.ply"
        subprocess.run([
            "colmap", "model_converter",
            "--input_path", str(model_dir),
            "--output_path", str(ply_output),
            "--output_type", "PLY"
        ])
        
        if ply_output.exists():
            print(f"✓ Exported to: {ply_output}")

            lidar_ply = colmap_dir / "sparse" / "0" / "sparse.ply"
            if lidar_ply.exists():
                compare_point_clouds(lidar_ply, ply_output)
            merged_ply = output_dir / "merged.ply"
            session_merged = Path(session_dir) / "merged_pointcloud.ply"
            base_ply = session_merged if session_merged.exists() else lidar_ply
            if base_ply.exists():
                merge_point_clouds(base_ply, ply_output, merged_ply)

            return True
    
    return False

def merge_point_clouds(lidar_ply, colmap_ply, output_ply):
    """Merge lidar (ROS frame) and COLMAP (COLMAP world frame) point clouds.
    COLMAP points are transformed back to ROS frame via R_ROS2COLMAP.T before merging.
    """
    import numpy as np
    import open3d as o3d

    def read_ply(path):
        pcd = o3d.io.read_point_cloud(str(path))
        pts = np.asarray(pcd.points)
        cols = (np.asarray(pcd.colors) * 255).astype(int) if pcd.has_colors() else np.zeros((len(pts), 3), int)
        return [[*pts[i].tolist(), *cols[i].tolist()] for i in range(len(pts))]

    R_ROS2COLMAP = np.array([[1,0,0],[0,0,-1],[0,1,0]], dtype=float)

    lidar_pts = read_ply(lidar_ply)
    # Transform lidar points (ROS frame) into COLMAP world frame
    for p in lidar_pts:
        xyz = R_ROS2COLMAP @ [p[0], p[1], p[2]]
        p[0], p[1], p[2] = xyz[0], xyz[1], xyz[2]

    colmap_pts = read_ply(colmap_ply)
    for p in colmap_pts:
        p[3], p[4], p[5] = 255, 0, 0  # red markers
    all_pts = lidar_pts + colmap_pts

    with open(output_ply, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(all_pts)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for p in all_pts:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(p[3])} {int(p[4])} {int(p[5])}\n')

    print(f'\u2713 Merged {len(lidar_pts)} lidar + {len(colmap_pts)} COLMAP points -> {output_ply}')


def compare_point_clouds(original_path, reconstructed_path):
    """Compare original and reconstructed point clouds"""
    print(f"\nComparing point clouds...")
    
    # Count points
    def count_ply_points(path):
        with open(path, 'rb') as f:
            for line in f:
                try:
                    line_str = line.decode('utf-8').strip()
                    if line_str.startswith('element vertex'):
                        return int(line_str.split()[2])
                except:
                    continue
        return 0
    
    orig_count = count_ply_points(original_path)
    recon_count = count_ply_points(reconstructed_path)
    
    print(f"  Original sparse.ply: {orig_count} points")
    print(f"  Reconstructed: {recon_count} points")
    
    if recon_count > 0:
        ratio = recon_count / orig_count * 100
        print(f"  Recovery rate: {ratio:.1f}%")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 erp_to_perspective_colmap.py <session_directory>")
        sys.exit(1)
    
    session_dir = sys.argv[1]
    
    # Convert ERP to perspective
    persp_dir = convert_erp_to_perspectives(session_dir)
    
    # Run COLMAP
    run_colmap_on_perspectives(persp_dir, session_dir)
