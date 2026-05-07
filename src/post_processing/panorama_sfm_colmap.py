#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Panorama SfM following colmap/python/examples/panorama_sfm.py.
# Slices each ERP into perspective tiles stored in per-face subfolders
# (SIMPLE_PINHOLE, one camera per folder), configures a rig via
# rig_configurator CLI, writes a binary init model with known LiDAR poses,
# triangulates with point_triangulator, and optionally refines with
# bundle_adjuster.

import json
import shutil
import sqlite3
import struct
import subprocess
import numpy as np
import yaml
import cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as R

R_ROS2COLMAP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)

# cam_from_pano rotations matching panorama_sfm.py get_virtual_rotations().
# 4 yaw steps at 0/90/180/270 deg = cubemap sides, plus ceiling and floor.
# Rotation.from_euler("XY", [-pitch, -yaw]) matches the example convention.
FACES = [
    {'name': 'ceiling', 'pitch':  90, 'yaw':   0},
    {'name': 'floor',   'pitch': -90, 'yaw':   0},
    {'name': 'front',   'pitch':   0, 'yaw':   0},
    {'name': 'back',    'pitch':   0, 'yaw': 180},
    {'name': 'left',    'pitch':   0, 'yaw':  90},
    {'name': 'right',   'pitch':   0, 'yaw': 270},
]
FACES_CAM_FROM_PANO = [
    R.from_euler('XY', [-f['pitch'], -f['yaw']], degrees=True).as_matrix()
    for f in FACES
]
NUM_FACES = len(FACES)
REF_FACE = 0  # ceiling is ref — must be first in cameras array for rig_configurator

# Face indices for each camera mode.
# For 180° single fisheye, only render faces in the front hemisphere (look dir Z > 0 in pano frame).
FACES_360 = list(range(NUM_FACES))
# FACES_180: all faces except 'back' (index 3) which points away from the single fisheye lens
FACES_180 = [i for i in range(NUM_FACES) if FACES[i]['name'] != 'back']
TILE_SIZE = 1024
FOV_DEG = 90.0
MIN_BASELINE_M = 0.10


def _load_calibration():
    calib_path = Path.home() / "atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml"
    with open(calib_path) as f:
        calib = yaml.safe_load(f)
    T_camera_lidar = np.eye(4)
    T_camera_lidar[:3, :3] = R.from_euler(
        'xyz', [calib['roll_offset'], calib['pitch_offset'], calib['yaw_offset']]
    ).as_matrix()
    T_camera_lidar[:3, 3] = [calib['x_offset'], calib['y_offset'], calib['z_offset']]
    return T_camera_lidar


def _load_pose(scan_dir, T_camera_lidar):
    """Return (camera_center_colmap [3], R_c2w_colmap [3x3], R_c2w_ros [3x3]) or None."""
    traj_file = scan_dir / 'trajectory_icp_refined.json'
    if not traj_file.exists():
        traj_file = scan_dir / 'trajectory.json'
    if not traj_file.exists():
        return None
    with open(traj_file) as f:
        traj = json.load(f)
    pose_data = traj.get('current_pose', {})
    if 'lidar_pose' in pose_data:
        lp = pose_data['lidar_pose']
    elif 'poses' in traj and traj['poses']:
        lp = traj['poses'][-1]
    else:
        return None
    pos = np.array([lp['position']['x'], lp['position']['y'], lp['position']['z']])
    q_xyzw = [lp['orientation']['x'], lp['orientation']['y'],
               lp['orientation']['z'], lp['orientation']['w']]
    T_lidar_camera = np.linalg.inv(T_camera_lidar)
    R_lidar_w = R.from_quat(q_xyzw).as_matrix()
    R_c2w_ros = R_lidar_w @ T_lidar_camera[:3, :3]
    C_ros = pos + R_lidar_w @ T_lidar_camera[:3, 3]
    return R_ROS2COLMAP @ C_ros, R_ROS2COLMAP @ R_c2w_ros, R_c2w_ros


def _find_erp_image(scan_dir):
    """Return (path, is_360) — is_360=True for dual fisheye (full sphere)."""
    for name in ['equirect_blended_masked.png', 'equirect_dual_fisheye_masked.png',
                 'equirect_dual_fisheye_raw_masked.png']:
        p = scan_dir / name
        if p.exists():
            return p, True
    for name in ['equirect_dual_fisheye.jpg']:
        p = scan_dir / name
        if p.exists():
            return p, True
    # Single fisheye — 180 degree ERP
    matches = list(scan_dir.glob('equirect_*_masked.png')) + \
              list(scan_dir.glob('equirect_*.jpg')) + \
              list(scan_dir.glob('equirect_*.png'))
    if matches:
        return matches[0], False
    return None, False


def _erp_to_perspective(erp_img, cam_from_pano_r, interpolation=cv2.INTER_LINEAR):
    """Sample a perspective tile from an ERP image.
    Follows panorama_sfm.py exactly: rays in cam frame -> rotate to pano frame
    -> project to spherical UV. No world pose involved.
    """
    pano_h, pano_w = erp_img.shape[:2]
    f = TILE_SIZE / (2 * np.tan(np.radians(FOV_DEG) / 2))
    c = TILE_SIZE / 2.0
    # Pixel centers at +0.5 offset (COLMAP convention)
    x, y = np.meshgrid(np.arange(TILE_SIZE) + 0.5, np.arange(TILE_SIZE) + 0.5)
    # Unproject to rays in camera frame, normalize
    rays_cam = np.stack([(x - c) / f, (y - c) / f, np.ones((TILE_SIZE, TILE_SIZE))], axis=-1)
    rays_cam /= np.linalg.norm(rays_cam, axis=-1, keepdims=True)
    # Rotate rays into panorama frame: rays_pano = rays_cam @ cam_from_pano_r
    rays_pano = rays_cam.reshape(-1, 3) @ cam_from_pano_r
    # Project to spherical UV (panorama_sfm.py spherical_img_from_cam)
    r = rays_pano.T
    yaw = np.arctan2(r[0], r[2])
    pitch = -np.arctan2(r[1], np.linalg.norm(r[[0, 2]], axis=0))
    u = ((1 + yaw / np.pi) / 2 * pano_w - 0.5).astype(np.float32).reshape(TILE_SIZE, TILE_SIZE)
    v = ((1 - pitch * 2 / np.pi) / 2 * pano_h - 0.5).astype(np.float32).reshape(TILE_SIZE, TILE_SIZE)
    return cv2.remap(erp_img, u, v, interpolation, borderMode=cv2.BORDER_WRAP)


def prepare_images(session_path, colmap_dir, T_camera_lidar):
    images_dir = colmap_dir / 'images'
    mask_dir = colmap_dir / 'masks'
    if images_dir.exists():
        shutil.rmtree(images_dir)
    if mask_dir.exists():
        shutil.rmtree(mask_dir)

    scan_dirs = sorted(d for d in session_path.iterdir()
                       if d.is_dir() and d.name.startswith('fusion_scan_'))

    panoramas = []
    session_is_360 = None
    for scan_dir in scan_dirs:
        erp_src, is_360 = _find_erp_image(scan_dir)
        if erp_src is None:
            continue
        if session_is_360 is None:
            session_is_360 = is_360
        pose = _load_pose(scan_dir, T_camera_lidar)
        if pose is None:
            continue
        C_col, R_c2w_col, R_c2w_ros = pose
        panoramas.append({'scan_dir': scan_dir, 'erp_src': erp_src,
                          'center': C_col, 'R_c2w': R_c2w_col, 'R_c2w_ros': R_c2w_ros})

    active_faces = FACES_360 if session_is_360 else FACES_180
    if session_is_360 is None:
        print("  No ERP images found")
        return []
    print(f"  Camera mode: {'360° dual fisheye' if session_is_360 else '180° single fisheye'} "
          f"({len(active_faces)} faces)")

    # Create per-face subdirectories only for active faces
    for face_idx in active_faces:
        (images_dir / f'face_{face_idx:02d}').mkdir(parents=True)
        (mask_dir / f'face_{face_idx:02d}').mkdir(parents=True)

    kept, positions = [], []
    for p in panoramas:
        if not positions or min(np.linalg.norm(p['center'] - q) for q in positions) >= MIN_BASELINE_M:
            kept.append(p)
            positions.append(p['center'])
    panoramas = kept
    print(f"  Kept {len(panoramas)} panoramas after baseline filter")

    f_px = TILE_SIZE / (2 * np.tan(np.radians(FOV_DEG) / 2))

    for pano_idx, pano in enumerate(panoramas, start=1):
        erp_img = cv2.imread(str(pano['erp_src']), cv2.IMREAD_UNCHANGED)
        erp_mask = None
        if erp_img is not None and erp_img.ndim == 3 and erp_img.shape[2] == 4:
            erp_mask = erp_img[:, :, 3]
            erp_img = erp_img[:, :, :3]

        R_cam_w2c_col = pano['R_c2w'].T
        tiles = []
        for face_idx in active_faces:
            cam_from_pano_r = FACES_CAM_FROM_PANO[face_idx]
            tile = _erp_to_perspective(erp_img, cam_from_pano_r)

            tile_mask = None
            if erp_mask is not None:
                tile_mask = _erp_to_perspective(erp_mask, cam_from_pano_r,
                                                interpolation=cv2.INTER_NEAREST)
                if np.count_nonzero(tile_mask) / tile_mask.size < 0.05:
                    continue

            fname = f"pano_{pano_idx:03d}.png"
            cv2.imwrite(str(images_dir / f'face_{face_idx:02d}' / fname), tile)
            if tile_mask is not None:
                cv2.imwrite(str(mask_dir / f'face_{face_idx:02d}' / f'{fname}.png'), tile_mask)

            # Tile w2c in COLMAP world = cam_from_pano @ pano_w2c_colmap
            # pano_w2c_colmap = R_cam_w2c_col (the panorama camera w2c)
            R_tile_w2c = cam_from_pano_r @ R_cam_w2c_col
            T_tile = -R_tile_w2c @ pano['center']
            q = R.from_matrix(R_tile_w2c).as_quat()  # xyzw
            if q[3] < 0:
                q = -q
            tiles.append({'face': face_idx,
                          'rel_path': f'face_{face_idx:02d}/{fname}',
                          'quat_wxyz': [q[3], q[0], q[1], q[2]],
                          'trans': T_tile.tolist()})

        pano['tiles'] = tiles
        pano['f_px'] = f_px
        pano['active_faces'] = active_faces

    print(f"  Generated tiles for {len(panoramas)} panoramas")
    return panoramas


def write_rig_config(colmap_dir, panoramas):
    """Write rig_config.json for rig_configurator CLI."""
    f_px = panoramas[0]['f_px']
    c = TILE_SIZE / 2.0

    # Only include faces that have images in the DB
    db_path = colmap_dir / 'database.db'
    conn = sqlite3.connect(str(db_path))
    present_faces = set()
    for face_idx in range(NUM_FACES):
        row = conn.execute(
            'SELECT 1 FROM images WHERE name LIKE ? LIMIT 1',
            (f'face_{face_idx:02d}/%',)
        ).fetchone()
        if row:
            present_faces.add(face_idx)
    conn.close()

    # Ref face must be present; fall back to first present face
    ref_face = REF_FACE if REF_FACE in present_faces else min(present_faces)
    R_ref_from_pano = FACES_CAM_FROM_PANO[ref_face]

    cameras = []
    # ref sensor must be first
    for face_idx in sorted(present_faces, key=lambda f: (f != ref_face, f)):
        entry = {
            "image_prefix": f"face_{face_idx:02d}/",
            "camera_model_name": "SIMPLE_PINHOLE",
            "camera_params": [f_px, c, c],
        }
        if face_idx == ref_face:
            entry["ref_sensor"] = True
        else:
            R_cam_from_rig = FACES_CAM_FROM_PANO[face_idx] @ R_ref_from_pano.T
            q = R.from_matrix(R_cam_from_rig).as_quat()  # xyzw
            if q[3] < 0:
                q = -q
            entry["cam_from_rig_rotation"] = [q[3], q[0], q[1], q[2]]
            entry["cam_from_rig_translation"] = [0.0, 0.0, 0.0]
        cameras.append(entry)

    rig_config_path = colmap_dir / 'rig_config.json'
    with open(rig_config_path, 'w') as f:
        json.dump([{"cameras": cameras}], f, indent=2)
    print(f"  Rig config: {len(cameras)} faces (present in DB)")
    return rig_config_path


def write_init_model_bin(colmap_dir, panoramas):
    """Write binary init sparse model with known poses for point_triangulator."""
    sparse_dir = colmap_dir / 'init_sparse' / '0'
    sparse_dir.mkdir(parents=True, exist_ok=True)

    db_path = colmap_dir / 'database.db'
    conn = sqlite3.connect(str(db_path))
    face_camera_ids = {}
    for face_idx in range(NUM_FACES):
        row = conn.execute(
            'SELECT camera_id FROM images WHERE name LIKE ? LIMIT 1',
            (f'face_{face_idx:02d}/%',)
        ).fetchone()
        if row:
            face_camera_ids[face_idx] = row[0]
    db_images = {name: img_id for img_id, name in
                 conn.execute('SELECT image_id, name FROM images').fetchall()}
    db_cameras = conn.execute(
        'SELECT camera_id, model, width, height, params FROM cameras ORDER BY camera_id'
    ).fetchall()
    conn.close()

    # cameras.bin
    with open(sparse_dir / 'cameras.bin', 'wb') as f:
        f.write(struct.pack('Q', len(db_cameras)))
        for cam_id, model, width, height, params_blob in db_cameras:
            f.write(struct.pack('I', cam_id))
            f.write(struct.pack('i', model))
            f.write(struct.pack('QQ', width, height))
            f.write(params_blob)

    # images.bin — only tiles that were actually written to disk
    all_tiles = []
    for pano in panoramas:
        for tile in pano['tiles']:
            img_id = db_images.get(tile['rel_path'])
            cam_id = face_camera_ids.get(tile['face'])
            if img_id is not None and cam_id is not None:
                # Verify the image file actually exists (not skipped as blank)
                img_file = colmap_dir / 'images' / tile['rel_path']
                if img_file.exists():
                    all_tiles.append((img_id, tile['quat_wxyz'], tile['trans'],
                                      cam_id, tile['rel_path']))

    with open(sparse_dir / 'images.bin', 'wb') as f:
        f.write(struct.pack('Q', len(all_tiles)))
        for img_id, quat_wxyz, trans, cam_id, name in all_tiles:
            qw, qx, qy, qz = quat_wxyz
            tx, ty, tz = trans
            f.write(struct.pack('I', img_id))
            f.write(struct.pack('dddd', qw, qx, qy, qz))
            f.write(struct.pack('ddd', tx, ty, tz))
            f.write(struct.pack('I', cam_id))
            f.write(name.encode('utf-8') + b'\x00')
            f.write(struct.pack('Q', 0))  # no points2D

    # points3D.bin — empty
    with open(sparse_dir / 'points3D.bin', 'wb') as f:
        f.write(struct.pack('Q', 0))

    return sparse_dir


def _strip_rig_from_db(db_path):
    """Remove rig/frame tables so point_triangulator treats images independently."""
    conn = sqlite3.connect(str(db_path))
    conn.execute('DELETE FROM rig_sensors')
    conn.execute('DELETE FROM frame_data')
    conn.execute('DELETE FROM frames')
    conn.execute('DELETE FROM rigs')
    conn.commit()
    conn.close()


def run_pipeline(session_dir, exhaustive=False, bundle_adjustment=True):
    session_path = Path(session_dir).expanduser()
    colmap_dir = session_path / 'colmap'
    colmap_dir.mkdir(exist_ok=True)

    T_camera_lidar = _load_calibration()

    print("1. Preparing perspective tiles from ERP images...")
    panoramas = prepare_images(session_path, colmap_dir, T_camera_lidar)
    if not panoramas:
        print("No valid panoramas found.")
        return False

    db_path = colmap_dir / 'database.db'
    if db_path.exists():
        db_path.unlink()

    f_px = panoramas[0]['f_px']
    c = TILE_SIZE / 2.0

    print("2. Extracting features (SIMPLE_PINHOLE, one camera per face folder)...")
    mask_dir = colmap_dir / 'masks'
    cmd = [
        'colmap', 'feature_extractor',
        '--database_path', str(db_path),
        '--image_path', str(colmap_dir / 'images'),
        '--ImageReader.camera_model', 'SIMPLE_PINHOLE',
        '--ImageReader.single_camera_per_folder', '1',
        '--ImageReader.camera_params', f'{f_px:.4f},{c:.1f},{c:.1f}',
    ]
    if any(mask_dir.rglob('*.png')):
        cmd += ['--ImageReader.mask_path', str(mask_dir)]
    subprocess.run(cmd, check=True)

    print("3. Configuring rig...")
    rig_config_path = write_rig_config(colmap_dir, panoramas)
    subprocess.run([
        'colmap', 'rig_configurator',
        '--database_path', str(db_path),
        '--rig_config_path', str(rig_config_path),
    ], check=True)

    print("4. Matching features...")
    matcher = 'exhaustive_matcher' if exhaustive else 'sequential_matcher'
    subprocess.run([
        'colmap', matcher,
        '--database_path', str(db_path),
        '--FeatureMatching.rig_verification', '1',
        '--FeatureMatching.skip_image_pairs_in_same_frame', '1',
    ], check=True)

    print("5. Writing binary init model with known poses...")
    init_sparse = write_init_model_bin(colmap_dir, panoramas)

    # Strip rig from DB before triangulation — point_triangulator crashes when
    # a triangulated point references an image whose rig frame is incomplete.
    # The rig was only needed for rig_verification during matching.
    _strip_rig_from_db(colmap_dir / 'database.db')

    output_dir = colmap_dir / 'sparse' / '0'
    output_dir.mkdir(parents=True, exist_ok=True)

    print("6. Triangulating points with known poses...")
    subprocess.run([
        'colmap', 'point_triangulator',
        '--database_path', str(db_path),
        '--image_path', str(colmap_dir / 'images'),
        '--input_path', str(init_sparse),
        '--output_path', str(output_dir),
        '--clear_points', '1',
        '--Mapper.tri_min_angle', '1.0',
        '--Mapper.tri_merge_max_reproj_error', '16.0',
        '--Mapper.tri_complete_max_reproj_error', '16.0',
        '--Mapper.tri_re_max_angle_error', '8.0',
    ], check=True)

    if bundle_adjustment:
        print("7. Running bundle adjustment...")
        subprocess.run([
            'colmap', 'bundle_adjuster',
            '--input_path', str(output_dir),
            '--output_path', str(output_dir),
            '--BundleAdjustment.refine_focal_length', '0',
            '--BundleAdjustment.refine_principal_point', '0',
            '--BundleAdjustment.refine_extra_params', '0',
            '--BundleAdjustment.refine_sensor_from_rig', '0',
        ], check=True)

    print(f"\n✓ Reconstruction complete: {output_dir}")
    ply_out = colmap_dir / 'sparse' / 'reconstructed.ply'
    subprocess.run([
        'colmap', 'model_converter',
        '--input_path', str(output_dir),
        '--output_path', str(ply_out),
        '--output_type', 'PLY',
    ])
    if ply_out.exists():
        print(f"✓ Exported PLY: {ply_out}")

    return True


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Panorama SfM: perspective tiles + rig + triangulation (panorama_sfm.py style)'
    )
    parser.add_argument('session_directory')
    parser.add_argument('--exhaustive', action='store_true',
                        help='Use exhaustive matcher instead of sequential')
    parser.add_argument('--no-bundle-adjustment', dest='bundle_adjustment',
                        action='store_false',
                        help='Skip bundle adjustment after triangulation')
    parser.set_defaults(bundle_adjustment=True)
    args = parser.parse_args()

    run_pipeline(args.session_directory,
                 exhaustive=args.exhaustive,
                 bundle_adjustment=args.bundle_adjustment)
