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
import os
import numpy as np
import yaml
import cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as R

_ALLOWED_DATA = Path(os.path.expanduser("~/atlas_ws/data")).resolve()
_ALLOWED_SRC  = Path(os.path.expanduser("~/atlas_ws/src")).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def _safe_src(p) -> Path:
    resolved = Path(p).resolve()
    for root in (_ALLOWED_SRC, _ALLOWED_DATA):
        if root in [resolved, *resolved.parents]:
            return resolved
    raise ValueError(f"Path '{resolved}' is outside allowed roots")


R_ROS2COLMAP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)

# cam_from_pano rotations matching panorama_sfm.py get_virtual_rotations().
# 8 yaw steps at 45° intervals on the equatorial band for maximum feature quality.
# Polar faces (ceiling/floor) are excluded — ERP pixel density is too low at the
# poles for reliable SIFT feature extraction, and they suffer from heavy
# interpolation blur when reprojected to perspective.
# The stitch seam runs vertically at yaw=0°/360° (left/right ERP edges) in
# dual_fisheye mode. Faces at yaw=0° and yaw=180° have the seam at their
# horizontal edges (within the 90° FOV overlap), but the seam artifacts are
# concentrated in a narrow band and empirically still yield better features
# than the polar tiles. A dedicated seam mask is applied below to exclude the
# affected pixel strip.
FACES = [
    {'name': 'front',       'pitch':   0, 'yaw':   0},
    {'name': 'front_left',  'pitch':   0, 'yaw':  45},
    {'name': 'left',        'pitch':   0, 'yaw':  90},
    {'name': 'back_left',   'pitch':   0, 'yaw': 135},
    {'name': 'back',        'pitch':   0, 'yaw': 180},
    {'name': 'back_right',  'pitch':   0, 'yaw': 225},
    {'name': 'right',       'pitch':   0, 'yaw': 270},
    {'name': 'front_right', 'pitch':   0, 'yaw': 315},
]
FACES_CAM_FROM_PANO = [
    R.from_euler('XY', [-f['pitch'], -f['yaw']], degrees=True).as_matrix()
    for f in FACES
]
# The Insta360 SDK ERP has forward (+X) at the top (v=0) and floor (-Z) at
# the center (v=H/2), equivalent to a standard ERP with the pole rotated by
# R_y(-90°). To sample equatorial faces correctly we pre-compose R_y(+90°)
# into each face rotation so the camera rays land on the actual horizon band
# of the Insta360 ERP rather than the polar region.
_R_INSTA_CORRECTION = R.from_euler('Y', 90, degrees=True).as_matrix()
FACES_CAM_FROM_PANO = [
    (R.from_euler('XY', [-f['pitch'], -f['yaw']], degrees=True).as_matrix()
     @ _R_INSTA_CORRECTION)
    for f in FACES
]
NUM_FACES = len(FACES)
REF_FACE = 0  # front is ref — must be first in cameras array for rig_configurator

# Face indices for each camera mode.
# For 180° single fisheye, only render faces in the front hemisphere (look dir Z > 0 in pano frame).
FACES_360 = list(range(NUM_FACES))
# FACES_180: front hemisphere only (yaw within ±90° of center)
FACES_180 = [i for i in range(NUM_FACES) if abs(((FACES[i]['yaw'] + 180) % 360) - 180) <= 90]
FOV_DEG = 65.0
MIN_BASELINE_M = 0.10
# Minimum fraction of unmasked pixels for a tile to be included in the COLMAP model.
# Tiles below this threshold are entirely (or near-entirely) covered by the scanner
# body mask and contribute no useful features.
MIN_TILE_VISIBLE = 0.20  # 20% of tile pixels must be unmasked
# Minimum fraction of unmasked pixels across the whole ERP for a panorama to be
# included at all. An ERP below this is fully occluded (e.g. cap/floor shot).
MIN_ERP_VISIBLE  = 0.10  # 10% of ERP pixels must be unmasked


def _load_calibration(session_path=None):
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from camera_hw import camera_hw_for_session, calibration_path
    hw = camera_hw_for_session(session_path) if session_path else 'onex2'
    calib_path = calibration_path(hw)
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
    try:
        traj_file = _safe_data(traj_file)
    except ValueError:
        return None
    with open(traj_file) as f:
        traj = json.load(f)
    pose_data = traj.get('current_pose', {})
    if 'lidar_pose' in pose_data:
        lp = pose_data['lidar_pose']
    else:
        # Fallback: interpolate from full_trajectory at capture_time
        si = traj.get('scan_info', {})
        capture_time = si.get('capture_time') or si.get('scan_request_time')
        full = traj.get('full_trajectory', [])
        if capture_time and full:
            from scipy.spatial.transform import Slerp as _Slerp
            times = np.array([p['timestamp'] for p in full])
            idx = np.searchsorted(times, capture_time)
            if 0 < idx < len(full):
                p0, p1 = full[idx-1], full[idx]
                alpha = (capture_time - times[idx-1]) / (times[idx] - times[idx-1])
                pos0 = np.array([p0['position']['x'], p0['position']['y'], p0['position']['z']])
                pos1 = np.array([p1['position']['x'], p1['position']['y'], p1['position']['z']])
                q0 = [p0['orientation']['x'], p0['orientation']['y'], p0['orientation']['z'], p0['orientation']['w']]
                q1 = [p1['orientation']['x'], p1['orientation']['y'], p1['orientation']['z'], p1['orientation']['w']]
                r_interp = _Slerp([0.0, 1.0], R.from_quat([q0, q1]))(alpha)
                lp = {'position': {'x': float(pos0[0]+alpha*(pos1[0]-pos0[0])),
                                   'y': float(pos0[1]+alpha*(pos1[1]-pos0[1])),
                                   'z': float(pos0[2]+alpha*(pos1[2]-pos0[2]))},
                      'orientation': dict(zip('xyzw', r_interp.as_quat()))}
            else:
                lp = full[max(0, idx-1)]
        elif full:
            lp = full[-1]
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


def _tile_size_for_erp(erp_w):
    """Tile size matched to ERP pixel density at the tile edge latitude.
    At ±FOV/2 latitude, ERP has cos(FOV/2) fewer px/deg than at equator.
    Tile px/deg = tile_size / FOV_DEG must not exceed ERP px/deg at the edge
    to avoid upsampling (which causes grain/blur)."""
    px_per_deg_equator = erp_w / 360.0
    px_per_deg_edge = px_per_deg_equator * np.cos(np.radians(FOV_DEG / 2))
    max_tile = int(px_per_deg_edge * FOV_DEG)
    # Round down to nearest power of 2 for GPU-friendly sizes
    p = int(2 ** int(np.log2(max_tile)))
    return max(512, min(2048, p))


def _erp_to_perspective(erp_img, cam_from_pano_r, tile_size, interpolation=cv2.INTER_LANCZOS4):
    """Sample a perspective tile from an ERP image using Lanczos for sharper output."""
    pano_h, pano_w = erp_img.shape[:2]
    f = tile_size / (2 * np.tan(np.radians(FOV_DEG) / 2))
    c = tile_size / 2.0
    x, y = np.meshgrid(np.arange(tile_size) + 0.5, np.arange(tile_size) + 0.5)
    rays_cam = np.stack([(x - c) / f, (y - c) / f, np.ones((tile_size, tile_size))], axis=-1)
    rays_cam /= np.linalg.norm(rays_cam, axis=-1, keepdims=True)
    rays_pano = rays_cam.reshape(-1, 3) @ cam_from_pano_r
    r = rays_pano.T
    yaw = np.arctan2(r[0], r[2])
    pitch = -np.arctan2(r[1], np.linalg.norm(r[[0, 2]], axis=0))
    u = ((1 + yaw / np.pi) / 2 * pano_w - 0.5).astype(np.float32).reshape(tile_size, tile_size)
    v = ((1 - pitch * 2 / np.pi) / 2 * pano_h - 0.5).astype(np.float32).reshape(tile_size, tile_size)
    return cv2.remap(erp_img, u, v, interpolation, borderMode=cv2.BORDER_WRAP)


def prepare_images(session_path, colmap_dir, T_camera_lidar):
    images_dir = colmap_dir / 'images'
    mask_dir = colmap_dir / 'masks'
    if images_dir.exists():
        shutil.rmtree(images_dir)
    if mask_dir.exists():
        shutil.rmtree(mask_dir)

    scan_dirs = sorted(d for d in session_path.iterdir()
                       if d.is_dir() and d.name.startswith('fusion_scan_')
                       and not (d / '.blur_skip').exists())

    panoramas = []
    session_is_360 = None
    for scan_dir in scan_dirs:
        erp_src, is_360 = _find_erp_image(scan_dir)
        if erp_src is None:
            continue
        # Skip panoramas where the ERP is almost entirely masked
        _probe = cv2.imread(str(erp_src), cv2.IMREAD_UNCHANGED)
        if _probe is not None and _probe.ndim == 3 and _probe.shape[2] == 4:
            _erp_visible = np.count_nonzero(_probe[:, :, 3]) / _probe[:, :, 3].size
            if _erp_visible < MIN_ERP_VISIBLE:
                print(f"  Skipping {scan_dir.name}: ERP only {_erp_visible*100:.1f}% visible (fully masked)")
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

    _probe_img = cv2.imread(str(panoramas[0]['erp_src']), cv2.IMREAD_UNCHANGED)
    tile_size = _tile_size_for_erp(_probe_img.shape[1]) if _probe_img is not None else 1024
    f_px = tile_size / (2 * np.tan(np.radians(FOV_DEG) / 2))
    print(f"  ERP width: {_probe_img.shape[1] if _probe_img is not None else '?'}px "
          f"-> tile_size: {tile_size}px  f_px: {f_px:.1f}")

    for pano_idx, pano in enumerate(panoramas, start=1):
        erp_img = cv2.imread(str(pano['erp_src']), cv2.IMREAD_UNCHANGED)
        erp_mask = None
        if erp_img is not None and erp_img.ndim == 3 and erp_img.shape[2] == 4:
            erp_mask = erp_img[:, :, 3]
            erp_img = erp_img[:, :, :3]

        # Apply a seam exclusion strip to the mask. The dual-fisheye stitch seam
        # runs vertically at the left/right ERP edges (u=0 and u=W, which wrap).
        # DYNAMICSTITCH blending distorts features in a ~5% horizontal band on
        # each side. Mask these out so SIFT doesn't match warped features.
        if erp_img is not None:
            _erp_w = erp_img.shape[1]
            _seam_strip = int(_erp_w * 0.04)  # 4% on each side of the wrap seam
            if erp_mask is None:
                erp_mask = np.full(erp_img.shape[:2], 255, dtype=np.uint8)
            erp_mask[:, :_seam_strip] = 0
            erp_mask[:, _erp_w - _seam_strip:] = 0

        R_cam_w2c_col = pano['R_c2w'].T
        tiles = []
        for face_idx in active_faces:
            cam_from_pano_r = FACES_CAM_FROM_PANO[face_idx]
            tile = _erp_to_perspective(erp_img, cam_from_pano_r, tile_size)

            tile_mask = None
            if erp_mask is not None:
                tile_mask = _erp_to_perspective(erp_mask, cam_from_pano_r, tile_size,
                                                interpolation=cv2.INTER_NEAREST)
                if np.count_nonzero(tile_mask) / tile_mask.size < MIN_TILE_VISIBLE:
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
        pano['tile_size'] = tile_size
        pano['active_faces'] = active_faces

    print(f"  Generated tiles for {len(panoramas)} panoramas")
    return panoramas


def write_rig_config(colmap_dir, panoramas):
    """Write rig_config.json for rig_configurator CLI."""
    f_px = panoramas[0]['f_px']
    c = panoramas[0]['tile_size'] / 2.0

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


def write_rig_bin(output_dir, db_path, panoramas):
    """Write rigs.bin and frames.bin into the sparse model so bundle_adjuster
    treats each panorama as a rigid rig body."""
    conn = sqlite3.connect(str(db_path))

    # Read rig from DB
    rig_row = conn.execute('SELECT rig_id, ref_sensor_id, ref_sensor_type FROM rigs LIMIT 1').fetchone()
    if rig_row is None:
        conn.close()
        return
    rig_id, ref_sensor_id, ref_sensor_type = rig_row

    non_ref_sensors = conn.execute(
        'SELECT sensor_id, sensor_type, sensor_from_rig FROM rig_sensors WHERE rig_id=?',
        (rig_id,)
    ).fetchall()

    # Read frames and their image IDs
    frames = conn.execute('SELECT frame_id, rig_id FROM frames ORDER BY frame_id').fetchall()
    frame_data = {}
    for frame_id, _ in frames:
        data = conn.execute(
            'SELECT sensor_type, sensor_id, data_id FROM frame_data WHERE frame_id=?',
            (frame_id,)
        ).fetchall()
        frame_data[frame_id] = data

    # Read image poses from images.bin to get rig_from_world for each frame
    # Use the ref sensor image pose as the rig pose
    img_poses = {}
    with open(output_dir / 'images.bin', 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            img_id = struct.unpack('I', f.read(4))[0]
            qw, qx, qy, qz = struct.unpack('dddd', f.read(32))
            tx, ty, tz = struct.unpack('ddd', f.read(24))
            cam_id = struct.unpack('I', f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00': break
                name += c
            n_pts = struct.unpack('Q', f.read(8))[0]
            f.read(n_pts * 24)
            img_poses[img_id] = (qw, qx, qy, qz, tx, ty, tz)

    conn.close()

    SENSOR_TYPE_CAMERA = 0

    # rigs.bin: num_rigs, then for each rig:
    # rig_id(u32), num_sensors(u32), ref_type(i32), ref_id(u32),
    # then for each non-ref: type(i32), id(u32), has_pose(u8), [qw,qx,qy,qz,tx,ty,tz](7xf64)
    with open(output_dir / 'rigs.bin', 'wb') as f:
        f.write(struct.pack('Q', 1))  # 1 rig
        f.write(struct.pack('I', rig_id))
        num_sensors = 1 + len(non_ref_sensors)  # ref + non-ref
        f.write(struct.pack('I', num_sensors))
        f.write(struct.pack('i', ref_sensor_type))
        f.write(struct.pack('I', ref_sensor_id))
        for sensor_id, sensor_type, sensor_from_rig_blob in non_ref_sensors:
            f.write(struct.pack('i', sensor_type))
            f.write(struct.pack('I', sensor_id))
            if sensor_from_rig_blob:
                f.write(struct.pack('B', 1))
                # blob is 7 doubles: qw,qx,qy,qz,tx,ty,tz
                vals = struct.unpack('ddddddd', sensor_from_rig_blob)
                for v in vals:
                    f.write(struct.pack('d', v))
            else:
                f.write(struct.pack('B', 0))

    # frames.bin: num_frames(u64), then for each frame with pose:
    # frame_id(u32), rig_id(u32), qw,qx,qy,qz,tx,ty,tz(7xf64),
    # num_data(u32), then for each: sensor_type(i32), sensor_id(u32), data_id(u64)
    # Frame pose = ref sensor image pose (rig_from_world)
    frames_with_pose = []
    for frame_id, _ in frames:
        data = frame_data[frame_id]
        ref_img_id = next((d[2] for d in data
                           if d[0] == ref_sensor_type and d[1] == ref_sensor_id), None)
        if ref_img_id and ref_img_id in img_poses:
            frames_with_pose.append((frame_id, data, img_poses[ref_img_id]))

    with open(output_dir / 'frames.bin', 'wb') as f:
        f.write(struct.pack('Q', len(frames_with_pose)))
        for frame_id, data, (qw, qx, qy, qz, tx, ty, tz) in frames_with_pose:
            f.write(struct.pack('I', frame_id))
            f.write(struct.pack('I', rig_id))
            for v in (qw, qx, qy, qz, tx, ty, tz):
                f.write(struct.pack('d', v))
            f.write(struct.pack('I', len(data)))
            for sensor_type, sensor_id, data_id in data:
                f.write(struct.pack('i', sensor_type))
                f.write(struct.pack('I', sensor_id))
                f.write(struct.pack('Q', data_id))

    print(f"  Wrote rigs.bin ({1} rig, {len(frames_with_pose)} frames)")


def _filter_sfm_to_lidar_extents(sfm_pts, lidar_pts, padding=0.5):
    """Remove SfM points outside the LiDAR AABB expanded by padding metres."""
    mn = lidar_pts.min(axis=0) - padding
    mx = lidar_pts.max(axis=0) + padding
    mask = np.all((sfm_pts >= mn) & (sfm_pts <= mx), axis=1)
    return mask


def _filter_sfm_isolated(sfm_pts, radius=1.5, min_neighbors=20):
    """Remove SfM points with fewer than min_neighbors within radius metres."""
    if len(sfm_pts) == 0:
        return np.ones(len(sfm_pts), dtype=bool)
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(sfm_pts)
    _, ind = pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)
    mask = np.zeros(len(sfm_pts), dtype=bool)
    mask[ind] = True
    return mask


def _filter_sfm_by_lidar_proximity(sfm_pts, lidar_pts, max_dist=0.50):
    """Remove SfM points that have no LiDAR point within max_dist metres.
    Ensures SfM points land on real surfaces rather than floating in space.
    Threshold is 0.5m (relaxed from 0.35m) to avoid over-filtering when
    ICP corrections shift the LiDAR cloud relative to triangulated SfM points."""
    if len(sfm_pts) == 0 or len(lidar_pts) == 0:
        return np.ones(len(sfm_pts), dtype=bool)
    from scipy.spatial import cKDTree
    tree = cKDTree(lidar_pts)
    dists, _ = tree.query(sfm_pts, k=1, workers=-1)
    return dists <= max_dist


def _merge_point_clouds(lidar_ply, colmap_ply, output_ply, transform_lidar=True,
                        lidar_voxel_size=0.0, filter_sfm=False):
    """Merge LiDAR (ROS world frame) and COLMAP point clouds into COLMAP frame."""
    import open3d as o3d

    lidar_pcd = o3d.io.read_point_cloud(str(lidar_ply))
    if lidar_voxel_size > 0:
        before = len(lidar_pcd.points)
        lidar_pcd = lidar_pcd.voxel_down_sample(lidar_voxel_size)
        print(f'  LiDAR voxel downsample {lidar_voxel_size}m: {before} -> {len(lidar_pcd.points)} pts')

    lidar_pts = np.asarray(lidar_pcd.points)
    lidar_cols = (np.asarray(lidar_pcd.colors) * 255).astype(int) if lidar_pcd.has_colors() \
                 else np.zeros((len(lidar_pts), 3), int)
    if transform_lidar:
        lidar_pts = (R_ROS2COLMAP @ lidar_pts.T).T

    colmap_pcd = o3d.io.read_point_cloud(str(colmap_ply))
    colmap_pts = np.asarray(colmap_pcd.points)
    colmap_cols = (np.asarray(colmap_pcd.colors) * 255).astype(int) if colmap_pcd.has_colors() \
                  else np.zeros((len(colmap_pts), 3), int)

    if filter_sfm and len(lidar_pts) > 0 and len(colmap_pts) > 0:
        sfm_mask = _filter_sfm_to_lidar_extents(colmap_pts, lidar_pts)
        colmap_pts  = colmap_pts[sfm_mask]
        colmap_cols = colmap_cols[sfm_mask]
        print(f'  SfM outlier filter: {sfm_mask.sum()}/{len(sfm_mask)} points kept')

    all_pts  = np.vstack([lidar_pts,  colmap_pts])
    all_cols = np.vstack([lidar_cols, colmap_cols])

    with open(output_ply, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(all_pts)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for p, c in zip(all_pts, all_cols):
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(c[0])} {int(c[1])} {int(c[2])}\n')

    print(f'✓ Merged {len(lidar_pts)} lidar + {len(colmap_pts)} COLMAP points -> {output_ply}')
    return all_pts, all_cols


def _write_merged_to_points3d(output_dir, lidar_pts_world, lidar_cols_world, lidar_pts_for_filter=None):
    """Write LiDAR points as the sole point cloud in points3D.bin.
    SfM triangulated points are intentionally discarded — they served their
    purpose driving pose estimation and are not needed in the final model.
    lidar_pts_world / lidar_cols_world are already in COLMAP world coordinates."""
    pts  = lidar_pts_world
    cols = lidar_cols_world
    n_lidar = len(pts)
    p3d_bin = output_dir / 'points3D.bin'
    with open(p3d_bin, 'wb') as f:
        f.write(struct.pack('Q', n_lidar))
        for i in range(n_lidar):
            f.write(struct.pack('Q', i + 1))
            f.write(struct.pack('ddd', *pts[i]))
            f.write(struct.pack('BBB', *cols[i]))
            f.write(struct.pack('d', 0.0))
            f.write(struct.pack('Q', 0))  # track_len=0: LiDAR-injected
    print(f'✓ points3D.bin: {n_lidar} LiDAR points')


def _strip_rig_from_db(db_path):
    """Remove rig/frame tables so point_triangulator treats images independently."""
    conn = sqlite3.connect(str(db_path))
    conn.execute('DELETE FROM rig_sensors')
    conn.execute('DELETE FROM frame_data')
    conn.execute('DELETE FROM frames')
    conn.execute('DELETE FROM rigs')
    conn.commit()
    conn.close()


def run_pipeline(session_dir, exhaustive=True, bundle_adjustment=False,
                 lidar_voxel_size=0.0):
    try:
        session_path = _safe_data(Path(session_dir).expanduser())
    except ValueError as e:
        print(f"Error: {e}")
        return False
    colmap_dir = session_path / 'colmap'
    colmap_dir.mkdir(exist_ok=True)

    T_camera_lidar = _load_calibration(session_path)
    print("1. Preparing perspective tiles from ERP images...")
    panoramas = prepare_images(session_path, colmap_dir, T_camera_lidar)
    if not panoramas:
        print("No valid panoramas found.")
        return False

    db_path = colmap_dir / 'database.db'
    if db_path.exists():
        db_path.unlink()

    f_px = panoramas[0]['f_px']
    c = panoramas[0]['tile_size'] / 2.0

    print("2. Extracting features (SIMPLE_PINHOLE, one camera per face folder)...")
    mask_dir = colmap_dir / 'masks'
    cmd = [
        'colmap', 'feature_extractor',
        '--database_path', str(db_path),
        '--image_path', str(colmap_dir / 'images'),
        '--ImageReader.camera_model', 'SIMPLE_PINHOLE',
        '--ImageReader.single_camera_per_folder', '1',
        '--ImageReader.camera_params', f'{f_px:.4f},{c:.1f},{c:.1f}',
        '--SiftExtraction.max_num_features', '32768',
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
        '--FeatureMatching.max_num_matches', '32768',
        '--FeatureMatching.gpu_index', '-1',
    ], check=True)

    print("5. Writing binary init model with known poses...")
    init_sparse = write_init_model_bin(colmap_dir, panoramas)

    # Strip rig from DB before triangulation — point_triangulator crashes when
    # a triangulated point references an image whose rig frame is incomplete.
    # The rig was only needed for rig_verification during matching.
    _strip_rig_from_db(colmap_dir / 'database.db')

    output_dir = colmap_dir / 'sparse' / '0'
    output_dir.mkdir(parents=True, exist_ok=True)
    # Remove stale rig binaries from any previous BA run — point_triangulator
    # must not see rigs.bin/frames.bin or it crashes on dangling image IDs.
    for _stale in ('rigs.bin', 'frames.bin'):
        _p = output_dir / _stale
        if _p.exists():
            _p.unlink()

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
        print("7. Restoring rig for bundle adjustment...")
        subprocess.run([
            'colmap', 'rig_configurator',
            '--database_path', str(db_path),
            '--rig_config_path', str(rig_config_path),
        ], check=True)

        print("8. Writing rig into sparse model...")
        write_rig_bin(output_dir, db_path, panoramas)

        print("9. Running rig-aware bundle adjustment...")
        subprocess.run([
            'colmap', 'bundle_adjuster',
            '--input_path', str(output_dir),
            '--output_path', str(output_dir),
            '--BundleAdjustment.refine_focal_length', '0',
            '--BundleAdjustment.refine_principal_point', '0',
            '--BundleAdjustment.refine_extra_params', '0',
            '--BundleAdjustment.refine_sensor_from_rig', '0',
            '--BundleAdjustment.refine_rig_from_world', '1',
            '--BundleAdjustmentCeres.max_num_iterations', '500',
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

    # Merge LiDAR point cloud with COLMAP reconstruction.
    # Positions from sensor_lidar.ply (200k pts, same as depth images).
    # Colors from sensor_colored_exact.ply via nearest-neighbour transfer.
    # This guarantees points3D.bin is consistent with depth images.
    print("  Building per-scan LiDAR cloud for points3D...")
    import open3d as o3d
    all_lidar_pts  = []
    all_lidar_cols = []
    scan_dirs_map  = {int(d.name.split('_')[-1]): d
                      for d in sorted(session_path.glob('fusion_scan_*'))
                      if d.is_dir() and not (d / '.blur_skip').exists()}
    for pi, scan_dir in sorted(scan_dirs_map.items()):
        ply_dense  = scan_dir / 'sensor_lidar.ply'
        ply_color  = scan_dir / 'sensor_colored_exact.ply'
        if not ply_dense.exists():
            continue
        traj_f = scan_dir / 'trajectory_icp_refined.json'
        if not traj_f.exists():
            traj_f = scan_dir / 'trajectory.json'
        if not traj_f.exists():
            continue
        with open(traj_f) as _f:
            _traj = json.load(_f)
        _lp  = _traj['current_pose']['lidar_pose']
        _pos = np.array([_lp['position']['x'], _lp['position']['y'], _lp['position']['z']])
        _q   = np.array([_lp['orientation']['x'], _lp['orientation']['y'],
                         _lp['orientation']['z'], _lp['orientation']['w']])
        _Rl  = R.from_quat(_q).as_matrix()

        pcd_dense = o3d.io.read_point_cloud(str(ply_dense))
        if lidar_voxel_size > 0:
            pcd_dense = pcd_dense.voxel_down_sample(lidar_voxel_size)
        pts_s   = np.asarray(pcd_dense.points)
        pts_ros = (_Rl @ pts_s.T).T + _pos
        pts_col = (R_ROS2COLMAP @ pts_ros.T).T

        # Transfer colors from sensor_colored_exact.ply using nearest-neighbour
        if ply_color.exists():
            pcd_c = o3d.io.read_point_cloud(str(ply_color))
            if pcd_c.has_colors() and len(pcd_c.points) > 0:
                from scipy.spatial import cKDTree as _KDT
                _tree = _KDT(np.asarray(pcd_c.points))
                _dists, _idx = _tree.query(pts_s, k=1, workers=-1)
                # Only transfer color when the nearest colored point is close enough.
                # sensor_colored_exact.ply is a filtered subset of sensor_lidar.ply
                # (distance-clipped, masked), so uncolored points have no valid match.
                # A threshold of 0.05m (~2x voxel size) rejects spurious far matches.
                _valid = _dists < 0.05
                cols = np.zeros((len(pts_s), 3), int)
                cols[_valid] = (np.asarray(pcd_c.colors)[_idx[_valid]] * 255).astype(int)
                # Drop points with no valid color transfer
                pts_col = pts_col[_valid]
                cols    = cols[_valid]
            else:
                cols = np.zeros((len(pts_s), 3), int)
        else:
            cols = np.zeros((len(pts_s), 3), int)

        all_lidar_pts.append(pts_col)
        all_lidar_cols.append(cols)

    if all_lidar_pts and ply_out.exists():
        merged_lidar_pts  = np.vstack(all_lidar_pts)
        merged_lidar_cols = np.vstack(all_lidar_cols)
        # Also merge in the SfM points
        colmap_pcd  = o3d.io.read_point_cloud(str(ply_out))
        colmap_pts  = np.asarray(colmap_pcd.points)
        colmap_cols = (np.asarray(colmap_pcd.colors) * 255).astype(int) \
                      if colmap_pcd.has_colors() else np.zeros((len(colmap_pts), 3), int)
        sfm_mask = _filter_sfm_to_lidar_extents(colmap_pts, merged_lidar_pts)
        colmap_pts  = colmap_pts[sfm_mask]
        colmap_cols = colmap_cols[sfm_mask]
        print(f'  SfM outlier filter: {sfm_mask.sum()}/{len(sfm_mask)} points kept')
        iso_mask = _filter_sfm_by_lidar_proximity(colmap_pts, merged_lidar_pts)
        colmap_pts  = colmap_pts[iso_mask]
        colmap_cols = colmap_cols[iso_mask]
        print(f'  SfM proximity filter: {iso_mask.sum()}/{len(iso_mask)} points kept')
        lidar_iso_mask = _filter_sfm_isolated(merged_lidar_pts, radius=0.3, min_neighbors=5)
        merged_lidar_pts  = merged_lidar_pts[lidar_iso_mask]
        merged_lidar_cols = merged_lidar_cols[lidar_iso_mask]
        print(f'  LiDAR isolated filter: {lidar_iso_mask.sum()}/{len(lidar_iso_mask)} points kept')
        all_pts  = np.vstack([merged_lidar_pts,  colmap_pts])
        all_cols = np.vstack([merged_lidar_cols, colmap_cols])
        merged_ply = colmap_dir / 'sparse' / 'merged.ply'
        with open(merged_ply, 'w') as _mf:
            _mf.write('ply\nformat ascii 1.0\n')
            _mf.write(f'element vertex {len(all_pts)}\n')
            _mf.write('property float x\nproperty float y\nproperty float z\n')
            _mf.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
            for _p, _c in zip(all_pts, all_cols):
                _mf.write(f'{_p[0]:.6f} {_p[1]:.6f} {_p[2]:.6f} {int(_c[0])} {int(_c[1])} {int(_c[2])}\n')
        print(f'  LiDAR {len(merged_lidar_pts)} + SfM {len(colmap_pts)} -> {merged_ply}')
        _write_merged_to_points3d(output_dir, merged_lidar_pts, merged_lidar_cols)
    elif ply_out.exists():
        # Fallback: use session merged cloud if no per-scan PLYs available
        session_aligned = session_path / 'merged_aligned_colored.ply'
        session_merged  = session_path / 'merged_pointcloud.ply'
        lidar_ply = session_aligned if session_aligned.exists() else \
                    session_merged  if session_merged.exists()  else None
        if lidar_ply:
            import open3d as _o3d
            merged_ply = colmap_dir / 'sparse' / 'merged.ply'
            _fb_lidar_pcd = _o3d.io.read_point_cloud(str(lidar_ply))
            _fb_lidar_pts = (R_ROS2COLMAP @ np.asarray(_fb_lidar_pcd.points).T).T
            _merge_point_clouds(lidar_ply, ply_out, merged_ply, transform_lidar=True,
                                lidar_voxel_size=lidar_voxel_size, filter_sfm=True)
            _write_merged_to_points3d(output_dir, _fb_lidar_pts,
                                      (np.asarray(_fb_lidar_pcd.colors) * 255).astype(int)
                                      if _fb_lidar_pcd.has_colors()
                                      else np.zeros((len(_fb_lidar_pts), 3), int))

    # Generate depth images using non-downsampled per-scan point clouds
    print("\n10. Generating depth images...")
    import importlib.util as _ilu
    _dep_spec = _ilu.spec_from_file_location(
        'generate_colmap_depth',
        str(Path(__file__).resolve().parent / 'generate_colmap_depth.py')
    )
    _dep_mod = _ilu.module_from_spec(_dep_spec)
    _dep_spec.loader.exec_module(_dep_mod)
    _dep_mod.generate_depth_images(str(session_path))

    print("\nCreating colmap.zip...")
    import zipfile
    zip_path = session_path / 'colmap.zip'
    with zipfile.ZipFile(str(zip_path), 'w', zipfile.ZIP_DEFLATED) as zf:
        for f in sorted(colmap_dir.rglob('*')):
            if f.is_file():
                zf.write(f, f.relative_to(session_path))
    print(f"✓ colmap.zip: {zip_path}")

    return True


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Panorama SfM: perspective tiles + rig + triangulation (panorama_sfm.py style)'
    )
    parser.add_argument('session_directory')
    parser.add_argument('--sequential', dest='exhaustive', action='store_false',
                        help='Use sequential matcher instead of exhaustive')
    parser.add_argument('--lidar-voxel-size', type=float, default=0.0,
                        help='Voxel downsample size for LiDAR cloud before merging '
                             'with COLMAP points (metres, 0 = no downsampling)')
    parser.set_defaults(exhaustive=True)
    parser.add_argument('--no-bundle-adjustment', dest='bundle_adjustment',
                        action='store_false',
                        help='Skip rig-aware bundle adjustment')
    parser.set_defaults(bundle_adjustment=False)
    args = parser.parse_args()

    try:
        _safe_data(Path(args.session_directory).expanduser())
    except ValueError as e:
        print(f"Error: {e}")
        import sys; sys.exit(1)
    run_pipeline(args.session_directory,
                 exhaustive=args.exhaustive,
                 bundle_adjustment=args.bundle_adjustment,
                 lidar_voxel_size=args.lidar_voxel_size)
