#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Post-export rig calibration refinement.
#
# LiDAR mode (triggered by colmap/pose_source.json {"pose_source":"lidar"}):
#   Aligns all exported image centres to the LiDAR-derived world positions,
#   fits one rotation+translation correction per camera body (all X5 faces
#   share one correction; OAK-1 gets its own), updates images.bin, frames.bin,
#   and sensor_from_rig in rigs.bin, then re-renders depth_images from the
#   corrected poses. Optionally writes the X5 correction back to the
#   calibration YAML so future sessions start from the improved extrinsic.
#
# Mobile mode (no pose_source.json, or pose_source != "lidar"):
#   Behaviour unchanged from the original calibrate_rig logic (not implemented
#   here — this file only handles the LiDAR-referenced desktop case).
#
# Usage:
#   python3 calibrate_rig.py <session_dir> [--write-calib] [--dry-run]

import argparse
import json
import struct
import sys
import os
import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation, Slerp
from scipy.optimize import minimize

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()
_ALLOWED_SRC  = Path(os.path.expanduser('~/atlas_ws/src')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if not resolved.is_relative_to(_ALLOWED_DATA):
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


# ROS → COLMAP axis change (same as panorama_sfm_colmap.py)
R_ROS2COLMAP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)


# ---------------------------------------------------------------------------
# Binary I/O helpers
# ---------------------------------------------------------------------------

def _read_images_bin(path):
    """Return dict img_id -> {name, qvec[4], tvec[3], cam_id, n_pts, pts_data}."""
    images = {}
    with open(path, 'rb') as f:
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
            pts_data = f.read(n_pts * 24)
            images[img_id] = {
                'name': name.decode(),
                'qvec': np.array([qw, qx, qy, qz]),
                'tvec': np.array([tx, ty, tz]),
                'cam_id': cam_id,
                'n_pts': n_pts,
                'pts_data': pts_data,
            }
    return images


def _write_images_bin(path, images):
    with open(path, 'wb') as f:
        f.write(struct.pack('Q', len(images)))
        for img_id, img in sorted(images.items()):
            qw, qx, qy, qz = img['qvec']
            tx, ty, tz = img['tvec']
            f.write(struct.pack('I', img_id))
            f.write(struct.pack('dddd', qw, qx, qy, qz))
            f.write(struct.pack('ddd', tx, ty, tz))
            f.write(struct.pack('I', img['cam_id']))
            f.write(img['name'].encode('utf-8') + b'\x00')
            f.write(struct.pack('Q', img['n_pts']))
            f.write(img['pts_data'])


def _read_rigs_bin(path):
    """Return (rig_id, ref_sensor_id, non_ref_sensors) where
    non_ref_sensors = list of (sensor_id, has_pose, sfr_7d_or_None)."""
    with open(path, 'rb') as f:
        n_rigs = struct.unpack('Q', f.read(8))[0]
        if n_rigs == 0:
            return None, None, []
        rig_id = struct.unpack('I', f.read(4))[0]
        n_sensors = struct.unpack('I', f.read(4))[0]
        struct.unpack('i', f.read(4))[0]  # ref_type
        ref_sensor_id = struct.unpack('I', f.read(4))[0]
        non_ref = []
        for _ in range(n_sensors - 1):
            struct.unpack('i', f.read(4))[0]  # sensor_type
            sid = struct.unpack('I', f.read(4))[0]
            has_pose = struct.unpack('B', f.read(1))[0]
            sfr = None
            if has_pose:
                sfr = list(struct.unpack('ddddddd', f.read(56)))
            non_ref.append((sid, has_pose, sfr))
    return rig_id, ref_sensor_id, non_ref


def _write_rigs_bin(path, rig_id, ref_sensor_id, non_ref_sensors):
    """Write rigs.bin with updated sensor_from_rig values."""
    SENSOR_TYPE_CAMERA = 0
    with open(path, 'wb') as f:
        f.write(struct.pack('Q', 1))
        f.write(struct.pack('I', rig_id))
        f.write(struct.pack('I', 1 + len(non_ref_sensors)))
        f.write(struct.pack('i', SENSOR_TYPE_CAMERA))
        f.write(struct.pack('I', ref_sensor_id))
        for sid, has_pose, sfr in non_ref_sensors:
            f.write(struct.pack('i', SENSOR_TYPE_CAMERA))
            f.write(struct.pack('I', sid))
            if sfr is not None:
                f.write(struct.pack('B', 1))
                for v in sfr:
                    f.write(struct.pack('d', v))
            else:
                f.write(struct.pack('B', 0))


def _read_frames_bin(path):
    """Return list of (frame_id, rig_id, pose_7d, [(sensor_type, sensor_id, data_id)])."""
    frames = []
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            fid = struct.unpack('I', f.read(4))[0]
            rid = struct.unpack('I', f.read(4))[0]
            pose = list(struct.unpack('ddddddd', f.read(56)))
            n_data = struct.unpack('I', f.read(4))[0]
            data = []
            for _ in range(n_data):
                st = struct.unpack('i', f.read(4))[0]
                si = struct.unpack('I', f.read(4))[0]
                di = struct.unpack('Q', f.read(8))[0]
                data.append((st, si, di))
            frames.append((fid, rid, pose, data))
    return frames


def _write_frames_bin(path, frames):
    with open(path, 'wb') as f:
        f.write(struct.pack('Q', len(frames)))
        for fid, rid, pose, data in frames:
            f.write(struct.pack('I', fid))
            f.write(struct.pack('I', rid))
            for v in pose:
                f.write(struct.pack('d', v))
            f.write(struct.pack('I', len(data)))
            for st, si, di in data:
                f.write(struct.pack('i', st))
                f.write(struct.pack('I', si))
                f.write(struct.pack('Q', di))


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def _qvec_to_R(qvec):
    """qvec = [qw, qx, qy, qz] -> 3x3 rotation matrix."""
    return Rotation.from_quat([qvec[1], qvec[2], qvec[3], qvec[0]]).as_matrix()


def _R_to_qvec(R_mat):
    """3x3 -> [qw, qx, qy, qz]."""
    q = Rotation.from_matrix(R_mat).as_quat()  # xyzw
    if q[3] < 0:
        q = -q
    return np.array([q[3], q[0], q[1], q[2]])


def _camera_center(qvec, tvec):
    """COLMAP w2c -> camera center in world."""
    R_w2c = _qvec_to_R(qvec)
    return -R_w2c.T @ tvec


def _w2c_from_center_R(center, R_c2w):
    R_w2c = R_c2w.T
    tvec = -R_w2c @ center
    return _R_to_qvec(R_w2c), tvec


# ---------------------------------------------------------------------------
# Subfolder classification
# ---------------------------------------------------------------------------

def _body_for_image(name):
    """Return 'x5' or 'oak1' based on image subfolder name."""
    sf = name.split('/')[0]
    if sf.startswith('face_oak1'):
        return 'oak1'
    return 'x5'


def _face_idx_for_image(name):
    sf = name.split('/')[0]
    try:
        return int(sf.split('_')[1])
    except (IndexError, ValueError):
        return 0


# ---------------------------------------------------------------------------
# LiDAR-referenced alignment
# ---------------------------------------------------------------------------

def _load_lidar_centers(session_path, images, R_ros2col):
    """For each image, load the LiDAR-derived camera center from trajectory.json.
    Returns dict img_id -> center_colmap (3,)."""
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from camera_hw import load_calibration_transform, cam_index_for_scan

    scan_dirs = {int(d.name.split('_')[-1]): d
                 for d in sorted(session_path.glob('fusion_scan_*'))
                 if d.is_dir()}

    # Cache T_camera_lidar per body
    _T_cache = {}

    def _get_T(body, scan_dir):
        if body not in _T_cache:
            try:
                ci = cam_index_for_scan(scan_dir)
                T, _ = load_calibration_transform(body, ci)
                _T_cache[body] = T
            except Exception:
                _T_cache[body] = np.eye(4)
        return _T_cache[body]

    lidar_centers = {}
    for img_id, img in images.items():
        name = img['name']
        pano_num = None
        try:
            pano_num = int(name.split('pano_')[1].split('.')[0])
        except (IndexError, ValueError):
            continue
        scan_dir = scan_dirs.get(pano_num)
        if scan_dir is None:
            continue
        traj_f = scan_dir / 'trajectory_icp_refined.json'
        if not traj_f.exists():
            traj_f = scan_dir / 'trajectory.json'
        if not traj_f.exists():
            continue
        try:
            traj = json.loads(traj_f.read_text())
            lp = traj['current_pose']['lidar_pose']
            pos = np.array([lp['position']['x'], lp['position']['y'], lp['position']['z']])
            q_xyzw = [lp['orientation']['x'], lp['orientation']['y'],
                      lp['orientation']['z'], lp['orientation']['w']]
            R_lidar_w = Rotation.from_quat(q_xyzw).as_matrix()
            body = _body_for_image(name)
            T = _get_T(body, scan_dir)
            T_lidar_cam = np.linalg.inv(T)
            C_ros = pos + R_lidar_w @ T_lidar_cam[:3, 3]
            lidar_centers[img_id] = R_ros2col @ C_ros
        except (KeyError, TypeError, json.JSONDecodeError):
            continue
    return lidar_centers


def _fit_correction(sfm_centers, lidar_centers, img_ids):
    """Fit a single rotation + translation that maps sfm_centers -> lidar_centers.
    Returns (R_corr 3x3, t_corr 3) using Procrustes (SVD)."""
    sfm = np.array([sfm_centers[i] for i in img_ids])
    lid = np.array([lidar_centers[i] for i in img_ids])
    sfm_c = sfm.mean(axis=0)
    lid_c = lid.mean(axis=0)
    H = (sfm - sfm_c).T @ (lid - lid_c)
    U, _, Vt = np.linalg.svd(H)
    R_corr = Vt.T @ U.T
    if np.linalg.det(R_corr) < 0:
        Vt[-1] *= -1
        R_corr = Vt.T @ U.T
    t_corr = lid_c - R_corr @ sfm_c
    return R_corr, t_corr


def _apply_correction_to_images(images, R_corr, t_corr, img_ids):
    """Apply (R_corr, t_corr) to camera centers, recompute w2c poses."""
    for img_id in img_ids:
        img = images[img_id]
        R_c2w = _qvec_to_R(img['qvec']).T  # c2w
        C_old = _camera_center(img['qvec'], img['tvec'])
        C_new = R_corr @ C_old + t_corr
        R_c2w_new = R_corr @ R_c2w
        img['qvec'], img['tvec'] = _w2c_from_center_R(C_new, R_c2w_new)


# ---------------------------------------------------------------------------
# Depth re-render
# ---------------------------------------------------------------------------

def _rerender_depth(session_path, sparse_dir):
    """Re-render depth_images from the corrected poses in sparse_dir/images.bin."""
    import importlib.util as _ilu
    _spec = _ilu.spec_from_file_location(
        'generate_colmap_depth',
        str(Path(__file__).resolve().parent / 'generate_colmap_depth.py')
    )
    _mod = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    rel = str(sparse_dir.relative_to(session_path))
    _mod.generate_depth_images(str(session_path), sparse_subdir=rel)


# ---------------------------------------------------------------------------
# Calibration YAML write-back
# ---------------------------------------------------------------------------

def _write_calib_correction(session_path, R_corr_x5):
    """Apply R_corr_x5 (COLMAP axes) to the X5 calibration YAML as a manual
    adjustment on top of the existing offsets."""
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from camera_hw import calibration_path, load_calibration_transform, _to_plain
    from scipy.spatial.transform import Rotation as _R

    # Convert correction from COLMAP axes back to ROS axes
    R_corr_ros = R_ROS2COLMAP.T @ R_corr_x5 @ R_ROS2COLMAP

    calib_p = calibration_path('x5', 0)
    calib = yaml.safe_load(calib_p.read_text())

    # Current total rotation
    roll  = calib['roll_offset']  + calib.get('manual_roll_adjustment',  0.0)
    pitch = calib['pitch_offset'] + calib.get('manual_pitch_adjustment', 0.0)
    yaw   = calib['yaw_offset']   + calib.get('manual_yaw_adjustment',   0.0)
    R_current = _R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    # New total rotation
    R_new = R_corr_ros @ R_current
    rpy_new = _R.from_matrix(R_new).as_euler('xyz')

    # Store delta as manual adjustments
    calib['manual_roll_adjustment']  = float(rpy_new[0] - calib['roll_offset'])
    calib['manual_pitch_adjustment'] = float(rpy_new[1] - calib['pitch_offset'])
    calib['manual_yaw_adjustment']   = float(rpy_new[2] - calib['yaw_offset'])

    calib_p.write_text(yaml.dump(_to_plain(calib), default_flow_style=False))
    delta_deg = np.degrees(np.linalg.norm(
        _R.from_matrix(R_corr_ros).as_rotvec()))
    print(f'  Wrote X5 calibration correction ({delta_deg:.2f}°) -> {calib_p.name}')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def calibrate_rig_lidar(session_path, dry_run=False, write_calib=False):
    sparse_dir = session_path / 'colmap' / 'sparse' / '0'
    if not sparse_dir.exists():
        print('No sparse model found.')
        return False

    images_bin  = sparse_dir / 'images.bin'
    rigs_bin    = sparse_dir / 'rigs.bin'
    frames_bin  = sparse_dir / 'frames.bin'

    if not images_bin.exists():
        print('images.bin not found.')
        return False

    images = _read_images_bin(images_bin)
    print(f'  Loaded {len(images)} images from images.bin')

    # Load LiDAR-derived centers for all images
    lidar_centers = _load_lidar_centers(session_path, images, R_ROS2COLMAP)
    print(f'  LiDAR centers loaded for {len(lidar_centers)} images')

    if len(lidar_centers) < 3:
        print('  ✗ Too few LiDAR centers — skipping calibration')
        return False

    # Compute SfM centers from current poses
    sfm_centers = {iid: _camera_center(img['qvec'], img['tvec'])
                   for iid, img in images.items()}

    # Partition images by body
    x5_ids  = [iid for iid, img in images.items()
                if _body_for_image(img['name']) == 'x5' and iid in lidar_centers]
    oak1_ids = [iid for iid, img in images.items()
                if _body_for_image(img['name']) == 'oak1' and iid in lidar_centers]

    if len(x5_ids) < 3:
        print(f'  ✗ Only {len(x5_ids)} X5 images with LiDAR centers — skipping')
        return False

    # Fit X5 correction
    R_corr_x5, t_corr_x5 = _fit_correction(sfm_centers, lidar_centers, x5_ids)
    angle_x5 = float(np.degrees(np.linalg.norm(
        Rotation.from_matrix(R_corr_x5).as_rotvec())))
    print(f'  X5 correction: {angle_x5:.3f}°  t={np.linalg.norm(t_corr_x5)*100:.1f}cm')

    # Gate: reject implausible corrections
    if angle_x5 > 5.0:
        print(f'  ✗ X5 correction {angle_x5:.2f}° exceeds 5° gate — keeping original poses')
        return False

    if dry_run:
        print('  Dry run — no files written')
        return True

    # Apply X5 correction to all X5 images
    _apply_correction_to_images(images, R_corr_x5, t_corr_x5, x5_ids)

    # Fit and apply OAK-1 correction independently (if present)
    R_corr_oak1 = None
    if len(oak1_ids) >= 3:
        R_corr_oak1, t_corr_oak1 = _fit_correction(sfm_centers, lidar_centers, oak1_ids)
        angle_oak1 = float(np.degrees(np.linalg.norm(
            Rotation.from_matrix(R_corr_oak1).as_rotvec())))
        print(f'  OAK-1 correction: {angle_oak1:.3f}°  t={np.linalg.norm(t_corr_oak1)*100:.1f}cm')
        if angle_oak1 <= 5.0:
            _apply_correction_to_images(images, R_corr_oak1, t_corr_oak1, oak1_ids)
        else:
            print(f'  ✗ OAK-1 correction {angle_oak1:.2f}° exceeds gate — keeping original')
            R_corr_oak1 = None

    # Write corrected images.bin
    _write_images_bin(images_bin, images)
    print(f'  Updated images.bin ({len(images)} images)')

    # Update frames.bin: recompute frame pose from corrected ref-face image
    if frames_bin.exists() and rigs_bin.exists():
        rig_id, ref_sensor_id, non_ref_sensors = _read_rigs_bin(rigs_bin)
        frames = _read_frames_bin(frames_bin)

        # Build sensor_id -> subfolder map from images
        sid_to_sf: dict[int, str] = {}
        # We need the sensor_id -> image_id mapping from frames
        # Build img_id -> sensor_id from frame data
        imgid_to_sid: dict[int, int] = {}
        for _, _, _, data in frames:
            for st, sid, did in data:
                imgid_to_sid[did] = sid

        # Recompute frame pose from the corrected ref-face image pose
        updated_frames = []
        for fid, rid, pose, data in frames:
            ref_img_id = next(
                (did for st, sid, did in data if sid == ref_sensor_id), None)
            if ref_img_id is not None and ref_img_id in images:
                img = images[ref_img_id]
                qw, qx, qy, qz = img['qvec']
                tx, ty, tz = img['tvec']
                new_pose = [qw, qx, qy, qz, tx, ty, tz]
            else:
                new_pose = pose
            updated_frames.append((fid, rid, new_pose, data))
        _write_frames_bin(frames_bin, updated_frames)
        print(f'  Updated frames.bin ({len(updated_frames)} frames)')

        # Update OAK-1 sensor_from_rig in rigs.bin if correction was applied
        if R_corr_oak1 is not None and non_ref_sensors:
            # Find the OAK-1 sensor by checking which sensor_id maps to face_oak1 images
            oak1_sid = next(
                (imgid_to_sid[iid] for iid in oak1_ids if iid in imgid_to_sid), None)
            if oak1_sid is not None:
                updated_non_ref = []
                for sid, has_pose, sfr in non_ref_sensors:
                    if sid == oak1_sid and sfr is not None:
                        # Apply R_corr_oak1 to the existing sensor_from_rig rotation
                        qw, qx, qy, qz = sfr[0], sfr[1], sfr[2], sfr[3]
                        R_sfr = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                        R_sfr_new = R_corr_oak1 @ R_sfr
                        q_new = Rotation.from_matrix(R_sfr_new).as_quat()
                        if q_new[3] < 0: q_new = -q_new
                        sfr_new = [q_new[3], q_new[0], q_new[1], q_new[2],
                                   sfr[4], sfr[5], sfr[6]]
                        updated_non_ref.append((sid, has_pose, sfr_new))
                    else:
                        updated_non_ref.append((sid, has_pose, sfr))
                _write_rigs_bin(rigs_bin, rig_id, ref_sensor_id, updated_non_ref)
                print('  Updated rigs.bin (OAK-1 sensor_from_rig corrected)')

    # Write report
    report = {
        'x5_correction_deg': angle_x5,
        'x5_correction_t_cm': float(np.linalg.norm(t_corr_x5) * 100),
        'n_x5_images': len(x5_ids),
        'n_oak1_images': len(oak1_ids),
        'gate_passed': True,
    }
    report_path = session_path / 'colmap' / 'calibrate_rig_report.json'
    report_path.write_text(json.dumps(report, indent=2))
    print(f'  Report: {report_path.name}')

    # Re-render depth images with corrected poses
    print('  Re-rendering depth images with corrected poses...')
    _rerender_depth(session_path, sparse_dir)

    # Optionally write X5 correction back to calibration YAML
    if write_calib:
        _write_calib_correction(session_path, R_corr_x5)

    print('✓ Rig calibration complete')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--write-calib', action='store_true',
                        help='Write X5 correction back to calibration YAML')
    parser.add_argument('--dry-run', action='store_true',
                        help='Compute corrections but do not write any files')
    args = parser.parse_args()

    try:
        session_path = _safe_data(args.session_dir)
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)

    pose_source_f = session_path / 'colmap' / 'pose_source.json'
    if not pose_source_f.exists():
        print('No pose_source.json found — mobile mode not implemented here')
        sys.exit(0)

    try:
        ps = json.loads(pose_source_f.read_text()).get('pose_source', '')
    except (OSError, json.JSONDecodeError):
        ps = ''

    if ps != 'lidar':
        print(f'pose_source={ps!r} — only "lidar" mode is handled here')
        sys.exit(0)

    print(f'LiDAR-referenced rig calibration: {session_path.name}')
    ok = calibrate_rig_lidar(session_path,
                              dry_run=args.dry_run,
                              write_calib=args.write_calib)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
