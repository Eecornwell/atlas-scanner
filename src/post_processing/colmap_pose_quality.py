#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Analyses COLMAP sparse model quality for a session.
# Reports per-image reprojection error, track length, observation count,
# compares LiDAR-derived init poses vs BA-refined poses, and flags
# outlier images that are degrading reconstruction quality.
#
# Usage:
#   python3 colmap_pose_quality.py <session_dir> [--init-sparse init_sparse/0]

import struct
import json
import argparse
import os
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


# ---------------------------------------------------------------------------
# COLMAP binary readers
# ---------------------------------------------------------------------------

def _read_cameras_bin(path):
    """Return {camera_id: {'model': int, 'w': int, 'h': int, 'params': np.ndarray}}."""
    cameras = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            cam_id = struct.unpack('I', f.read(4))[0]
            model  = struct.unpack('i', f.read(4))[0]
            w, h   = struct.unpack('QQ', f.read(16))
            # SIMPLE_PINHOLE=0 (1 param), PINHOLE=1 (4), SIMPLE_RADIAL=2 (4),
            # RADIAL=3 (5), SPHERICAL=7 (0)
            nparams = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8, 5: 8, 6: 8, 7: 0}.get(model, 0)
            params = np.array(struct.unpack(f'{nparams}d', f.read(8 * nparams)))
            cameras[cam_id] = {'model': model, 'w': w, 'h': h, 'params': params}
    return cameras


def _read_images_bin(path):
    """Return {image_id: {'name', 'qvec', 'tvec', 'camera_id', 'xys', 'p3d_ids'}}."""
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            img_id = struct.unpack('I', f.read(4))[0]
            qw, qx, qy, qz = struct.unpack('dddd', f.read(32))
            tx, ty, tz      = struct.unpack('ddd',  f.read(24))
            cam_id          = struct.unpack('I',    f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00':
                    break
                name += c
            n_pts = struct.unpack('Q', f.read(8))[0]
            xys     = []
            p3d_ids = []
            for _ in range(n_pts):
                x, y  = struct.unpack('dd', f.read(16))
                p3d   = struct.unpack('q', f.read(8))[0]
                xys.append((x, y))
                p3d_ids.append(p3d)
            images[img_id] = {
                'name':      name.decode('utf-8'),
                'qvec':      np.array([qw, qx, qy, qz]),
                'tvec':      np.array([tx, ty, tz]),
                'camera_id': cam_id,
                'xys':       np.array(xys) if xys else np.zeros((0, 2)),
                'p3d_ids':   np.array(p3d_ids, dtype=np.int64),
            }
    return images


def _read_points3d_bin(path):
    """Return {point3d_id: {'xyz', 'rgb', 'error', 'track'}}."""
    points = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            p3d_id       = struct.unpack('Q', f.read(8))[0]
            x, y, z      = struct.unpack('ddd', f.read(24))
            r, g, b      = struct.unpack('BBB', f.read(3))
            error        = struct.unpack('d', f.read(8))[0]
            track_len    = struct.unpack('Q', f.read(8))[0]
            track        = []
            for _ in range(track_len):
                img_id, feat_id = struct.unpack('Ii', f.read(8))
                track.append((img_id, feat_id))
            points[p3d_id] = {'xyz': np.array([x, y, z]), 'rgb': (r, g, b),
                              'error': error, 'track': track, 'track_len': track_len}
    return points


# ---------------------------------------------------------------------------
# Reprojection
# ---------------------------------------------------------------------------

def _project_simple_pinhole(p3d_cam, f, cx, cy):
    if p3d_cam[2] <= 0:
        return None
    x = p3d_cam[0] / p3d_cam[2] * f + cx
    y = p3d_cam[1] / p3d_cam[2] * f + cy
    return np.array([x, y])


def _reproject(p3d_world, R_w2c, t_w2c, camera):
    p_cam = R_w2c @ p3d_world + t_w2c
    model = camera['model']
    params = camera['params']
    if model == 0:  # SIMPLE_PINHOLE
        return _project_simple_pinhole(p_cam, params[0], params[1], params[2])
    if model == 7:  # SPHERICAL — no reprojection error available
        return None
    if model == 1:  # PINHOLE
        if p_cam[2] <= 0:
            return None
        x = p_cam[0] / p_cam[2] * params[0] + params[2]
        y = p_cam[1] / p_cam[2] * params[1] + params[3]
        return np.array([x, y])
    return None


def per_image_reprojection_error(images, points3d, cameras):
    """Return {image_id: mean_reproj_error} for images with triangulated points.
    Only uses points with track_len > 0 (skips LiDAR-injected points)."""
    errors = {}
    for img_id, img in images.items():
        cam  = cameras.get(img['camera_id'])
        if cam is None:
            continue
        R_w2c = Rotation.from_quat([img['qvec'][1], img['qvec'][2],
                                     img['qvec'][3], img['qvec'][0]]).as_matrix()
        t_w2c = img['tvec']
        errs  = []
        for (x_obs, y_obs), p3d_id in zip(img['xys'], img['p3d_ids']):
            if p3d_id < 0 or p3d_id not in points3d:
                continue
            if points3d[p3d_id]['track_len'] == 0:  # LiDAR-injected, skip
                continue
            proj = _reproject(points3d[p3d_id]['xyz'], R_w2c, t_w2c, cam)
            if proj is None:
                continue
            errs.append(np.linalg.norm(proj - np.array([x_obs, y_obs])))
        if errs:
            errors[img_id] = np.array(errs)
    return errors


# ---------------------------------------------------------------------------
# Pose comparison (init vs refined)
# ---------------------------------------------------------------------------

def _camera_center(qvec, tvec):
    """Camera center in world frame: C = -R^T @ t."""
    R_w2c = Rotation.from_quat([qvec[1], qvec[2], qvec[3], qvec[0]]).as_matrix()
    return -R_w2c.T @ tvec


def compare_poses(init_images, refined_images):
    """Return list of (name, dt_trans_m, dt_rot_deg) for matching image names."""
    name_to_init = {v['name']: v for v in init_images.values()}
    results = []
    for img in refined_images.values():
        init = name_to_init.get(img['name'])
        if init is None:
            continue
        C_init     = _camera_center(init['qvec'], init['tvec'])
        C_refined  = _camera_center(img['qvec'],  img['tvec'])
        dt_t = float(np.linalg.norm(C_refined - C_init))

        R_init     = Rotation.from_quat([init['qvec'][1], init['qvec'][2],
                                          init['qvec'][3], init['qvec'][0]])
        R_refined  = Rotation.from_quat([img['qvec'][1],  img['qvec'][2],
                                          img['qvec'][3],  img['qvec'][0]])
        dR = R_refined * R_init.inv()
        dt_r = float(np.degrees(np.linalg.norm(dR.as_rotvec())))
        results.append({'name': img['name'], 'dt_trans_m': dt_t, 'dt_rot_deg': dt_r})
    return results


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def analyse(session_dir, init_subdir='init_sparse/0', top_n=5):
    try:
        session_path = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return

    sparse_dir = session_path / 'colmap' / 'sparse' / '0'
    if not sparse_dir.exists():
        print(f"No sparse model at {sparse_dir}")
        return

    print(f"\n=== COLMAP Pose Quality: {session_path.name} ===\n")

    cameras  = _read_cameras_bin(sparse_dir / 'cameras.bin')
    images   = _read_images_bin(sparse_dir  / 'images.bin')
    points3d = _read_points3d_bin(sparse_dir / 'points3D.bin')

    n_registered = len(images)
    n_points     = len(points3d)
    n_triangulated = sum(1 for p in points3d.values() if p['track_len'] > 0)
    n_lidar_injected = n_points - n_triangulated
    obs_total    = sum(len(v['track']) for v in points3d.values())
    mean_track   = obs_total / max(n_triangulated, 1)
    sfm_errors   = np.array([p['error'] for p in points3d.values() if p['track_len'] > 0])

    print(f"Registered images : {n_registered}")
    print(f"3D points         : {n_points}  "
          f"(triangulated={n_triangulated}, lidar_injected={n_lidar_injected})")
    if n_triangulated > 0:
        print(f"Mean track length : {mean_track:.2f}")
        print(f"Reproj error (SfM): mean={sfm_errors.mean():.2f}px  "
              f"median={np.median(sfm_errors):.2f}px  "
              f"p95={np.percentile(sfm_errors, 95):.2f}px")
    else:
        print("No triangulated points — points3D.bin contains only LiDAR-injected points.")
        print("Reprojection error stats require COLMAP-triangulated points.")

    # Per-image reprojection error
    img_errors = per_image_reprojection_error(images, points3d, cameras)
    if img_errors:
        means = {iid: e.mean() for iid, e in img_errors.items()}
        obs   = {iid: len(e)   for iid, e in img_errors.items()}
        print(f"\n--- Per-image reprojection error (top {top_n} worst) ---")
        for iid in sorted(means, key=means.get, reverse=True)[:top_n]:
            print(f"  {images[iid]['name']:40s}  mean={means[iid]:.2f}px  "
                  f"obs={obs[iid]}")
        overall_mean = np.concatenate(list(img_errors.values())).mean()
        print(f"\nOverall mean per-image reproj error: {overall_mean:.3f}px")

    # Observations per image (proxy for match quality)
    print(f"\n--- Observations per image ---")
    img_obs = {iid: int((img['p3d_ids'] >= 0).sum()) for iid, img in images.items()}
    obs_vals = list(img_obs.values())
    if obs_vals:
        print(f"  mean={np.mean(obs_vals):.0f}  "
              f"min={min(obs_vals)}  max={max(obs_vals)}  "
              f"median={np.median(obs_vals):.0f}")
        low_obs = [(images[iid]['name'], v) for iid, v in img_obs.items()
                   if v < np.percentile(obs_vals, 20)]
        if low_obs:
            print(f"  Low-observation images (bottom 20%):")
            for name, v in sorted(low_obs, key=lambda x: x[1])[:top_n]:
                print(f"    {name:40s}  obs={v}")

    # Init vs refined pose drift
    init_dir = session_path / 'colmap' / init_subdir
    if init_dir.exists() and (init_dir / 'images.bin').exists():
        init_images = _read_images_bin(init_dir / 'images.bin')
        drift = compare_poses(init_images, images)
        if drift:
            trans = np.array([d['dt_trans_m']  for d in drift])
            rots  = np.array([d['dt_rot_deg']  for d in drift])
            print(f"\n--- Init → BA pose drift ({len(drift)} images) ---")
            print(f"  Translation: mean={trans.mean()*100:.1f}cm  "
                  f"median={np.median(trans)*100:.1f}cm  "
                  f"max={trans.max()*100:.1f}cm")
            print(f"  Rotation:    mean={rots.mean():.2f}°  "
                  f"median={np.median(rots):.2f}°  "
                  f"max={rots.max():.2f}°")
            print(f"\n  Top {top_n} largest translation drifts (LiDAR pose quality):")
            for d in sorted(drift, key=lambda x: x['dt_trans_m'], reverse=True)[:top_n]:
                print(f"    {d['name']:40s}  Δt={d['dt_trans_m']*100:.1f}cm  "
                      f"Δr={d['dt_rot_deg']:.2f}°")

    # Camera model summary
    print(f"\n--- Camera models ---")
    model_names = {0: 'SIMPLE_PINHOLE', 1: 'PINHOLE', 2: 'SIMPLE_RADIAL',
                   3: 'RADIAL', 7: 'SPHERICAL'}
    for cid, cam in cameras.items():
        mname = model_names.get(cam['model'], f"model_{cam['model']}")
        print(f"  cam_id={cid}  {mname}  {cam['w']}x{cam['h']}  "
              f"params={np.round(cam['params'], 2).tolist()}")

    # Save JSON report
    report = {
        'session': session_path.name,
        'n_registered': n_registered,
        'n_points_triangulated': n_triangulated,
        'n_points_lidar_injected': n_lidar_injected,
        'mean_track_length': round(mean_track, 3),
        'reproj_error_mean_px': round(float(sfm_errors.mean()), 3) if sfm_errors.size else None,
        'reproj_error_p95_px': round(float(np.percentile(sfm_errors, 95)), 3) if sfm_errors.size else None,
        'per_image_reproj': {
            images[iid]['name']: round(float(e.mean()), 3)
            for iid, e in img_errors.items()
        } if img_errors else {},
        'pose_drift': drift if 'drift' in dir() else [],
    }
    report_path = session_path / 'colmap' / 'pose_quality_report.json'
    report_path.write_text(json.dumps(report, indent=2))
    print(f"\n✓ Report saved: {report_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--init-sparse', default='init_sparse/0',
                        help='Subdirectory under colmap/ containing the init model '
                             '(default: init_sparse/0)')
    parser.add_argument('--top', type=int, default=5,
                        help='Number of worst images to show in each table (default: 5)')
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        import sys; sys.exit(1)
    analyse(args.session_dir, args.init_sparse, args.top)
