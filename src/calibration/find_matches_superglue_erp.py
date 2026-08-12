#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: SuperGlue matching for ERP camera/lidar image pairs.
# Two modes:
#   --mode crops     (default) Extract perspective crops from both ERPs, run
#                    SuperGlue on each crop pair, map keypoints back to ERP coords.
#   --mode panorama  Run SuperGlue directly on the full downscaled ERP images.
#                    Better when perspective crops miss features or produce poor
#                    matches due to distortion at the crop boundaries.
#
# WARNING: SuperGlue is for non-commercial research use only.
# https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE

import sys
import cv2
import json
import numpy as np
import argparse
import torch
import matplotlib
from models.matching import Matching
from models.utils import frame2tensor


def ray_to_erp(rays, W, H):
    """Convert unit rays to ERP pixel coords using equirectangular.hpp convention:
       lat = -asin(Y), lon = atan2(X, Z)
    """
    X, Y, Z = rays[..., 0], rays[..., 1], rays[..., 2]
    lat = -np.arcsin(np.clip(Y, -1, 1))
    lon = np.arctan2(X, Z)
    u = W * (0.5 + lon / (2 * np.pi))
    v = H * (0.5 - lat / np.pi)
    return u, v


def ray_to_erp_sdk(rays, W, H):
    """Convert unit rays to SDK-stitch lidar intensity ERP pixel coords.
       Since the intensity image is projected into camera frame using the seed
       calibration, this uses the same formula as ray_to_erp (equirectangular.hpp).
    """
    X, Y, Z = rays[..., 0], rays[..., 1], rays[..., 2]
    lat = -np.arcsin(np.clip(Y, -1, 1))
    lon = np.arctan2(X, Z)
    u = W * (0.5 + lon / (2 * np.pi))
    v = H * (0.5 - lat / np.pi)
    return u, v


def extract_perspective_crop(erp_img, yaw_deg, pitch_deg, fov_deg=90, crop_size=512, sdk_stitch=False):
    """Extract a perspective crop from an ERP image.
    Returns (crop, map_x, map_y) where map_x/y give ERP source pixel coords."""
    H, W = erp_img.shape[:2]
    f = crop_size / (2 * np.tan(np.radians(fov_deg / 2)))

    xs = np.arange(crop_size) - crop_size / 2
    ys = np.arange(crop_size) - crop_size / 2
    Xp, Yp = np.meshgrid(xs, ys)
    Zp = np.full_like(Xp, f)

    rays = np.stack([Xp, Yp, Zp], axis=-1)
    rays = rays / np.linalg.norm(rays, axis=-1, keepdims=True)

    pitch = np.radians(pitch_deg)
    yaw   = np.radians(yaw_deg)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(pitch), -np.sin(pitch)],
                   [0, np.sin(pitch),  np.cos(pitch)]])
    Ry = np.array([[ np.cos(yaw), 0, np.sin(yaw)],
                   [0,            1, 0           ],
                   [-np.sin(yaw), 0, np.cos(yaw)]])
    R = Ry @ Rx
    rays_world = (R @ rays.reshape(-1, 3).T).T.reshape(crop_size, crop_size, 3)

    if sdk_stitch:
        map_x, map_y = ray_to_erp_sdk(rays_world, W, H)
    else:
        map_x, map_y = ray_to_erp(rays_world, W, H)
    map_x = map_x.astype(np.float32)
    map_y = map_y.astype(np.float32)
    crop = cv2.remap(erp_img, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
    return crop, map_x, map_y


def crop_kpts_to_erp(kpts, map_x, map_y):
    """Map keypoint pixel coords from crop space back to ERP pixel coords."""
    h, w = map_x.shape
    erp_pts = []
    for kp in kpts:
        cx = int(round(float(kp[0]))); cy = int(round(float(kp[1])))
        cx = np.clip(cx, 0, w - 1);   cy = np.clip(cy, 0, h - 1)
        erp_pts.append([float(map_x[cy, cx]), float(map_y[cy, cx])])
    return erp_pts


def main():
    print('\033[93m' + '*' * 96 + '\033[0m')
    print('\033[93m* WARNING: SuperGlue is not allowed for commercial use. Check licensing conditions. *\033[0m')
    print('\033[93m' + '*' * 96 + '\033[0m')

    parser = argparse.ArgumentParser()
    parser.add_argument('data_path')
    parser.add_argument('--mode', choices={'crops', 'panorama'}, default='crops',
                        help='crops: perspective crop matching (default); '
                             'panorama: match directly on full downscaled ERP images')
    parser.add_argument('--superglue', choices={'indoor', 'outdoor'}, default='indoor')
    parser.add_argument('--max_keypoints', type=int, default=1024)
    parser.add_argument('--keypoint_threshold', type=float, default=0.005)
    parser.add_argument('--nms_radius', type=int, default=4)
    parser.add_argument('--match_threshold', type=float, default=0.2)
    parser.add_argument('--fov', type=float, default=90.0)
    parser.add_argument('--crop_size', type=int, default=512)
    parser.add_argument('--panorama_size', type=int, default=1024,
                        help='Width to downscale ERP to for panorama mode (height = width/2)')
    parser.add_argument('--ransac_threshold', type=float, default=8.0,
                        help='RANSAC reprojection threshold in pixels for outlier filtering (panorama mode)')
    parser.add_argument('--force_cpu', action='store_true')
    opt = parser.parse_args()
    print(opt)

    torch.set_grad_enabled(False)
    device = 'cuda' if torch.cuda.is_available() and not opt.force_cpu else 'cpu'
    print(f'Running on device: {device}')

    config = {
        'superpoint': {
            'nms_radius': opt.nms_radius,
            'keypoint_threshold': opt.keypoint_threshold,
            'max_keypoints': opt.max_keypoints,
        },
        'superglue': {
            'weights': opt.superglue,
            'sinkhorn_iterations': 20,
            'match_threshold': opt.match_threshold,
        }
    }

    data_path = opt.data_path
    with open(f'{data_path}/calib.json') as f:
        calib = json.load(f)

    # Detect SDK stitch and OAK-1 from sidecar files
    import json as _json
    from pathlib import Path as _P
    first_sidecar = _P(data_path) / '000000_source.json'
    is_sdk = False
    is_oak1_session = False
    if first_sidecar.exists():
        sd = _json.load(open(first_sidecar))
        scan_path = _P(sd.get('scan_dir', ''))
        is_sdk = bool(list(scan_path.glob('*.insp'))) if scan_path.is_dir() else False
        is_oak1_session = sd.get('camera_hw', '') == 'oak1'
    if is_sdk:
        print('Detected SDK stitch mode — using perspective crop matching')
    if is_oak1_session:
        print('Detected OAK-1 session — using direct image matching (no ERP crops)')
        opt.mode = 'panorama'  # reuse panorama path: direct full-image match, no ERP reprojection
        # For oak1 override panorama_size to match the actual image dimensions
        # so we don\'t resize a pinhole image to ERP dimensions
        opt.panorama_size = -1  # sentinel: use native image size    # Read seed calibration yaw to bias crop directions toward camera FOV.
    # When the camera has a large yaw offset (e.g. cam_1 faces right at -90°),
    # the LiDAR intensity image is projected into that rotated frame, so content
    # appears at a different ERP location than for a forward-facing camera.
    # We add the seed yaw offset to all crop directions so SuperGlue samples
    # where camera and LiDAR content actually overlap.
    #
    # Priority order for calibration file:
    #   1. ATLAS_CALIBRATION_FILE env var (set by calibrate_camera.sh to the
    #      exact per-slot path used by generate_intensity_images.py)
    #   2. ATLAS_CALIBRATION_CAM_INDEX + multi_camera.yaml slot lookup
    #   3. hw-level file detected from session_config.json (old fallback)
    seed_yaw_deg = 0.0
    try:
        import yaml as _yaml
        import glob as _glob
        import os as _os
        _src_root = _P.home() / 'atlas_ws/src/atlas-scanner/src'
        _x5_path = _src_root / 'config' / 'calibrations' / 'x5' / 'fusion_calibration.yaml'
        _x5_yaw = _yaml.safe_load(_x5_path.read_text())['yaw_offset'] if _x5_path.exists() else 0.0

        _calib_path = None

        # Priority 1: explicit path set by calibrate_camera.sh
        _env_file = _os.environ.get('ATLAS_CALIBRATION_FILE', '')
        if _env_file and _P(_env_file).exists():
            _calib_path = _P(_env_file)
            print(f'Seed calibration (ATLAS_CALIBRATION_FILE): {_calib_path}')

        # Priority 2: cam index → multi_camera.yaml slot path
        if _calib_path is None:
            _cam_idx = _os.environ.get('ATLAS_CALIBRATION_CAM_INDEX', '')
            if _cam_idx:
                _mc_path = _src_root / 'config' / 'multi_camera.yaml'
                if _mc_path.exists():
                    _mc = _yaml.safe_load(_mc_path.read_text()) or {}
                    # Try exact slot first, then find by hw match
                    _slot_cfg = _mc.get('cameras', {}).get(f'cam_{_cam_idx}', {})
                    if not _slot_cfg:
                        # cam_idx is the dropdown index (0), find the real slot by hw
                        _src_jsons2 = sorted(_glob.glob(f'{data_path}/*_source.json'))
                        _sess_hw = ''
                        if _src_jsons2:
                            _s = _json.load(open(_src_jsons2[0]))
                            _sess_hw = _s.get('camera_hw', '')
                        if _sess_hw:
                            _slot_cfg = next(
                                (v for v in _mc.get('cameras', {}).values()
                                 if v.get('camera_hw') == _sess_hw), {})
                    _calib_rel = _slot_cfg.get('calibration', '')
                    if _calib_rel:
                        _slot_path = _src_root / 'config' / _calib_rel
                        if _slot_path.exists():
                            _calib_path = _slot_path
                            print(f'Seed calibration (cam_{_cam_idx} slot): {_calib_path}')

        # Priority 3: hw-level file from session_config.json
        if _calib_path is None:
            _src_jsons = sorted(_glob.glob(f'{data_path}/*_source.json'))
            _hw = 'x5'
            if _src_jsons:
                _src = _json.load(open(_src_jsons[0]))
                _sc = _P(_src.get('scan_dir', '')) / '..' / 'session_config.json'
                if _sc.exists():
                    _hw = _json.loads(_sc.read_text()).get('camera_hw', 'x5')
            _hw_path = _src_root / 'config' / 'calibrations' / _hw / 'fusion_calibration.yaml'
            _calib_path = _hw_path if _hw_path.exists() else _src_root / 'config' / 'fusion_calibration.yaml'
            print(f'Seed calibration (hw fallback, {_hw}): {_calib_path}')

        if _calib_path and _calib_path.exists():
            _calib = _yaml.safe_load(_calib_path.read_text())
            seed_yaw_deg = float(np.degrees(_calib['yaw_offset'] - _x5_yaw))
            # OAK-1: yaw offset is irrelevant since we match full image directly
            if is_oak1_session:
                seed_yaw_deg = 0.0
            print(f'Seed yaw offset: {seed_yaw_deg:.1f}° — biasing crop directions')
    except Exception as e:
        print(f'Could not read seed yaw: {e}')

    # Perspective crop directions: sample full 360 at multiple pitches.
    # Rotated by seed_yaw_deg so crops are centred on the camera's actual FOV.
    _base_yaws = [0, 90, 180, 270, 45, 135, 225, 315, 0, 90, 180, 270, 0, 90, 180, 270]
    _pitches   = [0,  0,   0,   0,  0,   0,   0,   0, 25, 25,  25,  25,-25,-25, -25, -25]
    crop_directions = [
        (int((y + seed_yaw_deg) % 360), p)
        for y, p in zip(_base_yaws, _pitches)
    ]

    print(f'Matching mode: {opt.mode}')

    for bag_name in calib['meta']['bag_names']:
        print(f'\nProcessing {bag_name}')

        cam_erp = cv2.imread(f'{data_path}/{bag_name}.png', cv2.IMREAD_GRAYSCALE)
        lid_erp = cv2.imread(f'{data_path}/{bag_name}_lidar_intensities.png', cv2.IMREAD_GRAYSCALE)
        if cam_erp is None or lid_erp is None:
            print(f'  Missing images, skipping')
            continue
        if lid_erp.shape != cam_erp.shape:
            lid_erp = cv2.resize(lid_erp, (cam_erp.shape[1], cam_erp.shape[0]), interpolation=cv2.INTER_AREA)

        H, W = cam_erp.shape
        matching = Matching(config).eval().to(device)
        keys = ['keypoints', 'scores', 'descriptors']

        if opt.mode == 'panorama':
            # ── Panorama / direct match mode ─────────────────────────────────
            # For ERP images: downscale to panorama_size.
            # For OAK-1 pinhole images (panorama_size==-1): use native size
            # downscaled to max 640px wide so SuperPoint runs at a sensible res.
            if opt.panorama_size == -1:
                # OAK-1: use 320x240 — SuperGlue matches better at lower res
                # where the depth/texture difference is less pronounced
                max_w = 320
                scale_x = W / max_w
                scale_y = H / int(H * max_w / W)
                pano_w = max_w
                pano_h = int(H * pano_w / W)
            else:
                pano_w = opt.panorama_size
                pano_h = pano_w // 2
                scale_x = W / pano_w
                scale_y = H / pano_h
            cam_small = cv2.resize(cam_erp, (pano_w, pano_h), interpolation=cv2.INTER_AREA)
            lid_small = cv2.resize(lid_erp, (pano_w, pano_h), interpolation=cv2.INTER_AREA)

            cam_t = frame2tensor(cam_small, device)
            lid_t = frame2tensor(lid_small, device)
            last_data = matching.superpoint({'image': cam_t})
            last_data = {k + '0': last_data[k] for k in keys}
            last_data['image0'] = cam_t
            pred = matching({**last_data, 'image1': lid_t})

            kpts0      = last_data['keypoints0'][0].cpu().numpy()
            kpts1      = pred['keypoints1'][0].cpu().numpy()
            matches    = pred['matches0'][0].cpu().numpy()
            confidence = pred['matching_scores0'][0].cpu().numpy()

            # Scale keypoints back to full ERP coordinates
            kpts0_erp = [[float(kp[0] * scale_x), float(kp[1] * scale_y)] for kp in kpts0]
            kpts1_erp = [[float(kp[0] * scale_x), float(kp[1] * scale_y)] for kp in kpts1]

            n_matched = int((matches >= 0).sum())

            # RANSAC outlier filter on valid matches
            valid_idx = [(i, int(m)) for i, m in enumerate(matches) if m >= 0]
            if len(valid_idx) >= 8:
                pts0 = np.array([kpts0_erp[i] for i, _ in valid_idx], dtype=np.float32)
                pts1 = np.array([kpts1_erp[m] for _, m in valid_idx], dtype=np.float32)
                _, mask = cv2.findFundamentalMat(pts0, pts1, cv2.FM_RANSAC, opt.ransac_threshold, 0.99)
                if mask is not None:
                    inlier_mask = mask.ravel().astype(bool)
                    n_inliers = int(inlier_mask.sum())
                    # Mark outlier matches as -1
                    for k, (i, _) in enumerate(valid_idx):
                        if not inlier_mask[k]:
                            matches[i] = -1
                    print(f'  Direct match: {n_inliers}/{n_matched} matches after RANSAC on {pano_w}x{pano_h}')
                    n_matched = n_inliers
                else:
                    print(f'  Direct match: {n_matched} matches on {pano_w}x{pano_h} (RANSAC skipped)')
            else:
                print(f'  Direct match: {n_matched} matches on {pano_w}x{pano_h} (too few for RANSAC)')

            result = {
                'kpts0':      np.array(kpts0_erp).flatten().tolist() if kpts0_erp else [],
                'kpts1':      np.array(kpts1_erp).flatten().tolist() if kpts1_erp else [],
                'matches':    matches.tolist(),
                'confidence': confidence.tolist(),
            }
            with open(f'{data_path}/{bag_name}_matches.json', 'w') as f:
                json.dump(result, f)

            # Visualization on full ERP
            canvas = np.concatenate([
                cv2.cvtColor(cam_erp, cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(lid_erp, cv2.COLOR_GRAY2BGR),
            ], axis=1)
            kpts0_arr = np.array(kpts0_erp) if kpts0_erp else np.zeros((0, 2))
            kpts1_arr = np.array(kpts1_erp) if kpts1_erp else np.zeros((0, 2))
            conf_norm = confidence / max(confidence.max(), 1e-6)
            cmap = matplotlib.cm.get_cmap('turbo')
            for i, m in enumerate(matches):
                if m < 0 or i >= len(kpts0_arr) or m >= len(kpts1_arr):
                    continue
                p0 = (int(kpts0_arr[i, 0]),     int(kpts0_arr[i, 1]))
                p1 = (int(kpts1_arr[m, 0]) + W, int(kpts1_arr[m, 1]))
                color = tuple(int(c) for c in (np.array(cmap(float(conf_norm[i]))) * 255)[:3])
                cv2.line(canvas, p0, p1, color, 1)
            cv2.imwrite(f'{data_path}/{bag_name}_superglue.png', canvas)
            print(f'  Saved {bag_name}_superglue.png')
            continue

        # ── Crops mode (default) ──────────────────────────────────────────────
        global_kpts0, global_kpts1 = [], []
        global_matches, global_conf = [], []
        offset0, offset1 = 0, 0

        for yaw, pitch in crop_directions:
            cam_crop, cam_mx, cam_my = extract_perspective_crop(cam_erp, yaw, pitch, opt.fov, opt.crop_size)
            lid_crop, lid_mx, lid_my = extract_perspective_crop(lid_erp, yaw, pitch, opt.fov, opt.crop_size)
            if cam_crop.mean() < 2 or lid_crop.mean() < 2:
                continue

            cam_t = frame2tensor(cam_crop, device)
            lid_t = frame2tensor(lid_crop, device)

            last_data = matching.superpoint({'image': cam_t})
            last_data = {k + '0': last_data[k] for k in keys}
            last_data['image0'] = cam_t
            pred = matching({**last_data, 'image1': lid_t})

            kpts0      = last_data['keypoints0'][0].cpu().numpy()
            kpts1      = pred['keypoints1'][0].cpu().numpy()
            matches    = pred['matches0'][0].cpu().numpy()
            confidence = pred['matching_scores0'][0].cpu().numpy()

            erp_kpts0 = crop_kpts_to_erp(kpts0, cam_mx, cam_my)
            erp_kpts1 = crop_kpts_to_erp(kpts1, lid_mx, lid_my)

            n0, n1 = len(kpts0), len(kpts1)
            global_kpts0.extend(erp_kpts0)
            global_kpts1.extend(erp_kpts1)

            crop_matches = np.full(n0, -1, dtype=int)
            for i, m in enumerate(matches):
                if m >= 0:
                    crop_matches[i] = offset1 + int(m)
            global_matches.extend(crop_matches.tolist())
            global_conf.extend(confidence.tolist())

            n_matched = int((matches >= 0).sum())
            print(f'  crop yaw={yaw:3d} pitch={pitch:2d}: {n_matched} matches')

            # Save perspective crop pair with matches drawn
            if n_matched > 0:
                canvas_crop = np.concatenate([cam_crop, lid_crop], axis=1)
                canvas_crop = cv2.cvtColor(canvas_crop, cv2.COLOR_GRAY2BGR)
                for i, m in enumerate(matches):
                    if m < 0: continue
                    p0 = (int(kpts0[i, 0]), int(kpts0[i, 1]))
                    p1 = (int(kpts1[m, 0]) + opt.crop_size, int(kpts1[m, 1]))
                    color = (0, int(confidence[i] * 255), 255 - int(confidence[i] * 255))
                    cv2.line(canvas_crop, p0, p1, color, 1)
                    cv2.circle(canvas_crop, p0, 3, (0, 255, 0), -1)
                    cv2.circle(canvas_crop, p1, 3, (0, 0, 255), -1)
                crop_out = f'{data_path}/{bag_name}_crop_y{yaw:03d}_p{pitch:+03d}.png'
                cv2.imwrite(crop_out, canvas_crop)
            offset0 += n0
            offset1 += n1

        total = sum(1 for m in global_matches if m >= 0)
        print(f'  Total: {total} matches across all crops')

        result = {
            'kpts0':      np.array(global_kpts0).flatten().tolist() if global_kpts0 else [],
            'kpts1':      np.array(global_kpts1).flatten().tolist() if global_kpts1 else [],
            'matches':    global_matches,
            'confidence': global_conf,
        }
        with open(f'{data_path}/{bag_name}_matches.json', 'w') as f:
            json.dump(result, f)

        # Visualization
        canvas = np.concatenate([
            cv2.cvtColor(cam_erp, cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(lid_erp, cv2.COLOR_GRAY2BGR),
        ], axis=1)
        kpts0_arr = np.array(global_kpts0) if global_kpts0 else np.zeros((0, 2))
        kpts1_arr = np.array(global_kpts1) if global_kpts1 else np.zeros((0, 2))
        conf_arr  = np.array(global_conf)
        if len(conf_arr) and conf_arr.max() > 0:
            conf_arr = conf_arr / conf_arr.max()
        cmap = matplotlib.cm.get_cmap('turbo')
        for i, m in enumerate(global_matches):
            if m < 0 or i >= len(kpts0_arr) or m >= len(kpts1_arr):
                continue
            p0 = (int(kpts0_arr[i, 0]),     int(kpts0_arr[i, 1]))
            p1 = (int(kpts1_arr[m, 0]) + W, int(kpts1_arr[m, 1]))
            color = tuple(int(c) for c in (np.array(cmap(float(conf_arr[i]))) * 255)[:3])
            cv2.line(canvas, p0, p1, color, 1)
        cv2.imwrite(f'{data_path}/{bag_name}_superglue.png', canvas)
        print(f'  Saved {bag_name}_superglue.png')


if __name__ == '__main__':
    main()
