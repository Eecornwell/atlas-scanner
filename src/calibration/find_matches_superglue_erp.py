#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: SuperGlue matching via perspective crops extracted from ERP images.
# Extracts perspective crops from both camera and lidar ERPs, runs SuperGlue on
# each crop pair, then maps matched keypoints back to ERP pixel coordinates for
# direct_visual_lidar_calibration.
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
       Matches generate_intensity_images.py SDK stitch formula:
       lat = arcsin(Y), lon = atan2(Z, X) + pi/2
    """
    X, Y, Z = rays[..., 0], rays[..., 1], rays[..., 2]
    lat = np.arcsin(np.clip(-Y, -1, 1))
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
    parser.add_argument('--superglue', choices={'indoor', 'outdoor'}, default='indoor')
    parser.add_argument('--max_keypoints', type=int, default=1024)
    parser.add_argument('--keypoint_threshold', type=float, default=0.005)
    parser.add_argument('--nms_radius', type=int, default=4)
    parser.add_argument('--match_threshold', type=float, default=0.2)
    parser.add_argument('--fov', type=float, default=90.0)
    parser.add_argument('--crop_size', type=int, default=512)
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

    # Detect SDK stitch from sidecar files
    import json as _json
    from pathlib import Path as _P
    first_sidecar = _P(data_path) / '000000_source.json'
    is_sdk = False
    if first_sidecar.exists():
        sd = _json.load(open(first_sidecar))
        scan_path = _P(sd.get('scan_dir', ''))
        is_sdk = bool(list(scan_path.glob('*.insp'))) if scan_path.is_dir() else False
    if is_sdk:
        print('Detected SDK stitch mode — using perspective crop matching')

    # Perspective crop directions: sample full 360 at multiple pitches for maximum coverage
    crop_directions = [
        (0,   0), (90,  0), (180,  0), (270,  0),
        (45,  0), (135, 0), (225,  0), (315,  0),
        (0,  25), (90, 25), (180, 25), (270, 25),
        (0, -25), (90,-25), (180,-25), (270,-25),
    ]

    for bag_name in calib['meta']['bag_names']:
        print(f'\nProcessing {bag_name}')

        cam_erp = cv2.imread(f'{data_path}/{bag_name}.png', cv2.IMREAD_GRAYSCALE)
        lid_erp = cv2.imread(f'{data_path}/{bag_name}_lidar_intensities.png', cv2.IMREAD_GRAYSCALE)
        if cam_erp is None or lid_erp is None:
            print(f'  Missing images, skipping')
            continue
        if lid_erp.shape != cam_erp.shape:
            lid_erp = cv2.resize(lid_erp, (cam_erp.shape[1], cam_erp.shape[0]), interpolation=cv2.INTER_AREA)

        if is_sdk:
            lid_erp = np.roll(np.flipud(np.fliplr(lid_erp)), lid_erp.shape[1] // 2, axis=1)

        H, W = cam_erp.shape
        matching = Matching(config).eval().to(device)
        keys = ['keypoints', 'scores', 'descriptors']

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
