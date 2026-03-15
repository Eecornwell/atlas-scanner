#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Converts a saved dual-fisheye JPG to an equirectangular image offline, replicating the same projection and calibration parameters used by the live equirectangular ROS node.
"""
Convert a saved dual-fisheye JPG to equirectangular (ERP) offline,
using the same projection logic as equirectangular.py.
"""

import argparse
import cv2
import numpy as np
import math
import yaml
import sys
import os


def load_erp_config(config_path):
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    p = cfg.get('equirectangular_node', {}).get('ros__parameters', cfg)
    rot = p.get('rotation_deg', [0.0, 0.0, 0.0])
    return {
        'cx_offset': p.get('cx_offset', 0.0),
        'cy_offset': p.get('cy_offset', 0.0),
        'crop_size': p.get('crop_size', 960),
        'translation': p.get('translation', [0.0, 0.0, 0.0]),
        'rotation_deg': rot,
        'out_width': p.get('out_width', 2560),
        'out_height': p.get('out_height', 1280),
    }


def build_maps(cfg, crop):
    out_w, out_h = cfg['out_width'], cfg['out_height']
    cx = crop / 2 + cfg['cx_offset']
    cy = crop / 2 + cfg['cy_offset']

    y_idx, x_idx = np.meshgrid(np.arange(out_h, dtype=np.float32),
                                np.arange(out_w, dtype=np.float32), indexing='ij')
    lon = (x_idx / out_w) * 2 * math.pi - math.pi
    lat = (y_idx / out_h) * math.pi - math.pi / 2

    X = np.cos(lat) * np.sin(lon)
    Y = np.sin(lat)
    Z = np.cos(lat) * np.cos(lon)

    front_mask = Z >= 0
    back_mask = ~front_mask

    def fisheye_uv(Xf, Yf, Zf):
        r = np.sqrt(Xf**2 + Yf**2).clip(1e-6)
        theta = np.arctan2(r, np.abs(Zf))
        r_img = 2 * theta / math.pi * (crop / 2)
        u = (cx + Xf / r * r_img).astype(np.float32)
        v = (cy + Yf / r * r_img).astype(np.float32)
        return u, v

    # Back-to-front rotation from equirectangular.yaml
    tx, ty, tz = cfg['translation']
    roll, pitch, yaw = [math.radians(d) for d in cfg['rotation_deg']]
    Rx = np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
    Ry = np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
    Rz = np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    R = Rz @ Ry @ Rx
    t = np.array([tx, ty, tz])

    # Front hemisphere -> front lens (right half, rotated 90° CCW)
    front_map_x = np.zeros((out_h, out_w), np.float32)
    front_map_y = np.zeros((out_h, out_w), np.float32)
    front_map_x[front_mask], front_map_y[front_mask] = fisheye_uv(
        X[front_mask], Y[front_mask], Z[front_mask])

    # Back hemisphere -> back lens (left half, rotated 90° CW), with back-to-front transform
    bp = np.stack([X[back_mask], Y[back_mask], Z[back_mask]], axis=1)
    tp = bp @ R.T + t
    Xb, Yb, Zb = -tp[:, 0], tp[:, 1], tp[:, 2]
    back_map_x = np.zeros((out_h, out_w), np.float32)
    back_map_y = np.zeros((out_h, out_w), np.float32)
    back_map_x[back_mask], back_map_y[back_mask] = fisheye_uv(Xb, Yb, Zb)

    return front_map_x, front_map_y, front_mask, back_map_x, back_map_y, back_mask


def fisheye_jpg_to_erp(fisheye_path, config_path, output_path, dual=False):
    cfg = load_erp_config(config_path)
    crop = cfg['crop_size']
    out_w, out_h = cfg['out_width'], cfg['out_height']

    img = cv2.imread(fisheye_path)
    if img is None:
        print(f"Warning: Cannot read: {fisheye_path}, skipping")
        return None

    if dual:
        # Full side-by-side dual-fisheye: split, rotate, map both hemispheres
        h, w = img.shape[:2]
        mid = w // 2
        front_raw = cv2.rotate(img[:, mid:], cv2.ROTATE_90_COUNTERCLOCKWISE)
        back_raw  = cv2.rotate(img[:, :mid], cv2.ROTATE_90_CLOCKWISE)

        def center_crop(im, size):
            ih, iw = im.shape[:2]
            y0 = (ih - size) // 2
            x0 = (iw - size) // 2
            return im[y0:y0+size, x0:x0+size]

        crop = min(crop, front_raw.shape[0], front_raw.shape[1])
        front = center_crop(front_raw, crop)
        back  = center_crop(back_raw,  crop)

        fmx, fmy, front_mask, bmx, bmy, back_mask = build_maps(cfg, crop)
        front_remap = cv2.remap(front, fmx, fmy, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
        back_remap  = cv2.remap(back,  bmx, bmy, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
        erp = np.where(front_mask[:, :, None], front_remap, back_remap).astype(np.uint8)
    else:
        # Single pre-extracted fisheye (already cropped/rotated by capture script)
        h, w = img.shape[:2]
        crop = min(crop, h, w)
        y0 = (h - crop) // 2
        x0 = (w - crop) // 2
        fisheye = img[y0:y0+crop, x0:x0+crop]

        # The single fisheye is the back lens saved with the same 90° CW rotation as the
        # dual path. Use the dual back maps directly with identity transform (zero rotation
        # and translation from equirectangular_single.yaml).
        _, _, _, bmx, bmy, back_mask = build_maps(cfg, crop)
        remapped = cv2.remap(fisheye, bmx, bmy, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
        erp = np.where(back_mask[:, :, None], remapped, 0).astype(np.uint8)

    cv2.imwrite(output_path, erp)
    print(f"\u2713 ERP saved: {output_path} ({out_w}x{out_h})")
    return output_path


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='Input fisheye JPG')
    parser.add_argument('output', help='Output ERP JPG')
    parser.add_argument('--config', help='Path to equirectangular yaml')
    parser.add_argument('--dual', action='store_true',
                        help='Use dual-fisheye calibration (translation/rotation from yaml). '
                             'Omit for single-lens (zeroed alignment).')
    args = parser.parse_args()

    cfg_dir = os.path.expanduser('~/atlas_ws/src/insta360_ros_driver/config')
    atlas_cfg_dir = os.path.expanduser('~/atlas_ws/src/atlas-scanner/src/config')
    if args.config:
        cfg_path = args.config
    elif args.dual:
        cfg_path = os.path.join(cfg_dir, 'equirectangular.yaml')
    else:
        cfg_path = os.path.join(atlas_cfg_dir, 'equirectangular_single.yaml')

    fisheye_jpg_to_erp(args.input, cfg_path, args.output, dual=args.dual)
