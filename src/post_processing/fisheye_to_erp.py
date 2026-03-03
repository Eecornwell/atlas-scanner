#!/usr/bin/env python3
"""
Convert a saved dual-fisheye JPG to equirectangular (ERP) offline,
using the same projection logic as equirectangular.py.
"""

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
        u = cx + Xf / r * r_img
        v = cy + Yf / r * r_img
        return u.astype(np.float32), v.astype(np.float32)

    # The saved fisheye is the back lens - apply back-to-front rotation from equirectangular.yaml
    tx, ty, tz = cfg['translation']
    roll, pitch, yaw = [math.radians(d) for d in cfg['rotation_deg']]
    Rx = np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
    Ry = np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
    Rz = np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    R = Rz @ Ry @ Rx
    t = np.array([tx, ty, tz])

    # Map back hemisphere to the saved fisheye (same logic as equirectangular.py back hemisphere)
    bp = np.stack([X[back_mask], Y[back_mask], Z[back_mask]], axis=1)
    tp = bp @ R.T + t
    Xb, Yb, Zb = -tp[:, 0], tp[:, 1], tp[:, 2]

    back_map_x = np.zeros((out_h, out_w), np.float32)
    back_map_y = np.zeros((out_h, out_w), np.float32)
    back_map_x[back_mask], back_map_y[back_mask] = fisheye_uv(Xb, Yb, Zb)

    return back_map_x, back_map_y, back_mask


def fisheye_jpg_to_erp(fisheye_path, config_path, output_path):
    cfg = load_erp_config(config_path)
    crop = cfg['crop_size']
    out_w, out_h = cfg['out_width'], cfg['out_height']

    # The saved fisheye is the back/LiDAR-facing lens: left half of dual image,
    # already rotated 90° CW by the capture script. Use it as-is.
    img = cv2.imread(fisheye_path)
    if img is None:
        raise FileNotFoundError(f"Cannot read: {fisheye_path}")

    h, w = img.shape[:2]
    # Center-crop to configured crop_size
    y0 = (h - crop) // 2
    x0 = (w - crop) // 2
    fisheye = img[y0:y0+crop, x0:x0+crop]

    fmx, fmy, back_mask = build_maps(cfg, crop)

    remapped = cv2.remap(fisheye, fmx, fmy, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)

    # Back lens maps to left+right edges; front hemisphere (center) stays black
    erp = np.where(back_mask[:, :, None], remapped, 0).astype(np.uint8)

    cv2.imwrite(output_path, erp)
    print(f"✓ ERP saved: {output_path} ({out_w}x{out_h})")
    return output_path


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: fisheye_to_erp.py <dual_fisheye.jpg> <output_erp.jpg> [equirectangular.yaml]")
        sys.exit(1)

    default_cfg = os.path.expanduser(
        '~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml')
    cfg_path = sys.argv[3] if len(sys.argv) > 3 else default_cfg
    fisheye_jpg_to_erp(sys.argv[1], cfg_path, sys.argv[2])
