#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Interactive calibration fine-tuning tool. Sweeps manual_roll/pitch/yaw
# adjustments and generates perspective overlay images so the correct angular offset
# can be found visually without re-running the full calibration pipeline.
# When no scan_dir is given, uses the calibration images in ~/atlas_ws/output/ directly.

import numpy as np
import cv2
import yaml
import sys
import json
import argparse
from pathlib import Path
from scipy.spatial.transform import Rotation as R

CALIB_PATH  = Path(__file__).parent.parent / 'config' / 'fusion_calibration.yaml'
OUTPUT_DIR  = Path.home() / 'atlas_ws/output'


def load_ply(ply_file):
    with open(ply_file, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            header += line
            if line.strip() == b'end_header':
                break
        hdr = header.decode('ascii')
        binary = 'binary_little_endian' in hdr
        n = int(next(l.split()[-1] for l in hdr.splitlines() if l.startswith('element vertex')))
        fields = [l.split()[-1] for l in hdr.splitlines() if l.startswith('property float')]
        if binary:
            data = np.frombuffer(f.read(n * len(fields) * 4), dtype=np.float32).reshape(n, len(fields))
            return data[:, :3]
        pts = []
        for line in f.read().decode('ascii').splitlines():
            p = line.strip().split()
            if len(p) >= 3:
                try: pts.append([float(p[0]), float(p[1]), float(p[2])])
                except ValueError: pass
        return np.array(pts)


def build_T(cfg, dr=0.0, dp=0.0, dy=0.0, dtx=0.0, dty=0.0, dtz=0.0):
    roll  = cfg['roll_offset']  + cfg.get('manual_roll_adjustment',  0.0) + dr
    pitch = cfg['pitch_offset'] + cfg.get('manual_pitch_adjustment', 0.0) + dp
    yaw   = cfg['yaw_offset']   + cfg.get('manual_yaw_adjustment',   0.0) + dy
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3]  = [cfg['x_offset'] + dtx, cfg['y_offset'] + dty, cfg['z_offset'] + dtz]
    return T


def persp_overlay(erp, pts_cam, dist, look_lon_deg, look_lat_deg=0, fov=90, size=800):
    Ry = R.from_euler('y', look_lon_deg, degrees=True).as_matrix()
    Rx = R.from_euler('x', -look_lat_deg, degrees=True).as_matrix()
    R_look = Rx @ Ry
    pts_l = (R_look @ pts_cam.T).T
    front = pts_l[:, 2] > 0.1
    pts_f, dist_f = pts_l[front], dist[front]
    if len(pts_f) == 0:
        return np.zeros((size, size, 3), dtype=np.uint8)

    f = size / (2 * np.tan(np.radians(fov / 2)))
    cx = cy = size / 2
    px = (pts_f[:, 0] / pts_f[:, 2] * f + cx).astype(int)
    py = (-pts_f[:, 1] / pts_f[:, 2] * f + cy).astype(int)
    valid = (px >= 0) & (px < size) & (py >= 0) & (py < size)
    px, py, dist_v = px[valid], py[valid], dist_f[valid]

    h, w = erp.shape[:2]
    uu, vv = np.meshgrid(np.arange(size), np.arange(size))
    rays = np.stack([(uu - cx) / f, -(vv - cy) / f, np.ones((size, size))], axis=-1)
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
    rw = (R_look.T @ rays.reshape(-1, 3).T).T.reshape(size, size, 3)
    lat_bg = -np.arcsin(np.clip(rw[..., 1], -1, 1))
    lon_bg = np.arctan2(rw[..., 0], rw[..., 2])
    eu = (w * (0.5 + lon_bg / (2 * np.pi))).astype(int).clip(0, w - 1)
    ev = (h * (0.5 - lat_bg / np.pi)).astype(int).clip(0, h - 1)
    bg = erp[ev, eu]

    if len(dist_v) == 0:
        return bg
    d_norm = ((dist_v - dist_v.min()) / (dist_v.max() - dist_v.min() + 1e-6) * 255).astype(np.uint8)
    colours = cv2.applyColorMap(d_norm.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
    overlay = bg.copy()
    for i in range(len(px)):
        cv2.circle(overlay, (px[i], py[i]), 3,
                   (int(colours[i, 0]), int(colours[i, 1]), int(colours[i, 2])), -1)
    return cv2.addWeighted(bg, 0.35, overlay, 0.65, 0)


def generate_grid(erp, pts, cfg, dr, dp, dy, dtx, dty, dtz, views, size=600):
    T = build_T(cfg, dr, dp, dy, dtx, dty, dtz)
    pts_h = np.hstack([pts, np.ones((len(pts), 1))])
    pts_cam = (T @ pts_h.T).T[:, :3]
    dist = np.linalg.norm(pts_cam, axis=1)
    mask = (dist > 0.3) & (dist < 10.0)
    pts_cam, dist = pts_cam[mask], dist[mask]

    panels = []
    for name, lon, lat in views:
        p = persp_overlay(erp, pts_cam, dist, lon, lat, size=size)
        cv2.putText(p, name, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        panels.append(p)

    rows = [np.hstack(panels[i:i+2]) for i in range(0, len(panels), 2)]
    return np.vstack(rows)


def main():
    parser = argparse.ArgumentParser(description='Fine-tune extrinsic calibration visually.')
    parser.add_argument('scan_dir', nargs='?', default=None,
                        help='Path to a fusion_scan_* directory. '
                             'If omitted, uses ~/atlas_ws/output/000000.ply + 000000.png')
    parser.add_argument('--axis', choices=['roll', 'pitch', 'yaw', 'tx', 'ty', 'tz'], default='pitch')
    parser.add_argument('--range', type=float, default=2.0,
                        help='Sweep ±range degrees (rotation) or cm (translation)')
    parser.add_argument('--steps', type=int, default=5)
    parser.add_argument('--apply', type=float, default=None,
                        help='Apply this offset to the yaml and exit')
    parser.add_argument('--verify', action='store_true',
                        help='Generate current-state overlay without sweeping')
    args = parser.parse_args()

    with open(CALIB_PATH) as f:
        cfg = yaml.safe_load(f)

    if args.apply is not None:
        if args.axis in ('roll', 'pitch', 'yaw'):
            key = f'manual_{args.axis}_adjustment'
            cfg[key] = float(cfg.get(key, 0.0)) + float(np.radians(args.apply))
            with open(CALIB_PATH, 'w') as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
            print(f'✓ Applied {args.apply}° to {key} -> {np.degrees(cfg[key]):.4f}° total')
        else:
            key = f'{args.axis[1]}_offset'
            cfg[key] = float(cfg.get(key, 0.0)) + float(args.apply / 100.0)
            with open(CALIB_PATH, 'w') as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
            print(f'✓ Applied {args.apply}cm to {key} -> {cfg[key]*100:.2f}cm total')
        return

    # Resolve PLY and image sources
    if args.scan_dir:
        scan_dir = Path(args.scan_dir)
        ply_path = next(scan_dir.glob('sensor_lidar*.ply'), None)
        img_path = next((f for f in sorted(scan_dir.iterdir())
                         if 'equirect' in f.name and f.suffix == '.jpg'), None)
        out_dir  = scan_dir / 'calib_sweep'
        if ply_path is None or img_path is None:
            print(f'Missing sensor_lidar*.ply or equirect*.jpg in {scan_dir}')
            sys.exit(1)
    else:
        # Use calibration dataset directly from output/
        ply_path = OUTPUT_DIR / '000000.ply'
        img_path = OUTPUT_DIR / '000000.png'
        out_dir  = OUTPUT_DIR / 'calib_sweep'
        if not ply_path.exists():
            print(f'No {ply_path} — run combine_scans_for_calibration.py first')
            sys.exit(1)

    out_dir.mkdir(exist_ok=True)

    if not args.verify:
        print(f'PLY:   {ply_path}')
        print(f'Image: {img_path}')

    pts = load_ply(str(ply_path)) if not args.verify or args.scan_dir else None
    if not args.verify or args.scan_dir:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f'Could not load image: {img_path}')
            sys.exit(1)
        erp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img
    else:
        erp = None

    # Views centred on the back hemisphere where LiDAR and single-fisheye camera overlap
    views = [('back-left', -135, 0), ('back-left-down', -135, -20),
             ('back-right', 135, 0), ('back-right-down', 135, -20)]

    if args.verify:
        if args.scan_dir:
            # Single explicit scan dir — original behaviour
            sources = [(ply_path, img_path, scan_dir.name)]
        else:
            # Load up to 3 scans from output/ (000000, 000001, 000002)
            sources = []
            for i in range(3):
                p = OUTPUT_DIR / f'{i:06d}.ply'
                q = OUTPUT_DIR / f'{i:06d}.png'
                if p.exists() and q.exists():
                    sources.append((p, q, f'scan {i:06d}'))
            if not sources:
                print(f'No scans found in {OUTPUT_DIR}')
                sys.exit(1)

        scan_grids = []
        for s_ply, s_img, label in sources:
            print(f'PLY:   {s_ply}')
            print(f'Image: {s_img}')
            s_pts = load_ply(str(s_ply))
            s_raw = cv2.imread(str(s_img))
            if s_raw is None:
                print(f'Could not load image: {s_img}, skipping')
                continue
            s_erp = cv2.cvtColor(s_raw, cv2.COLOR_GRAY2BGR) if s_raw.ndim == 2 else s_raw
            g = generate_grid(s_erp, s_pts, cfg, 0, 0, 0, 0, 0, 0, views)
            cv2.putText(g, label, (10, g.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            scan_grids.append(g)

        out = out_dir / 'current.jpg'
        cv2.imwrite(str(out), np.vstack(scan_grids))
        print(f'\n✓ Saved current calibration overlay ({len(scan_grids)} scan(s)) -> {out}')
        return

    step = args.range / args.steps
    offsets = [i * step for i in range(-args.steps, args.steps + 1)]
    is_rot = args.axis in ('roll', 'pitch', 'yaw')
    unit = 'deg' if is_rot else 'cm'
    print(f'\nSweeping {args.axis} ±{args.range}{unit} in {len(offsets)} steps...')
    for off in offsets:
        dr  = np.radians(off) if args.axis == 'roll'  else 0.0
        dp  = np.radians(off) if args.axis == 'pitch' else 0.0
        dy  = np.radians(off) if args.axis == 'yaw'   else 0.0
        dtx = off / 100.0     if args.axis == 'tx'    else 0.0
        dty = off / 100.0     if args.axis == 'ty'    else 0.0
        dtz = off / 100.0     if args.axis == 'tz'    else 0.0
        grid = generate_grid(erp, pts, cfg, dr, dp, dy, dtx, dty, dtz, views)
        label = f'{args.axis}_{off:+.2f}{unit}'
        cv2.putText(grid, label, (10, grid.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        out = out_dir / f'{label}.jpg'
        cv2.imwrite(str(out), grid)
        print(f'  {out.name}')

    print(f'\n✓ Saved {len(offsets)} images to {out_dir}')
    unit_help = 'degrees' if is_rot else 'cm'
    print(f'  python3 {__file__} --axis {args.axis} --apply <{unit_help}>')


if __name__ == '__main__':
    main()
