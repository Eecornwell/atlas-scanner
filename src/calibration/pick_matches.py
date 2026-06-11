#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Perspective crop manual match picker for SDK-stitch camera-lidar calibration.
# Works entirely in perspective space - extracts perspective crops of both camera ERP and
# lidar indices image, lets user pick matches, then looks up 3D point directly from the
# indices crop at the clicked pixel. Saves correspondences in initial_guess_auto format.
#
# Usage:
#   python3 pick_matches.py <output_dir> --yaw <deg> [--pair 000000] [--pitch <deg>] [--fov <deg>] [--crop-size <px>]

import sys
import argparse
import json
import os
import numpy as np
import shutil
import tempfile
import cv2
from pathlib import Path

_ALLOWED_OUTPUT = Path(os.path.expanduser("~/atlas_ws/output")).resolve()
_ALLOWED_DATA   = Path(os.path.expanduser("~/atlas_ws/data")).resolve()


def _safe_output(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_OUTPUT not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed output root '{_ALLOWED_OUTPUT}'")
    return resolved


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed data root '{_ALLOWED_DATA}'")
    return resolved


def _safe_pair(pair: str) -> str:
    """Reject pair values that contain path separators or traversal sequences."""
    if not pair or '/' in pair or '\\' in pair or '..' in pair:
        raise ValueError(f"Invalid pair value: '{pair}'")
    return pair


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_dir')
    parser.add_argument('--yaw',       type=float, default=90.0)
    parser.add_argument('--pitch',     type=float, default=0.0)
    parser.add_argument('--fov',       type=float, default=90.0)
    parser.add_argument('--crop-size', type=int,   default=800)
    parser.add_argument('--pair',      default=None)
    args = parser.parse_args()

    sys.path.insert(0, str(Path(__file__).parent))
    from find_matches_superglue_erp import extract_perspective_crop
    from manual_matches import ManualMatchTool
    import tkinter as tk

    output_dir = Path(args.output_dir).expanduser().resolve()
    try:
        output_dir = _safe_output(output_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    if args.pair:
        try:
            _safe_pair(args.pair)
        except ValueError as e:
            print(f"Error: {e}")
            sys.exit(1)

    pairs = [args.pair] if args.pair else [_safe_pair(p.stem) for p in output_dir.glob('*.ply')
                                           if '/' not in p.stem and '..' not in p.stem]

    # Detect SDK stitch
    is_sdk = False
    try:
        sidecar = _safe_output(output_dir / '000000_source.json')
    except ValueError:
        sidecar = None
    if sidecar and sidecar.exists():
        sd = json.load(open(sidecar))
        try:
            scan_path = _safe_data(sd.get('scan_dir', ''))
            is_sdk = bool(list(scan_path.glob('*.insp'))) if scan_path.is_dir() else False
        except ValueError:
            is_sdk = False

    for pair in pairs:
        try:
            pair = _safe_pair(pair)
        except ValueError as e:
            print(f"Skipping invalid pair '{pair}': {e}")
            continue
        # Load camera ERP - use root PNG (same as initial_guess_auto uses) for coordinates
        cam_path = _safe_output(output_dir / f'{pair}.png')
        cam_erp = cv2.imread(str(cam_path), cv2.IMREAD_GRAYSCALE)
        if cam_erp is None:
            print(f'Missing camera image for {pair}, skipping')
            continue

        # Load full-res RGB for display in GUI (better visibility for picking)
        cam_display_path = _safe_output(output_dir / 'images' / f'{pair}.png')
        cam_display = cv2.imread(str(cam_display_path), cv2.IMREAD_GRAYSCALE) if cam_display_path.exists() else cam_erp
        # Scale display to root PNG resolution so coordinates match
        if cam_display.shape != cam_erp.shape:
            cam_display = cv2.resize(cam_display, (cam_erp.shape[1], cam_erp.shape[0]))

        # Load lidar indices image
        idx_path = _safe_output(output_dir / f'{pair}_lidar_indices.png')
        if not idx_path.exists():
            print(f'Missing lidar indices for {pair}, skipping')
            continue
        idx_raw = cv2.imread(str(idx_path), cv2.IMREAD_UNCHANGED)
        idx_img = np.frombuffer(idx_raw.tobytes(), dtype=np.int32).reshape(idx_raw.shape[:2])

        # Load lidar intensity for display
        lid_erp = cv2.imread(str(_safe_output(output_dir / f'{pair}_lidar_intensities.png')), cv2.IMREAD_GRAYSCALE)

        # Apply SDK stitch transform to lidar images
        if is_sdk:
            lid_erp = np.roll(np.flipud(np.fliplr(lid_erp)), lid_erp.shape[1] // 2, axis=1)
            # Apply same transform to indices image
            idx_img = np.roll(np.flipud(np.fliplr(idx_img)), idx_img.shape[1] // 2, axis=1)

        # Resize lidar to match camera ERP for consistent perspective extraction
        if cam_erp.shape != lid_erp.shape:
            lid_erp = cv2.resize(lid_erp, (cam_erp.shape[1], cam_erp.shape[0]), interpolation=cv2.INTER_NEAREST)
            idx_img = cv2.resize(idx_img, (cam_erp.shape[1], cam_erp.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Extract perspective crops - use cam_erp for coordinate maps, cam_display for GUI
        cam_crop, cam_mx, cam_my = extract_perspective_crop(cam_erp, args.yaw, args.pitch, args.fov, args.crop_size)
        cam_crop_display, _, _ = extract_perspective_crop(cam_display, args.yaw, args.pitch, args.fov, args.crop_size)
        lid_crop, _, _ = extract_perspective_crop(lid_erp, args.yaw, args.pitch, args.fov, args.crop_size)
        # Extract indices crop using INTER_NEAREST to preserve exact index values
        idx_crop = cv2.remap(idx_img.astype(np.float32),
                             *[m.astype(np.float32) for m in
                               extract_perspective_crop(lid_erp, args.yaw, args.pitch, args.fov, args.crop_size)[1:]],
                             cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=-1).astype(np.int32)

        # Save crops for reference
        yaw_tag = f'y{int(args.yaw):03d}'
        cam_crop_path = _safe_output(output_dir / f'{pair}_persp_{yaw_tag}_cam.png')
        lid_crop_path = _safe_output(output_dir / f'{pair}_persp_{yaw_tag}_lid.png')
        cv2.imwrite(str(cam_crop_path), cam_crop_display)
        cv2.imwrite(str(lid_crop_path), cv2.applyColorMap(lid_crop, cv2.COLORMAP_INFERNO))
        preview = np.concatenate([cam_crop, lid_crop], axis=1)
        cv2.imwrite(str(_safe_output(output_dir / f'{pair}_persp_{yaw_tag}_preview.png')), preview)
        print(f'Crops saved: {cam_crop_path.name}  {lid_crop_path.name}')

        # Check how many valid indices are in the crop
        valid_in_crop = (idx_crop >= 0).sum()
        print(f'Valid lidar indices in crop: {valid_in_crop}')
        if valid_in_crop == 0:
            print(f'WARNING: No valid lidar points in this crop view. Try a different --yaw angle.')
            continue

        # Launch GUI
        tmp = Path(tempfile.mkdtemp())
        shutil.copy(str(cam_crop_path), str(tmp / f'{pair}.png'))
        shutil.copy(str(lid_crop_path), str(tmp / f'{pair}_lidar_intensities.png'))

        # Save idx_crop for use in write callback
        idx_crop_path = _safe_output(output_dir / f'{pair}_persp_{yaw_tag}_idx.npy')
        np.save(str(idx_crop_path), idx_crop)

        # Also need cam_mx for camera ERP coordinate lookup
        _, cam_mx, cam_my = extract_perspective_crop(cam_erp, args.yaw, args.pitch, args.fov, args.crop_size)
        cam_map_path = _safe_output(output_dir / f'{pair}_persp_{yaw_tag}_cam_maps.npz')
        np.savez(str(cam_map_path), cam_mx=cam_mx, cam_my=cam_my)

        root = tk.Tk()
        app = ManualMatchTool(root, tmp, pair)

        def _write(icp=idx_crop_path, cmp=cam_map_path, p=pair, yt=yaw_tag):
            idx_c = np.load(str(icp))
            cam_maps = np.load(str(cmp))
            cmx, cmy = cam_maps['cam_mx'], cam_maps['cam_my']

            # Load existing matches
            matches_path = _safe_output(output_dir / f'{p}_matches.json')
            if matches_path.exists():
                try:
                    existing = json.load(open(matches_path))
                    kpts0 = list(existing.get('kpts0', []))
                    kpts1 = list(existing.get('kpts1', []))
                    matches = list(existing.get('matches', []))
                    confidence = list(existing.get('confidence', []))
                except Exception:
                    kpts0, kpts1, matches, confidence = [], [], [], []
            else:
                kpts0, kpts1, matches, confidence = [], [], [], []

            saved, skipped = 0, 0
            for (cx, cy), (lx, ly) in app.completed:
                # Camera: map crop pixel to full-res ERP coordinate
                ci = int(np.clip(cx, 0, cmx.shape[1] - 1))
                cj = int(np.clip(cy, 0, cmx.shape[0] - 1))
                cam_ex = float(cmx[cj, ci])
                cam_ey = float(cmy[cj, ci])

                # Lidar: look up point index directly from indices crop
                li = int(np.clip(lx, 0, idx_c.shape[1] - 1))
                lj = int(np.clip(ly, 0, idx_c.shape[0] - 1))
                point_idx = int(idx_c[lj, li])

                if point_idx < 0:
                    # Search small window for nearest valid index
                    found = False
                    for r in range(1, 8):
                        for dy in range(-r, r+1):
                            for dx in range(-r, r+1):
                                ni, nj = li+dx, lj+dy
                                if 0<=ni<idx_c.shape[1] and 0<=nj<idx_c.shape[0]:
                                    if idx_c[nj, ni] >= 0:
                                        point_idx = int(idx_c[nj, ni])
                                        found = True
                                        break
                            if found: break
                        if found: break

                if point_idx < 0:
                    print(f'  Skipping match at lidar crop ({lx:.0f},{ly:.0f}): no valid point index nearby')
                    skipped += 1
                    continue

                # Store as (cam_erp_x, cam_erp_y) -> point_index
                # We encode point_index into kpts1 as a special marker:
                # kpts1 stores the lidar crop pixel, matches stores point_index directly
                # But initial_guess_auto expects kpts1 as ERP pixel coords...
                # So we need to find the ERP pixel that corresponds to this point_index.
                # The indices image maps ERP pixel -> point_index, so we need the reverse.
                # Store cam ERP coords in kpts0, and the point_index encoded as kpts1 pixel
                # by finding where this point_index appears in the original indices image.
                idx_orig_raw = cv2.imread(str(_safe_output(output_dir / f'{p}_lidar_indices.png')), cv2.IMREAD_UNCHANGED)
                idx_orig = np.frombuffer(idx_orig_raw.tobytes(), dtype=np.int32).reshape(idx_orig_raw.shape[:2])
                ys, xs = np.where(idx_orig == point_idx)
                if len(ys) == 0:
                    print(f'  Skipping: point_index {point_idx} not found in original indices image')
                    skipped += 1
                    continue
                # Use the first occurrence
                lid_ex, lid_ey = int(xs[0]), int(ys[0])

                # Scale cam ERP coords are already in root PNG space (loaded from root PNG)
                cam_ex_final = int(round(cam_ex))
                cam_ey_final = int(round(cam_ey))

                m = len(kpts1) // 2
                kpts0 += [cam_ex_final, cam_ey_final]
                kpts1 += [lid_ex, lid_ey]
                matches.append(m)
                confidence.append(1.0)
                saved += 1

            try:
                safe_matches_path = _safe_output(matches_path)
            except ValueError as e:
                print(f"Error: refusing to write matches to unsafe path: {e}")
                return
            json.dump({'kpts0': kpts0, 'kpts1': kpts1,
                       'matches': matches, 'confidence': confidence},
                      open(safe_matches_path, 'w'))
            print(f'✓ Saved {saved} matches to {safe_matches_path.name} ({skipped} skipped, {len(matches)} total)')

        app._write_matches = _write
        root.mainloop()

        if len(pairs) > 1 and pair != pairs[-1]:
            if input('Continue to next pair? [Y/n]: ').strip().lower() == 'n':
                break


if __name__ == '__main__':
    main()
