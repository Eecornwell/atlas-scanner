#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Verifies that saved match coordinates land on the correct pixels
# in both the camera ERP and lidar intensity images. Run after pick_matches.py
# to confirm coordinates are correct before spending time on full matching.
#
# Usage: python3 verify_matches.py ~/atlas_ws/output --pair 000000

import sys
import argparse
import json
import os
import numpy as np
import cv2
from pathlib import Path

_ALLOWED_OUTPUT = Path(os.path.expanduser('~/atlas_ws/output')).resolve()


def _safe_output(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_OUTPUT not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed output root '{_ALLOWED_OUTPUT}'")
    return resolved


def _safe_pair(pair: str) -> str:
    if not pair or '/' in pair or '\\' in pair or '..' in pair:
        raise ValueError(f"Invalid pair value: '{pair}'")
    return pair


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_dir')
    parser.add_argument('--pair', default='000000')
    args = parser.parse_args()

    try:
        output_dir = _safe_output(Path(args.output_dir).expanduser().resolve())
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)

    try:
        pair = _safe_pair(args.pair)
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)

    d = json.load(open(_safe_output(output_dir / f'{pair}_matches.json')))
    kpts0 = np.array(d['kpts0']).reshape(-1, 2)
    kpts1 = np.array(d['kpts1']).reshape(-1, 2)
    matches = d['matches']

    cam = cv2.imread(str(_safe_output(output_dir / f'{pair}.png')))  # root PNG - same as initial_guess_auto
    lid = cv2.imread(str(_safe_output(output_dir / f'{pair}_lidar_intensities.png')), cv2.IMREAD_GRAYSCALE)

    print(f'Camera ERP size: {cam.shape[1]}x{cam.shape[0]}')
    print(f'Lidar intensity size: {lid.shape[1]}x{lid.shape[0]}')
    print(f'Matches: {sum(1 for m in matches if m >= 0)} valid\n')

    # Scale camera down to match lidar size for side-by-side display
    cam_small = cv2.resize(cam, (lid.shape[1], lid.shape[0]))
    scale_x = lid.shape[1] / cam.shape[1]
    scale_y = lid.shape[0] / cam.shape[0]

    lid_color = cv2.applyColorMap(lid, cv2.COLORMAP_INFERNO)

    valid_count = 0
    for i, m in enumerate(matches):
        if m < 0 or m >= len(kpts1):
            continue
        cx, cy = int(kpts0[i][0]), int(kpts0[i][1])
        lx, ly = int(kpts1[m][0]), int(kpts1[m][1])

        in_cam = 0 <= cy < cam.shape[0] and 0 <= cx < cam.shape[1]
        in_lid = 0 <= ly < lid.shape[0] and 0 <= lx < lid.shape[1]
        lid_val = lid[ly, lx] if in_lid else -1
        has_lidar = lid_val > 0

        status = 'OK' if (in_cam and in_lid and has_lidar) else 'BLANK' if (in_cam and in_lid) else 'OOB'
        color = (0, 255, 0) if status == 'OK' else (0, 165, 255) if status == 'BLANK' else (0, 0, 255)

        print(f'  match {i:2d}: cam=({cx:4d},{cy:4d}) lid=({lx:4d},{ly:4d}) lid_val={lid_val:3d} [{status}]')

        # Draw on camera (scaled)
        cv2.circle(cam_small, (int(cx * scale_x), int(cy * scale_y)), 10, color, 2)
        cv2.putText(cam_small, str(i), (int(cx * scale_x) + 8, int(cy * scale_y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Draw on lidar
        if in_lid:
            cv2.circle(lid_color, (lx, ly), 10, color, 2)
            cv2.putText(lid_color, str(i), (lx + 8, ly - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if status == 'OK':
            valid_count += 1

    print(f'\n{valid_count}/{sum(1 for m in matches if m >= 0)} matches land on valid lidar points')
    print('Green=OK  Orange=blank lidar pixel  Red=out of bounds')

    out = np.hstack([cam_small, lid_color])
    out_path = _safe_output(output_dir / f'{pair}_verify.png')
    cv2.imwrite(str(out_path), out)
    print(f'\nSaved: {out_path}')

    import subprocess
    subprocess.Popen(['eog', str(out_path)])


if __name__ == '__main__':
    main()
