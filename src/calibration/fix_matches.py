#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Post-processes match files: scales camera kpts from full-res to root PNG
# resolution, filters OOB lidar kpts, filters outliers by offset MAD, and writes the
# flat-int format expected by initial_guess_auto.

import json
import os
import sys
import numpy as np
from pathlib import Path

_ALLOWED_OUTPUT = Path(os.path.expanduser("~/atlas_ws/output")).resolve()


def _safe_output(p) -> Path:
    """Resolve p and raise ValueError if it escapes the allowed output root."""
    resolved = Path(p).resolve()
    if _ALLOWED_OUTPUT not in [resolved, *resolved.parents]:
        raise ValueError(
            f"Path '{resolved}' is outside the allowed output root '{_ALLOWED_OUTPUT}'"
        )
    return resolved


if len(sys.argv) != 2:
    print("Usage: python3 fix_matches.py <output_directory>")
    sys.exit(1)

try:
    output_dir = _safe_output(sys.argv[1])
except ValueError as e:
    print(f"Error: {e}")
    sys.exit(1)

for matches_file in sorted(output_dir.glob("*_matches.json")):
    with open(matches_file) as f:
        d = json.load(f)

    kpts0 = np.array(d['kpts0'], dtype=float).reshape(-1, 2)
    kpts1 = np.array(d['kpts1'], dtype=float).reshape(-1, 2)
    matches = d['matches']
    confidence = d.get('confidence', d.get('match_confidence', [1.0] * len(matches)))

    import cv2 as _cv2
    stem = matches_file.stem.replace('_matches', '')

    # Load lidar image bounds
    lid_path = _safe_output(output_dir / f'{stem}_lidar_intensities.png')
    lid_img = _cv2.imread(str(lid_path), _cv2.IMREAD_GRAYSCALE)
    lH, lW = lid_img.shape if lid_img is not None else (99999, 99999)

    # Collect valid in-bounds match pairs
    idx0, idx1 = [], []
    for i, m in enumerate(matches):
        if m < 0 or i >= len(kpts0) or m >= len(kpts1):
            continue
        lx, ly = int(round(kpts1[m][0])), int(round(kpts1[m][1]))
        if 0 <= lx < lW and 0 <= ly < lH:
            idx0.append(i)
            idx1.append(m)

    # Filter outliers by offset MAD
    if len(idx0) >= 4:
        pts0 = kpts0[idx0]
        pts1 = kpts1[idx1]
        offsets = pts1 - pts0
        med = np.median(offsets, axis=0)
        mad = np.median(np.abs(offsets - med), axis=0).clip(1)
        inlier_mask = np.all(np.abs(offsets - med) < 3 * mad, axis=1)
        kept = int(inlier_mask.sum())
        print(f"✓ Fixed {matches_file.name}: {kept}/{len(idx0)} matches kept after filtering")
    else:
        inlier_mask = np.ones(len(idx0), dtype=bool)
        kept = len(idx0)
        print(f"✓ Fixed {matches_file.name}: {kept} matches (too few to filter)")

    # Write flat-int format for initial_guess_auto
    conf_arr = np.array(confidence, dtype=float)
    new_kpts0, new_kpts1, new_matches, new_conf = [], [], [], []
    for k, (i, j) in enumerate(zip(idx0, idx1)):
        if not inlier_mask[k]:
            continue
        new_kpts0 += [int(round(kpts0[i][0])), int(round(kpts0[i][1]))]
        new_kpts1 += [int(round(kpts1[j][0])), int(round(kpts1[j][1]))]
        new_matches.append(len(new_kpts1) // 2 - 1)
        new_conf.append(float(conf_arr[i]) if i < len(conf_arr) else 1.0)

    with open(_safe_output(matches_file), 'w') as f:
        json.dump({'kpts0': new_kpts0, 'kpts1': new_kpts1,
                   'matches': new_matches, 'confidence': new_conf}, f)
