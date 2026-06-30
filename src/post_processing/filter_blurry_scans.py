#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Measures Laplacian-variance blur score on each scan's ERP image
# and marks blurry scans with a .blur_skip sentinel file. Downstream tools
# (coloring, ICP, merge, COLMAP) check for this sentinel and skip the scan.
# Run once after stitching/coloring, before any merge or ICP step.

import sys
import os
import argparse
import numpy as np
from pathlib import Path

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def blur_score(erp_path) -> float:
    """Laplacian variance on a downsampled ERP. Higher = sharper."""
    try:
        import cv2
        img = cv2.imread(str(erp_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return 0.0
        small = cv2.resize(img, (960, 480))
        return float(cv2.Laplacian(small, cv2.CV_64F).var())
    except Exception:
        return 0.0


def filter_blurry_scans(session_dir, percentile=20, min_blur=None, dry_run=False):
    """Mark blurry scans with .blur_skip sentinel.

    Args:
        session_dir: path to sync_fusion_* session directory
        percentile:  bottom N% of scans by blur score are marked blurry
        min_blur:    absolute minimum blur score (overrides percentile if set)
        dry_run:     print what would be skipped without writing sentinels
    Returns:
        (kept, skipped) lists of scan dir names
    """
    try:
        session_path = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    ERP_CANDIDATES = [
        'equirect_dual_fisheye_masked.png',
        'equirect_dual_fisheye.jpg',
    ]

    scan_dirs = sorted(session_path.glob('fusion_scan_*'))
    if not scan_dirs:
        print("No fusion_scan_* directories found")
        return [], []

    # Remove any existing sentinels first (re-running is safe)
    for sd in scan_dirs:
        skip = sd / '.blur_skip'
        if skip.exists():
            skip.unlink()

    scores = []
    for sd in scan_dirs:
        erp = next((sd / n for n in ERP_CANDIDATES if (sd / n).exists()), None)
        score = blur_score(erp) if erp else 0.0
        scores.append((sd, score))

    if not scores:
        return [], []

    all_scores = [s for _, s in scores]

    # Determine threshold
    if min_blur is not None:
        threshold = min_blur
    elif len(all_scores) >= 4:
        threshold = float(np.percentile(all_scores, percentile))
    else:
        # Too few scans to use percentile — keep all
        threshold = 0.0

    kept, skipped = [], []
    print(f"\nBlur filter (threshold={threshold:.0f}, percentile={percentile}):")
    for sd, score in sorted(scores, key=lambda x: x[1], reverse=True):
        if score >= threshold:
            kept.append(sd.name)
            print(f"  ✓ {sd.name}: blur_score={score:.0f}  [keep]")
        else:
            skipped.append(sd.name)
            print(f"  ✗ {sd.name}: blur_score={score:.0f}  [skip — blurry]")
            if not dry_run:
                (sd / '.blur_skip').write_text(
                    f"blur_score={score:.1f} threshold={threshold:.1f}\n"
                )

    print(f"\nKept {len(kept)}/{len(scores)} scans  "
          f"({'dry run' if dry_run else 'sentinels written'})")
    return kept, skipped


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Mark blurry scans with .blur_skip sentinel.')
    parser.add_argument('session_dir')
    parser.add_argument('--percentile', type=float, default=20,
                        help='Bottom N%% of scans by blur score are marked blurry (default: 20)')
    parser.add_argument('--min-blur', type=float, default=None,
                        help='Absolute minimum blur score (overrides --percentile)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print results without writing sentinel files')
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    filter_blurry_scans(args.session_dir, args.percentile, args.min_blur, args.dry_run)
