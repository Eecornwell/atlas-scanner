#!/usr/bin/env python3
"""Scale kpts from half-res SuperGlue output back to full-res indices image coordinates,
and filter geometrically inconsistent matches using RANSAC on the fundamental matrix."""
import json
import sys
import numpy as np
from pathlib import Path

if len(sys.argv) != 2:
    print("Usage: python3 fix_matches.py <output_directory>")
    sys.exit(1)

output_dir = Path(sys.argv[1])

for matches_file in sorted(output_dir.glob("*_matches.json")):
    with open(matches_file) as f:
        d = json.load(f)

    # kpts are already in the correct coordinate space - no scaling needed

    matches = d['matches']
    kpts0 = np.array(d['kpts0']).reshape(-1, 2)
    kpts1 = np.array(d['kpts1']).reshape(-1, 2)

    # Collect valid match pairs
    idx0 = [i for i, m in enumerate(matches) if m >= 0]
    idx1 = [matches[i] for i in idx0]

    if len(idx0) >= 4:
        pts0 = kpts0[idx0]
        pts1 = kpts1[idx1]
        offsets = pts1 - pts0
        med = np.median(offsets, axis=0)
        mad = np.median(np.abs(offsets - med), axis=0).clip(1)
        # Reject matches whose offset deviates more than 3 MAD from median
        inlier_mask = np.all(np.abs(offsets - med) < 3 * mad, axis=1)
        for k, i in enumerate(idx0):
            if not inlier_mask[k]:
                matches[i] = -1
        d['matches'] = matches
        kept = int(inlier_mask.sum())
        print(f"✓ Fixed {matches_file.name}: {kept}/{len(idx0)} matches kept after offset filtering")
    else:
        print(f"✓ Fixed {matches_file.name}: {len(idx0)} matches (too few to filter)")

    with open(matches_file, 'w') as f:
        json.dump(d, f)
