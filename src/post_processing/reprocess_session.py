#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Re-runs coloring (exact_match_fusion) and COLMAP export on every scan in a
# session using the current fusion_calibration.yaml. Use this to recover sessions that were
# processed with a bad extrinsic calibration.

import sys
import os
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from exact_match_fusion import exact_match_calibration_tool
from export_to_colmap import export_to_colmap

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def reprocess_session(session_dir, skip_coloring=False, skip_colmap=False):
    try:
        session_path = _safe_data(Path(session_dir).expanduser())
    except ValueError as e:
        print(f"Error: {e}")
        return False
    scan_dirs = sorted(session_path.glob('fusion_scan_*'))

    if not scan_dirs:
        print(f"No fusion_scan_* directories found in {session_path}")
        return False

    print(f"Found {len(scan_dirs)} scans in {session_path.name}")

    if not skip_coloring:
        print("\n--- Re-coloring point clouds ---")
        for scan_dir in scan_dirs:
            sensor_ply = next(scan_dir.glob('sensor_lidar*.ply'), None)
            if sensor_ply is None:
                print(f"  SKIP {scan_dir.name} — no sensor_lidar*.ply")
                continue
            try:
                _safe_data(scan_dir)
            except ValueError:
                print(f"  SKIP {scan_dir.name} — path outside allowed root")
                continue
            print(f"  {scan_dir.name} ...", end=' ', flush=True)
            ok = exact_match_calibration_tool(str(scan_dir))
            print("\u2713" if ok else "\u2717")

    if not skip_colmap:
        print("\n--- Re-exporting to COLMAP ---")
        export_to_colmap(str(session_path))

    print("\n✓ Reprocessing complete.")
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Reprocess a scan session with the current calibration.')
    parser.add_argument('session_dir', help='Path to the sync_fusion_* session directory')
    parser.add_argument('--skip-coloring', action='store_true', help='Skip re-coloring step')
    parser.add_argument('--skip-colmap',   action='store_true', help='Skip COLMAP export step')
    args = parser.parse_args()
    try:
        _safe_data(Path(args.session_dir).expanduser())
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    ok = reprocess_session(args.session_dir, args.skip_coloring, args.skip_colmap)
    sys.exit(0 if ok else 1)
