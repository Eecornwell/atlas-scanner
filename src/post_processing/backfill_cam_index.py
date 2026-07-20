#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Backfills the camera serial into old-format .cam_index files
# (which only stored the SDK runtime index "0") so calibration_path() can
# resolve the correct slot calibration by serial lookup.
#
# Usage:
#   python3 backfill_cam_index.py <session_dir>
#   python3 backfill_cam_index.py <session_dir> --serial IXSE46EN77TP9E
#
# If --serial is not given, the serial is read from multi_camera.yaml using
# the session's camera_hw from session_config.json.

import sys
import json
import yaml
import argparse
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1]
_ALLOWED_DATA = Path.home() / 'atlas_ws/data'


def backfill(session_dir: str, serial_override: str = None):
    session = Path(session_dir).resolve()
    if not session.is_dir():
        print(f"Session directory not found: {session}"); sys.exit(1)

    cfg_path = session / 'session_config.json'
    if not cfg_path.exists():
        print(f"No session_config.json in {session}"); sys.exit(1)
    cfg = json.loads(cfg_path.read_text())
    hw = cfg.get('camera_hw', '')

    mc = yaml.safe_load((_SRC / 'config' / 'multi_camera.yaml').read_text())

    # Determine serial
    serial = serial_override
    if not serial:
        # Check .sdk_camera_map first (most accurate)
        sdk_map = session / '.sdk_camera_map'
        if sdk_map.exists():
            for line in sdk_map.read_text().splitlines():
                parts = line.strip().split()
                if len(parts) == 2:
                    serial = parts[1]
                    print(f"  Serial from .sdk_camera_map: {serial}")
                    break
    if not serial:
        # Fall back to multi_camera.yaml lookup by hw
        for cam in mc.get('cameras', {}).values():
            if cam.get('camera_hw', '') == hw:
                serial = cam.get('serial', '')
                break
    if not serial:
        print(f"Could not determine serial for camera_hw={hw}. "
              f"Pass --serial <SERIAL> explicitly."); sys.exit(1)

    # Resolve slot from serial
    slot_idx = None
    for slot_key, cam in mc.get('cameras', {}).items():
        if cam.get('serial', '') == serial:
            slot_idx = int(slot_key.split('_')[1])
            break
    if slot_idx is None:
        print(f"Serial {serial} not found in multi_camera.yaml. "
              f"Add it to the cameras section first."); sys.exit(1)

    print(f"Session:    {session.name}")
    print(f"camera_hw:  {hw}")
    print(f"Serial:     {serial}  ->  slot {slot_idx}")

    updated = skipped = 0
    for scan_dir in sorted(session.glob('fusion_scan_*')):
        if not scan_dir.is_dir():
            continue
        ci_path = scan_dir / '.cam_index'
        if ci_path.exists():
            parts = ci_path.read_text().strip().split()
            if len(parts) >= 2:
                skipped += 1
                continue  # already has serial
            sdk_idx = parts[0] if parts else '0'
        else:
            sdk_idx = '0'
        ci_path.write_text(f"{sdk_idx} {serial}")
        updated += 1

    print(f"Updated:    {updated} scan dirs")
    if skipped:
        print(f"Skipped:    {skipped} (already had serial)")

    # Verify
    sys.path.insert(0, str(_SRC))
    from camera_hw import cam_index_for_scan, calibration_path
    first = sorted(session.glob('fusion_scan_*'))[0]
    resolved_slot = cam_index_for_scan(str(first))
    calib = calibration_path(hw, resolved_slot)
    src = _SRC / 'config'
    print(f"Calibration: {calib.relative_to(src)}")
    print()
    print("Now reprocess with:")
    print(f"  python3 src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \\")
    print(f"      {session} --lidar-window 0.3 --camera-mode dual_fisheye")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--serial', default=None,
                        help='Camera serial number (auto-detected if not given)')
    args = parser.parse_args()
    backfill(args.session_dir, args.serial)
