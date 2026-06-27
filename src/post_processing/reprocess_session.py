#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Re-runs coloring (exact_match_fusion) and COLMAP export on every scan in a
# session using the current fusion_calibration.yaml. Use this to recover sessions that were
# processed with a bad extrinsic calibration.

import sys
import os
import subprocess
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from exact_match_fusion import exact_match_calibration_tool
from export_to_colmap import export_to_colmap

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()
_STITCH_BIN = Path(os.path.expanduser('~/insta360-dev/build/insta360_stitch'))


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def _stitch_scan(scan_dir: Path, camera_mode: str, erp_width: int, erp_height: int) -> bool:
    """Stitch .insp → equirect_dual_fisheye.jpg if not already present."""
    erp = scan_dir / 'equirect_dual_fisheye.jpg'
    if erp.exists():
        return True
    insp = next(scan_dir.glob('*.insp'), None)
    if insp is None:
        return False
    if not _STITCH_BIN.exists():
        print(f"  ⚠ insta360_stitch not found at {_STITCH_BIN}")
        return False
    cmd = [str(_STITCH_BIN), str(insp), str(erp),
           '--width', str(erp_width), '--height', str(erp_height)]
    if camera_mode == 'single_fisheye':
        cmd.append('--single')
    env = os.environ.copy()
    env['INSTA360_ERP_WIDTH']  = str(erp_width)
    env['INSTA360_ERP_HEIGHT'] = str(erp_height)
    result = subprocess.run(cmd, env=env, capture_output=True, text=True)
    return result.returncode == 0 and erp.exists()


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

    # Read session config for camera mode and ERP resolution
    import json
    cfg_path = session_path / 'session_config.json'
    camera_mode = 'dual_fisheye'
    camera_hw   = 'onex2'
    erp_width   = 5760
    erp_height  = 2880
    if cfg_path.exists():
        try:
            cfg = json.loads(cfg_path.read_text())
            camera_mode = cfg.get('camera_mode', camera_mode)
            camera_hw   = cfg.get('camera_hw',   camera_hw)
            erp_width   = int(cfg.get('erp_width',  erp_width))
            erp_height  = int(cfg.get('erp_height', erp_height))
        except Exception:
            pass

    if not skip_coloring:
        # Promote .insp files from .sdk_shot_N/ into fusion_scan_NNN/ by shot order
        # if they haven't been moved yet (continuous mode leaves them in hidden dirs)
        shot_dirs = sorted(session_path.glob('.sdk_shot_*'),
                           key=lambda d: int(d.name.split('_')[-1]))
        if shot_dirs:
            scan_list = sorted(session_path.glob('fusion_scan_*'))
            for idx, shot_dir in enumerate(shot_dirs):
                if idx >= len(scan_list):
                    break
                target = scan_list[idx]
                for insp in shot_dir.glob('*.insp'):
                    if insp.stat().st_size < 100_000:
                        continue
                    dest = target / insp.name
                    if not dest.exists():
                        insp.rename(dest)
                    ct = insp.with_suffix('.insp.capture_time')
                    if ct.exists() and not (target / ct.name).exists():
                        ct.rename(target / ct.name)

        # Stitch any .insp files that don't yet have an ERP image
        needs_stitch = [d for d in scan_dirs
                        if not (d / 'equirect_dual_fisheye.jpg').exists()
                        and next(d.glob('*.insp'), None) is not None]
        if needs_stitch:
            print(f"\n--- Stitching {len(needs_stitch)} .insp files ---")
            for scan_dir in needs_stitch:
                print(f"  {scan_dir.name} ...", end=' ', flush=True)
                ok = _stitch_scan(scan_dir, camera_mode, erp_width, erp_height)
                print('✓' if ok else '✗ stitch failed')
            # Regenerate masked images from freshly stitched ERPs
            import importlib.util as _ilu
            _rmi_path = Path(__file__).parent / 'regenerate_masked_images.py'
            _spec = _ilu.spec_from_file_location('regenerate_masked_images', _rmi_path)
            _rmi = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_rmi)
            _rmi.regenerate_masked_images(str(session_path), camera_mode, sdk_stitch=True,
                                          camera_hw=camera_hw)

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
            print('✓' if ok else '✗')

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
