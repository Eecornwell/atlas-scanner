#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Re-applies the LiDAR mask to all equirectangular images in a
# session directory. Mask images are loaded once and cached; all scans are
# processed in parallel via ThreadPoolExecutor (cv2 releases the GIL).

import sys
import os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))
from camera_hw import load_camera_profile, camera_hw_for_session, mask_path as _mask_path, cam_index_for_scan

MASK_BASE = Path(os.path.expanduser('~/atlas_ws/src/atlas-scanner/src'))
_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()
_ALLOWED_SRC  = Path(os.path.expanduser('~/atlas_ws/src')).resolve()
_MASK_DIR = MASK_BASE / 'config' / 'masks'
_MULTI_CAM_YAML = MASK_BASE / 'config' / 'multi_camera.yaml'


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def _mask_for_hw(camera_mode, sdk_stitch, camera_hw):
    profile = load_camera_profile(camera_hw)
    return str(_mask_path(profile, camera_mode, sdk_stitch))


def _mask_for_cam(cam_idx: int, camera_mode: str) -> str | None:
    """Look up per-camera dual mask from multi_camera.yaml."""
    if not _MULTI_CAM_YAML.exists():
        return None
    try:
        import yaml
        cfg = yaml.safe_load(_MULTI_CAM_YAML.read_text()) or {}
        cam_cfg = cfg.get('cameras', {}).get(f'cam_{cam_idx}', {})
        mask_name = cam_cfg.get('mask_dual')
        if not mask_name:
            return None
        mask_file = _MASK_DIR / mask_name
        if mask_file.exists():
            return str(mask_file)
        legacy = MASK_BASE / mask_name
        if legacy.exists():
            return str(legacy)
    except Exception:
        pass
    return None


def regenerate_masked_images(session_dir, camera_mode="dual_fisheye", sdk_stitch=False,
                             camera_hw="onex2"):
    """Regenerate masked images from blended ERP images in parallel.
    Mask images are loaded once and cached; all scans processed concurrently."""
    import cv2
    import numpy as np

    try:
        session_path = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return

    default_mask = _mask_for_hw(camera_mode, sdk_stitch, camera_hw)
    print(f"  Default mask: {Path(default_mask).name}  (hw={camera_hw})")

    # Load and cache each unique mask file once — avoids re-reading the same
    # PNG from disk for every scan (3 cameras x N scans = same 3 mask files).
    _mask_cache: dict[str, np.ndarray] = {}

    def _get_mask(mask_file: str, h: int, w: int) -> np.ndarray:
        key = f"{mask_file}:{w}x{h}"
        if key not in _mask_cache:
            m = cv2.imread(mask_file, cv2.IMREAD_GRAYSCALE)
            if m is None:
                m = np.full((h, w), 255, dtype=np.uint8)
            elif m.shape != (h, w):
                m = cv2.resize(m, (w, h))
            _, m = cv2.threshold(m, 128, 255, cv2.THRESH_BINARY)
            _mask_cache[key] = m
        return _mask_cache[key]

    def _process_scan(scan_dir: Path) -> int:
        if not scan_dir.is_dir() or (scan_dir / '.blur_skip').exists():
            return 0
        cam_idx = cam_index_for_scan(scan_dir)
        per_cam_mask = _mask_for_cam(cam_idx, camera_mode)
        mask_file = per_cam_mask if per_cam_mask else default_mask
        n = 0
        for img_file in scan_dir.glob("equirect_*.jpg"):
            if '_bak' in img_file.name:
                continue
            try:
                safe_img = _safe_data(img_file)
                safe_out = _safe_data(img_file.with_name(img_file.stem + "_masked.png"))
            except ValueError:
                continue
            img = cv2.imread(str(safe_img))
            if img is None:
                continue
            h, w = img.shape[:2]
            mask = _get_mask(mask_file, h, w).copy()
            if camera_mode == 'single_fisheye':
                mask[:, w // 4: 3 * w // 4] = 0
            b, g, r = cv2.split(img)
            cv2.imwrite(str(safe_out), cv2.merge([b, g, r, mask]))
            n += 1
            if per_cam_mask:
                print(f"    {scan_dir.name}: cam_{cam_idx} mask={Path(mask_file).name}")
        return n

    scan_dirs = sorted(session_path.glob("fusion_scan_*"))
    # Cap workers by available RAM — each worker holds one decoded ERP
    # (read: H*W*3 bytes) plus the RGBA output (H*W*4 bytes).
    # Use at most 40% of available RAM for this step.
    import psutil as _psutil
    _probe_path = next(session_path.glob('fusion_scan_*/equirect_*.jpg'), None)
    # Estimate decoded size: JPEG compresses ~30:1 from raw RGB; read+write = *2
    _erp_bytes = (os.path.getsize(_probe_path) * 30 * 2) if _probe_path else (11520 * 5760 * 7)
    _avail = _psutil.virtual_memory().available
    _mem_workers = max(1, int(_avail * 0.4 / _erp_bytes))
    n_workers = min(len(scan_dirs) or 1, 8, _mem_workers)
    with ThreadPoolExecutor(max_workers=n_workers) as pool:
        count = sum(pool.map(_process_scan, scan_dirs))
    print(f"\u2713 Regenerated {count} masked images from blended ERPs")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir")
    parser.add_argument("--camera-mode", default="dual_fisheye", choices=["dual_fisheye", "single_fisheye"])
    parser.add_argument("--sdk-stitch", action="store_true")
    parser.add_argument("--camera-hw", default="onex2", choices=["onex2", "x3", "x5"])
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    regenerate_masked_images(args.session_dir, args.camera_mode, args.sdk_stitch,
                             args.camera_hw)
