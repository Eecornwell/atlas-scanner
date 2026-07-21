#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Re-applies the LiDAR mask to all equirectangular images in a session directory. Useful after re-running seam blending or updating the mask file.
import sys
import os
from pathlib import Path

# Reuse apply_mask logic
sys.path.insert(0, str(Path(__file__).parent))
from apply_lidar_mask import apply_mask
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
    """Look up per-camera dual mask from multi_camera.yaml.
    Always returns the dual mask — single fisheye center masking is applied
    programmatically in apply_mask_to_image() rather than via a separate file.
    """
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


def _apply_mask_single(img_path: str, mask_path: str, out_path: str) -> bool:
    """Apply dual mask then zero the center half (rear hemisphere) for single fisheye."""
    import cv2
    import numpy as np
    img = cv2.imread(img_path)
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if img is None or mask is None:
        return False
    if mask.shape != img.shape[:2]:
        mask = cv2.resize(mask, (img.shape[1], img.shape[0]))
    _, mask = cv2.threshold(mask, 128, 255, cv2.THRESH_BINARY)
    # Black out center half (columns w/4 to 3w/4) — the rear hemisphere
    w = mask.shape[1]
    mask[:, w // 4 : 3 * w // 4] = 0
    b, g, r = cv2.split(img)
    cv2.imwrite(out_path, cv2.merge([b, g, r, mask]))
    return True


def regenerate_masked_images(session_dir, camera_mode="dual_fisheye", sdk_stitch=False,
                             camera_hw="onex2"):
    """Regenerate masked images from blended ERP images.
    Uses per-camera masks from multi_camera.yaml when available."""
    try:
        session_path = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return

    # Default mask for cameras without a per-camera override
    default_mask = _mask_for_hw(camera_mode, sdk_stitch, camera_hw)
    print(f"  Default mask: {Path(default_mask).name}  (hw={camera_hw})")

    count = 0
    for scan_dir in sorted(session_path.glob("fusion_scan_*")):
        if not scan_dir.is_dir() or (scan_dir / '.blur_skip').exists():
            continue

        # Determine mask for this scan based on camera index.
        # Always attempt per-camera mask lookup from multi_camera.yaml —
        # in single_fisheye mode each camera has its own mask_single entry
        # regardless of whether this is a single- or multi-camera session.
        cam_idx = cam_index_for_scan(scan_dir)
        per_cam_mask = _mask_for_cam(cam_idx, camera_mode)
        mask_file = per_cam_mask if per_cam_mask else default_mask

        for img_file in scan_dir.glob("equirect_*.jpg"):
                if '_bak' not in img_file.name:
                    try:
                        safe_img = _safe_data(img_file)
                        safe_out = _safe_data(img_file.with_name(img_file.stem + "_masked.png"))
                    except ValueError:
                        continue
                    if camera_mode == 'single_fisheye':
                        ok = _apply_mask_single(str(safe_img), mask_file, str(safe_out))
                    else:
                        ok = apply_mask(str(safe_img), mask_file, str(safe_out))
                    if ok:
                        count += 1
                        if per_cam_mask:
                            print(f"    {scan_dir.name}: cam_{cam_idx} mask={Path(mask_file).name}")

    print(f"✓ Regenerated {count} masked images from blended ERPs")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir")
    parser.add_argument("--camera-mode", default="dual_fisheye", choices=["dual_fisheye", "single_fisheye"])
    parser.add_argument("--sdk-stitch", action="store_true")
    parser.add_argument("--camera-hw", default="onex2",
                        choices=["onex2", "x3", "x5"])
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    regenerate_masked_images(args.session_dir, args.camera_mode, args.sdk_stitch,
                             args.camera_hw)
