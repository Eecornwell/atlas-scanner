#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Normalizes ERP images from secondary cameras to match the color
# profile of the primary (reference) camera. Uses per-camera color profiles
# stored in config/calibrations/<hw>/cam_N/color_profile.npz. If no profile
# exists, computes a session-level match from overlapping scene statistics.

import sys
import json
import numpy as np
import cv2
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1]  # atlas-scanner/src/
_CALIB_DIR = _SRC / 'config' / 'calibrations'
_ALLOWED_DATA = Path.home() / 'atlas_ws/data'


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA.resolve() not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def load_color_profile(camera_hw: str, cam_index: int) -> dict | None:
    """Load a pre-calibrated color profile for a specific camera.
    
    Profile contains:
      - gain_lab: [L_gain, A_gain, B_gain] multiplicative gains in LAB space
      - offset_lab: [L_off, A_off, B_off] additive offsets in LAB space
      - gamma: per-channel gamma correction [R, G, B]
      - ccm: 3x3 color correction matrix (sensor RGB -> reference RGB)
    """
    profile_path = _CALIB_DIR / camera_hw / f'cam_{cam_index}' / 'color_profile.npz'
    if not profile_path.exists():
        return None
    data = np.load(str(profile_path), allow_pickle=True)
    return {k: data[k] for k in data.files}


def save_color_profile(camera_hw: str, cam_index: int, profile: dict):
    """Save a color profile for a specific camera."""
    out_dir = _CALIB_DIR / camera_hw / f'cam_{cam_index}'
    out_dir.mkdir(parents=True, exist_ok=True)
    np.savez(str(out_dir / 'color_profile.npz'), **profile)
    print(f"  Saved color profile: {out_dir / 'color_profile.npz'}")


def compute_lab_stats(img_bgr: np.ndarray, mask: np.ndarray = None) -> dict:
    """Compute mean and std of LAB channels over non-black pixels."""
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    if mask is None:
        # Exclude near-black pixels (masked/invalid regions)
        mask = img_bgr.max(axis=2) > 15
    if not mask.any():
        return {'mean': np.zeros(3), 'std': np.ones(3)}
    pixels = lab[mask]
    return {'mean': pixels.mean(axis=0), 'std': pixels.std(axis=0)}


def apply_color_profile(img_bgr: np.ndarray, profile: dict) -> np.ndarray:
    """Apply a color profile to transform an image to the reference color space.
    
    Pipeline: CCM (linear RGB) -> gamma -> LAB gain/offset
    """
    img = img_bgr.astype(np.float32)

    # 1. Color correction matrix (3x3, applied in linear RGB)
    if 'ccm' in profile:
        ccm = profile['ccm']
        h, w = img.shape[:2]
        flat = img.reshape(-1, 3)  # BGR order
        # CCM expects RGB
        flat_rgb = flat[:, ::-1]
        corrected_rgb = (ccm @ flat_rgb.T).T
        img = np.clip(corrected_rgb[:, ::-1].reshape(h, w, 3), 0, 255)

    # 2. Per-channel gamma
    if 'gamma' in profile:
        gamma = profile['gamma']  # [R, G, B]
        for c, g in enumerate([gamma[2], gamma[1], gamma[0]]):  # BGR order
            img[:, :, c] = 255.0 * np.power(img[:, :, c] / 255.0, g)

    # 3. LAB gain and offset
    if 'gain_lab' in profile or 'offset_lab' in profile:
        lab = cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_BGR2LAB).astype(np.float32)
        if 'gain_lab' in profile:
            lab *= profile['gain_lab']
        if 'offset_lab' in profile:
            lab += profile['offset_lab']
        lab[:, :, 0] = np.clip(lab[:, :, 0], 0, 255)
        lab[:, :, 1:] = np.clip(lab[:, :, 1:], 0, 255)
        img = cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_LAB2BGR).astype(np.float32)

    return np.clip(img, 0, 255).astype(np.uint8)


def compute_profile_from_images(ref_img: np.ndarray, src_img: np.ndarray) -> dict:
    """Compute a color profile that transforms src to match ref.
    
    Uses a conservative diagonal CCM (white balance correction only)
    clamped to reasonable bounds to prevent overcorrection when cameras
    see slightly different scene content.
    """
    # Compute channel means over valid (non-black) pixels
    ref_mask = ref_img.max(axis=2) > 20
    src_mask = src_img.max(axis=2) > 20
    if not ref_mask.any() or not src_mask.any():
        return {'ccm': np.eye(3, dtype=np.float32),
                'gain_lab': np.array([1.0, 1.0, 1.0], dtype=np.float32),
                'offset_lab': np.array([0.0, 0.0, 0.0], dtype=np.float32),
                'gamma': np.array([1.0, 1.0, 1.0], dtype=np.float32)}

    # RGB channel means (BGR -> RGB)
    ref_rgb = ref_img[ref_mask].astype(np.float32).mean(axis=0)[::-1]
    src_rgb = src_img[src_mask].astype(np.float32).mean(axis=0)[::-1]

    # Diagonal CCM: simple white balance correction
    # Clamp gains to [0.8, 1.2] to prevent extreme overcorrection
    diag = np.clip(ref_rgb / np.maximum(src_rgb, 1e-6), 0.8, 1.2)
    ccm = np.diag(diag).astype(np.float32)

    # No LAB offset — it's too aggressive for non-colocated cameras
    return {
        'ccm': ccm,
        'gain_lab': np.array([1.0, 1.0, 1.0], dtype=np.float32),
        'offset_lab': np.array([0.0, 0.0, 0.0], dtype=np.float32),
        'gamma': np.array([1.0, 1.0, 1.0], dtype=np.float32),
    }


def calibrate_from_session(session_dir: str, reference_cam: int = 0):
    """Build color profiles for all secondary cameras using a session's ERP images.
    
    Finds pairs of ERP images from different cameras that were captured close in
    time (same scene), computes the transform from each secondary camera to the
    reference camera, and saves the profile.
    """
    session_path = _safe_data(session_dir)
    
    # Read session config
    cfg_path = session_path / 'session_config.json'
    if not cfg_path.exists():
        print("No session_config.json found")
        return
    cfg = json.loads(cfg_path.read_text())
    camera_hw = cfg.get('camera_hw', 'x5')

    # Group scans by camera index
    cam_scans = {}  # cam_idx -> [(scan_dir, erp_path)]
    stitch_bin = Path(__file__).resolve().parents[1] / 'capture' / 'sdk' / 'build' / 'insta360_stitch'
    import subprocess

    for scan_dir in sorted(session_path.glob('fusion_scan_*')):
        if not scan_dir.is_dir():
            continue
        ci_file = scan_dir / '.cam_index'
        cam_idx = int(ci_file.read_text().strip().split()[0]) if ci_file.exists() else 0
        # Always stitch from .insp for clean calibration source
        insp_file = next(scan_dir.glob('*.insp'), None)
        if insp_file and stitch_bin.exists():
            clean_erp = scan_dir / '_erp_calib_tmp.jpg'
            result = subprocess.run(
                [str(stitch_bin), str(insp_file), str(clean_erp)],
                capture_output=True,
            )
            if result.returncode == 0 and clean_erp.exists():
                cam_scans.setdefault(cam_idx, []).append((scan_dir, clean_erp))
                continue
        # Fallback to existing ERP
        erp = scan_dir / 'equirect_dual_fisheye.jpg'
        if erp.exists():
            cam_scans.setdefault(cam_idx, []).append((scan_dir, erp))

    if reference_cam not in cam_scans:
        print(f"Reference camera {reference_cam} has no ERP images")
        return

    ref_images = cam_scans[reference_cam]
    print(f"Reference camera: cam_{reference_cam} ({len(ref_images)} images)")
    print(f"Camera HW: {camera_hw}")

    # Remove any existing profile for the reference camera — it must never be modified.
    # Also remove profiles for ALL cameras so a fresh calibration starts clean.
    for ci in range(3):
        old_profile = _CALIB_DIR / camera_hw / f'cam_{ci}' / 'color_profile.npz'
        if old_profile.exists():
            old_profile.unlink()
            print(f"  Removed old profile: cam_{ci}")

    # For each secondary camera, find temporally adjacent reference images
    # and compute the average color transform
    for cam_idx, scans in cam_scans.items():
        if cam_idx == reference_cam:
            continue

        print(f"\nComputing profile for cam_{cam_idx} ({len(scans)} images)...")

        # Use up to 5 image pairs for robust statistics
        profiles = []
        for src_dir, src_erp in scans[:10]:
            # Find closest reference image by scan index
            src_num = int(src_dir.name.split('_')[-1])
            best_ref = min(ref_images, key=lambda r: abs(int(r[0].name.split('_')[-1]) - src_num))
            ref_erp = best_ref[1]

            ref_img = cv2.imread(str(ref_erp))
            src_img = cv2.imread(str(src_erp))
            if ref_img is None or src_img is None:
                continue

            # Resize to same dimensions for comparison
            if ref_img.shape != src_img.shape:
                src_img = cv2.resize(src_img, (ref_img.shape[1], ref_img.shape[0]))

            p = compute_profile_from_images(ref_img, src_img)
            profiles.append(p)

            if len(profiles) >= 5:
                break

        if not profiles:
            print(f"  No valid image pairs found for cam_{cam_idx}")
            continue

        # Average the profiles
        avg_profile = {
            'gain_lab': np.median([p['gain_lab'] for p in profiles], axis=0),
            'offset_lab': np.median([p['offset_lab'] for p in profiles], axis=0),
            'ccm': np.median([p['ccm'] for p in profiles], axis=0),
            'gamma': np.array([1.0, 1.0, 1.0], dtype=np.float32),
        }

        print(f"  LAB gain: [{avg_profile['gain_lab'][0]:.3f}, {avg_profile['gain_lab'][1]:.3f}, {avg_profile['gain_lab'][2]:.3f}]")
        print(f"  LAB offset: [{avg_profile['offset_lab'][0]:.2f}, {avg_profile['offset_lab'][1]:.2f}, {avg_profile['offset_lab'][2]:.2f}]")
        print(f"  CCM diag: [{avg_profile['ccm'][0,0]:.3f}, {avg_profile['ccm'][1,1]:.3f}, {avg_profile['ccm'][2,2]:.3f}]")

        save_color_profile(camera_hw, cam_idx, avg_profile)

    print(f"\n✓ Color calibration complete. Reference: cam_{reference_cam} ({camera_hw})")

    # Clean up temp stitch files
    for scan_dir in session_path.glob('fusion_scan_*'):
        for tmp in scan_dir.glob('_erp_calib_tmp.jpg'):
            tmp.unlink()


def normalize_session(session_dir: str):
    """Apply saved color profiles to all ERP images in a session.

    Always works from a clean source to prevent compounding:
    1. Re-stitches from .insp if available (pristine sensor data)
    2. Applies the color profile once to produce equirect_dual_fisheye.jpg
    """
    import subprocess
    import os

    session_path = _safe_data(session_dir)

    cfg_path = session_path / 'session_config.json'
    if not cfg_path.exists():
        print("No session_config.json found")
        return
    cfg = json.loads(cfg_path.read_text())
    camera_hw = cfg.get('camera_hw', 'x5')

    stitch_bin = Path(__file__).resolve().parents[1] / 'capture' / 'sdk' / 'build' / 'insta360_stitch'

    normalized = 0
    for scan_dir in sorted(session_path.glob('fusion_scan_*')):
        if not scan_dir.is_dir():
            continue
        ci_file = scan_dir / '.cam_index'
        cam_idx = int(ci_file.read_text().strip().split()[0]) if ci_file.exists() else 0

        profile = load_color_profile(camera_hw, cam_idx)
        if profile is None:
            continue  # no profile = reference camera, leave untouched

        erp = scan_dir / 'equirect_dual_fisheye.jpg'
        if not erp.exists():
            continue

        # Get clean source: re-stitch from .insp to avoid compounding
        insp_file = next(scan_dir.glob('*.insp'), None)
        img = None
        if insp_file and stitch_bin.exists():
            clean_erp = scan_dir / '_erp_clean_tmp.jpg'
            result = subprocess.run(
                [str(stitch_bin), str(insp_file), str(clean_erp)],
                capture_output=True,
            )
            if result.returncode == 0 and clean_erp.exists():
                img = cv2.imread(str(clean_erp))
                clean_erp.unlink()

        if img is None:
            # No .insp or stitch failed — only apply if not already done
            if (scan_dir / '.color_normalized').exists():
                continue
            img = cv2.imread(str(erp))

        if img is None:
            continue

        corrected = apply_color_profile(img, profile)
        cv2.imwrite(str(erp), corrected, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (scan_dir / '.color_normalized').write_text(str(cam_idx))
        normalized += 1

    print(f"✓ Normalized {normalized} images to reference camera profile")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Multi-camera color normalization')
    sub = parser.add_subparsers(dest='command')

    cal = sub.add_parser('calibrate', help='Build color profiles from a session')
    cal.add_argument('session_dir')
    cal.add_argument('--reference-cam', type=int, default=0,
                     help='Camera index to use as reference (default: 0 = highest model)')

    norm = sub.add_parser('normalize', help='Apply color profiles to a session')
    norm.add_argument('session_dir')

    args = parser.parse_args()
    if args.command == 'calibrate':
        calibrate_from_session(args.session_dir, args.reference_cam)
    elif args.command == 'normalize':
        normalize_session(args.session_dir)
    else:
        parser.print_help()
