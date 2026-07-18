#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Generate a calibration mask for a specific camera position.
#
# Takes a stitched ERP image from a single-camera session and creates a binary
# mask where white=valid scene, black=scanner body/LiDAR/hardware to exclude.
#
# Methods:
#   --from-existing <mask>   Start from an existing mask and refine
#   --auto                   Auto-detect dark/occluded regions (scanner body)
#   --interactive            Open GUI for manual painting (requires display)
#
# Usage:
#   python3 generate_calibration_mask.py <erp_image> <output_mask> [options]
#   python3 generate_calibration_mask.py <session_dir> <cam_index> [options]

import argparse
import sys
import os
import numpy as np
import cv2
from pathlib import Path

_MASKS_DIR = Path(__file__).resolve().parent.parent / 'config' / 'masks'


def auto_mask(erp_img, threshold=15, min_region_area=5000):
    """Auto-detect scanner body as consistently dark regions across the ERP."""
    gray = cv2.cvtColor(erp_img, cv2.COLOR_BGR2GRAY) if erp_img.ndim == 3 else erp_img

    # Scanner body appears as very dark pixels (near-black) in consistent positions
    dark = (gray < threshold).astype(np.uint8) * 255

    # Morphological close to fill small gaps in the scanner body
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel)

    # Remove small noise regions
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask = np.ones_like(gray) * 255
    for c in contours:
        if cv2.contourArea(c) > min_region_area:
            cv2.drawContours(mask, [c], -1, 0, -1)

    # Also mask the nadir (bottom ~5%) which typically shows the tripod
    h = mask.shape[0]
    nadir_rows = int(h * 0.05)
    mask[h - nadir_rows:, :] = 0

    return mask


def from_existing(existing_mask_path, erp_shape):
    """Load an existing mask and resize to match the ERP dimensions."""
    mask = cv2.imread(str(existing_mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        print(f"✗ Could not load mask: {existing_mask_path}")
        sys.exit(1)
    if mask.shape[:2] != erp_shape[:2]:
        mask = cv2.resize(mask, (erp_shape[1], erp_shape[0]), interpolation=cv2.INTER_NEAREST)
    return mask


def interactive_mask(erp_img, initial_mask=None):
    """Open a GUI window for manual mask painting."""
    h, w = erp_img.shape[:2]
    # Scale down for display if too large
    scale = min(1.0, 1920 / w, 1080 / h)
    disp_w, disp_h = int(w * scale), int(h * scale)

    if initial_mask is not None:
        mask = initial_mask.copy()
    else:
        mask = np.ones((h, w), dtype=np.uint8) * 255

    brush_size = max(10, int(30 / scale))
    painting = [False]
    erasing = [False]

    def mouse_cb(event, x, y, flags, param):
        # Map display coords back to full-res
        fx, fy = int(x / scale), int(y / scale)
        if event == cv2.EVENT_LBUTTONDOWN:
            painting[0] = True
        elif event == cv2.EVENT_RBUTTONDOWN:
            erasing[0] = True
        elif event == cv2.EVENT_LBUTTONUP:
            painting[0] = False
        elif event == cv2.EVENT_RBUTTONUP:
            erasing[0] = False
        elif event == cv2.EVENT_MOUSEMOVE:
            if painting[0]:
                cv2.circle(mask, (fx, fy), brush_size, 0, -1)
            elif erasing[0]:
                cv2.circle(mask, (fx, fy), brush_size, 255, -1)

    cv2.namedWindow('Calibration Mask Editor', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Calibration Mask Editor', disp_w, disp_h)
    cv2.setMouseCallback('Calibration Mask Editor', mouse_cb)

    print("\nMask Editor Controls:")
    print("  Left-click + drag  = paint BLACK (exclude region)")
    print("  Right-click + drag = paint WHITE (include region)")
    print("  +/-                = increase/decrease brush size")
    print("  s                  = save and exit")
    print("  q                  = quit without saving")
    print("")

    while True:
        # Overlay mask on image
        overlay = erp_img.copy()
        red_overlay = np.zeros_like(overlay)
        red_overlay[:, :, 2] = 255  # Red channel
        excluded = mask < 128
        overlay[excluded] = cv2.addWeighted(
            overlay[excluded], 0.3,
            red_overlay[excluded], 0.7, 0
        )

        disp = cv2.resize(overlay, (disp_w, disp_h))
        cv2.putText(disp, f"Brush: {brush_size}px | L-click=exclude R-click=include | s=save q=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow('Calibration Mask Editor', disp)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('s'):
            cv2.destroyAllWindows()
            return mask
        elif key == ord('q'):
            cv2.destroyAllWindows()
            return None
        elif key == ord('+') or key == ord('='):
            brush_size = min(200, brush_size + 5)
        elif key == ord('-'):
            brush_size = max(5, brush_size - 5)

    return mask


def find_erp_from_session(session_dir, scan_index=0):
    """Find a stitched ERP image from a session directory."""
    session = Path(session_dir)
    scan_dirs = sorted(d for d in session.iterdir()
                       if d.is_dir() and d.name.startswith('fusion_scan'))
    if scan_index >= len(scan_dirs):
        scan_index = 0
    if not scan_dirs:
        return None

    scan = scan_dirs[scan_index]
    # Prefer the unmasked ERP
    candidates = (
        list(scan.glob('equirect_dual_fisheye.jpg')) +
        list(scan.glob('equirect_dual_fisheye_raw.jpg')) +
        [f for f in scan.glob('equirect_*.jpg') if '_masked' not in f.name]
    )
    return str(candidates[0]) if candidates else None


def main():
    parser = argparse.ArgumentParser(
        description='Generate a calibration mask for a camera position.')
    parser.add_argument('input', help='ERP image path, or session directory')
    parser.add_argument('output', nargs='?', default=None,
                        help='Output mask path (default: auto from cam_index)')
    parser.add_argument('--cam-index', type=int, default=None,
                        help='Camera index (used for auto-naming output)')
    parser.add_argument('--from-existing', type=str, default=None,
                        help='Start from an existing mask file')
    parser.add_argument('--auto', action='store_true',
                        help='Auto-detect scanner body (dark regions)')
    parser.add_argument('--interactive', action='store_true',
                        help='Open GUI editor for manual painting')
    parser.add_argument('--threshold', type=int, default=15,
                        help='Dark pixel threshold for --auto mode (default: 15)')
    args = parser.parse_args()

    # Resolve input
    input_path = Path(args.input)
    if input_path.is_dir():
        erp_path = find_erp_from_session(str(input_path))
        if erp_path is None:
            print(f"✗ No ERP image found in session: {input_path}")
            sys.exit(1)
        print(f"Using ERP: {erp_path}")
    else:
        erp_path = str(input_path)

    erp_img = cv2.imread(erp_path)
    if erp_img is None:
        print(f"✗ Could not load image: {erp_path}")
        sys.exit(1)

    # Determine output path
    if args.output:
        output_path = Path(args.output)
    elif args.cam_index is not None:
        # Read mask_calibration name from multi_camera.yaml
        import yaml
        mc_yaml = Path(__file__).resolve().parent.parent / 'config' / 'multi_camera.yaml'
        if mc_yaml.exists():
            d = yaml.safe_load(mc_yaml.read_text())
            c = d.get('cameras', {}).get(f'cam_{args.cam_index}', {})
            mask_name = c.get('mask_calibration', f'lidar_mask_calib_cam{args.cam_index}.png')
        else:
            mask_name = f'lidar_mask_calib_cam{args.cam_index}.png'
        output_path = _MASKS_DIR / mask_name
    else:
        output_path = _MASKS_DIR / 'lidar_mask_calib_new.png'

    print(f"Output mask: {output_path}")
    print(f"ERP size: {erp_img.shape[1]}x{erp_img.shape[0]}")

    # Generate mask
    if args.from_existing:
        mask = from_existing(args.from_existing, erp_img.shape)
        print(f"Loaded existing mask: {args.from_existing}")
        if args.interactive:
            mask = interactive_mask(erp_img, mask)
            if mask is None:
                print("Cancelled.")
                sys.exit(0)
    elif args.auto:
        mask = auto_mask(erp_img, threshold=args.threshold)
        excluded_pct = (mask < 128).sum() / mask.size * 100
        print(f"Auto-detected {excluded_pct:.1f}% excluded area")
        if args.interactive:
            mask = interactive_mask(erp_img, mask)
            if mask is None:
                print("Cancelled.")
                sys.exit(0)
    elif args.interactive:
        mask = interactive_mask(erp_img)
        if mask is None:
            print("Cancelled.")
            sys.exit(0)
    else:
        # Default: auto + save (no GUI)
        mask = auto_mask(erp_img, threshold=args.threshold)
        excluded_pct = (mask < 128).sum() / mask.size * 100
        print(f"Auto-detected {excluded_pct:.1f}% excluded area")

    # Save
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), mask)
    print(f"\n✓ Saved calibration mask: {output_path}")
    print(f"  White (valid):    {(mask >= 128).sum() / mask.size * 100:.1f}%")
    print(f"  Black (excluded): {(mask < 128).sum() / mask.size * 100:.1f}%")


if __name__ == '__main__':
    main()
