#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Offline depth enhancement tool for ATLAS colmap.zip exports.
#
# Replaces the sparse LiDAR depth images in a colmap.zip with dense,
# continuous depth maps produced by PromptDA (depth-anything/promptda_vitl).
# PromptDA uses the sparse LiDAR depth as a metric anchor and the RGB tile
# as guidance, producing smooth planar surfaces with sharp edges aligned to
# object boundaries — eliminating the blocky splat appearance from sparse
# LiDAR projection.
#
# Input:  colmap.zip  (output from ATLAS scanner pipeline)
# Output: colmap_enhanced.zip  (drop-in replacement, same structure)
#
# Usage:
#   python enhance_depth.py colmap.zip
#   python enhance_depth.py colmap.zip --output colmap_enhanced.zip
#   python enhance_depth.py colmap.zip --batch-size 4   # tiles per GPU batch

import argparse
import io
import sys
import zipfile
from pathlib import Path, PurePosixPath

import cv2
import numpy as np
import torch
from PIL import Image
from promptda.promptda import PromptDA
from tqdm import tqdm


# ---------------------------------------------------------------------------
# Model loading (cached singleton)
# ---------------------------------------------------------------------------

_MODEL: PromptDA | None = None
_DEVICE: str = ''


def _get_model() -> tuple[PromptDA, str]:
    global _MODEL, _DEVICE
    if _MODEL is None:
        _DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
        if _DEVICE == 'cpu':
            print('WARNING: No CUDA GPU detected — running on CPU (slow).')
        print(f'Loading PromptDA (depth-anything/promptda_vitl) on {_DEVICE}...')
        _MODEL = PromptDA.from_pretrained('depth-anything/promptda_vitl').to(_DEVICE).eval()
        print('Model ready.')
    return _MODEL, _DEVICE


# ---------------------------------------------------------------------------
# Per-tile enhancement
# ---------------------------------------------------------------------------

def enhance_tile(
    rgb_png: bytes,
    depth_png: bytes,
    model: PromptDA,
    device: str,
) -> bytes:
    """
    Run PromptDA on one (rgb, sparse_depth) tile pair.

    rgb_png:   PNG bytes of the RGB tile (uint8, 1024x1024x3)
    depth_png: PNG bytes of the sparse depth tile (uint16, mm)

    Returns PNG bytes of the dense depth tile (uint16, mm), same resolution.
    """
    # Decode RGB
    rgb_arr = cv2.imdecode(np.frombuffer(rgb_png, np.uint8), cv2.IMREAD_COLOR)
    rgb_pil = Image.fromarray(cv2.cvtColor(rgb_arr, cv2.COLOR_BGR2RGB))

    # Decode sparse depth (uint16 mm → float32 metres)
    depth_arr = cv2.imdecode(np.frombuffer(depth_png, np.uint8), cv2.IMREAD_UNCHANGED)
    sparse_m = depth_arr.astype(np.float32) / 1000.0

    # PromptDA: prompt_depth must be (1, 1, H, W) float32 tensor in metres
    prompt = torch.from_numpy(sparse_m).unsqueeze(0).unsqueeze(0).to(device)

    with torch.no_grad():
        depth_pred = model.predict(image=rgb_pil, prompt_depth=prompt)

    # Output: (H, W) float32 metres → uint16 mm
    depth_m = depth_pred.squeeze().cpu().numpy()
    depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)

    # Encode back to PNG
    ok, buf = cv2.imencode('.png', depth_mm)
    if not ok:
        raise RuntimeError('Failed to encode enhanced depth tile')
    return buf.tobytes()


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def enhance_colmap_zip(input_zip: Path, output_zip: Path) -> None:
    model, device = _get_model()

    with zipfile.ZipFile(input_zip, 'r') as zin:
        all_names = zin.namelist()

        # Index depth tiles and their matching RGB tiles
        depth_names = [
            n for n in all_names
            if 'depth_images' in n and n.endswith('.png')
        ]
        if not depth_names:
            print('ERROR: No depth_images/*.png found in zip.')
            sys.exit(1)

        # Build depth → rgb path mapping
        # depth: colmap/depth_images/face_XX/pano_NNN.png
        # rgb:   colmap/images/face_XX/pano_NNN.png
        def depth_to_rgb(depth_path: str) -> str:
            p = PurePosixPath(depth_path)
            # Replace 'depth_images' component with 'images'
            parts = list(p.parts)
            di = parts.index('depth_images')
            parts[di] = 'images'
            return str(PurePosixPath(*parts))

        pairs = []
        missing_rgb = []
        for d in depth_names:
            r = depth_to_rgb(d)
            if r in all_names:
                pairs.append((d, r))
            else:
                missing_rgb.append(d)

        if missing_rgb:
            print(f'WARNING: {len(missing_rgb)} depth tiles have no matching RGB — will copy unchanged.')

        print(f'Enhancing {len(pairs)} depth tiles...')

        # Build set of depth paths that will be replaced
        enhanced_depth: dict[str, bytes] = {}

        for depth_name, rgb_name in tqdm(pairs, unit='tile'):
            rgb_bytes   = zin.read(rgb_name)
            depth_bytes = zin.read(depth_name)
            try:
                enhanced = enhance_tile(rgb_bytes, depth_bytes, model, device)
                enhanced_depth[depth_name] = enhanced
            except Exception as e:
                print(f'\nWARNING: Failed to enhance {depth_name}: {e} — copying original.')
                enhanced_depth[depth_name] = depth_bytes

        # Write output zip: copy everything, replacing depth tiles
        print(f'Writing {output_zip.name}...')
        with zipfile.ZipFile(output_zip, 'w', zipfile.ZIP_DEFLATED) as zout:
            for name in tqdm(all_names, unit='file', desc='Packing'):
                if name in enhanced_depth:
                    zout.writestr(name, enhanced_depth[name])
                else:
                    zout.writestr(name, zin.read(name))

    print(f'\n✓ Enhanced colmap zip: {output_zip}')
    print(f'  Input:  {input_zip.stat().st_size / 1e6:.1f} MB')
    print(f'  Output: {output_zip.stat().st_size / 1e6:.1f} MB')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Enhance ATLAS colmap.zip depth images using PromptDA.'
    )
    parser.add_argument('input_zip', type=Path,
                        help='Path to colmap.zip from ATLAS scanner pipeline')
    parser.add_argument('--output', type=Path, default=None,
                        help='Output zip path (default: <input>_enhanced.zip)')
    args = parser.parse_args()

    if not args.input_zip.exists():
        print(f'ERROR: {args.input_zip} not found.')
        sys.exit(1)

    output = args.output or args.input_zip.with_stem(args.input_zip.stem + '_enhanced')
    enhance_colmap_zip(args.input_zip, output)


if __name__ == '__main__':
    main()
