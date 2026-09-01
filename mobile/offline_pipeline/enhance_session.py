"""
Offline enhancement pipeline for Atlas Mobile capture sessions.

Reads raw capture data from the iPhone app and produces:
1. Dense depth maps via PromptDA (LiDAR-anchored)
2. Surface normal maps via StableNormal
3. Enhanced COLMAP model with dense geometry

Usage:
    uv run python enhance_session.py --session-dir /path/to/session_YYYY-MM-DD_HH-MM-SS
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


def run_prompt_da(session_dir: Path) -> None:
    """Stage 1: Depth completion with PromptDA.

    Input:  raw/iphone/scan_NNN/depth.bin (Float32, 256x192, metres)
            raw/iphone/scan_NNN/rgb.jpg   (12MP)
    Output: enhanced/depth_dense/scan_NNN.png (uint16, mm, at RGB resolution)
    """
    import torch
    from promptda.promptda import PromptDA  # installed from models/PromptDA

    raw_dir    = session_dir / "raw" / "iphone"
    output_dir = session_dir / "enhanced" / "depth_dense"
    output_dir.mkdir(parents=True, exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"  PromptDA device: {device}")
    model = PromptDA.from_pretrained("depth-anything/promptda_vitl").to(device).eval()

    scan_dirs = sorted(raw_dir.glob("scan_*"))
    for scan_dir in scan_dirs:
        rgb_path   = scan_dir / "rgb.jpg"
        depth_path = scan_dir / "depth.bin"
        if not rgb_path.exists() or not depth_path.exists():
            continue

        scan_name   = scan_dir.name
        output_path = output_dir / f"{scan_name}.png"
        if output_path.exists():
            print(f"  PromptDA: {scan_name} already done, skipping")
            continue

        # Load RGB
        rgb = Image.open(rgb_path).convert("RGB")
        rgb_w, rgb_h = rgb.size

        # Load sparse LiDAR depth (Float32, 256x192, metres)
        raw = np.frombuffer(depth_path.read_bytes(), dtype=np.float32)
        sparse = raw[: 256 * 192].reshape(192, 256)
        # Resize sparse depth to RGB resolution for PromptDA prompt input
        sparse_resized = cv2.resize(sparse, (rgb_w, rgb_h), interpolation=cv2.INTER_NEAREST)

        with torch.no_grad():
            depth_pred = model.predict(
                image=rgb,
                prompt_depth=torch.from_numpy(sparse_resized).unsqueeze(0).unsqueeze(0).to(device),
            )  # returns (1, 1, H, W) tensor in metres

        depth_m = depth_pred.squeeze().cpu().numpy()  # (H, W) float32, metres
        depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
        cv2.imwrite(str(output_path), depth_mm)
        print(f"  PromptDA: {scan_name} -> {output_path.name}")


def run_stable_normal(session_dir: Path) -> None:
    """Stage 2: Normal estimation with StableNormal.

    Input:  raw/iphone/scan_NNN/rgb.jpg
    Output: enhanced/normals/scan_NNN.png (RGB-encoded world-space normals)
    """
    import torch
    from stablenormal.pipeline_yoso_normal import YOSONormalsPipeline
    from stablenormal.pipeline_stablenormal import StableNormalPipeline

    raw_dir    = session_dir / "raw" / "iphone"
    output_dir = session_dir / "enhanced" / "normals"
    output_dir.mkdir(parents=True, exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"  StableNormal device: {device}")

    # Two-step pipeline: coarse YOSO pass → refined diffusion pass
    yoso = YOSONormalsPipeline.from_pretrained(
        "Stable-X/yoso-normal-v0-3", trust_remote_code=True
    ).to(device)
    pipe = StableNormalPipeline.from_pretrained(
        "Stable-X/stable-normal-v0-1", trust_remote_code=True,
        yoso_version=yoso,
    ).to(device)

    scan_dirs = sorted(raw_dir.glob("scan_*"))
    for scan_dir in scan_dirs:
        rgb_path = scan_dir / "rgb.jpg"
        if not rgb_path.exists():
            continue

        scan_name   = scan_dir.name
        output_path = output_dir / f"{scan_name}.png"
        if output_path.exists():
            print(f"  StableNormal: {scan_name} already done, skipping")
            continue

        rgb = Image.open(rgb_path).convert("RGB")
        with torch.no_grad():
            result = pipe(rgb, match_input_resolution=True)

        # result.prediction is (H, W, 3) float32 in [-1, 1]
        normal = result.prediction
        # Encode to uint8 RGB: map [-1,1] → [0,255]
        normal_rgb = ((normal * 0.5 + 0.5) * 255).clip(0, 255).astype(np.uint8)
        cv2.imwrite(str(output_path), cv2.cvtColor(normal_rgb, cv2.COLOR_RGB2BGR))
        print(f"  StableNormal: {scan_name} -> {output_path.name}")


def assemble_colmap(session_dir: Path, tile_size: int = 1024) -> None:
    """Stage 3: Assemble COLMAP model from raw capture data.

    Slices Insta360 ERPs into perspective tiles, unprojects iPhone LiDAR
    depth into world-frame point clouds, and writes the binary COLMAP model.
    """
    from assemble_colmap import assemble_colmap as _assemble
    _assemble(session_dir, tile_size=tile_size)


def main() -> None:
    parser = argparse.ArgumentParser(description="Atlas Mobile offline enhancement")
    parser.add_argument(
        "--session-dir",
        type=Path,
        required=True,
        help="Path to captured session directory",
    )
    parser.add_argument(
        "--skip-depth",
        action="store_true",
        help="Skip PromptDA depth completion",
    )
    parser.add_argument(
        "--skip-normals",
        action="store_true",
        help="Skip StableNormal estimation",
    )
    args = parser.parse_args()

    session_dir = args.session_dir
    if not session_dir.exists():
        raise FileNotFoundError(f"Session directory not found: {session_dir}")

    print(f"Enhancing session: {session_dir}")

    if not args.skip_depth:
        print("Stage 1: PromptDA depth completion")
        run_prompt_da(session_dir)

    if not args.skip_normals:
        print("Stage 2: StableNormal estimation")
        run_stable_normal(session_dir)

    print("Stage 3: COLMAP model assembly")
    assemble_colmap(session_dir, tile_size=1024)

    print("Done.")


if __name__ == "__main__":
    main()
