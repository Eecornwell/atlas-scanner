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
from pathlib import Path


def run_prompt_da(session_dir: Path) -> None:
    """Stage 1: Depth completion with PromptDA.

    Input: raw/iphone/scan_NNN/depth.bin (sparse 256x192) + rgb.jpg (12MP)
    Output: enhanced/depth_dense/scan_NNN.png (uint16 mm at RGB resolution)
    """
    raw_dir = session_dir / "raw" / "iphone"
    output_dir = session_dir / "enhanced" / "depth_dense"
    output_dir.mkdir(parents=True, exist_ok=True)

    scan_dirs = sorted(raw_dir.glob("scan_*"))
    for scan_dir in scan_dirs:
        rgb_path = scan_dir / "rgb.jpg"
        depth_path = scan_dir / "depth.bin"
        if not rgb_path.exists() or not depth_path.exists():
            continue

        scan_name = scan_dir.name
        output_path = output_dir / f"{scan_name}.png"

        # TODO: Load sparse depth (Float32, 256x192) and RGB
        # TODO: Run PromptDA model
        # TODO: Save as uint16 PNG in millimeters
        print(f"  PromptDA: {scan_name} -> {output_path}")


def run_stable_normal(session_dir: Path) -> None:
    """Stage 2: Normal estimation with StableNormal.

    Input: raw/iphone/scan_NNN/rgb.jpg
    Output: enhanced/normals/scan_NNN.png (RGB-encoded normal map)
    """
    raw_dir = session_dir / "raw" / "iphone"
    output_dir = session_dir / "enhanced" / "normals"
    output_dir.mkdir(parents=True, exist_ok=True)

    scan_dirs = sorted(raw_dir.glob("scan_*"))
    for scan_dir in scan_dirs:
        rgb_path = scan_dir / "rgb.jpg"
        if not rgb_path.exists():
            continue

        scan_name = scan_dir.name
        output_path = output_dir / f"{scan_name}.png"

        # TODO: Run StableNormal model
        # TODO: Save as RGB-encoded normal map
        print(f"  StableNormal: {scan_name} -> {output_path}")


def assemble_colmap(session_dir: Path) -> None:
    """Stage 3: Assemble enhanced COLMAP model.

    Uses dense depth maps and original poses to produce a
    COLMAP-compatible dataset for splat-toolbox ingestion.
    """
    colmap_dir = session_dir / "colmap"

    # TODO: Slice Insta360 ERP images into perspective face tiles
    # TODO: Apply masks to tiles
    # TODO: Unproject dense depth maps into point clouds
    # TODO: Write cameras.bin, images.bin, points3D.bin
    # TODO: Optionally run COLMAP feature matching + BA
    print(f"  COLMAP assembly -> {colmap_dir}")


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
    assemble_colmap(session_dir)

    print("Done.")


if __name__ == "__main__":
    main()
