"""
Equirectangular → perspective tile slicer.

Port of atlas-scanner's ERP tile extraction for offline use.
Used when tile slicing is deferred from the iOS app to the offline pipeline.

Usage:
    uv run python erp_tile_slicer.py \
        --erp-image /path/to/panorama.jpg \
        --output-dir /path/to/output/ \
        --face-count 8 \
        --tile-fov 90 \
        --tile-size 1024
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def compute_face_rotations(face_count: int) -> list[np.ndarray]:
    """Compute rotation matrices for each face tile direction.

    face_00 = ceiling (looking up)
    face_01..N = evenly spaced around the horizontal ring
    """
    rotations: list[np.ndarray] = []

    # Ceiling face (looking up, -Y in ERP convention)
    r_ceiling = cv2.Rodrigues(np.array([np.pi / 2, 0, 0], dtype=np.float64))[0]
    rotations.append(r_ceiling)

    # Horizontal ring faces
    horizontal_count = face_count - 1
    for i in range(horizontal_count):
        yaw = 2 * np.pi * i / horizontal_count
        r_face = cv2.Rodrigues(np.array([0, yaw, 0], dtype=np.float64))[0]
        rotations.append(r_face)

    return rotations


def erp_to_perspective(
    erp_image: np.ndarray,
    rotation: np.ndarray,
    fov_deg: float,
    tile_size: int,
) -> np.ndarray:
    """Reprojects an equirectangular image to a perspective tile."""
    f = tile_size / (2.0 * np.tan(np.radians(fov_deg / 2.0)))
    cx = tile_size / 2.0
    cy = tile_size / 2.0

    h_erp, w_erp = erp_image.shape[:2]

    # Build pixel grid for output tile
    u, v = np.meshgrid(np.arange(tile_size), np.arange(tile_size))
    x = (u - cx) / f
    y = (v - cy) / f
    z = np.ones_like(x)

    # Normalize to unit sphere
    norm = np.sqrt(x**2 + y**2 + z**2)
    dirs = np.stack([x / norm, y / norm, z / norm], axis=-1)

    # Rotate directions
    dirs_world = dirs @ rotation.T

    # Convert to spherical coordinates
    lon = np.arctan2(dirs_world[..., 0], dirs_world[..., 2])
    lat = np.arcsin(np.clip(dirs_world[..., 1], -1, 1))

    # Map to ERP pixel coordinates
    u_erp = ((lon / np.pi + 1) / 2 * w_erp).astype(np.float32)
    v_erp = ((0.5 - lat / np.pi) * h_erp).astype(np.float32)

    # Sample
    tile = cv2.remap(erp_image, u_erp, v_erp, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
    return tile


def slice_erp(
    erp_path: Path,
    output_dir: Path,
    face_count: int = 8,
    tile_fov: float = 90.0,
    tile_size: int = 1024,
    mask_path: Path | None = None,
) -> list[Path]:
    """Slice an ERP image into perspective face tiles, optionally with masks."""
    erp_image = cv2.imread(str(erp_path))
    if erp_image is None:
        raise FileNotFoundError(f"Could not read: {erp_path}")

    erp_mask = None
    if mask_path and mask_path.exists():
        erp_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)

    rotations = compute_face_rotations(face_count)
    output_paths: list[Path] = []

    for face_idx, rotation in enumerate(rotations):
        face_dir = output_dir / f"face_{face_idx:02d}"
        face_dir.mkdir(parents=True, exist_ok=True)

        tile = erp_to_perspective(erp_image, rotation, tile_fov, tile_size)
        tile_path = face_dir / erp_path.with_suffix(".jpg").name
        cv2.imwrite(str(tile_path), tile)
        output_paths.append(tile_path)

        if erp_mask is not None:
            tile_mask = erp_to_perspective(
                erp_mask[..., np.newaxis], rotation, tile_fov, tile_size
            )
            mask_dir = output_dir.parent / "masks" / f"face_{face_idx:02d}"
            mask_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(mask_dir / "mask.png"), tile_mask)

    return output_paths


def main() -> None:
    parser = argparse.ArgumentParser(description="ERP → perspective tile slicer")
    parser.add_argument("--erp-image", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--face-count", type=int, default=8)
    parser.add_argument("--tile-fov", type=float, default=90.0)
    parser.add_argument("--tile-size", type=int, default=1024)
    parser.add_argument("--mask", type=Path, default=None)
    args = parser.parse_args()

    paths = slice_erp(
        args.erp_image,
        args.output_dir,
        args.face_count,
        args.tile_fov,
        args.tile_size,
        args.mask,
    )
    for p in paths:
        print(f"  → {p}")


if __name__ == "__main__":
    main()
