"""
Equirectangular → perspective tile slicer.

Port of atlas-scanner's ERP tile extraction (panorama_sfm_colmap.py).
8 equatorial faces at 45° yaw intervals, 65° FOV, Lanczos sampling.
Face layout matches atlas exactly so COLMAP models are compatible.

Usage:
    uv run python erp_tile_slicer.py \
        --erp-image /path/to/panorama.jpg \
        --output-dir /path/to/colmap/images/ \
        --scan-name scan_000 \
        [--mask /path/to/mask.png] \
        [--tile-fov 65] \
        [--tile-size 1024]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

# 8 equatorial faces at 45° yaw intervals — matches atlas panorama_sfm_colmap.py.
# No polar (ceiling/floor) faces: ERP pixel density is too low at the poles
# for reliable feature extraction.
FACES = [
    {"name": "front",       "pitch": 0, "yaw":   0},
    {"name": "front_left",  "pitch": 0, "yaw":  45},
    {"name": "left",        "pitch": 0, "yaw":  90},
    {"name": "back_left",   "pitch": 0, "yaw": 135},
    {"name": "back",        "pitch": 0, "yaw": 180},
    {"name": "back_right",  "pitch": 0, "yaw": 225},
    {"name": "right",       "pitch": 0, "yaw": 270},
    {"name": "front_right", "pitch": 0, "yaw": 315},
]

# Insta360 SDK ERP has forward (+X) at top (v=0); pre-compose R_y(+90°) to
# land equatorial faces on the actual horizon band (same correction as atlas).
_R_INSTA_CORRECTION = R.from_euler("Y", 90, degrees=True).as_matrix()

FACES_CAM_FROM_PANO: list[np.ndarray] = [
    R.from_euler("XY", [-f["pitch"], -f["yaw"]], degrees=True).as_matrix()
    @ _R_INSTA_CORRECTION
    for f in FACES
]

FOV_DEG = 65.0
NUM_FACES = len(FACES)


def tile_intrinsics(tile_size: int, fov_deg: float = FOV_DEG) -> tuple[float, float, float]:
    """Returns (f_px, cx, cy) for SIMPLE_PINHOLE."""
    f = tile_size / (2.0 * np.tan(np.radians(fov_deg / 2.0)))
    c = tile_size / 2.0
    return f, c, c


def erp_to_perspective(
    erp_img: np.ndarray,
    cam_from_pano: np.ndarray,
    tile_size: int,
    fov_deg: float = FOV_DEG,
    interpolation: int = cv2.INTER_LANCZOS4,
) -> np.ndarray:
    """Sample a perspective tile from an ERP image."""
    h_erp, w_erp = erp_img.shape[:2]
    f, cx, cy = tile_intrinsics(tile_size, fov_deg)

    x, y = np.meshgrid(np.arange(tile_size) + 0.5, np.arange(tile_size) + 0.5)
    rays = np.stack([(x - cx) / f, (y - cy) / f, np.ones_like(x)], axis=-1)
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)

    rays_pano = (rays.reshape(-1, 3) @ cam_from_pano).reshape(tile_size, tile_size, 3)
    r = rays_pano.transpose(2, 0, 1)  # (3, H, W)

    yaw   = np.arctan2(r[0], r[2])
    pitch = -np.arctan2(r[1], np.linalg.norm(r[[0, 2]], axis=0))

    u = ((1 + yaw / np.pi) / 2 * w_erp - 0.5).astype(np.float32)
    v = ((1 - pitch * 2 / np.pi) / 2 * h_erp - 0.5).astype(np.float32)

    return cv2.remap(erp_img, u, v, interpolation, borderMode=cv2.BORDER_WRAP)


def slice_erp(
    erp_path: Path,
    images_dir: Path,
    scan_name: str,
    mask_path: Path | None = None,
    masks_dir: Path | None = None,
    tile_size: int = 1024,
    fov_deg: float = FOV_DEG,
    min_visible: float = 0.20,
) -> list[tuple[int, Path]]:
    """
    Slice an ERP into perspective face tiles.

    Returns list of (face_index, tile_path) for faces that passed the
    visibility threshold (>= min_visible fraction of unmasked pixels).
    """
    erp_img = cv2.imread(str(erp_path), cv2.IMREAD_UNCHANGED)
    if erp_img is None:
        raise FileNotFoundError(f"Could not read: {erp_path}")

    erp_mask: np.ndarray | None = None
    if erp_img.ndim == 3 and erp_img.shape[2] == 4:
        erp_mask = erp_img[:, :, 3]
        erp_img = erp_img[:, :, :3]
    elif mask_path and mask_path.exists():
        erp_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)

    # Mask stitch seam (4% strip on each horizontal edge, same as atlas)
    if erp_mask is not None:
        w = erp_img.shape[1]
        seam = int(w * 0.04)
        erp_mask[:, :seam] = 0
        erp_mask[:, w - seam:] = 0

    results: list[tuple[int, Path]] = []

    for face_idx, cam_from_pano in enumerate(FACES_CAM_FROM_PANO):
        tile = erp_to_perspective(erp_img, cam_from_pano, tile_size, fov_deg)

        tile_mask: np.ndarray | None = None
        if erp_mask is not None:
            tile_mask = erp_to_perspective(
                erp_mask, cam_from_pano, tile_size, fov_deg,
                interpolation=cv2.INTER_NEAREST,
            )
            if np.count_nonzero(tile_mask) / tile_mask.size < min_visible:
                continue

        face_dir = images_dir / f"face_{face_idx:02d}"
        face_dir.mkdir(parents=True, exist_ok=True)
        tile_path = face_dir / f"{scan_name}.jpg"
        cv2.imwrite(str(tile_path), tile, [cv2.IMWRITE_JPEG_QUALITY, 95])
        results.append((face_idx, tile_path))

        if tile_mask is not None and masks_dir is not None:
            mask_face_dir = masks_dir / f"face_{face_idx:02d}"
            mask_face_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(mask_face_dir / "mask.png"), tile_mask)

    return results


def main() -> None:
    parser = argparse.ArgumentParser(description="ERP → perspective tile slicer (atlas-compatible)")
    parser.add_argument("--erp-image", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="colmap/images/ directory")
    parser.add_argument("--scan-name", type=str, required=True,
                        help="Output filename stem, e.g. scan_000")
    parser.add_argument("--mask", type=Path, default=None)
    parser.add_argument("--masks-dir", type=Path, default=None,
                        help="colmap/masks/ directory for per-face mask output")
    parser.add_argument("--tile-fov", type=float, default=FOV_DEG)
    parser.add_argument("--tile-size", type=int, default=1024)
    args = parser.parse_args()

    results = slice_erp(
        args.erp_image,
        args.output_dir,
        args.scan_name,
        mask_path=args.mask,
        masks_dir=args.masks_dir,
        tile_size=args.tile_size,
        fov_deg=args.tile_fov,
    )
    for face_idx, path in results:
        print(f"  face_{face_idx:02d} → {path}")


if __name__ == "__main__":
    main()
