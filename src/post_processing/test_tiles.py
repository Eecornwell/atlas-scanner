#!/usr/bin/env python3
"""Quick tile generation test — outputs perspective tiles from one ERP for visual inspection.

Usage:
  python3 test_tiles.py <erp_image.jpg> [output_dir]
  python3 test_tiles.py /path/to/session/fusion_scan_001/equirect_dual_fisheye.jpg /tmp/tiles

Generates face_00.jpg through face_07.jpg in the output directory.
"""
import cv2
import numpy as np
import sys
from pathlib import Path
from scipy.spatial.transform import Rotation as R

FOV_DEG = 65.0

FACES = [
    {'name': 'front',       'pitch':   0, 'yaw':   0},
    {'name': 'front_left',  'pitch':   0, 'yaw':  45},
    {'name': 'left',        'pitch':   0, 'yaw':  90},
    {'name': 'back_left',   'pitch':   0, 'yaw': 135},
    {'name': 'back',        'pitch':   0, 'yaw': 180},
    {'name': 'back_right',  'pitch':   0, 'yaw': 225},
    {'name': 'right',       'pitch':   0, 'yaw': 270},
    {'name': 'front_right', 'pitch':   0, 'yaw': 315},
]

_R_INSTA_CORRECTION = R.from_euler('Y', 90, degrees=True).as_matrix()
FACES_CAM_FROM_PANO = [
    (R.from_euler('XY', [-f['pitch'], -f['yaw']], degrees=True).as_matrix()
     @ _R_INSTA_CORRECTION)
    for f in FACES
]


def erp_to_perspective(erp_img, cam_from_pano_r, tile_size, interpolation=cv2.INTER_LANCZOS4):
    pano_h, pano_w = erp_img.shape[:2]
    f = tile_size / (2 * np.tan(np.radians(FOV_DEG) / 2))
    c = tile_size / 2.0
    x, y = np.meshgrid(np.arange(tile_size) + 0.5, np.arange(tile_size) + 0.5)
    rays_cam = np.stack([(x - c) / f, (y - c) / f, np.ones((tile_size, tile_size))], axis=-1)
    rays_cam /= np.linalg.norm(rays_cam, axis=-1, keepdims=True)
    rays_pano = rays_cam.reshape(-1, 3) @ cam_from_pano_r
    r = rays_pano.T
    yaw = np.arctan2(r[0], r[2])
    pitch = -np.arctan2(r[1], np.linalg.norm(r[[0, 2]], axis=0))
    u = ((1 + yaw / np.pi) / 2 * pano_w - 0.5).astype(np.float32).reshape(tile_size, tile_size)
    v = ((1 - pitch * 2 / np.pi) / 2 * pano_h - 0.5).astype(np.float32).reshape(tile_size, tile_size)
    return cv2.remap(erp_img, u, v, interpolation, borderMode=cv2.BORDER_WRAP)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    erp_path = sys.argv[1]
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path('/tmp/tiles')
    out_dir.mkdir(parents=True, exist_ok=True)

    erp_img = cv2.imread(erp_path)
    if erp_img is None:
        print(f"Failed to read: {erp_path}")
        sys.exit(1)

    erp_h, erp_w = erp_img.shape[:2]
    tile_size = int(erp_w / 360.0 * np.cos(np.radians(FOV_DEG / 2)) * FOV_DEG)
    tile_size = max(512, min(2048, int(2 ** int(np.log2(tile_size)))))

    print(f"ERP: {erp_w}x{erp_h}")
    print(f"Tile size: {tile_size}x{tile_size}")
    print(f"FOV: {FOV_DEG}°")
    print(f"Output: {out_dir}")
    print()

    for i, (face, cam_from_pano) in enumerate(zip(FACES, FACES_CAM_FROM_PANO)):
        tile = erp_to_perspective(erp_img, cam_from_pano, tile_size)
        out_path = out_dir / f"face_{i:02d}_{face['name']}.jpg"
        cv2.imwrite(str(out_path), tile, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"  {out_path.name}  (yaw={face['yaw']:3d}°)")

    print(f"\nDone. Inspect tiles in: {out_dir}")


if __name__ == '__main__':
    main()
