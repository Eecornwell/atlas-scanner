#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Projects sensor-frame PLY point clouds onto a 2D intensity image for use as LiDAR input to the SuperGlue feature matcher during camera-LiDAR calibration.
import numpy as np
import cv2
import sys
from pathlib import Path

CAM_LEFT = (0, 960)      # camera content left strip (cols)
CAM_RIGHT = (2880, 3840) # camera content right strip (cols)

def remap_to_strips(image):
    """Zero out the black middle zone in the camera image."""
    out = image.copy()
    h, w = out.shape[:2]
    # Scale column boundaries to actual image size
    mid_start = w // 4
    mid_end = w * 3 // 4
    out[:, mid_start:mid_end] = 0
    return out

def remap_lidar_to_strips(image):
    """Lidar covers full ERP - no transformation needed."""
    return image

def generate_intensity_image(ply_file, output_image, point_indices_image, camera_image=None, width=3840, height=1920):
    # Auto-detect dimensions from camera image if available
    if camera_image is not None:
        cam_probe = cv2.imread(camera_image)
        if cam_probe is not None:
            height, width = cam_probe.shape[:2]
    points = []
    intensities = []

    with open(ply_file, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            header_lines.append(line.decode('ascii'))
            if line.strip() == b'end_header':
                break
        header = ''.join(header_lines)
        binary = 'binary_little_endian' in header
        n_verts = int(next(l.split()[-1] for l in header_lines if l.startswith('element vertex')))
        fields = [l.split()[-1].strip() for l in header_lines if l.startswith('property float')]
        has_intensity = 'intensity' in fields
        n_fields = len(fields)
        if binary:
            raw = np.frombuffer(f.read(n_verts * n_fields * 4), dtype=np.float32).reshape(n_verts, n_fields)
            points = raw[:, :3].tolist()
            intensities = raw[:, 3].tolist() if has_intensity and n_fields >= 4 else [0.5] * n_verts
        else:
            for line in f.read().decode('ascii').splitlines():
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        intensity = float(parts[3]) if has_intensity and len(parts) > 3 else 0.5
                        points.append([x, y, z])
                        intensities.append(intensity)
                    except ValueError:
                        continue

    points = np.array(points)
    intensities = np.array(intensities)

    if len(points) == 0:
        print(f"No points found in {ply_file}")
        return False

    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # Pre-rotate 90° yaw to align lidar projection with camera ERP orientation
    r = np.pi / 2
    x, y = x*np.cos(r) - y*np.sin(r), x*np.sin(r) + y*np.cos(r)

    norm = np.sqrt(x**2 + y**2 + z**2).clip(1e-6)
    lat = np.arcsin(np.clip(y / norm, -1, 1))
    lon = np.arctan2(z, -x)

    u = (width  * (0.5 + lon / (2 * np.pi))).astype(int)
    v = (height * (0.5 - lat / np.pi)).astype(int)
    u = (u + width // 4) % width

    valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    ranges = norm[valid]
    u, v = u[valid], v[valid]
    intensities = intensities[valid]
    point_indices = np.where(valid)[0]

    out_w, out_h = width // 2, height // 2
    range_image = np.zeros((out_h, out_w), dtype=np.float32)
    indices_image = np.full((out_h, out_w), -1, dtype=np.int32)

    if intensities.max() > 1.0:
        intensities = intensities / 255.0

    u2, v2 = u // 2, v // 2
    valid2 = (u2 >= 0) & (u2 < out_w) & (v2 >= 0) & (v2 < out_h)
    u2, v2 = u2[valid2], v2[valid2]
    ranges2 = ranges[valid2]
    intensities2 = intensities[valid2]
    point_indices2 = point_indices[valid2]

    rmin, rmax = ranges2.min(), ranges2.max()
    range_norm = np.clip((1.0 - (ranges2 - rmin) / max(rmax - rmin, 1e-6)) * 255, 0, 255).astype(np.uint8)
    int_vals = np.clip(intensities2 * 255, 0, 255).astype(np.uint8)
    if int_vals.max() > int_vals.min():
        lo, hi = np.percentile(int_vals, 5), np.percentile(int_vals, 95)
        int_vals = np.clip((int_vals.astype(np.float32) - lo) / max(hi - lo, 1e-6) * 200, 0, 255).astype(np.uint8)
    blended_vals = ((0.5 * range_norm + 0.5 * int_vals)).astype(np.uint8)

    blended = np.zeros((out_h, out_w), dtype=np.uint8)
    blended[v2, u2] = blended_vals
    range_image[v2, u2] = ranges2
    indices_image[v2, u2] = point_indices2

    # Dual ERP (2560 wide) has content across the full image — don't zero the middle strip
    is_dual = (width == 2560)
    mid_start, mid_end = out_w // 4, out_w * 3 // 4
    if not is_dual:
        blended[:, mid_start:mid_end] = 0

    # 1. Dilate sparse points slightly so inpainting has more signal to propagate from
    pt_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    blended_thick = cv2.dilate(blended, pt_kernel)
    if not is_dual:
        blended_thick[:, mid_start:mid_end] = 0

    # 2. Inpaint remaining holes
    hole_mask = (blended_thick == 0).astype(np.uint8)
    if not is_dual:
        hole_mask[:, mid_start:mid_end] = 0
    filled = cv2.inpaint(blended_thick, hole_mask, inpaintRadius=12, flags=cv2.INPAINT_TELEA)
    if not is_dual:
        filled[:, mid_start:mid_end] = 0

    # 3. Erode slightly to pull back over-dilated edges
    filled = cv2.erode(filled, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    if not is_dual:
        filled[:, mid_start:mid_end] = 0

    # 4. Gaussian blur to smooth transitions
    filled = cv2.GaussianBlur(filled, (7, 7), 1.5)
    if not is_dual:
        filled[:, mid_start:mid_end] = 0

    # 5. CLAHE with higher clip and larger tile for global contrast recovery (not local washout)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    intensity_image = clahe.apply(filled)
    if not is_dual:
        intensity_image[:, mid_start:mid_end] = 0
    intensity_image = np.fliplr(intensity_image)

    if not is_dual:
        indices_image[:, mid_start:mid_end] = -1
    indices_image = np.fliplr(indices_image)

    cv2.imwrite(output_image, intensity_image)

    indices_rgba = np.frombuffer(indices_image.astype(np.int32).tobytes(), dtype=np.uint8).reshape((out_h, out_w, 4))
    cv2.imwrite(point_indices_image, indices_rgba)

    if camera_image is not None:
        cam = cv2.imread(camera_image, cv2.IMREAD_UNCHANGED)
        if cam is not None:
            cam_out = cam if is_dual else remap_to_strips(cam)
            if len(cam_out.shape) == 3:
                cam_gray = cv2.cvtColor(cam_out, cv2.COLOR_BGR2GRAY)
            else:
                cam_gray = cam_out
            # Apply the correct lidar mask PNG for dual or single
            mask_path = Path(__file__).parent.parent / ('lidar_mask_dual.png' if is_dual else 'lidar_mask_single.png')
            lidar_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if lidar_mask is not None:
                lidar_mask = cv2.resize(lidar_mask, (cam_gray.shape[1], cam_gray.shape[0]), interpolation=cv2.INTER_NEAREST)
                cam_gray = cv2.bitwise_and(cam_gray, cam_gray, mask=lidar_mask)
            cv2.imwrite(camera_image, cv2.resize(cam_gray, (out_w, out_h)))

    print(f"✓ Generated {output_image}")
    print(f"✓ Generated {point_indices_image}")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 generate_intensity_images.py <output_directory>")
        sys.exit(1)

    output_dir = Path(sys.argv[1])

    ply_files = sorted(output_dir.glob("*.ply"))
    if not ply_files:
        print(f"No PLY files found in {output_dir}")
        sys.exit(1)

    print(f"\nGenerating intensity images for {len(ply_files)} PLY files...")

    for ply_file in ply_files:
        base_name = ply_file.stem
        output_image = output_dir / f"{base_name}_lidar_intensities.png"
        point_indices_image = output_dir / f"{base_name}_lidar_indices.png"
        generate_intensity_image(str(ply_file), str(output_image), str(point_indices_image), str(output_dir / f"{base_name}.png"))

    print(f"\n✓ Generated {len(ply_files)} intensity images (remapped to matching strips)")
    print("Now you can run SuperGlue:")
    print(f"  cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration")
    print(f"  python3 ./find_matches_superglue.py {output_dir}")
    print(f"Then fix keypoint coordinates:")
    print(f"  python3 ~/atlas_ws/src/atlas-scanner/src/calibration/fix_matches.py {output_dir}")
