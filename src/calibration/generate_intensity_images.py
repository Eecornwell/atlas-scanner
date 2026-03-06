#!/usr/bin/env python3
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

def generate_intensity_image(ply_file, output_image, point_indices_image, camera_image=None, width=1920, height=960):
    points = []
    intensities = []

    with open(ply_file, 'r') as f:
        lines = f.readlines()

    header_end = next(i+1 for i, line in enumerate(lines) if line.strip() == 'end_header')
    has_intensity = any('intensity' in line for line in lines[:header_end])

    for line in lines[header_end:]:
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
        int_vals = np.clip((int_vals.astype(np.float32) - int_vals.min()) / (int_vals.max() - int_vals.min()) * 255, 0, 255).astype(np.uint8)
    blended_vals = ((0.5 * range_norm + 0.5 * int_vals)).astype(np.uint8)

    blended = np.zeros((out_h, out_w), dtype=np.uint8)
    blended[v2, u2] = blended_vals
    range_image[v2, u2] = ranges2
    indices_image[v2, u2] = point_indices2

    mid_start, mid_end = out_w // 4, out_w * 3 // 4
    blended[:, mid_start:mid_end] = 0
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
    intensity_image = clahe.apply(blended)
    intensity_image[:, mid_start:mid_end] = 0
    intensity_image = cv2.dilate(intensity_image, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    intensity_image[:, mid_start:mid_end] = 0
    intensity_image = np.fliplr(intensity_image)

    # Flip indices to match — pixel (u,v) in flipped image = original pixel (W-1-u, v)
    indices_image[:, mid_start:mid_end] = -1
    indices_image = np.fliplr(indices_image)

    # Build coverage mask - erode to avoid SuperPoint detecting the mask boundary as features
    coverage_mask = (intensity_image > 0).astype(np.uint8)
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    coverage_mask = cv2.erode(coverage_mask, erode_kernel)

    lidar_out = remap_lidar_to_strips(intensity_image)
    cv2.imwrite(output_image, lidar_out)

    indices_rgba = np.frombuffer(indices_image.astype(np.int32).tobytes(), dtype=np.uint8).reshape((out_h, out_w, 4))
    cv2.imwrite(point_indices_image, remap_lidar_to_strips(indices_rgba))

    if camera_image is not None:
        cam = cv2.imread(camera_image, cv2.IMREAD_UNCHANGED)
        if cam is not None:
            cam_out = remap_to_strips(cam)
            if len(cam_out.shape) == 3:
                cam_gray = cv2.cvtColor(cam_out, cv2.COLOR_BGR2GRAY)
            else:
                cam_gray = cam_out
            # Zero camera regions where lidar has no coverage so SuperPoint ignores them
            cam_mask = cv2.resize(coverage_mask, (cam_gray.shape[1], cam_gray.shape[0]), interpolation=cv2.INTER_NEAREST)
            cam_gray = cv2.bitwise_and(cam_gray, cam_gray, mask=cam_mask)
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
