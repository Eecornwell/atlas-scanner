#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Projects sensor-frame PLY point clouds onto a 2D intensity image for use as LiDAR input to the SuperGlue feature matcher during camera-LiDAR calibration.
import numpy as np
import cv2
import sys
import yaml
import json
from pathlib import Path
from scipy.spatial.transform import Rotation

CAM_LEFT = (0, 960)      # camera content left strip (cols)
CAM_RIGHT = (2880, 3840) # camera content right strip (cols)

def remap_to_strips(image):
    """Zero out the black middle zone in the camera image."""
    out = image.copy()
    h, w = out.shape[:2]
    mid_start = w // 4
    mid_end = w * 3 // 4
    out[:, mid_start:mid_end] = 0
    return out

def remap_lidar_to_strips(image):
    """Lidar covers full ERP - no transformation needed."""
    return image

def _load_T_cam_lidar():
    """Load T_camera_lidar from fusion_calibration.yaml."""
    calib_path = Path(__file__).parent.parent / 'config' / 'fusion_calibration.yaml'
    with open(calib_path) as f:
        calib = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [
        calib['roll_offset'], calib['pitch_offset'], calib['yaw_offset']
    ]).as_matrix()
    T[:3, 3] = [calib['x_offset'], calib['y_offset'], calib['z_offset']]
    return T

def _load_trajectory_pose(scan_dir):
    """Return (origin, R_world_lidar) for a scan directory, or None if unavailable."""
    traj_file = Path(scan_dir) / 'trajectory.json'
    if not traj_file.exists():
        return None, None
    with open(traj_file) as f:
        traj = json.load(f)
    lp = traj.get('current_pose', {}).get('lidar_pose', {})
    pos = lp.get('position', {})
    ori = lp.get('orientation', {})
    origin = np.array([pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)])
    R = Rotation.from_quat([
        ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)
    ]).as_matrix()
    return origin, R

def generate_intensity_image(ply_file, output_image, point_indices_image, camera_image=None, width=3840, height=1920, scan_dir=None):
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

    # Project lidar points into the camera ERP frame so the intensity image
    # always aligns with the gravity-stabilised camera ERP regardless of scanner yaw.
    #
    # Pipeline:
    #   world_lidar -> sensor_lidar (undo trajectory pose)
    #   sensor_lidar -> camera frame (apply T_camera_lidar)
    #   camera frame -> ERP (lon=arctan2(x,z), lat=arcsin(-y/r))
    #
    # Falls back to the original sensor-frame fixed-yaw projection if no
    # trajectory is available (e.g. standalone PLY without a scan directory).
    use_camframe = Path(scan_dir).is_dir() if scan_dir else False
    T_cam_lidar = _load_T_cam_lidar()
    out_w, out_h = width // 2, height // 2

    if use_camframe:
        # Read world_frame flag from sidecar to decide whether to undo trajectory pose.
        # sensor_lidar.ply is already in sensor frame; world_lidar.ply needs trajectory undo.
        import json as _json
        is_world_frame = False  # default: sensor_lidar.ply (sensor frame)
        if scan_dir:
            for sf in sorted(Path(scan_dir).parent.parent.glob('*_source.json')):
                try:
                    with open(sf) as _f:
                        _sd = _json.load(_f)
                    if _sd.get('scan_dir') == scan_dir:
                        is_world_frame = _sd.get('world_frame', False)
                        break
                except: pass
        origin, R_world_lidar = _load_trajectory_pose(scan_dir)
        if is_world_frame and origin is not None and R_world_lidar is not None:
            pts_lidar = (R_world_lidar.T @ (points - origin).T).T
        else:
            pts_lidar = points  # sensor frame: use directly
        pts_h   = np.hstack([pts_lidar, np.ones((len(pts_lidar), 1))])
        pts_cam = (T_cam_lidar @ pts_h.T).T[:, :3]
        norm_c = np.sqrt((pts_cam**2).sum(axis=1)).clip(1e-6)
        # Project using the same ERP convention as fisheye_to_erp.py:
        #   X = cos(lat)*sin(lon),  Y = sin(lat),  Z = cos(lat)*cos(lon)
        #   back hemisphere Z < 0 -> outer strips (single back fisheye)
        Xc = pts_cam[:, 0] / norm_c
        Yc = pts_cam[:, 1] / norm_c
        Zc = pts_cam[:, 2] / norm_c
        back = Zc < 0
        # Mirror left-right to rigidly align lidar ERP with camera ERP
        Xb =  Xc[back]
        Yb = -Yc[back]
        Zb =  Zc[back]
        intensities  = intensities[back]
        orig_indices = np.where(back)[0]
        norm_c       = norm_c[back]
        lon = np.arctan2(Xb, Zb)
        lat = np.arcsin(np.clip(Yb, -1, 1))   # visual projection (matches camera ERP orientation)
        u = (out_w * (0.5 + lon / (2 * np.pi))).astype(int) % out_w
        v = (out_h * (0.5 - lat / np.pi)).astype(int)
    else:
        # Fallback: sensor frame with fixed 90-deg yaw pre-rotation
        r = np.pi / 2
        x, y = x*np.cos(r) - y*np.sin(r), x*np.sin(r) + y*np.cos(r)
        norm = np.sqrt(x**2 + y**2 + z**2).clip(1e-6)
        lat = np.arcsin(np.clip(y / norm, -1, 1))
        lon = np.arctan2(z, -x)
        u = ((out_w * (0.5 + lon / (2 * np.pi))).astype(int) + out_w // 4) % out_w
        v = (out_h * (0.5 - lat / np.pi)).astype(int)

    valid = (u >= 0) & (u < out_w) & (v >= 0) & (v < out_h)
    if use_camframe:
        ranges = norm_c[valid]
        point_indices = orig_indices[valid]
    else:
        ranges = norm[valid]
        point_indices = np.where(valid)[0]
    u, v = u[valid], v[valid]
    intensities = intensities[valid]

    range_image = np.zeros((out_h, out_w), dtype=np.float32)
    indices_image = np.full((out_h, out_w), -1, dtype=np.int32)

    if intensities.max() > 1.0:
        intensities = intensities / 255.0

    valid2 = (u >= 0) & (u < out_w) & (v >= 0) & (v < out_h)
    u2, v2 = u[valid2], v[valid2]
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

    # Dual ERP (2560 wide) has content across the full image.
    # For both sensor-frame and camera-frame paths the lidar projects into the
    # outer strips (|lon|>90deg); zero the middle zone to match the camera ERP.
    is_dual = (width == 2560)
    suppress_middle = not is_dual
    mid_start, mid_end = out_w // 4, out_w * 3 // 4
    BOUNDARY_MARGIN = 12
    # Expand the suppressed zone by the boundary margin before inpainting
    # so inpainted content never reaches the final boundary edge
    mid_start_inpaint = max(0, mid_start - BOUNDARY_MARGIN)
    mid_end_inpaint   = min(out_w, mid_end + BOUNDARY_MARGIN)
    if suppress_middle:
        blended[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 1. Dilate sparse points slightly so inpainting has more signal to propagate from
    pt_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    blended_thick = cv2.dilate(blended, pt_kernel)
    if suppress_middle:
        blended_thick[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 2. Inpaint remaining holes
    hole_mask = (blended_thick == 0).astype(np.uint8)
    if suppress_middle:
        hole_mask[:, mid_start_inpaint:mid_end_inpaint] = 0
    filled = cv2.inpaint(blended_thick, hole_mask, inpaintRadius=12, flags=cv2.INPAINT_TELEA)
    if suppress_middle:
        filled[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 3. Erode slightly to pull back over-dilated edges
    filled = cv2.erode(filled, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    if suppress_middle:
        filled[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 4. Gaussian blur to smooth transitions
    filled = cv2.GaussianBlur(filled, (7, 7), 1.5)
    if suppress_middle:
        filled[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 5. CLAHE with higher clip and larger tile for global contrast recovery (not local washout)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    intensity_image = clahe.apply(filled)
    if suppress_middle:
        intensity_image[:, mid_start_inpaint:mid_end_inpaint] = 0
        indices_image[:, mid_start_inpaint:mid_end_inpaint] = -1

    # Process camera image first to determine valid row range for blind-spot filtering
    cam_valid_row_limit = out_h  # default: all rows valid
    if camera_image is not None:
        src_erp = None
        if scan_dir:
            from pathlib import Path as _P
            scan_path = _P(scan_dir)
            candidates = sorted(scan_path.glob('equirect_*_masked.png')) or \
                         [f for f in sorted(scan_path.glob('equirect_*.jpg')) if '_masked' not in f.name]
            if candidates:
                src_erp = str(candidates[0])
        cam_src = src_erp or camera_image
        cam = cv2.imread(cam_src, cv2.IMREAD_UNCHANGED)
        if cam is not None:
            cam_out = cam if is_dual else remap_to_strips(cam)
            cam_gray = cv2.cvtColor(cam_out, cv2.COLOR_BGR2GRAY) if cam_out.ndim == 3 else cam_out
            mask_path = Path(__file__).parent.parent / ('lidar_mask_dual.png' if is_dual else 'lidar_mask_single.png')
            lidar_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if lidar_mask is not None:
                lidar_mask = cv2.resize(lidar_mask, (cam_gray.shape[1], cam_gray.shape[0]), interpolation=cv2.INTER_NEAREST)
                cam_gray = cv2.bitwise_and(cam_gray, cam_gray, mask=lidar_mask)
            cam_gray = cv2.resize(cam_gray, (out_w, out_h), interpolation=cv2.INTER_AREA)
            # Zero margin around all boundaries
            mid_s_lr = out_w // 4
            mid_e_lr = out_w * 3 // 4
            cam_gray[:, max(0, mid_s_lr - BOUNDARY_MARGIN):min(out_w, mid_e_lr + BOUNDARY_MARGIN)] = 0
            # Find last valid row then zero from (limit - margin) downward
            cam_valid_rows = (cam_gray > 5).any(axis=1)
            if not cam_valid_rows.all():
                cam_valid_row_limit = int(np.argmin(cam_valid_rows))
            cam_gray[max(0, cam_valid_row_limit - BOUNDARY_MARGIN):, :] = 0
            cv2.imwrite(camera_image, cam_gray)

    # Zero lidar rows outside camera coverage (with same margin)
    intensity_image[max(0, cam_valid_row_limit - BOUNDARY_MARGIN):, :] = 0
    indices_image[max(0, cam_valid_row_limit - BOUNDARY_MARGIN):, :] = -1

    cv2.imwrite(output_image, intensity_image)

    # The indices image pixel (u,v) must map to the same 3D point as intensity pixel (u,v).
    # No flip needed - the projection already places indices at the correct pixels.
    indices_rgba = np.frombuffer(indices_image.astype(np.int32).tobytes(), dtype=np.uint8).reshape((out_h, out_w, 4))
    cv2.imwrite(point_indices_image, indices_rgba)

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
        # Read source scan_dir from sidecar written by combine_scans_for_calibration.py
        import json as _json
        sidecar = output_dir / f"{base_name}_source.json"
        src_scan_dir = None
        if sidecar.exists():
            with open(sidecar) as sf:
                src_scan_dir = _json.load(sf).get('scan_dir')
        generate_intensity_image(str(ply_file), str(output_image), str(point_indices_image),
                                 str(output_dir / f"{base_name}.png"), scan_dir=src_scan_dir)

    print(f"\n✓ Generated {len(ply_files)} intensity images (remapped to matching strips)")
    print("Now you can run SuperGlue:")
    print(f"  cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration")
    print(f"  python3 ./find_matches_superglue.py {output_dir} --superglue indoor")
    print(f"Then fix keypoint coordinates:")
    print(f"  python3 ~/atlas_ws/src/atlas-scanner/src/calibration/fix_matches.py {output_dir}")
