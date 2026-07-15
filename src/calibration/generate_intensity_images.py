#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Projects sensor-frame PLY point clouds onto a 2D intensity image for use as LiDAR input to the SuperGlue feature matcher during camera-LiDAR calibration.
import numpy as np
import cv2
import os
import sys
import yaml
import json
from pathlib import Path
from scipy.spatial.transform import Rotation

_ALLOWED_OUTPUT = Path(os.path.expanduser("~/atlas_ws/output")).resolve()
_ALLOWED_DATA   = Path(os.path.expanduser("~/atlas_ws/data")).resolve()


def _safe_output(p) -> Path:
    """Resolve p and raise ValueError if it escapes the allowed output root."""
    resolved = Path(p).resolve()
    if _ALLOWED_OUTPUT not in [resolved, *resolved.parents]:
        raise ValueError(
            f"Path '{resolved}' is outside the allowed output root '{_ALLOWED_OUTPUT}'"
        )
    return resolved


def _safe_data(p) -> Path:
    """Resolve p and raise ValueError if it escapes the allowed data root."""
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(
            f"Path '{resolved}' is outside the allowed data root '{_ALLOWED_DATA}'"
        )
    return resolved

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
    """Load T_camera_lidar from the appropriate calibration file.
    Prefers per-hw calibration detected from output dataset, falls back to active."""
    import glob as _g
    calib_path = Path(__file__).parent.parent / 'config' / 'fusion_calibration.yaml'
    # Try to detect camera_hw from the output dataset
    _source_jsons = sorted(_g.glob(str(Path.home() / 'atlas_ws/output/*_source.json')))
    if _source_jsons:
        try:
            import json as _j
            _src = _j.loads(Path(_source_jsons[0]).read_text())
            _sess_cfg = Path(_src.get('scan_dir', '')) / '..' / 'session_config.json'
            if _sess_cfg.exists():
                _hw = _j.loads(_sess_cfg.read_text()).get('camera_hw', '')
                _hw_path = Path(__file__).parent.parent / 'config' / 'calibrations' / _hw / 'fusion_calibration.yaml'
                if _hw_path.exists():
                    calib_path = _hw_path
        except Exception:
            pass
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
    try:
        safe_dir = _safe_data(scan_dir)
    except ValueError:
        return None, None
    traj_file = safe_dir / 'trajectory.json'
    if not traj_file.exists():
        return None, None
    try:
        traj_file = _safe_data(traj_file)
    except ValueError:
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
    # Auto-detect dimensions from the root PNG (matcher resolution, 640x320).
    # images/ holds the full-res copy; the root PNG is what SuperGlue actually reads.
    if camera_image is not None:
        try:
            probe_path = _safe_output(camera_image)
        except ValueError:
            probe_path = None
        if probe_path is not None:
            cam_probe = cv2.imread(str(probe_path))
            if cam_probe is not None:
                height, width = cam_probe.shape[:2]
    points = []
    intensities = []

    try:
        safe_ply = _safe_output(ply_file)
    except ValueError as e:
        print(f"Rejected PLY path: {e}")
        return False
    with open(safe_ply, 'rb') as f:
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
    try:
        _safe_scan_dir = _safe_data(scan_dir) if scan_dir else None
    except ValueError:
        _safe_scan_dir = None
    use_camframe = _safe_scan_dir.is_dir() if _safe_scan_dir else False
    is_sdk_stitch = bool(_safe_scan_dir and list(_safe_scan_dir.glob('*.insp')))
    T_cam_lidar = _load_T_cam_lidar()
    # SDK stitch: full ERP, project at full resolution. Manual: halve to match strip content.
    out_w, out_h = (width, height) if is_sdk_stitch else (width // 2, height // 2)

    if use_camframe:
        # Read world_frame flag from sidecar to decide whether to undo trajectory pose.
        # sensor_lidar.ply is already in sensor frame; world_lidar.ply needs trajectory undo.
        import json as _json
        is_world_frame = False  # default: sensor_lidar.ply (sensor frame)
        if _safe_scan_dir:
            for sf in sorted(_safe_scan_dir.parent.parent.glob('*_source.json')):
                try:
                    sf = _safe_output(sf)
                    with open(sf) as _f:
                        _sd = _json.load(_f)
                    if _sd.get('scan_dir') == scan_dir:
                        is_world_frame = _sd.get('world_frame', False)
                        break
                except: pass
        origin, R_world_lidar = _load_trajectory_pose(_safe_scan_dir)
        if is_sdk_stitch:
            # SDK stitch: project lidar into camera ERP frame using the
            # current seed calibration. This coarsely aligns the intensity
            # image with the camera ERP so SuperGlue can find matches.
            # The compose_initial_guess.py step then accounts for the
            # pre-applied rotation when computing the final T_lidar_camera.
            if origin is not None and R_world_lidar is not None and is_world_frame:
                pts_lidar = (R_world_lidar.T @ (points - origin).T).T
            else:
                pts_lidar = points
            # Apply T_camera_lidar from current calibration
            T_cl = _load_T_cam_lidar()
            R_cl = T_cl[:3, :3]
            pts_cam = (R_cl @ pts_lidar.T).T
            norm_c = np.sqrt((pts_cam**2).sum(axis=1)).clip(1e-6)
            bearing = pts_cam / norm_c[:, None]
            orig_indices = np.arange(len(pts_cam))
            # Equirectangular projection matching equirectangular.hpp:
            #   lat = -asin(bearing[1]), lon = atan2(bearing[0], bearing[2])
            #   u = W*(0.5 + lon/(2*pi)), v = H*(0.5 - lat/pi)
            lat2 = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
            lon2 = np.arctan2(bearing[:, 0], bearing[:, 2])
            u = (out_w * (0.5 + lon2 / (2 * np.pi))).astype(int) % out_w
            v = (out_h * (0.5 - lat2 / np.pi)).astype(int)
            v = np.clip(v, 0, out_h - 1)
        else:
            if origin is not None and R_world_lidar is not None and is_world_frame:
                # world_lidar.ply: undo full pose (rotation + translation) back to sensor frame
                pts_lidar = (R_world_lidar.T @ (points - origin).T).T
            else:
                pts_lidar = points
            pts_h   = np.hstack([pts_lidar, np.ones((len(pts_lidar), 1))])
            pts_cam = (T_cam_lidar @ pts_h.T).T[:, :3]
            norm_c = np.sqrt((pts_cam**2).sum(axis=1)).clip(1e-6)
            Xc = pts_cam[:, 0] / norm_c
            Yc = pts_cam[:, 1] / norm_c
            Zc = pts_cam[:, 2] / norm_c
            # Manual dual fisheye: content only in back hemisphere (Zc < 0)
            back = Zc < 0
            Xb =  Xc[back]
            Yb = -Yc[back]
            Zb =  Zc[back]
            intensities  = intensities[back]
            orig_indices = np.where(back)[0]
            norm_c       = norm_c[back]
            lon = np.arctan2(Xb, Zb)
            lat = np.arcsin(np.clip(Yb, -1, 1))
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

    if len(intensities) == 0:
        print(f"No valid points after projection for {ply_file}")
        return False

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
    suppress_middle = not is_dual or is_sdk_stitch
    mid_start, mid_end = out_w // 4, out_w * 3 // 4
    BOUNDARY_MARGIN = 12
    # Expand the suppressed zone by the boundary margin before inpainting
    # so inpainted content never reaches the final boundary edge
    mid_start_inpaint = max(0, mid_start - BOUNDARY_MARGIN)
    mid_end_inpaint   = min(out_w, mid_end + BOUNDARY_MARGIN)
    if suppress_middle:
        blended[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 1. Dilate sparse points to connect adjacent scan lines
    pt_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    blended_thick = cv2.dilate(blended, pt_kernel)
    if suppress_middle:
        blended_thick[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 2. Inpaint holes
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

    # 4. Mild blur to smooth inpainting transitions
    filled = cv2.GaussianBlur(filled, (5, 5), 1.0)
    if suppress_middle:
        filled[:, mid_start_inpaint:mid_end_inpaint] = 0

    # 5. CLAHE for contrast enhancement
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    intensity_image = clahe.apply(filled)
    if suppress_middle:
        intensity_image[:, mid_start_inpaint:mid_end_inpaint] = 0
        indices_image[:, mid_start_inpaint:mid_end_inpaint] = -1

    # Build a coverage mask from the actual projected points so that inpainted
    # regions outside the lidar FOV are zeroed in both the lidar intensity image
    # and the camera image before SuperGlue runs. This prevents keypoints from
    # being detected in blurry/fake inpainted areas that have no real lidar data.
    coverage_mask = None
    feather = None

    if coverage_mask is not None:
        intensity_image = (intensity_image.astype(np.float32) * feather).clip(0, 255).astype(np.uint8)
        indices_image[coverage_mask == 0] = -1

    # Process camera image first to determine valid row range for blind-spot filtering
    cam_valid_row_limit = out_h  # default: all rows valid
    if camera_image is not None:
        src_erp = None
        if _safe_scan_dir:
            scan_path = _safe_scan_dir
            candidates = sorted(scan_path.glob('equirect_*_masked.png')) or \
                         [f for f in sorted(scan_path.glob('equirect_*.jpg')) if '_masked' not in f.name]
            if candidates:
                src_erp = str(candidates[0])
        cam_src = src_erp or camera_image
        cam = cv2.imread(cam_src, cv2.IMREAD_UNCHANGED)
        if cam is not None:
            cam_out = cam if is_dual else remap_to_strips(cam)
            cam_gray = cv2.cvtColor(cam_out, cv2.COLOR_BGR2GRAY) if cam_out.ndim == 3 else cam_out
            if not is_dual:
                mask_path_src = 'lidar_mask_dual.png' if (is_sdk_stitch or width > 2560) else 'lidar_mask_single.png'
                mask_path = Path(__file__).parent.parent / mask_path_src
                lidar_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
                if lidar_mask is not None:
                    lidar_mask = cv2.resize(lidar_mask, (cam_gray.shape[1], cam_gray.shape[0]), interpolation=cv2.INTER_NEAREST)
                    cam_gray = cv2.bitwise_and(cam_gray, cam_gray, mask=lidar_mask)
            cam_gray = cv2.resize(cam_gray, (out_w, out_h), interpolation=cv2.INTER_AREA)
            # Zero middle zone for non-dual (front lens has no lidar overlap)
            if not is_dual:
                mid_s_lr = out_w // 4
                mid_e_lr = out_w * 3 // 4
                cam_gray[:, max(0, mid_s_lr - BOUNDARY_MARGIN):min(out_w, mid_e_lr + BOUNDARY_MARGIN)] = 0
            # Find last valid row then zero from (limit - margin) downward
            cam_valid_rows = (cam_gray > 5).any(axis=1)
            if not cam_valid_rows.all():
                cam_valid_row_limit = int(np.argmin(cam_valid_rows))
            cam_gray[max(0, cam_valid_row_limit - BOUNDARY_MARGIN):, :] = 0
            if feather is not None:
                cam_gray = (cam_gray.astype(np.float32) * feather).clip(0, 255).astype(np.uint8)
            # Write processed camera gray to a sidecar — never overwrite the source RGB
            try:
                cam_gray_path = str(_safe_output(
                    Path(camera_image).with_name(
                        Path(camera_image).stem + '_camera_gray.png')))
            except ValueError:
                cam_gray_path = None
            if cam_gray_path:
                cv2.imwrite(cam_gray_path, cam_gray)
            # Overwrite the root PNG SuperGlue reads with the masked grayscale
            if feather is not None:
                try:
                    cv2.imwrite(str(_safe_output(camera_image)), cam_gray)
                except ValueError:
                    pass

    # Zero lidar rows outside camera coverage (with same margin)
    intensity_image[max(0, cam_valid_row_limit - BOUNDARY_MARGIN):, :] = 0
    indices_image[max(0, cam_valid_row_limit - BOUNDARY_MARGIN):, :] = -1

    cv2.imwrite(str(_safe_output(output_image)), intensity_image)

    # The indices image pixel (u,v) must map to the same 3D point as intensity pixel (u,v).
    indices_rgba = np.frombuffer(indices_image.astype(np.int32).tobytes(), dtype=np.uint8).reshape((out_h, out_w, 4))
    cv2.imwrite(str(_safe_output(point_indices_image)), indices_rgba)

    print(f"✓ Generated {output_image}")
    print(f"✓ Generated {point_indices_image}")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 generate_intensity_images.py <output_directory>")
        sys.exit(1)

    try:
        output_dir = _safe_output(sys.argv[1])
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    ply_files = sorted(output_dir.glob("*.ply"))
    if not ply_files:
        print(f"No PLY files found in {output_dir}")
        sys.exit(1)

    print(f"\nGenerating intensity images for {len(ply_files)} PLY files...")

    for ply_file in ply_files:
        base_name = ply_file.stem
        output_image = str(_safe_output(output_dir / f"{base_name}_lidar_intensities.png"))
        point_indices_image = str(_safe_output(output_dir / f"{base_name}_lidar_indices.png"))
        # Read source scan_dir from sidecar written by combine_scans_for_calibration.py
        import json as _json
        sidecar = _safe_output(output_dir / f"{base_name}_source.json")
        src_scan_dir = None
        if sidecar.exists():
            with open(sidecar) as sf:
                src_scan_dir = _json.load(sf).get('scan_dir')
        generate_intensity_image(str(_safe_output(ply_file)), output_image, point_indices_image,
                                 str(_safe_output(output_dir / f"{base_name}.png")), scan_dir=src_scan_dir)

    print(f"\n✓ Generated {len(ply_files)} intensity images (remapped to matching strips)")
    print("Now you can run SuperGlue:")
    print(f"  cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration")
    print(f"  python3 ./find_matches_superglue.py {output_dir} --superglue indoor")
    print(f"Then fix keypoint coordinates:")
    print(f"  python3 ~/atlas_ws/src/atlas-scanner/src/calibration/fix_matches.py {output_dir}")
