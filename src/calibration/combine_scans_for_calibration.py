#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Merges multiple scan session directories into a single calibration dataset under ~/atlas_ws/output. Copies equirectangular images and sensor-frame PLY files into the flat structure expected by direct_visual_lidar_calibration.

import os
import shutil
import json
import numpy as np
from pathlib import Path

_ALLOWED_ROOT   = Path(os.path.expanduser("~/atlas_ws/data")).resolve()
_ALLOWED_OUTPUT = Path(os.path.expanduser("~/atlas_ws/output")).resolve()


def _safe_resolve(p: Path) -> Path:
    """Resolve path and raise ValueError if it escapes the allowed data root."""
    resolved = p.resolve()
    if not resolved.is_relative_to(_ALLOWED_ROOT):
        raise ValueError(f"Path '{resolved}' is outside the allowed root '{_ALLOWED_ROOT}'")
    return resolved


def _safe_output(output_dir: str, *parts) -> Path:
    """Join parts onto output_dir, resolve, and confirm the result stays within
    the allowed output root before returning the Path."""
    base = Path(output_dir).resolve()
    if not base.is_relative_to(_ALLOWED_OUTPUT):
        raise ValueError(f"Output dir '{base}' is outside the allowed output root '{_ALLOWED_OUTPUT}'")
    # Strip leading separators and collapse '..' from each part to prevent
    # joinpath from silently discarding the base on absolute components.
    sanitized = [Path(p).name if Path(p).is_absolute() else Path(os.path.normpath(p)) for p in parts]
    resolved = base.joinpath(*sanitized).resolve()
    if not resolved.is_relative_to(_ALLOWED_OUTPUT):
        raise ValueError(f"Output path '{resolved}' is outside the allowed output root '{_ALLOWED_OUTPUT}'")
    return resolved


def combine_scans_for_calibration(base_dir, output_dir, max_scans=4, cam_index=None):
    """Combine multiple scan sessions into one calibration dataset.
    
    If cam_index is specified, only scans from that camera are included
    (reads .cam_index file in each fusion_scan directory).
    """

    out = _safe_output(output_dir)  # validates output_dir is within _ALLOWED_OUTPUT
    os.makedirs(out / "images", exist_ok=True)
    os.makedirs(out / "points", exist_ok=True)

    base_path = _safe_resolve(Path(base_dir))
    
    # Find all fusion_scan directories, resolving each to guard against symlinks
    # that escape the allowed data root.
    fusion_dirs = []
    for d in sorted(base_path.iterdir()):
        if not (d.is_dir() and d.name.startswith('fusion_scan')):
            continue
        try:
            fusion_dirs.append(_safe_resolve(d))
        except ValueError:
            print(f"  Skipping '{d.name}': resolves outside allowed root")
            continue
    
    # Filter by camera index if specified
    if cam_index is not None:
        import sys as _sys
        _sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from camera_hw import cam_index_for_scan
        # Check if this is a single-camera session (all scans resolve to same slot)
        unique_indices = set()
        for d in fusion_dirs:
            unique_indices.add(cam_index_for_scan(str(d)))
        if len(unique_indices) <= 1:
            # Single-camera session — use all scans regardless of cam_index filter
            print(f"  Single-camera session detected, using all {len(fusion_dirs)} scans")
        else:
            filtered = [d for d in fusion_dirs if cam_index_for_scan(str(d)) == cam_index]
            fusion_dirs = filtered
            print(f"  Filtering for cam_{cam_index}: {len(fusion_dirs)} scans")
    
    timestamps = []
    total_count = 0
    
    for fusion_dir in fusion_dirs:
        def _safe_glob(pattern):
            results = []
            for f in fusion_dir.glob(pattern):
                try:
                    results.append(_safe_resolve(f))
                except ValueError:
                    pass
            return results

        # Find masked equirectangular image - prefer blended version
        equirect_files = _safe_glob("equirect_*.jpg") + _safe_glob("equirect_*.png")

        # Prefer raw dual-fisheye ERP; strip masked variants
        equirect_files = [f for f in equirect_files if '_masked' not in f.name]
        dual_raw = fusion_dir / 'equirect_dual_fisheye_raw.jpg'
        if dual_raw.exists():
            equirect_files = [_safe_resolve(dual_raw)]

        # Prefer masked ERP for SuperGlue (excludes scanner body from feature detection)
        masked = sorted(_safe_glob('equirect_*_masked.png'))
        if masked:
            equirect_files = [masked[-1]]  # use most recent masked variant

        # Dual-fisheye fallback: synthesise ERP from dual_fisheye.jpg
        if not equirect_files:
            dual_src = fusion_dir / 'dual_fisheye.jpg'
            single_files = _safe_glob('fisheye_*.jpg')
            if dual_src.exists() or single_files:
                import sys as _sys
                _sys.path.insert(0, str(Path(__file__).parent.parent / 'capture'))
                from fisheye_to_erp import fisheye_jpg_to_erp
                if dual_src.exists():
                    dst = fusion_dir / 'equirect_dual_fisheye_raw.jpg'
                    if not dst.exists():
                        fisheye_jpg_to_erp(str(_safe_resolve(dual_src)), str(dst))
                    equirect_files = [_safe_resolve(dst)]
                    print(f"  Synthesised ERP from dual fisheye for {fusion_dir.name}")
                elif single_files:
                    src = single_files[0]
                    dst = fusion_dir / f'equirect_{src.stem}.jpg'
                    if not dst.exists():
                        fisheye_jpg_to_erp(str(src), str(dst))
                    equirect_files = [_safe_resolve(dst)]
                    print(f"  Synthesised ERP from single fisheye for {fusion_dir.name}")

        # OAK-1 fallback: use undistorted PNG directly (already in pinhole space)
        if not equirect_files:
            oak1_files = sorted(_safe_glob('oak1_*_undistorted.png'))
            if not oak1_files:
                oak1_files = [f for f in sorted(_safe_glob('oak1_*.png'))
                              if '_undistorted' not in f.name]
            if oak1_files:
                equirect_files = [oak1_files[0]]
                print(f"  Using OAK-1 image for {fusion_dir.name}: {oak1_files[0].name}")

        ply_files = _safe_glob("sensor_lidar*.ply") or _safe_glob("world_lidar.ply")
        
        if total_count >= max_scans:
            break

        if equirect_files and ply_files:
            # Copy and convert image to PNG (handle alpha channel if present)
            src_img = _safe_resolve(equirect_files[0])
            dst_img = str(_safe_output(output_dir, "images", f"{total_count:06d}.png"))
            
            import cv2
            img = cv2.imread(str(src_img), cv2.IMREAD_UNCHANGED)
            if img is not None:
                # Downsample to MATCHER_MAX_W for both the root copy (SuperGlue)
                # and the images/ copy (initial_guess_manual display).
                # The full-res original stays in the session dir; no need to
                # duplicate it here.
                # Use higher resolution for high-res sensors so SuperGlue has
                # enough texture detail to find reliable matches.
                src_h, src_w = img.shape[:2]
                _hw_for_res = ''
                try:
                    import json as _j
                    _sc = _safe_resolve(src_img.parent.parent) / 'session_config.json'
                    if _sc.exists():
                        _hw_for_res = _j.loads(_sc.read_text()).get('camera_hw', '')
                except (OSError, _j.JSONDecodeError, KeyError, ValueError):
                    pass
                # Fallback: detect OAK-1 from image filename
                if not _hw_for_res and src_img.name.startswith('oak1_'):
                    _hw_for_res = 'oak1'
                MATCHER_MAX_W = 1600 if _hw_for_res in ('x5',) else 1024 if _hw_for_res in ('x3',) else 800
                scale = MATCHER_MAX_W / src_w
                MATCHER_W = MATCHER_MAX_W
                MATCHER_H = round(src_h * scale)
                if img.ndim == 3 and img.shape[2] == 4:
                    alpha = img[:, :, 3:4]
                    img_rgb = (img[:, :, :3] * (alpha / 255.0)).astype(img.dtype)
                else:
                    img_rgb = img
                img_small = cv2.resize(img_rgb, (MATCHER_W, MATCHER_H), interpolation=cv2.INTER_AREA)
                # Write downsampled image to both locations the tool reads
                root_img = str(_safe_output(output_dir, f"{total_count:06d}.png"))
                cv2.imwrite(root_img, img_small)
                cv2.imwrite(dst_img, img_small)

                # Copy PLY file
                src_ply = _safe_resolve(ply_files[0])
                dst_ply = str(_safe_output(output_dir, "points", f"{total_count:06d}.ply"))
                shutil.copy2(src_ply, dst_ply)

                # Also copy to root directory
                root_ply = str(_safe_output(output_dir, f"{total_count:06d}.ply"))
                shutil.copy2(src_ply, root_ply)

                # Store source scan_dir and PLY frame so generate_intensity_images knows
                # whether to undo the trajectory pose before projecting
                import json as _json
                is_world_frame = ply_files[0].name == 'world_lidar.ply'
                # Detect camera_hw from session_config.json so generate_intensity_images
                # can load the correct calibration without relying on multi_camera.yaml
                try:
                    _sess_cfg = _safe_resolve(fusion_dir.parent) / 'session_config.json'
                except ValueError:
                    _sess_cfg = Path('/dev/null')
                _cam_hw = ''
                if _sess_cfg.exists():
                    try:
                        _cam_hw = _json.loads(_sess_cfg.read_text()).get('camera_hw', '')
                    except (OSError, _json.JSONDecodeError, KeyError):
                        pass
                with open(_safe_output(output_dir, f"{total_count:06d}_source.json"), 'w') as sf:
                    _json.dump({'scan_dir': str(_safe_resolve(fusion_dir)),
                                'world_frame': is_world_frame,
                                'camera_hw': _cam_hw}, sf)
                
                # Create timestamp entry
                timestamps.append({
                    "image_id": total_count,
                    "timestamp": total_count * 1000000000,
                    "image_file": f"images/{total_count:06d}.png",
                    "points_file": f"points/{total_count:06d}.ply"
                })
                
                print(f"  Added scan {total_count}: {fusion_dir.name}")
                total_count += 1
    
    if total_count == 0:
        print("No valid scans found!")
        return 0
    
    # Detect matcher resolution from root PNG (downsampled for SuperGlue)
    import cv2 as _cv2
    first_root = _cv2.imread(str(_safe_output(output_dir, "000000.png")))
    img_h, img_w = first_root.shape[:2] if first_root is not None else (MATCHER_H, MATCHER_W)

    # Detect camera model from source session
    _is_oak1 = False
    _oak1_intrinsics = None
    _oak1_distortion = None
    _source_jsons = sorted(_safe_output(output_dir).glob('*_source.json'))
    if _source_jsons:
        try:
            _sd = json.loads(_source_jsons[0].read_text())
            _cam_hw = _sd.get('camera_hw', '')
            _scan_p_raw = Path(_sd.get('scan_dir', ''))
            # Fallback 1: oak1_*_undistorted.png in scan dir
            if not _cam_hw:
                try:
                    _sp = _safe_resolve(_scan_p_raw)
                    if list(_sp.glob('oak1_*_undistorted.png')) or list(_sp.glob('oak1_*.png')):
                        _cam_hw = 'oak1'
                except (ValueError, OSError):
                    pass
            # Fallback 2: plumb_bob already in calib.json from a previous run
            if not _cam_hw:
                _cj = _safe_output(output_dir) / 'calib.json'
                if _cj.exists():
                    try:
                        if json.loads(_cj.read_text()).get('camera', {}).get('camera_model') == 'plumb_bob':
                            _cam_hw = 'oak1'
                    except (OSError, ValueError, KeyError):
                        pass
            if _cam_hw == 'oak1':
                _is_oak1 = True
                try:
                    _scan_p = _safe_resolve(_scan_p_raw)
                except ValueError:
                    _scan_p = None
                _ci_p = None
                if _scan_p is not None:
                    _ci_p = _scan_p / 'camera_info.yaml'
                    if not _ci_p.exists():
                        try:
                            _ci_p = _safe_resolve(_scan_p.parent.parent) / 'camera_info.yaml'
                        except ValueError:
                            _ci_p = None
                if _ci_p is not None and _ci_p.exists():
                    import yaml as _yaml
                    _ci = _yaml.safe_load(_ci_p.read_text())
                    _sx = img_w / _ci['width']
                    _sy = img_h / _ci['height']
                    _oak1_intrinsics = [
                        _ci['fx']*_sx, _ci['fy']*_sy,
                        _ci['cx']*_sx, _ci['cy']*_sy
                    ]
                    _oak1_distortion = _ci['D'][:8]
        except (OSError, json.JSONDecodeError, KeyError, _yaml.YAMLError):
            pass

    # Create metadata files
    bag_names = [f"{i:06d}" for i in range(total_count)]

    if _is_oak1 and _oak1_intrinsics:
        calib_data = {
            "camera": {
                "camera_model": "plumb_bob",
                "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
                "intrinsics": _oak1_intrinsics
            },
            "meta": {
                "bag_names": bag_names,
                "camera_info_topic": "/camera/camera_info",
                "data_path": output_dir,
                "image_topic": "/camera/image",
                "intensity_channel": "intensity",
                "points_topic": "/livox/lidar"
            },
            "results": {
                "T_lidar_camera": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                "init_T_lidar_camera": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
            }
        }
        print(f"  OAK-1: using plumb_bob camera model  fx={_oak1_intrinsics[0]:.0f} fy={_oak1_intrinsics[1]:.0f}")
        # Warn if scene depth variation is low — calibration may be degenerate
        # (rotation and translation become indistinguishable at a single depth)
        _depth_range = 0.0
        try:
            import struct as _struct
            _ply0 = Path(output_dir) / '000000.ply'
            if _ply0.exists():
                with open(_ply0, 'rb') as _pf:
                    while _pf.readline().strip() != b'end_header': pass
                    _n0 = int(next(l.split()[-1] for l in open(str(_ply0)).readlines() if l.startswith('element vertex')))
                    _raw = np.frombuffer(_pf.read(_n0*4*4), dtype=np.float32).reshape(_n0,4)
                _ranges = np.linalg.norm(_raw[:,:3], axis=1)
                _depth_range = float(_ranges.max() - _ranges.min())
        except Exception:
            pass
        if _depth_range > 0 and _depth_range < 1.5:
            print(f"  ⚠ OAK-1 calibration WARNING: scene depth range is only {_depth_range:.1f}m.")
            print(f"    Calibration may be degenerate (rotation/translation indistinguishable).")
            print(f"    For accurate calibration, include objects at 0.5m AND 3m+ in the same frame.")
    else:
        calib_data = {
            "camera": {
                "camera_model": "equirectangular",
                "distortion_coeffs": [],
                "intrinsics": [img_w, img_h]
            },
            "meta": {
                "bag_names": bag_names,
                "camera_info_topic": "/camera/camera_info",
                "data_path": output_dir,
                "image_topic": "/equirectangular/image",
                "intensity_channel": "intensity",
                "points_topic": "/livox/lidar"
            },
            "results": {
                "T_lidar_camera": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                "init_T_lidar_camera": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
            }
        }
    
    with open(_safe_output(output_dir, "calib.json"), 'w') as f:
        json.dump(calib_data, f, indent=2)

    # Create preprocessing metadata
    if _is_oak1 and _oak1_intrinsics:
        metadata = {
            "camera_model": "plumb_bob",
            "image_size": [img_w, img_h],
            "camera_intrinsics": _oak1_intrinsics,
            "camera_distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
            "image_topic": "/camera/image",
            "points_topic": "/livox/lidar",
            "intensity_channel": "intensity",
            "num_images": total_count,
            "num_points": total_count,
            "timestamps": timestamps
        }
    else:
        metadata = {
            "camera_model": "equirectangular",
            "image_size": [img_w, img_h],
            "camera_intrinsics": [img_w, img_h, img_w, img_h],
            "camera_distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            "image_topic": "/equirectangular/image",
            "points_topic": "/livox/lidar",
            "intensity_channel": "intensity",
            "num_images": total_count,
            "num_points": total_count,
            "timestamps": timestamps
        }

    with open(_safe_output(output_dir, "preprocessing_result.json"), 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"\n✓ Combined calibration dataset created!")
    print(f"Total images: {total_count}")
    print(f"Output: {output_dir}")
    
    return total_count

if __name__ == '__main__':
    import sys
    import argparse

    parser = argparse.ArgumentParser(description='Combine scans into calibration dataset')
    parser.add_argument('session_directory', help='Path to session directory')
    parser.add_argument('max_scans', nargs='?', type=int, default=10, help='Max scans to include')
    parser.add_argument('--cam-index', type=int, default=None,
                        help='Only include scans from this camera index (reads .cam_index files)')
    args = parser.parse_args()

    base_dir = args.session_directory
    output_dir = str(Path(os.path.expanduser("~/atlas_ws/output")).resolve())

    if not os.path.exists(base_dir):
        print(f"Error: Input directory {base_dir} does not exist")
        sys.exit(1)

    try:
        base_dir = str(_safe_resolve(Path(base_dir)))
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    count = combine_scans_for_calibration(base_dir, output_dir, args.max_scans, cam_index=args.cam_index)
    
    if count > 1:
        print(f"\n✓ Ready for calibration with {count} images!")
        print("\nNext steps:")
        print("  cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration")
        print("  python3 ./find_matches_superglue.py ~/atlas_ws/output")
        print("  ./initial_guess_manual --data_path ~/atlas_ws/output")
        print("  ./calibrate --data_path ~/atlas_ws/output")
    else:
        print("Need more scan data for calibration")
