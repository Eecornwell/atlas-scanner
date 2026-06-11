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
    if _ALLOWED_ROOT not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside the allowed root '{_ALLOWED_ROOT}'")
    return resolved


def _safe_output(output_dir: str, *parts) -> Path:
    """Join parts onto output_dir, resolve, and confirm the result stays within
    the allowed output root before returning the Path."""
    resolved = Path(output_dir).joinpath(*parts).resolve()
    if _ALLOWED_OUTPUT not in [resolved, *resolved.parents]:
        raise ValueError(f"Output path '{resolved}' is outside the allowed output root '{_ALLOWED_OUTPUT}'")
    return resolved


def combine_scans_for_calibration(base_dir, output_dir, max_scans=4):
    """Combine multiple scan sessions into one calibration dataset"""

    out = _safe_output(output_dir)  # validates output_dir is within _ALLOWED_OUTPUT
    os.makedirs(out / "images", exist_ok=True)
    os.makedirs(out / "points", exist_ok=True)

    base_path = _safe_resolve(Path(base_dir))
    
    # Find all fusion_scan directories
    fusion_dirs = sorted([d for d in base_path.iterdir() if d.is_dir() and d.name.startswith('fusion_scan')])
    
    timestamps = []
    total_count = 0
    
    for fusion_dir in fusion_dirs:
        # Find masked equirectangular image - prefer blended version
        masked_files = list(fusion_dir.glob("equirect_*_masked.png"))
        masked_raw_files = list(fusion_dir.glob("equirect_*_masked_raw.png"))
        equirect_files = list(fusion_dir.glob("equirect_*.jpg")) + list(fusion_dir.glob("equirect_*.png"))

        # Prefer raw dual-fisheye ERP; strip masked variants
        equirect_files = [f for f in equirect_files if '_masked' not in f.name]
        dual_raw = fusion_dir / 'equirect_dual_fisheye_raw.jpg'
        if dual_raw.exists():
            equirect_files = [dual_raw]

        # Prefer masked ERP for SuperGlue (excludes scanner body from feature detection)
        masked = sorted(fusion_dir.glob('equirect_*_masked.png'))
        if masked:
            equirect_files = [masked[-1]]  # use most recent masked variant

        # Dual-fisheye fallback: synthesise ERP from dual_fisheye.jpg
        if not equirect_files:
            import sys as _sys
            _sys.path.insert(0, str(Path(__file__).parent.parent / 'post_processing'))
            from fisheye_to_erp import fisheye_jpg_to_erp
            dual_src = fusion_dir / 'dual_fisheye.jpg'
            single_files = list(fusion_dir.glob('fisheye_*.jpg'))
            if dual_src.exists():
                erp_cfg = os.path.expanduser('~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml')
                dst = fusion_dir / 'equirect_dual_fisheye_raw.jpg'
                if not dst.exists():
                    fisheye_jpg_to_erp(str(dual_src), erp_cfg, str(dst), dual=True)
                equirect_files = [dst]
                print(f"  Synthesised ERP from dual fisheye for {fusion_dir.name}")
            elif single_files:
                erp_cfg = os.path.expanduser('~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml')
                src = single_files[0]
                dst = fusion_dir / f'equirect_{src.stem}.jpg'
                if not dst.exists():
                    fisheye_jpg_to_erp(str(src), erp_cfg, str(dst))
                equirect_files = [dst]
                print(f"  Synthesised ERP from single fisheye for {fusion_dir.name}")
        
        ply_files = list(fusion_dir.glob("sensor_lidar*.ply")) or list(fusion_dir.glob("world_lidar.ply"))
        
        if total_count >= max_scans:
            break

        if equirect_files and ply_files:
            # Copy and convert image to PNG (handle alpha channel if present)
                src_img = _safe_resolve(equirect_files[0])
            dst_img = str(_safe_output(output_dir, "images", f"{total_count:06d}.png"))
            
            import cv2
            # Read with alpha channel if present
            img = cv2.imread(str(src_img), cv2.IMREAD_UNCHANGED)
            if img is not None:
                # Full-res copy for post-processing reference
                cv2.imwrite(dst_img, img)

                # Downsample root copy for SuperGlue/SuperPoint — the matcher was
                # trained on ~640px images and produces very few keypoints on
                # 3840x1920 inputs. Scale to MATCHER_MAX_W preserving exact aspect
                # ratio so calib.json intrinsics are derived from actual dimensions.
                MATCHER_MAX_W = 2560
                src_h, src_w = img.shape[:2]
                scale = MATCHER_MAX_W / src_w
                MATCHER_W = MATCHER_MAX_W
                MATCHER_H = round(src_h * scale)
                # Apply alpha mask to RGB before downsampling so masked regions
                # (scanner body/tripod) are black and ignored by SuperPoint.
                if img.ndim == 3 and img.shape[2] == 4:
                    alpha = img[:, :, 3:4]
                    img_rgb = (img[:, :, :3] * (alpha / 255.0)).astype(img.dtype)
                else:
                    img_rgb = img
                img_small = cv2.resize(img_rgb, (MATCHER_W, MATCHER_H), interpolation=cv2.INTER_AREA)
                root_img = str(_safe_output(output_dir, f"{total_count:06d}.png"))
                cv2.imwrite(root_img, img_small)

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
                with open(_safe_output(output_dir, f"{total_count:06d}_source.json"), 'w') as sf:
                    _json.dump({'scan_dir': str(_safe_resolve(fusion_dir)), 'world_frame': is_world_frame}, sf)
                
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

    # Create metadata files
    bag_names = [f"{i:06d}" for i in range(total_count)]

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
    
    if len(sys.argv) < 2:
        print("Usage: python3 combine_scans_for_calibration.py <session_directory>")
        print("Example: python3 combine_scans_for_calibration.py /home/orion/atlas_ws/data/synchronized_scans/sync_fusion_20260121_170652")
        sys.exit(1)

    base_dir = sys.argv[1]
    output_dir = "/home/orion/atlas_ws/output"

    if not os.path.exists(base_dir):
        print(f"Error: Input directory {base_dir} does not exist")
        sys.exit(1)

    try:
        _safe_resolve(Path(base_dir))
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    if os.path.exists(output_dir):
        # Preserve calib.json if it contains valid calibration results
        _calib_backup = None
        try:
            _calib_path = _safe_output(output_dir, 'calib.json')
        except ValueError as e:
            print(f"Error: {e}")
            sys.exit(1)
        if _calib_path.exists():
            import json as _json
            try:
                _c = _json.loads(_calib_path.read_text())
                _vec = _c.get('results', {}).get('T_lidar_camera', [])
                if len(_vec) == 7 and any(abs(v) > 1e-6 for v in _vec[:3] + _vec[4:]):
                    _calib_backup = _calib_path.read_text()
            except Exception:
                pass
        shutil.rmtree(output_dir)
        if _calib_backup:
            os.makedirs(output_dir, exist_ok=True)
            _calib_path.write_text(_calib_backup)
            print(f"  Preserved existing calib.json")

    count = combine_scans_for_calibration(base_dir, output_dir, int(sys.argv[2]) if len(sys.argv) > 2 else 10)
    
    if count > 1:
        print(f"\n✓ Ready for calibration with {count} images!")
        print("\nNext steps:")
        print("  cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration")
        print("  python3 ./find_matches_superglue.py ~/atlas_ws/output")
        print("  ./initial_guess_manual --data_path ~/atlas_ws/output")
        print("  ./calibrate --data_path ~/atlas_ws/output")
    else:
        print("Need more scan data for calibration")
