#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Applies ERP-based point cloud coloring to a single scan directory, selecting between world-frame and sensor-frame PLY inputs automatically.
import sys
import os
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def _run_exact_match(scan_dir_str):
    """Worker: import and run exact_match_fusion in this process (no subprocess overhead)."""
    import sys, os
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from exact_match_fusion import exact_match_calibration_tool
    return exact_match_calibration_tool(scan_dir_str), scan_dir_str


def process_scan_coloring(scan_dir, use_exact=False):
    """Apply coloring to a single scan directory"""
    try:
        scan_path = _safe_data(scan_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return False

    sensor_ply = scan_path / "sensor_lidar.ply"
    world_ply  = scan_path / "world_lidar.ply"
    equirect_imgs = list(scan_path.glob("equirect_*.jpg"))

    if not sensor_ply.exists() and not world_ply.exists():
        print(f"No LiDAR PLY found in {scan_dir}")
        return False
    if not equirect_imgs:
        print(f"No equirectangular image found in {scan_dir}")
        return False

    print(f"Processing {scan_path.name}")
    try:
        if use_exact:
            from exact_match_fusion import exact_match_calibration_tool
            ok = exact_match_calibration_tool(str(scan_dir))
        else:
            import subprocess
            script_path = Path(__file__).resolve().parent / "sensor_fusion.py"
            result = subprocess.run(
                [sys.executable, str(script_path), str(scan_dir)],
                check=False, cwd=os.path.expanduser("~/atlas_ws"),
            )
            ok = result.returncode == 0

        colored_candidates = [
            scan_path / "sensor_colored_exact.ply",
            scan_path / "sensor_colored_pointcloud.ply",
            scan_path / "sensor_colored.ply",
            scan_path / "world_colored_exact.ply",
            scan_path / "world_colored.ply",
        ]
        for colored_ply in colored_candidates:
            if colored_ply.exists():
                print(f"\u2713 Created {colored_ply.name} ({scan_path.name})")
                return True
        print(f"\u2717 Failed to create colored PLY for {scan_path.name}")
        return False
    except Exception as e:
        print(f"\u2717 Error processing {scan_path.name}: {e}")
        return False

def post_process_session_coloring(session_dir, use_exact=False):
    """Apply coloring to all scans in a session directory in parallel."""
    try:
        session_path = _safe_data(session_dir)
    except ValueError as e:
        print(f"Error: {e}")
        return
    if not session_path.exists():
        print(f"Session directory not found: {session_dir}")
        return

    scan_dirs = sorted([d for d in session_path.iterdir()
                        if d.is_dir() and d.name.startswith('fusion_scan_')
                        and not (d / '.blur_skip').exists()])
    if not scan_dirs:
        print("No scan directories found")
        return

    method = "exact match" if use_exact else "original method"
    print(f"Post-processing coloring for {len(scan_dirs)} scans using {method} (parallel)...")

    n_workers = min(len(scan_dirs), os.cpu_count() or 4)
    success_count = 0

    if use_exact:
        with ProcessPoolExecutor(max_workers=n_workers) as pool:
            futures = {pool.submit(_run_exact_match, str(d)): d for d in scan_dirs}
            for fut in as_completed(futures):
                try:
                    ok, _ = fut.result()
                    if ok:
                        success_count += 1
                except Exception as e:
                    print(f"  \u2717 Worker error: {e}")
    else:
        for scan_dir in scan_dirs:
            try:
                _safe_data(scan_dir)
            except ValueError:
                continue
            if process_scan_coloring(scan_dir, use_exact=False):
                success_count += 1

    print(f"\n\u2713 Coloring complete: {success_count}/{len(scan_dirs)} scans processed successfully")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python post_process_coloring.py <session_directory> [--use-exact]")
        sys.exit(1)
    try:
        _safe_data(sys.argv[1])
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    use_exact = "--use-exact" in sys.argv
    post_process_session_coloring(sys.argv[1], use_exact)