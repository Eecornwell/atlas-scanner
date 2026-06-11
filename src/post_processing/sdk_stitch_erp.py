#!/usr/bin/env python3
"""
SDK-based ERP stitching for raw .insp files already downloaded from the camera.

Replaces the fisheye_to_erp.py pipeline with Insta360 MediaSDK stitching.
Can be used as a drop-in replacement in the post-processing pipeline.

Usage:
    python3 sdk_stitch_erp.py <input.insp> <output.jpg> [--width 3840] [--height 1920]
    python3 sdk_stitch_erp.py <scan_dir> --scan-dir [--width 3840] [--height 1920]

In --scan-dir mode, finds any .insp files in the scan directory and stitches them.
"""

import argparse
import os
import sys
import subprocess
import glob
from pathlib import Path

# Path to the SDK stitcher binary
STITCHER_BINARY = os.path.expanduser("~/insta360-dev/build/insta360_stitch")

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def stitch_file(input_path, output_path, width=3840, height=1920):
    """Stitch a single .insp file to ERP using the SDK."""
    try:
        safe_in  = _safe_data(input_path)
        safe_out = _safe_data(output_path)
    except ValueError as e:
        print(f"✗ Path rejected: {e}")
        return False
    env = os.environ.copy()
    env["INSTA360_ERP_WIDTH"] = str(width)
    env["INSTA360_ERP_HEIGHT"] = str(height)

    result = subprocess.run(
        [STITCHER_BINARY, str(safe_in), str(safe_out)],
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )
    if result.returncode != 0:
        print(f"✗ Stitch failed for {safe_in.name}: {result.stderr.strip()}")
        return False
    print(f"✓ ERP saved: {safe_out}")
    return True


def process_scan_dir(scan_dir, width=3840, height=1920):
    """Find .insp files in a scan directory and stitch them."""
    try:
        safe_scan = _safe_data(scan_dir)
    except ValueError as e:
        print(f"✗ Path rejected: {e}")
        return False
    insp_files = glob.glob(os.path.join(str(safe_scan), "*.insp"))
    if not insp_files:
        insp_files = glob.glob(os.path.join(str(safe_scan), "**/*.insp"), recursive=True)

    if not insp_files:
        print(f"No .insp files found in {safe_scan}")
        return False

    for insp in sorted(insp_files):
        try:
            _safe_data(insp)
        except ValueError:
            continue
        output = str(_safe_data(safe_scan / "equirect_dual_fisheye.jpg"))
        if not stitch_file(insp, output, width, height):
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description="SDK-based ERP stitching")
    parser.add_argument("input", help="Input .insp file or scan directory")
    parser.add_argument("output", nargs="?", help="Output .jpg path (file mode)")
    parser.add_argument("--scan-dir", action="store_true",
                        help="Process all .insp files in directory")
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=1920)
    args = parser.parse_args()

    if not os.path.isfile(STITCHER_BINARY):
        print(f"✗ Stitcher binary not found: {STITCHER_BINARY}")
        print("  Build it with: cd ~/insta360-dev && ./build.sh")
        sys.exit(1)

    try:
        _safe_data(args.input)
        if args.output:
            _safe_data(args.output)
    except ValueError as e:
        print(f"✗ {e}")
        sys.exit(1)
    if args.scan_dir:
        ok = process_scan_dir(args.input, args.width, args.height)
    else:
        if not args.output:
            args.output = str(_safe_data(Path(args.input).with_suffix(".jpg")))
        ok = stitch_file(args.input, args.output, args.width, args.height)

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
