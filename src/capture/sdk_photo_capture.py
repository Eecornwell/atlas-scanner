#!/usr/bin/env python3
"""
SDK-based 360 photo capture and ERP stitching.

Uses the insta360_capture binary (CameraSDK + MediaSDK) to take a full-resolution
still photo and stitch it into an equirectangular image using factory calibration.

Usage:
    python3 sdk_photo_capture.py <output_dir> [--width 3840] [--height 1920]

Output:
    <output_dir>/equirect_dual_fisheye.jpg   (stitched ERP image)
    <output_dir>/raw_photo.insp              (raw camera file, kept for reference)
"""

import argparse
import os
import subprocess
import sys
import glob

CAPTURE_BINARY = os.path.expanduser("~/insta360-dev/build/insta360_capture")


def main():
    parser = argparse.ArgumentParser(description="SDK 360 photo capture with ERP stitching")
    parser.add_argument("output_dir", help="Directory to save output images")
    parser.add_argument("--width", type=int, default=3840, help="ERP output width")
    parser.add_argument("--height", type=int, default=1920, help="ERP output height")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    if not os.path.isfile(CAPTURE_BINARY):
        print(f"✗ Capture binary not found: {CAPTURE_BINARY}")
        print("  Build it with: cd ~/insta360-dev && ./build.sh")
        sys.exit(1)

    # Run the capture binary with output dir
    env = os.environ.copy()
    env["INSTA360_OUTPUT_DIR"] = args.output_dir
    env["INSTA360_ERP_WIDTH"] = str(args.width)
    env["INSTA360_ERP_HEIGHT"] = str(args.height)

    result = subprocess.run(
        [CAPTURE_BINARY],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )

    if result.returncode != 0:
        print(f"✗ Capture failed (exit {result.returncode})")
        if result.stderr:
            print(f"  stderr: {result.stderr.strip()}")
        if result.stdout:
            print(f"  stdout: {result.stdout.strip()}")
        sys.exit(1)

    # Find the ERP output
    erp_files = glob.glob(os.path.join(args.output_dir, "erp_*.jpg"))
    if erp_files:
        erp_file = sorted(erp_files)[-1]  # most recent
        # Rename to canonical name expected by the pipeline
        canonical = os.path.join(args.output_dir, "equirect_dual_fisheye.jpg")
        os.rename(erp_file, canonical)
        print(f"✓ ERP image saved: {canonical}")
    else:
        print("✗ No ERP output found after capture")
        sys.exit(1)


if __name__ == "__main__":
    main()
