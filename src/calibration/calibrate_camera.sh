#!/bin/bash

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Per-camera calibration for multi-camera setups.
#
# Each camera is calibrated independently using a dedicated single-camera
# session (only that camera connected). This gives full 360° coverage without
# other cameras occluding the view. A camera-specific calibration mask blocks
# only the scanner body/LiDAR as seen from that camera's mounting position.
#
# Usage:
#   ./calibrate_camera.sh <session_dir> <cam_index> [options]
#
# Examples:
#   # Calibrate cam_0 using a session captured with only the X5 connected:
#   ./calibrate_camera.sh ~/atlas_ws/data/synchronized_scans/sync_fusion_20260715_183444 0
#
#   # Calibrate cam_1 (X3, left position) from its own session:
#   ./calibrate_camera.sh ~/atlas_ws/data/synchronized_scans/sync_fusion_20260716_091200 1
#
#   # Use production mask instead of calibration mask:
#   ./calibrate_camera.sh <session> 0 --use-production-mask
#
# Each camera gets its own output directory: ~/atlas_ws/output_cam_N/
# Results are written to the calibration path specified in multi_camera.yaml.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MULTI_CAM_YAML="$SRC_DIR/config/multi_camera.yaml"
MASKS_DIR="$SRC_DIR/config/masks"
CALIB_TOOL_DIR="$HOME/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration"

if [ $# -lt 2 ]; then
    echo "Usage: $0 <session_dir> <cam_index> [options]"
    echo ""
    echo "Calibrates a single camera using a dedicated single-camera session."
    echo "Capture the session with only the target camera connected to get full"
    echo "360° coverage without other cameras blocking the view."
    echo ""
    echo "Arguments:"
    echo "  session_dir   Path to a sync_fusion_* session (single-camera capture)"
    echo "  cam_index     Camera index (0, 1, or 2) from multi_camera.yaml"
    echo ""
    echo "Options:"
    echo "  --max-scans N          Max scans to use (default: 8)"
    echo "  --skip-matches         Reuse existing SuperGlue matches"
    echo "  --tune-only            Only generate verification overlay"
    echo "  --use-production-mask  Use mask_dual instead of mask_calibration"
    echo ""
    echo "Camera configuration (from multi_camera.yaml):"
    if [ -f "$MULTI_CAM_YAML" ]; then
        python3 -c "
import yaml
d = yaml.safe_load(open('$MULTI_CAM_YAML'))
cams = d.get('cameras', {})
for i in range(3):
    c = cams.get(f'cam_{i}', {})
    if c:
        print(f'  cam_{i}: {c.get(\"camera_hw\",\"?\")} serial={c.get(\"serial\",\"?\")}')
        print(f'          mask_calibration={c.get(\"mask_calibration\", c.get(\"mask_dual\",\"default\"))}')
        print(f'          calibration={c.get(\"calibration\",\"default\")}')
"
    fi
    exit 1
fi

SESSION_DIR="$1"
CAM_INDEX="$2"
shift 2

MAX_SCANS=8
SKIP_MATCHES=false
TUNE_ONLY=false
USE_PRODUCTION_MASK=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --max-scans) MAX_SCANS="$2"; shift 2 ;;
        --skip-matches) SKIP_MATCHES=true; shift ;;
        --tune-only) TUNE_ONLY=true; shift ;;
        --use-production-mask) USE_PRODUCTION_MASK=true; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

if [ ! -d "$SESSION_DIR" ]; then
    echo "✗ Session directory not found: $SESSION_DIR"
    exit 1
fi

if [ ! -f "$MULTI_CAM_YAML" ]; then
    echo "✗ multi_camera.yaml not found: $MULTI_CAM_YAML"
    exit 1
fi

# Read camera config from multi_camera.yaml
read -r CAM_HW CAM_MASK_CALIB CAM_MASK_PROD CAM_CALIB_PATH <<< $(python3 -c "
import yaml
d = yaml.safe_load(open('$MULTI_CAM_YAML'))
c = d.get('cameras', {}).get(f'cam_$CAM_INDEX', {})
hw = c.get('camera_hw', '')
mask_calib = c.get('mask_calibration', c.get('mask_dual', ''))
mask_prod = c.get('mask_dual', '')
calib = c.get('calibration', '')
print(f'{hw} {mask_calib} {mask_prod} {calib}')
")

if [ -z "$CAM_HW" ]; then
    echo "✗ cam_$CAM_INDEX not found in multi_camera.yaml"
    exit 1
fi

# Select mask
if [ "$USE_PRODUCTION_MASK" = "true" ]; then
    CAM_MASK="$CAM_MASK_PROD"
    MASK_LABEL="production (mask_dual)"
else
    CAM_MASK="$CAM_MASK_CALIB"
    MASK_LABEL="calibration (mask_calibration)"
fi

MASK_FILE="$MASKS_DIR/$CAM_MASK"
CALIB_OUTPUT="$SRC_DIR/config/$CAM_CALIB_PATH"
OUTPUT_DIR="$HOME/atlas_ws/output_cam_${CAM_INDEX}"

echo "=========================================="
echo "PER-CAMERA CALIBRATION"
echo "=========================================="
echo "  Session:     $(basename $SESSION_DIR)"
echo "  Camera:      cam_$CAM_INDEX ($CAM_HW)"
echo "  Mask:        $CAM_MASK ($MASK_LABEL)"
echo "  Output:      $OUTPUT_DIR"
echo "  Calibration: $CAM_CALIB_PATH"
echo "  Max scans:   $MAX_SCANS"
echo "=========================================="

# Check mask exists
if [ ! -f "$MASK_FILE" ]; then
    echo ""
    echo "⚠ Mask file not found: $MASK_FILE"
    echo ""
    echo "  To create a calibration mask for cam_$CAM_INDEX:"
    echo "  1. Capture a single scan with only this camera connected"
    echo "  2. Open the stitched ERP image in an image editor"
    echo "  3. Paint white (255) over regions with valid scene content"
    echo "  4. Paint black (0) over the scanner body, LiDAR, and mounting hardware"
    echo "  5. Save as: $MASK_FILE"
    echo ""
    echo "  Tip: Start from an existing mask and modify:"
    echo "    cp $MASKS_DIR/lidar_mask_dual_${CAM_HW}.png $MASK_FILE"
    echo ""
    read -p "Continue without mask? (y/n): " _cont
    [ "$_cont" != "y" ] && exit 1
    MASK_FILE=""
fi

# Clean output directory
if [ "$SKIP_MATCHES" = "false" ] && [ "$TUNE_ONLY" = "false" ]; then
    rm -rf "$OUTPUT_DIR"
fi
mkdir -p "$OUTPUT_DIR"

# Step 1: Combine scans (no cam-index filter — single-camera session uses all scans)
echo ""
echo "Step 1: Extracting scans from single-camera session..."

# Temporarily redirect output to per-camera dir
_ORIG_OUTPUT="$HOME/atlas_ws/output"
if [ -d "$_ORIG_OUTPUT" ]; then
    mv "$_ORIG_OUTPUT" "${_ORIG_OUTPUT}.bak_cam${CAM_INDEX}" 2>/dev/null || true
fi

python3 "$SCRIPT_DIR/combine_scans_for_calibration.py" \
    "$SESSION_DIR" "$MAX_SCANS"

# Move to per-camera output dir
if [ -d "$_ORIG_OUTPUT" ]; then
    rm -rf "$OUTPUT_DIR"
    mv "$_ORIG_OUTPUT" "$OUTPUT_DIR"
fi
# Restore original output if it was backed up
if [ -d "${_ORIG_OUTPUT}.bak_cam${CAM_INDEX}" ]; then
    mv "${_ORIG_OUTPUT}.bak_cam${CAM_INDEX}" "$_ORIG_OUTPUT" 2>/dev/null || true
fi

SCAN_COUNT=$(ls "$OUTPUT_DIR"/*.ply 2>/dev/null | wc -l)
if [ "$SCAN_COUNT" -lt 1 ]; then
    echo "✗ No scans found in session"
    exit 1
fi
echo "  ✓ $SCAN_COUNT scans extracted"

# Step 2: Generate intensity images with camera-specific calibration mask
echo ""
echo "Step 2: Generating intensity images..."
if [ -n "$MASK_FILE" ]; then
    echo "  Using mask: $(basename $MASK_FILE)"
fi

export ATLAS_CALIBRATION_MASK="${MASK_FILE:-}"
export ATLAS_CALIBRATION_CAM_INDEX="$CAM_INDEX"
export ATLAS_CALIBRATION_CAM_HW="$CAM_HW"
# Set the exact calibration file path so find_matches_superglue_erp.py uses
# the same per-slot calibration that generate_intensity_images.py used.
# Without this, SuperGlue reads the hw-level file and aims crops at the wrong
# ERP region for cameras with non-zero yaw (cam_1 left, cam_2 right).
export ATLAS_CALIBRATION_FILE="$CALIB_OUTPUT"

python3 "$SCRIPT_DIR/generate_intensity_images.py" "$OUTPUT_DIR"

if [ "$TUNE_ONLY" = "true" ]; then
    echo ""
    echo "Step 3: Generating verification overlay..."
    if [ -f "$CALIB_OUTPUT" ]; then
        cp "$CALIB_OUTPUT" "$SRC_DIR/config/fusion_calibration.yaml"
    fi
    python3 "$SCRIPT_DIR/tune_calibration.py" --verify
    echo "✓ Tune-only mode complete. Check $OUTPUT_DIR/calib_sweep/current.jpg"
    exit 0
fi

if [ "$SKIP_MATCHES" = "false" ]; then
    # Step 3: Run SuperGlue feature matching
    echo ""
    echo "Step 3: Running SuperGlue feature matching..."
    if [ -d "$CALIB_TOOL_DIR" ]; then
        (cd "$CALIB_TOOL_DIR" && python3 ./find_matches_superglue.py "$OUTPUT_DIR" --superglue indoor)
    else
        python3 "$SCRIPT_DIR/find_matches_superglue_erp.py" "$OUTPUT_DIR"
    fi

    # Step 4: Fix match coordinates
    echo ""
    echo "Step 4: Fixing match coordinates..."
    python3 "$SCRIPT_DIR/fix_matches.py" "$OUTPUT_DIR"
fi

# Step 5: Initial guess
echo ""
echo "Step 5: Computing initial guess..."
if [ -f "$CALIB_OUTPUT" ]; then
    echo "  Seeding from existing: $(basename $CALIB_OUTPUT)"
    python3 "$SCRIPT_DIR/compose_initial_guess.py" "$OUTPUT_DIR" --seed "$CALIB_OUTPUT"
elif [ -d "$CALIB_TOOL_DIR" ]; then
    (cd "$CALIB_TOOL_DIR" && ./initial_guess_manual --data_path "$OUTPUT_DIR")
else
    python3 "$SCRIPT_DIR/compose_initial_guess.py" "$OUTPUT_DIR"
fi

# Step 6: Optimize
echo ""
echo "Step 6: Running calibration optimization..."
if [ -d "$CALIB_TOOL_DIR" ]; then
    (cd "$CALIB_TOOL_DIR" && ./calibrate --data_path "$OUTPUT_DIR")
fi

# Step 7: Extract and save
echo ""
echo "Step 7: Extracting calibration result..."
mkdir -p "$HOME/atlas_ws/output"
cp "$OUTPUT_DIR/calib.json" "$HOME/atlas_ws/output/calib.json"

python3 "$SCRIPT_DIR/extract_calibration.py" --camera-hw "$CAM_HW"

# Copy to the per-camera position path
CALIB_OUTPUT_DIR="$(dirname "$CALIB_OUTPUT")"
mkdir -p "$CALIB_OUTPUT_DIR"
cp "$SRC_DIR/config/calibrations/$CAM_HW/fusion_calibration.yaml" "$CALIB_OUTPUT"

echo ""
echo "=========================================="
echo "✓ CALIBRATION COMPLETE"
echo "=========================================="
echo "  Camera:      cam_$CAM_INDEX ($CAM_HW)"
echo "  Result:      $CALIB_OUTPUT"
echo "  Dataset:     $OUTPUT_DIR"
echo ""
echo "Verify:"
echo "  $0 $SESSION_DIR $CAM_INDEX --tune-only"
echo ""
echo "Fine-tune:"
echo "  python3 $SCRIPT_DIR/tune_calibration.py --axis pitch --range 2.0"
echo "  python3 $SCRIPT_DIR/tune_calibration.py --axis pitch --apply <degrees>"
