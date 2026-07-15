#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake "$SCRIPT_DIR" ${CAMERA_SDK_DIR:+-DCAMERA_SDK_DIR="$CAMERA_SDK_DIR"}
make -j$(nproc)
echo ""
echo "Build complete:"
echo "  $BUILD_DIR/insta360_capture"
echo "  $BUILD_DIR/insta360_capture_multi"
echo "  $BUILD_DIR/insta360_stitch"
echo "  $BUILD_DIR/insta360_reset_clock"
echo "  $BUILD_DIR/livox_time_sync"
