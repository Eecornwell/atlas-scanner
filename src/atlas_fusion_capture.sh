#!/bin/bash

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Headless launcher for the ATLAS system. Sources the ROS2 workspace and runs
# atlas_fusion_capture.sh directly without a GUI, logging all output to the session directory.

# ATLAS Fusion Capture Script
# CAMERA_MODE: dual_fisheye | single_fisheye
# CAPTURE_MODE: stationary | continuous

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# ─── User Configuration ────────────────────────────────────────────────────────
CAMERA_MODE="dual_fisheye"        # dual_fisheye | single_fisheye
CAPTURE_MODE="continuous"         # stationary | continuous
CONTINUOUS_INTERVAL=5             # seconds between captures (continuous mode only; camera minimum is 5s)
STATIONARY_WAIT=false             # stationary only: wait 3s before starting rosbag (allows scanner to settle)
CAMERA_HW="onex2"                 # Camera hardware model: onex2 | x5

CLEAN_POINTCLOUD=true             # statistical outlier removal on merged cloud
DOWNSAMPLE_VOXEL_SIZE=0.03        # Merged point cloud voxel downsample in metres (0 = skip)
COLMAP_LIDAR_VOXEL_SIZE=0.05      # LiDAR downsample before COLMAP merge in metres (0 = skip)

# Allow CLI overrides: atlas_fusion_capture.sh [--camera dual_fisheye|single_fisheye] [--capture stationary|continuous]
while [[ $# -gt 0 ]]; do
    case $1 in
        --camera) CAMERA_MODE="$2"; shift 2 ;;
        --capture) CAPTURE_MODE="$2"; shift 2 ;;
        --interval) CONTINUOUS_INTERVAL="$2"; shift 2 ;;
        --bag-only) BAG_ONLY=true; shift ;;
        --no-sync-benchmark) RUN_SYNC_BENCHMARK=false; shift ;;
        --stationary-wait) STATIONARY_WAIT=true; shift ;;
        --no-stationary-wait) STATIONARY_WAIT=false; shift ;;
        --icp) ENABLE_ICP_ALIGNMENT=true; shift ;;
        --no-icp) ENABLE_ICP_ALIGNMENT=false; shift ;;
        --colmap) EXPORT_COLMAP=true; shift ;;
        --no-colmap) EXPORT_COLMAP=false; shift ;;
        --lidar-voxel-size) COLMAP_LIDAR_VOXEL_SIZE="$2"; shift 2 ;;
        --camera-hw) CAMERA_HW="$2"; shift 2 ;;
        *) shift ;;
    esac
done
RUN_SYNC_BENCHMARK=true

ENABLE_POST_PROCESSING_BAGS=false
SKIP_LIVE_FUSION=true
AUTO_CREATE_COLORED=true

# ─── Camera hardware model config ─────────────────────────────────────────────
_CAM_MODEL_DIR="$SCRIPT_DIR/config/camera_models"
_CAM_CALIB_DIR="$SCRIPT_DIR/config/calibrations"
_VALID_HW="onex2 x5"
if ! echo "$_VALID_HW" | grep -qw "$CAMERA_HW"; then
    echo "⚠ Unknown CAMERA_HW='$CAMERA_HW', defaulting to onex2"
    CAMERA_HW="onex2"
fi

# Load ERP resolution and mask filenames from camera model YAML
_hw_yaml="$_CAM_MODEL_DIR/${CAMERA_HW}.yaml"
if [ -f "$_hw_yaml" ]; then
    INSTA360_ERP_WIDTH=$(python3 -c "import yaml,sys; d=yaml.safe_load(open('$_hw_yaml')); print(d.get('erp_width',5760))")
    INSTA360_ERP_HEIGHT=$(python3 -c "import yaml,sys; d=yaml.safe_load(open('$_hw_yaml')); print(d.get('erp_height',2880))")
    _LIDAR_MASK_DUAL=$(python3 -c "import yaml,sys; d=yaml.safe_load(open('$_hw_yaml')); print(d.get('lidar_mask_dual','lidar_mask_dual_sdk.png'))")
    _LIDAR_MASK_SINGLE=$(python3 -c "import yaml,sys; d=yaml.safe_load(open('$_hw_yaml')); print(d.get('lidar_mask_single','lidar_mask_single.png'))")
    _HW_DISPLAY=$(python3 -c "import yaml,sys; d=yaml.safe_load(open('$_hw_yaml')); print(d.get('display_name',sys.argv[1]))" "$CAMERA_HW")
else
    echo "⚠ No camera model config at $_hw_yaml — using defaults"
    INSTA360_ERP_WIDTH=5760
    INSTA360_ERP_HEIGHT=2880
    _LIDAR_MASK_DUAL="lidar_mask_dual_sdk.png"
    _LIDAR_MASK_SINGLE="lidar_mask_single.png"
    _HW_DISPLAY="$CAMERA_HW"
fi
export INSTA360_ERP_WIDTH INSTA360_ERP_HEIGHT

# Point all tools at the per-model calibration file.
# Falls back to the shared config/fusion_calibration.yaml if not present.
_hw_calib="$_CAM_CALIB_DIR/${CAMERA_HW}/fusion_calibration.yaml"
if [ -f "$_hw_calib" ]; then
    cp "$_hw_calib" "$SCRIPT_DIR/config/fusion_calibration.yaml"
    echo "Loaded calibration: $_hw_calib"
else
    echo "⚠ No per-model calibration at $_hw_calib — using existing fusion_calibration.yaml"
fi

cd "$ROS_WS_DIR"
export PATH=/home/orion/cmake-3.25/bin:$PATH
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/orion:/usr/local/lib
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_atlas.xml"
export ROS_DISABLE_DAEMON=1
# Prevent ros2 CLI tools from hanging when no display is available (headless / SSH)
export QT_QPA_PLATFORM=offscreen

mkdir -p data/synchronized_scans
# Increase UDP receive buffer to prevent Livox MID360 packet drops under load.
# Default 208KB is insufficient for ~2-4MB/s point cloud UDP stream; 32MB gives
# enough headroom during rosbag compression bursts between scans.
# Increase UDP receive buffer to prevent packet drops on the lidar topic.
# Try sudo first; fall back to writing directly if we have permission.
if ! sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=33554432 > /dev/null 2>&1; then
    echo 33554432 | sudo tee /proc/sys/net/core/rmem_max > /dev/null 2>&1 || true
    echo 33554432 | sudo tee /proc/sys/net/core/rmem_default > /dev/null 2>&1 || true
fi
_cur_rmem=$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)
if [ "$_cur_rmem" -lt 4194304 ]; then
    echo "⚠ UDP receive buffer is small ($_cur_rmem bytes) — lidar packets may be dropped between scans"
fi
SCAN_DIR="$(pwd)/data/synchronized_scans/sync_fusion_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SCAN_DIR"

# Tee all stdout+stderr to a session log
LOG_FILE="$SCAN_DIR/session.log"
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== Session log started: $(date) ==="
echo "=== ATLAS Fusion Capture | camera=$CAMERA_MODE capture=$CAPTURE_MODE hw=$CAMERA_HW ==="
# Write session metadata so post-processing tools know the camera hardware
cat > "$SCAN_DIR/session_config.json" << _EOF
{"camera_mode": "$CAMERA_MODE", "capture_mode": "$CAPTURE_MODE", "camera_hw": "$CAMERA_HW",
 "erp_width": $INSTA360_ERP_WIDTH, "erp_height": $INSTA360_ERP_HEIGHT}
_EOF

PIDS=()
FUSION_READY="false"
LIO_ENABLED="false"
CLEANUP_DONE=false
TRAJECTORY_RECORDING=false
SCAN_COUNT=0

# ─── Cleanup / Post-processing ────────────────────────────────────────────────
cleanup() {
    [ "$CLEANUP_DONE" = "true" ] && return
    CLEANUP_DONE=true
    trap '' SIGINT SIGTERM
    echo "Shutting down..."

    # Shut down camera driver first with a long graceful wait so the SDK can
    # call StopLiveStreaming()+Close() cleanly. SIGKILL before this completes
    # leaves the firmware in streaming state for the next session.
    # Send SIGINT (not SIGTERM) to the ros2 launch process — ros2 launch handles
    # SIGINT gracefully by forwarding it to children, whereas SIGTERM causes it
    # to SIGKILL children immediately without giving the SDK time to shut down.
    for _cam_pid in $(pgrep -f "insta360_capture" 2>/dev/null); do
        kill -INT "$_cam_pid" 2>/dev/null
    done
    for _w in $(seq 1 150); do
        pgrep -f "insta360_capture" > /dev/null 2>&1 || break
        sleep 0.1
    done
    pgrep -f "insta360_capture" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true

    # Kill all other tracked processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null
            for _w in $(seq 1 20); do
                kill -0 "$pid" 2>/dev/null || break
                sleep 0.1
            done
            kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null
        fi
    done

    TRAJ_PID_FILE="$SCAN_DIR/.trajectory_recorder.pid"
    if [ -f "$TRAJ_PID_FILE" ]; then
        TRAJ_PID=$(cat "$TRAJ_PID_FILE" 2>/dev/null)
        if [ -n "$TRAJ_PID" ] && kill -0 "$TRAJ_PID" 2>/dev/null; then
            for i in $(seq 1 30); do
                PENDING=$(ls "$SCAN_DIR"/.save_trajectory_* 2>/dev/null | wc -l)
                [ "$PENDING" -eq 0 ] && break
                sleep 0.2
            done
            kill -TERM "$TRAJ_PID" 2>/dev/null
            wait "$TRAJ_PID" 2>/dev/null
        fi
        rm -f "$TRAJ_PID_FILE"
    fi

    _pkill "enhanced_trajectory_recorder"
    _pkill "buffered_odom_bridge"
    _pkill "rviz2"
    _pkill "insta360_capture"
    _pkill "livox_ros_driver2"
    _pkill "rko_lio"
    _pkill "static_transform_publisher"
    _pkill "ros2 bag record"
    _pkill "topic_tools"
    _pkill "continuous_trajectory_recorder"

    # Wait for the SDK capture daemon to call StopTimeLapse and exit cleanly.
    # It was deliberately excluded from PIDS so the loop above didn't kill it.
    if [ -n "${SDK_CAPTURE_PID:-}" ] && kill -0 "$SDK_CAPTURE_PID" 2>/dev/null; then
        echo "Waiting for SDK capture daemon to stop timelapse..."
        for _w in $(seq 1 300); do
            kill -0 "$SDK_CAPTURE_PID" 2>/dev/null || break
            sleep 1
        done
        kill -0 "$SDK_CAPTURE_PID" 2>/dev/null && kill -KILL "$SDK_CAPTURE_PID" 2>/dev/null || true
        wait "$SDK_CAPTURE_PID" 2>/dev/null || true
    fi

    if [ "$BAG_ONLY" = "true" ]; then
        echo "Bag-only mode: skipping post-processing. Bag saved in: $SCAN_DIR"
        exit 0
    fi

    # Run all post-processing in a subshell with SIGINT ignored.
    # Children (Python scripts) inherit SIG_IGN and cannot be killed by Ctrl+C.
    (trap '' SIGINT SIGTERM

    # Continuous mode: reconstruct scans from the single session bag
    if [ "$CAPTURE_MODE" = "continuous" ]; then
        BAG_DIR=$(ls -d "$SCAN_DIR"/rosbag_* 2>/dev/null | grep -v '_imu$' | head -1)
        if [ -n "$BAG_DIR" ]; then
            echo "Estimating IMU clock offset..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/imu_sync.py" \
                "$SCAN_DIR" --window 20.0 || echo "  ⚠ IMU sync failed — continuing without offset"
            echo "Reconstructing scans from bag using header timestamps..."
            _SDK_STITCH_ARG="--sdk-stitch"
            _TRIM_ENDS="0"  # scan centres are at precise shutter times, no SLAM trim needed
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py" \
                "$SCAN_DIR" --interval "$CONTINUOUS_INTERVAL" --lidar-window 0.6 --camera-mode "$CAMERA_MODE" --max-gyro 0.15 --trim-ends "$_TRIM_ENDS" $_SDK_STITCH_ARG
            # Promote .insp files from .sdk_shot_N/ into fusion_scan_NNN/ by shot order,
            # then stitch each one to equirect_dual_fisheye.jpg
            _scan_idx=0
            for _shot_dir in $(ls -d "$SCAN_DIR"/.sdk_shot_* 2>/dev/null | sort -t_ -k3 -n); do
                _scan_idx=$((_scan_idx + 1))
                _target_scan="$SCAN_DIR/fusion_scan_$(printf '%03d' $_scan_idx)"
                [ -d "$_target_scan" ] || continue
                for _insp in "$_shot_dir"/*.insp; do
                    [ -f "$_insp" ] || continue
                    [ "$(stat -c%s "$_insp" 2>/dev/null)" -lt 100000 ] && continue
                    mv "$_insp" "$_target_scan/" 2>/dev/null
                    [ -f "${_insp}.capture_time" ] && mv "${_insp}.capture_time" "$_target_scan/" 2>/dev/null
                done
            done
            echo "Converting fisheye images to ERP (SDK stitcher)..."
            for _scan_dir in "$SCAN_DIR"/fusion_scan_*; do
                [ -d "$_scan_dir" ] || continue
                INSP_FILE=$(find "$_scan_dir" -maxdepth 1 -name "*.insp" | head -1)
                [ -z "$INSP_FILE" ] && continue
                _stitch_args=""
                [ "$CAMERA_MODE" = "single_fisheye" ] && _stitch_args="--single"
                ~/insta360-dev/build/insta360_stitch "$INSP_FILE" "$_scan_dir/equirect_dual_fisheye.jpg" \
                    --width "$INSTA360_ERP_WIDTH" --height "$INSTA360_ERP_HEIGHT" \
                    $_stitch_args \
                    || echo "  Warning: SDK stitch failed for $(basename $_scan_dir), skipping"
            done
            # SDK stitch: re-run coloring after reconstruction so sensor_colored_exact.ply
            # is built from the freshly reconstructed sensor_lidar.ply, not stale data.
            echo "Generating masked images..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/regenerate_masked_images.py" "$SCAN_DIR" --camera-mode "$CAMERA_MODE" --sdk-stitch --camera-hw "$CAMERA_HW"
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_coloring.py" "$SCAN_DIR" --use-exact
        fi
    fi

    SCAN_COUNT=$(ls -d "$SCAN_DIR"/fusion_scan_* 2>/dev/null | wc -l)

    if [ "$SCAN_COUNT" -gt 0 ]; then
        echo ""
        echo "=========================================="
        echo "SCANNING SESSION COMPLETE"
        echo "=========================================="
        echo "Total scans captured: $SCAN_COUNT"
        echo "Data saved in: $SCAN_DIR"
        echo ""

        # Coloring
        if [ "$CAPTURE_MODE" = "stationary" ] && [ "$SKIP_LIVE_FUSION" = "true" ] && [ "$AUTO_CREATE_COLORED" = "true" ]; then
            # Promote .insp files into fusion_scan_NNN/ by capture order.
                # Source 1: .sdk_shot_N/ subdirectories (older capture method)
                _scan_idx=0
                for _shot_dir in $(ls -d "$SCAN_DIR"/.sdk_shot_* 2>/dev/null | sort -t_ -k2 -n); do
                    _scan_idx=$((_scan_idx + 1))
                    _target_scan="$SCAN_DIR/fusion_scan_$(printf '%03d' $_scan_idx)"
                    [ -d "$_target_scan" ] || continue
                    for _insp in "$_shot_dir"/*.insp; do
                        [ -f "$_insp" ] || continue
                        [ "$(stat -c%s "$_insp" 2>/dev/null)" -lt 100000 ] && continue
                        mv "$_insp" "$_target_scan/" 2>/dev/null
                        [ -f "${_insp}.capture_time" ] && mv "${_insp}.capture_time" "$_target_scan/" 2>/dev/null
                    done
                done
                # Source 2: .insp files directly at session root (current capture method)
                _root_insps=($(ls "$SCAN_DIR"/*.insp 2>/dev/null | sort))
                if [ ${#_root_insps[@]} -gt 0 ]; then
                    _scan_idx=0
                    for _insp in "${_root_insps[@]}"; do
                        [ -f "$_insp" ] || continue
                        [ "$(stat -c%s "$_insp" 2>/dev/null)" -lt 100000 ] && continue
                        _scan_idx=$((_scan_idx + 1))
                        _target_scan="$SCAN_DIR/fusion_scan_$(printf '%03d' $_scan_idx)"
                        [ -d "$_target_scan" ] || continue
                        mv "$_insp" "$_target_scan/" 2>/dev/null
                        [ -f "${_insp}.capture_time" ] && mv "${_insp}.capture_time" "$_target_scan/" 2>/dev/null
                    done
                fi

                # SDK stitching: use MediaSDK for high-quality ERP from .insp files
                echo "Converting fisheye images to ERP (SDK stitcher)..."
                for scan_dir in "$SCAN_DIR"/fusion_scan_*; do
                    [ -d "$scan_dir" ] || continue
                    # If we have a raw .insp file, stitch it directly
                    INSP_FILE=$(find "$scan_dir" -maxdepth 1 -name "*.insp" | head -1)
                    if [ -n "$INSP_FILE" ]; then
                        _stitch_args=""
                        [ "$CAMERA_MODE" = "single_fisheye" ] && _stitch_args="--single"
                        ~/insta360-dev/build/insta360_stitch "$INSP_FILE" "$scan_dir/equirect_dual_fisheye.jpg" \
                            --width "$INSTA360_ERP_WIDTH" --height "$INSTA360_ERP_HEIGHT" \
                            $_stitch_args \
                            || echo "  Warning: SDK stitch failed for $(basename $scan_dir), skipping"
                    else
                        echo "  Warning: No .insp file for $(basename $scan_dir), skipping"
                    fi
                done
                # SDK stitcher handles seam blending internally, no need for blend_erp_seams
                echo "Creating masked images..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/regenerate_masked_images.py" "$SCAN_DIR" --camera-mode "$CAMERA_MODE" --sdk-stitch --camera-hw "$CAMERA_HW"
                echo "Creating colored point clouds..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_coloring.py" "$SCAN_DIR" --use-exact
            echo "✓ Colored point clouds created"
        fi
        # continuous mode coloring is handled inside reconstruct_from_bag.py

        MERGED_FILE=""

        if [ "$SCAN_COUNT" -gt 1 ]; then
            if [ "$ENABLE_ICP_ALIGNMENT" = "true" ] && [ "$CAMERA_MODE" = "dual_fisheye" ]; then
                echo "Skipping trajectory-based merge (ICP alignment will merge instead)..."
            else
                echo "Merging scans using trajectory poses..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/merge_with_trajectory.py" "$SCAN_DIR"
                MERGED_FILE="$SCAN_DIR/merged_pointcloud.ply"
                if [ "$SAVE_E57" = "true" ] && [ -f "$MERGED_FILE" ]; then
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/ply_to_e57.py" "$MERGED_FILE" "${MERGED_FILE%.ply}.e57"
                fi
            fi
        elif [ "$SCAN_COUNT" -eq 1 ]; then
            FIRST_SCAN=$(ls -d "$SCAN_DIR"/fusion_scan_* | head -1)
            COLORED_PLY=$(ls "$FIRST_SCAN"/world_colored*.ply 2>/dev/null | head -1)
            if [ -n "$COLORED_PLY" ]; then
                cp "$COLORED_PLY" "$SCAN_DIR/merged_pointcloud.ply"
                MERGED_FILE="$SCAN_DIR/merged_pointcloud.ply"
                echo "✓ Single scan copied as merged"
                if [ "$SAVE_E57" = "true" ] && [ -f "$MERGED_FILE" ]; then
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/ply_to_e57.py" "$MERGED_FILE" "${MERGED_FILE%.ply}.e57"
                fi
            fi
        fi

        if [ "$SCAN_COUNT" -gt 1 ] && [ "$ENABLE_ICP_ALIGNMENT" = "true" ]; then
            echo "Applying ICP alignment..."
        # Continuous mode: scan centres already gyro-filtered by reconstruct_from_bag,
        # so use a higher threshold here to avoid over-filtering.
        _POSEGRAPH_GYRO="99.0"  # scan centres are at shutter times, no gyro filter needed
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/align_scan_session_posegraph.py" "$SCAN_DIR" --max-gyro "$_POSEGRAPH_GYRO" --iterations 1
            if [ $? -eq 0 ] && [ -f "$SCAN_DIR/merged_aligned_colored.ply" ]; then
                MERGED_FILE="$SCAN_DIR/merged_aligned_colored.ply"
                if [ "$SAVE_E57" = "true" ]; then
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/ply_to_e57.py" "$MERGED_FILE" "${MERGED_FILE%.ply}.e57"
                fi
            fi
        fi

        # Clean / downsample merged cloud
        if [ -n "$MERGED_FILE" ] && [ -f "$MERGED_FILE" ]; then
            if [ "$CLEAN_POINTCLOUD" = "true" ]; then
                CLEAN_ARGS="--nb-neighbors 20 --std-ratio 2.0"
            else
                CLEAN_ARGS="--nb-neighbors 0"
            fi
            CLEAN_ARGS="$CLEAN_ARGS --voxel ${DOWNSAMPLE_VOXEL_SIZE:-0}"
            if [ "$CLEAN_POINTCLOUD" = "true" ] || [ "${DOWNSAMPLE_VOXEL_SIZE:-0}" != "0" ]; then
                echo "Cleaning/downsampling point cloud..."
                CLEANED_NAME=$(python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/clean_pointcloud.py" \
                    "$MERGED_FILE" $CLEAN_ARGS | grep 'Saved:' | awk '{print $NF}')
                [ -n "$CLEANED_NAME" ] && MERGED_FILE="$(dirname "$MERGED_FILE")/$CLEANED_NAME"
            fi
        fi

        # Web viewer
        [ -z "$MERGED_FILE" ] && MERGED_FILE=$(find "$SCAN_DIR" -name "world_colored_exact.ply" | head -1)
        if [ -n "$MERGED_FILE" ] && [ -f "$MERGED_FILE" ]; then
            echo "Opening 3D viewer..."
            DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/web_3d_viewer.py" "$MERGED_FILE"
        fi

        if [ "$EXPORT_COLMAP" = "true" ]; then
            echo "Exporting session to COLMAP format (panorama SfM)..."
            _colmap_args=""
            [ "${COLMAP_LIDAR_VOXEL_SIZE:-0}" != "0" ] && _colmap_args="--lidar-voxel-size ${COLMAP_LIDAR_VOXEL_SIZE}"
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/panorama_sfm_colmap.py" "$SCAN_DIR" --no-bundle-adjustment $_colmap_args
        fi

        echo "Complete. Scans saved in: $SCAN_DIR"

        if [ "$RUN_SYNC_BENCHMARK" = "true" ]; then
            echo "Running sync benchmark..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/sync_benchmark.py" \
                "$SCAN_DIR" --out "$SCAN_DIR/sync_benchmark.json"
        fi
    else
        if [ "$FUSION_READY" = "true" ]; then
            echo "Session ended with no scans captured (stopped before any scan was triggered)."
        else
            echo "Failed to initialize sensors. Check hardware connections."
        fi
    fi
    ) # end post-processing subshell    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ─── Helpers ──────────────────────────────────────────────────────────────────
wait_for_topic() {
    local topic=$1 timeout=${2:-30} count=0
    echo "Waiting for topic $topic..."
    while [ $count -lt $timeout ]; do
        ros2 topic list 2>/dev/null | grep -q "$topic" && { echo "✓ Topic $topic is available"; return 0; }
        sleep 1; count=$((count + 1))
    done
    echo "✗ Timeout waiting for topic $topic"; return 1
}

wait_for_topic_data() {
    local topic=$1 timeout=${2:-15}
    echo "Waiting for data on $topic..."
    timeout "$timeout" ros2 topic echo "$topic" --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then echo "✓ Topic $topic has data"; return 0
    else echo "✗ No data on topic $topic"; return 1; fi
}

# ─── Startup ──────────────────────────────────────────────────────────────────
echo "=== ATLAS Fusion Capture | camera=$CAMERA_MODE capture=$CAPTURE_MODE ==="
echo "Scans will be saved to: $SCAN_DIR"

if [ "$SKIP_SUDO_CHECK" != "1" ]; then
    sudo -v || { echo "Failed to obtain sudo access"; exit 1; }
fi

if [ "$USE_EXISTING_CALIBRATION" = "true" ]; then
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/calibration/coordinate_transform.py" "$ROS_WS_DIR/src/atlas-scanner/src" --use-existing --camera-hw "$CAMERA_HW"
else
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/calibration/coordinate_transform.py" "$ROS_WS_DIR/src/atlas-scanner/src" --camera-hw "$CAMERA_HW"
fi

# Kill stale processes (use pgrep+kill to avoid pkill matching its own cmdline args)
# Sends SIGTERM first so C++ destructors (StopLiveStreaming/Close) can run cleanly,
# then SIGKILL only if the process is still alive after a short wait.
_pkill() {
    local pat="$1"
    local pids
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -v "^$$\$")
    [ -z "$pids" ] && return 0
    echo $pids | xargs -r kill -INT 2>/dev/null || true
    # For the camera driver, allow up to 15s for StopLiveStreaming()+Close() to complete
    local _max=20
    echo "$pat" | grep -qE "insta360|dual_fisheye|bringup" && _max=150
    # Wait for all PIDs in parallel rather than sequentially
    local _wait_pids=()
    for _p in $pids; do
        (
            for _w in $(seq 1 $_max); do
                kill -0 "$_p" 2>/dev/null || exit 0
                sleep 0.1
            done
            kill -0 "$_p" 2>/dev/null && kill -KILL "$_p" 2>/dev/null || true
        ) &
        _wait_pids+=($!)
    done
    wait "${_wait_pids[@]}" 2>/dev/null || true
}

# Block until all insta360_capture processes have fully exited.
_wait_camera_dead() {
    # Skip entirely if no camera processes are running
    if ! pgrep -f "insta360_capture" > /dev/null 2>&1; then
        return 0
    fi
    local _usb_dev
    _usb_dev=$(readlink -f /dev/insta 2>/dev/null)
    # Primary gate: fuser on the USB device node — cleared only when the last
    # process releases its libusb fd, which happens after Close() completes.
    if [ -n "$_usb_dev" ]; then
        for _i in $(seq 1 60); do
            fuser "$_usb_dev" > /dev/null 2>&1 || break
            sleep 0.5
        done
        if fuser "$_usb_dev" > /dev/null 2>&1; then
            echo "  Force-killing USB device holders..."
            fuser "$_usb_dev" 2>/dev/null | tr ' ' '\n' | grep -v '^$' | xargs -r kill -KILL 2>/dev/null || true
            for _i in $(seq 1 20); do
                fuser "$_usb_dev" > /dev/null 2>&1 || break
                sleep 0.5
            done
        fi
    else
        # No /dev/insta — fall back to pgrep
        for _i in $(seq 1 20); do
            pgrep -f "insta360_capture" > /dev/null 2>&1 || break
            sleep 1
        done
        pgrep -f "insta360_capture" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true
    fi
    sleep 1
}

# After killing the driver, the Insta360 firmware may still be in streaming
# mode for up to ~30s. Poll a fresh driver instance to detect when the firmware
# has reset (sync succeeds) before handing control back to the caller.
_wait_firmware_ready() {
    # Clear sentinel if it predates the current boot
    if [ -f /tmp/.insta360_session_ran ]; then
        _boot_time=$(date -d "$(uptime -s)" +%s 2>/dev/null || echo 0)
        _sentinel_time=$(stat -c %Y /tmp/.insta360_session_ran 2>/dev/null || echo 0)
        [ "$_sentinel_time" -lt "$_boot_time" ] && rm -f /tmp/.insta360_session_ran
    fi
    [ ! -f /tmp/.insta360_session_ran ] && return 0
    echo "Previous SDK session detected — force-resetting camera to clear firmware state..."
    # A USB reset forces the firmware to restart its HTTP command server,
    # clearing any stuck timelapse or capture state from the previous session.
    _usb_force_reset_camera
    echo "✓ Camera firmware ready"
}

# USB-level reset — clears any firmware streaming state from an unclean shutdown.
# Only resets if the device is in a bad state (bConfigurationValue=0/empty),
# because USBDEVFS_RESET on a configured ONE X2 causes it to shut down rather
# than re-enumerate, requiring a physical button press.
_usb_reset_camera() {
    [ -e /dev/insta ] || return 0
    local _dev _busdev _cfg
    _dev=$(readlink -f /dev/insta 2>/dev/null) || return 0
    _busdev=$(udevadm info --query=path --name="$_dev" 2>/dev/null \
        | grep -oP '[0-9]+-[0-9.]+$')
    _cfg=$(cat /sys/bus/usb/devices/$_busdev/bConfigurationValue 2>/dev/null | tr -d '[:space:]')
    if [ -n "$_cfg" ] && [ "$_cfg" != "0" ]; then
        echo "✓ Camera USB device already configured (bConfigurationValue=$_cfg), skipping reset"
        return 0
    fi
    echo "Resetting USB device ($_busdev)..."
    sudo python3 -c "
import fcntl, struct, sys
USBDEVFS_RESET = 0x5514
try:
    with open('$_dev', 'wb') as f:
        fcntl.ioctl(f, USBDEVFS_RESET, 0)
except Exception as e:
    sys.exit(1)
" 2>/dev/null || true
    for _i in $(seq 1 15); do
        [ -e /dev/insta ] || { sleep 1; continue; }
        [ -d "/sys/bus/usb/devices/$_busdev" ] || { sleep 1; continue; }
        _cfg=$(cat /sys/bus/usb/devices/$_busdev/bConfigurationValue 2>/dev/null | tr -d '[:space:]')
        [ -n "$_cfg" ] && [ "$_cfg" != "0" ] && break
        sleep 1
    done
    _cfg=$(cat /sys/bus/usb/devices/$_busdev/bConfigurationValue 2>/dev/null | tr -d '[:space:]')
    if [ -n "$_cfg" ] && [ "$_cfg" != "0" ]; then
        sleep 2
        return 0
    fi
    echo "  ⚠ USB reset could not enumerate device (bConfigurationValue='$_cfg') — driver may fail to connect"
}

# Force USB reset regardless of bConfigurationValue — used when the firmware's
# HTTP command server is unresponsive (stuck in timelapse state from a previous
# session). Unlike _usb_reset_camera, does not skip when configured=1.
_usb_force_reset_camera() {
    [ -e /dev/insta ] || return 0
    local _dev _busdev _cfg
    _dev=$(readlink -f /dev/insta 2>/dev/null) || return 0
    _busdev=$(udevadm info --query=path --name="$_dev" 2>/dev/null \
        | grep -oP '[0-9]+-[0-9.]+$')
    echo "Force-resetting camera USB device ($_busdev) to clear firmware state..."
    sudo python3 -c "
import fcntl, struct, sys
USBDEVFS_RESET = 0x5514
try:
    with open('$_dev', 'wb') as f:
        fcntl.ioctl(f, USBDEVFS_RESET, 0)
except Exception as e:
    sys.exit(1)
" 2>/dev/null || true
    for _i in $(seq 1 20); do
        [ -e /dev/insta ] || { sleep 1; continue; }
        [ -d "/sys/bus/usb/devices/$_busdev" ] || { sleep 1; continue; }
        _cfg=$(cat /sys/bus/usb/devices/$_busdev/bConfigurationValue 2>/dev/null | tr -d '[:space:]')
        [ -n "$_cfg" ] && [ "$_cfg" != "0" ] && break
        sleep 1
    done
    sleep 3
    echo "✓ Camera USB reset complete"
}
# Kill all stale processes in parallel — capture PIDs so wait only blocks on these
_pkill "livox_ros_driver2" & _kpids=($!)
_pkill "rko_lio" & _kpids+=($!)
_pkill "insta360_capture" & _kpids+=($!)
_pkill "static_transform_publisher" & _kpids+=($!)
_pkill "ros2 bag record" & _kpids+=($!)
_pkill "continuous_trajectory_recorder" & _kpids+=($!)
_pkill "imu_frame" & _kpids+=($!)
_pkill "imu_stabilized" & _kpids+=($!)
wait "${_kpids[@]}"
_wait_camera_dead
_wait_firmware_ready
# Wipe all FastDDS SHM files now that every process is confirmed dead.
# This must happen after _wait_camera_dead, not before — the decoder holds
# _el lock files open until it exits, so an earlier rm just gets recreated.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
# Kill the ros2 CLI daemon — it holds SHM ports open between sessions
ros2 daemon stop 2>/dev/null || true
pkill -f "ros2-daemon" 2>/dev/null || true

# Camera permissions
if [ "$SKIP_SUDO_CHECK" != "1" ]; then
    "$SCRIPT_DIR/setup_camera_permissions.sh"
fi

# Always reset the USB device at startup — clears firmware streaming state from
# a previous unclean session and ensures bConfigurationValue is set before the
# driver attempts to claim the interface.
_usb_reset_camera

# Wait for /dev/insta — may still be re-enumerating after USB reset
for _i in $(seq 1 15); do
    [ -e /dev/insta ] && break
    echo "Waiting for /dev/insta... (attempt $_i/15)"
    sleep 1
done
if [ ! -e /dev/insta ]; then
    echo "✗ Camera device not found at /dev/insta"
    exit 1
fi

# Verify camera is in Android USB mode by checking for a network interface
# or RNDIS-class USB interface. If absent, the SDK cannot connect.
_cam_sysfs=$(find /sys/bus/usb/devices/ -name "idVendor" 2>/dev/null | \
    xargs grep -l "2e1a" 2>/dev/null | head -1 | xargs dirname 2>/dev/null)
if [ -n "$_cam_sysfs" ]; then
    _cam_class=$(cat "$_cam_sysfs/bDeviceClass" 2>/dev/null | tr -d '[:space:]')
    _has_rndis=false
    for _iface in "$_cam_sysfs"/*:*; do
        [ -d "$_iface" ] || continue
        _ic=$(cat "$_iface/bInterfaceClass" 2>/dev/null | tr -d '[:space:]')
        if [ "$_ic" = "02" ] || [ "$_ic" = "e0" ] || [ "$_ic" = "0a" ]; then
            _has_rndis=true
            break
        fi
    done
    if [ "$_has_rndis" = "false" ]; then
        echo "⚠ Camera USB mode may not be set to Android (no RNDIS/CDC interface found)."
        echo "  If streaming fails: on the camera go to Settings → General → USB Mode → Android"
        echo "  then reconnect the USB cable."
    fi
fi

echo "Starting LiDAR driver..."
# Pin Livox driver to cores 0-1 so its UDP receive thread is isolated
# from the bag recorders on cores 2-3, preventing IMU packet drops.
taskset -c 0,1 setsid ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
{ sleep 3 && renice -n -5 -p $LIDAR_PID 2>/dev/null 1>/dev/null; } &
LIDAR_PGID=$(ps -o pgid= -p $LIDAR_PID 2>/dev/null | tr -d ' ')
PIDS+=($LIDAR_PID)

echo "Waiting for topic /livox/lidar..."
_lidar_timeout=60; _lidar_count=0
while [ $_lidar_count -lt $_lidar_timeout ]; do
    if grep -q "livox/lidar publish" /tmp/lidar.log 2>/dev/null; then
        echo "✓ Topic /livox/lidar is available"; break
    fi
    kill -0 $LIDAR_PID 2>/dev/null || { echo "✗ LiDAR driver exited unexpectedly"; exit 1; }
    sleep 1; _lidar_count=$((_lidar_count + 1))
done
[ $_lidar_count -ge $_lidar_timeout ] && { echo "✗ Timeout waiting for topic /livox/lidar"; echo "Failed to start LiDAR driver"; exit 1; }
# Poll for first LiDAR data instead of a fixed warmup sleep
echo "Waiting for first LiDAR data..."
_lidar_data_wait=0
while [ $_lidar_data_wait -lt 10 ]; do
    grep -q "livox/lidar publish" /tmp/lidar.log 2>/dev/null && break
    sleep 1; _lidar_data_wait=$((_lidar_data_wait + 1))
done

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame > /tmp/tf.log 2>&1 &
TF_PID=$!
PIDS+=($TF_PID)
echo "✓ LiDAR driver ready"

# ─── IMU / RKO-LIO ────────────────────────────────────────────────────────────
IMU_AVAILABLE=false
IMU_SOURCE="none"

if grep -q "livox/imu publish" /tmp/lidar.log 2>/dev/null; then
    IMU_AVAILABLE=true; IMU_SOURCE="livox"
    echo "✓ Livox IMU available"
fi

if [ "$IMU_AVAILABLE" = "false" ]; then
    if ros2 topic list 2>/dev/null | grep -q "/imu/data_raw"; then
        IMU_AVAILABLE=true; IMU_SOURCE="camera"
        echo "✓ Camera IMU available"
    fi
fi

if [ "$IMU_AVAILABLE" = "true" ]; then
    echo "Starting RKO-LIO with $IMU_SOURCE IMU..."

    if [ "$IMU_SOURCE" = "camera" ]; then
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/init/imu_transform_node.py" > /tmp/imu_transform.log 2>&1 &
        PIDS+=($!)
        if [ "$CAMERA_MODE" = "dual_fisheye" ]; then
            ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera livox_frame > /tmp/camera_tf.log 2>&1 &
        else
            # RPY (rad): -3.1147, 0.0352, -1.6088  =>  quat xyzw: -0.6932, 0.7204, 0.0025, 0.0220
            ros2 run tf2_ros static_transform_publisher \
                0 0 0 -0.6932 0.7204 0.0025 0.0220 camera livox_frame > /tmp/camera_tf.log 2>&1 &
        fi
        PIDS+=($!)
        sleep 2
    fi

    > /tmp/rko_lio.log
    _RKO_CONFIG="rko_lio_config_robust.yaml"
    [ "$CAPTURE_MODE" = "continuous" ] && _RKO_CONFIG="rko_lio_config_continuous.yaml"
    ros2 launch rko_lio odometry.launch.py \
        config_file:="$ROS_WS_DIR/src/atlas-scanner/src/config/$_RKO_CONFIG" \
        rviz:=false > /tmp/rko_lio.log 2>&1 &
    LIO_PID=$!
    sleep 1 && renice -n -15 -p $LIO_PID 2>/dev/null 1>/dev/null &
    PIDS+=($LIO_PID)
    _rlio_count=0
    while [ $_rlio_count -lt 20 ]; do
        grep -q "RKO LIO Node is up" /tmp/rko_lio.log 2>/dev/null && break
        kill -0 $LIO_PID 2>/dev/null || { echo "⚠ RKO-LIO exited unexpectedly"; break; }
        sleep 1; _rlio_count=$((_rlio_count + 1))
    done

    if grep -q "RKO LIO Node is up" /tmp/rko_lio.log 2>/dev/null; then
        echo "✓ RKO-LIO ready with $IMU_SOURCE IMU"
        LIO_ENABLED="true"
        if [ -n "${DISPLAY:-}" ]; then
            env -u QT_QPA_PLATFORM QT_QPA_PLATFORM=xcb rviz2 -d "$ROS_WS_DIR/src/atlas-scanner/src/config/atlas_display.rviz" \
                -stylesheet "$ROS_WS_DIR/src/atlas-scanner/src/config/rviz_kiosk.qss" > /tmp/rviz.log 2>&1 &
            RVIZ_PID=$!
            # Not added to PIDS — RViz crash should not terminate the session
            # Give RViz higher priority than camera pipeline in dual_fisheye mode
            [ "$CAMERA_MODE" = "dual_fisheye" ] && sleep 2 && renice -n 5 -p $RVIZ_PID 2>/dev/null 1>/dev/null &
        else
            echo "⚠ No display available — skipping RViz"
        fi
    else
        echo "⚠ RKO-LIO failed to start"
        LIO_ENABLED="false"
    fi
else
    echo "⚠ No IMU available - RKO-LIO disabled"
    LIO_ENABLED="false"
fi

# ─── Trajectory recorder (start before camera so no delay after camera is live) ────
if [ "$LIO_ENABLED" = "true" ]; then
    TRAJ_PID_FILE="$SCAN_DIR/.trajectory_recorder.pid"
    _pkill "enhanced_trajectory_recorder"
    _pkill "buffered_odom_bridge"
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/enhanced_trajectory_recorder.py" \
        "$SCAN_DIR" > "$SCAN_DIR/trajectory_recorder.log" 2>&1 &
    TRAJECTORY_PID=$!
    echo "$TRAJECTORY_PID" > "$TRAJ_PID_FILE"
    PIDS+=($TRAJECTORY_PID)
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_odom_bridge.py" \
        > "$SCAN_DIR/odom_buffer.log" 2>&1 &
    PIDS+=($!)
    TRAJECTORY_RECORDING=true
fi

# ─── Camera driver ────────────────────────────────────────────────────────────
# Started last so the Insta360 SDK streaming timeout doesn't expire during
# the LiDAR + RKO-LIO startup sequence (~50s).
# ─── Camera driver ────────────────────────────────────────────────────────────
# In SDK stitch mode the CameraSDK owns the USB device for the whole session
# — no ROS driver needed. The driver is only required for stream-based capture.
# SDK capture daemon
    echo "SDK stitch mode: skipping camera driver (SDK owns USB device)"
    CAMERA_STATUS="SDK (insta360_capture)"
    touch /tmp/.insta360_session_ran
    # Wait for any previous SDK daemon to fully release the USB device
    _wait_camera_dead
    # Start the persistent SDK session daemon
    export INSTA360_INTERVAL_MS=$(( CONTINUOUS_INTERVAL * 1000 ))
    INSTA360_SESSION_DIR="$SCAN_DIR" \
        ~/insta360-dev/build/insta360_capture > "$SCAN_DIR/sdk_capture.log" 2>&1 &
    SDK_CAPTURE_PID=$!
    # Not added to PIDS — cleanup must not kill the daemon before StopTimeLapse runs.
    # The shell waits for it to exit naturally after the download drain below.
    # Wait for the daemon to signal ready
    echo "Waiting for SDK camera session..."
    for _i in $(seq 1 120); do
        [ -f "$SCAN_DIR/.sdk_ready" ] && break
        if ! kill -0 $SDK_CAPTURE_PID 2>/dev/null; then
            echo "✗ SDK capture daemon exited — check $SCAN_DIR/sdk_capture.log"
            exit 1
        fi
        sleep 1
    done
    if [ ! -f "$SCAN_DIR/.sdk_ready" ]; then
        echo "✗ SDK camera session timed out — force-resetting camera and retrying once..."
        kill -KILL $SDK_CAPTURE_PID 2>/dev/null || true
        wait $SDK_CAPTURE_PID 2>/dev/null || true
        _usb_force_reset_camera
        rm -f "$SCAN_DIR/.sdk_ready"
        # One retry after force reset
        INSTA360_SESSION_DIR="$SCAN_DIR" \
            ~/insta360-dev/build/insta360_capture >> "$SCAN_DIR/sdk_capture.log" 2>&1 &
        SDK_CAPTURE_PID=$!
        for _i in $(seq 1 60); do
            [ -f "$SCAN_DIR/.sdk_ready" ] && break
            if ! kill -0 $SDK_CAPTURE_PID 2>/dev/null; then
                echo "✗ SDK capture daemon exited on retry"; exit 1
            fi
            sleep 1
        done
        if [ ! -f "$SCAN_DIR/.sdk_ready" ]; then
            echo "✗ SDK camera session timed out after force reset"; exit 1
        fi
    fi
    echo "✓ SDK camera session ready"
FUSION_READY="true"

echo ""
echo "=========================================="
echo "FUSION CAPTURE READY"
echo "  Camera mode : $CAMERA_MODE"
echo "  Capture mode: $CAPTURE_MODE"
echo "  LiDAR       : /livox/lidar"
echo "  Camera      : $CAMERA_STATUS"
[ "$LIO_ENABLED" = "true" ] && echo "  Odometry    : RKO-LIO ($IMU_SOURCE IMU)"
echo "=========================================="

# ─── Capture loop ─────────────────────────────────────────────────────────────
if [ "$CAPTURE_MODE" = "continuous" ]; then
    echo ""
    echo "=========================================="
    echo "CONTINUOUS MODE: recording session bag"
    echo "Press 'q' + ENTER to stop"
    echo "=========================================="

    ROSBAG_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    ROSBAG_DIR="$SCAN_DIR/rosbag_$ROSBAG_TIMESTAMP"
    # Main bag: LiDAR + camera + odometry on cores 2-3
    CONTINUOUS_TOPICS="/livox/lidar /camera/shutter_time"
    [ "$LIO_ENABLED" = "true" ] && CONTINUOUS_TOPICS="$CONTINUOUS_TOPICS /rko_lio/odometry"
    # shellcheck disable=SC2086
    taskset -c 2,3 nice -n 10 ros2 bag record -o "$ROSBAG_DIR" --max-cache-size 100000000 \
        $CONTINUOUS_TOPICS > "$SCAN_DIR/rosbag_main.log" 2>&1 &
    ROSBAG_PID=$!
    PIDS+=($ROSBAG_PID)

    # IMU bag: separate high-priority recorder on core 3 so IMU topics
    # are never starved by LiDAR/image write bursts in the main bag.
    IMU_TOPICS="/livox/imu"
    ros2 topic list 2>/dev/null | grep -q "/imu/data$"   && IMU_TOPICS="$IMU_TOPICS /imu/data"
    ros2 topic list 2>/dev/null | grep -q "/imu/data_raw" && IMU_TOPICS="$IMU_TOPICS /imu/data_raw"
    # shellcheck disable=SC2086
    taskset -c 3 nice -n 5 ros2 bag record -o "${ROSBAG_DIR}_imu" --max-cache-size 20000000 \
        $IMU_TOPICS > "$SCAN_DIR/rosbag_imu.log" 2>&1 &
    IMU_BAG_PID=$!
    PIDS+=($IMU_BAG_PID)
    echo "✓ IMU bag recording started: ${ROSBAG_DIR}_imu"
    echo "✓ Rosbag recording started: $ROSBAG_DIR"
    echo "  Press 'q' + ENTER when scanning is complete."

    rm -f "$SCAN_DIR/.sdk_capture_done" "$SCAN_DIR/.sdk_capture_failed"
    echo "$SCAN_DIR" > "$SCAN_DIR/.sdk_capture_trigger"
    for _i in $(seq 1 100); do
        [ -f "$SCAN_DIR/.sdk_capture_done" ] && break
        [ -f "$SCAN_DIR/.sdk_capture_failed" ] && { echo "✗ SDK timelapse failed to start"; exit 1; }
        sleep 0.1
    done
    rm -f "$SCAN_DIR/.sdk_capture_done"
    echo "✓ SDK continuous capture started (interval=${CONTINUOUS_INTERVAL}s)"
    # Start shutter event publisher — watches for capture_N.shutter_event files
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/shutter_event_publisher.py" \
        "$SCAN_DIR" > "$SCAN_DIR/shutter_publisher.log" 2>&1 &
    PIDS+=($!)
    echo "✓ Shutter event publisher started"

    while true; do
        if [ ! -r /dev/tty ] || ! { true < /dev/tty; } 2>/dev/null; then
            # GUI mode: poll for quit trigger file
            [ -f "$SCAN_DIR/.quit_trigger" ] && rm -f "$SCAN_DIR/.quit_trigger" && break
            # Stop automatically if the SDK daemon crashed
            if [ -n "${SDK_CAPTURE_PID:-}" ] && ! kill -0 "$SDK_CAPTURE_PID" 2>/dev/null; then
                echo "⚠ SDK capture daemon exited unexpectedly — stopping session"
                break
            fi
            sleep 0.2
        else
            read -r _input < /dev/tty 2>/dev/null
            [ "$_input" = "q" ] || [ "$_input" = "quit" ] && break
        fi
    done
    touch "$SCAN_DIR/.session_done"  # signal watchdog/trigger loop to exit

    # For SDK stitch mode: wait for all background downloads to finish
    # before stopping the bag recorder. The daemon writes .sdk_downloads_pending
    # with the outstanding count; poll until it reaches 0.
    if [ -n "${SDK_CAPTURE_PID:-}" ]; then
        echo "Waiting for SDK to stop timelapse and complete downloads..."
        _sdk_wait=0
        while [ $_sdk_wait -lt 300 ]; do
            # Daemon exits naturally after StopTimeLapse + drain completes
            kill -0 $SDK_CAPTURE_PID 2>/dev/null || break
            sleep 1; _sdk_wait=$((_sdk_wait + 1))
        done
        # Hard kill only if daemon is still alive after 5 min
        kill -0 $SDK_CAPTURE_PID 2>/dev/null && kill -KILL $SDK_CAPTURE_PID 2>/dev/null || true
        wait $SDK_CAPTURE_PID 2>/dev/null || true
        echo "✓ SDK capture daemon exited"
    fi

else
    # ── Stationary mode ──────────────────────────────────────────────────────
    # Use /dev/tty if available (interactive terminal), otherwise use GUI trigger file mode
    _GUI_MODE=false
    if [ ! -r /dev/tty ] || ! { true < /dev/tty; } 2>/dev/null; then
        _GUI_MODE=true
        _TTY=/dev/null
        echo "GUI mode: waiting for trigger files in $SCAN_DIR"
    else
        _TTY=/dev/tty
    fi

    # Flush any buffered newlines left in stdin
    [ "$_GUI_MODE" = "false" ] && while read -r -t 0.1 _flush < "$_TTY"; do :; done 2>/dev/null

    while true; do
        if [ "$_GUI_MODE" = "true" ]; then
            # Wait for GUI to write a trigger file: .capture_trigger (scan) or .quit_trigger (stop)
            while true; do
                if [ -f "$SCAN_DIR/.quit_trigger" ]; then
                    rm -f "$SCAN_DIR/.quit_trigger"
                    break 2
                fi
                if [ -f "$SCAN_DIR/.capture_trigger" ]; then
                    rm -f "$SCAN_DIR/.capture_trigger"
                    input=""
                    break
                fi
                sleep 0.2
            done
        else
            printf 'Scan %d - Press ENTER to capture (or q to exit): ' "$((SCAN_COUNT + 1))" > /dev/tty 2>/dev/null
            read input < "$_TTY" 2>/dev/null
            [ "$input" = "quit" ] || [ "$input" = "q" ] && break
        fi

        SCAN_COUNT=$((SCAN_COUNT + 1))
        SCAN_NAME="fusion_scan_$(printf "%03d" $SCAN_COUNT)"
        INDIVIDUAL_SCAN_DIR="$SCAN_DIR/$SCAN_NAME"
        mkdir -p "$INDIVIDUAL_SCAN_DIR"

        # Verify LiDAR is alive and actively publishing.
        # Avoid ros2 topic info/echo — both spawn a temporary DDS participant whose
        # discovery multicast can flood the MID360 UDP path and kill the driver.
        # Instead check: (1) driver process is alive, (2) log shows it is publishing.
        _lidar_ok=false
        if kill -0 $LIDAR_PID 2>/dev/null && grep -q "livox/lidar publish" /tmp/lidar.log 2>/dev/null; then
            _lidar_ok=true
        fi
        if [ "$_lidar_ok" = "false" ]; then
            echo "⚠ LiDAR driver died, restarting..."
            rm -f /tmp/lidar.log
            setsid ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar.log 2>&1 &
            LIDAR_PID=$!
            LIDAR_PGID=$(ps -o pgid= -p $LIDAR_PID 2>/dev/null | tr -d ' ')
            PIDS+=($LIDAR_PID)
            _w=0
            while [ $_w -lt 15 ]; do
                grep -q "livox/lidar publish" /tmp/lidar.log 2>/dev/null && break
                sleep 1; _w=$((_w+1))
            done
            if ! grep -q "livox/lidar publish" /tmp/lidar.log 2>/dev/null; then
                echo "✗ LiDAR driver failed to restart, skipping scan"
                SCAN_COUNT=$((SCAN_COUNT - 1)); continue
            fi
            echo "✓ LiDAR driver restarted"
            # RKO-LIO loses its input when the LiDAR driver dies — restart it too.
            if [ "$LIO_ENABLED" = "true" ]; then
                echo "Restarting RKO-LIO after LiDAR restart..."
                _pkill "rko_lio"
                sleep 1
                > /tmp/rko_lio.log
                ros2 launch rko_lio odometry.launch.py \
                    config_file:="$ROS_WS_DIR/src/atlas-scanner/src/config/rko_lio_config_robust.yaml" \
                    rviz:=false > /tmp/rko_lio.log 2>&1 &
                LIO_PID=$!
                sleep 1 && renice -n -15 -p $LIO_PID 2>/dev/null 1>/dev/null &
                PIDS+=($LIO_PID)
                _rlio_w=0
                while [ $_rlio_w -lt 20 ]; do
                    grep -q "RKO LIO Node is up" /tmp/rko_lio.log 2>/dev/null && break
                    kill -0 $LIO_PID 2>/dev/null || break
                    sleep 1; _rlio_w=$((_rlio_w + 1))
                done
                if grep -q "RKO LIO Node is up" /tmp/rko_lio.log 2>/dev/null; then
                    echo "✓ RKO-LIO restarted"
                else
                    echo "⚠ RKO-LIO failed to restart — odometry unavailable for this scan"
                    LIO_ENABLED="false"
                fi
            fi
            sleep 3
        fi

        # Optional settle wait before starting the rosbag — gives the scanner
        # time to stop moving after the trigger before recording begins.
        if [ "$STATIONARY_WAIT" = "true" ]; then
            echo "Waiting 3s for scanner to settle..."
            sleep 3
        fi

        # Start per-scan rosbag immediately before capture so idle setup time
        # between scans is not recorded — keeps bag sizes proportional to the
        # ~4s capture window rather than the full inter-scan gap.
        _t0=$EPOCHREALTIME
        ROSBAG_TIMESTAMP=$(date +%Y%m%d_%H%M%S_%N | cut -c1-18)
        ROSBAG_DIR="$INDIVIDUAL_SCAN_DIR/rosbag_$ROSBAG_TIMESTAMP"
        TOPICS_TO_RECORD="/livox/lidar /livox/imu"
        # Only record camera stream if not using SDK stitch — with SDK stitch
        # the .insp file is the authoritative image and the stream is never used
        [ "$LIO_ENABLED" = "true" ] && TOPICS_TO_RECORD="$TOPICS_TO_RECORD /rko_lio/odometry"
        setsid env FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_rosbag.xml" \
            ros2 bag record -o "$ROSBAG_DIR" \
            --compression-mode file --compression-format zstd \
            --max-cache-size 100000000 $TOPICS_TO_RECORD > "$ROSBAG_DIR.log" 2>&1 &
        ROSBAG_PID=$!
        # Wait briefly for setsid to establish the new process group before reading PGID
        sleep 0.1
        ROSBAG_PGID=$(ps -o pgid= -p $ROSBAG_PID 2>/dev/null | tr -d ' ')
        # Safety check: never send kill to PGID 0 or the lidar driver's group
        [ -z "$ROSBAG_PGID" ] || [ "$ROSBAG_PGID" = "0" ] || [ "$ROSBAG_PGID" = "$LIDAR_PGID" ] && ROSBAG_PGID=""
        # Wait for bag recorder to subscribe to all topics (max 5s)
        _bag_wait=0
        while [ $_bag_wait -lt 10 ]; do
            grep -q "All requested topics are subscribed" "$ROSBAG_DIR.log" 2>/dev/null && break
            sleep 0.5; _bag_wait=$((_bag_wait + 1))
        done

        echo "Capturing scan $SCAN_COUNT..."
        sleep 2

        # Signal the SDK daemon to capture — .sdk_capture_done fires at shutter click
        rm -f "$SCAN_DIR/.sdk_capture_done" "$SCAN_DIR/.sdk_capture_failed"
        echo "$INDIVIDUAL_SCAN_DIR" > "$SCAN_DIR/.sdk_capture_trigger"
        for _i in $(seq 1 300); do
            [ -f "$SCAN_DIR/.sdk_capture_done" ] && { CAPTURE_EXIT_CODE=0; break; }
            [ -f "$SCAN_DIR/.sdk_capture_failed" ] && { CAPTURE_EXIT_CODE=1; break; }
            kill -0 $SDK_CAPTURE_PID 2>/dev/null || { echo "  SDK daemon died"; CAPTURE_EXIT_CODE=1; break; }
            sleep 0.1
        done
        [ -z "$CAPTURE_EXIT_CODE" ] && { echo "  SDK capture timed out"; CAPTURE_EXIT_CODE=1; }
        rm -f "$SCAN_DIR/.sdk_capture_done" "$SCAN_DIR/.sdk_capture_failed"
        if [ $CAPTURE_EXIT_CODE -eq 0 ]; then
            _insp=$(ls "$INDIVIDUAL_SCAN_DIR"/*.insp 2>/dev/null | head -1)
            [ -n "$_insp" ] && echo "  ✓ SDK raw file: $(basename "$_insp") (stitch in post-processing)"
            # Collect LiDAR point cloud from live topics into sensor_lidar_*.ply / world_lidar_*.ply
            FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_capture.xml" \
            ROS_DISABLE_LOANED_MESSAGES=1 \
            timeout 25 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_camera_capture.py" \
                "$INDIVIDUAL_SCAN_DIR" 15 2.0 --lidar-only || echo "  ⚠ LiDAR capture failed"
        fi


        # Stop rosbag — send SIGINT to the setsid process group so zstd compressor flushes cleanly
        if kill -0 $ROSBAG_PID 2>/dev/null; then
            if [ -n "$ROSBAG_PGID" ] && [ "$ROSBAG_PGID" != "$LIDAR_PGID" ]; then
                kill -INT -$ROSBAG_PGID 2>/dev/null
            else
                kill -INT $ROSBAG_PID
            fi
            for _i in $(seq 1 20); do
                kill -0 $ROSBAG_PID 2>/dev/null || break
                sleep 0.5
            done
            if kill -0 $ROSBAG_PID 2>/dev/null; then
                [ -n "$ROSBAG_PGID" ] && kill -KILL -$ROSBAG_PGID 2>/dev/null || kill -KILL $ROSBAG_PID 2>/dev/null
            fi
            wait $ROSBAG_PID 2>/dev/null || true
        fi
        # Wait for zstd compressor to finish flushing after the bag recorder exits.
        # With --compression-mode file, files are named *.db3.zstd (no plain .db3).
        # Poll all .zstd files until they stop growing.
        for _zstd in "$ROSBAG_DIR"/*.zstd; do
            [ -f "$_zstd" ] || continue
            _prev=0; _stable=0
            for _si in $(seq 1 120); do
                _cur=$(stat -c%s "$_zstd" 2>/dev/null || echo 0)
                if [ "$_cur" = "$_prev" ] && [ "$_cur" -gt 0 ]; then
                    _stable=$((_stable + 1))
                    [ "$_stable" -ge 3 ] && break
                else
                    _stable=0
                fi
                _prev=$_cur; sleep 0.5
            done
        done
        # Also remove any leftover uncompressed .db3 files
        for _db3 in "$ROSBAG_DIR"/*.db3; do
            [ -f "$_db3" ] || continue
            _zstd="${_db3}.zstd"
            [ -s "$_zstd" ] && rm -f "$_db3"
        done

        if [ $CAPTURE_EXIT_CODE -ne 0 ]; then
            echo "✗ Capture failed"
            SCAN_COUNT=$((SCAN_COUNT - 1)); continue
        fi

        # Trajectory trigger
        if [ "$LIO_ENABLED" = "true" ] && [ "$TRAJECTORY_RECORDING" = "true" ]; then
            CAPTURE_TIMESTAMP=$(python3 -c "import time; print(time.time())")
            echo "{\"scan_name\": \"$SCAN_NAME\", \"scan_dir\": \"$INDIVIDUAL_SCAN_DIR\", \"capture_time\": $CAPTURE_TIMESTAMP}" \
                > "$SCAN_DIR/.save_trajectory_${SCAN_NAME}"
            echo "✓ Trajectory trigger saved"
        fi

        # Rename PLY to canonical names
        WORLD_PLY=$(ls -t "$INDIVIDUAL_SCAN_DIR"/world_lidar_*.ply 2>/dev/null | head -1)
        SENSOR_PLY=$(ls -t "$INDIVIDUAL_SCAN_DIR"/sensor_lidar_*.ply 2>/dev/null | head -1)
        [ -n "$WORLD_PLY" ]  && mv "$WORLD_PLY"  "$INDIVIDUAL_SCAN_DIR/world_lidar.ply"
        [ -n "$SENSOR_PLY" ] && mv "$SENSOR_PLY" "$INDIVIDUAL_SCAN_DIR/sensor_lidar.ply"

        echo "✓ Scan $SCAN_COUNT completed: $SCAN_NAME/"
        echo ""
    done
    touch "$SCAN_DIR/.session_done"  # signal stationary watchdog to exit
fi

if [ "$ENABLE_POST_PROCESSING_BAGS" = "true" ]; then
    read -p "Run post-processing on all bag files? (y/n): " post_process < "$_TTY" 2>/dev/null
    if [ "$post_process" = "y" ] || [ "$post_process" = "Y" ]; then
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_bags.py" "$SCAN_DIR"
    fi
fi
