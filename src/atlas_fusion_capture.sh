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
CAMERA_MODE="single_fisheye"      # dual_fisheye | single_fisheye
CAPTURE_MODE="stationary"         # stationary | continuous
CONTINUOUS_INTERVAL=3             # seconds between captures (continuous mode only)
STATIONARY_WAIT=true              # stationary only: wait 3s before starting rosbag (allows scanner to settle)

# Allow CLI overrides: atlas_fusion_capture.sh [--camera dual_fisheye|single_fisheye] [--capture stationary|continuous]
while [[ $# -gt 0 ]]; do
    case $1 in
        --camera) CAMERA_MODE="$2"; shift 2 ;;
        --capture) CAPTURE_MODE="$2"; shift 2 ;;
        --interval) CONTINUOUS_INTERVAL="$2"; shift 2 ;;
        --bag-only) BAG_ONLY=true; shift ;;
        --no-sync-benchmark) RUN_SYNC_BENCHMARK=false; shift ;;
        --no-stationary-wait) STATIONARY_WAIT=false; shift ;;
        *) shift ;;
    esac
done

BAG_ONLY=${BAG_ONLY:-false}

SAVE_E57=false
USE_EXISTING_CALIBRATION=false    # if true, won't reload calibration from ~/atlas_ws/output/calib.json
ENABLE_ICP_ALIGNMENT=true
EXPORT_COLMAP=false
BLEND_ERP_SEAMS=true              # dual_fisheye only: blend fisheye seams in ERP images
CLEAN_POINTCLOUD=true             # statistical outlier removal on merged cloud
DOWNSAMPLE_VOXEL_SIZE=0.03        # voxel downsample in metres (0 = skip)
RUN_SYNC_BENCHMARK=true

ENABLE_POST_PROCESSING_BAGS=false
SKIP_LIVE_FUSION=true
AUTO_CREATE_COLORED=true

# ──────────────────────────────────────────────────────────────────────────────

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
SCAN_DIR="data/synchronized_scans/sync_fusion_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SCAN_DIR"

# Tee all stdout+stderr to a session log
LOG_FILE="$SCAN_DIR/session.log"
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== Session log started: $(date) ==="

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
    for _cam_pid in $(pgrep -f "insta360_ros_driver" 2>/dev/null); do
        kill -INT "$_cam_pid" 2>/dev/null
    done
    for _w in $(seq 1 150); do
        pgrep -f "insta360_ros_driver" > /dev/null 2>&1 || break
        sleep 0.1
    done
    pgrep -f "insta360_ros_driver" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true

    # Kill all other tracked processes (excluding camera driver already handled above)
    for pid in "${PIDS[@]}"; do
        pgrep -f "insta360_ros_driver" 2>/dev/null | grep -qw "$pid" && continue
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
    _pkill "insta360_ros_driver"
    _pkill "insta360_ros_driver/lib/insta360_ros_driver/decoder"
    _pkill "image_decoder"
    _pkill "equirectangular"
    _pkill "dual_fisheye"
    _pkill "bringup.launch"
    _pkill "livox_ros_driver2"
    _pkill "rko_lio"
    _pkill "static_transform_publisher"
    _pkill "ros2 bag record"
    _pkill "topic_tools"
    _pkill "continuous_trajectory_recorder"
    _pkill "ros2 launch.*insta360"
    _pkill "python.*equirectangular"

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
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py" \
                "$SCAN_DIR" --interval "$CONTINUOUS_INTERVAL" --lidar-window 1.5 --camera-mode "$CAMERA_MODE" --max-gyro 0.3 --trim-ends 2
            echo "Generating masked images..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/regenerate_masked_images.py" "$SCAN_DIR" --camera-mode "$CAMERA_MODE"
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
            if [ "$CAMERA_MODE" = "dual_fisheye" ]; then
                echo "Converting fisheye images to ERP..."
                for scan_dir in "$SCAN_DIR"/fusion_scan_*; do
                    [ -d "$scan_dir" ] || continue
                    BAG_DIR=$(find "$scan_dir" -maxdepth 1 -name "rosbag_*" -type d 2>/dev/null | grep -v '_imu$' | head -1)
                    [ -z "$BAG_DIR" ] && continue
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/extract_fisheye_from_bag.py" \
                        "$BAG_DIR" "$scan_dir" --dual || echo "  Warning: ERP extraction failed for $(basename $scan_dir), skipping"
                done
                if [ "$BLEND_ERP_SEAMS" = "true" ]; then
                    echo "Blending ERP image seams..."
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/blend_erp_seams_simple.py" "$SCAN_DIR"
                fi
                echo "Creating masked images from blended ERPs..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/regenerate_masked_images.py" "$SCAN_DIR" --camera-mode "$CAMERA_MODE"
                echo "Creating colored point clouds..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_coloring.py" "$SCAN_DIR" --use-exact
            else
                echo "Converting fisheye images to ERP (single fisheye)..."
                for scan_dir in "$SCAN_DIR"/fusion_scan_*; do
                    [ -d "$scan_dir" ] || continue
                    BAG_DIR=$(find "$scan_dir" -maxdepth 1 -name "rosbag_*" -type d 2>/dev/null | grep -v '_imu$' | head -1)
                    [ -z "$BAG_DIR" ] && continue
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/extract_fisheye_from_bag.py" \
                        "$BAG_DIR" "$scan_dir" || echo "  Warning: ERP extraction failed for $(basename $scan_dir), skipping"
                done
                echo "Creating masked images..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/regenerate_masked_images.py" "$SCAN_DIR" --camera-mode "$CAMERA_MODE"
                echo "Creating colored point clouds..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_coloring.py" "$SCAN_DIR" --use-exact
            fi
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
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/align_scan_session_posegraph.py" "$SCAN_DIR"
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
            echo "Exporting session to COLMAP format..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/export_to_colmap.py" "$SCAN_DIR"
            if [ $? -eq 0 ]; then
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/erp_to_perspective_colmap.py" "$SCAN_DIR"
            fi
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
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/calibration/coordinate_transform.py" "$ROS_WS_DIR/src/atlas-scanner/src" --use-existing
else
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/calibration/coordinate_transform.py" "$ROS_WS_DIR/src/atlas-scanner/src"
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
    echo "$pat" | grep -q "insta360" && _max=150
    for _p in $pids; do
        for _w in $(seq 1 $_max); do
            kill -0 "$_p" 2>/dev/null || break
            sleep 0.1
        done
        kill -0 "$_p" 2>/dev/null && kill -KILL "$_p" 2>/dev/null || true
    done
}

# Block until all insta360_ros_driver processes have fully exited.
# This is the critical gate: the camera firmware needs the SDK to complete
# StopLiveStreaming()+Close() before a new session can open cleanly.
_wait_camera_dead() {
    local _i
    for _i in $(seq 1 20); do
        pgrep -f "insta360_ros_driver" > /dev/null 2>&1 || break
        sleep 1
    done
    pgrep -f "insta360_ros_driver" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true
    for _i in $(seq 1 5); do
        pgrep -f "insta360_ros_driver/lib/insta360_ros_driver/decoder" > /dev/null 2>&1 || break
        sleep 1
    done
    pgrep -f "insta360_ros_driver" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true
    pgrep -f "insta360_ros_driver/lib/insta360_ros_driver/decoder" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true
    sleep 3
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
    echo "Waiting for camera firmware to reset..."
    sleep 3
    echo "✓ Camera firmware ready"
}

# USB-level reset of the Insta360 camera — clears any firmware streaming state
# that survived an unclean SDK shutdown.
# NOTE: The Insta360 ONE firmware shuts down (rather than re-enumerates) in
# response to USBDEVFS_RESET, requiring a physical button press to recover.
# Only reset if the device is already in a bad state (bConfigurationValue=0/empty).
_usb_reset_camera() {
    [ -e /dev/insta ] || return 0
    local _dev _busdev _cfg
    _dev=$(readlink -f /dev/insta 2>/dev/null) || return 0
    _busdev=$(udevadm info --query=path --name="$_dev" 2>/dev/null \
        | grep -oP '[0-9]+-[0-9.]+$')
    # Check if device is already configured — if so, skip the reset entirely.
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
    # Wait for device node to re-appear and bConfigurationValue to be set
    for _i in $(seq 1 15); do
        [ -e /dev/insta ] || { sleep 1; continue; }
        [ -d "/sys/bus/usb/devices/$_busdev" ] || { sleep 1; continue; }
        _cfg=$(cat /sys/bus/usb/devices/$_busdev/bConfigurationValue 2>/dev/null | tr -d '[:space:]')
        [ -n "$_cfg" ] && [ "$_cfg" != "0" ] && break
        sleep 1
    done
    _cfg=$(cat /sys/bus/usb/devices/$_busdev/bConfigurationValue 2>/dev/null | tr -d '[:space:]')
    if [ -n "$_cfg" ] && [ "$_cfg" != "0" ]; then
        sleep 2  # allow firmware to settle
        return 0
    fi
    echo "  ⚠ USB reset could not enumerate device (bConfigurationValue='$_cfg') — driver may fail to connect"
}

# Kill all stale processes in parallel — capture PIDs so wait only blocks on these
_pkill "livox_ros_driver2" & _kpids=($!)
_pkill "rko_lio" & _kpids+=($!)
_pkill "insta360_ros_driver" & _kpids+=($!)
_pkill "insta360_ros_driver/lib/insta360_ros_driver/decoder" & _kpids+=($!)
_pkill "image_decoder" & _kpids+=($!)
_pkill "equirectangular" & _kpids+=($!)
_pkill "dual_fisheye" & _kpids+=($!)
_pkill "bringup.launch" & _kpids+=($!)
_pkill "static_transform_publisher" & _kpids+=($!)
_pkill "ros2 bag record" & _kpids+=($!)
_pkill "continuous_trajectory_recorder" & _kpids+=($!)
_pkill "ros2 launch.*insta360" & _kpids+=($!)
_pkill "python.*equirectangular" & _kpids+=($!)
_pkill "imu_frame" & _kpids+=($!)
_pkill "imu_stabilized" & _kpids+=($!)
wait "${_kpids[@]}"
# Block until the camera driver process is fully gone so StopLiveStreaming()/Close()
# completes before we attempt to open a new session.
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
    ros2 launch rko_lio odometry.launch.py \
        config_file:="$ROS_WS_DIR/src/atlas-scanner/src/config/rko_lio_config_robust.yaml" \
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
echo "Starting camera driver (mode: $CAMERA_MODE)..."
_pkill "insta360_ros_driver"
sleep 1

CAMERA_STARTED=false
for _cam_attempt in 1 2 3; do
    _IMU_FILTER="false"
    [ "$CAPTURE_MODE" = "continuous" ] && _IMU_FILTER="true"
    > /tmp/camera.log  # clear log for this attempt
    env -u FASTRTPS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_camera.xml" ROS_DISABLE_LOANED_MESSAGES=1 ros2 launch insta360_ros_driver bringup.launch.xml \
        equirectangular:=false \
        imu_filter:=${_IMU_FILTER} > /tmp/camera.log 2>&1 &
    CAMERA_PID=$!
    PIDS+=($CAMERA_PID)
    { sleep 2 && renice -n 5 -p $CAMERA_PID 2>/dev/null 1>/dev/null; } &

    # Wait up to 70s for "Live streaming started" in /tmp/camera.log.
    _stream_ready=false
    for _sw in $(seq 1 70); do
        if grep -q "Live streaming started" /tmp/camera.log 2>/dev/null; then
            _stream_ready=true
            break
        fi
        if ! pgrep -f "insta360_ros_driver" > /dev/null 2>&1; then
            break
        fi
        sleep 1
    done
    if [ "$_stream_ready" = "true" ]; then
        echo "✓ Camera driver ready"
        CAMERA_STATUS="/dual_fisheye/image/compressed"
        # Verify the camera is actually streaming within 10s — if not, the driver
        # is in a bad state and this attempt should be treated as a failure.
        _iframe_topic="/dual_fisheye/image/compressed"
        [ "$CAMERA_MODE" = "single_fisheye" ] && _iframe_topic="/dual_fisheye/image"
        # Check publisher count without spawning a new DDS participant (ros2 topic echo
        # floods the MID360 UDP path and is slow to discover). Instead poll the topic
        # info which uses the existing daemon's graph cache.
        _cam_data_ok=false
        for _cw in $(seq 1 20); do
            _pub=$(ros2 topic info "$_iframe_topic" 2>/dev/null | grep -m1 'Publisher count' | grep -o '[0-9]*')
            [ "${_pub:-0}" -gt 0 ] && { _cam_data_ok=true; break; }
            sleep 0.5
        done
        if [ "$_cam_data_ok" = "false" ]; then
            echo "⚠ Camera attempt $_cam_attempt: no data on $_iframe_topic within 10s"
            _stream_ready=false
            _pkill "insta360_ros_driver"
            _pkill "insta360_ros_driver/lib/insta360_ros_driver/decoder"
            _wait_camera_dead
            _wait_firmware_ready
            _usb_reset_camera
            for _ci in $(seq 1 10); do [ -e /dev/insta ] && break; sleep 1; done
            [ "$SKIP_SUDO_CHECK" != "1" ] && sudo chmod 777 /dev/insta 2>/dev/null || true
            continue
        fi
        CAMERA_STARTED=true
        break
    else
        echo "⚠ Camera attempt $_cam_attempt failed (driver exited), retrying..."
        _pkill "insta360_ros_driver"
        _pkill "insta360_ros_driver/lib/insta360_ros_driver/decoder"
        _wait_camera_dead
        _wait_firmware_ready
        _usb_reset_camera
        for _ci in $(seq 1 10); do [ -e /dev/insta ] && break; sleep 1; done
        [ "$SKIP_SUDO_CHECK" != "1" ] && sudo chmod 777 /dev/insta 2>/dev/null || true
    fi
done

if [ "$CAMERA_STARTED" = "false" ]; then
    echo "✗ Camera failed to start after 3 attempts"; exit 1
fi
echo "✓ Camera data confirmed"
touch /tmp/.insta360_session_ran  # sentinel: firmware needs settle time before next session
# Truncate camera log now so pre-ready decoder errors don't pollute per-scan health checks
> /tmp/camera.log

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
    CONTINUOUS_TOPICS="/livox/lidar /dual_fisheye/image/compressed"
    [ "$LIO_ENABLED" = "true" ] && CONTINUOUS_TOPICS="$CONTINUOUS_TOPICS /rko_lio/odometry"
    # Main bag: LiDAR + camera + odometry on cores 2-3
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

    # Camera watchdog: restart driver if it dies mid-session
    (
        while true; do
            sleep 2
            [ -f "$SCAN_DIR/.session_done" ] && exit 0
            if ! pgrep -f "insta360_ros_driver" > /dev/null 2>&1; then
                echo "⚠ Camera driver died, restarting..."
                _pkill "insta360_ros_driver"
                _pkill "insta360_ros_driver/lib/insta360_ros_driver/decoder"
                _wait_camera_dead
                _usb_reset_camera
                for _ci in $(seq 1 10); do [ -e /dev/insta ] && break; sleep 1; done
                [ "$SKIP_SUDO_CHECK" != "1" ] && sudo chmod 777 /dev/insta 2>/dev/null || true
                env -u FASTRTPS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_camera.xml" ROS_DISABLE_LOANED_MESSAGES=1 ros2 launch insta360_ros_driver bringup.launch.xml \
                    equirectangular:=false imu_filter:=true > /tmp/camera.log 2>&1 &
                CAMERA_PID=$!
                echo "✓ Camera driver restarted (pid $CAMERA_PID)"
                sleep 10  # give driver time to stabilise before next check
            fi
        done
    ) &
    WATCHDOG_PID=$!
    PIDS+=("$WATCHDOG_PID")

    while true; do
        if [ ! -r /dev/tty ] || ! { true < /dev/tty; } 2>/dev/null; then
            # GUI mode: poll for quit trigger file
            [ -f "$SCAN_DIR/.quit_trigger" ] && rm -f "$SCAN_DIR/.quit_trigger" && break
            sleep 0.2
        else
            read -r _input < /dev/tty 2>/dev/null
            [ "$_input" = "q" ] || [ "$_input" = "quit" ] && break
        fi
    done
    touch "$SCAN_DIR/.session_done"  # signal watchdog to exit

else
    # ── Stationary mode ──────────────────────────────────────────────────────
    # Keep the camera stream alive between scans - the Insta360 SDK closes the
    # connection if no subscriber consumes frames for ~40s.
    env -u FASTRTPS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_camera.xml" python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/camera_keepalive.py" "$CAMERA_MODE" &
    KEEPALIVE_PID=$!
    PIDS+=($KEEPALIVE_PID)

    # Camera watchdog: restart driver + wait for decoder I-frame if it dies between scans
    (
        while true; do
            sleep 5
            [ -f "$SCAN_DIR/.session_done" ] && exit 0
            if ! pgrep -f "insta360_ros_driver" > /dev/null 2>&1; then
                echo "⚠ Camera driver died, restarting..."
                _pkill "insta360_ros_driver"
                _pkill "insta360_ros_driver/lib/insta360_ros_driver/decoder"
                _wait_camera_dead
                _usb_reset_camera
                for _ci in $(seq 1 10); do [ -e /dev/insta ] && break; sleep 1; done
                [ "$SKIP_SUDO_CHECK" != "1" ] && sudo chmod 777 /dev/insta 2>/dev/null || true
                env -u FASTRTPS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_camera.xml" ROS_DISABLE_LOANED_MESSAGES=1 ros2 launch insta360_ros_driver bringup.launch.xml \
                    equirectangular:=false imu_filter:=false > /tmp/camera.log 2>&1 &
                CAMERA_PID=$!
                for _sw in $(seq 1 70); do
                    grep -q "Live streaming started" /tmp/camera.log 2>/dev/null && break
                    sleep 1
                done
                _iframe_topic="/dual_fisheye/image/compressed"
                [ "$CAMERA_MODE" = "single_fisheye" ] && _iframe_topic="/dual_fisheye/image"
                _cam_data_ok=false
                for _cw in $(seq 1 20); do
                    _pub=$(ros2 topic info "$_iframe_topic" 2>/dev/null | grep -m1 'Publisher count' | grep -o '[0-9]*')
                    [ "${_pub:-0}" -gt 0 ] && { _cam_data_ok=true; break; }
                    sleep 0.5
                done
                if [ "$_cam_data_ok" = "false" ]; then
                    echo "⚠ Camera watchdog: no data on $_iframe_topic within 10s, will retry next cycle"
                    _pkill "insta360_ros_driver"
                    _pkill "insta360_ros_driver/lib/insta360_ros_driver/decoder"
                    _wait_camera_dead
                    _usb_reset_camera
                else
                    echo "✓ Camera driver restarted (pid $CAMERA_PID)"
                fi
            fi
        done
    ) &
    STATIONARY_WATCHDOG_PID=$!
    PIDS+=("$STATIONARY_WATCHDOG_PID")

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

        # Check decoder health for single_fisheye — if it's stuck on corrupt
        # H.264 frames since the last scan, restart it. The log is truncated at
        # session ready so only post-ready errors are counted.
        if [ "$CAMERA_MODE" = "single_fisheye" ]; then
            _decoder_pid=$(pgrep -f "insta360_ros_driver/lib/insta360_ros_driver/decoder" | head -1)
            if [ -n "$_decoder_pid" ]; then
                _recent_errors=$(grep -c "non-existing PPS\|decode_slice_header error\|no frame" /tmp/camera.log 2>/dev/null)
                _recent_errors=${_recent_errors:-0}
                if [ "$_recent_errors" -gt 10 ]; then
                    echo "⚠ Decoder stuck on corrupt H.264 stream ($_recent_errors errors), restarting..."
                    kill -TERM "$_decoder_pid" 2>/dev/null
                    for _dw in $(seq 1 20); do
                        kill -0 "$_decoder_pid" 2>/dev/null || break
                        sleep 0.1
                    done
                    kill -0 "$_decoder_pid" 2>/dev/null && kill -KILL "$_decoder_pid" 2>/dev/null || true
                    for _dw in $(seq 1 20); do
                        pgrep -f "insta360_ros_driver/lib/insta360_ros_driver/decoder" > /dev/null 2>&1 && break
                        sleep 0.5
                    done
                    sleep 5
                    # Truncate log again so errors from this restart don't carry forward
                    > /tmp/camera.log
                    echo "✓ Decoder restarted"
                fi
            fi
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
        TOPICS_TO_RECORD="/livox/lidar /dual_fisheye/image/compressed /livox/imu"
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

        # Capture — unset the atlas FastDDS profile so capture nodes use default
        # multicast discovery, which reliably finds all publishers on the same machine.
        echo "Capturing scan $SCAN_COUNT..."
        # Wait for the previous capture process to fully release its FastDDS SHM ports.
        # Session processes (LiDAR, camera, RKO-LIO) permanently hold their own ports,
        # so polling fuser would never clear — use a fixed wait matching the capture
        # script's 2s exit sleep instead.
        sleep 2
        # Note: inter-scan SHM cleanup removed — selectively removing port files
        # while session processes are live causes lidar/decoder ports to be deleted
        # during the brief window when a process releases its _el lock for DDS
        # participant activity. Full SHM cleanup only happens at session startup
        # after all processes are confirmed dead.
        if [ "$CAMERA_MODE" = "single_fisheye" ]; then
            FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_capture.xml" ROS_DISABLE_LOANED_MESSAGES=1 \
            timeout 70 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/single_fisheye_capture_decoded.py" \
                "$INDIVIDUAL_SCAN_DIR" 30 4.0
            CAPTURE_EXIT_CODE=$?
            if [ $CAPTURE_EXIT_CODE -ne 0 ]; then
                echo "  Primary capture failed, trying buffered camera capture..."
                FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_capture.xml" ROS_DISABLE_LOANED_MESSAGES=1 \
                timeout 25 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_camera_capture.py" \
                    "$INDIVIDUAL_SCAN_DIR" 15 2.0
                CAPTURE_EXIT_CODE=$?
            fi
        else
            FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_capture.xml" ROS_DISABLE_LOANED_MESSAGES=1 \
            timeout 25 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_camera_capture.py" \
                "$INDIVIDUAL_SCAN_DIR" 15 2.0
            CAPTURE_EXIT_CODE=$?
        fi

        # Fallback: LiDAR-driven capture
        if [ $CAPTURE_EXIT_CODE -ne 0 ]; then
            echo "  Trying LiDAR-driven capture..."
            FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_DIR/src/atlas-scanner/src/config/fastdds_capture.xml" ROS_DISABLE_LOANED_MESSAGES=1 \
            timeout 15 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/lidar_driven_capture.py" \
                "$INDIVIDUAL_SCAN_DIR" 8
            CAPTURE_EXIT_CODE=$?
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
        if [ -n "$WORLD_PLY" ]; then
            mv "$WORLD_PLY" "$INDIVIDUAL_SCAN_DIR/world_lidar.ply"
        elif [ -n "$SENSOR_PLY" ]; then
            mv "$SENSOR_PLY" "$INDIVIDUAL_SCAN_DIR/sensor_lidar.ply"
        fi

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
