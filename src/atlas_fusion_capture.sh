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

# Allow CLI overrides: atlas_fusion_capture.sh [--camera dual_fisheye|single_fisheye] [--capture stationary|continuous]
while [[ $# -gt 0 ]]; do
    case $1 in
        --camera) CAMERA_MODE="$2"; shift 2 ;;
        --capture) CAPTURE_MODE="$2"; shift 2 ;;
        --interval) CONTINUOUS_INTERVAL="$2"; shift 2 ;;
        --bag-only) BAG_ONLY=true; shift ;;
        --no-sync-benchmark) RUN_SYNC_BENCHMARK=false; shift ;;
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
# Prevent ros2 CLI tools from hanging when no display is available (headless / SSH)
export QT_QPA_PLATFORM=offscreen

mkdir -p data/synchronized_scans
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
    echo "Shutting down..."

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null
            sleep 2
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
    sleep 1
    _pkill "buffered_odom_bridge"
    _pkill "insta360_ros_driver"
    _pkill "image_decoder"
    _pkill "decoder"
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

    sleep 3

    if [ "$BAG_ONLY" = "true" ]; then
        echo "Bag-only mode: skipping post-processing. Bag saved in: $SCAN_DIR"
        exit 0
    fi

    # Continuous mode: reconstruct scans from the single session bag
    if [ "$CAPTURE_MODE" = "continuous" ]; then
        BAG_DIR=$(ls -d "$SCAN_DIR"/rosbag_* 2>/dev/null | head -1)
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
                    BAG_DIR=$(ls -d "$scan_dir"/rosbag_* 2>/dev/null | head -1)
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
                echo "Creating colored point clouds using fisheye images..."
                for scan_dir in "$SCAN_DIR"/fusion_scan_*; do
                    [ -d "$scan_dir" ] && python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/color_with_fisheye.py" "$scan_dir"
                done
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
    exit 0
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
    echo $pids | xargs -r kill -TERM 2>/dev/null || true
    sleep 2
    for _p in $pids; do kill -0 "$_p" 2>/dev/null && kill -KILL "$_p" 2>/dev/null || true; done
}

# Block until all insta360_ros_driver processes have fully exited.
# This is the critical gate: the camera firmware needs the SDK to complete
# StopLiveStreaming()+Close() before a new session can open cleanly.
_wait_camera_dead() {
    local _i
    for _i in $(seq 1 15); do
        pgrep -f "insta360_ros_driver" > /dev/null 2>&1 || return 0
        sleep 1
    done
    # Still alive after 15s — force kill as last resort
    pgrep -f "insta360_ros_driver" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true
    pgrep -f "decoder" 2>/dev/null | xargs -r kill -KILL 2>/dev/null || true
    sleep 1
}

# USB-level reset of the Insta360 camera — clears any firmware streaming state
# that survived an unclean SDK shutdown.
_usb_reset_camera() {
    [ -e /dev/insta ] || return 0
    local _dev
    _dev=$(readlink -f /dev/insta 2>/dev/null) || return 0
    if command -v usbreset > /dev/null 2>&1; then
        echo "Resetting USB device $_dev..."
        sudo usbreset "$_dev" 2>/dev/null || true
    else
        local _busdev
        _busdev=$(udevadm info --query=path --name="$_dev" 2>/dev/null \
            | grep -oP '[0-9]+-[0-9.]+(?=/[^/]+$)' | head -1)
        if [ -n "$_busdev" ] && [ -d "/sys/bus/usb/devices/$_busdev" ]; then
            echo "Resetting USB device via sysfs ($_busdev)..."
            echo "$_busdev" | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null 2>&1 || true
            sleep 1
            echo "$_busdev" | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null 2>&1 || true
        fi
    fi
    # Wait for device node to re-appear after reset
    for _i in $(seq 1 10); do
        [ -e /dev/insta ] && return 0
        sleep 1
    done
}

# Kill all stale processes in parallel — capture PIDs so wait only blocks on these
_pkill "livox_ros_driver2" & _kpids=($!)
_pkill "rko_lio" & _kpids+=($!)
_pkill "insta360_ros_driver" & _kpids+=($!)
_pkill "image_decoder" & _kpids+=($!)
_pkill "decoder" & _kpids+=($!)
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
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port* 2>/dev/null || true
sleep 1

# Camera permissions
if [ "$SKIP_SUDO_CHECK" != "1" ]; then
    "$SCRIPT_DIR/setup_camera_permissions.sh"
fi

# USB reset clears any firmware streaming state left over from the previous session.
# Runs after the driver process is confirmed dead so the reset isn't racing with
# an in-progress StopLiveStreaming() call.
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

# ─── Camera driver ────────────────────────────────────────────────────────────
echo "Starting camera driver (mode: $CAMERA_MODE)..."
_pkill "insta360_ros_driver"
sleep 1

CAMERA_STARTED=false
for _cam_attempt in 1 2 3; do
    # Enable the Madgwick IMU filter in continuous mode only — it smooths /imu/data_raw
    # into a regularly-timed /imu/data output needed for cross-correlation in imu_sync.py.
    # Disabled in stationary mode to avoid unnecessary CPU load during capture.
    _IMU_FILTER="false"
    [ "$CAPTURE_MODE" = "continuous" ] && _IMU_FILTER="true"

    if [ "$CAMERA_MODE" = "dual_fisheye" ]; then
        ros2 launch insta360_ros_driver bringup.launch.xml \
            equirectangular:=false \
            decode:=false \
            imu_filter:=${_IMU_FILTER} &
        CAMERA_PID=$!
        PIDS+=($CAMERA_PID)
        { sleep 2 && renice -n 5 -p $CAMERA_PID 2>/dev/null; } &
    else
        ros2 launch insta360_ros_driver bringup.launch.xml \
            equirectangular:=false \
            decode:=false \
            imu_filter:=${_IMU_FILTER} &
        CAMERA_PID=$!
        PIDS+=($CAMERA_PID)
        { sleep 2 && renice -n 15 -p $CAMERA_PID 2>/dev/null; } &
    fi

    # Poll until driver confirms streaming or exits — wait up to 20s
    # The driver now exits with -1 if StartLiveStreaming() succeeds but no
    # frames arrive within 8s, so a dead process here means a real failure.
    for _w in $(seq 1 20); do
        kill -0 $CAMERA_PID 2>/dev/null || break
        sleep 1
    done
    if kill -0 $CAMERA_PID 2>/dev/null; then
        echo "✓ Camera driver ready"
        CAMERA_STATUS="/dual_fisheye/image/compressed"
        CAMERA_STARTED=true
        break
    else
        echo "⚠ Camera attempt $_cam_attempt failed (driver exited), retrying..."
        _pkill "insta360_ros_driver"
        _pkill "decoder"
        _wait_camera_dead
        # USB reset to clear firmware streaming state before next attempt
        _usb_reset_camera
        for _ci in $(seq 1 10); do [ -e /dev/insta ] && break; sleep 1; done
        [ "$SKIP_SUDO_CHECK" != "1" ] && sudo chmod 777 /dev/insta 2>/dev/null || true
    fi
done

if [ "$CAMERA_STARTED" = "false" ]; then
    echo "✗ Camera failed to start after 3 attempts"; exit 1
fi
echo "✓ Camera data confirmed"

echo "Starting LiDAR driver..."
# Pin Livox driver to cores 0-1 so its UDP receive thread is isolated
# from the bag recorders on cores 2-3, preventing IMU packet drops.
taskset -c 0,1 setsid ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
{ sleep 3 && renice -n -5 -p $LIDAR_PID 2>/dev/null; } &
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
while [ $_lidar_data_wait -lt 15 ]; do
    timeout 3 ros2 topic echo /livox/lidar --once > /dev/null 2>&1 && break
    _lidar_data_wait=$((_lidar_data_wait + 1))
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

    ros2 launch rko_lio odometry.launch.py \
        config_file:="$ROS_WS_DIR/src/atlas-scanner/src/config/rko_lio_config_robust.yaml" \
        rviz:=false > /tmp/rko_lio.log 2>&1 &
    LIO_PID=$!
    sleep 1 && renice -n -15 -p $LIO_PID 2>/dev/null &
    PIDS+=($LIO_PID)
    _rlio_count=0
    while [ $_rlio_count -lt 20 ]; do
        grep -q "RKO LIO Node is up" /tmp/rko_lio.log 2>/dev/null && break
        kill -0 $LIO_PID 2>/dev/null || { echo "⚠ RKO-LIO exited unexpectedly"; break; }
        sleep 1; _rlio_count=$((_rlio_count + 1))
    done

    if grep -q "RKO LIO Node is up" /tmp/rko_lio.log 2>/dev/null; then
        echo "✓ RKO-LIO ready with $IMU_SOURCE IMU"
        echo "Waiting for IMU/LIO to stabilize..."
        # Poll for odometry output instead of a fixed 15s sleep
        _lio_stab=0
        while [ $_lio_stab -lt 20 ]; do
            timeout 3 ros2 topic echo /rko_lio/odometry --once > /dev/null 2>&1 && break
            _lio_stab=$((_lio_stab + 1))
        done
        LIO_ENABLED="true"
        QT_QPA_PLATFORM=xcb rviz2 -d "$ROS_WS_DIR/src/atlas-scanner/src/config/atlas_display.rviz" \
            -stylesheet "$ROS_WS_DIR/src/atlas-scanner/src/config/rviz_kiosk.qss" > /tmp/rviz.log 2>&1 &
        RVIZ_PID=$!
        # Not added to PIDS — RViz crash should not terminate the session
        # Give RViz higher priority than camera pipeline in dual_fisheye mode
        [ "$CAMERA_MODE" = "dual_fisheye" ] && sleep 2 && renice -n 5 -p $RVIZ_PID 2>/dev/null &
    else
        echo "⚠ RKO-LIO failed to start"
        LIO_ENABLED="false"
    fi
else
    echo "⚠ No IMU available - RKO-LIO disabled"
    LIO_ENABLED="false"
fi

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

# ─── Trajectory recorder (start once, used by both modes) ─────────────────────
if [ "$LIO_ENABLED" = "true" ]; then
    echo "Starting trajectory recording..."
    TRAJ_PID_FILE="$SCAN_DIR/.trajectory_recorder.pid"
    _pkill "enhanced_trajectory_recorder"
    _pkill "buffered_odom_bridge"
    sleep 2
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/enhanced_trajectory_recorder.py" \
        "$SCAN_DIR" > "$SCAN_DIR/trajectory_recorder.log" 2>&1 &
    TRAJECTORY_PID=$!
    echo "$TRAJECTORY_PID" > "$TRAJ_PID_FILE"
    PIDS+=($TRAJECTORY_PID)
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_odom_bridge.py" \
        > "$SCAN_DIR/odom_buffer.log" 2>&1 &
    PIDS+=($!)
    TRAJECTORY_RECORDING=true
    echo "✓ Trajectory recording started"
fi

# ─── Capture loop ─────────────────────────────────────────────────────────────
if [ "$CAPTURE_MODE" = "continuous" ]; then
    echo ""
    echo "=========================================="
    echo "CONTINUOUS MODE: recording session bag"
    echo "Press Ctrl+C to stop"
    echo "=========================================="

    ROSBAG_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    ROSBAG_DIR="$SCAN_DIR/rosbag_$ROSBAG_TIMESTAMP"
    CONTINUOUS_TOPICS="/livox/lidar /dual_fisheye/image/compressed_15fps"
    [ "$LIO_ENABLED" = "true" ] && CONTINUOUS_TOPICS="$CONTINUOUS_TOPICS /rko_lio/odometry"
    # IMU topics are recorded in a separate bag (${ROSBAG_DIR}_imu) to avoid
    # write contention with the high-bandwidth LiDAR and image topics.
    # Throttle camera to 15fps before recording — driver publishes at 30fps but bag only needs 15fps
    ros2 run topic_tools throttle messages /dual_fisheye/image/compressed 15.0 \
        /dual_fisheye/image/compressed_15fps > /dev/null 2>&1 &
    PIDS+=($!)
    sleep 1
    # Main bag: LiDAR + camera + odometry on cores 2-3
    # shellcheck disable=SC2086
    taskset -c 2,3 nice -n 10 ros2 bag record -o "$ROSBAG_DIR" --max-cache-size 100000000 \
        $CONTINUOUS_TOPICS &
    ROSBAG_PID=$!
    PIDS+=($ROSBAG_PID)

    # IMU bag: separate high-priority recorder on core 3 so IMU topics
    # are never starved by LiDAR/image write bursts in the main bag.
    IMU_TOPICS="/livox/imu"
    ros2 topic list 2>/dev/null | grep -q "/imu/data$"   && IMU_TOPICS="$IMU_TOPICS /imu/data"
    ros2 topic list 2>/dev/null | grep -q "/imu/data_raw" && IMU_TOPICS="$IMU_TOPICS /imu/data_raw"
    # shellcheck disable=SC2086
    taskset -c 3 nice -n 5 ros2 bag record -o "${ROSBAG_DIR}_imu" --max-cache-size 20000000 \
        $IMU_TOPICS &
    IMU_BAG_PID=$!
    PIDS+=($IMU_BAG_PID)
    echo "✓ IMU bag recording started: ${ROSBAG_DIR}_imu"
    echo "✓ Rosbag recording started: $ROSBAG_DIR"
    echo "  Press Ctrl+C when scanning is complete."

    wait $ROSBAG_PID 2>/dev/null || true
    SCAN_COUNT=0

else
    # ── Stationary mode ──────────────────────────────────────────────────────
    echo "Press ENTER to capture a scan, 'q' to exit."

    while true; do
        read -p "Scan $((SCAN_COUNT + 1)) - Press ENTER to capture (or 'q' to exit): " input
        [ "$input" = "quit" ] || [ "$input" = "q" ] && break

        SCAN_COUNT=$((SCAN_COUNT + 1))
        SCAN_NAME="fusion_scan_$(printf "%03d" $SCAN_COUNT)"
        INDIVIDUAL_SCAN_DIR="$SCAN_DIR/$SCAN_NAME"
        mkdir -p "$INDIVIDUAL_SCAN_DIR"

        # Verify camera is alive, restart if needed
        if ! kill -0 $CAMERA_PID 2>/dev/null || ! pgrep -f "insta360_ros_driver" > /dev/null 2>&1; then
            echo "⚠ Camera driver died, restarting..."
            _pkill "insta360_ros_driver"
            _pkill "decoder"
            sleep 2
            for _ci in $(seq 1 15); do [ -e /dev/insta ] && break; sleep 1; done
            [ "$SKIP_SUDO_CHECK" != "1" ] && sudo chmod 777 /dev/insta 2>/dev/null || true
            if [ "$CAMERA_MODE" = "dual_fisheye" ]; then
                ros2 launch insta360_ros_driver bringup.launch.xml \
                    equirectangular:=false decode:=false imu_filter:=false &
            else
                ros2 launch insta360_ros_driver bringup.launch.xml \
                    equirectangular:=false imu_filter:=false decode:=false &
            fi
            CAMERA_PID=$!
            PIDS+=($CAMERA_PID)
            sleep 10
            if ! kill -0 $CAMERA_PID 2>/dev/null; then
                echo "✗ Camera driver failed to restart, skipping scan"
                SCAN_COUNT=$((SCAN_COUNT - 1)); continue
            fi
            echo "✓ Camera driver restarted"
        fi

        # Verify LiDAR is alive, restart if needed
        if ! kill -0 $LIDAR_PID 2>/dev/null || ! grep -q "livox/lidar publish" /tmp/lidar.log 2>/dev/null; then
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
            sleep 3
        fi

        # Ensure camera topic is live — use a 3s timeout per attempt to allow
        # DDS peer discovery to complete before declaring the topic dead
        _cam_topic_wait=0
        while [ $_cam_topic_wait -lt 10 ]; do
            timeout 3 ros2 topic echo /dual_fisheye/image/compressed --once > /dev/null 2>&1 && break
            _cam_topic_wait=$((_cam_topic_wait + 1))
        done
        if [ $_cam_topic_wait -ge 10 ]; then
            echo "⚠ Camera topic not publishing, restarting driver..."
            _pkill "insta360_ros_driver"
            _pkill "decoder"
            sleep 2
            for _ci in $(seq 1 15); do [ -e /dev/insta ] && break; sleep 1; done
            [ "$SKIP_SUDO_CHECK" != "1" ] && sudo chmod 777 /dev/insta 2>/dev/null || true
            ros2 launch insta360_ros_driver bringup.launch.xml \
                equirectangular:=false imu_filter:=false decode:=false &
            CAMERA_PID=$!
            PIDS+=($CAMERA_PID)
            sleep 8
        fi

        # Start per-scan rosbag immediately before capture so idle setup time
        # between scans is not recorded — keeps bag sizes proportional to the
        # ~4s capture window rather than the full inter-scan gap.
        _t0=$EPOCHREALTIME
        ROSBAG_TIMESTAMP=$(date +%Y%m%d_%H%M%S_%N | cut -c1-18)
        ROSBAG_DIR="$INDIVIDUAL_SCAN_DIR/rosbag_$ROSBAG_TIMESTAMP"
        TOPICS_TO_RECORD="/livox/lidar /dual_fisheye/image/compressed /livox/imu"
        [ "$LIO_ENABLED" = "true" ] && TOPICS_TO_RECORD="$TOPICS_TO_RECORD /rko_lio/odometry_buffered"
        setsid ros2 bag record -o "$ROSBAG_DIR" \
            --compression-mode file --compression-format zstd \
            --max-cache-size 100000000 $TOPICS_TO_RECORD > "$ROSBAG_DIR.log" 2>&1 &
        ROSBAG_PID=$!
        ROSBAG_PGID=$(ps -o pgid= -p $ROSBAG_PID 2>/dev/null | tr -d ' ')
        # Wait for bag recorder to subscribe to all topics (max 30s)
        _bag_wait=0
        while [ $_bag_wait -lt 60 ]; do
            grep -q "All requested topics are subscribed" "$ROSBAG_DIR.log" 2>/dev/null && break
            sleep 0.5; _bag_wait=$((_bag_wait + 1))
        done
        # Wait for RKO-LIO odometry to be live rather than a fixed 2s sleep
        if [ "$LIO_ENABLED" = "true" ]; then
            _odom_wait=0
            while [ $_odom_wait -lt 10 ]; do
                timeout 3 ros2 topic echo /rko_lio/odometry_buffered --once > /dev/null 2>&1 && break
                _odom_wait=$((_odom_wait + 1))
            done
        fi

        # Capture
        echo "Capturing scan $SCAN_COUNT..."
        if [ "$CAMERA_MODE" = "single_fisheye" ]; then
            ROS_DISABLE_LOANED_MESSAGES=1 \
            timeout 70 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/single_fisheye_capture_decoded.py" \
                "$INDIVIDUAL_SCAN_DIR" 30 4.0
            CAPTURE_EXIT_CODE=$?
            if [ $CAPTURE_EXIT_CODE -ne 0 ]; then
                echo "  Primary capture failed, trying buffered camera capture..."
                ROS_DISABLE_LOANED_MESSAGES=1 \
                timeout 25 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_camera_capture.py" \
                    "$INDIVIDUAL_SCAN_DIR" 15 2.0
                CAPTURE_EXIT_CODE=$?
            fi
        else
            # dual_fisheye: use buffered camera capture (produces equirectangular via driver)
            ROS_DISABLE_LOANED_MESSAGES=1 \
            timeout 25 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_camera_capture.py" \
                "$INDIVIDUAL_SCAN_DIR" 15 2.0
            CAPTURE_EXIT_CODE=$?
        fi

        # Fallback: LiDAR-driven capture
        if [ $CAPTURE_EXIT_CODE -ne 0 ]; then
            echo "  Trying LiDAR-driven capture..."
            ROS_DISABLE_LOANED_MESSAGES=1 \
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
        # Remove uncompressed .db3 files once the zstd file is stable (non-growing)
        # instead of a fixed sleep — avoids waiting longer than necessary.
        for _db3 in "$ROSBAG_DIR"/*.db3; do
            [ -f "$_db3" ] || continue
            _zstd="${_db3}.zstd"
            [ -f "$_zstd" ] || continue
            _prev=0; _stable=0
            for _si in $(seq 1 10); do
                _cur=$(stat -c%s "$_zstd" 2>/dev/null || echo 0)
                if [ "$_cur" = "$_prev" ] && [ "$_cur" -gt 0 ]; then
                    _stable=$((_stable + 1))
                    [ "$_stable" -ge 2 ] && break
                else
                    _stable=0
                fi
                _prev=$_cur; sleep 0.3
            done
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
fi

if [ "$ENABLE_POST_PROCESSING_BAGS" = "true" ]; then
    read -p "Run post-processing on all bag files? (y/n): " post_process
    if [ "$post_process" = "y" ] || [ "$post_process" = "Y" ]; then
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_bags.py" "$SCAN_DIR"
    fi
fi
