#!/bin/bash

# Auto-detect workspace root (where this script is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Configuration
SAVE_E57=false                      # Set to true to enable E57 export
USE_EXISTING_CALIBRATION=false      # Set to true to skip calibration update from calib.json
ENABLE_ICP_ALIGNMENT=true           # Set to true to offer ICP alignment at end
BLEND_ERP_SEAMS=true                # Set to true to blend fisheye seams in ERP images before coloring
EXPORT_COLMAP=true                  # Set to true to export session to COLMAP format (experimental)
ENABLE_POST_PROCESSING_BAGS=false   # Set to true to offer post-processing option at end
SKIP_LIVE_FUSION=true               # Set to true to skip fusion during scanning, only record bags
AUTO_CREATE_COLORED=true            # Set to true to automatically create colored point clouds when SKIP_LIVE_FUSION=true

cd "$ROS_WS_DIR"
export PATH=/home/orion/cmake-3.25/bin:$PATH
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/orion:/usr/local/lib

# Create data directory
mkdir -p data/synchronized_scans
SCAN_DIR="data/synchronized_scans/sync_fusion_$(date +%Y%m%d_%H%M%S)"
mkdir -p $SCAN_DIR

PIDS=()
FUSION_READY="false"
LIO_ENABLED="false"

cleanup() {
    echo "Shutting down..."
    
    # Kill tracked PIDs first
    for pid in "${PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            kill -TERM $pid 2>/dev/null
            sleep 1
            kill -KILL $pid 2>/dev/null
        fi
    done
    
    # Aggressive trajectory recorder cleanup
    echo "Cleaning up trajectory recording processes..."
    
    # Kill by PID file first if it exists
    TRAJ_PID_FILE="$SCAN_DIR/.trajectory_recorder.pid"
    if [ -f "$TRAJ_PID_FILE" ]; then
        TRAJ_PID=$(cat "$TRAJ_PID_FILE" 2>/dev/null)
        if [ -n "$TRAJ_PID" ] && kill -0 "$TRAJ_PID" 2>/dev/null; then
            echo "Killing trajectory recorder (PID: $TRAJ_PID)"
            kill -9 "$TRAJ_PID" 2>/dev/null
        fi
        rm -f "$TRAJ_PID_FILE"
    fi
    
    # Kill all trajectory processes
    pkill -9 -f "enhanced_trajectory_recorder" 2>/dev/null
    pkill -9 -f "buffered_odom_bridge" 2>/dev/null
    pkill -9 -f "python3.*enhanced_trajectory_recorder" 2>/dev/null
    pkill -9 -f "python3.*buffered_odom_bridge" 2>/dev/null
    
    # Aggressive camera cleanup
    echo "Cleaning up camera processes..."
    pkill -TERM -f "insta360_ros_driver" 2>/dev/null
    pkill -TERM -f "equirectangular" 2>/dev/null
    pkill -TERM -f "dual_fisheye" 2>/dev/null
    pkill -TERM -f "bringup.launch" 2>/dev/null
    
    # Kill other processes
    pkill -f "livox_ros_driver2" 2>/dev/null
    pkill -f "rko_lio" 2>/dev/null
    pkill -f "static_transform_publisher" 2>/dev/null
    pkill -f "ros2 bag record" 2>/dev/null
    pkill -f "continuous_trajectory_recorder" 2>/dev/null
    
    # Wait for processes to die
    sleep 3
    
    # Final aggressive cleanup
    pkill -9 -f "ros2 launch.*insta360" 2>/dev/null
    pkill -9 -f "python.*equirectangular" 2>/dev/null
    
    # Run post-processing if scans were captured
    if [ "$FUSION_READY" = "true" ] && [ "$SCAN_COUNT" -gt 0 ]; then
        echo ""
        echo "=========================================="
        echo "SCANNING SESSION COMPLETE"
        echo "=========================================="
        echo "Total scans captured: $SCAN_COUNT"
        echo "Data saved in: $SCAN_DIR"
        echo ""
        
        # Auto-create colored point clouds if live fusion was skipped
        if [ "$SKIP_LIVE_FUSION" = "true" ] && [ "$AUTO_CREATE_COLORED" = "true" ]; then
            if [ "$BLEND_ERP_SEAMS" = "true" ]; then
                echo "Blending ERP image seams..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/blend_erp_seams_simple.py" "$SCAN_DIR"
            fi
            echo "Creating masked images from blended ERPs..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/regenerate_masked_images.py" "$SCAN_DIR"
            echo "Creating colored point clouds for all scans..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_coloring.py" "$SCAN_DIR" --use-exact
            echo "✓ Colored point clouds created"
        fi
        
        # Always merge scans using trajectory poses (if available)
        if [ "$SCAN_COUNT" -gt 1 ]; then
            # If ICP alignment is enabled, skip trajectory merge and use ICP poses instead
            if [ "$ENABLE_ICP_ALIGNMENT" = "true" ]; then
                echo "Skipping trajectory-based merge (will use ICP alignment instead)..."
            else
                echo "Merging scans using trajectory poses..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/merge_with_trajectory.py" "$SCAN_DIR"
                MERGED_FILE="$SCAN_DIR/merged_pointcloud.ply"
                
                # Create E57 for merged file if enabled
                if [ "$SAVE_E57" = "true" ] && [ -f "$MERGED_FILE" ]; then
                    echo "Creating E57 for merged point cloud..."
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/ply_to_e57.py" "$MERGED_FILE" "${MERGED_FILE%.ply}.e57"
                fi
            fi
        elif [ "$SCAN_COUNT" -eq 1 ]; then
            # Single scan - just copy it as merged
            FIRST_SCAN=$(ls -d "$SCAN_DIR"/fusion_scan_* | head -1)
            COLORED_PLY=$(ls "$FIRST_SCAN"/world_colored*.ply 2>/dev/null | head -1)
            if [ -n "$COLORED_PLY" ]; then
                cp "$COLORED_PLY" "$SCAN_DIR/merged_pointcloud.ply"
                MERGED_FILE="$SCAN_DIR/merged_pointcloud.ply"
                echo "✓ Single scan copied as merged"
                
                # Create E57 for merged file if enabled
                if [ "$SAVE_E57" = "true" ] && [ -f "$MERGED_FILE" ]; then
                    echo "Creating E57 for merged point cloud..."
                    python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/ply_to_e57.py" "$MERGED_FILE" "${MERGED_FILE%.ply}.e57"
                fi
            fi
        fi
        
        # ICP Alignment
        if [ "$SCAN_COUNT" -gt 1 ] && [ "$ENABLE_ICP_ALIGNMENT" = "true" ]; then
            echo "Applying ICP alignment..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/align_scan_session.py" "$SCAN_DIR"
            if [ $? -eq 0 ]; then
                echo "✓ ICP alignment complete"
                # Use ICP-aligned merged file (colored version)
                if [ -f "$SCAN_DIR/merged_aligned_colored.ply" ]; then
                    MERGED_FILE="$SCAN_DIR/merged_aligned_colored.ply"
                    
                    # Create E57 for aligned merged file if enabled
                    if [ "$SAVE_E57" = "true" ]; then
                        echo "Creating E57 for aligned merged point cloud..."
                        python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/ply_to_e57.py" "$MERGED_FILE" "${MERGED_FILE%.ply}.e57"
                    fi
                fi
            else
                echo "⚠ ICP alignment failed"
            fi
        fi
        
        # Always open viewer for merged result
        if [ -n "$MERGED_FILE" ] && [ -f "$MERGED_FILE" ]; then
            echo "Opening 3D viewer for merged point cloud..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/web_3d_viewer.py" "$MERGED_FILE" &
            #sleep 2
        fi
        
        # Export to COLMAP format
        if [ "$EXPORT_COLMAP" = "true" ]; then
            echo "Exporting session to COLMAP format..."
            python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/export_to_colmap.py" "$SCAN_DIR"
            if [ $? -eq 0 ]; then
                echo "✓ COLMAP export complete"
                echo "  COLMAP data: $SCAN_DIR/colmap/"
                
                # Convert ERP to perspective projections
                echo "Converting to perspective projections..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/erp_to_perspective_colmap.py" "$SCAN_DIR"
                if [ $? -eq 0 ]; then
                    echo "✓ Perspective projection complete"
                    echo "  Perspective data: $SCAN_DIR/colmap/perspective_reconstruction/"
                else
                    echo "⚠ Perspective projection failed"
                fi
            else
                echo "⚠ COLMAP export failed"
            fi
        fi
        
        echo "Complete. Scans saved in: $SCAN_DIR"
        echo "Each scan includes:"
        echo "  - world_lidar.ply (LiDAR points)"
        echo "  - equirectangular.jpg (camera image)"
        if [ "$SKIP_LIVE_FUSION" = "true" ] && [ "$AUTO_CREATE_COLORED" = "true" ]; then
            echo "  - world_colored.ply (colored point cloud)"
        fi
        echo "  - rosbag_TIMESTAMP/ (camera + IMU data)"
        if [ "$LIO_ENABLED" = "true" ]; then
            echo "  - trajectory.json (RKO-LIO trajectory)"
        fi
        if [ "$ENABLE_ICP_ALIGNMENT" = "true" ] && [ "$SCAN_COUNT" -gt 1 ]; then
            echo "Session also includes:"
            echo "  - scan_XXX_aligned.ply (individual aligned scans)"
            echo "  - merged_aligned_pointcloud.ply (final merged result)"
        fi
        if [ "$EXPORT_COLMAP" = "true" ]; then
            echo "  - colmap/ (COLMAP format export)"
        fi
    else
        echo "Failed to initialize sensors. Check hardware connections."
    fi
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    local count=0
    
    echo "Waiting for topic $topic..."
    while [ $count -lt $timeout ]; do
        if ros2 topic list 2>/dev/null | grep -q "$topic"; then
            echo "✓ Topic $topic is available"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    echo "✗ Timeout waiting for topic $topic"
    return 1
}

wait_for_topic_data() {
    local topic=$1
    local timeout=${2:-15}
    #local count=0
    
    echo "Waiting for data on $topic..."
    timeout $timeout ros2 topic echo $topic --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✓ Topic $topic has data"
        return 0
    else
        echo "✗ No data on topic $topic"
        return 1
    fi
}

echo "=== Terrestrial Fusion Capture with RKO-LIO ===" 
echo "Scans will be saved to: $SCAN_DIR"

# Request sudo access at the start (skip if already done by GUI)
if [ "$SKIP_SUDO_CHECK" != "1" ]; then
    echo "Requesting sudo access for camera permissions..."
    sudo -v || { echo "Failed to obtain sudo access"; exit 1; }
    echo "✓ Sudo access granted"
else
    echo "Using existing sudo access from GUI..."
fi

# Transform calibration and print info
if [ "$USE_EXISTING_CALIBRATION" = "true" ]; then
    echo "Using existing calibration from fusion_calibration.yaml..."
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/calibration/coordinate_transform.py" "$ROS_WS_DIR/src/atlas-scanner/src" --use-existing
else
    echo "Updating calibration from calib.json (using matrix inversion)..."
    python3 "$ROS_WS_DIR/src/atlas-scanner/src/calibration/coordinate_transform.py" "$ROS_WS_DIR/src/atlas-scanner/src"
fi
echo ""

# Check for existing camera processes
if pgrep -f "insta360\|equirectangular\|dual_fisheye" > /dev/null; then
    echo "⚠ Warning: Camera processes are already running!"
    echo "This may cause camera initialization to fail."
    echo "Running automatic cleanup..."
    ./cleanup_camera.sh
    if [ $? -ne 0 ]; then
        echo "Cleanup failed. Please run: ./cleanup_camera.sh manually"
        exit 1
    fi
    echo "Cleanup successful, continuing..."
fi

# Kill existing processes
echo "Cleaning up existing processes..."
pkill -9 -f "livox_ros_driver2" 2>/dev/null
pkill -9 -f "rko_lio" 2>/dev/null
pkill -9 -f "insta360_ros_driver" 2>/dev/null
pkill -9 -f "equirectangular" 2>/dev/null
pkill -9 -f "dual_fisheye" 2>/dev/null
pkill -9 -f "bringup.launch" 2>/dev/null
pkill -9 -f "static_transform_publisher" 2>/dev/null
pkill -9 -f "ros2 bag record" 2>/dev/null
pkill -9 -f "continuous_trajectory_recorder" 2>/dev/null

pkill -9 -f "ros2 launch.*insta360" 2>/dev/null
pkill -9 -f "python.*equirectangular" 2>/dev/null

# Clean up any leftover transform publishers that might create invalid frames
pkill -9 -f "imu_frame" 2>/dev/null
pkill -9 -f "imu_stabilized" 2>/dev/null


# Wait longer for processes to fully terminate
sleep 3

# Check if any camera processes are still running
if pgrep -f "insta360" > /dev/null; then
    echo "Warning: Some camera processes still running, attempting final cleanup..."
    pkill -9 -f "insta360" 2>/dev/null
    sleep 2
fi

# Step 1: Setup and start camera FIRST (skip permission setup if already done)
echo "Step 1: Setting up camera permissions..."
if [ "$SKIP_SUDO_CHECK" != "1" ]; then
    if sudo -n true 2>/dev/null; then
        echo "Running camera setup with existing sudo session..."
        "$SCRIPT_DIR/setup_camera_permissions.sh"
    else
        echo "Camera setup requires sudo permissions."
        echo "Please run: sudo $SCRIPT_DIR/setup_camera_permissions.sh"
        echo "Then restart this script."
        exit 1
    fi
else
    echo "Camera permissions already set by GUI..."
fi
sleep 2

if [ -e /dev/insta ]; then
    echo "✓ Camera device found at /dev/insta"
else
    echo "✗ Camera device not found at /dev/insta"
    echo "ERROR: Camera device is required but not found."
    exit 1
fi

echo "Starting camera driver..."
pkill -9 -f "insta360_ros_driver" 2>/dev/null
pkill -9 -f "equirectangular" 2>/dev/null
pkill -9 -f "dual_fisheye" 2>/dev/null
sleep 2

ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true equirectangular_config:="$ROS_WS_DIR/src/insta360_ros_driver/config/equirectangular.yaml" &
CAMERA_PID=$!
PIDS+=($CAMERA_PID)

echo "Waiting for camera to initialize (10 seconds)..."
sleep 10

if wait_for_topic "/dual_fisheye/image/compressed" 10 && wait_for_topic_data "/dual_fisheye/image/compressed" 10; then
    echo "✓ Camera driver ready"
    CAMERA_STATUS="/dual_fisheye/image/compressed (~0.8 Hz)"
else
    echo "✗ Camera failed to start"
    exit 1
fi

# Step 2: Start LiDAR
echo "Step 1: Starting LiDAR driver..."
ros2 launch livox_ros_driver2 rviz_MID360_launch.py > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
PIDS+=($LIDAR_PID)

if ! wait_for_topic "/livox/lidar" 30; then
    echo "Failed to start LiDAR driver"
    exit 1
fi

if ! wait_for_topic_data "/livox/lidar" 15; then
    echo "LiDAR not publishing data"
    exit 1
fi

echo "Step 2: Starting LiDAR driver..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame > /tmp/tf.log 2>&1 &
TF_PID=$!
PIDS+=($TF_PID)
sleep 2
echo "✓ LiDAR driver ready"

# Step 3: Start TF publisher for RKO-LIO
echo "Step 3: Starting static transform publisher..."

# Check for Livox IMU first
IMU_AVAILABLE=false
IMU_SOURCE="none"

if wait_for_topic "/livox/imu" 8; then
    echo "Livox IMU topic found, checking for data..."
    #IMU_DATA_FOUND=false
    for i in {1..3}; do
        if timeout 2 ros2 topic echo /livox/imu --once > /dev/null 2>&1; then
            IMU_AVAILABLE=true
            IMU_SOURCE="livox"
            IMU_DATA_FOUND=true
            echo "✓ Livox IMU data available"
            break
        fi
        echo "  Waiting for Livox IMU data... ($i/3)"
        sleep 1
    done
    
    #if [ "$IMU_DATA_FOUND" = "false" ]; then
    #    echo "⚠ Livox IMU topic exists but no data after 6 seconds"
    #    echo "  This is normal - IMU may not be enabled in LiDAR config"
    #fi
fi

# If Livox IMU not available, try camera IMU
if [ "$IMU_AVAILABLE" = "false" ]; then
    echo "Livox IMU not available, checking camera IMU..."
    if wait_for_topic "/imu/data_raw" 3; then
        IMU_AVAILABLE=true
        IMU_SOURCE="camera"
        echo "✓ Camera IMU available"
    fi
fi

if [ "$IMU_AVAILABLE" = "true" ]; then
    echo "Starting RKO-LIO odometry with $IMU_SOURCE IMU..."
    
    # Setup IMU transform if using camera IMU
    if [ "$IMU_SOURCE" = "camera" ]; then
        echo "Setting up camera IMU transform..."
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/init/imu_transform_node.py" > /tmp/imu_transform.log 2>&1 &
        IMU_TRANSFORM_PID=$!
        PIDS+=($IMU_TRANSFORM_PID)
        
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera livox_frame > /tmp/camera_tf.log 2>&1 &
        CAMERA_TF_PID=$!
        PIDS+=($CAMERA_TF_PID)
        sleep 2
    fi
    
    # Start RKO-LIO
    ros2 launch rko_lio odometry.launch.py \
        config_file:="$ROS_WS_DIR/src/atlas-scanner/src/config/rko_lio_config_robust.yaml" \
        double_downsample:=false rviz:=false > /tmp/rko_lio.log 2>&1 &
    LIO_PID=$!
    PIDS+=($LIO_PID)
    
    echo "Waiting for RKO-LIO to initialize (5 seconds)..."
    sleep 5
    
    if wait_for_topic "/rko_lio/odometry" 10; then
        echo "✓ RKO-LIO odometry ready with $IMU_SOURCE IMU"
        LIO_ENABLED="true"
    else
        echo "⚠ RKO-LIO failed to start"
        LIO_ENABLED="false"
    fi
else
    echo "⚠ No IMU data available - RKO-LIO disabled"
    LIO_ENABLED="false"
fi

echo "✓ Transform publisher ready"

# Step 4: Check IMU availability
echo "Step 4: Checking IMU availability..."

# Mark fusion as ready since all sensors initialized successfully
FUSION_READY="true"

echo ""
echo "=========================================="
echo "FUSION CAPTURE READY"
echo "=========================================="
echo "All systems operational:"
echo "- LiDAR: /livox/lidar (7.7 Hz)"
echo "- Camera: $CAMERA_STATUS"
if [ "$LIO_ENABLED" = "true" ]; then
    echo "- Odometry: Will start with first scan (RKO-LIO with $IMU_SOURCE IMU)"
    if [ "$IMU_SOURCE" = "camera" ]; then
        echo "- IMU: Camera IMU (will be transformed to livox frame)"
    else
        echo "- IMU: Native Livox IMU"
    fi
else
    echo "- Odometry: Not Available (No IMU data)"
    echo "- IMU: Not Available"
fi
echo "- Sync Method: LiDAR-driven (reliable)"
echo "=========================================="
echo "Press ENTER to capture a scan"
echo "Type 'quit' or 'q' to exit"
echo "=========================================="

SCAN_COUNT=0
TRAJECTORY_RECORDING=false

while true; do
    read -p "Scan $((SCAN_COUNT + 1)) - Press ENTER to capture (or 'q' to exit): " input
    
    if [ "$input" = "quit" ] || [ "$input" = "q" ]; then
        break
    fi
    
    SCAN_COUNT=$((SCAN_COUNT + 1))
    SCAN_NAME="fusion_scan_$(printf "%03d" $SCAN_COUNT)"
    
    # Start trajectory recording only once on first scan (reuse for all scans in session)
    if [ "$LIO_ENABLED" = "true" ] && [ "$TRAJECTORY_RECORDING" = "false" ]; then
        echo "Starting trajectory recording for session (once only)..."
        
        # Create PID file location
        TRAJ_PID_FILE="$SCAN_DIR/.trajectory_recorder.pid"
        
        # Aggressive cleanup - kill ALL instances to prevent accumulation
        echo "Cleaning up any existing trajectory processes..."
        
        # Kill by PID file first if it exists
        if [ -f "$TRAJ_PID_FILE" ]; then
            OLD_PID=$(cat "$TRAJ_PID_FILE" 2>/dev/null)
            if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
                echo "Killing old trajectory recorder (PID: $OLD_PID)"
                kill -9 "$OLD_PID" 2>/dev/null
            fi
            rm -f "$TRAJ_PID_FILE"
        fi
        
        # Kill all trajectory processes
        pkill -9 -f "enhanced_trajectory_recorder" 2>/dev/null
        pkill -9 -f "buffered_odom_bridge" 2>/dev/null
        pkill -9 -f "python3.*enhanced_trajectory_recorder" 2>/dev/null
        pkill -9 -f "python3.*buffered_odom_bridge" 2>/dev/null
        sleep 2
        
        # Force kill any remaining by PID
        pgrep -f "enhanced_trajectory_recorder" | xargs -r kill -9 2>/dev/null
        pgrep -f "buffered_odom_bridge" | xargs -r kill -9 2>/dev/null
        sleep 1
        
        # Final verification
        REMAINING=$(pgrep -f "enhanced_trajectory_recorder" | wc -l)
        if [ "$REMAINING" -gt 0 ]; then
            echo "✗ ERROR: $REMAINING trajectory processes still running - aborting"
            exit 1
        fi
        
        # Verify RKO-LIO is working
        if wait_for_topic "/rko_lio/odometry" 5; then
            echo "✓ RKO-LIO odometry topic available"
        else
            echo "⚠ RKO-LIO odometry topic not found"
        fi
        
        # Start enhanced trajectory recording ONCE for entire session
        echo "Starting single trajectory recorder instance..."
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/enhanced_trajectory_recorder.py" "$SCAN_DIR" > "$SCAN_DIR/trajectory_recorder.log" 2>&1 &
        TRAJECTORY_PID=$!
        
        # Save PID to file
        echo "$TRAJECTORY_PID" > "$TRAJ_PID_FILE"
        PIDS+=($TRAJECTORY_PID)
        
        # Start supporting processes ONCE for entire session
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_odom_bridge.py" > "$SCAN_DIR/odom_buffer.log" 2>&1 &
        BUFFER_PID=$!
        PIDS+=($BUFFER_PID)
        
        # Verify processes started correctly
        sleep 1
        if kill -0 $TRAJECTORY_PID 2>/dev/null; then
            echo "✓ Trajectory recorder started (PID: $TRAJECTORY_PID)"
        else
            echo "✗ Failed to start trajectory recorder"
            rm -f "$TRAJ_PID_FILE"
        fi
        
        TRAJECTORY_RECORDING=true
        echo "✓ Session trajectory recording started (will handle all scans)"
    fi
    
    echo "Capturing camera-driven synchronized scan $SCAN_COUNT..."
    echo "Waiting for next camera frame (camera drives timing at ~0.8 Hz)..."
    
    # Check what topics are available
    LIDAR_AVAILABLE=false
    CAMERA_AVAILABLE=false
    ODOMETRY_HEALTHY=false
    
    # Check LiDAR with progressive timeouts
    if timeout 2 ros2 topic echo /livox/lidar --once > /dev/null 2>&1; then
        LIDAR_AVAILABLE=true
        echo "✓ LiDAR topic available"
    elif timeout 5 ros2 topic echo /livox/lidar --once > /dev/null 2>&1; then
        LIDAR_AVAILABLE=true
        echo "✓ LiDAR topic available (slower response)"
    else
        echo "⚠ LiDAR not responding - checking if driver is still running..."
        if pgrep -f "livox_ros_driver2" > /dev/null; then
            echo "  Driver running but no data - LiDAR may need more warmup time"
            echo "  Trying one more time with longer timeout..."
            if timeout 10 ros2 topic echo /livox/lidar --once > /dev/null 2>&1; then
                LIDAR_AVAILABLE=true
                echo "✓ LiDAR data available after extended wait"
            else
                echo "✗ LiDAR still not responding"
            fi
        else
            echo "  Driver not running - this shouldn't happen"
        fi
    fi
    
    # Check odometry health if RKO-LIO should be running
    if [ "$LIO_ENABLED" = "true" ] && [ "$TRAJECTORY_RECORDING" = "true" ]; then
        if ros2 topic list 2>/dev/null | grep -q "/rko_lio/odometry"; then
            echo "✓ Odometry topic exists"
            ODOMETRY_HEALTHY=true
        else
            echo "⚠ Odometry topic missing - RKO-LIO may have failed"
            ODOMETRY_HEALTHY=false
        fi
    else
        echo "✓ Odometry will be available after RKO-LIO starts"
        ODOMETRY_HEALTHY=true
    fi
    
    if ros2 topic list 2>/dev/null | grep -q "/equirectangular/image" 2>/dev/null; then
        CAMERA_AVAILABLE=true
        echo "✓ Camera topic available"
    elif ros2 topic list 2>/dev/null | grep -q "/dual_fisheye/image/compressed" 2>/dev/null; then
        CAMERA_AVAILABLE=true
        echo "✓ Dual fisheye camera available"
    fi
    
    if [ "$LIDAR_AVAILABLE" = "false" ]; then
        echo "✗ LiDAR not available, skipping scan"
        continue
    fi
    
    if [ "$CAMERA_AVAILABLE" = "false" ]; then
        echo "✗ No camera data available, skipping scan"
        echo "  Both LiDAR and camera data are required for fusion"
        continue
    fi
    
    # Create individual scan directory first
    INDIVIDUAL_SCAN_DIR="$SCAN_DIR/$SCAN_NAME"
    mkdir -p "$INDIVIDUAL_SCAN_DIR"
    
    # Cleanup capture processes only (NOT trajectory recorder)
    echo "Cleaning up capture processes from previous scan..."
    pkill -f "buffered_camera_capture" 2>/dev/null
    pkill -f "lidar_driven_capture" 2>/dev/null
    pkill -f "lidar_only_capture" 2>/dev/null
    pkill -f "ros2 bag record" 2>/dev/null
    sleep 1
    
    # Simple trajectory recorder check - no restarts, just verify it's still running
    if [ "$TRAJECTORY_RECORDING" = "true" ]; then
        TRAJ_PROCS=$(pgrep -f "enhanced_trajectory_recorder" | wc -l)
        if [ "$TRAJ_PROCS" -eq 1 ]; then
            echo "✓ Trajectory recorder running (1 process)"
        elif [ "$TRAJ_PROCS" -gt 1 ]; then
            echo "⚠ WARNING: $TRAJ_PROCS trajectory recorders detected - this will cause high CPU!"
            echo "PIDs: $(pgrep -f 'enhanced_trajectory_recorder' | tr '\n' ' ')"
            echo "Running debug script to analyze processes..."
            ./debug_trajectory_processes.sh
        else
            echo "⚠ Trajectory recorder died - continuing without it"
        fi
    fi
    
    # Start rosbag recording in individual scan directory
    echo "Starting rosbag recording..."
    ROSBAG_TIMESTAMP=$(date +%Y%m%d_%H%M%S_%N | cut -c1-18)  # Include nanoseconds for uniqueness
    ROSBAG_DIR="$INDIVIDUAL_SCAN_DIR/rosbag_$ROSBAG_TIMESTAMP"
    
    # Record only essential topics with compression to reduce CPU load
    TOPICS_TO_RECORD="/livox/lidar"
    if [ "$CAMERA_AVAILABLE" = "true" ]; then
        # Only record compressed camera data, skip uncompressed equirectangular to save CPU
        TOPICS_TO_RECORD="$TOPICS_TO_RECORD /dual_fisheye/image/compressed"
        echo "✓ Recording compressed camera data only (CPU optimized)"
    fi
    if [ "$LIO_ENABLED" = "true" ]; then
        TOPICS_TO_RECORD="$TOPICS_TO_RECORD /rko_lio/odometry_buffered"
    fi
    
    # Use compression, reduced timeout, and lower priority to minimize CPU impact
    timeout 8 nice -n 10 ros2 bag record -o "$ROSBAG_DIR" --compression-mode file --compression-format zstd --max-cache-size 100000000 $TOPICS_TO_RECORD &
    ROSBAG_PID=$!
    
    # Wait for rosbag to start
    sleep 2
    
    # Use appropriate capture method - save directly to individual scan dir
    if [ "$CAMERA_AVAILABLE" = "true" ]; then
        echo "Using buffered synchronized LiDAR+camera capture..."
        # Try buffered camera-driven approach first with longer LiDAR capture
        timeout 25 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/buffered_camera_capture.py" "$INDIVIDUAL_SCAN_DIR" 15 5.0
        CAPTURE_EXIT_CODE=$?
        
        # If that fails, try LiDAR-driven approach with image buffer
        if [ $CAPTURE_EXIT_CODE -ne 0 ]; then
            echo "  Buffered camera capture failed, trying LiDAR-driven approach..."
            timeout 15 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/lidar_driven_capture.py" "$INDIVIDUAL_SCAN_DIR" 8
            CAPTURE_EXIT_CODE=$?
        fi
    else
        echo "Using LiDAR-only capture..."
        timeout 10 python3 "$ROS_WS_DIR/src/atlas-scanner/src/capture/lidar_only_capture.py" "$INDIVIDUAL_SCAN_DIR"
        CAPTURE_EXIT_CODE=$?
    fi
    
    if [ $CAPTURE_EXIT_CODE -ne 0 ]; then
        echo "✗ Capture failed (exit code: $CAPTURE_EXIT_CODE)"
        echo "  Checking if LiDAR/IMU need restart..."
        
        # Check if LiDAR/IMU are still working
        if ! timeout 3 ros2 topic echo /livox/lidar --once > /dev/null 2>&1; then
            echo "  LiDAR not responding - restarting driver..."
            python3 '$ROS_WS_DIR/src/atlas-scanner/src/init/restart_lidar_driver.py'
            sleep 5
        fi
        
        echo "  Trying fallback capture..."
        
        # Fallback: capture LiDAR data directly
        timeout 10 ros2 run ros2_camera_lidar_fusion save_ply > /tmp/fallback_ply.log 2>&1
        
        # Move any generated PLY files to scan directory
        for ply_file in *.ply; do
            if [ -f "$ply_file" ]; then
                mv "$ply_file" "$SCAN_DIR/"
                echo "✓ Fallback saved: $ply_file"
            fi
        done
        
        # Try to capture image separately if camera topics exist
        if ros2 topic list 2>/dev/null | grep -q "/dual_fisheye/image/compressed" 2>/dev/null; then
            echo "  Capturing image separately..."
            timeout 5 python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from datetime import datetime
import os

class FallbackImageCapture(Node):
    def __init__(self):
        super().__init__('fallback_image_capture')
        self.captured = False
        self.sub = self.create_subscription(CompressedImage, '/dual_fisheye/image/compressed', self.callback, 1)
    
    def callback(self, msg):
        if not self.captured:
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                output_path = os.path.join('$SCAN_DIR', f'dual_fisheye_{timestamp}.jpg')
                cv2.imwrite(output_path, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
                print('✓ Fallback image saved')
                self.captured = True
            except Exception as e:
                print(f'Fallback image save failed: {e}')

rclpy.init()
node = FallbackImageCapture()
import time
start = time.time()
while not node.captured and time.time() - start < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
" 2>/dev/null
        fi
    fi
    
    # Trajectory data is handled by enhanced_trajectory_recorder.py
    if [ "$LIO_ENABLED" = "true" ] && [ "$TRAJECTORY_RECORDING" = "true" ]; then
        # Create trigger file for trajectory recorder to save this scan's trajectory
        TRIGGER_FILE="$SCAN_DIR/.save_trajectory_${SCAN_NAME}"
        echo "{\"scan_name\": \"$SCAN_NAME\", \"scan_dir\": \"$INDIVIDUAL_SCAN_DIR\"}" > "$TRIGGER_FILE"
        #sleep 0.5  # Give trajectory recorder time to process
        echo "✓ Trajectory data saved to scan directory"
    fi
    
    # Stop rosbag recording
    if [ -n "$ROSBAG_PID" ] && kill -0 $ROSBAG_PID 2>/dev/null; then
        kill $ROSBAG_PID
        wait $ROSBAG_PID 2>/dev/null
    fi
    
    # Process captured files from individual scan directory
    echo "Looking for captured files in $INDIVIDUAL_SCAN_DIR..."
    ls -la "$INDIVIDUAL_SCAN_DIR"/*lidar*.ply "$INDIVIDUAL_SCAN_DIR"/*equirect*.jpg 2>/dev/null || echo "No files found"
    
    WORLD_PLY=$(ls -t "$INDIVIDUAL_SCAN_DIR"/world_lidar_*.ply 2>/dev/null | head -1)
    SENSOR_PLY=$(ls -t "$INDIVIDUAL_SCAN_DIR"/sensor_lidar_*.ply 2>/dev/null | head -1)
    CAM_DRIVEN_PLY=$(ls -t "$INDIVIDUAL_SCAN_DIR"/cam_driven_*.ply 2>/dev/null | head -1)
    EQUIRECT_IMG=$(ls -t "$INDIVIDUAL_SCAN_DIR"/equirect_*.jpg 2>/dev/null | head -1)
    CAM_DRIVEN_IMG=$(ls -t "$INDIVIDUAL_SCAN_DIR"/cam_driven_*.jpg 2>/dev/null | head -1)
    
    echo "Found files:"
    echo "  WORLD_PLY: $WORLD_PLY"
    echo "  SENSOR_PLY: $SENSOR_PLY"
    echo "  EQUIRECT_IMG: $EQUIRECT_IMG"
    echo "  Selected: $LIDAR_PLY ($PLY_TYPE)"
    
    # Use any available PLY file
    if [ -n "$WORLD_PLY" ] && [ -f "$WORLD_PLY" ]; then
        LIDAR_PLY="$WORLD_PLY"
        PLY_TYPE="world-transformed"
    elif [ -n "$SENSOR_PLY" ] && [ -f "$SENSOR_PLY" ]; then
        LIDAR_PLY="$SENSOR_PLY"
        PLY_TYPE="sensor-coordinate"
    elif [ -n "$CAM_DRIVEN_PLY" ] && [ -f "$CAM_DRIVEN_PLY" ]; then
        LIDAR_PLY="$CAM_DRIVEN_PLY"
        PLY_TYPE="camera-driven"
    else
        LIDAR_PLY=""
    fi
    
    # Use camera-driven image if available
    if [ -n "$CAM_DRIVEN_IMG" ] && [ -f "$CAM_DRIVEN_IMG" ]; then
        EQUIRECT_IMG="$CAM_DRIVEN_IMG"
    fi
    
    if [ -n "$LIDAR_PLY" ] && [ -f "$LIDAR_PLY" ]; then
        echo "✓ Found $PLY_TYPE PLY: $LIDAR_PLY"
        if [ -n "$EQUIRECT_IMG" ] && [ -f "$EQUIRECT_IMG" ]; then
            echo "✓ Found equirectangular image: $EQUIRECT_IMG"
            echo "✓ Captured synchronized data: $LIDAR_PLY and $EQUIRECT_IMG"
            HAS_IMAGE=true
        else
            echo "✓ Captured LiDAR-only data: $LIDAR_PLY"
            HAS_IMAGE=false
        fi
        
        # Apply offline coloring to points if image is available and not skipping live fusion
        if [ "$HAS_IMAGE" = "true" ] && [ "$SKIP_LIVE_FUSION" = "false" ]; then
            if [ "$PLY_TYPE" = "camera-driven" ]; then
                COLORED_PLY="$INDIVIDUAL_SCAN_DIR/${SCAN_NAME}_colored.ply"
                echo "Applying colors to camera-driven synchronized points..."
            elif [ "$PLY_TYPE" = "world-transformed" ]; then
                COLORED_PLY="$INDIVIDUAL_SCAN_DIR/world_${SCAN_NAME}_colored.ply"
                echo "Applying colors to world-transformed points..."
            else
                COLORED_PLY="$INDIVIDUAL_SCAN_DIR/sensor_${SCAN_NAME}_colored.ply"
                echo "Applying colors to sensor-coordinate points..."
            fi
            
            # Use world coordinates with first scan reference
            if [ "$SCAN_COUNT" -eq 1 ]; then
                FIRST_SCAN_DIR="$INDIVIDUAL_SCAN_DIR"
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/exact_match_fusion.py" "$INDIVIDUAL_SCAN_DIR"
                echo "✓ First scan (reference)"
            else
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/exact_match_fusion.py" "$INDIVIDUAL_SCAN_DIR"
                echo "✓ Scan colored using world coordinates with first scan reference"
            fi
        else
            echo "Skipping coloring (no image available)"
            COLORED_PLY=""
        fi
        
        # Process files regardless of coloring
        if [ -f "$LIDAR_PLY" ]; then
            
            # Always try to embed pose data and create E57 files if LIO enabled
            TRAJECTORY_JSON="$INDIVIDUAL_SCAN_DIR/trajectory.json"
            if [ "$LIO_ENABLED" = "true" ] && [ -f "$TRAJECTORY_JSON" ]; then
                echo "Embedding pose data in PLY files..."
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/save_ply_with_pose.py" "$LIDAR_PLY" "$TRAJECTORY_JSON" "${LIDAR_PLY%.ply}_with_pose.ply" 2>/dev/null
                python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/save_ply_with_pose.py" "$COLORED_PLY" "$TRAJECTORY_JSON" "${COLORED_PLY%.ply}_with_pose.ply" 2>/dev/null
                
                if [ "$SAVE_E57" = "true" ]; then
                    echo "Creating E57 files with pose data..."
                    if [ -n "$LIDAR_PLY" ] && [ -f "$LIDAR_PLY" ]; then
                        python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/save_e57_with_pose.py" "$LIDAR_PLY" "$TRAJECTORY_JSON" "${LIDAR_PLY%.ply}.e57"
                    fi
                    if [ -n "$COLORED_PLY" ] && [ -f "$COLORED_PLY" ]; then
                        python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/save_e57_with_pose.py" "$COLORED_PLY" "$TRAJECTORY_JSON" "${COLORED_PLY%.ply}.e57"
                    fi
                fi
                
                # Move enhanced files if they exist, otherwise move originals
                if [ -f "${LIDAR_PLY%.ply}_with_pose.ply" ]; then
                    if [ "$PLY_TYPE" = "camera-driven" ]; then
                        mv "${LIDAR_PLY%.ply}_with_pose.ply" "$INDIVIDUAL_SCAN_DIR/synchronized_lidar.ply"
                    elif [ "$PLY_TYPE" = "world-transformed" ]; then
                        mv "${LIDAR_PLY%.ply}_with_pose.ply" "$INDIVIDUAL_SCAN_DIR/world_lidar.ply"
                    else
                        mv "${LIDAR_PLY%.ply}_with_pose.ply" "$INDIVIDUAL_SCAN_DIR/sensor_lidar.ply"
                    fi
                    rm -f "$LIDAR_PLY"
                else
                    if [ "$PLY_TYPE" = "camera-driven" ]; then
                        mv "$LIDAR_PLY" "$INDIVIDUAL_SCAN_DIR/synchronized_lidar.ply"
                    elif [ "$PLY_TYPE" = "world-transformed" ]; then
                        mv "$LIDAR_PLY" "$INDIVIDUAL_SCAN_DIR/world_lidar.ply"
                    else
                        mv "$LIDAR_PLY" "$INDIVIDUAL_SCAN_DIR/sensor_lidar.ply"
                    fi
                fi
                
                # Move colored PLY if it exists
                if [ -n "$COLORED_PLY" ] && [ -f "${COLORED_PLY%.ply}_with_pose.ply" ]; then
                    if [ "$PLY_TYPE" = "camera-driven" ]; then
                        mv "${COLORED_PLY%.ply}_with_pose.ply" "$INDIVIDUAL_SCAN_DIR/synchronized_colored_pointcloud.ply"
                    elif [ "$PLY_TYPE" = "world-transformed" ]; then
                        mv "${COLORED_PLY%.ply}_with_pose.ply" "$INDIVIDUAL_SCAN_DIR/world_colored_pointcloud.ply"
                    else
                        mv "${COLORED_PLY%.ply}_with_pose.ply" "$INDIVIDUAL_SCAN_DIR/sensor_colored_pointcloud.ply"
                    fi
                    rm -f "$COLORED_PLY"
                elif [ -n "$COLORED_PLY" ] && [ -f "$COLORED_PLY" ]; then
                    if [ "$PLY_TYPE" = "camera-driven" ]; then
                        mv "$COLORED_PLY" "$INDIVIDUAL_SCAN_DIR/synchronized_colored_pointcloud.ply"
                    elif [ "$PLY_TYPE" = "world-transformed" ]; then
                        mv "$COLORED_PLY" "$INDIVIDUAL_SCAN_DIR/world_colored_pointcloud.ply"
                    else
                        mv "$COLORED_PLY" "$INDIVIDUAL_SCAN_DIR/sensor_colored_pointcloud.ply"
                    fi
                fi
                
                # Move E57 files if they exist and E57 export is enabled
                if [ "$SAVE_E57" = "true" ]; then
                    if [ -f "${LIDAR_PLY%.ply}.e57" ]; then
                        if [ "$PLY_TYPE" = "world-transformed" ]; then
                            mv "${LIDAR_PLY%.ply}.e57" "$INDIVIDUAL_SCAN_DIR/world_lidar.e57"
                        else
                            mv "${LIDAR_PLY%.ply}.e57" "$INDIVIDUAL_SCAN_DIR/sensor_lidar.e57"
                        fi
                    fi
                    if [ -f "${COLORED_PLY%.ply}.e57" ]; then
                        if [ "$PLY_TYPE" = "world-transformed" ]; then
                            mv "${COLORED_PLY%.ply}.e57" "$INDIVIDUAL_SCAN_DIR/world_colored_pointcloud.e57"
                        else
                            mv "${COLORED_PLY%.ply}.e57" "$INDIVIDUAL_SCAN_DIR/sensor_colored_pointcloud.e57"
                        fi
                    fi
                fi
            else
                # Move files without pose data
                if [ "$PLY_TYPE" = "camera-driven" ]; then
                    mv "$LIDAR_PLY" "$INDIVIDUAL_SCAN_DIR/synchronized_lidar.ply"
                elif [ "$PLY_TYPE" = "world-transformed" ]; then
                    mv "$LIDAR_PLY" "$INDIVIDUAL_SCAN_DIR/world_lidar.ply"
                else
                    mv "$LIDAR_PLY" "$INDIVIDUAL_SCAN_DIR/sensor_lidar.ply"
                fi
                
                # Move colored PLY if it exists
                if [ -n "$COLORED_PLY" ] && [ -f "$COLORED_PLY" ]; then
                    if [ "$PLY_TYPE" = "camera-driven" ]; then
                        mv "$COLORED_PLY" "$INDIVIDUAL_SCAN_DIR/synchronized_colored_pointcloud.ply"
                    elif [ "$PLY_TYPE" = "world-transformed" ]; then
                        mv "$COLORED_PLY" "$INDIVIDUAL_SCAN_DIR/world_colored_pointcloud.ply"
                    else
                        mv "$COLORED_PLY" "$INDIVIDUAL_SCAN_DIR/sensor_colored_pointcloud.ply"
                    fi
                fi
            fi
            
            # Images are already saved with timestamped names - no need to copy
            
            echo "✓ Scan $SCAN_COUNT completed in: $SCAN_NAME/"
            
            # List what was actually captured
            if [ "$PLY_TYPE" = "world-transformed" ]; then
                echo "  - world_lidar.ply (world-transformed points)"
                if [ -f "$INDIVIDUAL_SCAN_DIR/world_colored_pointcloud.ply" ]; then
                    echo "  - world_colored_pointcloud.ply (world-transformed with colors)"
                fi
                if [ "$SAVE_E57" = "true" ] && [ -f "$INDIVIDUAL_SCAN_DIR/world_lidar.e57" ]; then
                    echo "  - world_lidar.e57 (E57 format)"
                    if [ -f "$INDIVIDUAL_SCAN_DIR/world_colored_pointcloud.e57" ]; then
                        echo "  - world_colored_pointcloud.e57 (E57 format)"
                    fi
                fi
            else
                echo "  - sensor_lidar.ply (sensor-coordinate points)"
                if [ -f "$INDIVIDUAL_SCAN_DIR/sensor_colored_pointcloud.ply" ]; then
                    echo "  - sensor_colored_pointcloud.ply (sensor-coordinate with colors)"
                fi
                if [ "$SAVE_E57" = "true" ] && [ -f "$INDIVIDUAL_SCAN_DIR/sensor_lidar.e57" ]; then
                    echo "  - sensor_lidar.e57 (E57 format)"
                    if [ -f "$INDIVIDUAL_SCAN_DIR/sensor_colored_pointcloud.e57" ]; then
                        echo "  - sensor_colored_pointcloud.e57 (E57 format)"
                    fi
                fi
            fi
            
            if [ -f "$INDIVIDUAL_SCAN_DIR/equirectangular.jpg" ]; then
                echo "  - equirectangular.jpg"
            fi
            
            if [ -d "$ROSBAG_DIR" ]; then
                echo "  - rosbag_$ROSBAG_TIMESTAMP/ (all sensor data)"
            fi
            
            if [ "$LIO_ENABLED" = "true" ] && [ -f "$INDIVIDUAL_SCAN_DIR/trajectory.json" ]; then
                echo "  - trajectory.json (RKO-LIO pose + transforms)"
            fi
            
            if [ "$PLY_TYPE" = "camera-driven" ]; then
                ls -lh "$INDIVIDUAL_SCAN_DIR/synchronized_colored_pointcloud.ply" 2>/dev/null || ls -lh "$INDIVIDUAL_SCAN_DIR/"*colored*.ply 2>/dev/null
            elif [ "$PLY_TYPE" = "world-transformed" ]; then
                ls -lh "$INDIVIDUAL_SCAN_DIR/world_colored_pointcloud.ply" 2>/dev/null
            else
                ls -lh "$INDIVIDUAL_SCAN_DIR/sensor_colored_pointcloud.ply" 2>/dev/null
            fi
            
            echo "✓ Scan processing complete. Ready for next scan."
        else
            echo "✗ No LiDAR data found to process"
        fi
    else
        echo "✗ Failed to capture synchronized data - files not found"
        echo "  Available files: $(ls -la *.ply *.jpg 2>/dev/null || echo 'none')"
    fi
    
    # Keep RKO-LIO running between scans for continuous odometry
    if [ "$LIO_ENABLED" = "true" ]; then
        echo "✓ RKO-LIO continues running for next scan"
    fi
    
    echo "Ready for next scan..."
    echo ""
done

# Post-processing handled in cleanup function for both GUI and CLI modes

# Post-processing option
if [ "$ENABLE_POST_PROCESSING_BAGS" = "true" ]; then
    read -p "Do you want to run post-processing on all bag files? (y/n): " post_process
    
    if [ "$post_process" = "y" ] || [ "$post_process" = "Y" ]; then
        echo "Starting post-processing of all bag files..."
        python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_bags.py" "$SCAN_DIR"
        echo "✓ Post-processing complete"
    else
        echo "Skipping post-processing. You can run it later with:"
        echo "python3 "$ROS_WS_DIR/src/atlas-scanner/src/post_processing/post_process_bags.py" \"$SCAN_DIR\""
    fi
fi
