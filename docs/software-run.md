# Running the Software

## Configuration

Edit the settings at the top of `~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh` before running:

```bash
CAMERA_MODE="dual_fisheye"     # dual_fisheye | single_fisheye
CAPTURE_MODE="continuous"      # stationary | continuous
CONTINUOUS_INTERVAL=3          # seconds between captures (continuous mode only;
                               # host-controlled TakePhoto() loop; minimum ~3s
                               # limited by X5 shutter + SD write time)
CAMERA_HW="x5"                 # onex2 | x5

SAVE_E57=false                 # export E57 files
USE_EXISTING_CALIBRATION=false # skip calibration update from calib.json
ENABLE_ICP_ALIGNMENT=true      # ICP pose graph refinement on merged cloud
EXPORT_COLMAP=false            # run panorama SfM (COLMAP) after session
CLEAN_POINTCLOUD=true          # statistical outlier removal on merged cloud
DOWNSAMPLE_VOXEL_SIZE=0.05     # voxel downsample in metres (0 = skip)
RUN_SYNC_BENCHMARK=true        # run sync benchmark after every session
```

| `CAMERA_MODE` | `CAPTURE_MODE` | Use case |
|---|---|---|
| `dual_fisheye` | `stationary` | Tripod scanning, full 360° ERP images |
| `single_fisheye` | `stationary` | Tripod scanning, LiDAR-facing fisheye only |
| `dual_fisheye` | `continuous` | Walking capture, full 360° images |
| `single_fisheye` | `continuous` | Walking capture, LiDAR-facing fisheye only |

All modes use the Insta360 CameraSDK + MediaSDK exclusively — the ROS camera driver is not used. The `insta360_capture` daemon takes ownership of the USB device for the whole session, captures `.insp` files via a host-controlled `TakePhoto()` loop (continuous) or single `TakePhoto` calls (stationary), and `insta360_stitch` produces the ERP during post-processing. For `single_fisheye`, the full 360° ERP is output with the rear hemisphere blank; a per-hardware LiDAR mask is then applied to exclude the blank region and scanner body before coloring.

**How continuous mode works:**

- The daemon drives the shutter from a host timer thread calling `TakePhoto()` at the configured `CONTINUOUS_INTERVAL` (default 3s). The X5 firmware minimum is ~3s per shot (shutter + SD write). Each shot's shutter time is recorded at `TakePhoto()` return (~50ms accuracy on host clock), then the `.insp` file is downloaded via HTTP GET from the camera's built-in file server while the session is still active.
- The `shutter_event_publisher.py` node watches for `capture_N.shutter_event` files and publishes on `/camera/shutter_time`, recorded into the bag. `reconstruct_from_bag.py` uses these timestamps (or `.insp.capture_time` sidecars) as scan centres.
- Camera IMU is not available in SDK stitch mode.

**USB session management:**

- After a successful session, the X5 firmware leaves residual protocol data in its USB bulk endpoint. The `insta360_capture` daemon flushes stale bytes from endpoint 0x81 using `libusb` before SDK device discovery, eliminating the need to physically unplug the camera between sessions.
- The shell script performs a `USBDEVFS_RESET` at startup to force the firmware to restart its command server, followed by an 8s settle time for the X5.

**Building the SDK tools:**

```bash
cd ~/atlas_ws/src/atlas-scanner/src/capture/sdk
bash build.sh
# Produces: ~/insta360-dev/build/insta360_capture
#           ~/insta360-dev/build/insta360_stitch
#           ~/insta360-dev/build/insta360_reset_clock
```

Dependencies: `libusb-1.0-dev`, Insta360 CameraSDK, Insta360 MediaSDK, OpenCV.
Re-run after any update to `main.cpp`, `stitch.cpp`, or `reset_clock.cpp`.

**Manually reconstruct a session:**

```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --camera-mode dual_fisheye --sdk-stitch \
    --interval 5.0 --lidar-window 0.3

# For single_fisheye:
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --camera-mode single_fisheye --sdk-stitch \
    --interval 5.0 --lidar-window 0.3
```

**Run ICP alignment on a session:**

```bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/align_scan_session_posegraph.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --iterations 1
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/merge_with_trajectory.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
```

## Normal Operation

- Ensure sensors are connected
  - Verify LiDAR
    ```bash
    cd ~/atlas_ws && source install/setup.bash
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py
    ```
    `Ctrl+C`
    > *Note: In `RVIZ->Displays->Global Options`, change the `Fixed Frame` to `map`*

  - Verify camera
    ```bash
    cd ~/atlas_ws && source install/setup.bash
    sudo ~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh
    ~/insta360-dev/build/insta360_capture
    # Should print: Found camera: <serial> ... Camera session open
    # Ctrl+C to exit
    ```

- Run the software

  - Headless
    ```bash
    ~/atlas_ws/src/atlas-scanner/src/run_headless.sh
    ```

  - GUI
    ```bash
    ~/atlas_ws/src/atlas-scanner/src/run_gui.sh
    ```

  - Directly (same as headless)
    ```bash
    ~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh

    # Optional CLI overrides (take precedence over in-file config):
    ~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh --camera dual_fisheye --capture continuous
    ~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh --camera single_fisheye --capture stationary
    ~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh --capture continuous --interval 5
    ~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh --camera-hw x5
    ~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh --bag-only           # record bag, skip post-processing
    ~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh --no-sync-benchmark  # skip sync benchmark
    ```

- In **stationary** mode: press `ENTER` to capture each scan, `q` to finish
- In **continuous** mode: walk the space, press `Ctrl+C` when done — scans are reconstructed automatically from the recorded bag

- Session data is saved to `~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}/`

![Running the GUI](../assets/media/atlas-software-screenshot.png)

## Advanced Operations
### Reconstruct scans from a bag manually (continuous mode)

Requires this structure inside the session directory:
```
sync_fusion_{TIMESTAMP}/
└── rosbag_{TIMESTAMP}/        # must be named rosbag_*
    └── *.db3  (or *.db3.zstd) # bag must contain:
                               #   /livox/lidar
                               #   /camera/shutter_time  (SDK shutter events)
                               #   /rko_lio/odometry     (optional, for trajectory poses)
```

```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --camera-mode dual_fisheye --sdk-stitch \
    --interval 5.0 --lidar-window 0.5

# For single_fisheye mode:
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --camera-mode single_fisheye --sdk-stitch \
    --interval 5.0 --lidar-window 0.5
```

### Reprocess an existing session with a new calibration

If the extrinsic calibration has been updated (e.g. after re-running `coordinate_transform.py`), use this to re-color point clouds and regenerate COLMAP poses from the corrected `fusion_calibration.yaml` without re-capturing:

```bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reprocess_session.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
```

Optional flags:
```bash
--skip-coloring   # skip re-coloring, only redo COLMAP export
--skip-colmap     # skip COLMAP export, only redo point cloud coloring
```

To reprocess all sessions at once:
```bash
for session in ~/atlas_ws/data/synchronized_scans/sync_fusion_*; do
    python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reprocess_session.py "$session"
done
```

### Re-run COLMAP export only (fastest - uses existing colored point clouds)

If you only need to regenerate the COLMAP reconstruction without re-coloring point clouds:

```bash
cd ~/atlas_ws/src/atlas-scanner/src/post_processing
python3 panorama_sfm_colmap.py ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
```

Alternatively, use `reprocess_session.py` with the `--skip-coloring` flag:
```bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reprocess_session.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} --skip-coloring
```

### Merge scans using trajectory poses
```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/merge_with_trajectory.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
```

### Merge scans with ICP alignment
```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/align_scan_session_posegraph.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --max-gyro 0.5 --iterations 1
```

Or via the capture script with `ENABLE_ICP_ALIGNMENT=true` (uses `align_scan_session_posegraph.py`).

### View a point cloud in the web viewer
```bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/web_3d_viewer.py <PLY_FILE>
```

### View per-scan alignment in the toggle viewer
```bash
# Shows each scan as a separate toggleable layer using its trajectory (or ICP-refined) pose.
# Double-click a scan row to isolate it and inspect color-geometry alignment.
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/scan_toggle_viewer.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
```

### Re-run COLMAP panorama SfM on an existing session

To completely regenerate the COLMAP reconstruction (removes existing COLMAP data first):

```bash
SESSION=~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
rm -rf $SESSION/colmap
cd ~/atlas_ws/src/atlas-scanner/src/post_processing
python3 panorama_sfm_colmap.py $SESSION
```

This uses the colmap `panorama_sfm.py` approach: ERP images are sliced into perspective tiles (SIMPLE_PINHOLE), grouped into a rig per panorama, triangulated with known LiDAR poses, and refined with rig-aware bundle adjustment. Exhaustive matching is used by default since sessions are sparse.

Optional flags:
```bash
python3 panorama_sfm_colmap.py $SESSION --sequential        # sequential matcher (faster, for large sessions)
python3 panorama_sfm_colmap.py $SESSION --no-bundle-adjustment  # skip rig-aware bundle adjustment
```

### Run synchronization benchmark on a session
```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/sync_benchmark.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}

# Optional flags:
#   --walk-speed 0.5   assumed walking speed in m/s (default: 0.5)
#   --out report.json  save full report to a JSON file
```
