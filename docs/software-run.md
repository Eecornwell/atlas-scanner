# Running the Software

## Configuration

Edit the settings at the top of `~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh` before running:

```bash
CAMERA_MODE="single_fisheye"   # dual_fisheye | single_fisheye
CAPTURE_MODE="continuous"      # stationary | continuous
CONTINUOUS_INTERVAL=5          # seconds between captures (continuous mode only;
                               # SDK stitch dual_fisheye: effective interval is
                               # download-limited to ~10s regardless of this setting)

SAVE_E57=false                 # export E57 files
USE_EXISTING_CALIBRATION=false # skip calibration update from calib.json
ENABLE_ICP_ALIGNMENT=true      # ICP pose graph refinement on merged cloud
BLEND_ERP_SEAMS=true           # dual_fisheye only: blend fisheye seams before coloring
EXPORT_COLMAP=false            # run panorama SfM (COLMAP) after session
CLEAN_POINTCLOUD=true          # statistical outlier removal on merged cloud
DOWNSAMPLE_VOXEL_SIZE=0.05     # voxel downsample in metres (0 = skip)
RUN_SYNC_BENCHMARK=true        # run sync benchmark after every session (outputs to screen + sync_benchmark.json)
```

| `CAMERA_MODE` | `CAPTURE_MODE` | Use case |
|---|---|---|
| `dual_fisheye` | `stationary` | Tripod scanning, full 360° ERP images |
| `single_fisheye` | `stationary` | Tripod scanning, single fisheye |
| `single_fisheye` | `continuous` | Walking capture, auto-captures every N seconds |
| `dual_fisheye` | `continuous` | Walking capture with full 360° images |

### SDK Stitch Mode (`USE_SDK_STITCH`)

When `CAMERA_MODE=dual_fisheye`, an additional option enables high-quality ERP stitching via the Insta360 MediaSDK:

```bash
USE_SDK_STITCH=true   # use Insta360 MediaSDK stitcher instead of ROS driver (dual_fisheye only)
```

| Mode | ERP source | Quality | Notes |
|---|---|---|---|
| `USE_SDK_STITCH=false` | ROS driver (real-time) | Good | Standard path, camera IMU available |
| `USE_SDK_STITCH=true` | Insta360 MediaSDK (post-session) | Best | Higher quality stitching, no camera IMU in bag |

**How it works (continuous mode):**

- The ROS camera driver is not started. Instead, `insta360_capture` (a small C++ daemon built from `src/capture/sdk/`) takes ownership of the USB device.
- In continuous mode, the daemon fires repeated `TakePhoto()` calls — one shot at a time, blocking until each download completes (~10s). This ensures each `.insp` file embeds its own per-shot IMU orientation data, which the SDK stitcher uses to produce an ERP correctly oriented to the camera’s physical frame at that exact moment. Using the SDK’s timelapse mode instead would fix the IMU reference at the first shot, causing ERP yaw drift as the scanner rotates and breaking color alignment.
- Shutter time = `insp_rtc` (integer UTC second from `.insp` filename, from `SyncLocalTimeToCamera`) + `sync_frac` (sub-second offset of system clock at sync time). This gives <1 ms timing accuracy on the ROS system clock without hardware sync.
- Each `.insp` file is stitched to a full 5760×2880 ERP JPEG using `TEMPLATE` stitch mode (no gravity correction, camera-frame orientation) by `insta360_stitch` during reconstruction.
- The `shutter_event_publisher.py` node watches for `.shutter_event` files and publishes them on `/camera/shutter_time`, which is recorded into the bag. `reconstruct_from_bag.py` uses these timestamps as scan centres.
- Because each `TakePhoto()` blocks for the USB download, the effective capture interval is ~10s regardless of `CONTINUOUS_INTERVAL`. The scanner should be held still for ~0.5s around each shot for best results.

**ERP orientation (continuous mode):**

Each `.insp` file is stitched independently using `TEMPLATE` mode (no gravity/IMU stabilisation applied by the stitcher). The camera’s per-shot IMU orientation embedded in the `.insp` file orients the ERP to the camera’s physical frame at capture time — the same convention used during extrinsic calibration. No post-processing EIS correction is needed or applied.

**Building the SDK daemon:**

The source lives in `src/capture/sdk/`. Copy to `~/insta360-dev/` and build:

```bash
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/main.cpp ~/insta360-dev/
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/stitch.cpp ~/insta360-dev/
cd ~/insta360-dev && bash build.sh
# Produces: build/insta360_capture  build/insta360_stitch
```

Re-run these commands after any update to `src/capture/sdk/main.cpp` or `stitch.cpp`.

**Manually reconstruct an SDK-stitch session:**

```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --camera-mode dual_fisheye --sdk-stitch \
    --interval 5.0 --lidar-window 0.3
```

**Run ICP alignment on a session:**

```bash
# icp_correct.py: leave-one-out ICP using world_lidar.ply, 2cm voxel, 2 passes
python3 /tmp/icp_correct.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} 0.02 2
# Writes trajectory_icp_refined.json for each scan
# Then regenerate merged cloud:
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
    ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true
    ```
    `Ctrl+C`

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
                               #   /dual_fisheye/image/compressed  (or any topic with "fisheye")
                               #   /rko_lio/odometry               (optional, for trajectory poses)
```

```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --camera-mode single_fisheye \
    --interval 3.0 --lidar-window 0.5

# Use --camera-mode dual_fisheye if the session was captured with dual_fisheye mode
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
python3 /tmp/icp_correct.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    0.02 2   # voxel_size=2cm, 2 passes
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
