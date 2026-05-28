# Running the Software

## Configuration

Edit the settings at the top of `~/atlas_ws/src/atlas-scanner/src/atlas_fusion_capture.sh` before running:

```bash
CAMERA_MODE="single_fisheye"   # dual_fisheye | single_fisheye
CAPTURE_MODE="continuous"      # stationary | continuous
CONTINUOUS_INTERVAL=3          # seconds between captures (continuous mode only)

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

**How it works:**

- The ROS camera driver is not started. Instead, `insta360_capture` (a small C++ daemon built from `~/insta360-dev/`) takes ownership of the USB device and runs a timelapse session at `CONTINUOUS_INTERVAL` seconds.
- During the session, `insta360_capture` polls the camera for new `.insp` raw files and downloads them to the session directory in the background. Shutter events are written as `.shutter_event` files only when a confirmed `.insp` file is found — not synthetically from elapsed time.
- After the session ends, `insta360_capture` continues polling until no new files appear in two consecutive passes, ensuring all buffered photos are downloaded before post-processing begins.
- Each `.insp` file is stitched to a full 5760×2880 ERP JPEG by `insta360_stitch` (also from `~/insta360-dev/`) during reconstruction.
- The `shutter_event_publisher.py` node watches for `.shutter_event` files and publishes them on `/camera/shutter_time`, which is recorded into the bag. `reconstruct_from_bag.py` uses these timestamps as scan centres.

**ERP orientation and EIS correction (continuous mode):**

The Insta360 SDK stitcher locks all ERPs in a timelapse session to the camera's orientation at the first shutter event (timelapse start). This means every ERP shares the same reference frame as the first shot — it is not body-fixed per shot and not gravity-stabilised.

To correct for this, `exact_match_fusion.py` applies an EIS rotation when the `.sdk_stitch_continuous` sentinel file is present:

1. The odometry pose at timelapse start is saved to `.sdk_stitch_ref_pose.json` by `reconstruct_from_bag.py`.
2. For each scan N (except scan_001), `T_eis = inv(T_ref) @ T_N` is computed and applied to the LiDAR points before UV projection.
3. All scans project into scan_001's masked ERP (`equirect_dual_fisheye_masked.png`), which has the correct orientation reference and masks the scanner body (nadir region).

**Building the SDK daemon:**

```bash
cd ~/insta360-dev && bash build.sh
# Produces: build/insta360_capture  build/insta360_stitch
```

**Manually reconstruct an SDK-stitch session:**

```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/reconstruct_from_bag.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} \
    --camera-mode dual_fisheye --sdk-stitch \
    --interval 3.0 --lidar-window 0.3
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

### Merge scans with ICP alignment (pose graph refinement)
```bash
cd ~/atlas_ws && source install/setup.bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/align_scan_session_posegraph.py \
    ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
```

### View a point cloud in the web viewer
```bash
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/web_3d_viewer.py <PLY_FILE>
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
