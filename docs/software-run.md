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
EXPORT_COLMAP=true             # export session to COLMAP format
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
    --interval 3.0 --lidar-window 0.5
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

### Re-run COLMAP export on an existing session
```bash
SESSION=~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
rm -rf $SESSION/colmap
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/export_to_colmap.py $SESSION
python3 ~/atlas_ws/src/atlas-scanner/src/post_processing/erp_to_perspective_colmap.py $SESSION
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
