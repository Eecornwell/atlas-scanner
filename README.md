![Automated Terrestrial LiDAR Acquisition System](assets/media/atlas_logo_no_bg.png "ATLAS")
# Automated Terrestrial LiDAR Acquisition System (ATLAS)
*A low cost, open source hardware and software system for capturing physical environments using sensor fusion and a variety of 3D capture methods including terrestrial and SLAM.*

## Motivation
- In order to fully unlock the possibilities of 3D, a method for creating 3D content from physical spaces is needed to both test new capabilities and ingest as a new digital twin asset. Unfortunately for current 3D scanning systems, there is both a steep learning curve and a steep price (most scanners range from $1k-30k depending on hardware specifications and software licensing).

- This repository aims to provide a low-cost solution (~$1100 for new sensor setup) for building, deploying, and operating a 3D scanner (built from commercial-off-the-shelf components) for scanning physical environments and outputting dense colored point clouds. This solution includes both hardware and software components, as well as a setup guide to calibrate and test the system.
  > *Note: See `Hardware Build` section below for component estimates at the time of writing this.*

## Technical Approach
![ATLAS Scanner](assets/media/atlas-splat.gif "ATLAS Scanner")
### Hardware
- LiDAR w/ IMU
    - [Livox Mid360](https://www.livoxtech.com/mid-360)
        - 360° * 59° FoV
        - Non-repeating
        - 0.1m-40m range
        - Range Precision³ (1σ)
            - ≤ 2 cm (@ 10m)
            - ≤ 3 cm (@ 0.2m)
        - Angular Precision（1σ）
            - < 0.15º
        - ICM40609 IMU
- Spherical (360 degree) Camera
    - [Insta360 X5](https://www.insta360.com/product/insta360-x5)
        - 8K max resolution (72MP .insp raw stills)
        - Dual fisheye lenses, SDK-stitched to ERP in post-processing
        - USB-C tethered control via CameraSDK
    - [Insta360 One X2](https://www.insta360.com/product/insta360-onex2) (legacy)
        - 4K max resolution at 30fps
- Compute
    - 3.4GHz Quad Core, x64 architecture
    - 16GB DDR4 RAM
    - 512 GB SSD
    - Integrated Graphics Processor
- Accessories (optional)
    - See `Hardware Build` section for more information
- Complete scanner with accessories
    - 8" wide x 8" tall x 14" deep
    - 7lb

#### OS
- Linux Ubuntu Jammy 22.04

#### TODO
- [x] Release v1.0.0 hardware build
- [x] Release software install guide
- [x] Enable terrestrial mode
- [x] Release calibration procedure
- [x] Implement scanning progress gui
- [x] Enable SLAM (continuous) mode
- [ ] Release v1.0.0 code
- [ ] Release sample dataset

- [ ] Replace direct_visual_lidar_calibration feature matcher with [OpenCV 2D matcher](https://docs.opencv.org/4.x/db/dd9/group__xfeatures2d__match.html)
- [ ] Implement remote capture capability

#### Overview
- Base System
    - [ROS2 Humble](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html) - ([Apache 2.0/BSD-3-clause](https://docs.ros.org/en/diamondback/api/licenses.html))
- Drivers
    - [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) - ([MIT](https://github.com/Livox-SDK/livox_ros_driver2?tab=License-1-ov-file#readme))
    - [Insta360 CameraSDK](https://www.insta360.com/developer/home) - (proprietary, USB device control + capture)
    - [Insta360 MediaSDK](https://www.insta360.com/developer/home) - (proprietary, .insp → ERP stitching)
- Tools
    - Calibration
        - [koide3/direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration) - ([MIT](https://github.com/koide3/direct_visual_lidar_calibration/blob/main/README.md))
    - Global Mapping
        - [PRBonn/rko_lio](https://github.com/PRBonn/rko_lio) - ([MIT](https://github.com/PRBonn/rko_lio?tab=MIT-1-ov-file#readme))
    - Point Cloud Alignment/ICP
        - [Open3D](https://github.com/isl-org/Open3D) - ([MIT](https://github.com/isl-org/Open3D?tab=License-1-ov-file#readme))

#### ROS2 Details
- Published Topics
    - Lidar
        - /livox/lidar
            - ~10Hz
        - /livox/imu
            - ~200Hz
    - Camera
        - /camera/shutter_time
            - per-shot (published when TakePhoto() returns in SDK capture daemon)
        - /imu/data_raw
            - ~60Hz (camera IMU, raw — NOT available in SDK stitch mode)
        - /imu/data
            - ~60Hz (camera IMU, Madgwick-filtered — NOT available in SDK stitch mode)
    - Mapping
        - /rko_lio/odometry
            - ~10Hz
        - /rko_lio/odometry_buffered
            - ~10Hz
        - /tf
            - ~60Hz
- Transforms
    - odom → base_link → livox_frame
    - camera_to_lidar
        - Handled in post-processing color projection, uses extrinsic calibration obtained during calibration step
    - dual_fisheye_to_equirectangular
        - Handled internally by the camera driver at launch, not a TF transform

#### Software Feature List
![Software Screenshot](assets/media/atlas-software-screenshot.png)
| **Feature**                         | Supported | Notes                                                                                                            |
| ------------------------------------| --------- | ---------------------------------------------------------------------------------------------------------------- |
| Intrinsic calibration               | No        | Currently modeling as spherical camera, even single fisheye is projected to spherical model to maximize coverage |
| Extrinsic calibration               | Yes       | Camera to lidar, dual fish-eye lens to ERP                                                                       |
| Image acquisition                   | Yes       | Masked panos saved as 5760×2880 (`dual_fisheye`) or full ERP with rear hemisphere blanked + masked (`single_fisheye`)        |
| Point cloud acquisition             | Yes       | Raw lidar with intensity saved with pose as .ply or optionally .e57                                              |
| Colorize point cloud                | Yes       | Projects the image onto lidar using calibration                                                                  |
| Blend panoramic image seams         | Yes       | Blend the cubemap face seams after calibration using simple weighting                                            |
| Export Colmap model files           | Yes       | Retriangulate poses, merges color pointcloud with colmap reconstructed point cloud                               |
| Record bag files                    | Yes       | Records lidar point cloud, trajectory, and images for 3 seconds per scan (mostly used for calibration)           |
| Record trajectory                   | Yes       | Trajectory (poses) are stored locally and updated if using ICP refinement                                        |
| Merge scans w/ trajectory or ICP    | Yes       | Refine poses by performing pose graph based ICP, initialized from the trajectory                                 |
| Terrestrial mode (interval scans)   | Yes       | Currently triggered with button, mostly used for calibration and debug system/sensors                            |
| Scan progress gui                   | Yes       | Local map is updated in RVIZ and shown during scanning                                                           |
| Slam mode (continuous scanning)     | Yes       | Automate terrestrial mode by automatically building and saving point cloud, trajectory, and images               |
| SDK stitch mode (high-quality ERP)  | Yes       | Insta360 MediaSDK stitcher produces full ERPs from .insp raw files; works in both `dual_fisheye` and `single_fisheye` modes  |
| Sample gallery                      | Future    | Providing outputs to view various datasets taken with the scanner                                                |
| Remote capture capability           | Future    | Enable ability to remotely trigger a scan on the device                                                          |

### Software Installation
Please review [Software Installation documentation](docs/software-install.md)

### Hardware Build
Please review [Hardware Build documentation](docs/atlas-hw-build-v1.pdf)

### Calibration
Please review [Calibration documentation](docs/calibration.md)

### Testing and Capturing
Please review [Running the Software documentation](docs/software-run.md)

#### Sample Output
![Point Cloud Result](assets/media/room-pointcloud.png "Result")

#### Important Notes
- Two capture modes are supported: stationary (manual trigger per scan) and continuous (full session recorded as a single rosbag, then reconstructed into individual scans at shutdown)

- The system uses the Insta360 CameraSDK exclusively — no ROS camera driver. The `insta360_capture` daemon owns the USB device for the entire session. In continuous mode it calls `TakePhoto()` on a host-controlled timer at `CONTINUOUS_INTERVAL` (default 3s). Each `.insp` raw file is downloaded concurrently via HTTP and a shutter event is published for bag recording. In `single_fisheye` mode the same `.insp` is stitched to a full 360° ERP with the rear hemisphere blank; a per-hardware mask (`lidar_mask_single_x5.png`) is applied before coloring to exclude the blank region and scanner body

- Acquisition is camera-triggered — the system waits for `TakePhoto()` to return before recording the shutter event, making the SDK response time the primary timing factor

- In continuous mode, IMU topics are recorded to a dedicated separate bag (`rosbag_*_imu`) on its own CPU core to prevent IMU starvation from LiDAR write bursts

- Color projection onto the point cloud is done entirely in post-processing, not during live capture

- If the SDK capture daemon crashes during a session, the continuous mode loop detects it and stops the session gracefully

- RKO-LIO odometry requires an IMU — the system uses the Livox Mid360's built-in IMU (~200Hz); camera IMU is not available in SDK stitch mode

- Calibration is reloaded from calib.json at startup by default; set USE_EXISTING_CALIBRATION=true to skip this step

- A sync benchmark report (sync_benchmark.json) is automatically generated at the end of each session to validate sensor timing alignment

#### Sensor Timing & Synchronization

- All timestamps are hardware-originated — the Livox driver stamps LiDAR and IMU messages from the sensor's own clock, and RKO-LIO preserves those stamps through to the odometry output. The buffered odometry bridge forwards live messages with their original LIO header timestamp intact; only stale fallback messages (published when odometry has dropped out) use wall clock time

- **Host→Livox clock offset:** The MID360 hardware clock runs in a separate domain from the host system clock, producing a fixed offset of ~65ms between LiDAR `header.stamp` values and host-clock shutter timestamps (`t_before`). This offset is stable to ±0.3ms session-to-session and is measured automatically from bag metadata during reconstruction. It is applied to convert shutter times to the Livox clock domain before LiDAR window selection.

- **MID360 clock sync (RMC):** At startup, `livox_time_sync` (built from `livox_time_sync.cpp` using Livox SDK2) connects to the MID360 before the ROS driver starts and calls `SetLivoxLidarRmcSyncTime()` with a synthesised GPRMC sentence encoding the current UTC time. This sets the device's internal wall-clock reference. The point cloud `header.stamp` domain is unaffected — the ~65ms offset correction in `reconstruct_from_bag.py` remains necessary and is always applied.

- Scan centers are defined by camera frame timestamps. During reconstruction (post processing), each camera stamp is used to interpolate a pose from the bracketing pair of odometry samples (SLERP for rotation, linear for translation). The interpolation bracket half-width — half the gap between the two surrounding odom samples — bounds the worst-case pose error at each scan centre. At ~18 Hz odometry this is typically ≤28ms

- Camera frames whose timestamps fall outside the odometry window (before the first or after the last odom sample) cannot be interpolated and are filtered out during reconstruction to avoid clamped extrapolation errors

- Per-frame motion compensation is applied to each LiDAR scan: individual LiDAR returns within a scan are de-skewed using interpolated poses across the scan duration before the scan is projected into world frame

- The Livox Mid360 IMU publishes at ~200 Hz using best-effort QoS (`SensorDataQoS`), matching RKO-LIO's subscriber. A QoS mismatch (reliable publisher vs best-effort subscriber) will cause IMU starvation and degrade odometry from ~18 Hz to ~6 Hz — the benchmark will report POOR in this case

- The sync benchmark measures five quantities: raw inter-message gaps per topic, interpolation residual (odom bracket half-width at each scan centre), odom coverage fraction over the session window, per-topic arrival jitter (std of inter-message gaps), and IMU soft-sync confidence and residual. Overall quality is rated GOOD / OK / POOR based on mean bracket half-width against a 33 ms threshold

- **Typical timing results (X5 + MID360, continuous mode):**
  - Host→Livox clock offset: ~65ms (stable, auto-measured each session)
  - Shutter → nearest LiDAR frame: mean ~8–12ms, max ~23ms
  - Positional error at 0.5 m/s: ~0.4–0.6 cm

#### Post-Processing Pipeline

- `imu_sync.py` — reads both IMU gyroscope streams from the dedicated IMU bag (falling back to the main bag), reconstructs monotonic camera IMU timestamps, auto-selects the best-correlated axis pair, and estimates any residual clock offset via FFT cross-correlation. If confidence ≥ 0.7 the offset is written to `sync_offset.json`; otherwise `delta_t_s` is written as `0.0` so the file is a safe no-op. Run automatically before reconstruction in continuous mode

- `reconstruct_from_bag.py` — replays the session bag, loads `sync_offset.json` (continuous mode only) and applies `delta_t_s` to camera timestamps only when confidence ≥ 0.7 — below that threshold the correction is skipped and a warning is printed. In SDK stitch mode (`--sdk-stitch`), scan centres come from `.insp` capture_time sidecars (preferred), `/camera/shutter_time` bag messages, or .insp filename RTC timestamps (fallback). The host→Livox clock offset (~65ms, stable ±0.3ms) is measured from bag metadata by computing `median(bag_receipt_time - lidar_header_stamp)` over 200 LiDAR frames and applied to convert host-clock shutter times to Livox hardware time for correct LiDAR window alignment. For `single_fisheye` mode, `.insp` files are promoted from `.sdk_shot_N/` directories, stitched to full 360° ERPs (with blank rear hemisphere), masked using the per-hardware mask (`lidar_mask_single_x5.png`), and then colorized. For `dual_fisheye`, the same pipeline applies with the dual mask.

- `exact_match_fusion.py` — projects `sensor_lidar.ply` into the equirectangular image using the extrinsic calibration (T_camera_lidar), samples RGB from the masked panorama (RGBA with alpha channel), and writes `sensor_colored_exact.ply`. Points projecting onto masked-out regions (alpha < 128) are discarded — this excludes the scanner body, nadir, and (in single_fisheye mode) the blank rear hemisphere. No EIS correction is applied — each scan uses its own ERP which is already in the correct camera-frame orientation (DYNAMICSTITCH, per-shot IMU).

- `align_scan_session_posegraph.py` — loads `world_lidar.ply` for each scan at 5 cm voxel resolution, transforms all clouds to relative-to-first frame using trajectory poses, then runs leave-one-out ICP (each scan vs merged reference of all others) over 1 pass by default to find small residual drift corrections. Corrected absolute poses are written to `trajectory_icp_refined.json`. Typical improvement: ~20–25% reduction in inter-scan median NN distance.

- `merge_with_trajectory.py` — applies the full rigid transform from `trajectory_icp_refined.json` (falling back to `trajectory.json`) to each sensor-frame colored PLY and concatenates them into a single world-frame merged cloud. ICP-refined poses are already in relative-to-first frame and applied directly; raw trajectory poses are in absolute odom frame and have `T_first_inv` applied first.

#### Process & Worker Management

- Each sensor node (camera driver, LiDAR driver, RKO-LIO, rosbag recorder, trajectory recorder, etc.) runs as a dedicated OS process

- CPU affinity is managed via `taskset` in addition to nice/renice — the Livox driver is pinned to cores 0–1 (nice −5) to prevent UDP packet drops, the main bag recorder runs on cores 2–3 (nice +10), and the dedicated IMU bag recorder runs on core 3 (nice +5). RKO-LIO gets highest scheduling priority (nice −15)

- All PIDs are tracked and bulk-terminated on exit via a trap cleanup handler bound to SIGINT, SIGTERM, and EXIT

- The trajectory recorder is given a graceful shutdown window to flush any pending scan saves before being killed

- There is no thread pool or worker queue — OS-level process isolation and nice values are the sole concurrency control mechanism
