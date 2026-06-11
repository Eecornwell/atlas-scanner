# Synchronization Benchmark

Measures timing alignment between LiDAR, camera, odometry, and IMU using a recorded rosbag.
No ROS runtime required — reads the bag's SQLite database directly.

## Usage

```bash
cd ~/atlas_ws
source install/setup.bash
python3 src/atlas-scanner/src/post_processing/sync_benchmark.py <session_dir>

# Optional: set walking speed for positional error estimate, save JSON report
python3 src/atlas-scanner/src/post_processing/sync_benchmark.py <session_dir> \
    --walk-speed 0.5 --out report.json
```

## What it measures

**Raw stream gaps** — for every camera frame, finds the nearest message from each other topic and measures the time gap. Camera frames are the anchor because they drive scan capture.

**Per-topic jitter** — measures consistency of message arrival gaps within each topic. High jitter means the driver is delivering data in bursts rather than steadily.

## Example output

```
=== Sync Benchmark: sync_fusion_20260228_153106 ===
  LiDAR        :  451 msgs  span=45.0s  rate=10.0Hz
  Camera       : 1312 msgs  span=43.8s  rate=30.0Hz
  Odom (odometry):  287 msgs  span=44.8s  rate=6.4Hz
  IMU (Livox)  :  711 msgs  span=45.1s  rate=15.8Hz
  IMU (Camera) :  NOT FOUND
  IMU (Filtered):  NOT FOUND

--- Raw stream gaps (nearest-neighbour, anchor = camera) ---
  Camera↔LiDAR                    mean= 28.1ms  std=22.0  p95= 68.3  max=132.7  pos_err≈1.4cm  [GOOD]
  Camera↔Odom                     mean= 46.4ms  std=35.4  p95=112.6  max=193.3  pos_err≈2.3cm  [OK]
  Camera↔IMU(Livox)               mean= 64.3ms  std=101.4  p95=206.1  max=1025.0  pos_err≈3.2cm  [OK]
  LiDAR↔Odom                      mean= 42.3ms  std=39.5  p95=104.5  max=202.7  pos_err≈2.1cm  [OK]

--- Interpolation residual (pose accuracy at scan centres) ---
  Each camera frame is a scan centre; interp_pose() brackets it
  between two odom samples.  Half-bracket width bounds pose error.
  Odom bracket half-width          mean= 51.2ms  std=18.3  p95= 82.1  max=213.4  pos_err≈2.6cm  [OK]

--- Odometry coverage ---
  Coverage: 97% of LiDAR window

--- Per-topic arrival jitter (scheduling noise) ---
  LiDAR     mean_gap=99.9ms  std=53.1ms  max_gap=511.8ms
  Camera    mean_gap=33.4ms  std=32.4ms  max_gap=384.1ms
  Odom      mean_gap=156.7ms  std=67.4ms  max_gap=444.6ms
  IMU(Livox)  mean_gap=63.5ms  std=126.3ms  max_gap=2127.0ms

--- IMU soft-sync (target ≤ 10 ms) ---
  Applied offset:  +0.00 ms  (confidence=0.978)
  Residual after correction: 1.23 ms  [✓ PASS — target ≤ 10 ms]

  Thresholds: GOOD < 33ms | OK < 100ms | POOR >= 100ms
  Walk speed: 0.5 m/s  (1ms ≈ 0.05cm positional error)

  Overall: OK
```

## How to read the output

| Field | Meaning |
|---|---|
| `mean` | Typical sync error. Drives the `pos_err` estimate |
| `std` | Variability. High std = occasional bad frames, not a constant offset |
| `p95` | 95% of frames are better than this. More representative than max |
| `max` | Worst single event (e.g. a USB stall or LIO estimator pause) |
| `pos_err` | Mean timing error converted to cm of color misalignment at your walking speed |

## Grading thresholds

| Grade | Threshold | Meaning |
|---|---|---|
| GOOD | < 33ms | Sub-frame — well within one LiDAR frame period (100ms) |
| OK | < 100ms | Within one LiDAR frame — acceptable for walking-speed capture |
| POOR | ≥ 100ms | Regularly grabbing the wrong LiDAR frame for a given camera image |

The overall grade is the worst grade across all sensor pairs.

## Which pairs matter most

- **Camera↔LiDAR** — most important. Directly determines color projection accuracy on the point cloud.
- **Camera↔Odom / LiDAR↔Odom** — affects scan-to-scan merge quality, not per-pixel coloring.
- **Camera↔IMU(Livox)** — IMU jitter causes RKO-LIO to stall, which propagates into odom spikes. The Livox IMU topic is `/livox/imu`.
- **Camera↔IMU(Camera)** — only present in continuous mode when the camera IMU bag is recorded (`/imu/data_raw`). Not available in SDK stitch mode.

## Notes on the current system

- Camera↔LiDAR is consistently ~28ms GOOD across sessions — this is a stable characteristic.
- Odom/IMU spikes (max 200–600ms) are caused by the LIO estimator competing for CPU with the bag recorder. Reducing system load during recording (close other processes, disable RViz) will improve these.
- The IMU `max_gap` spike (2127ms) is a single USB burst stall on `/livox/imu`. The mean (63ms) is fine so it does not meaningfully affect coloring quality.
