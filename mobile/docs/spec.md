# Atlas Mobile - iPhone Capture App Specification

## Overview

Atlas Mobile is an iOS application for multi-sensor 3D scene capture using iPhone LiDAR, iPhone pinhole camera, and Insta360 X5 (via WiFi). It produces a COLMAP-compatible dataset identical in structure to the desktop Atlas Scanner output, enabling direct ingestion into the splat-toolbox pipeline for 3DGS training.

## Goals

1. Capture synchronized multi-sensor data (iPhone LiDAR + RGB + Insta360 X5 panoramas)
2. Output a COLMAP model directory compatible with atlas-scanner's format
3. Keep the app as a thin capture client — heavy ML processing (PromptDA, StableNormal) runs offline
4. Maintain metric accuracy via LiDAR-anchored depth

## Hardware

| Sensor | Role | Specs |
|--------|------|-------|
| iPhone 14 Pro LiDAR | Sparse depth + metric scale | 256x192, ~5m range, dToF |
| iPhone 14 Pro Wide Camera | High-res RGB (pinhole) | 12MP, known intrinsics via AVCaptureDevice |
| Insta360 camera(s) | 360-degree panoramic imagery | Multiple models supported (see below) |
| iPhone IMU | Supplementary motion data | Accelerometer + Gyroscope |

### Multi-Camera Support

The system supports multiple rigidly-mounted Insta360 cameras simultaneously, following atlas-scanner's `multi_camera.yaml` pattern. Each camera has its own calibrated extrinsic and can be a different model.

| Model | Resolution | Connection | Notes |
|-------|-----------|-----------|-------|
| Insta360 X5 | 8K/72MP ERP | WiFi | Primary 360 camera |
| Insta360 X4 | 8K ERP | WiFi | Supported alternative |
| Insta360 X3 | 5.7K ERP | WiFi | Lower-res option |
| Insta360 ONE RS | 5.7K ERP | WiFi | Supported alternative |

Multiple cameras connect via the Insta360 SDK's multi-device WiFi management (or sequential connection if SDK doesn't support simultaneous WiFi). Each camera is independently calibrated with its own `T_camera_iphone` extrinsic.

**Configuration (`multi_camera.yaml`):**

```yaml
cameras:
  - id: insta360_primary
    model: X5
    serial: "XXXXX"
    extrinsic:
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
      x: 0.0
      y: 0.05
      z: 0.03
    mask: masks/insta360_primary.png
    face_count: 8
    tile_fov: 90

  - id: insta360_secondary
    model: X4
    serial: "YYYYY"
    extrinsic:
      roll: 0.0
      pitch: 180.0
      yaw: 0.0
      x: 0.0
      y: -0.05
      z: 0.03
    mask: masks/insta360_secondary.png
    face_count: 8
    tile_fov: 90

iphone:
  mask: masks/iphone_wide.png
```

Connection: Insta360 cameras connect to iPhone via WiFi (Direct WiFi / AP mode).

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                      Atlas Mobile (Swift)                          │
├───────────────────────────┬──────────────────────────────────────┤
│      ARKitCapture         │     Insta360CaptureManager           │
│  ┌─────────────────────┐  │  ┌──────────────────────────────┐    │
│  │ ARSession            │  │  │ Multi-device management       │    │
│  │  - worldTracking     │  │  │  - WiFi discovery/connect    │    │
│  │  - sceneDepth        │  │  │  - Per-camera state machine  │    │
│  │  - smoothedSceneDepth│  │  │  - Sequential or parallel    │    │
│  │  - confidenceMap     │  │  │    capture trigger           │    │
│  └─────────────────────┘  │  ├──────────────────────────────┤    │
│                           │  │ CameraInstance[] (N cameras)   │    │
│                           │  │  - id, model, serial          │    │
│                           │  │  - triggerCapture()           │    │
│                           │  │  - onCaptureComplete          │    │
│                           │  │  - downloadMedia()            │    │
│                           │  │  - clockOffset                │    │
│                           │  └──────────────────────────────┘    │
├───────────────────────────┴──────────────────────────────────────┤
│                  CaptureSessionManager                             │
│  - Temporal anchor pattern (primary Insta360 shutter = reference) │
│  - ARKit pose recording at shutter moment                         │
│  - Per-camera clock offset estimation                             │
│  - Scan metadata accumulation                                     │
├──────────────────────────────────────────────────────────────────┤
│                     MaskManager                                    │
│  - Per-camera alpha masks (occluded regions, mount, self-view)    │
│  - iPhone mask (mount hardware in frame)                          │
│  - Applied during ERP → tile slicing and COLMAP export            │
├──────────────────────────────────────────────────────────────────┤
│                     DataRecorder                                   │
│  - Per-scan: RGB frame, depth map, confidence map, pose           │
│  - Per-scan: Insta360 panorama per camera (downloaded post-cap)   │
│  - Session: trajectory, calibration, timing, multi_camera.yaml    │
├──────────────────────────────────────────────────────────────────┤
│                     ExportManager                                  │
│  - COLMAP directory structure (with rig model)                    │
│  - cameras.bin / images.bin / points3D.bin / rigs.bin             │
│  - Masked tile images + mask PNGs                                 │
│  - Depth maps as uint16 PNG (millimeters)                         │
│  - Raw assets for offline ML enhancement                          │
└──────────────────────────────────────────────────────────────────┘
```

## Capture Flow

### Per-Scan Capture Sequence

1. User triggers capture (tap or automatic interval)
2. `CaptureSessionManager` sends capture command to all connected Insta360 cameras
   - Primary camera triggered first (temporal anchor)
   - Secondary cameras triggered immediately after (or simultaneously if SDK supports)
3. Simultaneously, record current `ARFrame`:
   - `ARFrame.timestamp` (monotonic clock)
   - `ARFrame.camera.transform` (4x4 world-to-camera)
   - `ARFrame.camera.intrinsics` (3x3 pinhole matrix)
   - `ARFrame.capturedImage` (CVPixelBuffer, YCbCr → RGB)
   - `ARFrame.sceneDepth.depthMap` (Float32, meters)
   - `ARFrame.sceneDepth.confidenceMap` (0/1/2 per pixel)
   - `ARFrame.smoothedSceneDepth.depthMap` (temporally filtered)
4. On each Insta360 capture completion callback:
   - Record per-camera Insta360 timestamp (from EXIF/sidecar)
   - Compute per-camera clock offset sample: `offset = arkit_time - insta360_rtc_time`
   - Queue media download for that camera
5. Download Insta360 images from all cameras (parallel downloads)
6. Apply masks to verify no unexpected occlusion (optional QA step)
7. Store all data to scan directory (organized by camera ID)

### Session Lifecycle

```
startSession()
  → Configure ARSession (worldTracking + sceneDepth)
  → Connect Insta360 via WiFi
  → Calibrate clock offset (N initial samples)
  → Begin continuous ARKit tracking

captureScans() (repeated)
  → Per-scan sequence above
  → Accumulate trajectory (poses at regular intervals)

endSession()
  → Stop ARSession
  → Disconnect Insta360
  → Run on-device export (COLMAP format)
  → Package for transfer to workstation
```

## Synchronization Strategy

Follows atlas-scanner's **temporal anchor pattern**:

- Insta360 shutter event is the temporal reference for each scan
- ARKit pose at closest timestamp is associated with that scan
- Clock offset estimated per-session (median of N samples)
- ARKit's internal sensors (LiDAR + camera) are hardware-synchronized (no cross-clock issue)

### Clock Domains

| Source | Clock | Resolution |
|--------|-------|-----------|
| ARKit (LiDAR, camera, pose) | `mach_absolute_time` (monotonic) | Sub-millisecond |
| Insta360 X5 | RTC (wall clock from EXIF) | ~1 second (from filename), sub-second from SDK callback |

Sync accuracy target: <50ms camera-to-ARKit alignment (likely achievable since ARKit capture is instantaneous at trigger time; the Insta360 latency is the variable).

## Output Format

### Directory Structure

Mirrors atlas-scanner's COLMAP export:

```
{session_name}/
├── colmap/
│   ├── database.db                      # COLMAP SQLite database (optional, for BA)
│   ├── images/
│   │   ├── face_iphone/                 # iPhone pinhole RGB frames
│   │   │   ├── scan_000.jpg
│   │   │   ├── scan_001.jpg
│   │   │   └── ...
│   │   ├── face_00/                     # Primary Insta360 ERP tiles (ceiling)
│   │   ├── face_01/                     # Primary Insta360 ERP tiles (front-left)
│   │   ├── ...                          # (up to face_07)
│   │   ├── face_10/                     # Secondary Insta360 ERP tiles (ceiling)
│   │   ├── face_11/                     # Secondary Insta360 ERP tiles (front-left)
│   │   └── ...                          # (face_1X for camera 2, face_2X for camera 3, etc.)
│   ├── masks/
│   │   ├── face_iphone/                 # iPhone masks (mount hardware in frame)
│   │   │   └── mask.png                 # Alpha mask, shared across all iPhone frames
│   │   ├── face_00/                     # Per-face masks for primary Insta360
│   │   │   └── mask.png                 # Occludes mount, tripod, self-view regions
│   │   ├── face_01/
│   │   │   └── mask.png
│   │   ├── ...
│   │   ├── face_10/                     # Per-face masks for secondary Insta360
│   │   │   └── mask.png
│   │   └── ...
│   ├── init_sparse/0/
│   │   ├── cameras.bin                  # Known intrinsics (iPhone + per-camera tile FoV)
│   │   ├── images.bin                   # ARKit-derived poses (all cameras via extrinsics)
│   │   └── points3D.bin                 # Empty or LiDAR-derived sparse points
│   ├── sparse/0/
│   │   ├── cameras.bin                  # Final intrinsics
│   │   ├── images.bin                   # Final poses (post-BA if run offline)
│   │   ├── points3D.bin                 # Triangulated + LiDAR points
│   │   ├── rigs.bin                     # Rig definition (iPhone + N Insta360 cameras)
│   │   ├── frames.bin                   # Frame-to-rig mapping
│   │   └── merged.ply                   # Combined point cloud
│   └── depth_images/
│       └── face_iphone/
│           ├── scan_000.png             # uint16 depth in millimeters
│           └── ...
├── raw/
│   ├── iphone/
│   │   ├── scan_000/
│   │   │   ├── rgb.jpg                  # Full-res iPhone RGB
│   │   │   ├── depth.bin                # Float32 depth map (meters), 256x192
│   │   │   ├── confidence.bin           # uint8 confidence map, 256x192
│   │   │   ├── depth_smoothed.bin       # Smoothed depth variant
│   │   │   └── pose.json               # 4x4 transform + intrinsics + timestamp
│   │   └── ...
│   ├── insta360_primary/
│   │   ├── scan_000.jpg                 # ERP panorama (or .insp for raw)
│   │   └── ...
│   ├── insta360_secondary/              # Second camera (if configured)
│   │   ├── scan_000.jpg
│   │   └── ...
│   └── trajectory.json                  # Full ARKit trajectory (all recorded poses)
├── masks/
│   ├── iphone_wide.png                  # Source mask for iPhone (mount/rig in frame)
│   ├── insta360_primary.png             # Source ERP mask (nadir, mount, self-view)
│   └── insta360_secondary.png           # Source ERP mask for second camera
├── calibration/
│   ├── session_info.json                # Device model, session params
│   ├── multi_camera.yaml                # Camera definitions + extrinsics + mask refs
│   └── clock_offset_samples.json        # Per-camera raw offset measurements
└── enhanced/                            # Populated by offline ML pipeline
    ├── depth_dense/                     # PromptDA output (dense metric depth)
    │   ├── scan_000.png                 # uint16, millimeters, at RGB resolution
    │   └── ...
    └── normals/                         # StableNormal output
        ├── scan_000.png                 # RGB-encoded normal map
        └── ...
```

### Camera Models

| Source | COLMAP Model | Parameters |
|--------|-------------|-----------|
| iPhone Wide | `PINHOLE` (ID 1) | fx, fy, cx, cy from `ARCamera.intrinsics` |
| Insta360 ERP tiles | `SIMPLE_PINHOLE` (ID 0) | f, cx, cy (same as atlas-scanner face tiles) |

### Coordinate System

- ARKit uses right-handed Y-up coordinate system
- COLMAP uses right-handed with camera looking down -Z, Y down
- Transform: `R_ARKIT2COLMAP = [[1,0,0],[0,-1,0],[0,0,-1]]`
- Quaternions stored in COLMAP's wxyz convention
- Poses stored as world-to-camera (extrinsic) in images.bin

### Pose Format (pose.json per scan)

```json
{
  "timestamp": 1234567.890,
  "transform_matrix": [[...], [...], [...], [...]],
  "intrinsics": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "image_width": 4032,
  "image_height": 3024,
  "depth_width": 256,
  "depth_height": 192,
  "insta360_timestamp": "2026-08-24T10:30:45.123Z",
  "clock_offset_ms": 28.5
}
```

## Offline Enhancement Pipeline

Runs on workstation (GPU), reads from `raw/`, writes to `enhanced/`:

### Stage 1: Depth Completion (PromptDA)

- **Input:** `raw/iphone/scan_NNN/depth.bin` (sparse 256x192) + `raw/iphone/scan_NNN/rgb.jpg` (12MP)
- **Output:** `enhanced/depth_dense/scan_NNN.png` (uint16 mm, at RGB resolution)
- **Model:** PromptDA (Depth Anything V2 conditioned on sparse LiDAR)
- **Key property:** Output is metrically accurate (LiDAR-anchored)

### Stage 2: Normal Estimation (StableNormal)

- **Input:** `raw/iphone/scan_NNN/rgb.jpg`
- **Output:** `enhanced/normals/scan_NNN.png` (RGB-encoded world-space normals)
- **Model:** StableNormal (diffusion-based, two-step: coarse → refined)
- **Use:** Regularization signal during 3DGS training

### Stage 3: COLMAP Model Assembly

- Convert enhanced depth → point cloud per frame (unproject with intrinsics + pose)
- Slice Insta360 ERP images into perspective face tiles (same as atlas-scanner)
- Derive Insta360 face tile poses via rigid extrinsic `T_insta360_iphone` (from calibration)
- Write cameras.bin, images.bin, points3D.bin (with rig model: rigs.bin, frames.bin)
- Write depth_images/ from enhanced dense depth
- Optionally run COLMAP feature matching + triangulation + BA for refinement

## Calibration

### iPhone Intrinsics
- Obtained directly from `ARCamera.intrinsics` per frame (auto-calibrated by ARKit)
- Lens distortion: ARKit provides undistorted frames, so no distortion model needed

### Insta360 → iPhone Extrinsic (Rigid Mount)

The Insta360 X5 is rigidly mounted to the iPhone via a physical bracket/cage. This gives a fixed 6-DOF extrinsic transform `T_insta360_iphone` that remains constant across the session.

**Calibration procedure** (same as atlas-scanner's LiDAR↔camera calibration):

1. **Feature matching:** SuperGlue matching between iPhone RGB and Insta360 ERP-projected intensity/RGB
   - Generate LiDAR intensity images from iPhone depth (projected to Insta360 ERP frame)
   - Match against Insta360 panorama using learned feature matching
   - Port of `find_matches_superglue_erp.py` from atlas-scanner

2. **Seed estimation:** Initial extrinsic guess from physical measurements of the mount
   - Measure XYZ offset and approximate RPY between iPhone camera and Insta360 lens center
   - Port of `physical_seed.py` / `seed_calib.py` from atlas-scanner

3. **Iterative refinement:** Optimize 6-DOF extrinsic by minimizing reprojection error
   - Port of `tune_calibration.py` from atlas-scanner
   - Iteratively adjusts RPY + XYZ to maximize feature correspondence alignment

4. **Verification:** Visual overlay of colorized LiDAR points onto Insta360 panorama
   - Port of `verify_seed_overlay.py` from atlas-scanner

**Stored calibration format** (YAML, same as atlas `fusion_calibration.yaml`):

```yaml
extrinsic_insta360_iphone:
  roll: 0.0      # degrees
  pitch: 0.0
  yaw: 0.0
  x: 0.0         # meters
  y: 0.05
  z: 0.03
```

**Usage at capture time:**
- Insta360 pose for each scan = `T_insta360_iphone @ T_iphone_world` (from ARKit)
- All Insta360 face tile poses in COLMAP derived from this fixed transform
- No per-scan pose estimation needed — same as atlas-scanner's rig model

### Masks

Masks define regions to exclude from feature matching, colorization, and 3DGS training. Same concept as atlas-scanner's `masks/` directory.

**Sources of occlusion:**

| Source | Affected sensor | Description |
|--------|----------------|-------------|
| Mounting bracket/cage | iPhone wide camera | Rig hardware visible in iPhone FOV edges |
| iPhone + mount | Insta360 cameras | Phone/bracket visible in 360 panorama |
| Tripod/monopod | Insta360 cameras | Support structure in nadir region |
| Other Insta360 camera | Each Insta360 | Secondary camera visible in primary's FOV |
| Operator | Insta360 cameras | Self-view if handheld (less relevant with mount) |

**Mask format:**
- Single-channel PNG (0 = masked/excluded, 255 = valid)
- Source masks stored at session level (`masks/`) in native sensor resolution
  - iPhone: mask at 4032x3024 (or camera resolution)
  - Insta360: mask in equirectangular projection at panorama resolution
- Per-face tile masks in `colmap/masks/face_XX/` — generated by slicing the ERP mask using the same projection as the image tiles

**Mask generation:**
- Created once per rig configuration (mount geometry doesn't change between sessions)
- Can be generated semi-automatically: capture a scan with known empty background, threshold the rig/mount regions
- Fine-tuned manually in an image editor
- Stored in calibration and reused across sessions with the same mount

**Usage in pipeline:**
- COLMAP feature matching: masked pixels excluded from feature detection
- Depth map projection: masked pixels not projected
- 3DGS training: masked pixels excluded from photometric loss
- Point cloud colorization: masked pixels not sampled

### Clock Offset
- Estimated at session start (N capture round-trips, per camera)
- Monitored for drift throughout session
- Stored in `calibration/clock_offset_samples.json` (keyed by camera ID)

## Technology Stack

### iOS App (Swift)

| Component | Framework/Library |
|-----------|------------------|
| LiDAR + Tracking | ARKit (ARWorldTrackingConfiguration) |
| Camera access | ARKit (capturedImage) / AVFoundation |
| Insta360 control | Insta360 CameraSDK (CocoaPods) |
| Async coordination | Swift Concurrency (async/await, actors) |
| File I/O | Foundation (FileManager) |
| Linear algebra | simd / Accelerate |
| UI | SwiftUI |
| Transfer to workstation | Files app share / AirDrop / WiFi transfer |

### Offline Pipeline (Python)

| Component | Library |
|-----------|---------|
| PromptDA | PyTorch + Depth Anything V2 |
| StableNormal | PyTorch + diffusion pipeline |
| COLMAP I/O | pycolmap / custom binary writer |
| ERP → tiles | OpenCV (same as atlas-scanner) |
| Point cloud ops | Open3D / NumPy |
| Orchestration | splat-toolbox pipeline stage |

## Development Phases

### Phase 1: ARKit Capture Prototype
- ARSession with scene depth enabled
- Save RGB + depth + pose per frame to disk
- Simple SwiftUI interface (start/stop/trigger)
- Export raw data to Files app
- **Deliverable:** Verified capture with correct intrinsics and poses

### Phase 2: Insta360 Integration
- WiFi connection management
- Programmatic capture triggering
- Media download to app sandbox
- Clock offset estimation
- **Deliverable:** Synchronized iPhone + Insta360 captures

### Phase 3: On-Device COLMAP Export
- ERP → perspective tile slicing (port from atlas-scanner Python)
- Binary COLMAP model writing (cameras.bin, images.bin, points3D.bin)
- Depth map export (uint16 PNG)
- **Deliverable:** Dataset that COLMAP/splat-toolbox can read directly

### Phase 4: Offline Enhancement Integration
- PromptDA integration in Python pipeline
- StableNormal integration in Python pipeline
- Enhanced COLMAP model assembly with dense depth
- Validation against splat-toolbox ingestion
- **Deliverable:** End-to-end capture → 3DGS training

### Phase 5: Polish
- Capture guidance UI (coverage visualization, scan quality indicators)
- Session management (resume, review, delete)
- Robust error handling (WiFi drops, thermal throttling)
- Performance optimization (battery, thermal)

## Open Questions

1. **Insta360 X5 iOS WiFi SDK support** — Verify capture trigger + media download works reliably. Test latency of capture command → completion callback.
2. **Thermal budget** — ARKit + WiFi + writes may throttle. Need to characterize sustainable capture rate.
3. **Mount design** — Physical bracket/cage to rigidly attach Insta360 X5 to iPhone 14 Pro. Needs to avoid occluding iPhone LiDAR/cameras and Insta360 lenses. 3D-printed or off-the-shelf cage?
4. **ERP tile slicing on-device** — Feasible in Swift (Metal/vImage) but adds complexity. Could defer to offline pipeline.
5. **Dense depth on-device** — CoreML port of PromptDA for real-time preview? Or purely offline?
6. **Transfer mechanism** — For large sessions (hundreds of scans), what's the fastest way to get data to workstation? USB cable transfer via Finder? WiFi? External SSD via USB-C adapter (not available on Lightning)?
7. **Calibration tooling** — Port atlas-scanner's SuperGlue calibration pipeline to work with iPhone LiDAR (sparse depth) instead of Livox (dense). May need PromptDA-enhanced depth for reliable feature matching on the LiDAR intensity projection.

## References

- [ARKit Documentation — Scene Depth](https://developer.apple.com/documentation/arkit/arframe/3566299-scenedepth)
- [Insta360 Camera SDK](https://www.insta360.com/developer)
- [PromptDA — Prompt Depth Anything](https://github.com/DepthAnything/PromptDA)
- [StableNormal](https://github.com/Stable-X/StableNormal)
- [Atlas Scanner](https://github.com/Eecornwell/atlas-scanner)
- [COLMAP Binary Format](https://colmap.github.io/format.html)
