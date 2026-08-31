## Atlas Mobile (iOS Capture App)

*An iOS companion app for capturing 3D scenes using iPhone LiDAR, iPhone camera, and rigidly-mounted Insta360 cameras. Outputs COLMAP-compatible datasets in the same format as the desktop Atlas scanner for ingestion into downstream 3DGS pipelines.*

> **Branch:** This app lives in the `mobile/` subdirectory on the `mobile` branch. The desktop Atlas scanner remains on `main`.

### Motivation

- The desktop Atlas scanner requires a full compute unit, Livox Mid-360, and tethered Insta360 — ideal for high-quality terrestrial scanning but not portable for quick captures
- iPhone 14 Pro and later models include a LiDAR sensor (256×192, ~5m range) and high-quality pinhole camera (12MP) — sufficient for lightweight scene capture when combined with AI-enhanced depth completion
- By rigidly mounting an Insta360 to the iPhone and reusing Atlas's calibration + COLMAP export pipeline, we get a portable capture rig that produces datasets compatible with the existing splat-toolbox pipeline
- AI depth enhancement (PromptDA + StableNormal) bridges the gap between iPhone LiDAR's sparse output and the Livox's dense point clouds

### Hardware

- iPhone 14 Pro (or later Pro model with LiDAR)
    - dToF LiDAR: 256×192, ~5m range
    - Wide camera: 12MP pinhole, intrinsics from `ARCamera.intrinsics`
    - IMU: accelerometer + gyroscope
- Insta360 camera(s), rigidly mounted via bracket/cage
    - Primary: [Insta360 X5](https://www.insta360.com/product/insta360-x5) — 8K/72MP ERP
    - Also supports: X4, X3, ONE RS (configured in `multi_camera.yaml`)
    - Connected via WiFi (Direct WiFi / AP mode)
- Physical mount/cage rigidly attaching Insta360 to iPhone
    - Must not occlude iPhone LiDAR sensor, iPhone wide camera, or Insta360 lenses
    - Fixed 6-DOF extrinsic calibrated using the same procedure as desktop Atlas

### Architecture

```
┌──────────────────────────────────────────────────────────┐
│                   Atlas Mobile (Swift/iOS)                 │
├─────────────────────────┬────────────────────────────────┤
│     ARKitCapture        │   Insta360CaptureManager       │
│  - ARSession            │   - Multi-device WiFi mgmt     │
│  - sceneDepth (LiDAR)  │   - Per-camera state machine   │
│  - smoothedSceneDepth   │   - CameraInstance[] (N cams)  │
│  - confidenceMap        │   - Per-camera clock offset    │
│  - 6DoF pose tracking   │   - Parallel capture trigger   │
├─────────────────────────┴────────────────────────────────┤
│                CaptureSessionManager                      │
│  - Temporal anchor pattern (Insta360 shutter = ref)      │
│  - ARKit pose at shutter moment                          │
│  - MaskManager (per-camera occlusion masks)              │
├──────────────────────────────────────────────────────────┤
│     DataRecorder              │     COLMAPExporter        │
│  - raw/ (depth, RGB, pose)   │  - cameras.bin/images.bin │
│  - Per-camera ERP download   │  - ERP → face tiles       │
│  - trajectory.json           │  - Rig model (rigs.bin)   │
│  - masks/                    │  - Depth maps (uint16 mm) │
└──────────────────────────────┴───────────────────────────┘
```

### Output Format

The app produces a COLMAP-compatible dataset matching the desktop Atlas scanner output:

| Output | Format | Description |
|--------|--------|-------------|
| iPhone RGB frames | JPEG | `colmap/images/face_iphone/scan_NNN.jpg` |
| Insta360 face tiles | JPEG | `colmap/images/face_XX/scan_NNN.jpg` (8 faces per camera) |
| Masks | PNG (alpha) | `colmap/masks/face_XX/mask.png` — mount, nadir, cross-camera occlusion |
| COLMAP model | Binary | `colmap/sparse/0/{cameras,images,points3D,rigs,frames}.bin` |
| Depth maps | uint16 PNG (mm) | `colmap/depth_images/face_iphone/scan_NNN.png` |
| Raw LiDAR depth | Float32 binary | `raw/iphone/scan_NNN/depth.bin` (256×192) |
| Raw confidence | uint8 binary | `raw/iphone/scan_NNN/confidence.bin` |
| Poses | JSON | `raw/iphone/scan_NNN/pose.json` (4×4 transform + intrinsics) |
| Trajectory | JSON | `raw/trajectory.json` (all ARKit poses) |
| Enhanced depth | uint16 PNG (mm) | `enhanced/depth_dense/scan_NNN.png` (PromptDA, offline) |
| Normal maps | PNG (RGB-encoded) | `enhanced/normals/scan_NNN.png` (StableNormal, offline) |

Camera models: `PINHOLE` (fx, fy, cx, cy) for iPhone, `SIMPLE_PINHOLE` (f, cx, cy) for Insta360 face tiles — same as desktop Atlas.

### Calibration

Uses the same extrinsic calibration procedure as the desktop Atlas scanner:

1. **Physical seed** — measure XYZ offset and approximate RPY from mount geometry
2. **Feature matching** — SuperGlue on LiDAR intensity projected to ERP vs Insta360 panorama (port of `find_matches_superglue_erp.py`)
3. **Iterative refinement** — optimize 6-DOF to minimize reprojection error (port of `tune_calibration.py`)
4. **Verification** — visual overlay of colorized LiDAR on panorama (port of `verify_seed_overlay.py`)

Calibration stored in `config/multi_camera.yaml` (same format as desktop `fusion_calibration.yaml`).

### Offline Enhancement Pipeline

Heavy ML processing runs on a GPU workstation after capture:

- **PromptDA** — uses sparse iPhone LiDAR as a prompt to guide Depth Anything V2, producing dense metrically-accurate depth at RGB resolution
- **StableNormal** — diffusion-based surface normal estimation from RGB, captures fine geometry even with reflections/transparency
- **COLMAP assembly** — combines enhanced depth, Insta360 face tiles, and poses into the final COLMAP model

```
Capture (iPhone)  →  Transfer  →  PromptDA + StableNormal (GPU)  →  COLMAP model  →  splat-toolbox
```

### Masks

Per-camera masks exclude occluded regions from feature matching, depth projection, and 3DGS training:

| Occlusion source | Affected sensor | Mask location |
|------------------|----------------|---------------|
| Mount/cage hardware | iPhone wide camera | `masks/iphone_wide.png` |
| iPhone + mount | Insta360 cameras | `masks/insta360_primary.png` |
| Tripod/monopod | Insta360 cameras | Included in ERP mask (nadir region) |
| Secondary camera | Each Insta360 | Included in respective ERP mask |

Source masks are in sensor-native resolution (ERP for Insta360, pinhole for iPhone). During COLMAP export, ERP masks are sliced into per-face-tile masks using the same projection as the image tiles.

### TODO

#### Phase 1: ARKit Capture Prototype
- [ ] ARSession with scene depth + smoothed scene depth enabled
- [ ] Save RGB frames as JPEG with correct orientation
- [ ] Save depth maps as Float32 binary (256×192)
- [ ] Save confidence maps as uint8 binary
- [ ] Save pose.json per scan (4×4 transform, intrinsics, timestamps)
- [ ] Record continuous trajectory (all ARKit poses)
- [ ] Simple SwiftUI interface (start/stop session, trigger capture)
- [ ] Export session to Files app
- [ ] Validate: load saved data in Python, verify intrinsics and poses are correct

#### Phase 2: Insta360 Integration
- [ ] Integrate Insta360 CameraSDK (CocoaPods or manual framework)
- [ ] WiFi camera discovery and connection
- [ ] Programmatic capture trigger via SDK
- [ ] Capture completion callback handling
- [ ] Media download from camera to app sandbox
- [ ] Per-camera clock offset estimation (N-sample median)
- [ ] Multi-camera support (discover and manage N cameras from `multi_camera.yaml`)
- [ ] Validate: timestamps, clock offset stability, download reliability

#### Phase 3: COLMAP Export
- [ ] ERP → perspective tile slicing (port from `panorama_sfm_colmap.py`)
- [ ] ARKit → COLMAP coordinate transform (`R_ARKIT2COLMAP`)
- [ ] Derive Insta360 tile poses from rigid extrinsic + ARKit pose
- [ ] Write cameras.bin (PINHOLE for iPhone, SIMPLE_PINHOLE for tiles)
- [ ] Write images.bin (quaternion wxyz + translation, all cameras)
- [ ] Write points3D.bin (from LiDAR depth unprojection)
- [ ] Write rigs.bin + frames.bin for rig-aware BA
- [ ] Mask slicing (ERP mask → per-face-tile masks)
- [ ] Depth map export as uint16 PNG in millimeters
- [ ] Validate: open output in COLMAP GUI, ingest into splat-toolbox

#### Phase 4: Offline Enhancement
- [ ] PromptDA integration (`enhance_session.py`)
- [ ] StableNormal integration (`enhance_session.py`)
- [ ] Enhanced COLMAP model assembly with dense depth + normals
- [ ] Validate: compare 3DGS quality with vs without enhancement

#### Phase 5: Calibration Tooling
- [ ] Port `physical_seed.py` for initial extrinsic guess
- [ ] Port `find_matches_superglue_erp.py` for feature matching (adapt for sparse iPhone LiDAR)
- [ ] Port `tune_calibration.py` for iterative refinement
- [ ] Port `verify_seed_overlay.py` for visual verification
- [ ] Validate: overlay accuracy on test captures

#### Phase 6: Polish
- [ ] Capture guidance UI (coverage indicator, scan quality, pose tracking status)
- [ ] Session management (list, resume, review, delete, storage usage)
- [ ] WiFi reconnection handling (Insta360 drop recovery)
- [ ] Thermal management (throttle capture rate, warn user)
- [ ] Battery usage optimization
- [ ] Session transfer to workstation (AirDrop, USB via Finder, WiFi)

### Local Development Setup

#### iOS App (requires macOS)

1. **Prerequisites**
    - macOS 13+ with Xcode 15+
    - iPhone 14 Pro or later (LiDAR required, simulator not supported)
    - Apple Developer account (for device deployment)
    - Insta360 X5 (or supported model) for integration testing

2. **Clone and checkout**
    ```bash
    git clone https://github.com/Eecornwell/atlas-scanner.git
    cd atlas-scanner
    git checkout mobile
    cd mobile/AtlasMobile
    ```

3. **Create Xcode project**
    - Open Xcode → File → New → Project → iOS → App
    - Product Name: `AtlasMobile`
    - Interface: SwiftUI
    - Language: Swift
    - Save into `mobile/AtlasMobile/`
    - Add all `.swift` files from the skeleton to the Xcode project
    - Add `Info.plist` to the target

4. **Add Insta360 SDK**
    - Download from [Insta360 Developer Portal](https://www.insta360.com/developer)
    - Add framework to project (CocoaPods or manual embed)
    - Ensure `NSLocalNetworkUsageDescription` and `NSBonjourServices` are in Info.plist

5. **Configure signing**
    - Select your team in Xcode → Target → Signing & Capabilities
    - Enable Camera capability

6. **Build and run**
    ```bash
    # Build from command line (optional, Xcode GUI preferred)
    xcodebuild -scheme AtlasMobile -destination 'platform=iOS,name=<your-device>'
    ```
    - Or press Cmd+R in Xcode with your iPhone connected

7. **Test ARKit capture (Phase 1)**
    - Launch app on device
    - Tap "Start Session" → "Capture Scan" → "End Session"
    - Verify output in Files app → AtlasMobile → atlas_sessions/

#### Offline Pipeline (macOS/Linux/Windows)

1. **Prerequisites**
    - Python 3.10+
    - uv package manager
    - CUDA GPU (for PromptDA/StableNormal)

2. **Install dependencies**
    ```bash
    cd mobile/offline_pipeline
    uv sync
    ```

3. **Run enhancement pipeline**
    ```bash
    # Transfer session from iPhone to workstation first
    uv run python enhance_session.py --session-dir /path/to/session_2026-08-24_10-30-00
    ```

4. **Run calibration**
    ```bash
    uv run python calibrate_extrinsic.py \
        --session-dir /path/to/calibration_session \
        --camera-id insta360_primary \
        --seed-from-physical \
        --seed-y 0.05 --seed-z 0.03
    ```

5. **Slice ERP tiles (standalone)**
    ```bash
    uv run python erp_tile_slicer.py \
        --erp-image /path/to/panorama.jpg \
        --output-dir /path/to/output/ \
        --mask /path/to/masks/insta360_primary.png
    ```

### Specification

See [docs/spec.md](docs/spec.md) for the full technical specification.
