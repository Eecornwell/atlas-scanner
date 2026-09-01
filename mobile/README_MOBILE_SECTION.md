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

Extrinsic calibration runs **on-device** in the iOS app — no host computer required.

1. Start a session, point the rig at a textured scene
2. Navigate to **Calibration** tab
3. Enter physical mount measurements (forward/left/up in inches)
4. Tap **Capture Frame** 3–5 times from slightly different positions
5. Tap **Run Optimisation** — Nelder-Mead 6-DOF refinement runs on-device
6. Review the verification overlay (edge alignment + depth dots)
7. Tap **Save to Device** — writes refined extrinsic to `Documents/atlas_sessions/multi_camera.yaml`

The saved `multi_camera.yaml` is automatically loaded by the app on next launch (takes priority over the bundled default). The same file is included in every exported session so the offline pipeline uses the correct extrinsic.

Calibration stored in `Documents/atlas_sessions/multi_camera.yaml` (same format as desktop `fusion_calibration.yaml`).

### Offline Enhancement Pipeline

Three export modes are available after each session:

| Mode | Where it runs | What it produces |
|---|---|---|
| **COLMAP Only** (default) | On-device, instant | `colmap/sparse/0/` binary model — ready for splat-toolbox |
| **Full On-Device** | On-device via CoreML (ANE) | COLMAP + dense depth (PromptDA) + normals (StableNormal YOSO) |
| **Host Processing** | GPU workstation via `enhance_server.py` | COLMAP + full PromptDA ViT-L + StableNormal diffusion refinement |

The export sheet appears automatically after ending a session, and is also accessible from the Sessions list via long-press on any session.

```
Capture (iPhone)
  └─ End Session
       ├─ COLMAP Only      → colmap/ ready immediately
       ├─ Full On-Device   → CoreML PromptDA + StableNormal YOSO on ANE
       └─ Host Processing  → ZIP upload → enhance_server.py → download results
```

#### Host processing setup

```bash
# On workstation (same WiFi network as iPhone)
cd mobile/offline_pipeline
bash setup.sh          # first time only
python3 enhance_server.py
# Prints: Local IP: http://192.168.1.X:8765
# Enter this URL in the app under Host Processing mode
```

#### On-device CoreML models

Run once on a Mac to convert PromptDA and StableNormal to CoreML:

```bash
cd mobile/offline_pipeline
python3 convert_models.py
# Outputs: AtlasMobile/Resources/PromptDA.mlpackage
#          AtlasMobile/Resources/StableNormal.mlpackage
```

Then drag both `.mlpackage` files into the Xcode project (Add to targets: AtlasMobile).

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

#### Phase 1: ARKit Capture Prototype ✅
- [x] ARSession with scene depth + smoothed scene depth enabled
- [x] Save RGB frames as JPEG (`CIImage` + `CIContext.jpegRepresentation`)
- [x] Save depth maps as Float32 binary (256×192)
- [x] Save confidence maps as uint8 binary
- [x] Save pose.json per scan (4×4 transform, intrinsics, timestamps)
- [x] Record continuous trajectory (ARKit delegate `didUpdate frame` → `TrajectoryRecorder`)
- [x] Simple SwiftUI interface (start/stop session, trigger capture)
- [x] Export session to Files app (`UIActivityViewController` + `ShareSheet`)
- [ ] **Validate:** load saved data in Python, verify intrinsics and poses are correct

#### Phase 2: Insta360 Integration ✅
- [x] Integrate Insta360 CameraSDK (manual `.xcframework` embed)
- [x] WiFi camera discovery and connection (`INSSocketDevice` + KVO on `cameraState`)
- [x] Programmatic capture trigger (`takePicture(with:completion:)`)
- [x] Capture completion callback handling (URI queued for download)
- [x] Media download from camera (`fetchResource(withURI:toLocalFile:)`)
- [x] Per-camera clock offset estimation (`syncTimeMsToCamera`, saved to `clock_offset_samples.json`)
- [x] Multi-camera support (parallel connect/capture/download over `multi_camera.yaml`)
- [x] Heartbeat timer (0.5 s, required for WiFi stability)
- [ ] **Validate:** timestamps, clock offset stability, download reliability on device

#### Phase 3: COLMAP Export ✅
- [x] ERP → perspective tile slicing (`erp_tile_slicer.py` — atlas-exact 8-face layout, Lanczos)
- [x] ARKit → COLMAP coordinate transform (`R_ARKIT2COLMAP = diag(1,-1,-1)`)
- [x] Derive Insta360 tile poses from rigid extrinsic + ARKit pose
- [x] Write `cameras.bin` (PINHOLE for iPhone, SIMPLE_PINHOLE for tiles)
- [x] Write `images.bin` (quaternion wxyz + translation, all cameras)
- [x] Write `points3D.bin` (from LiDAR depth unprojection)
- [x] Depth map export as uint16 PNG in millimeters (`depth_images/`)
- [x] On-device export wired into `endSession()` (`COLMAPExporter.swift`)
- [x] Offline assembly script (`assemble_colmap.py`) with baseline filter + mask slicing
- [x] Write `rigs.bin` + `frames.bin` for rig-aware BA (one rig per session, iPhone as ref sensor)
- [ ] **Validate:** open output in COLMAP GUI, ingest into splat-toolbox

#### Phase 4: Offline Enhancement ✅
- [x] `setup.sh` — installs PromptDA, StableNormal, Flask, coremltools, and all Python deps
- [x] `ExportMode` enum — three modes: COLMAP Only, Full On-Device, Host Processing
- [x] `SessionExportView` — SwiftUI sheet shown after `endSession()` and from Sessions list
- [x] `PostProcessingManager` — on-device CoreML inference for PromptDA + StableNormal YOSO
- [x] `HostUploader` — pure-Swift zip writer (Compression framework, no SPM dependency) + real inflate for download
- [x] `enhance_server.py` — Flask server: receives zip, runs `enhance_session.py`, streams log, serves result zip
- [x] `convert_models.py` — one-time CoreML conversion of PromptDA ViT-L + StableNormal YOSO (macOS)
- [x] PromptDA integration (`enhance_session.py` Stage 1 — sparse depth → dense uint16 PNG)
- [x] StableNormal integration (`enhance_session.py` Stage 2 — RGB → normal map PNG)
- [x] COLMAP assembly wired as Stage 3 in `enhance_session.py`
- [ ] **Validate:** compare 3DGS quality with vs without enhancement
- [ ] **Validate:** CoreML models on A16 — confirm ANE dispatch and per-scan timing

#### Phase 5: Calibration Tooling ✅
- [x] Physical seed from mount measurements (forward/left/up inches + RPY degrees in `CalibrationView`)
- [x] Project iPhone LiDAR depth into ERP intensity image (`CalibrationOverlayRenderer`, `ExtrinsicOptimizer`)
- [x] Nelder-Mead 6-DOF optimisation on ERP edge-alignment cost — **on-device, no SuperGlue needed** (`ExtrinsicOptimizer.swift`)
- [x] Visual overlay verification — edge alignment + depth dots composite (`CalibrationOverlayRenderer.swift`)
- [x] `CalibrationView` — SwiftUI UI: seed inputs, capture frames, run optimisation, overlay display, save
- [x] Saves refined extrinsic to `Documents/atlas_sessions/multi_camera.yaml` (auto-loaded on next launch)
- [x] `MultiCameraConfig.saveToDocuments()` + `loadFromDeviceOrBundle()` priority chain
- [ ] **Validate:** overlay accuracy on test captures with known geometry

> Calibration runs entirely on-device. No host computer required. The offline pipeline
> (`calibrate_extrinsic.py`) remains available as a higher-accuracy alternative using
> SuperGlue feature matching when a GPU workstation is available.

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
    - CUDA GPU recommended (≥8 GB VRAM for PromptDA + StableNormal)
    - Internet connection on first run (model weights downloaded from HuggingFace, ~2–4 GB)

2. **Install dependencies**
    ```bash
    cd mobile/offline_pipeline
    bash setup.sh
    ```
    Re-running is safe — all steps are idempotent. Skips torch if a CUDA build is already present.

3. **Run enhancement pipeline**
    ```bash
    # Transfer session from iPhone to workstation first (AirDrop / USB / WiFi)
    python3 enhance_session.py --session-dir /path/to/session_2026-08-24_10-30-00
    ```
    Stages run in order: PromptDA → StableNormal → COLMAP assembly.
    Use `--skip-depth` or `--skip-normals` to run individual stages.

4. **Run COLMAP assembly only**
    ```bash
    python3 assemble_colmap.py --session-dir /path/to/session_2026-08-24_10-30-00
    ```

5. **Run calibration**
    ```bash
    python3 calibrate_extrinsic.py \
        --session-dir /path/to/calibration_session \
        --camera-id insta360_primary \
        --seed-from-physical \
        --seed-y 0.05 --seed-z 0.03
    ```

6. **Slice ERP tiles (standalone)**
    ```bash
    python3 erp_tile_slicer.py \
        --erp-image /path/to/panorama.jpg \
        --output-dir /path/to/colmap/images/ \
        --scan-name scan_000 \
        --mask /path/to/masks/insta360_primary.png
    ```

### Specification

See [docs/spec.md](docs/spec.md) for the full technical specification.

See [docs/setup-and-testing.md](docs/setup-and-testing.md) for Xcode project setup, SDK embedding, device provisioning, and per-phase validation.
