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

#### Phase 6: Live Capture UI
- [ ] **AR camera preview** — replace the blank capture screen with an `ARView` (RealityKit) or `ARSCNView` (SceneKit) showing the live iPhone camera feed during sessions
- [ ] **Tracking state indicator** — display ARKit tracking quality (normal / limited / not available) with a colored badge or banner; show the reason when limited (insufficient features, excessive motion, initializing)
- [ ] **Scan counter overlay** — show scan count on top of the camera preview instead of in a separate status bar
- [ ] **Capture flash feedback** — brief screen flash or border animation on "Capture Scan" so the user knows it fired
- [ ] **Last-capture thumbnail** — after each scan, show a small thumbnail of the captured RGB frame in a corner of the preview
- [ ] **LiDAR depth overlay** — optional toggle to render the live depth map as a colored overlay on the camera feed (confidence-weighted coloring)
- [ ] **Point cloud visualization** — render accumulated LiDAR points as a 3D point cloud in the AR view, giving the user real-time spatial feedback
- [ ] **Mesh visualization** — optionally render `ARMeshAnchor` geometry (available on LiDAR devices) as a wireframe overlay

#### Phase 7: Insta360 Connection UI
- [ ] **Connection status screen** — show camera discovery and connection progress when starting a session (connecting, connected, failed per camera)
- [ ] **Camera status badge** — persistent indicator showing connected camera count and names during an active session
- [ ] **Manual retry** — button to retry connection if a camera fails to connect
- [ ] **Disconnection alert** — notify the user if an Insta360 camera drops mid-session (heartbeat failure)
- [ ] **WiFi reconnection handling** — automatic reconnect attempt when Insta360 WiFi drops, with user notification
- [ ] **Preview from Insta360** — show the most recent Insta360 capture thumbnail after each scan (downloaded at capture time, not deferred to end-session)

#### Phase 8: Session Management & Polish
- [ ] **Session browser improvements** — add delete, rename, and storage usage display to `SessionListView`
- [ ] **Session resume** — allow resuming a previously ended session (re-open the same directory, continue scan numbering)
- [ ] **Capture guidance** — coverage indicator showing which directions have been scanned, suggesting where to scan next
- [ ] **Thermal management** — monitor `ProcessInfo.thermalState`, throttle capture rate or warn user when thermal pressure is high
- [ ] **Battery usage optimization** — reduce ARKit frame rate when idle (between captures), disable unnecessary sensors
- [ ] **Session transfer** — streamlined export to workstation via AirDrop, USB (Finder), or WiFi direct
- [ ] **3D session viewer** — after ending a session, show captured point cloud and camera positions in a 3D viewer for quality review

### Local Development Setup

#### iOS App (requires macOS)

1. **Prerequisites**
    - macOS 13+ with Xcode 15+ (16.2+ recommended)
    - iPhone 14 Pro or later (LiDAR required, simulator not supported)
    - Apple Developer account (for device deployment)
    - Insta360 X5 (or supported model) for integration testing
    - Insta360 iOS SDK zip (`iOS-SDK-1.10.4.zip`) from the [Insta360 Developer Portal](https://www.insta360.com/developer) or `s3://<BUCKET>/code/iOS-SDK-1.10.4.zip`
    - OpenCV iOS framework from [GitHub releases](https://github.com/opencv/opencv/releases)

2. **Clone and checkout**
    ```bash
    git clone https://github.com/Eecornwell/atlas-scanner.git
    cd atlas-scanner
    git checkout mobile
    cd mobile/AtlasMobile
    ```

3. **Install frameworks**

    Extract the Insta360 SDK and OpenCV framework, then copy into the project:
    ```bash
    mkdir -p Frameworks

    # Insta360 SDK
    unzip ~/Downloads/iOS-SDK-1.10.4.zip -d /tmp/iOS-SDK-1.10.4/
    cp -R /tmp/iOS-SDK-1.10.4/iOS_v1.10.4/INSCameraSDKSample-bluetooth/Frameworks/INSCameraSDK.xcframework Frameworks/
    cp -R /tmp/iOS-SDK-1.10.4/iOS_v1.10.4/INSCameraSDKSample-bluetooth/Frameworks/INSCameraServiceSDK.xcframework Frameworks/
    cp -R /tmp/iOS-SDK-1.10.4/iOS_v1.10.4/INSCameraSDKSample-bluetooth/Frameworks/INSCoreMedia.xcframework Frameworks/
    cp -R /tmp/iOS-SDK-1.10.4/iOS_v1.10.4/INSCameraSDKSample-bluetooth/Frameworks/SSZipArchive.xcframework Frameworks/

    # OpenCV
    curl -L -o /tmp/opencv-ios.zip https://github.com/opencv/opencv/releases/download/5.0.0/opencv-5.0.0-ios-framework.zip
    unzip /tmp/opencv-ios.zip -d /tmp/opencv/
    cp -R /tmp/opencv/opencv2.framework Frameworks/
    ```

4. **Create the Xcode project**

    The `AtlasMobile/` directory contains Swift source files but no `.xcodeproj`. There are two options:

    **Option A: Headless build with XcodeGen (CI / EC2 Mac instances)**

    Install XcodeGen:
    ```bash
    curl -fsSL -o /tmp/xcodegen.zip https://github.com/yonaskolb/XcodeGen/releases/latest/download/xcodegen.zip
    cd /tmp && unzip xcodegen.zip
    ```

    Create `project.yml` in `mobile/AtlasMobile/`:
    ```yaml
    name: AtlasMobile
    options:
      bundleIdPrefix: com.atlas
      deploymentTarget:
        iOS: "17.0"
      xcodeVersion: "16.2"

    settings:
      base:
        SWIFT_VERSION: "5.9"
        CLANG_CXX_LANGUAGE_STANDARD: c++20
        CLANG_CXX_LIBRARY: libc++
        FRAMEWORK_SEARCH_PATHS: $(inherited) $(SRCROOT)/Frameworks
        SWIFT_OBJC_BRIDGING_HEADER: AtlasMobile/AtlasMobile-Bridging-Header.h

    targets:
      AtlasMobile:
        type: application
        platform: iOS
        sources:
          - path: AtlasMobile
            excludes:
              - "**/*.xcodeproj"
        info:
          path: AtlasMobile/Info.plist
          properties:
            NSCameraUsageDescription: "ATLAS Mobile uses the camera and LiDAR sensor for 3D scene capture."
            NSLocalNetworkUsageDescription: "ATLAS Mobile connects to Insta360 cameras over the local network."
            NSBonjourServices:
              - "_insta360._tcp"
            UIRequiredDeviceCapabilities:
              - arkit
              - lidar
            UILaunchScreen: {}
        settings:
          base:
            PRODUCT_BUNDLE_IDENTIFIER: com.cornwell.atlas.AtlasMobile
            CODE_SIGN_IDENTITY: ""
            CODE_SIGNING_REQUIRED: "NO"
            CODE_SIGNING_ALLOWED: "NO"
            DEVELOPMENT_TEAM: ""
        dependencies:
          - framework: Frameworks/INSCameraSDK.xcframework
            embed: true
          - framework: Frameworks/INSCameraServiceSDK.xcframework
            embed: true
          - framework: Frameworks/INSCoreMedia.xcframework
            embed: true
          - framework: Frameworks/SSZipArchive.xcframework
            embed: true
          - framework: Frameworks/opencv2.framework
            embed: false
          - sdk: ARKit.framework
          - sdk: CoreML.framework
          - sdk: CoreVideo.framework
          - sdk: CoreImage.framework
          - sdk: Accelerate.framework
    ```

    > **Important:** `opencv2.framework` must use `embed: false`. OpenCV is a static library — its code is linked into the main binary at build time. Embedding the framework bundle causes iOS to reject the app with `MissingBundleExecutable` because the `.framework` wrapper has no valid executable.

    Create the bridging header at `AtlasMobile/AtlasMobile-Bridging-Header.h`:
    ```objc
    #import "FeatureMatcher.h"
    ```

    Generate and build:
    ```bash
    /tmp/xcodegen/bin/xcodegen generate --spec project.yml
    xcodebuild -project AtlasMobile.xcodeproj \
      -scheme AtlasMobile \
      -sdk iphoneos \
      -configuration Release \
      CODE_SIGN_IDENTITY="" \
      CODE_SIGNING_REQUIRED=NO \
      CODE_SIGNING_ALLOWED=NO
    ```

    **Option B: Interactive Xcode GUI (local Mac)**

    See [docs/setup-and-testing.md](docs/setup-and-testing.md) for step-by-step Xcode GUI instructions including manual framework embedding, bridging header setup, and build settings.

5. **Configure signing** (required for device deployment)
    - Select your team in Xcode → Target → Signing & Capabilities
    - Or pass `CODE_SIGN_IDENTITY` and `DEVELOPMENT_TEAM` to `xcodebuild`

6. **Build and run**
    ```bash
    # Command line
    xcodebuild -scheme AtlasMobile -destination 'platform=iOS,name=<your-device>'
    ```
    Or press Cmd+R in Xcode with your iPhone connected.

7. **Test ARKit capture (Phase 1)**
    - Launch app on device
    - Tap "Start Session" → "Capture Scan" → "End Session"
    - Verify output in Files app → AtlasMobile → atlas_sessions/

#### EC2 Mac Build (no local Mac)

If you don't have a Mac, you can build on an EC2 Mac Dedicated Host. **Expect ~$26/day** (mac1.metal on-demand) with a **24-hour minimum allocation**.

> **Known restriction:** `mac2.metal` (Apple Silicon) may be blocked by your AWS organization's Service Control Policies. If you get `UnsupportedHostConfiguration`, fall back to `mac1.metal` (Intel). Both work for iOS cross-compilation.

1. **Allocate a Dedicated Host** (24-hour minimum billing applies)
    ```bash
    aws ec2 allocate-hosts \
      --instance-type mac1.metal \
      --availability-zone us-east-1a \
      --quantity 1
    ```
    Save the `HostId` from the response — you'll need it for the next step.

2. **Launch an instance** using a macOS AMI

    Find the latest macOS AMI. The architecture filter must be `x86_64_mac` (not `x86_64`):
    ```bash
    aws ec2 describe-images --owners amazon \
      --filters "Name=name,Values=amzn-ec2-macos-15*" \
                "Name=architecture,Values=x86_64_mac" \
      --query "Images | sort_by(@, &CreationDate) | [-1].[ImageId,Name]" \
      --output table
    ```

    Create `bdm.json` for the 200 GB root volume:
    ```json
    [{"DeviceName":"/dev/sda1","Ebs":{"VolumeSize":200,"VolumeType":"gp3"}}]
    ```

    Launch the instance (use `--placement` not `--host-id`):
    ```bash
    aws ec2 run-instances \
      --instance-type mac1.metal \
      --placement "HostId=<host-id>" \
      --image-id <ami-id> \
      --key-name <key-name> \
      --security-group-ids <sg-id> \
      --block-device-mappings file://bdm.json
    ```

    > **PowerShell note:** JSON with quotes in `--block-device-mappings` breaks in PowerShell. Always use `file://bdm.json` instead of inline JSON.

3. **Connect via SSM** (no SSH key required — avoids key pair issues)
    ```bash
    aws ssm start-session --target <instance-id>
    ```

    The SSM session starts as `ssm-user` with a minimal environment. Fix it immediately:
    ```bash
    export HOME=/Users/ec2-user
    export USER=ec2-user
    cd /Users/ec2-user
    ```

4. **Install Xcode** on the instance

    Homebrew is unsupported on Intel EC2 Macs. Download the `xcodes` binary directly:
    ```bash
    curl -fsSL -o /tmp/xcodes.zip \
      https://github.com/XcodesOrg/xcodes/releases/latest/download/xcodes.zip
    cd /tmp && unzip xcodes.zip && chmod +x xcodes

    # Install Xcode (prompts for Apple ID)
    ./xcodes install 16.2
    ```

    If `xcodes` hangs during xip extraction (common under SSM), extract manually:
    ```bash
    # Find the downloaded xip
    ls /Users/ec2-user/Library/Caches/com.robotsandpencils.xcodes/*.xip

    # Extract with sudo (TMPDIR must be writable)
    cd /Applications
    sudo xip -x "/Users/ec2-user/Library/Caches/com.robotsandpencils.xcodes/Xcode_16.2.xip"
    sudo mv Xcode.app /Applications/Xcode-16.2.0.app
    ```

    Configure Xcode and install the iOS platform:
    ```bash
    sudo xcode-select -s /Applications/Xcode-16.2.0.app/Contents/Developer
    sudo xcodebuild -runFirstLaunch
    sudo xcodebuild -license accept
    sudo xcodebuild -downloadPlatform iOS
    ```

    > **Important:** The iOS platform download (`-downloadPlatform iOS`) is required even though `xcodebuild -showsdks` may list `iphoneos18.2`. Without it, builds fail with "Found no destinations" or "iOS 18.2 is not installed."

5. **Install XcodeGen**

    XcodeGen generates the `.xcodeproj` from `project.yml`:
    ```bash
    curl -fsSL -o /tmp/xcodegen.zip \
      https://github.com/yonaskolb/XcodeGen/releases/latest/download/xcodegen.zip
    unzip -o /tmp/xcodegen.zip -d /Users/ec2-user/
    ```
    The binary is at `/Users/ec2-user/xcodegen/bin/xcodegen`.

6. **Clone the repo and install frameworks**
    ```bash
    cd /Users/ec2-user
    git clone https://github.com/Eecornwell/atlas-scanner.git
    cd atlas-scanner && git checkout mobile
    cd mobile/AtlasMobile
    mkdir -p Frameworks
    ```

    Copy the Insta360 SDK frameworks. Upload the SDK zip to S3 or transfer via SCP:
    ```bash
    # Download SDK from S3
    aws s3 cp s3://<BUCKET>/iOS-SDK-1.10.4.zip /tmp/
    cd /tmp && unzip iOS-SDK-1.10.4.zip -d iOS-SDK-1.10.4/

    SDK_FW=/tmp/iOS-SDK-1.10.4/iOS_v1.10.4/INSCameraSDKSample-bluetooth/Frameworks
    cp -R $SDK_FW/INSCameraSDK.xcframework Frameworks/
    cp -R $SDK_FW/INSCameraServiceSDK.xcframework Frameworks/
    cp -R $SDK_FW/INSCoreMedia.xcframework Frameworks/
    cp -R $SDK_FW/SSZipArchive.xcframework Frameworks/

    # OpenCV
    curl -L -o /tmp/opencv-ios.zip \
      https://github.com/opencv/opencv/releases/download/5.0.0/opencv-5.0.0-ios-framework.zip
    unzip /tmp/opencv-ios.zip -d /tmp/opencv/
    cp -R /tmp/opencv/opencv2.framework Frameworks/
    ```

    Verify all five frameworks are present:
    ```bash
    ls -d Frameworks/*.xcframework Frameworks/*.framework
    # Expected:
    # Frameworks/INSCameraSDK.xcframework
    # Frameworks/INSCameraServiceSDK.xcframework
    # Frameworks/INSCoreMedia.xcframework
    # Frameworks/SSZipArchive.xcframework
    # Frameworks/opencv2.framework
    ```

    > **Warning:** If you re-extract the project zip later, the Frameworks/ directory will be overwritten. Re-copy from `/tmp` after any re-extraction.

7. **Generate and build**
    ```bash
    export HOME=/Users/ec2-user
    export USER=ec2-user
    cd /Users/ec2-user/atlas-scanner/mobile/AtlasMobile

    /Users/ec2-user/xcodegen/bin/xcodegen generate --spec project.yml

    xcodebuild -project AtlasMobile.xcodeproj \
      -scheme AtlasMobile \
      -sdk iphoneos \
      -destination 'generic/platform=iOS' \
      -configuration Release \
      CODE_SIGN_IDENTITY="" \
      CODE_SIGNING_REQUIRED=NO \
      CODE_SIGNING_ALLOWED=NO
    ```

    > **Troubleshooting:**
    > - `"Couldn't find current username"` from XcodeGen → set `HOME` and `USER` env vars
    > - `"Found no destinations"` → run `sudo xcodebuild -downloadPlatform iOS`
    > - `"overlapping accesses"` in HostUploader.swift → capture `.count` to a local variable before `withUnsafeMutableBytes`/`withUnsafeBytes` closures
    > - FeatureMatcher.mm compile errors → ensure `AtlasMobile-Bridging-Header.h` exists with `#import "FeatureMatcher.h"`

8. **Archive and export a signed IPA** (for device deployment)

    To deploy to a physical device, you need a signed IPA. This requires an Apple Developer account with a provisioning profile that includes the target device's UDID.

    Create `ExportOptionsAdHoc.plist`:
    ```xml
    <?xml version="1.0" encoding="UTF-8"?>
    <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
      "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
    <plist version="1.0">
    <dict>
      <key>method</key>
      <string>ad-hoc</string>
      <key>teamID</key>
      <string>YOUR_TEAM_ID</string>
      <key>signingStyle</key>
      <string>manual</string>
      <key>signingCertificate</key>
      <string>Apple Distribution</string>
      <key>provisioningProfiles</key>
      <dict>
        <key>com.cornwell.atlas.AtlasMobile</key>
        <string>YOUR_PROFILE_NAME</string>
      </dict>
    </dict>
    </plist>
    ```

    Copy the provisioning profile to the build user's directory. SSM `send-command` runs as root, interactive SSM runs as `ssm-user`:
    ```bash
    # For interactive SSM sessions (ssm-user)
    mkdir -p /Users/ssm-user/Library/MobileDevice/Provisioning\ Profiles/
    cp /Users/ec2-user/YourProfile.mobileprovision \
       /Users/ssm-user/Library/MobileDevice/Provisioning\ Profiles/

    # For SSM send-command (runs as root)
    sudo mkdir -p /var/root/Library/MobileDevice/Provisioning\ Profiles/
    sudo cp /Users/ec2-user/YourProfile.mobileprovision \
       /var/root/Library/MobileDevice/Provisioning\ Profiles/
    ```

    Archive and export:
    ```bash
    xcodebuild archive \
      -project AtlasMobile.xcodeproj \
      -scheme AtlasMobile \
      -archivePath ~/AtlasMobile.xcarchive \
      -destination "generic/platform=iOS" \
      CODE_SIGN_STYLE=Manual \
      CODE_SIGN_IDENTITY="Apple Distribution" \
      PROVISIONING_PROFILE_SPECIFIER="YOUR_PROFILE_NAME" \
      DEVELOPMENT_TEAM="YOUR_TEAM_ID"

    xcodebuild -exportArchive \
      -archivePath ~/AtlasMobile.xcarchive \
      -exportPath ~/AtlasMobile_export \
      -exportOptionsPlist ~/ExportOptionsAdHoc.plist
    ```

    The IPA is at `~/AtlasMobile_export/AtlasMobile.ipa`.

    > **Important:** After running XcodeGen, verify that the `Embed Frameworks` build phase in the generated `.xcodeproj` does NOT include `opencv2.framework`. If it does, remove it:
    > ```bash
    > sed -i.bak '/opencv2.framework in Embed Frameworks/d' \
    >   AtlasMobile.xcodeproj/project.pbxproj
    > ```

9. **Install the IPA on a device** (no Mac required)

    Transfer the IPA to any machine (Windows/Mac/Linux) with USB access to the iPhone:
    ```bash
    # Install pymobiledevice3 (requires Python 3.10+)
    pip install pymobiledevice3

    # Connect iPhone via USB, then:
    pymobiledevice3 apps install AtlasMobile.ipa
    ```

    Or host the IPA for OTA installation via HTTPS (requires valid TLS certificate):
    - Upload the IPA and a manifest plist to an HTTPS server
    - Open `itms-services://?action=download-manifest&url=<manifest-url>` on the device

10. **Release the host** after 24 hours to stop billing
    ```bash
    aws ec2 terminate-instances --instance-ids <instance-id>
    # Wait for instance to terminate, then:
    aws ec2 release-hosts --host-ids <host-id>
    ```
    The host cannot be released until 24 hours after allocation. Check with:
    ```bash
    aws ec2 describe-hosts --host-ids <host-id> \
      --query "Hosts[0].AllocationTime"
    ```

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
