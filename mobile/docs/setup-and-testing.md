# Atlas Mobile — Deployment & Testing Setup

## Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| macOS | 13.0+ (Ventura) | Required for Xcode 15 |
| Xcode | 15.0+ | Download from Mac App Store |
| iPhone | 14 Pro or later | LiDAR required — simulator not supported |
| Apple Developer account | Any tier | Free tier works for personal device testing |
| Insta360 X5 (or X4/X3/ONE RS) | Any firmware | For Phase 2+ testing |
| iOS SDK zip | `iOS-SDK-1.10.4.zip` | From Insta360 Developer Portal |

---

## 1. Create the Xcode Project

The `mobile/AtlasMobile/` directory contains Swift source files but no `.xcodeproj` yet. Create it once:

1. Open Xcode → **File → New → Project**
2. Choose **iOS → App**
3. Fill in:
   - Product Name: `AtlasMobile`
   - Team: your Apple Developer team
   - Bundle Identifier: `com.yourname.AtlasMobile`
   - Interface: **SwiftUI**
   - Language: **Swift**
4. Save into `mobile/AtlasMobile/` — Xcode will create `AtlasMobile.xcodeproj` there

### Add source files

In the Xcode project navigator, right-click the `AtlasMobile` group → **Add Files to "AtlasMobile"**. Add all `.swift` files from these directories (check **Copy items if needed** = OFF, **Create groups** = ON):

```
mobile/AtlasMobile/AtlasMobile/App/
mobile/AtlasMobile/AtlasMobile/Calibration/
mobile/AtlasMobile/AtlasMobile/Capture/
mobile/AtlasMobile/AtlasMobile/Export/
mobile/AtlasMobile/AtlasMobile/Models/
mobile/AtlasMobile/AtlasMobile/Utils/
mobile/AtlasMobile/AtlasMobile/Views/
```

Also add `mobile/AtlasMobile/AtlasMobile/Info.plist` to the target.

---

## 2. Embed the Insta360 SDK

## 2a. Embed the OpenCV iOS Framework

OpenCV provides KAZE feature detection and FLANN matching used by `FeatureMatcher.mm`.

### Download

```bash
cd ~/Downloads
curl -L -o opencv-5.0.0-ios-framework.zip \
  https://github.com/opencv/opencv/releases/download/5.0.0/opencv-5.0.0-ios-framework.zip
unzip opencv-5.0.0-ios-framework.zip
# Produces: opencv2.framework
```

### Add to Xcode

1. Xcode → target → **General → Frameworks, Libraries, and Embedded Content**
2. Click **+** → **Add Other → Add Files** → select `opencv2.framework`
3. Set to **Embed & Sign**

### Bridging header

Create `mobile/AtlasMobile/AtlasMobile/AtlasMobile-Bridging-Header.h`:

```objc
#import "FeatureMatcher.h"
```

Then in Xcode → target → **Build Settings → Swift Compiler - General → Objective-C Bridging Header**, set the path to:
```
$(SRCROOT)/AtlasMobile/AtlasMobile-Bridging-Header.h
```

### Build settings

In **Build Settings**, set:
- **C++ Language Dialect** → `C++20`
- **C++ Standard Library** → `libc++ (LLVM C++ standard library)`

---

## 2b. Embed the Insta360 SDK

The SDK ships as `.xcframework` bundles inside `iOS-SDK-1.10.4.zip`.

### Extract

```bash
cd ~/Downloads
unzip iOS-SDK-1.10.4.zip -d iOS-SDK-1.10.4/
```

The frameworks are at:
```
iOS-SDK-1.10.4/iOS_v1.10.4/INSCameraSDKSample-bluetooth/Frameworks/
  INSCameraSDK.xcframework
  INSCameraServiceSDK.xcframework
  SSZipArchive.xcframework
  Eureka.xcframework          # sample app only — not needed
```

### Add to Xcode

1. In Xcode, select the `AtlasMobile` target → **General → Frameworks, Libraries, and Embedded Content**
2. Click **+** → **Add Other → Add Files**
3. Navigate to the extracted `Frameworks/` directory and add:
   - `INSCameraSDK.xcframework`
   - `INSCameraServiceSDK.xcframework`
   - `SSZipArchive.xcframework`
4. Set all three to **Embed & Sign**

---

## 3. Configure Info.plist

Add these keys to `Info.plist` (Xcode → target → Info tab, or edit the file directly):

| Key | Type | Value |
|---|---|---|
| `NSCameraUsageDescription` | String | `Atlas Mobile uses the camera for 3D scene capture` |
| `NSLocalNetworkUsageDescription` | String | `Atlas Mobile connects to Insta360 cameras via WiFi Direct` |
| `NSBonjourServices` | Array | `_http._tcp` |
| `UIRequiredDeviceCapabilities` | Array | `arkit`, `lidar-depth-camera` |

The `lidar-depth-camera` capability ensures the app is only installable on LiDAR-equipped devices and prevents ARKit from silently falling back to estimated depth.

---

## 4. Signing & Capabilities

1. Xcode → target → **Signing & Capabilities**
2. Set **Team** to your Apple Developer account
3. Click **+ Capability** and add:
   - **Camera** — required for ARKit `capturedImage`
   - **Access WiFi Information** — required for Insta360 WiFi Direct connection

---

## 5. Build & Deploy

Connect your iPhone via USB, select it as the run destination, then:

```
Cmd+R   — build and run
Cmd+B   — build only
```

On first run, iOS will prompt to trust the developer certificate:
**iPhone → Settings → General → VPN & Device Management → [your team] → Trust**

---

## 6. Phase 1 Validation — ARKit Capture

This validates that poses, intrinsics, depth, and RGB are saved correctly before involving the Insta360.

### On device

1. Launch the app
2. Tap **Start Session** — ARKit tracking begins, session directory created
3. Move the phone around the room for a few seconds (builds trajectory)
4. Tap **Capture Scan** 3–5 times from different positions
5. Tap **End Session**
6. Tap **Share Last Session** → **Save to Files** → save to iCloud Drive or AirDrop to Mac

### On workstation

Transfer the session directory, then run the validation script:

```bash
cd mobile/offline_pipeline
python3 - <<'EOF'
import json, numpy as np, struct
from pathlib import Path

session = Path("/path/to/session_YYYY-MM-DD_HH-MM-SS")

for scan_dir in sorted((session / "raw/iphone").glob("scan_*")):
    pose = json.loads((scan_dir / "pose.json").read_text())

    T = np.array(pose["transform_matrix"])
    K = np.array(pose["intrinsics"])
    assert T.shape == (4, 4), "transform_matrix must be 4x4"
    assert abs(np.linalg.det(T[:3,:3]) - 1.0) < 1e-3, "rotation not orthonormal"
    assert T[3] == [0,0,0,1], "last row must be [0,0,0,1]"

    fx, fy = K[0,0], K[1,1]
    cx, cy = K[0,2], K[1,2]
    w, h = pose["image_width"], pose["image_height"]
    assert 1000 < fx < 4000, f"fx={fx:.1f} out of expected range for iPhone"
    assert abs(cx - w/2) < w*0.1, f"cx={cx:.1f} far from image centre"

    depth = np.frombuffer((scan_dir / "depth.bin").read_bytes(), dtype=np.float32)
    depth = depth[:256*192].reshape(192, 256)
    valid = depth[(depth > 0.1) & (depth < 5.0)]
    assert len(valid) > 1000, f"Too few valid depth pixels: {len(valid)}"

    print(f"  {scan_dir.name}: T ok, fx={fx:.0f} fy={fy:.0f} cx={cx:.0f} cy={cy:.0f}, "
          f"depth valid={len(valid)}/{depth.size} mean={valid.mean():.2f}m")

traj = json.loads((session / "raw/trajectory.json").read_text())
print(f"\nTrajectory: {len(traj['poses'])} poses")
print("Phase 1 validation PASSED")
EOF
```

Expected output:
```
  scan_000: T ok, fx=2016 fy=2016 cx=2016 cy=1512, depth valid=18432/49152 mean=1.84m
  scan_001: ...
Trajectory: 847 poses
Phase 1 validation PASSED
```

---

## 7. Phase 2 Validation — Insta360 Integration

### Camera setup

1. Power on the Insta360 X5
2. On the camera: **Settings → Connection → WiFi Direct** — confirm it's broadcasting its AP SSID
3. On iPhone: **Settings → WiFi** — connect to the camera's SSID (e.g. `Insta360 X5 XXXXXX`, password on camera screen)
4. Return to the Atlas Mobile app

> The app connects programmatically via `INSSocketDevice(host: "192.168.42.1", port: 6666)` — you do not need to stay in the WiFi settings screen.

### Validate

1. Tap **Start Session** — the app will attempt to connect to the camera. `connectedCameraCount` in the status view should show `1`
2. Tap **Capture Scan** — the camera shutter should fire
3. Tap **End Session** — media downloads, clock offset saved
4. Share session to Mac and check:

```bash
python3 - <<'EOF'
import json
from pathlib import Path

session = Path("/path/to/session_YYYY-MM-DD_HH-MM-SS")

# Check clock offset was recorded
offsets_path = session / "calibration/clock_offset_samples.json"
assert offsets_path.exists(), "clock_offset_samples.json missing"
offsets = json.loads(offsets_path.read_text())
for o in offsets:
    print(f"  {o['camera_id']}: offset={o['offset_ms']:.1f}ms samples={o['sample_count']}")
    assert o['sample_count'] > 0, "No clock offset samples recorded"

# Check Insta360 images downloaded
for cam_dir in (session / "raw").iterdir():
    if cam_dir.name == "iphone": continue
    jpgs = list(cam_dir.glob("scan_*.jpg"))
    print(f"  {cam_dir.name}: {len(jpgs)} images downloaded")
    assert len(jpgs) > 0, f"No images downloaded for {cam_dir.name}"

print("Phase 2 validation PASSED")
EOF
```

---

## 8. Calibration — Insta360 ↔ iPhone Extrinsic

Calibration must be done **before** running COLMAP export. The `multi_camera.yaml` extrinsic is used by both `assemble_colmap.py` and `COLMAPExporter.swift` to derive Insta360 tile poses.

### Capture a calibration session

Capture 3–5 scans of a **textured scene** (bookshelves, posters, furniture — avoid blank walls). The scene should be visible to both the iPhone wide camera and the Insta360. Move the rig slightly between scans to get varied viewpoints.

### Step 1: Physical seed

Measure the physical offset of the Insta360 lens centre from the iPhone camera in your mount:

```bash
cd mobile/offline_pipeline
python3 calibrate_extrinsic.py \
    --session-dir /path/to/session_YYYY-MM-DD_HH-MM-SS \
    --camera-id insta360_primary \
    --forward 0.0 --left 0.0 --up 2.0
    # forward/left/up in inches from iPhone camera to Insta360 lens centre
    # --skip-matching to generate seed overlay only (no SuperGlue needed)
```

This writes `calibration/insta360_primary_seed_overlay.jpg`. Open it and check:
- Green LiDAR edges should roughly align with red Insta360 edges
- If shifted horizontally → adjust `--yaw`
- If shifted vertically → adjust `--pitch`
- If rotated → adjust `--roll`

### Step 2: SuperGlue refinement

Once the seed overlay looks roughly aligned (within ~5°), run the full pipeline:

```bash
python3 calibrate_extrinsic.py \
    --session-dir /path/to/session \
    --camera-id insta360_primary \
    --forward 0.0 --left 0.0 --up 2.0
```

Requires SuperGluePretrainedNetwork at `~/atlas_ws/SuperGluePretrainedNetwork`:

```bash
cd ~/atlas_ws
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git
```

Outputs:
- `calibration/insta360_primary_extrinsic.yaml` — refined 6-DOF transform
- `calibration/insta360_primary_refined_overlay.jpg` — post-refinement verification
- `calibration/multi_camera.yaml` — updated with refined extrinsic

### Step 3: Visual fine-tuning (if needed)

If the refined overlay still shows misalignment on one axis, sweep it:

```bash
python3 calibrate_extrinsic.py \
    --session-dir /path/to/session \
    --camera-id insta360_primary \
    --forward 0.0 --left 0.0 --up 2.0 \
    --sweep-axis yaw --sweep-range 5.0 --sweep-steps 5
# Generates calibration/sweep_yaw/sweep_yaw_*.jpg
# Pick the best-aligned image, note its offset, re-run with adjusted --yaw
```

### Step 4: Copy calibration to config

Once satisfied, copy the calibrated `multi_camera.yaml` to the shared config so all future sessions use it:

```bash
cp /path/to/session/calibration/multi_camera.yaml mobile/config/multi_camera.yaml
```

---

## 9. Phase 3 Validation — COLMAP Export

Run the offline assembler and open the result in COLMAP:

```bash
cd mobile/offline_pipeline
python3 assemble_colmap.py --session-dir /path/to/session_YYYY-MM-DD_HH-MM-SS
```

Then open in COLMAP GUI:

```bash
colmap gui
# File → Import Model → select session/colmap/sparse/0/
```

Expected: cameras and image poses visible, LiDAR point cloud rendered as sparse points.

Quick binary sanity check:

```bash
python3 - <<'EOF'
import struct
from pathlib import Path

sparse = Path("/path/to/session/colmap/sparse/0")

with open(sparse / "cameras.bin", "rb") as f:
    n_cams = struct.unpack("Q", f.read(8))[0]
    print(f"cameras.bin: {n_cams} cameras")

with open(sparse / "images.bin", "rb") as f:
    n_imgs = struct.unpack("Q", f.read(8))[0]
    print(f"images.bin: {n_imgs} images")

with open(sparse / "points3D.bin", "rb") as f:
    n_pts = struct.unpack("Q", f.read(8))[0]
    print(f"points3D.bin: {n_pts} points")

assert n_cams >= 2, "Expected at least iPhone + 1 tile camera"
assert n_imgs >= 1, "No images written"
assert n_pts > 0, "No LiDAR points written"
print("Phase 3 binary check PASSED")
EOF
```

---

## 10. Phase 4 Validation — Export Modes

### Option A: COLMAP Only (default)

The COLMAP model is built automatically when you tap **End Session**. No extra steps needed.

Validate with the binary check from Section 8.

### Option B: Full On-Device (CoreML)

#### One-time: convert models to CoreML (requires macOS + Apple Silicon)

```bash
cd mobile/offline_pipeline
bash setup.sh          # installs coremltools on macOS automatically
python3 convert_models.py
# Output: AtlasMobile/Resources/PromptDA.mlpackage
#         AtlasMobile/Resources/StableNormal.mlpackage
```

Then in Xcode:
1. Drag both `.mlpackage` files into the project navigator
2. Ensure **Add to targets: AtlasMobile** is checked for both
3. Rebuild and deploy to device

#### On device

1. End a session — the Export sheet appears automatically
2. Select **Full On-Device**
3. Tap **Export** — PromptDA runs first (~1–3s/scan), then StableNormal YOSO (~5–15s/scan)
4. If either model shows "Not bundled", complete the conversion step above

Validate outputs:

```bash
python3 - <<'EOF'
import cv2, numpy as np
from pathlib import Path

session = Path("/path/to/session_YYYY-MM-DD_HH-MM-SS")

for p in sorted((session / "enhanced/depth_dense").glob("*.png")):
    d = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
    assert d.dtype == np.uint16, f"{p.name}: expected uint16"
    valid = d[d > 0]
    print(f"  depth {p.name}: {len(valid)} valid px, mean={valid.mean()/1000:.2f}m")

for p in sorted((session / "enhanced/normals").glob("*.png")):
    n = cv2.imread(str(p))
    assert n is not None and n.shape[2] == 3
    print(f"  normal {p.name}: {n.shape[1]}x{n.shape[0]}")

print("On-device enhancement PASSED")
EOF
```

### Option C: Host Processing

#### On workstation (same WiFi network as iPhone)

```bash
cd mobile/offline_pipeline
bash setup.sh          # first time only
python3 enhance_server.py
# Prints: Local IP: http://192.168.1.X:8765
```

#### On device

1. End a session — the Export sheet appears
2. Select **Host Processing**
3. Enter the host URL shown by `enhance_server.py` (e.g. `http://192.168.1.42:8765`)
4. Tap **Export** — session is zipped, uploaded, processed, and results downloaded automatically
5. Watch the **Host Log** section for real-time pipeline output

The host runs the full pipeline: PromptDA ViT-L + StableNormal diffusion refinement + COLMAP assembly. Results are downloaded back into the session directory under `enhanced_host/`.

Verify the server is reachable before starting:

```bash
curl http://192.168.1.42:8765/health
# {"status": "ok"}
```

---

## 11. Common Issues

| Symptom | Cause | Fix |
|---|---|---|
| `ARSession` returns `nil` depth | Running on simulator or non-LiDAR device | Must use iPhone 14 Pro or later |
| `connectedCameraCount` stays 0 | iPhone not connected to camera WiFi AP | Connect to camera SSID in iOS Settings first |
| `takePicture` callback never fires | Heartbeat not running or camera disconnected | Check `INSCameraManager.socket().cameraState == .connected` |
| `depth.bin` all zeros | `sceneDepth` not enabled or device too close | Minimum LiDAR range is ~0.1m; ensure `frameSemantics = [.sceneDepth]` |
| COLMAP shows no points | LiDAR depth all invalid or pose wrong | Run Phase 1 validation script first |
| PromptDA OOM on host | GPU VRAM < 8 GB | Add `--skip-depth` and run PromptDA separately with smaller batch |
| CoreML model shows "Not bundled" | `.mlpackage` not added to Xcode target | Run `convert_models.py`, drag into Xcode, check "Add to targets" |
| Host upload fails | Server not running or wrong IP | Run `enhance_server.py` and verify with `curl .../health` |
| Host upload fails | iPhone and workstation on different networks | Both must be on the same WiFi network |
| `clock_offset_samples.json` missing | Session ended before Insta360 connected | Ensure `connectedCameraCount > 0` before ending session |
