# Calibration Procedure

> *Note: Ensure the camera is turned on and the software installation procedure was successful before performing calibration.*

## Camera Hardware Selection

ATLAS supports multiple Insta360 camera models. Each model has its own:
- ERP output resolution
- Lidar mask image
- Calibration file stored at `config/calibrations/<model>/fusion_calibration.yaml`

| Model | CLI flag | ERP Resolution | Config dir |
|-------|----------|----------------|------------|
| Insta360 One X2 | `--camera-hw onex2` | 5760 × 2880  | `config/calibrations/onex2/` |
| Insta360 X3     | `--camera-hw x3`    | 7680 × 3840  | `config/calibrations/x3/`   |
| Insta360 X5     | `--camera-hw x5`    | 11520 × 5760 | `config/calibrations/x5/`   |

> *Note: All calibration scripts accept `--camera-hw onex2|x3|x5`. Always pass the correct model for the hardware you are calibrating. Calibration results are saved to the per-model directory and also copied to the shared `config/fusion_calibration.yaml` which is the active calibration used at runtime.*

> *Note: When you start a capture session the script automatically loads `config/calibrations/<camera_hw>/fusion_calibration.yaml` based on the selected `CAMERA_HW` setting (or `--camera-hw` CLI flag / GUI dropdown). You do not need to manually copy files between captures — just set the correct model.*

---

## Lidar/Rig Masks

Each camera model requires its own lidar mask — a black/white PNG where white = use, black = ignore. Masks must match the scanner body shape for the specific camera.

| File | Used for |
|------|----------|
| `config/masks/lidar_mask_dual_sdk.png`        | One X2, dual fisheye, SDK stitch |
| `config/masks/lidar_mask_dual.png`            | One X2, dual fisheye, stream mode |
| `config/masks/lidar_mask_single.png`          | One X2, single fisheye |
| `config/masks/lidar_mask_dual_x3_cam_left.png`| X3, dual fisheye |
| `config/masks/lidar_mask_dual_x5.png`         | X5, dual fisheye |
| `config/masks/lidar_mask_single_x5.png`       | X5, single fisheye |

To create a mask for a new camera/mount combination:
1. Capture one scan with the new camera to get a sample ERP image
2. Open the ERP in an image editor (GIMP, Photoshop)
3. Paint white over the scene, black over the scanner body and tripod
4. Save to the appropriate filename above (match ERP resolution)

---

## ERP–LiDAR Calibration

The scanner mounts **upside-down** — lidar faces up, camera faces down, sharing the forward axis. The SDK-stitched ERP requires its own calibration procedure using manual point correspondences.

### File locations

| File | Description |
|------|-------------|
| `~/atlas_ws/output/calib.json` | Raw `T_lidar_camera` output from `direct_visual_lidar_calibration` |
| `config/calibrations/<hw>/fusion_calibration.yaml` | Per-model calibration (written by `coordinate_transform.py`) |
| `config/fusion_calibration.yaml` | Active shared calibration (auto-updated at session start and by calibration scripts) |

### Prerequisites
- Insta360 SDK stitcher binary built at `~/insta360-dev/build/insta360_stitch`
- At least **5** scans from different positions

### Procedure

> *All steps below are also available in the ATLAS GUI under the **Calibration** tab, including a **▶ Run Full Calibration Pipeline** button that runs the entire sequence automatically. The GUI exposes the physical seed fields (Fwd/Left/Up/Yaw), the interactive viewer, and individual step buttons for combine scans, generate intensity images, seed, run calibration, apply, verify, and fine-tune sweeps.*

1. **Capture calibration scans** (5–10 from different positions):
    ```bash
    cd ~/atlas_ws/src/atlas-scanner/src
    ./atlas_fusion_capture.sh --capture stationary --camera dual_fisheye --camera-hw x5
    ```
    > *Replace `x5` with `x3` or `onex2` for other models.*

2. **Create a lidar mask** for your camera/mount (see [Lidar Masks](#lidar-masks) above).

3. **Combine scans** into the calibration dataset:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/combine_scans_for_calibration.py \
        ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} 2
    ```

4. **Generate lidar intensity images**:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/generate_intensity_images.py \
        ~/atlas_ws/output
    ```

5. **Seed the initial camera position** from physical measurements (stand behind the scanner, MID360 cable facing you — +X=forward, +Y=left, +Z=up):
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/physical_seed.py \
        --camera-hw x5 --forward 3.0 --left 0.0 --up 0.0 --yaw 0
    ```
    > *Measure the camera's position relative to the LiDAR center in inches. `--yaw` is rotation in degrees (0=faces forward, +90=faces left, -90=faces right).*

6. **Visually verify and coarsely adjust the seed** using the interactive GUI:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/interactive_seed.py \
        ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} --camera-hw x5
    ```
    Adjust the `Fwd`, `Left`, `Up` (inches) and `Yaw` sliders until LiDAR points land on the correct surfaces. Press `V` to toggle edge overlay. Press `S` to save when satisfied.

    > *This step is also available in the ATLAS GUI under the **Calibration** tab — select the session, adjust the sliders, and click **Save Seed**.*

    To verify without the GUI:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/verify_seed_overlay.py \
        ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} --open
    ```
    > *Opens `seed_composite.jpg` — left side shows edge alignment (red=camera, green=lidar, yellow=aligned), right side shows TURBO-colored lidar dots on the camera image.*

7. **Seed `calib.json`** with the verified position as the initial guess for the optimizer:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/seed_calib.py
    ```

8. **Run calibration**:
    ```bash
    cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration
    ./calibrate --data_path ~/atlas_ws/output
    ```
    > *Check the overlay with the `blend` slider. When satisfied, save the result.*

9. **Apply calibration** — saves to both `config/calibrations/<hw>/` and the shared `config/fusion_calibration.yaml`:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/coordinate_transform.py \
        ~/atlas_ws/src/atlas-scanner/src \
        --camera-hw x5
    ```
    > *Replace `x5` with `x3` or `onex2` for other models.*

    > *Alternatively use `extract_calibration.py` which handles the R_align undo step automatically:*
    > ```bash
    > python3 ~/atlas_ws/src/atlas-scanner/src/calibration/extract_calibration.py \
    >     --camera-hw x5
    > ```

10. **Verify alignment**:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/tune_calibration.py --verify
    eog ~/atlas_ws/output/calib_sweep/current.jpg
    ```
    > *Points should land on the correct surfaces. Near objects (blue) offset = translation error. All objects shift same amount = rotation error.*

    Fine-tune if needed:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/tune_calibration.py \
        --axis pitch --range 2.0 --steps 5
    # Then apply the best offset:
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/tune_calibration.py \
        --axis pitch --apply <degrees>
    ```
    > *After `--apply`, re-run `--verify` to confirm. The updated values are written to both the active `config/fusion_calibration.yaml` and `config/calibrations/<hw>/fusion_calibration.yaml`.*

11. **Calibration file reference** (`fusion_calibration.yaml`):

    ```yaml
    roll_offset: -3.119514987103984    # T_camera_lidar rotation (euler XYZ rad)
    pitch_offset: 0.00905554991564661
    yaw_offset: -1.5803468816018391
    manual_roll_adjustment: 0.0        # Fine-tune without re-running full calibration
    manual_pitch_adjustment: 0.0
    manual_yaw_adjustment: 0.0
    azimuth_offset: 0.0
    elevation_offset: 0.0
    x_offset: 0.0015582605849303532    # T_camera_lidar translation (metres)
    y_offset: 0.14399450677839698
    z_offset: -0.05149098901271928
    flip_x: false
    flip_y: false
    image_width: 5760                  # ERP resolution — set per camera model:
    image_height: 2880                 #   onex2: 5760×2880  x3: 7680×3840  x5: 11520×5760
    use_fisheye: false
    skip_rate: 5
    ```

12. Ready to scan — see [software-run.md](software-run.md).

> ***Note: If point cloud colors appear misaligned, use `tune_calibration.py --verify` to diagnose and `--axis`/`--apply` to correct, or rerun the full procedure above.***

---

## Multi-Camera Configuration

When using more than one camera simultaneously, the system reads `config/multi_camera.yaml` to assign each physical device to a logical slot (`cam_0`, `cam_1`, etc.) and apply shared exposure settings.

### Finding Camera Serial Numbers

Serial numbers are required for deterministic slot assignment — without them, cameras can swap indices between sessions depending on USB enumeration order.

With all cameras connected, run:

```bash
~/insta360-dev/build/insta360_capture_multi --discover
```

Example output:
```
Found 2 camera(s):
  [0] serial=IAHEA26019RESN  type=4  name=Insta360 X5
  [1] serial=IAHEA12345ABCD  type=3  name=Insta360 X3
```

### Authoring the Config File

Edit `~/atlas_ws/src/atlas-scanner/src/config/multi_camera.yaml`:

```yaml
# Shared settings applied to ALL cameras (overrides per-model defaults)
settings:
  exposure_mode: 0   # 0=AUTO  2=SHUTTER_PRIORITY  4=ADAPTIVE
  white_balance: 0   # 0=AUTO  1=2700K  2=4000K  3=5000K  4=6500K  5=7500K
  ev_bias: 0

cameras:
  cam_0:                          # Primary/reference camera — never color-corrected
    serial: "IAHEA26019RESN"      # From --discover output above
    camera_hw: x5                 # x5 | x3 | onex2
    mask_dual: lidar_mask_dual_x5_cam2.png          # Mask for dual_fisheye mode
    mask_calibration: lidar_mask_dual_x5_calib.png  # Mask used during calibration
    calibration: calibrations/x5/center/fusion_calibration.yaml
  cam_1:                          # Secondary camera — color-matched to cam_0
    serial: "IAHEA12345ABCD"
    camera_hw: x3
    mask_dual: lidar_mask_dual_x3_cam_left.png
    mask_calibration: lidar_mask_dual_x3_calib.png
    calibration: calibrations/x3/left/fusion_calibration.yaml
```

**Field reference:**

| Field | Required | Description |
|-------|----------|-------------|
| `serial` | Yes | Camera serial from `--discover`. Ensures correct slot assignment regardless of USB order. |
| `camera_hw` | Yes | Hardware model — selects ERP resolution and default mask. |
| `mask_dual` | Yes | Lidar mask for `dual_fisheye` mode (in `config/masks/`). |
| `mask_calibration` | No | Mask used during single-camera calibration captures. Falls back to `mask_dual`. |
| `mask_single` | No | Mask for `single_fisheye` mode. Falls back to `mask_dual`. |
| `calibration` | No | Per-camera extrinsic calibration path (relative to `config/`). Falls back to `calibrations/<camera_hw>/fusion_calibration.yaml`. |

**Notes:**
- `cam_0` is the primary reference camera. Secondary cameras (`cam_1`, `cam_2`) have color profiles applied to match it.
- Each camera needs its own calibration run (see [ERP–LiDAR Calibration](#erp-lidar-calibration)) with only that camera connected, using its `mask_calibration` mask.
- The `settings` block overrides per-model defaults from `config/camera_models/*.yaml` to ensure consistent exposure across mixed hardware.

---


At session start the capture script automatically loads the calibration for the selected `CAMERA_HW`:

```bash
# One X2 session
./atlas_fusion_capture.sh --camera dual_fisheye --camera-hw onex2

# X3 session
./atlas_fusion_capture.sh --camera dual_fisheye --camera-hw x3

# X5 session
./atlas_fusion_capture.sh --camera dual_fisheye --camera-hw x5
```

In the GUI, select the model from the **Camera HW** dropdown before clicking **Start System**.

Each model's calibration is stored independently:
```
config/calibrations/
  onex2/fusion_calibration.yaml   ← One X2 extrinsics
  x3/fusion_calibration.yaml      ← X3 extrinsics
  x5/fusion_calibration.yaml      ← X5 extrinsics
config/camera_models/
  onex2.yaml                      ← ERP resolution, mask filenames
  x3.yaml                         ← ERP resolution, mask filenames
  x5.yaml                         ← ERP resolution, mask filenames
```

To add a new mount variant (same camera, different bracket position), create a new subdirectory:
```bash
mkdir -p ~/atlas_ws/src/atlas-scanner/src/config/calibrations/x5_mount_b
cp ~/atlas_ws/src/atlas-scanner/src/config/calibrations/x5/fusion_calibration.yaml \
   ~/atlas_ws/src/atlas-scanner/src/config/calibrations/x5_mount_b/
# Then add x5_mount_b.yaml to config/camera_models/ and run calibration
```

---

## ERP Alignment for New Camera Models

See [SDK_STITCH_ERP_ALIGNMENT.md](../src/calibration/SDK_STITCH_ERP_ALIGNMENT.md) for how to adapt the lidar intensity projection when adding a camera with a different ERP axis convention.
