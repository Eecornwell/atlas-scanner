# Calibration Procedure

> *Note: Ensure the camera is turned on and the software installation procedure was successful before performing calibration.*

## Camera Hardware Selection

ATLAS supports multiple Insta360 camera models. Each model has its own:
- ERP output resolution
- Lidar mask image
- Calibration file stored at `config/calibrations/<model>/fusion_calibration.yaml`

| Model | CLI flag | ERP Resolution | Config dir |
|-------|----------|----------------|------------|
| Insta360 One X2 | `--camera-hw onex2` | 5760 × 2880 | `config/calibrations/onex2/` |
| Insta360 X5     | `--camera-hw x5`    | 7680 × 3840 | `config/calibrations/x5/`   |

> *Note: All calibration scripts accept `--camera-hw onex2|x5`. Always pass the correct model for the hardware you are calibrating. Calibration results are saved to the per-model directory and also copied to the shared `config/fusion_calibration.yaml` which is the active calibration used at runtime.*

> *Note: When you start a capture session the script automatically loads `config/calibrations/<camera_hw>/fusion_calibration.yaml` based on the selected `CAMERA_HW` setting (or `--camera-hw` CLI flag / GUI dropdown). You do not need to manually copy files between captures — just set the correct model.*

---

## Lidar Masks

Each camera model requires its own lidar mask — a black/white PNG where white = use, black = ignore. Masks must match the scanner body shape for the specific camera.

| File | Used for |
|------|----------|
| `lidar_mask_dual_sdk.png` | One X2, dual fisheye, SDK stitch |
| `lidar_mask_dual.png` | One X2, dual fisheye, stream mode |
| `lidar_mask_single.png` | One X2, single fisheye |
| `lidar_mask_dual_x5.png` | X5, dual fisheye, SDK stitch *(replace placeholder)* |
| `lidar_mask_single_x5.png` | X5, single fisheye *(replace placeholder)* |

To create a mask for a new camera/mount combination:
1. Capture one scan with the new camera to get a sample ERP image
2. Open the ERP in an image editor (GIMP, Photoshop)
3. Paint white over the scene, black over the scanner body and tripod
4. Save to the appropriate filename above (match ERP resolution)

---

## Fisheye to Equirectangular Calibration

> *Note: Required for `single_fisheye` mode and for `dual_fisheye` stream mode (`USE_SDK_STITCH=false`). Skip for SDK stitch mode — the SDK handles stitching internally.*

Start the camera:
```bash
sudo ~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh
cd ~/atlas_ws && source install/setup.bash
ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true
```

Run the equirectangular calibration tool:
```bash
cd ~/atlas_ws && source install/setup.bash
export QT_QPA_PLATFORM=xcb
ros2 run insta360_ros_driver equirectangular.py --calibrate --auto-update \
    --ros-args --params-file ~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml
```

Starting parameters for `equirectangular.yaml`:
```yaml
equirectangular_node:
  ros__parameters:
    cx_offset: 0.0
    cy_offset: 0.0
    crop_size: 1920
    translation: [-0.007, 0.007, -0.153]
    rotation_deg: [0.7, -3.0, 1.5]
    gpu: false
    out_width: 3840    # One X2: 3840 — X5: 5120 (adjust to match sensor)
    out_height: 1920   # One X2: 1920 — X5: 2560
```

**Tips:**
- Adjust `TZ` first to align the image centre
- Then `Pitch` to centre left/right
- Rotate the camera on the tripod while calibrating to confirm alignment across orientations
- Press `s` to save when satisfied

---

## Equirectangular to Lidar Calibration

### File locations

| File | Description |
|------|-------------|
| `~/atlas_ws/output/calib.json` | Raw `T_lidar_camera` output from `direct_visual_lidar_calibration` |
| `config/calibrations/<hw>/fusion_calibration.yaml` | Per-model calibration (written by `coordinate_transform.py`) |
| `config/fusion_calibration.yaml` | Active shared calibration (auto-updated at session start and by calibration scripts) |

### Procedure

1. **Capture a scan** to generate a sample ERP image for mask creation:
    ```bash
    cd ~/atlas_ws/src/atlas-scanner/src
    ./atlas_fusion_capture.sh --capture stationary --camera dual_fisheye --camera-hw onex2
    ```
    > *Replace `onex2` with `x5` if calibrating the X5.*

2. **Create a lidar mask** for your camera/mount (see [Lidar Masks](#lidar-masks) above).

3. **Capture calibration scans** (5–10 from different positions):
    ```bash
    cd ~/atlas_ws/src/atlas-scanner/src
    ./atlas_fusion_capture.sh --capture stationary --camera dual_fisheye --camera-hw onex2
    ```

4. **Combine scans** into the calibration dataset:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/combine_scans_for_calibration.py \
        ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
    ```

5. **Get initial guess** (choose one):

    - *Manual (recommended)*:
        ```bash
        cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration
        ./initial_guess_manual --data_path ~/atlas_ws/output
        ```

    - *Auto*:
        ```bash
        python3 ~/atlas_ws/src/atlas-scanner/src/calibration/generate_intensity_images.py \
            ~/atlas_ws/output
        cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration
        python3 ./find_matches_superglue.py ~/atlas_ws/output --superglue indoor
        python3 ~/atlas_ws/src/atlas-scanner/src/calibration/fix_matches.py ~/atlas_ws/output
        ./initial_guess_auto ~/atlas_ws/output
        python3 -c "
        import json; f=open('$HOME/atlas_ws/output/calib.json','r+')
        d=json.load(f); d['results']['init_T_lidar_camera']=d['results']['init_T_lidar_camera_auto']
        f.seek(0); json.dump(d,f,indent=2); f.truncate()
        "
        ```

6. **Run calibration**:
    ```bash
    cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration
    ./calibrate --data_path ~/atlas_ws/output \
        --nid_bins 32 \
        --nelder_mead_convergence_criteria 1e-10
    ```
    > *Check the overlay with the `blend` slider. When satisfied, save the result.*

    > *To seed from the current known-good calibration instead of the auto guess:*
    > ```bash
    > python3 ~/atlas_ws/src/atlas-scanner/src/calibration/seed_calib.py
    > ```

7. **Apply the calibration** — saves to both `config/calibrations/<hw>/` and the shared `config/fusion_calibration.yaml`:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/coordinate_transform.py \
        ~/atlas_ws/src/atlas-scanner/src \
        --camera-hw onex2
    ```
    > *Replace `onex2` with `x5` for the X5.*

    > *Alternatively use `extract_calibration.py` which handles the R_align undo step automatically:*
    > ```bash
    > python3 ~/atlas_ws/src/atlas-scanner/src/calibration/extract_calibration.py \
    >     --camera-hw onex2
    > ```

8. **Verify alignment**:
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

9. **Calibration file reference** (`fusion_calibration.yaml`):

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
    image_width: 5760                  # ERP resolution (set from camera model profile)
    image_height: 2880
    use_fisheye: false
    skip_rate: 5
    ```

10. Ready to scan — see [software-run.md](software-run.md).

> ***Note: If point cloud colors appear misaligned, use `tune_calibration.py --verify` to diagnose and `--axis`/`--apply` to correct, or rerun the full procedure above.***

---

## SDK Stitch Calibration

> *Applies when `USE_SDK_STITCH=true` with `--camera dual_fisheye`. The SDK-stitched ERP is gravity-aligned and has a different camera frame orientation, requiring its own calibration.*

The scanner mounts **upside-down** — lidar faces up, camera faces down, sharing the forward axis. The SDK ERP is a full 360° gravity-aligned equirectangular image.

### Prerequisites
- Insta360 SDK stitcher binary built at `~/insta360-dev/build/insta360_stitch`
- At least **2** SDK stitch scans from different positions

### Procedure

1. **Capture scans**:
    ```bash
    cd ~/atlas_ws/src/atlas-scanner/src
    ./atlas_fusion_capture.sh --capture stationary --camera dual_fisheye \
        --sdk-stitch --camera-hw onex2
    ```

2. **Combine scans**:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/combine_scans_for_calibration.py \
        ~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP} 2
    ```

3. **Generate lidar intensity images**:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/generate_intensity_images.py \
        ~/atlas_ws/output
    ```
    > *SDK stitch scans are detected automatically via `.insp` files and use the corrected ERP projection.*

4. **Preview perspective crops** to find good matching views:
    ```bash
    python3 -c "
    import sys, numpy as np, cv2
    sys.path.insert(0, '/home/orion/atlas_ws/src/atlas-scanner/src/calibration')
    from find_matches_superglue_erp import extract_perspective_crop
    lid = cv2.imread('/home/orion/atlas_ws/output/000000_lidar_intensities.png', cv2.IMREAD_GRAYSCALE)
    cam = cv2.imread('/home/orion/atlas_ws/output/000000.png', cv2.IMREAD_GRAYSCALE)
    lid = np.roll(np.flipud(np.fliplr(lid)), lid.shape[1]//2, axis=1)
    if cam.shape != lid.shape: lid = cv2.resize(lid, (cam.shape[1], cam.shape[0]))
    for yaw in [0, 45, 90, 135, 180, 225, 270, 315]:
        cc, _, _ = extract_perspective_crop(cam, yaw, 0, 90, 800)
        lc, _, _ = extract_perspective_crop(lid, yaw, 0, 90, 800)
        cv2.imwrite(f'/tmp/preview_y{yaw:03d}.png',
            np.concatenate([cv2.cvtColor(cc, cv2.COLOR_GRAY2BGR),
                            cv2.applyColorMap(lc, cv2.COLORMAP_INFERNO)], axis=1))
    print('Saved previews to /tmp/preview_y*.png')
    "
    eog /tmp/preview_y*.png
    ```

5. **Pick manual matches** (aim for 6–8 per view across different depths):
    ```bash
    cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/pick_matches.py \
        ~/atlas_ws/output --yaw 90  --pair 000000
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/pick_matches.py \
        ~/atlas_ws/output --yaw 270 --pair 000000
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/pick_matches.py \
        ~/atlas_ws/output --yaw 90  --pair 000001
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/pick_matches.py \
        ~/atlas_ws/output --yaw 270 --pair 000001
    ```
    **Controls:** Right-click camera (left) → camera keypoint. Right-click lidar (right) → complete pair. `Backspace` = undo. `s`/`Enter` = save. `q`/`Esc` = quit.

6. **Verify matches**:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/verify_matches.py \
        ~/atlas_ws/output --pair 000000
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/verify_matches.py \
        ~/atlas_ws/output --pair 000001
    ```

7. **Estimate initial transform and run calibration**:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/fix_matches.py ~/atlas_ws/output
    cd ~/atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration
    ./initial_guess_auto --data_path ~/atlas_ws/output
    python3 -c "
    import json; f=open('$HOME/atlas_ws/output/calib.json','r+')
    d=json.load(f); d['results']['init_T_lidar_camera']=d['results']['init_T_lidar_camera_auto']
    f.seek(0); json.dump(d,f,indent=2); f.truncate()
    "
    ./calibrate --data_path ~/atlas_ws/output
    ```

8. **Apply calibration** — specify the correct camera hardware:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/coordinate_transform.py \
        ~/atlas_ws/src/atlas-scanner/src \
        --camera-hw onex2
    ```

9. **Verify**:
    ```bash
    python3 ~/atlas_ws/src/atlas-scanner/src/calibration/tune_calibration.py --verify
    eog ~/atlas_ws/output/calib_sweep/current.jpg
    ```

10. Ready to scan — see [software-run.md](software-run.md).

---

## Swapping Between Camera Models

At session start the capture script automatically loads the calibration for the selected `CAMERA_HW`:

```bash
# One X2 session
./atlas_fusion_capture.sh --camera dual_fisheye --camera-hw onex2

# X5 session
./atlas_fusion_capture.sh --camera dual_fisheye --camera-hw x5
```

In the GUI, select the model from the **Camera HW** dropdown before clicking **Start System**.

Each model's calibration is stored independently:
```
config/calibrations/
  onex2/fusion_calibration.yaml   ← One X2 extrinsics
  x5/fusion_calibration.yaml      ← X5 extrinsics
config/camera_models/
  onex2.yaml                      ← ERP resolution, mask filenames
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
