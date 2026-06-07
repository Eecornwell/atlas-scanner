# SDK Stitch ERP Alignment Guide

## Overview

The lidar intensity image must match the camera ERP layout for SuperGlue feature matching during calibration. Different cameras produce ERPs with different axis conventions. This document explains how the alignment works and how to adapt it for a new sensor.

## Lidar Coordinate Frame (ROS Sensor Frame)

The PLY point cloud uses standard ROS conventions:
- **+X** = forward (scanner front)
- **+Y** = left
- **+Z** = up (ceiling)

## Standard Spherical ERP (Lidar Default)

Without any transform, the lidar projects as:
```
lon = atan2(Y, X)       # azimuth in XY plane from +X
lat = asin(Z / r)       # elevation above XY plane
u = W * (0.5 - lon/(2π)) % W   # center = forward, left/right = sides
v = H * (0.5 - lat/π)          # top = ceiling, bottom = floor
```

## Insta360 SDK Stitch Convention

The Insta360 X3/X4 SDK stitch produces an ERP with a 90° pitch offset:

| Image Position | Scene Direction |
|----------------|-----------------|
| Top (v=0) | Forward (+X) |
| Bottom (v=H) | Backward (-X) |
| Center (u=W/2) | Floor (-Z) |
| Left edge (u=0) | Ceiling (+Z) |
| Right edge (u=W) | Ceiling (+Z, wraps) |
| Left quarter (u=W/4) | Physical left (+Y) |
| Right quarter (u=3W/4) | Physical right (-Y) |

## The Transform

To match the Insta360 layout, apply `R_y(-90°)` to the normalized lidar directions, then project with a standard ERP formula (sign-flipped v):

```python
# Normalize points to unit sphere
X, Y, Z = pts / norm  # unit directions

# R_y(-90°) rotation: remap axes
X_new = -Z
Y_new = Y
Z_new = X

# Project to ERP
lon2 = arctan2(Y_new, X_new)
lat2 = arcsin(Z_new)
u = W * (0.5 - lon2 / (2π))  % W
v = H * (0.5 - lat2 / π)
```

### Why This Works

The Insta360 ERP is effectively a standard ERP where the camera's optical axis (pole) points forward instead of up. The `R_y(-90°)` rotation converts from the lidar frame (Z=up pole) to the Insta360 frame (X=forward pole):

- Lidar +X (forward) → Z_new = +1 → lat2 = +90° → v = 0 (top) ✓
- Lidar -X (backward) → Z_new = -1 → lat2 = -90° → v = H (bottom) ✓
- Lidar +Z (ceiling) → X_new = -1 → lon2 = 180° → u = 0 (left edge) ✓
- Lidar -Z (floor) → X_new = +1 → lon2 = 0° → u = W/2 (center) ✓

## Adapting for a New Camera

### Step 1: Identify the Camera ERP Layout

Generate the raw standard lidar intensity (use identity projection — no rotation) and compare side-by-side with the camera ERP. Identify 3–4 landmarks:

1. **What direction is at the image center** (u=W/2, v=H/2)?
2. **What direction is at the top** (v=0)?
3. **What direction is at the left edge** (u=0)?
4. **Is the image mirrored** (do labels read backwards)?

### Step 2: Determine the Required Rotation

The transform is always a 3×3 rotation applied to the unit-sphere directions `(X, Y, Z)` before the standard ERP projection. Common cases:

| Camera Convention | Rotation | Axis Remap |
|-------------------|----------|------------|
| Standard (up=top, fwd=center) | Identity | X,Y,Z unchanged |
| Insta360 (fwd=top, ceiling=edges) | R_y(-90°) | X_new=-Z, Y_new=Y, Z_new=X |
| Nadir-down (fwd=top, floor=center) | R_y(+90°) | X_new=Z, Y_new=Y, Z_new=-X |
| Back-centered (back=center) | R_y(180°) | X_new=-X, Y_new=Y, Z_new=-Z |

If the image is also **left-right mirrored**, negate the `lon` term in the `u` formula (change `0.5 - lon/(2π)` to `0.5 + lon/(2π)`).

If the image is **vertically flipped**, change `0.5 - lat/π` to `0.5 + lat/π`.

### Step 3: Edit the Code

In `generate_intensity_images.py`, find the `if is_sdk_stitch:` block. Replace the axis remap lines:

```python
# Current Insta360 mapping:
X_new = -Z
Y_new = Y
Z_new = X
```

With the appropriate remap for your camera. Then adjust the `u` and `v` sign if needed.

### Step 4: Verify with Landmarks

Run the pipeline and confirm:
- A known object (speaker, door, etc.) appears at the same position in both images
- Text/labels read in the correct direction (no mirror)
- Ceiling/floor are at the correct image locations
- Object orientation matches (top of objects points same direction in both)

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Content left-right mirrored | Flip sign in u formula: `0.5 + lon/(2π)` ↔ `0.5 - lon/(2π)` |
| Content upside-down | Flip sign in v formula: `0.5 + lat/π` ↔ `0.5 - lat/π` |
| Content rotated 90° | Wrong rotation matrix — swap which axes map to X_new/Y_new/Z_new |
| Content rotated 180° | Both u and v signs are wrong, or rotation is off by 180° in yaw |
| Correct position but wrong orientation of objects | Need per-half flip — try negating v for left and/or right halves independently |

## File Location

The SDK stitch projection is in:
```
src/atlas-scanner/src/calibration/generate_intensity_images.py
```

In the `generate_intensity_image()` function, under the `if is_sdk_stitch:` branch (approximately line 150–170).
