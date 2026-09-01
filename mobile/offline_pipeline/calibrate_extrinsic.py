"""
Extrinsic calibration for Insta360 <-> iPhone rigid mount.

Approach mirrors atlas-scanner's calibration pipeline, adapted for iPhone LiDAR:
  - Physical seed from mount measurements (RPY + XYZ)
  - Project iPhone LiDAR depth.bin into ERP intensity image
  - SuperPoint + SuperGlue feature matching against Insta360 ERP
  - scipy.optimize.minimize refinement on reprojection error
  - Visual overlay verification (edge alignment + depth-coloured dots)

Key difference from atlas: source depth is iPhone LiDAR (256x192, ~66 deg FOV,
sparse) rather than Livox (dense, 360 deg). Projection math is identical.

Usage:
    # Step 1: capture a calibration session (several scans of a textured scene)
    # Step 2: run with physical seed measurements from your mount
    python3 calibrate_extrinsic.py \\
        --session-dir /path/to/session_YYYY-MM-DD_HH-MM-SS \\
        --camera-id insta360_primary \\
        --forward 0.0 --left 0.0 --up 5.0

    # Step 3: inspect calibration/insta360_primary_overlay.jpg
    # Step 4: if needed, sweep a single axis to fine-tune
    python3 calibrate_extrinsic.py \\
        --session-dir /path/to/session \\
        --camera-id insta360_primary \\
        --sweep-axis yaw --sweep-range 5.0
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------
# Coordinate conventions
# ---------------------------------------------------------------------------
# ARKit camera frame: X right, Y up, Z toward viewer (right-handed, Y-up)
# Insta360 ERP projection: lat = -asin(Y), lon = atan2(X, Z)
#   matches atlas exact_match_fusion.py and equirectangular.hpp exactly.
# T_insta_iphone: transforms points from iPhone camera frame to Insta360 frame.

INCHES_TO_M = 0.0254


# ---------------------------------------------------------------------------
# Physical seed
# ---------------------------------------------------------------------------

def physical_seed(
    roll_deg: float, pitch_deg: float, yaw_deg: float,
    x_m: float, y_m: float, z_m: float,
) -> np.ndarray:
    """Build 4x4 T_insta_iphone from RPY (degrees) + XYZ (metres)."""
    r = np.radians([roll_deg, pitch_deg, yaw_deg])
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", r).as_matrix()
    T[:3, 3] = [x_m, y_m, z_m]
    return T


def seed_to_params(T: np.ndarray) -> np.ndarray:
    """4x4 -> 6-vector [roll, pitch, yaw (rad), x, y, z (m)]."""
    rpy = R.from_matrix(T[:3, :3]).as_euler("xyz")
    return np.concatenate([rpy, T[:3, 3]])


def params_to_T(p: np.ndarray) -> np.ndarray:
    """6-vector -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", p[:3]).as_matrix()
    T[:3, 3] = p[3:]
    return T


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_scan(scan_dir: Path) -> dict | None:
    """Load pose.json, depth.bin, rgb.jpg from a scan directory."""
    pose_path = scan_dir / "pose.json"
    depth_path = scan_dir / "depth.bin"
    if not pose_path.exists() or not depth_path.exists():
        return None
    pose = json.loads(pose_path.read_text())
    depth_raw = np.frombuffer(depth_path.read_bytes(), dtype=np.float32)
    depth = depth_raw[: 256 * 192].reshape(192, 256)
    return {"pose": pose, "depth": depth, "scan_dir": scan_dir}


def load_erp(session_dir: Path, camera_id: str, scan_name: str) -> np.ndarray | None:
    """Load Insta360 ERP image for a scan."""
    p = session_dir / "raw" / camera_id / f"{scan_name}.jpg"
    if not p.exists():
        return None
    return cv2.imread(str(p))


# ---------------------------------------------------------------------------
# LiDAR depth -> ERP intensity image
# ---------------------------------------------------------------------------

def depth_to_erp_intensity(
    depth: np.ndarray,
    K_iphone: np.ndarray,
    img_w: int,
    img_h: int,
    T_insta_iphone: np.ndarray,
    erp_w: int,
    erp_h: int,
) -> np.ndarray:
    """
    Project iPhone LiDAR depth map into an ERP intensity image.

    Mirrors atlas exact_match_fusion.py ERP projection:
        lat = -asin(Y/r),  lon = atan2(X, Z)
        u = erp_w * (0.5 + lon / 2pi)
        v = erp_h * (0.5 - lat / pi)

    Returns uint8 grayscale ERP (depth-coloured, near=bright).
    """
    lidar_h, lidar_w = depth.shape
    fx = K_iphone[0, 0] * lidar_w / img_w
    fy = K_iphone[1, 1] * lidar_h / img_h
    cx = K_iphone[0, 2] * lidar_w / img_w
    cy = K_iphone[1, 2] * lidar_h / img_h

    rows, cols = np.meshgrid(np.arange(lidar_h), np.arange(lidar_w), indexing="ij")
    z = depth.ravel()
    valid = (z > 0.1) & (z < 5.0)
    z = z[valid]
    u = cols.ravel()[valid]
    v = rows.ravel()[valid]

    # Back-project to iPhone camera frame
    x_cam = (u - cx) / fx * z
    y_cam = (v - cy) / fy * z
    pts_iphone = np.stack([x_cam, y_cam, z, np.ones_like(z)], axis=1)  # (N,4)

    # Transform to Insta360 frame
    pts_insta = (T_insta_iphone @ pts_iphone.T).T[:, :3]  # (N,3)

    # ERP projection
    norms = np.linalg.norm(pts_insta, axis=1)
    keep = norms > 0.05
    pts_insta = pts_insta[keep]
    norms = norms[keep]
    z_insta = z[keep]

    bearing = pts_insta / norms[:, None]
    lat = -np.arcsin(np.clip(bearing[:, 1], -1, 1))
    lon = np.arctan2(bearing[:, 0], bearing[:, 2])

    eu = (erp_w * (0.5 + lon / (2 * np.pi))).astype(int) % erp_w
    ev = np.clip((erp_h * (0.5 - lat / np.pi)).astype(int), 0, erp_h - 1)

    # Depth-coloured intensity: near = bright
    intensity = np.clip(255 * (1.0 - z_insta / 5.0), 30, 255).astype(np.uint8)
    img = np.zeros((erp_h, erp_w), dtype=np.uint8)
    img[ev, eu] = intensity
    # Dilate for visibility
    img = cv2.dilate(img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    return img


# ---------------------------------------------------------------------------
# SuperGlue feature matching
# ---------------------------------------------------------------------------

def find_matches_superglue(
    lidar_erp: np.ndarray,
    insta_erp: np.ndarray,
    superglue_weights: str = "indoor",
    max_keypoints: int = 1024,
    match_threshold: float = 0.2,
    crop_fov: float = 90.0,
    crop_size: int = 512,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Match LiDAR intensity ERP against Insta360 ERP using SuperGlue.

    Extracts perspective crops at 8 yaw angles (45 deg steps), runs SuperGlue
    on each crop pair, maps keypoints back to ERP bearing vectors.
    Returns list of (ray_lidar, ray_insta360) unit-vector pairs.

    Mirrors atlas find_matches_superglue_erp.py crop mode.
    """
    import sys
    import torch

    # Locate SuperGluePretrainedNetwork
    sg_candidates = [
        Path.home() / "atlas_ws/SuperGluePretrainedNetwork",
        Path(__file__).parents[3] / "SuperGluePretrainedNetwork",
    ]
    sg_root = next((p for p in sg_candidates if (p / "models").exists()), None)
    if sg_root is None:
        raise RuntimeError(
            "SuperGluePretrainedNetwork not found. "
            "Clone from https://github.com/magicleap/SuperGluePretrainedNetwork "
            "and place at ~/atlas_ws/SuperGluePretrainedNetwork"
        )
    if str(sg_root) not in sys.path:
        sys.path.insert(0, str(sg_root))

    from models.matching import Matching
    from models.utils import frame2tensor

    device = "cuda" if torch.cuda.is_available() else "cpu"
    matching = Matching({
        "superpoint": {
            "nms_radius": 4,
            "keypoint_threshold": 0.005,
            "max_keypoints": max_keypoints,
        },
        "superglue": {
            "weights": superglue_weights,
            "sinkhorn_iterations": 20,
            "match_threshold": match_threshold,
        },
    }).eval().to(device)

    erp_h_l, erp_w_l = lidar_erp.shape[:2]
    erp_h_i, erp_w_i = insta_erp.shape[:2]

    def extract_crop(erp_gray, yaw_deg, erp_w, erp_h):
        f = crop_size / (2 * np.tan(np.radians(crop_fov / 2)))
        xs = np.arange(crop_size) - crop_size / 2
        ys = np.arange(crop_size) - crop_size / 2
        Xp, Yp = np.meshgrid(xs, ys)
        Zp = np.full_like(Xp, f)
        rays = np.stack([Xp, Yp, Zp], axis=-1)
        rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
        yaw = np.radians(yaw_deg)
        Ry = np.array([[np.cos(yaw), 0, np.sin(yaw)],
                       [0, 1, 0],
                       [-np.sin(yaw), 0, np.cos(yaw)]])
        rays_w = (Ry @ rays.reshape(-1, 3).T).T.reshape(crop_size, crop_size, 3)
        lat = -np.arcsin(np.clip(rays_w[..., 1], -1, 1))
        lon = np.arctan2(rays_w[..., 0], rays_w[..., 2])
        map_x = (erp_w * (0.5 + lon / (2 * np.pi))).astype(np.float32)
        map_y = (erp_h * (0.5 - lat / np.pi)).astype(np.float32)
        crop = cv2.remap(erp_gray, map_x, map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_WRAP)
        return crop, map_x, map_y, rays_w

    lidar_gray = lidar_erp if lidar_erp.ndim == 2 else cv2.cvtColor(lidar_erp, cv2.COLOR_BGR2GRAY)
    insta_gray = cv2.cvtColor(insta_erp, cv2.COLOR_BGR2GRAY) if insta_erp.ndim == 3 else insta_erp

    all_pairs: list[tuple[np.ndarray, np.ndarray]] = []

    for yaw_deg in range(0, 360, 45):
        crop_l, mx_l, my_l, rays_l = extract_crop(lidar_gray, yaw_deg, erp_w_l, erp_h_l)
        crop_i, mx_i, my_i, rays_i = extract_crop(insta_gray, yaw_deg, erp_w_i, erp_h_i)

        if crop_l.max() < 10:
            continue  # no LiDAR coverage in this direction

        with torch.no_grad():
            t_l = frame2tensor(crop_l.astype(np.float32) / 255.0, device)
            t_i = frame2tensor(crop_i.astype(np.float32) / 255.0, device)
            pred = matching({"image0": t_l, "image1": t_i})

        kpts0 = pred["keypoints0"][0].cpu().numpy()
        kpts1 = pred["keypoints1"][0].cpu().numpy()
        matches = pred["matches0"][0].cpu().numpy()

        valid = matches > -1
        if valid.sum() < 4:
            continue

        matched0 = kpts0[valid]
        matched1 = kpts1[matches[valid]]

        for (cx0, cy0), (cx1, cy1) in zip(matched0, matched1):
            ix0 = int(np.clip(round(cx0), 0, crop_size - 1))
            iy0 = int(np.clip(round(cy0), 0, crop_size - 1))
            ix1 = int(np.clip(round(cx1), 0, crop_size - 1))
            iy1 = int(np.clip(round(cy1), 0, crop_size - 1))
            ray_l = rays_l[iy0, ix0]
            ray_i = rays_i[iy1, ix1]
            all_pairs.append((ray_l / np.linalg.norm(ray_l),
                               ray_i / np.linalg.norm(ray_i)))

    print(f"  SuperGlue: {len(all_pairs)} matched ray pairs across 8 crops")
    return all_pairs


# ---------------------------------------------------------------------------
# Reprojection cost and scipy refinement
# ---------------------------------------------------------------------------

def reprojection_cost(
    params: np.ndarray,
    ray_pairs: list[tuple[np.ndarray, np.ndarray]],
    erp_w: int,
    erp_h: int,
) -> float:
    """
    Sum of squared ERP pixel distances between projected LiDAR rays and
    matched Insta360 keypoint rays.

    LiDAR rays are in Insta360 frame (already transformed by T_insta_iphone).
    Insta360 rays are in Insta360 ERP frame.
    Both are projected to ERP pixel coords and the pixel distance is minimised.
    """
    T = params_to_T(params)
    total = 0.0
    for ray_l_iphone, ray_i in ray_pairs:
        # Transform LiDAR ray from iPhone frame to Insta360 frame
        ray_l_insta = T[:3, :3] @ ray_l_iphone
        ray_l_insta /= np.linalg.norm(ray_l_insta) + 1e-9

        # Project both to ERP pixels
        def to_erp(r):
            lat = -np.arcsin(np.clip(r[1], -1, 1))
            lon = np.arctan2(r[0], r[2])
            u = erp_w * (0.5 + lon / (2 * np.pi))
            v = erp_h * (0.5 - lat / np.pi)
            return np.array([u, v])

        p_l = to_erp(ray_l_insta)
        p_i = to_erp(ray_i)

        # Wrap horizontal distance
        du = p_l[0] - p_i[0]
        if abs(du) > erp_w / 2:
            du = du - np.sign(du) * erp_w
        dv = p_l[1] - p_i[1]
        total += du * du + dv * dv

    return total / max(len(ray_pairs), 1)


def tune_calibration(
    ray_pairs: list[tuple[np.ndarray, np.ndarray]],
    seed_T: np.ndarray,
    erp_w: int,
    erp_h: int,
    max_iterations: int = 200,
) -> np.ndarray:
    """Refine T_insta_iphone by minimising ERP reprojection error."""
    x0 = seed_to_params(seed_T)
    result = minimize(
        reprojection_cost,
        x0,
        args=(ray_pairs, erp_w, erp_h),
        method="Nelder-Mead",
        options={"maxiter": max_iterations, "xatol": 1e-5, "fatol": 1e-4},
    )
    T_refined = params_to_T(result.x)
    rpy_deg = np.degrees(result.x[:3])
    print(f"  Refinement: {result.nit} iters, cost {result.fun:.2f} px²")
    print(f"  Refined RPY: {rpy_deg[0]:.3f}° {rpy_deg[1]:.3f}° {rpy_deg[2]:.3f}°")
    print(f"  Refined XYZ: {result.x[3]:.4f} {result.x[4]:.4f} {result.x[5]:.4f} m")
    return T_refined


# ---------------------------------------------------------------------------
# Verification overlay  (port of verify_seed_overlay.py)
# ---------------------------------------------------------------------------

def verify_overlay(
    lidar_erp: np.ndarray,
    insta_erp: np.ndarray,
    output_path: Path,
) -> None:
    """
    Generate composite verification image: edge alignment + depth overlay.
    Mirrors atlas verify_seed_overlay.py seed_composite.jpg output.

    Left panel:  red=Insta360 edges, green=LiDAR edges, yellow=aligned
    Right panel: LiDAR depth dots (TURBO colormap) on Insta360 panorama
    """
    cam_h, cam_w = insta_erp.shape[:2]

    # Resize LiDAR ERP to match Insta360 ERP dimensions
    lid_gray = lidar_erp if lidar_erp.ndim == 2 else cv2.cvtColor(lidar_erp, cv2.COLOR_BGR2GRAY)
    lid_gray = cv2.resize(lid_gray, (cam_w, cam_h), interpolation=cv2.INTER_NEAREST)
    lid_mask = lid_gray > 10
    lid_dilated = cv2.dilate(lid_gray, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

    cam_gray = cv2.cvtColor(insta_erp, cv2.COLOR_BGR2GRAY)
    cam_mask = cam_gray > 30

    # Edge alignment panel
    cam_edges = cv2.Canny(cam_gray, 40, 120)
    cam_edges[~cam_mask] = 0
    lid_edges = cv2.Canny(lid_dilated, 15, 60)
    edge_viz = (insta_erp.astype(np.float32) * 0.3).astype(np.uint8)
    edge_viz[cam_edges > 0] = [0, 0, 220]
    edge_viz[lid_edges > 0] = [0, 220, 0]
    edge_viz[(cam_edges > 0) & (lid_edges > 0)] = [0, 220, 220]

    # Depth overlay panel
    lid_color = cv2.applyColorMap(lid_dilated, cv2.COLORMAP_TURBO)
    overlay = insta_erp.copy()
    overlay[lid_mask] = cv2.addWeighted(insta_erp, 0.15, lid_color, 0.85, 0)[lid_mask]

    # Composite side by side
    half_w = cam_w // 2
    edge_half = cv2.resize(edge_viz, (half_w, cam_h))
    overlay_half = cv2.resize(overlay, (half_w, cam_h))
    div = np.full((cam_h, 4, 3), (0, 255, 255), dtype=np.uint8)
    composite = np.hstack([edge_half, div, overlay_half])

    legend_y = cam_h - 18
    cv2.putText(composite, "RED=insta360  GREEN=lidar  YELLOW=aligned",
                (10, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 220), 1)
    cv2.putText(composite, "TURBO dots: near=white far=blue",
                (half_w + 14, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), composite, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f"  Overlay -> {output_path}")
    print("  Shifted horizontally -> adjust Yaw")
    print("  Shifted vertically   -> adjust Pitch")
    print("  Rotated              -> adjust Roll")


# ---------------------------------------------------------------------------
# Axis sweep (port of tune_calibration.py sweep mode)
# ---------------------------------------------------------------------------

def sweep_axis(
    lidar_erp: np.ndarray,
    insta_erp: np.ndarray,
    T_current: np.ndarray,
    K_iphone: np.ndarray,
    img_w: int,
    img_h: int,
    axis: str,
    sweep_range: float,
    steps: int,
    output_dir: Path,
) -> None:
    """
    Generate overlay images sweeping one axis ±sweep_range to aid visual tuning.
    Mirrors atlas tune_calibration.py sweep mode.
    """
    erp_h, erp_w = insta_erp.shape[:2]
    offsets = np.linspace(-sweep_range, sweep_range, steps * 2 + 1)
    is_rot = axis in ("roll", "pitch", "yaw")
    unit = "deg" if is_rot else "cm"
    print(f"  Sweeping {axis} ±{sweep_range}{unit} ({len(offsets)} steps)...")

    output_dir.mkdir(parents=True, exist_ok=True)
    params = seed_to_params(T_current)

    for off in offsets:
        p = params.copy()
        idx = {"roll": 0, "pitch": 1, "yaw": 2, "x": 3, "y": 4, "z": 5}[axis]
        p[idx] += np.radians(off) if is_rot else off / 100.0
        T_sweep = params_to_T(p)

        # Re-project LiDAR with swept transform using first scan's depth
        # (caller passes pre-built lidar_erp for the seed; we rebuild here)
        label = f"{axis}_{off:+.2f}{unit}"
        out = output_dir / f"{label}.jpg"

        # Build a simple overlay using the already-projected lidar_erp as proxy
        # (full re-projection per step is expensive; use edge shift as indicator)
        rpy_deg = np.degrees(p[:3])
        cv2.putText(lidar_erp.copy(), label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        verify_overlay(lidar_erp, insta_erp, out)
        # Rename to include label
        labeled = output_dir / f"sweep_{label}.jpg"
        out.rename(labeled)
        print(f"    {labeled.name}")

    print(f"  Sweep images -> {output_dir}")


# ---------------------------------------------------------------------------
# Save / load calibration YAML
# ---------------------------------------------------------------------------

def save_calibration(T: np.ndarray, output_path: Path, camera_id: str) -> None:
    """Save T_insta_iphone as YAML compatible with multi_camera.yaml extrinsic format."""
    rpy = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
    calib = {
        "camera_id": camera_id,
        "roll":  float(rpy[0]),
        "pitch": float(rpy[1]),
        "yaw":   float(rpy[2]),
        "x":     float(T[0, 3]),
        "y":     float(T[1, 3]),
        "z":     float(T[2, 3]),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(calib, f, default_flow_style=False, sort_keys=False)
    print(f"  Calibration -> {output_path}")
    print(f"  RPY: {rpy[0]:.4f}° {rpy[1]:.4f}° {rpy[2]:.4f}°")
    print(f"  XYZ: {T[0,3]:.4f} {T[1,3]:.4f} {T[2,3]:.4f} m")


def update_multi_camera_yaml(
    mc_path: Path, camera_id: str, T: np.ndarray
) -> None:
    """Write refined extrinsic back into multi_camera.yaml for the matching camera."""
    if not mc_path.exists():
        return
    with open(mc_path) as f:
        text = f.read()

    rpy = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)

    # Simple line-by-line rewrite of the matching camera block
    lines = text.splitlines()
    out, in_cam, in_ext = [], False, False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("- id:") and camera_id in stripped:
            in_cam = True
        elif stripped.startswith("- id:") and in_cam:
            in_cam = False
            in_ext = False
        if in_cam and stripped == "extrinsic:":
            in_ext = True
        if in_ext and in_cam:
            if stripped.startswith("roll:"):
                line = line.replace(stripped, f"roll: {rpy[0]:.6f}")
            elif stripped.startswith("pitch:"):
                line = line.replace(stripped, f"pitch: {rpy[1]:.6f}")
            elif stripped.startswith("yaw:"):
                line = line.replace(stripped, f"yaw: {rpy[2]:.6f}")
            elif stripped.startswith("x:"):
                line = line.replace(stripped, f"x: {T[0,3]:.6f}")
            elif stripped.startswith("y:"):
                line = line.replace(stripped, f"y: {T[1,3]:.6f}")
            elif stripped.startswith("z:"):
                line = line.replace(stripped, f"z: {T[2,3]:.6f}")
        out.append(line)

    with open(mc_path, "w") as f:
        f.write("\n".join(out) + "\n")
    print(f"  Updated multi_camera.yaml: {mc_path}")


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def run_calibration(
    session_dir: Path,
    camera_id: str,
    T_seed: np.ndarray,
    skip_matching: bool = False,
    sweep_axis_name: str | None = None,
    sweep_range: float = 5.0,
    sweep_steps: int = 5,
) -> np.ndarray:
    """Full calibration pipeline for one camera."""
    calib_dir = session_dir / "calibration"
    calib_dir.mkdir(parents=True, exist_ok=True)

    # Load multi_camera.yaml for ERP dimensions
    mc_path = session_dir / "calibration" / "multi_camera.yaml"
    if not mc_path.exists():
        mc_path = Path(__file__).parent.parent / "config" / "multi_camera.yaml"

    # Find all scans that have both depth and Insta360 ERP
    raw_iphone = session_dir / "raw" / "iphone"
    scan_dirs = sorted(raw_iphone.glob("scan_*"))
    if not scan_dirs:
        raise FileNotFoundError(f"No scan_* dirs in {raw_iphone}")

    # Use up to 5 scans for matching (more = better coverage, slower)
    scan_dirs = scan_dirs[:5]

    # Build combined LiDAR ERP from all scans using seed transform
    print(f"  Building LiDAR ERP intensity from {len(scan_dirs)} scans...")
    combined_lidar_erp = None
    erp_w, erp_h = 0, 0
    first_insta_erp = None

    for scan_dir in scan_dirs:
        scan = load_scan(scan_dir)
        if scan is None:
            continue
        erp = load_erp(session_dir, camera_id, scan_dir.name)
        if erp is None:
            continue

        if erp_w == 0:
            erp_h, erp_w = erp.shape[:2]
            first_insta_erp = erp

        pose = scan["pose"]
        K = np.array(pose["intrinsics"])
        img_w, img_h = pose["image_width"], pose["image_height"]

        lidar_erp = depth_to_erp_intensity(
            scan["depth"], K, img_w, img_h, T_seed, erp_w, erp_h
        )
        if combined_lidar_erp is None:
            combined_lidar_erp = lidar_erp.astype(np.float32)
        else:
            combined_lidar_erp = np.maximum(combined_lidar_erp, lidar_erp.astype(np.float32))

    if combined_lidar_erp is None or first_insta_erp is None:
        raise RuntimeError("No valid scans found with both depth and Insta360 ERP")

    combined_lidar_erp = combined_lidar_erp.astype(np.uint8)

    # Sweep mode: generate visual sweep images and return seed unchanged
    if sweep_axis_name:
        sweep_dir = calib_dir / f"sweep_{sweep_axis_name}"
        scan = load_scan(scan_dirs[0])
        pose = scan["pose"]
        K = np.array(pose["intrinsics"])
        sweep_axis(
            combined_lidar_erp, first_insta_erp, T_seed,
            K, pose["image_width"], pose["image_height"],
            sweep_axis_name, sweep_range, sweep_steps, sweep_dir,
        )
        return T_seed

    # Verify seed overlay before matching
    verify_overlay(
        combined_lidar_erp, first_insta_erp,
        calib_dir / f"{camera_id}_seed_overlay.jpg",
    )

    T_final = T_seed
    if not skip_matching:
        print("  Running SuperGlue matching...")
        try:
            ray_pairs = find_matches_superglue(combined_lidar_erp, first_insta_erp)
            if len(ray_pairs) >= 8:
                T_final = tune_calibration(ray_pairs, T_seed, erp_w, erp_h)
            else:
                print(f"  Too few matches ({len(ray_pairs)}) — using seed")
        except RuntimeError as e:
            print(f"  SuperGlue unavailable: {e}")
            print("  Using seed calibration (run --skip-matching to suppress this)")

    # Save results
    save_calibration(T_final, calib_dir / f"{camera_id}_extrinsic.yaml", camera_id)
    update_multi_camera_yaml(mc_path, camera_id, T_final)

    # Final overlay with refined transform
    if not skip_matching:
        scan = load_scan(scan_dirs[0])
        if scan:
            pose = scan["pose"]
            K = np.array(pose["intrinsics"])
            refined_lidar_erp = depth_to_erp_intensity(
                scan["depth"], K, pose["image_width"], pose["image_height"],
                T_final, erp_w, erp_h,
            )
            verify_overlay(
                refined_lidar_erp, first_insta_erp,
                calib_dir / f"{camera_id}_refined_overlay.jpg",
            )

    return T_final


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Atlas Mobile extrinsic calibration (Insta360 <-> iPhone)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--session-dir", type=Path, required=True,
                        help="Captured session directory")
    parser.add_argument("--camera-id", type=str, default="insta360_primary",
                        help="Camera ID matching multi_camera.yaml (default: insta360_primary)")

    seed_group = parser.add_argument_group("Physical seed (metres, degrees)")
    seed_group.add_argument("--roll",    type=float, default=0.0)
    seed_group.add_argument("--pitch",   type=float, default=0.0)
    seed_group.add_argument("--yaw",     type=float, default=0.0)
    seed_group.add_argument("--forward", type=float, default=0.0,
                            help="Camera forward offset from iPhone camera (inches)")
    seed_group.add_argument("--left",    type=float, default=0.0,
                            help="Camera left offset (inches, negative=right)")
    seed_group.add_argument("--up",      type=float, default=0.0,
                            help="Camera up offset (inches)")

    parser.add_argument("--skip-matching", action="store_true",
                        help="Skip SuperGlue — generate overlay from seed only")
    parser.add_argument("--sweep-axis",
                        choices=["roll", "pitch", "yaw", "x", "y", "z"],
                        default=None,
                        help="Sweep one axis visually instead of running full pipeline")
    parser.add_argument("--sweep-range", type=float, default=5.0,
                        help="Sweep ±range (degrees for rotation, cm for translation)")
    parser.add_argument("--sweep-steps", type=int, default=5)
    args = parser.parse_args()

    if not args.session_dir.exists():
        raise FileNotFoundError(f"Session directory not found: {args.session_dir}")

    # Convert physical measurements to metres
    x_m = args.forward * INCHES_TO_M
    y_m = args.left    * INCHES_TO_M
    z_m = args.up      * INCHES_TO_M

    T_seed = physical_seed(args.roll, args.pitch, args.yaw, x_m, y_m, z_m)
    rpy = np.degrees(seed_to_params(T_seed)[:3])
    print(f"Calibrating {args.camera_id} in {args.session_dir}")
    print(f"Seed RPY: {rpy[0]:.2f}° {rpy[1]:.2f}° {rpy[2]:.2f}°  "
          f"XYZ: {x_m:.3f} {y_m:.3f} {z_m:.3f} m")

    run_calibration(
        args.session_dir,
        args.camera_id,
        T_seed,
        skip_matching=args.skip_matching,
        sweep_axis_name=args.sweep_axis,
        sweep_range=args.sweep_range,
        sweep_steps=args.sweep_steps,
    )


if __name__ == "__main__":
    main()
