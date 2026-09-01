"""
COLMAP model assembler for Atlas Mobile capture sessions.

Reads the raw capture output from the iOS app and produces a COLMAP-compatible
sparse model matching the atlas-scanner desktop output format.

Pipeline:
  1. Load pose.json per scan (ARKit 4x4 transform + intrinsics)
  2. Slice Insta360 ERP images into 8 perspective face tiles (erp_tile_slicer)
  3. Unproject iPhone LiDAR depth.bin into world-frame point cloud
  4. Write cameras.bin, images.bin, points3D.bin (COLMAP binary format)
  5. Write depth_images/ (uint16 PNG, mm) for each face tile

Coordinate conventions:
  - ARKit: right-handed, Y-up, camera looks down -Z
  - COLMAP: right-handed, Y-down, camera looks down +Z
  - R_ARKIT2COLMAP = diag(1, -1, -1)  (flip Y and Z)

Usage:
    uv run python assemble_colmap.py --session-dir /path/to/session_YYYY-MM-DD_HH-MM-SS
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from erp_tile_slicer import (
    FACES_CAM_FROM_PANO,
    FOV_DEG,
    NUM_FACES,
    slice_erp,
    tile_intrinsics,
)

# ARKit Y-up → COLMAP Y-down (flip Y and Z axes).
# Same role as R_ROS2COLMAP in atlas-scanner.
R_ARKIT2COLMAP = np.diag([1.0, -1.0, -1.0])

# iPhone LiDAR sensor resolution
LIDAR_W, LIDAR_H = 256, 192

# Minimum baseline between scan positions to include in model (metres)
MIN_BASELINE_M = 0.10

# COLMAP camera model IDs
MODEL_SIMPLE_PINHOLE = 0  # f, cx, cy  — Insta360 face tiles
MODEL_PINHOLE = 1         # fx, fy, cx, cy — iPhone wide camera


# ---------------------------------------------------------------------------
# Pose loading
# ---------------------------------------------------------------------------

def load_pose(pose_path: Path) -> dict:
    with open(pose_path) as f:
        return json.load(f)


def arkit_to_colmap_pose(transform_matrix: list[list[float]]) -> tuple[np.ndarray, np.ndarray]:
    """
    Convert ARKit camera-to-world 4x4 to COLMAP world-to-camera (R_w2c, t_w2c).

    ARKit transform_matrix is T_world_camera (camera-to-world).
    COLMAP images.bin stores world-to-camera: R_w2c, t_w2c = -R_w2c @ C_world.
    """
    T_wc = np.array(transform_matrix, dtype=np.float64)  # 4x4 camera-to-world

    # Camera centre in ARKit world frame
    C_arkit = T_wc[:3, 3]
    R_c2w_arkit = T_wc[:3, :3]

    # Convert to COLMAP world frame
    C_colmap = R_ARKIT2COLMAP @ C_arkit
    R_c2w_colmap = R_ARKIT2COLMAP @ R_c2w_arkit

    R_w2c = R_c2w_colmap.T
    t_w2c = -R_w2c @ C_colmap
    return R_w2c, t_w2c, C_colmap


def rotation_to_quat_wxyz(R_mat: np.ndarray) -> np.ndarray:
    q = R.from_matrix(R_mat).as_quat()  # xyzw
    if q[3] < 0:
        q = -q
    return np.array([q[3], q[0], q[1], q[2]])  # wxyz


# ---------------------------------------------------------------------------
# LiDAR depth unprojection
# ---------------------------------------------------------------------------

def load_depth_bin(path: Path) -> np.ndarray:
    """Load Float32 depth map (metres) from raw binary, shape (192, 256)."""
    data = np.frombuffer(path.read_bytes(), dtype=np.float32)
    # depth.bin may have stride padding; take first LIDAR_H * LIDAR_W floats
    return data[: LIDAR_H * LIDAR_W].reshape(LIDAR_H, LIDAR_W)


def unproject_depth(
    depth_m: np.ndarray,
    intrinsics: list[list[float]],
    T_w2c_inv: np.ndarray,
    min_depth: float = 0.1,
    max_depth: float = 5.0,
) -> np.ndarray:
    """
    Unproject iPhone LiDAR depth map to world-frame 3D points.

    intrinsics is the 3x3 ARKit camera intrinsics matrix [[fx,0,cx],[0,fy,cy],[0,0,1]].
    T_w2c_inv is the camera-to-world 4x4 (ARKit T_world_camera).
    Returns (N, 3) array in COLMAP world frame.
    """
    K = np.array(intrinsics, dtype=np.float64)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # ARKit depth is at LiDAR resolution (256x192); intrinsics are for the
    # full RGB image. Scale intrinsics to LiDAR resolution.
    # pose.json stores image_width/image_height for the RGB frame.
    # We scale here assuming the caller passes already-scaled intrinsics,
    # or the caller pre-scales. See assemble_colmap() for the scaling.

    h, w = depth_m.shape
    v_idx, u_idx = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")

    valid = (depth_m > min_depth) & (depth_m < max_depth)
    z = depth_m[valid]
    u = u_idx[valid].astype(np.float64)
    v = v_idx[valid].astype(np.float64)

    # Back-project to camera frame
    x_cam = (u - cx) / fx * z
    y_cam = (v - cy) / fy * z
    pts_cam = np.stack([x_cam, y_cam, z], axis=-1)  # (N, 3) in ARKit camera frame

    # Camera frame → ARKit world frame
    pts_h = np.hstack([pts_cam, np.ones((len(pts_cam), 1))])
    pts_arkit = (T_w2c_inv @ pts_h.T).T[:, :3]

    # ARKit world → COLMAP world
    return (R_ARKIT2COLMAP @ pts_arkit.T).T


# ---------------------------------------------------------------------------
# Depth image rendering (port of generate_colmap_depth.py _render_depth)
# ---------------------------------------------------------------------------

def render_depth_tile(
    pts_colmap: np.ndarray,
    R_w2c: np.ndarray,
    t_w2c: np.ndarray,
    f_px: float,
    cx: float,
    cy: float,
    w: int,
    h: int,
    radius: int = 2,
) -> np.ndarray:
    """Project world-frame points into a tile camera and return uint16 depth (mm)."""
    pts_cam = (R_w2c @ pts_colmap.T).T + t_w2c
    valid = (pts_cam[:, 2] > 0.1) & (pts_cam[:, 2] < 20.0)
    pts_cam = pts_cam[valid]
    if len(pts_cam) == 0:
        return np.zeros((h, w), dtype=np.uint16)

    z = pts_cam[:, 2]
    ui = np.round(pts_cam[:, 0] / z * f_px + cx).astype(np.int32)
    vi = np.round(pts_cam[:, 1] / z * f_px + cy).astype(np.int32)
    in_bounds = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    ui, vi, z = ui[in_bounds], vi[in_bounds], z[in_bounds]

    depth_f = np.full((h, w), np.inf, dtype=np.float64)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            vj = np.clip(vi + dy, 0, h - 1)
            uj = np.clip(ui + dx, 0, w - 1)
            closer = z < depth_f[vj, uj]
            depth_f[vj[closer], uj[closer]] = z[closer]

    depth_mm = np.clip(depth_f * 1000.0, 0, 65535)
    depth_mm[depth_f == np.inf] = 0
    return depth_mm.astype(np.uint16)


# ---------------------------------------------------------------------------
# COLMAP binary writers (port of panorama_sfm_colmap.py write_init_model_bin)
# ---------------------------------------------------------------------------

def _pack_le(fmt: str, *args) -> bytes:
    return struct.pack("<" + fmt, *args)


def write_cameras_bin(
    path: Path,
    iphone_camera_id: int,
    iphone_w: int,
    iphone_h: int,
    iphone_fx: float,
    iphone_fy: float,
    iphone_cx: float,
    iphone_cy: float,
    tile_camera_ids: dict[int, int],  # face_idx -> camera_id
    tile_size: int,
    tile_fov: float = FOV_DEG,
) -> None:
    """Write cameras.bin: PINHOLE for iPhone, SIMPLE_PINHOLE for each face."""
    f_tile, c_tile, _ = tile_intrinsics(tile_size, tile_fov)
    total = 1 + len(tile_camera_ids)

    with open(path, "wb") as f:
        f.write(_pack_le("Q", total))

        # iPhone — PINHOLE (model 1): fx, fy, cx, cy
        f.write(_pack_le("I", iphone_camera_id))
        f.write(_pack_le("i", MODEL_PINHOLE))
        f.write(_pack_le("QQ", iphone_w, iphone_h))
        for v in (iphone_fx, iphone_fy, iphone_cx, iphone_cy):
            f.write(_pack_le("d", v))

        # Insta360 face tiles — SIMPLE_PINHOLE (model 0): f, cx, cy
        for face_idx, cam_id in sorted(tile_camera_ids.items()):
            f.write(_pack_le("I", cam_id))
            f.write(_pack_le("i", MODEL_SIMPLE_PINHOLE))
            f.write(_pack_le("QQ", tile_size, tile_size))
            for v in (f_tile, c_tile, c_tile):
                f.write(_pack_le("d", v))


def write_images_bin(path: Path, image_entries: list[dict]) -> None:
    """
    Write images.bin.

    Each entry: {image_id, quat_wxyz, tvec, camera_id, name}
    """
    with open(path, "wb") as f:
        f.write(_pack_le("Q", len(image_entries)))
        for e in image_entries:
            f.write(_pack_le("I", e["image_id"]))
            for v in e["quat_wxyz"]:
                f.write(_pack_le("d", v))
            for v in e["tvec"]:
                f.write(_pack_le("d", v))
            f.write(_pack_le("I", e["camera_id"]))
            f.write(e["name"].encode("utf-8") + b"\x00")
            f.write(_pack_le("Q", 0))  # no 2D points


def write_points3d_bin(path: Path, pts: np.ndarray) -> None:
    """Write points3D.bin from (N, 3) world-frame array. No track info."""
    with open(path, "wb") as f:
        f.write(_pack_le("Q", len(pts)))
        for i, p in enumerate(pts):
            f.write(_pack_le("Q", i + 1))
            f.write(_pack_le("ddd", *p))
            f.write(_pack_le("BBB", 128, 128, 128))  # grey placeholder
            f.write(_pack_le("d", 0.0))              # error
            f.write(_pack_le("Q", 0))                # track_len=0


# ---------------------------------------------------------------------------
# Main assembly
# ---------------------------------------------------------------------------

def assemble_colmap(session_dir: Path, tile_size: int = 1024) -> None:
    raw_iphone = session_dir / "raw" / "iphone"
    colmap_dir = session_dir / "colmap"
    images_dir = colmap_dir / "images"
    masks_dir  = colmap_dir / "masks"
    sparse_dir = colmap_dir / "sparse" / "0"
    depth_dir  = colmap_dir / "depth_images"

    for d in (sparse_dir, depth_dir):
        d.mkdir(parents=True, exist_ok=True)

    scan_dirs = sorted(raw_iphone.glob("scan_*"))
    if not scan_dirs:
        raise FileNotFoundError(f"No scan_* directories in {raw_iphone}")

    # Load multi_camera.yaml for Insta360 extrinsics
    mc_path = session_dir / "calibration" / "multi_camera.yaml"
    if not mc_path.exists():
        mc_path = Path(__file__).parent.parent / "config" / "multi_camera.yaml"
    with open(mc_path) as f:
        mc = yaml.safe_load(f)
    camera_configs = mc.get("cameras", [])

    # --- Pass 1: load all poses, filter by baseline ---
    scans: list[dict] = []
    positions: list[np.ndarray] = []

    for scan_dir in scan_dirs:
        pose_path = scan_dir / "pose.json"
        if not pose_path.exists():
            continue
        pose = load_pose(pose_path)

        R_w2c, t_w2c, C_colmap = arkit_to_colmap_pose(pose["transform_matrix"])

        # Baseline filter
        if positions and min(np.linalg.norm(C_colmap - q) for q in positions) < MIN_BASELINE_M:
            continue
        positions.append(C_colmap)

        scans.append({
            "scan_dir": scan_dir,
            "scan_name": scan_dir.name,
            "pose": pose,
            "R_w2c": R_w2c,
            "t_w2c": t_w2c,
            "C_colmap": C_colmap,
            "T_wc": np.array(pose["transform_matrix"]),  # ARKit camera-to-world
        })

    print(f"  {len(scans)} scans after baseline filter (min {MIN_BASELINE_M}m)")

    # --- Pass 2: determine active faces from first scan's ERP ---
    # Find which Insta360 cameras have images
    active_camera_ids = [c["id"] for c in camera_configs]

    # Collect all face indices that appear across all scans
    active_faces: set[int] = set()
    for scan in scans:
        for cam_id in active_camera_ids:
            erp_path = session_dir / "raw" / cam_id / f"{scan['scan_name']}.jpg"
            if erp_path.exists():
                # Probe which faces pass visibility threshold
                mask_path = session_dir / "masks" / f"{cam_id}.png"
                try:
                    results = slice_erp(
                        erp_path, images_dir, scan["scan_name"],
                        mask_path=mask_path if mask_path.exists() else None,
                        masks_dir=masks_dir,
                        tile_size=tile_size,
                    )
                    for face_idx, _ in results:
                        active_faces.add(face_idx)
                except Exception as e:
                    print(f"  Warning: ERP slice failed for {erp_path.name}: {e}")
        break  # probe first scan only

    print(f"  Active faces: {sorted(active_faces)}")

    # --- Camera ID assignment ---
    # camera_id 1 = iPhone PINHOLE
    # camera_id 2..N = one SIMPLE_PINHOLE per active face
    iphone_camera_id = 1
    tile_camera_ids: dict[int, int] = {
        face_idx: 2 + i for i, face_idx in enumerate(sorted(active_faces))
    }

    # Get iPhone intrinsics from first scan (ARKit auto-calibrated, stable)
    first_pose = scans[0]["pose"]
    img_w = first_pose["image_width"]
    img_h = first_pose["image_height"]
    K = np.array(first_pose["intrinsics"])
    iphone_fx, iphone_fy = float(K[0, 0]), float(K[1, 1])
    iphone_cx, iphone_cy = float(K[0, 2]), float(K[1, 2])

    write_cameras_bin(
        sparse_dir / "cameras.bin",
        iphone_camera_id, img_w, img_h,
        iphone_fx, iphone_fy, iphone_cx, iphone_cy,
        tile_camera_ids, tile_size,
    )
    print(f"  cameras.bin: 1 iPhone PINHOLE + {len(tile_camera_ids)} face tiles")

    # --- Pass 3: slice all ERPs, build image entries, unproject LiDAR ---
    image_entries: list[dict] = []
    all_pts_colmap: list[np.ndarray] = []
    image_id = 1

    # iPhone face dir
    iphone_face_dir = images_dir / "face_iphone"
    iphone_face_dir.mkdir(parents=True, exist_ok=True)
    iphone_depth_dir = depth_dir / "face_iphone"
    iphone_depth_dir.mkdir(parents=True, exist_ok=True)

    for scan in scans:
        scan_name = scan["scan_name"]
        pose = scan["pose"]
        R_w2c = scan["R_w2c"]
        t_w2c = scan["t_w2c"]
        C_colmap = scan["C_colmap"]
        T_wc = scan["T_wc"]  # ARKit camera-to-world 4x4

        # --- iPhone RGB image entry ---
        rgb_src = scan["scan_dir"] / "rgb.jpg"
        if rgb_src.exists():
            import shutil
            shutil.copy2(rgb_src, iphone_face_dir / f"{scan_name}.jpg")

        quat = rotation_to_quat_wxyz(R_w2c)
        image_entries.append({
            "image_id": image_id,
            "quat_wxyz": quat.tolist(),
            "tvec": t_w2c.tolist(),
            "camera_id": iphone_camera_id,
            "name": f"face_iphone/{scan_name}.jpg",
        })
        image_id += 1

        # --- Insta360 face tiles ---
        for cam_cfg in camera_configs:
            cam_id = cam_cfg["id"]
            erp_path = session_dir / "raw" / cam_id / f"{scan_name}.jpg"
            if not erp_path.exists():
                continue

            # Compute Insta360 world pose: T_insta360_world = T_insta360_iphone @ T_iphone_world
            ext = cam_cfg["extrinsic"]
            T_insta_iphone = _rpy_xyz_to_matrix(
                ext["roll"], ext["pitch"], ext["yaw"],
                ext["x"], ext["y"], ext["z"],
            )
            # T_wc is ARKit camera-to-world; invert to get world-to-camera, then
            # compose with extrinsic to get insta360-to-world
            T_iphone_world = np.linalg.inv(T_wc)  # world-to-iphone (w2c)
            T_insta_world = T_insta_iphone @ T_iphone_world  # insta360 w2c

            R_insta_w2c = T_insta_world[:3, :3]
            t_insta_w2c = T_insta_world[:3, 3]
            C_insta = -R_insta_w2c.T @ t_insta_w2c
            C_insta_colmap = R_ARKIT2COLMAP @ C_insta

            mask_path = session_dir / "masks" / f"{cam_id}.png"
            try:
                results = slice_erp(
                    erp_path, images_dir, scan_name,
                    mask_path=mask_path if mask_path.exists() else None,
                    masks_dir=masks_dir,
                    tile_size=tile_size,
                )
            except Exception as e:
                print(f"  Warning: slice failed {erp_path}: {e}")
                continue

            for face_idx, _ in results:
                cam_from_pano = FACES_CAM_FROM_PANO[face_idx]
                # Tile w2c = cam_from_pano @ insta360_w2c (in COLMAP frame)
                R_insta_w2c_col = R_ARKIT2COLMAP @ R_insta_w2c
                R_tile_w2c = cam_from_pano @ R_insta_w2c_col
                t_tile_w2c = -R_tile_w2c @ C_insta_colmap

                quat_tile = rotation_to_quat_wxyz(R_tile_w2c)
                cam_id_colmap = tile_camera_ids.get(face_idx)
                if cam_id_colmap is None:
                    continue

                image_entries.append({
                    "image_id": image_id,
                    "quat_wxyz": quat_tile.tolist(),
                    "tvec": t_tile_w2c.tolist(),
                    "camera_id": cam_id_colmap,
                    "name": f"face_{face_idx:02d}/{scan_name}.jpg",
                })
                image_id += 1

        # --- LiDAR depth unprojection ---
        depth_path = scan["scan_dir"] / "depth.bin"
        if depth_path.exists():
            depth_m = load_depth_bin(depth_path)

            # Scale ARKit intrinsics to LiDAR resolution
            scale_x = LIDAR_W / img_w
            scale_y = LIDAR_H / img_h
            K_lidar = [
                [iphone_fx * scale_x, 0.0, iphone_cx * scale_x],
                [0.0, iphone_fy * scale_y, iphone_cy * scale_y],
                [0.0, 0.0, 1.0],
            ]
            pts = unproject_depth(depth_m, K_lidar, T_wc)
            if len(pts) > 0:
                all_pts_colmap.append(pts)

        # --- iPhone depth image (project LiDAR onto iPhone tile) ---
        if all_pts_colmap:
            pts_for_depth = all_pts_colmap[-1] if all_pts_colmap else np.zeros((0, 3))
            if len(pts_for_depth) > 0:
                depth_img = render_depth_tile(
                    pts_for_depth, R_w2c, t_w2c,
                    iphone_fx, iphone_cx, iphone_cy,
                    img_w, img_h,
                )
                cv2.imwrite(
                    str(iphone_depth_dir / f"{scan_name}.png"),
                    depth_img,
                )

        # --- Insta360 face tile depth images ---
        if all_pts_colmap:
            pts_for_depth = all_pts_colmap[-1]
            f_tile, c_tile, _ = tile_intrinsics(tile_size)
            for entry in image_entries:
                if not entry["name"].startswith("face_") or "iphone" in entry["name"]:
                    continue
                if scan_name not in entry["name"]:
                    continue
                face_depth_dir = depth_dir / entry["name"].split("/")[0]
                face_depth_dir.mkdir(parents=True, exist_ok=True)
                R_e = np.array(
                    R.from_quat([
                        entry["quat_wxyz"][1], entry["quat_wxyz"][2],
                        entry["quat_wxyz"][3], entry["quat_wxyz"][0],
                    ]).as_matrix()
                )
                t_e = np.array(entry["tvec"])
                d_img = render_depth_tile(
                    pts_for_depth, R_e, t_e,
                    f_tile, c_tile, c_tile,
                    tile_size, tile_size,
                )
                cv2.imwrite(
                    str(face_depth_dir / f"{scan_name}.png"),
                    d_img,
                )

    # --- Write binary model ---
    merged_pts = np.vstack(all_pts_colmap) if all_pts_colmap else np.zeros((0, 3))
    write_images_bin(sparse_dir / "images.bin", image_entries)
    write_points3d_bin(sparse_dir / "points3D.bin", merged_pts)

    print(f"  images.bin: {len(image_entries)} images")
    print(f"  points3D.bin: {len(merged_pts)} LiDAR points")
    print(f"  depth_images/: {len(scans)} scans × tiles")
    print(f"  ✓ COLMAP model → {sparse_dir}")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rpy_xyz_to_matrix(roll: float, pitch: float, yaw: float,
                        x: float, y: float, z: float) -> np.ndarray:
    """Build 4x4 rigid transform from RPY (degrees) + XYZ (metres)."""
    r = np.radians(roll)
    p = np.radians(pitch)
    y_ = np.radians(yaw)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y_), np.sin(y_)
    rot = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [x, y, z]
    return T


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Assemble COLMAP model from Atlas Mobile session")
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--tile-size", type=int, default=1024)
    args = parser.parse_args()

    if not args.session_dir.exists():
        raise FileNotFoundError(f"Session directory not found: {args.session_dir}")

    print(f"Assembling COLMAP model: {args.session_dir}")
    assemble_colmap(args.session_dir, tile_size=args.tile_size)


if __name__ == "__main__":
    main()
