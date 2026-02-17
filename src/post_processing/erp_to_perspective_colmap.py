#!/usr/bin/env python3
"""
Convert equirectangular images to perspective projections for COLMAP.
Uses OpenCV and COLMAP CLI with camera mask support.
"""

import sys
import cv2
import numpy as np
from pathlib import Path
import subprocess
import json
from scipy.spatial.transform import Rotation as R

def compute_perspective_pose(erp_quat, erp_trans, yaw, pitch, roll=0):
    """Compute perspective camera pose from ERP pose and view direction
    
    Input: erp_quat/erp_trans in COLMAP format (w2c quat + T vector)
    Output: perspective w2c quat + T vector with same projection center
    """
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)
    roll_rad = np.radians(roll)
    
    # COLMAP: +X right, +Y down, +Z forward
    # Build rotation for cubemap face in panorama's local frame
    Ry = np.array([[np.cos(yaw_rad), 0, np.sin(yaw_rad)],
                   [0, 1, 0],
                   [-np.sin(yaw_rad), 0, np.cos(yaw_rad)]])
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
                   [0, np.sin(pitch_rad), np.cos(pitch_rad)]])
    
    Rz = np.array([[np.cos(roll_rad), -np.sin(roll_rad), 0],
                   [np.sin(roll_rad), np.cos(roll_rad), 0],
                   [0, 0, 1]])
    
    # Apply roll AFTER yaw and pitch
    R_local = Ry @ Rx @ Rz
    
    # Get panorama w2c rotation and T
    qw, qx, qy, qz = erp_quat
    R_pano_w2c = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T_pano = np.array(erp_trans)
    
    # Compute panorama projection center: C = -R_pano^T * T_pano
    C = -R_pano_w2c.T @ T_pano
    
    # Apply local rotation: R_persp_w2c = R_local * R_pano_w2c
    R_persp_w2c = R_local @ R_pano_w2c
    
    # Same projection center, new T: T_persp = -R_persp * C
    T_persp = -R_persp_w2c @ C
    
    r = R.from_matrix(R_persp_w2c)
    quat = r.as_quat()  # [x, y, z, w]
    
    return [quat[3], quat[0], quat[1], quat[2]], T_persp.tolist()

def erp_to_perspective(erp_img, fov_deg, yaw, pitch, output_size=1024):
    """Convert ERP image to perspective projection (roll not applied to image)"""
    h, w = erp_img.shape[:2]
    
    # Output image
    out_h, out_w = output_size, output_size
    
    # Camera parameters
    f = out_w / (2 * np.tan(np.radians(fov_deg) / 2))
    cx, cy = out_w / 2, out_h / 2
    
    # Create meshgrid for output image
    u, v = np.meshgrid(np.arange(out_w), np.arange(out_h))
    
    # Convert to normalized camera coordinates
    x = (u - cx) / f
    y = (v - cy) / f
    z = np.ones_like(x)
    
    # Rotation matrices (NO roll for image sampling)
    yaw_rad = np.radians(-yaw)  # Negate yaw to fix mirror
    pitch_rad = np.radians(pitch)
    
    Ry = np.array([[np.cos(yaw_rad), 0, np.sin(yaw_rad)],
                   [0, 1, 0],
                   [-np.sin(yaw_rad), 0, np.cos(yaw_rad)]])
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
                   [0, np.sin(pitch_rad), np.cos(pitch_rad)]])
    
    R = Ry @ Rx
    
    # Apply rotation
    xyz = np.stack([x, y, z], axis=-1)
    xyz_rot = xyz @ R.T
    
    # Convert to spherical coordinates (negate X to fix mirror)
    lon = np.arctan2(-xyz_rot[..., 0], xyz_rot[..., 2])
    lat = np.arcsin(np.clip(xyz_rot[..., 1] / np.linalg.norm(xyz_rot, axis=-1), -1, 1))
    
    # Map to ERP image coordinates
    erp_u = ((lon + np.pi) / (2 * np.pi) * w).astype(np.float32)
    erp_v = ((np.pi / 2 - lat) / np.pi * h).astype(np.float32)
    
    # Remap with cubic interpolation and border replication to reduce seams
    perspective = cv2.remap(erp_img, erp_u, erp_v, cv2.INTER_CUBIC, borderMode=cv2.BORDER_WRAP)
    
    return perspective

def convert_erp_to_perspectives(session_dir):
    """Convert all ERP images to perspective projections"""
    session_path = Path(session_dir)
    colmap_dir = session_path / "colmap"
    erp_dir = colmap_dir / "images"
    persp_dir = colmap_dir / "perspective_images"
    mask_dir = colmap_dir / "perspective_masks"
    sparse_dir = colmap_dir / "sparse" / "0"
    
    # Clean up previous perspective images and reconstruction
    import shutil
    if persp_dir.exists():
        shutil.rmtree(persp_dir)
    persp_dir.mkdir(exist_ok=True)
    
    if mask_dir.exists():
        shutil.rmtree(mask_dir)
    mask_dir.mkdir(exist_ok=True)
    
    recon_dir = colmap_dir / "perspective_reconstruction"
    if recon_dir.exists():
        shutil.rmtree(recon_dir)
    
    db_file = colmap_dir / "perspective_database.db"
    if db_file.exists():
        db_file.unlink()
    
    print(f"Converting ERP images to perspective projections...")
    print(f"  Input: {erp_dir}")
    print(f"  Output: {persp_dir}")
    print(f"  Masks: {mask_dir}")
    
    # Load poses from images.txt
    images_txt = sparse_dir / "images.txt"
    poses = {}
    
    with open(images_txt) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) >= 10:
                img_id = int(parts[0])
                qw, qx, qy, qz = map(float, parts[1:5])
                tx, ty, tz = map(float, parts[5:8])
                img_name = parts[9]
                poses[img_name] = {
                    'id': img_id,
                    'quat': [qw, qx, qy, qz],
                    'trans': [tx, ty, tz]
                }
    
    # Generate 6 cubemap faces: (yaw, pitch, image_roll)
    # image_roll rotates the final image to match scene orientation
    views = [
        (0, 0, 0),       # Front (+Z) - view00
        (-90, 0, 90),    # Right (+X) - view01 - SWAPPED with view03
        (180, 0, 0),     # Back (-Z) - view02
        (90, 0, -90),    # Left (-X) - view03 - SWAPPED with view01
        (0, -90, 180),   # Top (-Y) - view04 - rotate 180
        (0, 90, 0),      # Bottom (+Y) - view05
    ]
    
    perspective_images = []
    
    # Process images from colmap/images directory
    for erp_file in sorted(erp_dir.glob("*.jpg")) + sorted(erp_dir.glob("*.png")):
        # Skip mask files
        if '.mask.' in erp_file.name:
            continue
            
        print(f"\nProcessing {erp_file.name}...")
        erp_img = cv2.imread(str(erp_file))
        
        if erp_img is None:
            print(f"  Failed to load {erp_file}")
            continue
        
        # Check for corresponding mask
        mask_file = erp_dir / f"{erp_file.name}.mask.png"
        erp_mask = None
        if mask_file.exists():
            erp_mask = cv2.imread(str(mask_file), cv2.IMREAD_GRAYSCALE)
        
        base_name = erp_file.stem
        pose = poses.get(erp_file.name, None)
        
        for i, (yaw, pitch, image_roll) in enumerate(views):
            persp_img = erp_to_perspective(erp_img, fov_deg=90, yaw=yaw, pitch=pitch)
            
            # Apply image rotation if needed (not applied to pose)
            if image_roll == 90:
                persp_img = cv2.rotate(persp_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif image_roll == -90:
                persp_img = cv2.rotate(persp_img, cv2.ROTATE_90_CLOCKWISE)
            elif image_roll == 180:
                persp_img = cv2.rotate(persp_img, cv2.ROTATE_180)
            
            out_name = f"{base_name}_view{i:02d}.png"
            out_path = persp_dir / out_name
            cv2.imwrite(str(out_path), persp_img)
            print(f"  Saved: {out_name}")
            
            # Generate mask for each perspective view
            if erp_mask is not None:
                persp_mask = erp_to_perspective(erp_mask, fov_deg=90, yaw=yaw, pitch=pitch)
                
                # Rotate mask same as image
                if image_roll == 90:
                    persp_mask = cv2.rotate(persp_mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif image_roll == -90:
                    persp_mask = cv2.rotate(persp_mask, cv2.ROTATE_90_CLOCKWISE)
                elif image_roll == 180:
                    persp_mask = cv2.rotate(persp_mask, cv2.ROTATE_180)
                # Threshold to binary: 0 = masked, 255 = valid
                _, persp_mask = cv2.threshold(persp_mask, 127, 255, cv2.THRESH_BINARY)
                # Save mask in separate directory with .png.png naming
                mask_out_name = f"{out_name}.png"
                cv2.imwrite(str(mask_dir / mask_out_name), persp_mask)
            
            # Compute perspective camera pose (no roll in pose)
            if pose:
                persp_quat, persp_trans = compute_perspective_pose(
                    pose['quat'], pose['trans'], yaw, pitch, roll=0
                )
            else:
                persp_quat = [1.0, 0.0, 0.0, 0.0]
                persp_trans = [0.0, 0.0, 0.0]
            
            perspective_images.append({
                'name': out_name,
                'original': erp_file.name,
                'view_index': i,
                'yaw': yaw,
                'pitch': pitch,
                'roll': 0,
                'image_roll': image_roll,
                'quat': persp_quat,
                'trans': persp_trans
            })
        
        # Swap poses between view01 and view03 for this panorama
        pano_views = [img for img in perspective_images if img['original'] == erp_file.name]
        if len(pano_views) >= 4:
            view01 = next((v for v in pano_views if v['view_index'] == 1), None)
            view03 = next((v for v in pano_views if v['view_index'] == 3), None)
            if view01 and view03:
                view01['quat'], view03['quat'] = view03['quat'], view01['quat']
                view01['trans'], view03['trans'] = view03['trans'], view01['trans']
        
        print(f"  Generated {len(views)} perspective views")
    
    print(f"\n✓ Generated {len(perspective_images)} perspective images")
    
    # Save metadata
    metadata_file = persp_dir / "metadata.json"
    with open(metadata_file, 'w') as f:
        json.dump(perspective_images, f, indent=2)
    
    return persp_dir

def run_colmap_on_perspectives(persp_dir, session_dir):
    """Run COLMAP reconstruction on perspective images using rig configuration"""
    session_path = Path(session_dir)
    colmap_dir = session_path / "colmap"
    database = colmap_dir / "perspective_database.db"
    output_dir = colmap_dir / "perspective_reconstruction"
    output_dir.mkdir(exist_ok=True)
    
    print(f"\nRunning COLMAP on perspective images with rig configuration...")
    
    # Remove old database
    if database.exists():
        database.unlink()
    
    # Load metadata to create rig configuration
    metadata_file = persp_dir / "metadata.json"
    with open(metadata_file) as f:
        metadata = json.load(f)
    
    # Group by original panorama
    panos = {}
    for item in metadata:
        orig = item['original']
        if orig not in panos:
            panos[orig] = []
        panos[orig].append(item)
    
    # Create rigs.txt for COLMAP
    rigs_txt = output_dir / "rigs.txt"
    with open(rigs_txt, 'w') as f:
        f.write("# Rig configuration for panorama views\n")
        f.write("# RIG_ID, NUM_CAMERAS\n")
        for rig_id, (pano_name, views) in enumerate(panos.items()):
            f.write(f"{rig_id} {len(views)}\n")
    
    print(f"Created rig configuration: {len(panos)} rigs with {len(metadata)} total views")
    
    # Write images.txt with perspective camera poses
    persp_sparse_dir = output_dir / "input"
    persp_sparse_dir.mkdir(exist_ok=True)
    
    images_txt_out = persp_sparse_dir / "images.txt"
    with open(images_txt_out, 'w') as f:
        f.write("# Image list with two lines of data per image:\n")
        f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
        for idx, item in enumerate(metadata, start=1):
            qw, qx, qy, qz = item['quat']
            tx, ty, tz = item['trans']
            f.write(f"{idx} {qw} {qx} {qy} {qz} {tx} {ty} {tz} {idx} {item['name']}\n")
            f.write("\n")  # Empty POINTS2D line
    
    print(f"✓ Wrote perspective camera poses to {images_txt_out}")
    print(f"  Sample pose: quat={metadata[0]['quat']}, trans={metadata[0]['trans']}")
    
    # Write cameras.txt with PINHOLE camera model
    cameras_txt_out = persp_sparse_dir / "cameras.txt"
    with open(cameras_txt_out, 'w') as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        # PINHOLE: f, cx, cy
        focal = 1024 / (2 * np.tan(np.radians(90) / 2))  # 90 degree FOV
        for idx, item in enumerate(metadata, start=1):
            image_roll = item.get('image_roll', 0)
            # If image is rotated 90 or -90, swap width/height
            if abs(image_roll) == 90:
                w, h = 1024, 1024  # Square, so no change needed
                cx, cy = 512, 512
            else:
                w, h = 1024, 1024
                cx, cy = 512, 512
            f.write(f"{idx} PINHOLE {w} {h} {focal} {focal} {cx} {cy}\n")
    
    # Write empty points3D.txt
    points_txt_out = persp_sparse_dir / "points3D.txt"
    with open(points_txt_out, 'w') as f:
        f.write("# 3D point list with one line of data per point:\n")
        f.write("#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n")
    
    print(f"✓ Wrote cameras.txt and points3D.txt")
    
    # Feature extraction with per-image masks
    print("\n1. Extracting features...")
    mask_dir = colmap_dir / "perspective_masks"
    
    # Check if mask directory has files
    mask_files = list(mask_dir.glob("*.png"))
    print(f"  Found {len(mask_files)} mask files")
    
    cmd = [
        "colmap", "feature_extractor",
        "--database_path", str(database),
        "--image_path", str(persp_dir),
        "--ImageReader.camera_model", "PINHOLE",
        "--ImageReader.single_camera", "0"
    ]
    
    # Only add mask path if masks exist
    if mask_files:
        cmd.extend(["--ImageReader.mask_path", str(mask_dir)])
    
    subprocess.run(cmd)
    
    # Feature matching with rig configuration
    print("\n2. Matching features (rig-aware)...")
    subprocess.run([
        "colmap", "exhaustive_matcher",
        "--database_path", str(database)
    ])
    
    # Reconstruction: Import features into model with known poses
    print("\n3. Importing features with known poses...")
    output_model_dir = output_dir / "0"
    output_model_dir.mkdir(parents=True, exist_ok=True)
    
    subprocess.run([
        "colmap", "point_triangulator",
        "--database_path", str(database),
        "--image_path", str(persp_dir),
        "--input_path", str(persp_sparse_dir),
        "--output_path", str(output_model_dir)
    ])
    
    # Export to PLY
    model_dir = output_dir / "0"
    if model_dir.exists():
        print(f"\n✓ Reconstruction complete: {model_dir}")
        
        ply_output = output_dir / "reconstructed.ply"
        subprocess.run([
            "colmap", "model_converter",
            "--input_path", str(model_dir),
            "--output_path", str(ply_output),
            "--output_type", "PLY"
        ])
        
        if ply_output.exists():
            print(f"✓ Exported to: {ply_output}")
            
            # Compare with original
            original_ply = colmap_dir / "sparse" / "0" / "sparse.ply"
            if original_ply.exists():
                compare_point_clouds(original_ply, ply_output)
            
            return True
    
    return False

def compare_point_clouds(original_path, reconstructed_path):
    """Compare original and reconstructed point clouds"""
    print(f"\nComparing point clouds...")
    
    # Count points
    def count_ply_points(path):
        with open(path, 'rb') as f:
            for line in f:
                try:
                    line_str = line.decode('utf-8').strip()
                    if line_str.startswith('element vertex'):
                        return int(line_str.split()[2])
                except:
                    continue
        return 0
    
    orig_count = count_ply_points(original_path)
    recon_count = count_ply_points(reconstructed_path)
    
    print(f"  Original sparse.ply: {orig_count} points")
    print(f"  Reconstructed: {recon_count} points")
    
    if recon_count > 0:
        ratio = recon_count / orig_count * 100
        print(f"  Recovery rate: {ratio:.1f}%")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 erp_to_perspective_colmap.py <session_directory>")
        sys.exit(1)
    
    session_dir = sys.argv[1]
    
    # Convert ERP to perspective
    persp_dir = convert_erp_to_perspectives(session_dir)
    
    # Run COLMAP
    run_colmap_on_perspectives(persp_dir, session_dir)
