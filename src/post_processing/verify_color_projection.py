#!/usr/bin/env python3
"""
Color projection verification tool.
Shows the actual colors sampled from the ERP at each projected LiDAR point,
overlaid on the ERP image. If colors match the underlying ERP content, the
projection is correct.
"""
import numpy as np, cv2, yaml, open3d as o3d, sys, os
from scipy.spatial.transform import Rotation
from pathlib import Path

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()
_ALLOWED_SRC  = Path(os.path.expanduser('~/atlas_ws/src')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def verify_color_projection(scan_dir, output_path=None):
    try:
        scan = _safe_data(scan_dir)
    except ValueError as e:
        print(f'Error: {e}'); return None
    calib_path = (_ALLOWED_SRC / 'atlas-scanner/src/config/fusion_calibration.yaml').resolve()
    config = yaml.safe_load(open(calib_path))
    R_mat = Rotation.from_euler('xyz', [config['roll_offset'], config['pitch_offset'], config['yaw_offset']]).as_matrix()
    T = np.eye(4); T[:3,:3] = R_mat; T[:3,3] = [config['x_offset'], config['y_offset'], config['z_offset']]

    erp = cv2.imread(str(_safe_data(scan / 'equirect_dual_fisheye.jpg')))
    if erp is None:
        print(f'No ERP in {scan_dir}'); return
    h, w = erp.shape[:2]

    pcd = o3d.io.read_point_cloud(str(_safe_data(scan / 'sensor_lidar.ply')))
    pts = np.asarray(pcd.points)
    # Sample points at various distances
    dists = np.linalg.norm(pts, axis=1)
    mask = (dists > 0.8) & (dists < 8.0) & (pts[:,2] > 0.05)
    pts = pts[mask]
    idx = np.random.choice(len(pts), min(500, len(pts)), replace=False)
    pts = pts[idx]

    pts_h = np.hstack([pts, np.ones((len(pts),1))])
    cam = (T @ pts_h.T).T[:,:3]
    bearing = cam / np.linalg.norm(cam, axis=1, keepdims=True)
    lat = -np.arcsin(np.clip(bearing[:,1],-1,1))
    lon = np.arctan2(bearing[:,0], bearing[:,2])
    u = np.clip((w*(0.5+lon/(2*np.pi))).astype(int), 0, w-1)
    v = np.clip((h*(0.5-lat/np.pi)).astype(int), 0, h-1)

    # Sample colors from ERP
    sampled_bgr = erp[v, u]  # (N, 3) BGR

    # Draw circles colored with the sampled ERP color (not red)
    vis = cv2.resize(erp, (1920, 960))
    su, sv = 1920/w, 960/h
    for i in range(len(pts)):
        ui, vi = int(u[i]*su), int(v[i]*sv)
        b, g, r = int(sampled_bgr[i,0]), int(sampled_bgr[i,1]), int(sampled_bgr[i,2])
        # Draw a larger circle with the sampled color, outlined in white
        cv2.circle(vis, (ui, vi), 8, (b, g, r), -1)
        cv2.circle(vis, (ui, vi), 8, (255,255,255), 1)

    try:
        out = _safe_data(output_path) if output_path else _safe_data(scan / 'color_projection_check.jpg')
    except ValueError as e:
        print(f'Error: {e}'); return None
    cv2.imwrite(str(out), vis)
    print(f'Saved {out}')
    print(f'If circles match the background color at their location -> projection is correct')
    print(f'If circles are wrong color -> timing or calibration issue')
    return out

if __name__ == '__main__':
    scan_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    try:
        _safe_data(scan_dir)
    except ValueError as e:
        print(f'Error: {e}'); sys.exit(1)
    out = verify_color_projection(scan_dir)
    if out:
        import subprocess
        subprocess.Popen(['eog', str(out)], env=os.environ)
