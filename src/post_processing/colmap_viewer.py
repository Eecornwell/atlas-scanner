#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Generates a self-contained HTML viewer for a COLMAP sparse model,
# showing the point cloud (SfM + LiDAR-injected) and per-panorama camera frustums.
#
# Usage:
#   python3 colmap_viewer.py <session_dir>

import struct
import json
import os
import sys
import argparse
import subprocess
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


# ---------------------------------------------------------------------------
# Binary readers (shared with colmap_pose_quality.py)
# ---------------------------------------------------------------------------

def _read_cameras(path):
    cameras = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            cam_id = struct.unpack('I', f.read(4))[0]
            model  = struct.unpack('i', f.read(4))[0]
            w, h   = struct.unpack('QQ', f.read(16))
            nparams = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8, 5: 8, 6: 8, 7: 0}.get(model, 0)
            params = list(struct.unpack(f'{nparams}d', f.read(8 * nparams)))
            cameras[cam_id] = {'model': model, 'w': w, 'h': h, 'params': params}
    return cameras


def _read_images(path):
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            img_id          = struct.unpack('I', f.read(4))[0]
            qw, qx, qy, qz = struct.unpack('dddd', f.read(32))
            tx, ty, tz      = struct.unpack('ddd',  f.read(24))
            cam_id          = struct.unpack('I',    f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00':
                    break
                name += c
            n_pts = struct.unpack('Q', f.read(8))[0]
            f.read(n_pts * 24)
            images[img_id] = {
                'name':      name.decode(),
                'qvec':      [qw, qx, qy, qz],
                'tvec':      [tx, ty, tz],
                'camera_id': cam_id,
            }
    return images


def _read_points3d(path, max_pts=200000):
    pts, cols, is_sfm = [], [], []
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        step = max(1, n // max_pts)
        for i in range(n):
            p3d_id    = struct.unpack('Q', f.read(8))[0]
            x, y, z   = struct.unpack('ddd', f.read(24))
            r, g, b   = struct.unpack('BBB', f.read(3))
            error     = struct.unpack('d', f.read(8))[0]
            tl        = struct.unpack('Q', f.read(8))[0]
            f.read(tl * 8)
            if i % step == 0:
                pts.append([x, y, z])
                cols.append([r / 255, g / 255, b / 255])
                is_sfm.append(tl > 0)
    return np.array(pts), np.array(cols), np.array(is_sfm)


# ---------------------------------------------------------------------------
# Derive per-panorama poses from per-tile images
# ---------------------------------------------------------------------------

def _pano_index(name):
    """face_XX/pano_NNN.png -> NNN"""
    try:
        return int(name.split('pano_')[1].split('.')[0])
    except Exception:
        return -1


def _face_index(name):
    """face_XX/pano_NNN.png -> XX"""
    try:
        return int(name.split('face_')[1].split('/')[0])
    except Exception:
        return -1


def _camera_center(qvec, tvec):
    qw, qx, qy, qz = qvec
    R_w2c = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    return (-R_w2c.T @ np.array(tvec)).tolist()


def build_panoramas(images, cameras):
    """Group tiles by panorama index; return list of panorama dicts."""
    by_pano = {}
    for img in images.values():
        pi = _pano_index(img['name'])
        fi = _face_index(img['name'])
        if pi < 0:
            continue
        by_pano.setdefault(pi, {})[fi] = img

    panoramas = []
    for pano_idx in sorted(by_pano):
        tiles = by_pano[pano_idx]
        # Use face_00 (ceiling) as the reference pose; fall back to any available face
        ref = tiles.get(0) or next(iter(tiles.values()))
        center = _camera_center(ref['qvec'], ref['tvec'])
        cam = cameras.get(ref['camera_id'], {})
        f_px = cam['params'][0] if cam.get('params') else 512.0
        tile_size = cam.get('w', 1024)

        # Collect per-face orientations for frustum drawing
        face_quats = {}
        for fi, tile in tiles.items():
            face_quats[fi] = tile['qvec']

        panoramas.append({
            'idx':        pano_idx,
            'center':     center,
            'ref_qvec':   ref['qvec'],
            'ref_tvec':   ref['tvec'],
            'face_quats': face_quats,
            'f_px':       f_px,
            'tile_size':  tile_size,
            'n_faces':    len(tiles),
        })
    return panoramas


# ---------------------------------------------------------------------------
# HTML generation
# ---------------------------------------------------------------------------

def _encode_float32(arr):
    """Encode a flat float32 numpy array as a base64 string for embedding."""
    import base64
    return base64.b64encode(arr.astype(np.float32).tobytes()).decode()


def generate_html(sparse_dir, output_path, max_pts=200000):
    sparse_dir = _safe_data(sparse_dir)
    cameras  = _read_cameras(sparse_dir / 'cameras.bin')
    images   = _read_images(sparse_dir  / 'images.bin')
    # points3D.bin now contains only LiDAR points (track_len=0).
    # SfM points come from reconstructed.ply (COLMAP model_converter output).
    lidar_pts, lidar_cols, _ = _read_points3d(sparse_dir / 'points3D.bin', max_pts)

    sfm_pts  = np.zeros((0, 3), dtype=np.float32)
    sfm_cols = np.zeros((0, 3), dtype=np.float32)
    recon_ply = sparse_dir.parent / 'reconstructed.ply'
    if recon_ply.exists():
        try:
            import open3d as _o3d
            _pcd = _o3d.io.read_point_cloud(str(recon_ply))
            sfm_pts  = np.asarray(_pcd.points, dtype=np.float32)
            sfm_cols = np.asarray(_pcd.colors, dtype=np.float32) if _pcd.has_colors() \
                       else np.zeros((len(sfm_pts), 3), dtype=np.float32)
            print(f'  SfM PLY: {len(sfm_pts)} points from {recon_ply.name}')
        except Exception as _e:
            print(f'  ⚠ Could not load SfM PLY: {_e}')

    panoramas = build_panoramas(images, cameras)
    n_panos  = len(panoramas)
    n_lidar  = len(lidar_pts)
    n_sfm    = len(sfm_pts)

    import base64
    # Encode LiDAR and SfM separately — no shared mask needed
    pts_b64      = _encode_float32(lidar_pts.flatten())
    cols_b64     = _encode_float32(lidar_cols.flatten())
    sfm_pts_b64  = _encode_float32(sfm_pts.flatten())
    sfm_cols_b64 = _encode_float32(sfm_cols.flatten())
    # Keep sfm_b64 as empty mask for backward compat (unused now)
    sfm_b64 = base64.b64encode(np.zeros(0, dtype=np.uint8).tobytes()).decode()

    # Build panorama JSON (frustum poses)
    pano_json = json.dumps(panoramas)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>COLMAP Model Viewer</title>
<style>
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{ background: #111; color: #eee; font-family: 'Segoe UI', sans-serif; overflow: hidden; }}
#canvas-container {{ position: absolute; inset: 0; }}
#panel {{
  position: absolute; top: 12px; left: 12px; width: 260px;
  background: rgba(0,0,0,0.75); border-radius: 8px; padding: 12px;
  font-size: 13px; user-select: none;
}}
#panel h2 {{ font-size: 15px; margin-bottom: 8px; color: #7cf; }}
.stat {{ color: #aaa; margin-bottom: 4px; }}
.stat span {{ color: #eee; float: right; }}
hr {{ border-color: #333; margin: 8px 0; }}
.row {{ display: flex; align-items: center; justify-content: space-between; margin: 5px 0; }}
.row label {{ cursor: pointer; }}
input[type=range] {{ width: 110px; }}
input[type=checkbox] {{ cursor: pointer; }}
#pano-list {{
  max-height: 160px; overflow-y: auto; margin-top: 6px;
  border: 1px solid #333; border-radius: 4px; padding: 4px;
}}
.pano-row {{
  display: flex; align-items: center; gap: 6px;
  padding: 2px 4px; border-radius: 3px; cursor: pointer; font-size: 12px;
}}
.pano-row:hover {{ background: #222; }}
.swatch {{ width: 12px; height: 12px; border-radius: 2px; flex-shrink: 0; }}
#help {{ position: absolute; bottom: 12px; left: 12px; font-size: 11px; color: #555; }}
</style>
</head>
<body>
<div id="canvas-container"></div>

<div id="panel">
  <h2>COLMAP Viewer</h2>
  <div class="stat">Panoramas <span id="s-panos">{n_panos}</span></div>
  <div class="stat">SfM points <span id="s-sfm">{n_sfm}</span></div>
  <div class="stat">LiDAR points <span id="s-lidar">{n_lidar}</span></div>
  <div class="stat">Showing <span id="s-showing">{n_lidar + n_sfm}</span></div>
  <hr>
  <div class="row">
    <label><input type="checkbox" id="chk-sfm" checked> SfM points</label>
  </div>
  <div class="row">
    <label><input type="checkbox" id="chk-lidar" checked> LiDAR points</label>
  </div>
  <div class="row">
    <label><input type="checkbox" id="chk-frustums" checked> Camera frustums</label>
  </div>
  <div class="row">
    <label><input type="checkbox" id="chk-axes"> Frustum axes</label>
  </div>
  <div class="row">
    <label for="sl-pt-size">LiDAR pt size</label>
    <input type="range" id="sl-pt-size" min="1" max="20" value="3" step="1">
  </div>
  <div class="row">
    <label for="sl-sfm-size">SfM pt size</label>
    <input type="range" id="sl-sfm-size" min="1" max="40" value="8" step="1">
  </div>
  <div class="row">
    <label for="sl-frustum-scale">Frustum scale</label>
    <input type="range" id="sl-frustum-scale" min="1" max="30" value="10" step="1">
  </div>
  <hr>
  <div style="font-size:11px;color:#888;margin-bottom:4px;">Panoramas (click to highlight)</div>
  <div id="pano-list"></div>
</div>

<div id="help">Left-drag: rotate &nbsp;|&nbsp; Right-drag / scroll: zoom &nbsp;|&nbsp; Middle-drag: pan</div>

<script type="importmap">
{{
  "imports": {{
    "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
    "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"
  }}
}}
</script>
<script type="module">
import * as THREE from 'three';
import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';

// ── decode base64 float32 ──────────────────────────────────────────────────
function b64toF32(b64) {{
  const bin = atob(b64);
  const buf = new ArrayBuffer(bin.length);
  const u8  = new Uint8Array(buf);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return new Float32Array(buf);
}}
function b64toU8(b64) {{
  const bin = atob(b64);
  const u8  = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return u8;
}}

const ptsFlat      = b64toF32('{pts_b64}');
const colsFlat     = b64toF32('{cols_b64}');
const sfmPtsFlat   = b64toF32('{sfm_pts_b64}');
const sfmColsFlat  = b64toF32('{sfm_cols_b64}');
const panoramas = {pano_json};

// ── scene setup ────────────────────────────────────────────────────────────
const renderer = new THREE.WebGLRenderer({{ antialias: true }});
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('canvas-container').appendChild(renderer.domElement);

const scene  = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
// COLMAP uses Y-down (ROS Y-left -> COLMAP Y-down via R_ROS2COLMAP).
// Rotate the root group 180deg around X so Y points up in the viewer.
const root = new THREE.Group();
root.rotation.x = Math.PI;
scene.add(root);
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 500);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// ── palette (one colour per panorama) ─────────────────────────────────────
function hsl(h, s, l) {{
  return new THREE.Color().setHSL(h, s, l);
}}
function paletteColor(i, n) {{
  return hsl((i / Math.max(n, 1) + 0.05) % 1.0, 0.85, 0.55);
}}

// ── point cloud ────────────────────────────────────────────────────────────
function _makeCloud(posFlat, colFlat, mat) {{
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(posFlat, 3));
  geo.setAttribute('color',    new THREE.BufferAttribute(colFlat, 3));
  return new THREE.Points(geo, mat);
}}

const lidarMat = new THREE.PointsMaterial({{ vertexColors: true, size: 0.03, sizeAttenuation: true }});
const sfmMat   = new THREE.PointsMaterial({{ vertexColors: true, size: 0.08, sizeAttenuation: true }});
const lidarCloud = _makeCloud(ptsFlat,    colsFlat,    lidarMat);
const sfmCloud   = _makeCloud(sfmPtsFlat, sfmColsFlat, sfmMat);
root.add(lidarCloud);
root.add(sfmCloud);

function rebuildPointCloud() {{
  lidarCloud.visible = document.getElementById('chk-lidar').checked;
  sfmCloud.visible   = document.getElementById('chk-sfm').checked;
  const showing = (lidarCloud.visible ? lidarCloud.geometry.attributes.position.count : 0)
                + (sfmCloud.visible   ? sfmCloud.geometry.attributes.position.count   : 0);
  document.getElementById('s-showing').textContent = showing;
}}

// ── frustum helpers ────────────────────────────────────────────────────────
function makeFrustum(color, scale) {{
  // Simple pyramid: apex at origin, base at z=1 in camera space
  const h = scale, hw = scale * 0.75;
  const verts = new Float32Array([
     0,    0,   0,   -hw, -hw,  h,
     0,    0,   0,    hw, -hw,  h,
     0,    0,   0,    hw,  hw,  h,
     0,    0,   0,   -hw,  hw,  h,
    -hw, -hw,   h,    hw, -hw,  h,
     hw, -hw,   h,    hw,  hw,  h,
     hw,  hw,   h,   -hw,  hw,  h,
    -hw,  hw,   h,   -hw, -hw,  h,
  ]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
  return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({{ color, linewidth: 1 }}));
}}

function makeAxisLines(scale) {{
  const s = scale * 1.3;
  const verts = new Float32Array([
    0,0,0, s,0,0,  0,0,0, 0,s,0,  0,0,0, 0,0,s
  ]);
  const colors = new Float32Array([
    1,0,0, 1,0,0,  0,1,0, 0,1,0,  0,0,1, 0,0,1
  ]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
  geo.setAttribute('color',    new THREE.BufferAttribute(colors, 3));
  return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({{ vertexColors: true }}));
}}

// ── build frustum group ────────────────────────────────────────────────────
const frustumGroup = new THREE.Group();
root.add(frustumGroup);
const axisGroup    = new THREE.Group();
root.add(axisGroup);
axisGroup.visible = false;

let frustumScale = 0.10;
const panoGroups = [];  // per-panorama THREE.Group for highlighting

function buildFrustums(scale) {{
  frustumGroup.clear();
  axisGroup.clear();
  panoGroups.length = 0;

  panoramas.forEach((pano, pi) => {{
    const color = paletteColor(pi, panoramas.length);
    const pg = new THREE.Group();
    frustumGroup.add(pg);
    panoGroups.push(pg);

    // One frustum per face
    Object.entries(pano.face_quats).forEach(([fi, qvec]) => {{
      const [qw, qx, qy, qz] = qvec;
      // w2c quaternion -> c2w rotation matrix
      const R_w2c = new THREE.Quaternion(qx, qy, qz, qw);
      const R_c2w = R_w2c.clone().invert();

      const frus = makeFrustum(color, scale);
      frus.quaternion.copy(R_c2w);
      frus.position.set(...pano.center);
      pg.add(frus);
    }});

    // Axis indicator at panorama center
    const ax = makeAxisLines(scale);
    const [qw, qx, qy, qz] = pano.ref_qvec;
    ax.quaternion.copy(new THREE.Quaternion(qx, qy, qz, qw).invert());
    ax.position.set(...pano.center);
    axisGroup.add(ax);
  }});
}}

buildFrustums(frustumScale);

// ── panorama list panel ────────────────────────────────────────────────────
const listEl = document.getElementById('pano-list');
let highlighted = null;

panoramas.forEach((pano, pi) => {{
  const color = paletteColor(pi, panoramas.length);
  const hex   = '#' + color.getHexString();
  const row   = document.createElement('div');
  row.className = 'pano-row';
  row.dataset.pi = pi;
  row.innerHTML = `<div class="swatch" style="background:${{hex}}"></div>pano_${{String(pano.idx).padStart(3,'0')}} <span style="color:#666;margin-left:auto">${{pano.n_faces}}f</span>`;
  row.addEventListener('click', () => {{
    // Just highlight — no camera movement
    if (highlighted !== null) {{
      panoGroups[highlighted].children.forEach(m => {{
        m.material.color = paletteColor(highlighted, panoramas.length);
      }});
    }}
    highlighted = pi;
    panoGroups[pi].children.forEach(m => m.material.color.set(0xffffff));
    listEl.querySelectorAll('.pano-row').forEach(r => r.style.background = '');
    row.style.background = '#1a2a3a';
  }});
  listEl.appendChild(row);
}});

// ── fit camera to scene ────────────────────────────────────────────────────
const _fitGeo = lidarCloud.geometry.attributes.position.count > 0
  ? lidarCloud.geometry : sfmCloud.geometry;
_fitGeo.computeBoundingBox();
const bbox   = _fitGeo.boundingBox;
const center = new THREE.Vector3();
bbox.getCenter(center);
const span   = bbox.getSize(new THREE.Vector3()).length();
camera.position.copy(center).addScalar(span * 0.6);
camera.position.y += span * 0.3;
controls.target.copy(center);
controls.update();

// ── controls wiring ────────────────────────────────────────────────────────
document.getElementById('chk-sfm').addEventListener('change',     rebuildPointCloud);
document.getElementById('chk-lidar').addEventListener('change',   rebuildPointCloud);
document.getElementById('chk-frustums').addEventListener('change', e => {{
  frustumGroup.visible = e.target.checked;
}});
document.getElementById('chk-axes').addEventListener('change', e => {{
  axisGroup.visible = e.target.checked;
}});
document.getElementById('sl-pt-size').addEventListener('input', e => {{
  lidarMat.size = parseFloat(e.target.value) * 0.01;
}});
document.getElementById('sl-sfm-size').addEventListener('input', e => {{
  sfmMat.size = parseFloat(e.target.value) * 0.01;
}});
document.getElementById('sl-frustum-scale').addEventListener('input', e => {{
  frustumScale = parseFloat(e.target.value) * 0.01;
  buildFrustums(frustumScale);
}});


// ── render loop ────────────────────────────────────────────────────────────
function animate() {{
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}}
animate();

window.addEventListener('resize', () => {{
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}});
</script>
</body>
</html>"""

    output_path = _safe_data(output_path)
    output_path.write_text(html)
    print(f"✓ COLMAP viewer: {output_path}  ({n_panos} panoramas, {n_lidar} LiDAR + {n_sfm} SfM points)")
    return output_path


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--sparse', default='colmap/sparse/0',
                        help='Sparse model subdirectory (default: colmap/sparse/0)')
    parser.add_argument('--max-points', type=int, default=200000)
    parser.add_argument('--no-open', action='store_true')
    args = parser.parse_args()

    try:
        session = _safe_data(args.session_dir)
    except ValueError as e:
        print(f"Error: {e}"); sys.exit(1)

    sparse_dir  = session / args.sparse
    output_path = session / 'colmap' / 'colmap_model_viewer.html'

    if not sparse_dir.exists():
        print(f"No sparse model at {sparse_dir}"); sys.exit(1)

    generate_html(sparse_dir, output_path, args.max_points)

    if os.environ.get('ATLAS_GUI_MODE'):
        print(f'\u2713 3D viewer ready: {output_path}')
        return

    if args.no_open:
        return

    url = f'file://{output_path}'
    env = os.environ.copy()
    env.setdefault('DISPLAY', ':0')
    for browser, cmd in [
        ('firefox',          ['firefox', '--new-window', url]),
        ('chromium-browser', ['chromium-browser', '--new-window', url]),
        ('chromium',         ['chromium', '--new-window', url]),
        ('google-chrome',    ['google-chrome', '--new-window', url]),
        ('xdg-open',         ['xdg-open', url]),
    ]:
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             env=env, preexec_fn=os.setpgrp)
            return
        except FileNotFoundError:
            continue


if __name__ == '__main__':
    main()
