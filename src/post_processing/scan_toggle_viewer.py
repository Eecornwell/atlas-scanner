#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Generates a self-contained HTML viewer showing each scan as a
# separately toggleable layer, using trajectory poses only (no ICP alignment).
# Useful for debugging color projection and per-scan alignment quality.

import sys
import os
import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def load_ply_colored(path, max_pts=8000):
    import numpy as np
    safe = _safe_data(path)
    with open(safe, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='replace').rstrip()
            header_lines.append(line)
            if line.strip() == 'end_header':
                break
        binary_le = any('binary_little_endian' in l for l in header_lines)
        total = int(next(l.split()[-1] for l in header_lines if l.startswith('element vertex')))

        if binary_le:
            type_map = {'float': '<f4', 'uchar': 'u1', 'int': '<i4', 'double': '<f8'}
            dtype_fields = []
            for l in header_lines:
                if not l.startswith('property'):
                    continue
                parts = l.split()
                dtype_fields.append((parts[2], type_map.get(parts[1], '<f4')))
            dt = np.dtype(dtype_fields)
            data = np.frombuffer(f.read(total * dt.itemsize), dtype=dt)
            step = max(1, total // max_pts)
            data = data[::step]
            pts  = np.column_stack([data['x'].astype(np.float32),
                                    data['y'].astype(np.float32),
                                    data['z'].astype(np.float32)])
            cols = np.column_stack([data['red'].astype(np.float32) / 255.0,
                                    data['green'].astype(np.float32) / 255.0,
                                    data['blue'].astype(np.float32) / 255.0])
        else:
            step = max(1, total // max_pts)
            pts_list, cols_list = [], []
            for i in range(total):
                line = f.readline().decode('ascii', errors='replace').split()
                if i % step == 0 and len(line) >= 6:
                    try:
                        pts_list.append([float(line[0]), float(line[1]), float(line[2])])
                        cols_list.append([int(line[3]) / 255.0,
                                          int(line[4]) / 255.0,
                                          int(line[5]) / 255.0])
                    except (ValueError, IndexError):
                        pass
            pts  = np.array(pts_list,  dtype=np.float32) if pts_list  else None
            cols = np.array(cols_list, dtype=np.float32) if cols_list else None

    return pts, cols


def pose_from_trajectory(traj_file):
    safe = _safe_data(traj_file)
    with open(safe) as f:
        t = json.load(f)
    # Prefer ICP-refined if available
    refined = safe.parent / 'trajectory_icp_refined.json'
    if refined.exists():
        try:
            refined = _safe_data(refined)
            with open(refined) as f:
                t = json.load(f)
        except ValueError:
            pass
    lp = t['current_pose']['lidar_pose']
    q = [lp['orientation']['x'], lp['orientation']['y'],
         lp['orientation']['z'], lp['orientation']['w']]
    pos = lp['position']
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = [pos['x'], pos['y'], pos['z']]
    return T


def create_scan_toggle_viewer(session_dir, use_icp=False):
    try:
        session = _safe_data(session_dir)
    except ValueError as e:
        print(f'Error: {e}')
        return None
    scan_dirs = sorted(session.glob('fusion_scan_*'))

    if not scan_dirs:
        print(f'No fusion_scan_* directories found in {session_dir}')
        return False

    # Collect scan data
    scans_js = []
    T_first_inv = None

    for scan_dir in scan_dirs:
        traj_file = scan_dir / 'trajectory.json'
        if not traj_file.exists():
            continue
        try:
            traj_file = _safe_data(traj_file)
        except ValueError:
            continue

        # All colored PLYs are sensor-frame regardless of filename —
        # world_colored_exact.ply is misnamed but contains identical sensor-frame
        # data (exact_match_fusion saves the same coords under both names).
        colored = next((scan_dir / n for n in [
            'sensor_colored_exact.ply', 'sensor_colored.ply',
            'world_colored_exact.ply', 'world_colored.ply',
        ] if (scan_dir / n).exists()), None)
        if colored is not None:
            try:
                colored = _safe_data(colored)
            except ValueError:
                colored = None

        if colored is None:
            print(f'  Skipping {scan_dir.name}: no colored PLY')
            continue

        T_abs = pose_from_trajectory(str(traj_file))
        if T_first_inv is None:
            T_first_inv = np.linalg.inv(T_abs)

        T_rel = T_first_inv @ T_abs

        pts, cols = load_ply_colored(str(colored))
        if pts is None or len(pts) == 0:
            continue

        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts_world = (T_rel @ pts_h.T).T[:, :3]

        pos_js = ','.join(f'{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}' for p in pts_world)
        col_js = ','.join(f'{c[0]:.3f},{c[1]:.3f},{c[2]:.3f}' for c in cols)

        euler = Rotation.from_matrix(T_rel[:3, :3]).as_euler('xyz', degrees=True)
        t = T_rel[:3, 3]
        label = (f'{scan_dir.name} | '
                 f'pos=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f}) | '
                 f'rot=({euler[0]:.0f},{euler[1]:.0f},{euler[2]:.0f})°')

        scans_js.append({
            'name': scan_dir.name,
            'label': label,
            'pts': pos_js,
            'cols': col_js,
            'count': len(pts_world),
        })
        print(f'  Loaded {scan_dir.name}: {len(pts_world)} pts  {colored.name}')

    if not scans_js:
        print('No scans with colored PLY found')
        return False

    # Build HTML
    scans_data_js = json.dumps([{
        'name': s['name'],
        'label': s['label'],
        'count': s['count'],
    } for s in scans_js])

    scan_arrays_js = '\n'.join(
        f'  scanPositions["{s["name"]}"] = new Float32Array([{s["pts"]}]);\n'
        f'  scanColors["{s["name"]}"] = new Float32Array([{s["cols"]}]);'
        for s in scans_js
    )

    html_file = str(_safe_data(session / 'scan_toggle_viewer.html'))

    html = f"""<!DOCTYPE html>
<html>
<head>
<title>Scan Toggle Viewer — {session.name}</title>
<meta http-equiv="Cache-Control" content="no-cache">
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<style>
  * {{ box-sizing:border-box; margin:0; padding:0; }}
  body {{ background:#111; font-family:monospace; color:#eee; overflow:hidden; }}
  #canvas-container {{ width:100vw; height:100vh; }}
  #overlay {{
    position:fixed; top:8px; left:8px; z-index:10;
    background:rgba(20,20,20,0.82); border:1px solid #444;
    border-radius:6px; padding:6px 8px; max-width:calc(100vw - 16px);
    backdrop-filter:blur(4px);
  }}
  #top-bar {{ display:flex; align-items:center; gap:6px; flex-wrap:wrap; margin-bottom:5px; }}
  #top-bar button {{
    background:#333; color:#eee; border:1px solid #555; padding:3px 7px;
    border-radius:3px; cursor:pointer; font-size:11px;
  }}
  #top-bar button:hover {{ background:#555; }}
  #size-row {{ display:flex; align-items:center; gap:5px; font-size:11px; color:#aaa; }}
  #size-row input {{ width:80px; }}
  #scan-grid {{
    display:flex; flex-wrap:wrap; gap:3px; max-height:calc(100vh - 80px);
    overflow-y:auto; padding-right:2px;
  }}
  .sb {{
    width:44px; height:22px; font-size:10px; font-family:monospace;
    border-radius:3px; border:1px solid #555; cursor:pointer;
    background:#2a2a2a; color:#7cf; text-align:center; line-height:20px;
    transition:background 0.1s;
  }}
  .sb.off {{ background:#1a1a1a; color:#555; border-color:#333; }}
  .sb:hover {{ border-color:#aaa; }}
  #status {{ font-size:10px; color:#666; margin-top:4px; }}
</style>
</head>
<body>
<div id="canvas-container"></div>
<div id="overlay">
  <div id="top-bar">
    <b style="font-size:12px;color:#adf">{session.name}</b>
    <button onclick="toggleAll(true)">All</button>
    <button onclick="toggleAll(false)">None</button>
    <button onclick="showOnly(0)">First</button>
    <div id="size-row">sz:<input type="range" min="1" max="100" value="40"
      oninput="updatePointSize(this.value)"><span id="size-val">0.04</span></div>
  </div>
  <div id="scan-grid"></div>
  <div id="status"></div>
</div>

<script>
const scansData = {scans_data_js};
const scanPositions = {{}};
const scanColors = {{}};
{scan_arrays_js}

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.01, 500);
const renderer = new THREE.WebGLRenderer({{antialias:true}});
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('canvas-container').appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const scanMeshes = {{}};
const scanVisible = {{}};

function buildScan(name) {{
  const pos = scanPositions[name];
  const col = scanColors[name];
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  if (col) geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  const mat = new THREE.PointsMaterial({{
    size: 0.04, vertexColors: !!col,
    color: col ? 0xffffff : 0x44aaff, sizeAttenuation: true,
  }});
  return new THREE.Points(geo, mat);
}}

let allPts = [];
scansData.forEach((s, idx) => {{
  const mesh = buildScan(s.name);
  scene.add(mesh);
  scanMeshes[s.name] = mesh;
  scanVisible[s.name] = true;
  const pos = scanPositions[s.name];
  for (let i = 0; i < pos.length; i += 3)
    allPts.push(new THREE.Vector3(pos[i], pos[i+1], pos[i+2]));
}});

const box = new THREE.Box3().setFromPoints(allPts);
const center = box.getCenter(new THREE.Vector3());
const size = box.getSize(new THREE.Vector3());
const maxDim = Math.max(size.x, size.y, size.z);
camera.position.set(center.x, center.y + maxDim * 0.3, center.z + maxDim * 1.5);
controls.target.copy(center);

// Build compact scan grid — one numbered button per scan
const grid = document.getElementById('scan-grid');
const hues = scansData.map((_, i) => Math.round(i * 360 / scansData.length));
scansData.forEach((s, idx) => {{
  const hue = hues[idx];
  const btn = document.createElement('div');
  btn.className = 'sb';
  btn.title = s.label;
  btn.style.borderColor = `hsl(${{hue}},60%,45%)`;
  btn.style.color = `hsl(${{hue}},70%,65%)`;
  btn.textContent = s.name.replace('fusion_scan_', '');
  btn.addEventListener('click', (e) => {{
    if (e.shiftKey) {{ showOnly(idx); return; }}
    const v = !scanVisible[s.name];
    setScanVisible(s.name, v, btn);
  }});

  grid.appendChild(btn);
}});

function setScanVisible(name, visible, btn) {{
  if (scanMeshes[name]) scanMeshes[name].visible = visible;
  scanVisible[name] = visible;
  if (btn) btn.classList.toggle('off', !visible);
  updateStatus();
}}

function toggleAll(visible) {{
  const btns = grid.querySelectorAll('.sb');
  scansData.forEach((s, i) => {{
    setScanVisible(s.name, visible, btns[i]);
  }});
}}

function showOnly(idx) {{
  const btns = grid.querySelectorAll('.sb');
  scansData.forEach((s, i) => {{
    setScanVisible(s.name, i === idx, btns[i]);
  }});
}}

function updateStatus() {{
  const n = Object.values(scanVisible).filter(Boolean).length;
  document.getElementById('status').textContent = n + '/' + scansData.length + ' visible  |  shift+click=isolate';
}}

function updatePointSize(val) {{
  const s = val / 1000;
  document.getElementById('size-val').textContent = s.toFixed(3);
  Object.values(scanMeshes).forEach(m => m.material.size = s);
}}
updateStatus();

window.addEventListener('resize', () => {{
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}});

function animate() {{
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}}
animate();
</script>
</body>
</html>"""

    with open(_safe_data(html_file), 'w') as f:
        f.write(html)

    print(f'\n✓ Scan toggle viewer: {html_file}')
    print(f'  {len(scans_js)} scans loaded')
    print(f'  Shift+click a scan row to isolate it')

    # Open in browser
    import subprocess, time
    html_path = os.path.abspath(html_file)
    file_url = f'file://{html_path}'
    display_env = os.environ.copy()
    display_env.setdefault('DISPLAY', ':0')
    display_env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    wayland = os.environ.get('_ATLAS_WAYLAND_DISPLAY') or os.environ.get('WAYLAND_DISPLAY', '')
    if wayland:
        display_env['WAYLAND_DISPLAY'] = wayland

    if os.environ.get('ATLAS_GUI_MODE'):
        print(f'\u2713 3D viewer ready: {html_path}')
        return html_file

    for browser, args_fn in [
        ('firefox',          lambda u: ['firefox', '--new-window', u]),
        ('chromium-browser', lambda u: ['chromium-browser', '--new-window', u]),
        ('chromium',         lambda u: ['chromium', '--new-window', u]),
        ('google-chrome',    lambda u: ['google-chrome', '--new-window', u]),
        ('xdg-open',         lambda u: ['xdg-open', u]),
    ]:
        try:
            subprocess.Popen(args_fn(file_url),
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL,
                             env=display_env,
                             preexec_fn=os.setpgrp)
            print(f'  Opened in browser: {html_path}')
            return html_file
        except FileNotFoundError:
            continue

    print(f'  Open manually: {html_path}')
    return html_file


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 scan_toggle_viewer.py <session_dir>')
        sys.exit(1)
    try:
        _safe_data(sys.argv[1])
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)
    result = create_scan_toggle_viewer(sys.argv[1])
