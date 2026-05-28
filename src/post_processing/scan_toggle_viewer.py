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


def load_ply_colored(path, max_pts=8000):
    pts, cols = [], []
    with open(path, 'rb') as f:
        total = 0
        while True:
            line = f.readline().decode('ascii', errors='replace')
            if 'end_header' in line:
                break
            if 'element vertex' in line:
                total = int(line.split()[-1])
        step = max(1, total // max_pts)
        for i in range(total):
            parts = f.readline().decode('ascii', errors='replace').split()
            if i % step == 0 and len(parts) >= 6:
                try:
                    pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    cols.append([int(parts[3]) / 255.0,
                                 int(parts[4]) / 255.0,
                                 int(parts[5]) / 255.0])
                except Exception:
                    pass
    return np.array(pts) if pts else None, np.array(cols) if cols else None


def pose_from_trajectory(traj_file):
    with open(traj_file) as f:
        t = json.load(f)
    # Prefer ICP-refined if available
    refined = Path(traj_file).parent / 'trajectory_icp_refined.json'
    if refined.exists():
        with open(refined) as f:
            t = json.load(f)
    lp = t['current_pose']['lidar_pose']
    q = [lp['orientation']['x'], lp['orientation']['y'],
         lp['orientation']['z'], lp['orientation']['w']]
    pos = lp['position']
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = [pos['x'], pos['y'], pos['z']]
    return T


def create_scan_toggle_viewer(session_dir, use_icp=False):
    session = Path(session_dir)
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

        # Find colored PLY - prefer world_colored (already in world frame)
        colored = next((scan_dir / n for n in [
            'world_colored_exact.ply', 'world_colored.ply',
            'sensor_colored_exact.ply', 'sensor_colored.ply'
        ] if (scan_dir / n).exists()), None)

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

        # world_colored_exact.ply and sensor_colored_exact.ply both contain
        # sensor-frame XYZ — the world transform is applied here via T_rel.
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

    html_file = str(session / 'scan_toggle_viewer.html')

    html = f"""<!DOCTYPE html>
<html>
<head>
<title>Scan Toggle Viewer — {session.name}</title>
<meta http-equiv="Cache-Control" content="no-cache">
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<style>
  body {{ margin:0; background:#111; font-family:monospace; color:#eee; display:flex; }}
  #panel {{ width:320px; min-width:320px; padding:10px; overflow-y:auto; background:#1a1a1a;
            border-right:1px solid #333; font-size:12px; }}
  #canvas-container {{ flex:1; position:relative; }}
  h3 {{ margin:4px 0; color:#adf; font-size:13px; }}
  .scan-row {{ margin:4px 0; padding:4px 6px; background:#222; border-radius:4px;
               border-left:3px solid #555; cursor:pointer; }}
  .scan-row:hover {{ background:#2a2a2a; }}
  .scan-row label {{ cursor:pointer; display:block; }}
  .scan-name {{ color:#7cf; font-weight:bold; }}
  .scan-info {{ color:#888; font-size:10px; margin-top:2px; }}
  #controls {{ margin-top:10px; padding:6px; background:#222; border-radius:4px; }}
  button {{ background:#333; color:#eee; border:1px solid #555; padding:4px 8px;
            border-radius:3px; cursor:pointer; margin:2px; font-size:11px; }}
  button:hover {{ background:#444; }}
  #status {{ margin-top:8px; color:#888; font-size:11px; }}
</style>
</head>
<body>
<div id="panel">
  <h3>Scan Toggle Viewer</h3>
  <div style="color:#888;font-size:10px;margin-bottom:8px">{session.name}<br>Trajectory poses only (no ICP)</div>
  <div id="controls">
    <button onclick="toggleAll(true)">Show All</button>
    <button onclick="toggleAll(false)">Hide All</button>
    <button onclick="showOnly(0)">First Only</button>
    <div style="margin-top:6px">
      <label style="font-size:11px">Point size: <span id="size-val">0.04</span></label><br>
      <input type="range" min="1" max="100" value="40" style="width:100%"
             oninput="updatePointSize(this.value)">
    </div>
  </div>
  <div id="scan-list" style="margin-top:8px"></div>
  <div id="status"></div>
</div>
<div id="canvas-container"></div>

<script>
const scansData = {scans_data_js};
const scanPositions = {{}};
const scanColors = {{}};
{scan_arrays_js}

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 500);
const renderer = new THREE.WebGLRenderer({{antialias:true}});
const container = document.getElementById('canvas-container');
renderer.setSize(container.clientWidth || window.innerWidth - 320, window.innerHeight);
container.appendChild(renderer.domElement);

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
    size: 0.04,
    vertexColors: !!col,
    color: col ? 0xffffff : 0x44aaff,
    sizeAttenuation: true,
  }});
  return new THREE.Points(geo, mat);
}}

// Build all scans and add to scene
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

// Center camera
const box = new THREE.Box3().setFromPoints(allPts);
const center = box.getCenter(new THREE.Vector3());
const size = box.getSize(new THREE.Vector3());
const maxDim = Math.max(size.x, size.y, size.z);
camera.position.set(center.x, center.y + maxDim * 0.3, center.z + maxDim * 1.5);
controls.target.copy(center);

// Build UI
const list = document.getElementById('scan-list');
const hues = scansData.map((_, i) => Math.round(i * 360 / scansData.length));
scansData.forEach((s, idx) => {{
  const hue = hues[idx];
  const row = document.createElement('div');
  row.className = 'scan-row';
  row.style.borderLeftColor = `hsl(${{hue}},70%,50%)`;
  row.innerHTML = `
    <label>
      <input type="checkbox" checked onchange="setScanVisible('${{s.name}}', this.checked)">
      <span class="scan-name">${{s.name}}</span>
      <span style="color:hsl(${{hue}},70%,60%);font-size:10px"> ● ${{s.count}} pts</span>
      <div class="scan-info">${{s.label.split('|').slice(1).join('|')}}</div>
    </label>`;
  row.addEventListener('dblclick', () => showOnly(idx));
  list.appendChild(row);
  // Color the mesh by scan index for easy identification when toggling
  // (keep vertex colors as primary, but allow override)
}});

function setScanVisible(name, visible) {{
  if (scanMeshes[name]) scanMeshes[name].visible = visible;
  scanVisible[name] = visible;
  updateStatus();
}}

function toggleAll(visible) {{
  scansData.forEach(s => {{
    setScanVisible(s.name, visible);
    const cb = list.querySelectorAll('input[type=checkbox]');
    cb.forEach(c => c.checked = visible);
  }});
}}

function showOnly(idx) {{
  scansData.forEach((s, i) => {{
    const v = (i === idx);
    setScanVisible(s.name, v);
  }});
  const cbs = list.querySelectorAll('input[type=checkbox]');
  cbs.forEach((c, i) => c.checked = (i === idx));
}}

function updateStatus() {{
  const n = Object.values(scanVisible).filter(Boolean).length;
  document.getElementById('status').textContent = `${{n}}/${{scansData.length}} scans visible`;
}}

function updatePointSize(val) {{
  const size = val / 1000;
  document.getElementById('size-val').textContent = size.toFixed(3);
  Object.values(scanMeshes).forEach(m => m.material.size = size);
}}
updateStatus();

window.addEventListener('resize', () => {{
  const w = container.clientWidth || window.innerWidth - 320;
  camera.aspect = w / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(w, window.innerHeight);
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

    with open(html_file, 'w') as f:
        f.write(html)

    print(f'\n✓ Scan toggle viewer: {html_file}')
    print(f'  {len(scans_js)} scans loaded')
    print(f'  Double-click a scan row to isolate it')

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
    result = create_scan_toggle_viewer(sys.argv[1])
    if result and os.environ.get('ATLAS_GUI_MODE'):
        print(f'✓ 3D viewer ready: {result}')
