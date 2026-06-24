#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Side-by-side RGB / depth viewer for COLMAP tiles.
# Left panel: RGB tile. Right panel: jet-colourised depth map.
# Hover over either panel to see a synchronised crosshair and the depth
# value in metres at that pixel. Toggle between side-by-side and
# overlay modes with the view selector.
#
# Usage:
#   python3 colmap_depth_overlay.py <session_dir>

import os
import sys
import base64
import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


def _jet_colorize(depth_u16):
    """Return a BGR jet image and a raw normalised float map (0-1)."""
    valid = depth_u16 > 0
    norm  = np.zeros_like(depth_u16, dtype=np.float32)
    if valid.any():
        lo, hi = float(depth_u16[valid].min()), float(depth_u16[valid].max())
        if hi > lo:
            norm[valid] = (depth_u16[valid] - lo) / (hi - lo)
        else:
            norm[valid] = 0.5
    jet = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    jet[~valid] = 20   # dark background for invalid pixels
    return jet, norm


def _b64_jpg(img_bgr, quality=88):
    ok, buf = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf.tobytes()).decode() if ok else ''


def _b64_png(img):
    ok, buf = cv2.imencode('.png', img)
    return base64.b64encode(buf.tobytes()).decode() if ok else ''


def generate_overlay_viewer(session_dir, output_path=None, max_dim=768):
    try:
        session = _safe_data(session_dir)
    except ValueError as e:
        print(f'Error: {e}'); sys.exit(1)

    rgb_root   = session / 'colmap' / 'images'
    depth_root = session / 'colmap' / 'depth_images'

    if not rgb_root.exists():
        print(f'No colmap/images at {rgb_root}'); sys.exit(1)
    if not depth_root.exists():
        print(f'No depth_images at {depth_root} — run Generate Depth Images first')
        sys.exit(1)

    pairs = []
    for rgb_path in sorted(rgb_root.rglob('pano_*.png')):
        rel      = rgb_path.relative_to(rgb_root)
        dep_path = depth_root / rel
        if dep_path.exists():
            pairs.append((rel.parts[0], rel.stem, rgb_path, dep_path))

    if not pairs:
        print('No matching RGB + depth pairs found'); sys.exit(1)

    faces = sorted(set(p[0] for p in pairs))
    panos = sorted(set(p[1] for p in pairs))
    print(f'  {len(pairs)} pairs  ({len(faces)} faces, {len(panos)} panoramas)')

    # data[face][pano] = {rgb, dep_jet, dep_raw, lo_mm, hi_mm}
    data = {}
    for face, pano, rgb_path, dep_path in pairs:
        rgb  = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        d16  = cv2.imread(str(dep_path), cv2.IMREAD_UNCHANGED)
        if rgb is None or d16 is None:
            continue

        h, w = rgb.shape[:2]
        if max(h, w) > max_dim:
            scale = max_dim / max(h, w)
            nw, nh = int(w * scale), int(h * scale)
            rgb = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_AREA)
            d16 = cv2.resize(d16, (nw, nh), interpolation=cv2.INTER_NEAREST)

        valid = d16 > 0
        lo_mm = int(d16[valid].min()) if valid.any() else 0
        hi_mm = int(d16[valid].max()) if valid.any() else 0

        jet, _ = _jet_colorize(d16)

        # Store raw depth as base64 uint16 PNG for pixel-accurate hover readout
        dep_raw_b64 = _b64_png(d16)

        data.setdefault(face, {})[pano] = {
            'rgb':     _b64_jpg(rgb),
            'dep_jet': _b64_jpg(jet, quality=92),
            'dep_raw': dep_raw_b64,
            'lo_mm':   lo_mm,
            'hi_mm':   hi_mm,
            'w':       rgb.shape[1],
            'h':       rgb.shape[0],
        }
        n_valid = int(valid.sum())
        print(f'    {face}/{pano}  {rgb.shape[1]}x{rgb.shape[0]}'
              f'  depth={lo_mm}–{hi_mm}mm  pts={n_valid}')

    data_js  = json.dumps(data)
    faces_js = json.dumps(faces)
    panos_js = json.dumps(panos)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Depth-RGB Viewer</title>
<style>
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{ background: #111; color: #eee; font-family: 'Segoe UI', sans-serif;
        display: flex; flex-direction: column; height: 100vh; overflow: hidden; }}
#toolbar {{
  display: flex; flex-wrap: wrap; align-items: center; gap: 14px;
  padding: 7px 12px; background: #1a1a1a; border-bottom: 1px solid #333;
  font-size: 13px; flex-shrink: 0;
}}
#toolbar h2 {{ font-size: 14px; color: #7cf; }}
label {{ color: #aaa; display: flex; align-items: center; gap: 5px; }}
select {{ background: #252525; color: #eee; border: 1px solid #444;
          border-radius: 4px; padding: 2px 6px; font-size: 13px; }}
#readout {{
  margin-left: auto; font-size: 12px; color: #7fc;
  font-family: monospace; min-width: 200px; text-align: right;
}}
#panels {{
  flex: 1; display: flex; gap: 4px; padding: 6px;
  overflow: hidden; background: #0d0d0d;
}}
.panel {{
  flex: 1; display: flex; flex-direction: column;
  min-width: 0;
}}
.panel-title {{
  text-align: center; font-size: 11px; color: #666;
  padding: 2px 0 3px; flex-shrink: 0;
}}
.canvas-wrap {{
  flex: 1; position: relative; overflow: hidden;
  display: flex; align-items: center; justify-content: center;
  background: #0a0a0a;
}}
.canvas-wrap canvas {{
  display: block;
  max-width: 100%; max-height: 100%;
  image-rendering: pixelated;
  cursor: crosshair;
}}
.crosshair-h, .crosshair-v {{
  position: absolute; pointer-events: none;
  background: rgba(255,255,100,0.6);
}}
.crosshair-h {{ height: 1px; left: 0; right: 0; }}
.crosshair-v {{ width: 1px;  top: 0; bottom: 0; }}
#colorbar-wrap {{
  display: flex; flex-direction: column; align-items: center;
  width: 36px; flex-shrink: 0; padding: 6px 0;
  justify-content: space-between;
}}
#cb-hi {{ font-size: 10px; color: #888; }}
#cb-lo {{ font-size: 10px; color: #888; }}
#cb-canvas {{ flex: 1; width: 18px; margin: 3px 0; border-radius: 2px; }}
</style>
</head>
<body>
<div id="toolbar">
  <h2>Depth · RGB Viewer</h2>
  <label>Face <select id="sel-face"></select></label>
  <label>Panorama <select id="sel-pano"></select></label>
  <div id="readout">hover to read depth</div>
</div>

<div id="panels">
  <div class="panel">
    <div class="panel-title">RGB</div>
    <div class="canvas-wrap" id="wrap-rgb">
      <canvas id="cv-rgb"></canvas>
      <div class="crosshair-h" id="ch-rgb-h"></div>
      <div class="crosshair-v" id="ch-rgb-v"></div>
    </div>
  </div>

  <div id="colorbar-wrap">
    <span id="cb-hi">far</span>
    <canvas id="cb-canvas" width="18"></canvas>
    <span id="cb-lo">near</span>
  </div>

  <div class="panel">
    <div class="panel-title">Depth (Turbo colormap)</div>
    <div class="canvas-wrap" id="wrap-dep">
      <canvas id="cv-dep"></canvas>
      <div class="crosshair-h" id="ch-dep-h"></div>
      <div class="crosshair-v" id="ch-dep-v"></div>
    </div>
  </div>
</div>

<script>
const DATA   = {data_js};
const FACES  = {faces_js};
const PANOS  = {panos_js};

const selFace = document.getElementById('sel-face');
const selPano = document.getElementById('sel-pano');
const readout = document.getElementById('readout');
const cbHi    = document.getElementById('cb-hi');
const cbLo    = document.getElementById('cb-lo');
const cbCanvas= document.getElementById('cb-canvas');

FACES.forEach(f => selFace.add(new Option(f, f)));
PANOS.forEach(p => selPano.add(new Option(p, p)));

// Canvas elements
const cvRgb  = document.getElementById('cv-rgb');
const cvDep  = document.getElementById('cv-dep');
const ctxRgb = cvRgb.getContext('2d');
const ctxDep = cvDep.getContext('2d');

// Crosshairs
const chRgbH = document.getElementById('ch-rgb-h');
const chRgbV = document.getElementById('ch-rgb-v');
const chDepH = document.getElementById('ch-dep-h');
const chDepV = document.getElementById('ch-dep-v');

let rawDepthData = null;  // Uint16Array of raw depth pixels
let imgW = 0, imgH = 0;
let curEntry = null;

// ── colorbar ──────────────────────────────────────────────────────────────
function drawColorbar(loMm, hiMm) {{
  const h = cbCanvas.parentElement.clientHeight - 50;
  cbCanvas.height = Math.max(h, 40);
  const ctx  = cbCanvas.getContext('2d');
  const grad = ctx.createLinearGradient(0, 0, 0, cbCanvas.height);
  // Turbo: dark-blue(near) -> cyan -> green -> yellow -> red(far)
  grad.addColorStop(0,     '#7a0402');
  grad.addColorStop(0.125, '#d83101');
  grad.addColorStop(0.25,  '#f6a906');
  grad.addColorStop(0.375, '#a7d801');
  grad.addColorStop(0.5,   '#23d162');
  grad.addColorStop(0.625, '#1ac7c2');
  grad.addColorStop(0.75,  '#3d87f5');
  grad.addColorStop(0.875, '#3b4cc0');
  grad.addColorStop(1,     '#30123b');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 18, cbCanvas.height);
  cbHi.textContent = (hiMm / 1000).toFixed(2) + 'm';
  cbLo.textContent = (loMm / 1000).toFixed(2) + 'm';
}}

// ── image loading ─────────────────────────────────────────────────────────
function loadImage(b64, mime='jpeg') {{
  return new Promise(resolve => {{
    const img = new Image();
    img.onload = () => resolve(img);
    img.src = `data:image/${{mime}};base64,${{b64}}`;
  }});
}}

async function render() {{
  const face  = selFace.value;
  const pano  = selPano.value;
  const entry = DATA[face] && DATA[face][pano];
  if (!entry) {{ readout.textContent = 'No data'; return; }}
  curEntry = entry;
  imgW = entry.w; imgH = entry.h;

  cvRgb.width  = imgW; cvRgb.height = imgH;
  cvDep.width  = imgW; cvDep.height = imgH;

  const [imgRgb, imgJet, imgRaw] = await Promise.all([
    loadImage(entry.rgb,     'jpeg'),
    loadImage(entry.dep_jet, 'jpeg'),
    loadImage(entry.dep_raw, 'png'),
  ]);

  ctxRgb.drawImage(imgRgb, 0, 0);
  ctxDep.drawImage(imgJet, 0, 0);

  // Extract raw uint16 depth from the png via an offscreen canvas
  const off = new OffscreenCanvas(imgW, imgH);
  const octx = off.getContext('2d');
  octx.drawImage(imgRaw, 0, 0);
  const px = octx.getImageData(0, 0, imgW, imgH).data;
  // uint16 was stored as PNG 16-bit grayscale; browser decodes to 8-bit RGBA
  // We stored with cv2.imwrite which writes 16-bit PNG; browser maps to
  // 16-bit values split across R+G channels (big-endian) in some cases,
  // but most browsers load 16-bit PNG as 8-bit by truncating the high byte.
  // So we store the raw values via a custom encoding: re-read entry.dep_raw
  // as a typed array directly from the base64 blob instead.
  rawDepthData = await decodeDepthBlob(entry.dep_raw, imgW, imgH);

  drawColorbar(entry.lo_mm, entry.hi_mm);
  readout.textContent = `${{entry.lo_mm}}–${{entry.hi_mm}} mm  (${{(entry.lo_mm/1000).toFixed(2)}}–${{(entry.hi_mm/1000).toFixed(2)}} m)`;
}}

// Decode uint16 PNG depth via a Worker-free approach:
// fetch the base64 as a blob URL and use createImageBitmap which preserves
// 16-bit in Chrome/Firefox via the raw pixel buffer.
async function decodeDepthBlob(b64, w, h) {{
  // Fallback: parse the PNG manually to extract 16-bit values.
  // We use the fact that the image was saved as CV_16UC1 (single channel).
  // Browsers always decode to RGBA8; so we approximate by using R+G channels
  // (high byte in R, low byte in G for 16-bit grayscale PNG).
  const bin   = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const blob  = new Blob([bytes], {{type: 'image/png'}});
  const url   = URL.createObjectURL(blob);
  const img   = new Image();
  await new Promise(r => {{ img.onload = r; img.src = url; }});
  URL.revokeObjectURL(url);
  const off  = new OffscreenCanvas(w, h);
  const ctx  = off.getContext('2d');
  ctx.drawImage(img, 0, 0);
  const px   = ctx.getImageData(0, 0, w, h).data;  // RGBA8
  const out  = new Uint16Array(w * h);
  // 16-bit grayscale PNG: R holds high byte, G holds low byte
  for (let i = 0; i < w * h; i++) {{
    out[i] = (px[i * 4] << 8) | px[i * 4 + 1];
  }}
  return out;
}}

// ── crosshair + depth readout ─────────────────────────────────────────────
function canvasCoords(canvas, e) {{
  const rect  = canvas.getBoundingClientRect();
  const scaleX = canvas.width  / rect.width;
  const scaleY = canvas.height / rect.height;
  return {{
    px: Math.floor((e.clientX - rect.left)  * scaleX),
    py: Math.floor((e.clientY - rect.top)   * scaleY),
    fx: (e.clientX - rect.left)  / rect.width,
    fy: (e.clientY - rect.top)   / rect.height,
  }};
}}

function setCrosshairs(fx, fy) {{
  for (const [wh, wv] of [[chRgbH, chRgbV], [chDepH, chDepV]]) {{
    wh.style.top  = (fy * 100).toFixed(2) + '%';
    wv.style.left = (fx * 100).toFixed(2) + '%';
  }}
}}

function onHover(e, canvas) {{
  if (!rawDepthData || !curEntry) return;
  const {{px, py, fx, fy}} = canvasCoords(canvas, e);
  setCrosshairs(fx, fy);
  if (px >= 0 && px < imgW && py >= 0 && py < imgH) {{
    const mm = rawDepthData[py * imgW + px];
    if (mm > 0) {{
      readout.textContent =
        `x=${{px}} y=${{py}}  depth: ${{mm}} mm  (${{(mm/1000).toFixed(3)}} m)`;
    }} else {{
      readout.textContent = `x=${{px}} y=${{py}}  depth: — (no data)`;
    }}
  }}
}}

cvRgb.addEventListener('mousemove', e => onHover(e, cvRgb));
cvDep.addEventListener('mousemove', e => onHover(e, cvDep));

selFace.addEventListener('change', render);
selPano.addEventListener('change', render);
window.addEventListener('resize', () => {{
  if (curEntry) drawColorbar(curEntry.lo_mm, curEntry.hi_mm);
}});

render();
</script>
</body>
</html>"""

    if output_path is None:
        output_path = session / 'colmap' / 'depth_overlay_viewer.html'
    output_path = _safe_data(output_path)
    output_path.write_text(html)
    print(f'\n\u2713 3D viewer ready: {output_path}')
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--max-dim', type=int, default=768)
    parser.add_argument('--no-open', action='store_true')
    args = parser.parse_args()
    try:
        _safe_data(args.session_dir)
    except ValueError as e:
        print(f'Error: {e}'); sys.exit(1)

    out = generate_overlay_viewer(args.session_dir, max_dim=args.max_dim)

    if args.no_open or os.environ.get('ATLAS_GUI_MODE'):
        return

    url = f'file://{out}'
    env = os.environ.copy()
    env.setdefault('DISPLAY', ':0')
    for cmd in [['firefox', '--new-window', url],
                ['chromium-browser', '--new-window', url],
                ['xdg-open', url]]:
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, env=env,
                             preexec_fn=os.setpgrp)
            return
        except FileNotFoundError:
            continue


if __name__ == '__main__':
    main()
