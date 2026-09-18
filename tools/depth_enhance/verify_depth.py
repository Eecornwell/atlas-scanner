#!/usr/bin/env python
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# ATLAS Depth Verification Viewer
#
# Reads RGB tiles and depth images directly from a colmap.zip (original or
# enhanced) and generates a self-contained HTML viewer for side-by-side
# comparison. Works with both colmap.zip and colmap_enhanced.zip.
#
# Usage:
#   python verify_depth.py colmap.zip
#   python verify_depth.py colmap_enhanced.zip
#   python verify_depth.py colmap.zip colmap_enhanced.zip   # A/B compare mode

import argparse
import base64
import json
import os
import sys
import webbrowser
import zipfile
from pathlib import Path, PurePosixPath

try:
    import cv2
    import numpy as np
except ImportError:
    print('ERROR: opencv-python and numpy are required.')
    print('Run install.bat first.')
    sys.exit(1)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _jet(depth_u16):
    valid = depth_u16 > 0
    norm  = np.zeros_like(depth_u16, dtype=np.float32)
    if valid.any():
        lo, hi = float(depth_u16[valid].min()), float(depth_u16[valid].max())
        norm[valid] = (depth_u16[valid] - lo) / (hi - lo) if hi > lo else 0.5
    jet = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    jet[~valid] = 20
    return jet, int(depth_u16[valid].min()) if valid.any() else 0, \
                 int(depth_u16[valid].max()) if valid.any() else 0


def _b64jpg(img, q=88):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return base64.b64encode(buf.tobytes()).decode() if ok else ''


def _b64png(img):
    ok, buf = cv2.imencode('.png', img)
    return base64.b64encode(buf.tobytes()).decode() if ok else ''


def _decode_img(data: bytes, flags=cv2.IMREAD_COLOR):
    return cv2.imdecode(np.frombuffer(data, np.uint8), flags)


def _depth_to_rgb(depth_path: str) -> str:
    p = PurePosixPath(depth_path)
    parts = list(p.parts)
    parts[parts.index('depth_images')] = 'images'
    return str(PurePosixPath(*parts))


# ---------------------------------------------------------------------------
# Load pairs from a zip
# ---------------------------------------------------------------------------

def load_pairs(zip_path: Path, max_dim: int, label: str) -> dict:
    """
    Returns data[face][pano] = {rgb, dep_jet, dep_raw, lo_mm, hi_mm, w, h}
    rgb is only loaded from the first zip (shared between A and B).
    """
    data = {}
    with zipfile.ZipFile(zip_path, 'r') as z:
        names = set(z.namelist())
        depth_names = sorted(
            n for n in names if 'depth_images' in n and n.endswith('.png')
        )
        if not depth_names:
            print(f'ERROR: No depth_images found in {zip_path.name}')
            sys.exit(1)

        total = len(depth_names)
        for i, dname in enumerate(depth_names, 1):
            rname = _depth_to_rgb(dname)
            if rname not in names:
                continue

            # Parse face / pano from path e.g. colmap/depth_images/face_00/pano_001.png
            parts = PurePosixPath(dname).parts
            try:
                face = parts[-2]   # face_XX
                pano = PurePosixPath(parts[-1]).stem  # pano_NNN
            except IndexError:
                continue

            rgb_raw = z.read(rname)
            dep_raw = z.read(dname)

            rgb = _decode_img(rgb_raw, cv2.IMREAD_COLOR)
            d16 = _decode_img(dep_raw, cv2.IMREAD_UNCHANGED)
            if rgb is None or d16 is None:
                continue

            h, w = rgb.shape[:2]
            if max(h, w) > max_dim:
                scale = max_dim / max(h, w)
                nw, nh = int(w * scale), int(h * scale)
                rgb = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_AREA)
                d16 = cv2.resize(d16, (nw, nh), interpolation=cv2.INTER_NEAREST)

            jet, lo, hi = _jet(d16)

            print(f'  [{label}] {i}/{total}  {face}/{pano}  '
                  f'{rgb.shape[1]}x{rgb.shape[0]}  {lo}-{hi}mm', end='\r')

            data.setdefault(face, {})[pano] = {
                'rgb':     _b64jpg(rgb),
                'dep_jet': _b64jpg(jet, q=92),
                'dep_raw': _b64png(d16),
                'lo_mm':   lo,
                'hi_mm':   hi,
                'w':       rgb.shape[1],
                'h':       rgb.shape[0],
            }

    print()
    n_pairs = sum(len(v) for v in data.values())
    print(f'  [{label}] {n_pairs} pairs loaded from {zip_path.name}')
    return data


# ---------------------------------------------------------------------------
# HTML generation
# ---------------------------------------------------------------------------

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>ATLAS Depth Verification Viewer</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: #111; color: #eee; font-family: 'Segoe UI', sans-serif;
       display: flex; flex-direction: column; height: 100vh; overflow: hidden; }
#toolbar {
  display: flex; flex-wrap: wrap; align-items: center; gap: 12px;
  padding: 6px 12px; background: #1a1a1a; border-bottom: 1px solid #333;
  font-size: 13px; flex-shrink: 0;
}
#toolbar h2 { font-size: 14px; color: #7cf; white-space: nowrap; }
label { color: #aaa; display: flex; align-items: center; gap: 5px; }
select, input[type=range] { background: #252525; color: #eee;
  border: 1px solid #444; border-radius: 4px; padding: 2px 6px; font-size: 13px; }
#readout { margin-left: auto; font-size: 12px; color: #7fc;
           font-family: monospace; min-width: 260px; text-align: right; }
#panels { flex: 1; display: flex; gap: 4px; padding: 6px;
          overflow: hidden; background: #0d0d0d; }
.panel { flex: 1; display: flex; flex-direction: column; min-width: 0; }
.panel-title { text-align: center; font-size: 11px; color: #888;
               padding: 2px 0 3px; flex-shrink: 0; }
.canvas-wrap { flex: 1; position: relative; overflow: hidden;
               display: flex; align-items: center; justify-content: center;
               background: #0a0a0a; }
.canvas-wrap canvas { display: block; max-width: 100%; max-height: 100%;
                      image-rendering: pixelated; cursor: crosshair; }
.crosshair-h, .crosshair-v { position: absolute; pointer-events: none;
                               background: rgba(255,255,100,0.65); }
.crosshair-h { height: 1px; left: 0; right: 0; }
.crosshair-v { width: 1px; top: 0; bottom: 0; }
#colorbar-wrap { display: flex; flex-direction: column; align-items: center;
                 width: 36px; flex-shrink: 0; padding: 6px 0;
                 justify-content: space-between; }
#cb-hi, #cb-lo { font-size: 10px; color: #888; }
#cb-canvas { flex: 1; width: 18px; margin: 3px 0; border-radius: 2px; }
#ab-badge { font-size: 11px; color: #fa0; display: none; }
</style>
</head>
<body>
<div id="toolbar">
  <h2>ATLAS Depth Viewer</h2>
  <label>Face <select id="sel-face"></select></label>
  <label>Panorama <select id="sel-pano"></select></label>
  <span id="ab-badge">A/B mode</span>
  <div id="readout">hover to read depth</div>
</div>

<div id="panels">
  <div class="panel">
    <div class="panel-title" id="title-left">RGB</div>
    <div class="canvas-wrap" id="wrap-left">
      <canvas id="cv-left"></canvas>
      <div class="crosshair-h" id="ch-lh"></div>
      <div class="crosshair-v" id="ch-lv"></div>
    </div>
  </div>

  <div id="colorbar-wrap">
    <span id="cb-hi">far</span>
    <canvas id="cb-canvas" width="18"></canvas>
    <span id="cb-lo">near</span>
  </div>

  <div class="panel">
    <div class="panel-title" id="title-right">Depth (Turbo)</div>
    <div class="canvas-wrap" id="wrap-right">
      <canvas id="cv-right"></canvas>
      <div class="crosshair-h" id="ch-rh"></div>
      <div class="crosshair-v" id="ch-rv"></div>
    </div>
  </div>
</div>

<script>
const DATA_A  = __DATA_A__;
const DATA_B  = __DATA_B__;
const FACES   = __FACES__;
const PANOS   = __PANOS__;
const AB_MODE = __AB_MODE__;

const selFace  = document.getElementById('sel-face');
const selPano  = document.getElementById('sel-pano');
const readout  = document.getElementById('readout');
const cbHi     = document.getElementById('cb-hi');
const cbLo     = document.getElementById('cb-lo');
const cbCanvas = document.getElementById('cb-canvas');
const abBadge  = document.getElementById('ab-badge');
const titleL   = document.getElementById('title-left');
const titleR   = document.getElementById('title-right');

if (AB_MODE) {
  abBadge.style.display = 'inline';
  titleL.textContent = 'Depth A (original)';
  titleR.textContent = 'Depth B (enhanced)';
} else {
  titleL.textContent = 'RGB';
  titleR.textContent = 'Depth (Turbo colormap)';
}

FACES.forEach(f => selFace.add(new Option(f, f)));
PANOS.forEach(p => selPano.add(new Option(p, p)));

const cvL  = document.getElementById('cv-left');
const cvR  = document.getElementById('cv-right');
const ctxL = cvL.getContext('2d');
const ctxR = cvR.getContext('2d');

let rawL = null, rawR = null;
let imgW = 0, imgH = 0;
let curEntry = null;

function drawColorbar(loMm, hiMm) {
  const h = Math.max(cbCanvas.parentElement.clientHeight - 50, 40);
  cbCanvas.height = h;
  const ctx  = cbCanvas.getContext('2d');
  const grad = ctx.createLinearGradient(0, 0, 0, h);
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
  ctx.fillRect(0, 0, 18, h);
  cbHi.textContent = (hiMm / 1000).toFixed(2) + 'm';
  cbLo.textContent = (loMm / 1000).toFixed(2) + 'm';
}

function loadImg(b64, mime) {
  return new Promise(r => {
    const img = new Image();
    img.onload = () => r(img);
    img.src = `data:image/${mime};base64,${b64}`;
  });
}

async function decodeDepth(b64, w, h) {
  const bin   = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const blob = new Blob([bytes], {type: 'image/png'});
  const url  = URL.createObjectURL(blob);
  const img  = new Image();
  await new Promise(r => { img.onload = r; img.src = url; });
  URL.revokeObjectURL(url);
  const off  = new OffscreenCanvas(w, h);
  const ctx  = off.getContext('2d');
  ctx.drawImage(img, 0, 0);
  const px  = ctx.getImageData(0, 0, w, h).data;
  const out = new Uint16Array(w * h);
  for (let i = 0; i < w * h; i++)
    out[i] = (px[i * 4] << 8) | px[i * 4 + 1];
  return out;
}

async function render() {
  const face  = selFace.value;
  const pano  = selPano.value;
  const entA  = DATA_A[face] && DATA_A[face][pano];
  const entB  = DATA_B[face] && DATA_B[face][pano];
  if (!entA) { readout.textContent = 'No data'; return; }
  curEntry = entA;
  imgW = entA.w; imgH = entA.h;

  cvL.width = imgW; cvL.height = imgH;
  cvR.width = imgW; cvR.height = imgH;

  if (AB_MODE) {
    // Left = depth A (original), Right = depth B (enhanced)
    const [imgJetA, imgJetB] = await Promise.all([
      loadImg(entA.dep_jet, 'jpeg'),
      entB ? loadImg(entB.dep_jet, 'jpeg') : Promise.resolve(null),
    ]);
    ctxL.drawImage(imgJetA, 0, 0);
    if (imgJetB) ctxR.drawImage(imgJetB, 0, 0);
    rawL = await decodeDepth(entA.dep_raw, imgW, imgH);
    rawR = entB ? await decodeDepth(entB.dep_raw, imgW, imgH) : null;
    const lo = Math.min(entA.lo_mm, entB ? entB.lo_mm : entA.lo_mm);
    const hi = Math.max(entA.hi_mm, entB ? entB.hi_mm : entA.hi_mm);
    drawColorbar(lo, hi);
    readout.textContent = `A: ${entA.lo_mm}–${entA.hi_mm}mm  B: ${entB ? entB.lo_mm+'–'+entB.hi_mm+'mm' : 'n/a'}`;
  } else {
    // Left = RGB, Right = depth
    const [imgRgb, imgJet] = await Promise.all([
      loadImg(entA.rgb,     'jpeg'),
      loadImg(entA.dep_jet, 'jpeg'),
    ]);
    ctxL.drawImage(imgRgb, 0, 0);
    ctxR.drawImage(imgJet, 0, 0);
    rawL = null;
    rawR = await decodeDepth(entA.dep_raw, imgW, imgH);
    drawColorbar(entA.lo_mm, entA.hi_mm);
    readout.textContent = `${entA.lo_mm}–${entA.hi_mm} mm  (${(entA.lo_mm/1000).toFixed(2)}–${(entA.hi_mm/1000).toFixed(2)} m)`;
  }
}

function canvasCoords(canvas, e) {
  const rect = canvas.getBoundingClientRect();
  return {
    px: Math.floor((e.clientX - rect.left) * canvas.width  / rect.width),
    py: Math.floor((e.clientY - rect.top)  * canvas.height / rect.height),
    fx: (e.clientX - rect.left) / rect.width,
    fy: (e.clientY - rect.top)  / rect.height,
  };
}

function setCrosshairs(fx, fy) {
  for (const [h, v] of [
    [document.getElementById('ch-lh'), document.getElementById('ch-lv')],
    [document.getElementById('ch-rh'), document.getElementById('ch-rv')],
  ]) {
    h.style.top  = (fy * 100).toFixed(2) + '%';
    v.style.left = (fx * 100).toFixed(2) + '%';
  }
}

function onHover(e, canvas, raw) {
  const {px, py, fx, fy} = canvasCoords(canvas, e);
  setCrosshairs(fx, fy);
  if (!raw || px < 0 || px >= imgW || py < 0 || py >= imgH) return;
  const mm = raw[py * imgW + px];
  if (AB_MODE) {
    const mmA = rawL ? rawL[py * imgW + px] : 0;
    const mmB = rawR ? rawR[py * imgW + px] : 0;
    readout.textContent =
      `x=${px} y=${py}  A: ${mmA > 0 ? mmA+'mm' : '—'}  B: ${mmB > 0 ? mmB+'mm' : '—'}` +
      (mmA > 0 && mmB > 0 ? `  Δ=${mmB - mmA}mm` : '');
  } else {
    readout.textContent = mm > 0
      ? `x=${px} y=${py}  depth: ${mm} mm  (${(mm/1000).toFixed(3)} m)`
      : `x=${px} y=${py}  depth: — (no data)`;
  }
}

cvL.addEventListener('mousemove', e => onHover(e, cvL, AB_MODE ? rawL : null));
cvR.addEventListener('mousemove', e => onHover(e, cvR, AB_MODE ? rawR : rawR));

selFace.addEventListener('change', render);
selPano.addEventListener('change', render);
window.addEventListener('resize', () => { if (curEntry) drawColorbar(curEntry.lo_mm, curEntry.hi_mm); });

render();
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Generate a depth verification viewer from colmap.zip.\n'
                    'Pass two zips for A/B comparison (original vs enhanced).'
    )
    parser.add_argument('zip_a', type=Path,
                        help='colmap.zip (or colmap_enhanced.zip for single view)')
    parser.add_argument('zip_b', type=Path, nargs='?', default=None,
                        help='Second zip for A/B comparison (optional)')
    parser.add_argument('--max-dim', type=int, default=768,
                        help='Max tile dimension in viewer (default: 768)')
    parser.add_argument('--output', type=Path, default=None,
                        help='Output HTML path (default: next to first zip)')
    parser.add_argument('--no-open', action='store_true',
                        help='Do not open browser automatically')
    args = parser.parse_args()

    if not args.zip_a.exists():
        print(f'ERROR: {args.zip_a} not found.')
        sys.exit(1)
    if args.zip_b and not args.zip_b.exists():
        print(f'ERROR: {args.zip_b} not found.')
        sys.exit(1)

    ab_mode = args.zip_b is not None

    print(f'Loading {"A: " if ab_mode else ""}{args.zip_a.name}...')
    data_a = load_pairs(args.zip_a, args.max_dim, 'A')

    if ab_mode:
        print(f'Loading B: {args.zip_b.name}...')
        data_b = load_pairs(args.zip_b, args.max_dim, 'B')
        # In A/B mode we don't need RGB in data_b (shared from A)
        for face in data_b:
            for pano in data_b[face]:
                data_b[face][pano].pop('rgb', None)
    else:
        data_b = {}

    faces = sorted(data_a.keys())
    panos = sorted({p for f in data_a.values() for p in f.keys()})

    html = HTML_TEMPLATE \
        .replace('__DATA_A__', json.dumps(data_a)) \
        .replace('__DATA_B__', json.dumps(data_b)) \
        .replace('__FACES__',  json.dumps(faces)) \
        .replace('__PANOS__',  json.dumps(panos)) \
        .replace('__AB_MODE__', 'true' if ab_mode else 'false')

    if args.output:
        out = args.output.resolve()
    elif ab_mode:
        out = args.zip_a.with_name('depth_compare.html').resolve()
    else:
        out = args.zip_a.with_suffix('.html').with_stem(
            args.zip_a.stem + '_viewer').resolve()

    out.write_text(html, encoding='utf-8')
    print(f'\n✓ Viewer: {out}')

    if not args.no_open:
        webbrowser.open(out.resolve().as_uri())


if __name__ == '__main__':
    main()
