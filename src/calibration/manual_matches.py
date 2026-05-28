#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Side-by-side manual match picker for camera ERP (left) and lidar
# intensity image (right). Right-click to place a point on each side alternately;
# matched pairs are written into the _matches.json format consumed by initial_guess_auto.
#
# Usage:
#   python3 manual_matches.py <output_dir> [--pair 000001]
#
# Controls:
#   Right-click left panel  -> place camera keypoint (shown in green)
#   Right-click right panel -> place lidar keypoint and complete the pair (shown in red)
#   Middle-click / Backspace -> undo last point
#   d                        -> delete nearest completed pair (click near it)
#   s / Enter                -> save and exit
#   q / Escape               -> quit without saving
#   scroll wheel             -> zoom in/out
#   left-click drag          -> pan

import sys
import json
import argparse
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path
from PIL import Image, ImageTk, ImageDraw, ImageFont


POINT_RADIUS = 6
CAM_COLOR  = '#00ff00'   # green  - camera keypoints
LID_COLOR  = '#ff4444'   # red    - lidar keypoints
LINE_COLOR = '#ffff00'   # yellow - match lines
PENDING_COLOR = '#00aaff' # blue  - waiting for second point


class ManualMatchTool:
    def __init__(self, root, output_dir: Path, pair_name: str):
        self.root = root
        self.output_dir = output_dir
        self.pair_name = pair_name

        # Load images
        cam_path = output_dir / f'{pair_name}.png'
        lid_path = output_dir / f'{pair_name}_lidar_intensities.png'
        if not cam_path.exists():
            raise FileNotFoundError(f'Camera image not found: {cam_path}')
        if not lid_path.exists():
            raise FileNotFoundError(f'Lidar image not found: {lid_path}')

        self.cam_img_orig = Image.open(cam_path).convert('RGB')
        lid_gray = Image.open(lid_path).convert('L')
        # Colorise lidar with inferno colormap for better visibility
        lid_arr = np.array(lid_gray, dtype=np.uint8)
        import cv2
        lid_color = cv2.applyColorMap(lid_arr, cv2.COLORMAP_INFERNO)
        lid_color = cv2.cvtColor(lid_color, cv2.COLOR_BGR2RGB)
        self.lid_img_orig = Image.fromarray(lid_color)

        self.img_w, self.img_h = self.cam_img_orig.size

        # Match state
        # completed: list of (cam_xy, lid_xy) in original image coords
        self.completed = []
        # pending: camera point waiting for lidar point
        self.pending_cam = None

        # View state (same zoom/pan applied to both panels)
        self.zoom = 1.0
        self.pan_x = 0.0   # offset in image pixels
        self.pan_y = 0.0
        self._drag_start = None
        self._drag_pan_start = None

        # Load existing manual matches if any
        self._matches_path = output_dir / f'{pair_name}_matches.json'
        self._existing = self._load_existing()

        self._build_ui()
        self._fit_to_window()
        self.root.after(100, self._render)

    def _load_existing(self):
        """Load existing SuperGlue matches to display as background."""
        if not self._matches_path.exists():
            return []
        with open(self._matches_path) as f:
            d = json.load(f)
        kpts0 = np.array(d['kpts0']).reshape(-1, 2)
        kpts1 = np.array(d['kpts1']).reshape(-1, 2)
        pairs = []
        for i, m in enumerate(d['matches']):
            if m >= 0:
                pairs.append((tuple(kpts0[i]), tuple(kpts1[m])))
        return pairs

    def _build_ui(self):
        self.root.title(f'Manual Match Picker — {self.pair_name}')
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.root.geometry(f'{min(1600, sw-40)}x{min(900, sh-60)}+20+20')

        # Top toolbar
        toolbar = ttk.Frame(self.root)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=2)
        ttk.Label(toolbar, text='Right-click: place point  |  Backspace: undo  |  d: delete nearest  |  s/Enter: save  |  q/Esc: quit  |  Scroll: zoom  |  Drag: pan').pack(side=tk.LEFT)
        self.status_var = tk.StringVar(value='Click on camera image (left) first')
        ttk.Label(toolbar, textvariable=self.status_var, foreground='blue').pack(side=tk.RIGHT, padx=8)

        # Canvas frame
        canvas_frame = ttk.Frame(self.root)
        canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(canvas_frame, bg='#1a1a1a', cursor='crosshair')
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Bind events
        self.canvas.bind('<Button-3>', self._on_right_click)
        self.canvas.bind('<Button-2>', self._on_undo)
        self.canvas.bind('<BackSpace>', self._on_undo)
        self.canvas.bind('<KeyPress-d>', self._on_delete_nearest)
        self.canvas.bind('<KeyPress-s>', self._on_save)
        self.canvas.bind('<Return>', self._on_save)
        self.canvas.bind('<KeyPress-q>', self._on_quit)
        self.canvas.bind('<Escape>', self._on_quit)
        self.canvas.bind('<MouseWheel>', self._on_scroll)
        self.canvas.bind('<Button-4>', self._on_scroll)
        self.canvas.bind('<Button-5>', self._on_scroll)
        self.canvas.bind('<ButtonPress-1>', self._on_drag_start)
        self.canvas.bind('<B1-Motion>', self._on_drag)
        self.canvas.bind('<Configure>', self._on_resize)
        self.canvas.focus_set()

    def _fit_to_window(self):
        self.root.update_idletasks()
        cw = self.canvas.winfo_width() or 1400
        ch = self.canvas.winfo_height() or 800
        # Two images side by side
        scale_w = (cw / 2) / self.img_w
        scale_h = ch / self.img_h
        self.zoom = min(scale_w, scale_h, 1.0)
        self.pan_x = 0.0
        self.pan_y = 0.0

    def _on_resize(self, event):
        self._render()

    # ── coordinate helpers ────────────────────────────────────────────────────

    def _canvas_to_img(self, cx, cy, panel):
        """Convert canvas pixel to image pixel. panel=0 (cam) or 1 (lid)."""
        cw = self.canvas.winfo_width()
        panel_w = cw // 2
        cx_local = cx - panel * panel_w
        ix = (cx_local / self.zoom) + self.pan_x
        iy = (cy / self.zoom) + self.pan_y
        return ix, iy

    def _img_to_canvas(self, ix, iy, panel):
        cw = self.canvas.winfo_width()
        panel_w = cw // 2
        cx = (ix - self.pan_x) * self.zoom + panel * panel_w
        cy = (iy - self.pan_y) * self.zoom
        return cx, cy

    def _which_panel(self, cx):
        return 1 if cx > self.canvas.winfo_width() // 2 else 0

    # ── event handlers ────────────────────────────────────────────────────────

    def _on_right_click(self, event):
        panel = self._which_panel(event.x)
        ix, iy = self._canvas_to_img(event.x, event.y, panel)
        ix = max(0, min(self.img_w - 1, ix))
        iy = max(0, min(self.img_h - 1, iy))

        if panel == 0:
            # Camera side — start a new pair
            self.pending_cam = (ix, iy)
            self.status_var.set(f'Camera point set at ({ix:.0f},{iy:.0f}) — now click lidar image (right)')
        else:
            # Lidar side — complete the pair
            if self.pending_cam is None:
                self.status_var.set('Click camera image (left) first!')
                return
            self.completed.append((self.pending_cam, (ix, iy)))
            self.pending_cam = None
            self.status_var.set(f'{len(self.completed)} pairs  |  Click camera image (left) for next pair')
        self._render()

    def _on_undo(self, event=None):
        if self.pending_cam is not None:
            self.pending_cam = None
            self.status_var.set('Undid pending camera point')
        elif self.completed:
            self.completed.pop()
            self.status_var.set(f'Undid last pair  ({len(self.completed)} remaining)')
        self._render()

    def _on_delete_nearest(self, event):
        if not self.completed:
            return
        # Find nearest completed pair to current mouse position
        mx, my = self.canvas.winfo_pointerxy()
        mx -= self.canvas.winfo_rootx()
        my -= self.canvas.winfo_rooty()
        best_i, best_d = 0, float('inf')
        for i, (cp, lp) in enumerate(self.completed):
            for panel, pt in [(0, cp), (1, lp)]:
                cx, cy = self._img_to_canvas(pt[0], pt[1], panel)
                d = (cx - mx)**2 + (cy - my)**2
                if d < best_d:
                    best_d, best_i = d, i
        self.completed.pop(best_i)
        self.status_var.set(f'Deleted pair {best_i}  ({len(self.completed)} remaining)')
        self._render()

    def _on_scroll(self, event):
        if event.num == 4 or event.delta > 0:
            factor = 1.15
        else:
            factor = 1 / 1.15
        # Zoom around mouse position
        panel = self._which_panel(event.x)
        ix, iy = self._canvas_to_img(event.x, event.y, panel)
        self.zoom *= factor
        self.zoom = max(0.05, min(20.0, self.zoom))
        # Adjust pan so the point under cursor stays fixed
        cw = self.canvas.winfo_width()
        panel_w = cw // 2
        cx_local = event.x - panel * panel_w
        self.pan_x = ix - cx_local / self.zoom
        self.pan_y = iy - event.y / self.zoom
        self._render()

    def _on_drag_start(self, event):
        self._drag_start = (event.x, event.y)
        self._drag_pan_start = (self.pan_x, self.pan_y)

    def _on_drag(self, event):
        if self._drag_start is None:
            return
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        self.pan_x = self._drag_pan_start[0] - dx / self.zoom
        self.pan_y = self._drag_pan_start[1] - dy / self.zoom
        self._render()

    def _on_save(self, event=None):
        if not self.completed:
            if not messagebox.askyesno('No matches', 'No manual matches added. Save anyway (keeps existing SuperGlue matches)?'):
                return
        self._write_matches()
        self.root.destroy()

    def _on_quit(self, event=None):
        if self.completed and not messagebox.askyesno('Quit', f'Discard {len(self.completed)} manual matches?'):
            return
        self.root.destroy()

    # ── rendering ─────────────────────────────────────────────────────────────

    def _render(self):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 10 or ch < 10:
            return

        panel_w = cw // 2
        disp_w = int(self.img_w * self.zoom)
        disp_h = int(self.img_h * self.zoom)

        # Crop region in image coords
        x0 = int(self.pan_x)
        y0 = int(self.pan_y)
        x1 = int(self.pan_x + panel_w / self.zoom)
        y1 = int(self.pan_y + ch / self.zoom)
        x0c = max(0, x0); y0c = max(0, y0)
        x1c = min(self.img_w, x1); y1c = min(self.img_h, y1)

        def render_panel(img_orig):
            crop = img_orig.crop((x0c, y0c, x1c, y1c))
            out_w = int((x1c - x0c) * self.zoom)
            out_h = int((y1c - y0c) * self.zoom)
            if out_w < 1 or out_h < 1:
                return Image.new('RGB', (panel_w, ch), (26, 26, 26))
            resized = crop.resize((out_w, out_h), Image.BILINEAR)
            canvas_img = Image.new('RGB', (panel_w, ch), (26, 26, 26))
            ox = int((x0c - x0) * self.zoom)
            oy = int((y0c - y0) * self.zoom)
            canvas_img.paste(resized, (ox, oy))
            return canvas_img

        cam_panel = render_panel(self.cam_img_orig)
        lid_panel = render_panel(self.lid_img_orig)

        draw_cam = ImageDraw.Draw(cam_panel)
        draw_lid = ImageDraw.Draw(lid_panel)

        def pt_to_panel(ix, iy, panel_offset=0):
            cx = (ix - x0) * self.zoom
            cy = (iy - y0) * self.zoom
            return cx - panel_offset, cy

        r = max(3, int(POINT_RADIUS * min(self.zoom, 2)))

        # Draw existing SuperGlue matches (dimmed)
        for i, (cp, lp) in enumerate(self._existing):
            cx, cy = pt_to_panel(cp[0], cp[1])
            lx, ly = pt_to_panel(lp[0], lp[1])
            draw_cam.ellipse([cx-r//2, cy-r//2, cx+r//2, cy+r//2], outline='#446644', width=1)
            draw_lid.ellipse([lx-r//2, ly-r//2, lx+r//2, ly+r//2], outline='#664444', width=1)

        # Draw completed manual pairs
        for i, (cp, lp) in enumerate(self.completed):
            cx, cy = pt_to_panel(cp[0], cp[1])
            lx, ly = pt_to_panel(lp[0], lp[1])
            draw_cam.ellipse([cx-r, cy-r, cx+r, cy+r], outline=CAM_COLOR, width=2)
            draw_cam.text((cx+r+2, cy-r), str(i), fill=CAM_COLOR)
            draw_lid.ellipse([lx-r, ly-r, lx+r, ly+r], outline=LID_COLOR, width=2)
            draw_lid.text((lx+r+2, ly-r), str(i), fill=LID_COLOR)

        # Draw pending camera point
        if self.pending_cam is not None:
            cx, cy = pt_to_panel(self.pending_cam[0], self.pending_cam[1])
            draw_cam.ellipse([cx-r, cy-r, cx+r, cy+r], outline=PENDING_COLOR, width=2)
            draw_cam.line([cx-r*2, cy, cx+r*2, cy], fill=PENDING_COLOR, width=1)
            draw_cam.line([cx, cy-r*2, cx, cy+r*2], fill=PENDING_COLOR, width=1)

        # Divider label
        combined = Image.new('RGB', (cw, ch), (26, 26, 26))
        combined.paste(cam_panel, (0, 0))
        combined.paste(lid_panel, (panel_w, 0))
        draw_all = ImageDraw.Draw(combined)
        draw_all.line([panel_w, 0, panel_w, ch], fill='#555555', width=2)
        draw_all.text((10, 10), 'CAMERA ERP', fill='#aaaaaa')
        draw_all.text((panel_w + 10, 10), 'LIDAR INTENSITY', fill='#aaaaaa')
        draw_all.text((10, ch - 20), f'{len(self.completed)} manual pairs', fill='#aaaaaa')

        self._tk_img = ImageTk.PhotoImage(combined)
        self.canvas.delete('all')
        self.canvas.create_image(0, 0, anchor='nw', image=self._tk_img)

    # ── output ────────────────────────────────────────────────────────────────

    def _write_matches(self):
        """Merge manual matches with existing SuperGlue matches and write JSON."""
        # Load existing file or create empty structure
        if self._matches_path.exists():
            with open(self._matches_path) as f:
                d = json.load(f)
        else:
            d = {'kpts0': [], 'kpts1': [], 'matches': [], 'match_confidence': []}

        existing_kpts0 = list(np.array(d['kpts0']).reshape(-1, 2)) if d['kpts0'] else []
        existing_kpts1 = list(np.array(d['kpts1']).reshape(-1, 2)) if d['kpts1'] else []
        matches = list(d['matches'])
        confidence = list(d.get('match_confidence', [1.0] * len(existing_kpts0)))

        for cam_pt, lid_pt in self.completed:
            # Append new keypoints
            i0 = len(existing_kpts0)
            i1 = len(existing_kpts1)
            existing_kpts0.append(list(cam_pt))
            existing_kpts1.append(list(lid_pt))
            # Extend matches array: -1 for all existing kpts0, then point to new kpt1
            while len(matches) < i0:
                matches.append(-1)
            matches.append(i1)
            confidence.append(1.0)

        d['kpts0'] = [c for pt in existing_kpts0 for c in pt]
        d['kpts1'] = [c for pt in existing_kpts1 for c in pt]
        d['matches'] = matches
        d['match_confidence'] = confidence

        with open(self._matches_path, 'w') as f:
            json.dump(d, f)

        print(f'✓ Saved {len(self.completed)} manual matches to {self._matches_path}')
        print(f'  Total matches in file: {sum(1 for m in matches if m >= 0)}')


def main():
    parser = argparse.ArgumentParser(description='Manual match picker for camera ERP vs lidar intensity')
    parser.add_argument('output_dir', help='Calibration output directory (e.g. ~/atlas_ws/output)')
    parser.add_argument('--pair', default=None, help='Pair name to edit (e.g. 000001). If omitted, shows a picker.')
    parser.add_argument('--perspective', action='store_true', help='Show perspective crops instead of full ERP')
    parser.add_argument('--yaw', type=float, default=0.0, help='Perspective crop yaw (degrees)')
    parser.add_argument('--pitch', type=float, default=0.0, help='Perspective crop pitch (degrees)')
    parser.add_argument('--fov', type=float, default=90.0, help='Perspective crop FOV (degrees)')
    parser.add_argument('--crop-size', type=int, default=800, help='Perspective crop size in pixels')
    args = parser.parse_args()

    output_dir = Path(args.output_dir).expanduser().resolve()

    if args.pair:
        pairs = [args.pair]
    else:
        pairs = sorted(p.stem for p in output_dir.glob('*.ply'))
        if not pairs:
            print(f'No PLY files found in {output_dir}')
            sys.exit(1)

    # Detect SDK stitch from sidecar
    import cv2 as _cv2
    first_sidecar = output_dir / '000000_source.json'
    is_sdk = False
    if first_sidecar.exists():
        import json as _json
        sd = _json.load(open(first_sidecar))
        scan_path = Path(sd.get('scan_dir', ''))
        is_sdk = bool(list(scan_path.glob('*.insp'))) if scan_path.is_dir() else False

    for pair in pairs:
        if args.perspective:
            import sys as _sys
            _sys.path.insert(0, str(Path(__file__).parent))
            from find_matches_superglue_erp import extract_perspective_crop

            cam_erp_path = output_dir / 'images' / f'{pair}.png'
            if not cam_erp_path.exists():
                cam_erp_path = output_dir / f'{pair}.png'
            cam_erp = _cv2.imread(str(cam_erp_path), _cv2.IMREAD_GRAYSCALE)
            lid_erp = _cv2.imread(str(output_dir / f'{pair}_lidar_intensities.png'), _cv2.IMREAD_GRAYSCALE)
            if cam_erp is None or lid_erp is None:
                print(f'Missing images for {pair}, skipping')
                continue
            if lid_erp.shape != cam_erp.shape:
                lid_erp = _cv2.resize(lid_erp, (cam_erp.shape[1], cam_erp.shape[0]))

            # For SDK stitch lidar, apply roll_half correction to align with camera ERP
            if is_sdk:
                lid_erp = np.roll(np.flipud(np.fliplr(lid_erp)), lid_erp.shape[1] // 2, axis=1)

            cam_crop, cam_mx, cam_my = extract_perspective_crop(
                cam_erp, args.yaw, args.pitch, args.fov, args.crop_size)
            lid_crop, lid_mx, lid_my = extract_perspective_crop(
                lid_erp, args.yaw, args.pitch, args.fov, args.crop_size)

            # Save crops to output_dir for inspection and load from there
            cam_tmp = output_dir / f'{pair}_persp_y{int(args.yaw):03d}_cam.png'
            lid_tmp = output_dir / f'{pair}_persp_y{int(args.yaw):03d}_lid.png'
            map_file = output_dir / f'{pair}_persp_y{int(args.yaw):03d}_maps.npz'
            _cv2.imwrite(str(cam_tmp), cam_crop)
            lid_color = _cv2.applyColorMap(lid_crop, _cv2.COLORMAP_INFERNO)
            _cv2.imwrite(str(lid_tmp), lid_color)
            np.savez(str(map_file), cam_mx=cam_mx, cam_my=cam_my, lid_mx=lid_mx, lid_my=lid_my)
            print(f'Saved crops: {cam_tmp.name}  {lid_tmp.name}')

            # Create a side-by-side preview
            preview = np.concatenate([cam_crop, lid_crop], axis=1)
            _cv2.imwrite(str(output_dir / f'{pair}_persp_y{int(args.yaw):03d}_preview.png'), preview)

            # Use a temp dir with symlink-named files for the tool
            import tempfile
            tmp_dir = Path(tempfile.mkdtemp())
            import shutil
            shutil.copy(str(cam_tmp), str(tmp_dir / f'{pair}.png'))
            shutil.copy(str(lid_tmp), str(tmp_dir / f'{pair}_lidar_intensities.png'))

            root = tk.Tk()
            try:
                app = ManualMatchTool(root, tmp_dir, pair)
                # Patch _write_matches to remap crop coords back to ERP coords
                orig_write = app._write_matches
                def _write_persp():
                    maps = np.load(str(map_file))
                    cmx, cmy = maps['cam_mx'], maps['cam_my']
                    lmx, lmy = maps['lid_mx'], maps['lid_my']
                    remapped = []
                    for (cx, cy), (lx, ly) in app.completed:
                        ci, cj = int(np.clip(cx, 0, cmx.shape[1]-1)), int(np.clip(cy, 0, cmx.shape[0]-1))
                        li, lj = int(np.clip(lx, 0, lmx.shape[1]-1)), int(np.clip(ly, 0, lmx.shape[0]-1))
                        app.completed[app.completed.index(((cx,cy),(lx,ly)))] = (
                            (float(cmx[cj,ci]), float(cmy[cj,ci])),
                            (float(lmx[lj,li]), float(lmy[lj,li]))
                        )
                    app._matches_path = output_dir / f'{pair}_matches.json'
                    orig_write()
                app._write_matches = _write_persp
            except FileNotFoundError as e:
                print(f'Skipping {pair}: {e}')
                root.destroy()
                continue
            root.mainloop()
        else:
            root = tk.Tk()
            try:
                app = ManualMatchTool(root, output_dir, pair)
            except FileNotFoundError as e:
                print(f'Skipping {pair}: {e}')
                root.destroy()
                continue
            root.mainloop()

        if len(pairs) > 1 and pair != pairs[-1]:
            cont = input(f'Continue to next pair? [Y/n]: ').strip().lower()
            if cont == 'n':
                break
