#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Tkinter GUI front-end for the ATLAS system. Provides scan triggering, live system log, embedded RViz2 viewer, and session controls for stationary and continuous capture modes.

import os
import sys

# X11 window embedding requires X11 — force XWayland if running under Wayland
_WAYLAND_DISPLAY = os.environ.get('WAYLAND_DISPLAY', '')
if _WAYLAND_DISPLAY and not os.environ.get('_ATLAS_REEXEC'):
    os.environ['GDK_BACKEND'] = 'x11'
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.environ['_ATLAS_REEXEC'] = '1'
    os.environ['_ATLAS_WAYLAND_DISPLAY'] = _WAYLAND_DISPLAY
    os.environ.pop('WAYLAND_DISPLAY', None)
    # Only pass the script path — drop all other argv to prevent argument injection
    os.execv(sys.executable, [sys.executable, os.path.abspath(__file__)])

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from PIL import Image, ImageTk
import subprocess
import threading
import signal
import time
import logging
import traceback
from datetime import datetime
import pathlib
import math


# ─── Coverage Panel ───────────────────────────────────────────────────────────
# Mirrors the logic in capture/coverage_tracker.py but renders into a tk.Canvas
# so it can be embedded as a notebook tab in the GUI.

_COV_CELL_SIZE        = 0.5
_COV_HEIGHT_BANDS     = 3
_COV_BAND_SIZE        = 0.4064  # 16" per band — fits the ~47" scanner extension range
_COV_HEIGHT_ORIGIN    = 0.0     # Low: [0, 16")  Mid: [16", 32")  High: [32"+)
_COV_MIN_VISITS       = 1
_COV_BAND_LABELS      = ['Low', 'Mid', 'High']
# Colour per number of covered bands (0..HEIGHT_BANDS)
# Cell background darkens as more bands are captured
# Cell colour keyed by frozenset of covered bands (band indices that have >= MIN_VISITS)
# Readable at any cell size — no sub-cell drawing needed
_COV_NONE   = '#f0f0f0'   # nothing shot
_COV_COLOUR = {
    frozenset()       : _COV_NONE,
    frozenset([0])    : '#4575b4',   # Low only      — cool blue
    frozenset([1])    : '#74add1',   # Mid only      — light blue
    frozenset([2])    : '#abd9e9',   # High only     — pale cyan
    frozenset([0,1])  : '#fee090',   # Low+Mid       — yellow
    frozenset([0,2])  : '#fdae61',   # Low+High      — amber
    frozenset([1,2])  : '#f46d43',   # Mid+High      — orange
    frozenset([0,1,2]): '#1a9641',   # All done      — green
}

def _cov_band(z: float, z_ref: float = 0.0) -> int:
    return max(0, min(_COV_HEIGHT_BANDS - 1,
                      int(math.floor((z - z_ref) / _COV_BAND_SIZE))))


class CoveragePanel:
    """Tkinter canvas widget showing live 2.5D coverage during a scan session."""

    def __init__(self, parent: tk.Frame):
        # grid[(gx,gy)] = {band: shot_count} — only incremented on confirmed captures
        self._grid: dict[tuple, dict] = {}
        self._current = (0.0, 0.0, 0.0)
        self._current_yaw = 0.0
        self._z_ref = None  # set on first pose; bands are relative to this
        self._pose_lock = threading.Lock()
        self._ros_thread = None
        self._running = False
        self._cell_px = 14
        self._cell_items: dict[tuple, int] = {}    # bg rectangle per cell

        self._player_item: int | None = None
        self._arrow_item: int | None = None
        self._current_yaw: float = 0.0
        self._scan_dir: str | None = None
        self._seen_shutters: set = set()
        self._perimeter: list[tuple] = []
        self._perimeter_item: int | None = None
        self._perimeter_done = False
        self._lidar_pts: list[tuple] = []

        # ── Layout ────────────────────────────────────────────────────────────
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        self._stats_var = tk.StringVar(value=f'Starting at low elevation — waiting for LiDAR perimeter…')
        ttk.Label(parent, textvariable=self._stats_var,
                  font=('Consolas', 9)).grid(row=0, column=0, sticky=tk.W, padx=6, pady=(4, 0))

        leg_frame = tk.Frame(parent)
        leg_frame.grid(row=0, column=1, sticky=tk.E, padx=6, pady=(4, 0))
        for col, label in [
            (_COV_NONE,                        'none'),
            (_COV_COLOUR[frozenset([0])],       'L'),
            (_COV_COLOUR[frozenset([1])],       'M'),
            (_COV_COLOUR[frozenset([2])],       'H'),
            (_COV_COLOUR[frozenset([0,1])],     'L+M'),
            (_COV_COLOUR[frozenset([0,2])],     'L+H'),
            (_COV_COLOUR[frozenset([1,2])],     'M+H'),
            (_COV_COLOUR[frozenset([0,1,2])],   'all ✓'),
        ]:
            tk.Label(leg_frame, bg=col, width=2, relief='flat').pack(side=tk.LEFT, padx=(4, 1))
            tk.Label(leg_frame, text=label, font=('Arial', 8)).pack(side=tk.LEFT, padx=(0, 4))

        self._canvas = tk.Canvas(parent, bg='white', highlightthickness=0)
        self._canvas.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        self._canvas.bind('<Configure>', self._on_resize)

        hb_frame = ttk.LabelFrame(parent, text='Current height band', padding='4')
        hb_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=6, pady=(4, 4))
        self._band_labels: list[ttk.Label] = []
        for i, name in enumerate(_COV_BAND_LABELS):
            lbl = ttk.Label(hb_frame, text=f'  {name}  ', relief='groove',
                            font=('Arial', 9, 'bold'), anchor='center')
            lbl.grid(row=0, column=i, padx=3)
            self._band_labels.append(lbl)

    # ── ROS thread ────────────────────────────────────────────────────────────

    def start(self, scan_dir: str | None = None):
        if self._running:
            return
        self._running = True
        self._scan_dir = scan_dir
        self._grid.clear()
        self._cell_items.clear()
        self._seen_shutters.clear()
        self._player_item = None
        self._arrow_item = None
        self._current_yaw = 0.0
        self._z_ref = None
        self._last_drawn_state = None
        self._perimeter: list[tuple] = []   # convex hull in grid coords
        self._perimeter_item: int | None = None
        self._perimeter_done = False
        self._lidar_pts: list[tuple] = []   # raw (x,y) accumulator
        self._current = (0.0, 0.0, 0.0)
        self._current_yaw = 0.0
        self._last_drawn_state = None
        self._latest_pose = None
        with self._pose_lock:
            self._pose_history = []
        self._canvas.delete('all')
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()
        self._update_stats()  # highlight correct band immediately at startup
        self._canvas.after(500, self._poll)

    def stop(self):
        self._running = False

    def _ros_spin(self):
        try:
            import rclpy
            from nav_msgs.msg import Odometry as _Odom
            from sensor_msgs.msg import PointCloud2 as _PC2
            import struct
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node('atlas_coverage_gui')

            def _odom_cb(msg):
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                # Extract yaw from quaternion (rotation about Z axis)
                # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
                cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                with self._pose_lock:
                    self._latest_pose = (p.x, p.y, p.z, yaw)
                    # Keep a rolling 60s history for shutter-time pose lookup
                    if not hasattr(self, '_pose_history'):
                        self._pose_history = []
                    self._pose_history.append((t, (p.x, p.y, p.z)))
                    if len(self._pose_history) > 1200:  # 20Hz * 60s
                        self._pose_history = self._pose_history[-1200:]
            node.create_subscription(_Odom, '/rko_lio/odometry', _odom_cb, 10)

            def _lidar_cb(msg):
                if self._perimeter_done:
                    return
                # Parse XYZ from PointCloud2 — find field offsets once
                fields = {f.name: f.offset for f in msg.fields}
                if 'x' not in fields or 'y' not in fields or 'z' not in fields:
                    return
                ox, oy, oz = fields['x'], fields['y'], fields['z']
                ps = msg.point_step
                data = bytes(msg.data)
                pts = []
                for i in range(msg.width * msg.height):
                    base = i * ps
                    x, y, z = (struct.unpack_from('<f', data, base + o)[0]
                               for o in (ox, oy, oz))
                    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                        continue
                    # Keep points in a plausible wall/furniture height band
                    if 0.1 <= z <= 2.5:
                        pts.append((x, y))
                with self._pose_lock:
                    self._lidar_pts.extend(pts)
            node.create_subscription(_PC2, '/livox/lidar', _lidar_cb, 2)

            while self._running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        except Exception:
            pass

    # ── Poll (main thread, 2 Hz) ──────────────────────────────────────────────

    def set_scan_dir(self, scan_dir: str):
        self._scan_dir = scan_dir

    def _pose_at_shutter(self, shutter_event_path) -> tuple | None:
        """Return (x, y, z) pose at shutter time by reading trajectory.json."""
        try:
            import json
            scan_dir = shutter_event_path.parent
            traj_f = scan_dir / 'trajectory.json'
            if traj_f.exists():
                t = json.loads(traj_f.read_text())
                p = t['current_pose']['position']
                return (p['x'], p['y'], p['z'])
            # Fallback: read shutter timestamp and find closest stored pose.
            # shutter_t is host clock; _pose_history uses Livox clock (~54ms offset).
            # The offset is negligible for 0.5m grid cells.
            shutter_t = float(shutter_event_path.read_text().strip())
            with self._pose_lock:
                hist = list(getattr(self, '_pose_history', []))
            if hist:
                closest = min(hist, key=lambda e: abs(e[0] - shutter_t))
                # Only use if within 5s — guards against empty/stale history
                if abs(closest[0] - shutter_t) < 5.0:
                    return closest[1]
        except Exception:
            pass
        return None

    # ── Perimeter estimate ────────────────────────────────────────────────────

    @staticmethod
    def _convex_hull(pts: list[tuple]) -> list[tuple]:
        """Graham scan convex hull. Returns hull vertices in CCW order."""
        pts = list(set(pts))
        if len(pts) < 3:
            return pts
        pivot = min(pts, key=lambda p: (p[1], p[0]))
        def _angle(p):
            dx, dy = p[0] - pivot[0], p[1] - pivot[1]
            return math.atan2(dy, dx)
        pts.sort(key=_angle)
        hull = []
        for p in pts:
            while len(hull) >= 2:
                ox, oy = hull[-2][0] - hull[-1][0], hull[-2][1] - hull[-1][1]
                nx, ny = p[0]  - hull[-1][0], p[1]  - hull[-1][1]
                if ox * ny - oy * nx >= 0:   # not a left turn
                    hull.pop()
                else:
                    break
            hull.append(p)
        return hull

    def _try_build_perimeter(self):
        """Called from _poll for the first 8 s. Builds hull once enough points exist."""
        with self._pose_lock:
            pts = list(self._lidar_pts)
        # Downsample to ~0.3 m grid to avoid hull dominated by dense nearby surfaces
        grid: dict[tuple, tuple] = {}
        for x, y in pts:
            k = (int(math.floor(x / 0.3)), int(math.floor(y / 0.3)))
            grid[k] = (x, y)
        sampled = list(grid.values())
        if len(sampled) < 20:
            return
        hull_world = self._convex_hull(sampled)
        # Convert world coords to grid-cell coords
        self._perimeter = [
            (x / _COV_CELL_SIZE, y / _COV_CELL_SIZE) for x, y in hull_world
        ]
        self._perimeter_done = True
        # Free the accumulator
        with self._pose_lock:
            self._lidar_pts.clear()
        self._draw_perimeter()

    def _draw_perimeter(self):
        """Draw (or redraw) the perimeter hull on the canvas."""
        if not self._perimeter:
            return
        if not self._grid:
            # No cells yet — seed a dummy origin so _layout has keys
            self._grid[(0, 0)] = {}
        min_x, max_y, px, off_x, off_y = self._layout()
        def _to_canvas(gx, gy):
            return (off_x + (gx - min_x) * px + px // 2,
                    off_y + (max_y - gy) * px + px // 2)
        coords = [c for gx, gy in self._perimeter for c in _to_canvas(gx, gy)]
        if len(coords) < 6:   # need at least 3 points
            return
        if self._perimeter_item is not None:
            try:
                self._canvas.delete(self._perimeter_item)
            except Exception:
                pass
        self._perimeter_item = self._canvas.create_polygon(
            coords, outline='#4a9eff', fill='', width=2, dash=(6, 4))
        # Label the perimeter so the operator knows what it represents
        cx = sum(c for i, c in enumerate(coords) if i % 2 == 0) // (len(coords) // 2)
        cy = min(c for i, c in enumerate(coords) if i % 2 == 1) - 10
        self._canvas.create_text(cx, cy, text='LiDAR perimeter (mid elev.)',
                                 fill='#4a9eff', font=('Arial', 7), tags='perimeter_label')
        # Keep perimeter below cell rectangles so it doesn't obscure pips
        self._canvas.tag_lower(self._perimeter_item)

    def _poll(self):
        # Update current position
        with self._pose_lock:
            pose = self._latest_pose
            self._latest_pose = None
        if pose is not None:
            self._current = (pose[0], pose[1], pose[2])
            self._current_yaw = pose[3] if len(pose) > 3 else 0.0
            # Set Z reference on first pose so bands are relative to start height.
            # Low = start height ± half-band, Mid = start + 24", High = start + 48"
            if self._z_ref is None:
                self._z_ref = self._current[2]
            self._update_player()
            self._update_stats()

        # Build perimeter from early LiDAR accumulation
        if not self._perimeter_done:
            self._try_build_perimeter()
        elif self._perimeter and self._perimeter_item is None and self._grid:
            self._draw_perimeter()

        # Check for new shutter events — each one is a confirmed capture
        if self._scan_dir:
            try:
                for se in pathlib.Path(self._scan_dir).glob(
                        'fusion_scan_*/capture_*.shutter_event'):
                    if str(se) in self._seen_shutters:
                        continue
                    self._seen_shutters.add(str(se))
                    # Use the pose at shutter time, not current pose.
                    # Read the shutter timestamp from the event file and find
                    # the closest trajectory pose.
                    shutter_pos = self._pose_at_shutter(se)
                    x, y, z = shutter_pos if shutter_pos else self._current
                    key  = (int(math.floor(x / _COV_CELL_SIZE)),
                            int(math.floor(y / _COV_CELL_SIZE)))
                    band = _cov_band(z, self._z_ref or 0.0)
                    cell = self._grid.setdefault(key, {})
                    cell[band] = cell.get(band, 0) + 1
                    self._update_cell(key)
                    self._update_stats()
            except OSError:
                pass

        if self._running:
            self._canvas.after(500, self._poll)

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _on_resize(self, event):
        self._cell_items.clear()
        self._player_item = None
        self._arrow_item = None
        self._perimeter_item = None
        self._canvas.delete('perimeter_label')
        self._last_drawn_state = None
        self._canvas.delete('all')
        for key in self._grid:
            self._update_cell(key)
        self._draw_perimeter()
        self._update_player()

    def _layout(self):
        """Return (min_x, max_y, px, off_x, off_y) sized to fit the perimeter (or grid)."""
        keys  = list(self._grid.keys())
        min_x = min(k[0] for k in keys)
        max_x = max(k[0] for k in keys)
        min_y = min(k[1] for k in keys)
        max_y = max(k[1] for k in keys)
        # Expand bounds to the perimeter hull so the whole room is visible
        if self._perimeter:
            pxs = [gx for gx, gy in self._perimeter]
            pys = [gy for gx, gy in self._perimeter]
            min_x = min(min_x, math.floor(min(pxs)))
            max_x = max(max_x, math.ceil(max(pxs)))
            min_y = min(min_y, math.floor(min(pys)))
            max_y = max(max_y, math.ceil(max(pys)))
        margin = 2   # cells of padding around the perimeter
        min_x -= margin; max_x += margin
        min_y -= margin; max_y += margin
        span_x = max(max_x - min_x + 1, 1)
        span_y = max(max_y - min_y + 1, 1)
        c  = self._canvas
        w  = max(c.winfo_width(),  1)
        h  = max(c.winfo_height(), 1)
        px = max(4, min(40, w // span_x, h // span_y))
        self._cell_px = px
        off_x = (w - span_x * px) // 2
        off_y = (h - span_y * px) // 2
        return min_x, max_y, px, off_x, off_y

    def _update_cell(self, key):
        """Create or update the background rect and three height-band pips for a cell."""
        if not self._grid:
            return
        bv = self._grid.get(key)
        if bv is None:
            return
        done_bands = frozenset(b for b, v in bv.items() if v >= _COV_MIN_VISITS)
        bg         = _COV_COLOUR.get(done_bands, _COV_NONE)
        gx, gy  = key
        min_x, max_y, px, off_x, off_y = self._layout()
        sx = off_x + (gx - min_x) * px
        sy = off_y + (max_y - gy) * px

        # Background rectangle
        if key in self._cell_items:
            self._canvas.itemconfig(self._cell_items[key], fill=bg)
        else:
            self._cell_items[key] = self._canvas.create_rectangle(
                sx, sy, sx + px, sy + px, fill=bg, outline='#cccccc', width=1)

        if self._player_item is not None:
            self._canvas.tag_raise(self._player_item)

    def _update_player(self):
        """Move or create the current-position indicator with direction arrow."""
        if not self._grid:
            return
        x, y, z = self._current
        yaw = getattr(self, '_current_yaw', 0.0)
        cx = int(math.floor(x / _COV_CELL_SIZE))
        cy = int(math.floor(y / _COV_CELL_SIZE))
        # Redraw if position or yaw changed significantly
        _prev = getattr(self, '_last_drawn_state', None)
        if _prev is not None:
            _pcx, _pcy, _pyaw = _prev
            if _pcx == cx and _pcy == cy and abs(yaw - _pyaw) < 0.1:
                return
        self._last_drawn_state = (cx, cy, yaw)
        min_x, max_y, px, off_x, off_y = self._layout()
        # Centre of the cell in canvas coords
        cell_cx = off_x + (cx - min_x) * px + px / 2
        cell_cy = off_y + (max_y - cy) * px + px / 2
        r = max(3, px * 0.35)  # radius of the circle

        # Delete old items
        if self._player_item is not None:
            self._canvas.delete(self._player_item)
        if getattr(self, '_arrow_item', None) is not None:
            self._canvas.delete(self._arrow_item)

        # Draw filled circle
        self._player_item = self._canvas.create_oval(
            cell_cx - r, cell_cy - r, cell_cx + r, cell_cy + r,
            fill='#e74c3c', outline='#c0392b', width=1)

        # Draw direction arrow from centre outward
        # In RKO-LIO, +X is forward in the lidar frame. On the canvas,
        # +X maps to right and +Y maps to down (Y is flipped).
        arrow_len = r * 1.8
        # yaw is rotation about Z (up): 0 = +X (right on canvas)
        ax = cell_cx + arrow_len * math.cos(yaw)
        ay = cell_cy - arrow_len * math.sin(yaw)  # minus because canvas Y is inverted
        self._arrow_item = self._canvas.create_line(
            cell_cx, cell_cy, ax, ay,
            fill='#00ff00', width=max(2, px // 6),
            arrow=tk.LAST, arrowshape=(max(4, px // 3), max(5, px // 2.5), max(2, px // 6)))

        self._canvas.tag_raise(self._player_item)
        self._canvas.tag_raise(self._arrow_item)

    def _update_stats(self):
        total   = len(self._grid)
        full    = sum(1 for bv in self._grid.values()
                      if sum(1 for v in bv.values() if v >= _COV_MIN_VISITS) >= _COV_HEIGHT_BANDS)
        partial = sum(1 for bv in self._grid.values()
                      if 0 < sum(1 for v in bv.values() if v >= _COV_MIN_VISITS) < _COV_HEIGHT_BANDS)
        shots   = len(self._seen_shutters)
        pct     = (full / total * 100) if total else 0
        x, y, z = self._current
        z_rel = z - (self._z_ref or 0.0)
        self._stats_var.set(
            f'Pos ({x:.1f}, {y:.1f}, {z:.1f}) m  z_rel={z_rel:+.2f}m  |  Shots: {shots}  |  '
            f'Full: {full}/{total} ({pct:.0f}%)  Partial: {partial}'
        )
        current_band = _cov_band(z, self._z_ref or 0.0)
        for i, lbl in enumerate(self._band_labels):
            if i == current_band:
                lbl.config(background='#7ed321', foreground='white')
            else:
                lbl.config(background='', foreground='')


class FusionCaptureGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ATLAS")

        # Set window icon (taskbar + title bar)
        try:
            import pathlib as _pl
            _icon_path = _pl.Path(__file__).parent.parent / 'assets/media/atlas_icon_256.png'
            _icon = tk.PhotoImage(file=str(_icon_path))
            self.root.iconphoto(True, _icon)
            self._icon_ref = _icon  # prevent GC
        except (FileNotFoundError, OSError, tk.TclError):
            pass

        # Fit initial window to screen — leave a small margin for taskbars
        sw = root.winfo_screenwidth()
        sh = root.winfo_screenheight()
        win_w = min(1400, sw - 40)
        win_h = min(900, sh - 60)
        self.root.geometry(f'{win_w}x{win_h}+20+20')
        # Allow the window to shrink as far as the screen allows
        self.root.minsize(min(600, sw), min(400, sh))
        self._small_screen = sw < 1100 or sh < 700
        
        # Get script directory for relative paths
        self.script_dir = pathlib.Path(__file__).parent.resolve()
        
        # Process management
        self.fusion_process = None
        self.rviz_process = None
        self.rviz_win_id = None
        self.web_viewer_process = None
        self.is_running = False
        self.scan_count = 0
        self._pending_viewer_html = None
        self._latest_thumb_photo = None
        
        # Setup GUI
        self.setup_gui()
        
        # Change to script directory for execution
        os.chdir(self.script_dir)
    
    def setup_gui(self):
        # Main frame with reduced padding
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create left and right panels for better space utilization.
        # The left panel is a scrollable canvas so content is never clipped on
        # small screens — a scrollbar appears only when needed.
        left_scroll_frame = ttk.Frame(main_frame)
        left_scroll_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        left_scroll_frame.rowconfigure(0, weight=1)
        left_scroll_frame.columnconfigure(0, weight=1)

        left_canvas = tk.Canvas(left_scroll_frame, highlightthickness=0)
        left_canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        left_scrollbar = ttk.Scrollbar(left_scroll_frame, orient=tk.VERTICAL, command=left_canvas.yview)
        left_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_panel = ttk.Frame(left_canvas)
        _left_window = left_canvas.create_window((0, 0), window=left_panel, anchor='nw')

        def _on_left_panel_configure(event):
            left_canvas.configure(scrollregion=left_canvas.bbox('all'))
            # Show/hide scrollbar based on whether content overflows
            if left_panel.winfo_reqheight() > left_canvas.winfo_height():
                left_scrollbar.grid()
            else:
                left_scrollbar.grid_remove()
        left_panel.bind('<Configure>', _on_left_panel_configure)

        def _on_left_canvas_configure(event):
            left_canvas.itemconfig(_left_window, width=event.width)
        left_canvas.bind('<Configure>', _on_left_canvas_configure)

        # Mouse-wheel scrolling on the left panel only
        def _on_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), 'units')
        def _on_button4(event):
            left_canvas.yview_scroll(-1, 'units')
        def _on_button5(event):
            left_canvas.yview_scroll(1, 'units')

        left_scroll_frame.bind('<Enter>', lambda e: (
            left_canvas.bind_all('<MouseWheel>', _on_mousewheel),
            left_canvas.bind_all('<Button-4>', _on_button4),
            left_canvas.bind_all('<Button-5>', _on_button5)))
        left_scroll_frame.bind('<Leave>', lambda e: (
            left_canvas.unbind_all('<MouseWheel>'),
            left_canvas.unbind_all('<Button-4>'),
            left_canvas.unbind_all('<Button-5>')))

        right_panel = ttk.Frame(main_frame)
        right_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        
        # Left panel: Logo — scaled smaller on small screens
        try:
            logo_path = self.script_dir / '..' / 'assets' / 'media' / 'atlas_logo_app.png'
            logo_image = Image.open(logo_path)
            aspect_ratio = logo_image.width / logo_image.height
            logo_h = 50 if self._small_screen else 80
            new_width = int(logo_h * aspect_ratio)
            logo_image = logo_image.resize((new_width, logo_h), Image.LANCZOS)
            self.logo_photo = ImageTk.PhotoImage(logo_image)
            logo_label = ttk.Label(left_panel, image=self.logo_photo)
            logo_label.grid(row=0, column=0, pady=(0, 4))
        except (FileNotFoundError, OSError, tk.TclError):
            pass  # Skip logo if not found
        
        # Status frame (compact)
        _pady_section = (0, 6) if self._small_screen else (0, 12)
        status_frame = ttk.LabelFrame(left_panel, text="Status", padding="3" if self._small_screen else "5")
        status_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=_pady_section)
        
        self.status_label = ttk.Label(status_frame, text="Ready to start", foreground="blue")
        self.status_label.grid(row=0, column=0, sticky=tk.W)
        
        # Control buttons frame (vertical layout for compactness)
        control_frame = ttk.LabelFrame(left_panel, text="Controls", padding="4" if self._small_screen else "8")
        control_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=_pady_section)

        # Mode selectors
        mode_frame = ttk.Frame(control_frame)
        mode_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 4))
        mode_frame.columnconfigure(1, weight=1)
        ttk.Label(mode_frame, text="Camera:").grid(row=0, column=0, sticky=tk.W, padx=(0, 4))
        self.camera_mode_var = tk.StringVar(value="dual_fisheye")
        ttk.Combobox(mode_frame, textvariable=self.camera_mode_var,
                     values=["dual_fisheye", "single_fisheye"],
                     state="readonly", width=14).grid(row=0, column=1, sticky=(tk.W, tk.E))
        ttk.Label(mode_frame, text="Camera HW:").grid(row=1, column=0, sticky=tk.W, padx=(0, 4), pady=(2, 0))
        self.camera_hw_var = tk.StringVar(value="onex2")
        ttk.Combobox(mode_frame, textvariable=self.camera_hw_var,
                     values=["onex2", "x3", "x5"],
                     state="readonly", width=14).grid(row=1, column=1, sticky=(tk.W, tk.E), pady=(2, 0))
        ttk.Label(mode_frame, text="Cameras:").grid(row=2, column=0, sticky=tk.W, padx=(0, 4), pady=(2, 0))
        self.num_cameras_var = tk.StringVar(value="auto")
        ttk.Combobox(mode_frame, textvariable=self.num_cameras_var,
                     values=["auto", "1", "2", "3"],
                     state="readonly", width=14).grid(row=2, column=1, sticky=(tk.W, tk.E), pady=(2, 0))
        ttk.Label(mode_frame, text="Capture:").grid(row=3, column=0, sticky=tk.W, padx=(0, 4), pady=(2, 0))
        self.capture_mode_var = tk.StringVar(value="continuous")
        ttk.Combobox(mode_frame, textvariable=self.capture_mode_var,
                     values=["continuous", "stationary"],
                     state="readonly", width=14).grid(row=3, column=1, sticky=(tk.W, tk.E), pady=(2, 0))
        self.stationary_wait_var = tk.BooleanVar(value=False)
        self.stationary_wait_cb = ttk.Checkbutton(mode_frame, text="Wait 3s before recording (stationary)",
                        variable=self.stationary_wait_var)
        self.stationary_wait_cb.grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=(2, 0))
        self.bag_only_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(mode_frame, text="Bag only (post-process later)",
                        variable=self.bag_only_var).grid(row=5, column=0, columnspan=2, sticky=tk.W, pady=(2, 0))
        self.icp_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(mode_frame, text="ICP alignment (post processing)",
                        variable=self.icp_var).grid(row=6, column=0, columnspan=2, sticky=tk.W, pady=(2, 0))
        self.colmap_var = tk.BooleanVar(value=False)
        self.colmap_lidar_voxel_size = 0.0
        ttk.Checkbutton(mode_frame, text="Export COLMAP model",
                        variable=self.colmap_var).grid(row=7, column=0, columnspan=2, sticky=tk.W, pady=(2, 0))
        self.capture_mode_var.trace_add('write', self._on_capture_mode_changed)

        self.start_button = ttk.Button(control_frame, text="Start System",
                                      command=self.start_fusion, style="Accent.TButton")
        self.start_button.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 2))
        
        # Capture button — reduced height on small screens
        _capture_ipady = 10 if self._small_screen else 30
        self.capture_button = ttk.Button(control_frame, text="CAPTURE SCAN",
                                        command=self.capture_scan, state="disabled",
                                        style="Large.TButton")
        self.capture_button.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 2), ipady=_capture_ipady)
        self.capture_button.configure(width=15)
        
        self.stop_button = ttk.Button(control_frame, text="Stop System", 
                                     command=self.stop_fusion, state="disabled")
        self.stop_button.grid(row=4, column=0, sticky=(tk.W, tk.E))
        
        # Scan info frame (compact)
        info_frame = ttk.LabelFrame(left_panel, text="Scan Info", padding="3" if self._small_screen else "5")
        info_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=_pady_section)
        
        ttk.Label(info_frame, text="Scans:").grid(row=0, column=0, sticky=tk.W)
        self.scan_count_label = ttk.Label(info_frame, text="0", font=('Arial', 12, 'bold'))
        self.scan_count_label.grid(row=0, column=1, sticky=tk.W, padx=(5, 0))
        
        ttk.Label(info_frame, text="Output:").grid(row=1, column=0, sticky=tk.W)
        self.output_dir_label = ttk.Label(info_frame, text="Not started", foreground="gray",
                                           wraplength=160 if self._small_screen else 200)
        self.output_dir_label.grid(row=1, column=1, sticky=tk.W, padx=(5, 0))

        # Latest scan thumbnail
        thumb_frame = ttk.LabelFrame(left_panel, text="Latest Scan", padding="3" if self._small_screen else "5")
        thumb_frame.grid(row=5, column=0, sticky=(tk.W, tk.E), pady=_pady_section)
        thumb_frame.columnconfigure(0, weight=1)
        self.thumb_label = ttk.Label(thumb_frame, text="No scan yet", foreground="gray", anchor='center')
        self.thumb_label.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        # Right panel: tabbed view — RViz2 | System Log
        notebook = ttk.Notebook(right_panel)
        notebook.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        _rviz_w = max(400, self.root.winfo_screenwidth() - 320)
        _rviz_h = max(300, self.root.winfo_screenheight() - 160)
        self.rviz_frame = tk.Frame(notebook, bg='black', width=_rviz_w, height=_rviz_h)
        self.rviz_frame.pack_propagate(False)
        self.rviz_frame.grid_propagate(False)
        self.rviz_frame.update_idletasks()
        notebook.add(self.rviz_frame, text="Viewer")

        log_tab = ttk.Frame(notebook, padding="5")
        self.log_text = scrolledtext.ScrolledText(log_tab, font=('Consolas', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        notebook.add(log_tab, text="System Log")

        # ── Post-Processing tab ──────────────────────────────────────────────
        pp_tab = ttk.Frame(notebook, padding="8")
        notebook.add(pp_tab, text="Post-Processing")

        # Session picker
        sess_frame = ttk.LabelFrame(pp_tab, text="Session", padding="5")
        sess_frame.pack(fill=tk.X, pady=(0, 6))
        self._pp_session_var = tk.StringVar(value="(use current session)")
        ttk.Entry(sess_frame, textvariable=self._pp_session_var, width=52).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(sess_frame, text="Browse…", command=self._pp_browse).pack(side=tk.LEFT, padx=(4, 0))

        # Buttons grid
        btn_frame = ttk.Frame(pp_tab)
        btn_frame.pack(fill=tk.BOTH, expand=True)
        btn_frame.columnconfigure(0, weight=1)
        btn_frame.columnconfigure(1, weight=1)

        def _pp_btn(row, col, label, cmd):
            ttk.Button(btn_frame, text=label, command=cmd).grid(
                row=row, column=col, sticky=(tk.W, tk.E), padx=3, pady=3)

        def _pp_sep(row, text):
            ttk.Separator(btn_frame, orient='horizontal').grid(
                row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=3, pady=(6, 0))
            ttk.Label(btn_frame, text=text, foreground='#888',
                      font=('Arial', 8)).grid(
                row=row+1, column=0, columnspan=2, sticky=tk.W, padx=6, pady=(0, 2))

        # ── Processing ────────────────────────────────────────────────────
        _pp_btn(0, 0, "Run Post-Processing",         self._pp_run_reconstruct)
        _pp_btn(0, 1, "Recolor with New Calibration", self._pp_reprocess)
        _pp_btn(1, 0, "ICP Alignment",                self._pp_icp)
        _pp_btn(1, 1, "Filter Blurry Frames",         self._pp_filter_blurry)
        _pp_btn(2, 0, "Merge (Trajectory Only)",      self._pp_merge_traj)
        _pp_btn(2, 1, "Run Sync Benchmark",           self._pp_sync_benchmark)
        _pp_btn(3, 0, "SDK Sync Validator",            self._pp_sdk_sync_validator)
        # ── COLMAP ────────────────────────────────────────────────────────
        _pp_sep(4, "COLMAP")
        _pp_btn(6, 0, "Export COLMAP Model",          self._pp_colmap)
        _pp_btn(6, 1, "COLMAP Pose Quality",          self._pp_colmap_quality)
        _pp_btn(7, 0, "Generate Depth Images",        self._pp_colmap_depth)
        # ── Viewers ───────────────────────────────────────────────────────
        _pp_sep(8, "Viewers")
        _pp_btn(10, 0, "View Point Cloud (Web)",       self._pp_web_viewer)
        _pp_btn(10, 1, "Per-Scan Alignment Viewer",    self._pp_toggle_viewer)
        _pp_btn(11, 0, "COLMAP Viewer",               self._pp_colmap_viewer)
        _pp_btn(11, 1, "Depth-RGB Overlay",           self._pp_depth_overlay)

        # Output log for post-processing tab
        self._pp_log = scrolledtext.ScrolledText(pp_tab, font=('Consolas', 8), height=10, state='disabled')
        self._pp_log.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        _pp_menu = tk.Menu(self.root, tearoff=0)
        _pp_menu.add_command(label="Copy",       command=lambda: self._pp_log.event_generate('<<Copy>>'))
        _pp_menu.add_command(label="Select All",  command=lambda: self._pp_log.tag_add(tk.SEL, '1.0', tk.END))
        _pp_menu.add_command(label="Clear Log",   command=lambda: (self._pp_log.config(state='normal'),
                                                                    self._pp_log.delete('1.0', tk.END),
                                                                    self._pp_log.config(state='disabled')))
        self._pp_log.bind('<Button-3>', lambda e: (_pp_menu.tk_popup(e.x_root, e.y_root),
                                                   _pp_menu.grab_release()))
        # ─────────────────────────────────────────────────────────────────────

        # ── Calibration tab ──────────────────────────────────────────────────
        cal_tab = ttk.Frame(notebook, padding="8")
        notebook.add(cal_tab, text="Calibration")

        # Session picker (reuses same var as post-processing)
        cal_sess_frame = ttk.LabelFrame(cal_tab, text="Session", padding="5")
        cal_sess_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Entry(cal_sess_frame, textvariable=self._pp_session_var, width=52).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(cal_sess_frame, text="Browse…", command=self._pp_browse).pack(side=tk.LEFT, padx=(4, 0))

        # Live indicator showing which camera is being calibrated
        cal_hw_frame = ttk.Frame(cal_tab)
        cal_hw_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(cal_hw_frame, text="Calibrating camera:",
                  font=('Arial', 9)).pack(side=tk.LEFT, padx=(0, 6))
        self._cal_hw_label = ttk.Label(cal_hw_frame,
            text="cam_0 (X5)",
            font=('Arial', 10, 'bold'), foreground='#1a6fb5')
        self._cal_hw_label.pack(side=tk.LEFT)

        # ── Options row ────────────────────────────────────────────
        opt_frame = ttk.Frame(cal_tab)
        opt_frame.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(opt_frame, text="Scene:").pack(side=tk.LEFT, padx=(0, 4))
        self._cal_scene_var = tk.StringVar(value="indoor")
        ttk.Combobox(opt_frame, textvariable=self._cal_scene_var,
                     values=["indoor", "outdoor"], state="readonly", width=9
                     ).pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(opt_frame, text="Initial guess:").pack(side=tk.LEFT, padx=(0, 4))
        self._cal_guess_var = tk.StringVar(value="auto")
        ttk.Combobox(opt_frame, textvariable=self._cal_guess_var,
                     values=["auto", "manual"], state="readonly", width=8
                     ).pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(opt_frame, text="Camera:").pack(side=tk.LEFT, padx=(0, 4))
        self._cal_cam_var = tk.StringVar(value="cam_0")
        ttk.Combobox(opt_frame, textvariable=self._cal_cam_var,
                     values=["cam_0", "cam_1", "cam_2"], state="readonly", width=7
                     ).pack(side=tk.LEFT)

        # Keep the calibrating-camera indicator in sync with the Camera dropdown
        def _on_cal_cam_change(*_):
            cam = self._cal_cam_var.get()
            try:
                import yaml as _y
                mc = self.script_dir / 'config' / 'multi_camera.yaml'
                if mc.exists():
                    cfg = _y.safe_load(open(mc))
                    hw = cfg.get('cameras', {}).get(cam, {}).get('camera_hw', '?')
                    self._cal_hw_label.config(text=f"{cam} ({hw.upper()})")
                    return
            except Exception:
                pass
            self._cal_hw_label.config(text=cam)
        self._cal_cam_var.trace_add('write', _on_cal_cam_change)
        _on_cal_cam_change()

        # ── Action buttons ──────────────────────────────────────────
        cal_btn_frame = ttk.Frame(cal_tab)
        cal_btn_frame.pack(fill=tk.BOTH, expand=False)
        cal_btn_frame.columnconfigure(0, weight=1)
        cal_btn_frame.columnconfigure(1, weight=1)

        def _cal_btn(row, col, label, cmd):
            ttk.Button(cal_btn_frame, text=label, command=cmd).grid(
                row=row, column=col, sticky=(tk.W, tk.E), padx=3, pady=3)

        def _cal_sep(row, text):
            ttk.Separator(cal_btn_frame, orient='horizontal').grid(
                row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=3, pady=(6, 0))
            ttk.Label(cal_btn_frame, text=text, foreground='#888',
                      font=('Arial', 8)).grid(
                row=row+1, column=0, columnspan=2, sticky=tk.W, padx=6, pady=(0, 2))

        # ── Physical Seed (run before calibration) ────────────────
        seed_frame = ttk.LabelFrame(cal_btn_frame, text="Physical Seed (stand behind scanner, cable facing you)")
        seed_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=3, pady=(3, 3))
        seed_frame.columnconfigure(1, weight=1)
        seed_frame.columnconfigure(3, weight=1)

        ttk.Label(seed_frame, text="Fwd \":").grid(row=0, column=0, sticky=tk.W, padx=2)
        self._seed_fwd_var = tk.StringVar(value="3.25")
        ttk.Entry(seed_frame, textvariable=self._seed_fwd_var, width=6).grid(row=0, column=1, padx=2)

        ttk.Label(seed_frame, text="Left \":").grid(row=0, column=2, sticky=tk.W, padx=2)
        self._seed_left_var = tk.StringVar(value="0.0")
        ttk.Entry(seed_frame, textvariable=self._seed_left_var, width=6).grid(row=0, column=3, padx=2)

        ttk.Label(seed_frame, text="Up \":").grid(row=1, column=0, sticky=tk.W, padx=2)
        self._seed_up_var = tk.StringVar(value="0.0")
        ttk.Entry(seed_frame, textvariable=self._seed_up_var, width=6).grid(row=1, column=1, padx=2)

        ttk.Label(seed_frame, text="Yaw \u00b0:").grid(row=1, column=2, sticky=tk.W, padx=2)
        self._seed_yaw_var = tk.StringVar(value="0.0")
        ttk.Entry(seed_frame, textvariable=self._seed_yaw_var, width=6).grid(row=1, column=3, padx=2)

        # Help text
        ttk.Label(seed_frame, text="+Fwd=front  -Fwd=back  |  +Left=left  -Left=right  |  Yaw: 0=fwd  +90=left  -90=right",
                  font=('Arial', 7), foreground='#666').grid(row=2, column=0, columnspan=4, sticky=tk.W, padx=2, pady=(2, 0))

        ttk.Button(seed_frame, text="Write Physical Seed", command=self._cal_write_physical_seed
                   ).grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=2, pady=(3, 3))
        ttk.Button(seed_frame, text="Interactive Viewer", command=self._cal_interactive_seed
                   ).grid(row=2, column=2, columnspan=2, sticky=(tk.W, tk.E), padx=2, pady=(3, 3))

        # \u2500\u2500 Full pipeline (one click) \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
        ttk.Button(cal_btn_frame, text="\u25b6  Run Full Calibration Pipeline",
                   command=self._cal_run_full_pipeline,
                   style="Accent.TButton").grid(
            row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=3, pady=(3, 6))

        # \u2500\u2500 Individual steps \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
        _cal_sep(2, "Individual Steps")
        _cal_btn(4, 0, "1. Combine Scans",                self._cal_combine_scans)
        _cal_btn(4, 1, "2. Generate Intensity Images",    self._cal_gen_intensity)
        _cal_btn(5, 0, "3. Match Features (SuperGlue)",   lambda: self._cal_superglue(self._cal_scene_var.get()))
        _cal_btn(5, 1, "4. Initial Guess",                self._cal_initial_guess)
        _cal_btn(6, 0, "5. Seed from Current Calib",      self._cal_seed)
        _cal_btn(6, 1, "6. Run Calibration",              self._cal_run_calibration)
        _cal_btn(7, 0, "7. Apply Calibration",            self._cal_apply)
        _cal_btn(7, 1, "8. Verify Calibration",           self._cal_verify)
        # \u2500\u2500 Fine-tune \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
        _cal_sep(8, "Fine-tune")
        _cal_btn(10, 0, "Tune \u2014 Sweep Pitch",             self._cal_tune_pitch)
        _cal_btn(10, 1, "Tune \u2014 Sweep Roll",              self._cal_tune_roll)
        _cal_btn(11, 0, "Tune \u2014 Sweep Yaw",              self._cal_tune_yaw)
        _cal_btn(11, 1, "Tune \u2014 Sweep TX/TY/TZ",         self._cal_tune_t)
        # ── Multi-camera color ────────────────────────────────────────
        _cal_sep(12, "Multi-Camera Color")
        _cal_btn(14, 0, "Calibrate Color Profiles",       self._cal_color_calibrate)
        _cal_btn(14, 1, "Apply Color Normalization",      self._cal_color_normalize)

        # Output log shared with post-processing tab
        ttk.Label(cal_tab, text="Output:", foreground='#888', font=('Arial', 8)).pack(anchor=tk.W, pady=(6,0))
        self._cal_log = scrolledtext.ScrolledText(cal_tab, font=('Consolas', 8), height=12, state='disabled')
        self._cal_log.pack(fill=tk.BOTH, expand=True)
        _cal_menu = tk.Menu(self.root, tearoff=0)
        _cal_menu.add_command(label="Copy",      command=lambda: self._cal_log.event_generate('<<Copy>>'))
        _cal_menu.add_command(label="Select All", command=lambda: self._cal_log.tag_add(tk.SEL, '1.0', tk.END))
        _cal_menu.add_command(label="Clear Log",  command=lambda: (self._cal_log.config(state='normal'),
                                                                    self._cal_log.delete('1.0', tk.END),
                                                                    self._cal_log.config(state='disabled')))
        self._cal_log.bind('<Button-3>', lambda e: (_cal_menu.tk_popup(e.x_root, e.y_root),
                                                    _cal_menu.grab_release()))
        # ─────────────────────────────────────────────────────────────────────

        # ── Coverage tab ───────────────────────────────────────────────────────────
        cov_tab = ttk.Frame(notebook)
        notebook.insert(1, cov_tab, text='Coverage')
        self._coverage_panel = CoveragePanel(cov_tab)
        # ───────────────────────────────────────────────────────────────────

        notebook.select(1)  # start on Coverage tab

        # Switch to log tab on new message if viewer is active, badge the tab
        self._log_tab_index = 2
        self._notebook = notebook
        notebook.bind('<<NotebookTabChanged>>', self._on_tab_changed)

        self.context_menu = tk.Menu(self.root, tearoff=0)
        self.context_menu.add_command(label="Copy", command=self.copy_text)
        self.context_menu.add_command(label="Select All", command=self.select_all_text)
        self.log_text.bind("<Button-3>", self.show_context_menu)

        # Configure grid weights for responsive layout
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=0)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        left_panel.columnconfigure(0, weight=1)
        control_frame.columnconfigure(0, weight=1)

        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=1)
        
    def log_message(self, message):
        """Add message to log with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        logging.info(message)
        # Badge the tab if it's not currently selected
        if self._notebook.index('current') != self._log_tab_index:
            self._notebook.tab(self._log_tab_index, text="System Log ●")

    def _on_tab_changed(self, event):
        if self._notebook.index('current') == self._log_tab_index:
            self._notebook.tab(self._log_tab_index, text="System Log")
        
    def show_context_menu(self, event):
        """Show context menu on right-click"""
        try:
            self.context_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.context_menu.grab_release()
            
    def copy_text(self):
        """Copy selected text to clipboard"""
        try:
            selected_text = self.log_text.selection_get()
            self.root.clipboard_clear()
            self.root.clipboard_append(selected_text)
        except tk.TclError:
            pass  # No text selected
            
    def select_all_text(self):
        """Select all text in log"""
        self.log_text.tag_add(tk.SEL, "1.0", tk.END)
        self.log_text.mark_set(tk.INSERT, "1.0")
        self.log_text.see(tk.INSERT)
        
    def update_status(self, status, color="black"):
        """Update status label"""
        self.status_label.config(text=status, foreground=color)
        
    def start_fusion(self):
        """Start the fusion system"""
        if self.is_running:
            return
        
        # Skip sudo check - assume permissions are already set up
        self.sudo_password = None
            
        # Reset scan count and session state for new session
        self.scan_count = 0
        self.scan_count_label.config(text="0")
        self._session_log_attached = False
        self.scan_dir = None
        self._latest_thumb_photo = None
        self.thumb_label.config(image='', text='No scan yet', foreground='gray')
        self._stale_rviz_wids = set()  # clear stale window IDs so new RViz can be found
        self.rviz_process = None
        self.rviz_win_id = None
        self.rviz_outer_win_id = None
        self._rviz_search_start = time.monotonic()  # reset so PID filter uses new session time
        # Clear any web viewer or previous RViz content from the viewer frame
        if self.web_viewer_process:
            self.web_viewer_process.set()  # signal _tick to stop and release GL context
            self.web_viewer_process = None
            # Wait two tick cycles (2 × 50ms) for the renderer to be released
            # before destroying the canvas, then clear on the next after() call.
            self.root.after(150, self._clear_viewer_frame)
        else:
            self._clear_viewer_frame()
        
        self.log_message("Starting Automated Terrestrial LiDAR Acquisition System (ATLAS)...")
        self.update_status("Starting system...", "orange")
        
        # Disable start button, enable stop button
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        
        # Snapshot all Tkinter StringVar/BooleanVar values on the main thread
        # before spawning — StringVar.get() is not thread-safe and can return
        # stale values (e.g. 'stationary') when called from a background thread.
        _launch_params = {
            'camera_val':        self.camera_mode_var.get(),
            'capture_val':       self.capture_mode_var.get(),
            'camera_hw_val':     self.camera_hw_var.get(),
            'num_cameras_val':   self.num_cameras_var.get(),
            'bag_only':          self.bag_only_var.get(),
            'stationary_wait':   self.stationary_wait_var.get(),
            'icp':               self.icp_var.get(),
            'colmap':            self.colmap_var.get(),
        }
        # Start fusion process in separate thread
        threading.Thread(target=self._run_fusion_process, args=(_launch_params,), daemon=True).start()
        
    def _run_fusion_process(self, params):
        """Run the fusion process (called in background thread; use `params` dict, not StringVars)"""
        try:
            # Prepare environment — ensure display vars are forwarded so
            # web_3d_viewer.py can open a browser window from within the script
            env = os.environ.copy()
            env['SKIP_SUDO_CHECK'] = '1'
            env['ATLAS_GUI_MODE'] = '1'
            env['DISPLAY'] = os.environ.get('DISPLAY', ':0')
            env['XAUTHORITY'] = os.environ.get('XAUTHORITY', '')
            env['DBUS_SESSION_BUS_ADDRESS'] = os.environ.get('DBUS_SESSION_BUS_ADDRESS', 'unix:path=/run/user/1000/bus')
            env['XDG_RUNTIME_DIR'] = os.environ.get('XDG_RUNTIME_DIR', '/run/user/1000')
            wayland = os.environ.get('_ATLAS_WAYLAND_DISPLAY') or os.environ.get('WAYLAND_DISPLAY', '')
            if wayland:
                env['WAYLAND_DISPLAY'] = wayland

            # Apply camera permissions — skip pkexec entirely if the udev rule is
            # already installed and the device is accessible (the common case after
            # first-time setup).  Only invoke pkexec when the rule file is missing,
            # which means this is a fresh install and a one-time password prompt is
            # acceptable.
            perm_script = os.path.realpath(
                os.path.join(os.path.dirname(os.path.abspath(__file__)), 'setup_camera_permissions.sh'))
            # Validate the script resolves within the expected source tree
            _src_root = os.path.realpath(os.path.dirname(os.path.abspath(__file__)))
            if not perm_script.startswith(_src_root + os.sep) and perm_script != _src_root:
                self.log_message('⚠ Camera permissions script path is outside source tree — aborted')
                return
            udev_rule = '/etc/udev/rules.d/99-insta.rules'
            dev_accessible = os.access('/dev/insta', os.R_OK | os.W_OK) if os.path.exists('/dev/insta') else False
            rule_installed = os.path.exists(udev_rule)

            if dev_accessible:
                self.root.after(0, lambda: self.log_message("✓ Camera permissions ready"))
            else:
                self.root.after(0, lambda: self.log_message("Setting up camera permissions..."))
                perm_done = threading.Event()

                def _run_perms():
                    try:
                        if rule_installed:
                            result = subprocess.run(
                                ['udevadm', 'trigger', '--subsystem-match=usb'],
                                text=True, timeout=10)
                            ok = result.returncode == 0
                        else:
                            if not os.access(perm_script, os.X_OK):
                                os.chmod(perm_script, 0o755)
                            result = subprocess.run(['pkexec', perm_script], text=True, timeout=120)
                            ok = result.returncode == 0
                    except subprocess.TimeoutExpired:
                        ok = False
                        self.root.after(0, lambda: self.log_message('⚠ Camera permissions timed out'))
                    except FileNotFoundError as e:
                        ok = False
                        self.root.after(0, lambda m=str(e): self.log_message(f'⚠ Camera permissions command not found: {m}'))
                    except PermissionError as e:
                        ok = False
                        self.root.after(0, lambda m=str(e): self.log_message(f'⚠ Camera permissions denied: {m}'))
                    msg = "✓ Camera permissions set" if ok else "⚠ Camera permissions failed — check polkit/pkexec"
                    self.root.after(0, lambda m=msg: self.log_message(m))
                    perm_done.set()

                threading.Thread(target=_run_perms, daemon=True).start()

            # Validate GUI-supplied values before building the command.
            # Use pre-snapshotted params (captured on main thread) — StringVar.get()
            # is not thread-safe and can return stale values from a background thread.
            _VALID_CAMERA  = {'dual_fisheye', 'single_fisheye'}
            _VALID_CAPTURE = {'stationary', 'continuous'}
            camera_val  = params['camera_val']
            capture_val = params['capture_val']
            if camera_val not in _VALID_CAMERA:
                self.log_message(f'Invalid camera mode: {camera_val!r}')
                return
            if capture_val not in _VALID_CAPTURE:
                self.log_message(f'Invalid capture mode: {capture_val!r}')
                return

            # Start the fusion script
            cmd = ['stdbuf', '-oL', './atlas_fusion_capture.sh',
                   '--camera', camera_val,
                   '--capture', capture_val,
                   '--camera-hw', params['camera_hw_val']]
            if params['num_cameras_val'] != 'auto':
                cmd += ['--num-cameras', params['num_cameras_val']]
            if params['bag_only']:
                cmd.append('--bag-only')
            if capture_val == 'stationary' and params['stationary_wait']:
                cmd.append('--stationary-wait')
            cmd.append('--icp' if params['icp'] else '--no-icp')
            cmd.append('--colmap' if params['colmap'] else '--no-colmap')
            self.fusion_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=0,
                env=env,
                preexec_fn=os.setsid  # Create new process group
            )
            # Wrap in a text reader that replaces undecodable bytes (e.g. raw
            # libusb address bytes from the Insta360 SDK log) instead of raising.
            import io
            stdout_text = io.TextIOWrapper(
                self.fusion_process.stdout, encoding='utf-8', errors='replace', line_buffering=True
            )

            self.is_running = True
            self.root.after(0, lambda: self._coverage_panel.start(
                getattr(self, 'scan_dir', None)))

            # Read output in real-time
            while self.fusion_process and self.is_running:
                try:
                    line = stdout_text.readline()
                    if line == '':  # EOF
                        break
                    line = line.strip()
                    if not line:
                        continue
                    self.root.after(0, lambda l=line: self.log_message(l))
                    if not getattr(self, '_session_log_attached', False):
                        import re
                        m = re.search(r'synchronized_scans/(sync_fusion_[\w]+)', line)
                        if m:
                            session_dir = self.script_dir / '../../..' / 'data' / 'synchronized_scans' / m.group(1)
                            self.scan_dir = str(session_dir.resolve())
                            self._pp_session_var.set(self.scan_dir)
                            _sd = self.scan_dir
                            self.root.after(0, lambda d=_sd: self._coverage_panel.set_scan_dir(d))
                            try:
                                _add_session_log_handler(self.scan_dir)
                                self._session_log_attached = True
                            except (OSError, PermissionError):
                                pass
                    if "FUSION CAPTURE READY" in line:
                        self.root.after(0, self._system_ready)
                    elif "Press ENTER to capture" in line:
                        self.root.after(0, self._system_ready)
                    elif line.startswith("✓ Scan") and "completed:" in line:
                        self.root.after(0, self._scan_completed)
                    elif "✓ Scan saved to:" in line:
                        self.root.after(0, self._scan_completed)
                    elif "_viewer.html" in line or ("3D viewer" in line and ".html" in line):
                        import re as _re, os as _os
                        m = _re.search(r'(/[^\s]+_viewer\.html)', line)
                        if m:
                            _candidate = _os.path.realpath(m.group(1))
                            _allowed = _os.path.realpath(_os.path.expanduser('~/atlas_ws/data'))
                            if _candidate.startswith(_allowed + _os.sep) or _candidate == _allowed:
                                self._pending_viewer_html = _candidate
                    elif "Session ended with no scans captured" in line:
                        self.root.after(0, lambda: self.update_status("No scans captured — stopped before triggering a scan", "orange"))
                    elif any(err in line for err in (
                        "✗ Camera failed", "✗ LiDAR", "Failed to start",
                        "failed to start", "exit 1", "not found at /dev/insta",
                        "Failed to initialize sensors"
                    )):
                        self.root.after(0, lambda l=line: self.update_status(f"Error: {l}", "red"))
                except Exception as e:
                    self.root.after(0, lambda e=e: self.log_message(f"Error reading output: {e}"))
                    break

            # Process exited — drain any remaining output before resetting
            try:
                remaining = stdout_text.read()
                if remaining:
                    for line in remaining.splitlines():
                        line = line.strip()
                        if not line:
                            continue
                        self.root.after(0, lambda l=line: self.log_message(l))
            except Exception:
                pass

            # Surface any error and reset buttons — only if stop_fusion hasn't
            # already scheduled _system_stopped via its own _wait_for_finish thread.
            if self.is_running:
                exit_code = self.fusion_process.poll() if self.fusion_process else -1
                if exit_code not in (None, 0):
                    self.root.after(0, lambda c=exit_code: self.update_status(
                        f"Script exited with error (code {c}) — check log", "red"))
                self.root.after(500, self._system_stopped)
                        
        except Exception as ex:
            error_msg = f"Error starting fusion: {ex}"
            self.root.after(0, lambda: self.log_message(error_msg))
            self.fusion_process = None
            self.root.after(0, self._system_stopped)
            
    def embed_web_viewer(self, html_path):
        """Render the point cloud directly in the Viewer tab using Open3D offscreen renderer."""
        import os as _os
        _allowed = _os.path.realpath(_os.path.expanduser('~/atlas_ws/data'))
        safe_html = _os.path.realpath(html_path)
        if not (safe_html.startswith(_allowed + _os.sep) or safe_html == _allowed):
            self.log_message(f'⚠ Viewer path outside allowed root: {html_path!r}')
            return

        if self.web_viewer_process:
            self.web_viewer_process.set()  # stop previous viewer if any
            self.web_viewer_process = None

        ply_path = _os.path.realpath(safe_html.replace('_viewer.html', '.ply'))
        if not (ply_path.startswith(_allowed + _os.sep) or ply_path == _allowed):
            self.log_message(f'⚠ PLY path outside allowed root: {ply_path!r}')
            return
        if not _os.path.exists(ply_path):
            self.log_message(f'⚠ PLY file not found: {ply_path}')
            return

        self._notebook.select(0)
        self.root.update_idletasks()

        # Clear the frame and embed the canvas viewer directly
        for child in self.rviz_frame.winfo_children():
            child.destroy()

        try:
            from post_processing.web_viewer_embed import embed_viewer
        except ImportError:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                'web_viewer_embed',
                str(self.script_dir / 'post_processing' / 'web_viewer_embed.py')
            )
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            embed_viewer = mod.embed_viewer

        self.web_viewer_process = embed_viewer(self.rviz_frame, ply_path)
        self.log_message('✓ 3D viewer loaded in panel')
        self.update_status('Session complete — 3D viewer ready', 'blue')

    def _clear_viewer_frame(self):
        """Destroy all children of rviz_frame safely."""
        for child in self.rviz_frame.winfo_children():
            try:
                child.destroy()
            except Exception:
                pass

    def _on_capture_mode_changed(self, *_):
        """Enable/disable stationary-only options when capture mode changes."""
        if self.capture_mode_var.get() == 'stationary':
            self.stationary_wait_cb.config(state='normal')
        else:
            self.stationary_wait_cb.config(state='disabled')

    def _is_continuous_mode(self):
        return self.capture_mode_var.get() == 'continuous'

    def embed_rviz(self):
        """Watch for the RViz2 window launched by the fusion script and embed it"""
        self.rviz_process = True  # sentinel — set immediately to prevent double-call
        self._rviz_search_start = time.monotonic()
        self._notebook.select(1)  # switch to Coverage tab when session starts

        def _start_when_ready(attempts=0):
            self.root.update_idletasks()
            frame_id = self.rviz_frame.winfo_id()
            if frame_id == 0 and attempts < 15:
                self.root.after(100, lambda: _start_when_ready(attempts + 1))
                return
            threading.Thread(target=self._reparent_rviz, args=(frame_id,), daemon=True).start()

        _start_when_ready()

    def _x11_embed(self, embed_wid_int, frame_id_int, w, h):
        """Reparent embed_wid into frame_id using xdotool + Xlib."""
        import tempfile, os as _os
        # Unmap from WM first to avoid SIGSEGV on XReparentWindow
        subprocess.run(['xdotool', 'windowunmap', '--sync', str(embed_wid_int)],
                       capture_output=True, timeout=5)
        code = f"""import ctypes, ctypes.util, sys
lib = ctypes.CDLL(ctypes.util.find_library('X11'))
vp = ctypes.c_void_p
lib.XOpenDisplay.restype = vp; lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
lib.XCloseDisplay.argtypes = [vp]
lib.XSync.argtypes = [vp, ctypes.c_int]
lib.XReparentWindow.argtypes = [vp, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_int, ctypes.c_int]
lib.XResizeWindow.argtypes = [vp, ctypes.c_ulong, ctypes.c_uint, ctypes.c_uint]
lib.XMapWindow.argtypes = [vp, ctypes.c_ulong]
lib.XMoveWindow.argtypes = [vp, ctypes.c_ulong, ctypes.c_int, ctypes.c_int]
lib.XSetErrorHandler.restype = vp
ok = [True]
CB = ctypes.CFUNCTYPE(ctypes.c_int, vp, vp)
cb = CB(lambda d, e: (ok.__setitem__(0, False), 0)[1])
lib.XSetErrorHandler(cb)
d = lib.XOpenDisplay(None)
if not d: sys.exit(2)
lib.XReparentWindow(d, {embed_wid_int}, {frame_id_int}, 0, 0)
lib.XMoveWindow(d, {embed_wid_int}, 0, 0)
lib.XResizeWindow(d, {embed_wid_int}, {w}, {h})
lib.XMapWindow(d, {embed_wid_int})
lib.XSync(d, False)
lib.XCloseDisplay(d)
print('ok' if ok[0] else 'x11err', flush=True)
sys.exit(0 if ok[0] else 4)
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(code)
            tmp = f.name
        try:
            env = _os.environ.copy()
            env.setdefault('DISPLAY', ':0')
            result = subprocess.run([sys.executable, tmp], env=env,
                                    capture_output=True, timeout=10)
            if result.returncode != 0:
                self.root.after(0, lambda rc=result.returncode, out=result.stdout.strip():
                    self.log_message(f"[embed] subprocess rc={rc} stdout={out}"))
            return result.returncode == 0
        finally:
            _os.unlink(tmp)

    def _x11_resize(self, wid_int, w, h):
        script = (
            "import ctypes, ctypes.util\n"
            "lib=ctypes.CDLL(ctypes.util.find_library('X11'));vp=ctypes.c_void_p\n"
            "lib.XOpenDisplay.restype=vp;lib.XOpenDisplay.argtypes=[ctypes.c_char_p]\n"
            "lib.XCloseDisplay.argtypes=[vp];lib.XFlush.argtypes=[vp]\n"
            "lib.XResizeWindow.argtypes=[vp,ctypes.c_ulong,ctypes.c_uint,ctypes.c_uint]\n"
            f"d=lib.XOpenDisplay(None)\n"
            "if not d: import sys;sys.exit(1)\n"
            f"lib.XResizeWindow(d,{wid_int},{w},{h})\n"
            "lib.XFlush(d);lib.XCloseDisplay(d)\n"
        )
        env = os.environ.copy()
        env.setdefault('DISPLAY', ':0')
        subprocess.Popen([sys.executable, '-c', script], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _reparent_rviz(self, frame_id):
        """Poll for the RViz2 main window then reparent its Qt child into rviz_frame"""
        import ctypes
        outer_win_id = None

        known_stale = getattr(self, '_stale_rviz_wids', set())
        search_start_wall = time.time() - (time.monotonic() - self._rviz_search_start)

        # Cache btime once — it never changes between polls
        try:
            btime = int(open('/proc/stat').read().split('btime')[1].split()[0])
            clk_tck = ctypes.CDLL(None).sysconf(2)  # _SC_CLK_TCK = 2
        except Exception:
            btime, clk_tck = None, None

        # Poll with a fast initial rate, backing off after the first 10 attempts
        for attempt in range(120):
            time.sleep(0.3 if attempt < 10 else 1.0)
            result = subprocess.run(['xdotool', 'search', '--classname', 'rviz2'],
                                    capture_output=True, text=True)
            candidates = []
            for wid in result.stdout.strip().split():
                if not wid.isdigit():
                    continue  # skip any non-integer token from xdotool output
                if wid in known_stale:
                    continue
                name = subprocess.run(['xdotool', 'getwindowname', wid],
                                      capture_output=True, text=True).stdout.strip()
                if not name.endswith('- RViz2') and not name.endswith('- RViz'):
                    continue
                # Only accept windows whose owning PID started after this session
                pid_out = subprocess.run(['xdotool', 'getwindowpid', wid],
                                         capture_output=True, text=True).stdout.strip()
                if not pid_out.isdigit():
                    continue
                if btime is not None and clk_tck:
                    try:
                        with open(f'/proc/{pid_out}/stat') as f:
                            fields = f.read().split(')')
                            start_ticks = int(fields[-1].split()[19])
                            pid_start_wall = btime + start_ticks / clk_tck
                            if pid_start_wall < search_start_wall - 30:
                                continue  # process predates this session by more than 30s
                    except Exception:
                        continue
                candidates.append(wid)
            if candidates:
                outer_win_id = candidates[-1]
                break

        if not outer_win_id:
            self.root.after(0, lambda: self.log_message("⚠ Could not find RViz2 window to embed"))
            return

        # Brief settle time — reduced from 1.0s; RViz just needs to finish mapping
        time.sleep(0.2)

        self.root.after(0, lambda: self.log_message(
            f"[embed] outer={outer_win_id} frame_id={frame_id}"))

        self.rviz_win_id = outer_win_id
        self.rviz_outer_win_id = outer_win_id
        self._resize_after_id = None
        self._rviz_last_size = (0, 0)

        def _initial_size(attempt=0):
            w = self.rviz_frame.winfo_width()
            h = self.rviz_frame.winfo_height()
            self._rviz_last_size = (w, h)
            self.log_message(f"[embed] frame size={w}x{h}, calling _x11_embed (attempt {attempt})")
            ok = self._x11_embed(int(self.rviz_win_id, 0), frame_id, w, h)
            self.log_message(f"[embed] _x11_embed returned {ok}")
            if not ok and attempt < 5:
                self.root.after(1000, lambda: _initial_size(attempt + 1))
                return
            self.rviz_frame.bind('<Configure>', self._on_rviz_frame_resize)
            if ok:
                self.log_message("✓ RViz2 embedded")
            else:
                self.log_message("⚠ RViz2 embed failed after retries")

        self.root.after(0, _initial_size)

    def _on_rviz_frame_resize(self, event):
        if not self.rviz_win_id or event.width < 10 or event.height < 10:
            return
        w, h = event.width, event.height
        if (w, h) == self._rviz_last_size:
            return
        self._rviz_last_size = (w, h)
        if self._resize_after_id:
            self.root.after_cancel(self._resize_after_id)
        wid_int = int(self.rviz_win_id, 0)
        self._resize_after_id = self.root.after(150, lambda: self._x11_resize(wid_int, w, h))
    def _system_ready(self):
        """Called when system is ready"""
        self._system_ready_time = time.time()
        self.update_status("System Ready - Ready to capture scans", "green")
        self.log_message("✓ System is ready for scanning!")
        if self.rviz_process is None:
            self.embed_rviz()

        # Only enable capture button in stationary mode
        if self._is_continuous_mode():
            self.capture_button.config(state="disabled")
            self.update_status("Continuous mode — press Stop when done", "green")
            self._seen_shutter_events = set()
            self.root.after(1000, self._continuous_poll)
        else:
            self.capture_button.config(state="normal")

        # Extract output directory from logs and set scan_dir
        log_content = self.log_text.get("1.0", tk.END)
        for line in log_content.split('\n'):
            if "Scans will be saved to:" in line:
                output_dir = line.split("Scans will be saved to:")[-1].strip()
                self.output_dir_label.config(text=output_dir, foreground="black")
                if not self.scan_dir and output_dir:
                    self.scan_dir = output_dir
                try:
                    _add_session_log_handler(output_dir)
                except (OSError, PermissionError):
                    pass
                break
                
    def _check_disk_space(self, warn_gb=5.0):
        """Warn if free disk space on the data partition is below warn_gb."""
        import shutil
        try:
            # Check the actual data directory, not the source directory
            check_path = self.scan_dir if (hasattr(self, 'scan_dir') and self.scan_dir) else str(self.script_dir / '../../../data')
            free_gb = shutil.disk_usage(check_path).free / 1e9
            if free_gb < warn_gb:
                self.log_message(f"⚠ Low disk space: {free_gb:.1f} GB free — consider freeing space before next scan")
                self.update_status(f"⚠ Low disk space ({free_gb:.1f} GB free)", "orange")
                return False
        except OSError:
            pass
        return True

    def _scan_completed(self):
        """Called when a scan is completed"""
        self.scan_count += 1
        self.scan_count_label.config(text=str(self.scan_count))
        self.log_message(f"✓ Scan {self.scan_count} completed successfully!")

        # Update thumbnail with the latest fisheye image
        if hasattr(self, 'scan_dir') and self.scan_dir:
            threading.Thread(target=self._load_latest_thumbnail, daemon=True).start()

        # Re-enable capture button after scan completion
        self.capture_button.config(state="normal")
        if self._check_disk_space():
            self.update_status("System Ready - Ready to capture scans", "green")

    def _load_latest_thumbnail(self):
        """Find the most recent captured image and update the thumbnail."""
        import time as _time
        try:
            if not self.scan_dir:
                return
            scan_path = pathlib.Path(self.scan_dir)
            if not scan_path.exists():
                return
            images = None
            for pattern in [
                'fusion_scan_*/equirect_*_masked.png',
                'fusion_scan_*/equirect_dual_fisheye.jpg',
                'fusion_scan_*/equirect_*.jpg',
                'fusion_scan_*/dual_fisheye_*.png',
                'fusion_scan_*/fisheye_*.jpg',
                'fusion_scan_*/dual_fisheye_*.jpg',
            ]:
                try:
                    found = sorted(scan_path.glob(pattern), key=lambda p: p.stat().st_mtime)
                except Exception:
                    continue
                if found:
                    images = found
                    break
            if not images:
                return
            img_path = images[-1]
            # Wait for file to finish writing (size stable for 0.5s)
            prev_size = -1
            for _ in range(10):
                try:
                    cur_size = img_path.stat().st_size
                except OSError:
                    return
                if cur_size == prev_size and cur_size > 0:
                    break
                prev_size = cur_size
                _time.sleep(0.1)
            img = Image.open(img_path)
            img.load()  # force full decode before leaving background thread
            thumb_w = 200 if not self._small_screen else 160
            ratio = thumb_w / img.width
            thumb_h = int(img.height * ratio)
            img = img.convert('RGB').resize((thumb_w, thumb_h), Image.LANCZOS)
            def _update(img=img):
                try:
                    if not self.root.winfo_exists():
                        return
                    photo = ImageTk.PhotoImage(img)
                    self._latest_thumb_photo = photo
                    self.thumb_label.config(image=photo, text='')
                except (tk.TclError, OSError):
                    pass
            self.root.after(0, _update)
        except (OSError, Image.UnidentifiedImageError):
            pass
        
    def capture_scan(self):
        """Trigger a scan capture"""
        if not self.is_running or not self.fusion_process:
            self.log_message("Cannot capture scan: system not ready")
            return

        self.log_message(f"Triggering scan {self.scan_count + 1}...")
        self.update_status(f"Capturing scan {self.scan_count + 1}...", "orange")

        # Disable capture button during scan
        self.capture_button.config(state="disabled")

        # Write trigger file for GUI mode; also write to stdin as fallback for terminal mode
        try:
            if hasattr(self, 'scan_dir') and self.scan_dir:
                trigger = pathlib.Path(self.scan_dir) / '.capture_trigger'
                trigger.touch()
            if self.fusion_process and self.fusion_process.stdin:
                self.fusion_process.stdin.write(b'\n')
                self.fusion_process.stdin.flush()
        except Exception as e:
            self.log_message(f"Error triggering scan: {e}")
            self.capture_button.config(state="normal")

    def stop_fusion(self):
        """Stop the fusion system"""
        if not self.is_running:
            return

        # Guard against accidental stop within 5s of system becoming ready
        # (can happen from stray keyboard events during RViz embedding)
        _ready_time = getattr(self, '_system_ready_time', 0)
        if time.time() - _ready_time < 5.0:
            self.log_message("⚠ Ignoring stop request (system just became ready — wait 5s)")
            return

        # Log the call stack to help diagnose unexpected stops
        import traceback
        caller = ''.join(traceback.format_stack()[-3:-1]).strip()
        logging.info(f"stop_fusion called from:\n{caller}")

        self.log_message("Stopping fusion system...")
        self.update_status("Processing and shutting down...", "orange")
        
        # Write quit trigger file for GUI mode; also write to stdin as fallback
        proc = self.fusion_process
        if proc:
            try:
                if hasattr(self, 'scan_dir') and self.scan_dir:
                    (pathlib.Path(self.scan_dir) / '.quit_trigger').touch()
                proc.stdin.write(b'q\n')
                proc.stdin.flush()
            except Exception as e:
                self.log_message(f"Error sending stop signal: {e}")

        def _wait_for_finish():
            # Poll until the process exits, using a silence-based watchdog rather
            # than a fixed wall-clock timeout. As long as the script is writing
            # output (coloring, ICP, benchmark, ...) the deadline keeps resetting.
            # Only trigger if the process goes completely silent for SILENCE_LIMIT
            # seconds, which catches genuine hangs without cutting off slow steps.
            SILENCE_LIMIT = 600  # seconds of no output before force-kill
            import time
            last_output = time.monotonic()

            while True:
                if proc.poll() is not None:
                    self.root.after(0, lambda: self.log_message("✓ Post-processing complete"))
                    break
                # Check whether the log file has grown since last poll as a proxy
                # for output activity (stdout is already consumed by the reader thread).
                try:
                    if hasattr(self, 'scan_dir') and self.scan_dir:
                        log_path = pathlib.Path(self.scan_dir) / 'session.log'
                        if log_path.exists() and log_path.stat().st_mtime > last_output:
                            last_output = log_path.stat().st_mtime
                except OSError:
                    pass
                if time.monotonic() - last_output > SILENCE_LIMIT:
                    self.root.after(0, lambda: self.log_message(
                        f"Post-processing timeout ({SILENCE_LIMIT}s silence), force stopping..."))
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        pass
                    break
                time.sleep(2)
            self.root.after(500, self._system_stopped)

        threading.Thread(target=_wait_for_finish, daemon=True).start()
        
    def _cleanup_ros_processes(self):
        """Clean up any remaining scanner processes"""
        try:
            for process in [
                "insta360_capture", "insta360_stitch",
                "livox_ros_driver2", "rko_lio", "static_transform_publisher",
                "trajectory_recorder", "livox_ros_driver2/lib"
            ]:
                import re as _re
                subprocess.run(["pkill", "-9", "-f", _re.escape(process)], capture_output=True)
            self.log_message("✓ Processes cleaned up")
        except Exception as e:
            self.log_message(f"Warning: Could not clean up all processes: {e}")
    
    def _continuous_poll(self):
        """Poll for new shutter events during continuous capture and update scan count."""
        if not self.is_running or not self._is_continuous_mode():
            return
        if hasattr(self, 'scan_dir') and self.scan_dir:
            import pathlib
            seen = getattr(self, '_seen_shutter_events', set())
            for se in pathlib.Path(self.scan_dir).glob('fusion_scan_*/capture_*.shutter_event'):
                if str(se) not in seen:
                    seen.add(str(se))
                    self.scan_count += 1
                    self.scan_count_label.config(text=str(self.scan_count))
                    self.log_message(f"📷 Shot {self.scan_count} captured")
            self._seen_shutter_events = seen
        self.root.after(1000, self._continuous_poll)

    # ── Post-Processing tab helpers ───────────────────────────────────────────

    def _pp_session(self):
        """Return the validated session directory to operate on."""
        import os as _os
        _allowed = _os.path.realpath(_os.path.expanduser('~/atlas_ws/data'))
        val = self._pp_session_var.get().strip()
        candidate = val if (val and val != "(use current session)") else getattr(self, 'scan_dir', None)
        if not candidate:
            return None
        resolved = _os.path.realpath(candidate)
        if not (resolved.startswith(_allowed + _os.sep) or resolved == _allowed):
            self._pp_log_write(f"\n[!] Session path is outside allowed data root: {candidate!r}\n")
            return None
        return resolved

    def _pp_browse(self):
        import tkinter.filedialog as _fd
        d = _fd.askdirectory(title="Select session directory",
                             initialdir=str(self.script_dir / '../../../data/synchronized_scans'))
        if d:
            self._pp_session_var.set(d)

    def _pp_log_write(self, text):
        self._pp_log.config(state='normal')
        self._pp_log.insert(tk.END, text)
        self._pp_log.see(tk.END)
        self._pp_log.config(state='disabled')

    def _pp_run(self, label, cmd):
        """Run a post-processing command in a background thread, streaming output to _pp_log."""
        sess = self._pp_session()
        if not sess:
            self._pp_log_write("\n[!] No session selected.\n")
            return
        safe_label = label.replace('\n', ' ').replace('\r', ' ')
        safe_cmd   = ' '.join(a.replace('\n', ' ').replace('\r', ' ') for a in cmd)
        self._pp_log_write(f"\n>> {safe_label}\n   {safe_cmd}\n")
        def _run():
            import subprocess as _sp
            import os as _os
            # Validate cmd[0] is the known-safe interpreter to prevent PATH hijacking.
            _safe_interp = _os.path.realpath(sys.executable)
            _actual_interp = _os.path.realpath(cmd[0]) if cmd else ''
            if _actual_interp != _safe_interp:
                self.root.after(0, self._pp_log_write,
                                f"Error: rejected unsafe interpreter: {cmd[0]!r}\n")
                return
            try:
                # Build a minimal, explicit environment rather than inheriting
                # os.environ wholesale — prevents PYTHONPATH, LD_PRELOAD and
                # LD_LIBRARY_PATH from being used to hijack the subprocess.
                _safe_env = {
                    k: os.environ[k] for k in (
                        'PATH', 'HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL',
                        'ROS_DISTRO', 'ROS_VERSION', 'AMENT_PREFIX_PATH',
                        'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                        'LD_LIBRARY_PATH', 'PYTHONPATH',
                        'DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR',
                        'DBUS_SESSION_BUS_ADDRESS', '_ATLAS_WAYLAND_DISPLAY',
                        'WAYLAND_DISPLAY',
                    ) if k in os.environ
                }
                _safe_env['PYTHONUNBUFFERED'] = '1'
                _safe_env['ATLAS_GUI_MODE'] = '1'
                proc = _sp.Popen(cmd, stdout=_sp.PIPE, stderr=_sp.STDOUT, text=True, bufsize=1,
                                  env=_safe_env)
                for line in proc.stdout:
                    safe_line = line.replace('\r', '').rstrip('\n') + '\n'
                    self.root.after(0, self._pp_log_write, safe_line)
                    if '3D viewer ready:' in line:
                        import re as _re, os as _os
                        m = _re.search(r'(/[^\s]+\.html)', line)
                        if m:
                            _candidate = _os.path.realpath(m.group(1))
                            _allowed = _os.path.realpath(_os.path.expanduser('~/atlas_ws/data'))
                            if _candidate.startswith(_allowed + _os.sep):
                                self.root.after(0, self._pp_log_write, '  Launching browser...\n')
                                self.root.after(200, lambda p=_candidate: self._open_colmap_viewer_in_browser(p))
                proc.wait()
                self.root.after(0, self._pp_log_write,
                                f"{'Done' if proc.returncode == 0 else 'FAILED'} (exit {proc.returncode})\n")
            except FileNotFoundError as e:
                self.root.after(0, self._pp_log_write, f"Error: command not found: {e}\n")
            except PermissionError as e:
                self.root.after(0, self._pp_log_write, f"Error: permission denied: {e}\n")
        threading.Thread(target=_run, daemon=True).start()

    def _pp_run_reconstruct(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        if not any(p.is_dir() and not str(p).endswith('_imu')
                   for p in pathlib.Path(sess).glob('rosbag_*')):
            self._pp_log_write("\n[!] No rosbag found in session.\n")
            self._pp_log_write("    The bag may have been deleted after processing.\n")
            self._pp_log_write("    Other utilities (ICP, merge, COLMAP, viewer) can still run on this session.\n")
            return
        self._pp_run("Run Post-Processing", [sys.executable, str(self.script_dir / 'post_processing/reconstruct_from_bag.py'), sess])

    def _pp_reprocess(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        self._pp_run("Recolor with New Calibration", [sys.executable, str(self.script_dir / 'post_processing/reprocess_session.py'), sess])

    def _pp_icp(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        self._pp_run("ICP Alignment", [sys.executable, str(self.script_dir / 'post_processing/align_scan_session_posegraph.py'), sess, '--iterations', '1'])

    def _pp_filter_blurry(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        self._pp_run("Filter Blurry Frames", [sys.executable,
            str(self.script_dir / 'post_processing/filter_blurry_scans.py'), sess])

    def _pp_merge_traj(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        self._pp_run("Merge (Trajectory Only)", [sys.executable, str(self.script_dir / 'post_processing/merge_trajectory_only.py'), sess])

    def _pp_colmap(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        cmd = [sys.executable, str(self.script_dir / 'post_processing/panorama_sfm_colmap.py'), sess,
               '--no-bundle-adjustment']
        voxel = getattr(self, 'colmap_lidar_voxel_size', 0.0)
        if voxel > 0:
            cmd += ['--lidar-voxel-size', str(voxel)]
        else:
            self._pp_log_write(f"  (COLMAP_LIDAR_VOXEL_SIZE={voxel} — no LiDAR downsampling)\n")
        self._pp_run("Export COLMAP Model", cmd)

    def _pp_web_viewer(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        _allowed = pathlib.Path(sess).resolve()
        ply_path = next((p for p in sorted(pathlib.Path(sess).glob('*.ply'), key=lambda p: p.stat().st_mtime, reverse=True)
                         if _allowed in [p.resolve(), *p.resolve().parents]), None)
        if not ply_path: self._pp_log_write("\n[!] No .ply file found in session.\n"); return
        self._pp_run("View Point Cloud (Web)", [sys.executable, str(self.script_dir / 'post_processing/web_3d_viewer.py'), str(ply_path.resolve())])

    def _pp_toggle_viewer(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        self._pp_run("Per-Scan Alignment Viewer", [sys.executable, str(self.script_dir / 'post_processing/scan_toggle_viewer.py'), sess])

    def _pp_sync_benchmark(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        self._pp_run("Sync Benchmark", [sys.executable, str(self.script_dir / 'post_processing/sync_benchmark.py'),
                                        sess, '--out', str(pathlib.Path(sess) / 'sync_benchmark.json')])

    def _pp_sdk_sync_validator(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        # Check this is an SDK stitch continuous session
        if not (pathlib.Path(sess) / '.sdk_stitch_continuous').exists():
            self._pp_log_write(
                "\n[!] SDK Sync Validator requires a continuous SDK-stitch session\n"
                "    (.sdk_stitch_continuous sentinel not found).\n")
            return
        self._pp_run("SDK Sync Validator",
                     [sys.executable,
                      str(self.script_dir / 'post_processing/sdk_sync_validator.py'),
                      sess, '--lidar-window', '0.6', '--walk-speed', '0.5'])

    def _pp_colmap_quality(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        colmap_sparse = pathlib.Path(sess) / 'colmap' / 'sparse' / '0'
        if not colmap_sparse.exists():
            self._pp_log_write("\n[!] No COLMAP sparse model found \u2014 run Export COLMAP Model first.\n")
            return
        self._pp_run("COLMAP Pose Quality", [sys.executable,
                                             str(self.script_dir / 'post_processing/colmap_pose_quality.py'),
                                             sess])

    def _pp_colmap_depth(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        if not (pathlib.Path(sess) / 'colmap' / 'sparse' / '0').exists():
            self._pp_log_write("\n[!] No COLMAP sparse model found \u2014 run Export COLMAP Model first.\n")
            return
        self._pp_run("Generate Depth Images", [sys.executable,
                                               str(self.script_dir / 'post_processing/generate_colmap_depth.py'),
                                               sess])

    def _pp_depth_overlay(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        if not (pathlib.Path(sess) / 'colmap' / 'depth_images').exists():
            self._pp_log_write("\n[!] No depth images found \u2014 run Generate Depth Images first.\n")
            return
        self._pp_run("Depth-RGB Overlay", [sys.executable,
                                           str(self.script_dir / 'post_processing/colmap_depth_overlay.py'),
                                           sess])

    def _pp_colmap_viewer(self):
        sess = self._pp_session()
        if not sess: self._pp_log_write("\n[!] No session selected.\n"); return
        if not (pathlib.Path(sess) / 'colmap' / 'sparse' / '0').exists():
            self._pp_log_write("\n[!] No COLMAP sparse model found \u2014 run Export COLMAP Model first.\n")
            return
        self._pp_run("COLMAP Viewer", [sys.executable,
                                       str(self.script_dir / 'post_processing/colmap_viewer.py'),
                                       sess])

    def _open_colmap_viewer_in_browser(self, html_path):
        import subprocess as _sp
        url = f'file://{html_path}'
        env = os.environ.copy()
        env.setdefault('DISPLAY', ':0')
        wayland = os.environ.get('_ATLAS_WAYLAND_DISPLAY') or os.environ.get('WAYLAND_DISPLAY', '')
        if wayland:
            env['WAYLAND_DISPLAY'] = wayland
        for cmd in [
            ['firefox', '--new-window', url],
            ['chromium-browser', '--new-window', url],
            ['chromium', '--new-window', url],
            ['xdg-open', url],
        ]:
            try:
                _sp.Popen(cmd, stdout=_sp.DEVNULL, stderr=_sp.DEVNULL,
                          env=env, preexec_fn=os.setpgrp)
                self._pp_log_write(f'  Opened in browser: {html_path}\n')
                return
            except FileNotFoundError:
                continue
        self._pp_log_write(f'  Could not open browser. Open manually:\n  {html_path}\n')

    # ───────────────────────────────────────────────────────────────

    # ── Calibration tab helpers ────────────────────────────────────────────

    def _cal_log_write(self, text):
        self._cal_log.config(state='normal')
        self._cal_log.insert(tk.END, text)
        self._cal_log.see(tk.END)
        self._cal_log.config(state='disabled')

    def _cal_run(self, label, cmd, env_extra=None):
        """Run a calibration command in a background thread, streaming output to _cal_log."""
        import os as _os
        _atlas_ws = str(pathlib.Path.home() / 'atlas_ws')
        _safe_interp = _os.path.realpath(sys.executable)
        _actual_cmd0 = _os.path.realpath(cmd[0]) if cmd else ''
        _is_python = _actual_cmd0 == _safe_interp
        _is_dvl_bin = _actual_cmd0.startswith(_atlas_ws) and _os.path.isfile(_actual_cmd0)
        if not _is_python and not _is_dvl_bin:
            self._cal_log_write(f"Error: rejected unsafe command: {cmd[0]!r}\n")
            return
        safe_cmd = ' '.join(str(a) for a in cmd)
        self._cal_log_write(f"\n>> {label}\n   {safe_cmd}\n")
        def _run():
            import subprocess as _sp
            _safe_env = {
                k: os.environ[k] for k in (
                    'PATH', 'HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL',
                    'ROS_DISTRO', 'ROS_VERSION', 'AMENT_PREFIX_PATH',
                    'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                    'LD_LIBRARY_PATH', 'PYTHONPATH',
                    'DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR',
                    'DBUS_SESSION_BUS_ADDRESS',
                ) if k in os.environ
            }
            _safe_env['PYTHONUNBUFFERED'] = '1'
            if env_extra:
                _safe_env.update(env_extra)
            try:
                proc = _sp.Popen(cmd, stdout=_sp.PIPE, stderr=_sp.STDOUT, text=True,
                                 bufsize=1, env=_safe_env)
                for line in proc.stdout:
                    self.root.after(0, self._cal_log_write, line.replace('\r', ''))
                proc.wait()
                self.root.after(0, self._cal_log_write,
                                f"{'Done' if proc.returncode == 0 else 'FAILED'} (exit {proc.returncode})\n")
            except FileNotFoundError as e:
                self.root.after(0, self._cal_log_write, f"Error: {e}\n")
        threading.Thread(target=_run, daemon=True).start()

    def _cal_src(self):
        """Return atlas-scanner/src directory."""
        return str(self.script_dir)

    def _cal_hw(self):
        """Return camera hardware for the selected calibration camera.
        Reads from multi_camera.yaml if available, falls back to global setting."""
        cam = self._cal_cam_var.get()  # e.g. "cam_0"
        try:
            import yaml
            mc_path = self.script_dir / 'config' / 'multi_camera.yaml'
            if mc_path.exists():
                with open(mc_path) as f:
                    cfg = yaml.safe_load(f)
                hw = cfg.get('cameras', {}).get(cam, {}).get('camera_hw', '')
                if hw:
                    return hw
        except Exception:
            pass
        return self.camera_hw_var.get()

    def _cal_combine_scans(self):
        sess = self._pp_session()
        if not sess: self._cal_log_write("\n[!] No session selected.\n"); return
        cmd = [
            sys.executable,
            str(self.script_dir / 'calibration' / 'combine_scans_for_calibration.py'),
            sess,
            '--cam-index', self._cal_cam_var.get().split('_')[1]
        ]
        self._cal_run("Combine Scans for Calibration", cmd)

    def _cal_gen_intensity(self):
        import os as _os
        output = str(pathlib.Path.home() / 'atlas_ws' / 'output')
        mask_file = self._cal_get_mask()
        cam_idx = self._cal_cam_var.get().split('_')[1]  # '0', '1', or '2'
        env_extra = {'ATLAS_CALIBRATION_CAM_INDEX': cam_idx}
        if mask_file and _os.path.isfile(mask_file):
            env_extra['ATLAS_CALIBRATION_MASK'] = mask_file
            self._cal_log_write(f"  Using calibration mask: {_os.path.basename(mask_file)}\n")
        self._cal_run("Generate Intensity Images", [
            sys.executable,
            str(self.script_dir / 'calibration' / 'generate_intensity_images.py'),
            output
        ], env_extra=env_extra)

    def _cal_get_mask(self):
        """Return the calibration mask path for the currently selected camera."""
        cam = self._cal_cam_var.get()
        try:
            import yaml
            mc_path = self.script_dir / 'config' / 'multi_camera.yaml'
            if mc_path.exists():
                cfg = yaml.safe_load(mc_path.read_text())
                c = cfg.get('cameras', {}).get(cam, {})
                # Prefer mask_calibration, fall back to mask_dual
                mask_name = c.get('mask_calibration', c.get('mask_dual', ''))
                if mask_name:
                    mask_path = str(self.script_dir / 'config' / 'masks' / mask_name)
                    return mask_path
        except Exception:
            pass
        return None

    def _cal_superglue_indoor(self):
        self._cal_superglue('indoor')

    def _cal_superglue_outdoor(self):
        self._cal_superglue('outdoor')

    def _cal_initial_guess(self):
        """Dispatch to auto or manual based on the dropdown."""
        if self._cal_guess_var.get() == 'manual':
            self._cal_initial_guess_manual()
        else:
            self._cal_initial_guess_auto()

    def _cal_run_full_pipeline(self):
        """Run all calibration steps in sequence using the current dropdown settings."""
        scene = self._cal_scene_var.get()
        guess = self._cal_guess_var.get()
        dvl = pathlib.Path.home() / 'atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration'
        output = str(pathlib.Path.home() / 'atlas_ws/output')
        erp_matcher = self.script_dir / 'calibration' / 'find_matches_superglue_erp.py'
        mask_file = self._cal_get_mask()
        import threading as _th
        import subprocess as _sp

        def _run():
            import os as _os
            _safe_env = {
                k: os.environ[k] for k in (
                    'PATH', 'HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL',
                    'PYTHONPATH', 'LD_LIBRARY_PATH', 'DISPLAY',
                    'ROS_DISTRO', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                ) if k in os.environ
            }
            _safe_env['PYTHONUNBUFFERED'] = '1'
            # Pass per-camera calibration mask and camera index
            cam_idx = self._cal_cam_var.get().split('_')[1]
            _safe_env['ATLAS_CALIBRATION_CAM_INDEX'] = cam_idx
            if mask_file and _os.path.isfile(mask_file):
                _safe_env['ATLAS_CALIBRATION_MASK'] = mask_file
                self.root.after(0, self._cal_log_write,
                                f'  Calibration mask: {_os.path.basename(mask_file)}\n')
            else:
                self.root.after(0, self._cal_log_write,
                                f'  ⚠ No calibration mask applied (mask_file={mask_file!r})\n')
            _superglue_dir = str(pathlib.Path.home() / 'atlas_ws/SuperGluePretrainedNetwork')
            _existing_pp = _safe_env.get('PYTHONPATH', '')
            _safe_env['PYTHONPATH'] = str(dvl) + ':' + _superglue_dir + (':' + _existing_pp if _existing_pp else '')

            _cam_args = ['--cam-index', self._cal_cam_var.get().split('_')[1]]

            steps = [
                ("1. Combine Scans", [sys.executable,
                    str(self.script_dir / 'calibration' / 'combine_scans_for_calibration.py'),
                    self._pp_session() or output] + _cam_args),
                ("2. Generate Intensity Images", [sys.executable,
                    str(self.script_dir / 'calibration' / 'generate_intensity_images.py'), output]),
                (f"3. Match Features (SuperGlue {scene})", [sys.executable, str(erp_matcher),
                    output, '--superglue', scene, '--max_keypoints', '2048', '--match_threshold', '0.2']),
                ("4. Seed from Current Calibration", [sys.executable,
                    str(self.script_dir / 'calibration' / 'seed_calib.py')]),
            ]
            # Step 4b: initial guess
            if guess == 'auto':
                steps.append(("5. Initial Guess (Auto)", [
                    str(dvl / 'initial_guess_auto'), '--data_path', output]))
            else:
                steps.append(("5. Initial Guess (Manual — open interactive window)", [
                    str(dvl / 'initial_guess_manual'), '--data_path', output]))
            steps += [
                ("6. Run Calibration", [str(dvl / 'calibrate'), '--data_path', output,
                    '--nid_bins', '32', '--nelder_mead_convergence_criteria', '1e-10']),
                ("7. Apply Calibration", [sys.executable,
                    str(self.script_dir / 'calibration' / 'coordinate_transform.py'),
                    self._cal_src(), '--camera-hw', self._cal_hw()]),
            ]

            for label, cmd in steps:
                safe_cmd = ' '.join(str(a) for a in cmd)
                self.root.after(0, self._cal_log_write, f'\n>> {label}\n   {safe_cmd}\n')
                # Skip session arg if no session selected
                if not self._pp_session() and 'combine_scans' in safe_cmd:
                    self.root.after(0, self._cal_log_write, '  Skipped (no session selected)\n')
                    continue
                proc = _sp.Popen(cmd, stdout=_sp.PIPE, stderr=_sp.STDOUT,
                                 text=True, bufsize=1, env=_safe_env, cwd=str(dvl))
                for line in proc.stdout:
                    self.root.after(0, self._cal_log_write, line.replace('\r', ''))
                proc.wait()
                status = 'Done' if proc.returncode == 0 else f'FAILED (exit {proc.returncode})'
                self.root.after(0, self._cal_log_write, f'{status}\n')
                if proc.returncode not in (0, None) and 'Manual' not in label:
                    self.root.after(0, self._cal_log_write,
                                    f'Pipeline stopped at: {label}\n')
                    return
            self.root.after(0, self._cal_log_write,
                            '\n✓ Full calibration pipeline complete. Run Verify Calibration to inspect.\n')

        _th.Thread(target=_run, daemon=True).start()

    def _cal_superglue(self, weights):
        dvl = pathlib.Path.home() / 'atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration'
        output = str(pathlib.Path.home() / 'atlas_ws/output')
        import threading as _th
        def _run_sequence():
            import subprocess as _sp
            _safe_env = {
                k: os.environ[k] for k in (
                    'PATH', 'HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL',
                    'PYTHONPATH', 'LD_LIBRARY_PATH', 'DISPLAY',
                    'ROS_DISTRO', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                ) if k in os.environ
            }
            _safe_env['PYTHONUNBUFFERED'] = '1'
            _superglue_dir = str(pathlib.Path.home() / 'atlas_ws/SuperGluePretrainedNetwork')
            _existing_pp = _safe_env.get('PYTHONPATH', '')
            _safe_env['PYTHONPATH'] = str(dvl) + ':' + _superglue_dir + (':' + _existing_pp if _existing_pp else '')
            # Use ERP-aware matcher (perspective crops from ERP, better for wide-angle images)
            erp_matcher = self.script_dir / 'calibration' / 'find_matches_superglue_erp.py'
            match_cmd = [sys.executable, str(erp_matcher),
                         output, '--superglue', weights,
                         '--max_keypoints', '2048',
                         '--match_threshold', '0.2']
            self.root.after(0, self._cal_log_write,
                            f'>> Match Features ERP (SuperGlue {weights})\n   {" ".join(match_cmd)}\n')
            proc = _sp.Popen(match_cmd, stdout=_sp.PIPE, stderr=_sp.STDOUT,
                             text=True, bufsize=1, env=_safe_env,
                             cwd=str(dvl))
            for line in proc.stdout:
                self.root.after(0, self._cal_log_write, line.replace('\r', ''))
            proc.wait()
            if proc.returncode != 0:
                self.root.after(0, self._cal_log_write,
                                f'FAILED (exit {proc.returncode})\n')
                return
            # fix_matches.py not needed for ERP matcher (already in ERP coords)
            self.root.after(0, self._cal_log_write, 'Done\n')
        _th.Thread(target=_run_sequence, daemon=True).start()

    def _cal_initial_guess_manual(self):
        dvl = pathlib.Path.home() / 'atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration'
        output = str(pathlib.Path.home() / 'atlas_ws/output')
        self._cal_run("Initial Guess (Manual)", [
            str(dvl / 'initial_guess_manual'),
            '--data_path', output
        ])

    def _cal_initial_guess_auto(self):
        dvl = pathlib.Path.home() / 'atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration'
        output = str(pathlib.Path.home() / 'atlas_ws/output')
        self._cal_run("Initial Guess (Auto)", [
            str(dvl / 'initial_guess_auto'),
            '--data_path', output
        ])

    def _cal_run_calibration(self):
        dvl = pathlib.Path.home() / 'atlas_ws/install/direct_visual_lidar_calibration/lib/direct_visual_lidar_calibration'
        output = str(pathlib.Path.home() / 'atlas_ws/output')
        self._cal_run("Run Calibration", [
            str(dvl / 'calibrate'),
            '--data_path', output,
            '--nid_bins', '32',
            '--nelder_mead_convergence_criteria', '1e-10'
        ])

    def _cal_seed(self):
        self._cal_run("Seed from Current Calibration", [
            sys.executable,
            str(self.script_dir / 'calibration' / 'seed_calib.py')
        ])

    def _cal_write_physical_seed(self):
        """Compute and write calibration seed from physical measurements."""
        try:
            fwd_in = float(self._seed_fwd_var.get())
            left_in = float(self._seed_left_var.get())
            up_in = float(self._seed_up_var.get())
            yaw_deg = float(self._seed_yaw_var.get())
        except ValueError:
            self._cal_log_write("\n[!] Invalid numeric input in seed fields.\n")
            return
        hw = self._cal_hw()
        cam_idx = self._cal_cam_var.get().split('_')[1]
        self._cal_run("Write Physical Seed", [
            sys.executable,
            str(self.script_dir / 'calibration' / 'physical_seed.py'),
            '--camera-hw', hw,
            '--forward', str(fwd_in),
            '--left', str(left_in),
            '--up', str(up_in),
            '--yaw', str(yaw_deg),
        ], env_extra={'ATLAS_CALIBRATION_CAM_INDEX': cam_idx})

    def _cal_verify_seed(self):
        """Write current seed values, generate overlay and open it."""
        import subprocess, pathlib
        sess = self._pp_session()
        if not sess:
            self._cal_log_write("\n[!] No session selected.\n")
            return
        hw = self._cal_hw()
        # Write current GUI seed values first
        try:
            fwd_in  = float(self._seed_fwd_var.get())
            left_in = float(self._seed_left_var.get())
            up_in   = float(self._seed_up_var.get())
            yaw_deg = float(self._seed_yaw_var.get())
            subprocess.run([
                sys.executable,
                str(self.script_dir / 'calibration' / 'physical_seed.py'),
                '--camera-hw', hw,
                '--forward', str(fwd_in),
                '--left', str(left_in),
                '--up', str(up_in),
                '--yaw', str(yaw_deg),
            ], capture_output=True)
        except ValueError:
            pass  # use whatever is already saved
        result = subprocess.run(
            [sys.executable,
             str(self.script_dir / 'calibration' / 'verify_seed_overlay.py'),
             sess],
            capture_output=True, text=True
        )
        self._cal_log_write(result.stdout)
        if result.returncode != 0:
            self._cal_log_write(result.stderr)
            return
        edge_path = pathlib.Path(sess) / 'seed_edges.jpg'
        if edge_path.exists():
            subprocess.Popen(['xdg-open', str(edge_path)])

    def _cal_interactive_seed(self):
        """Write current seed values then launch interactive seed viewer window."""
        import subprocess
        sess = self._pp_session()
        if not sess:
            self._cal_log_write("\n[!] No session selected.\n")
            return
        hw = self._cal_hw()
        # Write the current GUI seed values first so the viewer starts with them
        try:
            fwd_in  = float(self._seed_fwd_var.get())
            left_in = float(self._seed_left_var.get())
            up_in   = float(self._seed_up_var.get())
            yaw_deg = float(self._seed_yaw_var.get())
        except ValueError:
            self._cal_log_write("\n[!] Invalid seed field values — fix before opening viewer.\n")
            return
        import subprocess as _sp, os as _os
        _sp.run([
            sys.executable,
            str(self.script_dir / 'calibration' / 'physical_seed.py'),
            '--camera-hw', hw,
            '--forward', str(fwd_in),
            '--left', str(left_in),
            '--up', str(up_in),
            '--yaw', str(yaw_deg),
        ], capture_output=True)
        subprocess.Popen([
            sys.executable,
            str(self.script_dir / 'calibration' / 'interactive_seed.py'),
            sess, '--camera-hw', hw,
        ])

    def _cal_apply(self):
        self._cal_run("Apply Calibration", [
            sys.executable,
            str(self.script_dir / 'calibration' / 'coordinate_transform.py'),
            self._cal_src(),
            '--camera-hw', self._cal_hw()
        ])

    def _cal_verify(self):
        sess = self._pp_session()
        scan_dir = None
        if sess:
            scans = sorted(pathlib.Path(sess).glob('fusion_scan_*'))
            if scans:
                scan_dir = str(scans[-1])
        cmd = [sys.executable,
               str(self.script_dir / 'calibration' / 'tune_calibration.py'),
               '--verify']
        if scan_dir:
            cmd.insert(2, scan_dir)
        # After verify completes, open the output image automatically
        def _after():
            import subprocess as _sp, os as _os
            candidates = []
            if scan_dir:
                candidates.append(pathlib.Path(scan_dir) / 'calib_sweep' / 'current.jpg')
            candidates.append(pathlib.Path.home() / 'atlas_ws/output/calib_sweep/current.jpg')
            for p in candidates:
                if p.exists():
                    env = _os.environ.copy()
                    env.setdefault('DISPLAY', ':0')
                    for viewer in ['eog', 'eom', 'xdg-open', 'feh']:
                        try:
                            _sp.Popen([viewer, str(p)], env=env,
                                      stdout=_sp.DEVNULL, stderr=_sp.DEVNULL)
                            self.root.after(0, self._cal_log_write,
                                            f'  Opened: {p}\n')
                            return
                        except FileNotFoundError:
                            continue
                    self.root.after(0, self._cal_log_write,
                                    f'  Saved: {p} (open manually)\n')
                    return
        # Run command then open image
        import threading as _th
        import subprocess as _sp
        def _run_then_open():
            import os as _os
            _safe_env = {
                k: os.environ[k] for k in (
                    'PATH', 'HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL',
                    'PYTHONPATH', 'LD_LIBRARY_PATH', 'DISPLAY',
                    'ROS_DISTRO', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                ) if k in os.environ
            }
            _safe_env['PYTHONUNBUFFERED'] = '1'
            label = 'Verify Calibration'
            safe_cmd = ' '.join(str(a) for a in cmd)
            self.root.after(0, self._cal_log_write, f'\n>> {label}\n   {safe_cmd}\n')
            proc = _sp.Popen(cmd, stdout=_sp.PIPE, stderr=_sp.STDOUT,
                             text=True, bufsize=1, env=_safe_env)
            for line in proc.stdout:
                self.root.after(0, self._cal_log_write, line.replace('\r', ''))
            proc.wait()
            self.root.after(0, self._cal_log_write,
                            f'{"Done" if proc.returncode == 0 else "FAILED"} (exit {proc.returncode})\n')
            if proc.returncode == 0:
                _after()
        _th.Thread(target=_run_then_open, daemon=True).start()

    def _cal_tune_pitch(self):
        self._cal_tune_axis('pitch')

    def _cal_tune_roll(self):
        self._cal_tune_axis('roll')

    def _cal_tune_yaw(self):
        self._cal_tune_axis('yaw')

    def _cal_tune_t(self):
        import tkinter.simpledialog as _sd
        axis = _sd.askstring(
            'Tune Translation', 'Axis to sweep (tx / ty / tz):',
            initialvalue='ty', parent=self.root)
        if axis and axis.strip() in ('tx', 'ty', 'tz'):
            self._cal_tune_axis(axis.strip())

    def _cal_tune_axis(self, axis):
        import tkinter.simpledialog as _sd
        rng = _sd.askfloat(
            f'Tune {axis}', f'Sweep range ± (degrees or metres):',
            initialvalue=2.0, parent=self.root)
        if rng is None:
            return
        sess = self._pp_session()
        scan_dir = None
        if sess:
            scans = sorted(pathlib.Path(sess).glob('fusion_scan_*'))
            if scans:
                scan_dir = str(scans[-1])
        cmd = [sys.executable,
               str(self.script_dir / 'calibration' / 'tune_calibration.py'),
               '--axis', axis, '--range', str(rng), '--steps', '5']
        if scan_dir:
            cmd.insert(2, scan_dir)
        self._cal_run(f'Tune {axis} sweep ±{rng}', cmd)

    def _cal_color_calibrate(self):
        sess = self._pp_session()
        if not sess: self._cal_log_write("\n[!] No session selected.\n"); return
        import tkinter.simpledialog as _sd
        ref = _sd.askinteger(
            'Reference Camera',
            'Camera index to use as color reference\n'
            '(all other cameras will be matched to this one):',
            initialvalue=0, minvalue=0, maxvalue=2, parent=self.root)
        if ref is None:
            return
        self._cal_run("Calibrate Color Profiles", [
            sys.executable,
            str(self.script_dir / 'post_processing' / 'color_normalize.py'),
            'calibrate', sess, '--reference-cam', str(ref)])

    def _cal_color_normalize(self):
        sess = self._pp_session()
        if not sess: self._cal_log_write("\n[!] No session selected.\n"); return
        self._cal_run("Apply Color Normalization", [
            sys.executable,
            str(self.script_dir / 'post_processing' / 'color_normalize.py'),
            'normalize', sess])

    # ───────────────────────────────────────────────────────────────────

    def _system_stopped(self):
        """Called when system is stopped"""
        if not self.is_running and self.fusion_process is None:
            return  # already stopped, ignore duplicate call
        self.is_running = False
        self.fusion_process = None
        self._coverage_panel.stop()

        if self.rviz_process:
            if self.rviz_win_id:
                self.rviz_frame.unbind('<Configure>')
                # Close the outer WM shell; rviz2 process will be killed by ROS shutdown
                outer = getattr(self, 'rviz_outer_win_id', None)
                if outer:
                    # Validate outer is a pure decimal window ID before use
                    if not str(outer).strip().isdigit():
                        self.log_message(f'⚠ Skipping windowclose: invalid window ID {outer!r}')
                    else:
                        subprocess.run(['xdotool', 'windowclose', str(outer).strip()], capture_output=True)
                    if not hasattr(self, '_stale_rviz_wids'):
                        self._stale_rviz_wids = set()
                    self._stale_rviz_wids.add(outer)
            self.rviz_process = None
            self.rviz_win_id = None
            self.rviz_outer_win_id = None

        # Do NOT terminate web_viewer_process here — let it stay embedded
        # so the user can inspect the point cloud after the session ends.
        self._browser_win_id = None

        if self._pending_viewer_html:
            html_path = self._pending_viewer_html
            self._pending_viewer_html = None
            self.root.after(500, lambda p=html_path: self.embed_web_viewer(p))

        self.update_status("System stopped", "red")
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.capture_button.config(state="disabled")

        self.log_message("System stopped.")
        
    def on_closing(self):
        """Handle window closing"""
        if self.is_running:
            if messagebox.askokcancel("Quit", "Fusion system is running. Stop and quit?"):
                self.stop_fusion()
                self.root.after(1000, self.root.destroy)
        else:
            if self.web_viewer_process:
                self.web_viewer_process.set()
            self.root.destroy()

def _setup_logging():
    log_path = pathlib.Path.home() / f"atlas_gui_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.FileHandler(log_path),
            logging.StreamHandler(sys.stdout),
        ]
    )
    logging.info(f"Log file: {log_path}")
    return log_path

def _add_session_log_handler(session_dir: str):
    import os as _os
    _allowed = _os.path.realpath(_os.path.expanduser('~/atlas_ws/data'))
    resolved = _os.path.realpath(session_dir)
    if not (resolved.startswith(_allowed + _os.sep) or resolved == _allowed):
        raise ValueError(f"session_dir '{resolved}' is outside allowed root '{_allowed}'")
    # Remove any previous session FileHandlers so old session dirs don't
    # accumulate across restarts within the same GUI process.
    root_logger = logging.getLogger()
    for h in root_logger.handlers[:]:
        if isinstance(h, logging.FileHandler) and 'synchronized_scans' in h.baseFilename:
            h.close()
            root_logger.removeHandler(h)
    dest = pathlib.Path(resolved) / 'gui.log'
    # Final check: confirm the log file itself stays within the allowed root
    if not str(dest.resolve()).startswith(_allowed + _os.sep):
        raise ValueError(f"Log file path '{dest}' is outside allowed root '{_allowed}'")
    handler = logging.FileHandler(dest)
    handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
    root_logger.addHandler(handler)
    logging.info(f"GUI log also writing to {dest}")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', choices=['dual_fisheye', 'single_fisheye'])
    parser.add_argument('--capture', choices=['continuous', 'stationary'])
    args = parser.parse_args()

    # Read defaults from atlas_fusion_capture.sh if no CLI args given
    script = pathlib.Path(__file__).parent / 'atlas_fusion_capture.sh'
    default_camera, default_capture, default_stationary_wait = 'dual_fisheye', 'continuous', False
    default_icp, default_colmap, default_colmap_lidar_voxel = False, False, 0.0
    default_camera_hw = 'onex2'
    try:
        for line in script.read_text().splitlines():
            line = line.strip()
            if line.startswith('CAMERA_MODE=') and '#' not in line.split('CAMERA_MODE=')[0]:
                default_camera = line.split('=', 1)[1].split('#')[0].strip('"\' ')
            elif line.startswith('CAPTURE_MODE=') and '#' not in line.split('CAPTURE_MODE=')[0]:
                default_capture = line.split('=', 1)[1].split('#')[0].strip('"\' ')
            elif line.startswith('STATIONARY_WAIT=') and '#' not in line.split('STATIONARY_WAIT=')[0]:
                default_stationary_wait = line.split('=', 1)[1].split('#')[0].strip('"\' ').lower() == 'true'
            elif line.startswith('ENABLE_ICP_ALIGNMENT=') and '#' not in line.split('ENABLE_ICP_ALIGNMENT=')[0]:
                default_icp = line.split('=', 1)[1].split('#')[0].strip('"\' ').lower() == 'true'
            elif line.startswith('EXPORT_COLMAP=') and '#' not in line.split('EXPORT_COLMAP=')[0]:
                default_colmap = line.split('=', 1)[1].split('#')[0].strip('"\' ').lower() == 'true'
            elif line.startswith('CAMERA_HW=') and '#' not in line.split('CAMERA_HW=')[0]:
                default_camera_hw = line.split('=', 1)[1].split('#')[0].strip('"\' ')
            elif line.startswith('COLMAP_LIDAR_VOXEL_SIZE=') and '#' not in line.split('COLMAP_LIDAR_VOXEL_SIZE=')[0]:
                try: default_colmap_lidar_voxel = float(line.split('=', 1)[1].split('#')[0].strip('"\' '))
                except ValueError: pass
            if line.startswith('while'):
                break  # stop before the CLI override block
    except OSError:
        pass  # use hardcoded defaults if script is missing or unreadable

    log_path = _setup_logging()
    logging.info("fusion_gui starting")
    logging.getLogger('PIL').setLevel(logging.WARNING)  # suppress PIL debug stream messages

    root = tk.Tk(className='ATLAS')

    # Configure style for better appearance
    style = ttk.Style()
    style.theme_use('clam')
    
    # Configure larger font for capture button
    style.configure('Large.TButton', font=('Arial', 12, 'bold'))
    
    app = FusionCaptureGUI(root)
    app.camera_mode_var.set(args.camera if args.camera else default_camera)
    app.capture_mode_var.set(args.capture if args.capture else default_capture)
    app.stationary_wait_var.set(default_stationary_wait)
    app.icp_var.set(default_icp)
    app.colmap_var.set(default_colmap)
    app.camera_hw_var.set(default_camera_hw)
    app.colmap_lidar_voxel_size = default_colmap_lidar_voxel
    app._on_capture_mode_changed()
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.stop_fusion()
    except Exception:
        logging.critical("Unhandled exception in mainloop:\n" + traceback.format_exc())
        raise

if __name__ == "__main__":
    main()