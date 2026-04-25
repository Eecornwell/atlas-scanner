#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Embeds an Open3D offscreen point cloud renderer directly into a Tkinter canvas widget, providing an interactive 3D view inside the ATLAS GUI.
"""Render a PLY point cloud into a Tk canvas using Open3D offscreen rendering."""
import sys
import os
import threading
import tkinter as tk
import numpy as np


def embed_viewer(parent_frame, ply_path):
    """Embed the point cloud viewer into parent_frame. Returns a stop_event."""
    import open3d as o3d
    from PIL import Image, ImageTk

    canvas = tk.Canvas(parent_frame, bg='#1a1a1a', highlightthickness=0)
    canvas.pack(fill=tk.BOTH, expand=True)
    stop_event = threading.Event()

    pcd = o3d.io.read_point_cloud(ply_path)
    if len(pcd.points) == 0:
        tk.Label(canvas, text='Empty point cloud', fg='white', bg='#1a1a1a').pack()
        return stop_event

    bounds = pcd.get_axis_aligned_bounding_box()
    center = np.asarray(bounds.get_center(), dtype=np.float32)
    extent = float(np.linalg.norm(bounds.get_extent()))

    state = {'drag': None, 'last': (0, 0), 'theta': 0.3, 'phi': 1.2, 'zoom': 1.0,
             'pan': [0.0, 0.0], 'dirty': True, 'w': 0, 'h': 0}
    img_ref = [None]
    renderer_box = [None]  # deferred until canvas has real dimensions

    def _make_renderer(w, h):
        r = o3d.visualization.rendering.OffscreenRenderer(w, h)
        r.scene.set_background([0.1, 0.1, 0.1, 1.0])
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = 'defaultUnlit'
        mat.point_size = 2.0
        r.scene.add_geometry('pcd', pcd, mat)
        renderer_box[0] = r
        state['w'], state['h'] = w, h
        _update_camera()

    def _update_camera():
        if renderer_box[0] is None:
            return
        t, p, z = state['theta'], state['phi'], state['zoom']
        dist = extent * 1.5 * z
        eye = center + np.array([
            dist * np.sin(p) * np.cos(t),
            dist * np.sin(p) * np.sin(t),
            dist * np.cos(p),
        ], dtype=np.float32)
        renderer_box[0].scene.camera.look_at(center.tolist(), eye.tolist(), [0, 0, 1])
        state['dirty'] = True

    def _tick():
        if stop_event.is_set():
            return
        cw = canvas.winfo_width()
        ch = canvas.winfo_height()
        if cw > 10 and ch > 10:
            if renderer_box[0] is None:
                _make_renderer(cw, ch)
            elif (cw, ch) != (state['w'], state['h']):
                state['w'], state['h'] = cw, ch
                renderer_box[0].resize(cw, ch)
                state['dirty'] = True
            if state['dirty'] and renderer_box[0] is not None:
                buf = renderer_box[0].render_to_image()
                arr = np.asarray(buf)
                photo = ImageTk.PhotoImage(Image.fromarray(arr))
                img_ref[0] = photo
                canvas.delete('all')
                canvas.create_image(0, 0, anchor='nw', image=photo)
                state['dirty'] = False
        canvas.after(50, _tick)

    def _press(e):
        state['drag'] = 'left' if e.num == 1 else 'right'
        state['last'] = (e.x, e.y)
    def _release(e):
        state['drag'] = None
    def _motion(e):
        dx = e.x - state['last'][0]; dy = e.y - state['last'][1]
        state['last'] = (e.x, e.y)
        if state['drag'] == 'left':
            state['theta'] -= dx * 0.01
            state['phi'] = max(0.05, min(np.pi - 0.05, state['phi'] - dy * 0.01))
        elif state['drag'] == 'right':
            state['pan'][0] -= dx * extent * 0.002
            state['pan'][1] += dy * extent * 0.002
        _update_camera()
    def _scroll(e):
        state['zoom'] *= 0.9 if (e.delta > 0 or e.num == 4) else 1.1
        _update_camera()

    canvas.bind('<ButtonPress-1>', _press)
    canvas.bind('<ButtonPress-3>', _press)
    canvas.bind('<ButtonRelease-1>', _release)
    canvas.bind('<ButtonRelease-3>', _release)
    canvas.bind('<B1-Motion>', _motion)
    canvas.bind('<B3-Motion>', _motion)
    canvas.bind('<MouseWheel>', _scroll)
    canvas.bind('<Button-4>', _scroll)
    canvas.bind('<Button-5>', _scroll)

    canvas.after(100, _tick)
    return stop_event


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ply', required=True)
    args = parser.parse_args()

    root = tk.Tk()
    root.title('ATLAS 3D Viewer')
    root.geometry('1200x800')
    stop = embed_viewer(root, args.ply)
    root.protocol('WM_DELETE_WINDOW', lambda: (stop.set(), root.destroy()))
    root.mainloop()
