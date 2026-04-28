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
    os.execv(sys.executable, [sys.executable] + sys.argv)

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

class FusionCaptureGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ATLAS - Automated Terrestrial LiDAR Acquisition System")

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
        except Exception:
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
        ttk.Label(mode_frame, text="Capture:").grid(row=1, column=0, sticky=tk.W, padx=(0, 4), pady=(2, 0))
        self.capture_mode_var = tk.StringVar(value="continuous")
        ttk.Combobox(mode_frame, textvariable=self.capture_mode_var,
                     values=["continuous", "stationary"],
                     state="readonly", width=14).grid(row=1, column=1, sticky=(tk.W, tk.E), pady=(2, 0))
        self.stationary_wait_var = tk.BooleanVar(value=True)
        self.stationary_wait_cb = ttk.Checkbutton(mode_frame, text="Wait 3s before recording (stationary)",
                        variable=self.stationary_wait_var)
        self.stationary_wait_cb.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(2, 0))
        self.bag_only_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(mode_frame, text="Bag only (post-process later)",
                        variable=self.bag_only_var).grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=(2, 0))
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
        notebook.select(1)

        # Switch to log tab on new message if viewer is active, badge the tab
        self._log_tab_index = 1
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
            self.web_viewer_process.set()
            self.web_viewer_process = None
        for child in self.rviz_frame.winfo_children():
            child.destroy()
        
        self.log_message("Starting Automated Terrestrial LiDAR Acquisition System (ATLAS)...")
        self.update_status("Starting system...", "orange")
        
        # Disable start button, enable stop button
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        
        # Start fusion process in separate thread
        threading.Thread(target=self._run_fusion_process, daemon=True).start()
        
    def _run_fusion_process(self):
        """Run the fusion process"""
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
            perm_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'setup_camera_permissions.sh')
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
                            # Rule exists but device not yet accessible — just trigger udev,
                            # no password needed.
                            result = subprocess.run(
                                ['udevadm', 'trigger', '--subsystem-match=usb'],
                                text=True, timeout=10)
                            ok = result.returncode == 0
                        else:
                            # First-time setup: need pkexec to install the udev rule.
                            result = subprocess.run(['pkexec', 'bash', perm_script], text=True, timeout=120)
                            ok = result.returncode == 0
                    except Exception:
                        ok = False
                    msg = "✓ Camera permissions set" if ok else "⚠ Camera permissions failed — check polkit/pkexec"
                    self.root.after(0, lambda m=msg: self.log_message(m))
                    perm_done.set()

                threading.Thread(target=_run_perms, daemon=True).start()

            # Start the fusion script
            cmd = ['stdbuf', '-oL', './atlas_fusion_capture.sh',
                   '--camera', self.camera_mode_var.get(),
                   '--capture', self.capture_mode_var.get()]
            if self.bag_only_var.get():
                cmd.append('--bag-only')
            if self.capture_mode_var.get() == 'stationary' and not self.stationary_wait_var.get():
                cmd.append('--no-stationary-wait')
            self.fusion_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.is_running = True
            
            # Read output in real-time
            while self.fusion_process and self.is_running:
                try:
                    line = self.fusion_process.stdout.readline()
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
                            try:
                                _add_session_log_handler(self.scan_dir)
                                self._session_log_attached = True
                            except Exception:
                                pass
                    if "FUSION CAPTURE READY" in line:
                        self.root.after(0, self._system_ready)
                    elif "Press ENTER to capture" in line:
                        self.root.after(0, self._system_ready)
                    elif "Scan" in line and "completed" in line:
                        self.root.after(0, self._scan_completed)
                    elif "✓ Scan saved to:" in line:
                        self.root.after(0, self._scan_completed)
                    elif "Scan capture complete" in line:
                        self.root.after(0, self._scan_completed)
                    elif "_viewer.html" in line or ("3D viewer" in line and ".html" in line):
                        import re
                        m = re.search(r'(/[^\s]+_viewer\.html)', line)
                        if m:
                            self._pending_viewer_html = m.group(1)
                    elif "Session ended with no scans captured" in line:
                        self.root.after(0, lambda: self.update_status("No scans captured — stopped before triggering a scan", "orange"))
                    elif any(err in line for err in (
                        "✗ Camera failed", "✗ LiDAR", "Failed to start",
                        "failed to start", "exit 1", "not found at /dev/insta",
                        "Failed to initialize sensors"
                    )):
                        self.root.after(0, lambda l=line: self.update_status(f"Error: {l}", "red"))
                except Exception as e:
                    self.root.after(0, lambda: self.log_message(f"Error reading output: {e}"))
                    break

            # Process exited — drain any remaining output before resetting
            if self.fusion_process:
                try:
                    remaining = self.fusion_process.stdout.read()
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
        if self.web_viewer_process:
            self.web_viewer_process.set()  # stop previous viewer if any
            self.web_viewer_process = None

        ply_path = html_path.replace('_viewer.html', '.ply')
        if not os.path.exists(ply_path):
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
        self._notebook.select(0)

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
        self.update_status("System Ready - Ready to capture scans", "green")
        self.log_message("✓ System is ready for scanning!")
        if self.rviz_process is None:
            self.embed_rviz()

        # Only enable capture button in stationary mode
        if self._is_continuous_mode():
            self.capture_button.config(state="disabled")
            self.update_status("Continuous mode — press Stop when done", "green")
        else:
            self.capture_button.config(state="normal")

        # Extract output directory from logs
        log_content = self.log_text.get("1.0", tk.END)
        for line in log_content.split('\n'):
            if "Scans will be saved to:" in line:
                output_dir = line.split("Scans will be saved to:")[-1].strip()
                self.output_dir_label.config(text=output_dir, foreground="black")
                try:
                    _add_session_log_handler(output_dir)
                except Exception:
                    pass
                break
                
    def _check_disk_space(self, warn_gb=5.0):
        """Warn if free disk space on the data partition is below warn_gb."""
        import shutil
        try:
            free_gb = shutil.disk_usage(str(self.script_dir)).free / 1e9
            if free_gb < warn_gb:
                self.log_message(f"⚠ Low disk space: {free_gb:.1f} GB free — consider freeing space before next scan")
                self.update_status(f"⚠ Low disk space ({free_gb:.1f} GB free)", "orange")
                return False
        except Exception:
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
        """Find the most recent fisheye image and update the thumbnail."""
        try:
            scan_path = pathlib.Path(self.scan_dir)
            images = sorted(scan_path.glob('fusion_scan_*/fisheye_*.jpg'), key=lambda p: p.stat().st_mtime)
            if not images:
                images = sorted(scan_path.glob('fusion_scan_*/dual_fisheye_*.jpg'), key=lambda p: p.stat().st_mtime)
            if not images:
                return
            img = Image.open(images[-1])
            thumb_w = 200 if not self._small_screen else 160
            ratio = thumb_w / img.width
            thumb_h = int(img.height * ratio)
            # Do all PIL work in background thread, convert to RGB to avoid mode issues
            img = img.convert('RGB').resize((thumb_w, thumb_h), Image.LANCZOS)
            # ImageTk.PhotoImage must be created on the main thread — pass the PIL image
            def _update(img=img):
                try:
                    photo = ImageTk.PhotoImage(img)
                    self._latest_thumb_photo = photo
                    self.thumb_label.config(image=photo, text='')
                except Exception:
                    pass
            self.root.after(0, _update)
        except Exception:
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
                self.fusion_process.stdin.write('\n')
                self.fusion_process.stdin.flush()
        except Exception as e:
            self.log_message(f"Error triggering scan: {e}")
            self.capture_button.config(state="normal")

    def stop_fusion(self):
        """Stop the fusion system"""
        if not self.is_running:
            return
            
        self.log_message("Stopping fusion system...")
        self.update_status("Processing and shutting down...", "orange")
        
        # Write quit trigger file for GUI mode; also write to stdin as fallback
        proc = self.fusion_process
        if proc:
            try:
                if hasattr(self, 'scan_dir') and self.scan_dir:
                    pathlib.Path(self.scan_dir, '.quit_trigger').touch()
                proc.stdin.write('q\n')
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
                except Exception:
                    pass
                if time.monotonic() - last_output > SILENCE_LIMIT:
                    self.root.after(0, lambda: self.log_message(
                        f"Post-processing timeout ({SILENCE_LIMIT}s silence), force stopping..."))
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
                    break
                time.sleep(2)
            self.root.after(500, self._system_stopped)

        threading.Thread(target=_wait_for_finish, daemon=True).start()
        
    def _cleanup_ros_processes(self):
        """Clean up any remaining ROS processes"""
        try:
            # Kill specific ROS processes
            processes_to_kill = [
                "insta360_ros_driver", "equirectangular", "dual_fisheye", "bringup.launch",
                "livox_ros_driver2", "rko_lio", "static_transform_publisher",
                "trajectory_recorder", "insta360_ros_driver/lib", "livox_ros_driver2/lib"
            ]
            
            for process in processes_to_kill:
                subprocess.run(["pkill", "-9", "-f", process], capture_output=True)
                
            self.log_message("✓ ROS processes cleaned up")
        except Exception as e:
            self.log_message(f"Warning: Could not clean up all processes: {e}")
    
    def _system_stopped(self):
        """Called when system is stopped"""
        if not self.is_running and self.fusion_process is None:
            return  # already stopped, ignore duplicate call
        self.is_running = False
        self.fusion_process = None

        if self.rviz_process:
            if self.rviz_win_id:
                self.rviz_frame.unbind('<Configure>')
                # Close the outer WM shell; rviz2 process will be killed by ROS shutdown
                outer = getattr(self, 'rviz_outer_win_id', None)
                if outer:
                    subprocess.run(['xdotool', 'windowclose', outer], capture_output=True)
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
    dest = pathlib.Path(session_dir) / 'gui.log'
    handler = logging.FileHandler(dest)
    handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
    logging.getLogger().addHandler(handler)
    logging.info(f"GUI log also writing to {dest}")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', choices=['dual_fisheye', 'single_fisheye'])
    parser.add_argument('--capture', choices=['continuous', 'stationary'])
    args = parser.parse_args()

    # Read defaults from atlas_fusion_capture.sh if no CLI args given
    script = pathlib.Path(__file__).parent / 'atlas_fusion_capture.sh'
    default_camera, default_capture = 'dual_fisheye', 'continuous'
    try:
        for line in script.read_text().splitlines():
            line = line.strip()
            if line.startswith('CAMERA_MODE=') and '#' not in line.split('CAMERA_MODE=')[0]:
                default_camera = line.split('=', 1)[1].split('#')[0].strip('"\' ')
            elif line.startswith('CAPTURE_MODE=') and '#' not in line.split('CAPTURE_MODE=')[0]:
                default_capture = line.split('=', 1)[1].split('#')[0].strip('"\' ')
            if line.startswith('while'):
                break  # stop before the CLI override block
    except Exception:
        pass

    log_path = _setup_logging()
    logging.info("fusion_gui starting")

    root = tk.Tk()

    # Configure style for better appearance
    style = ttk.Style()
    style.theme_use('clam')
    
    # Configure larger font for capture button
    style.configure('Large.TButton', font=('Arial', 12, 'bold'))
    
    app = FusionCaptureGUI(root)
    app.camera_mode_var.set(args.camera if args.camera else default_camera)
    app.capture_mode_var.set(args.capture if args.capture else default_capture)
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