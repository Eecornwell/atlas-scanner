#!/usr/bin/env python3

import os
import sys

# X11 window embedding requires X11 — force XWayland if running under Wayland
if os.environ.get('WAYLAND_DISPLAY'):
    os.environ['GDK_BACKEND'] = 'x11'
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.environ.pop('WAYLAND_DISPLAY', None)
    os.execv(sys.executable, [sys.executable] + sys.argv)

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from PIL import Image, ImageTk
import subprocess
import threading
import signal
import time
from datetime import datetime
import pathlib

class FusionCaptureGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ATLAS - Automated Terrestrial LiDAR Acquisition System")
        self.root.geometry('1400x900+50+50')
        self.root.minsize(1000, 600)
        
        # Get script directory for relative paths
        self.script_dir = pathlib.Path(__file__).parent.resolve()
        
        # Process management
        self.fusion_process = None
        self.rviz_process = None
        self.rviz_win_id = None
        self.is_running = False
        self.scan_count = 0
        
        # Setup GUI
        self.setup_gui()
        
        # Change to script directory for execution
        os.chdir(self.script_dir)
    
    def setup_gui(self):
        # Main frame with reduced padding
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create left and right panels for better space utilization
        left_panel = ttk.Frame(main_frame)
        left_panel.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        right_panel = ttk.Frame(main_frame)
        right_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        
        # Left panel: Logo and title (smaller)
        try:
            # Use script directory for logo path
            logo_path = self.script_dir / '..' / 'assets' / 'media' / 'atlas_logo_app.png'
            logo_image = Image.open(logo_path)
            aspect_ratio = logo_image.width / logo_image.height
            new_width = int(100 * aspect_ratio)  # Reduced from 150 to 100
            logo_image = logo_image.resize((new_width, 100), Image.LANCZOS)
            self.logo_photo = ImageTk.PhotoImage(logo_image)
            logo_label = ttk.Label(left_panel, image=self.logo_photo)
            logo_label.grid(row=0, column=0, pady=(0, 5))
        except Exception as e:
            pass  # Skip logo if not found
        
        # Status frame (compact)
        status_frame = ttk.LabelFrame(left_panel, text="Status", padding="5")
        status_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 12))
        
        self.status_label = ttk.Label(status_frame, text="Ready to start", foreground="blue")
        self.status_label.grid(row=0, column=0, sticky=tk.W)
        
        # Control buttons frame (vertical layout for compactness)
        control_frame = ttk.LabelFrame(left_panel, text="Controls", padding="8")
        control_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 12))

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
        self.bag_only_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(mode_frame, text="Bag only (post-process later)",
                        variable=self.bag_only_var).grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(2, 0))

        self.start_button = ttk.Button(control_frame, text="Start System",
                                      command=self.start_fusion, style="Accent.TButton")
        self.start_button.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 2))
        
        # Larger capture button for easier pressing on small screens
        self.capture_button = ttk.Button(control_frame, text="CAPTURE SCAN", 
                                        command=self.capture_scan, state="disabled",
                                        style="Large.TButton")
        self.capture_button.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 2), ipady=30)
        self.capture_button.configure(width=15)  # Make it wider
        
        self.stop_button = ttk.Button(control_frame, text="Stop System", 
                                     command=self.stop_fusion, state="disabled")
        self.stop_button.grid(row=3, column=0, sticky=(tk.W, tk.E))
        
        # Scan info frame (compact)
        info_frame = ttk.LabelFrame(left_panel, text="Scan Info", padding="5")
        info_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 12))
        
        ttk.Label(info_frame, text="Scans:").grid(row=0, column=0, sticky=tk.W)
        self.scan_count_label = ttk.Label(info_frame, text="0", font=('Arial', 12, 'bold'))
        self.scan_count_label.grid(row=0, column=1, sticky=tk.W, padx=(5, 0))
        
        ttk.Label(info_frame, text="Output:").grid(row=1, column=0, sticky=tk.W)
        self.output_dir_label = ttk.Label(info_frame, text="Not started", foreground="gray", wraplength=200)
        self.output_dir_label.grid(row=1, column=1, sticky=tk.W, padx=(5, 0))
        
        # Right panel: tabbed view — RViz2 | System Log
        notebook = ttk.Notebook(right_panel)
        notebook.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.rviz_frame = tk.Frame(notebook, bg='black')
        self.rviz_frame.pack_propagate(False)
        self.rviz_frame.update_idletasks()
        notebook.add(self.rviz_frame, text="Viewer")

        log_tab = ttk.Frame(notebook, padding="5")
        self.log_text = scrolledtext.ScrolledText(log_tab, font=('Consolas', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        notebook.add(log_tab, text="System Log")

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
        self.root.update_idletasks()
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
            
        # Reset scan count for new session
        self.scan_count = 0
        self.scan_count_label.config(text="0")
        
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
            # Prepare environment
            env = os.environ.copy()
            env['SKIP_SUDO_CHECK'] = '1'  # Skip sudo checks - permissions already set up
            
            # Skip sudo authentication - assume permissions are set up
            self.log_message("Using existing camera permissions...")
            
            # Start the fusion script
            cmd = ['stdbuf', '-oL', './atlas_fusion_capture.sh',
                   '--camera', self.camera_mode_var.get(),
                   '--capture', self.capture_mode_var.get()]
            if self.bag_only_var.get():
                cmd.append('--bag-only')
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
            while self.fusion_process and self.fusion_process.poll() is None and self.is_running:
                try:
                    line = self.fusion_process.stdout.readline()
                    if line:
                        line = line.strip()
                        self.log_message(line)
                        
                        # Check for system ready status
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
                        elif any(err in line for err in (
                            "✗ Camera failed", "✗ LiDAR", "Failed to start",
                            "failed to start", "exit 1", "not found at /dev/insta"
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
                            if line:
                                self.root.after(0, lambda l=line: self.log_message(l))
                except Exception:
                    pass

            # Surface any error and reset buttons
            if self.is_running:
                exit_code = self.fusion_process.poll() if self.fusion_process else -1
                if exit_code not in (None, 0):
                    self.root.after(0, lambda c=exit_code: self.update_status(
                        f"Script exited with error (code {c}) — check log", "red"))
                self.root.after(0, self._system_stopped)
                        
        except Exception as ex:
            error_msg = f"Error starting fusion: {ex}"
            self.root.after(0, lambda: self.log_message(error_msg))
            self.fusion_process = None
            self.root.after(0, self._system_stopped)
            
    def _is_continuous_mode(self):
        return self.capture_mode_var.get() == 'continuous'

    def embed_rviz(self):
        """Watch for the RViz2 window launched by the fusion script and embed it"""
        self.rviz_process = True  # sentinel so _system_ready doesn't call us twice
        threading.Thread(target=self._reparent_rviz, daemon=True).start()

    def _reparent_rviz(self):
        """Poll for the RViz2 main window then reparent it into rviz_frame"""
        frame_id = self.rviz_frame.winfo_id()
        rviz_win_id = None

        for _ in range(40):
            time.sleep(0.5)
            result = subprocess.run(['xdotool', 'search', '--name', 'RViz'],
                                    capture_output=True, text=True)
            for wid in result.stdout.strip().split():
                name = subprocess.run(['xdotool', 'getwindowname', wid],
                                      capture_output=True, text=True).stdout.strip()
                if name.endswith('- RViz'):
                    rviz_win_id = wid
                    break
            if rviz_win_id:
                break

        if not rviz_win_id:
            self.root.after(0, lambda: self.log_message("⚠ Could not find RViz2 window to embed"))
            return

        w = str(self.rviz_frame.winfo_width())
        h = str(self.rviz_frame.winfo_height())

        # Unmap first so the WM doesn't fight us
        subprocess.run(['xdotool', 'windowunmap', '--sync', rviz_win_id])

        # Set override-redirect so the window manager stops managing this window
        # entirely — this is what makes it move with the parent
        subprocess.run(['xdotool', 'set_window', '--overrideredirect', '1', rviz_win_id])

        # Strip decorations
        subprocess.run(['xprop', '-id', rviz_win_id, '-f', '_MOTIF_WM_HINTS', '32c',
                        '-set', '_MOTIF_WM_HINTS', '2, 0, 0, 0, 0'], capture_output=True)

        # Reparent into our frame, anchor to top-left, size to fill
        subprocess.run(['xdotool', 'windowreparent', rviz_win_id, str(frame_id)])
        subprocess.run(['xdotool', 'windowmove', '--sync', rviz_win_id, '0', '0'])
        subprocess.run(['xdotool', 'windowsize', '--sync', rviz_win_id, w, h])
        subprocess.run(['xdotool', 'windowmap', rviz_win_id])

        # Lower the frame so RViz2 renders inside it, not behind Tkinter widgets
        self.root.after(0, self.rviz_frame.lower)
        self.rviz_frame.bind('<Configure>', self._on_rviz_frame_resize)
        self.root.after(0, lambda: self.log_message("✓ RViz2 embedded"))

    def _on_rviz_frame_resize(self, event):
        if self.rviz_win_id:
            subprocess.run(['xdotool', 'windowsize', self.rviz_win_id,
                            str(event.width), str(event.height)], capture_output=True)

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
                break
                
    def _scan_completed(self):
        """Called when a scan is completed"""
        self.scan_count += 1
        self.scan_count_label.config(text=str(self.scan_count))
        self.log_message(f"✓ Scan {self.scan_count} completed successfully!")
        
        # Re-enable capture button after scan completion
        self.capture_button.config(state="normal")
        self.update_status("System Ready - Ready to capture scans", "green")
        
    def capture_scan(self):
        """Trigger a scan capture"""
        if not self.is_running or not self.fusion_process:
            self.log_message("Cannot capture scan: system not ready")
            return
            
        self.log_message(f"Triggering scan {self.scan_count + 1}...")
        self.update_status(f"Capturing scan {self.scan_count + 1}...", "orange")
        
        # Disable capture button during scan
        self.capture_button.config(state="disabled")
        
        # Send enter key to fusion process
        try:
            if self.fusion_process and self.fusion_process.stdin:
                self.fusion_process.stdin.write('\n')
                self.fusion_process.stdin.flush()
            else:
                raise Exception("Fusion process not available")
        except Exception as e:
            self.log_message(f"Error triggering scan: {e}")
            # Re-enable button if error occurs
            self.capture_button.config(state="normal")

    def stop_fusion(self):
        """Stop the fusion system"""
        if not self.is_running:
            return
            
        self.log_message("Stopping fusion system...")
        self.update_status("Processing and shutting down...", "orange")
        
        # Send SIGINT to the script process only (not the whole group) so the
        # trap cleanup/post-processing runs to completion before children are killed.
        proc = self.fusion_process
        if proc:
            try:
                proc.send_signal(signal.SIGINT)
            except Exception as e:
                self.log_message(f"Error sending stop signal: {e}")

        def _wait_for_finish():
            try:
                proc.wait(timeout=300)
                self.root.after(0, lambda: self.log_message("✓ Post-processing complete"))
            except subprocess.TimeoutExpired:
                self.root.after(0, lambda: self.log_message("Post-processing timeout, force stopping..."))
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.root.after(0, self._system_stopped)

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
        self.is_running = False
        self.fusion_process = None

        if self.rviz_process:
            self.rviz_process = None
            self.rviz_win_id = None

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
                time.sleep(1)
                self.root.destroy()
        else:
            self.root.destroy()

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

    root = tk.Tk()
    
    # Configure style for better appearance
    style = ttk.Style()
    style.theme_use('clam')
    
    # Configure larger font for capture button
    style.configure('Large.TButton', font=('Arial', 12, 'bold'))
    
    app = FusionCaptureGUI(root)
    app.camera_mode_var.set(args.camera if args.camera else default_camera)
    app.capture_mode_var.set(args.capture if args.capture else default_capture)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.stop_fusion()

if __name__ == "__main__":
    main()