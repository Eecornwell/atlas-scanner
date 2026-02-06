#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from PIL import Image, ImageTk
import subprocess
import threading
import os
import signal
import time
from datetime import datetime
import pathlib

class FusionCaptureGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ATLAS - Automated Terrestrial LiDAR Acquisition System")
        w = 900 # width for the Tk root - increased for side-by-side layout
        h = 550 # height for the Tk root - increased to prevent cropping

        # set the dimensions of the screen 
        # and where it is placed
        self.root.geometry('%dx%d+%d+%d' % (w, h, 100, 100))
        self.root.minsize(800, 400)  # Set minimum window size
        
        # Get script directory for relative paths
        self.script_dir = pathlib.Path(__file__).parent.resolve()
        
        # Process management
        self.fusion_process = None
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
        
        # Compact title
        title_label = ttk.Label(left_panel, text="ATLAS Control", 
                               font=('Arial', 14, 'bold'))
        title_label.grid(row=1, column=0, pady=(0, 10))
        
        # Status frame (compact)
        status_frame = ttk.LabelFrame(left_panel, text="Status", padding="5")
        status_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        self.status_label = ttk.Label(status_frame, text="Ready to start", foreground="blue")
        self.status_label.grid(row=0, column=0, sticky=tk.W)
        
        # Control buttons frame (vertical layout for compactness)
        control_frame = ttk.LabelFrame(left_panel, text="Controls", padding="5")
        control_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        self.start_button = ttk.Button(control_frame, text="Start System", 
                                      command=self.start_fusion, style="Accent.TButton")
        self.start_button.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 2))
        
        # Larger capture button for easier pressing on small screens
        self.capture_button = ttk.Button(control_frame, text="CAPTURE SCAN", 
                                        command=self.capture_scan, state="disabled",
                                        style="Large.TButton")
        self.capture_button.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 2), ipady=10)
        self.capture_button.configure(width=15)  # Make it wider
        
        self.stop_button = ttk.Button(control_frame, text="Stop System", 
                                     command=self.stop_fusion, state="disabled")
        self.stop_button.grid(row=2, column=0, sticky=(tk.W, tk.E))
        
        # Scan info frame (compact)
        info_frame = ttk.LabelFrame(left_panel, text="Scan Info", padding="5")
        info_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        ttk.Label(info_frame, text="Scans:").grid(row=0, column=0, sticky=tk.W)
        self.scan_count_label = ttk.Label(info_frame, text="0", font=('Arial', 12, 'bold'))
        self.scan_count_label.grid(row=0, column=1, sticky=tk.W, padx=(5, 0))
        
        ttk.Label(info_frame, text="Output:").grid(row=1, column=0, sticky=tk.W)
        self.output_dir_label = ttk.Label(info_frame, text="Not started", foreground="gray", wraplength=200)
        self.output_dir_label.grid(row=1, column=1, sticky=tk.W, padx=(5, 0))
        
        # Right panel: Log frame (takes most of the space)
        log_frame = ttk.LabelFrame(right_panel, text="System Log", padding="5")
        log_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Reduced log height for smaller screens
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, width=60, font=('Consolas', 9))
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Add context menu for copy functionality
        self.context_menu = tk.Menu(self.root, tearoff=0)
        self.context_menu.add_command(label="Copy", command=self.copy_text)
        self.context_menu.add_command(label="Select All", command=self.select_all_text)
        self.log_text.bind("<Button-3>", self.show_context_menu)  # Right-click
        
        # Configure grid weights for responsive layout
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=0)  # Left panel fixed width
        main_frame.columnconfigure(1, weight=1)  # Right panel expands
        main_frame.rowconfigure(0, weight=1)
        
        left_panel.columnconfigure(0, weight=1)
        control_frame.columnconfigure(0, weight=1)
        
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
    def log_message(self, message):
        """Add message to log with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.root.update_idletasks()
        
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
            self.fusion_process = subprocess.Popen(
                ['./terrestrial_fusion_with_lio.sh'],
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
                except Exception as e:
                    self.root.after(0, lambda: self.log_message(f"Error reading output: {e}"))
                    break
                        
        except Exception as ex:
            error_msg = f"Error starting fusion: {ex}"
            self.root.after(0, lambda: self.log_message(error_msg))
            self.fusion_process = None
            self.root.after(0, self._system_stopped)
            
    def _system_ready(self):
        """Called when system is ready"""
        self.update_status("System Ready - Ready to capture scans", "green")
        self.capture_button.config(state="normal")
        self.log_message("✓ System is ready for scanning!")
        
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
        
        # Send quit command and let the script handle post-processing
        if self.fusion_process:
            try:
                if self.fusion_process.stdin:
                    self.fusion_process.stdin.write('q\n')
                    self.fusion_process.stdin.flush()
                
                # Wait for the script to complete post-processing (up to 60 seconds)
                self.log_message("Waiting for post-processing to complete...")
                try:
                    self.fusion_process.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    self.log_message("Post-processing timeout, force stopping...")
                    os.killpg(os.getpgid(self.fusion_process.pid), signal.SIGKILL)
                        
            except Exception as e:
                self.log_message(f"Error stopping process: {e}")
        
        # Additional cleanup of ROS processes
        self._cleanup_ros_processes()
        
        self._system_stopped()
        
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
    root = tk.Tk()
    
    # Configure style for better appearance
    style = ttk.Style()
    style.theme_use('clam')
    
    # Configure larger font for capture button
    style.configure('Large.TButton', font=('Arial', 12, 'bold'))
    
    app = FusionCaptureGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.stop_fusion()

if __name__ == "__main__":
    main()