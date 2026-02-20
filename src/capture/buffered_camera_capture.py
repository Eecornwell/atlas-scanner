#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import struct
import cv2
import sys
import time
import os
from datetime import datetime
from collections import deque
import threading

class BufferedCameraCapture(Node):
    def __init__(self, output_dir=".", buffer_size=10, scan_duration=2.0):
        super().__init__('buffered_camera_capture')
        self.bridge = CvBridge()
        self.captured = False
        self.output_dir = output_dir
        self.buffer_size = buffer_size
        self.scan_duration = scan_duration
        
        # Buffered storage with timestamps - larger odom buffer for persistence
        self.lidar_buffer = deque(maxlen=buffer_size)
        self.odom_buffer = deque(maxlen=buffer_size * 3)  # Larger odom buffer
        self.buffer_lock = threading.Lock()
        self.last_odom = None  # Keep last known odometry
        self.odom_received_count = 0
        self.buffers_ready = False  # Flag to prevent premature capture
        
        # Slower camera subscription rate to reduce timing pressure
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_cb, 20)
        self.image_sub = self.create_subscription(Image, '/equirectangular/image', self.image_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, '/rko_lio/odometry', self.odom_cb, 10)
        
        print(f"Debug: Buffered capture initialized with buffer size {buffer_size}")
        print(f"Debug: Will capture LiDAR data for {scan_duration} seconds for increased density")

    def lidar_cb(self, msg):
        """Buffer LiDAR messages with timestamps"""
        timestamp = time.time()
        with self.buffer_lock:
            self.lidar_buffer.append((timestamp, msg))
        
        if not hasattr(self, '_lidar_count'):
            self._lidar_count = 0
        self._lidar_count += 1
        if self._lidar_count % 50 == 0:
            print(f"Debug: LiDAR buffer size: {len(self.lidar_buffer)}")
        
    def odom_cb(self, msg):
        """Buffer odometry messages with timestamps"""
        timestamp = time.time()
        with self.buffer_lock:
            self.odom_buffer.append((timestamp, msg))
            self.last_odom = msg  # Always keep the latest odometry
            self.odom_received_count += 1
            
        if self.odom_received_count % 10 == 1:  # Log first and every 10th message
            pos = msg.pose.pose.position
            print(f"Debug: Received odometry #{self.odom_received_count}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

    def find_closest_message(self, buffer, target_time, max_age=0.5):
        """Find the message in buffer closest to target_time"""
        if not buffer:
            return None
            
        best_msg = None
        best_diff = float('inf')
        
        for timestamp, msg in buffer:
            diff = abs(timestamp - target_time)
            if diff < best_diff and diff < max_age:
                best_diff = diff
                best_msg = msg
                
        return best_msg

    def image_cb(self, msg):
        """Camera-driven capture with buffered synchronization"""
        if self.captured or not self.buffers_ready:
            return
            
        self.captured = True
        print(f"Debug: Starting {self.scan_duration}s LiDAR capture for increased density...")
        
        # Capture image immediately (at start of LiDAR accumulation)
        capture_time = time.time()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            # Process and save image immediately
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            img_file = os.path.join(self.output_dir, f'equirect_{timestamp}.jpg')
            cv2.imwrite(img_file, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
            print(f"Debug: Saved image at start of LiDAR capture: {img_file}")
        except Exception as e:
            print(f"Error saving image: {e}")
            return
        
        # Now capture points for the specified duration
        points = self.capture_dense_points()
        
        # Save the LiDAR data with same timestamp
        self.save_lidar_data(points, timestamp, capture_time)
    def capture_dense_points(self):
        """Collect already-buffered points without blocking the ROS executor."""
        scan_start = time.time()
        all_points = []
        processed_messages = set()

        while time.time() - scan_start < self.scan_duration:
            with self.buffer_lock:
                current = list(self.lidar_buffer)
            for timestamp, lidar_msg in current:
                msg_id = id(lidar_msg)
                if msg_id in processed_messages:
                    continue
                processed_messages.add(msg_id)
                intensity_offset = next(
                    (f.offset for f in lidar_msg.fields if f.name == 'intensity'), None)
                for i in range(0, len(lidar_msg.data), lidar_msg.point_step):
                    if i + 12 > len(lidar_msg.data):
                        break
                    x = struct.unpack('<f', lidar_msg.data[i:i+4])[0]
                    y = struct.unpack('<f', lidar_msg.data[i+4:i+8])[0]
                    z = struct.unpack('<f', lidar_msg.data[i+8:i+12])[0]
                    if abs(x) < 20 and abs(y) < 20 and abs(z) < 10:
                        intensity = 0.5
                        if intensity_offset is not None and i + intensity_offset + 4 <= len(lidar_msg.data):
                            try:
                                raw = struct.unpack('<f', lidar_msg.data[i+intensity_offset:i+intensity_offset+4])[0]
                                intensity = max(0.0, min(1.0, raw / 255.0))
                            except Exception:
                                pass
                        all_points.append([x, y, z, intensity])
            # Yield to ROS executor so LIO callbacks are not starved
            rclpy.spin_once(self, timeout_sec=0.05)

        print(f"Debug: Captured {len(all_points)} points over {self.scan_duration}s")
        return all_points
    
    def save_lidar_data(self, points, timestamp, capture_time):
        """Save the LiDAR point cloud data"""
        print(f"Debug: Saving LiDAR data - {len(points)} points")
        
        try:
            
            # Get odometry for transformation (from when image was captured)
            with self.buffer_lock:
                closest_odom = self.find_closest_message(self.odom_buffer, capture_time, max_age=5.0)
                if closest_odom is None and len(self.odom_buffer) > 0:
                    _, closest_odom = self.odom_buffer[-1]
                elif closest_odom is None and self.last_odom is not None:
                    closest_odom = self.last_odom
            
            # Always save sensor coordinates first (for accurate color projection)
            sensor_ply_file = os.path.join(self.output_dir, f'sensor_lidar_{timestamp}.ply')
            with open(sensor_ply_file, 'w') as f:
                f.write('ply\nformat ascii 1.0\n')
                f.write(f'element vertex {len(points)}\n')
                f.write('property float x\nproperty float y\nproperty float z\n')
                if len(points) > 0 and len(points[0]) > 3:
                    f.write('property float intensity\n')
                f.write('end_header\n')
                for p in points:
                    if len(p) > 3:
                        f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\n')
                    else:
                        f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
            print(f"Debug: Saved sensor coordinates to {sensor_ply_file}")
            
            # Also save world coordinates if odometry available
            if closest_odom:
                import numpy as np
                pos = closest_odom.pose.pose.position
                ori = closest_odom.pose.pose.orientation
                
                print(f"Debug: Also transforming {len(points)} points to world coordinates")
                
                # Rotation matrix from quaternion
                x, y, z, w = ori.x, ori.y, ori.z, ori.w
                norm = np.sqrt(x*x + y*y + z*z + w*w)
                if norm > 0:
                    x, y, z, w = x/norm, y/norm, z/norm, w/norm
                    
                    R = np.array([
                        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
                    ])
                    
                    # Transform points to world coordinates
                    world_points = []
                    for point in points:
                        p = np.array(point[:3])  # Only transform XYZ
                        p_world = R @ p + np.array([pos.x, pos.y, pos.z])
                        if len(point) > 3:
                            world_points.append(p_world.tolist() + [point[3]])  # Add intensity
                        else:
                            world_points.append(p_world.tolist())
                    
                    # Save world coordinates
                    world_ply_file = os.path.join(self.output_dir, f'world_lidar_{timestamp}.ply')
                    with open(world_ply_file, 'w') as f:
                        f.write('ply\nformat ascii 1.0\n')
                        f.write(f'element vertex {len(world_points)}\n')
                        f.write('property float x\nproperty float y\nproperty float z\n')
                        if len(world_points) > 0 and len(world_points[0]) > 3:
                            f.write('property float intensity\n')
                        f.write('end_header\n')
                        for p in world_points:
                            if len(p) > 3:
                                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\n')
                            else:
                                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
                    print(f"Debug: Saved world coordinates to {world_ply_file}")
                    ply_file = world_ply_file  # For display message
                else:
                    print(f"Debug: Invalid quaternion, only sensor coordinates saved")
                    ply_file = sensor_ply_file
            else:
                print(f"Debug: No odometry, only sensor coordinates saved")
                ply_file = sensor_ply_file
            
            print(f'✓ Dense {self.scan_duration}s capture: {len(points)} points')
            print(f'  - {sensor_ply_file} (sensor coordinates - use for color projection)')
            if closest_odom and 'world' in ply_file:
                print(f'  - {ply_file} (world coordinates - use for mapping)')
            
        except Exception as e:
            print(f'✗ LiDAR data save failed: {e}')
            import traceback
            traceback.print_exc()
    
    def save_masked_image(self, image, mask_file, output_file):
        """Save image with mask applied as alpha channel"""
        try:
            # Load mask
            mask = cv2.imread(mask_file, cv2.IMREAD_GRAYSCALE)
            if mask is None:
                print(f'Warning: Could not load mask {mask_file}')
                return
            
            # Resize mask to match image
            if mask.shape != image.shape[:2]:
                mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
                print(f'Debug: Resized mask to {image.shape[1]}x{image.shape[0]}')
            
            # Create RGBA image
            rgba_image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            
            # Apply mask to alpha channel (white=opaque, black=transparent)
            rgba_image[:, :, 3] = mask
            
            # Save as PNG with alpha
            cv2.imwrite(output_file, rgba_image)
            
        except Exception as e:
            print(f'Error creating masked image: {e}')

def main():
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        output_dir = "."
    
    # Buffer size can be passed as second argument
    buffer_size = 15 if len(sys.argv) <= 2 else int(sys.argv[2])
    
    # Scan duration can be passed as third argument (default 2.0 seconds)
    scan_duration = 2.0 if len(sys.argv) <= 3 else float(sys.argv[3])
    
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    rclpy.init()
    node = BufferedCameraCapture(output_dir, buffer_size, scan_duration)
    
    try:
        # Allow time to fill buffers before capture
        print("Debug: Filling buffers for 3 seconds...")
        
        # Spin for a bit to get initial data
        for i in range(30):  # 3 seconds at 10Hz
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Check buffer status after filling
        with node.buffer_lock:
            print(f"Debug: Initial buffer status - LiDAR: {len(node.lidar_buffer)}, Odom: {len(node.odom_buffer)}")
            if len(node.odom_buffer) == 0 and node.last_odom is None:
                print("Warning: No odometry data received - check if /rko_lio/odometry is publishing")
            elif node.last_odom is not None:
                pos = node.last_odom.pose.pose.position
                print(f"Debug: Have fallback odometry data available at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            print(f"Debug: Total odometry messages received: {node.odom_received_count}")
            
            # Enable capture only after buffers are filled
            node.buffers_ready = True
            print("Debug: Buffers ready, enabling camera capture")
        
        # Spin until capture is complete or timeout
        start_time = time.time()
        timeout = 20.0  # 20 second timeout
        
        while rclpy.ok() and not node.captured:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.time() - start_time
            if elapsed > timeout:
                print("✗ Timeout waiting for camera frame")
                print(f"Debug: Final buffer status - LiDAR: {len(node.lidar_buffer)}, Odom: {len(node.odom_buffer)}")
                print("Debug: Camera may need restart or /equirectangular/image topic not publishing")
                break
                
            # Show buffer status while waiting
            if elapsed > 5.0 and not node.captured and int(elapsed) % 5 == 0:  # Every 5 seconds after initial wait
                with node.buffer_lock:
                    print(f"Debug: Waiting for camera frame - LiDAR: {len(node.lidar_buffer)}, Odom: {len(node.odom_buffer)} (elapsed: {elapsed:.1f}s)")
            

                
    except KeyboardInterrupt:
        print("✗ Interrupted by user")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()