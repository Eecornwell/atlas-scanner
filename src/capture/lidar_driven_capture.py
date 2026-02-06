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

class LidarDrivenCapture(Node):
    def __init__(self, output_dir=".", image_buffer_size=5):
        super().__init__('lidar_driven_capture')
        self.bridge = CvBridge()
        self.captured = False
        self.output_dir = output_dir
        self.image_buffer_size = image_buffer_size
        
        # Image buffer to store recent camera frames
        self.image_buffer = deque(maxlen=image_buffer_size)
        self.latest_odom = None
        self.buffer_lock = threading.Lock()
        
        # Subscriptions - LiDAR drives the timing
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_cb, 1)
        self.image_sub = self.create_subscription(Image, '/equirectangular/image', self.image_cb, 5)
        self.odom_sub = self.create_subscription(Odometry, '/rko_lio/odometry', self.odom_cb, 1)
        
        # Wait for image buffer to fill
        self.min_images_before_capture = 2
        
        print(f"Debug: LiDAR-driven capture initialized with image buffer size {image_buffer_size}")

    def image_cb(self, msg):
        """Store images in buffer with timestamps"""
        timestamp = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.buffer_lock:
                self.image_buffer.append((timestamp, cv_image))
            
            if not hasattr(self, '_image_count'):
                self._image_count = 0
            self._image_count += 1
            if self._image_count % 5 == 0:
                print(f"Debug: Image buffer size: {len(self.image_buffer)}")
        except Exception as e:
            print(f"Debug: Failed to process image: {e}")
        
    def odom_cb(self, msg):
        """Store latest odometry"""
        self.latest_odom = msg

    def lidar_cb(self, msg):
        """LiDAR-driven capture - use most recent image from buffer"""
        if self.captured:
            return
            
        # Wait for image buffer to have some data
        with self.buffer_lock:
            if len(self.image_buffer) < self.min_images_before_capture:
                return
                
        self.captured = True
        capture_time = time.time()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        print(f"Debug: LiDAR frame triggered capture, using most recent image from buffer...")
        
        try:
            # Get most recent image from buffer
            most_recent_image = None
            image_age = float('inf')
            
            with self.buffer_lock:
                if self.image_buffer:
                    img_timestamp, most_recent_image = self.image_buffer[-1]  # Most recent
                    image_age = capture_time - img_timestamp
                    print(f"Debug: Using image from buffer (age: {image_age:.3f}s)")
            
            if most_recent_image is None:
                print("Debug: No images available in buffer")
                return
                
            # Save image
            img_file = os.path.join(self.output_dir, f'equirect_{timestamp}.jpg')
            cv2.imwrite(img_file, most_recent_image)
            
            # Process LiDAR data
            points = []
            print(f"Debug: LiDAR data size: {len(msg.data)} bytes, point_step: {msg.point_step}")
            
            raw_points = 0
            filtered_points = 0
            for i in range(0, len(msg.data), msg.point_step):
                if i + 12 <= len(msg.data):
                    x = struct.unpack('<f', msg.data[i:i+4])[0]
                    y = struct.unpack('<f', msg.data[i+4:i+8])[0]
                    z = struct.unpack('<f', msg.data[i+8:i+12])[0]
                    raw_points += 1
                    if abs(x) < 20 and abs(y) < 20 and abs(z) < 10:
                        points.append([x, y, z])
                        filtered_points += 1
            print(f"Debug: Raw points: {raw_points}, Filtered points: {filtered_points}")
            
            # Transform to world coordinates if odometry available
            if self.latest_odom and points:
                import numpy as np
                pos = self.latest_odom.pose.pose.position
                ori = self.latest_odom.pose.pose.orientation
                
                # Rotation matrix from quaternion
                x, y, z, w = ori.x, ori.y, ori.z, ori.w
                norm = np.sqrt(x*x + y*y + z*z + w*w)
                x, y, z, w = x/norm, y/norm, z/norm, w/norm
                
                R = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                    [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                    [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
                ])
                
                # Transform points to world coordinates
                world_points = []
                for point in points:
                    p = np.array(point)
                    p_world = R @ p + np.array([pos.x, pos.y, pos.z])
                    world_points.append(p_world.tolist())
                points = world_points
                ply_file = os.path.join(self.output_dir, f'world_lidar_{timestamp}.ply')
            else:
                ply_file = os.path.join(self.output_dir, f'sensor_lidar_{timestamp}.ply')
            
            # Save PLY
            with open(ply_file, 'w') as f:
                f.write('ply\nformat ascii 1.0\n')
                f.write(f'element vertex {len(points)}\n')
                f.write('property float x\nproperty float y\nproperty float z\nend_header\n')
                for p in points:
                    f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
            
            print(f'✓ LiDAR-driven capture: {len(points)} points')
            print(f'  Image age: {image_age:.3f}s (acceptable if < 2.0s)')
            print(f'  - {img_file}')
            print(f'  - {ply_file}')
            
        except Exception as e:
            print(f'✗ LiDAR-driven capture failed: {e}')
            import traceback
            traceback.print_exc()

def main():
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        output_dir = "."
    
    # Image buffer size can be passed as second argument
    buffer_size = 5 if len(sys.argv) <= 2 else int(sys.argv[2])
    
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    rclpy.init()
    node = LidarDrivenCapture(output_dir, buffer_size)
    
    try:
        # Allow time to fill image buffer
        print("Debug: Filling image buffer for 5 seconds...")
        start_time = time.time()
        
        while time.time() - start_time < 5:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        print(f"Debug: Image buffer filled with {len(node.image_buffer)} images")
        
        # Now wait for LiDAR to trigger capture
        timeout = 30.0
        capture_start = time.time()
        
        while rclpy.ok() and not node.captured:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - capture_start > timeout:
                print("✗ Timeout waiting for LiDAR frame")
                break
                
    except KeyboardInterrupt:
        print("✗ Interrupted by user")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()