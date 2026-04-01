#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: ROS2 node that triggers a synchronised LiDAR and camera capture on the first incoming LiDAR frame after buffers are ready. Used as a fallback capture path.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
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
    def __init__(self, output_dir=".", image_buffer_size=5, context=None):
        super().__init__('lidar_driven_capture', context=context)
        self.bridge = CvBridge()
        self.captured = False
        self.output_dir = output_dir
        self.image_buffer_size = image_buffer_size
        
        # Image buffer to store recent camera frames
        self.image_buffer = deque(maxlen=image_buffer_size)
        self.latest_odom = None
        self.buffer_lock = threading.Lock()
        
        # Subscriptions - LiDAR drives the timing
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        image_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_cb, lidar_qos)
        self.image_sub = self.create_subscription(CompressedImage, '/dual_fisheye/image/compressed', self.image_cb, image_qos)
        self.image_raw_sub = self.create_subscription(Image, '/dual_fisheye/image', self.image_raw_cb, image_qos)
        self.odom_sub = self.create_subscription(Odometry, '/rko_lio/odometry', self.odom_cb, 1)
        
        # Wait for image buffer to fill
        self.min_images_before_capture = 2
        

    def image_cb(self, msg):
        """Store compressed images in buffer — skip H.264 streams (decoded via image_raw_cb)."""
        fmt = getattr(msg, 'format', '').lower()
        if 'h264' in fmt or 'h.264' in fmt:
            return
        timestamp = time.time()
        try:
            import numpy as _np
            buf = _np.frombuffer(bytes(msg.data), dtype=_np.uint8)
            cv_image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if cv_image is None:
                return
            with self.buffer_lock:
                self.image_buffer.append((timestamp, cv_image))
        except Exception as e:
            print(f"Failed to process image: {e}")

    def image_raw_cb(self, msg):
        """Store software-decoded frames (from the H.264 decoder node)."""
        timestamp = time.time()
        try:
            import numpy as _np
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.buffer_lock:
                self.image_buffer.append((timestamp, cv_image))
        except Exception as e:
            print(f"Failed to process decoded image: {e}")
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
        
        
        try:
            # Get most recent image from buffer
            most_recent_image = None
            image_age = float('inf')
            
            with self.buffer_lock:
                if self.image_buffer:
                    img_timestamp, most_recent_image = self.image_buffer[-1]  # Most recent
                    image_age = capture_time - img_timestamp
            
            if most_recent_image is None:
                return
                
            # Save image
            img_file = os.path.join(self.output_dir, f'dual_fisheye_{timestamp}.jpg')
            cv2.imwrite(img_file, most_recent_image)
            
            # Process LiDAR data
            points = []
            
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
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    buffer_size = 5 if len(sys.argv) <= 2 else int(sys.argv[2])

    os.makedirs(output_dir, exist_ok=True)

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = LidarDrivenCapture(output_dir, buffer_size, context=ctx)
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(2.0)  # allow DDS discovery with node already spinning

    try:
        time.sleep(8.0)


        if len(node.image_buffer) == 0:
            print("✗ No camera frames received - check /dual_fisheye/image/compressed or /dual_fisheye/image is publishing")
            return

        timeout = 30.0
        capture_start = time.time()
        while not node.captured:
            time.sleep(0.1)
            if time.time() - capture_start > timeout:
                print("✗ Timeout waiting for LiDAR frame")
                break

    except KeyboardInterrupt:
        print("✗ Interrupted by user")
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            rclpy.shutdown(context=ctx)
        except Exception:
            pass

if __name__ == '__main__':
    main()