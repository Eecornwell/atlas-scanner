#!/usr/bin/env python3
"""
Single fisheye capture using decoded images (alternative to compressed).
Uses /dual_fisheye/image topic which is already decoded by the decoder node.
More reliable but slightly higher bandwidth.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import struct
import cv2
import sys
import time
import os
import numpy as np
from datetime import datetime
from collections import deque
import threading

class SingleFisheyeCaptureDecoded(Node):
    def __init__(self, output_dir=".", buffer_size=10, scan_duration=2.0):
        super().__init__('single_fisheye_capture_decoded')
        self.captured = False
        self.output_dir = output_dir
        self.buffer_size = buffer_size
        self.scan_duration = scan_duration
        
        # Buffered storage
        self.lidar_buffer = deque(maxlen=buffer_size)
        self.odom_buffer = deque(maxlen=buffer_size * 3)
        self.buffer_lock = threading.Lock()
        self.last_odom = None
        self.odom_received_count = 0
        self.buffers_ready = False
        
        self.latest_image = None
        self.image_lock = threading.Lock()

        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_cb, 20)
        self.image_sub = self.create_subscription(Image, '/dual_fisheye/image', self.image_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, '/rko_lio/odometry', self.odom_cb, 10)
        
        print(f"✓ Single fisheye capture initialized")
        print(f"  - Using /dual_fisheye/image/compressed (direct decode)")
        print(f"  - LiDAR capture duration: {scan_duration}s")

    def lidar_cb(self, msg):
        timestamp = time.time()
        with self.buffer_lock:
            self.lidar_buffer.append((timestamp, msg))
        
    def odom_cb(self, msg):
        timestamp = time.time()
        with self.buffer_lock:
            self.odom_buffer.append((timestamp, msg))
            self.last_odom = msg
            self.odom_received_count += 1

    def find_closest_message(self, buffer, target_time, max_age=0.5):
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

    def extract_front_fisheye(self, image_msg):
        """Extract LiDAR-facing (back) fisheye from decoded dual image"""
        try:
            from cv_bridge import CvBridge
            dual_fisheye = CvBridge().imgmsg_to_cv2(image_msg, 'bgr8')
            back_fisheye = dual_fisheye[:, :dual_fisheye.shape[1] // 2]
            return cv2.rotate(back_fisheye, cv2.ROTATE_90_CLOCKWISE)
        except Exception as e:
            print(f"Error extracting fisheye: {e}")
            return None

    def image_cb(self, msg):
        """Cache latest compressed image"""
        with self.image_lock:
            self.latest_image = msg

    def capture_dense_points(self):
        """Collect buffered LiDAR points"""
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
            
            rclpy.spin_once(self, timeout_sec=0.05)

        print(f"✓ Captured {len(all_points)} points")
        return all_points
    
    def save_lidar_data(self, points, timestamp, capture_time):
        """Save LiDAR point cloud"""
        try:
            # Get odometry
            with self.buffer_lock:
                closest_odom = self.find_closest_message(self.odom_buffer, capture_time, max_age=5.0)
                if closest_odom is None and len(self.odom_buffer) > 0:
                    _, closest_odom = self.odom_buffer[-1]
                elif closest_odom is None and self.last_odom is not None:
                    closest_odom = self.last_odom
            
            # Save sensor coordinates
            sensor_ply_file = os.path.join(self.output_dir, f'sensor_lidar_{timestamp}.ply')
            with open(sensor_ply_file, 'w') as f:
                f.write('ply\\nformat ascii 1.0\\n')
                f.write(f'element vertex {len(points)}\\n')
                f.write('property float x\\nproperty float y\\nproperty float z\\n')
                if len(points) > 0 and len(points[0]) > 3:
                    f.write('property float intensity\\n')
                f.write('end_header\\n')
                for p in points:
                    if len(p) > 3:
                        f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\\n')
                    else:
                        f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\\n')
            
            # Save world coordinates if odometry available
            if closest_odom:
                pos = closest_odom.pose.pose.position
                ori = closest_odom.pose.pose.orientation
                
                x, y, z, w = ori.x, ori.y, ori.z, ori.w
                norm = np.sqrt(x*x + y*y + z*z + w*w)
                if norm > 0:
                    x, y, z, w = x/norm, y/norm, z/norm, w/norm
                    
                    R = np.array([
                        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
                    ])
                    
                    world_points = []
                    for point in points:
                        p = np.array(point[:3])
                        p_world = R @ p + np.array([pos.x, pos.y, pos.z])
                        if len(point) > 3:
                            world_points.append(p_world.tolist() + [point[3]])
                        else:
                            world_points.append(p_world.tolist())
                    
                    world_ply_file = os.path.join(self.output_dir, f'world_lidar_{timestamp}.ply')
                    with open(world_ply_file, 'w') as f:
                        f.write('ply\\nformat ascii 1.0\\n')
                        f.write(f'element vertex {len(world_points)}\\n')
                        f.write('property float x\\nproperty float y\\nproperty float z\\n')
                        if len(world_points) > 0 and len(world_points[0]) > 3:
                            f.write('property float intensity\\n')
                        f.write('end_header\\n')
                        for p in world_points:
                            if len(p) > 3:
                                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\\n')
                            else:
                                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\\n')
                    print(f"✓ Saved: {sensor_ply_file} + {world_ply_file}")
            else:
                print(f"✓ Saved: {sensor_ply_file}")
            
        except Exception as e:
            print(f'✗ LiDAR save failed: {e}')
            import traceback
            traceback.print_exc()

def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    buffer_size = int(sys.argv[2]) if len(sys.argv) > 2 else 15
    scan_duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    
    os.makedirs(output_dir, exist_ok=True)
    
    rclpy.init()
    node = SingleFisheyeCaptureDecoded(output_dir, buffer_size, scan_duration)
    
    try:
        print("✓ Filling buffers...")
        for i in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        
        with node.buffer_lock:
            node.buffers_ready = True
            print(f"✓ Buffers ready - LiDAR: {len(node.lidar_buffer)}, Odom: {len(node.odom_buffer)}")

        # Wait up to 60s for decoder to produce first frame (H.264 needs I-frame)
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            with node.image_lock:
                has_image = node.latest_image is not None
            if has_image:
                break
            if time.time() - start_time > 60.0:
                print("✗ Timeout waiting for camera decoder")
                return

        # Capture
        capture_time = time.time()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        with node.image_lock:
            image_msg = node.latest_image

        print("✓ Capturing front fisheye at full resolution...")
        front_fisheye = node.extract_front_fisheye(image_msg)
        if front_fisheye is None:
            print("✗ Failed to extract fisheye image")
            return

        img_file = os.path.join(output_dir, f'fisheye_{timestamp}.jpg')
        cv2.imwrite(img_file, front_fisheye, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"✓ Saved fisheye image: {img_file} ({front_fisheye.shape[1]}x{front_fisheye.shape[0]})")

        print(f"✓ Capturing LiDAR data for {node.scan_duration}s...")
        points = node.capture_dense_points()
        node.save_lidar_data(points, timestamp, capture_time)
                
    except KeyboardInterrupt:
        print("✗ Interrupted")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
