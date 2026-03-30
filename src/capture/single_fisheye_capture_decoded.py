#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: ROS2 node that captures a synchronised single-fisheye image and a multi-second LiDAR point cloud for stationary scans. Subscribes to the decoded /dual_fisheye/image topic with a compressed fallback, and saves binary PLY output.
"""
Single fisheye capture using decoded images (alternative to compressed).
Uses /dual_fisheye/image topic which is already decoded by the decoder node.
More reliable but slightly higher bandwidth.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import cv2
import sys
import time
import os
import numpy as np
from datetime import datetime
from collections import deque
import threading


def _write_ply_binary(path, pts):
    """Write Nx3 or Nx4 float32 array as binary PLY."""
    has_intensity = pts.ndim == 2 and pts.shape[1] >= 4
    header = (
        'ply\nformat binary_little_endian 1.0\n'
        f'element vertex {len(pts)}\n'
        'property float x\nproperty float y\nproperty float z\n'
        + ('property float intensity\n' if has_intensity else '')
        + 'end_header\n'
    )
    cols = 4 if has_intensity else 3
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(pts[:, :cols].astype(np.float32).tobytes())


class SingleFisheyeCaptureDecoded(Node):
    def __init__(self, output_dir=".", buffer_size=10, scan_duration=2.0):
        super().__init__('single_fisheye_capture_decoded')
        self.captured = False
        self.output_dir = output_dir
        self.buffer_size = buffer_size
        self.scan_duration = scan_duration

        self.lidar_buffer = deque(maxlen=buffer_size)
        self.odom_buffer = deque(maxlen=buffer_size * 3)
        self.buffer_lock = threading.Lock()
        self.last_odom = None
        self.odom_received_count = 0
        self.buffers_ready = False

        self.latest_image = None
        self.image_lock = threading.Lock()

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        image_qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        image_qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        from sensor_msgs.msg import CompressedImage
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_cb, lidar_qos)
        self.image_sub = self.create_subscription(Image, '/dual_fisheye/image', self.image_cb, image_qos_reliable)
        self.compressed_sub = self.create_subscription(CompressedImage, '/dual_fisheye/image/compressed', self.compressed_cb, image_qos_be)
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
        with self.image_lock:
            self.latest_image = ('decoded', msg)

    def compressed_cb(self, msg):
        """Fallback: decode compressed frame inline if decoded topic not yet received."""
        with self.image_lock:
            if self.latest_image is not None and self.latest_image[0] == 'decoded':
                return  # decoded topic already has data, prefer it
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.latest_image = ('compressed', frame)

    def capture_dense_points(self):
        """Collect buffered LiDAR points using vectorised numpy unpacking."""
        scan_start = time.time()
        chunks = []
        processed_messages = set()

        while time.time() - scan_start < self.scan_duration:
            with self.buffer_lock:
                current = list(self.lidar_buffer)

            for _ts, lidar_msg in current:
                msg_id = id(lidar_msg)
                if msg_id in processed_messages:
                    continue
                processed_messages.add(msg_id)

                step = lidar_msg.point_step
                raw = np.frombuffer(bytes(lidar_msg.data), dtype=np.uint8)
                n = len(raw) // step
                if n == 0:
                    continue
                raw = raw[:n * step].reshape(n, step)
                xyz = np.frombuffer(raw[:, :12].tobytes(), dtype=np.float32).reshape(n, 3)
                mask = (np.abs(xyz[:, 0]) < 20) & (np.abs(xyz[:, 1]) < 20) & (np.abs(xyz[:, 2]) < 10)
                xyz = xyz[mask]
                intensity_offset = next(
                    (f.offset for f in lidar_msg.fields if f.name == 'intensity'), None)
                if intensity_offset is not None and intensity_offset + 4 <= step:
                    inten = np.frombuffer(
                        raw[:, intensity_offset:intensity_offset + 4].tobytes(),
                        dtype=np.float32).reshape(n)[mask]
                    inten = np.clip(inten / 255.0, 0.0, 1.0).reshape(-1, 1)
                else:
                    inten = np.full((len(xyz), 1), 0.5, dtype=np.float32)
                if len(xyz):
                    chunks.append(np.hstack([xyz, inten]))

            time.sleep(0.05)

        result = np.vstack(chunks) if chunks else np.empty((0, 4), dtype=np.float32)
        print(f"✓ Captured {len(result)} points")
        return result

    def save_lidar_data(self, points, timestamp, capture_time):
        """Save LiDAR point cloud"""
        try:
            with self.buffer_lock:
                closest_odom = self.find_closest_message(self.odom_buffer, capture_time, max_age=5.0)
                if closest_odom is None and len(self.odom_buffer) > 0:
                    _, closest_odom = self.odom_buffer[-1]
                elif closest_odom is None and self.last_odom is not None:
                    closest_odom = self.last_odom

            pts = np.asarray(points, dtype=np.float32)
            sensor_ply_file = os.path.join(self.output_dir, f'sensor_lidar_{timestamp}.ply')
            _write_ply_binary(sensor_ply_file, pts)

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
                    world_xyz = (R @ pts[:, :3].T).T + np.array([pos.x, pos.y, pos.z])
                    world_pts = np.hstack([world_xyz, pts[:, 3:4]]).astype(np.float32)
                    world_ply_file = os.path.join(self.output_dir, f'world_lidar_{timestamp}.ply')
                    _write_ply_binary(world_ply_file, world_pts)
                    print(f"✓ Saved: {sensor_ply_file} + {world_ply_file}")
                    return
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
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    _t_start = time.time()
    # Wait for LiDAR first (fast), then keep waiting for camera up to 20s total
    while time.time() - _t_start < 20.0:
        time.sleep(0.05)
        with node.buffer_lock:
            lidar_ok = len(node.lidar_buffer) > 0
        with node.image_lock:
            cam_ok = node.latest_image is not None
        if lidar_ok and cam_ok:
            break

    try:
        with node.buffer_lock:
            lidar_count = len(node.lidar_buffer)
            odom_count = len(node.odom_buffer)
        print(f"✓ Buffers ready - LiDAR: {lidar_count}, Odom: {odom_count}")
        if lidar_count == 0:
            print("✗ No LiDAR data received")
            sys.exit(1)

        with node.image_lock:
            has_image = node.latest_image is not None
        if not has_image:
            print("✗ No camera frame received")
            sys.exit(1)


        capture_time = time.time()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        with node.image_lock:
            image_data = node.latest_image

        print("✓ Capturing front fisheye at full resolution...")
        kind, payload = image_data
        if kind == 'decoded':
            front_fisheye = node.extract_front_fisheye(payload)
        else:
            half = payload.shape[1] // 2
            front_fisheye = cv2.rotate(payload[:, :half], cv2.ROTATE_90_CLOCKWISE)
        if front_fisheye is None:
            print("✗ Failed to extract fisheye image")
            sys.exit(1)

        img_file = os.path.join(output_dir, f'fisheye_{timestamp}.jpg')
        cv2.imwrite(img_file, front_fisheye, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"✓ Saved fisheye image: {img_file} ({front_fisheye.shape[1]}x{front_fisheye.shape[0]})")


        print(f"✓ Capturing LiDAR data for {node.scan_duration}s...")
        points = node.capture_dense_points()
        node.save_lidar_data(points, timestamp, capture_time)

    except KeyboardInterrupt:
        print("✗ Interrupted")
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        executor.shutdown(timeout_sec=0.5)
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
