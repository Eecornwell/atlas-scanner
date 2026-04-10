#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: ROS2 node that buffers incoming LiDAR, camera, and odometry messages, then captures a synchronised snapshot on demand. Used as the primary capture backend for dual_fisheye mode and as a fallback for single_fisheye mode.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import time
import os
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


class BufferedCameraCapture(Node):
    def __init__(self, output_dir=".", buffer_size=10, scan_duration=2.0, context=None):
        super().__init__('buffered_camera_capture', context=context)
        self.bridge = CvBridge()
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
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        image_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        odom_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_cb, lidar_qos)
        self.image_sub = self.create_subscription(CompressedImage, '/dual_fisheye/image/compressed', self.image_cb, image_qos)
        self.image_raw_sub = self.create_subscription(Image, '/dual_fisheye/image', self.image_raw_cb, image_qos)
        self.odom_sub = self.create_subscription(Odometry, '/rko_lio/odometry', self.odom_cb, odom_qos)

        print(f"✓ Buffered capture initialized")

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

    def image_cb(self, msg):
        # Always buffer the latest decodable frame regardless of buffers_ready.
        # Dropping frames here causes misses when buffers_ready flips after
        # RELIABLE delivery has already occurred and won't be resent.
        fmt = getattr(msg, 'format', '').lower()
        if 'h264' in fmt or 'h.264' in fmt:
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self.frame_lock:
            self.latest_frame = frame
        if not self.captured and self.buffers_ready:
            self._trigger_capture(frame)

    def image_raw_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            print(f"Error converting raw image: {e}")
            return
        with self.frame_lock:
            self.latest_frame = frame
        if not self.captured and self.buffers_ready:
            self._trigger_capture(frame)

    def _trigger_capture(self, frame):
        self.captured = True
        self.save_complete = False
        threading.Thread(target=self._do_capture, args=(frame,), daemon=True).start()

    def _do_capture(self, frame):
        capture_time = time.time()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        try:
            if frame is None:
                print("Error saving image: could not decode frame")
                self.save_complete = True
                return
            img_file = os.path.join(self.output_dir, f'dual_fisheye_{timestamp}.jpg')
            cv2.imwrite(img_file, frame)
            print(f"✓ Saved fisheye frame: {img_file}")
        except Exception as e:
            print(f"Error saving image: {e}")
            self.save_complete = True
            return

        points = self.capture_dense_points()
        self.save_lidar_data(points, timestamp, capture_time)
        self.save_complete = True

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
        return result

    def save_lidar_data(self, points, timestamp, capture_time):
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
                    ply_file = world_ply_file
                else:
                    ply_file = sensor_ply_file
            else:
                ply_file = sensor_ply_file

            print(f'✓ Dense {self.scan_duration}s capture: {len(pts)} points')
            print(f'  - {sensor_ply_file} (sensor coordinates - use for color projection)')
            if closest_odom and 'world' in ply_file:
                print(f'  - {ply_file} (world coordinates - use for mapping)')

        except Exception as e:
            print(f'✗ LiDAR data save failed: {e}')
            import traceback
            traceback.print_exc()

    def save_masked_image(self, image, mask_file, output_file):
        try:
            mask = cv2.imread(mask_file, cv2.IMREAD_GRAYSCALE)
            if mask is None:
                print(f'Warning: Could not load mask {mask_file}')
                return
            if mask.shape != image.shape[:2]:
                mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
            rgba_image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            rgba_image[:, :, 3] = mask
            cv2.imwrite(output_file, rgba_image)
        except Exception as e:
            print(f'Error creating masked image: {e}')



def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    buffer_size = 15 if len(sys.argv) <= 2 else int(sys.argv[2])
    scan_duration = 2.0 if len(sys.argv) <= 3 else float(sys.argv[3])

    os.makedirs(output_dir, exist_ok=True)

    # Use a fresh context so any prior rclpy.shutdown() in the same process
    # group (e.g. from ros2 topic echo calls in the shell script) cannot
    # poison this node's DDS context and cause ExternalShutdownException.
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = BufferedCameraCapture(output_dir, buffer_size, scan_duration, context=ctx)
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    _t_start = time.time()
    time.sleep(2.0)  # allow DDS discovery with node already spinning


    try:
        fill_start = time.time()
        while time.time() - fill_start < 15.0:
            with node.buffer_lock:
                if len(node.lidar_buffer) > 0:
                    break
            time.sleep(0.1)

        # Wait up to 5s for first odometry message (DDS discovery can lag behind LiDAR)
        odom_wait_start = time.time()
        while time.time() - odom_wait_start < 5.0:
            with node.buffer_lock:
                if node.last_odom is not None:
                    break
            time.sleep(0.1)

        with node.buffer_lock:
            if len(node.odom_buffer) == 0 and node.last_odom is None:
                print("Warning: No odometry data received - check if /rko_lio/odometry is publishing")
            node.save_complete = True
            node.buffers_ready = True

        # If a frame already arrived during the discovery/buffer-fill window,
        # use it immediately rather than waiting for the next one.
        with node.frame_lock:
            buffered = node.latest_frame
        if buffered is not None and not node.captured:
            node._trigger_capture(buffered)

        start_time = time.time()
        timeout = 30.0

        while not node.captured:
            time.sleep(0.1)
            elapsed = time.time() - start_time
            if elapsed > timeout:
                print("✗ Timeout waiting for camera frame")
                break


        save_wait = time.time()
        while node.captured and not node.save_complete:
            time.sleep(0.1)
            if time.time() - save_wait > 60.0:
                print("\u2717 Timeout waiting for PLY save")
                break


    except KeyboardInterrupt:
        print("\u2717 Interrupted by user")
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        executor.shutdown(timeout_sec=0.5)
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        try:
            rclpy.shutdown(context=ctx)
        except Exception:
            pass
        time.sleep(2.0)  # allow FastDDS to release SHM port locks before process exits

    sys.exit(0 if node.captured else 1)


if __name__ == '__main__':
    main()
