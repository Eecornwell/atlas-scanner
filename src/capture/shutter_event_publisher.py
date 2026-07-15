#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Watches a session directory for .shutter_event files written by
# insta360_capture and publishes them as std_msgs/Float64 on /camera/shutter_time.
# Re-publishes when a file is updated with a refined camera-RTC time.

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
from pathlib import Path

_ALLOWED_DATA = Path(os.path.expanduser('~/atlas_ws/data')).resolve()


def _safe_data(p) -> Path:
    resolved = Path(p).resolve()
    if _ALLOWED_DATA not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' is outside allowed root '{_ALLOWED_DATA}'")
    return resolved


class ShutterEventPublisher(Node):
    def __init__(self, session_dir):
        super().__init__('shutter_event_publisher')
        self.session_dir = _safe_data(session_dir)
        self.pub = self.create_publisher(Float64, '/camera/shutter_time', 10)
        # Track: filename -> (last_value, published_value)
        self.file_state = {}
        self.timer = self.create_timer(0.05, self.poll)  # 20Hz polling
        self.get_logger().info(f'Watching {session_dir} for shutter events')

    def poll(self):
        # Watch fusion_scan_*/ subdirs for capture_N.shutter_event files
        for f in self.session_dir.glob('fusion_scan_*/capture_*.shutter_event'):
            try:
                safe_f = _safe_data(f)
                parts = safe_f.read_text().strip().split()
                val = float(parts[0])
            except (ValueError, IndexError):
                continue

            prev = self.file_state.get(str(f))

            # Publish if new file or value changed (refined from t_after to cam_rtc)
            if prev is None or abs(val - prev) > 0.01:
                self.file_state[str(f)] = val
                msg = Float64()
                msg.data = val
                self.pub.publish(msg)
                cam_idx = int(parts[1]) if len(parts) > 1 else 0
                action = 'Published' if prev is None else f'Refined ({prev:.3f}->{val:.3f})'
                self.get_logger().info(f'{action} shutter_t={val:.6f} cam={cam_idx}  ({f.name})')


def main():
    if len(sys.argv) < 2:
        print('Usage: shutter_event_publisher.py <session_dir>')
        sys.exit(1)

    rclpy.init()
    try:
        node = ShutterEventPublisher(sys.argv[1])
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except rclpy.exceptions.InvalidHandle:
            pass


if __name__ == '__main__':
    main()
