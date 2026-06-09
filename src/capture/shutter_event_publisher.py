#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Watches a session directory for .shutter_event files written by
# insta360_capture and publishes them as std_msgs/Float64 on /camera/shutter_time.
# Re-publishes when a file is updated with a refined camera-RTC time.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
from pathlib import Path


class ShutterEventPublisher(Node):
    def __init__(self, session_dir):
        super().__init__('shutter_event_publisher')
        self.session_dir = Path(session_dir)
        self.pub = self.create_publisher(Float64, '/camera/shutter_time', 10)
        # Track: filename -> (last_value, published_value)
        self.file_state = {}
        self.timer = self.create_timer(0.05, self.poll)  # 20Hz polling
        self.get_logger().info(f'Watching {session_dir} for shutter events')

    def poll(self):
        # Watch fusion_scan_*/ subdirs for capture_N.shutter_event files
        for f in self.session_dir.glob('fusion_scan_*/capture_*.shutter_event'):
            try:
                val = float(f.read_text().strip())
            except Exception:
                continue

            prev = self.file_state.get(str(f))

            # Publish if new file or value changed (refined from t_after to cam_rtc)
            if prev is None or abs(val - prev) > 0.01:
                self.file_state[str(f)] = val
                msg = Float64()
                msg.data = val
                self.pub.publish(msg)
                action = 'Published' if prev is None else f'Refined ({prev:.3f}->{val:.3f})'
                self.get_logger().info(f'{action} shutter_t={val:.6f}  ({f.name})')


def main():
    if len(sys.argv) < 2:
        print('Usage: shutter_event_publisher.py <session_dir>')
        sys.exit(1)

    rclpy.init()
    node = ShutterEventPublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
