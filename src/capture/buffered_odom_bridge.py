#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: ROS2 node that republishes the latest RKO-LIO odometry pose on /rko_lio/odometry_buffered at a fixed rate, ensuring downstream consumers always have a recent pose even during brief LIO gaps.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import copy

class BufferedOdomBridge(Node):
    def __init__(self):
        super().__init__('buffered_odom_bridge')
        self.last_pose = None
        
        # Subscribe to original odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/rko_lio/odometry', self.odom_callback, 10)
        
        # Publish buffered odometry
        self.odom_pub = self.create_publisher(Odometry, '/rko_lio/odometry_buffered', 10)
        
        # Timer to publish buffered pose when needed
        self.timer = self.create_timer(0.05, self.publish_buffered)  # 20 Hz
        
    def odom_callback(self, msg):
        self.last_pose = copy.deepcopy(msg)
        self.odom_pub.publish(msg)  # forward with original LIO header stamp intact
        
    def publish_buffered(self):
        """Always republish last known pose at timer rate to fill LIO gaps."""
        if self.last_pose is None:
            return
        buffered_msg = copy.deepcopy(self.last_pose)
        buffered_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(buffered_msg)

def main():
    rclpy.init()
    bridge = BufferedOdomBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception as e:
        if str(e):
            print(f"Odometry bridge error: {e}")
    finally:
        try:
            bridge.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()