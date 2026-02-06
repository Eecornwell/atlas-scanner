#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import copy

class BufferedOdomBridge(Node):
    def __init__(self):
        super().__init__('buffered_odom_bridge')
        
        self.last_pose = None
        self.last_update_time = 0
        self.buffer_timeout = 5.0  # Use buffered pose if no update for 5 seconds
        
        # Subscribe to original odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/rko_lio/odometry', self.odom_callback, 10)
        
        # Publish buffered odometry
        self.odom_pub = self.create_publisher(Odometry, '/rko_lio/odometry_buffered', 10)
        
        # Timer to publish buffered pose when needed
        self.timer = self.create_timer(0.2, self.publish_buffered)  # 5 Hz
        
    def odom_callback(self, msg):
        """Store latest pose and republish immediately"""
        self.last_pose = copy.deepcopy(msg)
        self.last_update_time = time.time()
        
        # Republish immediately with current timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(msg)
        
    def publish_buffered(self):
        """Publish last known pose if odometry has dropped out"""
        if self.last_pose is None:
            return
            
        time_since_update = time.time() - self.last_update_time
        
        if time_since_update > self.buffer_timeout:
            # Odometry has dropped out, publish last known pose
            buffered_msg = copy.deepcopy(self.last_pose)
            buffered_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Mark as buffered in frame_id
            buffered_msg.header.frame_id = "odom_buffered"
            
            self.odom_pub.publish(buffered_msg)

def main():
    rclpy.init()
    bridge = BufferedOdomBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
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