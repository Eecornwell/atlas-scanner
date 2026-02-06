#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import struct
import sys
import time
import os
from datetime import datetime

class LidarOnlyCapture(Node):
    def __init__(self, output_dir="."):
        super().__init__('lidar_only_capture')
        self.captured = False
        self.output_dir = output_dir
        
        self.latest_lidar = None
        self.latest_odom = None
        
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, '/rko_lio/odometry', self.odom_cb, 1)

    def lidar_cb(self, msg): 
        if not self.captured:
            self.latest_lidar = msg
            self.capture_data()
        
    def odom_cb(self, msg): 
        self.latest_odom = msg

    def capture_data(self):
        if self.captured:
            return
            
        self.captured = True
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            points = []
            if self.latest_lidar:
                for i in range(0, len(self.latest_lidar.data), self.latest_lidar.point_step):
                    if i + 12 <= len(self.latest_lidar.data):
                        x = struct.unpack('<f', self.latest_lidar.data[i:i+4])[0]
                        y = struct.unpack('<f', self.latest_lidar.data[i+4:i+8])[0]
                        z = struct.unpack('<f', self.latest_lidar.data[i+8:i+12])[0]
                        if abs(x) < 20 and abs(y) < 20 and abs(z) < 10:
                            points.append([x, y, z])
            
            # Save sensor coordinates PLY
            sensor_ply = os.path.join(self.output_dir, f'sensor_lidar_{timestamp}.ply')
            with open(sensor_ply, 'w') as f:
                f.write('ply\nformat ascii 1.0\n')
                f.write(f'element vertex {len(points)}\n')
                f.write('property float x\nproperty float y\nproperty float z\nend_header\n')
                for p in points:
                    f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
            
            # Transform to world coordinates if odometry available
            if self.latest_odom:
                import numpy as np
                pos = self.latest_odom.pose.pose.position
                ori = self.latest_odom.pose.pose.orientation
                
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
                        p = np.array(point)
                        p_world = R @ p + np.array([pos.x, pos.y, pos.z])
                        world_points.append(p_world.tolist())
                    
                    # Save world coordinates PLY
                    world_ply = os.path.join(self.output_dir, f'world_lidar_{timestamp}.ply')
                    with open(world_ply, 'w') as f:
                        f.write('ply\nformat ascii 1.0\n')
                        f.write(f'element vertex {len(world_points)}\n')
                        f.write('property float x\nproperty float y\nproperty float z\nend_header\n')
                        for p in world_points:
                            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
                    
                    print(f'✓ LiDAR capture: {len(points)} points')
                    print(f'  - {sensor_ply}')
                    print(f'  - {world_ply}')
                else:
                    print(f'✓ LiDAR capture: {len(points)} points')
                    print(f'  - {sensor_ply}')
            else:
                print(f'✓ LiDAR capture: {len(points)} points')
                print(f'  - {sensor_ply}')
            
        except Exception as e:
            print(f'✗ Capture failed: {e}')

def main():
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        output_dir = "."
    
    os.makedirs(output_dir, exist_ok=True)
    
    rclpy.init()
    node = LidarOnlyCapture(output_dir)
    
    try:
        start_time = time.time()
        timeout = 10.0
        
        while rclpy.ok() and not node.captured:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                print("✗ Timeout waiting for LiDAR data")
                break
                
    except KeyboardInterrupt:
        print("✗ Interrupted by user")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()