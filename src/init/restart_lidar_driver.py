#!/usr/bin/env python3

import subprocess
import time

def restart_lidar():
    """Restart LiDAR driver"""
    print("Restarting LiDAR driver...")
    
    # Kill existing LiDAR processes
    subprocess.run(['pkill', '-f', 'livox_ros_driver2'], capture_output=True)
    time.sleep(3)
    
    # Start LiDAR driver
    cmd = ['ros2', 'launch', 'livox_ros_driver2', 'rviz_MID360_launch.py']
    process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # Wait for topics to appear
    for i in range(30):
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            if '/livox/lidar' in result.stdout and '/livox/imu' in result.stdout:
                print("✓ LiDAR driver restarted successfully")
                return True
        except:
            pass
        time.sleep(1)
    
    print("✗ LiDAR driver restart failed")
    return False

if __name__ == '__main__':
    restart_lidar()