#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Utility script that kills any running livox_ros_driver2 process and relaunches it, polling until /livox/lidar and /livox/imu topics are available.

import subprocess
import re
import time

def restart_lidar():
    """Restart LiDAR driver"""
    print("Restarting LiDAR driver...")

    # Kill existing LiDAR processes — escape pattern so regex metacharacters
    # in the process name are treated as literals (CWE-88).
    subprocess.run(['pkill', '-f', re.escape('livox_ros_driver2')], capture_output=True)
    time.sleep(3)

    # Start LiDAR driver
    cmd = ['ros2', 'launch', 'livox_ros_driver2', 'rviz_MID360_launch.py']
    process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Wait for topics to appear
    for i in range(30):
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            if '/livox/lidar' in result.stdout and '/livox/imu' in result.stdout:
                print("\u2713 LiDAR driver restarted successfully")
                return True
        except (subprocess.SubprocessError, OSError):
            pass
        time.sleep(1)

    print("\u2717 LiDAR driver restart failed")
    return False

if __name__ == '__main__':
    restart_lidar()