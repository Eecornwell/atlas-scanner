#!/usr/bin/env python3
"""
Fix livox_ros_driver2 CMakeLists.txt for ROS2 Humble compatibility.
This script modifies the target_include_directories section to conditionally
include LIVOX_INTERFACES_INCLUDE_DIRECTORIES only when not using Humble.

Usage: python3 fix_livox_cmake.py
"""

import os
import sys

def main():
    # Path to the CMakeLists.txt file
    cmake_file = os.path.expanduser('~/atlas_ws/src/livox_ros_driver2/CMakeLists.txt')
    
    # Check if file exists
    if not os.path.exists(cmake_file):
        print(f"Error: File not found: {cmake_file}")
        sys.exit(1)
    
    # Read the file
    with open(cmake_file, 'r') as f:
        content = f.read()
    
    # Original code to replace
    old = '''  # include file direcotry
  target_include_directories(${PROJECT_NAME} PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
    ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs
    3rdparty
    src
  )'''
    
    # New code with conditional logic
    new = '''  # include file direcotry
  target_include_directories(${PROJECT_NAME} PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
    3rdparty
    src
  )
  if(NOT HUMBLE_ROS STREQUAL "humble")
    if(LIVOX_INTERFACES_INCLUDE_DIRECTORIES)
      target_include_directories(${PROJECT_NAME} PUBLIC
        ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}
      )
    endif()
  endif()'''
    
    # Check if the old pattern exists
    if old not in content:
        print("Warning: Original pattern not found in CMakeLists.txt")
        print("The file may have already been modified or has a different structure.")
        sys.exit(1)
    
    # Replace the content
    content = content.replace(old, new)
    
    # Write back to file
    with open(cmake_file, 'w') as f:
        f.write(content)
    
    print(f"Successfully modified {cmake_file}")
    print("The CMakeLists.txt has been updated for ROS2 Humble compatibility.")

if __name__ == '__main__':
    main()
