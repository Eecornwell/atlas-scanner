#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import numpy as np
from datetime import datetime
import os

class EnhancedTrajectoryRecorder(Node):
    def __init__(self, output_dir):
        super().__init__('enhanced_trajectory_recorder')
        
        self.output_dir = output_dir
        self.trajectory = []
        self.start_time = None
        self.first_pose = None  # Store first pose as reference origin
        self.recording_started = True
        
        # Camera-LiDAR calibration parameters (from your calibration)
        self.camera_lidar_transform = {
            'translation': {'x': -0.2, 'y': -0.05, 'z': -0.09},  # Camera relative to LiDAR
            'rotation': {'roll': 178, 'pitch': -0.5, 'yaw': -3.0}  # degrees
        }
        
        self.subscription = self.create_subscription(
            Odometry,
            '/rko_lio/odometry',
            self.odometry_callback,
            10)
        
        # Timer to check for save requests
        self.timer = self.create_timer(0.1, self.check_save_requests)
        
        self.get_logger().info(f'Enhanced trajectory recording ready. Output: {output_dir}')
        self.get_logger().info('Recording trajectory data immediately...')
    
    def calculate_camera_pose(self, lidar_pose):
        """Calculate camera pose from LiDAR pose using calibration"""
        # Extract LiDAR pose
        pos = lidar_pose['position']
        ori = lidar_pose['orientation']
        
        # Convert quaternion to rotation matrix
        x, y, z, w = ori['x'], ori['y'], ori['z'], ori['w']
        
        # Normalize quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # LiDAR rotation matrix
        R_lidar = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        
        # Camera-LiDAR calibration transformation
        cal = self.camera_lidar_transform
        
        # Convert calibration angles to radians
        roll_rad = np.radians(cal['rotation']['roll'])
        pitch_rad = np.radians(cal['rotation']['pitch'])
        yaw_rad = np.radians(cal['rotation']['yaw'])
        
        # Create calibration rotation matrix (ZYX order)
        cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)
        cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)
        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
        
        R_cal = np.array([
            [cos_y*cos_p, cos_y*sin_p*sin_r - sin_y*cos_r, cos_y*sin_p*cos_r + sin_y*sin_r],
            [sin_y*cos_p, sin_y*sin_p*sin_r + cos_y*cos_r, sin_y*sin_p*cos_r - cos_y*sin_r],
            [-sin_p, cos_p*sin_r, cos_p*cos_r]
        ])
        
        # Translation offset
        t_cal = np.array([cal['translation']['x'], cal['translation']['y'], cal['translation']['z']])
        
        # Calculate camera pose
        # Camera position = LiDAR position + LiDAR rotation * calibration translation
        camera_pos = np.array([pos['x'], pos['y'], pos['z']]) + R_lidar @ t_cal
        
        # Camera orientation = LiDAR rotation * calibration rotation
        R_camera = R_lidar @ R_cal
        
        # Convert camera rotation matrix back to quaternion
        trace = R_camera[0,0] + R_camera[1,1] + R_camera[2,2]
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R_camera[2,1] - R_camera[1,2]) / s
            qy = (R_camera[0,2] - R_camera[2,0]) / s
            qz = (R_camera[1,0] - R_camera[0,1]) / s
        elif R_camera[0,0] > R_camera[1,1] and R_camera[0,0] > R_camera[2,2]:
            s = np.sqrt(1.0 + R_camera[0,0] - R_camera[1,1] - R_camera[2,2]) * 2  # s = 4 * qx
            qw = (R_camera[2,1] - R_camera[1,2]) / s
            qx = 0.25 * s
            qy = (R_camera[0,1] + R_camera[1,0]) / s
            qz = (R_camera[0,2] + R_camera[2,0]) / s
        elif R_camera[1,1] > R_camera[2,2]:
            s = np.sqrt(1.0 + R_camera[1,1] - R_camera[0,0] - R_camera[2,2]) * 2  # s = 4 * qy
            qw = (R_camera[0,2] - R_camera[2,0]) / s
            qx = (R_camera[0,1] + R_camera[1,0]) / s
            qy = 0.25 * s
            qz = (R_camera[1,2] + R_camera[2,1]) / s
        else:
            s = np.sqrt(1.0 + R_camera[2,2] - R_camera[0,0] - R_camera[1,1]) * 2  # s = 4 * qz
            qw = (R_camera[1,0] - R_camera[0,1]) / s
            qx = (R_camera[0,2] + R_camera[2,0]) / s
            qy = (R_camera[1,2] + R_camera[2,1]) / s
            qz = 0.25 * s
        
        return {
            'position': {'x': float(camera_pos[0]), 'y': float(camera_pos[1]), 'z': float(camera_pos[2])},
            'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
        }
    
    def odometry_callback(self, msg):
        # Only start recording when first scan is requested
        if not self.recording_started:
            return
            
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info('Started trajectory recording with first scan')
        
        # Extract pose components
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Store LiDAR pose
        lidar_pose = {
            'position': {'x': pos.x, 'y': pos.y, 'z': pos.z},
            'orientation': {'x': ori.x, 'y': ori.y, 'z': ori.z, 'w': ori.w}
        }
        
        # Calculate camera pose
        camera_pose = self.calculate_camera_pose(lidar_pose)
        
        # Convert quaternion to rotation matrix for CloudCompare
        x, y, z, w = ori.x, ori.y, ori.z, ori.w
        
        # ROS to CloudCompare transformation matrix (4x4)
        R_ros = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        
        # Transform from ROS to CloudCompare coordinates
        T_ros_to_cc = np.array([
            [0, 1, 0, 0],  # ROS Y -> CC X
            [1, 0, 0, 0],  # ROS X -> CC Y  
            [0, 0, 1, 0],  # ROS Z -> CC Z
            [0, 0, 0, 1]
        ])
        
        # Create ROS pose matrix
        T_ros = np.eye(4)
        T_ros[0:3, 0:3] = R_ros
        T_ros[0:3, 3] = [pos.x, pos.y, pos.z]
        
        # Store first pose as reference origin
        if self.first_pose is None:
            self.first_pose = T_ros.copy()
        
        # Calculate relative pose from first scan
        T_relative = np.linalg.inv(self.first_pose) @ T_ros
        
        # Transform to CloudCompare coordinates (relative to first pose)
        T_cc_relative = T_ros_to_cc @ T_relative @ np.linalg.inv(T_ros_to_cc)
        
        # Also keep absolute pose for reference
        T_cc_absolute = T_ros_to_cc @ T_ros @ np.linalg.inv(T_ros_to_cc)
        
        # Convert matrices to space-separated strings
        matrix_str_relative = ' '.join([' '.join([f'{val:.6f}' for val in row]) for row in T_cc_relative])
        matrix_str_absolute = ' '.join([' '.join([f'{val:.6f}' for val in row]) for row in T_cc_absolute])
        
        # Store pose data with both LiDAR and camera poses
        pose_data = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'relative_time': (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.start_time,
            'lidar_pose': lidar_pose,
            'camera_pose': camera_pose,
            'position': lidar_pose['position'],  # Keep for backward compatibility
            'orientation': lidar_pose['orientation'],  # Keep for backward compatibility
            'transformation_matrix_cloudcompare': matrix_str_relative,
            'transformation_matrix_cloudcompare_absolute': matrix_str_absolute
        }
        
        self.trajectory.append(pose_data)
    
    def check_save_requests(self):
        """Check for trajectory save requests via trigger files"""
        try:
            for filename in os.listdir(self.output_dir):
                if filename.startswith('.save_trajectory_'):
                    trigger_file = os.path.join(self.output_dir, filename)
                    
                    try:
                        with open(trigger_file, 'r') as f:
                            request_data = json.load(f)
                        
                        scan_name = request_data['scan_name']
                        scan_dir = request_data['scan_dir']
                        
                        capture_time = request_data.get('capture_time')
                        current_pose_snapshot = self.get_pose_at_time(capture_time)
                        if current_pose_snapshot:
                            # Save trajectory for this scan with current pose
                            if self.save_scan_trajectory_to_dir(scan_name, scan_dir, current_pose_snapshot):
                                self.get_logger().info(f'Saved trajectory for {scan_name} to {scan_dir}')
                            else:
                                self.get_logger().error(f'Failed to save trajectory for {scan_name}')
                        else:
                            self.get_logger().error(f'No current pose available for {scan_name}')
                        
                        # Remove trigger file
                        os.remove(trigger_file)
                        
                    except Exception as e:
                        self.get_logger().error(f'Error processing trigger file {filename}: {e}')
                        try:
                            os.remove(trigger_file)
                        except:
                            pass
        except FileNotFoundError:
            pass  # Directory doesn't exist yet
        except Exception as e:
            pass  # Ignore other errors
    
    def get_pose_at_time(self, capture_time=None):
        """Get the pose closest to capture_time (wall clock). Falls back to latest."""
        if not self.trajectory:
            self.get_logger().error(
                'No odometry received - is /rko_lio/odometry publishing? '
                'Check RKO-LIO logs at /tmp/rko_lio.log')
            return None
        if capture_time is None:
            return self.trajectory[-1].copy()
        best = min(self.trajectory, key=lambda p: abs(p['timestamp'] - capture_time))
        return best.copy()
    
    def save_scan_trajectory_to_dir(self, scan_name, scan_dir, pose_snapshot):
        """Save trajectory data for a specific scan to its directory"""
        if not pose_snapshot:
            return False
        
        # Get current timestamp for this specific scan
        scan_timestamp = datetime.now().isoformat()
        scan_request_time = self.get_clock().now().nanoseconds / 1e9
        
        scan_data = {
            'scan_info': {
                'name': scan_name,
                'timestamp': scan_timestamp,
                'scan_request_time': scan_request_time,
                'scan_pose_time': pose_snapshot['timestamp']
            },
            'coordinate_system': {
                'standard': 'ROS_REP_103',
                'frame': 'odom',
                'convention': 'X_forward_Y_left_Z_up',
                'units': {'position': 'meters', 'orientation': 'quaternion_xyzw'}
            },
            'current_pose': {
                'timestamp': pose_snapshot['timestamp'],
                'relative_time': pose_snapshot['relative_time'],
                'lidar_pose': pose_snapshot['lidar_pose'].copy(),
                'camera_pose': pose_snapshot['camera_pose'].copy(),
                'position': pose_snapshot['position'].copy(),  # LiDAR position for compatibility
                'orientation': pose_snapshot['orientation'].copy(),  # LiDAR orientation for compatibility
                'transformation_matrix_cloudcompare': pose_snapshot.get('transformation_matrix_cloudcompare', None),
                'transformation_matrix_cloudcompare_absolute': pose_snapshot.get('transformation_matrix_cloudcompare_absolute', None)
            },
            'calibration': {
                'camera_lidar_transform': self.camera_lidar_transform
            },
            'trajectory_summary': {
                'total_poses': len(self.trajectory),
                'start_time': self.start_time,
                'current_time': pose_snapshot['timestamp'],
                'duration': pose_snapshot['relative_time'],
                'distance_traveled': self.calculate_distance_traveled()
            },
            'full_trajectory': [pose.copy() for pose in self.trajectory]
        }
        
        # Save trajectory only to the individual scan directory
        scan_file = os.path.join(scan_dir, "trajectory.json")
        try:
            with open(scan_file, 'w') as f:
                json.dump(scan_data, f, indent=2)
            self.get_logger().info(f'Saved enhanced trajectory for {scan_name} to {scan_file}: {len(self.trajectory)} poses')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save trajectory: {e}')
            return False
    
    def calculate_distance_traveled(self):
        """Calculate total distance traveled"""
        if len(self.trajectory) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(self.trajectory)):
            prev_pos = self.trajectory[i-1]['position']
            curr_pos = self.trajectory[i]['position']
            
            dx = curr_pos['x'] - prev_pos['x']
            dy = curr_pos['y'] - prev_pos['y']
            dz = curr_pos['z'] - prev_pos['z']
            
            total_distance += np.sqrt(dx*dx + dy*dy + dz*dz)
        
        return total_distance

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 enhanced_trajectory_recorder.py <output_dir>")
        sys.exit(1)
    
    output_dir = sys.argv[1]
    os.makedirs(output_dir, exist_ok=True)
    
    rclpy.init()
    recorder = EnhancedTrajectoryRecorder(output_dir)
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()