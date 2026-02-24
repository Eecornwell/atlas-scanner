#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import yaml
import numpy as np
from datetime import datetime
import os
from pathlib import Path
from scipy.spatial.transform import Rotation

CALIB_PATH = Path.home() / 'atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml'

def _load_camera_lidar_transform():
    """Load T_camera_lidar from fusion_calibration.yaml and return T_lidar_camera (4x4).
    
    fusion_calibration.yaml stores T_camera_lidar (lidar->camera) as euler XYZ + translation.
    Invert once to get T_lidar_camera (camera->lidar frame, i.e. camera position in lidar frame).
    """
    with open(CALIB_PATH) as f:
        calib = yaml.safe_load(f)
    roll = calib['roll_offset']
    pitch = calib['pitch_offset']
    yaw = calib['yaw_offset']
    tx = calib['x_offset']
    ty = calib['y_offset']
    tz = calib['z_offset']
    T_camera_lidar = np.eye(4)
    T_camera_lidar[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T_camera_lidar[:3, 3] = [tx, ty, tz]
    return np.linalg.inv(T_camera_lidar)  # T_lidar_camera: camera position in lidar frame

class EnhancedTrajectoryRecorder(Node):
    def __init__(self, output_dir):
        super().__init__('enhanced_trajectory_recorder')
        
        self.output_dir = output_dir
        self.trajectory = []
        self.start_time = None
        self.first_pose = None  # Store first pose as reference origin
        self.recording_started = True
        
        # Load camera-lidar transform from calibration file
        try:
            self._T_lidar_camera = _load_camera_lidar_transform()
            t = self._T_lidar_camera[:3, 3]
            self.camera_lidar_transform = {
                'translation': {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2])},
                'source': str(CALIB_PATH)
            }
            self.get_logger().info(f'Loaded camera-lidar calibration from {CALIB_PATH}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration ({e}), using identity')
            self._T_lidar_camera = np.eye(4)
            self.camera_lidar_transform = {'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        
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
        """Calculate camera pose from LiDAR pose using calibrated T_lidar_camera."""
        pos = lidar_pose['position']
        ori = lidar_pose['orientation']
        
        q = np.array([ori['x'], ori['y'], ori['z'], ori['w']])
        q /= np.linalg.norm(q)
        R_lidar = Rotation.from_quat(q).as_matrix()
        
        # T_world_camera = T_world_lidar @ T_lidar_camera
        T_world_lidar = np.eye(4)
        T_world_lidar[:3, :3] = R_lidar
        T_world_lidar[:3, 3] = [pos['x'], pos['y'], pos['z']]
        
        T_world_camera = T_world_lidar @ self._T_lidar_camera
        
        cam_pos = T_world_camera[:3, 3]
        cam_quat = Rotation.from_matrix(T_world_camera[:3, :3]).as_quat()  # [x,y,z,w]
        
        return {
            'position': {'x': float(cam_pos[0]), 'y': float(cam_pos[1]), 'z': float(cam_pos[2])},
            'orientation': {'x': float(cam_quat[0]), 'y': float(cam_quat[1]),
                            'z': float(cam_quat[2]), 'w': float(cam_quat[3])}
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