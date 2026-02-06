#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUTransformNode(Node):
    def __init__(self):
        super().__init__('imu_transform_node')
        
        # Parameters
        self.declare_parameter('input_imu_topic', '/imu/data_raw')
        self.declare_parameter('output_imu_topic', '/livox/imu')
        self.declare_parameter('input_frame', 'camera')
        self.declare_parameter('output_frame', 'livox_frame')
        
        input_topic = self.get_parameter('input_imu_topic').value
        output_topic = self.get_parameter('output_imu_topic').value
        self.input_frame = self.get_parameter('input_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        
        # Create subscriber and publisher
        self.imu_sub = self.create_subscription(
            Imu, input_topic, self.imu_callback, 10)
        self.imu_pub = self.create_publisher(Imu, output_topic, 10)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f'IMU Transform Node started')
        self.get_logger().info(f'Input: {input_topic} ({self.input_frame})')
        self.get_logger().info(f'Output: {output_topic} ({self.output_frame})')
        
    def imu_callback(self, msg):
        try:
            # Get transform from camera to livox frame
            transform = self.tf_buffer.lookup_transform(
                self.output_frame, self.input_frame, rclpy.time.Time())
            
            # Transform IMU data
            transformed_msg = self.transform_imu(msg, transform)
            transformed_msg.header.frame_id = self.output_frame
            
            # Publish transformed IMU
            self.imu_pub.publish(transformed_msg)
            
        except Exception as e:
            # If no transform available, just republish with new frame
            msg.header.frame_id = self.output_frame
            self.imu_pub.publish(msg)
    
    def transform_imu(self, imu_msg, transform):
        """Transform IMU data using the provided transform"""
        # Extract rotation from transform
        q = transform.transform.rotation
        rotation = R.from_quat([q.x, q.y, q.z, q.w])
        
        # Transform angular velocity
        angular_vel = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        transformed_angular_vel = rotation.apply(angular_vel)
        
        # Transform linear acceleration
        linear_acc = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        transformed_linear_acc = rotation.apply(linear_acc)
        
        # Create new message
        transformed_msg = Imu()
        transformed_msg.header = imu_msg.header
        
        # Set transformed values
        transformed_msg.angular_velocity.x = transformed_angular_vel[0]
        transformed_msg.angular_velocity.y = transformed_angular_vel[1]
        transformed_msg.angular_velocity.z = transformed_angular_vel[2]
        
        transformed_msg.linear_acceleration.x = transformed_linear_acc[0]
        transformed_msg.linear_acceleration.y = transformed_linear_acc[1]
        transformed_msg.linear_acceleration.z = transformed_linear_acc[2]
        
        # Copy covariances (could be transformed too, but keeping simple)
        transformed_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        transformed_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        transformed_msg.orientation_covariance = imu_msg.orientation_covariance
        
        # Transform orientation if available
        if imu_msg.orientation.w != 0.0:  # Check if orientation is set
            orig_orientation = R.from_quat([
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w
            ])
            transformed_orientation = rotation * orig_orientation
            quat = transformed_orientation.as_quat()
            
            transformed_msg.orientation.x = quat[0]
            transformed_msg.orientation.y = quat[1]
            transformed_msg.orientation.z = quat[2]
            transformed_msg.orientation.w = quat[3]
        else:
            transformed_msg.orientation = imu_msg.orientation
        
        return transformed_msg

def main(args=None):
    rclpy.init(args=args)
    node = IMUTransformNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()