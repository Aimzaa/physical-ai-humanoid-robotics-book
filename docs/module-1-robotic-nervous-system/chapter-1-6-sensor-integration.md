---
id: chapter-1-6-sensor-integration
title: Sensor Integration (Camera, LiDAR, IMU)
sidebar_label: Sensor Integration
---

# Sensor Integration (Camera, LiDAR, IMU)

## Goal
Integrate various sensors commonly used in humanoid robots, including cameras, LiDAR, and IMUs, into ROS 2 systems with proper data processing and calibration.

## Learning Objectives
- Integrate camera sensors and process image data in ROS 2
- Connect and utilize LiDAR sensors for environment perception
- Integrate IMU sensors for orientation and balance information
- Understand sensor data processing pipelines in ROS 2
- Implement sensor fusion techniques for humanoid robotics

## Overview
Sensor integration is critical for humanoid robots to perceive and interact with their environment. Cameras provide visual information, LiDAR offers precise distance measurements, and IMUs give orientation and acceleration data. Proper integration of these sensors in ROS 2 requires understanding of sensor drivers, data formats, calibration procedures, and processing pipelines. For humanoid robots, sensor fusion is particularly important for balance, navigation, and interaction with the environment.

## Key Concepts
- **sensor_msgs**: Standard message types for sensor data in ROS 2
- **image_transport**: Framework for efficient image data transmission
- **tf2**: Transform library for coordinate frame management
- **Calibration**: Process of determining sensor parameters and relationships
- **Sensor Fusion**: Combining data from multiple sensors for better perception
- **Driver Nodes**: ROS 2 nodes that interface with physical sensors

## Step-by-Step Breakdown
1. **Camera Integration**
   - Use camera drivers that publish sensor_msgs/Image
   - Implement image processing pipelines
   - Use image_transport for efficient transmission
   - Apply camera calibration for accurate measurements

2. **LiDAR Integration**
   - Connect LiDAR sensors via USB, Ethernet, or serial
   - Process LaserScan or PointCloud2 messages
   - Implement obstacle detection and mapping
   - Handle different LiDAR types (2D, 3D, multi-layer)

3. **IMU Integration**
   - Process sensor_msgs/Imu messages
   - Understand orientation, angular velocity, and linear acceleration
   - Implement sensor fusion for attitude estimation
   - Use IMU data for balance control in humanoid robots

4. **Coordinate Frame Management**
   - Use tf2 for managing sensor positions and orientations
   - Establish proper transform chains
   - Transform data between coordinate frames
   - Handle dynamic transforms for moving sensors

5. **Sensor Calibration**
   - Camera intrinsic and extrinsic calibration
   - LiDAR calibration for multi-sensor systems
   - IMU bias and alignment calibration
   - Multi-sensor calibration procedures

## Code Examples
```python
# Example sensor integration node for humanoid robot
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, PointCloud2
from geometry_msgs.msg import Vector3, Quaternion
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorIntegrator(Node):
    def __init__(self):
        super().__init__('sensor_integrator')

        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Create subscribers for different sensor types
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publisher for processed sensor data
        self.fused_sensor_pub = self.create_publisher(
            PointCloud2,
            '/fused_pointcloud',
            10
        )

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Sensor Integrator initialized')

    def image_callback(self, msg):
        """Process camera image data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform image processing (e.g., object detection)
            # This is where you'd implement your computer vision algorithms
            processed_image = self.process_image(cv_image)

            # Publish processed data or publish transforms
            self.get_logger().info(f'Received image: {msg.width}x{msg.height}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR scan data"""
        # Process LaserScan message
        ranges = np.array(msg.ranges)

        # Filter out invalid measurements
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'Min distance: {min_distance:.2f}m')

            # Implement obstacle detection logic here
            obstacles = self.detect_obstacles(ranges, msg.angle_min, msg.angle_increment)

    def imu_callback(self, msg):
        """Process IMU data for balance and orientation"""
        # Extract orientation from IMU
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Calculate roll, pitch, yaw from quaternion
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # For humanoid robots, use IMU data for balance control
        balance_info = {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'angular_velocity': (angular_velocity.x, angular_velocity.y, angular_velocity.z),
            'linear_acceleration': (linear_acceleration.x, linear_acceleration.y, linear_acceleration.z)
        }

        self.get_logger().info(f'Orientation - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')

    def process_image(self, cv_image):
        """Placeholder for image processing logic"""
        # Example: Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        return gray

    def detect_obstacles(self, ranges, angle_min, angle_increment):
        """Detect obstacles from LiDAR data"""
        angles = np.arange(len(ranges)) * angle_increment + angle_min
        valid_indices = np.isfinite(ranges)

        if np.any(valid_indices):
            valid_ranges = ranges[valid_indices]
            valid_angles = angles[valid_indices]

            # Detect obstacles within a certain range (e.g., 1 meter)
            obstacle_indices = valid_ranges < 1.0
            if np.any(obstacle_indices):
                obstacle_angles = valid_angles[obstacle_indices]
                self.get_logger().info(f'Found {len(obstacle_angles)} obstacles')
                return list(zip(obstacle_angles, valid_ranges[obstacle_indices]))

        return []

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    sensor_integrator = SensorIntegrator()

    try:
        rclpy.spin(sensor_integrator)
    except KeyboardInterrupt:
        sensor_integrator.get_logger().info('Interrupted by user')
    finally:
        sensor_integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Diagrams
```
Sensor Integration Architecture:

┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Camera    │    │   LiDAR     │    │    IMU      │
│             │    │             │    │             │
│ sensor_msgs │    │ sensor_msgs │    │ sensor_msgs │
│    /Image   │    │   /LaserScan│    │    /Imu     │
└─────┬───────┘    └─────┬───────┘    └─────┬───────┘
      │                  │                  │
      │                  │                  │
      └──────────────────┼──────────────────┘
                         │
              ┌──────────▼──────────┐
              │   Sensor Integrator │
              │      Node           │
              │ (processes sensor   │
              │  data and fuses     │
              │  information)       │
              └─────────────────────┘
                         │
              ┌──────────▼──────────┐
              │   tf2 Transform     │
              │    Management       │
              └─────────────────────┘
                         │
              ┌──────────▼──────────┐
              │  Processed Sensor   │
              │     Data (e.g.,     │
              │   PointCloud2)      │
              └─────────────────────┘
```

## Case Study
In humanoid robotics, sensor integration is critical for balance and navigation. For example, the ATLAS robot uses IMU data for balance control, LiDAR for environment mapping, and cameras for object recognition. The sensor data is fused to create a comprehensive understanding of the robot's state and environment. The IMU provides real-time feedback for the balance controller, while the LiDAR and cameras help the robot navigate and interact with objects in its environment.

## References
- [sensor_msgs Package](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [image_transport Package](https://docs.ros.org/en/humble/p/image_transport/)
- [tf2 Documentation](https://docs.ros.org/en/humble/p/tf2/)
- [Camera Calibration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulated-Sensors.html)

## Review Questions
1. What are the main sensor_msgs types used for camera, LiDAR, and IMU data?
2. How does tf2 help in sensor integration for humanoid robots?
3. Explain the difference between LaserScan and PointCloud2 message types.

## Practical Exercises
1. Create a node that subscribes to camera image data and performs basic processing
2. Implement a simple obstacle detection algorithm using LiDAR data
3. Create a node that processes IMU data and calculates orientation
4. Integrate multiple sensors in a single launch file and visualize the data