---
id: chapter-2-4-sensor-simulation
title: Sensor Simulation
sidebar_label: Sensor Simulation
---

# Sensor Simulation

## Goal
Implement realistic sensor simulation for humanoid robots in Gazebo and Unity, including cameras, LiDAR, IMU, and force/torque sensors.

## Learning Objectives
- Configure and simulate various sensor types in Gazebo
- Implement sensor plugins for custom sensor behaviors
- Understand the differences between simulated and real sensor data
- Create realistic sensor noise models and imperfections
- Integrate simulated sensors with ROS 2 message formats
- Validate sensor simulation accuracy for sim-to-real transfer

## Overview
Sensor simulation is critical for humanoid robot development, as robots rely on sensors to perceive their environment and maintain balance. Simulated sensors must provide realistic data that closely matches real sensors to enable effective sim-to-real transfer of algorithms and controllers. This includes modeling sensor noise, latency, and other imperfections that affect real-world performance.

## Key Concepts
- **Sensor Plugins**: Custom code that generates sensor data in simulation
- **Noise Models**: Adding realistic noise to simulated sensor data
- **Sensor Parameters**: Configuring resolution, range, and accuracy
- **Gazebo Sensors**: Camera, LiDAR, IMU, force/torque, contact sensors
- **Unity Sensors**: Custom sensor implementations in Unity environment
- **Sensor Fusion**: Combining data from multiple simulated sensors
- **Calibration**: Ensuring simulated sensors match real sensor characteristics

## Step-by-Step Breakdown
1. **Camera Simulation**
   - Configure RGB, depth, and stereo cameras
   - Set resolution, field of view, and frame rate
   - Add noise models and distortion parameters
   - Implement image processing pipelines

2. **LiDAR Simulation**
   - Configure 2D and 3D LiDAR sensors
   - Set range, resolution, and scan patterns
   - Model measurement noise and dropouts
   - Implement custom LiDAR plugins if needed

3. **IMU Simulation**
   - Configure accelerometer and gyroscope data
   - Model bias, drift, and noise characteristics
   - Account for mounting position and orientation
   - Simulate magnetometer data if present

4. **Force/Torque Sensors**
   - Configure joint force/torque sensors
   - Model sensor resolution and noise
   - Implement contact force sensors
   - Account for sensor placement in robot

5. **Contact Sensors**
   - Implement foot contact sensors for humanoid walking
   - Configure collision detection and response
   - Model contact state and force information
   - Integrate with balance control systems

6. **Sensor Integration**
   - Publish sensor data to appropriate ROS topics
   - Ensure proper timing and synchronization
   - Implement sensor calibration procedures
   - Validate sensor simulation accuracy

## Code Examples
```xml
<!-- Example Gazebo SDF with various sensors for a humanoid robot -->
<sdf version="1.7">
  <model name="humanoid_with_sensors">
    <!-- Torso link with IMU -->
    <link name="torso">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.3</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.4</iyy>
          <iyz>0.0</iyz>
          <izz>0.2</izz>
        </inertia>
      </inertial>

      <!-- IMU sensor -->
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <pose>0 0 0 0 0 0</pose>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
          <ros>
            <namespace>/imu</namespace>
            <remapping>~/out:=data</remapping>
          </ros>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
          <gaussian_noise>0.01</gaussian_noise>
        </plugin>
      </sensor>

      <!-- RGB-D Camera -->
      <sensor name="camera" type="depth">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <camera name="camera">
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/camera</namespace>
            <remapping>image_raw:=image_color</remapping>
            <remapping>camera_info:=camera_info</remapping>
          </ros>
          <camera_name>camera</camera_name>
          <image_topic_name>image_raw</image_topic_name>
          <camera_info_topic_name>camera_info</camera_info_topic_name>
          <frame_name>camera_frame</frame_name>
          <hack_baseline>0.07</hack_baseline>
          <distortion_k1>0.0</distortion_k1>
          <distortion_k2>0.0</distortion_k2>
          <distortion_k3>0.0</distortion_k3>
          <distortion_t1>0.0</distortion_t1>
          <distortion_t2>0.0</distortion_t2>
        </plugin>
      </sensor>
    </link>

    <!-- Left foot with force/torque sensor -->
    <link name="left_foot">
      <pose>0 0 -0.1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>

      <!-- Force/Torque sensor -->
      <sensor name="left_foot_ft" type="force_torque">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <pose>0 0 0 0 0 0</pose>
        <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
          <ros>
            <namespace>/ft_sensor</namespace>
            <remapping>~/wrench:=left_foot_wrench</remapping>
          </ros>
          <frame_name>left_foot_frame</frame_name>
          <topic>left_foot_wrench</topic>
        </plugin>
      </sensor>

      <!-- Contact sensor for ground detection -->
      <sensor name="left_foot_contact" type="contact">
        <always_on>true</always_on>
        <update_rate>1000</update_rate>
        <contact>
          <collision>left_foot_collision</collision>
        </contact>
        <plugin name="contact_plugin" filename="libgazebo_ros_bumper.so">
          <ros>
            <namespace>/contact_sensor</namespace>
            <remapping>~/bumper_states:=contact_states</remapping>
          </ros>
          <frame_name>left_foot_frame</frame_name>
        </plugin>
      </sensor>

      <collision name="left_foot_collision">
        <geometry>
          <box>
            <size>0.2 0.1 0.05</size>
          </box>
        </geometry>
      </collision>

      <visual name="left_foot_visual">
        <geometry>
          <box>
            <size>0.2 0.1 0.05</size>
          </box>
        </geometry>
      </visual>
    </link>

    <!-- LiDAR sensor on head -->
    <link name="head">
      <pose>0 0 0.2 0 0 0</pose>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.02</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.02</iyy>
          <iyz>0.0</iyz>
          <izz>0.02</izz>
        </inertia>
      </inertial>

      <!-- 2D LiDAR -->
      <sensor name="lidar_2d" type="ray">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>720</samples>
              <resolution>1</resolution>
              <min_angle>-1.570796</min_angle>
              <max_angle>1.570796</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/lidar</namespace>
            <remapping>~/out:=scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
```

## Diagrams
```
Sensor Simulation Architecture:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Physical       │    │  Simulation     │    │  ROS Interface  │
│  World          │    │  World          │    │                 │
│  (Objects,      │    │  (Models,       │    │  (Message      │
│   Environment)   │    │   Physics)      │    │   Publishing)   │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          │              ┌───────▼────────┐             │
          │              │  Sensor        │             │
          │              │  Simulation     │             │
          │              │  (Physics-based │             │
          │              │   measurements) │             │
          │              └───────┬────────┘             │
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │   Noise & Imperfection  │
                    │   Modeling              │
                    └─────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │    ROS Message          │
                    │    Generation           │
                    └─────────────────────────┘

Simulated Sensor Data Pipeline:

Raw Physical Property → Sensor Model → Noise Addition → ROS Message → Robot Algorithm
      (e.g., distance)    (e.g., LiDAR)    (Gaussian, bias)   (LaserScan)   (Obstacle detection)
```

## Case Study
The NAO humanoid robot simulation includes comprehensive sensor simulation for development and testing. The simulation includes cameras for vision processing, IMUs for balance control, force/torque sensors in the feet for contact detection, and encoders for joint position feedback. Researchers have successfully used this simulation to develop walking controllers that transfer to the real robot with minimal retuning, thanks to the realistic sensor simulation that includes appropriate noise models and delays.

## References
- [Gazebo Sensors Documentation](http://gazebosim.org/tutorials/?tut=ros_gzplugins#Sensor-plugins)
- [ROS Sensor Message Types](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [Gazebo Sensor Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins#Sensor-plugins)

## Review Questions
1. What are the key differences between simulated and real sensor data?
2. How do noise models improve the realism of sensor simulation?
3. Why is sensor calibration important for sim-to-real transfer?

## Practical Exercises
1. Configure a camera sensor in Gazebo and visualize the output
2. Add noise to a simulated IMU and observe the effects
3. Implement a simple LiDAR sensor plugin with custom characteristics
4. Create a sensor fusion node that combines multiple simulated sensors