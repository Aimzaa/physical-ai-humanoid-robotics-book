---
id: chapter-1-3-urdf-humanoid-robots
title: URDF for Humanoid Robots
sidebar_label: URDF for Humanoid Robots
---

# URDF for Humanoid Robots

## Goal
Learn to create and use URDF (Unified Robot Description Format) files to model humanoid robots in ROS 2, including kinematic chains, visual properties, and physical characteristics.

## Learning Objectives
- Understand the structure and elements of URDF files
- Create URDF models for humanoid robots with proper joint configurations
- Define visual and collision properties for robot links
- Understand the relationship between URDF and robot kinematics
- Learn to visualize URDF models in RViz

## Overview
URDF (Unified Robot Description Format) is an XML format used in ROS to describe robots. For humanoid robots, URDF files define the robot's physical structure, including links (rigid parts), joints (connections between links), and their properties. URDF is essential for simulation, visualization, motion planning, and control of humanoid robots.

## Key Concepts
- **Links**: Rigid parts of the robot (e.g., torso, limbs, head)
- **Joints**: Connections between links with defined degrees of freedom
- **Kinematic Chains**: Sequences of links connected by joints
- **Inertial Properties**: Mass, center of mass, and inertia tensor
- **Visual Properties**: How the robot appears in simulation/visualization
- **Collision Properties**: How the robot interacts with the environment

## Step-by-Step Breakdown
1. **URDF File Structure**
   - The root element is `<robot>`
   - Contains `<link>` and `<joint>` elements
   - Links represent rigid bodies
   - Joints define connections and degrees of freedom

2. **Defining Links**
   - Each link has a unique name
   - Visual elements define appearance
   - Collision elements define interaction with environment
   - Inertial elements define physical properties

3. **Defining Joints**
   - Joints connect two links (parent and child)
   - Joint types: fixed, revolute, continuous, prismatic, floating, planar
   - Joint limits and safety controllers can be specified

4. **Humanoid-Specific Considerations**
   - Multiple kinematic chains (arms, legs, spine, head)
   - Balance and center of mass considerations
   - Actuator limitations and joint constraints

5. **URDF Validation and Visualization**
   - Use tools like check_urdf to validate URDF files
   - Visualize in RViz or simulation environments

## Code Examples
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Hip -->
  <link name="left_hip">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_hip"/>
    <origin xyz="-0.1 -0.1 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Diagrams
```
Humanoid Robot Kinematic Structure:

        head
          |
        neck (revolute joint)
          |
        torso
       /     \
  left_hip  right_hip
      |        |
   left_leg  right_leg
      |        |
   left_foot right_foot
```

## Case Study
In humanoid robotics, URDF models are crucial for sim-to-real transfer. A robot's URDF must accurately represent its physical dimensions, joint limits, and mass distribution to ensure that controllers developed in simulation will work effectively on the real robot. For example, the Atlas robot's URDF includes detailed descriptions of its hydraulic actuators, which are critical for accurate simulation of its dynamic capabilities.

## References
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [xacro for URDF preprocessing](http://wiki.ros.org/xacro)

## Review Questions
1. What are the main components of a URDF file?
2. Explain the difference between visual and collision properties in URDF.
3. Why is it important to accurately model inertial properties in humanoid robots?

## Practical Exercises
1. Create a simple URDF file for a 2-link robotic arm
2. Use check_urdf to validate your URDF file
3. Visualize your robot model in RViz
4. Create a URDF for a simple humanoid with at least 6 degrees of freedom