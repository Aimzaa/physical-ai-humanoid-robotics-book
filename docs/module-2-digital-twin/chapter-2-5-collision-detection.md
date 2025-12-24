---
id: chapter-2-5-collision-detection
title: Collision Detection and Environment Modeling
sidebar_label: Collision Detection
---

# Collision Detection and Environment Modeling

## Goal
Implement accurate collision detection and environment modeling for humanoid robots in simulation, ensuring realistic interaction with the environment and self-collision avoidance.

## Learning Objectives
- Understand collision detection algorithms used in robotics simulation
- Configure collision properties for humanoid robot models
- Implement environment modeling for complex scenarios
- Set up self-collision detection and avoidance
- Understand the impact of collision detection on simulation performance
- Validate collision behavior for sim-to-real transfer

## Overview
Collision detection is fundamental to humanoid robot simulation, enabling realistic interaction with the environment and preventing self-intersection. For humanoid robots with complex kinematic structures and many degrees of freedom, proper collision detection is essential for stable simulation and accurate behavior prediction. This includes modeling the robot's collision geometry, environment obstacles, and implementing efficient detection algorithms.

## Key Concepts
- **Collision Geometry**: Simplified shapes used for collision detection (boxes, spheres, meshes)
- **Broad Phase Detection**: Fast algorithms to identify potentially colliding pairs
- **Narrow Phase Detection**: Precise algorithms to determine actual collisions
- **Contact Manifolds**: Information about collision points and forces
- **Self-Collision**: Collision detection between different parts of the same robot
- **Environment Collision**: Collision detection with world objects
- **Collision Response**: How the physics engine handles collisions

## Step-by-Step Breakdown
1. **Collision Geometry Design**
   - Choose appropriate collision shapes for each link
   - Balance accuracy vs performance with simplified geometries
   - Consider different geometries for different purposes
   - Optimize collision meshes for humanoid-specific interactions

2. **Broad Phase Algorithms**
   - Understand spatial partitioning methods (octrees, bounding volume hierarchies)
   - Configure collision detection parameters
   - Understand the trade-offs between different broad phase methods
   - Optimize for humanoid robot's specific kinematic structure

3. **Narrow Phase Implementation**
   - Understand GJK, EPA, and other collision detection algorithms
   - Configure collision response parameters
   - Set up contact point generation
   - Handle special cases (mesh-mesh collisions)

4. **Self-Collision Configuration**
   - Define which robot parts can collide with each other
   - Set up collision masks to ignore certain pairs
   - Configure self-collision detection for safety
   - Optimize performance for many-joint robots

5. **Environment Modeling**
   - Create collision models for environment objects
   - Configure static vs dynamic collision objects
   - Set up complex environments with multiple obstacles
   - Implement level-of-detail for performance

6. **Performance Optimization**
   - Balance accuracy and performance requirements
   - Configure collision detection frequency
   - Use appropriate collision shapes for performance
   - Understand the impact of collision complexity on simulation speed

## Code Examples
```xml
<!-- Example SDF with collision detection configuration for humanoid robot -->
<sdf version="1.7">
  <model name="humanoid_collision_model">
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

      <!-- Visual geometry -->
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>

      <!-- Collision geometry (simplified for performance) -->
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0.001</soft_cfm>
              <soft_erp>0.8</soft_erp>
              <kp>1e+5</kp>
              <kd>1.0</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>0.5</mu>
              <mu2>0.5</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.01</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
        </surface>
      </collision>
    </link>

    <link name="left_arm_upper">
      <pose>0.2 0.15 -0.1 0 0 0</pose>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.05</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.05</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0.001</soft_cfm>
              <soft_erp>0.8</soft_erp>
              <kp>1e+5</kp>
              <kd>1.0</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <link name="left_arm_lower">
      <pose>0.2 0.15 -0.4 0 0 0</pose>
      <inertial>
        <mass>1.5</mass>
        <inertia>
          <ixx>0.03</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.03</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.25</length>
          </cylinder>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.25</length>
          </cylinder>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0.001</soft_cfm>
              <soft_erp>0.8</soft_erp>
              <kp>1e+5</kp>
              <kd>1.0</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <!-- Joint with collision exclusion -->
    <joint name="left_shoulder" type="revolute">
      <parent>torso</parent>
      <child>left_arm_upper</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>50</effort>
          <velocity>2</velocity>
        </limit>
      </axis>
    </joint>

    <joint name="left_elbow" type="revolute">
      <parent>left_arm_upper</parent>
      <child>left_arm_lower</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>0</lower>
          <upper>2.0</upper>
          <effort>30</effort>
          <velocity>2</velocity>
        </limit>
      </axis>
    </joint>

    <!-- Self-collision exclusion (for adjacent links) -->
    <joint name="left_shoulder_collision_exclude" type="fixed">
      <parent>left_arm_upper</parent>
      <child>torso</child>
      <physics>
        <ode>
          <provide_feedback>false</provide_feedback>
        </ode>
      </physics>
    </joint>
  </model>

  <!-- Environment with complex collision geometry -->
  <model name="obstacle_environment">
    <static>true</static>

    <!-- Complex obstacle with detailed collision mesh -->
    <link name="complex_obstacle">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://environment/meshes/complex_obstacle.dae</uri>
            <scale>1.0 1.0 1.0</scale>
          </mesh>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0.001</soft_cfm>
              <soft_erp>0.8</soft_erp>
              <kp>1e+6</kp>
              <kd>10.0</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://environment/meshes/complex_obstacle.dae</uri>
            <scale>1.0 1.0 1.0</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

## Diagrams
```
Collision Detection Hierarchy:

┌─────────────────────────────────────────────────────────┐
│                   Broad Phase                           │
│  (Quick elimination of non-colliding pairs)            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │   AABB Tree │  │   Octree    │  │   Grid      │    │
│  │   (Boxes)   │  │   Spatial   │  │   Spatial   │    │
│  └─────────────┘  └─────────────┘  └─────────────┘    │
└─────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────┐
│                   Narrow Phase                          │
│  (Precise collision detection)                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │   GJK       │  │   EPA       │  │   SAT       │    │
│  │  (Convex)   │  │  (Penetration│  │  (Separating│    │
│  └─────────────┘  │   Depth)    │  │   Axes)     │    │
│                   └─────────────┘  └─────────────┘    │
└─────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────┐
│                Contact Generation                       │
│  (Collision points, normals, forces)                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │  Contact    │  │  Friction   │  │  Response   │    │
│  │  Points     │  │  Model      │  │  Model      │    │
│  └─────────────┘  └─────────────┘  └─────────────┘    │
└─────────────────────────────────────────────────────────┘

Self-Collision Management:

Humanoid Robot Structure:
  Torso
  ├── Head
  ├── Left Arm
  │   ├── Upper Arm
  │   └── Lower Arm
  ├── Right Arm
  │   ├── Upper Arm
  │   └── Lower Arm
  ├── Left Leg
  │   ├── Upper Leg
  │   ├── Lower Leg
  │   └── Foot
  └── Right Leg
      ├── Upper Leg
      ├── Lower Leg
      └── Foot

Collision Pairs to Consider:
  ✓ Torso ↔ Arms (when arms cross)
  ✓ Arms ↔ Legs (during movement)
  ✓ Feet ↔ Ground
  ✗ Adjacent links (elbow joint parts) - typically excluded
  ✓ End effectors ↔ Environment

Performance vs Accuracy Trade-off:

High Accuracy ──────────────────────────────────────● Real Robot
              ╱                                     ╱ │
            ╱                                     ╱   │
          ╱                                     ╱     │
        ╱                                     ╱       │
      ╱                                     ╱         │
    ╱                                     ╱           │
  ╱                                     ╱             │
Low Accuracy ──────────────────────────●               │
                Low Performance    High Performance
```

## Case Study
The Atlas humanoid robot simulation includes sophisticated collision detection to handle its complex 28-DOF structure. The simulation uses simplified collision geometries for performance while maintaining accuracy in critical areas like the feet for balance control. Researchers have found that proper collision detection is essential for stable walking simulation, as incorrect collision behavior can lead to unrealistic gait patterns that don't transfer to the real robot.

## References
- [Gazebo Collision Detection Documentation](http://gazebosim.org/tutorials?tut=collision&cat=physics)
- [Bullet Physics Collision Detection](https://pybullet.org/wordpress/)
- [ODE Collision Detection](https://www.ode.org/)

## Review Questions
1. What is the difference between broad phase and narrow phase collision detection?
2. Why is self-collision detection important for humanoid robots?
3. How do collision geometry simplifications affect simulation performance?

## Practical Exercises
1. Create a simple humanoid model with collision geometry
2. Implement a collision detection test between robot and environment
3. Configure self-collision exclusion for adjacent links
4. Compare simulation performance with different collision geometries