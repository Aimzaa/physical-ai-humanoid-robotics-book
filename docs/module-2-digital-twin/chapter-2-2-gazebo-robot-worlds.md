---
id: chapter-2-2-gazebo-robot-worlds
title: Gazebo Robot Worlds
sidebar_label: Gazebo Robot Worlds
---

# Gazebo Robot Worlds

## Goal
Create and configure realistic Gazebo worlds for humanoid robot testing, including environment modeling, lighting, and physics properties.

## Learning Objectives
- Design and implement Gazebo world files with appropriate environments
- Configure lighting, textures, and visual properties for realistic simulation
- Create obstacle courses and testing environments for humanoid robots
- Understand the structure of SDF world files
- Implement dynamic elements and interactive objects in simulation

## Overview
Gazebo worlds provide the virtual environments where humanoid robots operate and are tested. These worlds must accurately represent real-world physics, lighting conditions, and spatial relationships to enable effective sim-to-real transfer. Creating realistic worlds is crucial for humanoid robot development, as these robots need to navigate complex environments and interact with various objects.

## Key Concepts
- **SDF (Simulation Description Format)**: XML-based format for describing simulation environments
- **World Files**: SDF files that define complete simulation environments
- **Ground Planes**: Infinite or finite surfaces for robot locomotion
- **Models**: 3D objects that populate the world (furniture, obstacles, etc.)
- **Plugins**: Custom code that adds functionality to the simulation
- **Lighting**: Environmental and directional lighting for realistic rendering
- **Physics Properties**: World-level physics parameters affecting all objects

## Step-by-Step Breakdown
1. **World File Structure**
   - Define the SDF version and world root element
   - Configure global physics parameters
   - Set up lighting and environment properties
   - Add models and static objects

2. **Physics Configuration**
   - Set gravity parameters
   - Configure solver parameters (type, iterations, step size)
   - Define real-time update rate and max step size

3. **Environment Creation**
   - Create ground planes with appropriate friction
   - Add static models (walls, furniture, obstacles)
   - Configure terrain properties for outdoor environments

4. **Lighting Setup**
   - Add directional lights for sun-like illumination
   - Configure ambient lighting
   - Set up point lights for indoor environments

5. **Robot Placement**
   - Position the robot model in the world
   - Set initial pose and state
   - Configure spawn parameters

6. **Dynamic Elements**
   - Add moving objects or platforms
   - Implement interactive elements
   - Create objects with custom physics properties

## Code Examples
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_indoor_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Indoor environment with obstacles -->
    <model name="wall_1">
      <pose>-5 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="table">
      <pose>2 2 0.4 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.6 0.8</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.5</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.6 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Humanoid robot spawn location -->
    <include>
      <uri>model://simple_humanoid</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>

    <!-- Spherical obstacle for navigation testing -->
    <model name="obstacle_sphere">
      <pose>-3 -3 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
          <surface>
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
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>1.0 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Diagrams
```
Gazebo World Structure:

┌─────────────────────────────────────────────────────────┐
│                    World Environment                    │
│                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐ │
│  │   Ground    │  │   Wall 1    │  │    Table        │ │
│  │   Plane     │  │             │  │                 │ │
│  └─────────────┘  └─────────────┘  └─────────────────┘ │
│                                                         │
│         ┌─────────────────────────────────────────┐     │
│         │        Humanoid Robot Model           │     │
│         │      (spawned at specific pose)       │     │
│         └─────────────────────────────────────────┘     │
│                                                         │
│  ┌─────────────────┐                                  │
│  │  Spherical      │                                  │
│  │  Obstacle       │                                  │
│  └─────────────────┘                                  │
│                                                         │
│  Lighting: Sun (directional), Ambient                    │
│  Physics: Gravity, Solver parameters                     │
└─────────────────────────────────────────────────────────┘

World Loading Process:

┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ World File  │───▶│ Gazebo      │───▶│ Simulation  │
│ (.world)    │    │ Simulator   │    │ Running     │
└─────────────┘    └─────────────┘    └─────────────┘