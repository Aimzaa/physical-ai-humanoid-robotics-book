---
id: chapter-1-1-introduction-ros2
title: Introduction to ROS 2 Architecture
sidebar_label: Introduction to ROS 2 Architecture
---

# Introduction to ROS 2 Architecture

## Goal
Understand the fundamental architecture of ROS 2 and its components, with a focus on how it applies to humanoid robotics.

## Learning Objectives
- Understand the core concepts of ROS 2 and its differences from ROS 1
- Identify the main components of the ROS 2 architecture
- Learn about the DDS (Data Distribution Service) implementation in ROS 2
- Recognize how ROS 2 architecture benefits humanoid robotics applications

## Overview
ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software that provides a collection of libraries and tools to help create robot applications. Unlike ROS 1, ROS 2 is built on DDS (Data Distribution Service), which provides a middleware that enables scalable, real-time, dependable, distributed applications. This architecture is particularly beneficial for humanoid robots, which require complex coordination between multiple sensors, actuators, and processing units.

## Key Concepts
- **DDS (Data Distribution Service)**: The middleware that underlies ROS 2, providing a standardized API for machine-to-machine communication
- **Nodes**: Processes that perform computation; in ROS 2, nodes are the fundamental building blocks
- **Packages**: Collections of related resources that are organized and distributed together
- **Workspaces**: Directories where you modify and build ROS 2 packages
- **RMW (ROS Middleware)**: The layer that allows ROS 2 to use different DDS implementations

## Step-by-Step Breakdown
1. **Understanding the ROS 2 Architecture**
   - ROS 2 uses a distributed architecture based on DDS
   - Nodes communicate through topics, services, and actions
   - Each node runs in its own process and can communicate with other nodes

2. **DDS Implementation**
   - DDS provides a standardized way for applications to communicate
   - Multiple DDS implementations are available (Fast DDS, Cyclone DDS, RTI Connext DDS)
   - The choice of DDS implementation can be changed via RMW layer

3. **Setting up a ROS 2 Environment**
   - Install a ROS 2 distribution (e.g., Humble Hawksbill, Iron Irwini)
   - Source the ROS 2 setup script
   - Verify the installation

4. **ROS 2 vs ROS 1 Differences**
   - ROS 2 is designed for production environments
   - Better security features
   - Real-time capabilities
   - Multi-robot systems support

## Code Examples
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check ROS 2 version
ros2 --version

# List available DDS implementations
printenv | grep RMW
```

## Diagrams
```
ROS 2 Architecture Overview:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Node A        │    │   Node B        │    │   Node C        │
│                 │    │                 │    │                 │
│ Publisher       │    │ Subscriber      │    │ Service Server  │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │     DDS Middleware      │
                    │   (Fast DDS/Cyclone DDS)│
                    └─────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │    Network Transport    │
                    │   (TCP/UDP, Shared Mem) │
                    └─────────────────────────┘
```

## Case Study
In humanoid robotics, ROS 2's architecture allows for distributed processing where different nodes can run on different computational units. For example, perception nodes might run on a GPU-intensive computer, while control nodes run on real-time microcontrollers. The DDS-based communication ensures reliable and efficient data exchange between these distributed components.

## References
- [ROS 2 Documentation](https://docs.ros.org/)
- [DDS Standard Overview](https://www.dds-foundation.org/)
- [ROS 2 Design Goals](https://design.ros2.org/)

## Review Questions
1. What is the main difference between ROS 1 and ROS 2 in terms of underlying architecture?
2. Explain the role of DDS in ROS 2.
3. Why is the distributed architecture of ROS 2 particularly beneficial for humanoid robots?

## Practical Exercises
1. Install a ROS 2 distribution on your development machine
2. Verify the installation by running `ros2 --version`
3. List all available ROS 2 commands using `ros2 --help`