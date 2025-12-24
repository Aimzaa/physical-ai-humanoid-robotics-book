---
id: chapter-1-4-python-rclpy-programming
title: Python rclpy Programming
sidebar_label: Python rclpy Programming
---

# Python rclpy Programming

## Goal
Master the rclpy library for creating ROS 2 nodes in Python, focusing on best practices for humanoid robot applications.

## Learning Objectives
- Create ROS 2 nodes using rclpy library
- Implement publishers, subscribers, services, and actions in Python
- Understand the ROS 2 execution model and event handling
- Apply Python-specific patterns for efficient ROS 2 programming
- Debug and test rclpy nodes effectively

## Overview
rclpy is the Python client library for ROS 2, providing a Python API for creating ROS 2 nodes and communicating with other nodes. For humanoid robotics applications, Python is often used for high-level control, perception processing, and AI algorithms due to its rich ecosystem of libraries. Understanding rclpy is essential for implementing robot behaviors and integrating with other ROS 2 components.

## Key Concepts
- **rclpy**: The Python client library for ROS 2
- **Node**: The basic execution unit in ROS 2
- **Executor**: Manages the execution of nodes and callbacks
- **QoS (Quality of Service)**: Policies for message delivery and reliability
- **Timers**: Mechanism for executing code at regular intervals
- **Callbacks**: Functions executed when messages are received or services are called

## Step-by-Step Breakdown
1. **Setting up rclpy**
   - Import rclpy and initialize
   - Create a node class inheriting from rclpy.node.Node
   - Implement proper cleanup with destroy_node()

2. **Creating Publishers**
   - Use create_publisher() to create message publishers
   - Define appropriate QoS profiles
   - Publish messages at desired rates

3. **Creating Subscribers**
   - Use create_subscription() to receive messages
   - Handle different message types
   - Implement appropriate QoS settings

4. **Implementing Services**
   - Create service servers with create_service()
   - Define service request/response types
   - Handle service calls in callbacks

5. **Working with Actions**
   - Implement action clients and servers
   - Handle goals, feedback, and results
   - Manage long-running operations

6. **Execution Management**
   - Use SingleThreadedExecutor or MultiThreadedExecutor
   - Spin nodes to process callbacks
   - Handle async operations appropriately

## Code Examples
```python
# Complete example of a humanoid control node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Create a publisher for joint commands
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', qos_profile)

        # Create a subscriber for sensor data
        self.sensor_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create a timer for periodic control updates
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop

        # Create an action client for trajectory execution
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory'
        )

        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Callback for processing joint state messages"""
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')
        # Process sensor data for control algorithms

    def control_loop(self):
        """Main control loop executed at 20Hz"""
        # Implement control algorithm here
        cmd_msg = JointState()
        cmd_msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        cmd_msg.position = [0.1, 0.2, 0.3]  # Example positions

        self.joint_pub.publish(cmd_msg)

    def send_trajectory_goal(self, trajectory):
        """Send a trajectory goal to the action server"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.trajectory_client.wait_for_server()
        self.trajectory_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Diagrams
```
rclpy Execution Model:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Publisher     │    │   Subscriber    │    │   Service/Action│
│   Callbacks     │    │   Callbacks     │    │   Callbacks     │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │     Executor            │
                    │  (Single/Multi-threaded)│
                    └─────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │   Node Event Loop       │
                    │   (rclpy.spin())        │
                    └─────────────────────────┘
```

## Case Study
In humanoid robotics, rclpy nodes are often used for high-level behaviors such as walking pattern generation, balance control, and perception processing. For example, a Python node might process camera images using OpenCV to detect obstacles, then publish this information to a navigation system. The asynchronous nature of rclpy allows the node to handle multiple sensor streams and respond to events in real-time.

## References
- [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

## Review Questions
1. What is the role of the executor in rclpy?
2. Explain the difference between SingleThreadedExecutor and MultiThreadedExecutor.
3. How do you properly handle cleanup when shutting down an rclpy node?

## Practical Exercises
1. Create a simple rclpy node that publishes a counter value every second
2. Create a subscriber node that receives and logs the counter values
3. Implement a service server that calculates the factorial of a number
4. Create a node with a timer that executes a control loop at 100Hz