---
id: chapter-1-2-nodes-topics-services
title: Nodes, Topics, Services, and Actions
sidebar_label: Nodes, Topics, Services, and Actions
---

# Nodes, Topics, Services, and Actions

## Goal
Understand the fundamental communication patterns in ROS 2: nodes, topics, services, and actions, and how they apply to humanoid robotics systems.

## Learning Objectives
- Define and create ROS 2 nodes for humanoid robot applications
- Implement publisher-subscriber communication using topics
- Create request-response communication using services
- Understand when to use actions for long-running tasks
- Compare communication patterns and select the appropriate one for different humanoid robot scenarios

## Overview
ROS 2 provides four main communication patterns that enable nodes to interact with each other: topics (publish/subscribe), services (request/response), actions (goal/cancel/result feedback), and parameters (configuration). These patterns are essential for building complex humanoid robot systems where multiple components need to coordinate their behavior.

## Key Concepts
- **Nodes**: Processes that perform computation and are the basic building blocks of ROS 2 programs
- **Topics**: Named buses over which nodes exchange messages in a publish/subscribe pattern
- **Services**: Synchronous request/response communication pattern
- **Actions**: Asynchronous communication pattern for long-running tasks with feedback
- **Messages**: Data structures that are passed between nodes

## Step-by-Step Breakdown
1. **Creating Nodes**
   - A node is an executable that uses ROS 2 to communicate with other nodes
   - Each node runs in its own process and can communicate with other nodes
   - Nodes are organized into packages for better modularity

2. **Topics and Publishers/Subscribers**
   - Topics enable asynchronous communication between nodes
   - Publishers send messages to topics
   - Subscribers receive messages from topics
   - Multiple publishers and subscribers can use the same topic

3. **Services**
   - Services provide synchronous request/response communication
   - A service client sends a request and waits for a response
   - A service server processes the request and sends back a response

4. **Actions**
   - Actions are designed for long-running tasks
   - They provide goal, feedback, and result mechanisms
   - Actions can be preempted or canceled

5. **Message Types**
   - Messages define the data structure for communication
   - Common types include std_msgs, sensor_msgs, geometry_msgs
   - Custom message types can be defined for specific applications

## Code Examples
```python
# Example ROS 2 Node with Publisher and Subscriber
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin_once(minimal_publisher)
    rclpy.spin_once(minimal_subscriber)

    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Diagrams
```
Topic Communication (Publisher-Subscriber):

┌─────────────┐              ┌─────────────┐
│ Publisher   │              │ Subscriber  │
│             │ publish(msg) │             │
│ ┌─────────┐ │ ────────────▶│ ┌─────────┐ │
│ │ Topic   │ │              │ │ Topic   │ │
│ │ (bus)   │ │ ◀────────────│ │ (bus)   │ │
│ └─────────┘ │    msg       │ └─────────┘ │
└─────────────┘              └─────────────┘

Service Communication (Request-Response):

┌─────────────┐              ┌─────────────┐
│ Client      │              │ Server      │
│             │ req          │             │
│ ┌─────────┐ │ ────────────▶│ ┌─────────┐ │
│ │ Service │ │              │ │ Service │ │
│ │ Request │ │ ◀────────────│ │ Response│ │
│ └─────────┘ │    resp      │ └─────────┘ │
└─────────────┘              └─────────────┘
```

## Case Study
In a humanoid robot, the head node might publish camera images to a topic that multiple other nodes subscribe to: the perception node for object recognition, the localization node for visual odometry, and the UI node for displaying the camera feed. Meanwhile, the planning node might use a service to request path planning from a navigation server, and the walking controller might use an action to execute a walking gait with continuous feedback about the robot's balance.

## References
- [ROS 2 Tutorials - Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes.html)
- [ROS 2 Tutorials - Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics.html)
- [ROS 2 Tutorials - Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services.html)

## Review Questions
1. What is the main difference between topics and services in ROS 2?
2. When should you use actions instead of services?
3. Explain the publish-subscribe pattern and give an example of its use in humanoid robotics.

## Practical Exercises
1. Create a simple publisher node that publishes counter values
2. Create a subscriber node that receives and logs the counter values
3. Create a service server that responds to simple math operations
4. Create a service client that calls the math service