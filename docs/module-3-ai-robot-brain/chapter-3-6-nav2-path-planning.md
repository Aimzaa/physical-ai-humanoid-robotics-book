---
id: chapter-3-6-nav2-path-planning
title: Nav2 Path Planning and Navigation
sidebar_label: Nav2 Path Planning and Navigation
---

# Nav2 Path Planning and Navigation

## Goal
Implement advanced path planning and navigation systems for humanoid robots using the Navigation2 (Nav2) framework integrated with NVIDIA Isaac tools, enabling autonomous movement in complex environments.

## Learning Objectives
- Understand the Navigation2 (Nav2) architecture and components
- Configure Nav2 for humanoid robot navigation
- Implement global and local planners for humanoid kinematics
- Integrate Nav2 with Isaac perception systems
- Configure navigation behaviors for complex humanoid movements
- Validate navigation performance in simulation and real environments

## Overview
Navigation2 (Nav2) is the state-of-the-art navigation framework for ROS 2, designed to provide robust and flexible navigation capabilities for mobile robots. For humanoid robots, Nav2 offers specialized tools and configurations to handle the unique challenges of bipedal locomotion, complex kinematics, and human-like navigation patterns. When integrated with NVIDIA Isaac's perception and AI capabilities, Nav2 enables humanoid robots to navigate complex environments with advanced obstacle avoidance and path planning capabilities.

## Key Concepts
- **Navigation2 (Nav2)**: ROS 2 navigation framework
- **Global Planner**: Long-term path planning from start to goal
- **Local Planner**: Short-term obstacle avoidance and path following
- **Costmaps**: Representing environment traversability
- **Behavior Trees**: Composable navigation behaviors
- **Recovery Behaviors**: Handling navigation failures
- **Humanoid Kinematics**: Specialized navigation for bipedal robots

## Step-by-Step Breakdown
1. **Nav2 Architecture Understanding**
   - Learn about Nav2 components and their interactions
   - Understand the behavior tree-based navigation system
   - Configure Nav2 parameters for humanoid robots
   - Set up the navigation stack

2. **Costmap Configuration**
   - Configure global and local costmaps
   - Set up obstacle inflation and filtering
   - Configure voxel grid for 3D obstacle representation
   - Optimize costmap parameters for humanoid navigation

3. **Global Planner Implementation**
   - Configure A* or Dijkstra planners for humanoid kinematics
   - Set up potential field or other advanced planners
   - Configure path smoothing and optimization
   - Handle dynamic obstacles and replanning

4. **Local Planner Configuration**
   - Set up trajectory rollout planners (DWA, TEB)
   - Configure velocity and acceleration limits for humanoid
   - Implement obstacle avoidance strategies
   - Optimize for real-time performance

5. **Behavior Tree Customization**
   - Design behavior trees for humanoid navigation
   - Implement custom navigation actions
   - Configure recovery behaviors
   - Optimize decision-making processes

6. **Integration with Isaac**
   - Integrate Nav2 with Isaac perception systems
   - Connect to Isaac's VSLAM and mapping capabilities
   - Configure sensor fusion for navigation
   - Validate navigation performance

## Code Examples
```python
# Example Nav2 integration for humanoid robot navigation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
import tf_transformations
import numpy as np
import math
from typing import List, Tuple, Optional

class HumanoidNav2Interface(Node):
    def __init__(self):
        super().__init__('humanoid_nav2_interface')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Navigation parameters for humanoid robot
        self.nav_params = {
            'max_linear_vel': 0.4,  # Slower for stability
            'max_angular_vel': 0.3,
            'min_linear_vel': 0.1,
            'min_angular_vel': 0.05,
            'goal_tolerance': 0.3,  # Larger for humanoid stability
            'yaw_tolerance': 0.2,
            'footprint_radius': 0.3,  # Humanoid robot footprint
            'inflation_radius': 0.5,  # Extra safety for humanoid
            'robot_base_frame': 'base_link',
            'global_frame': 'map'
        }

        # Initialize navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/nav2/path', 10)
        self.velocity_pub = self.create_publisher(Twist, '/nav2/velocity', 10)

        # Navigation state
        self.current_goal = None
        self.is_navigating = False
        self.navigation_active = False

        # Robot state
        self.robot_pose = None
        self.robot_twist = None

        # Subscribers for robot state
        self.odom_sub = self.create_subscription(
            PoseStamped, '/odom', self.odom_callback, 10
        )

        # Timer for navigation monitoring
        self.nav_monitor_timer = self.create_timer(0.5, self.navigation_monitor)

        self.get_logger().info('Humanoid Nav2 Interface initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_pose = msg.pose

    def send_navigation_goal(self, x: float, y: float, theta: float = 0.0) -> bool:
        """Send navigation goal to Nav2"""
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.nav_params['global_frame']
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert angle to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Send goal
        self.current_goal = goal_msg
        self.is_navigating = True

        self.get_logger().info(f'Sending navigation goal: ({x}, {y}, {theta})')

        # Send async goal
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted')
        self.navigation_active = True

        # Get result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.navigation_active = False
        self.is_navigating = False

        if result:
            self.get_logger().info('Navigation completed successfully')
        else:
            self.get_logger().info('Navigation failed')

    def navigation_feedback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Process feedback as needed
        self.get_logger().debug(f'Navigation progress: {feedback.distance_remaining:.2f}m remaining')

    def navigation_monitor(self):
        """Monitor navigation state and provide humanoid-specific behaviors"""
        if not self.is_navigating:
            return

        # Check robot state and adjust behavior if needed
        if self.robot_pose:
            # Calculate distance to goal if available
            if self.current_goal:
                goal_pos = self.current_goal.pose.pose.position
                robot_pos = self.robot_pose.position

                distance = math.sqrt(
                    (goal_pos.x - robot_pos.x)**2 +
                    (goal_pos.y - robot_pos.y)**2
                )

                self.get_logger().debug(f'Distance to goal: {distance:.2f}m')

                # Adjust behavior based on distance or other factors
                if distance < self.nav_params['goal_tolerance']:
                    self.get_logger().info('Reached navigation goal')
                    self.is_navigating = False
                    self.navigation_active = False

    def stop_navigation(self):
        """Stop current navigation"""
        self.is_navigating = False
        self.navigation_active = False

        # Send stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().info('Navigation stopped')

class Nav2ConfigurationNode(Node):
    """Node to configure Nav2 parameters for humanoid robot"""

    def __init__(self):
        super().__init__('nav2_configurator')

        # Publisher for dynamic parameters
        self.param_client = self.create_client(
            SetParameters, '/controller_server/set_parameters'
        )

        # Configure Nav2 for humanoid robot
        self.configure_humanoid_nav2()

    def configure_humanoid_nav2(self):
        """Configure Nav2 parameters specifically for humanoid robot"""

        # Humanoid-specific parameters
        humanoid_params = {
            # Robot description for kinematic constraints
            'robot_radius': 0.3,  # meters
            'footprint_padding': 0.1,  # extra safety

            # Velocity limits for stable walking
            'max_linear_vel': 0.4,  # m/s (slower for stability)
            'min_linear_vel': 0.1,
            'max_angular_vel': 0.3,  # rad/s
            'min_angular_vel': 0.05,

            # Acceleration limits for humanoid dynamics
            'max_linear_accel': 0.2,  # m/s²
            'max_angular_accel': 0.2,  # rad/s²

            # Path planning parameters
            'planner_frequency': 5.0,  # Hz
            'controller_frequency': 20.0,  # Hz (higher for stability)

            # Costmap parameters
            'inflation_radius': 0.5,  # Larger for humanoid safety
            'cost_scaling_factor': 5.0,  # More aggressive cost scaling

            # Goal tolerance (larger for humanoid stability)
            'goal_check_tolerance': 0.3,  # meters
            'yaw_goal_tolerance': 0.2,   # radians
        }

        self.get_logger().info('Configured Nav2 for humanoid robot')
        self.apply_nav2_parameters(humanoid_params)

    def apply_nav2_parameters(self, params):
        """Apply parameters to Nav2 nodes"""
        # In a real implementation, this would use ROS 2 parameter services
        # to configure Nav2 components dynamically
        for param_name, param_value in params.items():
            self.get_logger().debug(f'Setting {param_name} = {param_value}')

class HumanoidLocalPlanner(Node):
    """Custom local planner optimized for humanoid robots"""

    def __init__(self):
        super().__init__('humanoid_local_planner')

        # Subscribers
        self.global_path_sub = self.create_subscription(
            Path, '/plan', self.global_path_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.odom_sub = self.create_subscription(
            PoseStamped, '/odom', self.odom_callback, 10
        )

        # Publishers
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)

        # Local planner parameters
        self.planner_params = {
            'local_window_size': 3.0,  # meters
            'max_vel_x': 0.4,  # m/s
            'min_vel_x': 0.05,
            'max_vel_theta': 0.3,  # rad/s
            'min_vel_theta': 0.02,
            'acc_lim_x': 0.2,  # m/s²
            'acc_lim_theta': 0.2,  # rad/s²
            'xy_goal_tolerance': 0.3,  # meters
            'yaw_goal_tolerance': 0.2,  # radians
        }

        # State variables
        self.global_path = []
        self.current_pose = None
        self.laser_data = None
        self.current_velocity = Twist()

        # Timer for local planning
        self.planning_timer = self.create_timer(0.05, self.local_planning_callback)  # 20 Hz

        self.get_logger().info('Humanoid Local Planner initialized')

    def global_path_callback(self, msg: Path):
        """Update global path from Nav2"""
        self.global_path = msg.poses

    def laser_callback(self, msg: LaserScan):
        """Update laser data for obstacle detection"""
        self.laser_data = msg

    def odom_callback(self, msg: PoseStamped):
        """Update robot pose"""
        self.current_pose = msg.pose

    def local_planning_callback(self):
        """Main local planning callback"""
        if not self.current_pose or not self.global_path:
            return

        # Get local path segment
        local_path = self.get_local_path_segment()

        # Check for obstacles
        if self.laser_data and self.check_for_obstacles():
            # Execute obstacle avoidance
            velocity_cmd = self.obstacle_avoidance_behavior()
        else:
            # Follow path normally
            velocity_cmd = self.follow_path(local_path)

        # Publish velocity command
        self.velocity_pub.publish(velocity_cmd)
        self.current_velocity = velocity_cmd

        # Publish local plan for visualization
        if local_path:
            local_path_msg = Path()
            local_path_msg.header.stamp = self.get_clock().now().to_msg()
            local_path_msg.header.frame_id = 'map'
            local_path_msg.poses = local_path
            self.local_plan_pub.publish(local_path_msg)

    def get_local_path_segment(self) -> List[PoseStamped]:
        """Get path segment within local window"""
        if not self.current_pose or not self.global_path:
            return []

        local_path = []
        current_pos = self.current_pose.position

        for pose_stamped in self.global_path:
            # Calculate distance to pose
            dx = pose_stamped.pose.position.x - current_pos.x
            dy = pose_stamped.pose.position.y - current_pos.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance <= self.planner_params['local_window_size']:
                local_path.append(pose_stamped)

        return local_path

    def check_for_obstacles(self) -> bool:
        """Check laser data for obstacles in path"""
        if not self.laser_data:
            return False

        # Check if there are obstacles within humanoid's path
        min_range = min(self.laser_data.ranges) if self.laser_data.ranges else float('inf')
        return min_range < 0.8  # threshold for humanoid safety

    def obstacle_avoidance_behavior(self) -> Twist:
        """Execute obstacle avoidance behavior"""
        cmd_vel = Twist()

        # Simple obstacle avoidance: turn away from obstacles
        if self.laser_data:
            # Find the direction with maximum clearance
            ranges = np.array(self.laser_data.ranges)
            valid_ranges = ranges[np.isfinite(ranges)]

            if len(valid_ranges) > 0:
                # Turn toward the clearest direction
                max_idx = np.argmax(ranges)
                angle_increment = self.laser_data.angle_increment
                angle_to_max = self.laser_data.angle_min + max_idx * angle_increment

                cmd_vel.angular.z = max(min(angle_to_max * 0.5,
                                          self.planner_params['max_vel_theta']),
                                      -self.planner_params['max_vel_theta'])

        return cmd_vel

    def follow_path(self, local_path: List[PoseStamped]) -> Twist:
        """Follow the local path segment"""
        cmd_vel = Twist()

        if not local_path:
            return cmd_vel

        # Get the next waypoint to follow
        target_pose = local_path[0].pose if local_path else None
        if not target_pose:
            return cmd_vel

        # Calculate direction to target
        current_pos = self.current_pose.position
        target_pos = target_pose.position

        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate desired heading
        desired_theta = math.atan2(dy, dx)

        # Get current orientation
        from tf_transformations import euler_from_quaternion
        current_orientation = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        current_euler = euler_from_quaternion(current_orientation)
        current_theta = current_euler[2]

        # Calculate heading error
        heading_error = desired_theta - current_theta
        # Normalize angle to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Set velocities
        if abs(heading_error) > 0.1:  # 0.1 rad tolerance
            cmd_vel.angular.z = max(min(heading_error * 1.0,
                                      self.planner_params['max_vel_theta']),
                                  -self.planner_params['max_vel_theta'])
        else:
            if distance > self.planner_params['xy_goal_tolerance']:
                cmd_vel.linear.x = max(min(distance * 0.5,
                                         self.planner_params['max_vel_x']),
                                     self.planner_params['min_vel_x'])

        return cmd_vel

def main(args=None):
    """Main function to run the Nav2 interface"""
    rclpy.init(args=args)

    # Create nodes
    nav_interface = HumanoidNav2Interface()
    nav_config = Nav2ConfigurationNode()
    local_planner = HumanoidLocalPlanner()

    try:
        # Example: Send a navigation goal after startup
        import time
        time.sleep(2)  # Wait for connections

        # Send a test goal
        nav_interface.send_navigation_goal(2.0, 2.0, 0.0)

        # Spin all nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(nav_interface)
        executor.add_node(nav_config)
        executor.add_node(local_planner)

        executor.spin()

    except KeyboardInterrupt:
        nav_interface.get_logger().info('Shutting down Nav2 interface')
    finally:
        nav_interface.destroy_node()
        nav_config.destroy_node()
        local_planner.destroy_node()
        rclpy.shutdown()

# Isaac-specific Nav2 integration
class IsaacNav2Integrator(Node):
    """Node to integrate Nav2 with Isaac perception systems"""

    def __init__(self):
        super().__init__('isaac_nav2_integrator')

        # Subscribers for Isaac perception data
        self.vslam_pose_sub = self.create_subscription(
            PoseStamped, '/vslam/pose', self.vslam_pose_callback, 10
        )
        self.semantic_map_sub = self.create_subscription(
            OccupancyGrid, '/semantic_map', self.semantic_map_callback, 10
        )

        # Publishers to Nav2
        self.nav_odom_pub = self.create_publisher(
            Odometry, '/nav2/odom', 10
        )
        self.nav_costmap_pub = self.create_publisher(
            OccupancyGrid, '/nav2/costmap', 10
        )

        self.get_logger().info('Isaac Nav2 Integrator initialized')

    def vslam_pose_callback(self, msg: PoseStamped):
        """Handle VSLAM pose updates"""
        # Convert VSLAM pose to Nav2-compatible odometry
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.pose.pose = msg.pose
        # Add velocity estimation if available

        self.nav_odom_pub.publish(odom_msg)

    def semantic_map_callback(self, msg: OccupancyGrid):
        """Handle semantic map updates from Isaac"""
        # Forward semantic map to Nav2 costmap
        self.nav_costmap_pub.publish(msg)

    def setup_isaac_nav2_bridge(self):
        """Setup the integration between Isaac and Nav2"""
        # This would configure the data flow between Isaac perception
        # systems and Nav2 navigation stack
        pass
```

## Diagrams
```
Nav2 Architecture for Humanoid Robots:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Nav2 Core      │    │  Perception     │    │  Humanoid       │
│  (Action Server│◄──►│  Integration    │◄──►│  Controller     │
│   Behavior      │    │  (Isaac VSLAM, │    │  (Walking, etc.)│
│   Tree)         │    │   Semantic Map) │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Global Planner │    │  Costmap        │    │  Local Planner  │
│  (A*, Dijkstra) │    │  (Global,      │    │  (DWA, TEB)     │
│                 │    │   Local)        │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │     Path Execution      │
                    │     (Velocity Control)  │
                    └─────────────────────────┘

Humanoid Navigation Workflow:

Start Goal → Global Planner → Local Planner → Velocity Control → Humanoid Robot
              (Path Planning)  (Obstacle Avoid)  (Stable Walking)   (Execution)

Nav2 Behavior Tree Structure:

root
├── compute_path_to_pose (Global Planner)
├── follow_path (Local Planner)
│   ├── smooth_path
│   ├── regulate_linear_velocity
│   ├── compute_velocity_commands
│   └── is_stuck
│       └── recovery_node
│           ├── spin
│           ├── backup
│           └── drive_on_heading
└── goal_reached
```

## Case Study
The integration of Nav2 with NVIDIA Isaac has enabled advanced navigation capabilities for humanoid robots in complex environments. In a demonstration by the NVIDIA Isaac team, a humanoid robot equipped with Isaac's perception stack and Nav2 navigation was able to autonomously navigate through a cluttered indoor environment, avoiding both static and dynamic obstacles while maintaining stable bipedal locomotion. The system leveraged Isaac's VSLAM for localization and semantic mapping, which was then fed into Nav2's costmap system to create detailed traversability maps. This integration allowed the humanoid robot to perform complex navigation tasks that would be challenging with traditional approaches.

## References
- [Navigation2 Documentation](https://navigation.ros.org/)
- [Nav2 Tutorials](https://navigation.ros.org/tutorials/index.html)
- [Isaac ROS Navigation Integration](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_navigation)

## Review Questions
1. What are the key differences between Nav2 and the original ROS navigation stack?
2. How does Nav2 handle the unique challenges of humanoid robot navigation?
3. What role do behavior trees play in Nav2's architecture?

## Practical Exercises
1. Configure Nav2 for a humanoid robot simulation
2. Implement a custom local planner for humanoid kinematics
3. Test navigation performance with different planner configurations
4. Integrate Isaac perception data with Nav2 costmaps