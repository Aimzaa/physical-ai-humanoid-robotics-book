---
id: chapter-4-4-ros2-action-execution
title: ROS 2 Action Execution
sidebar_label: ROS 2 Action Execution
---

# ROS 2 Action Execution

## Goal
Implement robust action execution systems for humanoid robots using ROS 2 actions, enabling reliable task completion with feedback, cancellation, and goal management capabilities.

## Learning Objectives
- Understand ROS 2 action architecture and message types
- Implement action servers for humanoid robot capabilities
- Create action clients for task execution
- Handle action feedback and result reporting
- Implement action-based navigation and manipulation
- Design action execution monitoring and recovery systems

## Overview
ROS 2 actions provide a powerful framework for executing long-running tasks with built-in feedback, goal management, and cancellation capabilities. For humanoid robots, actions are essential for implementing complex behaviors like walking, manipulation, navigation, and other tasks that take time to complete and require monitoring. Unlike simple topics or services, actions allow for continuous feedback during execution and the ability to cancel or preempt tasks, making them ideal for the complex, multi-step behaviors required by humanoid robots.

## Key Concepts
- **Action Server**: Node that executes action goals and provides feedback
- **Action Client**: Node that sends action goals and receives feedback
- **Goal**: Request to perform a specific task
- **Feedback**: Continuous status updates during execution
- **Result**: Final outcome of action execution
- **Preemption**: Canceling current goal for new one
- **Goal Handles**: Managing active action goals

## Step-by-Step Breakdown
1. **Action Message Definition**
   - Define custom action message types
   - Specify goal, feedback, and result structures
   - Generate action message files
   - Validate message definitions

2. **Action Server Implementation**
   - Create action server nodes for robot capabilities
   - Implement goal acceptance/rejection logic
   - Handle action execution with feedback
   - Manage goal preemption and cancellation

3. **Action Client Development**
   - Create action clients for task execution
   - Implement goal sending and monitoring
   - Handle feedback and result processing
   - Implement timeout and error handling

4. **Humanoid-Specific Actions**
   - Implement walking and locomotion actions
   - Create manipulation and grasping actions
   - Design navigation actions for humanoid kinematics
   - Configure balance and stability actions

5. **Action Execution Monitoring**
   - Monitor action progress and status
   - Implement recovery behaviors for failures
   - Handle action timeouts and errors
   - Log action execution for debugging

6. **Integration with Planning Systems**
   - Connect action execution to planning systems
   - Implement action-based task scheduling
   - Handle multi-action coordination
   - Validate action execution success

## Code Examples
```python
# Example ROS 2 action execution for humanoid robot
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, Point, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
import time
import threading
from typing import Optional
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration

# Import custom action messages (these would be defined in your package)
# For this example, we'll define them inline as they would typically be in .action files

# Since we can't import custom actions, we'll use a simulated approach
# In a real implementation, these would be generated from .action files
from rclpy.action import Action
from rclpy.action.goal_status import GoalStatus

class HumanoidWalkGoal:
    def __init__(self):
        self.target_pose = Pose()
        self.speed = 0.5
        self.enable_balance = True

class HumanoidWalkFeedback:
    def __init__(self):
        self.current_pose = Pose()
        self.progress = 0.0
        self.status = "walking"
        self.balance_stable = True

class HumanoidWalkResult:
    def __init__(self):
        self.success = False
        self.error_message = ""
        self.final_pose = Pose()

class HumanoidWalkAction:
    """Simulated action class for humanoid walking"""
    Goal = HumanoidWalkGoal
    Feedback = HumanoidWalkFeedback
    Result = HumanoidWalkResult

class HumanoidManipulationGoal:
    def __init__(self):
        self.object_id = ""
        self.target_pose = Pose()
        self.grasp_type = "precision"
        self.force_limit = 50.0

class HumanoidManipulationFeedback:
    def __init__(self):
        self.current_pose = Pose()
        self.progress = 0.0
        self.status = "moving"
        self.gripper_position = 0.0

class HumanoidManipulationResult:
    def __init__(self):
        self.success = False
        self.error_message = ""
        self.object_grasped = False

class HumanoidManipulationAction:
    """Simulated action class for humanoid manipulation"""
    Goal = HumanoidManipulationGoal
    Feedback = HumanoidManipulationFeedback
    Result = HumanoidManipulationResult

class HumanoidWalkActionServer(Node):
    """Action server for humanoid walking behavior"""

    def __init__(self):
        super().__init__('humanoid_walk_action_server')

        # Create action server with reentrant callback group for concurrent goals
        self._action_server = ActionServer(
            self,
            HumanoidWalkAction,
            'humanoid_walk',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Robot state publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        # Robot state
        self.current_pose = Pose()
        self.is_walking = False
        self.current_goal_handle = None

        self.get_logger().info('Humanoid Walk Action Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject goal requests"""
        self.get_logger().info(f'Received walk goal: {goal_request.target_pose}')

        # Check if goal is valid (e.g., target is reachable)
        if self.is_target_reachable(goal_request.target_pose):
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.get_logger().warn('Goal is not reachable')
            return rclpy.action.server.GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info('Received cancel request')
        return rclpy.action.CancelResponse.ACCEPT

    def is_target_reachable(self, target_pose):
        """Check if target pose is reachable"""
        # In real implementation, this would check navigation maps, kinematics, etc.
        return True

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the walking action"""
        self.get_logger().info('Executing walk action...')

        # Store goal handle
        self.current_goal_handle = goal_handle

        # Get goal parameters
        target_pose = goal_handle.request.target_pose
        speed = goal_handle.request.speed
        enable_balance = goal_handle.request.enable_balance

        # Initialize feedback
        feedback_msg = HumanoidWalkFeedback()
        result = HumanoidWalkResult()

        try:
            # Execute walking behavior
            success = self.perform_walking(target_pose, speed, enable_balance, goal_handle)

            if success:
                result.success = True
                result.final_pose = self.current_pose
                goal_handle.succeed()
                self.get_logger().info('Walk action completed successfully')
            else:
                result.success = False
                result.error_message = "Failed to reach target pose"
                goal_handle.abort()
                self.get_logger().info('Walk action failed')

        except Exception as e:
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            self.get_logger().error(f'Walk action exception: {e}')

        # Publish final status
        status_msg = String()
        status_msg.data = "walk_completed" if result.success else "walk_failed"
        self.status_pub.publish(status_msg)

        return result

    def perform_walking(self, target_pose, speed, enable_balance, goal_handle: ServerGoalHandle):
        """Perform the actual walking behavior"""
        # Calculate distance to target
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance = (dx*dx + dy*dy)**0.5

        # Simulate walking progress
        steps = int(distance / 0.1)  # 10cm per step simulation
        for i in range(steps):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Walk action cancelled')
                return False

            # Update progress
            progress = (i + 1) / steps
            self.current_pose.position.x += dx / steps
            self.current_pose.position.y += dy / steps

            # Create and publish feedback
            feedback_msg = HumanoidWalkFeedback()
            feedback_msg.current_pose = self.current_pose
            feedback_msg.progress = progress
            feedback_msg.status = "walking"
            feedback_msg.balance_stable = True  # Simulated

            goal_handle.publish_feedback(feedback_msg)

            # Publish velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = speed if progress < 0.95 else speed * 0.5  # Slow down near target
            self.cmd_vel_pub.publish(cmd_vel)

            # Sleep to simulate walking time
            time.sleep(0.1)

            # Check for obstacles or other issues
            if self.detect_obstacles():
                self.get_logger().warn('Obstacle detected during walking')
                return False

        # Final position adjustment
        self.current_pose.position.x = target_pose.position.x
        self.current_pose.position.y = target_pose.position.y
        self.current_pose.orientation = target_pose.orientation

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        return True

    def detect_obstacles(self):
        """Detect obstacles during walking (simulated)"""
        # In real implementation, this would check sensor data
        import random
        return random.random() < 0.01  # 1% chance of obstacle

class HumanoidManipulationActionServer(Node):
    """Action server for humanoid manipulation behavior"""

    def __init__(self):
        super().__init__('humanoid_manipulation_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            HumanoidManipulationAction,
            'humanoid_manipulation',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publishers for manipulation
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.status_pub = self.create_publisher(String, '/manipulation/status', 10)

        # Robot state
        self.current_joint_positions = [0.0] * 12  # Simulated joint positions
        self.current_goal_handle = None

        self.get_logger().info('Humanoid Manipulation Action Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject manipulation goals"""
        self.get_logger().info(f'Received manipulation goal: {goal_request.object_id}')

        # Check if object exists and is reachable
        if self.is_object_reachable(goal_request.object_id, goal_request.target_pose):
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.get_logger().warn('Object not reachable')
            return rclpy.action.server.GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Handle manipulation goal cancellation"""
        self.get_logger().info('Received manipulation cancel request')
        return rclpy.action.CancelResponse.ACCEPT

    def is_object_reachable(self, object_id, target_pose):
        """Check if object is reachable for manipulation"""
        # In real implementation, this would check kinematics and collision
        return True

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the manipulation action"""
        self.get_logger().info(f'Executing manipulation action for object: {goal_handle.request.object_id}')

        # Store goal handle
        self.current_goal_handle = goal_handle

        # Get goal parameters
        object_id = goal_handle.request.object_id
        target_pose = goal_handle.request.target_pose
        grasp_type = goal_handle.request.grasp_type
        force_limit = goal_handle.request.force_limit

        # Initialize result
        result = HumanoidManipulationResult()
        feedback_msg = HumanoidManipulationFeedback()

        try:
            # Execute manipulation behavior
            success = self.perform_manipulation(object_id, target_pose, grasp_type, force_limit, goal_handle)

            if success:
                result.success = True
                result.object_grasped = True
                goal_handle.succeed()
                self.get_logger().info('Manipulation action completed successfully')
            else:
                result.success = False
                result.error_message = "Failed to manipulate object"
                goal_handle.abort()
                self.get_logger().info('Manipulation action failed')

        except Exception as e:
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            self.get_logger().error(f'Manipulation action exception: {e}')

        # Publish final status
        status_msg = String()
        status_msg.data = "manipulation_completed" if result.success else "manipulation_failed"
        self.status_pub.publish(status_msg)

        return result

    def perform_manipulation(self, object_id, target_pose, grasp_type, force_limit, goal_handle: ServerGoalHandle):
        """Perform the actual manipulation behavior"""
        # Simulate manipulation steps
        steps = 100
        for i in range(steps):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Manipulation action cancelled')
                return False

            # Update progress
            progress = (i + 1) / steps
            feedback_msg = HumanoidManipulationFeedback()
            feedback_msg.progress = progress
            feedback_msg.status = "moving_to_object" if progress < 0.3 else ("grasping" if progress < 0.7 else "moving_to_target")
            feedback_msg.gripper_position = progress if grasp_type == "precision" else progress * 0.8

            goal_handle.publish_feedback(feedback_msg)

            # Update joint positions to simulate movement
            for j in range(len(self.current_joint_positions)):
                self.current_joint_positions[j] += (0.1 / len(self.current_joint_positions))

            # Publish joint commands
            joint_cmd = Float64MultiArray()
            joint_cmd.data = self.current_joint_positions
            self.joint_cmd_pub.publish(joint_cmd)

            # Sleep to simulate manipulation time
            time.sleep(0.05)

        return True

class ActionExecutionClient(Node):
    """Action client for executing humanoid actions"""

    def __init__(self):
        super().__init__('action_execution_client')

        # Create action clients
        self.walk_client = ActionClient(self, HumanoidWalkAction, 'humanoid_walk')
        self.manipulation_client = ActionClient(self, HumanoidManipulationAction, 'humanoid_manipulation')

        # Publishers for command interface
        self.command_sub = self.create_subscription(
            String, '/action/command', self.command_callback, 10
        )

        # Publishers for results
        self.result_pub = self.create_publisher(String, '/action/result', 10)
        self.feedback_pub = self.create_publisher(String, '/action/feedback', 10)

        self.get_logger().info('Action Execution Client initialized')

    def command_callback(self, msg: String):
        """Process action commands"""
        command = msg.data.strip()
        self.get_logger().info(f'Received action command: {command}')

        # Parse command and execute appropriate action
        if command.startswith('walk_to '):
            # Extract target coordinates from command like "walk_to 1.0 2.0 0.0"
            try:
                parts = command.split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])

                target_pose = Pose()
                target_pose.position.x = x
                target_pose.position.y = y
                target_pose.position.z = z

                self.send_walk_goal(target_pose)
            except (ValueError, IndexError):
                self.get_logger().error(f'Invalid walk command format: {command}')

        elif command.startswith('grasp_object '):
            # Extract object ID from command like "grasp_object cup_1"
            try:
                object_id = command.split(' ', 1)[1]
                self.send_manipulation_goal(object_id)
            except IndexError:
                self.get_logger().error(f'Invalid manipulation command format: {command}')

    def send_walk_goal(self, target_pose: Pose):
        """Send walk action goal"""
        # Wait for action server
        if not self.walk_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Walk action server not available')
            return

        # Create goal
        goal_msg = HumanoidWalkAction.Goal()
        goal_msg.target_pose = target_pose
        goal_msg.speed = 0.3
        goal_msg.enable_balance = True

        # Send goal
        self.get_logger().info(f'Sending walk goal to ({target_pose.position.x}, {target_pose.position.y})')

        self._send_goal_future = self.walk_client.send_goal_async(
            goal_msg,
            feedback_callback=self.walk_feedback_callback
        )

        self._send_goal_future.add_done_callback(self.walk_goal_response_callback)

    def send_manipulation_goal(self, object_id: str):
        """Send manipulation action goal"""
        # Wait for action server
        if not self.manipulation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Manipulation action server not available')
            return

        # Create goal
        goal_msg = HumanoidManipulationAction.Goal()
        goal_msg.object_id = object_id
        goal_msg.target_pose = Pose()  # Default pose
        goal_msg.grasp_type = "precision"
        goal_msg.force_limit = 50.0

        # Send goal
        self.get_logger().info(f'Sending manipulation goal for object: {object_id}')

        self._send_manipulation_goal_future = self.manipulation_client.send_goal_async(
            goal_msg,
            feedback_callback=self.manipulation_feedback_callback
        )

        self._send_manipulation_goal_future.add_done_callback(self.manipulation_goal_response_callback)

    def walk_feedback_callback(self, feedback_msg):
        """Handle walk action feedback"""
        self.get_logger().debug(f'Walk progress: {feedback_msg.progress:.2f}')

        # Publish feedback
        feedback_status = String()
        feedback_status.data = f'walk_progress:{feedback_msg.progress:.2f}'
        self.feedback_pub.publish(feedback_status)

    def manipulation_feedback_callback(self, feedback_msg):
        """Handle manipulation action feedback"""
        self.get_logger().debug(f'Manipulation progress: {feedback_msg.progress:.2f}')

        # Publish feedback
        feedback_status = String()
        feedback_status.data = f'manipulation_progress:{feedback_msg.progress:.2f}'
        self.feedback_pub.publish(feedback_status)

    def walk_goal_response_callback(self, future):
        """Handle walk goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Walk goal rejected')
            return

        self.get_logger().info('Walk goal accepted')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.walk_result_callback)

    def manipulation_goal_response_callback(self, future):
        """Handle manipulation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Manipulation goal rejected')
            return

        self.get_logger().info('Manipulation goal accepted')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.manipulation_result_callback)

    def walk_result_callback(self, future):
        """Handle walk action result"""
        result = future.result().result
        self.get_logger().info(f'Walk result: {result.success}')

        # Publish result
        result_msg = String()
        result_msg.data = f'walk_result:{result.success}:{result.error_message}'
        self.result_pub.publish(result_msg)

    def manipulation_result_callback(self, future):
        """Handle manipulation action result"""
        result = future.result().result
        self.get_logger().info(f'Manipulation result: {result.success}')

        # Publish result
        result_msg = String()
        result_msg.data = f'manipulation_result:{result.success}:{result.error_message}'
        self.result_pub.publish(result_msg)

class ActionExecutionMonitor(Node):
    """Monitor for action execution status and recovery"""

    def __init__(self):
        super().__init__('action_execution_monitor')

        # Subscribers for action status
        self.result_sub = self.create_subscription(
            String, '/action/result', self.result_callback, 10
        )
        self.feedback_sub = self.create_subscription(
            String, '/action/feedback', self.feedback_callback, 10
        )

        # Publishers for recovery commands
        self.recovery_pub = self.create_publisher(String, '/action/recovery', 10)

        # Action execution tracking
        self.action_history = []
        self.active_actions = {}

        self.get_logger().info('Action Execution Monitor initialized')

    def result_callback(self, msg: String):
        """Handle action result messages"""
        result_data = msg.data
        self.get_logger().info(f'Action result received: {result_data}')

        # Parse result and track in history
        self.action_history.append({
            'timestamp': self.get_clock().now().nanoseconds,
            'result': result_data,
            'success': 'True' in result_data
        })

        # Check if action failed and needs recovery
        if 'False' in result_data:
            self.handle_action_failure(result_data)

    def feedback_callback(self, msg: String):
        """Handle action feedback messages"""
        feedback_data = msg.data
        self.get_logger().debug(f'Action feedback: {feedback_data}')

        # Update active action tracking
        if feedback_data.startswith('walk_progress:'):
            progress = float(feedback_data.split(':')[1])
            self.active_actions['walk'] = progress
        elif feedback_data.startswith('manipulation_progress:'):
            progress = float(feedback_data.split(':')[1])
            self.active_actions['manipulation'] = progress

    def handle_action_failure(self, result_data: str):
        """Handle action failure and trigger recovery"""
        self.get_logger().warn(f'Action failed: {result_data}')

        # Determine appropriate recovery action
        recovery_action = self.determine_recovery_action(result_data)

        if recovery_action:
            recovery_msg = String()
            recovery_msg.data = recovery_action
            self.recovery_pub.publish(recovery_msg)

            self.get_logger().info(f'Sent recovery action: {recovery_action}')

    def determine_recovery_action(self, failure_info: str) -> Optional[str]:
        """Determine appropriate recovery action based on failure"""
        if 'obstacle' in failure_info.lower():
            return 'navigate_around_obstacle'
        elif 'timeout' in failure_info.lower():
            return 'retry_action'
        elif 'collision' in failure_info.lower():
            return 'move_to_safe_position'
        else:
            return 'abort_task'

def main(args=None):
    """Main function for action execution system"""
    rclpy.init(args=args)

    # Create nodes
    walk_server = HumanoidWalkActionServer()
    manipulation_server = HumanoidManipulationActionServer()
    action_client = ActionExecutionClient()
    monitor = ActionExecutionMonitor()

    try:
        # Run nodes with multi-threaded executor to handle concurrent actions
        executor = MultiThreadedExecutor()
        executor.add_node(walk_server)
        executor.add_node(manipulation_server)
        executor.add_node(action_client)
        executor.add_node(monitor)

        executor.spin()

    except KeyboardInterrupt:
        walk_server.get_logger().info('Shutting down action execution system')
    finally:
        walk_server.destroy_node()
        manipulation_server.destroy_node()
        action_client.destroy_node()
        monitor.destroy_node()
        rclpy.shutdown()

# Example of using actions in a behavior tree or task planner
class ActionBasedTaskPlanner(Node):
    """Task planner using ROS 2 actions"""

    def __init__(self):
        super().__init__('action_based_task_planner')

        # Action clients for different capabilities
        self.walk_client = ActionClient(self, HumanoidWalkAction, 'humanoid_walk')
        self.manipulation_client = ActionClient(self, HumanoidManipulationAction, 'humanoid_manipulation')

        # Task queue
        self.task_queue = []
        self.current_task_index = 0

        # Timer for task execution
        self.task_timer = self.create_timer(1.0, self.execute_next_task)

        self.get_logger().info('Action-Based Task Planner initialized')

    def add_task(self, task_type: str, parameters: dict):
        """Add task to execution queue"""
        task = {
            'type': task_type,
            'parameters': parameters,
            'completed': False
        }
        self.task_queue.append(task)
        self.get_logger().info(f'Added task: {task_type} to queue')

    def execute_next_task(self):
        """Execute the next task in the queue"""
        if self.current_task_index >= len(self.task_queue):
            self.get_logger().info('All tasks completed')
            return

        current_task = self.task_queue[self.current_task_index]
        if not current_task['completed']:
            self.execute_task(current_task)
            self.current_task_index += 1

    def execute_task(self, task: dict):
        """Execute a single task using appropriate action"""
        task_type = task['type']
        parameters = task['parameters']

        if task_type == 'walk':
            self.execute_walk_task(parameters)
        elif task_type == 'manipulate':
            self.execute_manipulation_task(parameters)
        else:
            self.get_logger().warn(f'Unknown task type: {task_type}')

    def execute_walk_task(self, params: dict):
        """Execute walk task"""
        target_pose = params.get('target_pose', Pose())
        speed = params.get('speed', 0.5)

        if self.walk_client.wait_for_server(timeout_sec=1.0):
            goal_msg = HumanoidWalkAction.Goal()
            goal_msg.target_pose = target_pose
            goal_msg.speed = speed
            goal_msg.enable_balance = True

            self.walk_client.send_goal_async(goal_msg)
            self.get_logger().info(f'Walk task sent to ({target_pose.position.x}, {target_pose.position.y})')
        else:
            self.get_logger().error('Walk action server not available')

    def execute_manipulation_task(self, params: dict):
        """Execute manipulation task"""
        object_id = params.get('object_id', '')
        grasp_type = params.get('grasp_type', 'precision')

        if self.manipulation_client.wait_for_server(timeout_sec=1.0):
            goal_msg = HumanoidManipulationAction.Goal()
            goal_msg.object_id = object_id
            goal_msg.grasp_type = grasp_type

            self.manipulation_client.send_goal_async(goal_msg)
            self.get_logger().info(f'Manipulation task sent for object: {object_id}')
        else:
            self.get_logger().error('Manipulation action server not available')
```

## Diagrams
```
ROS 2 Action Architecture:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Action         │    │  Action         │    │  Action         │
│  Client         │◄──►│  Communication  │◄──►│  Server         │
│  (Goal Sender)   │    │  (ROS 2)       │    │  (Goal Executor) │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Goal Request   │    │  Goal, Feedback │    │  Goal Execution │
│  (Command)      │    │  Messages       │    │  (Robot Control)│
└─────────────────┘    └─────────────────┘    └─────────────────┘

Action Execution Flow:

Client Sends Goal ──► Server Accepts Goal ──► Execute Action ──► Send Feedback
                    (Goal Handle Created)    (Long-running      (Progress Updates)
                                             Task)

Action States:

┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  PENDING    │───►│  ACTIVE     │───►│  SUCCEEDED  │    │  ABORTED    │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                        │                     │
                        ▼                     ▼
                   ┌─────────────┐    ┌─────────────┐
                   │  CANCELLED  │    │  REJECTED   │
                   └─────────────┘    └─────────────┘

Humanoid Task Sequence:

Walk Action ──► Manipulation Action ──► Walk Action ──► Speak Action
(Single Goal)   (Single Goal)        (Single Goal)   (Single Goal)
```

## Case Study
The ROS 2 navigation stack (Nav2) extensively uses actions for robot navigation tasks. When a robot receives a navigation goal, the navigation system creates an action that provides continuous feedback about the robot's progress, allows for goal preemption if a new goal is received, and reports the final result when the navigation is complete. This approach has proven highly effective for humanoid robots as well, where complex multi-step behaviors like "walk to the kitchen, pick up the cup, and bring it to the table" can be broken down into a sequence of actions that provide feedback and can be monitored and adjusted during execution.

## References
- [ROS 2 Actions Documentation](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Actions-In-ROS2.html)
- [Action Server and Client Tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Python.html)
- [ROS 2 Action Design Patterns](https://design.ros2.org/articles/actions.html)

## Review Questions
1. What are the key differences between ROS 2 actions, topics, and services?
2. How do actions enable better task management for humanoid robots?
3. What are the benefits of using actions for long-running robot behaviors?

## Practical Exercises
1. Implement a simple action server for a basic robot movement
2. Create an action client that sends goals and monitors progress
3. Test action preemption by sending new goals during execution
4. Implement error handling and recovery in action execution