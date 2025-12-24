---
id: chapter-1-5-launch-files-parameters
title: Launch Files and Parameters
sidebar_label: Launch Files and Parameters
---

# Launch Files and Parameters

## Goal
Learn to create and use ROS 2 launch files to manage complex robot systems and configure parameters for humanoid robot applications.

## Learning Objectives
- Create launch files to start multiple nodes simultaneously
- Configure and manage parameters for robot systems
- Use launch arguments for flexible deployments
- Understand the launch system architecture in ROS 2
- Implement best practices for organizing launch files

## Overview
Launch files in ROS 2 allow you to start multiple nodes with specific configurations simultaneously. For humanoid robots, which typically involve dozens of nodes for control, perception, and communication, launch files are essential for system management. Parameters provide a way to configure node behavior without recompiling code, which is crucial for tuning robot performance across different scenarios.

## Key Concepts
- **Launch Files**: XML or Python files that define how to start multiple nodes
- **Launch Arguments**: Parameters that can be passed to launch files at runtime
- **Parameters**: Configuration values that nodes can access at runtime
- **Composable Nodes**: Nodes that can be loaded into a single process
- **Launch Conditions**: Conditional execution of launch elements
- **Node Remapping**: Changing topic/service names at launch time

## Step-by-Step Breakdown
1. **Launch File Structure**
   - Launch files can be written in Python or XML
   - Python launch files offer more flexibility and logic
   - XML launch files are more declarative

2. **Creating Launch Files**
   - Define the launch file function
   - Add nodes to the launch description
   - Include other launch files if needed
   - Set up parameters and arguments

3. **Parameter Management**
   - Define parameters in launch files
   - Load parameters from YAML files
   - Pass parameters to nodes
   - Use parameter files for different configurations

4. **Launch Arguments**
   - Define arguments that can be passed at runtime
   - Use arguments to customize launch behavior
   - Provide default values for arguments

5. **Advanced Launch Features**
   - Conditional launching based on arguments
   - Including other launch files
   - Composable nodes for performance
   - Node remapping for flexible connections

## Code Examples
```python
# Example launch file for a humanoid robot
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        DeclareLaunchArgument(
            'robot_name',
            default_value='simple_humanoid',
            description='Name of the robot'
        ),

        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_description':
                 Command(['xacro ', FindFile('my_humanoid_description',
                                           'urdf/simple_humanoid.urdf.xacro')])}
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),

        # Joint state publisher node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # Humanoid controller node
        Node(
            package='humanoid_controller',
            executable='controller_node',
            name='humanoid_controller',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_name': LaunchConfiguration('robot_name')},
                # Load additional parameters from file
                PathJoinSubstitution([
                    FindPackageShare('humanoid_controller'),
                    'config',
                    'controller_params.yaml'
                ])
            ],
            remappings=[
                ('/joint_states', 'joint_states'),
                ('/cmd_vel', 'cmd_vel')
            ]
        ),

        # Composable node container for perception stack
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudNode',
                    name='pointcloud_node',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ]
                )
            ],
            output='screen'
        )
    ])
```

Parameter configuration file example (YAML):
```yaml
/**:  # Apply to all nodes
  ros__parameters:
    use_sim_time: false

robot_state_publisher:
  ros__parameters:
    publish_frequency: 50.0
    ignore_timestamp: false

humanoid_controller:
  ros__parameters:
    control_frequency: 200.0
    balance_control:
      kp: 10.0
      ki: 0.1
      kd: 0.05
    walking_pattern:
      step_height: 0.05
      step_length: 0.3
      step_duration: 1.0
```

## Diagrams
```
Launch System Architecture:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Launch File     │    │ Parameter File  │    │ Node Definition │
│ (Python/XML)    │    │ (YAML)          │    │ (Executable)    │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │     Launch Service      │
                    │   (launch service)      │
                    └─────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │     Process Manager     │
                    │   (starts nodes)        │
                    └─────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │        Nodes            │
                    │   (with parameters)     │
                    └─────────────────────────┘
```

## Case Study
In humanoid robotics, launch files are critical for managing the complex system architecture. A typical humanoid robot might have separate launch files for: perception stack, control stack, communication stack, and simulation interface. Parameters allow tuning of control gains, walking patterns, and sensor configurations without recompiling. For example, different parameter sets might be used for standing, walking, or running behaviors.

## References
- [ROS 2 Launch System](https://docs.ros.org/en/humble/p/launch/)
- [Parameters in ROS 2](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-CPP.html)
- [Launch File Best Practices](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

## Review Questions
1. What is the difference between launch arguments and parameters?
2. When would you use a ComposableNodeContainer instead of regular nodes?
3. Explain how parameter files can be used to manage different robot configurations.

## Practical Exercises
1. Create a launch file that starts a publisher and subscriber node
2. Create a parameter file and use it in your launch file
3. Implement a launch file with arguments that change node behavior
4. Create a launch file that starts multiple nodes with different namespaces