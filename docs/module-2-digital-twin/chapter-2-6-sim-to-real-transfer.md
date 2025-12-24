---
id: chapter-2-6-sim-to-real-transfer
title: Sim-to-Real Transfer Concepts
sidebar_label: Sim-to-Real Transfer
---

# Sim-to-Real Transfer Concepts

## Goal
Understand the principles and techniques for transferring behaviors, controllers, and algorithms developed in simulation to real humanoid robots.

## Learning Objectives
- Identify the challenges in sim-to-real transfer for humanoid robots
- Understand domain randomization techniques to improve transferability
- Learn about system identification and model calibration methods
- Implement reality gap mitigation strategies
- Evaluate the effectiveness of sim-to-real transfer
- Apply sim-to-real techniques to humanoid robot control

## Overview
Sim-to-real transfer is the process of taking controllers, behaviors, and algorithms developed in simulation and successfully deploying them on real robots. This is particularly challenging for humanoid robots due to their complex dynamics, multiple contact points, and sensitivity to modeling errors. The "reality gap" - the difference between simulated and real environments - must be carefully addressed to ensure successful transfer.

## Key Concepts
- **Reality Gap**: The difference between simulated and real robot behavior
- **Domain Randomization**: Training in varied simulation conditions to improve robustness
- **System Identification**: Determining real robot parameters to improve simulation accuracy
- **Domain Adaptation**: Adjusting simulation to better match real conditions
- **Transfer Learning**: Adapting simulation-trained models for real robot use
- **Fine-tuning**: Adjusting simulation-trained controllers on the real robot

## Step-by-Step Breakdown
1. **Reality Gap Analysis**
   - Identify sources of simulation errors (mass, friction, compliance, etc.)
   - Measure discrepancies between simulation and reality
   - Categorize gap sources (kinematic, dynamic, environmental)
   - Quantify gap impact on controller performance

2. **System Identification**
   - Measure real robot parameters (mass, inertia, friction)
   - Identify actuator dynamics and limitations
   - Characterize sensor noise and delays
   - Calibrate simulation parameters to match reality

3. **Domain Randomization**
   - Randomize physical parameters during training (mass, friction, etc.)
   - Vary environmental conditions in simulation
   - Add noise to sensors and actuators
   - Train controllers to be robust to parameter variations

4. **Model Calibration**
   - Use real robot data to refine simulation models
   - Implement parameter estimation algorithms
   - Validate simulation accuracy through experiments
   - Iterate between simulation and reality

5. **Transfer Validation**
   - Test controllers in simulation before real deployment
   - Implement safety measures for real robot testing
   - Gradually increase controller complexity
   - Monitor and record transfer performance

6. **Adaptation Techniques**
   - Implement online parameter adaptation
   - Use machine learning for controller tuning
   - Apply gain scheduling for different conditions
   - Implement robust control methods

## Code Examples
```python
# Example system identification and parameter adaptation for humanoid robot
import numpy as np
from scipy.optimize import minimize
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimToRealTransfer(Node):
    def __init__(self):
        super().__init__('sim_to_real_transfer')

        # Robot parameters to identify
        self.robot_params = {
            'mass': 30.0,  # Base estimate
            'com_offset_x': 0.0,  # Center of mass offset
            'com_offset_y': 0.0,
            'friction_coeff': 0.1,
            'actuator_delay': 0.01,
        }

        # Subscribe to joint states and controller states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/controller_state',
            self.controller_state_callback,
            10
        )

        # Publisher for parameter updates
        self.param_pub = self.create_publisher(
            JointTrajectory,
            '/parameter_adaptation',
            10
        )

        # Data collection for system identification
        self.collected_data = {
            'timestamps': [],
            'positions': [],
            'velocities': [],
            'accelerations': [],
            'torques': [],
            'commands': []
        }

        # Timer for parameter estimation
        self.param_estimation_timer = self.create_timer(
            1.0,  # Every second
            self.estimate_parameters
        )

        self.get_logger().info('Sim-to-Real Transfer node initialized')

    def joint_state_callback(self, msg):
        """Collect joint state data for system identification"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Store current state
        self.collected_data['timestamps'].append(current_time)
        self.collected_data['positions'].append(np.array(msg.position))
        self.collected_data['velocities'].append(np.array(msg.velocity))
        self.collected_data['torques'].append(np.array(msg.effort))

        # Estimate accelerations from velocities (if needed)
        if len(self.collected_data['velocities']) >= 2:
            dt = current_time - self.collected_data['timestamps'][-2]
            if dt > 0:
                acc = (np.array(msg.velocity) -
                       np.array(self.collected_data['velocities'][-2])) / dt
                self.collected_data['accelerations'].append(acc)
            else:
                self.collected_data['accelerations'].append(np.zeros(len(msg.velocity)))

        # Limit data storage to last 1000 samples
        if len(self.collected_data['timestamps']) > 1000:
            for key in self.collected_data:
                self.collected_data[key] = self.collected_data[key][-1000:]

    def controller_state_callback(self, msg):
        """Collect controller state data"""
        # Store command information
        self.collected_data['commands'].append({
            'positions': np.array(msg.desired.positions),
            'velocities': np.array(msg.desired.velocities),
            'accelerations': np.array(msg.desired.accelerations)
        })

        # Limit command storage
        if len(self.collected_data['commands']) > 1000:
            self.collected_data['commands'] = self.collected_data['commands'][-1000:]

    def estimate_parameters(self):
        """Estimate robot parameters from collected data"""
        if len(self.collected_data['positions']) < 10:
            return  # Not enough data

        # Use collected data to estimate parameters
        # This is a simplified example - real system identification is more complex
        try:
            # Estimate mass properties using inverse dynamics
            estimated_params = self.inverse_dynamics_identification()

            # Update internal parameter estimates
            for param_name, new_value in estimated_params.items():
                if param_name in self.robot_params:
                    # Apply smoothing to parameter updates
                    alpha = 0.1  # Learning rate
                    old_value = self.robot_params[param_name]
                    self.robot_params[param_name] = (
                        alpha * new_value + (1 - alpha) * old_value
                    )

            self.get_logger().info(f'Updated parameters: {self.robot_params}')

        except Exception as e:
            self.get_logger().error(f'Parameter estimation failed: {e}')

    def inverse_dynamics_identification(self):
        """Estimate parameters using inverse dynamics approach"""
        # Collect recent data for identification
        n_samples = min(50, len(self.collected_data['positions']))
        if n_samples < 10:
            return {}

        # Extract data arrays
        pos_data = np.array(self.collected_data['positions'][-n_samples:])
        vel_data = np.array(self.collected_data['velocities'][-n_samples:])
        torque_data = np.array(self.collected_data['torques'][-n_samples:])

        # For simplicity, estimate mass using gravity effects
        # In practice, this would use full inverse dynamics model
        # and more sophisticated optimization

        # Estimate average gravity effect on a joint
        # (This is a very simplified example)
        gravity_torque = np.mean(torque_data[:, 0])  # Assume joint 0 is affected by gravity
        estimated_mass = abs(gravity_torque) / 9.81  # Very simplified estimate

        return {
            'mass': max(1.0, min(100.0, estimated_mass)),  # Clamp to reasonable range
        }

    def domain_randomization_callback(self):
        """Apply domain randomization to simulation parameters"""
        # Randomize parameters for training
        randomized_params = {}

        for param_name, base_value in self.robot_params.items():
            if param_name == 'mass':
                # Randomize mass by ±20%
                variation = np.random.uniform(0.8, 1.2)
                randomized_params[param_name] = base_value * variation
            elif param_name == 'friction_coeff':
                # Randomize friction by ±50%
                variation = np.random.uniform(0.5, 1.5)
                randomized_params[param_name] = max(0.01, base_value * variation)
            elif 'com_offset' in param_name:
                # Randomize center of mass offset
                variation = np.random.uniform(-0.05, 0.05)
                randomized_params[param_name] = base_value + variation
            else:
                randomized_params[param_name] = base_value

        return randomized_params

    def evaluate_transfer_performance(self, controller_output):
        """Evaluate how well simulation controller works on real robot"""
        # Monitor key performance metrics
        metrics = {
            'tracking_error': np.mean(np.abs(controller_output['position_error'])),
            'stability': self.check_stability(controller_output),
            'energy_efficiency': self.calculate_energy(controller_output),
            'safety': self.check_safety_bounds(controller_output)
        }

        return metrics

    def check_stability(self, controller_output):
        """Check if robot remains stable"""
        # Check for excessive oscillations or divergence
        if 'position_error' in controller_output:
            error_history = controller_output['position_error']
            if len(error_history) > 10:
                recent_error = np.array(error_history[-10:])
                std_error = np.std(recent_error)
                return float(std_error < 0.5)  # Stable if std < 0.5
        return 1.0

    def calculate_energy(self, controller_output):
        """Calculate energy efficiency metric"""
        if 'torques' in controller_output and 'velocities' in controller_output:
            power = np.sum(np.abs(controller_output['torques'] *
                                 controller_output['velocities']))
            return 1.0 / (1.0 + power)  # Higher efficiency = lower power
        return 0.5

    def check_safety_bounds(self, controller_output):
        """Check if robot remains within safety bounds"""
        # Check joint limits, velocities, accelerations
        safety_score = 1.0
        if 'positions' in controller_output:
            pos = np.array(controller_output['positions'])
            # Check if positions are within reasonable bounds
            if np.any(np.abs(pos) > 3.0):  # Assuming reasonable joint limits
                safety_score *= 0.5
        return float(safety_score)

def main(args=None):
    rclpy.init(args=args)
    sim_to_real_node = SimToRealTransfer()

    try:
        rclpy.spin(sim_to_real_node)
    except KeyboardInterrupt:
        sim_to_real_node.get_logger().info('Shutting down')
    finally:
        sim_to_real_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Diagrams
```
Sim-to-Real Transfer Process:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Simulation    │───▶│  Real Robot     │───▶│  Performance    │
│   Environment   │    │  (Hardware)     │    │  Evaluation     │
│                 │    │                 │    │                 │
│ • Accurate      │    │ • Physical      │    │ • Success rate  │
│   Physics       │    │   Dynamics      │    │ • Error metrics │
│ • Sensor        │    │ • Real Sensors  │    │ • Stability     │
│   Models        │    │ • Actuator      │    │ • Efficiency    │
│ • Environment   │    │   Limitations   │    │                 │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                       │                       │
        │                ┌──────▼──────┐               │
        └────────────────┤  Transfer   ├───────────────┘
                         │  Process     │
                         │              │
                         │ • Parameter  │
                         │   Calibration│
                         │ • Domain     │
                         │   Random.    │
                         │ • Adaptation │
                         └─────────────┘

Reality Gap Mitigation Strategies:

┌─────────────────────────────────────────────────────────┐
│                    Reality Gap                          │
│  (Simulation vs Real World Differences)                 │
│                                                         │
│  ┌─────────────────┐    ┌─────────────────┐            │
│  │  Physical       │    │  Environmental  │            │
│  │  Differences    │    │  Differences    │            │
│  │                │    │                 │            │
│  │ • Mass, inertia │    │ • Floor texture │            │
│  │ • Friction      │    │ • Lighting      │            │
│  │ • Compliance    │    │ • Objects       │            │
│  │ • Actuator      │    │ • Temperature   │            │
│  │   dynamics      │    │                 │            │
│  └─────────────────┘    └─────────────────┘            │
│                                                         │
│  Mitigation Strategies:                                 │
│  • System Identification ───▶ Calibrate simulation     │
│  • Domain Randomization ──▶ Robust controllers         │
│  • Fine-tuning ───────────▶ Adapt on real robot        │
└─────────────────────────────────────────────────────────┘
```

## Case Study
The Boston Dynamics Atlas robot demonstrates successful sim-to-real transfer through sophisticated simulation models and careful reality gap mitigation. The company uses detailed physics simulation with accurate mass properties, actuator dynamics, and contact models. They employ domain randomization techniques during controller training, randomizing physical parameters to create robust controllers that work in both simulation and reality. The result is that many of Atlas's complex behaviors, including dynamic walking and parkour, are first developed and refined in simulation before being deployed on the real robot.

## References
- [Domain Randomization for Transferring Deep Neural Networks](https://arxiv.org/abs/1703.06907)
- [Sim-to-Real Transfer in Robotics](https://arxiv.org/abs/1802.01568)
- [System Identification for Robotics](https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-ident.pdf)

## Review Questions
1. What are the main sources of the reality gap in humanoid robot simulation?
2. How does domain randomization improve sim-to-real transfer?
3. What role does system identification play in improving simulation accuracy?

## Practical Exercises
1. Implement a simple system identification routine for a single joint
2. Create a simulation with randomized parameters and test controller robustness
3. Compare controller performance in simulation vs with added noise/uncertainty
4. Design a parameter adaptation algorithm for a simulated humanoid