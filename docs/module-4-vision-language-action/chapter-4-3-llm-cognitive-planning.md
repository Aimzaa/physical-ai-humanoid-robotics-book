---
id: chapter-4-3-llm-cognitive-planning
title: LLM Cognitive Planning
sidebar_label: LLM Cognitive Planning
---

# LLM Cognitive Planning

## Goal
Implement cognitive planning systems for humanoid robots using Large Language Models (LLMs), enabling high-level reasoning and task decomposition for complex robotic behaviors.

## Learning Objectives
- Understand how Large Language Models can be used for robotic planning
- Configure LLMs for cognitive planning in robotic systems
- Implement task decomposition and reasoning capabilities
- Integrate LLM planning with robot execution systems
- Optimize LLM usage for real-time robotic applications
- Evaluate the effectiveness of LLM-based planning

## Overview
Large Language Models (LLMs) have emerged as powerful tools for cognitive planning in robotics, offering the ability to understand complex natural language commands and decompose them into executable robotic tasks. For humanoid robots, LLMs can serve as high-level cognitive controllers that interpret user intentions, reason about the environment, and generate detailed action plans. This approach enables more flexible and adaptable robot behaviors, allowing humanoid robots to handle novel situations and complex multi-step tasks through natural language interaction.

## Key Concepts
- **Large Language Models (LLMs)**: Neural networks trained on vast text corpora
- **Cognitive Planning**: High-level reasoning and task decomposition
- **Task Decomposition**: Breaking complex tasks into executable steps
- **Symbolic Reasoning**: Logical reasoning about objects and actions
- **Context Awareness**: Understanding environmental and situational context
- **Plan Execution**: Connecting LLM plans to robot control systems
- **Replanning**: Adapting plans based on execution feedback

## Step-by-Step Breakdown
1. **LLM Selection and Configuration**
   - Choose appropriate LLM for robot planning tasks
   - Configure model parameters for planning applications
   - Set up API access and rate limiting
   - Optimize for robot hardware constraints

2. **Planning Prompt Engineering**
   - Design effective prompts for task decomposition
   - Create context templates for robot environment
   - Implement few-shot learning examples
   - Optimize prompts for planning accuracy

3. **Task Decomposition Implementation**
   - Parse high-level commands into subtasks
   - Generate sequential action plans
   - Handle dependencies between actions
   - Create fallback plans for failures

4. **Environment Context Integration**
   - Provide environmental information to LLM
   - Update context based on sensor data
   - Handle dynamic environment changes
   - Integrate with perception systems

5. **Plan Execution Interface**
   - Connect LLM plans to robot execution systems
   - Implement plan monitoring and feedback
   - Handle plan execution failures
   - Enable replanning capabilities

6. **Performance Optimization**
   - Optimize LLM query frequency
   - Implement caching for common plans
   - Configure timeout and error handling
   - Validate planning accuracy and safety

## Code Examples
```python
# Example LLM cognitive planning for humanoid robot
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import openai
import json
import re
import time
from typing import Dict, List, Optional, Tuple, Any
import requests
import threading
import queue
from dataclasses import dataclass
from enum import Enum

@dataclass
class RobotAction:
    """Represents a single robot action"""
    action_type: str  # 'move', 'grasp', 'speak', etc.
    parameters: Dict[str, Any]
    description: str
    priority: int = 1

@dataclass
class TaskPlan:
    """Represents a complete task plan"""
    task_id: str
    original_command: str
    actions: List[RobotAction]
    context: Dict[str, Any]
    created_at: float

class PlanStatus(Enum):
    """Status of plan execution"""
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

class LLMCognitivePlanningNode(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planning')

        # Initialize OpenAI API key (in real implementation, this should be configured securely)
        # openai.api_key = "your-api-key-here"  # Don't hardcode in real implementation

        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Planning parameters
        self.planning_params = {
            'model': 'gpt-3.5-turbo',  # or 'gpt-4' for better reasoning
            'temperature': 0.3,        # Lower for more consistent planning
            'max_tokens': 1000,        # Limit response length
            'timeout': 30.0,           # API timeout in seconds
            'max_retries': 3,          # Number of retry attempts
            'context_window': 4096     # Maximum context tokens
        }

        # Robot state and environment
        self.robot_pose = None
        self.environment_objects = {}
        self.current_plan = None
        self.plan_queue = queue.Queue()
        self.plan_status = PlanStatus.PENDING

        # Initialize publishers
        self.plan_pub = self.create_publisher(String, '/llm/plan', 10)
        self.action_pub = self.create_publisher(String, '/robot/action', 10)
        self.status_pub = self.create_publisher(String, '/llm/status', 10)
        self.feedback_pub = self.create_publisher(String, '/llm/feedback', 10)

        # Initialize subscribers
        self.command_sub = self.create_subscription(
            String, '/llm/command', self.command_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, '/perception/objects', self.perception_callback, 10
        )
        self.execution_feedback_sub = self.create_subscription(
            String, '/execution/feedback', self.execution_feedback_callback, 10
        )

        # Initialize plan execution timer
        self.plan_timer = self.create_timer(0.1, self.plan_execution_callback)

        # Store for completed plans
        self.completed_plans = []

        self.get_logger().info('LLM Cognitive Planning node initialized')

    def command_callback(self, msg: String):
        """Process high-level commands for cognitive planning"""
        command = msg.data.strip()
        self.get_logger().info(f'Received command for planning: {command}')

        # Update status
        status_msg = String()
        status_msg.data = f'Planning for: {command}'
        self.status_pub.publish(status_msg)

        # Generate plan using LLM
        plan = self.generate_plan(command)

        if plan:
            self.current_plan = plan
            self.plan_status = PlanStatus.PENDING

            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps({
                'task_id': plan.task_id,
                'command': plan.original_command,
                'actions': [action.__dict__ for action in plan.actions]
            })
            self.plan_pub.publish(plan_msg)

            self.get_logger().info(f'Generated plan with {len(plan.actions)} actions')
        else:
            self.get_logger().error('Failed to generate plan')

    def generate_plan(self, command: str) -> Optional[TaskPlan]:
        """Generate a task plan using LLM"""
        try:
            # Create context for the LLM
            context = self.build_planning_context(command)

            # Create prompt for task decomposition
            prompt = self.create_planning_prompt(context)

            # Call LLM to generate plan
            response = self.call_llm(prompt)

            if response:
                # Parse LLM response into structured plan
                plan = self.parse_plan_response(response, command)
                return plan

        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')

        return None

    def build_planning_context(self, command: str) -> Dict[str, Any]:
        """Build context for LLM planning"""
        context = {
            'command': command,
            'robot_capabilities': [
                'move_to_location',
                'grasp_object',
                'release_object',
                'speak_text',
                'navigate',
                'detect_objects',
                'manipulate_objects'
            ],
            'environment_objects': self.environment_objects,
            'robot_pose': self.robot_pose,
            'available_actions': [
                'move_forward', 'move_backward', 'turn_left', 'turn_right',
                'grasp', 'release', 'speak', 'stop', 'wait'
            ],
            'constraints': {
                'safety': 'avoid obstacles and ensure stable movement',
                'efficiency': 'minimize path length and execution time',
                'accuracy': 'ensure precise object manipulation'
            }
        }

        return context

    def create_planning_prompt(self, context: Dict[str, Any]) -> str:
        """Create prompt for LLM planning"""
        prompt = f"""
You are a cognitive planner for a humanoid robot. Your task is to decompose high-level commands into executable action sequences.

Context:
- Robot capabilities: {context['robot_capabilities']}
- Environment objects: {context['environment_objects']}
- Current robot pose: {context['robot_pose']}
- Available actions: {context['available_actions']}
- Constraints: {context['constraints']}

Command: {context['command']}

Please decompose this command into a sequence of specific actions that the robot can execute. Each action should be:
1. Specific and executable
2. Include necessary parameters
3. Consider environmental constraints
4. Follow logical sequence

Format your response as a JSON array of actions with the following structure:
[
    {{
        "action_type": "string",
        "parameters": {{"param1": "value1", "param2": "value2"}},
        "description": "Human-readable description of the action"
    }}
]

Example for "pick up the red cup and bring it to the table":
[
    {{"action_type": "detect_object", "parameters": {{"object_type": "cup", "color": "red"}}, "description": "Locate the red cup"}},
    {{"action_type": "navigate_to", "parameters": {{"target": "red_cup_location"}}, "description": "Move to the red cup"}},
    {{"action_type": "grasp_object", "parameters": {{"object_id": "red_cup"}}, "description": "Pick up the red cup"}},
    {{"action_type": "detect_object", "parameters": {{"object_type": "table"}}, "description": "Locate the table"}},
    {{"action_type": "navigate_to", "parameters": {{"target": "table_location"}}, "description": "Move to the table"}},
    {{"action_type": "release_object", "parameters": {{"object_id": "red_cup"}}, "description": "Place the cup on the table"}}
]

Now provide the action sequence for the given command:
"""

        return prompt

    def call_llm(self, prompt: str) -> Optional[str]:
        """Call LLM API to generate plan"""
        try:
            # In a real implementation, you would use the actual OpenAI API
            # For this example, we'll simulate the response
            # response = openai.ChatCompletion.create(
            #     model=self.planning_params['model'],
            #     messages=[{"role": "user", "content": prompt}],
            #     temperature=self.planning_params['temperature'],
            #     max_tokens=self.planning_params['max_tokens'],
            #     timeout=self.planning_params['timeout']
            # )
            # return response.choices[0].message.content

            # Simulated response for demonstration
            simulated_response = self.simulate_llm_response(prompt)
            return simulated_response

        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {e}')
            return None

    def simulate_llm_response(self, prompt: str) -> str:
        """Simulate LLM response for demonstration purposes"""
        # This is a simplified simulation - in real implementation,
        # this would call the actual LLM API
        if "pick up" in prompt.lower() and "cup" in prompt.lower():
            return json.dumps([
                {"action_type": "detect_object", "parameters": {"object_type": "cup"}, "description": "Locate the cup"},
                {"action_type": "navigate_to", "parameters": {"target": "cup_location"}, "description": "Move to the cup"},
                {"action_type": "grasp_object", "parameters": {"object_id": "cup"}, "description": "Pick up the cup"},
                {"action_type": "navigate_to", "parameters": {"target": "destination"}, "description": "Move to destination"},
                {"action_type": "release_object", "parameters": {"object_id": "cup"}, "description": "Place the cup down"}
            ])
        elif "go to" in prompt.lower() or "move to" in prompt.lower():
            return json.dumps([
                {"action_type": "navigate_to", "parameters": {"target": "destination"}, "description": "Move to the specified location"},
                {"action_type": "stop", "parameters": {}, "description": "Stop at destination"}
            ])
        else:
            return json.dumps([
                {"action_type": "speak", "parameters": {"text": "I understand the command"}, "description": "Acknowledge command"},
                {"action_type": "wait", "parameters": {"duration": 1}, "description": "Wait for next instruction"}
            ])

    def parse_plan_response(self, response: str, original_command: str) -> Optional[TaskPlan]:
        """Parse LLM response into structured TaskPlan"""
        try:
            # Parse JSON response
            actions_data = json.loads(response)

            # Convert to RobotAction objects
            actions = []
            for action_data in actions_data:
                action = RobotAction(
                    action_type=action_data['action_type'],
                    parameters=action_data['parameters'],
                    description=action_data['description']
                )
                actions.append(action)

            # Create TaskPlan
            plan = TaskPlan(
                task_id=f"plan_{int(time.time())}",
                original_command=original_command,
                actions=actions,
                context=self.build_planning_context(original_command),
                created_at=time.time()
            )

            return plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing LLM response as JSON: {e}')
            return None
        except KeyError as e:
            self.get_logger().error(f'Missing key in LLM response: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error parsing plan response: {e}')
            return None

    def perception_callback(self, msg: String):
        """Update environment context from perception system"""
        try:
            objects_data = json.loads(msg.data)
            self.environment_objects.update(objects_data)

            self.get_logger().debug(f'Updated environment objects: {list(self.environment_objects.keys())}')

        except json.JSONDecodeError:
            self.get_logger().warn('Invalid perception data format')

    def execution_feedback_callback(self, msg: String):
        """Handle feedback from action execution system"""
        try:
            feedback_data = json.loads(msg.data)
            action_id = feedback_data.get('action_id')
            status = feedback_data.get('status')
            details = feedback_data.get('details', '')

            self.get_logger().info(f'Action feedback: {action_id} - {status}')

            # Update plan status based on feedback
            if status == 'failed' and self.current_plan:
                self.handle_action_failure(action_id, details)

        except json.JSONDecodeError:
            self.get_logger().warn('Invalid feedback data format')

    def handle_action_failure(self, action_id: str, details: str):
        """Handle action failure and potentially replan"""
        self.get_logger().warn(f'Action failed: {action_id}, details: {details}')

        # Publish feedback about failure
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            'event': 'action_failure',
            'action_id': action_id,
            'details': details,
            'suggest_replan': True
        })
        self.feedback_pub.publish(feedback_msg)

        # In a real implementation, you might trigger replanning here
        # For now, just log the failure
        self.get_logger().info('Action failure handled - replanning logic would go here')

    def plan_execution_callback(self):
        """Main plan execution callback"""
        if not self.current_plan or self.plan_status != PlanStatus.PENDING:
            return

        # Start executing the plan
        self.plan_status = PlanStatus.EXECUTING

        # Execute each action in the plan
        for i, action in enumerate(self.current_plan.actions):
            self.get_logger().info(f'Executing action {i+1}/{len(self.current_plan.actions)}: {action.description}')

            # Publish action for execution
            action_msg = String()
            action_msg.data = json.dumps({
                'action_type': action.action_type,
                'parameters': action.parameters,
                'plan_id': self.current_plan.task_id,
                'step': i + 1
            })
            self.action_pub.publish(action_msg)

            # Wait for action completion (in real implementation, this would be asynchronous)
            time.sleep(0.5)  # Simulate action execution time

        # Mark plan as completed
        self.plan_status = PlanStatus.COMPLETED
        self.completed_plans.append(self.current_plan)

        # Publish completion feedback
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            'event': 'plan_completed',
            'plan_id': self.current_plan.task_id,
            'actions_completed': len(self.current_plan.actions)
        })
        self.feedback_pub.publish(feedback_msg)

        self.get_logger().info(f'Plan completed: {self.current_plan.original_command}')

        # Clear current plan
        self.current_plan = None

    def get_environment_context(self) -> Dict[str, Any]:
        """Get current environment context for planning"""
        return {
            'objects': self.environment_objects,
            'robot_pose': self.robot_pose,
            'navigation_map': 'available',  # In real implementation, this would be actual map
            'time': time.time()
        }

class LLMPlanningOptimizer(Node):
    """Node for optimizing LLM planning performance"""

    def __init__(self):
        super().__init__('llm_planning_optimizer')

        # Initialize optimization parameters
        self.optimization_params = {
            'plan_caching': True,
            'context_compression': True,
            'api_rate_limiting': True,
            'response_validation': True
        }

        # Plan cache
        self.plan_cache = {}
        self.cache_size_limit = 100

        # Rate limiting
        self.api_call_times = []
        self.max_calls_per_minute = 20  # Adjust based on API limits

        # Initialize optimization components
        self.initialize_optimization()

        self.get_logger().info('LLM Planning Optimizer initialized')

    def initialize_optimization(self):
        """Initialize optimization components"""
        try:
            # Set up plan caching
            if self.optimization_params['plan_caching']:
                self.setup_plan_caching()

            # Set up context management
            if self.optimization_params['context_compression']:
                self.setup_context_compression()

            # Set up API rate limiting
            if self.optimization_params['api_rate_limiting']:
                self.setup_api_rate_limiting()

        except Exception as e:
            self.get_logger().error(f'Error initializing optimization: {e}')

    def setup_plan_caching(self):
        """Set up plan caching for common commands"""
        self.get_logger().info('Plan caching configured')

    def setup_context_compression(self):
        """Set up context compression for efficient LLM communication"""
        self.get_logger().info('Context compression configured')

    def setup_api_rate_limiting(self):
        """Set up API rate limiting to respect quotas"""
        self.get_logger().info('API rate limiting configured')

    def is_api_call_allowed(self) -> bool:
        """Check if API call is allowed based on rate limiting"""
        current_time = time.time()

        # Remove old calls (older than 1 minute)
        self.api_call_times = [t for t in self.api_call_times if current_time - t < 60]

        # Check if we're under the limit
        if len(self.api_call_times) < self.max_calls_per_minute:
            self.api_call_times.append(current_time)
            return True

        return False

    def get_cached_plan(self, command: str) -> Optional[TaskPlan]:
        """Get cached plan for command if available"""
        if command in self.plan_cache:
            cached_plan = self.plan_cache[command]
            # Check if cache is still valid (not too old)
            if time.time() - cached_plan.created_at < 300:  # 5 minutes
                return cached_plan
            else:
                # Remove expired cache
                del self.plan_cache[command]

        return None

    def cache_plan(self, command: str, plan: TaskPlan):
        """Cache plan for command"""
        # Remove oldest entries if cache is too large
        if len(self.plan_cache) >= self.cache_size_limit:
            # Remove oldest entry
            oldest_key = min(self.plan_cache.keys(), key=lambda k: self.plan_cache[k].created_at)
            del self.plan_cache[oldest_key]

        self.plan_cache[command] = plan

class CognitivePlanningInterface(Node):
    """Interface for integrating cognitive planning with other systems"""

    def __init__(self):
        super().__init__('cognitive_planning_interface')

        # Publishers for different subsystems
        self.navigation_pub = self.create_publisher(String, '/navigation/plan', 10)
        self.manipulation_pub = self.create_publisher(String, '/manipulation/plan', 10)
        self.speech_pub = self.create_publisher(String, '/speech/plan', 10)

        # Subscriber for LLM plans
        self.plan_sub = self.create_subscription(
            String, '/llm/plan', self.plan_callback, 10
        )

        self.get_logger().info('Cognitive Planning Interface initialized')

    def plan_callback(self, msg: String):
        """Process LLM-generated plan and distribute to subsystems"""
        try:
            plan_data = json.loads(msg.data)
            actions = plan_data.get('actions', [])

            # Distribute actions to appropriate subsystems
            for action in actions:
                self.route_action_to_subsystem(action)

        except json.JSONDecodeError:
            self.get_logger().warn('Invalid plan data format')

    def route_action_to_subsystem(self, action: Dict[str, Any]):
        """Route action to appropriate subsystem"""
        action_type = action.get('action_type', '').lower()

        if any(navigation_action in action_type for navigation_action in ['move', 'navigate', 'go', 'walk', 'turn']):
            self.navigation_pub.publish(String(data=json.dumps(action)))
        elif any(manipulation_action in action_type for manipulation_action in ['grasp', 'pick', 'lift', 'release', 'place', 'manipulate']):
            self.manipulation_pub.publish(String(data=json.dumps(action)))
        elif any(speech_action in action_type for speech_action in ['speak', 'say', 'talk', 'greet']):
            self.speech_pub.publish(String(data=json.dumps(action)))
        else:
            # General action - might need further processing
            self.get_logger().info(f'Routing general action: {action_type}')

def main(args=None):
    """Main function for LLM cognitive planning system"""
    rclpy.init(args=args)

    # Create nodes
    planning_node = LLMCognitivePlanningNode()
    optimizer_node = LLMPlanningOptimizer()
    interface_node = CognitivePlanningInterface()

    try:
        # Run the nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(planning_node)
        executor.add_node(optimizer_node)
        executor.add_node(interface_node)

        executor.spin()

    except KeyboardInterrupt:
        planning_node.get_logger().info('Shutting down LLM cognitive planning system')
    finally:
        planning_node.destroy_node()
        optimizer_node.destroy_node()
        interface_node.destroy_node()
        rclpy.shutdown()

# Alternative implementation using local LLM (e.g., Hugging Face models)
class LocalLLMPlanningNode(Node):
    """LLM planning using local models for privacy and offline capability"""

    def __init__(self):
        super().__init__('local_llm_planning')

        # Initialize local model (example with Hugging Face transformers)
        # In real implementation, you would load a local model
        # self.local_model = AutoModelForCausalLM.from_pretrained("microsoft/DialoGPT-medium")
        # self.tokenizer = AutoTokenizer.from_pretrained("microsoft/DialoGPT-medium")

        self.get_logger().info('Local LLM Planning node initialized')

    def generate_plan_local(self, command: str) -> Optional[TaskPlan]:
        """Generate plan using local LLM model"""
        # Implementation would use local model for planning
        # This provides privacy and offline capability
        pass
```

## Diagrams
```
LLM Cognitive Planning Architecture:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Natural        │    │  LLM Cognitive  │    │  Task           │
│  Language       │───►│  Planning       │───►│  Execution      │
│  Command        │    │  (Decomposition) │    │  (Robot Control)│
│  ("Pick up     │    │                 │    │                 │
│   the red cup") │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Context        │    │  Plan           │    │  Action         │
│  (Environment,  │    │  (Action       │    │  (Motor Control,│
│   Capabilities) │    │   Sequence)     │    │   Navigation)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘

Cognitive Planning Process:

High-Level Command ──► Context Building ──► LLM Reasoning ──► Action Sequence
(Natural Language)    (Environment,        (Task              (Executable
                     Capabilities,         Decomposition)      Actions)
                     Constraints)

Plan Execution Loop:

Plan Generated ──► Action Distribution ──► Execution Feedback ──► Plan Monitoring
(Sequence of      (To Subsystems)        (Success/Failure)     (Status, Updates)
 Actions)
```

## Case Study
OpenAI's collaboration with robotics researchers has demonstrated the effectiveness of LLMs for cognitive planning in robotic systems. In one study, an LLM was used to interpret high-level commands and generate detailed manipulation plans for a robotic arm. The system was able to handle complex, multi-step tasks like "set the table for dinner" by decomposing them into specific actions like "pick up plate", "place plate on table", "pick up fork", etc. For humanoid robots, similar approaches have enabled more sophisticated behaviors by leveraging the LLM's ability to reason about objects, spatial relationships, and task dependencies.

## References
- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [Large Language Models for Robotics](https://arxiv.org/abs/2302.12022)
- [Language-Conditioned Neural Networks for Robot Planning](https://arxiv.org/abs/2204.02389)

## Review Questions
1. How do LLMs enable more flexible robotic planning compared to traditional methods?
2. What are the challenges in using LLMs for real-time robotic applications?
3. How can LLM planning be made more reliable and safe for robot execution?

## Practical Exercises
1. Implement a simple LLM-based planner for basic robot tasks
2. Test the planner with various natural language commands
3. Evaluate the accuracy of task decomposition
4. Implement plan validation and safety checks