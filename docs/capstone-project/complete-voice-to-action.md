---
id: complete-voice-to-action
title: Complete Voice-to-Action System Implementation
sidebar_label: Complete Voice-to-Action System
---

# Complete Voice-to-Action System Implementation

## Goal
Implement a complete end-to-end voice-to-action system that integrates speech recognition, natural language understanding, AI planning, and robot execution into a seamless pipeline for humanoid robot control.

## Learning Objectives
- Understand the complete voice-to-action pipeline architecture
- Implement real-time speech recognition and processing
- Integrate natural language understanding with AI planning
- Connect voice commands to robot action execution
- Optimize the pipeline for real-time performance
- Validate system accuracy and response time

## Overview
The complete voice-to-action system represents the culmination of the VLA (Vision-Language-Action) module concepts, creating an end-to-end pipeline that allows users to control humanoid robots using natural language commands. This system integrates automatic speech recognition (ASR), natural language understanding (NLU), large language model (LLM) planning, and action execution into a unified pipeline that provides natural human-robot interaction. The system must handle real-time processing requirements while maintaining accuracy and safety.

## Key Concepts
- **End-to-End Pipeline**: Complete flow from voice input to robot action
- **Real-Time Processing**: Meeting timing constraints for responsive interaction
- **Natural Language Understanding**: Interpreting user intent from speech
- **AI Planning**: Generating action sequences from commands
- **Action Execution**: Executing planned actions on the robot
- **Pipeline Optimization**: Techniques to improve performance
- **Error Handling**: Managing failures gracefully

## Step-by-Step Breakdown
1. **Pipeline Architecture Design**
   - Design complete voice-to-action system architecture
   - Plan component interfaces and data flow
   - Configure real-time processing requirements
   - Establish safety and error handling protocols

2. **Speech Recognition Integration**
   - Integrate Whisper or similar ASR system
   - Configure audio preprocessing and noise reduction
   - Optimize for real-time performance
   - Handle different acoustic environments

3. **Natural Language Understanding**
   - Implement intent classification and entity extraction
   - Connect to language understanding models
   - Handle ambiguous or unclear commands
   - Create command-to-action mappings

4. **AI Planning System**
   - Integrate LLM-based planning capabilities
   - Create task decomposition from commands
   - Handle multi-step action sequences
   - Implement plan validation and safety checks

5. **Action Execution Pipeline**
   - Connect to robot action execution systems
   - Implement action sequencing and monitoring
   - Handle action failures and recovery
   - Ensure safe robot operation

6. **System Validation and Optimization**
   - Test pipeline performance and accuracy
   - Optimize for real-time response
   - Validate safety and reliability
   - Document performance metrics

## Code Examples
```python
# Complete voice-to-action system implementation
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import AudioData, Image
from geometry_msgs.msg import Twist, Pose
from rclpy.action import ActionServer, ActionClient
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
import numpy as np
import torch
import whisper
import speech_recognition as sr
import threading
import queue
import time
import json
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
import openai
import asyncio
from concurrent.futures import ThreadPoolExecutor
import cv2
from cv_bridge import CvBridge

@dataclass
class VoiceCommand:
    """Represents a voice command in the pipeline"""
    audio_data: Optional[bytes] = None
    text: str = ""
    confidence: float = 0.0
    timestamp: float = 0.0
    user_id: str = ""
    processed: bool = False

@dataclass
class PipelineStage:
    """Represents a stage in the voice-to-action pipeline"""
    name: str
    start_time: float = 0.0
    end_time: float = 0.0
    success: bool = False
    result: Optional[Any] = None
    error: Optional[str] = None

@dataclass
class ActionPlan:
    """Represents an action plan generated from voice command"""
    command: str
    intent: str
    entities: List[str]
    actions: List[Dict[str, Any]]
    timestamp: float
    confidence: float

class VoiceToActionPipeline(Node):
    """Complete voice-to-action pipeline system"""

    def __init__(self):
        super().__init__('voice_to_action_pipeline')

        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Initialize pipeline state
        self.pipeline_state = {
            'current_stage': 'idle',
            'active_command': None,
            'pipeline_active': False,
            'error_count': 0,
            'success_count': 0,
            'average_latency': 0.0
        }

        # Initialize queues for pipeline stages
        self.audio_queue = queue.Queue(maxsize=20)
        self.recognition_queue = queue.Queue(maxsize=20)
        self.nlu_queue = queue.Queue(maxsize=20)
        self.planning_queue = queue.Queue(maxsize=20)
        self.execution_queue = queue.Queue(maxsize=20)

        # Initialize Whisper model
        self.whisper_model = None
        self.load_whisper_model()

        # Initialize OpenAI client for planning (use API key securely in real implementation)
        # openai.api_key = "your-api-key-here"

        # Initialize audio processing parameters
        self.audio_params = {
            'sample_rate': 16000,
            'chunk_size': 1024,
            'channels': 1,
            'format': 'int16',
            'vad_threshold': 0.01,  # Voice activity detection threshold
            'silence_duration': 0.5  # Seconds of silence to trigger processing
        }

        # Initialize pipeline parameters
        self.pipeline_params = {
            'max_command_length': 100,
            'confidence_threshold': 0.7,
            'command_timeout': 10.0,
            'pipeline_retry_limit': 3,
            'response_timeout': 5.0,
            'wake_word': 'robot',
            'enable_confirmation': True
        }

        # Initialize publishers
        self.command_pub = self.create_publisher(String, '/voice/command', 10)
        self.action_pub = self.create_publisher(String, '/robot/action', 10)
        self.status_pub = self.create_publisher(String, '/voice_pipeline/status', 10)
        self.feedback_pub = self.create_publisher(String, '/voice_pipeline/feedback', 10)
        self.response_pub = self.create_publisher(String, '/voice_pipeline/response', 10)

        # Initialize robot control publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(
            Float32MultiArray, '/joint_group_position_controller/commands', 10
        )

        # Initialize subscribers
        self.audio_sub = self.create_subscription(
            AudioData, '/audio/data', self.audio_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )

        # Initialize action servers
        self.voice_action_server = ActionServer(
            self,
            VoiceCommandAction,
            'voice_command',
            self.execute_voice_command
        )

        # Initialize pipeline processing threads
        self.pipeline_executor = ThreadPoolExecutor(max_workers=6)

        self.audio_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.recognition_thread = threading.Thread(target=self.process_recognition, daemon=True)
        self.nlu_thread = threading.Thread(target=self.process_nlu, daemon=True)
        self.planning_thread = threading.Thread(target=self.process_planning, daemon=True)
        self.execution_thread = threading.Thread(target=self.process_execution, daemon=True)

        self.audio_thread.start()
        self.recognition_thread.start()
        self.nlu_thread.start()
        self.planning_thread.start()
        self.execution_thread.start()

        # Initialize performance tracking
        self.latency_measurements = []
        self.performance_timer = self.create_timer(1.0, self.performance_monitor)

        self.get_logger().info('Complete Voice-to-Action Pipeline initialized')

    def load_whisper_model(self):
        """Load Whisper model for speech recognition"""
        try:
            # Load a model suitable for real-time processing
            self.whisper_model = whisper.load_model('tiny', device='cpu')
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Error loading Whisper model: {e}')
            # Fallback to speech_recognition library
            self.speech_recognizer = sr.Recognizer()

    def audio_callback(self, msg: AudioData):
        """Handle incoming audio data"""
        try:
            # Add audio data to processing queue
            audio_item = {
                'data': msg.data,
                'timestamp': time.time(),
                'header': msg.header
            }

            try:
                self.audio_queue.put_nowait(audio_item)
                self.pipeline_state['current_stage'] = 'audio_processing'
            except queue.Full:
                self.get_logger().warn('Audio queue full, dropping frame')

        except Exception as e:
            self.get_logger().error(f'Error in audio callback: {e}')

    def image_callback(self, msg: Image):
        """Process image for context-aware voice commands"""
        try:
            # Convert image for context understanding
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # In real implementation, this would provide context for voice commands
            # For example, object detection to understand "pick up the red cup"
            # that's visible in the camera
            pass

        except Exception as e:
            self.get_logger().error(f'Error processing image for context: {e}')

    def process_audio(self):
        """Process audio data and perform speech recognition"""
        audio_buffer = []
        silence_start_time = None
        speech_detected = False

        while rclpy.ok():
            try:
                # Get audio data from queue
                audio_item = self.audio_queue.get(timeout=0.1)
                audio_data = audio_item['data']

                # Convert audio data to numpy array
                audio_array = np.frombuffer(audio_data, dtype=np.int16)

                # Calculate audio energy for VAD (Voice Activity Detection)
                energy = np.mean(np.abs(audio_array.astype(np.float32)))

                if energy > self.audio_params['vad_threshold']:
                    # Speech detected, add to buffer
                    audio_buffer.extend(audio_array)
                    speech_detected = True
                    silence_start_time = None
                else:
                    # Silence detected
                    if speech_detected:
                        if silence_start_time is None:
                            silence_start_time = time.time()
                        elif time.time() - silence_start_time > self.audio_params['silence_duration']:
                            # Process the buffered audio
                            if len(audio_buffer) > 0:
                                self.process_speech_buffer(audio_buffer)
                                audio_buffer = []
                                speech_detected = False
                                silence_start_time = None
                    else:
                        # Continue accumulating silence (for longer speech segments)
                        audio_buffer.extend(audio_array)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in audio processing: {e}')

    def process_speech_buffer(self, audio_buffer):
        """Process accumulated speech buffer"""
        try:
            # Convert buffer to numpy array
            audio_array = np.array(audio_buffer, dtype=np.int16)
            audio_float = audio_array.astype(np.float32) / 32768.0

            # Perform speech recognition
            if self.whisper_model is not None:
                start_time = time.time()
                result = self.whisper_model.transcribe(
                    audio_float,
                    language='en',
                    task='transcribe'
                )
                processing_time = time.time() - start_time

                text = result['text'].strip()
                confidence = result.get('avg_logprob', -1.0)

                if text and confidence > self.pipeline_params['confidence_threshold']:
                    # Create voice command
                    voice_cmd = VoiceCommand(
                        text=text,
                        confidence=confidence,
                        timestamp=time.time()
                    )

                    # Add to recognition queue
                    try:
                        self.recognition_queue.put_nowait(voice_cmd)
                        self.get_logger().info(f'Recognized: "{text}" (conf: {confidence:.2f})')
                    except queue.Full:
                        self.get_logger().warn('Recognition queue full')
                else:
                    self.get_logger().debug(f'Low confidence recognition: {confidence:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error processing speech buffer: {e}')

    def process_recognition(self):
        """Process recognized text through NLU"""
        while rclpy.ok():
            try:
                voice_cmd = self.recognition_queue.get(timeout=0.1)

                # Update pipeline state
                self.pipeline_state['current_stage'] = 'nlu_processing'
                self.pipeline_state['active_command'] = voice_cmd

                # Check for wake word if configured
                if self.pipeline_params['wake_word']:
                    if self.pipeline_params['wake_word'].lower() in voice_cmd.text.lower():
                        # Extract command after wake word
                        command_text = voice_cmd.text.lower().replace(
                            self.pipeline_params['wake_word'].lower(), ''
                        ).strip()
                        voice_cmd.text = command_text
                    else:
                        # No wake word detected, skip processing
                        continue

                # Perform natural language understanding
                nlu_result = self.perform_nlu(voice_cmd.text)

                if nlu_result:
                    # Add to NLU queue
                    nlu_item = {
                        'command': voice_cmd,
                        'nlu_result': nlu_result,
                        'timestamp': time.time()
                    }

                    try:
                        self.nlu_queue.put_nowait(nlu_item)
                        self.get_logger().debug(f'NLU completed for: {voice_cmd.text}')
                    except queue.Full:
                        self.get_logger().warn('NLU queue full')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in recognition processing: {e}')
                self.handle_pipeline_error(e)

    def perform_nlu(self, text: str) -> Optional[Dict[str, Any]]:
        """Perform natural language understanding on text"""
        try:
            # Classify intent
            intent = self.classify_intent(text)

            # Extract entities
            entities = self.extract_entities(text)

            # Determine action type
            action_type = self.determine_action_type(text)

            # Extract parameters
            parameters = self.extract_parameters(text)

            nlu_result = {
                'intent': intent,
                'entities': entities,
                'action_type': action_type,
                'parameters': parameters,
                'original_text': text,
                'timestamp': time.time()
            }

            return nlu_result

        except Exception as e:
            self.get_logger().error(f'Error in NLU: {e}')
            return None

    def classify_intent(self, text: str) -> str:
        """Classify the intent of the command"""
        text_lower = text.lower()

        # Navigation intents
        navigation_keywords = ['move', 'go', 'walk', 'navigate', 'drive', 'turn', 'forward', 'backward', 'left', 'right']
        if any(keyword in text_lower for keyword in navigation_keywords):
            return 'navigation'

        # Manipulation intents
        manipulation_keywords = ['pick', 'grasp', 'take', 'grab', 'lift', 'place', 'put', 'release', 'hold']
        if any(keyword in text_lower for keyword in manipulation_keywords):
            return 'manipulation'

        # Communication intents
        communication_keywords = ['speak', 'say', 'tell', 'hello', 'hi', 'goodbye', 'bye', 'talk']
        if any(keyword in text_lower for keyword in communication_keywords):
            return 'communication'

        # Stop/interrupt intents
        stop_keywords = ['stop', 'halt', 'pause', 'wait', 'cancel']
        if any(keyword in text_lower for keyword in stop_keywords):
            return 'stop'

        # Default to unknown
        return 'unknown'

    def extract_entities(self, text: str) -> List[str]:
        """Extract entities from text"""
        entities = []
        text_lower = text.lower()

        # Common objects
        common_objects = ['cup', 'bottle', 'book', 'chair', 'table', 'person', 'door', 'window', 'box', 'ball']
        for obj in common_objects:
            if obj in text_lower:
                entities.append(obj)

        # Common locations
        common_locations = ['kitchen', 'living room', 'bedroom', 'office', 'hallway', 'bathroom', 'garden']
        for loc in common_locations:
            if loc in text_lower:
                entities.append(loc)

        # Colors
        colors = ['red', 'blue', 'green', 'yellow', 'black', 'white', 'orange', 'purple']
        for color in colors:
            if color in text_lower:
                entities.append(color)

        return entities

    def determine_action_type(self, text: str) -> str:
        """Determine the action type from text"""
        text_lower = text.lower()

        if any(word in text_lower for word in ['move', 'go', 'walk', 'navigate']):
            return 'move_to_location'
        elif any(word in text_lower for word in ['pick', 'grasp', 'take', 'grab']):
            return 'grasp_object'
        elif any(word in text_lower for word in ['speak', 'say', 'tell']):
            return 'speak_text'
        elif any(word in text_lower for word in ['stop', 'halt', 'pause']):
            return 'stop_robot'
        elif any(word in text_lower for word in ['dance', 'wave', 'greet']):
            return 'perform_action'
        else:
            return 'unknown'

    def extract_parameters(self, text: str) -> Dict[str, Any]:
        """Extract parameters from text"""
        params = {}

        # Extract numeric values
        import re
        numbers = re.findall(r'\d+\.?\d*', text)
        if numbers:
            params['numeric_values'] = [float(n) for n in numbers if n]

        # Extract direction words
        directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
        found_directions = [d for d in directions if d in text.lower()]
        if found_directions:
            params['direction'] = found_directions[0]

        # Extract duration
        duration_match = re.search(r'(\d+)\s*(seconds?|secs?|minutes?|mins?)', text.lower())
        if duration_match:
            value, unit = duration_match.groups()
            multiplier = 60 if 'minute' in unit else 1
            params['duration'] = float(value) * multiplier

        return params

    def process_nlu(self):
        """Process NLU results through cognitive planning"""
        while rclpy.ok():
            try:
                nlu_item = self.nlu_queue.get(timeout=0.1)

                # Update pipeline state
                self.pipeline_state['current_stage'] = 'planning'

                # Generate plan using AI system
                plan = self.generate_plan(nlu_item['nlu_result'], nlu_item['command'].text)

                if plan:
                    # Add to planning queue
                    planning_item = {
                        'command': nlu_item['command'],
                        'nlu_result': nlu_item['nlu_result'],
                        'plan': plan,
                        'timestamp': time.time()
                    }

                    try:
                        self.planning_queue.put_nowait(planning_item)
                        self.get_logger().debug(f'Plan generated for: {nlu_item["command"].text}')
                    except queue.Full:
                        self.get_logger().warn('Planning queue full')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in NLU processing: {e}')
                self.handle_pipeline_error(e)

    def generate_plan(self, nlu_result: Dict[str, Any], command_text: str) -> Optional[ActionPlan]:
        """Generate action plan based on NLU result"""
        try:
            intent = nlu_result.get('intent', 'unknown')
            entities = nlu_result.get('entities', [])
            action_type = nlu_result.get('action_type', 'unknown')
            parameters = nlu_result.get('parameters', {})

            # Generate action sequence based on intent
            actions = []

            if intent == 'navigation':
                # Navigation plan
                target_location = entities[0] if entities else 'destination'
                actions = [
                    {'type': 'speak', 'parameters': {'text': f'Navigating to {target_location}'}},
                    {'type': 'navigate_to', 'parameters': {'location': target_location}},
                    {'type': 'confirm_arrival', 'parameters': {'location': target_location}}
                ]
            elif intent == 'manipulation':
                # Manipulation plan
                target_object = entities[0] if entities else 'object'
                actions = [
                    {'type': 'speak', 'parameters': {'text': f'Looking for {target_object}'}},
                    {'type': 'locate_object', 'parameters': {'object': target_object}},
                    {'type': 'approach_object', 'parameters': {'object': target_object}},
                    {'type': 'grasp_object', 'parameters': {'object': target_object}},
                    {'type': 'confirm_grasp', 'parameters': {}}
                ]
            elif intent == 'communication':
                # Communication plan
                actions = [
                    {'type': 'speak', 'parameters': {'text': f'You said: {command_text}'}},
                    {'type': 'wait_for_response', 'parameters': {'duration': 2.0}}
                ]
            elif intent == 'stop':
                # Stop plan
                actions = [
                    {'type': 'stop_robot', 'parameters': {}},
                    {'type': 'speak', 'parameters': {'text': 'Robot stopped'}}
                ]
            elif intent == 'perform_action':
                # Action performance plan
                action_name = command_text.split()[0] if command_text.split() else 'action'
                actions = [
                    {'type': 'speak', 'parameters': {'text': f'Performing {action_name}'}},
                    {'type': 'perform_action', 'parameters': {'action': action_name}},
                    {'type': 'confirm_completion', 'parameters': {}}
                ]
            else:
                # Unknown intent - ask for clarification
                actions = [
                    {'type': 'speak', 'parameters': {'text': f'I don\'t understand: {command_text}. Please try again.'}},
                    {'type': 'request_clarification', 'parameters': {'command': command_text}}
                ]

            # Create action plan
            action_plan = ActionPlan(
                command=command_text,
                intent=intent,
                entities=entities,
                actions=actions,
                timestamp=time.time(),
                confidence=0.8  # Default confidence
            )

            return action_plan

        except Exception as e:
            self.get_logger().error(f'Error in plan generation: {e}')
            return None

    def process_planning(self):
        """Process plans through action execution"""
        while rclpy.ok():
            try:
                planning_item = self.planning_queue.get(timeout=0.1)

                # Update pipeline state
                self.pipeline_state['current_stage'] = 'execution'

                # Execute the plan
                execution_success = self.execute_plan(planning_item['plan'])

                if execution_success:
                    # Add to execution queue for final processing
                    execution_item = {
                        'command': planning_item['command'],
                        'plan': planning_item['plan'],
                        'success': True,
                        'timestamp': time.time()
                    }

                    try:
                        self.execution_queue.put_nowait(execution_item)
                        self.get_logger().info(f'Plan executed successfully: {planning_item["command"].text}')
                    except queue.Full:
                        self.get_logger().warn('Execution queue full')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in planning processing: {e}')
                self.handle_pipeline_error(e)

    def execute_plan(self, plan: ActionPlan) -> bool:
        """Execute an action plan"""
        try:
            for i, action in enumerate(plan.actions):
                action_type = action['type']
                parameters = action['parameters']

                self.get_logger().info(f'Executing action {i+1}/{len(plan.actions)}: {action_type}')

                # Execute action
                success = self.execute_single_action(action_type, parameters)

                if not success:
                    self.get_logger().error(f'Action failed: {action_type}')
                    return False

                # Small delay between actions for safety
                time.sleep(0.1)

            return True

        except Exception as e:
            self.get_logger().error(f'Error in plan execution: {e}')
            return False

    def execute_single_action(self, action_type: str, parameters: Dict[str, Any]) -> bool:
        """Execute a single action"""
        try:
            if action_type == 'speak':
                text = parameters.get('text', '')
                return self.speak_text(text)
            elif action_type == 'navigate_to':
                location = parameters.get('location', 'unknown')
                return self.navigate_to_location(location)
            elif action_type == 'grasp_object':
                obj = parameters.get('object', 'unknown')
                return self.grasp_object(obj)
            elif action_type == 'locate_object':
                obj = parameters.get('object', 'unknown')
                return self.locate_object(obj)
            elif action_type == 'approach_object':
                obj = parameters.get('object', 'unknown')
                return self.approach_object(obj)
            elif action_type == 'stop_robot':
                return self.stop_robot()
            elif action_type == 'perform_action':
                action_name = parameters.get('action', 'unknown')
                return self.perform_action(action_name)
            elif action_type == 'confirm_arrival':
                location = parameters.get('location', 'unknown')
                return self.confirm_arrival(location)
            elif action_type == 'confirm_grasp':
                return self.confirm_grasp()
            elif action_type == 'confirm_completion':
                return self.confirm_completion()
            elif action_type == 'wait_for_response':
                duration = parameters.get('duration', 1.0)
                time.sleep(duration)
                return True
            elif action_type == 'request_clarification':
                command = parameters.get('command', '')
                return self.request_clarification(command)
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing action {action_type}: {e}')
            return False

    def speak_text(self, text: str) -> bool:
        """Speak text using text-to-speech"""
        self.get_logger().info(f'Speaking: {text}')
        # In real implementation, this would use a TTS system
        # For now, we'll just log it
        return True

    def navigate_to_location(self, location: str) -> bool:
        """Navigate to a specific location"""
        self.get_logger().info(f'Navigating to {location}')
        # In real implementation, this would send navigation commands
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # Move forward
        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(2.0)  # Simulate navigation time
        self.stop_robot()
        return True

    def grasp_object(self, obj: str) -> bool:
        """Grasp a specific object"""
        self.get_logger().info(f'Grasping {obj}')
        # In real implementation, this would send manipulation commands
        joint_cmd = Float32MultiArray()
        joint_cmd.data = [0.1, 0.1, 0.1, 0.1, 0.1]  # Example joint positions
        self.joint_cmd_pub.publish(joint_cmd)
        time.sleep(1.0)  # Simulate grasping time
        return True

    def locate_object(self, obj: str) -> bool:
        """Locate an object using perception system"""
        self.get_logger().info(f'Locating {obj}')
        # In real implementation, this would use computer vision
        return True

    def approach_object(self, obj: str) -> bool:
        """Approach an object"""
        self.get_logger().info(f'Approaching {obj}')
        # In real implementation, this would navigate to object location
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1  # Move slowly forward
        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(1.0)
        self.stop_robot()
        return True

    def stop_robot(self) -> bool:
        """Stop robot movement"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Robot stopped')
        return True

    def perform_action(self, action_name: str) -> bool:
        """Perform a specific action (dance, wave, etc.)"""
        self.get_logger().info(f'Performing action: {action_name}')
        # In real implementation, this would execute predefined behaviors
        return True

    def confirm_arrival(self, location: str) -> bool:
        """Confirm arrival at location"""
        self.get_logger().info(f'Confirmed arrival at {location}')
        return True

    def confirm_grasp(self) -> bool:
        """Confirm successful grasp"""
        self.get_logger().info('Confirmed successful grasp')
        return True

    def confirm_completion(self) -> bool:
        """Confirm task completion"""
        self.get_logger().info('Task completed successfully')
        return True

    def request_clarification(self, command: str) -> bool:
        """Request clarification for ambiguous command"""
        self.get_logger().info(f'Requesting clarification for: {command}')
        return self.speak_text(f"I didn't understand '{command}'. Could you please repeat or rephrase?")

    def process_execution(self):
        """Final processing and user feedback"""
        while rclpy.ok():
            try:
                execution_item = self.execution_queue.get(timeout=0.1)

                # Update pipeline state
                self.pipeline_state['current_stage'] = 'completed'

                # Calculate and record latency
                start_time = execution_item['plan'].timestamp
                end_time = execution_item['timestamp']
                latency = end_time - start_time
                self.latency_measurements.append(latency)

                # Send success feedback to user
                feedback_msg = String()
                feedback_msg.data = json.dumps({
                    'event': 'command_completed',
                    'command': execution_item['command'].text,
                    'success': execution_item['success'],
                    'latency': latency,
                    'timestamp': execution_item['timestamp']
                })
                self.feedback_pub.publish(feedback_msg)

                # Send response to user
                response_msg = String()
                if execution_item['success']:
                    response_msg.data = f'Successfully executed: {execution_item["command"].text}'
                else:
                    response_msg.data = f'Failed to execute: {execution_item["command"].text}'
                self.response_pub.publish(response_msg)

                # Update success counter
                if execution_item['success']:
                    self.pipeline_state['success_count'] += 1
                else:
                    self.pipeline_state['error_count'] += 1

                self.get_logger().info(f'Command completed: {execution_item["command"].text} '
                                     f'(Latency: {latency:.2f}s)')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in execution processing: {e}')
                self.handle_pipeline_error(e)

    def handle_pipeline_error(self, error: Exception):
        """Handle pipeline errors and recovery"""
        self.pipeline_state['error_count'] += 1

        # Publish error feedback
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            'event': 'pipeline_error',
            'error': str(error),
            'stage': self.pipeline_state['current_stage'],
            'timestamp': time.time()
        })
        self.feedback_pub.publish(feedback_msg)

        self.get_logger().error(f'Pipeline error in {self.pipeline_state["current_stage"]}: {error}')

    def performance_monitor(self):
        """Monitor pipeline performance"""
        if self.latency_measurements:
            avg_latency = sum(self.latency_measurements) / len(self.latency_measurements)
            self.pipeline_state['average_latency'] = avg_latency

            # Keep only recent measurements (last 100)
            if len(self.latency_measurements) > 100:
                self.latency_measurements = self.latency_measurements[-100:]

        # Publish performance status
        status_msg = String()
        status_data = {
            'stage': self.pipeline_state['current_stage'],
            'success_count': self.pipeline_state['success_count'],
            'error_count': self.pipeline_state['error_count'],
            'average_latency': self.pipeline_state['average_latency'],
            'queue_sizes': {
                'audio': self.audio_queue.qsize(),
                'recognition': self.recognition_queue.qsize(),
                'nlu': self.nlu_queue.qsize(),
                'planning': self.planning_queue.qsize(),
                'execution': self.execution_queue.qsize()
            }
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

    def execute_voice_command(self, goal_handle):
        """Execute voice command action (for external calls)"""
        try:
            command_text = goal_handle.request.command
            self.get_logger().info(f'Executing external voice command: {command_text}')

            # Create voice command object
            voice_cmd = VoiceCommand(
                text=command_text,
                confidence=1.0,  # External commands have full confidence
                timestamp=time.time()
            )

            # Process through pipeline by adding to recognition queue
            # (since it's already text, skip ASR)
            nlu_result = self.perform_nlu(command_text)
            if nlu_result:
                plan = self.generate_plan(nlu_result, command_text)
                if plan:
                    execution_success = self.execute_plan(plan)

                    if execution_success:
                        result = VoiceCommandResult()
                        result.success = True
                        result.message = f'Successfully executed: {command_text}'
                        goal_handle.succeed()
                    else:
                        result = VoiceCommandResult()
                        result.success = False
                        result.message = f'Failed to execute: {command_text}'
                        goal_handle.abort()
                else:
                    result = VoiceCommandResult()
                    result.success = False
                    result.message = f'Could not generate plan for: {command_text}'
                    goal_handle.abort()
            else:
                result = VoiceCommandResult()
                result.success = False
                result.message = f'Could not understand command: {command_text}'
                goal_handle.abort()

        except Exception as e:
            result = VoiceCommandResult()
            result.success = False
            result.message = f'Error processing command: {e}'
            goal_handle.abort()

        return result

class VoiceCommandAction:
    """Action definition for voice commands"""
    pass

class VoiceCommandResult:
    """Result definition for voice commands"""
    def __init__(self):
        self.success = False
        self.message = ""

class PipelineOptimizer:
    """Optimizer for voice-to-action pipeline performance"""

    def __init__(self, node: VoiceToActionPipeline):
        self.node = node
        self.optimization_params = {
            'adaptive_buffer_size': True,
            'model_caching': True,
            'pipeline_parallelism': True,
            'resource_management': True,
            'dynamic_batching': True
        }

    def optimize_pipeline(self):
        """Apply optimizations to improve pipeline performance"""
        try:
            # Optimize audio buffer sizes based on load
            if self.optimization_params['adaptive_buffer_size']:
                self.optimize_buffer_sizes()

            # Optimize model caching
            if self.optimization_params['model_caching']:
                self.optimize_model_caching()

            # Optimize pipeline parallelism
            if self.optimization_params['pipeline_parallelism']:
                self.optimize_parallelism()

            # Optimize resource usage
            if self.optimization_params['resource_management']:
                self.optimize_resources()

            self.node.get_logger().info('Pipeline optimizations applied')

        except Exception as e:
            self.node.get_logger().error(f'Error in pipeline optimization: {e}')

    def optimize_buffer_sizes(self):
        """Optimize queue buffer sizes based on pipeline load"""
        # Adjust buffer sizes based on queue utilization
        avg_queue_size = (
            self.node.audio_queue.qsize() +
            self.node.recognition_queue.qsize() +
            self.node.nlu_queue.qsize() +
            self.node.planning_queue.qsize() +
            self.node.execution_queue.qsize()
        ) / 5

        if avg_queue_size > 15:  # High load
            # Increase buffer sizes to handle load
            pass
        elif avg_queue_size < 5:  # Low load
            # Decrease buffer sizes to save memory
            pass

    def optimize_model_caching(self):
        """Optimize model caching for faster inference"""
        # Preload frequently used models
        pass

    def optimize_parallelism(self):
        """Optimize pipeline parallelism"""
        # Adjust thread pool sizes based on system capabilities
        pass

    def optimize_resources(self):
        """Optimize resource usage"""
        # Monitor and adjust resource allocation
        pass

def main(args=None):
    """Main function for complete voice-to-action system"""
    rclpy.init(args=args)

    # Create voice-to-action pipeline node
    vta_pipeline = VoiceToActionPipeline()

    # Create optimizer
    optimizer = PipelineOptimizer(vta_pipeline)

    try:
        # Optimize pipeline periodically
        import threading
        def optimization_loop():
            while rclpy.ok():
                optimizer.optimize_pipeline()
                time.sleep(10.0)  # Optimize every 10 seconds

        optimization_thread = threading.Thread(target=optimization_loop, daemon=True)
        optimization_thread.start()

        # Run the pipeline
        rclpy.spin(vta_pipeline)

    except KeyboardInterrupt:
        vta_pipeline.get_logger().info('Shutting down voice-to-action pipeline')
    finally:
        vta_pipeline.destroy_node()
        rclpy.shutdown()

# Example configuration for the voice-to-action system
"""
# voice_to_action_config.yaml
voice_to_action_pipeline:
  ros__parameters:
    # Audio processing parameters
    audio:
      sample_rate: 16000
      chunk_size: 1024
      channels: 1
      vad_threshold: 0.01
      silence_duration: 0.5

    # Pipeline parameters
    pipeline:
      max_command_length: 100
      confidence_threshold: 0.7
      command_timeout: 10.0
      pipeline_retry_limit: 3
      response_timeout: 5.0
      wake_word: "robot"
      enable_confirmation: true

    # Performance parameters
    performance:
      max_latency: 2.0
      target_throughput: 5  # commands per minute
      queue_sizes:
        audio: 20
        recognition: 20
        nlu: 20
        planning: 20
        execution: 20

    # Safety parameters
    safety:
      emergency_stop_keywords: ["stop", "halt", "emergency"]
      maximum_velocity: 0.5
      minimum_distance: 0.5
"""

# Example launch file for the voice-to-action system
"""
# voice_to_action_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file', default='voice_to_action_config.yaml')

    return LaunchDescription([
        # Voice-to-action pipeline
        Node(
            package='voice_to_action',
            executable='voice_to_action_pipeline',
            name='voice_to_action_pipeline',
            parameters=[
                {'use_sim_time': use_sim_time},
                config_file
            ],
            output='screen'
        ),

        # Audio input node
        Node(
            package='audio_system',
            executable='audio_input',
            name='audio_input',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Camera input node (for context-aware commands)
        Node(
            package='image_common',
            executable='camera_node',
            name='camera',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        )
    ])
"""