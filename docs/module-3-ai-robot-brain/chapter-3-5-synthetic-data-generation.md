---
id: chapter-3-5-synthetic-data-generation
title: Synthetic Data Generation
sidebar_label: Synthetic Data Generation
---

# Synthetic Data Generation

## Goal
Learn to generate high-quality synthetic data using NVIDIA Isaac Sim for training AI models in humanoid robotics applications, reducing the need for real-world data collection.

## Learning Objectives
- Understand the principles and benefits of synthetic data generation
- Configure Isaac Sim for synthetic data creation
- Implement domain randomization techniques for robust AI models
- Generate diverse datasets for perception and control tasks
- Validate synthetic-to-real transfer performance
- Optimize synthetic data generation pipelines

## Overview
Synthetic data generation is a critical capability for developing AI-powered humanoid robots, as it allows for the creation of large, diverse, and perfectly labeled datasets without the time and cost constraints of real-world data collection. NVIDIA Isaac Sim provides powerful tools for generating photorealistic synthetic data with accurate physics and lighting, enabling the training of robust AI models that can transfer effectively to real robots. This approach is particularly valuable for humanoid robots, which require extensive training on complex behaviors and interactions.

## Key Concepts
- **Synthetic Data**: Computer-generated data that mimics real-world observations
- **Domain Randomization**: Varying simulation parameters to improve model robustness
- **Physically-Based Rendering**: Accurate simulation of lighting and materials
- **Sensor Simulation**: Modeling camera, LiDAR, and other sensor characteristics
- **Data Annotation**: Automatic labeling of synthetic datasets
- **Reality Gap**: Differences between synthetic and real data
- **Transfer Learning**: Adapting synthetic-trained models for real-world use

## Step-by-Step Breakdown
1. **Synthetic Data Fundamentals**
   - Understand the benefits and limitations of synthetic data
   - Learn about different types of synthetic data (images, point clouds, etc.)
   - Study domain randomization techniques
   - Explore the synthetic-to-real transfer problem

2. **Isaac Sim Configuration**
   - Set up Isaac Sim for data generation
   - Configure rendering and physics parameters
   - Design diverse environments and scenarios
   - Optimize simulation performance

3. **Domain Randomization Implementation**
   - Randomize lighting conditions (intensity, color, position)
   - Vary material properties (textures, colors, reflectance)
   - Change camera parameters (position, angle, noise)
   - Modify environmental conditions (weather, occlusions)

4. **Data Annotation Pipeline**
   - Implement automatic labeling systems
   - Generate ground truth data (depth, segmentation, bounding boxes)
   - Create metadata for each data sample
   - Validate annotation accuracy

5. **Dataset Generation**
   - Design diverse scenarios for humanoid robot tasks
   - Generate large-scale datasets efficiently
   - Implement data augmentation techniques
   - Organize data for different AI tasks

6. **Validation and Transfer**
   - Test synthetic-trained models on real data
   - Measure transfer performance metrics
   - Identify and address reality gaps
   - Optimize simulation parameters for better transfer

## Code Examples
```python
# Example synthetic data generation pipeline using Isaac Sim
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import cv2
import json
import os
from PIL import Image
import random
import datetime
from typing import Dict, List, Tuple, Any

class IsaacSyntheticDataGenerator:
    def __init__(self, output_dir: str = "synthetic_data", num_samples: int = 1000):
        """
        Initialize synthetic data generator for humanoid robot perception

        Args:
            output_dir: Directory to save generated data
            num_samples: Number of samples to generate
        """
        self.output_dir = output_dir
        self.num_samples = num_samples
        self.world = None
        self.camera = None
        self.robot = None
        self.objects = []

        # Create output directories
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "metadata"), exist_ok=True)

        # Domain randomization parameters
        self.domain_params = {
            'lighting': {
                'intensity_range': (100, 1000),
                'color_temperature_range': (3000, 8000),
                'direction_range': ((-1, -1, -1), (1, 1, 1))
            },
            'materials': {
                'colors': [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)],
                'textures': ['rough', 'smooth', 'metallic', 'matte'],
                'reflectance_range': (0.0, 1.0)
            },
            'camera': {
                'position_range': ((-2, -2, 1), (2, 2, 3)),
                'angle_range': ((-0.5, -0.5, -1.5), (0.5, 0.5, 1.5)),
                'noise_level_range': (0.0, 0.1)
            },
            'environment': {
                'floor_textures': ['wood', 'tile', 'carpet', 'concrete'],
                'background_objects': ['table', 'chair', 'box', 'plant']
            }
        }

        # Initialize world
        self._setup_world()

        self.get_logger().info(f'Synthetic data generator initialized. Output dir: {output_dir}')

    def _setup_world(self):
        """Setup Isaac Sim world for data generation"""
        # Create world
        self.world = World(stage_units_in_meters=1.0)

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add humanoid robot
        robot_prim_path = "/World/Robot"
        add_reference_to_stage(
            usd_path="path/to/humanoid_robot.usd",  # Replace with actual path
            prim_path=robot_prim_path
        )
        self.robot = self.world.scene.add_robot(
            Robot(prim_path=robot_prim_path, name="humanoid_robot")
        )

        # Add camera
        camera_prim_path = "/World/Camera"
        self.camera = self.world.scene.add(Camera(
            prim_path=camera_prim_path,
            frequency=30,
            resolution=(640, 480)
        ))

        # Set initial camera position
        self.camera.set_world_pose(position=np.array([1.0, 0.0, 1.5]))

        # Play the world
        self.world.reset()
        self.world.play()

    def randomize_lighting(self):
        """Apply random lighting conditions"""
        # In a real implementation, this would modify light sources in the scene
        # For demonstration, we'll just log the changes
        intensity = random.uniform(*self.domain_params['lighting']['intensity_range'])
        color_temp = random.uniform(*self.domain_params['lighting']['color_temperature_range'])

        self.get_logger().debug(f'Applied random lighting: intensity={intensity}, color_temp={color_temp}')

    def randomize_materials(self):
        """Apply random material properties"""
        # Randomize object materials
        color = random.choice(self.domain_params['materials']['colors'])
        texture = random.choice(self.domain_params['materials']['textures'])
        reflectance = random.uniform(*self.domain_params['materials']['reflectance_range'])

        self.get_logger().debug(f'Applied random materials: color={color}, texture={texture}, reflectance={reflectance}')

    def randomize_camera(self):
        """Apply random camera parameters"""
        # Randomize camera position and orientation
        pos_min, pos_max = self.domain_params['camera']['position_range']
        position = [
            random.uniform(pos_min[0], pos_max[0]),
            random.uniform(pos_min[1], pos_max[1]),
            random.uniform(pos_min[2], pos_max[2])
        ]

        angle_min, angle_max = self.domain_params['camera']['angle_range']
        orientation = [
            random.uniform(angle_min[0], angle_max[0]),
            random.uniform(angle_min[1], angle_max[1]),
            random.uniform(angle_min[2], angle_max[2])
        ]

        # Apply noise
        noise_level = random.uniform(*self.domain_params['camera']['noise_level_range'])

        self.camera.set_world_pose(position=np.array(position))

        self.get_logger().debug(f'Applied random camera: pos={position}, noise={noise_level}')

    def randomize_environment(self):
        """Apply random environmental conditions"""
        floor_texture = random.choice(self.domain_params['environment']['floor_textures'])
        background_obj = random.choice(self.domain_params['environment']['background_objects'])

        self.get_logger().debug(f'Applied random environment: floor={floor_texture}, bg_obj={background_obj}')

    def generate_sample(self, sample_id: int) -> Dict[str, Any]:
        """Generate a single synthetic data sample"""
        try:
            # Apply domain randomization
            self.randomize_lighting()
            self.randomize_materials()
            self.randomize_camera()
            self.randomize_environment()

            # Step the simulation to apply changes
            self.world.step(render=True)

            # Capture RGB image
            rgb_image = self.camera.get_rgb()

            # Capture depth image
            depth_image = self.camera.get_depth()

            # Capture segmentation mask (if available)
            try:
                segmentation = self.camera.get_segmentation()
            except:
                segmentation = np.zeros_like(rgb_image[:, :, 0])

            # Create metadata
            metadata = {
                'sample_id': sample_id,
                'timestamp': datetime.datetime.now().isoformat(),
                'camera_pose': self.camera.get_world_pose(),
                'domain_params': {
                    'lighting_intensity': random.uniform(*self.domain_params['lighting']['intensity_range']),
                    'material_reflectance': random.uniform(*self.domain_params['materials']['reflectance_range']),
                    'camera_noise': random.uniform(*self.domain_params['camera']['noise_level_range'])
                }
            }

            # Save images
            rgb_path = os.path.join(self.output_dir, "images", f"rgb_{sample_id:06d}.png")
            depth_path = os.path.join(self.output_dir, "images", f"depth_{sample_id:06d}.png")
            seg_path = os.path.join(self.output_dir, "labels", f"seg_{sample_id:06d}.png")
            meta_path = os.path.join(self.output_dir, "metadata", f"meta_{sample_id:06d}.json")

            # Save RGB image
            Image.fromarray(rgb_image).save(rgb_path)

            # Save depth image (normalize for 16-bit PNG)
            depth_normalized = ((depth_image - depth_image.min()) / (depth_image.max() - depth_image.min()) * 65535).astype(np.uint16)
            Image.fromarray(depth_normalized).save(depth_path)

            # Save segmentation mask
            Image.fromarray(segmentation).save(seg_path)

            # Save metadata
            with open(meta_path, 'w') as f:
                json.dump(metadata, f, indent=2)

            return {
                'rgb_path': rgb_path,
                'depth_path': depth_path,
                'seg_path': seg_path,
                'meta_path': meta_path,
                'metadata': metadata
            }

        except Exception as e:
            self.get_logger().error(f'Error generating sample {sample_id}: {e}')
            return None

    def generate_dataset(self):
        """Generate the complete synthetic dataset"""
        self.get_logger().info(f'Generating {self.num_samples} synthetic samples...')

        generated_samples = []

        for i in range(self.num_samples):
            sample_data = self.generate_sample(i)
            if sample_data:
                generated_samples.append(sample_data)

                if (i + 1) % 100 == 0:
                    self.get_logger().info(f'Generated {i + 1}/{self.num_samples} samples')

        # Create dataset manifest
        manifest = {
            'dataset_name': 'Humanoid_Robot_Synthetic_Data',
            'total_samples': len(generated_samples),
            'generated_at': datetime.datetime.now().isoformat(),
            'domain_parameters': self.domain_params,
            'sample_paths': [s['meta_path'] for s in generated_samples if s]
        }

        manifest_path = os.path.join(self.output_dir, 'dataset_manifest.json')
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)

        self.get_logger().info(f'Dataset generation complete. Total samples: {len(generated_samples)}')
        return generated_samples

# ROS 2 node for synthetic data generation workflow
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import queue

class SyntheticDataROSNode(Node):
    def __init__(self):
        super().__init__('synthetic_data_ros_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Publishers for synthetic data
        self.rgb_pub = self.create_publisher(Image, '/synthetic/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/synthetic/depth', 10)
        self.status_pub = self.create_publisher(String, '/synthetic/status', 10)

        # Subscribers for control
        self.start_sub = self.create_subscription(
            String, '/synthetic/start_generation', self.start_generation_callback, 10
        )
        self.progress_sub = self.create_subscription(
            Int32, '/synthetic/progress', self.progress_callback, 10
        )

        # Generation parameters
        self.generation_params = {
            'num_samples': 1000,
            'output_dir': '/tmp/synthetic_data',
            'domain_randomization': True
        }

        # Generation control
        self.generation_active = False
        self.generation_thread = None

        # Data queue for publishing
        self.data_queue = queue.Queue(maxsize=10)

        self.get_logger().info('Synthetic Data ROS Node initialized')

    def start_generation_callback(self, msg):
        """Start synthetic data generation"""
        try:
            params = json.loads(msg.data) if msg.data else {}
            self.generation_params.update(params)

            if not self.generation_active:
                self.generation_active = True
                self.generation_thread = threading.Thread(target=self._run_generation)
                self.generation_thread.start()

                status_msg = String()
                status_msg.data = f'Started generation: {self.generation_params["num_samples"]} samples'
                self.status_pub.publish(status_msg)
            else:
                self.get_logger().warn('Generation already active')

        except Exception as e:
            self.get_logger().error(f'Error starting generation: {e}')

    def _run_generation(self):
        """Run the actual generation in a separate thread"""
        try:
            # Create synthetic data generator
            generator = IsaacSyntheticDataGenerator(
                output_dir=self.generation_params['output_dir'],
                num_samples=self.generation_params['num_samples']
            )

            # Generate dataset
            samples = generator.generate_dataset()

            # Update status
            status_msg = String()
            status_msg.data = f'Generation complete: {len(samples)} samples'
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Generation error: {e}')
        finally:
            self.generation_active = False

    def progress_callback(self, msg):
        """Handle progress updates"""
        progress = msg.data
        self.get_logger().info(f'Generation progress: {progress}%')

    def publish_synthetic_data(self, rgb_image, depth_image):
        """Publish synthetic data to ROS topics"""
        try:
            # Convert numpy arrays to ROS Image messages
            rgb_ros = self.cv_bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            depth_ros = self.cv_bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")

            # Add timestamps
            current_time = self.get_clock().now().to_msg()
            rgb_ros.header.stamp = current_time
            depth_ros.header.stamp = current_time
            rgb_ros.header.frame_id = 'synthetic_camera'
            depth_ros.header.frame_id = 'synthetic_camera'

            # Publish images
            self.rgb_pub.publish(rgb_ros)
            self.depth_pub.publish(depth_ros)

        except Exception as e:
            self.get_logger().error(f'Error publishing synthetic data: {e}')

def main(args=None):
    """Main function for synthetic data generation"""
    # In a real Isaac Sim extension, this would be integrated differently
    # This example shows how the ROS integration component would work

    rclpy.init(args=args)
    synthetic_node = SyntheticDataROSNode()

    try:
        # Run the ROS node
        rclpy.spin(synthetic_node)
    except KeyboardInterrupt:
        synthetic_node.get_logger().info('Shutting down synthetic data node')
    finally:
        synthetic_node.destroy_node()
        rclpy.shutdown()

# Isaac Sim Extension for Synthetic Data Generation
class IsaacSyntheticDataExtension:
    def __init__(self):
        self._world = None
        self._data_generator = None
        self._generation_active = False

    def on_startup(self, ext_id):
        """Called when the extension is started"""
        carb.log_info("[isaac_synthetic_data] Isaac Synthetic Data Generator starting...")

    def on_shutdown(self):
        """Called when the extension is shut down"""
        carb.log_info("[isaac_synthetic_data] Isaac Synthetic Data Generator shutting down...")

        if self._generation_active:
            self.stop_generation()

    def start_generation(self, output_dir: str, num_samples: int, domain_params: Dict):
        """Start synthetic data generation"""
        try:
            self._data_generator = IsaacSyntheticDataGenerator(
                output_dir=output_dir,
                num_samples=num_samples
            )

            # Update domain parameters
            self._data_generator.domain_params.update(domain_params)

            # Start generation in a separate thread
            import threading
            gen_thread = threading.Thread(target=self._data_generator.generate_dataset)
            gen_thread.start()

            self._generation_active = True
            carb.log_info(f"[isaac_synthetic_data] Started generation: {num_samples} samples to {output_dir}")

        except Exception as e:
            carb.log_error(f"[isaac_synthetic_data] Error starting generation: {e}")

    def stop_generation(self):
        """Stop synthetic data generation"""
        self._generation_active = False
        carb.log_info("[isaac_synthetic_data] Stopped generation")
```

## Diagrams
```
Synthetic Data Generation Pipeline:

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Domain         │    │  Isaac Sim      │    │  Data           │
│  Randomization  │───►│  Simulation     │───►│  Collection &   │
│  (Lighting,     │    │  (Physics,      │    │  Annotation     │
│   Materials,     │    │   Rendering)    │    │                 │
│   Camera, etc.)  │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Isaac ROS Integration                        │
│                    (ROS 2 Bridge for Control)                   │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                       ┌─────────────────┐
                       │  Dataset        │
                       │  (Images,       │
                       │   Labels, Meta)  │
                       └─────────────────┘

Domain Randomization Strategy:

Base Scene ──┬──► Lighting Randomization ──► Diverse Illumination
             ├──► Material Randomization ──► Varied Textures/Colors
             ├──► Camera Randomization ────► Different Viewpoints
             └──► Environment Randomization ─► Multiple Scenarios

Synthetic-to-Real Transfer:

High Domain   ──►  Robust AI Models  ──►  Better Real-World
Randomization         (Generalized)         Performance
```

## Case Study
The NVIDIA Isaac team has demonstrated the effectiveness of synthetic data generation for training humanoid robot perception systems. In one example, they used Isaac Sim to generate over 100,000 synthetic images of a humanoid robot in various poses and environments, with randomized lighting, materials, and camera parameters. The synthetic-trained models achieved comparable performance to real-data-trained models when tested on actual robot hardware, while reducing data collection time from months to hours. This approach has enabled faster development cycles and more robust AI models for humanoid robots operating in diverse environments.

## References
- [Isaac Sim Synthetic Data Generation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_advanced_synthetic_data.html)
- [Domain Randomization in Robotics](https://arxiv.org/abs/1703.06907)
- [Synthetic Data for Robotics Applications](https://developer.nvidia.com/blog/generating-synthetic-data-for-ai-training-with-isaac-sim/)

## Review Questions
1. What are the main benefits of synthetic data generation for robotics?
2. How does domain randomization improve model robustness?
3. What are the challenges in synthetic-to-real transfer?

## Practical Exercises
1. Configure Isaac Sim for synthetic data generation
2. Implement domain randomization for lighting conditions
3. Generate a synthetic dataset for object detection
4. Compare synthetic-trained models with real-data-trained models