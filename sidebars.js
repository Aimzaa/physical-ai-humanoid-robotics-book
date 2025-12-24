// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  book: [
    'index',
    {
      type: 'category',
      label: 'Module 1 — The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-robotic-nervous-system/chapter-1-1-introduction-ros2',
        'module-1-robotic-nervous-system/chapter-1-2-nodes-topics-services',
        'module-1-robotic-nervous-system/chapter-1-3-urdf-humanoid-robots',
        'module-1-robotic-nervous-system/chapter-1-4-python-rclpy-programming',
        'module-1-robotic-nervous-system/chapter-1-5-launch-files-parameters',
        'module-1-robotic-nervous-system/chapter-1-6-sensor-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 2 — The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/chapter-2-1-physics-simulation',
        'module-2-digital-twin/chapter-2-2-gazebo-robot-worlds',
        'module-2-digital-twin/chapter-2-3-unity-visualization',
        'module-2-digital-twin/chapter-2-4-sensor-simulation',
        'module-2-digital-twin/chapter-2-5-collision-detection',
        'module-2-digital-twin/chapter-2-6-sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Module 3 — The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-ai-robot-brain/chapter-3-1-nvidia-isaac-platform',
        'module-3-ai-robot-brain/chapter-3-2-isaac-sim-ros',
        'module-3-ai-robot-brain/chapter-3-3-vslam-implementation',
        'module-3-ai-robot-brain/chapter-3-4-perception-navigation',
        'module-3-ai-robot-brain/chapter-3-5-synthetic-data-generation',
        'module-3-ai-robot-brain/chapter-3-6-nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 — Vision-Language-Action (VLA)',
      items: [
        'module-4-vision-language-action/chapter-4-1-vla-systems',
        'module-4-vision-language-action/chapter-4-2-whisper-voice-commands',
        'module-4-vision-language-action/chapter-4-3-llm-cognitive-planning',
        'module-4-vision-language-action/chapter-4-4-ros2-action-execution',
        'module-4-vision-language-action/chapter-4-5-multimodal-robotics',
        'module-4-vision-language-action/chapter-4-6-voice-to-action-pipeline',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project/autonomous-humanoid-implementation',
        'capstone-project/integration-concepts',
        'capstone-project/complete-voice-to-action',
        'capstone-project/integrated-system-diagram',
      ],
    },
    {
      type: 'doc',
      id: 'references',
    },
  ],
};

module.exports = sidebars;