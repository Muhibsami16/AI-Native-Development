---
sidebar_position: 2
---

# Quickstart Guide

Get started with the ROS2 Humanoid Education module in minutes.

## Prerequisites

Before beginning with this module, ensure you have:
- Basic knowledge of Python programming
- Understanding of fundamental robotics concepts
- A computer capable of running ROS 2 (Ubuntu 22.04 recommended)

## Installation

1. **Install ROS 2 Humble Hawksbill** (or latest LTS version)
   Follow the official installation guide at https://docs.ros.org/en/humble/Installation.html

2. **Set up your ROS 2 workspace**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. **Verify your installation**
   ```bash
   ros2 topic list
   ```

## Getting Started with the Educational Content

1. **Navigate to the documentation**
   The content is organized in modules, with Module 1 covering ROS 2 fundamentals. Start with the [Introduction](./intro) to understand the learning path.

2. **Progress through the modules**
   - Begin with [Module 1: The Robotic Nervous System (ROS 2)](./module-1)
   - Follow the chapters in sequence for optimal learning
   - Complete exercises at the end of each chapter

3. **Try the code examples**
   Each chapter includes practical code examples. Try running them in your ROS 2 environment to reinforce your learning.

## Key Learning Path

The educational content follows this progression:

1. **Chapter 1: ROS 2 Fundamentals** - Learn the core concepts of ROS 2 as middleware for humanoid robotics
2. **Chapter 2: ROS 2 Communication with Python** - Master rclpy nodes and communication patterns
3. **Chapter 3: Humanoid Modeling with URDF** - Understand URDF structure for robot modeling

## Running Examples

To run the Python examples from the documentation:

1. Create a new ROS 2 package:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_robot_examples
   cd my_robot_examples
   ```

2. Add the example code to your Python files

3. Build and run:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_examples
   source install/setup.bash
   ros2 run my_robot_examples example_node
   ```

## Next Steps

After completing Module 1:
- Explore additional ROS 2 tutorials at https://docs.ros.org/en/humble/Tutorials.html
- Try building simple robots with URDF
- Experiment with ROS 2 simulation tools like Gazebo
- Join the ROS community forums for additional support and learning resources

## Troubleshooting

- **Can't find ROS 2 commands?** Make sure you've sourced your ROS 2 installation: `source /opt/ros/humble/setup.bash`
- **Python packages not found?** Ensure you've created and sourced your workspace properly
- **Need help?** Check the ROS 2 documentation or visit the ROS answers forum