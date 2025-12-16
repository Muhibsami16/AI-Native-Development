# Quickstart: Digital Twin Simulation Module

## Prerequisites

- Node.js 18+ installed
- Git installed
- Basic knowledge of ROS 2 concepts (from Module 1)
- Understanding of simulation environments (Gazebo/Unity)

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install Docusaurus dependencies**:
   ```bash
   cd website
   npm install
   ```

3. **Start the development server**:
   ```bash
   npm run start
   ```

4. **Open your browser** to `http://localhost:3000` to view the documentation.

## Setting up Simulation Environments

### Gazebo Environment
1. **Install Gazebo Garden** (or compatible version):
   ```bash
   # For Ubuntu
   sudo apt install ros-humble-gazebo-*
   ```

2. **Verify installation**:
   ```bash
   gazebo --version
   ```

### Unity Environment
1. **Install Unity Hub** and **Unity 2022.3 LTS** or later
2. **Install ROS# package** for Unity-ROS communication
3. **Configure ROS TCP Connector** for communication with ROS 2

## Running Simulation Examples

### Physics Simulation with Gazebo
1. **Launch the simulation environment**:
   ```bash
   ros2 launch <package_name> <gazebo_launch_file>.launch.py
   ```

2. **Interact with the simulation** using ROS 2 commands:
   ```bash
   ros2 topic list
   ros2 topic echo /<robot_name>/imu/data
   ```

### Sensor Simulation
1. **Launch sensor simulation**:
   ```bash
   ros2 launch <package_name> <sensor_launch_file>.launch.py
   ```

2. **Monitor sensor data**:
   ```bash
   ros2 topic echo /<robot_name>/lidar/points
   ros2 topic echo /<robot_name>/camera/depth/image_raw
   ```

### Unity Integration
1. **Launch the Unity bridge**:
   ```bash
   ros2 run unity_robotics_demo unity_publisher
   ```

2. **Start Unity simulation** and verify ROS 2 communication

## Building for Production

1. **Build the static site**:
   ```bash
   npm run build
   ```

2. **Serve the built site locally for testing**:
   ```bash
   npm run serve
   ```

## Deployment to GitHub Pages

1. **Set up GitHub Pages** in your repository settings to use the `gh-pages` branch

2. **Deploy using the provided script**:
   ```bash
   npm run deploy
   ```

This will build the site and push it to the `gh-pages` branch for GitHub Pages hosting.

## Customizing the Documentation

- Modify `website/docusaurus.config.js` to change site metadata, theme, and plugins
- Update `website/src/css/custom.css` to add custom styling
- Add images and other static assets to `website/static/`

## Module 2: Complete Workflow

Now that you have the simulation environments set up, here's the complete workflow for Module 2:

1. **Start with Physics Simulation** (Chapter 1):
   - Launch Gazebo with your robot model
   - Configure physics properties (gravity, mass, friction)
   - Test collision detection and response

2. **Add Sensor Simulation** (Chapter 2):
   - Configure LiDAR, camera, and IMU sensors
   - Process sensor data using ROS 2
   - Implement sensor fusion techniques

3. **Integrate Unity Visualization** (Chapter 3):
   - Set up Unity-ROS bridge
   - Visualize robot behaviors in Unity
   - Implement human-robot interaction scenarios

## Complete Learning Path

The complete learning path across all modules:
- **Module 1**: ROS 2 fundamentals and basic robot control
- **Module 2**: Simulation and digital twin technology (Gazebo & Unity)
- **Module 3**: Advanced robot control and navigation (upcoming)