---
sidebar_position: 4
---

# Chapter 3: Humanoid Modeling with URDF

## Learning Objectives

By the end of this chapter, you will be able to:
- Create URDF models with proper link and joint definitions
- Understand URDF structure in detail
- Document links and joints concepts for humanoid robots
- Integrate robot models with ROS 2 and simulation environments
- Include learning objectives and key concepts for Chapter 3
- Add exercises and practice problems for URDF modeling

## Table of Contents
- [Introduction to URDF](#introduction-to-urdf)
- [URDF Structure](#urdf-structure)
- [Links](#links)
- [Joints](#joints)
- [Visual and Collision Elements](#visual-and-collision-elements)
- [Materials and Colors](#materials-and-colors)
- [URDF for Humanoid Robots](#urdf-for-humanoid-robots)
- [Integration with ROS 2 and Simulation](#integration-with-ros-2-and-simulation)
- [Best Practices](#best-practices)
- [Exercises](#exercises)

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, and how they connect to form the robot's structure.

URDF is fundamental to robotics in ROS because it allows:
- Simulation of robots in environments like Gazebo
- Visualization in tools like RViz
- Kinematic analysis and planning
- Collision detection and avoidance

### Why URDF for Humanoid Robots?

Humanoid robots have complex kinematic structures with many degrees of freedom. URDF is particularly valuable for humanoid robots because:
- It provides a clear hierarchical representation of the robot's structure
- It allows for precise definition of joint limits and dynamics
- It supports complex visual and collision geometries
- It integrates seamlessly with ROS 2 tools and simulation environments

## URDF Structure

A URDF file is an XML document with a specific structure. The root element is `<robot>`, which contains all other elements.

### Basic Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links definition -->
  <link name="base_link">
    <!-- Link properties -->
  </link>

  <!-- Joints definition -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <!-- Joint properties -->
  </joint>

  <!-- Additional links and joints -->
</robot>
```

### Robot Element Attributes

The `<robot>` element requires a `name` attribute and can include:
- `version`: URDF specification version
- `xmlns:xacro`: For using Xacro macros

## Links

Links represent rigid bodies in the robot. Each link has physical and visual properties.

### Basic Link Structure

```xml
<link name="link_name">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
</link>
```

### Link Components

1. **Inertial**: Defines mass properties for physics simulation
2. **Visual**: Defines how the link appears in visualization
3. **Collision**: Defines collision geometry for physics simulation

### Inertial Properties

The inertial element defines the mass properties of a link:

```xml
<inertial>
  <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  <mass value="0.1"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

- `origin`: Position and orientation of the center of mass
- `mass`: Mass of the link in kilograms
- `inertia`: Inertia matrix values (for symmetric objects)

### Visual Properties

The visual element defines how a link appears:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://my_robot/meshes/link_name.dae"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Properties

The collision element defines collision geometry:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://my_robot/meshes/link_name_collision.dae"/>
  </geometry>
</collision>
```

## Joints

Joints define how links connect and move relative to each other. URDF supports several joint types:

### Joint Types

1. **Revolute**: Rotational joint with limited range
2. **Continuous**: Rotational joint without limits
3. **Prismatic**: Linear sliding joint with limits
4. **Fixed**: No movement (used to attach links rigidly)
5. **Floating**: 6 DOF joint (rarely used)
6. **Planar**: Movement on a plane (rarely used)

### Joint Structure

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Joint Components

- `parent` and `child`: Links connected by the joint
- `origin`: Position and orientation of the joint relative to the parent
- `axis`: Axis of rotation or translation
- `limit`: For revolute and prismatic joints (min/max values, effort, velocity)
- `dynamics`: Damping and friction coefficients

### Example Joint Types

**Revolute Joint (Elbow)**:
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0.2 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="50" velocity="2"/>
</joint>
```

**Fixed Joint**:
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base"/>
  <child link="sensor_mount"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

## Visual and Collision Elements

### Geometry Types

URDF supports several geometry types:

1. **Box**: Defined by size="x y z"
2. **Cylinder**: Defined by radius and length
3. **Sphere**: Defined by radius
4. **Mesh**: Defined by filename and scale

```xml
<!-- Box geometry -->
<geometry>
  <box size="0.1 0.1 0.1"/>
</geometry>

<!-- Cylinder geometry -->
<geometry>
  <cylinder radius="0.05" length="0.1"/>
</geometry>

<!-- Sphere geometry -->
<geometry>
  <sphere radius="0.05"/>
</geometry>

<!-- Mesh geometry -->
<geometry>
  <mesh filename="package://my_robot/meshes/link.dae" scale="1 1 1"/>
</geometry>
```

### Origin Transform

The origin element specifies position and orientation using:
- `xyz`: Position vector (x, y, z)
- `rpy`: Roll, pitch, yaw angles in radians

## Materials and Colors

Materials define the visual appearance of links:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="green">
  <color rgba="0 1 0 1"/>
</material>

<material name="black">
  <color rgba="0 0 0 1"/>
</material>

<material name="white">
  <color rgba="1 1 1 1"/>
</material>
```

Colors are specified using RGBA values (Red, Green, Blue, Alpha), where each component ranges from 0.0 to 1.0.

## URDF for Humanoid Robots

### Humanoid Robot Structure

Humanoid robots typically have a structure similar to:

```
base_link (pelvis)
├── torso
│   ├── head
│   ├── left_upper_arm
│   │   └── left_forearm
│   │       └── left_hand
│   ├── right_upper_arm
│   │   └── right_forearm
│   │       └── right_hand
│   └── upper_body (if needed)
├── left_thigh
│   └── left_shin
│       └── left_foot
└── right_thigh
    └── right_shin
        └── right_foot
```

### Complete Humanoid Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Pelvis Link -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.15 0.4"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.15 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso Joint -->
  <joint name="torso_joint" type="revolute">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Head Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.05 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Additional links and joints would continue in similar fashion -->

  <!-- Materials -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
</robot>
```

## Integration with ROS 2 and Simulation

### Robot State Publisher

To publish the robot's joint states to TF (Transforms), use the robot_state_publisher package:

```xml
<!-- In a launch file -->
<node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
  <param name="robot_description" value="$(var robot_description)"/>
</node>
```

### Joint State Publisher

For visualizing the robot with interactive joint control:

```xml
<!-- In a launch file -->
<node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"/>
```

### URDF in ROS 2 Launch Files

```python
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF file path
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'simple_humanoid.urdf'
    )

    # Read URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Joint State Publisher node
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    return LaunchDescription([
        rsp_node,
        jsp_node
    ])
```

### Using Xacro

Xacro is a macro tool that simplifies complex URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_size" value="0.2 0.2 0.2" />

  <!-- Macro for creating links -->
  <xacro:macro name="simple_link" params="name size mass xyz:='0 0 0' rpy:='0 0 0'">
    <link name="${name}">
      <inertial>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <mass value="${mass}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_link name="base_link" size="${base_size}" mass="5.0"/>

</robot>
```

## Best Practices

### URDF Design Principles

1. **Start Simple**: Begin with a basic skeleton and add complexity gradually
2. **Use Proper Inertias**: Calculate or estimate mass and inertia properties accurately
3. **Collision vs Visual**: Use simpler geometries for collision to improve performance
4. **Consistent Naming**: Use descriptive, consistent names for links and joints
5. **Hierarchy**: Organize the robot as a tree structure with a single base link

### Performance Considerations

- Use primitive shapes (boxes, cylinders, spheres) for collision geometries when possible
- Limit the number of small collision objects
- Use mesh files only for visual elements when detailed geometry is needed
- Keep the URDF file size reasonable for processing

### Validation

Always validate your URDF files:

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# View the robot structure
urdf_to_graphiz /path/to/robot.urdf
```

## Exercises

1. **Basic URDF**: Create a simple 2-link robot with a revolute joint and visualize it in RViz.

2. **Humanoid Skeleton**: Create a basic humanoid skeleton with at least 10 links (head, torso, arms, legs) and appropriate joints.

3. **Inertial Properties**: Research and calculate proper inertial properties for a simple link, then verify with a URDF validation tool.

4. **Xacro Conversion**: Convert a simple URDF file to use Xacro macros for better maintainability.

5. **Simulation Integration**: Create a launch file that loads your URDF into Gazebo simulation and test the joint movements.

6. **Complex Joint Limits**: Create a robot with different joint types (revolute, prismatic, fixed) and experiment with various limit configurations.

## Summary

This chapter covered the fundamentals of URDF (Unified Robot Description Format) for modeling humanoid robots. You learned about the structure of URDF files, how to define links and joints, and how to integrate URDF models with ROS 2 and simulation environments. The chapter provided examples of humanoid robot structures and best practices for creating accurate and efficient robot models. These skills are essential for simulating and controlling humanoid robots in ROS 2 environments.