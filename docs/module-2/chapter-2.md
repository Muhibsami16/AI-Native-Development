---
sidebar_position: 3
title: Chapter 2 - Sensor Simulation
---

# Chapter 2: Sensor Simulation

## Learning Objectives

- Understand how to simulate various robot sensors in Gazebo
- Configure LiDAR, depth cameras, and IMU sensors
- Process simulated sensor data in ROS 2
- Generate and process point cloud data
- Apply sensor fusion techniques in simulation

## Topics Covered

- LiDAR sensor simulation setup
- Depth camera simulation
- IMU sensor configuration
- Sensor data processing in ROS 2
- Point cloud generation and processing
- Sensor fusion techniques
- Troubleshooting sensor simulation

## Introduction

Sensor simulation is crucial for developing and testing robot perception algorithms. Gazebo provides realistic simulation of various sensor types that output data in standard ROS 2 message formats, making it ideal for educational purposes and algorithm development.

## LiDAR Sensor Simulation

LiDAR sensors are essential for navigation and mapping. In Gazebo, you can simulate a 2D or 3D LiDAR using the libgazebo_ros_ray_sensor plugin:

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Camera Simulation

Depth cameras provide both color and depth information. Here's how to simulate a depth camera in Gazebo:

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
        <remapping>depth/image_raw:=camera/depth/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Sensor Configuration

Inertial Measurement Units (IMUs) provide orientation and acceleration data:

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Processing Sensor Data in ROS 2

Here's an example of how to process LiDAR data in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.lidar_callback,
            10)

    def lidar_callback(self, msg):
        # Process LiDAR data
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Min distance: {min_distance}')

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()
    rclpy.spin(sensor_processor)
    sensor_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Point Cloud Generation and Processing

Depth cameras generate point clouds that can be processed using the PCL library:

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudProcessor {
public:
    void depthToPointCloud(const sensor_msgs::msg::Image::SharedPtr depth_image) {
        // Convert depth image to point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Process the conversion...
    }
};
```

## Sensor Fusion Techniques

Sensor fusion combines data from multiple sensors to improve perception accuracy:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class SensorFusion:
    def __init__(self):
        self.imu_orientation = None
        self.camera_position = None

    def fuse_data(self):
        # Combine IMU and camera data for better pose estimation
        if self.imu_orientation and self.camera_position:
            # Apply sensor fusion algorithm
            fused_pose = self.complementary_filter()
            return fused_pose
```

## Troubleshooting Sensor Simulation

Common issues with sensor simulation:

1. **Empty sensor data**: Check sensor configuration and update rates
2. **Incorrect frame IDs**: Verify TF transforms between sensor and robot base
3. **Noisy data**: Adjust noise parameters in sensor configuration
4. **Performance issues**: Reduce update rates or simplify sensor models

Use `ros2 topic echo` to verify sensor data is being published correctly:

```bash
ros2 topic echo /robot/scan sensor_msgs/msg/LaserScan
```

## Summary

Sensor simulation in Gazebo provides realistic data for developing and testing robot perception algorithms. Proper configuration of LiDAR, cameras, and IMUs is essential for accurate simulation results.

## Exercises and Assessment

1. Configure a LiDAR sensor on a robot model and visualize its scan data in RViz.
2. Set up a depth camera simulation and process the point cloud data using PCL.
3. Implement a sensor fusion algorithm combining IMU and camera data.
4. Debug common sensor simulation issues like empty data or incorrect frame IDs.

## Next Steps

Continue to [Chapter 3: High-Fidelity Interaction with Unity](./chapter-3) to explore advanced visualization techniques, or return to [Chapter 1: Physics Simulation with Gazebo](./chapter-1) if you need to review physics concepts.