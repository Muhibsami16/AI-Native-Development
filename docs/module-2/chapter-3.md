---
sidebar_position: 4
title: Chapter 3 - High-Fidelity Interaction with Unity
---

# Chapter 3: High-Fidelity Interaction with Unity

## Learning Objectives

- Set up Unity-ROS bridge for bidirectional communication
- Create Unity scenes for robot visualization
- Implement human-robot interaction scenarios
- Develop Unity scripts for ROS 2 message handling
- Optimize Unity simulations for performance
- Understand deployment considerations for Unity-ROS integration

## Topics Covered

- Unity-ROS bridge setup (ROS# or similar)
- Unity scene configuration for robot visualization
- Bidirectional communication between Unity and ROS 2
- Human-Robot Interaction (HRI) scenarios
- Unity scripts for ROS 2 message handling
- Performance optimization
- Deployment considerations

## Introduction

Unity provides high-fidelity visualization capabilities that complement Gazebo's physics simulation. The Unity-ROS bridge enables bidirectional communication, allowing you to visualize robot behaviors in a realistic 3D environment while maintaining the ROS 2 ecosystem for control and perception.

## Unity-ROS Bridge Setup

The Unity-ROS bridge enables communication between Unity and ROS 2. The most common approach is using ROS# (ROS Sharp) or similar middleware:

### Installing ROS# in Unity

1. Download the ROS# package from the Unity Asset Store or GitHub
2. Import it into your Unity project
3. Configure the ROS connection settings

### Basic ROS Connection Setup

```csharp
using RosSharp.RosBridgeClient;

public class UnityRosConnector : MonoBehaviour
{
    public string rosBridgeServerUrl = "ws://192.168.1.100:9090";
    private RosSocket rosSocket;

    void Start()
    {
        WebSocketNativeClient webSocket = new WebSocketNativeClient(rosBridgeServerUrl);
        rosSocket = new RosSocket(webSocket);

        // Subscribe to ROS topics
        SubscribeToTopics();
    }

    void SubscribeToTopics()
    {
        rosSocket.Subscribe<sensor_msgs.Image>(
            "/robot/camera/image_raw",
            ProcessImageMessage
        );
    }

    void ProcessImageMessage(sensor_msgs.Image imageMsg)
    {
        // Process the image message
    }
}
```

## Unity Scene Configuration

Create a Unity scene that represents your robot and environment:

### Robot Model Setup

1. Import your robot model (or create a simplified version for visualization)
2. Set up the robot hierarchy with appropriate joint configurations
3. Configure materials and textures for realistic appearance

### Scene Lighting and Environment

```csharp
// Example script for environment setup
public class EnvironmentSetup : MonoBehaviour
{
    public Light mainLight;
    public Material robotMaterial;

    void Start()
    {
        // Configure lighting for realistic rendering
        mainLight.type = LightType.Directional;
        mainLight.intensity = 1.0f;

        // Apply materials to robot components
        ApplyRobotMaterials();
    }

    void ApplyRobotMaterials()
    {
        // Apply materials to robot components
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            renderer.material = robotMaterial;
        }
    }
}
```

## Bidirectional Communication

### Publishing Messages from Unity

```csharp
using RosSharp.RosBridgeClient;
using std_msgs;

public class UnityPublisher : MonoBehaviour
{
    private RosSocket rosSocket;

    public void PublishPositionData(float x, float y, float z)
    {
        geometry_msgs.PointStamped pointMsg = new geometry_msgs.PointStamped();
        pointMsg.header.frame_id = "unity_frame";
        pointMsg.header.stamp = new Time();
        pointMsg.point.x = x;
        pointMsg.point.y = y;
        pointMsg.point.z = z;

        rosSocket.Publish("/unity/robot_position", pointMsg);
    }
}
```

### Subscribing to ROS Messages

```csharp
using RosSharp.RosBridgeClient;
using geometry_msgs;

public class UnitySubscriber : MonoBehaviour
{
    private RosSocket rosSocket;
    public GameObject robotObject;

    void Start()
    {
        rosSocket.Subscribe<geometry_msgs.Twist>(
            "/cmd_vel",
            ProcessVelocityCommand
        );
    }

    void ProcessVelocityCommand(Twist twistMsg)
    {
        // Apply velocity to Unity robot object
        Vector3 linearVelocity = new Vector3(
            (float)twistMsg.linear.x,
            (float)twistMsg.linear.y,
            (float)twistMsg.linear.z
        );

        robotObject.transform.Translate(linearVelocity * Time.deltaTime);
    }
}
```

## Human-Robot Interaction (HRI) Scenarios

### Interactive Control Interface

Create an interface for human operators to interact with the robot:

```csharp
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using geometry_msgs;

public class HRIController : MonoBehaviour
{
    public Button forwardButton;
    public Button backwardButton;
    public Button leftButton;
    public Button rightButton;

    private RosSocket rosSocket;

    void Start()
    {
        forwardButton.onClick.AddListener(() => SendCommand(1, 0));
        backwardButton.onClick.AddListener(() => SendCommand(-1, 0));
        leftButton.onClick.AddListener(() => SendCommand(0, 1));
        rightButton.onClick.AddListener(() => SendCommand(0, -1));
    }

    void SendCommand(double linearX, double angularZ)
    {
        Twist twistMsg = new Twist();
        twistMsg.linear.x = linearX;
        twistMsg.angular.z = angularZ;

        rosSocket.Publish("/cmd_vel", twistMsg);
    }
}
```

### VR/AR Integration

For advanced HRI, consider VR/AR integration:

```csharp
#if UNITY_STANDALONE_WIN || UNITY_EDITOR
using UnityEngine.XR;
#endif

public class VRHRIController : MonoBehaviour
{
    void Update()
    {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR
        if (XRSettings.enabled)
        {
            // Handle VR controller input
            HandleVRInput();
        }
#endif
    }

    void HandleVRInput()
    {
        // Map VR controller actions to ROS commands
    }
}
```

## Unity Scripts for ROS 2 Message Handling

### Custom Message Types

Create Unity scripts that handle custom ROS message types:

```csharp
using RosSharp.RosBridgeClient;
using System;

public class CustomMessageHandler : MonoBehaviour
{
    [System.Serializable]
    public class RobotState
    {
        public float position_x;
        public float position_y;
        public float position_z;
        public float orientation_x;
        public float orientation_y;
        public float orientation_z;
        public float orientation_w;
    }

    void ProcessCustomMessage(RobotState state)
    {
        // Update Unity robot position based on ROS message
        transform.position = new Vector3(
            state.position_x,
            state.position_y,
            state.position_z
        );

        transform.rotation = new Quaternion(
            state.orientation_x,
            state.orientation_y,
            state.orientation_z,
            state.orientation_w
        );
    }
}
```

## Performance Optimization

### Level of Detail (LOD) System

Implement LOD to maintain performance with complex scenes:

```csharp
using UnityEngine;

public class RobotLODSystem : MonoBehaviour
{
    public GameObject[] lodLevels;
    public float[] lodDistances;

    void Update()
    {
        float distance = Vector3.Distance(
            Camera.main.transform.position,
            transform.position
        );

        for (int i = 0; i < lodLevels.Length; i++)
        {
            if (distance < lodDistances[i])
            {
                lodLevels[i].SetActive(true);
            }
            else
            {
                lodLevels[i].SetActive(false);
            }
        }
    }
}
```

### Efficient Rendering

```csharp
public class EfficientRenderer : MonoBehaviour
{
    void Start()
    {
        // Optimize rendering settings
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 60;

        // Use occlusion culling
        GetComponent<Renderer>().occlusionPortal = true;
    }
}
```

## Deployment Considerations

### Network Configuration

For distributed Unity-ROS systems, consider network latency:

```csharp
public class NetworkOptimization : MonoBehaviour
{
    public float networkUpdateRate = 30.0f; // Hz
    private float lastUpdateTime;

    void Update()
    {
        if (Time.time - lastUpdateTime > 1.0f / networkUpdateRate)
        {
            SendOptimizedUpdate();
            lastUpdateTime = Time.time;
        }
    }

    void SendOptimizedUpdate()
    {
        // Send only significant changes to reduce network traffic
    }
}
```

### Resource Management

```csharp
public class ResourceManager : MonoBehaviour
{
    void Start()
    {
        // Preload assets to avoid hitches
        Resources.LoadAll("RobotModels");

        // Set up object pooling for frequently created objects
        SetupObjectPooling();
    }

    void SetupObjectPooling()
    {
        // Initialize object pools for sensor visualization, etc.
    }
}
```

## Troubleshooting Unity-ROS Integration

Common issues and solutions:

1. **Connection failures**: Check WebSocket URLs and firewall settings
2. **Message serialization**: Verify message format compatibility
3. **Performance issues**: Optimize scene complexity and update rates
4. **Synchronization problems**: Implement proper time synchronization

Use ROS tools to verify communication:

```bash
# Check if topics are being published from Unity
ros2 topic list
ros2 topic echo /unity/robot_position
```

## Summary

Unity integration provides high-fidelity visualization and interaction capabilities that complement Gazebo's physics simulation. The Unity-ROS bridge enables rich human-robot interaction scenarios while maintaining compatibility with the ROS 2 ecosystem. Proper performance optimization is essential for smooth operation of complex visualization systems.

## Exercises and Assessment

1. Set up a Unity-ROS bridge connection and verify communication between Unity and ROS 2.
2. Create a Unity scene with a robot model and implement basic movement controls.
3. Develop a VR/AR interaction interface for controlling a simulated robot.
4. Optimize a Unity scene for performance when visualizing complex robot behaviors.

## Next Steps

Return to [Chapter 1: Physics Simulation with Gazebo](./chapter-1) or [Chapter 2: Sensor Simulation](./chapter-2) to review previous concepts, or continue with the next module in the series.