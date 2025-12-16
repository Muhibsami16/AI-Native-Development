---
sidebar_position: 3
---

# Chapter 2: ROS 2 Communication with Python

## Learning Objectives

By the end of this chapter, you will be able to:
- Create functional rclpy nodes that communicate via topics and services
- Implement communication patterns using topics and services
- Connect Python AI agents to controllers through ROS 2
- Understand rclpy nodes creation and management
- Document topics and services implementation in detail

## Table of Contents
- [Introduction to rclpy](#introduction-to-rclpy)
- [Creating rclpy Nodes](#creating-rclpy-nodes)
- [Topics and Publishers](#topics-and-publishers)
- [Topics and Subscribers](#topics-and-subscribers)
- [Services](#services)
- [Connecting Python AI Agents to Controllers](#connecting-python-ai-agents-to-controllers)
- [Best Practices](#best-practices)
- [Exercises](#exercises)

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides Python APIs that are conceptually similar to rcl (the C client library) and the ROS 2 concept documentation. rclpy provides the interface between your Python code and the ROS 2 middleware (RMW - ROS Middleware).

### Why rclpy?

rclpy is essential for Python developers working with ROS 2 because:
- It provides native Python bindings to the ROS 2 ecosystem
- It allows Python AI agents to communicate with ROS 2 systems
- It supports all ROS 2 communication patterns (topics, services, actions)
- It's well-integrated with the Python ecosystem and popular libraries

## Creating rclpy Nodes

### Basic Node Structure

A basic rclpy node follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here
        self.get_logger().info('Node has been initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

When creating rclpy nodes, it's important to understand the lifecycle:

1. **Initialization**: The node is created and initialized with a unique name
2. **Configuration**: Publishers, subscribers, services, and timers are set up
3. **Execution**: The node runs and processes callbacks
4. **Shutdown**: The node is properly cleaned up

### Node Management

Proper node management includes:
- Resource cleanup in destroy_node()
- Proper exception handling
- Graceful shutdown procedures
- Logging for debugging and monitoring

## Topics and Publishers

### Publishers

Publishers send messages to topics. Here's how to create a publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS) Settings

When creating publishers, you can specify QoS settings:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a custom QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Use the QoS profile when creating a publisher
publisher = self.create_publisher(String, 'topic', qos_profile)
```

## Topics and Subscribers

### Subscribers

Subscribers receive messages from topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Message Filters

For more complex subscription patterns, you can use message filters and callbacks with different configurations:

```python
def listener_callback_with_header(self, msg):
    """Callback that includes header information"""
    timestamp = msg.header.stamp
    self.get_logger().info(f'Received message at: {timestamp}')

def listener_callback_async(self, msg):
    """Async callback for handling messages"""
    # Perform async operations here
    self.get_logger().info(f'Processing message asynchronously: {msg.data}')
```

## Services

### Service Servers

Services provide request-response communication:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Clients

Service clients make requests to service servers:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = MinimalClientAsync()

    # Send request
    future = client.send_request(1, 2)

    # Wait for response
    rclpy.spin_until_future_complete(client, future)
    response = future.result()

    client.get_logger().info(f'Result: {response.sum}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting Python AI Agents to Controllers

### AI Agent Integration Pattern

A common pattern for connecting Python AI agents to ROS 2 controllers:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Subscribe to sensor data
        self.sensor_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.sensor_callback,
            10
        )

        # Publish control commands
        self.command_publisher = self.create_publisher(
            Float32MultiArray,
            '/joint_commands',
            10
        )

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

        # Initialize AI model or controller
        self.initialize_ai_agent()

    def initialize_ai_agent(self):
        """Initialize the AI agent or controller"""
        self.get_logger().info('Initializing AI agent...')
        # Add your AI agent initialization code here
        # e.g., load neural network model, set up controller parameters, etc.

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        # Convert ROS message to format suitable for AI agent
        sensor_data = np.array(msg.position)  # Example: joint positions
        self.process_sensor_data(sensor_data)

    def process_sensor_data(self, sensor_data):
        """Process sensor data for AI agent"""
        # Store sensor data for AI processing
        self.current_sensor_data = sensor_data

    def ai_processing_callback(self):
        """Main AI processing loop"""
        if hasattr(self, 'current_sensor_data'):
            # Run AI agent to compute control commands
            control_commands = self.compute_control_commands(self.current_sensor_data)

            # Publish control commands
            msg = Float32MultiArray()
            msg.data = control_commands.tolist()
            self.command_publisher.publish(msg)

    def compute_control_commands(self, sensor_data):
        """Compute control commands using AI agent"""
        # This is where your AI logic would go
        # For example, run neural network inference
        # or execute a control policy
        commands = np.zeros(len(sensor_data))  # Placeholder
        return commands

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AIAgentNode()
    rclpy.spin(ai_agent_node)
    ai_agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Controller Integration Example

Example of connecting a Python-based controller to a ROS 2 robot:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Robot state
        self.current_pose = None
        self.target_pose = [1.0, 1.0]  # Target x, y coordinates

    def odom_callback(self, msg):
        """Update robot's current pose from odometry"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Main control loop"""
        if self.current_pose is not None:
            # Simple proportional controller to reach target
            dx = self.target_pose[0] - self.current_pose['x']
            dy = self.target_pose[1] - self.current_pose['y']

            # Calculate distance and angle to target
            distance = math.sqrt(dx*dx + dy*dy)
            target_angle = math.atan2(dy, dx)

            # Create twist message
            cmd_vel = Twist()

            # Proportional control for linear and angular velocity
            if distance > 0.1:  # If not close to target
                cmd_vel.linear.x = min(0.5, distance * 0.5)  # Max 0.5 m/s
                cmd_vel.angular.z = (target_angle - self.current_pose['theta']) * 1.0
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0

            # Publish command
            self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### Error Handling

Always include proper error handling in your rclpy nodes:

```python
def safe_publish(self, publisher, msg):
    """Safely publish a message with error handling"""
    try:
        publisher.publish(msg)
    except Exception as e:
        self.get_logger().error(f'Failed to publish message: {e}')
```

### Resource Management

Properly manage resources in your nodes:

```python
def destroy_node(self):
    """Clean up resources before node destruction"""
    # Cancel timers
    if hasattr(self, 'timer') and self.timer is not None:
        self.timer.cancel()

    # Destroy publishers/subscribers
    # (Automatically handled by rclpy, but good to be explicit)

    # Call parent method
    super().destroy_node()
```

### Logging

Use appropriate logging levels:

```python
# Informational messages
self.get_logger().info('Node initialized successfully')

# Warning messages
self.get_logger().warn('Parameter value is at boundary')

# Error messages
self.get_logger().error('Failed to connect to hardware')
```

## Exercises

1. **Basic Publisher/Subscriber**: Create a publisher that sends temperature readings and a subscriber that logs these readings to the console.

2. **Custom Service**: Create a service that accepts two poses and returns the distance between them.

3. **AI Agent Integration**: Create a simple AI agent (e.g., a neural network using PyTorch or TensorFlow) that takes sensor inputs and outputs motor commands through ROS 2 topics.

4. **Controller Implementation**: Implement a PID controller for a simulated robot that follows a specific trajectory.

5. **QoS Configuration**: Experiment with different Quality of Service settings and observe their impact on communication reliability and performance.

## Summary

This chapter covered the fundamentals of ROS 2 communication using Python (rclpy). You learned how to create nodes, implement publishers and subscribers for topic-based communication, and use services for request-response patterns. The chapter also provided examples of how to connect Python AI agents to ROS 2 controllers, which is crucial for humanoid robotics applications. These communication patterns form the backbone of distributed robotic systems.