---
sidebar_position: 2
---

# Chapter 1: ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Articulate the purpose of ROS 2 in Physical AI and humanoid robotics contexts
- Explain how ROS 2 nodes operate and interact in a distributed system
- Understand the nodes and execution model in detail
- Describe communication overview in ROS 2

## Table of Contents
- [Introduction to ROS 2](#introduction-to-ros-2)
- [Purpose of ROS 2 in Physical AI](#purpose-of-ros-2-in-physical-ai)
- [Nodes and Execution Model](#nodes-and-execution-model)
- [Communication Overview](#communication-overview)
- [Key Concepts](#key-concepts)
- [Exercises](#exercises)

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike traditional operating systems, ROS 2 is not an actual OS but rather a middleware that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Purpose of ROS 2 in Physical AI

ROS 2 serves as the "nervous system" for robots, providing the communication infrastructure that allows different components of a robot to work together. In the context of Physical AI and humanoid robotics, ROS 2:

- Provides a standardized way for different robot components to communicate
- Enables modularity in robot design, allowing components to be developed and tested independently
- Offers a rich ecosystem of tools and packages for common robotics tasks
- Facilitates collaboration between different teams and institutions working on robotics projects
- Supports real-time and safety-critical applications required in humanoid robotics

### Why ROS 2 for Humanoid Robotics?

Humanoid robots present unique challenges that make ROS 2 particularly valuable:

1. **Complexity Management**: Humanoid robots have many degrees of freedom and complex sensor systems that require sophisticated coordination.

2. **Modularity**: Different aspects of humanoid control (locomotion, manipulation, perception) can be developed as separate nodes that communicate through ROS 2.

3. **Simulation Integration**: ROS 2 seamlessly integrates with simulation environments like Gazebo, allowing safe testing before deployment on real hardware.

4. **Real-time Performance**: ROS 2's Quality of Service (QoS) policies allow for real-time performance requirements critical for humanoid stability.

## Nodes and Execution Model

### What is a Node?

In ROS 2, a **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Each node is designed to perform a specific task and can communicate with other nodes through topics, services, or actions.

### Node Characteristics

- **Lightweight**: Nodes are designed to be lightweight and focused on a single task
- **Distributed**: Nodes can run on different machines and still communicate seamlessly
- **Language Agnostic**: Nodes can be written in different programming languages (C++, Python, etc.) and still communicate
- **Composable**: Multiple nodes can be combined to perform complex tasks

### Execution Model

The ROS 2 execution model is based on a distributed architecture:

1. **Node Discovery**: When nodes start, they discover other nodes on the network using the DDS (Data Distribution Service) discovery protocol.

2. **Communication Setup**: Nodes negotiate communication parameters using Quality of Service (QoS) policies.

3. **Message Exchange**: Nodes exchange messages through topics (publish/subscribe), services (request/response), or actions (goal/cancel/result feedback).

4. **Lifecycle Management**: Nodes can have different lifecycle states (unconfigured, inactive, active, finalized) for more robust system management.

### Example Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example Service Server

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example Service Client

```python
import sys
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Overview

ROS 2 provides several communication patterns for nodes to interact:

### Topics (Publish/Subscribe)

- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Many-to-Many**: Multiple publishers can send to the same topic, multiple subscribers can listen to the same topic
- **Data Flow**: Unidirectional from publishers to subscribers

### Services (Request/Response)

- **Synchronous**: Client waits for response from server
- **One-to-One**: One client communicates with one server
- **Data Flow**: Bidirectional (request from client, response from server)

### Actions (Goal/Cancel/Result with Feedback)

- **Long-running**: For tasks that take time to complete
- **Interruptible**: Actions can be canceled
- **Feedback**: Continuous feedback during execution

## Key Concepts

### Packages
A package is the basic building and distribution unit in ROS 2. It contains:
- Source code
- Build instructions
- Dependencies
- Documentation

### Workspaces
A workspace is a directory containing one or more packages. It's where you build and develop your ROS 2 projects.

### Launch Files
Launch files allow you to start multiple nodes at once with predefined configurations.

### Parameters
Parameters allow you to configure nodes without recompiling, making your system more flexible.

## Exercises

1. **Node Creation**: Create a simple ROS 2 node in Python that prints "Hello from ROS 2" to the console every 2 seconds.

2. **Package Structure**: Create a new ROS 2 package called "my_robot_basics" with proper directory structure.

3. **Understanding QoS**: Research and explain the different Quality of Service policies in ROS 2 and when to use each one.

4. **Communication Patterns**: For each communication pattern (topic, service, action), provide an example of when it would be most appropriate in a humanoid robot system.

## Summary

This chapter introduced the fundamental concepts of ROS 2, focusing on its purpose in Physical AI and humanoid robotics. You learned about nodes as the basic building blocks of ROS 2 systems, the execution model that enables distributed computing, and the communication patterns that allow nodes to interact. These concepts form the foundation for more advanced topics in subsequent chapters.