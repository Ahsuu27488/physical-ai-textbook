---
title: Module 1 - The Robotic Nervous System (ROS 2)
sidebar_position: 2
id: intro
---

# Module 1: The Robotic Nervous System (ROS 2)

## Focus: Middleware for robot control.

This module introduces the Robot Operating System (ROS 2), which serves as the nervous system for robotic applications. ROS 2 provides the communication infrastructure that allows different components of a robot to work together seamlessly.

## Learning Objectives

By the end of this module, students will be able to:

1. Understand ROS 2 architecture and core concepts
2. Create and manage ROS 2 nodes, topics, and services
3. Build ROS 2 packages with Python
4. Use launch files and parameter management
5. Bridge Python AI agents to ROS controllers using rclpy
6. Understand URDF (Unified Robot Description Format) for humanoids

## Table of Contents

- [Weeks 1-2: Introduction to Physical AI](../week-01-02/intro.md)
- [Weeks 3-5: ROS 2 Fundamentals](../week-03-05/intro.md)

## Key Topics

### ROS 2 Architecture
- **DDS (Data Distribution Service)**: The underlying communication layer
- **Nodes**: Independent processes that perform computation
- **Topics**: Asynchronous publish/subscribe communication
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous communication with feedback and goal management

### Core Communication Patterns
- **Publisher-Subscriber Pattern**: Broadcasting sensor data and robot state
- **Request-Response Pattern**: Querying robot status or requesting specific actions
- **Action Pattern**: Long-running tasks with feedback (navigation, manipulation)

### ROS 2 Tools
- **rclpy**: Python client library for ROS 2
- **rclcpp**: C++ client library for ROS 2
- **ros2 command line tools**: For introspection, debugging, and system management
- **Launch files**: For starting multiple nodes at once
- **Parameters**: For configuring nodes at runtime

### Robot Description
- **URDF (Unified Robot Description Format)**: Describing robot kinematics and visual properties
- **XACRO**: XML macros for more complex robot descriptions
- **TF (Transforms)**: Managing coordinate frames and transformations

## Module Overview

ROS 2 is not an operating system but rather a middleware framework that provides services designed for complex robotic applications. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

The key difference between ROS and ROS 2 is that ROS 2 is designed to be production-ready, with features like:

- **Real-time support**: For time-critical applications
- **Multi-robot systems**: Managing multiple robots simultaneously
- **Security**: Authentication, authorization, and encryption
- **Improved build system**: Using CMake and colcon
- **Better cross-platform support**: Including Windows, macOS, and Linux

### Why ROS 2 for Physical AI?

ROS 2 serves as the "nervous system" of Physical AI systems by providing:

1. **Decentralized Architecture**: Components can be distributed across multiple computers
2. **Language Independence**: Nodes can be written in different programming languages
3. **Hardware Abstraction**: Same code can run on simulation or real hardware
4. **Rich Ecosystem**: Hundreds of packages for perception, control, navigation, and more
5. **Simulation Integration**: Seamless transition between Gazebo simulation and real robots

## Getting Started

To begin working with ROS 2, you'll need to:

1. Install ROS 2 (Humble Hawksbill recommended for this course)
2. Set up your development workspace
3. Learn the basic command-line tools
4. Create your first publisher and subscriber nodes

In the next sections, we'll dive deeper into each of these concepts with practical examples and exercises that bridge AI agents to ROS controllers.