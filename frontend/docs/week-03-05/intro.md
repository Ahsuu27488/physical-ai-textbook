---
title: Weeks 3-5 - ROS 2 Fundamentals
sidebar_position: 7
id: intro
---

# Weeks 3-5: ROS 2 Fundamentals

## Learning Objectives

During these three weeks, students will master the core concepts and tools of ROS 2, the middleware framework that enables communication between different components of a robotic system.

## Topics Covered

- **ROS 2 architecture and core concepts**
  - Understanding the distributed computing model
  - Nodes, topics, services, and actions
  - The DDS (Data Distribution Service) communication layer

- **Nodes, topics, services, and actions**
  - Creating and managing ROS 2 nodes
  - Publisher-subscriber pattern with topics
  - Request-response pattern with services
  - Long-running tasks with actions

- **Building ROS 2 packages with Python**
  - Package structure and organization
  - Creating nodes in Python using rclpy
  - Managing dependencies and build configurations

- **Launch files and parameter management**
  - Organizing complex systems with launch files
  - Managing system parameters
  - Configuration management for different environments

## Key Concepts

### ROS 2 Architecture

ROS 2 uses a distributed computing architecture based on the Data Distribution Service (DDS) standard. This provides:

- **Decentralized communication**: No single point of failure
- **Language independence**: Nodes can be written in different programming languages
- **Transport flexibility**: Multiple transport mechanisms (TCP, UDP, shared memory)
- **Quality of Service (QoS)**: Configurable communication behavior for different use cases

### Communication Patterns

ROS 2 provides four main communication patterns:

1. **Topics (Publish/Subscribe)**: Asynchronous, many-to-many communication
2. **Services (Request/Response)**: Synchronous, one-to-one communication
3. **Actions**: Asynchronous, long-running tasks with feedback
4. **Parameters**: Configuration values shared across nodes

### rclpy

rclpy is the Python client library for ROS 2. It provides:

- Node creation and management
- Publisher and subscriber interfaces
- Service and action clients/servers
- Parameter handling
- Time and logging utilities

## Practical Exercises

1. Create a simple publisher and subscriber
2. Implement a service server and client
3. Build a ROS 2 package with multiple nodes
4. Use launch files to start complex systems
5. Manage parameters for different robot configurations

## Assignments

1. ROS 2 package development project
2. Implement a simple robot controller using ROS 2
3. Create a launch file for a multi-node system