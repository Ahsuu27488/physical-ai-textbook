---
title: Lesson 1 - ROS 2 Architecture and DDS
sidebar_position: 3
---

# Lesson 1: ROS 2 Architecture and DDS

## Learning Objectives

By the end of this lesson, students will be able to:
- Explain the DDS (Data Distribution Service) communication layer
- Identify the key differences between ROS 1 and ROS 2
- Understand the client library architecture (rclpy, rclcpp)
- Describe the concept of ROS 2 domains and namespaces

## Introduction to DDS

The Data Distribution Service (DDS) is the underlying communication middleware that powers ROS 2. Unlike ROS 1's custom TCPROS/UDPROS protocols, ROS 2 uses DDS as its communication layer, which provides:

- **Decentralized communication**: No master required
- **Language and platform independence**: Standardized API
- **Quality of Service (QoS) policies**: Configurable communication behavior
- **Security**: Built-in authentication and encryption capabilities

### DDS Concepts

#### Topics and Publishers/Subscribers
In DDS terminology:
- **Topic**: A named data stream (e.g., `/cmd_vel`, `/scan`)
- **Publisher**: Entity that sends data on a topic
- **Subscriber**: Entity that receives data from a topic

#### Services and Clients
- **Service**: Request/reply communication pattern
- **Server**: Responds to service requests
- **Client**: Makes service requests

#### Actions
- **Action**: Long-running tasks with feedback and goal management
- **Action Server**: Manages action goals and provides feedback
- **Action Client**: Sends goals and receives feedback/results

## Quality of Service (QoS) Policies

ROS 2 introduces QoS policies that allow fine-tuning of communication behavior:

### Reliability Policy
- **Reliable**: All messages are guaranteed to be delivered
- **Best Effort**: Messages may be lost, but faster delivery

### Durability Policy
- **Transient Local**: Late-joining subscribers receive last known value
- **Volatile**: Only new messages are sent to subscribers

### History Policy
- **Keep Last**: Store a specific number of most recent messages
- **Keep All**: Store all messages (use with caution)

## Practical Exercise: Creating a Simple Publisher

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

## Summary

This lesson introduced the fundamental concepts of ROS 2 architecture and the DDS communication layer. Understanding these concepts is crucial for developing robust robotic applications that can handle real-world communication challenges.

