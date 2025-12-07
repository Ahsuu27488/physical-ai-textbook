---
title: Lesson 2 - Nodes, Topics, and Services
sidebar_position: 4
---

# Lesson 2: Nodes, Topics, and Services

## Learning Objectives

By the end of this lesson, students will be able to:
- Create and manage ROS 2 nodes using Python
- Implement publisher and subscriber patterns
- Build request/response services
- Use ROS 2 command-line tools for introspection

## ROS 2 Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system.

### Node Creation in Python

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Node Parameters

Nodes can accept parameters that can be configured at runtime:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('int_param', 42)

        # Get parameter value
        param_value = self.get_parameter('param_name').value
```

## Topics and Publishers/Subscribers

### Publishers

A publisher sends messages to a topic:

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from publisher'
        self.publisher_.publish(msg)
```

### Subscribers

A subscriber receives messages from a topic:

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
```

## Services

Services provide request/response communication:

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## ROS 2 Command Line Tools

### Node Commands
- `ros2 node list` - List active nodes
- `ros2 node info <node_name>` - Get detailed information about a node

### Topic Commands
- `ros2 topic list` - List active topics
- `ros2 topic echo <topic_name>` - Print messages on a topic
- `ros2 topic pub <topic_name> <msg_type> <args>` - Publish to a topic

### Service Commands
- `ros2 service list` - List active services
- `ros2 service call <service_name> <service_type> <args>` - Call a service

## Practical Exercise: Publisher-Subscriber Pair

Create a publisher that sends velocity commands and a subscriber that processes them:

**publisher.py:**
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 1.0  # Move forward at 1 m/s
        msg.angular.z = 0.5  # Rotate at 0.5 rad/s
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()
```

**subscriber.py:**
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)

    def velocity_callback(self, msg):
        self.get_logger().info(
            f'Linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), '
            f'Angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})'
        )

def main(args=None):
    rclpy.init(args=args)
    velocity_subscriber = VelocitySubscriber()
    rclpy.spin(velocity_subscriber)
    velocity_subscriber.destroy_node()
    rclpy.shutdown()
```

## Summary

This lesson covered the core communication patterns in ROS 2: nodes, topics, and services. These form the foundation for building distributed robotic systems where different components can communicate effectively.

