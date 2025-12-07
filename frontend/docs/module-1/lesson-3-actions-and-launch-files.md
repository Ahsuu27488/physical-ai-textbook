---
title: Lesson 3 - Actions and Launch Files
sidebar_position: 5
---

# Lesson 3: Actions and Launch Files

## Learning Objectives

By the end of this lesson, students will be able to:
- Implement and use ROS 2 actions for long-running tasks
- Create and configure launch files for complex systems
- Understand the role of actions in robotics applications
- Organize multi-node systems using launch files

## ROS 2 Actions

Actions are a communication pattern that provides feedback during long-running tasks and allows for goal preemption. They're essential for robotics tasks like navigation, manipulation, and calibration.

### Action Structure

An action has three parts:
1. **Goal**: The request to start the action
2. **Feedback**: Periodic updates during execution
3. **Result**: The final outcome of the action

### Action Server

```python
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=rclpy.callback_group.ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Create messages to be sent
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Update Fibonacci sequence
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Sleep to simulate work
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Action Client

```python
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()
    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes with a single command, making it easier to manage complex robotic systems.

### Basic Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop'
        )
    ])
```

### Launch File with Parameters

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            name='frequency',
            default_value='10.0',
            description='Publish frequency in Hz'
        ),

        # Use launch arguments in node configuration
        Node(
            package='demo_nodes_py',
            executable='talker',
            parameters=[
                {
                    'frequency': LaunchConfiguration('frequency')
                }
            ]
        )
    ])
```

### Conditional Launch

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='demo_nodes_py',
            executable='listener',
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        )
    ])
```

## Practical Exercise: Navigation Action Client

Create an action client for navigation that sends goals to a navigation server:

**navigation_client.py:**
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class NavigationActionClient(Node):

    def __init__(self):
        super().__init__('navigation_action_client')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()

        # Create target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info(f'Sending navigation goal to ({x}, {y}, {theta})')

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation completed: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigating... Current pose: {feedback.current_pose}')


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()

    # Navigate to coordinates (2.0, 2.0, 0.0)
    action_client.send_goal(2.0, 2.0, 0.0)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## Advanced Launch Concepts

### Launch Substitutions

Launch files support substitutions for dynamic configuration:

```python
from launch import LaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('my_package'),
                    'config',
                    TextSubstitution(text='my_config.yaml')
                ])
            ]
        )
    ])
```

### Launch File Includes

You can include other launch files:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('turtlebot3_bringup'),
                '/launch/turtlebot3_world.launch.py'
            ])
        )
    ])
```

## Summary

This lesson covered ROS 2 actions for long-running tasks with feedback and launch files for managing complex multi-node systems. These are essential tools for building real-world robotic applications.

## Next Steps

In the next lesson, we'll explore URDF (Unified Robot Description Format) and how to describe robots in ROS 2.