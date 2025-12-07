---
title: Lesson 3 - ROS-Gazebo Integration
sidebar_position: 6
---

# Lesson 3: ROS-Gazebo Integration

## Learning Objectives

By the end of this lesson, students will be able to:
- Integrate ROS 2 with Gazebo simulation
- Control simulated robots using ROS 2 nodes
- Process sensor data from Gazebo in ROS 2
- Create launch files for complete simulation environments

## Introduction to ROS-Gazebo Integration

The integration between ROS and Gazebo enables seamless development and testing of robotic systems. Gazebo provides realistic physics simulation and sensor modeling, while ROS provides the communication framework and tooling for robot development.

### Key Components of ROS-Gazebo Integration

1. **Gazebo ROS Packages**: Bridge between Gazebo and ROS
2. **Robot Description**: URDF models loaded into Gazebo
3. **Sensor Plugins**: Publish sensor data to ROS topics
4. **Controller Plugins**: Subscribe to ROS topics for actuator control
5. **TF Publishers**: Maintain coordinate frame relationships

## Installing Gazebo ROS Packages

### Prerequisites

```bash
# Install ROS 2 Gazebo packages
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-ros2-control-demos
```

## Basic ROS-Gazebo Launch File

### Simple Robot Launch File

```xml
<launch>
  <!-- Arguments -->
  <arg name="world" default="empty"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- Gazebo server -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch.py">
    <arg name="world_name" value="$(var world)"/>
    <arg name="paused" value="$(var paused)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
    <arg name="gui" value="$(var gui)"/>
    <arg name="headless" value="$(var headless)"/>
    <arg name="debug" value="$(var debug)"/>
  </include>

  <!-- Load robot description parameter -->
  <param name="robot_description"
         command="xacro $(find my_robot_description)/urdf/my_robot.xacro"/>

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_entity.py"
        args="-topic robot_description -entity my_robot"/>

  <!-- Robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher" respawn="false" output="screen"/>
</launch>
```

## Gazebo Plugins for ROS Integration

### Differential Drive Plugin

```xml
<!-- In your URDF/Xacro file -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <!-- Wheel information -->
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>

    <!-- Limits -->
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>

    <!-- Topics -->
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>

    <!-- Rate -->
    <publish_rate>50</publish_rate>
    <odometry_publish_rate>50</odometry_publish_rate>
  </plugin>
</gazebo>
```

### Camera Plugin

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <topic_name>camera/image_raw</topic_name>
      <camera_info_topic_name>camera/camera_info</camera_info_topic_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Plugin

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
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <frame_name>imu_link</frame_name>
      <topic_name>imu/data</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

## Controlling Robots in Simulation

### Robot Control Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initialize variables
        self.current_pose = None
        self.laser_data = None
        self.bridge = CvBridge()
        self.get_logger().info('Robot controller initialized')

    def odom_callback(self, msg):
        """Handle odometry messages"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Handle laser scan messages"""
        self.laser_data = msg.ranges

    def image_callback(self, msg):
        """Handle image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image here
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_image(self, image):
        """Process the received image"""
        # Example: Detect red objects
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                # Calculate center of contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    image_width = image.shape[1]
                    center_x = cx - image_width // 2  # Center relative to image center

                    # Generate control command based on object position
                    cmd_vel = Twist()
                    cmd_vel.angular.z = -0.002 * center_x  # Turn toward object
                    cmd_vel.linear.x = 0.2  # Move forward slowly
                    self.cmd_vel_pub.publish(cmd_vel)

    def control_loop(self):
        """Main control loop"""
        if self.laser_data is not None:
            # Simple obstacle avoidance
            min_distance = min(self.laser_data) if self.laser_data else float('inf')

            cmd_vel = Twist()
            if min_distance < 0.5:  # Obstacle detected within 0.5m
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5  # Turn away from obstacle
            else:
                cmd_vel.linear.x = 0.2  # Move forward
                cmd_vel.angular.z = 0.0

            self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File with Complete Robot Simulation

### Advanced Launch File

```xml
<launch>
  <!-- Arguments -->
  <arg name="world" default="worlds/empty.world"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- Gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch.py">
    <arg name="world_name" value="$(var world)"/>
    <arg name="paused" value="$(var paused)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
    <arg name="gui" value="$(var gui)"/>
    <arg name="headless" value="$(var headless)"/>
    <arg name="debug" value="$(var debug)"/>
  </include>

  <!-- Robot Description -->
  <param name="robot_description"
         command="xacro $(find my_robot_description)/urdf/my_robot.xacro"/>

  <!-- Spawn Robot -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_entity.py"
        args="-topic robot_description -entity my_robot"
        output="screen"/>

  <!-- Robot State Publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher" respawn="false" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Joint State Publisher (for non-actuated joints) -->
  <node name="joint_state_publisher" pkg="joint_state_publisher"
        type="joint_state_publisher" respawn="false" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Robot Controller -->
  <node name="robot_controller" pkg="my_robot_controller"
        type="robot_controller.py" respawn="false" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- RViz -->
  <node name="rviz" pkg="rviz2" type="rviz2"
        args="-d $(find my_robot_description)/config/robot.rviz"
        respawn="false"/>
</launch>
```

## Processing Sensor Data from Simulation

### Sensor Data Processing Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
from cv_bridge import CvBridge
import cv2

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)

        # Create publishers
        self.obstacle_distance_pub = self.create_publisher(
            Float32, 'obstacle_distance', 10)
        self.velocity_cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        # Initialize
        self.bridge = CvBridge()
        self.latest_scan = None
        self.latest_imu = None
        self.get_logger().info('Sensor processor initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.latest_scan = msg

        # Calculate minimum distance in front of robot (forward 30 degrees)
        ranges = np.array(msg.ranges)
        # Filter out invalid ranges
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_distance = Float32()
            min_distance.data = float(np.min(valid_ranges))
            self.obstacle_distance_pub.publish(min_distance)

    def image_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Example: Detect lines using Hough transform
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100,
                                   minLineLength=50, maxLineGap=10)

            if lines is not None:
                # Draw detected lines
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Show processed image
                cv2.imshow('Processed Image', cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Process IMU data"""
        self.latest_imu = msg

        # Example: Calculate orientation from quaternion
        import math
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        # Convert quaternion to Euler angles (simplified)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        # Use IMU data for navigation
        self.adjust_navigation(yaw)

    def adjust_navigation(self, current_yaw):
        """Adjust navigation based on IMU data"""
        if self.latest_scan is not None:
            # Simple wall following algorithm
            scan = self.latest_scan
            # Get ranges for left, front, right
            mid_idx = len(scan.ranges) // 2
            front_range = scan.ranges[mid_idx] if np.isfinite(scan.ranges[mid_idx]) else float('inf')
            left_range = scan.ranges[len(scan.ranges)//4] if np.isfinite(scan.ranges[len(scan.ranges)//4]) else float('inf')
            right_range = scan.ranges[3*len(scan.ranges)//4] if np.isfinite(scan.ranges[3*len(scan.ranges)//4]) else float('inf')

            cmd_vel = Twist()
            if front_range < 0.5:  # Too close to front obstacle
                cmd_vel.angular.z = 0.5  # Turn right
            elif right_range < 0.3:  # Too close to right wall
                cmd_vel.angular.z = 0.3  # Turn left slightly
            elif left_range < 0.3:  # Too close to left wall
                cmd_vel.angular.z = -0.3  # Turn right slightly
            else:
                cmd_vel.linear.x = 0.2  # Move forward

            self.velocity_cmd_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    processor = SensorProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Simulation Features

### Creating Custom Worlds

```xml
<!-- custom_world.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="custom_world">
    <!-- Physics -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom objects -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add more objects as needed -->
  </world>
</sdf>
```

### Dynamic Obstacles

```xml
<!-- Dynamic obstacle that moves in a circle -->
<model name="moving_obstacle">
  <link name="link">
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.1</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.1</radius>
        </sphere>
      </geometry>
    </collision>
    <inertial>
      <mass>0.1</mass>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Model plugin for movement -->
  <plugin name="model_move" filename="libgazebo_ros_p3d.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>30</updateRate>
    <bodyName>link</bodyName>
    <topicName>model_pose</topicName>
    <gaussianNoise>0.0</gaussianNoise>
    <frameName>world</frameName>
  </plugin>
</model>
```

## Practical Exercise: Complete Simulation Setup

Create a complete simulation environment with:

1. **Custom Robot Model**: Differential drive robot with camera and IMU
2. **Custom World**: Room with obstacles
3. **Control Node**: Navigation and obstacle avoidance
4. **Launch File**: Complete simulation setup

### Steps:

1. **Create robot URDF** with differential drive, camera, and IMU
2. **Create custom world file** with obstacles
3. **Implement control node** that processes sensor data and controls the robot
4. **Create launch file** that starts Gazebo, spawns robot, and starts control nodes
5. **Test the simulation** and verify sensor data processing

## Troubleshooting Common Issues

### 1. Robot Not Spawning
- **Check**: URDF syntax and joint definitions
- **Solution**: Validate URDF with `check_urdf` command

### 2. Sensor Data Not Publishing
- **Check**: Plugin configuration and topic names
- **Solution**: Verify plugin parameters match ROS topic expectations

### 3. Robot Control Issues
- **Check**: Joint names match between URDF and controller
- **Solution**: Verify transmission elements in URDF

### 4. TF Issues
- **Check**: Robot state publisher is running
- **Solution**: Ensure `use_sim_time` parameter is set correctly

## Performance Optimization

### Simulation Speed
- Reduce physics update rate for complex scenes
- Simplify collision geometries
- Use fewer but larger time steps

### Sensor Processing
- Reduce sensor update rates if not needed
- Use smaller image resolutions for processing
- Implement data throttling for high-frequency sensors

## Summary

This lesson covered the integration between ROS and Gazebo, including plugin configuration, sensor data processing, and robot control. Proper ROS-Gazebo integration is essential for developing and testing robotic systems in simulation before deployment on real hardware.

## Next Steps

In the next lesson, we'll explore advanced simulation techniques including physics parameter tuning, sensor noise modeling, and realistic environment creation.