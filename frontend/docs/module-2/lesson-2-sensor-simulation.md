---
title: Lesson 2 - Sensor Simulation in Gazebo
sidebar_position: 5
---

# Lesson 2: Sensor Simulation in Gazebo

## Learning Objectives

By the end of this lesson, students will be able to:
- Configure and simulate various sensor types in Gazebo
- Understand the physics behind sensor simulation
- Integrate simulated sensors with ROS 2
- Validate sensor data accuracy and noise characteristics

## Introduction to Sensor Simulation

Sensor simulation is a critical aspect of robotics development, allowing you to test perception algorithms without physical hardware. Gazebo provides realistic simulation of various sensor types, including cameras, LiDAR, IMUs, GPS, and force/torque sensors.

## Camera Simulation

Cameras are fundamental sensors for visual perception in robotics. Gazebo provides realistic camera simulation with configurable parameters.

### Camera SDF Configuration

```xml
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
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
    <frame_name>camera_frame</frame_name>
    <topic_name>camera/image_raw</topic_name>
  </plugin>
</sensor>
```

### Camera Parameters Explained

- **`horizontal_fov`**: Horizontal field of view in radians
- **`image`**: Resolution and format settings
- **`clip`**: Near and far clipping planes
- **`noise`**: Simulated sensor noise characteristics
- **`plugin`**: ROS 2 interface for the sensor

## LiDAR Simulation

LiDAR (Light Detection and Ranging) sensors are crucial for navigation and mapping. Gazebo supports both 2D and 3D LiDAR simulation.

### 2D LiDAR Configuration

```xml
<sensor name="laser" type="ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle> <!-- -90 degrees -->
        <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="laser_scan" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <argument>~/out:=scan</argument>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### 3D LiDAR Configuration (HDL-32E Example)

```xml
<sensor name="velodyne" type="ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159265</min_angle>
        <max_angle>3.14159265</max_angle>
      </horizontal>
      <vertical>
        <samples>32</samples>
        <resolution>1</resolution>
        <min_angle>-0.523599</min_angle> <!-- -30 degrees -->
        <max_angle>0.157080</max_angle>   <!-- 9 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="velodyne_controller" filename="libgazebo_ros_velodyne_gpu_laser.so">
    <topic_name>velodyne_points</topic_name>
    <frame_name>velodyne</frame_name>
    <min_range>0.1</min_range>
    <max_range>100.0</max_range>
    <gaussian_noise>0.01</gaussian_noise>
  </plugin>
</sensor>
```

## IMU Simulation

Inertial Measurement Units (IMUs) provide orientation, velocity, and gravitational data. Gazebo simulates IMUs with realistic noise characteristics.

### IMU SDF Configuration

```xml
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
    <ros>
      <argument>~/out:=imu/data</argument>
    </ros>
    <frame_name>imu_link</frame_name>
    <body_name>imu_body</body_name>
  </plugin>
</sensor>
```

## GPS Simulation

GPS sensors provide global position information, useful for outdoor robotics applications.

### GPS SDF Configuration

```xml
<sensor name="gps_sensor" type="gps">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <plugin name="gps_plugin" filename="libgazebo_ros_gps.so">
    <ros>
      <argument>~/out:=gps/fix</argument>
    </ros>
    <frame_name>gps_link</frame_name>
    <topic_name>gps/fix</topic_name>
    <gaussian_noise>0.1</gaussian_noise>
  </plugin>
</sensor>
```

## Force/Torque Sensor Simulation

Force/torque sensors are essential for manipulation tasks, providing information about contact forces.

### Force/Torque SDF Configuration

```xml
<sensor name="ft_sensor" type="force_torque">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <plugin name="ft_plugin" filename="libgazebo_ros_ft_sensor.so">
    <ros>
      <argument>~/out:=ft_sensor/wrench</argument>
    </ros>
    <frame_name>ft_sensor_link</frame_name>
    <topic_name>ft_sensor/wrench</topic_name>
  </plugin>
</sensor>
```

## Complete Robot Model with Sensors

Here's an example of a robot model with multiple sensors:

```xml
<?xml version="1.0" ?>
<robot name="sensor_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Camera Mount -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Camera Sensor -->
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
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- LiDAR Mount -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- LiDAR Sensor -->
  <gazebo reference="lidar_link">
    <sensor name="laser" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_scan" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <argument>~/out:=scan</argument>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Mount -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </visual>
  </link>

  <!-- IMU Sensor -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <argument>~/out:=imu/data</argument>
        </ros>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Practical Exercise: Sensor Integration

Create a robot model with at least 3 different sensor types and test their functionality:

1. Create a URDF file with a simple robot base
2. Add camera, LiDAR, and IMU sensors
3. Load the robot in Gazebo
4. Verify that sensor data is published on ROS topics
5. Visualize the sensor data using RViz

**Commands to test:**
```bash
# Launch Gazebo with your robot
ros2 launch your_package your_launch_file.py

# Check available topics
ros2 topic list | grep -E "(camera|scan|imu)"

# View sensor data
ros2 topic echo /camera/image_raw
ros2 topic echo /scan
ros2 topic echo /imu/data
```

## Sensor Noise and Realism

Real sensors have noise and imperfections. When simulating sensors:

- **Add realistic noise models** that match real sensor specifications
- **Consider environmental factors** like lighting conditions for cameras
- **Validate sensor performance** against real hardware when possible
- **Use domain randomization** to make algorithms robust to sensor variations

## Troubleshooting Sensor Issues

### Sensor Data Not Publishing
- Check that the Gazebo ROS plugin is loaded
- Verify topic names and frame IDs
- Ensure the sensor is properly attached to a link

### Performance Issues
- Reduce sensor update rates
- Lower resolution for cameras
- Decrease the number of LiDAR rays

### Inaccurate Data
- Check sensor mounting position and orientation
- Verify noise parameters
- Validate physics properties of the environment

## Summary

This lesson covered sensor simulation in Gazebo, including cameras, LiDAR, IMUs, and other sensor types. Understanding sensor simulation is crucial for developing and testing perception algorithms in robotics.

## Next Steps

In the next lesson, we'll explore the integration between Gazebo and ROS 2, learning how to control simulated robots and process sensor data.