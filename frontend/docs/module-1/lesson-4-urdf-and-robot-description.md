---
title: Lesson 4 - URDF and Robot Description
sidebar_position: 6
---

# Lesson 4: URDF and Robot Description

## Learning Objectives

By the end of this lesson, students will be able to:
- Create and understand URDF (Unified Robot Description Format) files
- Define robot kinematics and dynamics using URDF
- Use Xacro for complex robot descriptions
- Integrate robot descriptions with Gazebo simulation

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of robots, including links, joints, inertial properties, and visual/collision geometries.

### Why URDF is Important

1. **Simulation**: Defines how robots appear and behave in simulation
2. **Visualization**: Enables proper display in RViz and other tools
3. **Kinematics**: Provides forward and inverse kinematics information
4. **Dynamics**: Specifies mass, inertia, and other dynamic properties
5. **Hardware Integration**: Bridges real robots with ROS software

## URDF Structure

### Basic URDF Components

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
</robot>
```

### Link Elements

A link represents a rigid body part of the robot:

```xml
<link name="link_name">
  <!-- Visual properties (for display) -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Shape: box, cylinder, sphere, or mesh -->
      <box size="1 1 1"/>
    </geometry>
    <material name="material_name">
      <color rgba="1 0 0 1"/> <!-- Red color -->
    </material>
  </visual>

  <!-- Collision properties (for physics) -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <!-- Inertial properties (for dynamics) -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joint Elements

Joints define the connection between links:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>

  <!-- Joint limits (for revolute/prismatic joints) -->
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>

  <!-- Joint dynamics -->
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Joint Types

1. **Revolute**: Rotational joint with limits
2. **Continuous**: Rotational joint without limits
3. **Prismatic**: Linear sliding joint with limits
4. **Fixed**: No movement (welded joint)
5. **Floating**: 6 DOF movement
6. **Planar**: Movement on a plane

## Practical URDF Example: Simple Robot Arm

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Shoulder joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="elbow_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Tool frame -->
  <joint name="tool_joint" type="fixed">
    <parent link="elbow_link"/>
    <child link="tool_frame"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>
  </joint>

  <link name="tool_frame"/>
</robot>
```

## Using Xacro for Complex Descriptions

Xacro (XML Macros) allows parameterization and reusability in URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arm_with_xacro">
  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Define a macro for a simple link -->
  <xacro:macro name="simple_link" params="name mass radius length color_x color_y color_z">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="${name}_material">
          <color rgba="${color_x} ${color_y} ${color_z} 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia
          ixx="${0.0833333 * mass * (3*radius*radius + length*length)}"
          ixy="0" ixz="0"
          iyy="${0.0833333 * mass * (3*radius*radius + length*length)}"
          iyz="0"
          izz="${0.5 * mass * radius * radius}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Define a macro for a joint -->
  <xacro:macro name="simple_joint" params="name type parent child xyz rpy axis lower upper">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <xacro:if value="${type != 'fixed'}">
        <limit lower="${lower}" upper="${upper}" effort="100" velocity="1"/>
      </xacro:if>
    </joint>
  </xacro:macro>

  <!-- Use the macros to build the robot -->
  <xacro:simple_link name="base_link" mass="2" radius="0.1" length="0.1"
                     color_x="0.5" color_y="0.5" color_z="0.5"/>

  <xacro:simple_link name="shoulder_link" mass="0.5" radius="0.05" length="0.1"
                     color_x="0.8" color_y="0.2" color_z="0.2"/>

  <xacro:simple_joint name="shoulder_joint" type="revolute"
                      parent="base_link" child="shoulder_link"
                      xyz="0 0 0.05" rpy="0 0 0" axis="0 0 1"
                      lower="-1.57" upper="1.57"/>
</robot>
```

## URDF for Gazebo Integration

To make URDF work properly in Gazebo, add Gazebo-specific elements:

```xml
<?xml version="1.0"?>
<robot name="gazebo_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.15</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

## Loading and Validating URDF

### Loading URDF in ROS

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Visualize URDF
urdf_to_graphiz /path/to/robot.urdf
```

### Launch file for visualization

```xml
<launch>
  <!-- Load robot description parameter -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'" />

  <!-- Publish robot state -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <!-- Joint state publisher (for visualization) -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

  <!-- RViz for visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_robot)/config/robot.rviz" />
</launch>
```

## Practical Exercise: Create a Custom Robot

Create a URDF description for a simple mobile robot with the following specifications:

1. **Base**: Cylindrical base with radius 0.2m and height 0.1m
2. **Wheels**: Two wheels, each with radius 0.05m and width 0.05m
3. **Wheels Position**: 0.15m from center, on left and right sides
4. **Camera**: RGB camera on front of robot
5. **Colors**: Base in blue, wheels in black

### Steps:
1. Create a new Xacro file `mobile_robot.xacro`
2. Define the base link with appropriate visual, collision, and inertial properties
3. Add left and right wheel links with appropriate joints
4. Add a camera mount and link
5. Include Gazebo-specific elements for simulation

### Example Solution Structure:

```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Properties -->
  <xacro:property name="base_mass" value="5"/>
  <xacro:property name="base_radius" value="0.2"/>
  <xacro:property name="base_height" value="0.1"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${base_mass}"/>
      <inertia
        ixx="${0.0833333 * base_mass * (3*base_radius*base_radius + base_height*base_height)}"
        ixy="0" ixz="0"
        iyy="${0.0833333 * base_mass * (3*base_radius*base_radius + base_height*base_height)}"
        iyz="0"
        izz="${0.5 * base_mass * base_radius * base_radius}"/>
    </inertial>
  </link>

  <!-- Add wheels and camera -->
  <!-- Continue with wheel and camera definitions -->
</robot>
```

## Common URDF Issues and Solutions

### 1. Invalid Inertial Properties
- **Issue**: "Inertial matrix is not positive definite"
- **Solution**: Ensure diagonal elements are positive and satisfy triangle inequality

### 2. Missing Joint Limits
- **Issue**: Controllers may request impossible positions
- **Solution**: Always specify limits for revolute and prismatic joints

### 3. Collision vs Visual Mismatch
- **Issue**: Robot appears to collide when it shouldn't
- **Solution**: Ensure collision and visual geometries are consistent

### 4. Mass Distribution Problems
- **Issue**: Robot behaves unrealistically in simulation
- **Solution**: Calculate realistic inertial properties or use CAD tools

## Advanced Topics

### Transmission Elements
For hardware interface:

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Safety Controllers
```xml
<gazebo reference="joint_name">
  <provideFeedback>true</provideFeedback>
  <disableFixedJointLumping>true</disableFixedJointLumping>
</gazebo>
```

## Summary

This lesson covered URDF (Unified Robot Description Format) and how to create robot descriptions for both visualization and simulation. URDF is essential for representing robots in ROS and integrating them with simulation environments like Gazebo.

## Next Steps

In the next lesson, we'll explore TF (Transforms) and how robots maintain spatial relationships between their components.