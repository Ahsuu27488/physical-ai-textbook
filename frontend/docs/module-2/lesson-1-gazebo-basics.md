---
title: Lesson 1 - Gazebo Basics and World Building
sidebar_position: 4
---

# Lesson 1: Gazebo Basics and World Building

## Learning Objectives

By the end of this lesson, students will be able to:
- Install and configure Gazebo Harmonic
- Create custom simulation worlds
- Understand Gazebo's physics engine integration
- Work with basic Gazebo models and environments

## Introduction to Gazebo

Gazebo is a 3D simulation environment for robotics that provides realistic physics simulation, high-quality rendering, and sensor simulation. It's an essential tool for testing and validating robotic systems before deployment on real hardware.

### Key Features of Gazebo

1. **Physics Simulation**: Accurate simulation of rigid body dynamics using ODE, Bullet, or DART
2. **Sensor Simulation**: Realistic simulation of cameras, LiDAR, IMUs, GPS, and other sensors
3. **Rendering**: High-quality 3D visualization with support for multiple rendering engines
4. **ROS Integration**: Seamless integration with ROS/ROS 2 for robotic simulation
5. **Model Database**: Access to a large database of pre-built robot and environment models

## Installing Gazebo Harmonic

For this course, we'll use Gazebo Harmonic (the latest stable version):

```bash
# On Ubuntu 22.04
sudo apt update
sudo apt install gazebo libgazebo-dev
```

## Basic Gazebo World Structure

Gazebo worlds are defined using SDF (Simulation Description Format), an XML-based format:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Include the default Gazebo world -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add your custom models here -->
    <model name="my_robot">
      <!-- Model definition -->
    </model>
  </world>
</sdf>
```

### World Elements

- **`<physics>`**: Physics engine configuration (ODE, Bullet, DART)
- **`<scene>`**: Visual properties (ambient lighting, shadows)
- **`<light>`**: Light sources in the environment
- **`<model>`**: Robot and object definitions
- **`<include>`**: References to models in the Gazebo model database

## Creating a Simple World

Let's create a custom world file called `simple_room.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
    <!-- Physics engine -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Scene settings -->
    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun light -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.4 -0.8</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Cube obstacle -->
    <model name="cube_obstacle">
      <pose>-1 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall -->
    <model name="wall">
      <pose>0 2 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>5 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>5 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Running Gazebo with Custom Worlds

To run Gazebo with your custom world:

```bash
# Run with a specific world file
gazebo simple_room.world

# Or run Gazebo and load the world later
gazebo --verbose
```

## Gazebo GUI Overview

When Gazebo starts, you'll see several key components:

1. **3D View**: The main simulation environment
2. **World Control**: Play, pause, step through simulation
3. **Model Database**: Pre-built models to insert into the world
4. **Scene Graph**: Tree view of all objects in the simulation
5. **Tools**: Additional utilities like the model editor

## Physics Configuration

The physics engine is crucial for realistic simulation. Key parameters include:

- **`max_step_size`**: Time step for physics updates (smaller = more accurate but slower)
- **`real_time_factor`**: Target simulation speed (1.0 = real-time)
- **`real_time_update_rate`**: Update rate in Hz

### Physics Engine Comparison

- **ODE (Open Dynamics Engine)**: Stable and widely used, good for most applications
- **Bullet**: Good for complex collision detection, handles soft bodies well
- **DART**: Advanced dynamics, good for articulated robots and complex contacts

## Practical Exercise: Create Your Own World

Create a simple world file with the following elements:
1. A ground plane
2. A light source
3. At least 3 different obstacles (cubes, spheres, or cylinders)
4. A simple maze or path for navigation

**Steps:**
1. Create a new directory for your world files: `mkdir ~/gazebo_worlds`
2. Create your world file: `nano ~/gazebo_worlds/my_world.world`
3. Copy the structure from the example above
4. Add your custom elements
5. Run with: `gazebo ~/gazebo_worlds/my_world.world`

## Gazebo Commands and Tools

### Command Line Options
- `gazebo --verbose`: Show detailed output
- `gazebo --physics=bullet`: Use a specific physics engine
- `gazebo --play`: Start simulation automatically

### Model Database
Gazebo comes with a rich database of models:
- Robots (PR2, TurtleBot, etc.)
- Objects (cups, books, furniture)
- Environments (rooms, outdoor scenes)

## Troubleshooting Common Issues

### Simulation Runs Too Slow
- Reduce `max_step_size` or increase `real_time_update_rate`
- Simplify collision geometries
- Reduce the number of objects in the scene

### Physics Behaviors Are Unrealistic
- Check mass and inertia values
- Verify friction and damping coefficients
- Adjust solver parameters

### Rendering Issues
- Check graphics drivers
- Try running with `--verbose` to see error messages
- Consider using software rendering if hardware acceleration fails

## Summary

This lesson introduced Gazebo simulation environment and how to create custom worlds. Understanding world building is fundamental for creating realistic testing environments for your robots.

## Next Steps

In the next lesson, we'll explore sensor simulation in Gazebo, including cameras, LiDAR, and IMU sensors.