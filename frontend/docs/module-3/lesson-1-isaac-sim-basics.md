---
title: Lesson 1 - Isaac Sim Basics and Setup
sidebar_position: 4
---

# Lesson 1: Isaac Sim Basics and Setup

## Learning Objectives

By the end of this lesson, students will be able to:
- Install and configure NVIDIA Isaac Sim
- Understand the Omniverse platform and USD format
- Create basic simulation environments
- Load and control robots in Isaac Sim

## Introduction to NVIDIA Isaac Sim

NVIDIA Isaac Sim is a high-fidelity simulation application built on NVIDIA Omniverse, designed for developing, testing, and validating AI-based robotic applications. It provides photorealistic rendering, accurate physics simulation, and seamless integration with the Isaac ecosystem.

### Key Features of Isaac Sim

1. **Photorealistic Rendering**: RTX-accelerated rendering for synthetic data generation
2. **Accurate Physics**: PhysX engine for realistic physical interactions
3. **USD Support**: Universal Scene Description for complex 3D scenes
4. **Synthetic Data Generation**: Tools for creating labeled datasets
5. **ROS 2 Integration**: Seamless integration with ROS 2 workflows
6. **AI Training Environment**: Framework for reinforcement learning and imitation learning

## System Requirements

Isaac Sim has significant hardware requirements due to its photorealistic rendering:

### Minimum Requirements
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher
- **CPU**: Intel Core i7 (13th Gen) or AMD Ryzen 9
- **RAM**: 32 GB (64 GB recommended)
- **OS**: Ubuntu 22.04 LTS or Windows 10/11

### Recommended Requirements
- **GPU**: NVIDIA RTX 4090 (24GB VRAM) or RTX A6000
- **CPU**: Intel Core i9 or AMD Threadripper
- **RAM**: 64 GB or more
- **Storage**: Fast NVMe SSD for scene loading

## Installing Isaac Sim

### Prerequisites
1. Install NVIDIA GPU drivers (535 or later)
2. Install CUDA Toolkit (12.0 or later)
3. Ensure RTX-capable GPU is available

### Installation Steps

1. **Download Isaac Sim** from NVIDIA Developer website
2. **Extract the package**:
   ```bash
   tar -xzf isaac-sim-2023.1.0.tar.gz
   ```

3. **Set up environment**:
   ```bash
   cd isaac-sim
   export ISAACSIM_PATH=$(pwd)
   ```

4. **Install dependencies**:
   ```bash
   python -m pip install -e .
   ```

### Alternative: Docker Installation

For easier setup, you can use the Isaac Sim Docker container:

```bash
docker run --gpus all -it --rm \
  --env "ACCEPT_EULA=Y" \
  --env "ISAACSIM_USERNAME=your_username" \
  --env "ISAACSIM_PASSWORD=your_password" \
  --volume $HOME/isaac-sim-cache:/isaac-sim/cache/Kit \
  --volume $HOME/isaac-sim-logs:/isaac-sim/logs \
  --volume $HOME/isaac-sim-data:/isaac-sim/data \
  --net=host \
  --privileged \
  --pid=host \
  -e "DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
  -e XAUTHORITY=/tmp/.docker.xauth \
  nvcr.io/nvidia/isaac-sim:2023.1.0
```

## Understanding Omniverse and USD

### Universal Scene Description (USD)

USD is Pixar's scene description format that Isaac Sim uses for representing 3D scenes:

```usd
# Example USD file (scene.usda)
def Xform "Robot" (
    prepend references = @./franka_panda.usd@
)
{
    def Xform "base"
    {
        def Xform "link0"
        {
            def Xform "joint0"
            {
                # Joint properties
            }
        }
    }
}
```

### USD Key Concepts

- **Prims (Primitives)**: Basic objects in the scene
- **Properties**: Attributes of prims (position, color, etc.)
- **Relationships**: Connections between prims
- **Variants**: Different configurations of the same object
- **Payloads**: Deferred loading of heavy assets

## Getting Started with Isaac Sim

### Launch Isaac Sim

```bash
# Direct launch
./isaac-sim/python.sh

# Or via Omniverse Launcher (recommended)
```

### Basic Interface Overview

When Isaac Sim launches, you'll see:

1. **Viewport**: 3D scene rendering window
2. **Stage Panel**: Scene hierarchy and object management
3. **Property Panel**: Selected object properties
4. **Timeline**: Animation and simulation controls
5. **Console**: Scripting and logging output

## Creating Your First Scene

### Using the Isaac Sim Assets

Isaac Sim comes with a rich library of assets:

1. **Robots**: Franka Panda, UR5, ABB, etc.
2. **Environments**: Warehouse, office, home scenes
3. **Objects**: Household items, tools, furniture
4. **Sensors**: Cameras, LiDAR, IMU models

### Loading a Robot

1. Open Isaac Sim
2. Go to **Window → Isaac Examples → 1a - Robot Bridge → Franka Cube Pick**
3. This loads a Franka robot with a cube in a simple environment

### Basic Scene Setup Script

Here's a Python script to programmatically create a simple scene:

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.franka import Franka
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np

# Create a world object
my_world = World(stage_units_in_meters=1.0)

# Get the assets root path
assets_root_path = get_assets_root_path()

# Add a robot to the stage
if assets_root_path is not None:
    # Load a Franka robot
    my_franka = my_world.scene.add(
        Franka(
            prim_path="/World/Franka",
            name="my_franka",
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1])
        )
    )

# Add a ground plane
my_world.scene.add_ground_plane("/World/GroundPlane")

# Play the simulation
my_world.reset()
```

## USD Scene Structure

### Basic USD Scene Example

```usda
#usda 1.0
(
    metersPerUnit = 1
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "GroundPlane"
    {
        def PhysicsGroundPlane "GroundPlane"
        {
            prepend references = </Isaac/Props/Prismarine/ground_plane.usd>
        }
    }

    def Xform "Robot"
    {
        def Xform "Franka"
        {
            prepend references = </Isaac/Robots/Franka/franka_alt_fingers.usd>
        }
    }

    def Xform "Light"
    {
        def DistantLight "DistantLight"
        {
            float intensity = 500
            color color = (0.9, 0.9, 0.9)
        }
    }
}
```

## Isaac Sim Extensions

Isaac Sim uses extensions to provide functionality:

### Key Extensions
- **Isaac Sim Robotics**: Core robotics functionality
- **Isaac Sim Sensors**: Camera, LiDAR, IMU simulation
- **Isaac Sim Navigation**: Path planning and navigation
- **Isaac Sim Manipulation**: Grasping and manipulation
- **Isaac Sim Gym**: Reinforcement learning environments

### Enabling Extensions

Extensions can be managed through:
1. **Window → Extensions** menu
2. Programmatically in scripts
3. Through configuration files

## Practical Exercise: Basic Robot Setup

Create a simple scene with a robot and basic environment:

1. **Launch Isaac Sim**
2. **Create a new stage** (File → New Stage)
3. **Add a ground plane** using the Create menu
4. **Add a robot** (e.g., Franka Panda) from the Isaac assets
5. **Add a light source** for proper illumination
6. **Run the simulation** to see the robot in the environment

### Python Script for the Exercise

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.franka import Franka
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np

# Initialize the world
my_world = World(stage_units_in_meters=1.0)

# Get assets root path
assets_root_path = get_assets_root_path()

if assets_root_path is not None:
    # Add ground plane
    my_world.scene.add_ground_plane("/World/GroundPlane", size=10.0)

    # Add a Franka robot
    franka_robot = my_world.scene.add(
        Franka(
            prim_path="/World/Franka",
            name="franka",
            position=np.array([0.0, 0.0, 0.0]),
        )
    )

    # Reset the world to apply changes
    my_world.reset()

    # Print robot information
    print("Robot added successfully!")
    print(f"Robot position: {franka_robot.get_world_pose()[0]}")

# To run simulation continuously
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
```

## Troubleshooting Common Issues

### Rendering Issues
- Ensure GPU drivers are up to date
- Check VRAM availability (minimum 8GB recommended)
- Verify RTX-capable GPU is being used

### Performance Issues
- Reduce scene complexity
- Lower rendering quality settings
- Use simpler physics models during development

### Asset Loading Problems
- Verify Omniverse connection
- Check assets root path
- Ensure proper permissions for asset directories

## Summary

This lesson introduced NVIDIA Isaac Sim, its installation, and basic scene creation. Isaac Sim provides a powerful platform for robotics simulation with photorealistic rendering and accurate physics.

## Next Steps

In the next lesson, we'll explore synthetic data generation and how to create training datasets for AI models using Isaac Sim.