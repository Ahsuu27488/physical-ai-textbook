---
title: Lesson 2 - Synthetic Data Generation
sidebar_position: 5
---

# Lesson 2: Synthetic Data Generation

## Learning Objectives

By the end of this lesson, students will be able to:
- Generate synthetic datasets for computer vision tasks
- Create labeled training data in Isaac Sim
- Understand domain randomization techniques
- Apply synthetic-to-real transfer methods

## Introduction to Synthetic Data

Synthetic data generation is a critical component of modern AI development, especially for robotics applications. Isaac Sim enables the creation of large, diverse, and perfectly labeled datasets that would be expensive or impossible to collect in the real world.

### Why Synthetic Data Matters

1. **Cost-Effective**: Generate thousands of images without physical setup
2. **Perfect Labels**: Automatic ground truth for segmentation, detection, etc.
3. **Safety**: Test dangerous scenarios without risk
4. **Variety**: Control lighting, textures, backgrounds, and object poses
5. **Scalability**: Generate data 24/7 without human intervention

## Isaac Sim's Synthetic Data Tools

### Isaac Sim Sensors Extension

Isaac Sim provides comprehensive sensor simulation with realistic noise models:

```python
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import Camera
import numpy as np

# Create a camera prim
camera_prim_path = "/World/Robot/Camera"
create_prim(camera_prim_path, "Camera", position=np.array([0.0, 0.0, 0.0]))

# Add camera to the scene
camera = Camera(
    prim_path=camera_prim_path,
    frequency=30,  # Hz
    resolution=(640, 480)
)
```

### RGB and Depth Data Generation

```python
from omni.isaac.synthetic_utils import plot
import carb

# Enable RGB and depth sensors
camera.add_color_render_product("/World/Robot/Camera")
camera.add_depth_to_world_render_product("/World/Robot/Camera")

# Capture data
rgb_data = camera.get_rgb()
depth_data = camera.get_depth()

# Save data
carb.log_info(f"RGB shape: {rgb_data.shape}")
carb.log_info(f"Depth shape: {depth_data.shape}")
```

## Domain Randomization

Domain randomization is a technique that varies environmental parameters to make models more robust:

### Lighting Randomization

```python
import random
from pxr import Gf

def randomize_lighting(world):
    """Randomize lighting conditions in the scene"""
    # Get all lights in the scene
    lights = world.scene.get_all_lights()

    for light in lights:
        # Randomize intensity
        intensity = random.uniform(50, 1000)
        light.set_intensity(intensity)

        # Randomize color temperature
        color_temp = random.uniform(3000, 8000)
        light.set_color_temperature(color_temp)

        # Randomize position slightly
        current_pos = light.get_world_pose()[0]
        new_pos = current_pos + np.array([
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5)
        ])
        light.set_world_pose(position=new_pos)

def randomize_materials(world):
    """Randomize material properties"""
    # Randomize object colors, textures, and reflectance
    objects = world.scene.get_all_objects()

    for obj in objects:
        # Randomize diffuse color
        diffuse_color = (random.uniform(0, 1),
                        random.uniform(0, 1),
                        random.uniform(0, 1))
        # Apply to material
```

### Texture and Material Randomization

```python
def randomize_textures(world):
    """Apply random textures to objects"""
    import omni.kit.asset_editor as asset_editor

    # Define texture library
    texture_library = [
        "/Isaac/Textures/Metal/metal_01.png",
        "/Isaac/Textures/Wood/wood_01.png",
        "/Isaac/Textures/Plastic/plastic_01.png",
        # Add more textures
    ]

    objects = world.scene.get_all_objects()

    for obj in objects:
        # Randomly assign texture
        random_texture = random.choice(texture_library)
        # Apply texture to object's material
```

## Creating a Synthetic Dataset Pipeline

### Basic Data Collection Script

```python
import os
import json
import numpy as np
from PIL import Image
from omni.isaac.core import World
from omni.isaac.sensor import Camera
import carb

class SyntheticDataGenerator:
    def __init__(self, world, camera, output_dir="synthetic_data"):
        self.world = world
        self.camera = camera
        self.output_dir = output_dir
        self.data_counter = 0

        # Create output directories
        os.makedirs(f"{output_dir}/rgb", exist_ok=True)
        os.makedirs(f"{output_dir}/depth", exist_ok=True)
        os.makedirs(f"{output_dir}/labels", exist_ok=True)

        # Data annotations
        self.annotations = []

    def capture_frame(self):
        """Capture a single frame with all modalities"""
        # Get sensor data
        rgb_image = self.camera.get_rgb()
        depth_image = self.camera.get_depth()
        segmentation = self.camera.get_semantic_segmentation()

        # Generate filename
        filename = f"frame_{self.data_counter:06d}"

        # Save RGB image
        rgb_pil = Image.fromarray(rgb_image)
        rgb_pil.save(f"{self.output_dir}/rgb/{filename}.png")

        # Save depth image
        depth_pil = Image.fromarray(depth_image)
        depth_pil.save(f"{self.output_dir}/depth/{filename}.png")

        # Save segmentation
        seg_pil = Image.fromarray(segmentation)
        seg_pil.save(f"{self.output_dir}/labels/{filename}.png")

        # Create annotation
        annotation = {
            "filename": filename,
            "rgb_path": f"rgb/{filename}.png",
            "depth_path": f"depth/{filename}.png",
            "labels_path": f"labels/{filename}.png",
            "timestamp": self.world.current_time_step_index,
            "camera_pose": self.camera.get_world_pose(),
            "objects_in_scene": self.get_scene_objects()
        }

        self.annotations.append(annotation)
        self.data_counter += 1

        carb.log_info(f"Captured frame {filename}")

    def get_scene_objects(self):
        """Get information about objects in the scene"""
        objects = []
        for prim in self.world.scene.get_objects():
            pose = prim.get_world_pose()
            size = prim.get_world_size()
            objects.append({
                "name": prim.name,
                "position": pose[0].tolist(),
                "orientation": pose[1].tolist(),
                "size": size
            })
        return objects

    def save_annotations(self):
        """Save all annotations to JSON file"""
        with open(f"{self.output_dir}/annotations.json", "w") as f:
            json.dump(self.annotations, f, indent=2)

# Usage example
def main():
    world = World(stage_units_in_meters=1.0)

    # Add camera to scene
    camera = Camera(
        prim_path="/World/Robot/Camera",
        frequency=30,
        resolution=(640, 480)
    )

    # Create data generator
    generator = SyntheticDataGenerator(world, camera)

    # Collect data
    for i in range(1000):  # Collect 1000 frames
        # Randomize scene
        randomize_lighting(world)
        randomize_textures(world)

        # Move objects randomly
        randomize_object_poses(world)

        # Capture frame
        generator.capture_frame()

        # Step simulation
        world.step(render=True)

    # Save annotations
    generator.save_annotations()
```

## Semantic Segmentation Data

Semantic segmentation requires pixel-level labels for each object class:

```python
def generate_segmentation_labels(world, camera):
    """Generate semantic segmentation labels"""
    # Enable semantic segmentation
    camera.add_semantic_segmentation_render_product("/World/Robot/Camera")

    # Get segmentation data
    seg_data = camera.get_semantic_segmentation()

    # Create class mapping
    class_mapping = {
        "robot": 1,
        "object": 2,
        "background": 0,
        "floor": 3,
        # Add more classes as needed
    }

    # Apply mapping to create labeled image
    labeled_image = np.zeros(seg_data.shape, dtype=np.uint8)
    for class_name, class_id in class_mapping.items():
        mask = (seg_data == class_name)
        labeled_image[mask] = class_id

    return labeled_image
```

## Instance Segmentation

Instance segmentation distinguishes between different instances of the same class:

```python
def generate_instance_labels(world, camera):
    """Generate instance segmentation labels"""
    # Get instance segmentation data
    instance_data = camera.get_instance_segmentation()

    # Create unique IDs for each instance
    unique_instances = np.unique(instance_data)

    instance_labels = np.zeros(instance_data.shape, dtype=np.uint16)
    for i, instance_id in enumerate(unique_instances):
        mask = (instance_data == instance_id)
        instance_labels[mask] = i + 1  # Start from 1, 0 is background

    return instance_labels
```

## Depth and Normal Data

Depth and surface normal data are crucial for 3D understanding:

```python
def capture_3d_data(camera):
    """Capture depth and normal data"""
    # Depth data
    depth_data = camera.get_depth()

    # Surface normals
    normal_data = camera.get_normals()

    # Convert to formats suitable for training
    depth_normalized = (depth_data - depth_data.min()) / (depth_data.max() - depth_data.min())
    normal_normalized = (normal_data + 1) / 2  # Normalize from [-1,1] to [0,1]

    return depth_normalized, normal_normalized
```

## Object Detection Annotations

For object detection tasks, we need bounding box annotations:

```python
def generate_bounding_boxes(world, camera):
    """Generate 2D bounding box annotations"""
    annotations = []

    # Get all objects in the scene
    objects = world.scene.get_all_objects()

    for obj in objects:
        # Get 3D bounding box in world coordinates
        bbox_3d = obj.get_world_bounding_box()

        # Project to 2D camera coordinates
        bbox_2d = project_3d_bbox_to_2d(bbox_3d, camera.get_intrinsics())

        # Create annotation
        annotation = {
            "object_name": obj.name,
            "bbox_2d": bbox_2d.tolist(),
            "bbox_3d": {
                "center": bbox_3d.center.tolist(),
                "size": bbox_3d.size.tolist()
            },
            "visibility": calculate_visibility(bbox_2d, camera.resolution)
        }

        annotations.append(annotation)

    return annotations

def project_3d_bbox_to_2d(bbox_3d, camera_intrinsics):
    """Project 3D bounding box to 2D image coordinates"""
    # Implementation of 3D to 2D projection
    # This involves using camera intrinsic matrix
    # and projecting 3D points to 2D image plane
    pass
```

## Practical Exercise: Create a Custom Dataset

Create a synthetic dataset for object detection with the following requirements:

1. **Scene Setup**: Create a scene with multiple objects of different classes
2. **Randomization**: Implement domain randomization for lighting and textures
3. **Data Capture**: Capture RGB, depth, and bounding box annotations
4. **Variety**: Generate 1000 diverse frames with objects in different positions

### Implementation Steps:

1. **Setup the scene**:
   ```python
   # Create a scene with various objects
   objects = [
       {"name": "cube", "type": "cube", "color": "red"},
       {"name": "sphere", "type": "sphere", "color": "blue"},
       {"name": "cylinder", "type": "cylinder", "color": "green"}
   ]
   ```

2. **Implement randomization**:
   - Random object positions
   - Random lighting conditions
   - Random backgrounds
   - Random camera viewpoints

3. **Capture and save data**:
   - RGB images
   - Depth maps
   - Annotations in COCO format

## COCO Format Annotations

For compatibility with popular computer vision frameworks:

```python
def create_coco_annotations(frames_data, categories):
    """Create COCO format annotations"""
    coco_format = {
        "info": {
            "description": "Synthetic Dataset",
            "version": "1.0",
            "year": 2024
        },
        "licenses": [{"id": 1, "name": "Synthetic Data License"}],
        "categories": categories,
        "images": [],
        "annotations": []
    }

    annotation_id = 1
    for frame_idx, frame in enumerate(frames_data):
        # Add image info
        image_info = {
            "id": frame_idx,
            "file_name": frame["filename"],
            "width": 640,
            "height": 480,
            "date_captured": "2024"
        }
        coco_format["images"].append(image_info)

        # Add annotations
        for obj in frame["objects"]:
            bbox = obj["bbox_2d"]
            annotation = {
                "id": annotation_id,
                "image_id": frame_idx,
                "category_id": obj["category_id"],
                "bbox": [bbox[0], bbox[1], bbox[2]-bbox[0], bbox[3]-bbox[1]],  # x, y, width, height
                "area": (bbox[2]-bbox[0]) * (bbox[3]-bbox[1]),
                "iscrowd": 0
            }
            coco_format["annotations"].append(annotation)
            annotation_id += 1

    return coco_format
```

## Quality Assurance

Ensure synthetic data quality:

1. **Visual Inspection**: Manually check a sample of generated images
2. **Statistical Analysis**: Verify distribution of objects, lighting, etc.
3. **Consistency Checks**: Ensure annotations match the visual content
4. **Realism Verification**: Compare synthetic and real data distributions

## Summary

This lesson covered synthetic data generation techniques in Isaac Sim, including RGB, depth, segmentation, and detection datasets. Synthetic data is crucial for training robust AI models in robotics applications.

## Next Steps

In the next lesson, we'll explore Isaac ROS packages and how to bridge synthetic data with real-world robotics applications.