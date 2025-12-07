---
title: Lesson 1 - Vision-Language Fundamentals
sidebar_position: 4
---

# Lesson 1: Vision-Language Fundamentals

## Learning Objectives

By the end of this lesson, students will be able to:
- Understand the fundamentals of vision-language models
- Implement basic visual question answering systems
- Connect computer vision outputs to language models
- Create simple vision-language pipelines

## Introduction to Vision-Language Models

Vision-Language (VL) models represent a significant advancement in AI, bridging the gap between visual perception and natural language understanding. These models enable robots to understand and respond to commands based on visual input, making human-robot interaction more intuitive.

### Key Concepts in Vision-Language AI

1. **Visual Grounding**: Connecting visual elements to linguistic concepts
2. **Multimodal Fusion**: Combining visual and textual information
3. **Cross-Modal Attention**: Focusing on relevant visual regions based on language
4. **Embodied Language Understanding**: Connecting language to physical actions

## Vision-Language Model Architectures

### CLIP (Contrastive Language-Image Pretraining)

CLIP represents one of the foundational architectures for vision-language understanding:

```python
import torch
import clip
from PIL import Image

# Load the model
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

# Prepare images and text
image = preprocess(Image.open("image.png")).unsqueeze(0).to(device)
text = clip.tokenize(["a photo of a cat", "a photo of a dog"]).to(device)

# Get predictions
with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    logits_per_image, logits_per_text = model(image, text)
    probs = logits_per_image.softmax(dim=-1).cpu().numpy()

print("Label probs:", probs)  # prints: [[0.9927937 0.0072063]]
```

### BLIP (Bootstrapping Language-Image Pre-training)

BLIP combines vision and language models for tasks like image captioning:

```python
from PIL import Image
from transformers import BlipProcessor, BlipForConditionalGeneration

# Load processor and model
processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")

# Load and process image
raw_image = Image.open("image.jpg").convert('RGB')
inputs = processor(raw_image, return_tensors="pt")

# Generate caption
out = model.generate(**inputs)
caption = processor.decode(out[0], skip_special_tokens=True)
print(f"Generated caption: {caption}")
```

## Visual Question Answering (VQA)

Visual Question Answering combines computer vision with natural language processing:

```python
import torch
from transformers import ViltProcessor, ViltForQuestionAnswering
from PIL import Image

# Load processor and model
processor = ViltProcessor.from_pretrained("dandelin/vilt-b32-finetuned-vqa")
model = ViltForQuestionAnswering.from_pretrained("dandelin/vilt-b32-finetuned-vqa")

# Prepare inputs
text = "How many cats are there?"
image = Image.open("cats_image.jpg")

inputs = processor(image, text, return_tensors="pt")

# Forward pass
with torch.no_grad():
    outputs = model(**inputs)
    logits = outputs.logits
    idx = logits.argmax(-1).item()

# Get answer
answer = model.config.id2label[idx]
print(f"Answer: {answer}")
```

## Implementing Vision-Language Pipelines

### Basic Object Detection + Language Integration

```python
import cv2
import numpy as np
import openai
from transformers import CLIPProcessor, CLIPModel
import torch

class VisionLanguagePipeline:
    def __init__(self):
        # Initialize models
        self.clip_model, self.clip_processor = CLIPModel.from_pretrained(
            "openai/clip-vit-base-patch32"
        ), CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

        # Object detection model (using YOLO as example)
        self.yolo_net = cv2.dnn.readNetFromDarknet("yolo_config.cfg", "yolo_weights.weights")
        self.yolo_classes = self.load_classes("coco.names")

    def detect_objects(self, image_path):
        """Detect objects in an image using YOLO"""
        image = cv2.imread(image_path)
        height, width, channels = image.shape

        # Create blob from image
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.yolo_net.setInput(blob)
        outputs = self.yolo_net.forward(self.get_output_layers())

        # Process outputs
        boxes, confidences, class_ids = self.process_yolo_output(outputs, width, height)

        # Draw bounding boxes
        detections = []
        for i in range(len(boxes)):
            x, y, w, h = boxes[i]
            class_name = self.yolo_classes[class_ids[i]]
            confidence = confidences[i]

            detections.append({
                "class": class_name,
                "confidence": confidence,
                "bbox": [x, y, w, h],
                "center": [x + w//2, y + h//2]
            })

        return detections, image

    def describe_objects(self, image_path, detections):
        """Generate descriptions for detected objects"""
        image = self.clip_processor(images=cv2.imread(image_path), return_tensors="pt")

        # Create text prompts for each detected object
        text_prompts = [f"a photo of {det['class']}" for det in detections]
        inputs = self.clip_processor(text=text_prompts, images=cv2.imread(image_path),
                                     return_tensors="pt", padding=True)

        # Get similarity scores
        outputs = self.clip_model(**inputs)
        logits_per_image = outputs.logits_per_image
        probs = logits_per_image.softmax(dim=-1)

        # Associate probabilities with detections
        for i, det in enumerate(detections):
            det["similarity"] = probs[0][i].item()

        return detections

    def answer_questions(self, image_path, questions):
        """Answer questions about the image"""
        answers = []
        image = cv2.imread(image_path)

        for question in questions:
            # This would typically use a VQA model
            # For simplicity, we'll use a basic approach
            detections = self.detect_objects(image_path)

            # Generate answer based on detections
            answer = self.generate_answer_from_detections(detections, question)
            answers.append({
                "question": question,
                "answer": answer
            })

        return answers

    def get_output_layers(self):
        """Get output layer names"""
        layer_names = self.yolo_net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.yolo_net.getUnconnectedOutLayers()]
        return output_layers

    def process_yolo_output(self, outputs, width, height):
        """Process YOLO outputs to get bounding boxes"""
        # Implementation for processing YOLO outputs
        # This is a simplified version
        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:  # Confidence threshold
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        filtered_boxes = [boxes[i] for i in indices]
        filtered_confidences = [confidences[i] for i in indices]
        filtered_class_ids = [class_ids[i] for i in indices]

        return filtered_boxes, filtered_confidences, filtered_class_ids

    def load_classes(self, file_path):
        """Load class names from file"""
        with open(file_path, "r") as f:
            classes = [line.strip() for line in f.readlines()]
        return classes

    def generate_answer_from_detections(self, detections, question):
        """Generate answer based on detections"""
        # This is a simplified implementation
        # In practice, you would use a specialized VQA model

        if "how many" in question.lower():
            unique_classes = set(det["class"] for det in detections)
            counts = {cls: sum(1 for det in detections if det["class"] == cls)
                     for cls in unique_classes}

            if len(unique_classes) == 1:
                return f"There are {counts[list(unique_classes)[0]]} {list(unique_classes)[0]}(s)"
            else:
                return f"Multiple objects detected: {', '.join([f'{count} {cls}' for cls, count in counts.items()])}"

        elif "what color" in question.lower():
            # Simplified color detection
            return "Color detection requires additional processing"

        else:
            return f"Detected objects: {', '.join([det['class'] for det in detections[:5]])}"  # Limit to 5
```

## Vision-Language Integration with ROS 2

### Creating a Vision-Language ROS 2 Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_language_interfaces.msg import VisionLanguageResponse
import cv2
from cv_bridge import CvBridge

class VisionLanguageNode(Node):
    def __init__(self):
        super().__init__('vision_language_node')

        # Initialize the pipeline
        self.vl_pipeline = VisionLanguagePipeline()
        self.bridge = CvBridge()

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.question_sub = self.create_subscription(
            String,
            'vl_question',
            self.question_callback,
            10
        )

        self.response_pub = self.create_publisher(
            VisionLanguageResponse,
            'vl_response',
            10
        )

        self.current_image = None
        self.get_logger().info('Vision-Language node initialized')

    def image_callback(self, msg):
        """Handle incoming image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Save the image temporarily for processing
            cv2.imwrite('/tmp/current_image.jpg', cv_image)
            self.current_image = '/tmp/current_image.jpg'
            self.get_logger().info('Image received and saved')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def question_callback(self, msg):
        """Handle incoming questions"""
        if self.current_image is None:
            self.get_logger().warn('No image available for processing')
            return

        try:
            # Process the question with the current image
            answers = self.vl_pipeline.answer_questions(
                self.current_image,
                [msg.data]
            )

            # Create response message
            response_msg = VisionLanguageResponse()
            response_msg.question = msg.data
            response_msg.answer = answers[0]['answer']
            response_msg.timestamp = self.get_clock().now().to_msg()

            # Publish response
            self.response_pub.publish(response_msg)
            self.get_logger().info(f'Answered: {answers[0]["answer"]}')
        except Exception as e:
            self.get_logger().error(f'Error processing question: {e}')
```

## Cross-Modal Attention Mechanisms

Cross-modal attention allows the model to focus on relevant parts of one modality based on information from another:

```python
import torch
import torch.nn as nn

class CrossModalAttention(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.query_projection = nn.Linear(dim, dim)
        self.key_projection = nn.Linear(dim, dim)
        self.value_projection = nn.Linear(dim, dim)
        self.scale = dim ** -0.5

    def forward(self, visual_features, language_features):
        """
        Apply cross-modal attention
        visual_features: [batch_size, num_visual_tokens, dim]
        language_features: [batch_size, num_language_tokens, dim]
        """
        # Project features
        Q = self.query_projection(visual_features)
        K = self.key_projection(language_features)
        V = self.value_projection(language_features)

        # Calculate attention scores
        attention_scores = torch.matmul(Q, K.transpose(-2, -1)) * self.scale
        attention_weights = torch.softmax(attention_scores, dim=-1)

        # Apply attention to values
        attended_features = torch.matmul(attention_weights, V)

        return attended_features

# Usage in a multimodal model
class MultimodalFusion(nn.Module):
    def __init__(self, dim=512):
        super().__init__()
        self.visual_encoder = nn.Linear(2048, dim)  # For ResNet features
        self.text_encoder = nn.Linear(768, dim)     # For BERT features
        self.cross_attention = CrossModalAttention(dim)
        self.classifier = nn.Linear(dim, num_classes)

    def forward(self, visual_input, text_input):
        # Encode modalities
        visual_features = self.visual_encoder(visual_input)
        text_features = self.text_encoder(text_input)

        # Apply cross-modal attention
        attended_visual = self.cross_attention(visual_features, text_features)

        # Combine features (simple concatenation or more complex fusion)
        combined_features = attended_visual.mean(dim=1)  # Pool visual tokens

        # Classify
        output = self.classifier(combined_features)
        return output
```

## Practical Exercise: Build a Simple VQA System

Create a basic visual question answering system that can answer simple questions about objects in an image:

1. **Setup**: Install required packages (transformers, torch, opencv)
2. **Object Detection**: Implement basic object detection
3. **Question Processing**: Parse simple questions
4. **Answer Generation**: Generate answers based on detected objects
5. **Integration**: Create a ROS 2 node that processes camera images and questions

### Example Implementation:

```python
def simple_vqa_system(image_path, question):
    """
    Simple VQA system that answers basic questions about objects in an image
    """
    # Load image
    image = cv2.imread(image_path)

    # Detect objects (using a pre-trained model)
    detections = detect_objects_yolo(image)

    # Parse question
    question_lower = question.lower()

    if "how many" in question_lower:
        # Count objects
        object_counts = {}
        for det in detections:
            obj_class = det['class']
            object_counts[obj_class] = object_counts.get(obj_class, 0) + 1

        if len(object_counts) == 1:
            obj_type = list(object_counts.keys())[0]
            count = object_counts[obj_type]
            return f"There are {count} {obj_type}(s)"
        else:
            result = []
            for obj_type, count in object_counts.items():
                result.append(f"{count} {obj_type}")
            return f"I see: {', '.join(result)}"

    elif "where is" in question_lower or "location" in question_lower:
        # Find specific object
        target_object = extract_object_from_question(question)
        locations = []

        for det in detections:
            if target_object in det['class'] or det['class'] in target_object:
                center_x, center_y = det['center']
                # Convert to relative position
                h, w = image.shape[:2]
                if center_y < h/3:
                    vertical_pos = "top"
                elif center_y < 2*h/3:
                    vertical_pos = "middle"
                else:
                    vertical_pos = "bottom"

                if center_x < w/3:
                    horizontal_pos = "left"
                elif center_x < 2*w/3:
                    horizontal_pos = "center"
                else:
                    horizontal_pos = "right"

                locations.append(f"{vertical_pos} {horizontal_pos}")

        if locations:
            return f"The {target_object} is in the {locations[0]} of the image"
        else:
            return f"I don't see any {target_object} in the image"

    else:
        # General description
        unique_objects = list(set([det['class'] for det in detections]))
        return f"I see: {', '.join(unique_objects[:5])}"  # Limit to 5 objects

def extract_object_from_question(question):
    """Extract object name from question"""
    # Simple extraction - in practice, use NLP techniques
    import re
    # Look for common object words in the question
    words = question.lower().split()
    common_objects = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus",
                      "train", "truck", "boat", "traffic light", "fire hydrant",
                      "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                      "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                      "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                      "skis", "snowboard", "sports ball", "kite", "baseball bat",
                      "baseball glove", "skateboard", "surfboard", "tennis racket",
                      "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                      "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
                      "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant",
                      "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
                      "remote", "keyboard", "cell phone", "microwave", "oven", "toaster",
                      "sink", "refrigerator", "book", "clock", "vase", "scissors",
                      "teddy bear", "hair drier", "toothbrush"]

    for word in words:
        # Remove common question words
        if word in ['is', 'are', 'the', 'a', 'an', 'where', 'how', 'many', 'what', 'on', 'in', 'at']:
            continue
        # Check if word matches any common object
        for obj in common_objects:
            if word in obj or obj in word:
                return obj

    return "object"  # Default if no specific object found
```

## Evaluation Metrics for Vision-Language Models

### CIDEr (Consensus-based Image Description Evaluation)
For image captioning tasks

### VQA Accuracy
For visual question answering tasks

### F1-Score
For object detection and grounding tasks

## Summary

This lesson introduced the fundamentals of vision-language models and how they can be implemented for robotics applications. Vision-language integration is essential for creating robots that can understand and respond to natural language commands based on visual input.

## Next Steps

In the next lesson, we'll explore how to connect vision-language systems with robotic action planning and execution.