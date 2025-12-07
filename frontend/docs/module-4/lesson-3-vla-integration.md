---
title: Lesson 3 - VLA Integration and Autonomous Systems
sidebar_position: 6
---

# Lesson 3: VLA Integration and Autonomous Systems

## Learning Objectives

By the end of this lesson, students will be able to:
- Integrate Vision-Language-Action systems for autonomous robot control
- Implement end-to-end pipelines from voice commands to robot actions
- Create conversational interfaces for robot interaction
- Build autonomous humanoid demonstration systems

## Introduction to VLA Integration

Vision-Language-Action (VLA) integration represents the convergence of three critical AI technologies that enable truly autonomous robots. This integration allows robots to understand natural language commands, perceive their environment visually, and execute complex actions in response.

### VLA System Architecture

```
Voice Command → NLP → Task Planning → Action Execution → Environment → Perception → Feedback
     ↑              ↓                    ↓                    ↓              ↑         ↑
   (Human)     (Understanding)    (Planning)          (Execution)    (Sensors) (Feedback)
```

## Complete VLA System Implementation

### Voice Command Processing Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from vision_language_interfaces.msg import VisionLanguageCommand, VisionLanguageResponse
import speech_recognition as sr
import openai
import asyncio
import threading
from queue import Queue
from cv_bridge import CvBridge

class VLACommandProcessor(Node):
    def __init__(self):
        super().__init__('vla_command_processor')

        # Initialize components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.bridge = CvBridge()

        # Configuration
        self.llm_api_key = "your-openai-api-key"  # In practice, use secure storage
        openai.api_key = self.llm_api_key

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/vla/voice_command', self.command_callback, 10)

        # Create publishers
        self.response_pub = self.create_publisher(
            VisionLanguageResponse, '/vla/response', 10)
        self.action_pub = self.create_publisher(
            String, '/robot/action_command', 10)

        # Initialize state
        self.current_image = None
        self.command_queue = Queue()
        self.processing_lock = threading.Lock()

        # Start voice recognition thread
        self.voice_thread = threading.Thread(target=self.voice_recognition_loop, daemon=True)
        self.voice_thread.start()

        self.get_logger().info('VLA Command Processor initialized')

    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().debug('Image updated')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def command_callback(self, msg):
        """Handle text commands"""
        self.get_logger().info(f'Received command: {msg.data}')
        self.process_command(msg.data)

    def voice_recognition_loop(self):
        """Continuous voice recognition in separate thread"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)

                # Recognize speech
                command = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Recognized: {command}')

                # Publish recognized command
                cmd_msg = String()
                cmd_msg.data = command
                self.command_callback(cmd_msg)

            except sr.WaitTimeoutError:
                # No speech detected, continue loop
                continue
            except sr.UnknownValueError:
                self.get_logger().warn('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
            except Exception as e:
                self.get_logger().error(f'Voice recognition error: {e}')

    def process_command(self, command):
        """Process a natural language command"""
        with self.processing_lock:
            try:
                # Step 1: Parse command using LLM
                parsed_command = self.parse_command_with_llm(command)

                # Step 2: Get visual context
                visual_context = self.get_visual_context()

                # Step 3: Generate action plan
                action_plan = self.generate_action_plan(parsed_command, visual_context)

                # Step 4: Execute action plan
                result = self.execute_action_plan(action_plan)

                # Step 5: Generate response
                response = self.generate_response(command, action_plan, result)

                # Publish response
                response_msg = VisionLanguageResponse()
                response_msg.command = command
                response_msg.response = response
                response_msg.success = result['success']
                response_msg.timestamp = self.get_clock().now().to_msg()

                self.response_pub.publish(response_msg)

            except Exception as e:
                self.get_logger().error(f'Error processing command: {e}')
                error_response = VisionLanguageResponse()
                error_response.command = command
                error_response.response = f"Error processing command: {str(e)}"
                error_response.success = False
                self.response_pub.publish(error_response)

    def parse_command_with_llm(self, command):
        """Parse command using LLM"""
        prompt = f"""
        Parse the following natural language command into structured action:
        Command: "{command}"

        Return a JSON object with:
        - action: primary action (e.g., "navigate", "grasp", "clean", "inspect")
        - target_object: target object if applicable
        - target_location: target location if applicable
        - additional_parameters: any additional parameters

        Example response:
        {{
            "action": "grasp",
            "target_object": "red cup",
            "target_location": null,
            "additional_parameters": {{"height": "table_level"}}
        }}
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                max_tokens=200
            )

            import json
            result = json.loads(response.choices[0].message['content'])
            return result
        except Exception as e:
            self.get_logger().error(f'LLM parsing error: {e}')
            # Fallback to simple parsing
            return self.fallback_parse_command(command)

    def fallback_parse_command(self, command):
        """Simple command parsing as fallback"""
        command_lower = command.lower()

        if any(word in command_lower for word in ['go to', 'navigate', 'move to']):
            return {"action": "navigate", "target_object": None, "target_location": "specified location", "additional_parameters": {}}
        elif any(word in command_lower for word in ['pick up', 'grasp', 'get', 'take']):
            return {"action": "grasp", "target_object": "object", "target_location": None, "additional_parameters": {}}
        elif any(word in command_lower for word in ['clean', 'wipe', 'dust']):
            return {"action": "clean", "target_object": "surface", "target_location": None, "additional_parameters": {}}
        else:
            return {"action": "unknown", "target_object": None, "target_location": None, "additional_parameters": {}}

    def get_visual_context(self):
        """Get visual context from current image"""
        if self.current_image is None:
            return {"objects": [], "locations": [], "description": "No visual data available"}

        # In real implementation, this would use object detection, segmentation, etc.
        # For demonstration, we'll return mock data
        return {
            "objects": ["red cup", "blue book", "white table", "black chair"],
            "locations": ["kitchen", "living room", "bedroom"],
            "description": "A room with furniture and objects"
        }

    def generate_action_plan(self, parsed_command, visual_context):
        """Generate detailed action plan based on parsed command and visual context"""
        action = parsed_command['action']
        target_object = parsed_command['target_object']
        target_location = parsed_command['target_location']

        plan = []

        if action == 'grasp' and target_object:
            # Find object in visual context
            if target_object in visual_context['objects']:
                plan = [
                    {"action": "find_object", "parameters": {"object_name": target_object}},
                    {"action": "navigate_to_object", "parameters": {"object_name": target_object}},
                    {"action": "grasp_object", "parameters": {"object_name": target_object}},
                    {"action": "verify_grasp", "parameters": {"object_name": target_object}}
                ]
            else:
                # Object not found, search for it
                plan = [
                    {"action": "search_for_object", "parameters": {"object_name": target_object}},
                    {"action": "grasp_object", "parameters": {"object_name": target_object}},
                    {"action": "verify_grasp", "parameters": {"object_name": target_object}}
                ]

        elif action == 'navigate' and target_location:
            plan = [
                {"action": "plan_path", "parameters": {"destination": target_location}},
                {"action": "execute_navigation", "parameters": {"destination": target_location}},
                {"action": "verify_arrival", "parameters": {"destination": target_location}}
            ]

        elif action == 'clean':
            plan = [
                {"action": "find_surface", "parameters": {"surface_type": "cleanable"}},
                {"action": "navigate_to_surface", "parameters": {"surface_type": "cleanable"}},
                {"action": "clean_surface", "parameters": {"surface_type": "cleanable"}},
                {"action": "verify_cleaning", "parameters": {"surface_type": "cleanable"}}
            ]

        else:
            plan = [{"action": "unknown_command", "parameters": {"command": parsed_command}}]

        return plan

    async def execute_action_plan_async(self, plan):
        """Execute action plan asynchronously"""
        results = []

        for step in plan:
            self.get_logger().info(f'Executing: {step["action"]}')

            # In real implementation, this would call actual robot actions
            # For demonstration, we'll simulate execution
            result = await self.execute_single_action(step)
            results.append(result)

            if not result['success']:
                self.get_logger().error(f'Action failed: {step["action"]}')
                break

        return {"success": all(r['success'] for r in results), "results": results}

    def execute_single_action(self, action_step):
        """Execute a single action step"""
        # Simulate action execution
        import asyncio
        asyncio.sleep(0.1)  # Simulate execution time

        # In real implementation, this would interface with robot controllers
        # For now, we'll return success for demonstration
        return {
            "action": action_step["action"],
            "success": True,
            "details": f"Executed {action_step['action']} with parameters {action_step['parameters']}"
        }

    def execute_action_plan(self, plan):
        """Execute action plan (synchronous wrapper)"""
        # In a real implementation, this would use asyncio properly
        # For demonstration, we'll use a simplified approach
        results = []

        for step in plan:
            self.get_logger().info(f'Executing: {step["action"]}')

            # Simulate execution
            result = self.execute_single_action(step)
            results.append(result)

            if not result['success']:
                break

        return {"success": all(r['success'] for r in results), "results": results}

    def generate_response(self, original_command, action_plan, execution_result):
        """Generate natural language response"""
        if execution_result['success']:
            return f"I have completed the task: {original_command}. All actions were successful."
        else:
            return f"I encountered an issue while executing: {original_command}. Some actions failed."

def main(args=None):
    rclpy.init(args=args)
    processor = VLACommandProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conversational Interface for Robots

### Dialogue Manager Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_language_interfaces.msg import VisionLanguageResponse
import openai
import json
from collections import deque

class RobotDialogueManager(Node):
    def __init__(self):
        super().__init__('robot_dialogue_manager')

        # Initialize LLM
        self.llm_api_key = "your-openai-api-key"
        openai.api_key = self.llm_api_key

        # Create subscribers
        self.user_input_sub = self.create_subscription(
            String, '/user_input', self.user_input_callback, 10)
        self.vla_response_sub = self.create_subscription(
            VisionLanguageResponse, '/vla/response', self.vla_response_callback, 10)

        # Create publishers
        self.robot_speech_pub = self.create_publisher(
            String, '/robot/speech_output', 10)
        self.vla_command_pub = self.create_publisher(
            String, '/vla/voice_command', 10)

        # Initialize conversation state
        self.conversation_history = deque(maxlen=10)  # Keep last 10 exchanges
        self.waiting_for_vla_response = False
        self.pending_user_query = None

        self.get_logger().info('Robot Dialogue Manager initialized')

    def user_input_callback(self, msg):
        """Handle user input"""
        user_text = msg.data
        self.get_logger().info(f'User said: {user_text}')

        # Add to conversation history
        self.conversation_history.append({"role": "user", "content": user_text})

        # Process the input
        response = self.process_user_input(user_text)

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.robot_speech_pub.publish(response_msg)

        # Add to conversation history
        self.conversation_history.append({"role": "assistant", "content": response})

    def vla_response_callback(self, msg):
        """Handle VLA system response"""
        if self.waiting_for_vla_response and self.pending_user_query:
            # Generate a conversational response based on VLA result
            response = self.generate_conversational_response(
                self.pending_user_query, msg.response, msg.success)

            # Publish response
            response_msg = String()
            response_msg.data = response
            self.robot_speech_pub.publish(response_msg)

            # Add to conversation history
            self.conversation_history.append({"role": "assistant", "content": response})

            # Reset state
            self.waiting_for_vla_response = False
            self.pending_user_query = None

    def process_user_input(self, user_input):
        """Process user input and determine appropriate response"""
        # Check if this is a command that needs VLA processing
        if self.is_command_that_needs_vla(user_input):
            # Forward to VLA system
            self.vla_command_pub.publish(String(data=user_input))

            # Set state to wait for VLA response
            self.waiting_for_vla_response = True
            self.pending_user_query = user_input

            # Return acknowledgment
            return "I'm processing your request. Please wait a moment."
        else:
            # Handle as a conversational query
            return self.handle_conversational_query(user_input)

    def is_command_that_needs_vla(self, text):
        """Determine if text is a command requiring VLA processing"""
        action_keywords = [
            'go to', 'navigate', 'move to', 'pick up', 'grasp', 'get', 'take',
            'put', 'place', 'clean', 'wipe', 'find', 'locate', 'bring',
            'move the', 'pick the', 'go', 'walk', 'drive', 'go there'
        ]

        text_lower = text.lower()
        return any(keyword in text_lower for keyword in action_keywords)

    def handle_conversational_query(self, query):
        """Handle conversational queries (not commands)"""
        # Use LLM for conversational responses
        messages = list(self.conversation_history)
        messages.append({"role": "user", "content": query})

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=messages,
                temperature=0.7,
                max_tokens=150
            )

            return response.choices[0].message['content']
        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
            return "I'm sorry, I didn't understand that. Could you please rephrase?"

    def generate_conversational_response(self, original_query, vla_result, success):
        """Generate a conversational response based on VLA result"""
        # Use LLM to create a natural response
        prompt = f"""
        Original query: "{original_query}"
        VLA system result: "{vla_result}"
        Success: {success}

        Generate a natural, conversational response that acknowledges the result.
        Make it sound like a helpful robot assistant responding to a human.
        Keep it concise but friendly.

        Response:
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.7,
                max_tokens=100
            )

            return response.choices[0].message['content']
        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
            return f"I've completed the task. The result was: {vla_result}"

def main(args=None):
    rclpy.init(args=args)
    dialogue_manager = RobotDialogueManager()

    try:
        rclpy.spin(dialogue_manager)
    except KeyboardInterrupt:
        pass
    finally:
        dialogue_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Autonomous Humanoid System

### Complete Autonomous System Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from vision_language_interfaces.msg import VisionLanguageResponse
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import asyncio
import threading
from queue import Queue
import time

class AutonomousHumanoidSystem(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_system')

        # Initialize components
        self.bridge = CvBridge()
        self.command_queue = Queue()
        self.emergency_stop = False

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.voice_command_sub = self.create_subscription(
            String, '/vla/voice_command', self.voice_command_callback, 10)
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.speech_pub = self.create_publisher(String, '/robot/speech_output', 10)
        self.status_pub = self.create_publisher(String, '/system_status', 10)

        # Initialize state
        self.current_image = None
        self.current_pose = None
        self.joint_states = {}
        self.system_active = True

        # Start autonomous processing thread
        self.autonomous_thread = threading.Thread(target=self.autonomous_loop, daemon=True)
        self.autonomous_thread.start()

        self.get_logger().info('Autonomous Humanoid System initialized')

    def image_callback(self, msg):
        """Handle camera images"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.current_pose = msg.pose.pose

    def joint_state_callback(self, msg):
        """Handle joint state data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]

    def voice_command_callback(self, msg):
        """Handle voice commands"""
        self.get_logger().info(f'Received voice command: {msg.data}')
        self.command_queue.put(msg.data)

    def emergency_stop_callback(self, msg):
        """Handle emergency stop"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.stop_robot()

    def stop_robot(self):
        """Stop all robot motion"""
        # Stop base movement
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Stop joint movements
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        self.joint_cmd_pub.publish(joint_cmd)

    def autonomous_loop(self):
        """Main autonomous processing loop"""
        while rclpy.ok() and self.system_active:
            try:
                # Check for new commands
                if not self.command_queue.empty():
                    command = self.command_queue.get()
                    self.process_command(command)

                # Perform autonomous behaviors
                self.perform_autonomous_behaviors()

                # Check system status
                self.check_system_status()

                # Small delay to prevent excessive CPU usage
                time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f'Autonomous loop error: {e}')
                time.sleep(0.1)  # Brief pause before continuing

    def process_command(self, command):
        """Process a high-level command"""
        if self.emergency_stop:
            self.speak("System is in emergency stop mode. Please clear the emergency before proceeding.")
            return

        try:
            # Parse command and execute appropriate behavior
            if "clean" in command.lower():
                self.execute_cleaning_task()
            elif "navigate" in command.lower() or "go to" in command.lower():
                self.execute_navigation_task(command)
            elif "grasp" in command.lower() or "pick up" in command.lower():
                self.execute_manipulation_task(command)
            else:
                self.speak(f"I received the command: {command}. I'm working on it.")

        except Exception as e:
            self.get_logger().error(f'Command execution error: {e}')
            self.speak("I encountered an error while executing the command.")

    def execute_cleaning_task(self):
        """Execute cleaning task"""
        self.speak("Starting cleaning task.")

        # Example cleaning sequence
        cleaning_sequence = [
            ("inspect_area", {}),
            ("navigate_to_dirty_spot", {"target": "kitchen_counter"}),
            ("clean_surface", {"surface": "counter", "method": "wipe"}),
            ("inspect_result", {}),
            ("report_completion", {})
        ]

        for action, params in cleaning_sequence:
            if self.emergency_stop:
                break
            self.execute_single_action(action, params)

        self.speak("Cleaning task completed.")

    def execute_navigation_task(self, command):
        """Execute navigation task"""
        self.speak(f"Navigating as requested: {command}")

        # In real implementation, this would parse the destination
        # and execute navigation to that location
        # For demonstration, we'll just move forward
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # Move forward slowly
        cmd_vel.angular.z = 0.0

        # Publish for 2 seconds
        for _ in range(20):  # 20 iterations * 0.1s = 2 seconds
            if self.emergency_stop:
                break
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)

        self.speak("Navigation task completed.")

    def execute_manipulation_task(self, command):
        """Execute manipulation task"""
        self.speak(f"Performing manipulation: {command}")

        # In real implementation, this would involve:
        # 1. Object detection and localization
        # 2. Motion planning for arms
        # 3. Grasping execution
        # 4. Verification
        # For demonstration, we'll just report completion

        self.speak("Manipulation task completed.")

    def execute_single_action(self, action, params):
        """Execute a single action with parameters"""
        self.get_logger().info(f'Executing action: {action} with params: {params}')

        # Map action to robot commands
        if action == "navigate_to_dirty_spot":
            # Navigate to a specific location
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.1
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(1.0)
        elif action == "clean_surface":
            # Simulate cleaning action
            self.get_logger().info(f'Cleaning {params.get("surface", "unknown")} surface')
            time.sleep(2.0)
        elif action == "inspect_area":
            # Use sensors to inspect area
            if self.current_image is not None:
                self.get_logger().info('Area inspection completed')
            time.sleep(0.5)

    def perform_autonomous_behaviors(self):
        """Perform background autonomous behaviors"""
        # Safety monitoring
        if self.current_pose:
            # Check if robot is in safe position
            pass

        # Environmental monitoring
        if self.current_image:
            # Process image for obstacles, people, etc.
            pass

        # System health monitoring
        if time.time() % 10 < 0.1:  # Every 10 seconds
            self.publish_system_status()

    def check_system_status(self):
        """Check overall system status"""
        # Check if all required nodes are running
        # Check battery level (if available)
        # Check for errors or warnings
        pass

    def publish_system_status(self):
        """Publish system status"""
        status_msg = String()
        status_msg.data = f"Active - Image: {'Yes' if self.current_image is not None else 'No'}, " \
                         f"Pose: {'Yes' if self.current_pose is not None else 'No'}, " \
                         f"Joints: {len(self.joint_states)} available"
        self.status_pub.publish(status_msg)

    def speak(self, text):
        """Publish speech output"""
        speech_msg = String()
        speech_msg.data = text
        self.speech_pub.publish(speech_msg)
        self.get_logger().info(f'Robot says: {text}')

def main(args=None):
    rclpy.init(args=args)
    system = AutonomousHumanoidSystem()

    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    finally:
        system.system_active = False
        system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for Complete VLA System

### Complete VLA System Launch

```xml
<launch>
  <!-- Arguments -->
  <arg name="use_sim_time" default="false"/>
  <arg name="robot_namespace" default="robot"/>
  <arg name="camera_namespace" default="camera"/>

  <!-- VLA Command Processor -->
  <node pkg="vla_system" exec="vla_command_processor" name="vla_command_processor"
        namespace="$(var robot_namespace)" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <remap from="/camera/image_raw" to="$(var camera_namespace)/image_raw"/>
  </node>

  <!-- Robot Dialogue Manager -->
  <node pkg="vla_system" exec="robot_dialogue_manager" name="robot_dialogue_manager"
        namespace="$(var robot_namespace)" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Autonomous Humanoid System -->
  <node pkg="vla_system" exec="autonomous_humanoid_system" name="autonomous_humanoid_system"
        namespace="$(var robot_namespace)" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <remap from="/camera/image_raw" to="$(var camera_namespace)/image_raw"/>
    <remap from="/odom" to="odom"/>
    <remap from="/joint_states" to="joint_states"/>
  </node>

  <!-- Navigation Stack (if needed) -->
  <include file="$(find nav2_bringup)/launch/navigation_launch.py">
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
  </include>

  <!-- Manipulation Stack (if needed) -->
  <!-- Add manipulation stack launch if robot has arms -->

  <!-- RViz for visualization -->
  <node pkg="rviz2" exec="rviz2" name="rviz" args="-d $(find vla_system)/config/vla_system.rviz"
        output="screen"/>
</launch>
```

## Capstone Project: Autonomous Humanoid Demonstration

### Complete System Integration Example

The capstone project integrates all VLA components into a complete demonstration:

```python
#!/usr/bin/env python3
"""
Complete VLA System for Autonomous Humanoid Demonstration
This script coordinates all VLA components for the final project.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import time
import threading

class VLACapstoneSystem(Node):
    def __init__(self):
        super().__init__('vla_capstone_system')

        # Create publishers
        self.speech_pub = self.create_publisher(String, '/robot/speech_output', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # System state
        self.system_ready = False
        self.demo_running = False

        # Start initialization
        self.initialize_system()

    def initialize_system(self):
        """Initialize the complete VLA system"""
        self.speak("Initializing Vision-Language-Action system for autonomous humanoid demonstration.")

        # In a real system, this would wait for all required nodes
        time.sleep(2.0)  # Simulate initialization time

        self.system_ready = True
        self.speak("System initialization complete. Ready for demonstration commands.")

    def run_demo_sequence(self):
        """Run the complete demo sequence"""
        if not self.system_ready:
            self.speak("System not ready. Please wait for initialization.")
            return

        self.demo_running = True
        self.speak("Starting autonomous humanoid demonstration.")

        # Demo sequence
        demo_steps = [
            ("Introduce yourself", self.introduction),
            ("Navigate to kitchen", self.navigate_to_kitchen),
            ("Find and clean counter", self.clean_counter),
            ("Return to starting position", self.return_home),
            ("Demonstration complete", self.conclusion)
        ]

        for step_name, step_func in demo_steps:
            if not self.demo_running:
                break
            self.speak(f"Executing: {step_name}")
            step_func()
            time.sleep(1.0)  # Brief pause between steps

        self.speak("Autonomous humanoid demonstration completed successfully!")

    def introduction(self):
        """Introduce the system"""
        self.speak("Hello! I am an autonomous humanoid robot powered by Vision-Language-Action technology. "
                  "I can understand natural language commands, perceive my environment visually, "
                  "and execute complex tasks in response.")

    def navigate_to_kitchen(self):
        """Navigate to kitchen area"""
        self.speak("Navigating to the kitchen area.")
        # Simulate navigation
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # Move forward
        for _ in range(10):  # Move for 1 second
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)

        # Stop
        cmd_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def clean_counter(self):
        """Clean the counter"""
        self.speak("Locating and cleaning the kitchen counter.")
        # Simulate cleaning action
        time.sleep(3.0)
        self.speak("Counter cleaning completed. Surface is now clean.")

    def return_home(self):
        """Return to starting position"""
        self.speak("Returning to starting position.")
        # Simulate returning
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.2  # Move backward
        for _ in range(10):  # Move for 1 second
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)

        # Stop
        cmd_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def conclusion(self):
        """Conclude the demonstration"""
        self.speak("This concludes the Vision-Language-Action demonstration. "
                  "I have successfully interpreted natural language commands, "
                  "perceived my environment visually, and executed complex tasks autonomously. "
                  "Thank you for watching!")

    def speak(self, text):
        """Publish speech output"""
        speech_msg = String()
        speech_msg.data = text
        self.speech_pub.publish(speech_msg)
        self.get_logger().info(f'Robot says: {text}')

def main(args=None):
    rclpy.init(args=args)
    capstone_system = VLACapstoneSystem()

    try:
        # Run demo after a short delay to allow system to fully initialize
        time.sleep(3.0)
        capstone_system.run_demo_sequence()

        # Keep node alive briefly to ensure all messages are sent
        time.sleep(2.0)

    except KeyboardInterrupt:
        capstone_system.get_logger().info('Demo interrupted by user')
    finally:
        capstone_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Complete VLA System

Create a complete VLA demonstration system that:

1. **Integrates all components**: Vision, Language, and Action
2. **Processes natural language commands**: "Clean the counter", "Go to the kitchen"
3. **Performs autonomous behaviors**: Navigation, manipulation, interaction
4. **Provides feedback**: Natural language responses to user

### Implementation Steps:

1. **Setup the complete system** with all required nodes
2. **Create a launch file** that starts the entire VLA system
3. **Implement the capstone demonstration** that showcases all capabilities
4. **Test with voice commands** to validate the complete pipeline
5. **Deploy to simulation** (or real robot if available) to demonstrate functionality

## Troubleshooting VLA Systems

### 1. Integration Issues
- **Problem**: Components not communicating properly
- **Solution**: Check topic names, message types, and frame IDs

### 2. Performance Issues
- **Problem**: Slow response times or dropped messages
- **Solution**: Optimize data processing, adjust queue sizes, tune LLM calls

### 3. Safety Concerns
- **Problem**: Robot performing unsafe actions
- **Solution**: Implement safety checks, emergency stops, and validation layers

### 4. Recognition Errors
- **Problem**: Misunderstanding commands or misidentifying objects
- **Solution**: Improve training data, add validation steps, implement confirmation requests

## Summary

This lesson covered the complete integration of Vision-Language-Action systems for autonomous humanoid robots. We implemented components for voice processing, natural language understanding, task planning, action execution, and conversational interfaces. The capstone project demonstrates how all these components work together to create truly autonomous robots that can understand and respond to natural human commands.

## Next Steps

This concludes the Physical AI & Humanoid Robotics textbook. You now have the knowledge to:
- Build and program robots using ROS 2
- Create realistic simulations with Gazebo and Isaac Sim
- Implement AI-powered perception and decision-making
- Integrate vision, language, and action systems for autonomous operation
- Deploy complete robotic systems that can interact naturally with humans