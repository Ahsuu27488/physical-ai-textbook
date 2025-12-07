---
title: Lesson 2 - Cognitive Planning and Action Generation
sidebar_position: 5
---

# Lesson 2: Cognitive Planning and Action Generation

## Learning Objectives

By the end of this lesson, students will be able to:
- Implement cognitive planning systems that translate natural language to robot actions
- Design action libraries for robotic systems
- Create task decomposition algorithms
- Integrate language models with robot control systems

## Introduction to Cognitive Planning

Cognitive planning is the process of converting high-level natural language commands into executable robotic actions. This involves understanding the command, decomposing it into subtasks, planning the sequence of actions, and executing them on the robot.

### Key Components of Cognitive Planning

1. **Natural Language Understanding**: Parsing and interpreting human commands
2. **Task Decomposition**: Breaking complex tasks into simpler subtasks
3. **Action Planning**: Sequencing actions to achieve the goal
4. **Execution Monitoring**: Tracking progress and handling failures
5. **Feedback Generation**: Communicating with the human operator

## Natural Language Understanding for Robotics

### Command Parsing

Natural language commands for robots often follow patterns that can be parsed:

```python
import re
from typing import Dict, List, Tuple

class CommandParser:
    def __init__(self):
        # Define action patterns
        self.patterns = {
            'move': [
                r'move to (.+)',
                r'go to (.+)',
                r'go to the (.+)',
                r'approach (.+)',
                r'navigate to (.+)'
            ],
            'grasp': [
                r'pick up (.+)',
                r'grasp (.+)',
                r'grab (.+)',
                r'get (.+)',
                r'take (.+)'
            ],
            'place': [
                r'put (.+) on (.+)',
                r'place (.+) on (.+)',
                r'place (.+) at (.+)',
                r'put (.+) in (.+)'
            ],
            'clean': [
                r'clean (.+)',
                r'wipe (.+)',
                r'clean up (.+)'
            ]
        }

    def parse_command(self, command: str) -> Dict:
        """Parse a natural language command into structured action"""
        command_lower = command.lower().strip()

        for action_type, patterns in self.patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command_lower)
                if match:
                    # Extract parameters based on pattern groups
                    params = match.groups()
                    return {
                        'action': action_type,
                        'parameters': params,
                        'original_command': command
                    }

        # If no pattern matches, return as unknown
        return {
            'action': 'unknown',
            'parameters': [command],
            'original_command': command
        }

# Example usage
parser = CommandParser()
command = "Please pick up the red cup and place it on the table"
result = parser.parse_command(command)
print(f"Parsed command: {result}")
```

### Semantic Role Labeling

For more complex understanding, we can use semantic role labeling:

```python
import spacy

class SemanticParser:
    def __init__(self):
        # Load spaCy model with NER and parsing
        self.nlp = spacy.load("en_core_web_sm")

    def extract_entities_and_roles(self, command: str) -> Dict:
        """Extract entities and their roles from command"""
        doc = self.nlp(command)

        entities = {}
        for ent in doc.ents:
            entities[ent.text] = {
                'label': ent.label_,
                'description': self.get_entity_description(ent.label_)
            }

        # Extract action verbs
        verbs = []
        for token in doc:
            if token.pos_ == "VERB":
                verbs.append({
                    'text': token.text,
                    'lemma': token.lemma_,
                    'children': [child.text for child in token.children]
                })

        return {
            'entities': entities,
            'verbs': verbs,
            'pos_tags': [(token.text, token.pos_) for token in doc]
        }

    def get_entity_description(self, label: str) -> str:
        """Get description for entity label"""
        descriptions = {
            'PERSON': 'Person or character',
            'NORP': 'Nationalities, religious or political groups',
            'FAC': 'Buildings, airports, highways, bridges',
            'ORG': 'Companies, agencies, institutions',
            'GPE': 'Countries, cities, states',
            'LOC': 'Non-GPE locations',
            'PRODUCT': 'Objects, vehicles, foods',
            'EVENT': 'Named hurricanes, battles, etc.',
            'WORK_OF_ART': 'Titles of books, songs',
            'LAW': 'Named documents',
            'LANGUAGE': 'Any named language'
        }
        return descriptions.get(label, 'Unknown entity type')
```

## Task Decomposition

Complex commands need to be broken down into simpler, executable subtasks:

```python
class TaskDecomposer:
    def __init__(self):
        self.action_library = {
            'pick_up': ['approach_object', 'grasp_object', 'lift_object'],
            'place_object': ['approach_location', 'lower_object', 'release_object'],
            'clean_surface': ['approach_surface', 'wipe_surface', 'retreat'],
            'navigate': ['path_planning', 'move_to_pose', 'localize']
        }

    def decompose_task(self, parsed_command: Dict) -> List[Dict]:
        """Decompose a high-level task into subtasks"""
        action = parsed_command['action']
        params = parsed_command['parameters']

        if action == 'place':
            # "place X on Y" -> pick up X, navigate to Y, place X
            object_to_place = params[0] if len(params) > 0 else 'object'
            target_location = params[1] if len(params) > 1 else 'location'

            subtasks = [
                {
                    'action': 'find_object',
                    'parameters': {'object_name': object_to_place},
                    'description': f'Locate the {object_to_place}'
                },
                {
                    'action': 'pick_up',
                    'parameters': {'object_name': object_to_place},
                    'description': f'Pick up the {object_to_place}'
                },
                {
                    'action': 'find_location',
                    'parameters': {'location_name': target_location},
                    'description': f'Locate the {target_location}'
                },
                {
                    'action': 'navigate',
                    'parameters': {'target': target_location},
                    'description': f'Navigate to the {target_location}'
                },
                {
                    'action': 'place_object',
                    'parameters': {'object_name': object_to_place, 'location': target_location},
                    'description': f'Place the {object_to_place} on the {target_location}'
                }
            ]
        elif action == 'clean':
            target_surface = params[0] if len(params) > 0 else 'surface'
            subtasks = [
                {
                    'action': 'find_surface',
                    'parameters': {'surface_name': target_surface},
                    'description': f'Locate the {target_surface}'
                },
                {
                    'action': 'approach_surface',
                    'parameters': {'surface_name': target_surface},
                    'description': f'Approach the {target_surface}'
                },
                {
                    'action': 'clean_surface',
                    'parameters': {'surface_name': target_surface},
                    'description': f'Clean the {target_surface}'
                },
                {
                    'action': 'retreat',
                    'parameters': {},
                    'description': 'Move away from the cleaned surface'
                }
            ]
        else:
            # For simple actions, decompose using action library
            if action in self.action_library:
                subtasks = []
                for sub_action in self.action_library[action]:
                    subtasks.append({
                        'action': sub_action,
                        'parameters': parsed_command['parameters'],
                        'description': f'Execute {sub_action} for the given parameters'
                    })
            else:
                # Single action if not in library
                subtasks = [parsed_command]

        return subtasks

# Example usage
decomposer = TaskDecomposer()
parsed_cmd = {'action': 'place', 'parameters': ['red cup', 'table']}
subtasks = decomposer.decompose_task(parsed_cmd)
for i, task in enumerate(subtasks):
    print(f"{i+1}. {task['description']}")
```

## Action Libraries and Robot Capabilities

Define what actions the robot can perform:

```python
from abc import ABC, abstractmethod
from typing import Any, Dict, List
import asyncio

class RobotAction(ABC):
    """Base class for robot actions"""

    def __init__(self, name: str, description: str):
        self.name = name
        self.description = description

    @abstractmethod
    async def execute(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute the action with given parameters"""
        pass

    def validate_parameters(self, parameters: Dict[str, Any]) -> bool:
        """Validate action parameters"""
        return True

class NavigationAction(RobotAction):
    def __init__(self):
        super().__init__("navigate", "Move the robot to a specified location")

    async def execute(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute navigation action"""
        target = parameters.get('target', parameters.get('pose'))
        if not target:
            return {'success': False, 'error': 'No target specified'}

        # Simulate navigation
        print(f"Navigating to {target}")

        # In real implementation, this would interface with navigation stack
        # await self.navigation_client.go_to_pose(target)

        # Simulate execution time
        await asyncio.sleep(2.0)

        return {
            'success': True,
            'message': f'Reached {target}',
            'executed_action': 'navigate',
            'parameters': parameters
        }

class GraspingAction(RobotAction):
    def __init__(self):
        super().__init__("grasp_object", "Grasp an object with the robot's end-effector")

    async def execute(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute grasping action"""
        object_name = parameters.get('object_name', 'object')
        approach_height = parameters.get('approach_height', 0.1)

        print(f"Attempting to grasp {object_name}")

        # In real implementation, this would:
        # 1. Find object in robot coordinates
        # 2. Plan grasp trajectory
        # 3. Execute grasp
        # 4. Verify grasp success

        # Simulate execution
        await asyncio.sleep(3.0)

        success = True  # In real implementation, check grasp success

        return {
            'success': success,
            'message': f'Grasp of {object_name} {"succeeded" if success else "failed"}',
            'executed_action': 'grasp_object',
            'parameters': parameters
        }

class ActionLibrary:
    def __init__(self):
        self.actions = {
            'navigate': NavigationAction(),
            'grasp_object': GraspingAction(),
            # Add more actions as needed
            'approach_object': NavigationAction(),
            'approach_location': NavigationAction(),
            'place_object': GraspingAction(),  # Simplified for example
            'lift_object': GraspingAction(),
            'release_object': GraspingAction(),
            'wipe_surface': RobotAction("wipe_surface", "Wipe a surface"),
            'lower_object': GraspingAction(),
            'retreat': NavigationAction(),
            'path_planning': RobotAction("path_planning", "Plan a path to target"),
            'move_to_pose': NavigationAction(),
            'localize': RobotAction("localize", "Localize the robot in the environment"),
            'find_object': RobotAction("find_object", "Locate an object in the environment"),
            'find_location': RobotAction("find_location", "Locate a specific location"),
            'approach_surface': NavigationAction(),
            'clean_surface': RobotAction("clean_surface", "Clean a surface")
        }

    async def execute_action(self, action_name: str, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a named action with parameters"""
        if action_name not in self.actions:
            return {
                'success': False,
                'error': f'Action {action_name} not found in library'
            }

        action = self.actions[action_name]
        if not action.validate_parameters(parameters):
            return {
                'success': False,
                'error': f'Invalid parameters for action {action_name}'
            }

        try:
            result = await action.execute(parameters)
            return result
        except Exception as e:
            return {
                'success': False,
                'error': f'Execution failed: {str(e)}'
            }
```

## Planning and Execution Framework

Create a framework to plan and execute sequences of actions:

```python
import asyncio
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

class ExecutionStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass
class Task:
    action: str
    parameters: Dict[str, Any]
    description: str
    dependencies: List[str] = None

class PlanExecutor:
    def __init__(self, action_library: ActionLibrary):
        self.action_library = action_library
        self.current_plan = []
        self.execution_history = []
        self.status = ExecutionStatus.PENDING

    async def execute_plan(self, tasks: List[Dict]) -> Dict[str, Any]:
        """Execute a sequence of tasks"""
        self.status = ExecutionStatus.RUNNING
        self.execution_history = []
        results = []

        for i, task_data in enumerate(tasks):
            print(f"Executing task {i+1}/{len(tasks)}: {task_data['description']}")

            result = await self.action_library.execute_action(
                task_data['action'],
                task_data['parameters']
            )

            task_result = {
                'task_index': i,
                'task_description': task_data['description'],
                'result': result,
                'timestamp': asyncio.get_event_loop().time()
            }

            self.execution_history.append(task_result)
            results.append(result)

            # Check if task failed
            if not result['success']:
                self.status = ExecutionStatus.FAILED
                return {
                    'success': False,
                    'completed_tasks': len(results),
                    'total_tasks': len(tasks),
                    'error': result.get('error', 'Task failed'),
                    'results': results
                }

        self.status = ExecutionStatus.SUCCESS
        return {
            'success': True,
            'completed_tasks': len(results),
            'total_tasks': len(tasks),
            'results': results
        }

    async def execute_with_monitoring(self, tasks: List[Dict], timeout: float = 300.0) -> Dict[str, Any]:
        """Execute plan with timeout and monitoring"""
        try:
            # Create a timeout task
            execution_task = asyncio.create_task(self.execute_plan(tasks))
            result = await asyncio.wait_for(execution_task, timeout=timeout)
            return result
        except asyncio.TimeoutError:
            self.status = ExecutionStatus.CANCELLED
            return {
                'success': False,
                'completed_tasks': len(self.execution_history),
                'error': 'Execution timed out',
                'results': [item['result'] for item in self.execution_history]
            }
```

## Integration with Large Language Models

Connect the planning system with LLMs for more sophisticated understanding:

```python
import openai
import json
from typing import Dict, Any

class LLMPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.action_descriptions = {
            'navigate': 'Move the robot to a specified location',
            'grasp_object': 'Grasp an object with the robot\'s end-effector',
            'place_object': 'Place an object at a specified location',
            'find_object': 'Locate an object in the environment',
            'find_location': 'Locate a specific location in the environment',
            'clean_surface': 'Clean a surface',
            'approach_object': 'Move close to an object',
            'approach_location': 'Move close to a location',
            'lift_object': 'Lift an object after grasping',
            'release_object': 'Release a grasped object',
            'wipe_surface': 'Wipe a surface clean',
            'lower_object': 'Lower an object',
            'retreat': 'Move away from current position'
        }

    def get_available_actions_prompt(self) -> str:
        """Get prompt describing available actions"""
        actions_str = "\n".join([
            f"- {action}: {desc}"
            for action, desc in self.action_descriptions.items()
        ])
        return f"""
Available robot actions:
{actions_str}

Each action should have appropriate parameters like object names, locations, etc.
"""

    async def plan_from_command(self, command: str, world_state: Dict[str, Any] = None) -> List[Dict]:
        """Generate a plan from natural language command using LLM"""
        prompt = f"""
You are a robot task planner. Convert the following natural language command into a sequence of executable robot actions.

{self.get_available_actions_prompt()}

Current world state: {json.dumps(world_state, indent=2) if world_state else 'Unknown'}

Command: "{command}"

Please respond with a JSON list of tasks, where each task has:
- "action": the action name
- "parameters": a dictionary of parameters
- "description": human-readable description

Example response format:
[
    {{
        "action": "find_object",
        "parameters": {{"object_name": "red cup"}},
        "description": "Locate the red cup in the environment"
    }},
    {{
        "action": "approach_object",
        "parameters": {{"object_name": "red cup"}},
        "description": "Move close to the red cup"
    }}
]

Response (JSON only, no other text):
"""

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                max_tokens=1000
            )

            # Extract JSON from response
            response_text = response.choices[0].message['content'].strip()

            # Clean up response if it contains markdown code blocks
            if response_text.startswith('```json'):
                response_text = response_text[7:]  # Remove ```json
            if response_text.endswith('```'):
                response_text = response_text[:-3]  # Remove ```

            plan = json.loads(response_text)
            return plan
        except Exception as e:
            print(f"Error generating plan with LLM: {e}")
            # Fallback to simple parsing
            parser = CommandParser()
            parsed = parser.parse_command(command)
            decomposer = TaskDecomposer()
            fallback_plan = decomposer.decompose_task(parsed)
            return fallback_plan
```

## Complete Cognitive Planning System

Putting it all together:

```python
class CognitivePlanningSystem:
    def __init__(self, llm_api_key: str = None):
        self.action_library = ActionLibrary()
        self.executor = PlanExecutor(self.action_library)
        self.llm_planner = LLMPlanner(llm_api_key) if llm_api_key else None
        self.command_parser = CommandParser()
        self.task_decomposer = TaskDecomposer()

    async def execute_command(self, command: str, world_state: Dict[str, Any] = None) -> Dict[str, Any]:
        """Execute a natural language command end-to-end"""
        print(f"Processing command: {command}")

        # Generate plan (either with LLM or simple decomposition)
        if self.llm_planner:
            plan = await self.llm_planner.plan_from_command(command, world_state)
        else:
            # Fallback to simple parsing and decomposition
            parsed = self.command_parser.parse_command(command)
            plan = self.task_decomposer.decompose_task(parsed)

        print(f"Generated plan with {len(plan)} tasks")
        for i, task in enumerate(plan):
            print(f"  {i+1}. {task['description']}")

        # Execute the plan
        result = await self.executor.execute_with_monitoring(plan)

        return {
            'command': command,
            'plan': plan,
            'execution_result': result,
            'success': result['success']
        }

# Example usage
async def main():
    # Initialize the system (you would provide a real API key)
    system = CognitivePlanningSystem()  # llm_api_key="your-api-key"

    # Example commands
    commands = [
        "Pick up the red cup and place it on the table",
        "Clean the kitchen counter",
        "Go to the living room and find the blue book"
    ]

    for cmd in commands:
        print(f"\n--- Executing: {cmd} ---")
        result = await system.execute_command(cmd)
        print(f"Success: {result['success']}")
        print(f"Completed {result['execution_result']['completed_tasks']} of {result['execution_result']['total_tasks']} tasks")

# To run: asyncio.run(main())
```

## Practical Exercise: Implement a Simple Command System

Create a simplified cognitive planning system:

1. **Create action classes** for basic robot capabilities
2. **Implement command parsing** for common robot commands
3. **Build a task decomposer** that breaks complex commands into simple actions
4. **Create a plan executor** that runs the sequence of actions
5. **Test with sample commands** like "pick up the ball" or "go to the kitchen"

### Example Implementation Structure:

```python
class SimpleCognitivePlanner:
    def __init__(self):
        self.actions = {
            'move': self.execute_move,
            'grasp': self.execute_grasp,
            'release': self.execute_release,
            'find': self.execute_find
        }

    async def execute_move(self, params):
        location = params.get('location', 'unknown')
        print(f"Moving to {location}")
        await asyncio.sleep(1)  # Simulate movement
        return {'success': True, 'location': location}

    async def execute_grasp(self, params):
        object_name = params.get('object', 'unknown object')
        print(f"Grasping {object_name}")
        await asyncio.sleep(1)  # Simulate grasping
        return {'success': True, 'object': object_name}

    # Add other action methods...

    async def process_command(self, command):
        # Parse, plan, and execute the command
        pass
```

## Summary

This lesson covered cognitive planning systems that translate natural language commands into robot actions. We explored command parsing, task decomposition, action libraries, and integration with language models. Cognitive planning is essential for creating robots that can understand and execute complex human instructions.

## Next Steps

In the next lesson, we'll explore how to integrate these planning systems with ROS 2 for real-world robotic applications and create the complete Vision-Language-Action pipeline.