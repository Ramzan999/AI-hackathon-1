---
sidebar_position: 2
---

# Hands-on Lab: Vision-Language-Action Systems

In this lab, you'll implement Vision-Language-Action systems that enable robots to interpret natural language commands and execute corresponding physical actions in the environment.

## Prerequisites

Before starting this lab, ensure you have:
- ROS 2 Humble Hawksbill installed
- Basic understanding of computer vision and natural language processing
- Completed previous modules (especially ROS 2 and perception modules)
- Access to a robot with camera and manipulation capabilities (or simulation)

## Setting up the VLA Environment

First, source your ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
```

## Basic VLA Architecture

Create a simple VLA system architecture. Start with the language understanding component. Create `language_understanding.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from vla_systems_msgs.msg import VLACommand, VLAActionResult
from builtin_interfaces.msg import Time
import re
import json


class LanguageUnderstandingNode(Node):
    def __init__(self):
        super().__init__('language_understanding_node')

        # Subscribe to natural language commands
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )

        # Subscribe to camera image for context
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for structured VLA commands
        self.vla_command_publisher = self.create_publisher(
            VLACommand,
            'vla_command',
            10
        )

        # Publisher for system status
        self.status_publisher = self.create_publisher(
            String,
            'vla_status',
            10
        )

        # Internal state
        self.latest_image = None
        self.command_history = []

        self.get_logger().info('Language Understanding Node initialized')

    def command_callback(self, msg):
        """Process natural language commands"""
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Simple command parsing (in practice, use more sophisticated NLP)
        parsed_command = self.parse_command(command_text)

        if parsed_command:
            # Create VLA command message
            vla_cmd = VLACommand()
            vla_cmd.header.stamp = self.get_clock().now().to_msg()
            vla_cmd.header.frame_id = 'base_link'
            vla_cmd.command_text = command_text
            vla_cmd.command_type = parsed_command['type']
            vla_cmd.target_object = parsed_command['object']
            vla_cmd.action = parsed_command['action']
            vla_cmd.parameters = json.dumps(parsed_command.get('parameters', {}))

            # Publish structured command
            self.vla_command_publisher.publish(vla_cmd)

            # Update status
            status_msg = String()
            status_msg.data = f"Parsed command: {parsed_command['action']} {parsed_command['object']}"
            self.status_publisher.publish(status_msg)

            # Add to history
            self.command_history.append({
                'timestamp': self.get_clock().now(),
                'command': command_text,
                'parsed': parsed_command
            })
        else:
            status_msg = String()
            status_msg.data = f"Could not parse command: {command_text}"
            self.status_publisher.publish(status_msg)

    def image_callback(self, msg):
        """Store latest image for context"""
        self.latest_image = msg

    def parse_command(self, command_text):
        """Simple command parser - in practice, use NLP models"""
        command_text = command_text.lower().strip()

        # Define action patterns
        action_patterns = {
            'pick_up': [r'pick up (.+)', r'grab (.+)', r'take (.+)', r'get (.+)'],
            'place': [r'place (.+) on (.+)', r'put (.+) on (.+)', r'set (.+) on (.+)'],
            'move_to': [r'go to (.+)', r'move to (.+)', r'go near (.+)', r'approach (.+)'],
            'navigate': [r'go (.+)', r'move (.+)', r'travel (.+)'],
            'describe': [r'what is (.+)', r'tell me about (.+)', r'describe (.+)'],
            'find': [r'find (.+)', r'locate (.+)', r'where is (.+)']
        }

        # Object extraction patterns
        object_patterns = [
            r'the (.+)',
            r'a (.+)',
            r'an (.+)',
            r'(.+)',
        ]

        # Parse the command
        for action, patterns in action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command_text)
                if match:
                    if action == 'place':
                        # Special handling for place command (object, location)
                        groups = match.groups()
                        if len(groups) >= 2:
                            return {
                                'type': 'manipulation',
                                'action': action,
                                'object': groups[0].strip(),
                                'location': groups[1].strip()
                            }
                    else:
                        # Extract object from remaining text
                        obj_text = match.group(1) if len(match.groups()) > 0 else ''
                        return {
                            'type': 'manipulation' if action in ['pick_up', 'place'] else 'navigation',
                            'action': action,
                            'object': obj_text.strip()
                        }

        # If no specific action matched, treat as general command
        return {
            'type': 'general',
            'action': 'unknown',
            'object': command_text
        }


def main(args=None):
    rclpy.init(args=args)

    language_node = LanguageUnderstandingNode()

    try:
        rclpy.spin(language_node)
    except KeyboardInterrupt:
        pass
    finally:
        language_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Vision Processing Component

Create the vision processing component. Create `vision_processor.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String


class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to camera info
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for object detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            'object_detections',
            10
        )

        # Publisher for 3D object positions
        self.position_publisher = self.create_publisher(
            String,  # In practice, use a custom message type
            'object_positions',
            10
        )

        # Internal state
        self.camera_info = None
        self.object_cache = {}  # Cache of detected objects

        self.get_logger().info('Vision Processor Node initialized')

    def camera_info_callback(self, msg):
        """Store camera calibration information"""
        self.camera_info = msg

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {str(e)}')
            return

        # Perform object detection (using OpenCV for this example)
        detections = self.detect_objects(cv_image)

        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        detection_array.detections = detections

        # Publish detections
        self.detection_publisher.publish(detection_array)

        # Process 3D positions if camera info is available
        if self.camera_info:
            positions_3d = self.compute_3d_positions(detections, cv_image.shape)
            positions_msg = String()
            positions_msg.data = str(positions_3d)
            self.position_publisher.publish(positions_msg)

    def detect_objects(self, image):
        """Simple object detection using color-based segmentation"""
        detections = []

        # Convert BGR to HSV for color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for common objects (red, green, blue, yellow)
        color_ranges = [
            ('red', (0, 50, 50), (10, 255, 255)),      # Red objects
            ('green', (40, 50, 50), (80, 255, 255)),    # Green objects
            ('blue', (100, 50, 50), (130, 255, 255)),   # Blue objects
            ('yellow', (20, 50, 50), (30, 255, 255)),   # Yellow objects
        ]

        for obj_name, lower, upper in color_ranges:
            # Create mask for color range
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Filter by size to avoid noise
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Create detection
                    detection = Detection2D()
                    detection.header.stamp = self.get_clock().now().to_msg()
                    detection.header.frame_id = 'camera_link'

                    # Set bounding box
                    detection.bbox.center.x = x + w/2
                    detection.bbox.center.y = y + h/2
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    # Set object hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = obj_name
                    hypothesis.score = 0.8  # Placeholder confidence

                    detection.results.append(hypothesis)

                    detections.append(detection)

        return detections

    def compute_3d_positions(self, detections, image_shape):
        """Compute 3D positions from 2D detections and camera parameters"""
        if not self.camera_info:
            return {}

        positions_3d = {}

        # Camera intrinsic parameters
        fx = self.camera_info.k[0]  # Focal length x
        fy = self.camera_info.k[4]  # Focal length y
        cx = self.camera_info.k[2]  # Principal point x
        cy = self.camera_info.k[5]  # Principal point y

        for detection in detections:
            obj_id = detection.results[0].id if detection.results else 'unknown'

            # Get 2D center
            center_x = detection.bbox.center.x
            center_y = detection.bbox.center.y

            # In a real system, you'd need depth information (from stereo, depth camera, etc.)
            # For this example, we'll assume a fixed distance
            z = 1.0  # Fixed distance in meters

            # Convert to 3D coordinates (simplified)
            x = (center_x - cx) * z / fx
            y = (center_y - cy) * z / fy

            positions_3d[obj_id] = {
                'x': x,
                'y': y,
                'z': z,
                'pixel_x': center_x,
                'pixel_y': center_y
            }

        return positions_3d


def main(args=None):
    rclpy.init(args=args)

    vision_node = VisionProcessorNode()

    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action Planning Component

Create the action planning component. Create `action_planner.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vla_systems_msgs.msg import VLACommand, VLAActionResult
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import numpy as np
import json


class ActionPlannerNode(Node):
    def __init__(self):
        super().__init__('action_planner_node')

        # Subscribe to VLA commands
        self.vla_command_subscriber = self.create_subscription(
            VLACommand,
            'vla_command',
            self.vla_command_callback,
            10
        )

        # Publisher for action results
        self.result_publisher = self.create_publisher(
            VLAActionResult,
            'vla_result',
            10
        )

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            String,  # In practice, use Twist for navigation, JointTrajectory for manipulation
            'robot_command',
            10
        )

        # Action server for complex tasks
        self.action_server = ActionServer(
            self,
            VLACommand,
            'vla_execute',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Internal state
        self.object_positions = {}  # Updated from vision system
        self.robot_state = {
            'position': Point(x=0.0, y=0.0, z=0.0),
            'orientation': Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
            'joint_states': {}
        }

        self.get_logger().info('Action Planner Node initialized')

    def goal_callback(self, goal_request):
        """Accept or reject a goal"""
        self.get_logger().info(f'Received goal request: {goal_request.command_text}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info('Executing goal...')

        feedback_msg = VLAActionResult()
        feedback_msg.status = "Planning"

        # Plan and execute the action
        success = self.plan_and_execute_action(goal_handle.goal)

        if success:
            goal_handle.succeed()
            result = VLAActionResult()
            result.status = "Success"
            result.message = "Action completed successfully"
        else:
            goal_handle.abort()
            result = VLAActionResult()
            result.status = "Failed"
            result.message = "Action execution failed"

        return result

    def vla_command_callback(self, msg):
        """Process VLA commands"""
        self.get_logger().info(f'Processing VLA command: {msg.command_text}')

        # Plan the action based on command type
        if msg.command_type == 'navigation':
            self.execute_navigation_command(msg)
        elif msg.command_type == 'manipulation':
            self.execute_manipulation_command(msg)
        else:
            self.execute_general_command(msg)

    def plan_and_execute_action(self, vla_cmd):
        """Plan and execute an action based on VLA command"""
        try:
            if vla_cmd.action == 'pick_up':
                return self.plan_pick_up(vla_cmd.target_object)
            elif vla_cmd.action == 'place':
                return self.plan_place(vla_cmd.target_object, vla_cmd.parameters)
            elif vla_cmd.action == 'move_to':
                return self.plan_move_to(vla_cmd.target_object)
            elif vla_cmd.action == 'navigate':
                return self.plan_navigate(vla_cmd.target_object)
            else:
                self.get_logger().warn(f'Unknown action: {vla_cmd.action}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error executing action: {str(e)}')
            return False

    def plan_pick_up(self, target_object):
        """Plan and execute pick up action"""
        self.get_logger().info(f'Planning pick up of {target_object}')

        # Find object position from vision system
        obj_pos = self.find_object_position(target_object)
        if not obj_pos:
            self.get_logger().error(f'Could not find object: {target_object}')
            return False

        # Plan approach trajectory
        approach_pose = self.calculate_approach_pose(obj_pos)

        # Execute approach
        self.execute_navigation(approach_pose)

        # Execute grasp
        success = self.execute_grasp(obj_pos)

        return success

    def plan_place(self, target_object, parameters):
        """Plan and execute place action"""
        self.get_logger().info(f'Planning place of {target_object}')

        # Parse parameters for placement location
        params = json.loads(parameters) if parameters else {}
        location = params.get('location', 'default')

        # Find placement position
        place_pos = self.find_placement_position(location)
        if not place_pos:
            self.get_logger().error(f'Could not find placement location: {location}')
            return False

        # Plan navigation to placement location
        nav_success = self.execute_navigation(place_pos)
        if not nav_success:
            return False

        # Execute placement
        place_success = self.execute_placement(place_pos)

        return place_success

    def plan_move_to(self, target_location):
        """Plan and execute move to action"""
        self.get_logger().info(f'Planning move to {target_location}')

        # Find target position
        target_pos = self.find_location_position(target_location)
        if not target_pos:
            self.get_logger().error(f'Could not find location: {target_location}')
            return False

        # Execute navigation
        success = self.execute_navigation(target_pos)

        return success

    def find_object_position(self, object_name):
        """Find position of object in environment"""
        # In a real system, this would query the object map
        # For this example, return a placeholder position
        if object_name in self.object_positions:
            return self.object_positions[object_name]
        else:
            # Return a default position for unknown objects
            return {'x': 1.0, 'y': 0.0, 'z': 0.0}

    def find_location_position(self, location_name):
        """Find position of named location"""
        # Define known locations in the environment
        known_locations = {
            'kitchen': {'x': 2.0, 'y': 1.0, 'z': 0.0},
            'living_room': {'x': -1.0, 'y': 2.0, 'z': 0.0},
            'bedroom': {'x': 0.0, 'y': -2.0, 'z': 0.0},
            'office': {'x': 3.0, 'y': -1.0, 'z': 0.0}
        }

        if location_name in known_locations:
            return known_locations[location_name]
        else:
            # Try to find by partial match
            for loc_name, pos in known_locations.items():
                if location_name in loc_name or loc_name in location_name:
                    return pos

        return None

    def find_placement_position(self, location_name):
        """Find suitable placement position"""
        # Similar to find_location_position but for placement surfaces
        placement_locations = {
            'table': {'x': 1.0, 'y': 0.0, 'z': 0.8},  # Table height
            'counter': {'x': 1.5, 'y': 0.5, 'z': 0.9},  # Counter height
            'shelf': {'x': 0.8, 'y': -0.5, 'z': 1.2},  # Shelf height
        }

        if location_name in placement_locations:
            return placement_locations[location_name]
        else:
            # Default placement position
            return {'x': 1.0, 'y': 0.0, 'z': 0.8}

    def calculate_approach_pose(self, object_pos):
        """Calculate approach pose for manipulation"""
        # Calculate approach position (slightly in front of object)
        approach = {
            'x': object_pos['x'] - 0.3,  # 30cm in front
            'y': object_pos['y'],
            'z': object_pos['z'] + 0.1  # Slightly above
        }
        return approach

    def execute_navigation(self, target_pos):
        """Execute navigation to target position"""
        self.get_logger().info(f'Navigating to position: {target_pos}')

        # In a real system, this would use navigation stack
        # For this example, publish a simple command
        cmd_msg = String()
        cmd_msg.data = f"navigate_to {target_pos['x']} {target_pos['y']} {target_pos['z']}"
        self.cmd_vel_publisher.publish(cmd_msg)

        # Simulate navigation completion
        # In real system, wait for navigation feedback
        return True

    def execute_grasp(self, object_pos):
        """Execute grasp action"""
        self.get_logger().info(f'Executing grasp at: {object_pos}')

        # In a real system, this would use manipulation stack
        # For this example, publish a simple command
        cmd_msg = String()
        cmd_msg.data = f"grasp_at {object_pos['x']} {object_pos['y']} {object_pos['z']}"
        self.cmd_vel_publisher.publish(cmd_msg)

        # Simulate grasp completion
        return True

    def execute_placement(self, place_pos):
        """Execute placement action"""
        self.get_logger().info(f'Executing placement at: {place_pos}')

        # In a real system, this would use manipulation stack
        # For this example, publish a simple command
        cmd_msg = String()
        cmd_msg.data = f"place_at {place_pos['x']} {place_pos['y']} {place_pos['z']}"
        self.cmd_vel_publisher.publish(cmd_msg)

        # Simulate placement completion
        return True

    def execute_navigation_command(self, vla_cmd):
        """Execute navigation-specific command"""
        target_pos = self.find_location_position(vla_cmd.target_object)
        if target_pos:
            self.execute_navigation(target_pos)

    def execute_manipulation_command(self, vla_cmd):
        """Execute manipulation-specific command"""
        if vla_cmd.action == 'pick_up':
            self.plan_pick_up(vla_cmd.target_object)
        elif vla_cmd.action == 'place':
            self.plan_place(vla_cmd.target_object, vla_cmd.parameters)

    def execute_general_command(self, vla_cmd):
        """Execute general command"""
        self.get_logger().info(f'Executing general command: {vla_cmd.command_text}')


def main(args=None):
    rclpy.init(args=args)

    action_node = ActionPlannerNode()

    try:
        rclpy.spin(action_node)
    except KeyboardInterrupt:
        pass
    finally:
        action_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Main VLA System Integration

Create the main launch file that integrates all components. Create `vla_system.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for the robot'
    )

    # Language understanding node
    language_understanding_node = Node(
        package='vla_systems',
        executable='language_understanding.py',
        name='language_understanding_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Vision processor node
    vision_processor_node = Node(
        package='vla_systems',
        executable='vision_processor.py',
        name='vision_processor_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Action planner node
    action_planner_node = Node(
        package='vla_systems',
        executable='action_planner.py',
        name='action_planner_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Example: Simple command publisher for testing
    command_publisher_node = Node(
        package='vla_systems',
        executable='command_publisher.py',
        name='command_publisher_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_namespace_cmd)

    ld.add_action(language_understanding_node)
    ld.add_action(vision_processor_node)
    ld.add_action(action_planner_node)
    ld.add_action(command_publisher_node)

    return ld
```

## Simple Command Publisher for Testing

Create a simple command publisher for testing. Create `command_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class CommandPublisherNode(Node):
    def __init__(self):
        super().__init__('command_publisher_node')

        # Publisher for natural language commands
        self.command_publisher = self.create_publisher(
            String,
            'natural_language_command',
            10
        )

        # Timer to send example commands
        self.timer = self.create_timer(5.0, self.send_example_command)
        self.command_index = 0

        # Example commands
        self.example_commands = [
            "Pick up the red block",
            "Move to the kitchen",
            "Place the object on the table",
            "Find the blue cup",
            "Go near the green box"
        ]

        self.get_logger().info('Command Publisher Node initialized')

    def send_example_command(self):
        """Send an example command"""
        if self.command_index < len(self.example_commands):
            command = self.example_commands[self.command_index]
            cmd_msg = String()
            cmd_msg.data = command

            self.command_publisher.publish(cmd_msg)
            self.get_logger().info(f'Sent command: {command}')

            self.command_index += 1
        else:
            # Reset after all commands sent
            self.command_index = 0


def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisherNode()

    try:
        rclpy.spin(command_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        command_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Message Definitions

Create custom message definitions for VLA system. Create the message files in a new package structure:

First, create the package directory structure:
```bash
mkdir -p vla_systems_msgs/msg
```

Create `VLACommand.msg` in `vla_systems_msgs/msg/`:
```text
# VLACommand.msg
std_msgs/Header header
string command_text
string command_type  # navigation, manipulation, general
string target_object
string action  # pick_up, place, move_to, etc.
string parameters  # JSON string of additional parameters
```

Create `VLAActionResult.msg` in `vla_systems_msgs/msg/`:
```text
# VLAActionResult.msg
std_msgs/Header header
string status  # Success, Failed, InProgress, etc.
string message
float64 completion_percentage
```

## Running the VLA System

1. Make sure all Python files are executable:
   ```bash
   chmod +x language_understanding.py
   chmod +x vision_processor.py
   chmod +x action_planner.py
   chmod +x command_publisher.py
   ```

2. Launch the VLA system:
   ```bash
   ros2 launch vla_system.launch.py
   ```

3. Or launch individual components:
   ```bash
   # Terminal 1
   ros2 run vla_systems language_understanding.py

   # Terminal 2
   ros2 run vla_systems vision_processor.py

   # Terminal 3
   ros2 run vla_systems action_planner.py

   # Terminal 4 - to send custom commands
   ros2 topic pub /natural_language_command std_msgs/String "data: 'pick up the red block'"
   ```

## Integration with Real Systems

For integration with real robots:

1. **Connect to hardware interfaces**: Replace simulated publishers with actual robot drivers
2. **Integrate with navigation stack**: Use Nav2 for navigation tasks
3. **Connect to manipulation stack**: Use MoveIt! for manipulation tasks
4. **Enhance with advanced AI models**: Integrate with large language models and vision transformers

## Lab Exercise

Create a complete VLA system:

1. Set up the basic VLA architecture with language, vision, and action components
2. Implement a simple command parser and object detector
3. Create action planners for navigation and manipulation
4. Test the system with various natural language commands
5. Evaluate the system's performance and identify areas for improvement

## Troubleshooting Common Issues

### Command Parsing Issues
- Ensure proper text preprocessing
- Handle different command variations and synonyms
- Implement fallback mechanisms for unknown commands

### Object Detection Problems
- Verify camera calibration
- Adjust detection thresholds
- Handle occlusions and lighting variations

### Action Execution Failures
- Check robot hardware status
- Verify joint limits and collisions
- Implement proper error handling and recovery

## Summary

In this lab, you've learned:
- How to create a VLA system architecture with language, vision, and action components
- How to implement natural language understanding for robotics
- How to integrate vision processing with action planning
- How to create a complete pipeline from command to action execution
- How to test and evaluate VLA systems