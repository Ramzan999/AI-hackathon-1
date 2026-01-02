---
sidebar_position: 2
---

# Hands-on Lab: NVIDIA Isaac Integration

In this lab, you'll implement perception and decision-making capabilities using NVIDIA Isaac, understanding how to leverage GPU acceleration for real-time AI processing on robots.

## Prerequisites

Before starting this lab, ensure you have:
- NVIDIA GPU with CUDA support (or NVIDIA Jetson device)
- ROS 2 Humble Hawksbill installed
- NVIDIA Isaac ROS packages installed
- Basic understanding of ROS 2 concepts

## Setting up Isaac ROS Environment

First, source your ROS 2 and Isaac ROS environments:

```bash
source /opt/ros/humble/setup.bash
source /usr/local/cuda/bin/cuda-profile.sh  # If CUDA is in non-standard location
```

Verify Isaac ROS packages are available:
```bash
ros2 pkg list | grep isaac
```

## Installing Isaac ROS Packages

Install the Isaac ROS packages:
```bash
sudo apt update
sudo apt install ros-humble-isaac-ros-*  # Install all Isaac ROS packages
```

Or build from source:
```bash
# Create workspace
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws/src

# Clone Isaac ROS repositories
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Build the workspace
cd ~/isaac_ws
colcon build --packages-select isaac_ros_common isaac_ros_image_pipeline isaac_ros_apriltag isaac_ros_dnn_inference
source install/setup.bash
```

## Basic Isaac ROS Image Pipeline

Create a launch file to demonstrate the Isaac ROS image pipeline. Create `isaac_image_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch configuration
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # Declare launch arguments
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    declare_camera_info_topic_cmd = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Input camera info topic'
    )

    # Create container for Isaac ROS nodes
    container = ComposableNodeContainer(
        name='image_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image Rectification node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', image_topic),
                    ('camera_info', camera_info_topic),
                    ('image_rect', 'rectified_image'),
                ],
            ),

            # Resize node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='resize_node',
                parameters=[{
                    'output_width': 224,
                    'output_height': 224,
                    'keep_aspect_ratio': True,
                }],
                remappings=[
                    ('image', 'rectified_image'),
                    ('camera_info', 'camera_info_rect'),
                    ('image_resized', 'resized_image'),
                    ('camera_info_resized', 'camera_info_resized'),
                ],
            ),
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_camera_info_topic_cmd)
    ld.add_action(container)

    return ld
```

## Isaac ROS DNN Inference Example

Create a DNN inference pipeline. Create `isaac_dnn_inference.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch configuration
    image_topic = LaunchConfiguration('image_topic')
    engine_file_path = LaunchConfiguration('engine_file_path')
    input_tensor_names = LaunchConfiguration('input_tensor_names')
    output_tensor_names = LaunchConfiguration('output_tensor_names')
    input_binding_names = LaunchConfiguration('input_binding_names')
    output_binding_names = LaunchConfiguration('output_binding_names')
    tensorrt_precision = LaunchConfiguration('tensorrt_precision')

    # Declare launch arguments
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Input image topic'
    )

    declare_engine_file_path_cmd = DeclareLaunchArgument(
        'engine_file_path',
        default_value='/path/to/your/model.plan',
        description='Path to the TensorRT engine file'
    )

    declare_input_tensor_names_cmd = DeclareLaunchArgument(
        'input_tensor_names',
        default_value=["input_tensor"],
        description='Input tensor names'
    )

    declare_output_tensor_names_cmd = DeclareLaunchArgument(
        'output_tensor_names',
        default_value=["output_tensor"],
        description='Output tensor names'
    )

    declare_input_binding_names_cmd = DeclareLaunchArgument(
        'input_binding_names',
        default_value=["input"],
        description='Input binding names'
    )

    declare_output_binding_names_cmd = DeclareLaunchArgument(
        'output_binding_names',
        default_value=["output"],
        description='Output binding names'
    )

    declare_tensorrt_precision_cmd = DeclareLaunchArgument(
        'tensorrt_precision',
        default_value='fp16',
        description='TensorRT precision (fp32, fp16, int8)'
    )

    # Create container for Isaac ROS DNN nodes
    container = ComposableNodeContainer(
        name='dnn_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # DNN inference node
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt_node',
                parameters=[{
                    'engine_file_path': engine_file_path,
                    'input_tensor_names': input_tensor_names,
                    'output_tensor_names': output_tensor_names,
                    'input_binding_names': input_binding_names,
                    'output_binding_names': output_binding_names,
                    'max_batch_size': 1,
                    'input_tensor_formats': ['nitros_tensor_list_nchw'],
                    'output_tensor_formats': ['nitros_tensor_list_nhwc'],
                    'tensorrt_precision': tensorrt_precision,
                    'tensorrt_workspace_size': 1073741824,  # 1GB
                    'verbose': False,
                }],
                remappings=[
                    ('tensor_sub', image_topic),
                    ('tensor_pub', 'inference_result'),
                ],
            ),
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_engine_file_path_cmd)
    ld.add_action(declare_input_tensor_names_cmd)
    ld.add_action(declare_output_tensor_names_cmd)
    ld.add_action(declare_input_binding_names_cmd)
    ld.add_action(declare_output_binding_names_cmd)
    ld.add_action(declare_tensorrt_precision_cmd)
    ld.add_action(container)

    return ld
```

## Isaac ROS AprilTag Detection

Create an AprilTag detection pipeline. Create `isaac_apriltag.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch configuration
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # Declare launch arguments
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    declare_camera_info_topic_cmd = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Input camera info topic'
    )

    # Create container for Isaac ROS AprilTag nodes
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # AprilTag node
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                parameters=[{
                    'family': 'tag36h11',
                    'max_tags': 10,
                    'tag_size': 0.032,  # Size in meters
                }],
                remappings=[
                    ('image', image_topic),
                    ('camera_info', camera_info_topic),
                    ('detections', 'tag_detections'),
                ],
            ),
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_camera_info_topic_cmd)
    ld.add_action(container)

    return ld
```

## Perception Pipeline Integration

Create a comprehensive perception pipeline. Create `isaac_perception_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    # Launch configuration
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # Declare launch arguments
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    declare_camera_info_topic_cmd = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Input camera info topic'
    )

    # Create main container for Isaac ROS perception pipeline
    perception_container = ComposableNodeContainer(
        name='perception_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image format converter (for GPU processing)
            ComposableNode(
                package='isaac_ros_image_format_converter',
                plugin='nvidia::isaac_ros::image_format_converter::ImageFormatConverterNode',
                name='image_format_converter',
                parameters=[{
                    'encoding_desired': 'rgb8',
                    'image_width': 640,
                    'image_height': 480,
                }],
                remappings=[
                    ('image_raw', image_topic),
                    ('image', 'converted_image'),
                ],
            ),

            # Image rectification
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', 'converted_image'),
                    ('camera_info', camera_info_topic),
                    ('image_rect', 'rectified_image'),
                ],
            ),

            # AprilTag detection
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                parameters=[{
                    'family': 'tag36h11',
                    'max_tags': 10,
                    'tag_size': 0.1,  # Size in meters
                }],
                remappings=[
                    ('image', 'rectified_image'),
                    ('camera_info', camera_info_topic),
                    ('detections', 'tag_detections'),
                ],
            ),
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_camera_info_topic_cmd)
    ld.add_action(perception_container)

    return ld
```

## Isaac Sim Integration Example

To work with Isaac Sim, create a launch file that connects to the simulation. Create `isaac_sim_bridge.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for the robot'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    # ROS bridge for Isaac Sim
    ros_bridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'ssl': False,
        }],
        output='screen'
    )

    # TF broadcaster for Isaac Sim
    tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_tf_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Robot state publisher (if using URDF from Isaac Sim)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': '<robot name="sim_robot"/>',  # Placeholder - replace with actual URDF
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(ros_bridge)
    ld.add_action(tf_broadcaster)
    ld.add_action(robot_state_publisher)

    return ld
```

## Perception Pipeline with Decision Making

Create a decision-making node that uses the perception results. Create `decision_maker.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from std_msgs.msg import String
import numpy as np


class IsaacDecisionMaker(Node):
    def __init__(self):
        super().__init__('isaac_decision_maker')

        # Subscribe to AprilTag detections
        self.tag_subscription = self.create_subscription(
            AprilTagDetectionArray,
            'tag_detections',
            self.tag_callback,
            10
        )

        # Subscribe to camera image
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Publisher for decision status
        self.status_publisher = self.create_publisher(
            String,
            'decision_status',
            10
        )

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.decision_callback)

        # Internal state
        self.latest_tags = None
        self.latest_image = None
        self.robot_cmd = Twist()

        self.get_logger().info('Isaac Decision Maker initialized')

    def tag_callback(self, msg):
        """Process AprilTag detections"""
        self.latest_tags = msg
        if len(msg.detections) > 0:
            self.get_logger().info(f'Detected {len(msg.detections)} tags')

    def image_callback(self, msg):
        """Process camera image"""
        self.latest_image = msg

    def decision_callback(self):
        """Make decisions based on perception data"""
        if self.latest_tags is not None and len(self.latest_tags.detections) > 0:
            # Example decision: move toward the first detected tag
            detection = self.latest_tags.detections[0]

            # Simple proportional controller for tag following
            # In practice, you'd use more sophisticated algorithms
            center_x = detection.pose.pose.position.x
            center_y = detection.pose.pose.position.y

            # Normalize to image coordinates (assuming 640x480)
            norm_x = (center_x - 320) / 320  # Range: -1 to 1
            norm_y = (center_y - 240) / 240  # Range: -1 to 1

            # Set robot commands based on tag position
            self.robot_cmd.linear.x = max(0.0, 0.5 - abs(norm_y) * 0.3)  # Move forward if tag is visible
            self.robot_cmd.angular.z = -norm_x * 0.5  # Turn toward tag

            # Publish command
            self.cmd_vel_publisher.publish(self.robot_cmd)

            # Publish status
            status_msg = String()
            status_msg.data = f"Following tag at ({center_x:.2f}, {center_y:.2f})"
            self.status_publisher.publish(status_msg)
        else:
            # Stop robot if no tags detected
            self.robot_cmd.linear.x = 0.0
            self.robot_cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.robot_cmd)

            status_msg = String()
            status_msg.data = "No tags detected, stopped"
            self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)

    decision_maker = IsaacDecisionMaker()

    try:
        rclpy.spin(decision_maker)
    except KeyboardInterrupt:
        pass
    finally:
        decision_maker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Isaac Perception Pipeline

1. Launch the perception pipeline:
   ```bash
   ros2 launch isaac_perception_pipeline.launch.py
   ```

2. In another terminal, run the decision maker:
   ```bash
   ros2 run your_package decision_maker.py
   ```

3. Publish test images or connect a camera:
   ```bash
   # If you have a camera
   ros2 launch your_camera_package camera.launch.py

   # Or use a test image publisher
   ros2 run image_publisher image_publisher_node --ros-args -p image_path:=/path/to/test/image.jpg
   ```

## Isaac Lab Example (Conceptual)

For Isaac Lab integration, you would typically:

1. Install Isaac Lab
2. Set up the learning environment
3. Create reinforcement learning tasks
4. Train policies in simulation

The integration would typically be done through Python APIs rather than ROS launch files.

## Lab Exercise

Create a complete perception and decision-making system:

1. Set up an Isaac ROS pipeline with multiple perception nodes
2. Implement a decision-making algorithm that uses the perception data
3. Integrate with a robot simulation
4. Test the system with various scenarios

## Troubleshooting Common Issues

### GPU Memory Issues
- Monitor GPU memory usage: `nvidia-smi`
- Reduce batch sizes in DNN inference nodes
- Use TensorRT optimization to reduce memory requirements

### CUDA Runtime Errors
- Verify CUDA version compatibility
- Check that Isaac ROS packages are built for your CUDA version
- Ensure proper GPU drivers are installed

### Performance Issues
- Monitor node processing times
- Use Isaac ROS performance monitoring tools
- Optimize pipeline for your specific hardware

## Summary

In this lab, you've learned:
- How to set up Isaac ROS packages and nodes
- How to create perception pipelines using Isaac ROS
- How to integrate perception with decision-making
- How to work with Isaac Sim and Isaac Lab concepts
- How to optimize GPU-accelerated robotics applications