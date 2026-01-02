---
sidebar_position: 2
---

# Capstone Project Implementation Guide

This guide provides step-by-step instructions for implementing the integrated robotic assistant system, combining all modules from the course into a complete working system.

## Phase 1: Foundation Setup

### 1.1 Workspace and Dependencies Setup

Create your project workspace:

```bash
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws/src

# Clone necessary repositories
git clone https://github.com/ros-planning/navigation2.git -b humble
git clone https://github.com/ros-perception/vision_msgs.git -b humble
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git
```

Create your main package:

```bash
cd ~/capstone_ws/src
ros2 pkg create --build-type ament_python capstone_robot --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav2_msgs vision_msgs
```

### 1.2 Robot Description and Configuration

Create the robot URDF in `capstone_robot/urdf/capstone_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="capstone_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and camera -->
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Add other joints and links as needed -->
</robot>
```

Create the launch file for the basic robot setup in `capstone_robot/launch/basic_setup.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_path = LaunchConfiguration('robot_description_path')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_robot_description_path_cmd = DeclareLaunchArgument(
        'robot_description_path',
        default_value=PathJoinSubstitution([
            get_package_share_directory('capstone_robot'),
            'urdf',
            'capstone_robot.urdf'
        ]),
        description='Path to robot URDF file'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': LaunchConfiguration('robot_description_path'),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Joint state publisher (for fixed joints in this example)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # TF broadcaster
    tf_broadcaster = Node(
        package='capstone_robot',
        executable='tf_broadcaster.py',
        name='tf_broadcaster',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_description_path_cmd)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(tf_broadcaster)

    return ld
```

### 1.3 Basic ROS 2 Nodes

Create the TF broadcaster in `capstone_robot/capstone_robot/tf_broadcaster.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.broadcast_frame)

    def broadcast_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FramePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 2: Navigation System Setup

### 2.1 Navigation Configuration

Create navigation configuration files in `capstone_robot/config/navigation.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the BT XML files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_consistent_localization_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
```

### 2.2 Navigation Launch File

Create the navigation launch file in `capstone_robot/launch/navigation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('capstone_robot'),
            'config',
            'navigation.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(navigation_launch)

    return ld
```

## Phase 3: Perception System Integration

### 3.1 Isaac ROS Perception Pipeline

Create the perception pipeline launch file in `capstone_robot/launch/perception.launch.py`:

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
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Perception container
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            # Image format converter
            ComposableNode(
                package='isaac_ros_image_format_converter',
                plugin='nvidia::isaac_ros::image_format_converter::ImageFormatConverterNode',
                name='image_format_converter',
                parameters=[{
                    'encoding_desired': 'rgb8',
                }],
                remappings=[
                    ('image_raw', image_topic),
                    ('image', 'rgb_image'),
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
                    'tag_size': 0.12,  # Adjust based on your tags
                }],
                remappings=[
                    ('image', 'rgb_image'),
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
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(perception_container)

    return ld
```

## Phase 4: Language Understanding Integration

### 4.1 Command Processing Node

Create the main command processing node in `capstone_robot/capstone_robot/command_processor.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from capstone_robot.vla_system import LanguageUnderstandingNode, VisionProcessorNode, ActionPlannerNode


class CapstoneCommandProcessor(Node):
    def __init__(self):
        super().__init__('capstone_command_processor')

        # Initialize subsystems
        self.language_understanding = LanguageUnderstandingNode()
        self.vision_processor = VisionProcessorNode()
        self.action_planner = ActionPlannerNode()

        # Subscribe to natural language commands
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )

        # Publisher for system status
        self.status_publisher = self.create_publisher(
            String,
            'capstone_status',
            10
        )

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Capstone Command Processor initialized')

    def command_callback(self, msg):
        """Process natural language commands through integrated system"""
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Update status
        status_msg = String()
        status_msg.data = f'Processing: {command_text}'
        self.status_publisher.publish(status_msg)

        # Process through language understanding
        parsed_command = self.language_understanding.parse_command(command_text)

        if parsed_command:
            # Process through vision system for context
            # (This would integrate with the vision system in a real implementation)

            # Plan and execute action
            success = self.action_planner.plan_and_execute_action(parsed_command)

            result_msg = String()
            result_msg.data = f"Command {'succeeded' if success else 'failed'}: {command_text}"
            self.status_publisher.publish(result_msg)
        else:
            status_msg = String()
            status_msg.data = f"Could not understand command: {command_text}"
            self.status_publisher.publish(status_msg)

    def wait_for_nav_server(self):
        """Wait for navigation server to be available"""
        self.nav_client.wait_for_server()


def main(args=None):
    rclpy.init(args=args)

    capstone_node = CapstoneCommandProcessor()

    try:
        rclpy.spin(capstone_node)
    except KeyboardInterrupt:
        pass
    finally:
        capstone_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 5: Main Integration Launch

### 5.1 Complete System Launch

Create the main launch file in `capstone_robot/launch/capstone_system.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_path = LaunchConfiguration('robot_description_path')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_robot_description_path_cmd = DeclareLaunchArgument(
        'robot_description_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('capstone_robot'),
            'urdf',
            'capstone_robot.urdf'
        ]),
        description='Path to robot URDF file'
    )

    # Include basic setup
    basic_setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('capstone_robot'),
            'launch',
            'basic_setup.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_description_path': robot_description_path
        }.items()
    )

    # Include navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('capstone_robot'),
            'launch',
            'navigation.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Include perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('capstone_robot'),
            'launch',
            'perception.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Capstone command processor
    command_processor = Node(
        package='capstone_robot',
        executable='command_processor.py',
        name='capstone_command_processor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Example command publisher for testing
    command_publisher = Node(
        package='capstone_robot',
        executable='command_publisher.py',
        name='command_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_description_path_cmd)

    ld.add_action(basic_setup_launch)
    ld.add_action(navigation_launch)
    ld.add_action(perception_launch)
    ld.add_action(command_processor)
    ld.add_action(command_publisher)

    return ld
```

## Phase 6: Testing and Validation

### 6.1 Test Script

Create a test script in `capstone_robot/test/capstone_test.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class CapstoneTester(Node):
    def __init__(self):
        super().__init__('capstone_tester')

        # Publisher for test commands
        self.command_publisher = self.create_publisher(
            String,
            'natural_language_command',
            10
        )

        # Publisher for direct robot control (for validation)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer for test sequence
        self.test_timer = self.create_timer(10.0, self.run_test_sequence)
        self.test_step = 0

        # Test commands sequence
        self.test_commands = [
            "Move forward slowly",
            "Turn left",
            "Stop",
            "Navigate to kitchen",
            "Find the red object",
            "Pick up the block"
        ]

        self.get_logger().info('Capstone Tester initialized')

    def run_test_sequence(self):
        """Run automated test sequence"""
        if self.test_step < len(self.test_commands):
            command = self.test_commands[self.test_step]

            cmd_msg = String()
            cmd_msg.data = command

            self.command_publisher.publish(cmd_msg)
            self.get_logger().info(f'Sent test command: {command}')

            self.test_step += 1
        else:
            self.get_logger().info('Test sequence completed')
            self.test_timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    tester = CapstoneTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Complete System

1. Build your workspace:
   ```bash
   cd ~/capstone_ws
   colcon build --packages-select capstone_robot
   source install/setup.bash
   ```

2. Launch the complete system:
   ```bash
   ros2 launch capstone_robot capstone_system.launch.py
   ```

3. Send test commands:
   ```bash
   ros2 topic pub /natural_language_command std_msgs/String "data: 'move forward'"
   ```

4. Monitor system status:
   ```bash
   ros2 topic echo /capstone_status
   ```

## Troubleshooting Common Issues

### Component Integration Issues
- Verify all nodes are using the same coordinate frames
- Check that topics are properly remapped between components
- Ensure parameter files are correctly loaded

### Performance Issues
- Monitor CPU and memory usage of each component
- Check that perception pipeline isn't blocking other operations
- Verify navigation stack is properly configured

### Communication Problems
- Use `ros2 topic list` and `ros2 node list` to verify connectivity
- Check QoS settings between components
- Verify that all required transforms are being published

This implementation guide provides the foundation for your capstone project. Each component builds upon the modules covered in the course, integrating ROS 2 communication, perception, navigation, and language understanding into a complete robotic system.