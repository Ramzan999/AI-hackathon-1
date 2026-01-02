---
sidebar_position: 2
---

# Hands-on Lab: Simulation Environments

In this lab, you'll set up and interact with robot simulation environments using Gazebo and understand how to test robot behaviors in safe virtual environments.

## Prerequisites

Before starting this lab, ensure you have:
- ROS 2 Humble Hawksbill installed
- Gazebo Harmonic installed
- Basic understanding of ROS 2 concepts (completed Module 1)

## Setting up Gazebo Environment

First, source your ROS 2 and Gazebo environments:

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash  # Adjust path based on your Gazebo installation
```

## Launching a Basic Gazebo World

Create a simple launch file to start Gazebo with a basic world. Create `basic_gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Launch configuration
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='SDF world file'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
```

## Creating a Simple Robot Model

Create a basic robot model in SDF format. Create `simple_robot.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 1 1</ambient>
          <diffuse>0.2 0.2 1 1</diffuse>
          <specular>0.2 0.2 1 1</specular>
        </material>
      </visual>
    </link>

    <!-- Simple differential drive plugin -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </model>
</sdf>
```

## Running the Simulation

1. Launch Gazebo with your robot:
   ```bash
   ros2 launch basic_gazebo.launch.py
   ```

2. In another terminal, send velocity commands to move the robot:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
   ```

## Creating a Gazebo World File

Create a simple world file `simple_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add your robot -->
    <include>
      <uri>model://simple_robot</uri>
    </include>

    <!-- Add some obstacles -->
    <model name="box1">
      <pose>2 2 0.5 0 0 0</pose>
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
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics properties -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Working with Sensors in Simulation

Create a robot model with sensors. Create `robot_with_sensors.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="robot_with_sensors">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 1 1</ambient>
          <diffuse>0.2 0.2 1 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Camera sensor -->
    <sensor name="camera" type="camera">
      <pose>0.25 0 0.1 0 0 0</pose>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_frame</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>

    <!-- Laser scanner -->
    <sensor name="laser" type="ray">
      <pose>0.2 0 0.15 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros_topic>scan</ros_topic>
        <frame_name>laser_frame</frame_name>
      </plugin>
    </sensor>

    <!-- IMU sensor -->
    <sensor name="imu" type="imu">
      <pose>0 0 0.1 0 0 0</pose>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <ros_topic>imu</ros_topic>
        <frame_name>imu_frame</frame_name>
      </plugin>
    </sensor>
  </model>
</sdf>
```

## Launch File with Sensors

Create `sensor_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='SDF world file'
    )

    # Start Gazebo server with sensors
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             PathJoinSubstitution([get_package_share_directory('your_package'), 'models', 'robot_with_sensors.sdf'])],
        output='screen'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # RViz for visualization
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('your_package'), 'rviz', 'sensors.rviz'])],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_cmd)

    return ld
```

## Working with Nav2 in Simulation

To integrate with Nav2 (Navigation 2), create a launch file `nav2_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the nav2 stack'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=PathJoinSubstitution([FindPackageShare('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml']),
        description='Full path to the behavior tree xml file to use'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'maps', 'map.yaml']),
        description='Full path to the map yaml file to use'
    )

    # Launch navigation stack
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'bt_xml_file': bt_xml_file
        }.items()
    )

    # Launch localization stack
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file
        }.items()
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(nav2_localization_launch)
    ld.add_action(nav2_bringup_launch)

    return ld
```

## Unity Simulation Setup (Conceptual)

For Unity simulation, you would typically:

1. Install Unity Robotics Package
2. Set up ROS TCP Connector
3. Create robot models in Unity
4. Configure sensors and communication

Unity simulation is typically configured through the Unity Editor, but you can create launch files to start the ROS connection:

```python
# unity_ros_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ROS TCP Connector node
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[
            {'host': 'localhost'},
            {'port': 10000},
            {'timeout': 100000},
            {'reconnection_delay': 1}
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(ros_tcp_endpoint)

    return ld
```

## Lab Exercise

Create a complete simulation scenario:

1. Create a world with multiple obstacles
2. Add a robot with differential drive and sensors
3. Implement a simple navigation task
4. Use RViz to visualize the robot's sensors and navigation

## Troubleshooting Common Issues

### Gazebo Won't Start
- Check if the correct Gazebo version is installed
- Verify environment variables are set
- Check for port conflicts

### Robot Not Moving
- Verify joint names match between URDF/SDF and controller
- Check topic names and message types
- Ensure proper permissions for simulation

### Sensor Data Not Publishing
- Check sensor plugin configuration
- Verify frame names and TF tree
- Ensure proper topic connections

## Summary

In this lab, you've learned:
- How to create and launch Gazebo simulations
- How to configure robot models with sensors
- How to integrate with navigation systems
- How to work with simulation-specific tools and configurations