---
sidebar_position: 1
---

# Simulation API Reference

This reference provides an overview of key simulation APIs and concepts used throughout the course, focusing on Gazebo Harmonic and NVIDIA Isaac Sim.

## Overview

Simulation environments are critical for robotics development, allowing for safe testing, rapid iteration, and cost-effective development. This reference covers the primary simulation platforms used in the course: Gazebo Harmonic for physics-based simulation and NVIDIA Isaac Sim for photorealistic simulation with GPU-accelerated rendering.

## Gazebo Harmonic API

### Gazebo Simulation Concepts

Gazebo Harmonic provides physics simulation capabilities with realistic sensor simulation and environment modeling.

#### World Files (SDF Format)

Simulation worlds are defined using Simulation Description Format (SDF):

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
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
      <direction>-0.0 0.0 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

#### Gazebo Services

Gazebo provides various ROS services for simulation control:

- `/gazebo/reset_simulation` - Reset entire simulation
- `/gazebo/reset_world` - Reset world to initial state
- `/gazebo/pause_physics` - Pause physics simulation
- `/gazebo/unpause_physics` - Resume physics simulation
- `/gazebo/set_model_state` - Set model position/orientation
- `/gazebo/get_model_state` - Get current model state
- `/gazebo/spawn_sdf_model` - Spawn new model in simulation

#### Example: Spawning a Model in Gazebo

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity, GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import EntityState

class GazeboController(Node):
    def __init__(self):
        super().__init__('gazebo_controller')

        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.reset_simulation_client = self.create_client(Empty, '/reset_simulation')
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        self.pause_physics_client = self.create_client(Empty, '/pause_physics')
        self.unpause_physics_client = self.create_client(Empty, '/unpause_physics')

        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')

        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset simulation service not available, waiting again...')

        self.get_logger().info('Gazebo controller initialized')

    def spawn_model(self, model_name, model_xml, pose):
        """Spawn a model in Gazebo"""
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.initial_pose = pose

        future = self.spawn_client.call_async(request)
        return future

    def reset_simulation(self):
        """Reset the entire simulation"""
        request = Empty.Request()
        future = self.reset_simulation_client.call_async(request)
        return future

    def pause_physics(self):
        """Pause physics simulation"""
        request = Empty.Request()
        future = self.pause_physics_client.call_async(request)
        return future

    def unpause_physics(self):
        """Resume physics simulation"""
        request = Empty.Request()
        future = self.unpause_physics_client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    gazebo_controller = GazeboController()

    # Example usage
    pose = Pose()
    pose.position = Point(x=0.0, y=0.0, z=1.0)
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # Example model XML (simplified)
    model_xml = """
    <robot name="simple_box">
      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.5 0.5 0.5"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="0.5 0.5 0.5"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083"/>
        </inertial>
      </link>
    </robot>
    """

    future = gazebo_controller.spawn_model("simple_box", model_xml, pose)

    try:
        rclpy.spin_until_future_complete(gazebo_controller, future)
        response = future.result()
        if response.success:
            print(f"Model spawned successfully: {response.status_message}")
        else:
            print(f"Failed to spawn model: {response.status_message}")
    except Exception as e:
        print(f"Service call failed: {e}")

    gazebo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## NVIDIA Isaac Sim API

### Isaac Sim Concepts

NVIDIA Isaac Sim is built on NVIDIA Omniverse and provides photorealistic simulation with GPU-accelerated rendering, physically-based sensors, and reinforcement learning capabilities.

### USD (Universal Scene Description) Format

Isaac Sim uses USD (Universal Scene Description) format for scene definition:

```python
# Example of creating a USD stage programmatically in Isaac Sim
import omni
from pxr import Usd, UsdGeom, Gf, Sdf
import carb

def create_simple_scene():
    """Create a simple scene using USD in Isaac Sim"""
    # Get the current stage
    stage = omni.usd.get_context().get_stage()

    # Create a Xform prim for the ground plane
    ground_prim = UsdGeom.Xform.Define(stage, "/World/GroundPlane")

    # Add a plane geometry
    plane = UsdGeom.Mesh.Define(stage, "/World/GroundPlane/Plane")
    plane.CreatePointsAttr([(-10, 0, -10), (10, 0, -10), (10, 0, 10), (-10, 0, 10)])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 2, 3])
    plane.CreateFaceVertexCountsAttr([3, 3])

    # Create a cube
    cube_prim = UsdGeom.Cube.Define(stage, "/World/Cube")
    cube_prim.CreateSizeAttr(1.0)
    cube_prim.AddTranslateOp().Set((0, 1, 0))

# Example of importing and manipulating a robot in Isaac Sim
def import_robot(robot_path, position):
    """Import a robot into Isaac Sim"""
    import omni.kit.commands

    # Import robot from USD file
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXform",
        prim_type="Xform",
        prim_path="/World/Robot",
        attributes={"xformOp:translate": position}
    )
```

### Isaac Sim Extensions and APIs

Isaac Sim provides several key APIs and extensions:

#### Robotics Extensions
- `omni.isaac.ros_bridge` - ROS 2 bridge for communication
- `omni.isaac.range_sensor` - Range sensor simulation (lidar, depth cameras)
- `omni.isaac.motion_generation` - Motion planning and control
- `omni.isaac.sensor` - Sensor simulation and data processing

#### Example: Using Isaac Sim Range Sensors

```python
import omni
from omni.isaac.core import World
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np

def setup_lidar_sensor():
    """Set up a LIDAR sensor in Isaac Sim"""
    # Get the range sensor interface
    range_sensor_interface = _range_sensor.acquire_range_sensor_interface()

    # Create a LIDAR sensor
    lidar_path = "/World/Robot/Lidar"

    # Configure LIDAR parameters
    horizontal_resolution = 0.25  # degrees
    horizontal_pixels = 1440      # number of horizontal pixels
    vertical_resolution = 2.0     # degrees
    vertical_pixels = 32          # number of vertical pixels
    range_distance = 20.0         # meters
    fps = 10                      # frames per second

    # Create the LIDAR sensor
    range_sensor_interface.new_range_sensor(
        lidar_path,
        horizontal_resolution,
        horizontal_pixels,
        vertical_resolution,
        vertical_pixels,
        range_distance,
        fps,
        0.0,  # offset
        0.0,  # rotation
        0.0,  # translation
        0.0,  # near clipping
        1.0,  # far clipping
        0,    # scan order
        0     # return type
    )

    # Get LIDAR data
    lidar_data = range_sensor_interface.get_linear_depth_data(lidar_path)
    return lidar_data
```

### Isaac Sim ROS Bridge

The ROS bridge enables communication between Isaac Sim and ROS 2 nodes:

```python
# Example ROS bridge configuration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch file for Isaac Sim ROS bridge"""

    # ROS TCP endpoint for Isaac Sim connection
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

    # Isaac Sim bridge node
    isaac_ros_bridge = Node(
        package='isaac_ros_bridges',
        executable='isaac_ros_bridge_node',
        name='isaac_ros_bridge',
        parameters=[
            {'world_name': 'small_room.wrl'},
            {'robot_name': 'franka_panda'}
        ],
        output='screen'
    )

    return LaunchDescription([
        ros_tcp_endpoint,
        isaac_ros_bridge
    ])
```

## Simulation Best Practices

### Performance Optimization
1. **Physics Settings**: Adjust max_step_size and real_time_update_rate based on required accuracy vs. performance
2. **Sensor Configuration**: Optimize sensor update rates and resolutions for your specific use case
3. **Scene Complexity**: Balance scene detail with simulation performance
4. **GPU Utilization**: Ensure proper GPU acceleration is enabled for rendering and physics

### Safety Considerations
1. **Validation**: Always validate simulation results against real-world data
2. **Physics Accuracy**: Configure physics parameters appropriately for your robot's dynamics
3. **Sensor Noise**: Include realistic sensor noise models in simulation
4. **Boundary Conditions**: Set appropriate limits for robot movement and interaction

### Simulation-to-Reality Transfer
1. **Domain Randomization**: Vary environmental parameters to improve robustness
2. **System Identification**: Match simulated dynamics to real robot behavior
3. **Sensor Calibration**: Calibrate simulated sensors to match real hardware
4. **Testing Protocol**: Develop systematic validation procedures

## Common Simulation Issues and Solutions

### Physics Instability
- **Issue**: Robot joints oscillating or exploding
- **Solution**: Reduce physics step size, adjust joint damping, verify inertial properties

### Sensor Data Issues
- **Issue**: Inconsistent or noisy sensor data
- **Solution**: Check sensor configuration, verify coordinate frames, adjust noise parameters

### Performance Problems
- **Issue**: Low simulation speed
- **Solution**: Optimize scene complexity, adjust physics parameters, ensure GPU acceleration

### ROS Communication Issues
- **Issue**: Delayed or lost messages between simulation and ROS nodes
- **Solution**: Check network configuration, adjust QoS settings, verify bridge configuration

This reference provides the core simulation APIs and concepts used throughout the course. For more detailed information about specific simulation packages, refer to the official Gazebo and NVIDIA Isaac Sim documentation.