---
sidebar_position: 3
---

# Troubleshooting: Simulation Environments

This guide provides solutions to common issues encountered when working with Gazebo, Unity, and other simulation environments for robotics.

## Gazebo Common Issues

### Gazebo Won't Start

**Problem**: Gazebo fails to launch with X11 or graphics errors.

**Solution**:
1. Check if X11 forwarding is enabled (for remote systems):
   ```bash
   export DISPLAY=:0
   ```

2. For headless systems, use:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gzserver --verbose world_file.sdf
   ```

3. Ensure proper graphics drivers are installed:
   ```bash
   nvidia-smi  # For NVIDIA systems
   glxinfo | grep "OpenGL renderer"  # Check OpenGL support
   ```

### Simulation Runs Too Slow

**Problem**: Gazebo simulation runs at less than real-time speed.

**Solution**:
1. Adjust physics parameters in your world file:
   ```xml
   <physics name="fast" type="ode">
     <max_step_size>0.01</max_step_size>  <!-- Increase from 0.001 -->
     <real_time_update_rate>100</real_time_update_rate>  <!-- Decrease if needed -->
   </physics>
   ```

2. Reduce visual complexity:
   - Use simpler meshes for collision detection
   - Reduce sensor update rates
   - Disable unnecessary visual plugins

3. Check system resources:
   ```bash
   htop  # Monitor CPU usage
   nvidia-smi  # Monitor GPU usage (if applicable)
   ```

### Robot Falls Through Ground

**Problem**: Robot model falls through the ground plane or other static objects.

**Solution**:
1. Verify collision properties are defined:
   ```xml
   <collision name="collision">
     <geometry>
       <box><size>1 1 1</size></box>
     </geometry>
   </collision>
   ```

2. Check that static objects have `<static>true</static>`:
   ```xml
   <model name="ground_plane">
     <static>true</static>
     <!-- other properties -->
   </model>
   ```

3. Ensure proper mass and inertia values:
   ```xml
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
   ```

### Sensors Not Publishing Data

**Problem**: Simulated sensors aren't publishing data to ROS topics.

**Solution**:
1. Verify sensor plugin is loaded:
   ```xml
   <sensor name="camera" type="camera">
     <!-- sensor configuration -->
     <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
       <frame_name>camera_frame</frame_name>
       <topic_name>camera/image_raw</topic_name>
     </plugin>
   </sensor>
   ```

2. Check that the plugin library is installed:
   ```bash
   # For camera
   apt list --installed | grep gazebo-ros-pkgs
   # Install if missing
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

3. Verify topics are being published:
   ```bash
   ros2 topic list | grep sensor
   ros2 topic echo /sensor_topic_name std_msgs/msg/String
   ```

## ROS Integration Issues

### TF Tree Problems

**Problem**: TF tree is broken or missing transforms between robot frames.

**Solution**:
1. Check that robot_state_publisher is running:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="..."
   ```

2. Verify joint_state_publisher is publishing joint states:
   ```bash
   ros2 topic echo /joint_states sensor_msgs/msg/JointState
   ```

3. Use tf2 tools to debug:
   ```bash
   ros2 run tf2_tools view_frames
   ros2 run tf2_ros tf2_echo base_link camera_frame
   ```

### Plugin Loading Errors

**Problem**: Gazebo plugins fail to load with "Library not found" errors.

**Solution**:
1. Check if the plugin library exists:
   ```bash
   find /opt/ros/humble -name "*gazebo_ros*.so"
   ```

2. Verify library path is set:
   ```bash
   echo $GAZEBO_PLUGIN_PATH
   export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
   ```

3. Check for version compatibility between Gazebo and ROS packages.

## Performance Issues

### High CPU Usage

**Problem**: Simulation consumes excessive CPU resources.

**Solution**:
1. Reduce physics update rate in world file:
   ```xml
   <physics name="physics">
     <max_step_size>0.01</max_step_size>
     <real_time_update_rate>100</real_time_update_rate>
   </physics>
   ```

2. Limit sensor update rates:
   ```xml
   <sensor name="camera" type="camera">
     <update_rate>10</update_rate>  <!-- Reduce from default -->
   </sensor>
   ```

3. Use simpler collision geometries (boxes instead of complex meshes).

### Memory Leaks in Long Simulations

**Problem**: Memory usage increases over time during long-running simulations.

**Solution**:
1. Monitor plugin memory usage:
   ```bash
   # Monitor specific processes
   watch -n 1 'ps aux | grep -E "(gzserver|gzclient)"'
   ```

2. Implement proper cleanup in custom plugins
3. Periodically reset simulation state if possible

## Unity Simulation Issues

### Connection Problems

**Problem**: Unity simulation won't connect to ROS network.

**Solution**:
1. Check that ROS TCP Connector is properly configured:
   - Verify IP address and port settings
   - Ensure firewall allows connections on specified port
   - Check that both systems are on same network (if remote)

2. Verify Unity Robotics Package is properly installed:
   - Check Unity Package Manager
   - Verify ROS Communication examples work

3. Test connection with simple ping:
   ```bash
   telnet unity_ip_address port_number
   ```

### Sensor Data Quality

**Problem**: Simulated sensor data doesn't match expected real-world behavior.

**Solution**:
1. Calibrate sensor parameters to match real hardware:
   - Camera intrinsics and extrinsics
   - LIDAR range and resolution
   - IMU noise characteristics

2. Use domain randomization techniques:
   - Add realistic noise models
   - Vary lighting conditions
   - Introduce systematic errors

## Navigation Integration Issues

### Nav2 Not Working in Simulation

**Problem**: Navigation stack doesn't work properly in simulated environment.

**Solution**:
1. Ensure proper simulation time is used:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
   ```

2. Check that transforms are properly published:
   - robot_base_frame matches robot model
   - Sensor frames are correctly positioned
   - Map frame is properly set

3. Verify sensor data quality:
   - Check sensor ranges are appropriate
   - Ensure sufficient sensor coverage
   - Verify sensor data is being published consistently

## Debugging Strategies

### Simulation Debugging Tools

1. **Gazebo GUI**: Use visualization tools to inspect physics, contacts, and transforms
2. **RViz**: Visualize sensor data, TF tree, and navigation components
3. **ROS 2 tools**: Use command-line tools to inspect topics, services, and parameters

### Logging and Monitoring

Enable detailed logging in your launch files:
```python
# In your launch file
Node(
    package='your_package',
    executable='your_node',
    parameters=[{'log_level': 'debug'}],
    output='screen'
)
```

### Performance Profiling

Monitor simulation performance:
```bash
# Monitor simulation real-time factor
gz stats

# Monitor ROS 2 communication
ros2 topic hz /topic_name

# Monitor system resources
htop
nvidia-smi  # For GPU systems
```

## Common Error Messages and Solutions

- **"Could not find required plugin"**: Plugin library not installed or path not set
- **"Joint not found"**: Joint names don't match between URDF/SDF and controller
- **"No transform from X to Y"**: TF tree is incomplete or robot_state_publisher not running
- **"Simulation clock not synchronized"**: Check use_sim_time parameter consistency
- **"Plugin failed to load"**: Version mismatch or missing dependencies

## Best Practices for Simulation

1. **Start Simple**: Begin with basic models and gradually add complexity
2. **Validate Early**: Compare simulation behavior with analytical models
3. **Document Assumptions**: Keep track of simulation limitations and approximations
4. **Test Transitions**: Ensure smooth transition from simulation to hardware when possible
5. **Monitor Performance**: Regularly check simulation real-time factor and resource usage

## Getting Help

When seeking help with simulation issues:
1. Provide your Gazebo/Unity and ROS 2 versions
2. Include relevant configuration files (SDF, URDF, launch files)
3. Share error messages exactly as they appear
4. Describe your expected vs. actual behavior
5. Mention any recent changes to your simulation setup