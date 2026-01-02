---
sidebar_position: 3
---

# Troubleshooting: ROS 2 Nervous System

This guide provides solutions to common issues encountered when working with ROS 2 and the robotic nervous system.

## Common Setup Issues

### ROS 2 Environment Not Sourced

**Problem**: Commands like `ros2` are not found or nodes can't communicate.

**Solution**: Ensure your ROS 2 environment is sourced:
```bash
source /opt/ros/humble/setup.bash
```

To make this permanent, add the source command to your shell's startup script:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Python Import Errors

**Problem**: `ModuleNotFoundError` when importing `rclpy` or other ROS 2 packages.

**Solution**: Make sure you're using the correct Python environment:
```bash
# Check if rclpy is installed
python3 -c "import rclpy; print('rclpy available')"
```

If not available, install it:
```bash
pip3 install rclpy
```

## Network and Communication Issues

### Nodes Can't Communicate Across Machines

**Problem**: Nodes on different machines can't discover each other.

**Solution**:
1. Ensure both machines are on the same network
2. Check firewall settings to allow DDS communication (ports 7400-9000)
3. Set the ROS domain ID consistently across machines:
   ```bash
   export ROS_DOMAIN_ID=42
   ```

### High Network Latency

**Problem**: Messages are delayed or dropped in simulation.

**Solution**: Adjust QoS settings for your use case:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For real-time applications
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

## Python Node Issues

### Node Doesn't Respond to Ctrl+C

**Problem**: The Python script doesn't terminate cleanly when pressing Ctrl+C.

**Solution**: Ensure you're using `rclpy.spin()` instead of a custom loop, or properly handle SIGINT:

```python
import signal
import sys

def signal_handler(sig, frame):
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
```

### Memory Leaks in Long-Running Nodes

**Problem**: Memory usage increases over time in nodes with timers or subscriptions.

**Solution**: Properly clean up resources:

```python
def destroy_node(self):
    # Cancel timers
    if self.timer:
        self.timer.cancel()

    # Destroy publishers/subscribers
    if self.publisher_:
        self.publisher_.destroy()

    # Call parent destroy
    super().destroy_node()
```

## Service and Action Issues

### Service Calls Timeout

**Problem**: Service clients time out waiting for responses.

**Solution**:
1. Verify the service server is running
2. Check that both nodes are using the same service name
3. Increase the timeout value in the client:

```python
# Wait for service with timeout
if not self.cli.wait_for_service(timeout_sec=5.0):
    self.get_logger().error('Service not available')
    return
```

### Action Server Not Responding

**Problem**: Action clients don't receive feedback or results.

**Solution**: Ensure the action server properly implements all required callbacks and sends feedback at appropriate intervals.

## URDF Issues

### URDF Doesn't Load in RViz

**Problem**: Robot model doesn't appear in visualization.

**Solution**:
1. Validate the URDF syntax:
   ```bash
   check_urdf /path/to/robot.urdf
   ```
2. Ensure joint limits are properly defined
3. Check that link names don't contain special characters

### TF Transform Errors

**Problem**: "No transform from X to Y" errors when visualizing robots.

**Solution**:
1. Make sure the robot_state_publisher node is running:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="..."
   ```
2. Verify that joint states are being published if the robot has moving parts

## Performance Issues

### High CPU Usage

**Problem**: ROS 2 nodes consume excessive CPU resources.

**Solution**:
1. Reduce timer frequencies in non-critical nodes
2. Use appropriate QoS settings (e.g., reduce history depth)
3. Check for infinite loops or excessive logging

### Message Queue Overflow

**Problem**: Messages are being dropped due to buffer overflow.

**Solution**: Increase queue sizes in QoS settings:

```python
qos_profile = QoSProfile(
    history=HistoryPolicy.KEEP_ALL,  # Keep all messages
    depth=100  # Increase buffer size
)
```

## Debugging Strategies

### Using ROS 2 Command Line Tools

Check available topics:
```bash
ros2 topic list
```

Echo messages on a topic:
```bash
ros2 topic echo /topic_name std_msgs/msg/String
```

Check available services:
```bash
ros2 service list
```

### Logging and Debugging

Use appropriate logging levels:

```python
self.get_logger().debug('Detailed debug info')
self.get_logger().info('General information')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal error')
```

### Profiling ROS 2 Applications

Use system tools to profile performance:
```bash
# Monitor CPU and memory usage
htop

# Monitor network usage
nethogs

# Monitor ROS 2 communication
ros2 topic hz /topic_name
```

## Best Practices for Troubleshooting

1. **Isolate the problem**: Test individual components separately before integrating
2. **Check the ROS 2 graph**: Use `ros2 node list` and `ros2 topic list` to understand the system state
3. **Use standard tools**: Leverage ROS 2's built-in debugging tools before custom solutions
4. **Check permissions**: Ensure proper file and network permissions for robot operation
5. **Validate configurations**: Double-check parameter values and network settings

## Common Error Messages and Solutions

- **"Failed to create subscription"**: Usually indicates a type mismatch between publisher and subscriber
- **"Node name already exists"**: A node with the same name is already running; terminate it first
- **"Service not available"**: The service server is not running or not yet ready
- **"Could not find the resource"**: Check package paths and ensure dependencies are installed

## Getting Help

When seeking help:
1. Provide your ROS 2 distribution and version
2. Include relevant code snippets
3. Share error messages exactly as they appear
4. Describe your expected vs. actual behavior
5. Mention any recent changes to your system