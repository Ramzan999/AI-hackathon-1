---
sidebar_position: 1
---

# ROS 2 API Reference

This reference provides an overview of key ROS 2 APIs and concepts used throughout the course.

## Core Concepts

### Nodes
A node is the fundamental unit of computation in ROS 2. Nodes are typically organized into packages to form the structure of a ROS-based system.

**Creating a Node in Python:**
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Topics and Messages
Topics are named buses over which nodes exchange messages. The publisher-subscriber pattern allows for asynchronous communication.

**Creating a Publisher:**
```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
```

**Creating a Subscriber:**
```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Services
Services provide synchronous request-response communication between nodes.

**Creating a Service Server:**
```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

**Creating a Service Client:**
```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)
        return future
```

### Actions
Actions are used for long-running tasks that may need feedback and the ability to cancel.

**Creating an Action Server:**
```python
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Parameters
Parameters provide a way to configure nodes at runtime.

**Using Parameters:**
```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')

        # Get parameter value
        param_value = self.get_parameter('param_name').value
```

## Quality of Service (QoS)
QoS settings allow fine-tuning of communication behavior.

**Common QoS Profiles:**
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Default profile
default_qos = QoSProfile(depth=10)

# Reliable communication
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Best effort for sensor data
best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)
```

## Timers
Timers allow periodic execution of functions.

**Creating a Timer:**
```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        self.timer = self.create_timer(0.5, self.timer_callback)  # Every 0.5 seconds

    def timer_callback(self):
        self.get_logger().info('Timer callback executed')
```

## Logging
ROS 2 provides different logging levels.

**Logging in Nodes:**
```python
# Different logging levels
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

## Launch Files
Launch files allow launching multiple nodes with specific configurations.

**Python Launch File Example:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_name',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        )
    ])
```

## Common Message Types

### Standard Messages
- `std_msgs/msg/String` - String data
- `std_msgs/msg/Int32`, `std_msgs/msg/Float64` - Numeric data
- `std_msgs/msg/Bool` - Boolean values
- `std_msgs/msg/Header` - Message header with timestamp and frame

### Geometry Messages
- `geometry_msgs/msg/Twist` - Linear and angular velocity
- `geometry_msgs/msg/Pose` - Position and orientation
- `geometry_msgs/msg/Point` - 3D point coordinates
- `geometry_msgs/msg/Quaternion` - Orientation as quaternion

### Sensor Messages
- `sensor_msgs/msg/Image` - Image data
- `sensor_msgs/msg/CameraInfo` - Camera calibration parameters
- `sensor_msgs/msg/LaserScan` - LIDAR scan data
- `sensor_msgs/msg/JointState` - Joint positions, velocities, efforts

This reference provides the core ROS 2 concepts and APIs used throughout the course. For more detailed information, refer to the official ROS 2 documentation.