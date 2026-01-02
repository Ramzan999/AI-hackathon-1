#!/usr/bin/env python3

"""
Integrated Robot Controller for Capstone Project
This demonstrates the integration of all course modules: ROS 2, Simulation, Isaac, and VLA.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import numpy as np


class IntegratedRobotController(Node):
    """
    An integrated controller that combines ROS 2 communication,
    simulation capabilities, Isaac perception, and VLA systems.
    """

    def __init__(self):
        super().__init__('integrated_robot_controller')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Robot state
        self.current_pose = None
        self.current_image = None
        self.laser_scan = None
        self.battery_level = 100.0
        self.is_safe = True

        # Subscribers for all sensor inputs
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.command_sub = self.create_subscription(
            String,
            'high_level_command',
            self.command_callback,
            10)

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Integrated Robot Controller initialized')

    def image_callback(self, msg):
        """Process incoming camera image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image

            # Process image for obstacle detection, object recognition, etc.
            self.process_vision_data(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def laser_callback(self, msg):
        """Process incoming laser scan data."""
        self.laser_scan = msg

        # Check for obstacles in front of robot
        if len(msg.ranges) > 0:
            # Get front-facing distance (assuming 0 is forward)
            front_distance = min(msg.ranges[0], msg.ranges[-1])  # Approximate front distance

            if front_distance < 0.5:  # Less than 0.5m is too close
                self.is_safe = False
                self.get_logger().warn('Obstacle detected! Stopping robot.')
            else:
                self.is_safe = True

    def odom_callback(self, msg):
        """Process odometry data for position tracking."""
        self.current_pose = msg.pose.pose

    def command_callback(self, msg):
        """Process high-level commands."""
        command = msg.data.lower()
        self.get_logger().info(f'Received high-level command: {command}')

        # Process complex commands that may involve multiple modules
        if 'navigate to' in command:
            self.handle_navigation_command(command)
        elif 'find object' in command:
            self.handle_object_search_command(command)
        elif 'follow path' in command:
            self.handle_path_following_command(command)
        else:
            self.handle_basic_command(command)

    def process_vision_data(self, image):
        """Process visual data using Isaac-inspired techniques."""
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect specific colors (example: red objects)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours of detected objects
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Filter small detections
                # Get center of object
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.get_logger().info(f'Object detected at ({cx}, {cy})')

    def handle_navigation_command(self, command):
        """Handle navigation commands using ROS 2 navigation stack."""
        # Parse target coordinates from command
        # This is a simplified example
        if 'kitchen' in command:
            target_pose = self.create_pose_stamped(2.0, 1.0, 0.0)  # x, y, theta
            self.nav_goal_pub.publish(target_pose)
        elif 'living room' in command:
            target_pose = self.create_pose_stamped(-1.0, 2.0, 1.57)  # x, y, theta
            self.nav_goal_pub.publish(target_pose)

    def handle_object_search_command(self, command):
        """Handle object search commands using VLA integration."""
        # This would integrate vision and language understanding
        self.get_logger().info(f'Searching for object: {command}')
        # Start search behavior
        self.search_for_object(command)

    def handle_path_following_command(self, command):
        """Handle path following using simulation and Isaac integration."""
        self.get_logger().info('Following predefined path')
        # Follow a predefined path
        self.follow_path()

    def handle_basic_command(self, command):
        """Handle basic movement commands."""
        msg = Twist()

        if 'forward' in command:
            msg.linear.x = 0.5
        elif 'backward' in command:
            msg.linear.x = -0.5
        elif 'left' in command:
            msg.angular.z = 0.5
        elif 'right' in command:
            msg.angular.z = -0.5
        elif 'stop' in command:
            pass  # Twist message is already zero
        else:
            self.get_logger().info(f'Unknown command: {command}')
            return

        self.cmd_vel_pub.publish(msg)

    def search_for_object(self, object_name):
        """Search behavior for finding specific objects."""
        # Rotate slowly while looking for the object
        msg = Twist()
        msg.angular.z = 0.2  # Slow rotation
        self.cmd_vel_pub.publish(msg)

    def follow_path(self):
        """Follow a predefined path."""
        # This would typically use a path planner
        msg = Twist()
        msg.linear.x = 0.3  # Move forward at moderate speed
        self.cmd_vel_pub.publish(msg)

    def create_pose_stamped(self, x, y, theta):
        """Create a PoseStamped message."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0

        # Convert theta to quaternion
        from math import sin, cos
        pose_msg.pose.orientation.z = sin(theta / 2.0)
        pose_msg.pose.orientation.w = cos(theta / 2.0)

        return pose_msg

    def safety_check(self):
        """Periodic safety checks."""
        if not self.is_safe:
            # Emergency stop
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

        # Monitor battery level
        if self.battery_level > 0:
            self.battery_level -= 0.01  # Simulate battery drain

        if self.battery_level < 10:
            self.get_logger().warn('Battery level low! Consider returning to charging station.')


def main(args=None):
    rclpy.init(args=args)
    controller = IntegratedRobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()