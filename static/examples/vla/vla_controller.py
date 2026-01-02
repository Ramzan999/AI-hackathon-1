#!/usr/bin/env python3

"""
Vision-Language-Action Controller
This demonstrates a basic VLA system that integrates vision, language understanding, and action planning.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np


class VLAController(Node):
    """
    Vision-Language-Action controller that processes visual input,
    understands language commands, and executes actions.
    """

    def __init__(self):
        super().__init__('vla_controller')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.command_sub = self.create_subscription(
            String,
            'vla_command',
            self.command_callback,
            10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Internal state
        self.current_image = None
        self.last_command = None
        self.object_detected = False
        self.target_position = None

        self.get_logger().info('VLA Controller initialized')

    def image_callback(self, msg):
        """Process incoming image data."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image

            # Simple object detection (placeholder)
            self.detect_objects(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def command_callback(self, msg):
        """Process incoming language command."""
        self.last_command = msg.data
        self.get_logger().info(f'Received command: {msg.data}')

        # Process command and execute action
        self.process_command(msg.data)

    def detect_objects(self, image):
        """Simple object detection (placeholder implementation)."""
        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color (example)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            self.object_detected = True
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Get center of contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                self.target_position = (cx, cy)
        else:
            self.object_detected = False
            self.target_position = None

    def process_command(self, command):
        """Process language command and determine appropriate action."""
        command_lower = command.lower()

        # Simple command parsing (placeholder)
        if 'forward' in command_lower or 'move forward' in command_lower:
            self.move_forward()
        elif 'backward' in command_lower or 'move backward' in command_lower:
            self.move_backward()
        elif 'left' in command_lower:
            self.turn_left()
        elif 'right' in command_lower:
            self.turn_right()
        elif 'stop' in command_lower:
            self.stop_robot()
        elif 'approach' in command_lower or 'go to' in command_lower:
            if self.object_detected and self.target_position:
                self.approach_object()
            else:
                self.get_logger().info('Object not detected for approach command')
        else:
            self.get_logger().info(f'Unknown command: {command}')

    def move_forward(self):
        """Move robot forward."""
        msg = Twist()
        msg.linear.x = 0.5  # Forward velocity
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Moving forward')

    def move_backward(self):
        """Move robot backward."""
        msg = Twist()
        msg.linear.x = -0.5  # Backward velocity
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Moving backward')

    def turn_left(self):
        """Turn robot left."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5  # Left turn
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Turning left')

    def turn_right(self):
        """Turn robot right."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.5  # Right turn
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Turning right')

    def stop_robot(self):
        """Stop robot movement."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Robot stopped')

    def approach_object(self):
        """Approach detected object."""
        if self.target_position:
            # Simple proportional controller
            image_width = 640  # Assuming 640x480 image
            center_x = image_width / 2
            target_x = self.target_position[0]

            error_x = target_x - center_x
            angular_velocity = 0.002 * error_x  # Proportional gain

            msg = Twist()
            msg.linear.x = 0.2  # Move forward slowly
            msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(msg)

            self.get_logger().info(f'Approaching object at x={target_x}, angular vel={angular_velocity}')


def main(args=None):
    rclpy.init(args=args)
    vla_controller = VLAController()

    try:
        rclpy.spin(vla_controller)
    except KeyboardInterrupt:
        pass
    finally:
        vla_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()