#!/usr/bin/env python3

"""
Isaac ROS Pipeline Example
This demonstrates how to create a composable Isaac ROS pipeline for image processing.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for Isaac ROS image processing pipeline."""

    # Container for all Isaac ROS nodes
    container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image Rectification Node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', 'camera/image_raw'),
                    ('camera_info', 'camera/camera_info'),
                    ('image_rect', 'camera/image_rect_color'),
                ],
            ),
            # AprilTag Detection Node
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
                    ('image', 'camera/image_rect_color'),
                    ('camera_info', 'camera/camera_info'),
                    ('detections', 'tag_detections'),
                ],
            ),
            # Image Format Converter
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
                    ('image_raw', 'camera/image_rect_color'),
                    ('image', 'camera/image_rgb'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])