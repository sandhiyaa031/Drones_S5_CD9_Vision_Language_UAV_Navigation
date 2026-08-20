"""
Lightweight VO + EKF Fusion Launch File for GPS-Denied Navigation.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        
        Node(
            package='gps_denied_nav',
            executable='vo_node',
            name='visual_odometry_node',
            output='screen'
        ),
        Node(
            package='gps_denied_nav',
            executable='ekf_fusion_node',
            name='ekf_fusion_node',
            output='screen'
        ),
        Node(
            package='gps_denied_nav',
            executable='tf_broadcaster',
            name='tf_broadcaster_node',
            output='screen'
        ),
        Node(
            package='gps_denied_nav',
            executable='mavros_vision_bridge',
            name='mavros_vision_bridge_node',
            output='screen'
        )
    ])
