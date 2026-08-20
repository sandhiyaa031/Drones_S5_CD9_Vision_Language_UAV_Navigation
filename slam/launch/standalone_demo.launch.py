"""
Standalone Simulation & Interactive Navigation Launch File.
Launches Synthetic Environment Generator + VO + EKF + Occupancy Grid + Diagnostics.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Synthetic Data Generator (Publishes simulated camera & IMU)
        Node(
            package='gps_denied_nav',
            executable='synthetic_data_generator',
            name='synthetic_data_generator',
            output='screen'
        ),
        # Visual Odometry
        Node(
            package='gps_denied_nav',
            executable='vo_node',
            name='visual_odometry_node',
            output='screen'
        ),
        # EKF Fusion
        Node(
            package='gps_denied_nav',
            executable='ekf_fusion_node',
            name='ekf_fusion_node',
            output='screen'
        ),
        # TF Broadcaster
        Node(
            package='gps_denied_nav',
            executable='tf_broadcaster',
            name='tf_broadcaster_node',
            output='screen'
        ),
        # MAVROS Bridge
        Node(
            package='gps_denied_nav',
            executable='mavros_vision_bridge',
            name='mavros_vision_bridge_node',
            output='screen'
        ),
        # 2D Occupancy Grid
        Node(
            package='gps_denied_nav',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen'
        )
    ])
