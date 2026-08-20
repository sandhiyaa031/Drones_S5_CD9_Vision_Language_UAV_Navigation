"""
RTAB-Map 3D Visual SLAM Launch File for GPS-Denied Drone Navigation.
Fuses RGB-D / Stereo camera feeds with IMU and odometry to generate dense 3D maps and loop closures.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/odom/filtered'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'queue_size': 10,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Kp/DetectorStrategy': '6',
                'Kp/MaxFeatures': '400',
                'Vis/MinInliers': '15',
                'Rtabmap/DetectionRate': '1.0',
                'Grid/FromDepth': 'true',
                'Grid/CellSize': '0.10',
                'Grid/RangeMax': '15.0'
            }],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('odom', LaunchConfiguration('odom_topic'))
            ]
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True
            }],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('odom', LaunchConfiguration('odom_topic'))
            ]
        )
    ])
