"""
Master ROS 2 Launch File for Member 4 GPS-Denied Navigation Suite.
Launches Visual Odometry, EKF Multi-Sensor Fusion, TF2 Broadcaster,
MAVROS Vision Bridge, Occupancy Grid Mapping, and Diagnostics.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('gps_denied_nav') if 'gps_denied_nav' in os.environ.get('AMENT_PREFIX_PATH', '') else '.'

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_topic = LaunchConfiguration('camera_topic', default='/camera/image_raw')
    imu_topic = LaunchConfiguration('imu_topic', default='/imu/data')
    rangefinder_topic = LaunchConfiguration('rangefinder_topic', default='/rangefinder/range')
    mavros_target_frame = LaunchConfiguration('mavros_target_frame', default='NED')

    # 1. Visual Odometry Node
    vo_node = Node(
        package='gps_denied_nav',
        executable='vo_node',
        name='visual_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_topic': camera_topic,
            'rangefinder_topic': rangefinder_topic,
            'frame_id': 'odom',
            'max_features': 250,
            'detector_type': 'FAST',
            'publish_debug_image': True
        }]
    )

    # 2. Multi-Sensor EKF Fusion Node
    ekf_node = Node(
        package='gps_denied_nav',
        executable='ekf_fusion_node',
        name='ekf_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'imu_topic': imu_topic,
            'vo_topic': '/vo/odom',
            'rangefinder_topic': rangefinder_topic,
            'output_frame_id': 'odom',
            'child_frame_id': 'base_link',
            'publish_rate_hz': 50.0
        }]
    )

    # 3. TF2 Coordinate Broadcaster
    tf_node = Node(
        package='gps_denied_nav',
        executable='tf_broadcaster',
        name='tf_broadcaster_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom/filtered',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }]
    )

    # 4. MAVROS Vision Pose Bridge (ArduPilot / PX4 EKF2)
    mavros_bridge_node = Node(
        package='gps_denied_nav',
        executable='mavros_vision_bridge',
        name='mavros_vision_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_odom_topic': '/odom/filtered',
            'target_frame': mavros_target_frame,
            'enable_covariance': True
        }]
    )

    # 5. Occupancy Grid Node (2D/3D Local & Global Obstacle Mapping)
    grid_node = Node(
        package='gps_denied_nav',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.10,
            'width_m': 40.0,
            'height_m': 40.0
        }]
    )

    # 6. Diagnostics & SLAM Health Monitor
    diag_node = Node(
        package='gps_denied_nav',
        executable='diagnostics_node',
        name='diagnostics_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw', description='Camera image topic'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data', description='IMU data topic'),
        DeclareLaunchArgument('rangefinder_topic', default_value='/rangefinder/range', description='Rangefinder topic'),
        DeclareLaunchArgument('mavros_target_frame', default_value='NED', description='Target coordinate frame for MAVROS'),
        vo_node,
        ekf_node,
        tf_node,
        mavros_bridge_node,
        grid_node,
        diag_node
    ])
